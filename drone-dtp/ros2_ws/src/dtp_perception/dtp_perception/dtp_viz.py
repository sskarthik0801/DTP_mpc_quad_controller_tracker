"""dtp_viz -- turn the DTP stack's state into something RViz can actually draw.

WHY THIS NODE HAS TO EXIST
--------------------------
RViz renders a fixed set of message types. It cannot render ANY of the things that would actually
show whether this stack is working:

  * `dtp_msgs/TargetArray` and `dtp_msgs/Target`   -- custom messages, no RViz plugin exists
  * `/dtp/lock_state`, `/mux/state`                -- std_msgs/String in a
    "FIELD|FIELD|detail" form
  * `/dtp/sensors_ok`, `/dtp/perception_ok`        -- bare Bools
  * the proximity veto keep-out                    -- a rule in command_mux, not a published shape

So "open RViz and look at the follower" is not a thing you can do, and during a field test that
means the only way to see what the stack believes is to read a terminal. This node converts all of
it into `visualization_msgs/MarkerArray` on ONE topic, so RViz needs one display and the operator
gets one picture.

IT IS DELIBERATELY PASSIVE
--------------------------
Subscribe-only. It publishes exactly one topic (`/dtp/viz`) that nothing in the flight path
consumes, and it holds no state the stack depends on. It can be started, killed or left out with no
effect on following -- which is the property that makes it safe to run during a test. If it ever
grows a subscriber inside the stack, that property is gone and this comment is a lie.

THE TWO THINGS THAT MAKE MARKER PUBLISHING GO WRONG
---------------------------------------------------
1. **Stale markers persist forever.** A marker is drawn until something deletes it.
   Publish a person
   marker with id 7, lose the track, and id 7 stays on screen -- so the display shows a person who
   is no longer detected, which is precisely the wrong failure for a safety demo. Every marker here
   therefore carries an explicit `lifetime`, so RViz expires it if we stop refreshing it. That is
   more robust than trying to track which ids to DELETE, because it also covers this node dying.
2. **The frame must come from the message, not from a constant.** Target positions are in whatever
   frame perception stamped them (`lidar_init` today, and that name has already been wrong twice in
   this project's history). Hardcoding a frame here would draw everything in the wrong
   place the day
   it changes, and it would look plausible. So the frame is read off `msg.header.frame_id`.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from dtp_msgs.msg import Target, TargetArray

# A person is drawn as a cylinder of roughly human size, centred on the reported position. The
# reported point is a torso centroid, not a foot point, so the cylinder is centred rather than
# standing on the ground plane -- drawing it on the ground would misrepresent what was measured.
PERSON_HEIGHT_M = 1.7
PERSON_RADIUS_M = 0.30

# command_mux's independent human-proximity veto: a CYLINDER, 1.5 m horizontal radius and 2.0 m
# half-height, around EVERY detected person (not only the followed one). Drawn at its true size so
# an operator can see the geometry the veto is actually testing, rather than imagining it.
VETO_RADIUS_M = 1.5
VETO_HALF_HEIGHT_M = 2.0

# Markers expire this long after they are published. Longer than one publish period (the detection
# rate is 10 Hz) so nothing flickers, short enough that a dropped track disappears promptly.
MARKER_TTL_S = 0.6

# *** HOW LONG THIS NODE'S OWN STATE STAYS VALID, WHICH IS NOT THE SAME THING. ***
#
# MARKER_TTL_S only tells RViz when to stop drawing a marker it already has. It says nothing
# about this node, which until 2026-09-12 never dropped an input at all: _targets was
# overwritten on arrival and otherwise kept forever, and _locked_id was set once and never
# cleared. So a detector that died mid-run left people, labels and veto cylinders being
# REPUBLISHED every 200 ms from a frozen list -- each one arriving fresh, so RViz never expired
# them -- while the status block in the same MarkerArray read perception_ok FALSE. And a lock
# that was dropped (target_manager LOST/ABSTAIN publish nothing) left the green LOCKED colour
# and the CTRV forecast on screen indefinitely.
#
# An adversarial review found a marker-verification script passing on exactly that state.
# Drawing stale data is worse than drawing nothing: nothing is visibly wrong.
#
# 0.5 s for targets is ~5 missed frames at the 10 Hz detection rate. 0.35 s for the lock is a
# little above follow_planner's own 0.30 s staleness budget, so the picture does not claim a
# lock the controller has already stopped trusting.
TARGETS_TTL_S = 0.5
LOCK_TTL_S = 0.35

# Namespaces, one per logical layer, so RViz's display tree can toggle them independently.
NS_PERSON = 'people'
NS_LABEL = 'labels'
NS_VETO = 'veto'
NS_PATH = 'forecast'
NS_GOAL = 'standoff_goal'
NS_STATUS = 'status'


def _rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


# Colour carries meaning, and the meanings are chosen so the dangerous states are the ones that
# look wrong: the locked subject is green, an unlocked detection grey, and anything the safety
# layer is unhappy about is red.
C_LOCKED = _rgba(0.18, 0.80, 0.44, 0.55)
C_OTHER = _rgba(0.60, 0.64, 0.66, 0.35)
C_VETO_IDLE = _rgba(0.20, 0.55, 0.90, 0.10)
C_VETO_ACTIVE = _rgba(0.90, 0.25, 0.20, 0.28)
C_PATH = _rgba(1.00, 0.78, 0.20, 0.95)
C_GOAL = _rgba(0.35, 0.70, 1.00, 0.95)
C_TEXT = _rgba(0.95, 0.96, 0.96, 1.0)
C_TEXT_BAD = _rgba(1.00, 0.45, 0.40, 1.0)


class DtpViz(Node):

    def __init__(self):
        super().__init__('dtp_viz')
        self.declare_parameter('targets_topic', '/dtp/targets')
        self.declare_parameter('follow_target_topic', '/dtp/follow_target')
        self.declare_parameter('lock_state_topic', '/dtp/lock_state')
        self.declare_parameter('mux_state_topic', '/mux/state')
        # Defaults to the SHADOW topic, because Stage A (observe-only flight) is the case where a
        # picture matters most: the goals exist but command nothing, so the terminal shows very
        # little. Point it at /planning/follow_goal for a live follow.
        self.declare_parameter('goal_topic', '/dtp/shadow/follow_goal')
        self.declare_parameter('status_anchor_z_m', 2.5)

        def g(name):
            return self.get_parameter(name).value

        self._frame = 'lidar_init'      # replaced by the first message's own frame_id
        self._targets = []
        self._targets_t = None      # when _targets last arrived; see TARGETS_TTL_S
        self._locked_id = None
        # Retained for a future coasted-forecast draw; currently written and never read, so it
        # is initialised here rather than springing into existence on the first message.
        self._locked = None
        self._locked_t = None       # when the locked target last arrived; see LOCK_TTL_S
        self._lock_state = '(no /dtp/lock_state yet)'
        self._mux_state = '(no /mux/state yet)'
        self._sensors_ok = None
        self._perception_ok = None
        self._goal = None
        self._veto_active = False

        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TargetArray, g('targets_topic'), self._on_targets,
                                 qos_profile_sensor_data)
        self.create_subscription(Target, g('follow_target_topic'), self._on_follow_target,
                                 qos_profile_sensor_data)
        self.create_subscription(String, g('lock_state_topic'), self._on_lock, 10)
        self.create_subscription(String, g('mux_state_topic'), self._on_mux, 10)
        self.create_subscription(Bool, '/dtp/sensors_ok', self._on_sensors, best)
        self.create_subscription(Bool, '/dtp/perception_ok', self._on_perception, best)
        self.create_subscription(PoseStamped, g('goal_topic'), self._on_goal,
                                 qos_profile_sensor_data)

        self._pub = self.create_publisher(MarkerArray, '/dtp/viz', 1)
        self.create_timer(0.2, self._publish)

        self.get_logger().info(
            'dtp_viz up: %s + %s -> /dtp/viz (MarkerArray). Passive: publishes only /dtp/viz, '
            'which nothing in the flight path consumes. Goal shown from %s.'
            % (g('targets_topic'), g('follow_target_topic'), g('goal_topic')))

    # -- inputs ------------------------------------------------------------------------------
    def _on_targets(self, msg):
        if msg.header.frame_id:
            self._frame = msg.header.frame_id
        self._targets = list(msg.targets)
        self._targets_t = self.get_clock().now()

    def _on_follow_target(self, msg):
        self._locked_id = int(msg.track_id)
        # The locked target is republished here with its forecast; target_manager coasts it through
        # occlusions, so this can be fresher than anything in /dtp/targets.
        self._locked = msg
        self._locked_t = self.get_clock().now()

    def _on_lock(self, msg):
        self._lock_state = msg.data

    def _on_mux(self, msg):
        self._mux_state = msg.data
        # /mux/state is "STATE|reason|detail"; the reason field carries PROXIMITY when the veto is
        # the thing holding the aircraft. Parsed rather than guessed so the drawn keep-out
        # turns red
        # exactly when the veto is what is acting.
        parts = msg.data.split('|')
        self._veto_active = len(parts) > 1 and 'PROXIMITY' in parts[1]

    def _on_sensors(self, msg):
        self._sensors_ok = bool(msg.data)

    def _on_perception(self, msg):
        self._perception_ok = bool(msg.data)

    def _on_goal(self, msg):
        self._goal = msg

    # -- drawing -----------------------------------------------------------------------------
    def _base(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.lifetime = rclpy.duration.Duration(seconds=MARKER_TTL_S).to_msg()
        m.pose.orientation.w = 1.0
        return m

    def _stale(self, stamp, ttl_s):
        """True when stamp is missing or older than ttl_s.

        Uses the node clock on both sides rather than message stamps: the question here is "has
        this node heard anything lately", not "how old is the observation", and mixing the two
        is how a stale-data check ends up measuring pipeline latency instead.
        """
        if stamp is None:
            return True
        return (self.get_clock().now() - stamp).nanoseconds > ttl_s * 1e9

    def _publish(self):
        arr = MarkerArray()

        # Drop what we have stopped hearing about, BEFORE drawing anything from it.
        if self._stale(self._targets_t, TARGETS_TTL_S):
            self._targets = []
        if self._stale(self._locked_t, LOCK_TTL_S):
            self._locked_id = None
            self._locked = None
        locked = self._locked_id

        for i, t in enumerate(self._targets):
            is_locked = locked is not None and int(t.track_id) == locked
            p = t.position

            body = self._base(NS_PERSON, i, Marker.CYLINDER)
            body.pose.position.x, body.pose.position.y, body.pose.position.z = p.x, p.y, p.z
            body.scale.x = body.scale.y = PERSON_RADIUS_M * 2.0
            body.scale.z = PERSON_HEIGHT_M
            body.color = C_LOCKED if is_locked else C_OTHER
            arr.markers.append(body)

            label = self._base(NS_LABEL, i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = p.x
            label.pose.position.y = p.y
            label.pose.position.z = p.z + PERSON_HEIGHT_M * 0.5 + 0.25
            label.scale.z = 0.22
            label.color = C_TEXT
            rng = math.hypot(p.x, p.y)
            label.text = ('LOCKED #%d  %.2f m  conf %.2f' if is_locked
                          else '#%d  %.2f m  conf %.2f') % (int(t.track_id), rng, t.confidence)
            arr.markers.append(label)

            # The veto keep-out, drawn around EVERY person because that is what command_mux checks.
            veto = self._base(NS_VETO, i, Marker.CYLINDER)
            veto.pose.position.x, veto.pose.position.y, veto.pose.position.z = p.x, p.y, p.z
            veto.scale.x = veto.scale.y = VETO_RADIUS_M * 2.0
            veto.scale.z = VETO_HALF_HEIGHT_M * 2.0
            veto.color = C_VETO_ACTIVE if self._veto_active else C_VETO_IDLE
            arr.markers.append(veto)

            if is_locked and len(t.predicted_path) >= 2:
                path = self._base(NS_PATH, 0, Marker.LINE_STRIP)
                path.scale.x = 0.06
                path.color = C_PATH
                path.points = [Point(x=q.x, y=q.y, z=q.z) for q in t.predicted_path]
                arr.markers.append(path)

        if self._goal is not None:
            gp = self._goal.pose.position
            goal = self._base(NS_GOAL, 0, Marker.SPHERE)
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = gp.x, gp.y, gp.z
            goal.scale.x = goal.scale.y = goal.scale.z = 0.32
            goal.color = C_GOAL
            arr.markers.append(goal)

            gl = self._base(NS_GOAL, 1, Marker.TEXT_VIEW_FACING)
            gl.pose.position.x, gl.pose.position.y = gp.x, gp.y
            gl.pose.position.z = gp.z + 0.35
            gl.scale.z = 0.20
            gl.color = C_GOAL
            gl.text = 'standoff goal'
            arr.markers.append(gl)

        # One text block, anchored above the origin, so the operator reads state and
        # geometry in the
        # same glance instead of alt-tabbing to a terminal.
        st = self._base(NS_STATUS, 0, Marker.TEXT_VIEW_FACING)
        st.pose.position.z = float(self.get_parameter('status_anchor_z_m').value)
        st.scale.z = 0.26
        bad = (self._sensors_ok is False or self._perception_ok is False
               or self._mux_state.split('|')[0] in ('HOLD', 'ABORT'))
        st.color = C_TEXT_BAD if bad else C_TEXT
        st.text = ('lock : %s\nmux  : %s\nsensors_ok %s   perception_ok %s\npeople %d'
                   % (self._lock_state, self._mux_state,
                      self._fmt(self._sensors_ok), self._fmt(self._perception_ok),
                      len(self._targets)))
        arr.markers.append(st)

        self._pub.publish(arr)

    @staticmethod
    def _fmt(v):
        return '?' if v is None else ('OK' if v else 'FALSE')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DtpViz()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how SIGTERM arrives -- the normal stop path for
        # `ros2 launch`. The other six spinning nodes gained this on 2026-09-07; this one
        # was written after and missed it, so every managed stop logged a stack trace and
        # "process has died [exit code 1]" for a node that had simply been told to stop.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
