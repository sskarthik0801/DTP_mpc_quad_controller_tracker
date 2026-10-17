#!/usr/bin/env python3
"""engage_gate_check.py -- the engage gate: the REAL command_mux_node, sandboxed, end to end.

Re-run after ANY change to command_mux (with fence_check.py, descent_guard_check.py and
terrain_check.py, one at a time). ~65 s, needs no aircraft, no LiDAR, no FC, and cannot touch any
of them: every participant is on a private localhost-only DDS domain (92), and the node's goal
and PX4 topics are remapped under /engage_gate_check/, where nothing listens. The fake PX4 mode
is published at 2 Hz, as the aircraft's vehicle_control_mode is.

Why it exists: at the 2026-09-21 20:05 Offboard switch SUPER's pos_cmd was 3.0 m from the
aircraft, which closed the gap at 1.7 m/s. The mux had fed SUPER goals since FOLLOW engaged on
the ground, and SUPER replans from its own reference, not the aircraft. The gate
(command_mux/engage_gate.py) holds every goal back until PX4 reports armed + Offboard. Each case
is one property of that, through the real node:

  1. armed on the ground, not Offboard, FOLLOW healthy: NO goal reaches SUPER, and /mux/state says
     FOLLOW|OK with "held until Offboard";
  2. a HOLD latched on the ground, then a takeoff to 1.5 m (inside the 3 m re-anchor radius):
     still nothing published;
  3. Offboard mid-HOLD: a goal is published within 1 s of the switch, anchored on the aircraft as
     it is NOW (1.5 + 0.75 climb), not the ground HOLD's stale 0.85;
  4. FOLLOW back: the follow goal is published;
  5. the PX4 mode report goes STALE (> 3 s) while in Offboard: goals keep flowing, and a person
     stepping next to the path still gets its HOLD|PROXIMITY goal to SUPER. The gate never
     silences a veto in the air;
  6. a fresh report of the pilot taking over (armed, not Offboard): goals stop;
  7. re-engage: goals resume at once;
  8. engage_gate_enable:=false: goals flow outside Offboard, and /mux/state says ENGAGE GATE OFF;
  9. abort_enable:=false (observe-only): goals flow, with no gate text.

PARKING SUPER (added 2026-09-30; engage_gate.py, SUPER patch 0009). When the gate shuts the mux
tells SUPER to drop its goal, on the park topic (remapped here like the rest):
  P1. nothing is sent on the park topic before the first takeover, stale report included;
  P2. the takeover of case 6 sends the park burst (3 messages) within 2 s;
  P3. the re-engage of case 7 sends no more;
  P4. gate off and observe-only (cases 8, 9) never park;
  P5. engage_gate_park_enable:=false: an engage and a takeover send nothing.

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/engage_gate_check.py
"""
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '92'
NS = '/engage_gate_check'
GOAL_TOPIC = NS + '/follow_goal'
MODE_TOPIC = NS + '/vehicle_control_mode'
PARK_TOPIC = NS + '/park'
PARK_REPEATS = 3               # command_mux PARK_REPEATS
FOLLOW_GOAL = (4.0, 0.0, 1.2)
MODE_PERIOD_S = 0.5            # the aircraft's vehicle_control_mode is ~2 Hz
HOLD_CLIMB = 0.75              # command_mux DEFAULT_HOLD_CLIMB_M


def _env():
    return dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')


def _start_mux(log, extra):
    argv = ['ros2', 'run', 'command_mux', 'command_mux_node', '--ros-args',
            '-p', 'arbitrated_goal_topic:=%s' % GOAL_TOPIC,
            '-p', 'vehicle_command_topic:=%s/vehicle_command' % NS,
            '-p', 'vehicle_command_ack_topic:=%s/vehicle_command_ack' % NS,
            '-p', 'vehicle_control_mode_topic:=%s' % MODE_TOPIC,
            '-p', 'park_topic:=%s' % PARK_TOPIC]
    for k, v in extra.items():
        argv += ['-p', '%s:=%s' % (k, str(v).lower() if isinstance(v, bool) else v)]
    return subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            env=_env(), start_new_session=True)


def _stop(proc):
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


class Harness:
    """Fake odom / targets / sensors_ok / follow goal / PX4 mode, plus goal and state listeners."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy, qos_profile_sensor_data)
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, Empty, String
        from dtp_msgs.msg import Target, TargetArray
        from px4_msgs.msg import VehicleControlMode

        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node('engage_gate_check')
        self.PoseStamped, self.Odometry, self.Bool = PoseStamped, Odometry, Bool
        self.Target, self.TargetArray, self.VCM = Target, TargetArray, VehicleControlMode
        self.drone = [0.0, 0.0, 0.1]
        self.person = None          # (x, y) or None
        self.armed = True
        self.offboard = False
        self.mode_on = True         # False: the PX4 report goes silent (stale)
        self.follow_on = True
        self.goals = []
        self.states = []
        self.parks = []
        self._last_mode = 0.0

        best = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                          durability=QoSDurabilityPolicy.VOLATILE)
        px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.node.create_subscription(PoseStamped, GOAL_TOPIC, self._on_goal, best)
        self.node.create_subscription(String, '/mux/state', self._on_state, 10)
        self.node.create_subscription(Empty, PARK_TOPIC, self._on_park, best)
        self.odom_pub = self.node.create_publisher(Odometry, '/lidar_slam/odom',
                                                   qos_profile_sensor_data)
        self.targets_pub = self.node.create_publisher(TargetArray, '/dtp/targets',
                                                      qos_profile_sensor_data)
        self.sensors_pub = self.node.create_publisher(Bool, '/dtp/sensors_ok', 10)
        self.follow_pub = self.node.create_publisher(PoseStamped, '/dtp/follow_goal',
                                                     qos_profile_sensor_data)
        self.mode_pub = self.node.create_publisher(VehicleControlMode, MODE_TOPIC, px4)
        self.node.create_timer(1.0 / 15.0, self._feed)

    def _on_goal(self, m):
        self.goals.append((time.time(), m.pose.position.x, m.pose.position.y,
                           m.pose.position.z))

    def _on_park(self, _m):
        self.parks.append(time.time())

    def _on_state(self, m):
        self.states.append((time.time(), m.data))

    def _feed(self):
        now = self.node.get_clock().now().to_msg()
        o = self.Odometry()
        o.header.stamp = now
        o.header.frame_id = 'world'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.drone
        o.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(o)

        arr = self.TargetArray()
        arr.header.stamp = now
        arr.header.frame_id = 'world'
        if self.person is not None:
            t = self.Target()
            t.header = arr.header
            t.track_id = 1
            t.position.x, t.position.y, t.position.z = self.person[0], self.person[1], 0.2
            t.track_confirmed = True
            arr.targets.append(t)
        self.targets_pub.publish(arr)
        self.sensors_pub.publish(self.Bool(data=True))

        if self.follow_on:
            g = self.PoseStamped()
            g.header.stamp = now
            g.header.frame_id = 'world'
            g.pose.position.x, g.pose.position.y, g.pose.position.z = FOLLOW_GOAL
            g.pose.orientation.w = 1.0
            self.follow_pub.publish(g)

        t = time.time()
        if self.mode_on and t - self._last_mode >= MODE_PERIOD_S:
            self._last_mode = t
            m = self.VCM()
            m.timestamp = int(t * 1e6)
            m.flag_armed = self.armed
            m.flag_control_offboard_enabled = self.offboard
            self.mode_pub.publish(m)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def goals_since(self, t):
        return [g for g in self.goals if g[0] >= t]

    def state(self):
        return self.states[-1][1] if self.states else ''

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def _near(a, b, tol=0.02):
    return a is not None and all(abs(x - y) <= tol for x, y in zip(a, b))


def main():
    results = []

    def check(name, ok, detail=''):
        results.append((name, bool(ok), detail))
        print('  [%s] %s  %s' % ('PASS' if ok else 'FAIL', name, detail))
        sys.stdout.flush()

    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = tempfile.mkdtemp(prefix='engage_gate_check_')
    print('\nengage_gate_check -- installed command_mux_node on ROS_DOMAIN_ID %s (localhost '
          'only); logs in %s\n' % (DOMAIN, logdir))

    h = Harness()
    log = open(os.path.join(logdir, 'mux_gate_on.log'), 'w')
    proc = _start_mux(log, {})
    try:
        # 1. on the ground, armed, not Offboard, FOLLOW healthy
        h.spin(8.0)                     # discovery, first healthy tick, FOLLOW engage dwell
        st = h.state()
        check('1 not Offboard: FOLLOW|OK, nothing published to SUPER, state says held',
              st.startswith('FOLLOW|OK') and 'held until Offboard' in st and not h.goals,
              '%d goals | %s' % (len(h.goals), st[:110]))

        # 2. a HOLD latched on the ground, then a takeoff inside the re-anchor radius
        h.follow_on = False
        h.spin(1.5)
        hold_st = h.state()
        h.drone = [0.0, 0.0, 1.5]
        h.spin(1.5)
        check('2 ground HOLD, then takeoff to 1.5 m: still nothing published',
              hold_st.startswith('HOLD|NO_SOURCE') and not h.goals,
              '%d goals | %s' % (len(h.goals), hold_st[:60]))

        # 3. Offboard mid-HOLD: published at once, anchored on the aircraft NOW
        h.offboard = True
        t_eng = time.time()
        h.spin(1.5)
        first = h.goals[0] if h.goals else None
        lat = (first[0] - t_eng) if first else None
        check('3 Offboard mid-HOLD: goal within 1 s, at the aircraft now (z 2.25, not 0.85)',
              first is not None and lat <= 1.0
              and _near(first[1:], (0.0, 0.0, 1.5 + HOLD_CLIMB)),
              'first %s after %s s | %s'
              % (first[1:] if first else None, '%.2f' % lat if lat is not None else '-',
                 h.state()[:60]))

        # 4. FOLLOW back
        h.follow_on = True
        h.spin(2.5)
        last = h.goals[-1] if h.goals else None
        check('4 FOLLOW back: the follow goal is published',
              last is not None and _near(last[1:], FOLLOW_GOAL)
              and h.state().startswith('FOLLOW|OK') and 'held' not in h.state(),
              'last %s | %s' % (last[1:] if last else None, h.state()[:60]))

        # 5. the PX4 report goes stale in Offboard: goals keep flowing, and a veto still lands
        h.mode_on = False
        h.spin(4.0)                     # > PX4_MODE_STALE_S 3.0
        t5 = time.time()
        h.spin(1.5)
        flowing = len(h.goals_since(t5))
        h.person = (2.0, 0.5)           # beside the drone->goal path
        t5b = time.time()
        h.spin(1.5)
        hold = [g for g in h.goals_since(t5b) if _near(g[1:], (0.0, 0.0, 1.5 + HOLD_CLIMB))]
        st5 = h.state()
        check('5 stale PX4 report in Offboard: goals keep flowing, HOLD|PROXIMITY reaches SUPER',
              flowing >= 2 and hold and st5.startswith('HOLD|PROXIMITY') and 'held' not in st5,
              '%d goals in 1.5 s stale; %d hold goals after the person | %s'
              % (flowing, len(hold), st5[:60]))
        h.person = None
        h.mode_on = True
        h.spin(3.0)
        check('P1 no park before the first takeover (ground, engage, stale report)',
              not h.parks, '%d park message(s)' % len(h.parks))

        # 6. a fresh report of the pilot taking over: goals stop
        h.offboard = False
        t_take = time.time()
        h.spin(1.5)                     # up to one mode period + a tick
        t6 = time.time()
        h.spin(3.0)
        after = h.goals_since(t6)
        check('6 pilot takes over (fresh report, not Offboard): goals stop, state says held',
              not after and 'held until Offboard' in h.state(),
              '%d goals in 3 s | %s' % (len(after), h.state()[:80]))
        lat_p = (h.parks[0] - t_take) if h.parks else None
        check('P2 the takeover parks SUPER: %d park messages, the first within 2 s' % PARK_REPEATS,
              len(h.parks) == PARK_REPEATS and lat_p <= 2.0,
              '%d park message(s), first after %s s'
              % (len(h.parks), '%.2f' % lat_p if lat_p is not None else '-'))

        # 7. re-engage
        h.offboard = True
        t7 = time.time()
        h.spin(1.5)
        g7 = h.goals_since(t7)
        lat7 = (g7[0][0] - t7) if g7 else None
        check('7 re-engage: goals resume within 1 s',
              g7 and lat7 <= 1.0 and 'held' not in h.state(),
              '%d goals, first after %s s'
              % (len(g7), '%.2f' % lat7 if lat7 is not None else '-'))
        check('P3 the re-engage sends no park', len(h.parks) == PARK_REPEATS,
              '%d park message(s) in all' % len(h.parks))
    finally:
        _stop(proc)
        log.close()

    # 8. the bench switch
    h.goals.clear()
    h.states.clear()
    h.offboard = False
    h.drone = [0.0, 0.0, 1.5]
    log = open(os.path.join(logdir, 'mux_gate_off.log'), 'w')
    proc = _start_mux(log, {'engage_gate_enable': False})
    try:
        h.spin(6.0)
        check('8 engage_gate_enable:=false: goals flow outside Offboard, state says ENGAGE GATE '
              'OFF', len(h.goals) >= 2 and 'ENGAGE GATE OFF' in h.state(),
              '%d goals | %s' % (len(h.goals), h.state()[:90]))
    finally:
        _stop(proc)
        log.close()

    # 9. observe-only
    h.goals.clear()
    h.states.clear()
    log = open(os.path.join(logdir, 'mux_observe.log'), 'w')
    proc = _start_mux(log, {'abort_enable': False})
    try:
        h.spin(6.0)
        st = h.state()
        check('9 abort_enable:=false (observe-only): goals flow, no gate text',
              len(h.goals) >= 2 and 'held' not in st and 'ENGAGE GATE' not in st,
              '%d goals | %s' % (len(h.goals), st[:90]))
    finally:
        _stop(proc)
        log.close()

    check('P4 gate off and observe-only never park', len(h.parks) == PARK_REPEATS,
          '%d park message(s) in all' % len(h.parks))

    # P5. the park switched off: engage, then a takeover
    h.goals.clear()
    h.states.clear()
    h.offboard = False
    log = open(os.path.join(logdir, 'mux_park_off.log'), 'w')
    proc = _start_mux(log, {'engage_gate_park_enable': False})
    try:
        h.spin(6.0)
        h.offboard = True
        h.spin(2.0)
        engaged = len(h.goals)
        h.offboard = False
        h.spin(3.0)
        check('P5 engage_gate_park_enable:=false: engaged, taken over, no park sent',
              engaged >= 1 and 'held until Offboard' in h.state()
              and len(h.parks) == PARK_REPEATS,
              '%d goals while engaged, %d park message(s) in all | %s'
              % (engaged, len(h.parks), h.state()[:60]))
    finally:
        _stop(proc)
        log.close()

    h.close()
    for name in ('mux_gate_on.log', 'mux_gate_off.log', 'mux_observe.log', 'mux_park_off.log'):
        text = open(os.path.join(logdir, name)).read()
        if 'Traceback' in text:
            results.append(('no traceback in ' + name, False, ''))
            print('  [FAIL] traceback in %s' % name)

    n_pass = sum(1 for _, ok, _ in results if ok)
    tail = '' if n_pass == len(results) else '  -- logs in ' + logdir
    print('\n%d/%d passed%s' % (n_pass, len(results), tail))
    return 0 if n_pass == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
