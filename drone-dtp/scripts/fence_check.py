#!/usr/bin/env python3
"""fence_check.py -- the M5.2 geofence gate: the REAL command_mux_node, sandboxed, end to end.

Re-run after ANY change to command_mux. ~40 s, needs no aircraft, no LiDAR, no FC, and cannot
touch any of them: every participant is on a private localhost-only DDS domain, and the node's
PX4 command topic is remapped under /fence_check/ besides, so the NAV_LAND it sends in case 6
lands nothing. Safe to run with the flight stack up (it was written with it up).

What it proves, each as a case:
  1. a NAV goal inside the box passes through untouched;
  2. a NAV goal outside is clamped onto the INSET edge (margin), and the state stays NAV -- a
     subject who walks out of the box is followed to the edge and no further, not vetoed;
  3. the aircraft 2 m outside (under the 3 m hard boundary) is REPORTED in /mux/state, the goal
     stays clamped inside, and there is no veto;
  4. the aircraft 4 m outside raises HOLD|FENCE, and the hold goal is clamped INSIDE the box
     (commanding it back in), not parked where the aircraft is;
  5. the hold goal is latched -- it does not run away or re-anchor at tick rate while the
     aircraft sits outside;
  6. with authority (bench flag) the FENCE veto escalates to ABORT|LAND after abort_after_s and a
     NAV_LAND VehicleCommand is sent; it is terminal;
  7. a box that does not contain the origin is refused at startup, with the reason;
  8. fence_enable:=false publishes far goals unclamped and says FENCE OFF in /mux/state.

No PX4 mode is faked here, so the engage gate is off (engage_gate_enable:=false): with it on, no
goal would be published. engage_gate_check.py covers the gate.

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/fence_check.py
"""
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '89'
NS = '/fence_check'
GOAL_TOPIC = NS + '/follow_goal'
CMD_TOPIC = NS + '/vehicle_command'
STATE_TOPIC = '/mux/state'
ABORT_AFTER_S = 1.5

# The box under test: x [-2, 8], y [-2, 2], margin 0.5 => goals inside x [-1.5, 7.5], y [-1.5, 1.5]
BOX = dict(fence_x_min_m=-2.0, fence_x_max_m=8.0, fence_y_min_m=-2.0, fence_y_max_m=2.0,
           fence_margin_m=0.5, fence_breach_land_m=3.0)


def _env():
    return dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')


def _start_mux(log, extra_params):
    params = dict(BOX)
    params.update(extra_params)
    argv = ['ros2', 'run', 'command_mux', 'command_mux_node', '--ros-args',
            '-p', 'arbitrated_goal_topic:=%s' % GOAL_TOPIC,
            '-p', 'vehicle_command_topic:=%s' % CMD_TOPIC,
            '-p', 'vehicle_command_ack_topic:=%s/vehicle_command_ack' % NS,
            '-p', 'vehicle_control_mode_topic:=%s/vehicle_control_mode' % NS,
            '-p', 'abort_after_s:=%.2f' % ABORT_AFTER_S,
            # No PX4 mode is faked here (module docstring); engage_gate_check.py covers the gate.
            '-p', 'engage_gate_enable:=false']
    for k, v in params.items():
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
    """Fake odom / targets / sensors_ok publishers plus goal / state / command listeners."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy, qos_profile_sensor_data)
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, String
        from dtp_msgs.msg import TargetArray
        from px4_msgs.msg import VehicleCommand

        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node('fence_check')
        self.PoseStamped, self.Odometry, self.Bool, self.TargetArray = (
            PoseStamped, Odometry, Bool, TargetArray)
        self.odom_xyz = (0.0, 0.0, 1.0)
        self.goals = []
        self.states = []
        self.commands = []

        best = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                          durability=QoSDurabilityPolicy.VOLATILE)
        px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.node.create_subscription(PoseStamped, GOAL_TOPIC, self._on_goal, best)
        self.node.create_subscription(String, STATE_TOPIC, self._on_state, 10)
        self.node.create_subscription(VehicleCommand, CMD_TOPIC, self._on_cmd, px4)

        self.odom_pub = self.node.create_publisher(Odometry, '/lidar_slam/odom',
                                                   qos_profile_sensor_data)
        self.targets_pub = self.node.create_publisher(TargetArray, '/dtp/targets',
                                                      qos_profile_sensor_data)
        self.sensors_pub = self.node.create_publisher(Bool, '/dtp/sensors_ok', 10)
        self.nav_pub = self.node.create_publisher(PoseStamped, '/dtp/nav_goal',
                                                  qos_profile_sensor_data)
        self.node.create_timer(0.05, self._feed)

    def _on_goal(self, m):
        self.goals.append((time.time(), m.pose.position.x, m.pose.position.y,
                           m.pose.position.z))

    def _on_state(self, m):
        self.states.append((time.time(), m.data))

    def _on_cmd(self, m):
        self.commands.append((time.time(), int(m.command)))

    def _feed(self):
        now = self.node.get_clock().now().to_msg()
        o = self.Odometry()
        o.header.stamp = now
        o.header.frame_id = 'world'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.odom_xyz
        o.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(o)
        t = self.TargetArray()
        t.header.stamp = now
        t.header.frame_id = 'world'
        self.targets_pub.publish(t)
        self.sensors_pub.publish(self.Bool(data=True))

    def nav(self, x, y, z=1.5):
        m = self.PoseStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        self.nav_pub.publish(m)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def latest_goal(self):
        return self.goals[-1][1:] if self.goals else None

    def latest_state(self):
        return self.states[-1][1] if self.states else None

    def goals_since(self, t):
        return [g[1:] for g in self.goals if g[0] >= t]

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def _near(a, b, tol=1e-3):
    return a is not None and all(abs(x - y) <= tol for x, y in zip(a, b))


def main():
    results = []

    def check(name, ok, detail=''):
        results.append((name, bool(ok), detail))
        print('  [%s] %s  %s' % ('PASS' if ok else 'FAIL', name, detail))
        sys.stdout.flush()

    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = tempfile.mkdtemp(prefix='fence_check_')
    print('\nfence_check -- real command_mux_node on ROS_DOMAIN_ID %s (localhost only); '
          'logs in %s\n' % (DOMAIN, logdir))

    # ---- 7. startup refusal (no harness needed) --------------------------------------------
    with open(os.path.join(logdir, 'refusal.log'), 'w+') as log:
        proc = _start_mux(log, {'fence_x_min_m': 1.0, 'abort_enable': False})
        try:
            rc = proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            rc = None
            _stop(proc)
        log.seek(0)
        text = log.read()
    check('7 box excluding the origin is refused at startup',
          rc not in (None, 0) and 'does not contain the origin' in text,
          'exit %s' % rc)

    # ---- main run: bench-flag authority so case 6 can fire ---------------------------------
    h = Harness()
    log = open(os.path.join(logdir, 'mux.log'), 'w')
    proc = _start_mux(log, {'abort_enable': True, 'abort_require_offboard': False})
    try:
        h.spin(4.0)                      # discovery + first healthy tick
        check('0 node up and publishing /mux/state', h.latest_state() is not None,
              str(h.latest_state()))

        # 1. inside
        h.nav(5.0, 0.0)
        h.spin(1.0)
        check('1 NAV goal inside the box passes untouched',
              _near(h.latest_goal(), (5.0, 0.0, 1.5))
              and (h.latest_state() or '').startswith('NAV|OK'),
              'goal %s state %s' % (h.latest_goal(), h.latest_state()))

        # 2. outside -> clamped to the inset edge, still NAV
        h.nav(20.0, 3.0)
        h.spin(1.0)
        check('2 NAV goal outside is clamped to the inset edge, state stays NAV',
              _near(h.latest_goal(), (7.5, 1.5, 1.5))
              and (h.latest_state() or '').startswith('NAV|OK'),
              'goal %s state %s' % (h.latest_goal(), h.latest_state()))

        # 3. aircraft 2 m outside: reported, not vetoed, goal still inside
        h.odom_xyz = (5.0, 4.0, 1.0)
        h.spin(1.5)
        st = h.latest_state() or ''
        check('3 aircraft 2 m outside is reported and not vetoed',
              st.startswith('NAV|OK') and 'fence: aircraft 2.00 m outside' in st
              and _near(h.latest_goal(), (7.5, 1.5, 1.5)),
              'state %s' % st)

        # 4. aircraft 4 m outside: HOLD|FENCE, hold goal clamped inside
        h.odom_xyz = (5.0, 6.0, 1.0)
        h.spin(0.8)
        st = h.latest_state() or ''
        g = h.latest_goal()
        check('4 aircraft 4 m outside raises HOLD|FENCE with the hold goal clamped inside',
              st.startswith('HOLD|FENCE') and _near(g, (5.0, 1.5, 1.75)),
              'state %s goal %s' % (st, g))

        # 5. latched: every goal published from HERE on (HOLD established, 2 Hz republish) is
        # the same point. The window starts only now so the last NAV republish before the odom
        # change took effect cannot be counted against it.
        t0 = time.time()
        h.spin(1.2)
        since = h.goals_since(t0)
        distinct = {tuple(round(v, 3) for v in x) for x in since}
        check('5 hold goal is latched (no re-anchor churn while outside)',
              len(since) >= 1 and len(distinct) == 1, '%d goals, %d distinct' % (len(since),
                                                                                 len(distinct)))

        # 6. escalation with bench-flag authority
        h.spin(ABORT_AFTER_S + 1.0)
        st = h.latest_state() or ''
        cmds = [c for _, c in h.commands]
        check('6 FENCE persisting escalates to ABORT|LAND and sends NAV_LAND (21)',
              st.startswith('ABORT|LAND') and 'FENCE' in st and 21 in cmds,
              'state %s commands %s' % (st, cmds))
        # ...and is terminal: back inside, still ABORT
        h.odom_xyz = (0.0, 0.0, 1.0)
        h.spin(1.0)
        check('6b abort is terminal after returning inside',
              (h.latest_state() or '').startswith('ABORT|LAND'), h.latest_state())
    finally:
        _stop(proc)
        log.close()
        h.close()

    # ---- 8. disabled ------------------------------------------------------------------------
    h = Harness()
    log = open(os.path.join(logdir, 'mux_off.log'), 'w')
    proc = _start_mux(log, {'abort_enable': False, 'fence_enable': False})
    try:
        h.spin(4.0)
        h.nav(20.0, 3.0)
        h.spin(1.0)
        st = h.latest_state() or ''
        check('8 fence_enable:=false passes far goals and says FENCE OFF',
              _near(h.latest_goal(), (20.0, 3.0, 1.5)) and 'FENCE OFF' in st,
              'goal %s state %s' % (h.latest_goal(), st))
    finally:
        _stop(proc)
        log.close()
        h.close()
        with open(os.path.join(logdir, 'mux_off.log')) as f:
            off_text = f.read()
    check('8b disabled fence is logged at ERROR', 'GEOFENCE DISABLED' in off_text)

    with open(os.path.join(logdir, 'mux.log')) as f:
        main_text = f.read()
    check('9 publish-point re-check never bit (no code path skipped the clamp)',
          'AT THE PUBLISHER' not in main_text)
    # Exactly one re-anchor is expected: the deliberate one when case 6b moved the aircraft 7.8 m
    # back to the origin. Any more means the fence-clamped hold goal was churning the latch.
    check('5b no hold re-anchor churn while outside (exactly the one on return to origin)',
          main_text.count('hold point re-anchored') == 1,
          '%d re-anchor lines' % main_text.count('hold point re-anchored'))

    n_ok = sum(1 for _, ok, _ in results if ok)
    print('\n%d/%d passed -- logs in %s' % (n_ok, len(results), logdir))
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
