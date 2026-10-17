#!/usr/bin/env python3
"""descent_guard_check.py -- the descent-guard latch gate: the REAL command_mux_node, sandboxed.

Re-run after ANY change to command_mux (with fence_check.py). ~45 s, needs no aircraft, no LiDAR,
no FC, and cannot touch any of them: every participant is on a private localhost-only DDS domain
(86), and the node's PX4 topics are remapped under /descent_guard_check/. The fake PX4 state says
armed but NOT Offboard, except during case 4b, which needs Offboard for the node to count its
HOLD as flown. Nothing listens on the remapped command topic, and no case raises a veto that
could escalate to a Land. The engage gate is OFF here (engage_gate_enable:=false): these cases
need goals while armed and NOT Offboard, which is exactly what the gate withholds.
engage_gate_check.py covers the gate.

Why it exists: in the 2026-09-15 20:08 flight the guard re-decided every tick, the follow goal sat
2.77-3.07 m from the walker against a 3.00 m radius, and the goal's altitude flapped between
0.50 m and the aircraft's altitude for the whole follow. The guard now latches a floor
(command_mux/descent_guard.py). Each case below is one property of that, through the real node:

  1. nobody near: the follow goal passes through at its own z;
  1b. a HOLD while PX4 is NOT flying the mux's goals (on the ground, or the pilot in Position),
     then a takeoff to 1.5 m: case 2 must still latch at 1.5. If the node counted that HOLD,
     the guard would see the drone at ground level for the rest of the flight and never trip;
  2. a person 2.8 m from the goal latches the floor at the drone's altitude, and /mux/state
     shows it;
  3. the person walking back and forth across the 3.0 m radius does NOT flap the goal's z
     (the old node, run with --node-script, fails exactly this);
  4. the drone dipping below the floor does not drag the floor down;
  4b. the 2026-09-21 18:41 flight: in Offboard, the follow goal goes quiet, the mux HOLDs and
     climbs 0.75 m, FOLLOW comes back -- and the floor stays at 1.50 (it went to 2.10 in the
     air). The pre-2026-09-21 node, run with --node-script, fails exactly this;
  5. a rise inside the raise margin leaves the floor; a real climb raises it;
  6. a person inside radius + margin holds the latch; clear of it, the floor is kept for the
     dwell and then released;
  7. a disarm drops the floor, and the next goal re-latches at the new altitude;
  8. a goal above the floor is never lowered.

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/descent_guard_check.py
    python3 drone-dtp/scripts/descent_guard_check.py --node-script OLD_command_mux_node.py
    python3 drone-dtp/scripts/descent_guard_check.py --mux-param brake_envelope_enable:=true
      (the aircraft here never moves, so the speed-scaled radii must change nothing: 13/13 again)
"""
import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '86'
NS = '/descent_guard_check'
GOAL_TOPIC = NS + '/follow_goal'
MODE_TOPIC = NS + '/vehicle_control_mode'
GOAL_XY = (1.0, 0.0)
DRONE_Z = 1.5


def _env():
    return dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')


def _start_mux(log, node_script, extra_params=()):
    ros_args = ['--ros-args',
                '-p', 'arbitrated_goal_topic:=%s' % GOAL_TOPIC,
                '-p', 'vehicle_command_topic:=%s/vehicle_command' % NS,
                '-p', 'vehicle_command_ack_topic:=%s/vehicle_command_ack' % NS,
                '-p', 'vehicle_control_mode_topic:=%s' % MODE_TOPIC,
                # Goals are needed outside Offboard (module docstring); engage_gate_check.py
                # covers the gate itself.
                '-p', 'engage_gate_enable:=false']
    for kv in extra_params:
        ros_args += ['-p', kv]
    if node_script:
        argv = [sys.executable, node_script] + ros_args
    else:
        argv = ['ros2', 'run', 'command_mux', 'command_mux_node'] + ros_args
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
        from std_msgs.msg import Bool, String
        from dtp_msgs.msg import Target, TargetArray
        from px4_msgs.msg import VehicleControlMode

        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node('descent_guard_check')
        self.PoseStamped, self.Odometry, self.Bool = PoseStamped, Odometry, Bool
        self.Target, self.TargetArray, self.VehicleControlMode = (
            Target, TargetArray, VehicleControlMode)
        self.drone_z = DRONE_Z
        self.goal_z = 0.5
        self.person = None          # (x, y) or None
        self.armed = True
        self.offboard = False
        self.follow_on = True       # False: the follow source goes quiet -> HOLD|NO_SOURCE
        self.goals = []
        self.states = []

        best = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                          durability=QoSDurabilityPolicy.VOLATILE)
        px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.node.create_subscription(PoseStamped, GOAL_TOPIC, self._on_goal, best)
        self.node.create_subscription(String, '/mux/state', self._on_state, 10)
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

    def _on_state(self, m):
        self.states.append((time.time(), m.data))

    def _feed(self):
        now = self.node.get_clock().now().to_msg()
        o = self.Odometry()
        o.header.stamp = now
        o.header.frame_id = 'world'
        o.pose.pose.position.z = self.drone_z
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
            t.is_locked_target = True
            arr.targets.append(t)
        self.targets_pub.publish(arr)
        self.sensors_pub.publish(self.Bool(data=True))

        if self.follow_on:
            g = self.PoseStamped()
            g.header.stamp = now
            g.header.frame_id = 'world'
            g.pose.position.x, g.pose.position.y, g.pose.position.z = GOAL_XY + (self.goal_z,)
            g.pose.orientation.w = 1.0
            self.follow_pub.publish(g)

        m = self.VehicleControlMode()
        m.timestamp = int(time.time() * 1e6)
        m.flag_armed = self.armed
        m.flag_control_offboard_enabled = self.offboard
        self.mode_pub.publish(m)

    def hold_episode(self, climb_to=None):
        """Silence the follow source until the mux HOLDs, optionally fly the drone up to
        climb_to, then restore the source and wait out the FOLLOW engage dwell. Returns
        (window, z of the HOLD goal, state during the HOLD)."""
        t0 = time.time()
        self.follow_on = False
        self.spin(1.0)
        hold_state = self.state()
        hold_z = self.z()
        if climb_to is not None:
            self.drone_z = climb_to
            self.spin(0.5)
        self.follow_on = True
        self.spin(2.0)
        return (t0, time.time()), hold_z, hold_state

    def person_from_goal(self, d):
        self.person = (GOAL_XY[0] + d, GOAL_XY[1])

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def z(self):
        return self.goals[-1][3] if self.goals else None

    def zs_since(self, t):
        return [g[3] for g in self.goals if g[0] >= t]

    def state(self):
        return self.states[-1][1] if self.states else ''

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def _eq(a, b, tol=1e-3):
    return a is not None and abs(a - b) <= tol


def _flips(zs):
    levels = [round(z, 2) for z in zs]
    return sum(1 for a, b in zip(levels, levels[1:]) if a != b)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--node-script', default=None,
                    help='run this command_mux_node.py file instead of the installed node '
                         '(e.g. the pre-latch version from git, to reproduce the flap)')
    ap.add_argument('--mux-param', action='append', default=[], metavar='KEY:=VALUE',
                    help='an extra command_mux parameter (repeatable), e.g. '
                         'brake_envelope_enable:=true')
    args = ap.parse_args()

    results = []
    hold_windows = []               # deliberate HOLD|NO_SOURCE episodes, excluded from check 10

    def check(name, ok, detail=''):
        results.append((name, bool(ok), detail))
        print('  [%s] %s  %s' % ('PASS' if ok else 'FAIL', name, detail))
        sys.stdout.flush()

    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = tempfile.mkdtemp(prefix='descent_guard_check_')
    print('\ndescent_guard_check -- %s on ROS_DOMAIN_ID %s (localhost only); logs in %s%s\n'
          % (args.node_script or 'installed command_mux_node', DOMAIN, logdir,
             ('; extra parameters: ' + ' '.join(args.mux_param)) if args.mux_param else ''))

    h = Harness()
    log = open(os.path.join(logdir, 'mux.log'), 'w')
    proc = _start_mux(log, args.node_script, args.mux_param)
    try:
        h.spin(5.0)                     # discovery, first healthy tick, FOLLOW engage dwell
        check('0 node up, FOLLOW engaged', h.state().startswith('FOLLOW|OK'), h.state())

        # 1. nobody near
        h.spin(1.0)
        check('1 nobody near: goal passes at its own z', _eq(h.z(), 0.5),
              'z %s' % h.z())

        # 1b. a HOLD that commands nothing: armed, not Offboard, drone on the ground
        h.drone_z = 0.1
        window, hold_z, hold_state = h.hold_episode()
        hold_windows.append(window)
        h.drone_z = DRONE_Z                     # the pilot takes off to 1.5 m
        h.spin(0.5)
        check('1b a HOLD outside Offboard (ground, then takeoff): not counted as a HOLD climb',
              hold_state.startswith('HOLD|NO_SOURCE') and h.state().startswith('FOLLOW|OK')
              and _eq(h.z(), 0.5),
              'during: %s | after: %s z %s' % (hold_state[:40], h.state()[:40], h.z()))

        # 2. latch
        h.person_from_goal(2.8)
        h.spin(1.0)
        check('2 person 2.8 m from the goal latches the floor at the drone altitude',
              _eq(h.z(), DRONE_Z) and 'alt floor 1.50' in h.state(),
              'z %s state %s' % (h.z(), h.state()))

        # 3. walking back and forth across the radius
        t0 = time.time()
        for i in range(14):
            h.person_from_goal(2.8 if i % 2 == 0 else 3.2)
            h.spin(0.3)
        zs = h.zs_since(t0)
        check('3 person crossing the 3.0 m radius does not flap the goal z',
              len(zs) >= 3 and _flips(zs) == 0 and all(_eq(z, DRONE_Z) for z in zs),
              '%d goals, %d flips, z in [%.2f, %.2f]'
              % (len(zs), _flips(zs), min(zs, default=-1), max(zs, default=-1)))

        # 4. a dip does not drag the floor down
        h.person_from_goal(2.8)
        h.drone_z = 0.8
        h.spin(1.0)
        check('4 drone dips to 0.8 m: goal stays at the 1.50 floor', _eq(h.z(), DRONE_Z),
              'z %s' % h.z())

        # 4b. the 2026-09-21 flight: a HOLD in Offboard climbs 0.75 m; the floor must stay
        h.drone_z = DRONE_Z
        h.offboard = True
        h.spin(1.0)
        window, hold_z, hold_state = h.hold_episode(climb_to=DRONE_Z + 0.75)
        hold_windows.append(window)
        resumed_z, resumed_state = h.z(), h.state()
        h.drone_z = DRONE_Z                     # SUPER brings it back down to the floor
        h.spin(1.0)
        h.offboard = False
        check('4b in Offboard, a HOLD climbs to 2.25 m and FOLLOW resumes: floor stays 1.50',
              hold_state.startswith('HOLD|NO_SOURCE') and _eq(hold_z, DRONE_Z + 0.75)
              and resumed_state.startswith('FOLLOW|OK') and _eq(resumed_z, DRONE_Z)
              and 'alt floor 1.50' in resumed_state,
              'HOLD goal z %s; after FOLLOW resumed at 2.25 m: z %s, %s'
              % (hold_z, resumed_z, resumed_state.split('|')[-1].strip()))

        # 5. small rise ignored, real climb raises
        h.drone_z = 1.75
        h.spin(1.0)
        small = h.z()
        h.drone_z = 2.1
        h.spin(1.0)
        check('5 +0.25 m leaves the floor; a climb to 2.1 m raises it',
              _eq(small, DRONE_Z) and _eq(h.z(), 2.1) and 'alt floor 2.10' in h.state(),
              'after +0.25: %s, after climb: %s' % (small, h.z()))

        # 6. release: held inside radius + margin, then kept for the dwell, then released
        h.person_from_goal(3.3)
        h.spin(2.0)
        held = h.z()
        h.person_from_goal(6.0)
        h.spin(0.5)
        in_dwell = h.z()
        h.spin(1.5)
        check('6 held at 3.3 m, kept 0.5 s into the dwell, released after it',
              _eq(held, 2.1) and _eq(in_dwell, 2.1) and _eq(h.z(), 0.5)
              and 'alt floor' not in h.state(),
              'at 3.3 m: %s, dwell: %s, after: %s' % (held, in_dwell, h.z()))

        # 7. disarm drops the floor
        h.person_from_goal(2.8)
        h.spin(1.0)
        latched = h.z()
        h.drone_z = 1.2
        h.spin(1.0)
        before = h.z()
        h.armed = False
        h.spin(1.0)
        check('7 disarm drops the 2.10 floor; the next goal re-latches at 1.20',
              _eq(latched, 2.1) and _eq(before, 2.1) and _eq(h.z(), 1.2),
              'latched %s, before disarm %s, after %s' % (latched, before, h.z()))

        # 8. a goal above the floor
        h.goal_z = 2.5
        h.spin(1.0)
        check('8 a goal above the floor is never lowered', _eq(h.z(), 2.5), 'z %s' % h.z())
    finally:
        _stop(proc)
        log.close()
        h.close()

    with open(os.path.join(logdir, 'mux.log')) as f:
        text = f.read()
    check('9 log names each event once per episode, and no HOLD climb raised the floor',
          text.count('floor latched') == 3 and 'floor raised to 2.10' in text
          and text.count('floor raised') == 1
          and 'floor released' in text and 'dropped on disarm' in text,
          '%d latched lines, %d raised lines'
          % (text.count('floor latched'), text.count('floor raised')))
    engaged = [s for t, s in h.states
               if not any(a <= t <= b for a, b in hold_windows)]
    while engaged and not engaged[0].startswith('FOLLOW|OK'):
        engaged.pop(0)
    others = sorted({s.split('|')[0] + '|' + s.split('|')[1] for s in engaged
                     if not s.startswith('FOLLOW|OK')})
    check('10 FOLLOW|OK throughout once engaged, outside the two deliberate HOLDs',
          engaged and not others, '%d states after engage, others: %s' % (len(engaged), others))

    n_ok = sum(1 for _, ok, _ in results if ok)
    print('\n%d/%d passed -- logs in %s' % (n_ok, len(results), logdir))
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
