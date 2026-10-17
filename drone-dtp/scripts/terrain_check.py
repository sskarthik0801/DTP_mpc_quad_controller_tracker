#!/usr/bin/env python3
"""terrain_check.py -- the terrain-relative height gate: the REAL command_mux_node and
terrain_estimator, sandboxed, closed loop over a synthetic slope.

Re-run after ANY change to command_mux or terrain_estimator (with descent_guard_check.py and
fence_check.py, which cover the switch-off path). ~70 s, needs no aircraft, no LiDAR, no FC:
every participant is on a private localhost-only DDS domain (91), and the mux's PX4 topics are
remapped under /terrain_check/, where nothing listens. The fake PX4 state is armed + Offboard, so
the offset is rate-limited and HOLDs are counted, as in the air. The abort stays ENABLED on
purpose: command_mux subscribes to PX4's mode only when it is, and with it off the node never
sees Offboard (the first run of this gate did exactly that and failed T1, T6, T7). No case raises
a sensor veto, so it never fires.

The harness plays FAST-LIO (odometry + a body-frame ground cloud of the scene) and SUPER (the
aircraft moves 1 m/s down x, and its z follows the mux's goal at up to 0.5 m/s). A walker stays
3.3 m ahead, inside the descent guard's radius the whole way, as in a real follow. The scene:
  x < 5 m        flat at -0.15 (the takeoff ground: LiDAR 0.15 m up)
  5 .. 19 m      downhill, 10 %, 1.4 m in all -- more than terrain_max_down_m 1.2, so it clamps
  x = 21.5 m     a ditch 0.5 m wide and 1 m deep across the track
  x = 24 m       a bush 1 x 1 m, 0.8 m tall, on the track

  T0  both nodes up, FOLLOW engaged, offset ~0 on the flat, goal at 1.2;
  T1  down the slope with the walker within 3 m: the goal FOLLOWS the ground down (the latched
      descent-guard floor is a height above the ground, so it does not hold the takeoff altitude)
      and never descends faster than terrain_rate_down_mps;
  T2  at the bottom the offset is clamped at -1.2 and says so;
  T3  the aircraft's height above the ground stays in [1.0, 2.0] m the whole way;
  T4  the narrow ditch does not pull the goal down; T5 the bush does not push it up;
  T6  a HOLD in terrain mode climbs 0.75 m from where the aircraft IS (not from the takeoff
      height), and FOLLOW comes back to the terrain height, the HOLD climb not raising the floor;
  T7  terrain_estimator killed: /mux/state says stale and the goal HOLDS its height -- no jump;
  T9  command_mux survived all of it (the first version of its stale-terrain log line raised
      inside the node and killed it at T7 -- rclpy refuses one call site at two severities).

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/terrain_check.py
"""
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

DOMAIN = '91'
NS = '/terrain_check'
GOAL_TOPIC = NS + '/follow_goal'
MODE_TOPIC = NS + '/vehicle_control_mode'
CLOUD_TOPIC = NS + '/cloud'
REF = -0.15
SPEED = 1.0          # aircraft m/s along x while flying
VZ_MAX = 0.5         # the fake SUPER's vertical speed
AHEAD = 1.0          # follow goal ahead of the aircraft
WALKER = 3.3         # walker ahead of the aircraft


def ground(x, y):
    g = np.where(x < 5.0, REF, np.where(x < 19.0, REF - 0.1 * (x - 5.0), REF - 1.4))
    g = np.where(np.abs(x - 21.5) < 0.25, g - 1.0, g)                          # ditch
    bush = (np.abs(x - 24.0) < 0.5) & (np.abs(y) < 0.5)
    return np.where(bush, g + 0.8, g)


def base_ground(x):
    """The ground without the ditch and the bush: what the aircraft should follow."""
    return REF if x < 5.0 else (REF - 0.1 * (x - 5.0) if x < 19.0 else REF - 1.4)


def _env():
    return dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')


def _start(argv, log):
    return subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            env=_env(), start_new_session=True)


def _stop(proc):
    if proc is None or proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


class Harness:

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy, qos_profile_sensor_data)
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Bool, Header, String
        from dtp_msgs.msg import Target, TargetArray
        from px4_msgs.msg import VehicleControlMode

        self.rclpy, self.pc2, self.Header = rclpy, point_cloud2, Header
        rclpy.init(args=None)
        self.node = Node('terrain_check')
        self.PoseStamped, self.Odometry, self.Bool = PoseStamped, Odometry, Bool
        self.Target, self.TargetArray, self.VCM = Target, TargetArray, VehicleControlMode
        self.x, self.z = 0.0, 1.35
        self.moving = False
        self.follow_on = True
        self.goal_z = None           # the mux's latest arbitrated goal z
        self.goals = []              # (t, x_aircraft, goal_z, z_aircraft)
        self.states = []
        self._last = time.time()
        self._tick = 0
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
        self.cloud_pub = self.node.create_publisher(PointCloud2, CLOUD_TOPIC,
                                                    qos_profile_sensor_data)
        self.targets_pub = self.node.create_publisher(TargetArray, '/dtp/targets',
                                                      qos_profile_sensor_data)
        self.sensors_pub = self.node.create_publisher(Bool, '/dtp/sensors_ok', 10)
        self.follow_pub = self.node.create_publisher(PoseStamped, '/dtp/follow_goal',
                                                     qos_profile_sensor_data)
        self.mode_pub = self.node.create_publisher(VehicleControlMode, MODE_TOPIC, px4)
        # A ring of ground around the aircraft, 1-12 m, 0.25 m apart (body frame = world - pos).
        g = np.arange(-12.0, 12.0, 0.25)
        gx, gy = np.meshgrid(g, g)
        r = np.hypot(gx, gy)
        self.ring = np.column_stack([gx[(r > 1.0) & (r < 12.0)], gy[(r > 1.0) & (r < 12.0)]])
        self.node.create_timer(0.05, self._sim)

    def _on_goal(self, m):
        self.goal_z = m.pose.position.z
        self.goals.append((time.time(), self.x, m.pose.position.z, self.z))

    def _on_state(self, m):
        self.states.append((time.time(), m.data))

    def _sim(self):
        now_t = time.time()
        dt, self._last = now_t - self._last, now_t
        self._tick += 1
        if self.moving:
            self.x += SPEED * dt
        if self.goal_z is not None:
            self.z += max(-VZ_MAX * dt, min(VZ_MAX * dt, self.goal_z - self.z))
        stamp = self.node.get_clock().now().to_msg()
        o = self.Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'world'
        o.pose.pose.position.x, o.pose.pose.position.z = self.x, self.z
        o.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(o)
        if self._tick % 2 == 0:
            wx = self.ring[:, 0] + self.x
            wy = self.ring[:, 1]
            pts = np.column_stack([self.ring, ground(wx, wy) - self.z]).astype(np.float32)
            h = self.Header()
            h.stamp = stamp
            h.frame_id = 'lidar_imu'
            self.cloud_pub.publish(self.pc2.create_cloud_xyz32(h, pts))
        if self._tick % 3 == 0:
            arr = self.TargetArray()
            arr.header.stamp = stamp
            arr.header.frame_id = 'world'
            t = self.Target()
            t.header = arr.header
            t.track_id = 1
            t.position.x, t.position.z = self.x + WALKER, float(base_ground(self.x + WALKER)) + 0.3
            t.track_confirmed = True
            t.is_locked_target = True
            arr.targets.append(t)
            self.targets_pub.publish(arr)
            self.sensors_pub.publish(self.Bool(data=True))
            if self.follow_on:
                g = self.PoseStamped()
                g.header.stamp = stamp
                g.header.frame_id = 'world'
                g.pose.position.x, g.pose.position.z = self.x + AHEAD, 1.2
                g.pose.orientation.w = 1.0
                self.follow_pub.publish(g)
            m = self.VCM()
            m.timestamp = int(now_t * 1e6)
            m.flag_armed = True
            m.flag_control_offboard_enabled = True
            self.mode_pub.publish(m)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def fly_to(self, x_end, timeout=60.0):
        self.moving = True
        end = time.time() + timeout
        while self.x < x_end and time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
        self.moving = False

    def state(self):
        return self.states[-1][1] if self.states else ''

    def goals_between(self, x0, x1):
        return [g for g in self.goals if x0 <= g[1] <= x1]

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    results = []

    def check(name, ok, detail=''):
        results.append((name, bool(ok), detail))
        print('  [%s] %s  %s' % ('PASS' if ok else 'FAIL', name, detail))
        sys.stdout.flush()

    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = tempfile.mkdtemp(prefix='terrain_check_')
    print('\nterrain_check -- real command_mux_node + terrain_estimator on ROS_DOMAIN_ID %s '
          '(localhost only); logs in %s\n' % (DOMAIN, logdir))
    h = Harness()
    mux_alive = False
    mux_log = open(os.path.join(logdir, 'mux.log'), 'w')
    est_log = open(os.path.join(logdir, 'terrain_estimator.log'), 'w')
    mux = _start(['ros2', 'run', 'command_mux', 'command_mux_node', '--ros-args',
                  '-p', 'terrain_follow_enable:=true',
                  '-p', 'arbitrated_goal_topic:=%s' % GOAL_TOPIC,
                  '-p', 'vehicle_command_topic:=%s/vehicle_command' % NS,
                  '-p', 'vehicle_command_ack_topic:=%s/vehicle_command_ack' % NS,
                  '-p', 'vehicle_control_mode_topic:=%s' % MODE_TOPIC], mux_log)
    est = _start(['ros2', 'run', 'command_mux', 'terrain_estimator', '--ros-args',
                  '-p', 'cloud_topic:=%s' % CLOUD_TOPIC], est_log)
    try:
        h.spin(6.0)
        s = h.state()
        check('T0 up: FOLLOW, offset ~0 on the flat, goal at 1.2',
              s.startswith('FOLLOW|OK') and 'terrain' in s and h.goal_z is not None
              and abs(h.goal_z - 1.35) < 0.08,
              'goal z %s | %s' % (h.goal_z, s.split('|', 2)[-1][-60:]))
        # The walker is 3.3 m ahead from the start, so the guard latched at the engage height
        # (1.35): the goal sits on that floor. That is the latched guard working, not terrain.

        h.fly_to(19.5)
        slope = h.goals_between(5.0, 19.5)
        zs = [g[2] for g in slope]
        # Over 1 s windows: consecutive goals are only as far apart as the mux's republish
        # timing, and arrival jitter on a 4 cm step reads as a spike.
        rates = []
        for i, a in enumerate(slope):
            later = [b for b in slope[i:] if b[0] - a[0] >= 1.0]
            if later:
                rates.append((later[0][2] - a[2]) / (later[0][0] - a[0]))
        floor_line = [st for _, st in h.states[-20:] if 'alt floor' in st]
        check('T1 downhill with the walker within 3 m: the goal follows the ground down, at '
              '<= 0.2 m/s (1 s windows)',
              len(zs) > 10 and zs[0] - zs[-1] > 1.0 and min(rates, default=0) > -0.22
              and floor_line,
              '%d goals, z %.2f -> %.2f, steepest %.2f m/s, floor shown: %s'
              % (len(zs), zs[0] if zs else -1, zs[-1] if zs else -1, min(rates, default=0),
                 bool(floor_line)))

        h.spin(8.0)            # let the offset settle at the bottom
        s = h.state()
        m = re.search(r'terrain ([+-][0-9.]+) (\w+)', s)
        off = float(m.group(1)) if m else None
        check('T2 at the bottom (1.4 m below takeoff) the offset clamps at -1.2 and says so '
              '(settled inside the 0.05 m deadband, never past the clamp)',
              m and m.group(2) == 'clamped' and -1.2 - 1e-6 <= off <= -1.15 + 1e-6,
              s.split('|', 2)[-1][-70:])

        agl = [g[3] - base_ground(g[1]) for g in h.goals if g[1] > 0.5]
        check('T3 aircraft height above the ground stays in [1.0, 2.0] m',
              agl and 1.0 <= min(agl) and max(agl) <= 2.0,
              'min %.2f max %.2f over %d goals'
              % (min(agl, default=-1), max(agl, default=-1), len(agl)))

        settled = h.goal_z
        h.fly_to(22.5)
        ditch = [g[2] for g in h.goals_between(20.5, 22.5)]
        check('T4 the narrow ditch does not pull the goal down',
              ditch and min(ditch) > settled - 0.06,
              'goal z %.2f..%.2f vs %.2f before'
              % (min(ditch, default=-1), max(ditch, default=-1), settled))
        h.fly_to(25.0)
        bush = [g[2] for g in h.goals_between(23.0, 25.0)]
        check('T5 the bush does not push the goal up',
              bush and max(bush) < settled + 0.06,
              'goal z %.2f..%.2f vs %.2f before'
              % (min(bush, default=-1), max(bush, default=-1), settled))

        z_before = h.z
        h.follow_on = False
        h.spin(1.5)
        hold_state, hold_z = h.state(), h.goal_z
        h.spin(2.0)
        h.follow_on = True
        h.spin(6.0)
        back_z, back_state = h.goal_z, h.state()
        check('T6 a HOLD climbs 0.75 m from where the aircraft is; FOLLOW returns to the '
              'terrain height (floor not raised)',
              hold_state.startswith('HOLD|NO_SOURCE') and abs(hold_z - (z_before + 0.75)) < 0.1
              and back_state.startswith('FOLLOW|OK') and abs(back_z - settled) < 0.06,
              'aircraft %.2f, HOLD goal %.2f, FOLLOW again %.2f (was %.2f)'
              % (z_before, hold_z, back_z, settled))

        _stop(est)
        est = None
        h.fly_to(26.5)
        h.spin(2.0)
        s = h.state()
        check('T7 terrain_estimator killed: state says stale, the goal holds its height',
              'stale' in s and abs(h.goal_z - settled) < 0.06,
              'goal z %.2f (was %.2f) | %s' % (h.goal_z, settled, s.split('|', 2)[-1][-50:]))
        mux_alive = mux.poll() is None
    finally:
        _stop(est)
        _stop(mux)
        mux_log.close()
        est_log.close()
        h.close()

    with open(os.path.join(logdir, 'mux.log')) as f:
        text = f.read()
    check('T8 the mux announced the mode, never raised the floor, and never vetoed',
          'TERRAIN-RELATIVE HEIGHT ON' in text and 'floor raised' not in text
          and 'PROXIMITY' not in text,
          '%d "floor raised" lines' % text.count('floor raised'))
    check('T9 command_mux still running at the end, no traceback in its log',
          mux_alive and 'Traceback' not in text,
          'alive %s, %d tracebacks' % (mux_alive, text.count('Traceback')))
    n_ok = sum(1 for _, ok, _ in results if ok)
    print('\n%d/%d passed -- logs in %s' % (n_ok, len(results), logdir))
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
