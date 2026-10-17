#!/usr/bin/env python3
"""brake_envelope_check.py -- the braking-envelope gate: the REAL command_mux_node, sandboxed.

Re-run after ANY change to command_mux's braking envelope or speed-scaled radii
(command_mux/brake_envelope.py, brake_envelope_enable). ~1.5 min, needs no aircraft, no LiDAR, no
FC, and cannot touch any of them: every participant is on a private localhost-only DDS domain (94),
and the node's PX4 topics are remapped under /brake_envelope_check/. The fake PX4 state says armed
but NOT Offboard, so no veto can escalate to a Land, and the engage gate is OFF (goals are needed
outside Offboard, as in descent_guard_check.py). The geofence is a 510 x 100 m box so the fake
aircraft, which flies ~40 m, never meets it.

The harness flies a fake aircraft: /lidar_slam/odom at 10 Hz, stamped, moving along +x at the
speed each case sets (the node differences the pose, as it must on this stack). People are placed
relative to it, with a velocity and a 1.5 s forecast like perception's.

  B0 ON:  node up, FOLLOW engaged, /mux/state carries the "brake" field at rest (v 0.00).
  B1 ON:  at rest, a person 1.6 m to the SIDE of the aircraft: no veto. The envelope of a
          hovering aircraft is its own position, so nothing new can veto it.
  B2 ON:  2.5 m/s at a STANDING person, the follow goal parked 3.0 m short of them. The path to
          the goal never comes within 1.5 m of them, so today's check never fires; the envelope
          fires where the stop (2.81 m) + overshoot (1.00 m) + radius (1.5 m) reach them: 5.31 m.
  B3 ON:  2.5 m/s behind a person WALKING AWAY at 2.5 m/s, 4.0 m ahead: no veto, closing ~0,
          extra 0.
  B4 ON:  2.0 m/s at a standing person with the follow goal 1 m ahead of the aircraft and BELOW
          it: the descent guard's trip radius has grown by the capped extra (1.5 m), so the floor
          latches with the goal ~4.5 m from the person (aircraft ~5.5 m), not 3.0.
  B5 ON:  the aircraft stops: the extra is held ~1 s, then shrinks no faster than 0.5 m/s.
  B6 OFF: the same B2 and B4 runs with brake_envelope_enable false (the shipped default): no
          envelope veto all the way to 4.0 m, the descent guard latches at the old 3.0 m, and
          /mux/state has no "brake" field. This is today's behaviour, untouched.
  B7 refuse: speed_brake_mps2 2.5 with super_max_acc_mps2 2.0 refuses to start.

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/brake_envelope_check.py
"""
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '94'
NS = '/brake_envelope_check'
GOAL_TOPIC = NS + '/follow_goal'
MODE_TOPIC = NS + '/vehicle_control_mode'
DRONE_Z = 1.5
ODOM_HZ = 10.0
FEED_HZ = 50.0

# the formula at the node's defaults (brake_envelope.SpeedParams): react 0.5, brake 2.0,
# track 0.4 s/m (capped 2.0), radius 1.5
B2_V = 2.5
B2_EXPECT = 1.5 + (B2_V * 0.5 + B2_V ** 2 / 4.0) + 0.4 * B2_V           # 5.3125 m


def _env():
    return dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')


def _start_mux(log, enable, extra=()):
    ros_args = ['--ros-args',
                '-p', 'arbitrated_goal_topic:=%s' % GOAL_TOPIC,
                '-p', 'vehicle_command_topic:=%s/vehicle_command' % NS,
                '-p', 'vehicle_command_ack_topic:=%s/vehicle_command_ack' % NS,
                '-p', 'vehicle_control_mode_topic:=%s' % MODE_TOPIC,
                '-p', 'engage_gate_enable:=false',
                '-p', 'fence_x_min_m:=-10.0', '-p', 'fence_x_max_m:=500.0',
                '-p', 'fence_y_min_m:=-50.0', '-p', 'fence_y_max_m:=50.0',
                '-p', 'brake_envelope_enable:=%s' % ('true' if enable else 'false'),
                '-p', 'super_max_acc_mps2:=2.0']
    for kv in extra:
        ros_args += ['-p', kv]
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
    """A fake aircraft flying along +x, people placed relative to it, a follow goal, PX4 mode."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy, qos_profile_sensor_data)
        from geometry_msgs.msg import Point, PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, String
        from dtp_msgs.msg import Target, TargetArray
        from px4_msgs.msg import VehicleControlMode

        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = Node('brake_envelope_check')
        self.Point, self.PoseStamped, self.Odometry, self.Bool = Point, PoseStamped, Odometry, Bool
        self.Target, self.TargetArray, self.VehicleControlMode = (
            Target, TargetArray, VehicleControlMode)
        self.x = 0.0                    # the aircraft along +x, y = 0, z = DRONE_Z
        self.v = 0.0
        self.person = None              # [x, y, vx] in the world, or None
        self.goal = (1.0, 0.0, DRONE_Z)
        self.goal_rel = True            # goal given relative to the aircraft
        self.states = []                # (t, aircraft x, person x or None, text)
        self.quiet = False              # stop every fake publisher (before a node is stopped)
        self._last = time.time()
        self._last_odom = 0.0

        best = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                          durability=QoSDurabilityPolicy.VOLATILE)
        px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.node.create_subscription(PoseStamped, GOAL_TOPIC, lambda m: None, best)
        self.node.create_subscription(String, '/mux/state', self._on_state, 50)
        self.odom_pub = self.node.create_publisher(Odometry, '/lidar_slam/odom',
                                                   qos_profile_sensor_data)
        self.targets_pub = self.node.create_publisher(TargetArray, '/dtp/targets',
                                                      qos_profile_sensor_data)
        self.sensors_pub = self.node.create_publisher(Bool, '/dtp/sensors_ok', 10)
        self.follow_pub = self.node.create_publisher(PoseStamped, '/dtp/follow_goal',
                                                     qos_profile_sensor_data)
        self.mode_pub = self.node.create_publisher(VehicleControlMode, MODE_TOPIC, px4)
        self.node.create_timer(1.0 / FEED_HZ, self._feed)

    def _on_state(self, m):
        px = self.person[0] if self.person is not None else None
        self.states.append((time.time(), self.x, px, m.data))

    def _feed(self):
        t = time.time()
        dt = t - self._last
        self._last = t
        self.x += self.v * dt
        if self.person is not None:
            self.person[0] += self.person[2] * dt
        if self.quiet:
            return
        now = self.node.get_clock().now().to_msg()
        if t - self._last_odom >= 1.0 / ODOM_HZ:
            self._last_odom = t
            o = self.Odometry()
            o.header.stamp = now
            o.header.frame_id = 'world'
            o.pose.pose.position.x = self.x
            o.pose.pose.position.z = DRONE_Z
            o.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(o)
        arr = self.TargetArray()
        arr.header.stamp = now
        arr.header.frame_id = 'world'
        if self.person is not None:
            px, py, vx = self.person
            tg = self.Target()
            tg.header = arr.header
            tg.track_id = 1
            tg.position.x, tg.position.y, tg.position.z = px, py, 0.2
            tg.velocity.x = vx
            tg.track_confirmed = True
            tg.is_locked_target = True
            for k in range(1, 21):
                d = 0.075 * k
                tg.predicted_path.append(self.Point(x=px + vx * d, y=py, z=0.2))
                tg.predicted_path_dt.append(d)
            arr.targets.append(tg)
        self.targets_pub.publish(arr)
        self.sensors_pub.publish(self.Bool(data=True))
        g = self.PoseStamped()
        g.header.stamp = now
        g.header.frame_id = 'world'
        gx, gy, gz = self.goal
        if self.goal_rel:
            gx += self.x
        g.pose.position.x, g.pose.position.y, g.pose.position.z = gx, gy, gz
        g.pose.orientation.w = 1.0
        self.follow_pub.publish(g)
        m = self.VehicleControlMode()
        m.timestamp = int(t * 1e6)
        m.flag_armed = True
        m.flag_control_offboard_enabled = False
        self.mode_pub.publish(m)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
            if cond():
                return True
        return False

    def state(self):
        return self.states[-1][3] if self.states else ''

    def first(self, t0, pred):
        for t, x, px, s in self.states:
            if t >= t0 and pred(s):
                return (t, x, px, s)
        return None

    def since(self, t0):
        return [s for s in self.states if s[0] >= t0]

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def _field(text, name):
    """The float after `name` in the brake field of a /mux/state line, or None."""
    if ' | brake ' not in text:
        return None
    part = text.split(' | brake ')[1].split(' | ')[0].split()
    for a, b in zip(part, part[1:]):
        if a == name:
            return float(b)
    return None


def _run(h, results, enable, logdir):
    def check(name, ok, detail=''):
        results.append((name, bool(ok), detail))
        print('  [%s] %s  %s' % ('PASS' if ok else 'FAIL', name, detail))
        sys.stdout.flush()

    tag = 'ON' if enable else 'OFF'
    log = open(os.path.join(logdir, 'mux_%s.log' % tag.lower()), 'w')
    proc = _start_mux(log, enable)
    try:
        h.v = 0.0
        h.person = None
        h.goal, h.goal_rel = (1.0, 0.0, DRONE_Z), True
        up = h.spin_until(lambda: h.state().startswith('FOLLOW|OK'), 15.0)
        h.spin(1.0)
        if enable:
            check('B0 ON: node up, FOLLOW engaged, brake field at rest',
                  up and h.state().startswith('FOLLOW|OK') and _field(h.state(), 'v') == 0.0,
                  h.state()[-90:])

            # B1 at rest, a person 1.6 m to the side
            h.person = [h.x, 1.6, 0.0]
            t0 = time.time()
            h.spin(2.0)
            vetoed = [s for _, _, _, s in h.since(t0) if s.startswith('HOLD')]
            check('B1 ON: hovering, person 1.6 m to the side: no veto', not vetoed,
                  '%d HOLD states' % len(vetoed))
        else:
            check('B0 OFF: node up, FOLLOW engaged, NO brake field',
                  up and h.state().startswith('FOLLOW|OK') and ' | brake ' not in h.state(),
                  h.state()[-60:])

        # B2 sprint at a standing person, goal parked 3.0 m short of them
        h.person = None
        h.v = B2_V
        h.spin(1.0)                             # speed established well before anyone is near
        h.person = [h.x + 12.0, 0.0, 0.0]
        h.goal, h.goal_rel = (h.person[0] - 3.0, 0.0, DRONE_Z), False
        t0 = time.time()
        h.spin_until(lambda: h.person[0] - h.x <= 4.0, 6.0)
        h.v = 0.0
        hit = h.first(t0, lambda s: s.startswith('HOLD|PROXIMITY'))
        if enable:
            rng = (hit[2] - hit[1]) if hit else None
            ok = hit is not None and 'braking path' in hit[3] and abs(rng - B2_EXPECT) <= 0.45
            check('B2 ON: 2.5 m/s at a standing person: envelope veto at ~%.2f m' % B2_EXPECT, ok,
                  'first PROXIMITY at range %s: %s'
                  % ('%.2f' % rng if rng is not None else 'never',
                     hit[3].split('|')[2][:110] if hit else ''))
        else:
            check('B6a OFF: 2.5 m/s to 4.0 m from a standing person: no veto (today)',
                  hit is None, 'first PROXIMITY: %s' % (hit[3][:100] if hit else 'none'))
        h.person = None
        h.goal, h.goal_rel = (1.0, 0.0, DRONE_Z), True
        # a HOLD releases (dwell), FOLLOW re-engages, and the extra's hold + shrink (1 + 3 s) ends
        h.spin(4.5)

        if enable:
            # B3 behind a person walking away at our speed
            h.v = B2_V
            h.spin(1.0)
            h.person = [h.x + 4.0, 0.0, B2_V]
            t0 = time.time()
            h.spin(4.0)
            win = h.since(t0 + 0.5)
            holds = [s for _, _, _, s in win if s.startswith('HOLD')]
            closing = [_field(s, 'closing') for _, _, _, s in win
                       if _field(s, 'closing') is not None]
            extras = [_field(s, 'extra') for _, _, _, s in win if _field(s, 'extra') is not None]
            check('B3 ON: 2.5 m/s behind a walker going away at 2.5 m/s: no veto, extra 0',
                  not holds and closing and max(closing) <= 0.35 and max(extras) == 0.0,
                  '%d HOLD states, closing max %.2f, extra max %.2f'
                  % (len(holds), max(closing or [-1]), max(extras or [-1])))
            h.person = None
            h.v = 0.0
            h.spin(4.0)                         # the extra (none) and any latch settle

        # B4 2.0 m/s at a standing person, goal 1 m ahead and 1 m BELOW the aircraft
        h.goal, h.goal_rel = (1.0, 0.0, DRONE_Z - 1.0), True
        h.v = 2.0
        h.spin(1.0)
        h.person = [h.x + 9.0, 0.0, 0.0]
        t0 = time.time()
        h.spin_until(lambda: h.person[0] - h.x <= (5.0 if enable else 3.6), 5.0)
        h.v = 0.0
        hit = h.first(t0, lambda s: 'alt floor' in s)
        rng = (hit[2] - hit[1]) if hit else None
        if enable:
            ok = hit is not None and 5.1 <= rng <= 5.8
            check('B4 ON: 2.0 m/s at a standing person: descent guard latches with the goal '
                  '~4.5 m away (radius 3.0 + 1.5)', ok,
                  'latched at aircraft range %s (goal range %s)'
                  % ('%.2f' % rng if rng else 'never', '%.2f' % (rng - 1.0) if rng else '-'))
            # B5 stopped: held ~1 s, then shrinking <= 0.5 m/s
            t_stop = time.time()
            h.spin(5.0)
            ser = [(t - t_stop, _field(s, 'extra')) for t, _, _, s in h.since(t_stop)
                   if _field(s, 'extra') is not None]
            peak = max(e for _, e in ser)
            t_drop = next((t for t, e in ser if e < peak - 1e-3), None)
            slopes = []
            for i, (ta, ea) in enumerate(ser):          # over >= 0.25 s windows: lines are ~20 ms
                tb, eb = next(((tb, eb) for tb, eb in ser[i:] if tb - ta >= 0.25), (None, None))
                if tb is not None and ea > eb:
                    slopes.append((ea - eb) / (tb - ta))
            ok = (peak >= 1.49 and t_drop is not None and t_drop >= 0.8
                  and (not slopes or max(slopes) <= 0.6) and ser[-1][1] < peak)
            check('B5 ON: stopped: extra held ~1 s, then shrinks <= 0.5 m/s', ok,
                  'peak %.2f, first drop at +%s s, steepest %.2f m/s, at +5 s %.2f'
                  % (peak, '%.2f' % t_drop if t_drop is not None else 'never',
                     max(slopes or [0.0]), ser[-1][1]))
        else:
            ok = hit is not None and 3.8 <= rng <= 4.15
            check('B6b OFF: descent guard latches at the old 3.0 m (goal) radius', ok,
                  'latched at aircraft range %s (goal range %s)'
                  % ('%.2f' % rng if rng else 'never', '%.2f' % (rng - 1.0) if rng else '-'))
            nobrake = all(' | brake ' not in s for _, _, _, s in h.states)
            check('B6c OFF: no brake field in any /mux/state line', nobrake, '')
        h.person = None
    finally:
        # Quiet first: stopping the node while messages are still arriving is the rclpy teardown
        # race engage_gate_check.py documents ("Unable to convert call argument" in
        # _take_subscription), which would read as a traceback in a healthy node.
        h.quiet = True
        h.spin(0.5)
        _stop(proc)
        h.quiet = False
        log.close()
        with open(os.path.join(logdir, 'states_%s.txt' % tag.lower()), 'w') as f:
            for t, x, px, s in h.states:
                f.write('%.3f x %.2f person %s | %s\n'
                        % (t, x, '%.2f' % px if px is not None else '-', s))
    with open(os.path.join(logdir, 'mux_%s.log' % tag.lower())) as f:
        text = f.read()
    check('B%s %s: no traceback in the node log' % ('8' if enable else '6d', tag),
          'Traceback' not in text, '')
    h.states.clear()


def main():
    results = []
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = tempfile.mkdtemp(prefix='brake_envelope_check_')
    print('\nbrake_envelope_check -- installed command_mux_node on ROS_DOMAIN_ID %s (localhost '
          'only); logs in %s\n' % (DOMAIN, logdir))
    h = Harness()
    try:
        _run(h, results, True, logdir)
        _run(h, results, False, logdir)
    finally:
        h.close()

    # B7 a braking deceleration SUPER never plans is refused at startup
    log = open(os.path.join(logdir, 'mux_refuse.log'), 'w')
    proc = _start_mux(log, True, extra=('speed_brake_mps2:=2.5',))
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        pass
    alive = proc.poll() is None
    _stop(proc)
    log.close()
    with open(os.path.join(logdir, 'mux_refuse.log')) as f:
        text = f.read()
    ok = (not alive) and 'speed_brake_mps2 2.50 > SUPER max_acc 2.00' in text
    results.append(('B7 refuse: brake 2.5 > SUPER max_acc 2.0', ok, ''))
    print('  [%s] B7 refuse: brake 2.5 > SUPER max_acc 2.0  %s'
          % ('PASS' if ok else 'FAIL', 'exited' if not alive else 'STILL RUNNING'))

    n_ok = sum(1 for _, ok, _ in results if ok)
    print('\n%d/%d passed -- logs in %s' % (n_ok, len(results), logdir))
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
