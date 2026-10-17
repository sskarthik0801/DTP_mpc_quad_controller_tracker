#!/usr/bin/env python3
"""M3 bench gate: does px4_offboard_bridge convert frames and fail safe correctly?

Runs the REAL bridge node in a sandbox -- every topic remapped under /m3_check/ --
feeds it synthetic PositionCommands and a synthetic PX4 state, and asserts what comes
out on the trajectory-setpoint topic. Touches no flight controller and cannot arm
anything, so it is safe to run any time, with or without the stack up.

    source /opt/ros/humble/setup.bash
    source ~/super_ws/install/setup.bash
    python3 scripts/px4/m3_bridge_check.py

What it is actually protecting against
--------------------------------------
* **The yaw trap.** The old SITL-era spec said `pi/2 - psi`; the correct conversion on
  this aircraft is `-psi` (docs/ImplementationDoc.md SS M3). A 90 deg constant yaw error
  builds and flies fine and only shows up as the drone accelerating off-axis. T2 asserts
  the sign explicitly *and* asserts the answer is not the pi/2 form.
* **A silently unmatched subscription.** SUPER publishes pos_cmd BEST_EFFORT/VOLATILE;
  rclpy subscriptions default to RELIABLE and would never connect. If T2 sees no
  commands at all, that is the bug.
* **Stale-setpoint hazards.** T4/T5 cover the two ways a held setpoint turns dangerous:
  SUPER going quiet mid-flight (must freeze in place, not drift) and the vehicle being
  disarmed and carried elsewhere (must NOT still command the old point).
* **Setpoints outside Offboard (since 2026-09-28).** The bridge's always-on IDLE stream
  was mixed by PX4 into Position and Altitude mode (retune Flight 2, 2026-09-21). T1
  asserts nothing reaches trajectory_setpoint while PX4 says Offboard is off -- disarmed,
  or armed in a manual mode -- while the heartbeat stays at rate; that the first sample
  after the Offboard report is the current estimate, for both engage orders; T4 that a
  mode report that goes SILENT in Offboard does not stop the stream (that would starve
  PX4 in the air); T5 that a fresh Offboard-off report (the pilot's takeover) does, and
  stays stopped while SUPER keeps commanding. A third sandboxed bridge with
  setpoint_only_in_offboard:=false must still show the old always-on stream.
* **Acceleration feedforward where it does not belong.** On by default since 2026-09-27.
  T2 asserts the bridge's DEFAULT forwards it as (x,-y,-z) -- no launch path passes the
  parameter -- and that a second bridge with publish_acceleration_ff:=false still sends
  NaN. T4, T5 and T7 assert it is NaN in HOLD, in IDLE (with and without a valid
  estimate) and after a stream of dropped commands: a held position must never be pushed
  by a stale acceleration.
* **The camera yaw servo (2026-09-29, default OFF).** Ya-Yk are plan-camera-yaw-servo.md
  section 4.2 a-k. The default bridge keeps both switches off and must ignore a live yaw
  target bit for bit (Ya: no subscription, no publisher, SUPER's yaw exactly). Three more
  sandboxed bridges run with the switch on (_servo: fix 1; _ff: fix 1 + 2; _nohold:
  aim_in_hold:=false) and share the same inputs plus a sandboxed yaw target. They assert
  the sign (a target to the LEFT is a LEFT turn, FRD yaw < 0, and not the pi/2 form), the
  90 deg/s slew on every pair of setpoints using their own timestamps, the fall-backs to
  SUPER's yaw (stale, future-stamped, wrong frame, NaN, lpos invalid) arriving at SUPER's
  yaw exactly, the close-range hold, the feed-forward's value, sign, cap and slew, that
  IDLE / outside Offboard / after a takeover nothing changes, that HOLD aims and falls
  back to the FROZEN yaw, and that position/velocity/acceleration/jerk are the same bytes
  with the servo on as off.
* **The yaw servo, round two (2026-09-29).** With the new defaults every fall-back is
  preceded by a 0.5 s stale hold, so Ye / Yj / Yi assert that (hold, then the fall-back).
  Then Y2: the default bridge and _nohold stop (never more than four bridges) and two
  ROUND-ONE bridges start (_servo_r1 / _ff_r1: yaw_servo_stale_hold_s:=0.0,
  yaw_servo_min_range_hysteresis_m:=0.0), where the round-one assertions of those checks
  (an immediate fall-back) still run. New: Yl, SUPER's yaw ramping at 100 deg/s with
  yaw_dot 100 deg/s while the target is unusable -- yawspeed exactly 0 on every tick the
  slew binds, never above 90 deg/s, and SUPER's rate clamped to 90 when not binding; Ym, a
  target at 2.4 -> 2.6 -> 2.9 m -- the default stays in the close-range hold at 2.6 and
  leaves at 2.9, the round-one bridge leaves at 2.6; Yn, the stale hold -- the heading held
  at the last aim with yawspeed 0 from age 0.5 s for 0.5 s of tick time, then the fall-back
  through the slew, and a target back during the hold aimed at again with no yawspeed step
  over the feed-forward slew.
* **A diverged FAST-LIO reaching EKF2 (G0.2).** T8-T14 sandbox px4_odom_bridge the same
  way. On 2026-09-12 it forwarded a pose 3.74 km wrong for ~9 min and the aircraft was
  armed on it (docs/fact-fastlio-divergence-2026-09-12.md). The cases replay the shape of
  that failure: a stall, a jump, and above all an estimate that SETTLES afterwards -- the
  part that makes it look healthy -- and assert nothing reaches PX4 until FAST-LIO is
  restarted, with reset_counter incremented when it is.

Like m2_gate_check.py, this is a ROS 2 script rather than a MAVLink one -- source the
workspace, no pymavlink needed.
"""
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from collections import namedtuple

import rclpy
from geometry_msgs.msg import PointStamped
from mars_quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleControlMode, VehicleLocalPosition,
                          VehicleOdometry)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float32MultiArray, String

NS = '/m3_check'
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BRIDGE = os.environ.get(
    'BRIDGE', os.path.join(REPO, 'super_bringup', 'scripts', 'px4_offboard_bridge.py'))
ODOM_BRIDGE = os.environ.get(
    'ODOM_BRIDGE', os.path.join(REPO, 'super_bringup', 'scripts', 'px4_odom_bridge.py'))
TOL = 1e-4

# The bridge's own defaults for these are 50 Hz / 0.3 s; keep the timeout short so the
# HOLD test does not have to wait long, and assert the rate we ask for.
RATE = 50.0
TIMEOUT = 0.3

# The yaw servo checks (Y). Three more sandboxed bridges with the switch ON; the default
# bridge keeps every switch at its default (off). Each has its own output topics and
# debug topic; the yaw target is shared, like every other input.
YT_TOPIC = f'{NS}/yaw_target'
SERVO_BRIDGES = {
    '_servo': ['yaw_servo_enable:=true'],                                   # fix 1
    '_ff': ['yaw_servo_enable:=true', 'yaw_servo_ff_enable:=true'],        # fix 1 + 2
    '_nohold': ['yaw_servo_enable:=true', 'yaw_servo_aim_in_hold:=false'],
}
# Round two (Y2): after the Y checks, the default bridge and _nohold stop (at most four
# bridges at once) and two ROUND-ONE bridges join _servo and _ff: no stale hold, no close-range
# hysteresis. The round-one assertions of the checks whose expectation the new defaults change
# (an immediate fall-back) run on these, and the new checks compare the two side by side.
SERVO_R1_BRIDGES = {
    '_servo_r1': ['yaw_servo_enable:=true', 'yaw_servo_stale_hold_s:=0.0',
                  'yaw_servo_min_range_hysteresis_m:=0.0'],
    '_ff_r1': ['yaw_servo_enable:=true', 'yaw_servo_ff_enable:=true',
               'yaw_servo_stale_hold_s:=0.0', 'yaw_servo_min_range_hysteresis_m:=0.0'],
}
SERVO_MAX_RATE = math.radians(90.0)    # the bridge defaults the Y checks assert
SERVO_FF_MAX = math.radians(60.0)
SERVO_FF_ACC = math.radians(90.0)
# Per-pair tolerance on every slew check, ON TOP of limit * (that pair's own dt from the
# setpoints' timestamps). The timestamp is the tick's clock truncated to 1 us, and yaw /
# yawspeed go out as float32 (<= 2.4e-7 rad of rounding at pi): 0.001 deg (1.7e-5 rad)
# covers both ~50x. It is NOT a jitter allowance -- jitter is carried exactly by using
# each pair's actual dt, which is also the dt the servo integrated.
SLEW_TOL = math.radians(0.001)
# One received setpoint: harness receive time, the setpoint's own timestamp (us, bridge
# clock), fields as floats, and the raw float32 bytes of position+velocity+acceleration+jerk.
Sp = namedtuple('Sp', 't ts pos vel acc jerk yaw yawspeed raw')


class Harness(Node):

    def __init__(self):
        super().__init__('m3_bridge_check')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        # Exactly SUPER's publisher QoS (FsmRos2::init): best_effort/keep_last(1)/volatile.
        super_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)

        self.cmd_pub = self.create_publisher(PositionCommand, f'{NS}/pos_cmd', super_qos)
        self.lpos_pub = self.create_publisher(
            VehicleLocalPosition, f'{NS}/vehicle_local_position', px4_qos)
        self.vcm_pub = self.create_publisher(
            VehicleControlMode, f'{NS}/vehicle_control_mode', px4_qos)

        self.sp = None
        self.n_sp = 0
        self.n_ocm = 0
        self.ocm = None
        self.sp_noacc = None   # the second bridge, publish_acceleration_ff:=false
        self.sp_legacy = None  # the third, setpoint_only_in_offboard:=false
        self.n_sp_legacy = 0
        # The first setpoint after mark(), and how long after mark() it arrived.
        self.t_mark = None
        self.first_sp = None
        self.first_sp_dt = None
        self.create_subscription(TrajectorySetpoint, f'{NS}/trajectory_setpoint',
                                 self.on_sp, px4_qos)
        self.create_subscription(OffboardControlMode, f'{NS}/offboard_control_mode',
                                 self.on_ocm, px4_qos)
        self.create_subscription(TrajectorySetpoint, f'{NS}/trajectory_setpoint_noacc',
                                 self.on_sp_noacc, px4_qos)
        self.create_subscription(TrajectorySetpoint, f'{NS}/trajectory_setpoint_legacy',
                                 self.on_sp_legacy, px4_qos)

        self.armed = False
        self.offboard = False
        self.lpos = (0.0, 0.0, 0.0, 0.0)
        self.lpos_valid = True
        self.vcm_silent = False   # True: stop reporting vehicle_control_mode (a stale link)

        # The yaw servo's sandboxed input, with follow_planner's exact publisher QoS
        # (follow_planner_node.hpp: best_effort / keep_last(1) / volatile).
        self.yt_pub = self.create_publisher(PointStamped, YT_TOPIC, super_qos)
        self.px4_qos = px4_qos
        self.ysp = {}             # suffix -> [Sp], from watch_servo() on
        self.ydbg = {}            # suffix -> [(receive time, [7 floats])]
        self.t_last_target = None

    def on_sp(self, msg):
        self.sp = msg
        self.n_sp += 1
        if self.t_mark is not None and self.first_sp is None:
            self.first_sp = msg
            self.first_sp_dt = time.time() - self.t_mark

    def on_sp_noacc(self, msg):
        self.sp_noacc = msg

    def on_sp_legacy(self, msg):
        self.sp_legacy = msg
        self.n_sp_legacy += 1

    def mark(self):
        """Forget the first setpoint; call right before the set_state() under test."""
        self.t_mark = time.time()
        self.first_sp = None
        self.first_sp_dt = None

    def on_ocm(self, msg):
        self.ocm = msg
        self.n_ocm += 1

    # ---- stimulus ---------------------------------------------------------

    def set_state(self, armed, offboard):
        self.armed, self.offboard = armed, offboard

    def set_lpos(self, x, y, z, heading):
        self.lpos = (x, y, z, heading)

    def set_lpos_valid(self, valid):
        self.lpos_valid = valid

    def tick_state(self):
        """Publish the synthetic PX4 state once (the real topics stream continuously)."""
        m = VehicleControlMode()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.flag_armed = self.armed
        m.flag_control_offboard_enabled = self.offboard
        if not self.vcm_silent:
            self.vcm_pub.publish(m)

        p = VehicleLocalPosition()
        p.timestamp = m.timestamp
        p.xy_valid = self.lpos_valid
        p.z_valid = self.lpos_valid
        p.x, p.y, p.z, p.heading = [float(v) for v in self.lpos]
        self.lpos_pub.publish(p)

    def send_cmd(self, pos, vel=(0.0, 0.0, 0.0), yaw=0.0, yaw_dot=0.0, flag=1,
                 acc=(0.0, 0.0, 0.0), jerk=(0.0, 0.0, 0.0)):
        m = PositionCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.position.x, m.position.y, m.position.z = [float(v) for v in pos]
        m.velocity.x, m.velocity.y, m.velocity.z = [float(v) for v in vel]
        m.acceleration.x, m.acceleration.y, m.acceleration.z = [float(v) for v in acc]
        m.jerk.x, m.jerk.y, m.jerk.z = [float(v) for v in jerk]
        m.yaw = float(yaw)
        m.yaw_dot = float(yaw_dot)
        m.trajectory_flag = flag
        self.cmd_pub.publish(m)

    def run(self, secs, cmd=None):
        """Spin for `secs`, streaming the PX4 state (and `cmd()` if given) at 100 Hz."""
        end = time.time() + secs
        while time.time() < end:
            self.tick_state()
            if cmd is not None:
                cmd()
            rclpy.spin_once(self, timeout_sec=0.01)

    # ---- the yaw servo checks (Y) ------------------------------------------

    def watch_servo(self, suffixes):
        """Record every setpoint of these bridges ('' = the default one), and the debug
        topic of the servo ones, from now on."""
        dbg_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        for s in suffixes:
            self.ysp[s] = []
            self.create_subscription(TrajectorySetpoint, f'{NS}/trajectory_setpoint{s}',
                                     lambda m, k=s: self._on_ysp(k, m), self.px4_qos)
            if s:
                self.ydbg[s] = []
                self.create_subscription(
                    Float32MultiArray, f'{NS}/yaw_servo{s}',
                    lambda m, k=s: self.ydbg[k].append((time.time(), list(m.data))), dbg_qos)

    def _on_ysp(self, key, m):
        self.ysp[key].append(Sp(
            time.time(), m.timestamp, tuple(float(v) for v in m.position),
            tuple(float(v) for v in m.velocity), tuple(float(v) for v in m.acceleration),
            tuple(float(v) for v in m.jerk), m.yaw, m.yawspeed,
            m.position.tobytes() + m.velocity.tobytes() + m.acceleration.tobytes()
            + m.jerk.tobytes()))

    def seg(self, key, t_from, t_to=None):
        """Setpoints of one bridge whose OWN timestamp is in [t_from, t_to)."""
        return [x for x in self.ysp[key]
                if x.ts / 1e6 >= t_from and (t_to is None or x.ts / 1e6 < t_to)]

    def dseg(self, key, t_from, t_to=None):
        """Debug messages of one bridge received in [t_from, t_to)."""
        return [d for t, d in self.ydbg[key]
                if t >= t_from and (t_to is None or t < t_to)]

    def dsegt(self, key, t_from, t_to=None):
        """As dseg, with each message's receive time: [(t, d)]."""
        return [(t, d) for t, d in self.ydbg[key]
                if t >= t_from and (t_to is None or t < t_to)]

    def pub_target(self, x, y, z=1.2, age=0.0, frame='lidar_init'):
        """One yaw target, FLU lidar_init, stamped `age` s in the past (negative: future)."""
        m = PointStamped()
        ns = self.get_clock().now().nanoseconds - int(round(age * 1e9))
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        m.header.frame_id = frame
        m.point.x, m.point.y, m.point.z = float(x), float(y), float(z)
        self.yt_pub.publish(m)
        self.t_last_target = time.time()

    def yrun(self, secs, cmd=None, target=None, target_hz=15.0):
        """Spin for `secs`: the PX4 state and `cmd()` at 100 Hz, `target()` at `target_hz`
        (the yaw target's rate in flight). Unlike run(), rate-limited: with four bridges
        a free-running loop would flood them."""
        t = time.time()
        end, next_s, next_t = t + secs, t, t
        while True:
            now = time.time()
            if now >= end:
                return
            if now >= next_s:
                self.tick_state()
                if cmd is not None:
                    cmd()
                next_s = max(next_s + 0.01, now)
            if target is not None and now >= next_t:
                target()
                next_t = max(next_t + 1.0 / target_hz, now)
            wake = min(next_s, next_t if target is not None else end, end)
            rclpy.spin_once(self, timeout_sec=max(0.0, wake - time.time()))


class OdomHarness(Node):
    """Feeds px4_odom_bridge synthetic FAST-LIO odometry; watches what it sends PX4."""

    def __init__(self):
        super().__init__('m3_bridge_check_odom')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.odom_pub = None
        self.far_pub = self.create_publisher(Odometry, f'{NS}/odom_far', 10)
        self.vo = {}      # output topic -> [count, last msg]
        self.status = {}  # status topic -> last text
        for suffix in ('', '_far'):
            out, st = f'{NS}/visual_odometry{suffix}', f'{NS}/odom_bridge_status{suffix}'
            self.vo[out] = [0, None]
            self.create_subscription(VehicleOdometry, out,
                                     lambda m, k=out: self._on_vo(k, m), px4_qos)
            self.create_subscription(String, st,
                                     lambda m, k=st: self.status.__setitem__(k, m.data), latched)
        self.new_publisher()

    def _on_vo(self, key, msg):
        self.vo[key][0] += 1
        self.vo[key][1] = msg

    def new_publisher(self):
        """A fresh publisher has a fresh GID -- what a restarted FAST-LIO looks like."""
        if self.odom_pub is not None:
            self.destroy_publisher(self.odom_pub)
        self.odom_pub = self.create_publisher(Odometry, f'{NS}/odom', 10)

    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def stream(self, secs, pos, pub=None, rate=20.0):
        """Publish odometry at `rate` for `secs`; `pos(t)` gives FLU position at t seconds."""
        pub = pub or self.odom_pub
        t0 = time.time()
        next_t = t0
        while time.time() - t0 < secs:
            if time.time() >= next_t:
                m = Odometry()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = 'lidar_init'
                m.child_frame_id = 'lidar_imu'
                x, y, z = pos(time.time() - t0)
                m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = x, y, z
                m.pose.pose.orientation.w = 1.0
                pub.publish(m)
                next_t += 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.005)


def run_odom_checks(check):
    """T8-T14: px4_odom_bridge's divergence guard. Two sandboxed bridge processes."""
    procs = []
    for suffix, src in (('', f'{NS}/odom'), ('_far', f'{NS}/odom_far')):
        procs.append(subprocess.Popen(
            [sys.executable, ODOM_BRIDGE, '--ros-args',
             '-r', f'__node:=m3_check_odom_bridge{suffix}',
             '-p', f'input_topic:={src}',
             '-p', f'output_topic:={NS}/visual_odometry{suffix}',
             '-p', f'status_topic:={NS}/odom_bridge_status{suffix}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    h = OdomHarness()
    out, st = f'{NS}/visual_odometry', f'{NS}/odom_bridge_status'
    out_far, st_far = f'{NS}/visual_odometry_far', f'{NS}/odom_bridge_status_far'
    try:
        print(f'\nm3_bridge_check -- sandboxing {os.path.relpath(ODOM_BRIDGE, REPO)}\n')
        print('T8  healthy FAST-LIO stream is forwarded')
        end = time.time() + 15.0
        while time.time() < end and not (h.status.get(st) and h.status.get(st_far)):
            h.spin_for(0.2)
        check('T8 setup: both odom bridges up and reporting a guard state',
              bool(h.status.get(st)) and bool(h.status.get(st_far)),
              f'{h.status.get(st)!r} / {h.status.get(st_far)!r}')

        # ---- T8: healthy stream is forwarded, FRD, reset_counter 0 ------------
        h.spin_for(1.0)   # let the bridge's graph poll see our publisher
        h.stream(2.0, lambda t: (0.1 + 0.2 * t, 0.2, 0.3))
        h.spin_for(0.3)
        n, vo = h.vo[out]
        check('T8 samples forwarded', n > 20, f'n={n}')
        check('T8 FLU->FRD still (x,-y,-z)',
              vo is not None and close(vo.position[1], -0.2, 1e-3)
              and close(vo.position[2], -0.3, 1e-3),
              f'{[round(v, 3) for v in vo.position]}' if vo else 'no output')
        check('T8 reset_counter 0 and status TRACKING',
              vo is not None and vo.reset_counter == 0
              and h.status.get(st, '').startswith('TRACKING'), h.status.get(st))

        # ---- T9/T10: a jump latches, and SETTLING afterwards does not clear it --
        print('\nT9  a position jump latches DIVERGED')
        h.stream(1.0, lambda t: (0.5, 0.2, 0.3))
        h.spin_for(0.3)   # drain; 0.3 s is inside max_gap_s
        n0 = h.vo[out][0]
        h.stream(0.5, lambda t: (3.5, 0.2, 0.3))   # +3 m in one sample
        check('T9 jumped sample not forwarded', h.vo[out][0] == n0, f'{h.vo[out][0] - n0} leaked')
        check('T9 status DIVERGED, reason names the jump',
              h.status.get(st, '').startswith('DIVERGED') and 'jumped' in h.status.get(st, ''),
              h.status.get(st))
        print('\nT10 the diverged estimate settles (the 09-12 shape) -- still latched')
        h.stream(3.0, lambda t: (3.5, 0.2, 0.3))
        h.spin_for(0.3)
        check('T10 a steady, self-consistent wrong estimate is NOT re-trusted',
              h.vo[out][0] == n0 and h.status.get(st, '').startswith('DIVERGED'),
              f'{h.vo[out][0] - n0} forwarded; {h.status.get(st)}')

        # ---- T11: restarted FAST-LIO near origin resumes, reset_counter++ -----
        print('\nT11 a restarted FAST-LIO (new publisher, near origin) resumes')
        h.new_publisher()
        h.spin_for(1.0)
        h.stream(1.5, lambda t: (0.0, 0.0, 0.0))
        h.spin_for(0.3)
        n, vo = h.vo[out]
        check('T11 forwarding resumed', n > n0 + 10, f'{n - n0} forwarded')
        check('T11 reset_counter incremented to 1 (EKF2 resets onto the new frame)',
              vo is not None and vo.reset_counter == 1,
              f'reset_counter={vo.reset_counter if vo else None}')
        check('T11 status TRACKING', h.status.get(st, '').startswith('TRACKING'),
              h.status.get(st))

        # ---- T12a: late but continuous output is NOT a stall (the 09-09 shape) --
        print('\nT12 bunched delivery with continuous stamps does not latch')
        n0 = h.vo[out][0]
        h.spin_for(1.6)   # nothing arrives for 1.6 s ...
        base = h.get_clock().now().nanoseconds / 1e9 - 1.6
        for i in range(32):   # ... then 1.6 s of 20 Hz samples arrive at once
            m = Odometry()
            s_ = base + i * 0.05
            m.header.stamp.sec, m.header.stamp.nanosec = int(s_), int((s_ % 1) * 1e9)
            m.pose.pose.orientation.w = 1.0
            h.odom_pub.publish(m)
        h.spin_for(0.5)
        # Only "stays TRACKING and still forwards" is asserted: a 32-message burst overflows
        # the depth-10 queues on both sides, so the forwarded count is not the property here.
        check('T12a arrival gap with 0.05 s stamp spacing stays TRACKING',
              h.status.get(st, '').startswith('TRACKING') and h.vo[out][0] > n0 + 5,
              f'{h.vo[out][0] - n0} forwarded; {h.status.get(st)}')

        # ---- T12b: a STAMP gap latches (FAST-LIO with no scans publishes nothing)
        h.spin_for(1.6)
        n0 = h.vo[out][0]
        h.stream(1.0, lambda t: (0.0, 0.0, 0.0))
        h.spin_for(0.3)
        check('T12b a 1.6 s stamp gap latches DIVERGED, reason names the gap',
              h.status.get(st, '').startswith('DIVERGED')
              and 'no scan-matched pose' in h.status.get(st, ''), h.status.get(st))
        check('T12b nothing forwarded after the stall', h.vo[out][0] == n0,
              f'{h.vo[out][0] - n0} leaked')

        # ---- T13: a new publisher that is FAR from origin is not a restart ----
        print('\nT13 a new publisher far from the origin does not un-latch')
        h.new_publisher()
        h.spin_for(1.0)
        h.stream(1.5, lambda t: (3340.1, 1672.2, 86.66))
        h.spin_for(0.3)
        check('T13 still DIVERGED, nothing forwarded, reset_counter unchanged',
              h.vo[out][0] == n0 and h.status.get(st, '').startswith('DIVERGED')
              and 'reset_counter=1' in h.status.get(st, ''), h.status.get(st))

        # ---- T14: the 09-12 session exactly: bridge starts on a diverged pose -
        print('\nT14 a bridge whose FIRST sample is the 09-12 pose (-3340, -1672, -86.7)')
        h.stream(2.0, lambda t: (-3340.11, -1672.20, -86.66), pub=h.far_pub)
        h.spin_for(0.3)
        check('T14 nothing forwarded', h.vo[out_far][0] == 0, f'n={h.vo[out_far][0]}')
        check('T14 status DIVERGED, reason names the start radius',
              h.status.get(st_far, '').startswith('DIVERGED')
              and 'first sample' in h.status.get(st_far, ''), h.status.get(st_far))
    finally:
        h.destroy_node()
        for p in procs:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()


RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append((name, ok, detail))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f'  {detail}' if detail else ''))


def close(a, b, tol=TOL):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def all_nan(vals):
    return all(math.isnan(v) for v in vals)


def rounded(vals):
    return [round(float(v), 3) for v in vals]


def start_offboard_bridge(suffix='', extra=(), log=None):
    """One sandboxed px4_offboard_bridge. Its inputs are the shared /m3_check/ topics.
    `log`: a file for its console output (default: discarded)."""
    argv = [sys.executable, BRIDGE, '--ros-args']
    if suffix:
        argv += ['-r', f'__node:=m3_check_offboard_bridge{suffix}']
    argv += ['-p', f'cmd_topic:={NS}/pos_cmd',
             '-p', f'setpoint_topic:={NS}/trajectory_setpoint{suffix}',
             '-p', f'control_mode_topic:={NS}/offboard_control_mode{suffix}',
             '-p', f'local_position_topic:={NS}/vehicle_local_position',
             '-p', f'vehicle_control_mode_topic:={NS}/vehicle_control_mode',
             '-p', f'publish_rate:={RATE}',
             '-p', f'cmd_timeout:={TIMEOUT}',
             '-p', 'publish_velocity_ff:=true',
             '-p', 'warn_setpoint_error:=0.0']
    for p in extra:
        argv += ['-p', p]
    if log is None:
        return subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(log, 'wb') as f:
        return subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT)


def stop(p):
    if p.poll() is None:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


# ---- the camera yaw servo (Y) -----------------------------------------------------------

def f32(x):
    """x as the float32 a setpoint carries (round to nearest, like the serialiser)."""
    return struct.unpack('f', struct.pack('f', x))[0]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def deg(a):
    return math.degrees(a)


def frd_bearing(tx, ty, px_ned, py_ned):
    """The heading the servo should send, derived the other way round from yaw_servo.py:
    the aircraft in FLU is (px, -py), and the FRD heading is the FLU bearing negated."""
    return -math.atan2(ty - (-py_ned), tx - px_ned)


def slew(samples, field, limit):
    """(worst |change|/dt, pairs over the limit) between consecutive setpoints of one
    bridge; dt from their OWN timestamps. A pair is over when |change| > limit*dt + SLEW_TOL."""
    worst, bad = 0.0, 0
    for a, b in zip(samples, samples[1:]):
        dt = (b.ts - a.ts) / 1e6
        dv = getattr(b, field) - getattr(a, field)
        if field == 'yaw':
            dv = wrap(dv)
        if dt <= 0.0:
            bad += 1
            continue
        worst = max(worst, abs(dv) / dt)
        if abs(dv) > limit * dt + SLEW_TOL:
            bad += 1
    return worst, bad


def worst_err(samples, psi):
    """Largest |yaw - psi| in degrees over the samples (inf if there are none)."""
    return max((abs(deg(wrap(x.yaw - psi))) for x in samples), default=math.inf)


def static_target(h, x, y, **kw):
    return lambda: h.pub_target(x, y, **kw)


def hold_of(recs, zero_ys=True, same_stamp=False):
    """The stale hold in one bridge's debug messages [(receive time, [active, psi_des, psi_out,
    yawspeed, range_m, age, source])]: the last aimed message (source 1/2) before the first
    source 3, the run of 3s, and the first source 0 after it. -> (ok, text, info).
    ok: the run exists, every message in it has psi_out equal to the last aimed psi_out (the
    aim had converged in every scenario that uses this) and, with zero_ys, yawspeed exactly
    0.0, and the fall-back came 0.5 s of tick time after the last aimed tick:
      same_stamp (the target stopped: one stamp throughout): by the age, which runs on the
        bridge's own tick clock -- (0.5, 0.58] s, i.e. one tick plus one dropped message;
      otherwise (new messages kept arriving, stamps changing): by the receive clock,
        (0.44, 0.66) s -- delivery jitter on top of the tick clock."""
    d = [x for _, x in recs]
    i3 = next((i for i, x in enumerate(d) if x[6] == 3.0), None)
    if i3 is None or i3 == 0 or d[i3 - 1][6] not in (1.0, 2.0):
        return False, f'no hold after an aimed tick (sources {sorted({x[6] for x in d})})', {}
    i0 = next((i for i in range(i3, len(d)) if d[i][6] != 3.0), None)
    if i0 is None or d[i0][6] != 0.0:
        return False, 'the hold did not end in a fall-back', {}
    last, run = d[i3 - 1], d[i3:i0]
    held = all(x[2] == last[2] and (not zero_ys or x[3] == 0.0) for x in run)
    dt_recv = recs[i0][0] - recs[i3 - 1][0]
    dt_age = d[i0][5] - last[5]
    if same_stamp:
        timing = 0.5 - 1e-6 < dt_age <= 0.58
        how = f'fall-back {dt_age:.3f} s (tick clock) after the last aimed tick'
    else:
        timing = 0.44 < dt_recv < 0.66
        how = f'fall-back {dt_recv:.3f} s (receive clock) after the last aimed tick'
    info = dict(n=len(run), dt_recv=dt_recv, dt_age=dt_age, age3=d[i3][5], held=last[2])
    return (held and timing,
            f'held {len(run)} ticks at {deg(last[2]):+.2f} deg, yawspeed '
            f'{"0.0" if all(x[3] == 0.0 for x in run) else "NON-ZERO"}; {how}', info)


def orbit(h, r, th0_deg, w_deg_s, t0, dur=None):
    """A target on a circle of radius r around the origin, at FLU angle th0 + w*(t - t0)
    (degrees; counter-clockwise, i.e. to the LEFT, positive), frozen after `dur` s."""
    def pub():
        s = time.time() - t0
        if dur is not None:
            s = min(max(s, 0.0), dur)
        th = math.radians(th0_deg + w_deg_s * s)
        h.pub_target(r * math.cos(th), r * math.sin(th))
    return pub


def run_servo_checks(h, proc_default):
    """Ya-Yk: the default bridge (switches off) stays up; three servo bridges join it. Then
    Y2: the default bridge and _nohold stop, two round-one bridges join _servo and _ff."""
    logdir = tempfile.mkdtemp(prefix='m3_servo_')
    n_results = len(RESULTS)
    procs = {}

    def start(s, extra):
        procs[s] = start_offboard_bridge(
            s, extra + [f'yaw_target_topic:={YT_TOPIC}',
                        f'yaw_servo_debug_topic:={NS}/yaw_servo{s}'],
            log=os.path.join(logdir, f'bridge{s}.log'))
    try:
        for s, extra in SERVO_BRIDGES.items():
            start(s, extra)
        up = _servo_checks(h)
        if up:
            stop(procs['_nohold'])
            stop(proc_default)
            for s, extra in SERVO_R1_BRIDGES.items():
                start(s, extra)
            up = _servo_checks_r2(h)
    finally:
        for p in procs.values():
            stop(p)
    if up:
        _servo_log_checks(logdir)
    if all(ok for _, ok, _ in RESULTS[n_results:]):
        shutil.rmtree(logdir, ignore_errors=True)
    else:
        print(f'  (the servo bridges\' logs are kept in {logdir})')


def _servo_checks(h):
    S = list(SERVO_BRIDGES)
    ALL = [''] + S
    YS, YD = -0.35, 0.1                         # SUPER's yaw and yaw rate, FLU, as commanded
    PSI_S, YSPD_S = f32(wrap(-YS)), f32(-YD)    # what the bridge sends for them: +20.05 deg
    P, V = (0.3, -0.4, 1.5), (0.1, 0.2, -0.05)
    A, J = (0.3, -0.2, 0.1), (0.05, -0.07, 0.02)

    def cmd():
        h.send_cmd(P, V, YS, YD, acc=A, jerk=J)

    def latch_yaw(heading):
        """The IDLE latch / HOLD's frozen yaw: PX4's float32 heading, wrapped, as float32."""
        return f32(wrap(f32(heading)))

    print('\nY   camera yaw servo: 3 more bridges, switch ON (plan-camera-yaw-servo.md 4.2)')
    h.watch_servo(ALL)
    h.set_state(False, True)                  # disarmed Offboard: every bridge sends its latch
    h.set_lpos(0.0, 0.0, -1.5, 0.7)

    def up():
        return (all(len(h.ysp[s]) > 10 for s in S)
                and h.count_subscribers(YT_TOPIC) == len(S)
                and all(h.count_publishers(f'{NS}/yaw_servo{s}') == 1 for s in S))
    end = time.time() + 20.0
    while time.time() < end and not up():
        h.yrun(0.5)
    ok = up()
    check('Y  setup: 3 servo bridges streaming; yaw-target subscriptions and debug publishers seen',
          ok, f'setpoints {[len(h.ysp[s]) for s in S]}, '
          f'{h.count_subscribers(YT_TOPIC)} subscribers on {YT_TOPIC}')
    if not ok:
        return False
    h.yrun(0.5)

    # ---- Ya (structure): switch off = no new subscription, no new publisher ---------
    def graph(node):
        try:
            return (h.get_subscriber_names_and_types_by_node(node, '/'),
                    h.get_publisher_names_and_types_by_node(node, '/'))
        except Exception as e:  # node not discovered: reported as the failure
            return [('?', [repr(e)])], [('?', [repr(e)])]
    subs, pubs = graph('px4_offboard_bridge')
    sub_t, pub_t = sorted(t for t, _ in subs), sorted(t for t, _ in pubs)
    new = ([t for t, ty in subs if t == YT_TOPIC or 'geometry_msgs/msg/PointStamped' in ty]
           + [t for t, ty in pubs if 'std_msgs/msg/Float32MultiArray' in ty])
    check('Ya switch off: the default bridge has no yaw-target subscription, no debug publisher',
          not new and sub_t == sorted([f'{NS}/pos_cmd', f'{NS}/vehicle_local_position',
                                       f'{NS}/vehicle_control_mode']),
          f'subscribes {sub_t}; publishes {pub_t}')
    subs, pubs = graph('m3_check_offboard_bridge_servo')
    check('Ya switch on (control): the servo bridge has both',
          any(t == YT_TOPIC for t, _ in subs)
          and any(t == f'{NS}/yaw_servo_servo' for t, _ in pubs),
          f'subscribes {sorted(t for t, _ in subs)}')

    # ---- Yh (IDLE, outside Offboard) -----------------------------------------------
    print('\nYh  IDLE (disarmed Offboard) and outside Offboard: no servo')
    tgt_off = static_target(h, 4.0 * math.cos(1.2), 4.0 * math.sin(1.2))  # 109 deg off the nose
    t0 = time.time()
    h.yrun(1.5, cmd=cmd, target=tgt_off)
    exp = latch_yaw(0.7)
    counts, bad = [], 0
    for s in S:
        w = h.seg(s, t0 + 0.3)
        counts.append(len(w))
        bad += sum(1 for x in w if not (x.yaw == exp and x.yawspeed == 0.0
                                        and x.pos == (0.0, 0.0, -1.5)))
    check('Yh IDLE, SUPER commanding, fresh target 109 deg off the nose: the latch exactly '
          '(yaw = heading), 3 servo bridges', min(counts) > 40 and bad == 0,
          f'{bad} of {sum(counts)} differ; yaw {exp:+.6f}')
    nd = sum(len(h.dseg(s, t0)) for s in S)
    check('Yh IDLE: no debug output (it is published only while engaged)', nd == 0,
          f'{nd} messages in 1.5 s')
    d0 = sum(len(h.ydbg[s]) for s in S)
    h.set_state(False, False)
    h.yrun(0.3, cmd=cmd, target=tgt_off)
    n0 = sum(len(h.ysp[s]) for s in S)
    h.yrun(1.0, cmd=cmd, target=tgt_off)
    n1 = sum(len(h.ysp[s]) for s in S) - n0
    check('Yh not in Offboard (disarmed): no setpoint from the servo bridges', n1 == 0,
          f'{n1} sent in 1.0 s')
    h.set_state(True, False)                  # armed in Position: PX4's task owns the topic
    h.yrun(0.3, cmd=cmd, target=tgt_off)
    n0 = sum(len(h.ysp[s]) for s in S)
    h.yrun(1.0, cmd=cmd, target=tgt_off)
    n2 = sum(len(h.ysp[s]) for s in S) - n0
    nd = sum(len(h.ydbg[s]) for s in S) - d0
    check('Yh armed, not in Offboard, SUPER and target live: no setpoint, no debug either',
          n2 == 0 and nd == 0, f'{n2} setpoints, {nd} debug in 1.0 s')

    # ---- Yi: HOLD with a fresh target ------------------------------------------------
    print('\nYi  HOLD (armed Offboard, SUPER silent) with a fresh target: aims, position frozen')
    E, E2 = (1.0, -2.0, -1.5, -0.4), (1.3, -2.2, -1.6, -0.35)      # NED; E2 = drifted
    tx, ty = 1.3 + 5.0 * math.cos(math.radians(100)), 2.2 + 5.0 * math.sin(math.radians(100))
    tgt_i = static_target(h, tx, ty)
    h.set_lpos(*E)
    h.yrun(0.6, target=tgt_i)                 # SUPER silent for > cmd_timeout
    t_eng = time.time()
    h.set_state(True, True)                   # the ch11 engage, armed: HOLD
    h.yrun(0.3, target=tgt_i)
    h.set_lpos(*E2)                           # drifts: HOLD must not chase; the servo aims from it
    h.yrun(2.2, target=tgt_i)
    exp = frd_bearing(tx, ty, E2[0], E2[1])   # -100 deg
    frozen = latch_yaw(E[3])
    t_now = time.time()
    errs = [worst_err(h.seg(s, t_now - 0.4), exp) for s in ('_servo', '_ff')]
    check('Yi HOLD, fresh target: aims from the CURRENT position (_servo, _ff) within 0.5 deg',
          max(errs) <= 0.5,
          f'expected {deg(exp):+.2f} deg, worst errors {errs[0]:.3f} / {errs[1]:.3f}')
    held, n = 0, 0
    for s in ALL:
        for x in h.seg(s, t_eng):
            n += 1
            held += (x.pos == (f32(E[0]), f32(E[1]), f32(E[2]))
                     and all_nan(x.vel) and all_nan(x.acc) and all_nan(x.jerk))
    check('Yi HOLD: position frozen at the engage point, velocity/acceleration/jerk NaN, all 4',
          n > 300 and held == n, f'{n - held} of {n} differ, pos {rounded(E[:3])}')
    w = h.seg('_nohold', t_eng)
    bad = sum(1 for x in w if not (x.yaw == frozen and x.yawspeed == 0.0))
    check('Yi yaw_servo_aim_in_hold:=false: the frozen yaw in the engage HOLD, exactly',
          len(w) > 100 and bad == 0, f'{bad} of {len(w)} differ from {deg(frozen):+.3f} deg')
    ds, dn = h.dseg('_servo', t_now - 0.4), h.dseg('_nohold', t_eng + 0.1)
    check('Yi debug: _servo active 1 source 1; _nohold active 0 source 0 (before the first TRACK)',
          ds and dn and all(d[0] == 1.0 and d[6] == 1.0 for d in ds)
          and all(d[0] == 0.0 and d[6] == 0.0 for d in dn), f'{len(ds)} / {len(dn)} messages')
    # The target goes silent. HOLD's fall-back is the FROZEN yaw: had the servo's output been
    # written back into the latched setpoint, it would stay where the servo left the nose.
    # Round two: the default bridges first hold the aim for 0.5 s (the round-one assertion,
    # an immediate fall-back, runs on the stale_hold_s:=0 bridges in Y2).
    t_st = time.time()
    h.yrun(2.6)
    res = []
    for s in ('_servo', '_ff'):
        worst, nbad = slew(h.seg(s, t_st - 0.1), 'yaw', SERVO_MAX_RATE)
        tail = h.seg(s, t_st + 2.2)
        hok, htxt, _ = hold_of(h.dsegt(s, t_st - 0.3), same_stamp=True)
        res.append((nbad, worst, tail, all(x.yaw == frozen and x.yawspeed == 0.0 for x in tail),
                    hok, htxt))
    check('Yi HOLD, target gone stale [default stale hold 0.5 s]: holds the aim (source 3, yawspeed '
          '0) 0.5 s, then back to the FROZEN yaw exactly, through the 90 deg/s slew (_servo, _ff)',
          all(nbad == 0 and tail and same and hok for nbad, _, tail, same, hok, _ in res),
          f'{res[0][5]}; yaw {deg(res[0][2][-1].yaw) if res[0][2] else math.nan:+.3f} deg, frozen '
          f'{deg(frozen):+.3f}; worst {deg(max(r[1] for r in res)):.1f} deg/s, '
          f'{sum(r[0] for r in res)} pairs over')

    # ---- Ya: switch off, TRACK -------------------------------------------------------
    print("\nYa  switches off: SUPER's yaw exactly, while a fresh target is published")
    h.set_lpos(0.0, 0.0, -1.5, 0.0)
    t0 = time.time()
    h.yrun(2.0, cmd=cmd, target=static_target(h, 5.0, 5.0))
    w = h.seg('', t0 + 0.5)
    bad = sum(1 for x in w if not (x.yaw == PSI_S and x.yawspeed == YSPD_S))
    check('Ya switch off, fresh target live: yaw == -yaw wrapped, yawspeed == -yaw_dot, '
          'float32-exact on every setpoint', len(w) > 50 and bad == 0,
          f'{bad} of {len(w)} differ; yaw {PSI_S:+.7f} yawspeed {YSPD_S:+.7f}')
    exp = frd_bearing(5.0, 5.0, 0.0, 0.0)
    err = worst_err(h.seg('_servo', t0 + 1.5), exp)
    check('Ya ... and the target IS live: the servo bridge aims at it (-45 deg)', err <= 0.5,
          f'worst error {err:.3f} deg')
    nd = len(h.dseg('_servo', t0 + 0.5, t0 + 2.0))
    check('Ya debug: one message per engaged tick', 40.0 <= nd / 1.5 <= 60.0,
          f'{nd / 1.5:.1f} Hz')

    # ---- Yb: sign --------------------------------------------------------------------
    print('\nYb  sign: a target to the LEFT is a LEFT turn (FRD yaw < 0), not pi/2 - psi')
    t0 = time.time()
    h.yrun(2.0, cmd=cmd, target=static_target(h, 0.0, 5.0))      # FLU +y: 5 m to the left
    w = h.seg('_servo', t0 + 1.4)
    exp = frd_bearing(0.0, 5.0, 0.0, 0.0)                        # -90 deg
    trap = math.pi / 2 - math.atan2(5.0, 0.0)                    # ENU->NED form: 0, ahead
    err = worst_err(w, exp)
    check('Yb target at FLU +y (left): FRD yaw < 0, = -atan2(dy,dx) = -90 deg within 0.5 deg, '
          'not pi/2-psi (0 deg)',
          len(w) > 20 and all(x.yaw < 0.0 for x in w) and err <= 0.5
          and min(abs(deg(wrap(x.yaw - trap))) for x in w) > 45.0,
          f'yaw {deg(w[-1].yaw) if w else math.nan:+.3f} deg, worst error {err:.3f}')
    d = h.dseg('_servo', t0 + 1.4)
    check('Yb debug: active 1, source 1 (servo), range 5.00 m',
          d and all(x[0] == 1.0 and x[6] == 1.0 and abs(x[4] - 5.0) < 0.01 for x in d),
          f'{len(d)} messages, range {d[-1][4] if d else math.nan:.3f}')
    h.set_lpos(2.0, -3.0, -1.5, 0.0)                               # FLU (2, 3)
    t0 = time.time()
    h.yrun(2.0, cmd=cmd, target=static_target(h, -1.0, 7.0))
    exp = frd_bearing(-1.0, 7.0, 2.0, -3.0)                        # -126.87 deg
    wrong = {'lpos ignored': -math.atan2(7.0, -1.0),
             'lpos y sign': -math.atan2(7.0 - (-3.0), -1.0 - 2.0),
             'pi/2-psi': math.pi / 2 - math.atan2(4.0, -3.0)}
    ws = [h.seg(s, t0 + 1.4) for s in S]
    err = max(worst_err(w, exp) for w in ws)
    far = min(abs(deg(wrap(x.yaw - v))) for w in ws for x in w for v in wrong.values()) \
        if all(ws) else 0.0
    check('Yb aircraft away from the origin (NED 2,-3): -126.87 deg within 0.5 deg on all 3; '
          'not the lpos-ignored / lpos-sign / pi/2 answers', err <= 0.5 and far > 5.0,
          f'worst error {err:.3f} deg; nearest wrong answer {far:.1f} deg away '
          f'({", ".join(f"{k} {deg(v):+.1f}" for k, v in wrong.items())})')

    # ---- Yc: a sweep through +-180 ------------------------------------------------------
    print('\nYc  target bearing swept through +-180 deg at 120 deg/s: continuous, <= 90 deg/s')
    h.set_lpos(0.0, 0.0, -1.5, 0.0)
    c120, s120 = math.cos(math.radians(120)), math.sin(math.radians(120))
    h.yrun(1.2, cmd=cmd, target=static_target(h, 5.0 * c120, 5.0 * s120))   # settle at FRD -120
    t0 = time.time()
    h.yrun(2.6, cmd=cmd, target=orbit(h, 5.0, 120.0, 120.0, t0, dur=1.0))   # FRD -120 -> +120
    worst, nbad, npairs = 0.0, 0, 0
    for s in S:
        w = h.seg(s, t0 - 0.2)
        wst, nb = slew(w, 'yaw', SERVO_MAX_RATE)
        worst, nbad, npairs = max(worst, wst), nbad + nb, npairs + len(w) - 1
    check('Yc every pair within 90 deg/s x its own dt (+0.001 deg), 3 bridges; the limit did bind',
          nbad == 0 and npairs > 300 and worst >= math.radians(85),
          f'{nbad} of {npairs} pairs over; worst {deg(worst):.2f} deg/s')
    w = h.seg('_servo', t0 - 0.2)
    travel = sum(wrap(b.yaw - a.yaw) for a, b in zip(w, w[1:]))
    crossed = any(abs(x.yaw) > math.radians(175) for x in w)
    fin = deg(wrap(w[-1].yaw - math.radians(120))) if w else math.nan
    check('Yc continuous through +-180: crossed it the short way (-120 deg), ended at +120 deg',
          crossed and abs(deg(travel) + 120.0) <= 0.5 and abs(fin) <= 0.5,
          f'travel {deg(travel):+.2f} deg, end error {fin:+.3f} deg')

    # ---- Yd: a 180 deg jump ---------------------------------------------------------------
    print('\nYd  a 180 deg target jump: every tick <= 90 deg/s x dt, arrival <= 2.2 s')
    h.yrun(2.0, cmd=cmd, target=static_target(h, 5.0, 0.0))        # settle at 0
    t_j = time.time()
    h.yrun(2.6, cmd=cmd, target=static_target(h, -5.0, -0.05))
    exp = frd_bearing(-5.0, -0.05, 0.0, 0.0)                       # +179.43 deg
    worst, nbad, arrs = 0.0, 0, []
    for s in S:
        w = h.seg(s, t_j - 0.2)
        wst, nb = slew(w, 'yaw', SERVO_MAX_RATE)
        worst, nbad = max(worst, wst), nbad + nb
        arrs.append(next((x.ts / 1e6 - t_j for x in w
                          if x.ts / 1e6 >= t_j and abs(wrap(x.yaw - exp)) <= math.radians(0.01)),
                         math.inf))
    check('Yd 180 deg jump: every pair within 90 deg/s x its own dt, 3 bridges',
          nbad == 0 and worst >= math.radians(85), f'{nbad} over; worst {deg(worst):.2f} deg/s')
    check('Yd ... and the heading arrives in <= 2.2 s', max(arrs) <= 2.2,
          f'{", ".join(f"{a:.3f}" for a in arrs)} s ({deg(exp):.2f} deg at 90 deg/s = '
          f'{deg(exp) / 90.0:.3f} s)')

    # ---- Ye: stale and future-stamped targets ----------------------------------------------
    print("\nYe  stale and future-stamped targets: fall back to SUPER's yaw, slew-limited")
    tex, tey = 5.0 * math.cos(math.radians(50)), 5.0 * math.sin(math.radians(50))
    exp = frd_bearing(tex, tey, 0.0, 0.0)                          # -50 deg
    h.yrun(2.0, cmd=cmd, target=static_target(h, tex, tey))
    # Round two: the default bridges hold the aim 0.5 s before each of these fall-backs. The
    # round-one assertions (immediate fall-back) run on the stale_hold_s:=0 bridges in Y2.
    t0 = time.time()
    h.yrun(2.1, cmd=cmd, target=static_target(h, tex, tey, age=0.8))   # arriving, 0.8 s old
    res, txt = [], ''
    for s in ('_servo', '_ff'):
        _, nb = slew(h.seg(s, t0 - 0.1), 'yaw', SERVO_MAX_RATE)
        tail = h.seg(s, t0 + 1.7)
        hok, htxt, _ = hold_of(h.dsegt(s, t0 - 0.3))
        txt = txt or htxt
        res.append(nb == 0 and len(tail) > 10 and hok
                   and all(x.yaw == PSI_S and x.yawspeed == YSPD_S for x in tail))
    d = h.dseg('_servo', t0 + 0.75)
    check("Ye stale stamps (0.8 s old, still arriving) [default stale hold 0.5 s]: holds the -50 deg "
          "aim 0.5 s (source 3, yawspeed 0), then SUPER's yaw and yawspeed exactly, no jump "
          '(_servo, _ff)', all(res), f'{res}; _servo {txt}')
    check('Ye ... debug after the hold: active 0, source 0, target age > 0.5 s',
          d and all(x[0] == 0.0 and x[6] == 0.0 and x[5] > 0.79 for x in d),
          f'ages {min(x[5] for x in d):.3f}-{max(x[5] for x in d):.3f} s' if d else 'none')
    t0 = time.time()
    h.yrun(1.5, cmd=cmd, target=static_target(h, tex, tey))
    err = worst_err(h.seg('_servo', t0 + 1.0), exp)
    check('Ye fresh again: the servo takes the heading back (-50 deg)', err <= 0.5,
          f'worst error {err:.3f} deg')
    t0 = time.time()
    h.yrun(2.4, cmd=cmd)                                            # the target goes silent
    d = h.dseg('_servo', t0 - 0.2)
    act = [x[5] for x in d if x[6] == 1.0]
    hold = [x for x in d if x[6] == 3.0]
    fb = [x[5] for x in d if x[6] == 0.0]
    _, nb = slew(h.seg('_servo', t0 - 0.1), 'yaw', SERVO_MAX_RATE)
    tail = h.seg('_servo', t0 + 2.1)
    dfb = fb[0] - max(act) if act and fb else math.nan
    check("Ye target silent [default stale hold 0.5 s]: aims until its age passes 0.5 s, holds that "
          "aim (source 3, yawspeed 0) until 0.5 s later, then SUPER's yaw exactly, slew-limited",
          act and hold and fb and max(act) <= 0.5 + 1e-6 and 0.5 <= hold[0][5] <= 0.55
          and 0.5 - 1e-6 < dfb <= 0.58
          and all(x[2] == hold[0][2] and x[3] == 0.0 for x in hold) and nb == 0
          and tail and all(x.yaw == PSI_S for x in tail),
          f'last active age {max(act) if act else math.nan:.3f} s, first hold age '
          f'{hold[0][5] if hold else math.nan:.3f} s ({len(hold)} ticks), first fall-back age '
          f'{fb[0] if fb else math.nan:.3f} s ({dfb:.3f} s after the last aimed tick), {nb} pairs '
          'over the slew')
    t0 = time.time()
    h.yrun(1.2, cmd=cmd, target=static_target(h, tex, tey, age=-0.5))  # 0.5 s in the future
    w = h.seg('_servo', t0)
    d = h.dseg('_servo', t0 + 0.1)
    check('Ye future-stamped target (+0.5 s, over the 0.2 s tolerance): ignored, '
          "SUPER's yaw exactly",
          len(w) > 40 and all(x.yaw == PSI_S for x in w)
          and d and all(x[0] == 0.0 and x[5] < -0.2 for x in d),
          f'{sum(1 for x in w if x.yaw != PSI_S)} of {len(w)} differ; ages '
          f'{min(x[5] for x in d):+.3f}..{max(x[5] for x in d):+.3f} s' if d else 'no debug')
    t0 = time.time()
    h.yrun(1.5, cmd=cmd, target=static_target(h, tex, tey, age=-0.1))  # inside the tolerance
    err = worst_err(h.seg('_servo', t0 + 1.0), exp)
    check('Ye ... while a target stamped +0.1 s ahead (inside the tolerance) is used', err <= 0.5,
          f'worst error {err:.3f} deg')

    # ---- Yf: close range ----------------------------------------------------------------
    print('\nYf  target at 1.5 m moving sideways fast: heading held, yawspeed 0')
    t0 = time.time()
    h.yrun(2.0, cmd=cmd, target=orbit(h, 5.0, 50.0, 20.0, t0))     # ff builds on _ff
    pre = h.seg('_ff', t0 + 1.4)
    pre_ff = deg(sum(x.yawspeed for x in pre) / len(pre)) if pre else math.nan
    t_c = time.time()

    def side():   # x 1.5 m, y +-1.8 m at 0.6 Hz: range <= 2.34 m, bearing +-50 deg, <= 6.8 m/s
        h.pub_target(1.5, 1.8 * math.sin(2.0 * math.pi * 0.6 * (time.time() - t_c)))
    h.yrun(2.0, cmd=cmd, target=side)
    spread, nonzero, n = 0.0, 0, 0
    for s in ('_servo', '_ff'):
        w = h.seg(s, t_c + 0.3)
        n += len(w)
        spread = max(spread, max((abs(wrap(x.yaw - w[0].yaw)) for x in w), default=math.inf))
        nonzero += sum(1 for x in h.seg(s, t_c + 0.15) if x.yawspeed != 0.0)
    check('Yf target at 1.5 m swinging +-50 deg at up to 6.8 m/s: heading held, yawspeed exactly 0 '
          '(_servo, _ff)', pre_ff < -10.0 and n > 100 and deg(spread) <= 0.01 and nonzero == 0,
          f'spread {deg(spread):.4f} deg, {nonzero} non-zero yawspeed; '
          f'_ff was {pre_ff:+.1f} deg/s before')
    d = h.dseg('_ff', t_c + 0.15)
    check('Yf debug: active 1, source 2 (close-range hold), range < 2.5 m',
          d and all(x[0] == 1.0 and x[6] == 2.0 and x[4] < 2.5 for x in d),
          f'{len(d)} messages, range {min(x[4] for x in d):.2f}-{max(x[4] for x in d):.2f} m'
          if d else 'none')

    # ---- Yg: the feed-forward -------------------------------------------------------------
    print('\nYg  feed-forward (fix 2): value, sign, cap, slew, a jump; and ff off = 0')
    t0 = time.time()
    h.yrun(3.5, cmd=cmd, target=orbit(h, 5.0, 90.0, 30.0, t0))     # to the LEFT at 30 deg/s
    v = [deg(x.yawspeed) for x in h.seg('_ff', t0 + 1.5)]
    stats = (f'mean {sum(v) / len(v):+.2f}, min {min(v):+.2f}, max {max(v):+.2f} deg/s over '
             f'{len(v)} ticks') if v else 'no samples'
    check('Yg ff: target orbiting at 30 deg/s, delivered at 15 Hz -> |yawspeed| 30 +- 5 deg/s on '
          'every settled tick', len(v) > 50 and all(25.0 <= abs(x) <= 35.0 for x in v), stats)
    check('Yg ff sign: a target circling to the LEFT (FLU counter-clockwise) -> NEGATIVE FRD '
          'yawspeed', v and max(v) < 0.0, stats)
    w = h.seg('_servo', t0 + 0.2)
    d = h.dseg('_servo', t0 + 0.2)
    check('Yg ff off (_servo): yawspeed exactly 0 on every tick while the servo is active',
          len(w) > 100 and all(x.yawspeed == 0.0 for x in w) and d and all(x[0] == 1.0 for x in d),
          f'{sum(1 for x in w if x.yawspeed != 0.0)} of {len(w)} non-zero')
    worst1, nb1 = slew(h.seg('_ff', t0 - 0.1), 'yawspeed', SERVO_FF_ACC)
    t1 = time.time()
    h.yrun(3.0, cmd=cmd, target=orbit(h, 5.0, 90.0 + 30.0 * (t1 - t0), -100.0, t1))
    w = h.seg('_ff', t1 - 0.1)
    worst2, nb2 = slew(w, 'yawspeed', SERVO_FF_ACC)
    peak = max((abs(x.yawspeed) for x in w), default=math.inf)
    tail = h.seg('_ff', t1 + 2.0)
    check('Yg ff: target orbiting at 100 deg/s the other way -> capped at 60 deg/s (reached, never '
          'exceeded)', peak <= SERVO_FF_MAX + SLEW_TOL and len(tail) > 30
          and all(abs(x.yawspeed - SERVO_FF_MAX) <= math.radians(0.01) for x in tail),
          f'peak {deg(peak):.4f} deg/s; last 1 s {deg(min(x.yawspeed for x in tail)):+.3f}..'
          f'{deg(max(x.yawspeed for x in tail)):+.3f}' if tail else 'no samples')
    check('Yg ff slew: every pair within 90 deg/s^2 x its own dt (+0.001), both orbits; '
          'it did bind',
          nb1 == 0 and nb2 == 0 and max(worst1, worst2) >= math.radians(85),
          f'{nb1 + nb2} over; worst {deg(max(worst1, worst2)):.2f} deg/s^2')
    h.yrun(1.5, cmd=cmd, target=static_target(h, 5.0, 0.0))        # ff decays to 0
    t_j = time.time()
    h.yrun(1.5, cmd=cmd, target=static_target(h, 5.0, 5.0))        # 5 m sideways: 45 deg
    worst, nb = slew(h.seg('_ff', t_j - 0.3), 'yawspeed', SERVO_FF_ACC)
    peak = max((abs(x.yawspeed) for x in h.seg('_ff', t_j)), default=math.inf)
    err = worst_err(h.seg('_ff', t_j + 1.1), frd_bearing(5.0, 5.0, 0.0, 0.0))
    check('Yg a 5 m target jump in one message: no yawspeed step over the ff slew, not fed forward '
          '(|yawspeed| <= 1 deg/s); the heading turns to it',
          nb == 0 and peak <= math.radians(1.0) and err <= 0.5,
          f'{nb} pairs over, peak |yawspeed| {deg(peak):.3f} deg/s, heading error {err:.3f} deg')

    # ---- Yj: bad inputs -> fall-back ---------------------------------------------------
    print("\nYj  wrong frame_id, NaN target, lpos invalid: fall back to SUPER's yaw")
    tjx, tjy = 5.0 * math.cos(math.radians(25)), 5.0 * math.sin(math.radians(25))
    exp_j = frd_bearing(tjx, tjy, 0.0, 0.0)                        # -25 deg; SUPER +20.05
    tgt_ok = static_target(h, tjx, tjy)
    bad_frame = static_target(h, tjx, tjy, frame='camera_color_optical_frame')
    cases = (('wrong frame_id', bad_frame, False),
             ('NaN target', static_target(h, float('nan'), tjy), False),
             ('lpos xy_valid false', tgt_ok, True))
    for name, stim, lpos_bad in cases:
        t0 = time.time()
        h.yrun(1.3, cmd=cmd, target=tgt_ok)
        aim = worst_err(h.seg('_servo', t0 + 0.9), exp_j)
        if lpos_bad:
            h.set_lpos_valid(False)
        t1 = time.time()
        h.yrun(1.8, cmd=cmd, target=stim)
        h.set_lpos_valid(True)
        _, nb = slew(h.seg('_servo', t1 - 0.1), 'yaw', SERVO_MAX_RATE)
        tail = h.seg('_servo', t1 + 1.4)
        d = h.dseg('_servo', t1 + 1.4)
        hok, htxt, _ = hold_of(h.dsegt('_servo', t1 - 0.3))
        check(f"Yj {name} [default stale hold 0.5 s]: holds the -25 deg aim 0.5 s (source 3, "
              "yawspeed 0), then back to SUPER's yaw exactly, no jump",
              aim <= 0.5 and nb == 0 and len(tail) > 10 and hok
              and all(x.yaw == PSI_S and x.yawspeed == YSPD_S for x in tail)
              and d and all(x[0] == 0.0 and x[6] == 0.0 for x in d),
              f'aim error before {aim:.3f} deg; {htxt}; {nb} pairs over the slew; end yaw '
              f'{deg(tail[-1].yaw) if tail else math.nan:+.3f} deg (SUPER {deg(PSI_S):+.3f})')

    # ---- Yk: position/velocity/acceleration/jerk identical --------------------------------
    print('\nYk  position/velocity/acceleration/jerk: the same float32 bytes, servo on or off')
    expected, k = set(), [0]

    def kcmd():   # a different command every 10 ms
        k[0] += 1
        i = k[0]
        pos = (0.3 + 0.01 * i, -0.4 + 0.02 * i, 1.5 + 0.005 * i)
        vel = (0.1 + 0.003 * i, 0.2 - 0.002 * i, -0.05 + 0.001 * i)
        acc = (0.3 - 0.004 * i, -0.2 + 0.003 * i, 0.1 + 0.002 * i)
        jerk = (0.05 + 0.001 * i, -0.07 + 0.0015 * i, 0.02 - 0.001 * i)
        h.send_cmd(pos, vel, YS, YD, acc=acc, jerk=jerk)
        expected.add(struct.pack('12f', *[c for v in (pos, vel, acc, jerk)
                                          for c in (v[0], -v[1], -v[2])]))
    t0 = time.time()
    h.yrun(2.0, cmd=kcmd, target=tgt_ok)
    res = [(s or 'default', len(w), sum(1 for x in w if x.raw not in expected))
           for s in ALL for w in [h.seg(s, t0 + 0.4)]]
    aiming = sum(1 for x in h.seg('_servo', t0 + 0.4) if x.yaw != PSI_S)
    check('Yk TRACK, a new command every 10 ms, servo active: every setpoint\'s pos/vel/acc/jerk '
          'is '
          'the exact float32 (x,-y,-z) of a command sent, all 4 bridges',
          all(n > 50 and nb == 0 for _, n, nb in res) and aiming > 50,
          '; '.join(f'{s} {nb}/{n} off' for s, n, nb in res) + f'; servo yaw != SUPER on {aiming}')

    # ---- Yi: HOLD after TRACK --------------------------------------------------------------
    # SUPER stops (at its goal, say) while the servo aims: HOLD at SUPER's last point. Then
    # the target goes stale. The fall-back is SUPER's LAST commanded yaw -- which the servo
    # must never have overwritten in the latched setpoint.
    h.yrun(0.8, cmd=cmd, target=tgt_ok)       # TRACK, aiming at -25 deg; SUPER's yaw +20.05
    h.yrun(0.8, target=tgt_ok)                # SUPER silent: HOLD, still aiming
    t_s = time.time()
    h.yrun(2.4)                               # the target goes silent too
    aim = worst_err(h.seg('_servo', t_s - 0.3, t_s), exp_j)
    tail = h.seg('_servo', t_s + 2.0)
    hok, htxt, _ = hold_of(h.dsegt('_servo', t_s - 0.3), same_stamp=True)
    p_last = (f32(P[0]), f32(-P[1]), f32(-P[2]))
    check("Yi HOLD after TRACK, then the target goes stale [default stale hold 0.5 s]: holds the aim "
          "0.5 s, then back to SUPER's LAST yaw exactly, position held at its last point",
          aim <= 0.5 and len(tail) > 10 and hok
          and all(x.yaw == PSI_S and x.yawspeed == 0.0 and x.pos == p_last and all_nan(x.vel)
                  for x in tail),
          f'aim error in HOLD {aim:.3f} deg; {htxt}; end yaw '
          f'{deg(tail[-1].yaw) if tail else math.nan:+.3f} deg (SUPER {deg(PSI_S):+.3f})')

    # ---- Yh: takeover, and re-engage from the vehicle's heading ---------------------------
    print("\nYh  takeover stops the servo bridges too; re-engage starts from the vehicle's heading")
    h.yrun(0.8, cmd=cmd, target=tgt_ok)       # engaged, aiming at -25 deg
    h.set_state(True, False)                  # the pilot's ch11 takeover: a fresh report
    h.yrun(0.3, cmd=cmd, target=tgt_ok)
    n0 = sum(len(h.ysp[s]) for s in S)
    d0 = sum(len(h.ydbg[s]) for s in S)
    h.yrun(1.2, cmd=cmd, target=tgt_ok)
    n = sum(len(h.ysp[s]) for s in S) - n0
    nd = sum(len(h.ydbg[s]) for s in S) - d0
    check('Yh takeover (fresh Offboard-off report): the servo bridges stop as T5, SUPER and target '
          'live; no debug', n == 0 and nd == 0, f'{n} setpoints, {nd} debug in 1.2 s')
    H2 = -2.6                                 # landed, carried, turned: 124 deg from the nose
    h.set_state(False, False)
    h.set_lpos(0.0, 0.0, -1.5, H2)
    h.yrun(0.6, target=tgt_ok)                # SUPER silent from here on
    h.set_state(True, False)                  # armed in Position ...
    h.yrun(0.4, target=tgt_ok)
    n0 = len(h.ysp['_servo'])
    h.set_state(True, True)                   # ... then ch11 (the field order): HOLD
    h.yrun(0.4, target=tgt_ok)
    first = h.ysp['_servo'][n0] if len(h.ysp['_servo']) > n0 else None
    off_h = abs(deg(wrap(first.yaw - f32(H2)))) if first else math.inf
    off_n = abs(deg(wrap(first.yaw - exp_j))) if first else 0.0
    check('Yh re-engage, arm then Offboard: the first servo setpoint starts from the vehicle '
          'heading '
          '(within one tick at the 0.1 s dt cap, 9 deg), not from the old nose',
          off_h <= 9.0 and off_n > 90.0,
          f'{off_h:.3f} deg from the heading, {off_n:.1f} deg from where the servo left the nose')
    h.yrun(1.2, target=tgt_ok)                # HOLD aims: nose back to -25 deg
    H3 = 2.5                                  # landed and turned again: 168 deg from the nose
    h.set_state(False, True)                  # disarmed in Offboard: IDLE, the latch is sent
    h.set_lpos(0.0, 0.0, -1.5, H3)
    t_idle = time.time()
    h.yrun(0.6, target=tgt_ok)
    latch = latch_yaw(H3)
    idle = h.seg('_servo', t_idle + 0.3)
    n0 = len(h.ysp['_servo'])
    h.set_state(True, True)                   # armed in Offboard
    h.yrun(0.5, target=tgt_ok)
    w = h.ysp['_servo'][n0 - 1:]
    i = next((j for j, x in enumerate(w) if x.yaw != latch), None)
    ok, det = False, 'no engaged setpoint'
    if i:
        dt = (w[i].ts - w[i - 1].ts) / 1e6
        step = abs(wrap(w[i].yaw - f32(H3)))
        ok = (w[i - 1].yaw == latch and step <= SERVO_MAX_RATE * dt + SLEW_TOL
              and abs(deg(wrap(w[i].yaw - exp_j))) > 90.0)
        det = (f'latch {deg(latch):+.3f} deg; first engaged setpoint {deg(step):.3f} deg from the '
               f'heading in {1e3 * dt:.1f} ms (limit {deg(SERVO_MAX_RATE * dt):.3f})')
    check('Yh re-engage from disarmed Offboard: IDLE sends the latch exactly, then the first '
          'engaged '
          'setpoint is one slew step from the heading',
          ok and len(idle) > 5 and all(x.yaw == latch for x in idle), det)

    alld = [d for s in S for _, d in h.ydbg[s]]
    check('Y  debug: every message has the 7 fields of DEBUG_LAYOUT',
          len(alld) > 1000 and all(len(d) == 7 for d in alld), f'{len(alld)} messages')
    return True


def _servo_checks_r2(h):
    """Y2 (round two): _servo and _ff (the new defaults) with _servo_r1 and _ff_r1 (stale hold
    0, hysteresis 0). The round-one assertions of Yi / Ye / Yj on the round-one bridges; then
    Yl (the fall-back yawspeed), Ym (hysteresis), Yn (the stale hold) side by side."""
    NEW, OLD = ['_servo', '_ff'], list(SERVO_R1_BRIDGES)
    S4 = NEW + OLD
    # The same SUPER command as _servo_checks.
    YS, YD = -0.35, 0.1
    PSI_S, YSPD_S = f32(wrap(-YS)), f32(-YD)
    P, V = (0.3, -0.4, 1.5), (0.1, 0.2, -0.05)
    A, J = (0.3, -0.2, 0.1), (0.05, -0.07, 0.02)

    def cmd():
        h.send_cmd(P, V, YS, YD, acc=A, jerk=J)

    def latch_yaw(heading):
        return f32(wrap(f32(heading)))

    print('\nY2  round two: the default bridge and _nohold stop; _servo_r1 and _ff_r1 (stale_hold_s 0, '
          'hysteresis 0 = round one) join _servo and _ff')
    h.watch_servo(OLD)

    def up():
        return (all(len(h.ysp[s]) > 10 for s in OLD) and h.count_subscribers(YT_TOPIC) == 4
                and all(h.count_publishers(f'{NS}/yaw_servo{s}') == 1 for s in OLD))
    end = time.time() + 20.0
    while time.time() < end and not up():
        h.yrun(0.5)
    ok = up()
    check('Y2 setup: 2 round-one bridges streaming; 4 servo bridges on the yaw target',
          ok, f'setpoints {[len(h.ysp[s]) for s in OLD]}, '
          f'{h.count_subscribers(YT_TOPIC)} subscribers on {YT_TOPIC}')
    if not ok:
        return False

    # ---- Yi, round one: the engage HOLD, the target goes stale -> the FROZEN yaw at once ---
    print('\nYi  [stale_hold_s 0, round one] engage HOLD with a fresh target, then the target stale')
    E, E2 = (1.0, -2.0, -1.5, -0.4), (1.3, -2.2, -1.6, -0.35)
    tx, ty = 1.3 + 5.0 * math.cos(math.radians(100)), 2.2 + 5.0 * math.sin(math.radians(100))
    tgt_i = static_target(h, tx, ty)
    h.set_state(False, True)                  # disarmed in Offboard: IDLE
    h.set_lpos(*E)
    h.yrun(0.6, target=tgt_i)
    h.set_state(True, True)                   # the engage, SUPER silent: HOLD
    h.yrun(0.3, target=tgt_i)
    h.set_lpos(*E2)
    h.yrun(2.2, target=tgt_i)
    exp = frd_bearing(tx, ty, E2[0], E2[1])
    frozen = latch_yaw(E[3])
    errs = [worst_err(h.seg(s, time.time() - 0.4), exp) for s in OLD]
    t_st = time.time()
    h.yrun(2.0)
    res = []
    for s in OLD:
        worst, nbad = slew(h.seg(s, t_st - 0.1), 'yaw', SERVO_MAX_RATE)
        tail = h.seg(s, t_st + 1.6)
        res.append((nbad, worst, tail, all(x.yaw == frozen and x.yawspeed == 0.0 for x in tail)))
    check('Yi [stale_hold_s 0, round one] HOLD, target gone stale: back to the FROZEN yaw exactly, '
          'through the 90 deg/s slew (_servo_r1, _ff_r1; they aimed first)',
          max(errs) <= 0.5 and all(nbad == 0 and tail and same for nbad, _, tail, same in res),
          f'aim errors {errs[0]:.3f} / {errs[1]:.3f} deg; yaw '
          f'{deg(res[0][2][-1].yaw) if res[0][2] else math.nan:+.3f} deg, frozen '
          f'{deg(frozen):+.3f}; worst {deg(max(r[1] for r in res)):.1f} deg/s, '
          f'{sum(r[0] for r in res)} pairs over')

    # ---- Ye, round one ---------------------------------------------------------------------
    print("\nYe  [stale_hold_s 0, round one] stale and silent targets: SUPER's yaw at once")
    h.set_lpos(0.0, 0.0, -1.5, 0.0)
    tex, tey = 5.0 * math.cos(math.radians(50)), 5.0 * math.sin(math.radians(50))
    h.yrun(2.0, cmd=cmd, target=static_target(h, tex, tey))
    t0 = time.time()
    h.yrun(1.5, cmd=cmd, target=static_target(h, tex, tey, age=0.8))
    res = []
    for s in OLD:
        _, nb = slew(h.seg(s, t0 - 0.1), 'yaw', SERVO_MAX_RATE)
        tail = h.seg(s, t0 + 1.1)
        res.append(nb == 0 and len(tail) > 10
                   and all(x.yaw == PSI_S and x.yawspeed == YSPD_S for x in tail))
    d = h.dseg('_servo_r1', t0 + 0.2)
    check("Ye [stale_hold_s 0, round one] stale stamps (0.8 s old, still arriving): SUPER's yaw and "
          'yawspeed exactly, no jump (_servo_r1, _ff_r1); debug active 0, source 0, age > 0.5 s',
          all(res) and d and all(x[0] == 0.0 and x[6] == 0.0 and x[5] > 0.79 for x in d),
          f'{res}; ages {min(x[5] for x in d):.3f}-{max(x[5] for x in d):.3f} s' if d else f'{res}')
    h.yrun(1.5, cmd=cmd, target=static_target(h, tex, tey))
    t0 = time.time()
    h.yrun(1.8, cmd=cmd)
    d = h.dseg('_servo_r1', t0 - 0.2)
    act = [x[5] for x in d if x[0] == 1.0]
    fb = [x[5] for x in d if x[0] == 0.0]
    _, nb = slew(h.seg('_servo_r1', t0 - 0.1), 'yaw', SERVO_MAX_RATE)
    tail = h.seg('_servo_r1', t0 + 1.5)
    check("Ye [stale_hold_s 0, round one] target silent: aims until its age passes 0.5 s, then "
          "SUPER's yaw exactly, slew-limited (_servo_r1)",
          act and fb and max(act) <= 0.5 + 1e-6 and 0.5 <= fb[0] <= 0.55 and nb == 0
          and not any(x[6] == 3.0 for x in d) and tail and all(x.yaw == PSI_S for x in tail),
          f'last active age {max(act) if act else math.nan:.3f} s, first fall-back age '
          f'{fb[0] if fb else math.nan:.3f} s, {nb} pairs over the slew')

    # ---- Yj, round one ---------------------------------------------------------------------
    print("\nYj  [stale_hold_s 0, round one] wrong frame_id, NaN target, lpos invalid")
    tjx, tjy = 5.0 * math.cos(math.radians(25)), 5.0 * math.sin(math.radians(25))
    exp_j = frd_bearing(tjx, tjy, 0.0, 0.0)
    tgt_ok = static_target(h, tjx, tjy)
    cases = (('wrong frame_id', static_target(h, tjx, tjy, frame='camera_color_optical_frame'),
              False),
             ('NaN target', static_target(h, float('nan'), tjy), False),
             ('lpos xy_valid false', tgt_ok, True))
    for name, stim, lpos_bad in cases:
        t0 = time.time()
        h.yrun(1.3, cmd=cmd, target=tgt_ok)
        aim = worst_err(h.seg('_servo_r1', t0 + 0.9), exp_j)
        if lpos_bad:
            h.set_lpos_valid(False)
        t1 = time.time()
        h.yrun(1.3, cmd=cmd, target=stim)
        h.set_lpos_valid(True)
        _, nb = slew(h.seg('_servo_r1', t1 - 0.1), 'yaw', SERVO_MAX_RATE)
        tail = h.seg('_servo_r1', t1 + 0.9)
        d = h.dseg('_servo_r1', t1 + 0.9)
        check(f"Yj [stale_hold_s 0, round one] {name}: back to SUPER's yaw exactly, no jump (was "
              'aiming at -25 deg)',
              aim <= 0.5 and nb == 0 and len(tail) > 10
              and all(x.yaw == PSI_S and x.yawspeed == YSPD_S for x in tail)
              and d and all(x[0] == 0.0 and x[6] == 0.0 for x in d),
              f'aim error before {aim:.3f} deg; {nb} pairs over the slew; end yaw '
              f'{deg(tail[-1].yaw) if tail else math.nan:+.3f} deg (SUPER {deg(PSI_S):+.3f})')

    # ---- Yi after TRACK, round one -----------------------------------------------------------
    h.yrun(0.8, cmd=cmd, target=tgt_ok)
    h.yrun(0.8, target=tgt_ok)
    t_s = time.time()
    h.yrun(1.8)
    aim = worst_err(h.seg('_servo_r1', t_s - 0.3, t_s), exp_j)
    tail = h.seg('_servo_r1', t_s + 1.4)
    p_last = (f32(P[0]), f32(-P[1]), f32(-P[2]))
    check("Yi [stale_hold_s 0, round one] HOLD after TRACK, then the target goes stale: back to "
          "SUPER's LAST yaw exactly, position held at its last point (_servo_r1)",
          aim <= 0.5 and len(tail) > 10
          and all(x.yaw == PSI_S and x.yawspeed == 0.0 and x.pos == p_last and all_nan(x.vel)
                  for x in tail),
          f'aim error in HOLD {aim:.3f} deg; end yaw {deg(tail[-1].yaw) if tail else math.nan:+.3f}'
          f' deg (SUPER {deg(PSI_S):+.3f})')

    # ---- Yl: the fall-back yawspeed (F-a), all four bridges -----------------------------------
    print("\nYl  fall-back yawspeed: SUPER's yaw ramping at 100 deg/s with yaw_dot 100, target "
          'silent; then at 60 deg/s with yaw_dot 100')
    h.yrun(1.2, cmd=cmd)                      # TRACK, target silent: all four on SUPER's yaw
    ramp_state = {}

    def ramp(rate_deg, ydot_deg, t_start, y0):
        def c():
            y = y0 + math.radians(rate_deg) * (time.time() - t_start)
            ramp_state['y'] = y
            h.send_cmd(P, V, y, math.radians(ydot_deg), acc=A, jerk=J)
        return c
    t_r = time.time()
    h.yrun(1.2, cmd=ramp(100.0, 100.0, t_r, YS))
    t_r2 = time.time()
    h.yrun(1.4, cmd=ramp(60.0, 100.0, t_r2, ramp_state['y']))
    t_r3 = time.time()
    y_end = ramp_state['y']
    h.yrun(0.4, cmd=lambda: h.send_cmd(P, V, y_end, 0.0, acc=A, jerk=J))
    cap = f32(-math.radians(90.0))            # FLU yaw_dot +100 -> FRD -100 -> clamped -90
    rows, ok1, ok2 = [], True, True
    for s in S4:
        d1 = h.dseg(s, t_r + 0.1, t_r2)
        bind = [x for x in d1 if x[6] == 0.0 and abs(wrap(x[1] - x[2])) > 1e-5]
        nz = sum(1 for x in bind if x[3] != 0.0)
        sp = h.seg(s, t_r, t_r3 + 0.4)
        mx = max((abs(x.yawspeed) for x in sp), default=math.inf)
        d2 = h.dseg(s, t_r2 + 0.7, t_r3)
        free = [x for x in d2 if x[6] == 0.0 and abs(wrap(x[1] - x[2])) <= 1e-5]
        at_cap = sum(1 for x in free if x[3] == cap)
        ok1 &= len(bind) >= 40 and nz == 0 and all(x[6] == 0.0 for x in d1)
        ok2 &= mx <= SERVO_MAX_RATE + SLEW_TOL and len(free) >= 20 and at_cap == len(free)
        rows.append(f'{s}: {len(bind)} binding ticks, {nz} with yawspeed != 0; max |yawspeed| '
                    f'{deg(mx):.3f} deg/s, {at_cap}/{len(free)} free ticks at {deg(cap):+.1f}')
    check("Yl SUPER's yaw ramping at 100 deg/s with yaw_dot 100 deg/s, target unusable: yawspeed "
          'exactly 0 on every tick the slew binds (4 bridges; round one sent up to 100 deg/s)',
          ok1, '; '.join(r.split(';')[0] for r in rows))
    check('Yl ... never above 90 deg/s on any setpoint; at 60 deg/s with yaw_dot 100 (clamp not '
          'clipping, in the band): exactly -90 deg/s, SUPER\'s rate clamped to max_rate', ok2,
          '; '.join(f'{r.split(":")[0]}:{r.split(";")[1]}' for r in rows))

    # ---- Ym: close-range hysteresis ------------------------------------------------------------
    print('\nYm  close-range hysteresis: the target at 2.4 m, then 2.6 m, then 2.9 m')
    h.set_lpos(0.0, 0.0, -1.5, 0.0)

    def at(r_m, brg):     # a target r_m from the origin at FRD bearing brg (deg)
        th = math.radians(-brg)
        return static_target(h, r_m * math.cos(th), r_m * math.sin(th))
    h.yrun(2.5, cmd=cmd, target=at(5.0, 30.0))   # up to ~150 deg from Yl's end: 1.7 s
    t1 = time.time()
    h.yrun(1.0, cmd=cmd, target=at(2.4, 30.0))
    t2 = time.time()
    h.yrun(1.2, cmd=cmd, target=at(2.6, 40.0))
    t3 = time.time()
    h.yrun(1.5, cmd=cmd, target=at(2.9, 50.0))
    b30, b40, b50 = (math.radians(b) for b in (30.0, 40.0, 50.0))
    ok_a = all(h.dseg(s, t1 + 0.2, t2) and all(x[6] == 2.0 for x in h.dseg(s, t1 + 0.2, t2))
               and worst_err(h.seg(s, t1 + 0.2, t2), b30) <= 0.01 for s in S4)
    check('Ym 2.4 m (entered from 5 m at 30 deg): all 4 bridges in the close-range hold (source 2), '
          'heading held at 30 deg', ok_a,
          ', '.join(f'{s} {sorted({x[6] for x in h.dseg(s, t1 + 0.2, t2)})}' for s in S4))
    new_src = {s: sorted({x[6] for x in h.dseg(s, t2 + 0.2, t3)}) for s in NEW}
    new_err = max(worst_err(h.seg(s, t2 + 0.2, t3), b30) for s in NEW)
    old_src = {s: sorted({x[6] for x in h.dseg(s, t2 + 0.2, t3)}) for s in OLD}
    old_err = max(worst_err(h.seg(s, t3 - 0.4, t3), b40) for s in OLD)
    check('Ym 2.6 m, bearing now 40 deg: _servo/_ff STAY in source 2 with the heading at 30 deg '
          '(hysteresis 0.3 m: out only at >= 2.8 m); the round-one bridges leave (source 1) and aim '
          'at 40', all(v == [2.0] for v in new_src.values()) and new_err <= 0.01
          and all(v == [1.0] for v in old_src.values()) and old_err <= 0.5,
          f'new {new_src}, off 30 deg by {new_err:.4f}; round one {old_src}, off 40 deg by '
          f'{old_err:.3f}')
    src29 = {s: sorted({x[6] for x in h.dseg(s, t3 + 0.2)}) for s in S4}
    err29 = max(worst_err(h.seg(s, t3 + 1.0), b50) for s in S4)
    check('Ym 2.9 m, bearing 50 deg: all 4 leave the hold (source 1) and aim at 50 deg',
          all(v == [1.0] for v in src29.values()) and err29 <= 0.5,
          f'{src29}; worst error {err29:.3f} deg')

    # ---- Yn: the stale hold on the real node -----------------------------------------------------
    print('\nYn  stale hold: the target stops; then it returns during the hold')
    tnx, tny = 5.0 * math.cos(math.radians(60)), 5.0 * math.sin(math.radians(60))   # FRD -60
    exp_n = frd_bearing(tnx, tny, 0.0, 0.0)
    h.yrun(2.0, cmd=cmd, target=static_target(h, tnx, tny))
    t0 = time.time()
    h.yrun(2.6, cmd=cmd)                      # the target stops
    rows, ok_new = [], True
    for s in NEW:
        hok, htxt, info = hold_of(h.dsegt(s, t0 - 0.3), same_stamp=True)
        _, nb = slew(h.seg(s, t0 - 0.1), 'yaw', SERVO_MAX_RATE)
        tail = h.seg(s, t0 + 2.2)
        good = (hok and 0.5 <= info.get('age3', 0.0) <= 0.55
                and abs(deg(wrap(info.get('held', math.nan) - exp_n))) <= 0.01 and nb == 0
                and len(tail) > 10 and all(x.yaw == PSI_S and x.yawspeed == YSPD_S for x in tail))
        ok_new &= good
        rows.append(f'{s}: {htxt} (hold from age {info.get("age3", math.nan):.3f} s), {nb} pairs over')
    check("Yn target stops [default stale hold 0.5 s]: heading held at the last aim (-60 deg) with "
          "yawspeed exactly 0 from age 0.5 s until 0.5 s later, then SUPER's yaw and yawspeed "
          'exactly, through the slew (_servo, _ff)', ok_new, '; '.join(rows))
    rows, ok_old = [], True
    for s in OLD:
        d = h.dseg(s, t0 - 0.3)
        fb = [x[5] for x in d if x[6] == 0.0]
        tail = h.seg(s, t0 + 2.2)
        good = (not any(x[6] == 3.0 for x in d) and fb and 0.5 <= fb[0] <= 0.55 and tail
                and all(x.yaw == PSI_S for x in tail))
        ok_old &= bool(good)
        rows.append(f'{s}: first fall-back age {fb[0] if fb else math.nan:.3f} s, '
                    f'{sum(1 for x in d if x[6] == 3.0)} hold ticks')
    check('Yn ... control: the round-one bridges (stale_hold_s 0) fall back at age 0.5 s, no hold',
          ok_old, '; '.join(rows))
    t0 = time.time()
    orb = orbit(h, 5.0, 90.0, 30.0, t0)       # to the LEFT at 30 deg/s: ff -30 deg/s
    h.yrun(3.0, cmd=cmd, target=orb)
    t_gap = time.time()
    h.set_lpos_valid(False)
    h.yrun(0.3, cmd=cmd, target=orb)
    h.set_lpos_valid(True)
    t_back = time.time()
    h.yrun(2.5, cmd=cmd, target=orb)
    t_end = time.time()
    rows, ok_ret = [], True
    for s in NEW:
        gap = h.dseg(s, t_gap + 0.06, t_back - 0.02)
        back = h.dseg(s, t_back + 0.08, t_back + 0.5)
        w = h.seg(s, t0 + 1.5)
        _, nby = slew(w, 'yaw', SERVO_MAX_RATE)
        _, nbs = slew(w, 'yawspeed', SERVO_FF_ACC)
        pre = max((abs(x.yawspeed) for x in h.seg(s, t0 + 1.5, t_gap)), default=math.inf)
        post = max((abs(x.yawspeed) for x in h.seg(s, t_back)), default=math.inf)
        # post vs pre: the settled ff ripples by ~+-2 deg/s at 15 Hz on the real node (Yg), so
        # 3 deg/s of allowance; a first step taken as a rate would add >= 18 deg/s (ff slew x
        # window). The exact "never above" is unit-checked (Q4).
        good = (gap and all(x[6] == 3.0 for x in gap) and back and all(x[6] == 1.0 for x in back)
                and nby == 0 and nbs == 0 and post <= pre + math.radians(3.0))
        if s == '_ff':
            v = [deg(x.yawspeed) for x in h.seg(s, t_back + 1.8)]
            good = good and v and all(-35.0 <= x <= -25.0 for x in v) and pre > math.radians(25)
            rows.append(f'{s}: |yawspeed| max {deg(pre):.2f} before, {deg(post):.2f} after the '
                        f'gap; re-settled {min(v) if v else math.nan:+.1f}..'
                        f'{max(v) if v else math.nan:+.1f} deg/s; {nbs} ff-slew / {nby} yaw pairs over')
        else:
            good = good and all(x[3] == 0.0 for x in gap)
            rows.append(f'{s}: {len(gap)} hold ticks, yawspeed 0; {nby} yaw pairs over')
        ok_ret &= bool(good)
    old_gap = {s: sorted({x[6] for x in h.dseg(s, t_gap + 0.06, t_back - 0.02)}) for s in OLD}
    check('Yn target back during the hold (lpos invalid 0.3 s, 30 deg/s orbit, ff -30): source 3 '
          'through the gap, aims again after it; no yawspeed step over the ff slew and |yawspeed| '
          'not above its pre-gap value + 3 deg/s of ripple (the first psi_des step after the gap is '
          'not a rate); the round-one bridges fell back (source 0) instead',
          ok_ret and all(v == [0.0] for v in old_gap.values()),
          '; '.join(rows) + f'; round one in the gap: {old_gap}')

    alld = [d for s in S4 for _, d in h.ydbg[s]]
    check('Y2 debug: every message has the 7 fields of DEBUG_LAYOUT; source 3 seen on _servo/_ff '
          'only', len(alld) > 1000 and all(len(d) == 7 for d in alld)
          and any(d[6] == 3.0 for s in NEW for _, d in h.ydbg[s])
          and not any(d[6] == 3.0 for s in OLD for _, d in h.ydbg[s]), f'{len(alld)} messages')
    return True


def _servo_log_checks(logdir):
    def read(s):
        try:
            with open(os.path.join(logdir, f'bridge{s}.log'), errors='replace') as f:
                return f.read()
        except OSError:
            return ''
    t, tf = read('_servo'), read('_ff')
    t1 = read('_servo_r1')
    check('Y  log: the startup line has the switches and limits, and a WARN that the servo is ON',
          'yaw_servo=ON ff=off' in t and 'max_rate=90deg/s' in t and 'YAW SERVO ON' in t
          and 'yaw_servo=ON ff=on' in tf, f'{len(t)} bytes of log')
    check('Y  log: the startup line has the round-two parameters (hysteresis=0.30m stale_hold=0.50s '
          'by default; 0.00/0.00 on the round-one bridges)',
          'hysteresis=0.30m stale_hold=0.50s' in t and 'hysteresis=0.30m stale_hold=0.50s' in tf
          and 'hysteresis=0.00m stale_hold=0.00s' in t1,
          next((ln.split('min_range=', 1)[-1].split(' camera')[0] for ln in t.splitlines()
                if 'min_range=' in ln), 'no startup line'))
    n_h, n_h1 = t.count('after holding the last aim point'), t1.count('after holding the last aim point')
    check('Y  log: an INACTIVE edge after a stale hold says how long it held (none on the round-one '
          'bridge)', n_h >= 3 and n_h1 == 0 and 'INACTIVE' in t1,
          next((ln.split('] ', 1)[-1] for ln in t.splitlines()
                if 'after holding the last aim point' in ln), 'none') + f' ({n_h} such lines)')
    holds = [ln for ln in t.splitlines() if 'stale-hold ' in ln and 'stale holds ' in ln]
    busy = [ln for ln in holds if 'stale holds 0 ' not in ln]
    check('Y  log: the 5 s line counts stale-hold ticks and the holds in its window',
          holds and busy, (busy[0].split(' | ', 1)[-1] if busy else f'{len(holds)} lines'))
    nw = t.count("frame_id 'camera_color_optical_frame' is not one of")
    check('Y  log: a bad frame_id is warned once, not per message', nw == 1, f'{nw} warnings')
    n_on, n_off = t.count('yaw servo ACTIVE'), t.count('yaw servo INACTIVE')
    reasons = [r for r in ('stale target', "frame_id 'camera_color_optical_frame'",
                           'non-finite target', 'lpos invalid', 'not engaged') if r not in t]
    check('Y  log: active/inactive edges logged once each, with the reason',
          3 <= n_on <= 40 and abs(n_on - n_off) <= 1 and not reasons,
          f'{n_on} ACTIVE / {n_off} INACTIVE lines; reasons missing: {reasons}')
    check("Y  log: the 5 s line reports the servo's state and its worst slew",
          'yaw servo on(' in t and 'longest bind' in t,
          next((ln.split(' | ', 1)[-1] for ln in t.splitlines() if 'longest bind' in ln), ''))


def main():
    if not os.path.exists(BRIDGE):
        sys.exit(f'bridge not found: {BRIDGE}')

    # publish_acceleration_ff is deliberately NOT passed to the main bridge: every launch
    # path (run.sh, the GCS, deploy_follow.sh, bench_bringup.sh) runs it with no parameters,
    # so T2 is asserting the default they get. The second bridge pins it off.
    proc = start_offboard_bridge()
    proc_noacc = start_offboard_bridge('_noacc', ['publish_acceleration_ff:=false'])
    # The old always-on IDLE stream, kept behind a parameter for the bench (2026-09-28).
    proc_legacy = start_offboard_bridge('_legacy', ['setpoint_only_in_offboard:=false'])

    rclpy.init()
    h = Harness()
    try:
        print(f'\nm3_bridge_check -- sandboxing {os.path.relpath(BRIDGE, REPO)}\n')

        # ---- T1: IDLE latches onto the live estimate; sent only in Offboard ---
        # Disarmed Offboard first: it proves the setpoint link is matched, so the "nothing
        # published" checks below cannot pass merely because DDS has not connected yet.
        print('T1  IDLE latches onto the estimate; setpoints only while PX4 says Offboard')
        E1, E2 = (1.5, -2.5, -3.5, 0.4), (-0.7, 1.9, -1.1, -2.2)
        h.set_state(False, True)
        h.set_lpos(*E1)
        end = time.time() + 6.0
        while time.time() < end and not (h.n_sp > 10 and h.n_sp_legacy > 10):
            h.run(0.5)
        sp = h.sp
        check('T1 disarmed Offboard: bridge is publishing setpoints',
              sp is not None and h.n_sp > 10, f'n={h.n_sp}')
        if sp is None:
            raise SystemExit(1)
        check('T1 disarmed Offboard: setpoint == current estimate',
              close(sp.position[0], E1[0]) and close(sp.position[1], E1[1])
              and close(sp.position[2], E1[2]) and close(sp.yaw, E1[3]),
              f'{rounded(sp.position)} yaw {sp.yaw:.3f}')

        h.set_state(False, False)
        h.run(0.3)                        # let the last Offboard samples drain
        n0, n0_ocm, n0_leg = h.n_sp, h.n_ocm, h.n_sp_legacy
        t0 = time.time()
        h.run(1.5)
        hz_ocm = (h.n_ocm - n0_ocm) / (time.time() - t0)
        check('T1 not in Offboard (disarmed): no trajectory_setpoint at all',
              h.n_sp == n0, f'{h.n_sp - n0} sent in 1.5 s')
        check('T1 heartbeat still at rate while setpoints are withheld',
              abs(hz_ocm - RATE) < 10.0, f'{hz_ocm:.1f} Hz')
        sl = h.sp_legacy
        check('T1 setpoint_only_in_offboard:=false: the old always-on IDLE stream (== estimate)',
              h.n_sp_legacy - n0_leg > 30 and sl is not None and close(sl.position[0], E1[0])
              and close(sl.position[2], E1[2]),
              f'{h.n_sp_legacy - n0_leg} sent, pos {rounded(sl.position) if sl else None}')

        h.set_state(True, False)          # armed in Position/Altitude: PX4's task owns it
        n0 = h.n_sp
        h.run(1.0)
        check('T1 armed, not in Offboard (Position/Altitude): still no trajectory_setpoint',
              h.n_sp == n0, f'{h.n_sp - n0} sent in 1.0 s')

        # The in-air engage: armed, Offboard switch on, SUPER silent (the engage gate).
        h.mark()
        h.set_state(True, True)
        h.run(0.4)
        h.set_lpos(*E2)                   # drifts after the switch: HOLD must not chase
        n0 = h.n_sp
        h.run(0.8)
        f = h.first_sp
        check('T1 armed engage: the first setpoint is the estimate at the switch',
              f is not None and close(f.position[0], E1[0]) and close(f.position[1], E1[1])
              and close(f.position[2], E1[2]) and close(f.yaw, E1[3]),
              f'{rounded(f.position)} yaw {f.yaw:.3f}' if f else 'no setpoint after the switch')
        check('T1 armed engage: first setpoint within 0.2 s of the Offboard report',
              h.first_sp_dt is not None and h.first_sp_dt < 0.2,
              f'{1e3 * h.first_sp_dt:.0f} ms' if h.first_sp_dt is not None else 'none')
        sp = h.sp
        check('T1 armed engage: HOLD at that point while the estimate moves, at rate',
              h.n_sp - n0 > 30 and close(sp.position[0], E1[0]) and close(sp.position[2], E1[2]),
              f'{h.n_sp - n0} in 0.8 s, pos {rounded(sp.position)}')

        # The other order, engage then arm: the first sample must be where it is NOW.
        h.set_state(False, False)
        h.run(0.3)
        h.mark()
        h.set_state(False, True)
        h.run(0.6)
        f = h.first_sp
        check('T1 disarmed engage: the first setpoint is the current estimate',
              f is not None and close(f.position[0], E2[0]) and close(f.position[1], E2[1])
              and close(f.position[2], E2[2]) and close(f.yaw, E2[3]),
              f'{rounded(f.position)} yaw {f.yaw:.3f}' if f else 'no setpoint after the switch')

        # ---- T2: engaged -> the frame conversion itself -----------------------
        print('\nT2  TRACK conversion  (x,y,z)->(x,-y,-z), psi->-psi')
        h.set_state(True, True)
        P, V, YAW, YAWD = (1.0, 2.0, 3.0), (0.5, -0.25, 0.75), 0.5, -0.2
        A = (0.4, -0.6, 0.8)   # a different magnitude per axis, so a swap shows
        h.run(1.5, cmd=lambda: h.send_cmd(P, V, YAW, YAWD, acc=A))
        sp = h.sp
        check('T2 position  (1,2,3) -> (1,-2,-3)',
              close(sp.position[0], 1.0) and close(sp.position[1], -2.0)
              and close(sp.position[2], -3.0),
              f'{[round(v, 3) for v in sp.position]}')
        check('T2 velocity  (0.5,-0.25,0.75) -> (0.5,0.25,-0.75)',
              close(sp.velocity[0], 0.5) and close(sp.velocity[1], 0.25)
              and close(sp.velocity[2], -0.75),
              f'{[round(v, 3) for v in sp.velocity]}')
        check('T2 acceleration (0.4,-0.6,0.8) -> (0.4,0.6,-0.8), with the bridge DEFAULTS',
              close(sp.acceleration[0], 0.4) and close(sp.acceleration[1], 0.6)
              and close(sp.acceleration[2], -0.8),
              f'{rounded(sp.acceleration)}')
        sn = h.sp_noacc
        check('T2 publish_acceleration_ff:=false: same TRACK setpoint, acceleration NaN',
              sn is not None and close(sn.position[1], -2.0) and close(sn.velocity[2], -0.75)
              and all_nan(sn.acceleration),
              f'pos {rounded(sn.position)} acc {rounded(sn.acceleration)}' if sn
              else 'no output from the second bridge')
        check('T2 yaw       psi=0.5 -> -0.5', close(sp.yaw, -0.5), f'{sp.yaw:.4f}')
        check('T2 yaw is NOT the pi/2-psi trap',
              not close(sp.yaw, math.pi / 2 - YAW, 1e-3),
              f'pi/2-psi would be {math.pi / 2 - YAW:+.4f}')
        check('T2 yawspeed  -0.2 -> +0.2', close(sp.yawspeed, 0.2), f'{sp.yawspeed:.4f}')

        # ---- T3: yaw stays inside PX4's declared -PI..+PI ---------------------
        print('\nT3  yaw wrapping')
        h.run(1.0, cmd=lambda: h.send_cmd(P, V, 3.2, 0.0))
        sp = h.sp
        check('T3 psi=3.2 -> wrapped into [-pi,pi]',
              -math.pi <= sp.yaw <= math.pi and close(sp.yaw, math.atan2(
                  math.sin(-3.2), math.cos(-3.2))),
              f'{sp.yaw:+.4f}')

        # ---- T4: SUPER goes quiet mid-flight -> freeze, do not drift ----------
        print('\nT4  HOLD on command timeout (still armed + offboard)')
        h.run(1.0, cmd=lambda: h.send_cmd(P, V, YAW, YAWD, acc=A))
        h.set_lpos(9.0, 9.0, -9.0, 1.2)   # estimate moves away; setpoint must not follow
        n0 = h.n_sp
        h.run(1.5)                        # no commands at all
        sp = h.sp
        check('T4 setpoint frozen at the last COMMANDED point',
              close(sp.position[0], 1.0) and close(sp.position[1], -2.0)
              and close(sp.position[2], -3.0),
              f'{[round(v, 3) for v in sp.position]}')
        check('T4 feedforward dropped (velocity NaN)',
              all(math.isnan(v) for v in sp.velocity) and close(sp.yawspeed, 0.0))
        check('T4 acceleration NaN in HOLD (the last command carried one)',
              all_nan(sp.acceleration), f'{rounded(sp.acceleration)}')
        check('T4 setpoint stream continues through the gap', h.n_sp - n0 > 30,
              f'{h.n_sp - n0} setpoints in 1.5 s')
        # The FC->Jetson link drops the mode report while PX4 is still in Offboard. The
        # bridge must keep feeding PX4: only a report that SAYS Offboard is off stops it.
        h.vcm_silent = True
        n0 = h.n_sp
        h.run(2.5)                        # past the bridge's 2 s stale-report warning
        h.vcm_silent = False
        sp = h.sp
        check('T4 mode report silent 2.5 s in Offboard: setpoints keep coming, still HOLD',
              h.n_sp - n0 > 90 and close(sp.position[0], 1.0) and close(sp.position[2], -3.0),
              f'{h.n_sp - n0} in 2.5 s, pos {rounded(sp.position)}')

        # ---- T5: takeover stops the stream; re-engage re-latches --------------
        print('\nT5  takeover stops the stream; re-engaging never commands the stale point')
        # The pilot flips Offboard off, still armed: a FRESH report says Offboard is off.
        h.set_state(True, False)
        h.run(0.3)
        n0, n0_leg = h.n_sp, h.n_sp_legacy
        h.run(1.2)
        check('T5 takeover (fresh Offboard-off report): the setpoint stream stops',
              h.n_sp == n0, f'{h.n_sp - n0} sent in 1.2 s after the report')
        check('T5 setpoint_only_in_offboard:=false: still streams after the takeover (old)',
              h.n_sp_legacy - n0_leg > 30, f'{h.n_sp_legacy - n0_leg} in 1.2 s')
        # A pilot takeover mid-trajectory: SUPER keeps commanding, with an acceleration.
        n0 = h.n_sp
        h.run(1.0, cmd=lambda: h.send_cmd(P, V, YAW, YAWD, acc=A))
        check('T5 SUPER still commanding after the takeover: still nothing sent',
              h.n_sp == n0, f'{h.n_sp - n0} sent in 1.0 s')
        # Landed, disarmed, carried elsewhere (the estimate is at 9,9,-9), then Offboard
        # engaged again, disarmed: the first sample must be there, not SUPER's old point.
        h.set_state(False, False)
        h.run(0.3)
        h.mark()
        h.set_state(False, True)
        h.run(1.0)
        f = h.first_sp
        check('T5 re-engaged: first setpoint is the estimate, not the stale command',
              f is not None and close(f.position[0], 9.0) and close(f.position[1], 9.0)
              and close(f.position[2], -9.0) and close(f.yaw, 1.2),
              f'{rounded(f.position)} yaw {f.yaw:.3f}' if f else 'no setpoint after the switch')
        # SUPER commanding with an acceleration while in disarmed Offboard (IDLE, sent).
        h.run(1.0, cmd=lambda: h.send_cmd(P, V, YAW, YAWD, acc=A))
        sp = h.sp
        check('T5 SUPER commanding in disarmed Offboard: estimate held, acceleration NaN',
              close(sp.position[0], 9.0) and close(sp.position[2], -9.0)
              and all_nan(sp.acceleration),
              f'pos {rounded(sp.position)} acc {rounded(sp.acceleration)}')
        # The same with no valid estimate: nothing to latch onto, so the bridge re-sends the
        # setpoint it has -- SUPER's latest command. That must still carry no acceleration.
        h.set_lpos_valid(False)
        h.run(1.0, cmd=lambda: h.send_cmd(P, V, YAW, YAWD, acc=A))
        sp = h.sp
        h.set_lpos_valid(True)
        check('T5 disarmed Offboard with no valid estimate: acceleration still NaN',
              all_nan(sp.acceleration),
              f'pos {rounded(sp.position)} acc {rounded(sp.acceleration)}')

        # ---- T6: the Offboard heartbeat PX4 gates the mode switch on ----------
        # Measured out of Offboard: the switch needs it BEFORE any setpoint is sent.
        print('\nT6  OffboardControlMode heartbeat')
        h.set_state(False, False)
        h.run(0.3)
        n0 = h.n_ocm
        t0 = time.time()
        h.run(2.0)
        hz = (h.n_ocm - n0) / (time.time() - t0)
        check('T6 heartbeat rate > 2 Hz (PX4 requirement)', hz > 2.0, f'{hz:.1f} Hz')
        check('T6 heartbeat near the configured rate', abs(hz - RATE) < 10.0,
              f'{hz:.1f} Hz vs {RATE:.0f} Hz configured')
        check('T6 position control level selected',
              h.ocm is not None and h.ocm.position and not h.ocm.attitude
              and not h.ocm.body_rate and not h.ocm.direct_actuator)

        # ---- T7: a diverged FAST-LIO must never reach the setpoint ------------
        print('\nT7  NaN rejection')
        h.set_state(True, True)
        # 2.0 s, not 1.0: coming out of T6 disengaged, 1.0 s to reach TRACK failed 1 run in 5
        # on 2026-09-12 (setpoint still T5's latch) -- a flaky gate gets re-run until it passes.
        h.run(2.0, cmd=lambda: h.send_cmd((4.0, 5.0, 6.0), V, 0.1, 0.0, acc=A))
        sp = h.sp
        check('T7 re-engaged TRACK forwards acceleration again (after HOLD and IDLE)',
              close(sp.position[0], 4.0) and close(sp.acceleration[0], 0.4)
              and close(sp.acceleration[1], 0.6) and close(sp.acceleration[2], -0.8),
              f'pos {rounded(sp.position)} acc {rounded(sp.acceleration)}')
        h.run(1.0, cmd=lambda: h.send_cmd(
            (float('nan'), 5.0, 6.0), V, 0.1, 0.0, acc=A))
        sp = h.sp
        check('T7 NaN command dropped, last good setpoint retained',
              close(sp.position[0], 4.0) and close(sp.position[1], -5.0)
              and close(sp.position[2], -6.0),
              f'{[round(v, 3) for v in sp.position]}')
        # Dropped commands do not refresh the timeout, so 1 s of them is a HOLD.
        check('T7 dropped commands keep no acceleration alive (HOLD, NaN)',
              all_nan(sp.acceleration), f'{rounded(sp.acceleration)}')

        # ---- Y: the camera yaw servo (2026-09-29) ---------------------------------
        # The two T-only bridges go first, so at most four run at once (the default one
        # plus three with the servo on).
        stop(proc_noacc)
        stop(proc_legacy)
        run_servo_checks(h, proc)

        h.destroy_node()
        h = None
        stop(proc)
        stop(proc_noacc)
        stop(proc_legacy)

        run_odom_checks(check)

    finally:
        if h is not None:
            h.destroy_node()
        rclpy.shutdown()
        stop(proc)
        stop(proc_noacc)
        stop(proc_legacy)

    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f'\n{len(RESULTS) - n_fail}/{len(RESULTS)} checks passed')
    if n_fail:
        print('\nFAILED:')
        for name, ok, detail in RESULTS:
            if not ok:
                print(f'  {name}  {detail}')
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
