#!/usr/bin/env python3
"""M3: SUPER trajectory commands -> PX4 Offboard setpoints.

Subscribes  /planning/pos_cmd            (mars_quadrotor_msgs/PositionCommand, SUPER fsm_node)
Publishes   /fmu/in/trajectory_setpoint  (px4_msgs/TrajectorySetpoint)
            /fmu/in/offboard_control_mode(px4_msgs/OffboardControlMode)
Subscribes  /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition) -- safety only
            /fmu/out/vehicle_control_mode      (px4_msgs/VehicleControlMode)   -- safety only
Only with yaw_servo_enable (default off; see "Camera yaw servo" below):
Subscribes  /planning/yaw_target             (geometry_msgs/PointStamped)
Publishes   /bridge/yaw_servo                (std_msgs/Float32MultiArray)   -- logging only

This node NEVER arms and NEVER changes flight mode. Engaging Offboard and arming
are deliberate operator actions -- see scripts/px4/offboard_engage.py.

Frame convention
----------------
SUPER plans in FAST-LIO's `lidar_init` world frame: gravity-aligned z-UP, x pointing
wherever the MID360 faced at init, i.e. FLU with a constant arbitrary heading offset
from True North. `px4_odom_bridge` feeds that to EKF2 as POSE_FRAME_FRD, so EKF2
*adopts that same frame, flipped*. The two share an origin AND a heading.

The conversion is therefore just the odometry bridge's flip run backwards:

    position / velocity / acceleration   (x, y, z) -> (x, -y, -z)
    yaw                                  psi       -> -psi
    yaw rate                             psi_dot   -> -psi_dot

Yaw simply negates because z flips from up to down, reversing the sense of rotation.

**NOT `pi/2 - psi`.** That is the textbook ENU->NED conversion, where the 90 deg
appears because North and East swap. SUPER's frame is not ENU and PX4's local frame
here is not North-aligned; applying it would put a constant 90 deg yaw error into
every setpoint. See docs/ImplementationDoc.md SS M3 and docs/fact-ev-frd-yaw-align.md.

This node must also NOT publish /lidar_slam/odom -- FAST-LIO owns that topic, and a
second publisher would give SUPER an interleaved mix of two estimates.

Why the command topic goes silent, and why that is not a fault
--------------------------------------------------------------
`FsmRos2::pubCmdTimerCallback` returns early unless the FSM is in FOLLOW_TRAJ or
EMER_STOP, so /planning/pos_cmd publishes *nothing at all* in INIT, WAIT_GOAL and
GENERATE_TRAJ. Every time SUPER reaches its goal it drops to WAIT_GOAL and the topic
stops. A bridge that treated silence as a fault -- or that stopped its heartbeat --
would drop Offboard on every completed trajectory. So the setpoint stream here is
driven by our own timer off the last received command, not by the input topic.

QoS
---
SUPER publishes pos_cmd BEST_EFFORT / KEEP_LAST(1) / VOLATILE (FsmRos2::init). A
default rclpy subscription is RELIABLE and would silently never match. Ours matches.

Setpoints only while PX4 is in Offboard (2026-09-28)
----------------------------------------------------
OffboardControlMode goes out on every tick, always: it is all PX4 needs to allow the
Offboard switch (offboardCheck.cpp:44-51, a recent heartbeat with a mode flag set plus
a valid local position). TrajectorySetpoint goes out only while the last
vehicle_control_mode report says flag_control_offboard_enabled. Until 2026-09-28 it went
out always, and uXRCE-DDS writes it into the same uORB topic that PX4's own Position,
Altitude and Auto-land flight tasks write, so mc_pos_control flew an interleave of the
pilot's task and our "hold here" (.claude/rules/px4-bridges-offboard.md, 2026-09-21).
Outside Offboard the setpoint is still latched to the live estimate every tick, just not
sent, so the first sample after the switch is "here".
  * The flag, not armed+offboard: Offboard can be entered disarmed (engage-then-arm).
  * Commander publishes vehicle_control_mode at 2 Hz AND immediately on a mode change
    (Commander.cpp:1931), and uXRCE-DDS limits it to 50 Hz, not 2 Hz
    (dds_topics.yaml:66). In the 2026-09-21/26 bags every Offboard edge reached the
    Jetson within the timesync noise (+-15 ms) of the switch.
  * A report that stops arriving never stops the stream: only a fresh report saying
    Offboard is off does. Stopping on silence would starve PX4 in the air.
  * setpoint_only_in_offboard:=false restores the old always-on stream (bench only).

Camera yaw servo (2026-09-29: built, bench-gated, NOT flown, default OFF)
------------------------------------------------------------------------
docs/plan-camera-yaw-servo.md. With yaw_servo_enable:=true, a stage between the state
machine and the publish rewrites ONLY yaw and yawspeed of the outgoing setpoint: the heading
is aimed from where the aircraft IS (vehicle_local_position) at /planning/yaw_target
(fix 1), and with yaw_servo_ff_enable:=true that bearing's rate goes out as yawspeed
(fix 2). The algorithm, its frames and its signs are in yaw_servo.py beside this file, the
one copy that drone-dtp/scripts/fix1_yaw_replay.py also imports. This node only feeds it
and applies what it returns.
  * Switch off (the default): no subscription, no publisher, the stage does not run, and
    the setpoint is today's, bit for bit (m3_bridge_check.py Ya).
  * The servo's yaw goes out on a COPY of the latched setpoint. self.sp is never written
    by the servo: it keeps SUPER's yaw -- in HOLD the frozen yaw, which is the servo's own
    fall-back -- and position, velocity, acceleration and jerk are the same values in
    every state.
  * It acts only engaged (armed + Offboard), in TRACK and HOLD. IDLE (disarmed Offboard)
    sends the latch as before, and outside Offboard nothing is sent (the 2026-09-28 rule).
    step() runs on every tick, so the first engaged tick starts from the vehicle's heading,
    never from where an earlier engagement left it.
  * HOLD aims too (yaw_servo_aim_in_hold, default true): at the engage, with SUPER silent
    behind the mux's engage gate, the nose turns to the person, slew-limited, while the
    position stays frozen. false: the frozen yaw until the first TRACK.
  * No usable target -- stale (age > yaw_servo_target_timeout, or stamped > 0.2 s in the
    future), in a frame other than lidar_init/world, non-finite, or lpos.xy_valid false --
    after a tick the servo owned: first a STALE HOLD (round two, 2026-09-29) of up to
    yaw_servo_stale_hold_s (0.5 s) of accumulated tick time, on the last aim point, with
    yawspeed 0 (ff off) or the feed-forward decaying to 0 (ff on), never SUPER's rate; then
    the fall-back to SUPER's yaw, through the same 90 deg/s slew. A target back within the
    hold is aimed at again at once. The target arrives ~0.25 s old (the perception stamp), so
    without the hold a ~0.25 s dip in the stream swung the nose toward SUPER's lagging heading
    and back (replay of the 2026-09-21 bag). yaw_servo_stale_hold_s:=0.0 = no hold (round one).
  * In the fall-back SUPER's yaw_dot goes out only on a tick where the slew clamp did not
    clip, while the heading is within 2 deg of SUPER's, and clamped to +-yaw_servo_max_rate:
    |yawspeed| never exceeds max(yaw_servo_ff_max_rate, yaw_servo_max_rate), whatever SUPER
    sends (round two; round one sent SUPER's 96-126 deg/s while the clamp bound).
  * Under yaw_servo_min_range_m it holds the heading with yawspeed 0, and only leaves that
    hold once the range is >= yaw_servo_min_range_m + yaw_servo_min_range_hysteresis_m
    (0.3 m; round two: a range flickering at 2.49-2.51 m flipped it 8 times in 0.42 s).
  * Target age = this tick's bridge clock - header.stamp, recomputed on every tick, not at
    reception. The stamp is the perception stamp (target_manager -> follow_planner copy it;
    the yaw hold stamps "now"), on this Jetson's ROS clock like ours; SUPER gates the same
    stamp against its own clock the same way. A bridge run with use_sim_time would see
    every target as stale. dt = the bridge clock between ticks (the first: 1/publish_rate).
  * QoS: follow_planner publishes /planning/yaw_target BEST_EFFORT / KEEP_LAST(1) /
    VOLATILE (follow_planner_node.hpp), and SUPER's fsm_node subscribes with the same.
    Ours is BEST_EFFORT / VOLATILE, which matches a publisher of either reliability.
  * Observe-only publishes /dtp/shadow/yaw_target instead, so there the servo never sees a
    target and stays in fall-back = SUPER's yaw.
  * Logging. /bridge/yaw_servo (yaw_servo_debug_topic), std_msgs/Float32MultiArray, one per
    ENGAGED tick, switch on only; its layout is yaw_servo.DEBUG_LAYOUT:
        [0] active        1.0 = the servo owns the heading (source 1, 2 or 3), else 0.0
        [1] psi_des       rad FRD, this tick's desired heading (NaN when passed through)
        [2] psi_out       rad FRD, the yaw sent
        [3] yawspeed      rad/s FRD, the yawspeed sent
        [4] range_m       m, horizontal range to the target (NaN: no usable target)
        [5] target_age_s  s, bridge clock - stamp (NaN: no target)
        [6] source        0 SUPER's yaw (fall-back), 1 servo, 2 close-range hold,
                          3 stale hold (the last aim point, no usable target)
    Active/inactive edges are logged once, with the reason when it goes inactive (and, after
    a stale hold, how long it held); a bad frame_id is warned once per value; the 5 s line
    gives the servo's state ("on(stale hold: <why>)" during one), its ticks per source, the
    stale holds in the window and the longest, and its worst slew. A hold that ends with
    the target back is not an edge (the servo owned the heading throughout), so it is not
    logged line by line: a target hovering at the 0.5 s freshness limit would do that at
    15 Hz. The target arrives at ~15 Hz, so the slew clamp binds for single
    ticks in a brisk normal follow (yaw_servo.py, step 5): the line gives the LONGEST run
    of consecutive binding ticks and the largest |psi_des - psi_out|, not a bare count.
  * yaw_servo_enable:=true logs a WARN at start-up: it is a bench/test setting until the
    user approves a flight. Then flip the default, as was done for publish_acceleration_ff
    on 2026-09-27. The servo parameters are read-only: `ros2 param set` on a running
    bridge is refused rather than silently ignored.
"""
import copy
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from mars_quadrotor_msgs.msg import PositionCommand
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleControlMode, VehicleLocalPosition)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# yaw_servo.py sits beside this script (plan section 2.1). Put this script's RESOLVED
# directory first on the path, however the node is launched (ros2 run through the install
# symlink, python3 on the repo copy, a gate's scratch copy), and refuse to run on a copy
# found anywhere else: there must be one module, the one beside the bridge that runs.
_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, _HERE)
import yaw_servo  # noqa: E402

if os.path.dirname(os.path.realpath(yaw_servo.__file__)) != _HERE:
    raise ImportError(f'px4_offboard_bridge: yaw_servo imported from {yaw_servo.__file__}, '
                      f'not from beside the bridge ({_HERE})')

NAN = float('nan')
# /bridge/yaw_servo's field names, carried in its layout so a bag is self-describing.
_SERVO_DEBUG_LABEL = ','.join(yaw_servo.DEBUG_LAYOUT)
# Log only: commander sends vehicle_control_mode at >= 2 Hz, so 2 s of silence is a link
# problem worth a warning. It never changes what is published.
MODE_STALE_S = 2.0

# What the bridge is doing with the setpoint stream, reported in the 5 s log line.
IDLE = 'IDLE'    # not armed+offboard: setpoint tracks the live estimate ("hold here");
                 # sent only in disarmed Offboard since 2026-09-28, withheld otherwise
TRACK = 'TRACK'  # armed+offboard, fresh pos_cmd: following SUPER (the only state
                 # that sends acceleration feedforward)
HOLD = 'HOLD'    # armed+offboard, pos_cmd stale: frozen at the last commanded point


def _finite(*vals):
    return all(math.isfinite(v) for v in vals)


def _wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class Px4OffboardBridge(Node):

    def __init__(self):
        super().__init__('px4_offboard_bridge')

        self.declare_parameter('cmd_topic', '/planning/pos_cmd')
        self.declare_parameter('setpoint_topic', '/fmu/in/trajectory_setpoint')
        self.declare_parameter('control_mode_topic', '/fmu/in/offboard_control_mode')
        self.declare_parameter('local_position_topic',
                               '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('vehicle_control_mode_topic',
                               '/fmu/out/vehicle_control_mode')
        # PX4 needs the OffboardControlMode heartbeat at >2 Hz before it will accept the
        # mode switch, and drops Offboard if it lapses for COM_OF_LOSS_T (1.0 s here).
        # 50 Hz gives 25x margin and is the usual PX4 offboard rate. SUPER's own
        # commands arrive at 100 Hz; we resample rather than forward 1:1 so that the
        # stream stays metronomic when SUPER goes quiet.
        self.declare_parameter('publish_rate', 50.0)
        # 0.3 s == 30 missed commands at SUPER's 100 Hz. Well under COM_OF_LOSS_T, so we
        # freeze the setpoint ourselves before PX4's own offboard-loss failsafe fires.
        self.declare_parameter('cmd_timeout', 0.3)
        # Velocity feedforward is the standard PX4 offboard trajectory-tracking path:
        # with a finite position setpoint, PositionControl adds velocity as feedforward.
        self.declare_parameter('publish_velocity_ff', True)
        # Acceleration feedforward: ON since 2026-09-27, by the user's decision after the
        # 2026-09-26 flights. Without it PX4's velocity loop produced SUPER's acceleration
        # only by falling behind the reference, and the aircraft then ran 0.5-0.8 m past
        # the point where the reference stopped (.claude/rules/px4-bridges-offboard.md,
        # 2026-09-27). Bench-gated by m3_bridge_check.py; not flown. PX4 adds it to its
        # velocity-loop output; z scales collective thrust (MPC_ACC_DECOUPLE 1 keeps it
        # out of the tilt). SUPER's max_acc is 1.0 m/s^2 (follow_test_ros2.yaml, since the
        # 2026-09-30 rollback; it was 2.0, and its 09-26 commands peaked at 2.05). Sent only
        # in TRACK: IDLE and HOLD send NaN. Kept through the 2026-09-30 rollback (user).
        self.declare_parameter('publish_acceleration_ff', True)
        # Setpoints only while PX4 reports Offboard: ON since 2026-09-28, by the user's
        # decision after the 2026-09-21 retune's Flight 2, where the always-on IDLE stream
        # was interleaved into Altitude mode and pitched the aircraft against the pilot's
        # stick (.claude/rules/px4-bridges-offboard.md). Bench-gated by m3_bridge_check.py;
        # not flown. The heartbeat is unaffected. false = the old always-on stream, for the
        # bench only: it puts the interference back into every manual mode.
        self.declare_parameter('setpoint_only_in_offboard', True)
        # Warn (do not reject) when the commanded point is implausibly far from the
        # estimate -- the signature of a frame/sign error or a stale latch.
        self.declare_parameter('warn_setpoint_error', 5.0)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.sp_topic = self.get_parameter('setpoint_topic').value
        self.ocm_topic = self.get_parameter('control_mode_topic').value
        self.lpos_topic = self.get_parameter('local_position_topic').value
        self.vcm_topic = self.get_parameter('vehicle_control_mode_topic').value
        self.rate = float(self.get_parameter('publish_rate').value)
        self.timeout = float(self.get_parameter('cmd_timeout').value)
        self.vel_ff = bool(self.get_parameter('publish_velocity_ff').value)
        self.acc_ff = bool(self.get_parameter('publish_acceleration_ff').value)
        self.only_in_offboard = bool(self.get_parameter('setpoint_only_in_offboard').value)
        self.warn_err = float(self.get_parameter('warn_setpoint_error').value)

        # Camera yaw servo (docstring; docs/plan-camera-yaw-servo.md section 2.5). Built
        # 2026-09-29, bench-gated by m3_bridge_check.py, NOT flown: every switch defaults
        # OFF, and no launch path passes these, so the defaults decide.
        self._declare_yaw_servo()

        # PX4 uXRCE-DDS uses BEST_EFFORT + TRANSIENT_LOCAL on /fmu/ topics, both ways.
        px4_pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        px4_sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=5)
        # Must match SUPER's publisher exactly (best_effort/keep_last(1)/volatile) or
        # rclpy's RELIABLE default silently refuses to connect.
        super_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)

        self.sp_pub = self.create_publisher(TrajectorySetpoint, self.sp_topic,
                                            px4_pub_qos)
        self.ocm_pub = self.create_publisher(OffboardControlMode, self.ocm_topic,
                                             px4_pub_qos)
        self.create_subscription(PositionCommand, self.cmd_topic, self.on_cmd,
                                 super_qos)
        self.create_subscription(VehicleLocalPosition, self.lpos_topic,
                                 self.on_local_position, px4_sub_qos)
        self.create_subscription(VehicleControlMode, self.vcm_topic,
                                 self.on_control_mode, px4_sub_qos)
        # The yaw servo's input and its debug topic exist only with the switch on.
        if self.servo is not None:
            # follow_planner publishes BEST_EFFORT/KEEP_LAST(1)/VOLATILE; a BEST_EFFORT,
            # VOLATILE reader matches that and a RELIABLE or TRANSIENT_LOCAL writer alike.
            self.create_subscription(PointStamped, self.yaw_target_topic,
                                     self.on_yaw_target, super_qos)
            # Logging only: best effort, so it can never hold up the tick. rosbag2 and
            # `ros2 topic echo` adapt to a best-effort publisher.
            self.servo_debug_pub = self.create_publisher(
                Float32MultiArray, self.servo_debug_topic,
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=10))

        # Latched setpoint, always already in PX4 NED. `None` means we have nothing
        # safe to send yet and publish only the heartbeat.
        self.sp = None
        self.state = IDLE
        self.last_cmd_time = None
        self.last_traj_flag = None
        self.lpos = None
        # PX4's last word on the mode. Changed ONLY by a received vehicle_control_mode, so
        # a report stream that goes silent leaves the stream as it was (see the docstring).
        self.armed = False
        self.offboard = False
        self.last_mode_time = None
        self.mode_stale_warned = False
        self.streaming = False   # did the last tick send a TrajectorySetpoint

        self.n_cmd = 0
        self.n_cmd_bad = 0
        self.n_sp = 0
        self.n_hold = 0
        self.report_cmd_mark = 0

        # Yaw servo state (unused with the switch off).
        self.yaw_target = None           # (x_flu, y_flu, stamp_ns, frame_id), latest message
        self.servo_last_tick_ns = None   # bridge clock at the previous tick, for dt
        self.servo_active = False        # for the edge log
        self.servo_status = 'idle'       # idle / active / close-range / stale hold / fall-back
        self.servo_reason = ''           # why it is in a stale hold or the fall-back
        self.servo_hold_s = 0.0          # length of the current (or last) stale hold, s
        self.servo_psi_out = NAN
        self.servo_frames_warned = set()
        self.sw_bind_run = 0             # consecutive slew-binding ticks, now
        self._reset_servo_window()

        self.create_timer(1.0 / self.rate, self.on_tick)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'px4_offboard_bridge: {self.cmd_topic} -> {self.sp_topic} '
            f'@ {self.rate:.0f} Hz (FLU->FRD, yaw psi->-psi, '
            f'vel_ff={self.vel_ff}, acc_ff={self.acc_ff}, '
            f'setpoint_only_in_offboard={self.only_in_offboard}, '
            f'{self._servo_settings()}). '
            'This node never arms and never changes mode.')
        if not self.only_in_offboard:
            self.get_logger().warn(
                'setpoint_only_in_offboard is false: setpoints go out in every mode and '
                'mix into Position/Altitude -- bench only')
        if self.servo is not None:
            self.get_logger().warn(
                f'YAW SERVO ON (ff {"ON" if self.servo_params.ff_enable else "off"}): the '
                f'heading sent in Offboard is aimed at {self.yaw_target_topic}, not SUPER\'s '
                '-- a bench/test setting, not flown, until the user approves a flight')

    # ---- inputs -----------------------------------------------------------

    def on_cmd(self, msg: PositionCommand):
        self.n_cmd += 1

        p, v, a, j = msg.position, msg.velocity, msg.acceleration, msg.jerk
        if not _finite(p.x, p.y, p.z, msg.yaw, msg.yaw_dot):
            self.n_cmd_bad += 1
            if self.n_cmd_bad % 50 == 1:
                self.get_logger().error(
                    'non-finite position command from SUPER, dropping '
                    f'(count={self.n_cmd_bad})')
            return

        sp = TrajectorySetpoint()
        # FLU -> FRD: y and z negate; x is unchanged.
        sp.position = [float(p.x), float(-p.y), float(-p.z)]
        sp.yaw = float(_wrap_pi(-msg.yaw))
        sp.yawspeed = float(-msg.yaw_dot)

        if self.vel_ff and _finite(v.x, v.y, v.z):
            sp.velocity = [float(v.x), float(-v.y), float(-v.z)]
        else:
            sp.velocity = [NAN, NAN, NAN]

        if self.acc_ff and _finite(a.x, a.y, a.z):
            sp.acceleration = [float(a.x), float(-a.y), float(-a.z)]
        else:
            sp.acceleration = [NAN, NAN, NAN]

        # Jerk is logging-only in PX4, but keep it in the same frame as the rest.
        if _finite(j.x, j.y, j.z):
            sp.jerk = [float(j.x), float(-j.y), float(-j.z)]
        else:
            sp.jerk = [NAN, NAN, NAN]

        self.sp = sp
        self.last_cmd_time = self.get_clock().now()

        # trajectory_flag 2 == TRAJECTORY_STATUS_EMER: SUPER has fallen back to its
        # backup (emergency-stop) trajectory. Worth seeing in the log the moment it
        # happens, but it is still a valid command and we keep following it.
        if msg.trajectory_flag != self.last_traj_flag:
            if msg.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_EMER:
                self.get_logger().warn(
                    'SUPER switched to its BACKUP (emergency) trajectory')
            elif self.last_traj_flag == PositionCommand.TRAJECTORY_STATUS_EMER:
                self.get_logger().info('SUPER back on its primary trajectory')
            self.last_traj_flag = msg.trajectory_flag

        if self.warn_err > 0.0 and self.lpos is not None and self.lpos.xy_valid:
            err = math.dist(sp.position,
                            [self.lpos.x, self.lpos.y, self.lpos.z])
            if err > self.warn_err and self.n_cmd % 50 == 1:
                self.get_logger().warn(
                    f'commanded point is {err:.1f} m from the estimate '
                    '-- check the frame convention before arming')

    def on_yaw_target(self, msg: PointStamped):
        """Store the latest yaw target as it arrives: FLU lidar_init, unconverted (the servo
        converts it), with its stamp. Its age is taken on each tick, not here."""
        st = msg.header.stamp
        frame = msg.header.frame_id
        self.yaw_target = (msg.point.x, msg.point.y,
                           st.sec * 1_000_000_000 + st.nanosec, frame)
        if (frame not in yaw_servo.ACCEPTED_FRAMES and frame not in self.servo_frames_warned
                and len(self.servo_frames_warned) < 8):
            self.servo_frames_warned.add(frame)
            self.get_logger().warn(
                f'{self.yaw_target_topic} frame_id {frame!r} is not one of '
                f'{"/".join(yaw_servo.ACCEPTED_FRAMES)}: the yaw servo ignores it and falls '
                'back to SUPER\'s yaw (warned once per frame_id)')

    def on_local_position(self, msg: VehicleLocalPosition):
        self.lpos = msg

    def on_control_mode(self, msg: VehicleControlMode):
        was = (self.armed, self.offboard)
        self.armed = msg.flag_armed
        self.offboard = msg.flag_control_offboard_enabled
        self.last_mode_time = self.get_clock().now()
        if self.mode_stale_warned:
            self.mode_stale_warned = False
            self.get_logger().info(f'{self.vcm_topic} is reporting again')
        if (self.armed, self.offboard) != was:
            self.get_logger().info(
                f'PX4 control mode: armed={self.armed} offboard={self.offboard}')

    # ---- output -----------------------------------------------------------

    def on_tick(self):
        now = self.get_clock().now()
        now_us = int(now.nanoseconds / 1000)

        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        # These booleans select which *controller level* PX4 runs, not which fields we
        # populated: position control consumes velocity/acceleration as feedforward.
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self.ocm_pub.publish(ocm)

        engaged = self.armed and self.offboard
        fresh = (self.last_cmd_time is not None
                 and (now - self.last_cmd_time).nanoseconds / 1e9 < self.timeout)

        if not engaged:
            # Not flying under our control. Re-latch onto wherever the vehicle actually
            # is, so Offboard can be engaged at any moment without the drone lurching
            # towards a stale point. Without this, landing, disarming and carrying the
            # rig elsewhere would leave a setpoint back at the old location. Since
            # 2026-09-28 the latch is sent only in disarmed Offboard (see below).
            self.state = IDLE
            self._latch_current_position()
            if self.sp is not None:
                # With no valid estimate the latch above keeps the previous setpoint, and
                # on_cmd may have stored a SUPER command there since the last tick (a pilot
                # takeover mid-trajectory). If it goes out (disarmed Offboard), its position
                # and velocity go as before, but never its acceleration: only TRACK feeds
                # acceleration forward.
                self.sp.acceleration = [NAN, NAN, NAN]
        elif fresh:
            self.state = TRACK
        else:
            # Armed and in Offboard, but SUPER stopped commanding. Freeze at the last
            # commanded point with no feedforward -- position hold. Do NOT chase the
            # live estimate here: that would command "wherever I have drifted to" and
            # hold nothing.
            if self.state == IDLE:
                # Straight from IDLE (the engage, with SUPER silent behind command_mux's
                # engage gate): freeze on this tick's estimate, not the previous tick's.
                self._latch_current_position()
            if self.state != HOLD:
                self.get_logger().warn(
                    f'no {self.cmd_topic} for {self.timeout:.2f} s while in Offboard '
                    '-- holding the last commanded position')
            self.state = HOLD
            self.n_hold += 1
            if self.sp is not None:
                self.sp.velocity = [NAN, NAN, NAN]
                self.sp.acceleration = [NAN, NAN, NAN]
                self.sp.jerk = [NAN, NAN, NAN]
                self.sp.yawspeed = 0.0

        # The yaw servo: runs on every tick while its switch is on, engaged or not (not
        # engaged, it resets itself and passes SUPER's yaw through). Switch off: None, and
        # nothing below differs from the bridge without it.
        servo_out = None
        if self.servo is not None:
            servo_out = self._servo_step(now, engaged)

        if self.sp is None:
            return

        if self.only_in_offboard and not self.offboard:
            # PX4 is not in Offboard (its last report says so): its own flight task, if
            # any, owns trajectory_setpoint. Keep the latch, send nothing.
            if self.streaming:
                self.get_logger().info(
                    'PX4 left Offboard: setpoint stream stopped (heartbeat continues)')
            self.streaming = False
            return

        if self.only_in_offboard and self.last_mode_time is not None:
            silent = (now - self.last_mode_time).nanoseconds / 1e9
            if silent > MODE_STALE_S and not self.mode_stale_warned:
                self.mode_stale_warned = True
                self.get_logger().warn(
                    f'no {self.vcm_topic} for {silent:.1f} s while PX4 was last in Offboard '
                    '-- still sending setpoints: only a report that Offboard is off stops them')

        self.sp.timestamp = now_us
        if servo_out is None or servo_out.passthrough:
            self.sp_pub.publish(self.sp)
        else:
            # The servo's heading goes out on a COPY. self.sp keeps SUPER's yaw (in HOLD the
            # frozen yaw, the servo's fall-back); position, velocity, acceleration, jerk and
            # timestamp are copied as they are.
            out = copy.deepcopy(self.sp)
            out.yaw = float(servo_out.yaw)
            out.yawspeed = float(servo_out.yawspeed)
            self.sp_pub.publish(out)
        self.n_sp += 1
        if not self.streaming:
            p = self.sp.position
            self.get_logger().info(
                f'setpoint stream started ({self.state}, offboard={self.offboard}, '
                f'armed={self.armed}): NED [{p[0]:+.2f} {p[1]:+.2f} {p[2]:+.2f}]')
        self.streaming = True

    def _latch_current_position(self):
        """Setpoint := hold the current estimate. Already NED; no conversion."""
        lp = self.lpos
        if lp is None or not (lp.xy_valid and lp.z_valid):
            return
        if not _finite(lp.x, lp.y, lp.z, lp.heading):
            return
        sp = TrajectorySetpoint()
        sp.position = [float(lp.x), float(lp.y), float(lp.z)]
        sp.velocity = [NAN, NAN, NAN]
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yaw = float(_wrap_pi(lp.heading))
        sp.yawspeed = 0.0
        self.sp = sp

    # ---- yaw servo --------------------------------------------------------

    def _declare_yaw_servo(self):
        """Declare and validate the yaw servo's parameters, and build the servo if it is on.
        A bad value stops the node here, at start-up, with the reason: it is never replaced
        by a default. Read-only, so a running bridge refuses `ros2 param set` instead of
        appearing to take a value it would never read."""
        ro = ParameterDescriptor(read_only=True)
        try:
            # fix 1: aim the heading at the yaw target from where the aircraft is.
            self.declare_parameter('yaw_servo_enable', False, ro)
            # fix 2: that bearing's rate as yawspeed. No effect unless yaw_servo_enable.
            self.declare_parameter('yaw_servo_ff_enable', False, ro)
            # Aim in HOLD too, e.g. the engage HOLD before SUPER's first command. false =
            # the frozen yaw until the first TRACK after an engage (plan sections 2.4, 6).
            self.declare_parameter('yaw_servo_aim_in_hold', True, ro)
            self.declare_parameter('yaw_target_topic', '/planning/yaw_target', ro)
            self.declare_parameter('yaw_servo_debug_topic', '/bridge/yaw_servo', ro)
            # s. Stamp age above which the target is stale: SUPER's kYawTargetTimeoutSec.
            self.declare_parameter('yaw_servo_target_timeout', 0.5, ro)
            # deg/s. The heading slew limit, on every engaged tick, servo or fall-back.
            self.declare_parameter('yaw_servo_max_rate', 90.0, ro)
            # deg/s and deg/s^2: the feed-forward's cap and its slew.
            self.declare_parameter('yaw_servo_ff_max_rate', 60.0, ro)
            self.declare_parameter('yaw_servo_ff_max_accel', 90.0, ro)
            # m. Closer than this: hold the heading, yawspeed 0 (rate = speed / range).
            self.declare_parameter('yaw_servo_min_range_m', 2.5, ro)
            # m. ... and leave that hold only at >= min_range_m + this (0 = round one).
            self.declare_parameter('yaw_servo_min_range_hysteresis_m', 0.3, ro)
            # s. No usable target after an aimed tick: hold the last aim this long before
            # falling back to SUPER's yaw (0 = round one: fall back at once).
            self.declare_parameter('yaw_servo_stale_hold_s', 0.5, ro)
            # deg, + = the camera looks LEFT of the nose (yaw_servo.py). 0 until measured.
            self.declare_parameter('yaw_servo_camera_offset_deg', 0.0, ro)

            def val(name):
                return self.get_parameter(name).value

            params = yaw_servo.YawServoParams(
                enable=val('yaw_servo_enable'),
                ff_enable=val('yaw_servo_ff_enable'),
                aim_in_hold=val('yaw_servo_aim_in_hold'),
                target_timeout=val('yaw_servo_target_timeout'),
                max_rate=val('yaw_servo_max_rate'),
                ff_max_rate=val('yaw_servo_ff_max_rate'),
                ff_max_accel=val('yaw_servo_ff_max_accel'),
                min_range_m=val('yaw_servo_min_range_m'),
                min_range_hysteresis_m=val('yaw_servo_min_range_hysteresis_m'),
                stale_hold_s=val('yaw_servo_stale_hold_s'),
                camera_offset_deg=val('yaw_servo_camera_offset_deg'))
            for name in ('yaw_target_topic', 'yaw_servo_debug_topic'):
                if not val(name):
                    raise ValueError(f'{name} must not be empty')
        except (InvalidParameterTypeException, TypeError, ValueError) as e:
            self.get_logger().fatal(f'bad yaw servo parameter: {e} -- not starting')
            raise SystemExit(1) from None
        self.servo_params = params
        self.yaw_target_topic = val('yaw_target_topic')
        self.servo_debug_topic = val('yaw_servo_debug_topic')
        self.servo = yaw_servo.YawServo(params) if params.enable else None

    def _servo_settings(self):
        """The switches and limits, for the startup line."""
        p = self.servo_params
        if not p.enable:
            return f'yaw_servo=off (ff={"on" if p.ff_enable else "off"})'
        return (f'yaw_servo=ON ff={"on" if p.ff_enable else "off"} '
                f'aim_in_hold={p.aim_in_hold} target={self.yaw_target_topic} '
                f'timeout={p.target_timeout:.2f}s max_rate={p.max_rate:.0f}deg/s '
                f'ff_max={p.ff_max_rate:.0f}deg/s ff_accel={p.ff_max_accel:.0f}deg/s^2 '
                f'min_range={p.min_range_m:.1f}m hysteresis={p.min_range_hysteresis_m:.2f}m '
                f'stale_hold={p.stale_hold_s:.2f}s camera_offset={p.camera_offset_deg:+.1f}deg '
                f'debug={self.servo_debug_topic}')

    def _servo_step(self, now, engaged):
        """One servo tick. Reads self.sp (SUPER's yaw, the IDLE latch or the HOLD's frozen
        yaw) and never writes it. Returns the YawServoOutput for the caller to apply."""
        now_ns = now.nanoseconds
        if self.servo_last_tick_ns is None:
            dt = 1.0 / self.rate
        else:
            dt = (now_ns - self.servo_last_tick_ns) / 1e9
        self.servo_last_tick_ns = now_ns

        sp, lp, tgt = self.sp, self.lpos, None
        if self.yaw_target is not None:
            x, y, stamp_ns, frame = self.yaw_target
            tgt = (x, y, (now_ns - stamp_ns) / 1e9, frame)   # age on THIS tick's clock
        out = self.servo.step(
            dt=dt, state=self.state, engaged=engaged,
            psi_super=sp.yaw if sp is not None else NAN,
            yawdot_super=sp.yawspeed if sp is not None else NAN,
            vehicle_heading=lp.heading if lp is not None else NAN,
            vehicle_x=lp.x if lp is not None else NAN,
            vehicle_y=lp.y if lp is not None else NAN,
            vehicle_xy_valid=bool(lp.xy_valid) if lp is not None else False,
            target=tgt)

        if not engaged:
            status, reason = 'idle', 'not engaged'
        elif out.source == yaw_servo.SOURCE_SERVO:
            status, reason = 'active', ''
        elif out.source == yaw_servo.SOURCE_CLOSE_HOLD:
            status, reason = 'close-range', ''
        elif out.source == yaw_servo.SOURCE_STALE_HOLD:
            status, reason = 'stale hold', self._servo_reason(out, tgt)
        else:
            status, reason = 'fall-back', self._servo_reason(out, tgt)
        if status == 'stale hold':
            # Log only: how long this hold has lasted, in bridge tick time.
            if self.servo_status != 'stale hold':
                self.servo_hold_s = 0.0
                self.sw_holds += 1
            if math.isfinite(dt) and dt > 0.0:
                self.servo_hold_s += dt
            self.sw_hold_max = max(self.sw_hold_max, self.servo_hold_s)
        if out.active != self.servo_active:
            self.servo_active = out.active
            if out.active:
                self.get_logger().info(
                    f'yaw servo ACTIVE ({status}): target {out.range_m:.1f} m away, bearing '
                    f'{math.degrees(out.psi_des):+.0f} deg FRD, heading '
                    f'{math.degrees(out.psi_out):+.0f} deg')
            elif engaged:
                held = (f'; after holding the last aim point {self.servo_hold_s:.2f} s'
                        if self.servo_status == 'stale hold' else '')
                self.get_logger().warn(
                    f'yaw servo INACTIVE ({reason}{held}): back to SUPER\'s yaw, slew-limited')
            else:
                self.get_logger().info('yaw servo INACTIVE (not engaged)')
        self.servo_status, self.servo_reason = status, reason

        if engaged and not out.passthrough:
            self.sw_ticks[status] += 1
            self.servo_psi_out = out.psi_out
            lag = abs(yaw_servo.wrap_pi(out.psi_des - out.psi_out))
            if lag > self.sw_lag_max:
                self.sw_lag_max = lag
            if out.slew_binding:
                self.sw_bind_run += 1
                self.sw_bind_run_max = max(self.sw_bind_run_max, self.sw_bind_run)
            else:
                self.sw_bind_run = 0
        else:
            self.sw_bind_run = 0

        if engaged:
            m = Float32MultiArray()
            m.layout.dim = [MultiArrayDimension(label=_SERVO_DEBUG_LABEL,
                                                size=len(yaw_servo.DEBUG_LAYOUT),
                                                stride=len(yaw_servo.DEBUG_LAYOUT))]
            m.data = out.debug_array()
            self.servo_debug_pub.publish(m)
        return out

    def _servo_reason(self, out, tgt):
        """Why the servo is in fall-back on an engaged tick, from its output and inputs.
        Checked in the module's order (yaw_servo.py step 1)."""
        p = self.servo_params
        if out.passthrough:
            return 'no finite heading or SUPER yaw to start from'
        if tgt is None:
            return 'no target yet'
        x, y, age, frame = tgt
        if not math.isfinite(age):
            return 'target stamp unusable'
        if age > p.target_timeout:
            return f'stale target, {age:.2f} s old'
        if age < -p.future_tolerance:
            return f'target stamped {-age:.2f} s in the future'
        if out.bad_frame:
            return f'target frame_id {frame!r}'
        lp = self.lpos
        if lp is None or not lp.xy_valid or not _finite(lp.x, lp.y, lp.heading):
            return 'lpos invalid'
        if not _finite(x, y):
            return 'non-finite target'
        if self.state == HOLD and not p.aim_in_hold:
            return 'HOLD before the first TRACK (aim_in_hold false)'
        return 'unknown'

    def _reset_servo_window(self):
        self.sw_ticks = {'active': 0, 'close-range': 0, 'stale hold': 0, 'fall-back': 0}
        self.sw_bind_run_max = 0
        self.sw_lag_max = 0.0
        self.sw_holds = 0                # stale holds that started in the window
        self.sw_hold_max = 0.0           # the longest stale hold seen in the window, s

    def _servo_report(self):
        """The yaw servo's part of the 5 s line; starts a new window."""
        if self.servo is None:
            return ' | yaw servo off'
        t = self.sw_ticks
        state = self.servo_status
        if state in ('fall-back', 'stale hold'):
            state = f'{state}: {self.servo_reason}'
        text = f' | yaw servo on({state})'
        if sum(t.values()):
            text += (f' yaw_out {self.servo_psi_out:+.2f}, ticks servo {t["active"]} '
                     f'close {t["close-range"]} stale-hold {t["stale hold"]} fall-back '
                     f'{t["fall-back"]}, stale holds {self.sw_holds} (longest '
                     f'{self.sw_hold_max:.2f} s), slew: longest bind {self.sw_bind_run_max} '
                     f'ticks, max |des-out| {math.degrees(self.sw_lag_max):.1f} deg')
        self._reset_servo_window()
        return text

    # ---- reporting --------------------------------------------------------

    def report(self):
        rate = (self.n_cmd - self.report_cmd_mark) / 5.0
        self.report_cmd_mark = self.n_cmd

        if self.sp is None:
            self.get_logger().warn(
                f'no setpoint to publish: nothing on {self.cmd_topic} yet and no '
                f'valid estimate on {self.lpos_topic} '
                '-- is the stack up and px4_odom_bridge running?')
            return

        age = 'never'
        if self.last_cmd_time is not None:
            age = f'{(self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9:.1f}s'
        sp = self.sp
        held = '' if self.streaming else ' (withheld: PX4 not in Offboard)'
        self.get_logger().info(
            f'{self.state} cmd_in={self.n_cmd} ({rate:.0f} Hz, last {age} ago) '
            f'bad={self.n_cmd_bad} sp_out={self.n_sp}{held} armed={self.armed} '
            f'offboard={self.offboard} | NED sp '
            f'[{sp.position[0]:+.2f} {sp.position[1]:+.2f} {sp.position[2]:+.2f}] '
            f'yaw {sp.yaw:+.2f}{self._servo_report()}')
        if self.state == IDLE and self.n_cmd == 0:
            self.get_logger().info(
                f'  ({self.cmd_topic} is silent -- normal: SUPER only publishes in '
                'FOLLOW_TRAJ/EMER_STOP, so it needs a /goal_pose first)')


def main(args=None):
    rclpy.init(args=args)
    node = Px4OffboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
