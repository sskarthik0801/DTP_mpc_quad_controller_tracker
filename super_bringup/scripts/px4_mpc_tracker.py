#!/usr/bin/env python3
"""MPC trajectory tracker: SUPER commands -> PX4 Offboard ACCELERATION setpoints.

A drop-in ALTERNATIVE to px4_offboard_bridge.py. Run one or the other, never both
(this node refuses to start if something already publishes the setpoint topic).
docs/mpc-tracker.md is the design; docs/mpc-tracker-integration.md is how to run it.

Subscribes  /planning/pos_cmd                  (mars_quadrotor_msgs/PositionCommand)
            /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition) -- MPC state
            /fmu/out/vehicle_control_mode      (px4_msgs/VehicleControlMode)
Publishes   /fmu/in/trajectory_setpoint        (px4_msgs/TrajectorySetpoint)
            /fmu/in/offboard_control_mode      (px4_msgs/OffboardControlMode)
Only with yaw_servo_enable (default off; the same switch and parameters as the bridge):
Subscribes  /planning/yaw_target               (geometry_msgs/PointStamped)
Publishes   /bridge/yaw_servo                  (std_msgs/Float32MultiArray)   -- logging only

The stock bridge sends a position setpoint and PX4's position/velocity PID loops do the
tracking. Here the MPC (mpc_tracker_core.py) does the tracking and sends an acceleration
setpoint; PX4 keeps attitude, rate and motor control. Frames, QoS and the silent
pos_cmd stream are handled exactly as in px4_offboard_bridge.py -- read its docstring.

This node NEVER arms and NEVER changes flight mode.

States (reported every 5 s)
---------------------------
IDLE      PX4 not in Offboard. Heartbeat ONLY -- no trajectory_setpoint, the same rule as
          the bridge's setpoint_only_in_offboard (2026-09-28): PX4's own Position,
          Altitude and Auto-land tasks write that uORB topic.
POS_HOLD  in Offboard but disarmed (engage-then-arm): a plain position setpoint at the
          current estimate. The MPC does not run on the ground.
TRACK     armed + Offboard + fresh pos_cmd: MPC following SUPER.
HOLD      armed + Offboard, pos_cmd stale or never received: MPC holding the last
          commanded point, or this tick's estimate at the engage (with command_mux's engage
          gate SUPER is silent until armed + Offboard, so every engage starts here).
FALLBACK  armed + Offboard but the estimate is stale/invalid or the MPC failed: a plain
          position setpoint, i.e. PX4's own controller takes over, at the last reference.

As in the bridge, the Offboard flag changes only on a RECEIVED vehicle_control_mode: a
report stream that goes silent never stops the setpoints (that would starve PX4 in the
air); it is only warned about after 2 s.

Camera yaw servo (default OFF, as in the bridge)
------------------------------------------------
yaw_servo.py beside this file is the one copy of the algorithm; px4_offboard_bridge.py's
docstring describes it. Here it rewrites only yaw and yawspeed of the outgoing setpoint in
TRACK and HOLD; the MPC's acceleration is never touched. In POS_HOLD and FALLBACK it is
handed IDLE, so it resets and passes the yaw through. Its parameters have the bridge's
names and defaults, so a launch line written for the bridge works here unchanged.
"""
import math
import os
import sys
import time

import numpy as np
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

# mpc_tracker_core.py and yaw_servo.py sit beside this script (installed with it by
# super_bringup/CMakeLists.txt). Import them from this script's RESOLVED directory and
# refuse a copy found anywhere else, as the bridge does for yaw_servo.
_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, _HERE)
import mpc_tracker_core  # noqa: E402
import yaw_servo  # noqa: E402
from mpc_tracker_core import Tracker, TrackerParams  # noqa: E402

for _mod in (mpc_tracker_core, yaw_servo):
    if os.path.dirname(os.path.realpath(_mod.__file__)) != _HERE:
        raise ImportError(f'px4_mpc_tracker: {_mod.__name__} imported from {_mod.__file__}, '
                          f'not from beside the tracker ({_HERE})')

NAN = float('nan')
IDLE, POS_HOLD, TRACK, HOLD, FALLBACK = 'IDLE', 'POS_HOLD', 'TRACK', 'HOLD', 'FALLBACK'
MPC_STATES = (TRACK, HOLD)
# Log only, as in the bridge: commander sends vehicle_control_mode at >= 2 Hz.
MODE_STALE_S = 2.0
_SERVO_DEBUG_LABEL = ','.join(yaw_servo.DEBUG_LAYOUT)

# Parameter name on the node -> TrackerParams attribute.
MPC_PARAMS = ('h', 'N', 'tau', 'qp', 'qv', 'r', 'rdu', 'terminal', 'a_xy_max',
              'a_up_max', 'a_down_max', 'max_pos_err', 'v_ref_max', 'acc_extrap_s',
              'd_time_const', 'd_max', 'max_iters')

# The yaw servo's node parameters, with the bridge's names and defaults
# (px4_offboard_bridge._declare_yaw_servo) -> YawServoParams keyword.
SERVO_PARAMS = (
    ('yaw_servo_enable', False, 'enable'),
    ('yaw_servo_ff_enable', False, 'ff_enable'),
    ('yaw_servo_aim_in_hold', True, 'aim_in_hold'),
    ('yaw_servo_target_timeout', 0.5, 'target_timeout'),
    ('yaw_servo_max_rate', 90.0, 'max_rate'),
    ('yaw_servo_ff_max_rate', 60.0, 'ff_max_rate'),
    ('yaw_servo_ff_max_accel', 90.0, 'ff_max_accel'),
    ('yaw_servo_min_range_m', 2.5, 'min_range_m'),
    ('yaw_servo_min_range_hysteresis_m', 0.3, 'min_range_hysteresis_m'),
    ('yaw_servo_stale_hold_s', 0.5, 'stale_hold_s'),
    ('yaw_servo_camera_offset_deg', 0.0, 'camera_offset_deg'),
)
_SOURCE_NAMES = {yaw_servo.SOURCE_SUPER: 'fall-back', yaw_servo.SOURCE_SERVO: 'servo',
                 yaw_servo.SOURCE_CLOSE_HOLD: 'close-range',
                 yaw_servo.SOURCE_STALE_HOLD: 'stale hold'}


def _finite(*vals):
    return all(math.isfinite(v) for v in vals)


def _wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class Px4MpcTracker(Node):

    def __init__(self):
        super().__init__('px4_mpc_tracker')

        # Read once at start-up, so read-only: `ros2 param set` on a running tracker is
        # refused rather than silently ignored.
        ro = ParameterDescriptor(read_only=True)
        self.declare_parameter('cmd_topic', '/planning/pos_cmd', ro)
        self.declare_parameter('setpoint_topic', '/fmu/in/trajectory_setpoint', ro)
        self.declare_parameter('control_mode_topic', '/fmu/in/offboard_control_mode', ro)
        self.declare_parameter('local_position_topic',
                               '/fmu/out/vehicle_local_position_v1', ro)
        self.declare_parameter('vehicle_control_mode_topic',
                               '/fmu/out/vehicle_control_mode', ro)
        self.declare_parameter('publish_rate', 50.0, ro)
        self.declare_parameter('cmd_timeout', 0.3, ro)
        # The MPC's state IS this estimate. Older than this and we hand back to PX4.
        self.declare_parameter('lpos_timeout', 0.2, ro)
        defaults = TrackerParams()
        for name in MPC_PARAMS:
            self.declare_parameter('mpc_' + name, getattr(defaults, name), ro)
        self.declare_parameter('yaw_target_topic', '/planning/yaw_target', ro)
        self.declare_parameter('yaw_servo_debug_topic', '/bridge/yaw_servo', ro)
        for name, default, _kw in SERVO_PARAMS:
            self.declare_parameter(name, default, ro)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cmd_topic = gp('cmd_topic')
        self.sp_topic = gp('setpoint_topic')
        self.vcm_topic = gp('vehicle_control_mode_topic')
        self.rate = float(gp('publish_rate'))
        self.timeout = float(gp('cmd_timeout'))
        self.lpos_timeout = float(gp('lpos_timeout'))
        prm = TrackerParams(**{n: type(getattr(defaults, n))(gp('mpc_' + n))
                               for n in MPC_PARAMS})
        self.tracker = Tracker(prm)
        try:
            self.servo_params = yaw_servo.YawServoParams(
                **{kw: gp(name) for name, _d, kw in SERVO_PARAMS})
            for name in ('yaw_target_topic', 'yaw_servo_debug_topic'):
                if not gp(name):
                    raise ValueError(f'{name} must not be empty')
        except (InvalidParameterTypeException, TypeError, ValueError) as e:
            self.get_logger().fatal(f'bad yaw servo parameter: {e} -- not starting')
            raise SystemExit(1) from None
        self.servo = (yaw_servo.YawServo(self.servo_params)
                      if self.servo_params.enable else None)
        self.yaw_target_topic = gp('yaw_target_topic')

        px4_pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        px4_sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=5)
        super_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)

        # Two setpoint publishers would interleave on PX4's uORB topic. Give DDS a moment
        # to discover an already-running px4_offboard_bridge, then refuse.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.count_publishers(self.sp_topic) > 0:
                raise RuntimeError(
                    f'{self.sp_topic} already has a publisher (px4_offboard_bridge?). '
                    'Run either the bridge or the MPC tracker, never both.')

        self.sp_pub = self.create_publisher(TrajectorySetpoint, self.sp_topic, px4_pub_qos)
        self.ocm_pub = self.create_publisher(OffboardControlMode, gp('control_mode_topic'),
                                             px4_pub_qos)
        self.create_subscription(PositionCommand, self.cmd_topic, self.on_cmd, super_qos)
        self.create_subscription(VehicleLocalPosition, gp('local_position_topic'),
                                 self.on_local_position, px4_sub_qos)
        self.create_subscription(VehicleControlMode, self.vcm_topic,
                                 self.on_control_mode, px4_sub_qos)
        # The yaw servo's input and debug topic exist only with the switch on (bridge rule).
        if self.servo is not None:
            self.create_subscription(PointStamped, self.yaw_target_topic,
                                     self.on_yaw_target, super_qos)
            self.servo_debug_pub = self.create_publisher(
                Float32MultiArray, gp('yaw_servo_debug_topic'),
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=10))

        # Latest SUPER command, already NED: (p, v, a, yaw, yawspeed).
        self.cmd = None
        self.last_cmd_time = None
        self.last_traj_flag = None
        self.hold_ref = None          # (p, yaw) the MPC holds when pos_cmd is stale
        self.lpos = None
        self.lpos_time = None
        self.armed = False
        self.offboard = False
        self.last_mode_time = None
        self.mode_stale_warned = False
        self.streaming = False        # did the last tick send a TrajectorySetpoint
        self.state = IDLE
        self.last_tick = None
        self.last_u = np.zeros(3)

        self.yaw_target = None        # (x_flu, y_flu, stamp_ns, frame_id), latest message
        self.servo_active = False
        self.servo_ticks = dict.fromkeys(_SOURCE_NAMES.values(), 0)

        self.n_cmd = self.n_cmd_bad = self.n_sp = self.n_fallback = 0
        self.solve_ms = []
        self.err = []

        self.create_timer(1.0 / self.rate, self.on_tick)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'px4_mpc_tracker: {self.cmd_topic} -> ACCELERATION setpoints on {self.sp_topic} '
            f'@ {self.rate:.0f} Hz, horizon {prm.N} x {prm.h:.3f} s, tau {prm.tau:.2f} s, '
            f'|a_xy|<={prm.a_xy_max}, a_up<={prm.a_up_max}, a_dn<={prm.a_down_max}, '
            f'max_pos_err {prm.max_pos_err} m, '
            f'yaw_servo={"ON" if self.servo else "off"}. '
            'Setpoints only while PX4 is in Offboard. '
            'This node never arms and never changes mode.')
        if self.servo is not None:
            self.get_logger().warn(
                f'YAW SERVO ON (ff {"ON" if self.servo_params.ff_enable else "off"}): the '
                f'heading sent in Offboard is aimed at {self.yaw_target_topic}, not SUPER\'s '
                '-- a bench/test setting, not flown, until the user approves a flight')

    # ---- inputs -----------------------------------------------------------

    def on_cmd(self, msg: PositionCommand):
        self.n_cmd += 1
        p, v, a = msg.position, msg.velocity, msg.acceleration
        if not _finite(p.x, p.y, p.z, msg.yaw, msg.yaw_dot):
            self.n_cmd_bad += 1
            if self.n_cmd_bad % 50 == 1:
                self.get_logger().error(f'non-finite command from SUPER, dropping '
                                        f'(count={self.n_cmd_bad})')
            return
        # FLU -> FRD/NED-like: (x, y, z) -> (x, -y, -z), yaw -> -yaw. NOT pi/2 - yaw.
        vel = np.array([v.x, -v.y, -v.z]) if _finite(v.x, v.y, v.z) else np.zeros(3)
        acc = np.array([a.x, -a.y, -a.z]) if _finite(a.x, a.y, a.z) else np.zeros(3)
        self.cmd = (np.array([p.x, -p.y, -p.z]), vel, acc,
                    _wrap_pi(-msg.yaw), -msg.yaw_dot)
        self.last_cmd_time = self.get_clock().now()
        if msg.trajectory_flag != self.last_traj_flag:
            if msg.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_EMER:
                self.get_logger().warn('SUPER switched to its BACKUP (emergency) trajectory')
            elif self.last_traj_flag == PositionCommand.TRAJECTORY_STATUS_EMER:
                self.get_logger().info('SUPER back on its primary trajectory')
            self.last_traj_flag = msg.trajectory_flag

    def on_yaw_target(self, msg: PointStamped):
        st = msg.header.stamp
        self.yaw_target = (msg.point.x, msg.point.y,
                           st.sec * 1_000_000_000 + st.nanosec, msg.header.frame_id)

    def on_local_position(self, msg: VehicleLocalPosition):
        self.lpos = msg
        self.lpos_time = self.get_clock().now()

    def on_control_mode(self, msg: VehicleControlMode):
        was = (self.armed, self.offboard)
        self.armed = msg.flag_armed
        self.offboard = msg.flag_control_offboard_enabled
        self.last_mode_time = self.get_clock().now()
        if self.mode_stale_warned:
            self.mode_stale_warned = False
            self.get_logger().info(f'{self.vcm_topic} is reporting again')
        if (self.armed, self.offboard) != was:
            self.get_logger().info(f'PX4 control mode: armed={self.armed} '
                                   f'offboard={self.offboard}')

    def _estimate(self, now):
        """(p, v, heading) in NED if the estimate is fresh and valid, else None."""
        lp = self.lpos
        if lp is None or self.lpos_time is None:
            return None
        if (now - self.lpos_time).nanoseconds / 1e9 > self.lpos_timeout:
            return None
        if not (lp.xy_valid and lp.z_valid and lp.v_xy_valid and lp.v_z_valid):
            return None
        if not _finite(lp.x, lp.y, lp.z, lp.vx, lp.vy, lp.vz, lp.heading):
            return None
        return (np.array([lp.x, lp.y, lp.z]), np.array([lp.vx, lp.vy, lp.vz]), lp.heading)

    # ---- output -----------------------------------------------------------

    def _set_state(self, s):
        if s != self.state:
            log = self.get_logger().warn if s == FALLBACK else self.get_logger().info
            log(f'{self.state} -> {s}')
            self.state = s

    def on_tick(self):
        now = self.get_clock().now()
        now_us = int(now.nanoseconds / 1000)
        dt = 1.0 / self.rate
        if self.last_tick is not None:
            dt = min(max((now - self.last_tick).nanoseconds / 1e9, 0.005), 0.1)
        self.last_tick = now

        # What to send: (mode, pos, acc, yaw, yawspeed), or None = heartbeat only.
        sp = self._control(now, dt)

        # The servo steps on every tick while its switch is on, engaged or not (bridge rule).
        if self.servo is not None:
            sp = self._servo_apply(now, dt, sp)

        mode = sp[0] if sp is not None else 'position'
        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        ocm.position = mode == 'position'
        ocm.acceleration = mode == 'acceleration'
        self.ocm_pub.publish(ocm)

        if sp is None:
            if self.streaming:
                self.get_logger().info(
                    'PX4 left Offboard: setpoint stream stopped (heartbeat continues)')
            self.streaming = False
            return

        if self.last_mode_time is not None and not self.mode_stale_warned:
            silent = (now - self.last_mode_time).nanoseconds / 1e9
            if silent > MODE_STALE_S:
                self.mode_stale_warned = True
                self.get_logger().warn(
                    f'no {self.vcm_topic} for {silent:.1f} s while PX4 was last in Offboard '
                    '-- still sending setpoints: only a report that Offboard is off stops them')

        _mode, pos, acc, yaw, yawspeed = sp
        msg = TrajectorySetpoint()
        msg.timestamp = now_us
        msg.position = [float(x) for x in pos] if pos is not None else [NAN] * 3
        msg.velocity = [NAN] * 3
        msg.acceleration = [float(x) for x in acc] if acc is not None else [NAN] * 3
        msg.jerk = [NAN] * 3
        msg.yaw = float(yaw)
        msg.yawspeed = float(yawspeed)
        self.sp_pub.publish(msg)
        self.n_sp += 1
        if not self.streaming:
            self.get_logger().info(
                f'setpoint stream started ({self.state}, {mode}, offboard={self.offboard}, '
                f'armed={self.armed})')
        self.streaming = True

    def _control(self, now, dt):
        """The state machine. Returns the setpoint to send, or None for heartbeat only."""
        est = self._estimate(now)

        if not self.offboard:
            # PX4's own flight task, if any, owns trajectory_setpoint: send nothing.
            self._set_state(IDLE)
            self.hold_ref = None
            return None

        if not self.armed:
            self._set_state(POS_HOLD)
            self.hold_ref = None
            if est is None:
                return None
            return ('position', est[0], None, est[2], 0.0)

        if est is None:
            return self._fallback('estimate stale or invalid')
        p, v, heading = est

        if self.state not in MPC_STATES:
            # Entering MPC control (engage, arm, or recovery from FALLBACK): start clean
            # and, unless SUPER is already commanding, hold where we are on THIS tick.
            self.tracker.reset()
            self.last_u = np.zeros(3)
            if self.hold_ref is None:
                self.hold_ref = (p.copy(), heading)

        fresh = (self.last_cmd_time is not None
                 and (now - self.last_cmd_time).nanoseconds / 1e9 < self.timeout)
        if fresh:
            ref_p, ref_v, ref_a, yaw, yawspeed = self.cmd
            age = (now - self.last_cmd_time).nanoseconds / 1e9
            self.hold_ref = (ref_p + ref_v * age, yaw)
            self._set_state(TRACK)
        else:
            if self.state == TRACK:
                self.get_logger().warn(f'no {self.cmd_topic} for {self.timeout:.2f} s '
                                       '-- MPC holding the last commanded point')
            ref_p, yaw = self.hold_ref
            ref_v = ref_a = np.zeros(3)
            yawspeed, age = 0.0, 0.0
            self._set_state(HOLD)

        t0 = time.perf_counter()
        try:
            u = self.tracker.step(p, v, ref_p, ref_v, ref_a, dt, ref_age=age)
        except (FloatingPointError, np.linalg.LinAlgError) as e:
            return self._fallback(f'MPC failed: {e}')
        self.solve_ms.append(1e3 * (time.perf_counter() - t0))
        self.err.append(float(np.linalg.norm(p - ref_p)))
        self.tracker.after_send(v, u, 1.0 / self.rate)
        self.last_u = u
        return ('acceleration', None, u, yaw, yawspeed)

    def _fallback(self, why):
        self.n_fallback += 1
        if self.state != FALLBACK:
            self.get_logger().error(f'{why} -- handing control back to PX4 position '
                                    'control at the last reference')
        self._set_state(FALLBACK)
        if self.hold_ref is None:
            return None
        return ('position', self.hold_ref[0], None, self.hold_ref[1], 0.0)

    # ---- yaw servo --------------------------------------------------------

    def _servo_apply(self, now, dt, sp):
        """One servo tick (only with the switch on). Rewrites yaw/yawspeed of sp in TRACK
        and HOLD; every other state is handed to the servo as IDLE, so it resets."""
        engaged = self.armed and self.offboard
        state = self.state if self.state in MPC_STATES else yaw_servo.IDLE
        lp, tgt = self.lpos, None
        if self.yaw_target is not None:
            x, y, stamp_ns, frame = self.yaw_target
            tgt = (x, y, (now.nanoseconds - stamp_ns) / 1e9, frame)
        out = self.servo.step(
            dt=dt, state=state, engaged=engaged,
            psi_super=sp[3] if sp is not None else NAN,
            yawdot_super=sp[4] if sp is not None else NAN,
            vehicle_heading=lp.heading if lp is not None else NAN,
            vehicle_x=lp.x if lp is not None else NAN,
            vehicle_y=lp.y if lp is not None else NAN,
            vehicle_xy_valid=bool(lp.xy_valid) if lp is not None else False,
            target=tgt)
        if out.active != self.servo_active:
            self.servo_active = out.active
            log = self.get_logger().info if out.active else self.get_logger().warn
            log(f'yaw servo {"ACTIVE" if out.active else "INACTIVE"} '
                f'({_SOURCE_NAMES.get(out.source, out.source)}, state {self.state})')
        if engaged:
            m = Float32MultiArray()
            m.layout.dim = [MultiArrayDimension(label=_SERVO_DEBUG_LABEL,
                                                size=len(yaw_servo.DEBUG_LAYOUT),
                                                stride=len(yaw_servo.DEBUG_LAYOUT))]
            m.data = out.debug_array()
            self.servo_debug_pub.publish(m)
        if sp is None or out.passthrough or state not in MPC_STATES:
            return sp
        self.servo_ticks[_SOURCE_NAMES.get(out.source, 'fall-back')] += 1
        return (sp[0], sp[1], sp[2], out.yaw, out.yawspeed)

    # ---- reporting --------------------------------------------------------

    def report(self):
        if self.count_publishers(self.sp_topic) > 1:
            self.get_logger().error(f'{self.sp_topic} has more than one publisher -- '
                                    'stop px4_offboard_bridge NOW')
        ms = np.array(self.solve_ms) if self.solve_ms else np.zeros(1)
        er = np.array(self.err) if self.err else np.zeros(1)
        d = self.tracker.obs.d_hat
        u = self.last_u
        held = '' if self.streaming else ' (withheld: PX4 not in Offboard)'
        servo = ' | yaw servo off'
        if self.servo is not None:
            servo = ' | yaw servo ticks ' + ' '.join(f'{k} {n}' for k, n in
                                                     self.servo_ticks.items())
            self.servo_ticks = dict.fromkeys(self.servo_ticks, 0)
        self.get_logger().info(
            f'{self.state} cmd_in={self.n_cmd} bad={self.n_cmd_bad} sp_out={self.n_sp}{held} '
            f'fallbacks={self.n_fallback} armed={self.armed} offboard={self.offboard} | '
            f'err mean {er.mean():.2f} max {er.max():.2f} m | '
            f'solve p50 {np.median(ms):.2f} max {ms.max():.2f} ms '
            f'iters {self.tracker.mpc.last_iters} | '
            f'u [{u[0]:+.2f} {u[1]:+.2f} {u[2]:+.2f}] '
            f'd_hat [{d[0]:+.2f} {d[1]:+.2f} {d[2]:+.2f}]{servo}')
        self.solve_ms.clear()
        self.err.clear()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Px4MpcTracker()
    except RuntimeError as e:
        print(f'px4_mpc_tracker: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
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
