#!/usr/bin/env python3
"""Bench gate for the px4_mpc_tracker NODE, with ROS stubbed out. No ROS, no px4_msgs, no FC.

    python3 scripts/px4/mpc_tracker_node_check.py        # prints PASS/FAIL per check, exit 0 = all pass
    NODE=<path> python3 scripts/px4/mpc_tracker_node_check.py   # test another copy of the node

mpc_tracker_sim.py tests the controller maths. This tests the node around it -- the state
machine, the frames, what goes on the wire in each state, and the yaw servo stage -- by
loading the REAL super_bringup/scripts/px4_mpc_tracker.py with rclpy, px4_msgs,
mars_quadrotor_msgs, geometry_msgs, std_msgs and rcl_interfaces replaced by small stand-ins,
calling its callbacks and its tick directly on a fake clock, and reading what it published.
A simple lagged point-mass plant closes the loop where a check needs the aircraft to move.

What it cannot see: real DDS (QoS matching, discovery), real px4_msgs field types, and PX4
itself. Those are the sandbox and props-off steps in docs/mpc-tracker-integration.md.

Run it after any change to px4_mpc_tracker.py, mpc_tracker_core.py or yaw_servo.py.
"""
import importlib.util
import math
import os
import sys
import types

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# NODE=<path> tests another copy, e.g. the installed one:
#   NODE=~/super_ws/install/super_bringup/lib/super_bringup/px4_mpc_tracker.py
NODE_PATH = os.path.expanduser(os.environ.get(
    'NODE', os.path.join(REPO, 'super_bringup', 'scripts', 'px4_mpc_tracker.py')))
NAN = float('nan')

# ---------------------------------------------------------------------------------------
# Stand-ins for the ROS modules the node imports.
# ---------------------------------------------------------------------------------------


class Msg:
    """A message: attributes set by the constructor's defaults, anything else allowed."""
    _defaults = {}

    def __init__(self):
        for k, v in self._defaults.items():
            setattr(self, k, v() if callable(v) else v)


class Vec(Msg):
    _defaults = {'x': 0.0, 'y': 0.0, 'z': 0.0}


class OffboardControlMode(Msg):
    _defaults = dict(timestamp=0, position=False, velocity=False, acceleration=False,
                     attitude=False, body_rate=False, thrust_and_torque=False,
                     direct_actuator=False)


class TrajectorySetpoint(Msg):
    _defaults = dict(timestamp=0, position=lambda: [NAN] * 3, velocity=lambda: [NAN] * 3,
                     acceleration=lambda: [NAN] * 3, jerk=lambda: [NAN] * 3,
                     yaw=NAN, yawspeed=NAN)


class VehicleControlMode(Msg):
    _defaults = dict(flag_armed=False, flag_control_offboard_enabled=False)


class VehicleLocalPosition(Msg):
    _defaults = dict(x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0, heading=0.0,
                     xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True)


class PositionCommand(Msg):
    TRAJECTORY_STATUS_EMER = 2
    _defaults = dict(position=Vec, velocity=Vec, acceleration=Vec, jerk=Vec,
                     yaw=0.0, yaw_dot=0.0, trajectory_flag=1)


class Stamp(Msg):
    _defaults = dict(sec=0, nanosec=0)


class Header(Msg):
    _defaults = dict(stamp=Stamp, frame_id='world')


class PointStamped(Msg):
    _defaults = dict(header=Header, point=Vec)


class MultiArrayDimension:
    def __init__(self, label='', size=0, stride=0):
        self.label, self.size, self.stride = label, size, stride


class Float32MultiArray(Msg):
    _defaults = dict(layout=lambda: types.SimpleNamespace(dim=[]), data=list)


class Duration:
    def __init__(self, ns):
        self.nanoseconds = ns


class Time:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return Duration(self.nanoseconds - other.nanoseconds)


class Clock:
    ns = 1_000_000_000_000

    def now(self):
        return Time(Clock.ns)


CLOCK = Clock()
FOREIGN_PUBLISHERS = {}           # topic -> count of publishers that are not the node's


class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level):
        return lambda m: self.lines.append((level, m))

    def __getattr__(self, level):
        if level in ('debug', 'info', 'warn', 'error', 'fatal'):
            return self._log(level)
        raise AttributeError(level)


class Publisher:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)


class Param:
    def __init__(self, v):
        self.value = v


OVERRIDES = {}                    # parameter overrides for the next node constructed


class Node:
    def __init__(self, name):
        self._name = name
        self._params = {}
        self._logger = Logger()
        self.pubs = {}
        self.subs = {}

    def declare_parameter(self, name, default, descriptor=None):
        self._params[name] = OVERRIDES.get(name, default)

    def get_parameter(self, name):
        return Param(self._params[name])

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return CLOCK

    def create_publisher(self, _type, topic, _qos):
        self.pubs[topic] = Publisher(topic)
        return self.pubs[topic]

    def create_subscription(self, _type, topic, cb, _qos):
        self.subs[topic] = cb

    def create_timer(self, _period, _cb):
        return None

    def count_publishers(self, topic):
        return FOREIGN_PUBLISHERS.get(topic, 0) + (1 if topic in getattr(self, 'pubs', {})
                                                   else 0)

    def destroy_node(self):
        pass


class InvalidParameterTypeException(Exception):
    pass


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    qos_names = ('DurabilityPolicy', 'HistoryPolicy', 'ReliabilityPolicy')
    enum = types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1, VOLATILE=0, TRANSIENT_LOCAL=1,
                                 KEEP_LAST=0)
    mod('rclpy', init=lambda args=None: None, shutdown=lambda: None, ok=lambda: False,
        spin=lambda n: None, spin_once=lambda n, timeout_sec=0: None)
    mod('rclpy.node', Node=Node)
    mod('rclpy.qos', QoSProfile=lambda **kw: kw, **{n: enum for n in qos_names})
    mod('rclpy.exceptions', InvalidParameterTypeException=InvalidParameterTypeException)
    mod('rcl_interfaces')
    mod('rcl_interfaces.msg', ParameterDescriptor=lambda **kw: kw)
    mod('px4_msgs')
    mod('px4_msgs.msg', OffboardControlMode=OffboardControlMode,
        TrajectorySetpoint=TrajectorySetpoint, VehicleControlMode=VehicleControlMode,
        VehicleLocalPosition=VehicleLocalPosition)
    mod('mars_quadrotor_msgs')
    mod('mars_quadrotor_msgs.msg', PositionCommand=PositionCommand)
    mod('geometry_msgs')
    mod('geometry_msgs.msg', PointStamped=PointStamped)
    mod('std_msgs')
    mod('std_msgs.msg', Float32MultiArray=Float32MultiArray,
        MultiArrayDimension=MultiArrayDimension)


def _load_node_module():
    _install_stubs()
    spec = importlib.util.spec_from_file_location('px4_mpc_tracker', NODE_PATH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)

    # The start-up loop waits 2 s of wall time for DDS discovery; make it a few fast passes.
    class FastTime:
        perf_counter = staticmethod(__import__('time').perf_counter)
        _t = [0.0]

        @staticmethod
        def monotonic():
            FastTime._t[0] += 0.5
            return FastTime._t[0]
    m.time = FastTime
    return m


# ---------------------------------------------------------------------------------------
# Harness
# ---------------------------------------------------------------------------------------

M = _load_node_module()
TICK_NS = 20_000_000              # 50 Hz
SP = '/fmu/in/trajectory_setpoint'
OCM = '/fmu/in/offboard_control_mode'


class Rig:
    """One node plus a lagged point-mass plant that feeds it vehicle_local_position."""

    def __init__(self, **overrides):
        OVERRIDES.clear()
        OVERRIDES.update(overrides)
        self.node = M.Px4MpcTracker()
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.heading = 0.0
        self.valid = True
        self.move = False         # integrate the plant from the acceleration setpoints
        self.lpos()

    def mode(self, armed, offboard):
        m = VehicleControlMode()
        m.flag_armed, m.flag_control_offboard_enabled = armed, offboard
        self.node.subs['/fmu/out/vehicle_control_mode'](m)

    def lpos(self):
        m = VehicleLocalPosition()
        m.x, m.y, m.z = map(float, self.p)
        m.vx, m.vy, m.vz = map(float, self.v)
        m.heading = self.heading
        m.xy_valid = m.z_valid = m.v_xy_valid = m.v_z_valid = self.valid
        self.node.subs['/fmu/out/vehicle_local_position_v1'](m)

    def cmd(self, p_flu, v_flu=(0, 0, 0), a_flu=(0, 0, 0), yaw=0.0, yaw_dot=0.0):
        m = PositionCommand()
        for vec, val in ((m.position, p_flu), (m.velocity, v_flu), (m.acceleration, a_flu)):
            vec.x, vec.y, vec.z = map(float, val)
        m.yaw, m.yaw_dot = yaw, yaw_dot
        self.node.subs['/planning/pos_cmd'](m)

    def target(self, x_flu, y_flu, frame='world'):
        m = PointStamped()
        m.point.x, m.point.y = x_flu, y_flu
        m.header.frame_id = frame
        m.header.stamp.sec = Clock.ns // 1_000_000_000
        m.header.stamp.nanosec = Clock.ns % 1_000_000_000
        self.node.subs['/planning/yaw_target'](m)

    def tick(self, n=1, every=None):
        """n ticks. `every` is called before each, e.g. to stream pos_cmd."""
        for _ in range(n):
            if every is not None:
                every()
            Clock.ns += TICK_NS
            sp_before = len(self.node.sp_pub.msgs)
            self.node.on_tick()
            if self.move and len(self.node.sp_pub.msgs) > sp_before:
                u = np.array(self.node.sp_pub.msgs[-1].acceleration, float)
                if np.all(np.isfinite(u)):
                    dt = TICK_NS / 1e9
                    self.a += (dt / 0.14) * (u - self.a)
                    self.p += self.v * dt + 0.5 * self.a * dt * dt
                    self.v += self.a * dt
            self.lpos()

    def sps(self):
        return self.node.sp_pub.msgs

    def ocms(self):
        return self.node.ocm_pub.msgs


RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append(bool(ok))
    print(f'  {"PASS" if ok else "FAIL"}  {name}' + (f'  -- {detail}' if detail else ''))


def nan3(v):
    return all(math.isnan(x) for x in v)


def fin3(v):
    return all(math.isfinite(x) for x in v)


# ---------------------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------------------

def c_refuses_second_publisher():
    FOREIGN_PUBLISHERS[SP] = 1
    try:
        M.Px4MpcTracker()
        check('N1 refuses to start beside another setpoint publisher', False, 'it started')
    except RuntimeError as e:
        check('N1 refuses to start beside another setpoint publisher', 'never both' in str(e))
    finally:
        FOREIGN_PUBLISHERS.clear()


def c_idle_and_pos_hold():
    r = Rig()
    r.cmd((1, 2, 3))                              # SUPER commanding, PX4 not in Offboard
    r.tick(25, every=lambda: r.cmd((1, 2, 3)))
    check('N2 not in Offboard: heartbeat only, no setpoint',
          len(r.sps()) == 0 and len(r.ocms()) == 25 and all(o.position for o in r.ocms())
          and r.node.state == 'IDLE', f'{len(r.sps())} setpoints, {len(r.ocms())} heartbeats')
    r.p[:] = (0.4, -0.3, -1.2)
    r.heading = 0.7
    r.lpos()
    r.mode(armed=False, offboard=True)
    r.tick(3)
    s = r.sps()[-1] if r.sps() else None
    ok = (s is not None and r.node.state == 'POS_HOLD' and np.allclose(s.position, r.p)
          and nan3(s.acceleration) and abs(s.yaw - 0.7) < 1e-9 and r.ocms()[-1].position)
    check('N3 Offboard + disarmed: POS_HOLD, position setpoint at the estimate', ok)


def c_hold_at_engage():
    r = Rig()
    r.p[:] = (2.0, 1.0, -1.2)
    r.lpos()
    r.mode(armed=True, offboard=True)
    r.move = True
    r.tick(1)
    s = r.sps()[-1]
    ok = (r.node.state == 'HOLD' and nan3(s.position) and nan3(s.velocity)
          and fin3(s.acceleration) and r.ocms()[-1].acceleration and not r.ocms()[-1].position)
    check('N4 armed + Offboard, SUPER silent: HOLD, acceleration-only setpoint', ok)
    r.tick(150)
    drift = np.linalg.norm(r.p - (2.0, 1.0, -1.2))
    check('N5 HOLD holds the engage point (3 s closed loop)', drift < 0.05, f'drift {drift:.3f} m')


def c_frames():
    r = Rig()
    r.mode(armed=True, offboard=True)
    # SUPER (FLU) says: 0.5 m LEFT (+y) and 0.5 m UP (+z), yaw +0.6 rad.
    r.tick(1, every=lambda: r.cmd((0, 0.5, 0.5), yaw=0.6, yaw_dot=0.2))
    s = r.sps()[-1]
    u = s.acceleration
    ok = r.node.state == 'TRACK' and u[1] < -0.1 and u[2] < -0.1 and abs(u[0]) < 0.05
    check('N6 FLU -> NED: left is -y, up is -z', ok, f'u = [{u[0]:+.2f} {u[1]:+.2f} {u[2]:+.2f}]')
    ok = abs(s.yaw - (-0.6)) < 1e-9 and abs(s.yawspeed - (-0.2)) < 1e-9
    check('N7 yaw -> -yaw, yaw_dot -> -yaw_dot (NOT pi/2 - yaw)', ok,
          f'yaw {s.yaw:+.3f} yawspeed {s.yawspeed:+.3f}')


def c_far_setpoint():
    r = Rig()
    r.mode(armed=True, offboard=True)
    r.move = True
    far = (3.0, 0.0, 0.0)                          # the 2026-09-21 engage offset
    us, speeds = [], []

    def stream():
        r.cmd(far)
    for _ in range(250):
        r.tick(1, every=stream)
        us.append(np.hypot(*r.sps()[-1].acceleration[:2]))
        speeds.append(np.linalg.norm(r.v))
    prm = r.node.tracker.prm
    check('N8 far setpoint: horizontal command inside the tilt disc',
          max(us) <= prm.a_xy_max + 1e-6, f'max |u_xy| {max(us):.2f} <= {prm.a_xy_max}')
    check('N9 far setpoint: a bounded approach, not a lunge', max(speeds) < 1.25,
          f'peak {max(speeds):.2f} m/s, final error {np.linalg.norm(r.p - (3, 0, 0)):.2f} m')


def c_stale_cmd():
    r = Rig()
    r.mode(armed=True, offboard=True)
    r.tick(5, every=lambda: r.cmd((0.2, 0, 0)))
    r.tick(20)                                    # 0.4 s with no pos_cmd > cmd_timeout 0.3
    check('N10 pos_cmd stale: TRACK -> HOLD', r.node.state == 'HOLD'
          and fin3(r.sps()[-1].acceleration) and r.sps()[-1].yawspeed == 0.0)


def c_fallback():
    r = Rig()
    r.mode(armed=True, offboard=True)
    r.tick(5, every=lambda: r.cmd((0.2, 0, 0)))
    r.valid = False
    r.tick(2)
    s = r.sps()[-1]
    ok = (r.node.state == 'FALLBACK' and fin3(s.position) and nan3(s.acceleration)
          and r.ocms()[-1].position)
    check('N11 invalid estimate: FALLBACK to a PX4 position setpoint', ok)
    r.valid = True
    r.tick(3, every=lambda: r.cmd((0.2, 0, 0)))
    check('N12 estimate back: recovers to TRACK', r.node.state == 'TRACK'
          and fin3(r.sps()[-1].acceleration))
    r2 = Rig()
    r2.mode(armed=True, offboard=True)
    r2.tick(2)
    Clock.ns += 300_000_000                       # 0.3 s with no lpos > lpos_timeout 0.2
    r2.node.on_tick()
    check('N13 stale estimate (> lpos_timeout): FALLBACK', r2.node.state == 'FALLBACK')


def c_takeover_and_silence():
    r = Rig()
    r.mode(armed=True, offboard=True)
    r.tick(10, every=lambda: r.cmd((0.2, 0, 0)))
    n = len(r.sps())
    r.mode(armed=True, offboard=False)            # pilot flips ch11
    r.tick(25, every=lambda: r.cmd((0.2, 0, 0)))
    logs = [m for _l, m in r.node.get_logger().lines]
    check('N14 takeover: setpoints stop at once, heartbeat continues',
          len(r.sps()) == n and r.node.state == 'IDLE' and len(r.ocms()) == 35
          and any('setpoint stream stopped' in m for m in logs))
    r.mode(armed=True, offboard=True)
    r.tick(150, every=lambda: r.cmd((0.2, 0, 0)))  # 3 s with no further mode report
    warned = any(lvl == 'warn' and 'still sending setpoints' in m
                 for lvl, m in r.node.get_logger().lines)
    check('N15 a silent mode report never stops the stream (warned after 2 s)',
          len(r.sps()) == n + 150 and warned)


def c_servo():
    off = Rig()
    check('N16 yaw servo off: no yaw_target subscription, no debug publisher',
          '/planning/yaw_target' not in off.node.subs and '/bridge/yaw_servo' not in off.node.pubs)
    try:
        Rig(yaw_servo_enable=True, yaw_servo_max_rate=-5.0)
        check('N17 a bad yaw servo parameter stops the node', False, 'it started')
    except SystemExit:
        check('N17 a bad yaw servo parameter stops the node', True)

    on = Rig(yaw_servo_enable=True)
    ref = Rig()
    for r in (on, ref):
        r.mode(armed=True, offboard=True)
    yaws, accs_on, accs_ref = [], [], []
    for _ in range(100):                          # 2 s
        on.target(0.0, 6.0)                       # 6 m to the LEFT (FLU) = -90 deg FRD
        for r in (on, ref):
            r.tick(1, every=lambda r=r: r.cmd((0.1, 0, 0), yaw=0.0))
        yaws.append(on.sps()[-1].yaw)
        accs_on.append(on.sps()[-1].acceleration)
        accs_ref.append(ref.sps()[-1].acceleration)
    # Both rigs advance the one shared clock, so each node ticks every 2 * TICK_NS.
    rate = max(abs(b - a) for a, b in zip(yaws, yaws[1:])) / (2 * TICK_NS / 1e9)
    check('N18 yaw servo on: heading turns to the target (-90 deg FRD)',
          abs(yaws[-1] - (-math.pi / 2)) < math.radians(2), f'final {math.degrees(yaws[-1]):+.1f} deg')
    check('N19 yaw servo slew limit holds', rate <= math.radians(90.0) + 1e-6,
          f'max {math.degrees(rate):.1f} deg/s')
    check('N20 yaw servo never touches the MPC acceleration',
          np.allclose(np.array(accs_on), np.array(accs_ref), atol=1e-12))
    dbg = on.node.pubs['/bridge/yaw_servo'].msgs
    n = len(dbg)
    on.mode(armed=True, offboard=False)
    on.tick(10)
    check('N21 /bridge/yaw_servo only while engaged', n == 100 and len(dbg) == n)


def main():
    print(f'px4_mpc_tracker node check (ROS stubbed): {NODE_PATH}')
    for c in (c_refuses_second_publisher, c_idle_and_pos_hold, c_hold_at_engage, c_frames,
              c_far_setpoint, c_stale_cmd, c_fallback, c_takeover_and_silence, c_servo):
        c()
    n_ok = sum(RESULTS)
    print(f'\n{"PASS" if n_ok == len(RESULTS) else "FAIL"} ({n_ok}/{len(RESULTS)})')
    return 0 if n_ok == len(RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
