#!/usr/bin/env python3
"""The M3 gate: does Offboard engage, arm, hold, and hand back to the pilot?

Watches PX4 and our own setpoint stream while **you** drive the sequence from the RC.
It commands nothing and cannot arm anything — it is a recorder, so the human stays in
the loop for every state change that spins a motor.

    source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
    python3 scripts/px4/m3_gate_check.py [seconds] [csv]

**PROPS OFF** for the bench gate. Airborne (flight-test P5) it is the same recorder.

Run it with **no `/goal_pose`** set, so SUPER stays in WAIT_GOAL and publishes nothing.
The bridge then sits in HOLD with a frozen setpoint, which is exactly what "position
holds on setpoint" means on a bench: the commanded point must not drift or run away
while the vehicle is armed and cannot move. Trajectory following is an in-flight test,
not this one.

**It refuses to record without the LiDAR chain up.** `px4_offboard_bridge` derives its
setpoint from PX4's *own* `vehicle_local_position`, so it publishes a perfectly healthy
frozen setpoint even when FAST-LIO is dead and EKF2 is holding on optical flow alone.
That is not a hypothetical: on 2026-09-04 a full 60 s airborne P5 was recorded 79 s after
the M1 stack had been Ctrl-C'd, and every visible number looked fine. So the preflight
below checks `/lidar_slam/odom` has a publisher and exits rather than record a run that
cannot mean what you will think it means. Set `ALLOW_NO_LIDAR=1` to record a deliberate
flow-only baseline anyway.

The sequence to perform while it runs:
  1. (script running, both bridges up, SUPER idle)
  2. Engage Offboard      -- the RC Offboard switch (`RC_MAP_OFFB_SW`), or `offboard_engage.py`
  3. Arm                  -- RC
  4. Leave it armed       -- watch that the setpoint stays put and Offboard is not lost
  5. Flip the Offboard switch off   -- the manual-takeover half of the gate
  6. Disarm               -- RC

**Props off you get about 10 seconds armed, not more.** `COM_DISARM_PRFLT` (10 s here)
auto-disarms if PX4 never detects a takeoff, and with props off it never can. That is why
the pass threshold below is 5 s of armed-in-Offboard rather than something longer: the
platform caps it. Repeat the engage/arm cycle a few times within the window if you want
more evidence — the script accumulates across episodes. Setting `COM_DISARM_PRFLT = 0`
disables the timeout for bench work, but it is a real safety feature (it is what stops a
forgotten aircraft sitting armed), so put it back before flight.

**Expect the motors to spin up to roughly `MPC_THR_HOVER` (0.41 here) the moment you arm in
Offboard.** That is not a fault and not an error-driven ramp: a position setpoint hands PX4
ownership of throttle, and its baseline output is hover thrust. Measured on the signed-off run,
the altitude error was under 7 mm throughout, so the thrust was pure feedforward. PROPS OFF.

Airborne, leave enough recording time to flip the Offboard switch back off *inside* the
window — the takeover check can only see what it recorded.

Resolution note: PX4 publishes `vehicle_control_mode` at ~2 Hz, so mode-transition times
here are accurate to about ±0.5 s. That is enough to show the takeover *happened* and
that nothing fought it; it is not a millisecond latency measurement, and this script does
not pretend otherwise. (Corrected 2026-09-28: commander also publishes it immediately on a
mode change, Commander.cpp:1931, and every Offboard edge in the 09-21/26 bags arrived
within ~15 ms. The ±0.5 s above is pessimistic; the printed "(+-0.5s)" is kept.)

Since 2026-09-28 the offboard bridge sends `trajectory_setpoint` only while PX4 reports
Offboard, so "setpoint stream seen: no" before the engage is normal, and the frozen point
is the first setpoint after the engage when you arm first.
"""
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (EstimatorStatusFlags, TrajectorySetpoint,
                          VehicleControlMode, VehicleLocalPosition,
                          VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/m3_gate.csv'

ODOM_TOPIC = '/lidar_slam/odom'
PREFLIGHT_SEC = 4.0       # discovery + a first look at the flags before t0
MIN_ODOM_HZ = 5.0         # FAST-LIO runs at 10 Hz; half of it is already broken

NAV = {v: k[len('NAVIGATION_STATE_'):]
       for k, v in vars(VehicleStatus).items() if k.startswith('NAVIGATION_STATE_')}


class Spans:
    """Contiguous stretches where a per-sample condition held.

    Counts alone cost a session: "317 dead-reckoning samples" is unactionable, while
    "one span, 2.9-9.2 s, i.e. the climb" names the cause.
    """

    def __init__(self):
        self.n = 0
        self.spans = []
        self._open = None

    def update(self, t, cond):
        if cond:
            self.n += 1
            if self._open is None:
                self._open = [t, t]
            else:
                self._open[1] = t
        else:
            self.close()

    def close(self):
        if self._open is not None:
            self.spans.append(tuple(self._open))
            self._open = None

    def describe(self, noun):
        self.close()
        if not self.n:
            return f'0 {noun}'
        where = ', '.join(f'{a:.1f}-{b:.1f}s' for a, b in self.spans[:4])
        if len(self.spans) > 4:
            where += f', +{len(self.spans) - 4} more'
        return f'{self.n} {noun} in {len(self.spans)} span(s): {where}'


class GateCheck(Node):

    def __init__(self):
        super().__init__('m3_gate_check')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.t0 = time.time()
        self.recording = False
        self.events = []          # (t, text)
        self.rows = []            # csv
        self.armed = self.offboard = False
        self.nav = None
        self.sp = None
        self.sp_t = None          # time.time() of the last setpoint heard
        self.lpos = None
        self.seen_armed_offboard = False
        self.armed_offboard_secs = 0.0
        self.sp_when_engaged = None
        self.max_sp_drift = 0.0
        self.invalid = Spans()
        self.deadreck = Spans()
        self.no_ev = Spans()
        self.resets = set()
        self.first_reset = None
        self.lost_offboard_while_armed = False
        self.takeover = None      # (t, from_mode, to_mode)
        self._last_tick = time.time()
        # LiDAR chain liveness -- the thing whose absence is otherwise invisible.
        self.ev_pos = None        # None = no estimator_status_flags seen yet
        self.n_flags = 0
        self.n_odom = 0
        self.last_odom = None

        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode',
                                 self.on_mode, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self.on_status, qos)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1', self.on_pos, qos)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self.on_sp, qos)
        self.create_subscription(EstimatorStatusFlags,
                                 '/fmu/out/estimator_status_flags', self.on_flags, qos)
        # FAST-LIO creates this publisher with default QoS (RELIABLE, depth 20,
        # laserMapping.cpp:943). Subscribe BEST_EFFORT anyway: a best-effort
        # subscriber matches a reliable publisher too, so this cannot miss either way.
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST, depth=10))

    def t(self):
        return time.time() - self.t0

    def log(self, text):
        self.events.append((self.t(), text))
        print(f'  [{self.t():6.1f}s] {text}', flush=True)

    def start_recording(self):
        """Zero the clock and the counters so they cover exactly the recorded window."""
        self.t0 = self._last_tick = time.time()
        self.rows.clear()
        self.invalid = Spans()
        self.deadreck = Spans()
        self.no_ev = Spans()
        self.armed_offboard_secs = 0.0
        self.n_odom = 0
        self.recording = True

    # ---- inputs -----------------------------------------------------------

    def on_mode(self, m):
        now = time.time()
        if self.armed and self.offboard:
            self.armed_offboard_secs += now - self._last_tick
        self._last_tick = now

        was = (self.armed, self.offboard)
        self.armed, self.offboard = m.flag_armed, m.flag_control_offboard_enabled
        if (self.armed, self.offboard) == was or not self.recording:
            return

        self.log(f'PX4: armed={self.armed} offboard={self.offboard}')
        if self.armed and self.offboard and not self.seen_armed_offboard:
            self.seen_armed_offboard = True
            # Since 2026-09-28 the bridge sends setpoints only in Offboard. Engaged before
            # arming, one is already flowing; armed first, none has been sent yet and the
            # last one heard (if any) is from an earlier episode. Only a setpoint from the
            # last 0.2 s is the frozen point; otherwise on_sp takes the first one to come.
            recent = self.sp is not None and time.time() - self.sp_t < 0.2
            self.sp_when_engaged = list(self.sp.position) if recent else None
            self.log('  >> ARMED IN OFFBOARD -- the gate condition. Setpoint frozen at '
                     + (f'{[round(v, 2) for v in self.sp_when_engaged]}' if recent
                        else '(the next setpoint)'))
        # Offboard dropping while still armed is the takeover (or a failsafe).
        if was == (True, True) and self.armed and not self.offboard:
            # Destination is filled in by on_status: vehicle_status is 2 Hz and lags
            # vehicle_control_mode, so self.nav still reads OFFBOARD at this instant.
            self.takeover = (self.t(), 'OFFBOARD', None)
            self.log('  >> LEFT OFFBOARD while armed (manual takeover)')

    def on_status(self, m):
        if m.nav_state != self.nav:
            prev, self.nav = self.nav, m.nav_state
            if prev is not None and self.recording:
                self.log(f'PX4 mode: {NAV.get(prev, prev)} -> {NAV.get(m.nav_state, m.nav_state)}')
            if self.takeover and self.takeover[2] is None \
                    and m.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.takeover = (self.takeover[0], 'OFFBOARD',
                                 NAV.get(m.nav_state, m.nav_state))

    def on_flags(self, m):
        self.n_flags += 1
        if self.recording and self.ev_pos is True and not m.cs_ev_pos:
            self.log('  !! EKF2 STOPPED fusing external vision (cs_ev_pos False)')
        self.ev_pos = bool(m.cs_ev_pos)

    def on_odom(self, m):
        self.n_odom += 1
        self.last_odom = time.time()

    def on_pos(self, m):
        self.lpos = m
        if not self.recording:
            return
        t = self.t()
        self.invalid.update(t, not (m.xy_valid and m.z_valid))
        self.deadreck.update(t, m.dead_reckoning)
        self.no_ev.update(t, self.ev_pos is not True)
        key = (m.xy_reset_counter, m.z_reset_counter)
        if self.resets and key not in self.resets and self.first_reset is None:
            self.first_reset = t
            self.log(f'  !! EKF reset (xy={m.xy_reset_counter} z={m.z_reset_counter})')
        self.resets.add(key)
        sp = tuple(self.sp.position) if self.sp is not None else (float('nan'),) * 3
        self.rows.append((t, int(self.armed), int(self.offboard),
                          self.nav if self.nav is not None else -1,
                          *sp, m.x, m.y, m.z,
                          int(m.xy_valid), int(m.z_valid), int(m.dead_reckoning),
                          m.eph, m.epv,
                          -1 if self.ev_pos is None else int(self.ev_pos)))

    def on_sp(self, m):
        self.sp = m
        self.sp_t = time.time()
        if self.seen_armed_offboard and self.sp_when_engaged is None and self.armed \
                and self.offboard:
            self.sp_when_engaged = list(m.position)
            self.log(f'  >> first setpoint in Offboard: {[round(v, 2) for v in m.position]}')
        # While armed in Offboard with SUPER idle, the commanded point must not move.
        if self.seen_armed_offboard and self.sp_when_engaged and self.armed and self.offboard:
            d = max(abs(a - b) for a, b in zip(m.position, self.sp_when_engaged))
            self.max_sp_drift = max(self.max_sp_drift, d)


def spin(n, seconds):
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)


def main():
    rclpy.init()
    n = GateCheck()

    print(f'\nm3_gate_check -- preflight ({PREFLIGHT_SEC:.0f}s discovery)...\n')
    spin(n, PREFLIGHT_SEC)

    pubs = n.count_publishers(ODOM_TOPIC)
    ev = {None: 'unknown (no estimator_status_flags yet)',
          True: 'YES', False: 'NO'}[n.ev_pos]
    print(f'  {ODOM_TOPIC:<24} publishers={pubs}  msgs seen={n.n_odom}')
    print(f'  EKF2 fusing external vision (cs_ev_pos): {ev}')
    print(f'  PX4 setpoint stream seen: {"yes" if n.sp is not None else "no"}'
          '  (none until Offboard is engaged -- normal since 2026-09-28)')

    if pubs == 0 or n.n_odom == 0:
        allow = os.environ.get('ALLOW_NO_LIDAR') == '1'
        print('\n' + '!' * 68)
        print('  THE LIDAR CHAIN IS DOWN -- nothing is publishing ' + ODOM_TOPIC + '.')
        print('  px4_offboard_bridge takes its setpoint from PX4\'s own estimate, so it')
        print('  will look perfectly healthy and this recording would prove nothing about')
        print('  FAST-LIO, the odom bridge, or the LiDAR->EKF2 chain. EKF2 would be')
        print('  holding on optical flow + range instead.')
        print('')
        print('  Start the stack first:   RVIZ=false bash scripts/run.sh')
        print('                           ros2 run super_bringup px4_odom_bridge.py')
        print('')
        print('  (ALLOW_NO_LIDAR=1 to record a deliberate flow-only baseline anyway.)')
        print('!' * 68)
        if not allow:
            n.destroy_node()
            rclpy.shutdown()
            return 2
        print('\n  ALLOW_NO_LIDAR=1 -- recording a flow-only baseline.\n')

    n.start_recording()
    print(f'\nm3_gate_check -- recording {DUR:.0f}s.\n')
    print('  Perform: Offboard switch ON -> arm -> hold -> Offboard switch OFF -> disarm')
    print('  (props off: ~10 s max armed per cycle, COM_DISARM_PRFLT. Repeat if time allows.)')
    print('  Flip Offboard back OFF *before* the window ends or the takeover check cannot see it.\n')
    try:
        spin(n, DUR)
    except KeyboardInterrupt:
        print('\n  (stopped early)')

    odom_hz = n.n_odom / DUR

    if OUT and n.rows:
        with open(OUT, 'w') as f:
            f.write('t,armed,offboard,nav_state,sp_n,sp_e,sp_d,est_n,est_e,est_d,'
                    'xy_valid,z_valid,dead_reck,eph,epv,ev_pos\n')
            for r in n.rows:
                f.write(','.join(f'{v:.4f}' if isinstance(v, float) else str(v)
                                 for v in r) + '\n')

    print('\n' + '=' * 68)
    print('M3 GATE RESULT')
    print('=' * 68)
    checks = []
    checks.append(('armed while in Offboard', n.seen_armed_offboard,
                   f'{n.armed_offboard_secs:.0f}s armed in Offboard'))
    # 5 s, not 10: COM_DISARM_PRFLT caps a props-off armed episode at ~10 s, so a longer
    # threshold would fail runs that are actually fine. Accumulated across episodes.
    checks.append(('held Offboard >=5 s armed (cumulative)', n.armed_offboard_secs >= 5.0,
                   f'{n.armed_offboard_secs:.0f}s'))
    checks.append(('setpoint stayed frozen (no runaway)', n.max_sp_drift < 0.5,
                   f'max drift {n.max_sp_drift:.3f} m'))
    checks.append(('manual takeover observed', n.takeover is not None,
                   f'-> {n.takeover[2] or "?"} at t={n.takeover[0]:.1f}s (+-0.5s)'
                   if n.takeover else 'never left Offboard while armed'))
    checks.append(('position valid throughout', n.invalid.n == 0,
                   n.invalid.describe('invalid samples')))
    checks.append(('never dead-reckoning', n.deadreck.n == 0,
                   n.deadreck.describe('samples')))
    checks.append(('no EKF resets', n.first_reset is None,
                   'none' if n.first_reset is None else f'at t={n.first_reset:.1f}s'))
    # The two below are what tell you the run was about the LiDAR at all.
    checks.append((f'LiDAR odometry live ({ODOM_TOPIC})', odom_hz >= MIN_ODOM_HZ,
                   f'{n.n_odom} msgs, {odom_hz:.1f} Hz'))
    checks.append(('EKF2 fused external vision throughout', n.no_ev.n == 0 and n.n_flags > 0,
                   n.no_ev.describe('samples without cs_ev_pos')
                   if n.n_flags else 'no estimator_status_flags received'))
    for name, ok, detail in checks:
        print(f'  [{"PASS" if ok else "FAIL"}] {name:<38} {detail}')

    n_fail = sum(1 for _, ok, _ in checks if not ok)
    print(f'\n{len(checks) - n_fail}/{len(checks)} checks passed')
    if n.rows:
        print(f'{len(n.rows)} samples -> {OUT}')
    if not n.seen_armed_offboard:
        print('\nNever saw armed+offboard together -- the gate did not run. Was Offboard '
              'engaged before arming?')
    n.destroy_node()
    rclpy.shutdown()
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
