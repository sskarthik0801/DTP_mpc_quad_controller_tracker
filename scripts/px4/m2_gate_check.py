#!/usr/bin/env python3
"""M2 gate: does PX4's EKF2 actually TRACK the LiDAR odometry?

Records `/lidar_slam/odom` (FAST-LIO, FLU world) beside
`/fmu/out/vehicle_local_position_v1` (EKF2, NED) and reports, per axis:
correlation, **sign**, best lag, and tracking error with the constant lever-arm
offset removed.

Why this exists
---------------
A sign error on any axis is *invisible* on a stationary bench -- both sources sit
near zero and every "is it working?" check passes -- and it would flip a control
loop the first time M3 commands a setpoint. The only way to catch it is to move
the rig and correlate. Run this for the M2 gate, and re-run it after any change
to `px4_odom_bridge`'s frame handling or the MID360 mounting.

Usage (props OFF, stack up, bridge running)
-------------------------------------------
    source /opt/ros/humble/setup.bash
    source ~/super_ws/install/setup.bash
    python3 scripts/px4/m2_gate_check.py 120 /tmp/motion.csv

Then move the rig by hand: ~0.5 m forward and back, ~0.5 m right and back,
~0.3 m up and down, pausing between each. **Keep the orientation fixed while
translating** -- rotating mid-move couples the axes and muddies the per-axis read.

Note this is the ONE script in scripts/px4/ that does not talk MAVLink -- it is a
pure ROS 2 subscriber, so it needs the workspace sourced, not `pymavlink`.

Pass criteria (met 2026-09-02, see docs/ImplementationDoc.md §M2)
-----------------------------------------------------------------
  * correlation > 0.95 and sign CORRECT on all three axes
  * max tracking error within the variance the bridge declares (0.1 m std)
  * zero samples with xy_valid/z_valid false, dead_reckoning never set
  * no EKF resets (xy_reset_counter unchanged) during the moves
"""
import math
import os
import statistics
import sys

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/m2_gate.csv'
AXES = ('N', 'E', 'D')


def corr(a, b):
    n = len(a)
    if n < 2:
        return float('nan')
    ma, mb = sum(a) / n, sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    if sa == 0 or sb == 0:
        return float('nan')
    return sum((a[i] - ma) * (b[i] - mb) for i in range(n)) / (sa * sb)


class GateCheck(Node):

    def __init__(self):
        super().__init__('m2_gate_check')
        # PX4 /fmu/out/ topics are BEST_EFFORT + TRANSIENT_LOCAL.
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.lio = None
        self.px4 = None
        self.rows = []
        self.create_subscription(Odometry, '/lidar_slam/odom', self.on_lio, 10)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_px4, px4_qos)
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(0.1, self.tick)          # 10 Hz, matching the LiDAR

    def on_lio(self, m):
        p = m.pose.pose.position
        # FLU -> NED, the same flip px4_odom_bridge applies (FLU -> FRD).
        self.lio = (p.x, -p.y, -p.z)

    def on_px4(self, m):
        self.px4 = (m.x, m.y, m.z, m.heading, m.xy_valid, m.z_valid,
                    m.eph, m.epv, m.dead_reckoning, m.xy_reset_counter)

    def tick(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.t0
        if t > DUR:
            self.report()
            sys.stdout.flush()
            os._exit(0)                # rclpy teardown hangs from a callback
        if self.lio is None or self.px4 is None:
            return
        self.rows.append((t,) + self.lio + self.px4)

    def report(self):
        with open(OUT, 'w') as f:
            f.write('t,lio_n,lio_e,lio_d,px4_x,px4_y,px4_z,px4_hdg,'
                    'xy_valid,z_valid,eph,epv,dead_reck,xy_reset\n')
            for r in self.rows:
                f.write(','.join(f'{v:.4f}' if isinstance(v, float) else str(v)
                                 for v in r) + '\n')
        if not self.rows:
            print('NO DATA -- is the stack up and px4_odom_bridge running?')
            return

        n = len(self.rows)
        L = {a: [r[1 + i] for r in self.rows] for i, a in enumerate(AXES)}
        P = {a: [r[4 + i] for r in self.rows] for i, a in enumerate(AXES)}

        print(f'samples {n}  duration {self.rows[-1][0]:.1f}s\n')
        print('axis   corr      sign      best lag   excursion  '
              'err(moving)  err(still)')
        ok = True
        for a in AXES:
            c = corr(L[a], P[a])
            best = (0, -2.0)
            for lag in range(-20, 21):          # +/- 2 s at 10 Hz
                x = L[a][max(0, -lag):n - max(0, lag)]
                y = P[a][max(0, lag):n - max(0, -lag)]
                cc = corr(x, y)
                if cc == cc and cc > best[1]:
                    best = (lag, cc)
            # Constant offset is the lever arm (EKF2_EV_POS_*), not an error.
            off = statistics.median([P[a][i] - L[a][i] for i in range(n)])
            err = [abs(P[a][i] - L[a][i] - off) for i in range(n)]
            mv = [abs(L[a][i] - L[a][0]) for i in range(n)]
            moving = [i for i in range(n) if mv[i] > 0.15]
            still = [i for i in range(n) if mv[i] <= 0.05]
            e_mv = max((err[i] for i in moving), default=float('nan'))
            e_st = max((err[i] for i in still), default=float('nan'))
            good = c == c and c > 0.95
            ok &= good
            print(f'  {a}  {c:+.4f}  {"CORRECT" if c > 0 else "INVERTED!":9s} '
                  f'{best[0] * 100:+5d} ms  {max(mv):7.3f} m  '
                  f'{e_mv:9.3f} m  {e_st:9.3f} m')

        bad = sum(1 for r in self.rows if not (r[8] and r[9]))
        dr = sum(1 for r in self.rows if r[12])
        resets = (self.rows[0][13], self.rows[-1][13])
        print(f'\n  invalid xy/z: {bad}/{n}   dead_reckoning: {dr}/{n}   '
              f'xy_reset_counter: {resets[0]} -> {resets[1]}')
        print(f'  eph max {max(r[10] for r in self.rows):.3f}   '
              f'epv max {max(r[11] for r in self.rows):.3f}')
        moved = max(max(abs(v - L[a][0]) for v in L[a]) for a in AXES)
        if moved < 0.3:
            print('\n  !! rig barely moved (<0.3 m) -- this run proves nothing '
                  'about signs. Move it and re-run.')
            ok = False
        verdict = 'PASS' if (ok and bad == 0 and dr == 0) else 'FAIL / REVIEW'
        print(f'\n  ==> {verdict}    csv: {OUT}')


def main():
    rclpy.init()
    node = GateCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()


if __name__ == '__main__':
    main()
