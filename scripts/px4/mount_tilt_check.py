#!/usr/bin/env python3
"""Measure how far the MID360 is tilted relative to the flight controller.

    source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
    python3 scripts/px4/mount_tilt_check.py [seconds]     # default 20

Keep the rig COMPLETELY STILL for the whole run.

Why this works
--------------
`EKF2_EV_CTRL=11` fuses external-vision **yaw only** (bit 8) -- EKF2's roll and
pitch come from the FC's own IMU, with no vision contribution. FAST-LIO's roll
and pitch come from the MID360's IMU. Both are gravity-referenced and completely
independent, so the difference between them IS the physical mounting tilt.

Why yaw is printed but must be IGNORED
--------------------------------------
EKF2's yaw is *defined* by the EV yaw this pipeline sends it, so the two agree by
construction no matter how the LiDAR is bolted on. Gravity gives you roll and
pitch for free; nothing on a stationary bench gives you yaw. A yaw mounting
offset is a MECHANICAL check -- is the MID360 square to the drone's nose? -- and
if it is ever non-zero that angle has to go into `px4_odom_bridge`, because PX4
has no external-vision rotation parameter at all (the complete set is
EKF2_EV_CTRL / _DELAY / _NOISE_MD / _POS_X / _POS_Y / _POS_Z / _QMIN).

Why it matters
--------------
A yaw mounting error leaves positions consistent -- they are in the world frame --
but makes EKF2's attitude wrong by that angle, so the vehicle accelerates
off-axis under position control. That is a spiral, not a wobble.

Reference: measured 2026-09-02 on this airframe as roll -1.55 deg, pitch +0.09 deg.
"""
import math
import os
import statistics
import sys

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
# Above this, stop and look at the mounting rather than trying to compensate.
TILT_WARN_DEG = 3.0


def rpy_deg(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TiltCheck(Node):

    def __init__(self):
        super().__init__('mount_tilt_check')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.lio = None
        self.px4 = None
        self.rows = []
        self.create_subscription(Odometry, '/lidar_slam/odom', self.on_lio, 10)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 self.on_px4, px4_qos)
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(0.1, self.tick)

    def on_lio(self, m):
        o = m.pose.pose.orientation
        # FLU -> FRD, the same flip px4_odom_bridge applies, so both are FRD/NED.
        self.lio = rpy_deg(o.w, o.x, -o.y, -o.z)

    def on_px4(self, m):
        self.px4 = rpy_deg(m.q[0], m.q[1], m.q[2], m.q[3])

    def tick(self):
        if self.get_clock().now().nanoseconds / 1e9 - self.t0 > DUR:
            self.report()
            sys.stdout.flush()
            os._exit(0)
        if self.lio and self.px4:
            self.rows.append((self.lio, self.px4))

    def report(self):
        if not self.rows:
            print('NO DATA -- is the M1 stack up and the FC link live?')
            return
        print(f'samples {len(self.rows)} over {DUR:.0f}s '
              f'(rig must have been STATIONARY)\n')
        worst = 0.0
        for i, name in enumerate(('roll', 'pitch', 'yaw')):
            lio = [r[0][i] for r in self.rows]
            px4 = [r[1][i] for r in self.rows]
            diff = [px4[k] - lio[k] for k in range(len(lio))]
            mean = statistics.mean(diff)
            sd = statistics.pstdev(diff)
            note = ''
            if name == 'yaw':
                note = '   <- IGNORE: circular, EV yaw defines it'
            else:
                worst = max(worst, abs(mean))
                if sd > 0.5:
                    note = '   <- noisy, was the rig really still?'
            print(f'  {name:5s}  FAST-LIO {statistics.mean(lio):+8.3f}  '
                  f'PX4(own IMU) {statistics.mean(px4):+8.3f}  '
                  f'diff {mean:+7.3f} deg (sd {sd:.3f}){note}')

        print()
        if worst > TILT_WARN_DEG:
            print(f'  ==> TILT {worst:.2f} deg EXCEEDS {TILT_WARN_DEG} deg. '
                  'Re-seat the MID360; do not compensate in software.')
        else:
            print(f'  ==> tilt OK ({worst:.2f} deg, threshold {TILT_WARN_DEG}). '
                  'Yaw alignment is still a MECHANICAL check -- see docstring.')


def main():
    rclpy.init()
    rclpy.spin(TiltCheck())


if __name__ == '__main__':
    main()
