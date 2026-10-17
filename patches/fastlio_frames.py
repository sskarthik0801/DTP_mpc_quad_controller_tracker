#!/usr/bin/env python3
"""
Patch FAST_LIO_ROS2 laserMapping.cpp: rename the TF frames to say what they are.

Upstream publishes odometry as camera_init -> body. Both names are actively
misleading on this airframe:

  "body"        is NOT the drone body. It is the MID360's internal IMU -- the
                frame FAST-LIO's ESIKF actually estimates. It sits 4.41 cm from
                the LiDAR's optical origin (mid360.yaml extrinsic_T) and some
                unknown distance from the FC, which is the *real* body origin
                as far as PX4 is concerned (EKF2_IMU_POS_* = 0). Reading "body"
                as the vehicle body is exactly the mistake that produces a wrong
                EKF2_EV_POS_* lever arm.
  "camera_init" has no camera behind it. It is the pose of that IMU frame at the
                first scan, gravity-aligned, heading arbitrary.

  camera_init -> lidar_init      body -> lidar_imu

Nothing downstream reads these names: SUPER/ROG-Map ignores frame_id and TF
entirely (it treats the odom pose as world-frame), rviz/m1_viewer.rviz uses only
Fixed Frame: world, and px4_odom_bridge never inspects frame_id. The static
world -> lidar_init TF in super_bringup keeps RViz working, and
scripts/px4/fake_odom_pub.py is kept in sync.

Idempotent; safe to re-run. Run AFTER fastlio_executor.py (they touch different
lines, but the executor patch anchors on unrenamed text near line 651).
"""
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/ideas-ad/super_ws/src/FAST_LIO_ROS2/src/laserMapping.cpp"

RENAMES = [("camera_init", "lidar_init"), ("body", "lidar_imu")]

s = open(SRC).read()
orig = s

for old, new in RENAMES:
    # Only string literals -- never bare identifiers. `body` in particular is a
    # very common variable name in this file (p_body, p_body_lidar, ...), so an
    # unquoted replace would destroy it.
    lit_old, lit_new = '"%s"' % old, '"%s"' % new
    n = s.count(lit_old)
    if n:
        s = s.replace(lit_old, lit_new)
        print('  ok    %-14s -> %-14s (%d site%s)'
              % (old, new, n, "" if n == 1 else "s"))
    elif lit_new in s:
        print('  --    %-14s -> %-14s (already applied)' % (old, new))
    else:
        print('  SKIP  %-14s -> %-14s (not found)' % (old, new))

if s != orig:
    open(SRC, "w").write(s)
    print("PATCHED %s" % SRC)
else:
    print("no change (already patched)")

# The topic /cloud_registered_body keeps its upstream NAME -- renaming a topic
# would break anyone's rviz config and is not what this patch is for. Only the
# frame_id inside it changes.
leftover = [l for l in ('"body"', '"camera_init"') if l in open(SRC).read()]
if leftover:
    sys.exit("ERROR: frame literals still present: %s" % ", ".join(leftover))
