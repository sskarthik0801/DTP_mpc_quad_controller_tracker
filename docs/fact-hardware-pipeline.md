---
name: super-human-follow-hardware
description: Hardware/odometry pipeline and flight-controller choice for the SUPER human-following drone on the Orin Nano
metadata: 
  node_type: memory
  type: project
  originSessionId: 44902c0c-5771-4b30-82cd-58ae6d892916
---

The SUPER planner on this Jetson Orin Nano is being turned into a vision-based human-following drone (plan: `/home/ideas-ad/super_ws/ImplementationDoc.md`, milestones M0–M5).

M1 state-estimation pipeline: Livox MID360 (LiDAR IP `192.168.1.154`; Jetson host `192.168.1.50/24`) → `livox_ros_driver2` in `~/ws_livox` (`xfer_format=1`, CustomMsg) → FAST-LIO2 (hku-mars `FAST_LIO` **ROS2 branch**, at `super_ws/src/FAST_LIO_ROS2`) → SUPER ROG-Map via `/cloud_registered` + `/lidar_slam/odom`. The M1 launch is `super_bringup/launch/m1_super_lidar.launch.py`.

**Corrections (2026-07-29) — two claims in the original version of this note were wrong:**

1. **The flight controller runs PX4 v1.17.0 + uXRCE-DDS, NOT ArduCopter + MAVROS.** Confirmed live over MAVLink (`MAV_AUTOPILOT_PX4`, firmware `v1.17.0`, `MAV_TYPE_QUADROTOR`). M2 is therefore `/lidar_slam/odom` → `px4_odom_bridge` → `/fmu/in/vehicle_visual_odometry`, which is **built and hardware-verified**. `ImplementationDoc.md` was right and this note was wrong.
2. **The wired iface is not `enP8p1s0` on this board, and the IP does persist.** It is **`eno1`** on the Orin Nano Engineering Reference devkit, and `scripts/network_setup.sh` creates a persistent NetworkManager profile — do **not** use `ip addr add`, which NetworkManager flushes within ~1 min (the `bind failed` failure mode).

Also note the Jetson runs a preinstalled **ARK OS** image whose services own both FC serial links; use the MAVLink UDP endpoint 14569 instead of opening them. See `uart_bringup_log.md` and [[fast-lio-ros2-cpp17-build-fix]].
