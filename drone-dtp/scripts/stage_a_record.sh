#!/usr/bin/env bash
# Stage A recording. Start BEFORE takeoff, stop AFTER landing. Prints the PID to stop it with.
#
# Records everything needed to answer Stage A's question offline -- "what would the follow stack
# have commanded, and did it make sense on a moving platform?" -- which means BOTH stacks: what
# we computed (/dtp/shadow/*, /mux/state, /dtp/*), what the aircraft actually did (/planning/*
# from SUPER, /fmu/out/* from PX4, /fmu/in/trajectory_setpoint from the bridge), and the raw
# sensor data to re-run perception offline (compressed colour, raw depth, both camera_infos,
# the registered LiDAR cloud, IMU, odometry, TF).
#
# Goes under ~/dtp_datasets, never /tmp: /tmp is cleared on every reboot and has eaten one
# 194 s recording already (see ~/dtp_datasets/README.md).
#
# Budget: ~6 MB/s depth + ~2 MB/s colour + ~3 MB/s cloud -> ~12 MB/s -> ~7 GB for a 10 min
# sortie. stage_a_preflight.py checks for 5 GB free; have 10.
set -euo pipefail

STAMP=$(date +%Y%m%d_%H%M)
OUT="${1:-$HOME/dtp_datasets/${STAMP}_stageA}"
LOG="$HOME/dtp_datasets/${STAMP}_stageA.record.log"

if [ -e "$OUT" ]; then
  echo "refusing: $OUT already exists" >&2
  exit 1
fi

# shellcheck disable=SC1090
# set +u around the source: dtp_env_jetson.sh (via ROS setup.bash) reads unset variables, and
# under set -u that is a FATAL shell error that || true cannot catch -- the script died here
# silently on 2026-09-12 at the field, from a cold shell. It had only ever dry-run from a shell
# that already carried the env.
set +u; source "$HOME/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh" >/dev/null 2>&1 || true; set -u

setsid nohup ros2 bag record -o "$OUT" \
  /dtp/shadow/follow_goal /dtp/shadow/yaw_target \
  /dtp/targets /dtp/follow_target /dtp/follow_goal /dtp/lock_state \
  /dtp/sensors_ok /dtp/sensor_health /dtp/perception_ok /dtp/abort /mux/state \
  /planning/follow_goal /planning/yaw_target /planning/goal_status /planning/pos_cmd \
  /goal_pose /dtp/nav_goal \
  /fmu/out/vehicle_status_v1 /fmu/out/vehicle_control_mode /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_command_ack /fmu/in/trajectory_setpoint /fmu/in/vehicle_command \
  /fmu/in/vehicle_visual_odometry \
  /lidar_slam/odom /tf /tf_static /livox/imu /cloud_registered_body \
  /camera/color/image_raw/compressed /camera/color/camera_info \
  /camera/depth/image_raw /camera/depth/camera_info \
  > "$LOG" 2>&1 < /dev/null &
PID=$!
sleep 3
if ! kill -0 "$PID" 2>/dev/null; then
  echo "recorder died immediately -- see $LOG" >&2
  tail -5 "$LOG" >&2
  exit 1
fi
echo "recording -> $OUT"
echo "recorder pid $PID   (stop AFTER landing with:  kill -INT $PID   -- by PID, never pkill -f)"
echo "$PID" > "$OUT.pid"
