#!/usr/bin/env bash
#
# Gemini 336L colour publisher -- the camera process the web GCS starts.
#
#   bash scripts/gemini_camera.sh                       # 640x480 @ 15, /gemini
#   COLOR_WIDTH=1280 COLOR_HEIGHT=720 COLOR_FPS=30 bash scripts/gemini_camera.sh
#   bash scripts/gemini_camera.sh serial_number:=CPC64630007M
#
# Ctrl-C stops it.  Started from the dashboard it is stopped by process group
# (scripts/gcs/supervisor.py), which is the same thing done from another shell.
#
# It publishes /gemini/color/image_raw/compressed -- the camera's own MJPEG,
# forwarded as CompressedImage with no decode.  scripts/gcs/camera.py hands
# those bytes straight to the browser, and RViz on the laptop can read them
# too (rviz/gemini_viewer.rviz).  Depth, IR, IMU and point clouds stay off:
# this is a viewing feed, not the M4 perception input.
#
# ONE PROCESS ONLY.  A Gemini can be opened by a single reader; a second one
# dies with `uvc_open ... Return Code: -6`.  Check who has it before starting:
#     python3 scripts/gcs/camera.py
#
set -eo pipefail   # not -u: ROS setup.bash references unset vars on a cold shell
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${SUPER_WS:-$HOME/super_ws}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if ! ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
    echo 'Orbbec driver missing. See docs/geminitest.md for installation.' >&2
    exit 1
fi

# The repository copy of the launch file, so no colcon build is needed to
# change a default here -- same reason run.sh reads config/ from the checkout.
exec ros2 launch "$REPO_DIR/super_bringup/launch/gemini_camera.launch.py" \
    "camera_name:=${CAMERA_NAME:-gemini}" \
    "color_width:=${COLOR_WIDTH:-640}" \
    "color_height:=${COLOR_HEIGHT:-480}" \
    "color_fps:=${COLOR_FPS:-15}" \
    ${SERIAL_NUMBER:+"serial_number:=$SERIAL_NUMBER"} \
    "$@"
