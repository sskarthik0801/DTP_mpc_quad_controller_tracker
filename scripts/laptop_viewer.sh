#!/usr/bin/env bash
#
# Remote RViz viewer for the SUPER M1 stack running on the Jetson.
# Run on a LAPTOP (Ubuntu 22.04 + ROS 2 Humble) that is on the SAME Wi-Fi as the Jetson.
# No build required.
#
#   bash scripts/laptop_viewer.sh
#
# All parameters below can be overridden via env vars. They must MATCH the Jetson:
#   ROS_DOMAIN_ID     - DDS domain (Jetson uses 0)
#   JETSON_IP         - Jetson Wi-Fi IP, used only for the reachability check
#   RMW_IMPLEMENTATION- DDS vendor (Jetson uses the default rmw_fastrtps_cpp)
#   DDS_WHITELIST=0   - do NOT bind DDS to the one interface facing the Jetson (default 1: bind
#                       it, so a laptop with Tailscale up stops advertising its 100.x address and
#                       the Jetson stops sending every datagram twice -- see
#                       docs/fact-fastdds-blocking-send.md). DDS_IFACE_IP=<ip> forces the address
#                       when the automatic pick (the route to JETSON_IP) is wrong.
#   DTP=1             - human-following view: rviz/dtp_viewer.rviz (the M1 view plus the camera
#                       picture with perception's person boxes, dtp_viz's people/lock markers and
#                       the follow goal), and starts scripts/laptop_image_decode.py, which turns
#                       the Jetson's JPEG into the raw image RViz needs. Same as
#                       scripts/laptop_dtp_viewer.sh. On the Jetson the boxes come from
#                       perception_node + dtp_image_overlay (dtp_system.launch.py starts both).
#
set -eo pipefail   # not -u: ROS setup.bash references unset vars (AMENT_TRACE_SETUP_FILES) on a cold shell

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
JETSON_IP="${JETSON_IP:-192.168.8.130}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
DTP="${DTP:-0}"
_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$DTP" = 1 ]; then
  RVIZ_CFG="${RVIZ_CFG:-$_repo/rviz/dtp_viewer.rviz}"
else
  RVIZ_CFG="${RVIZ_CFG:-$_repo/rviz/m1_viewer.rviz}"
fi
DECODE_PY="$_repo/scripts/laptop_image_decode.py"

if [ ! -d /opt/ros/humble ]; then
  echo "ERROR: ROS 2 Humble not found. Run: bash scripts/laptop_setup.sh" >&2
  exit 1
fi

# Match the Jetson's DDS configuration so discovery works across the LAN.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION
echo "==> ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  (localhost_only=0)"

# Bind DDS to the ONE interface that reaches the Jetson (2026-09-14). With Tailscale up this
# laptop otherwise advertises its 100.x address as a second DDS locator and the Jetson's FastDDS
# sends every datagram to both -- twice the Wi-Fi load for the same cloud, the second copy via
# tailscaled and possibly a relay on the internet. FastDDS 2.6 has no sender-side fix, so it is
# done here: scripts/laptop_dds_profile.sh writes an interfaceWhiteList profile and it is exported
# for the discovery check below AND carried into RViz's allow-list environment.
DDS_WHITELIST="${DDS_WHITELIST:-1}"
if [ "$DDS_WHITELIST" = 1 ]; then
  _jip="$JETSON_IP"
  case "$_jip" in *[!0-9.]*) _jip="$(getent ahostsv4 "$JETSON_IP" 2>/dev/null | awk 'NR==1{print $1}')";; esac
  DDS_IFACE_IP="${DDS_IFACE_IP:-$(ip -4 route get "${_jip:-192.168.8.130}" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -1)}"
  if [ -n "$DDS_IFACE_IP" ]; then
    FASTRTPS_DEFAULT_PROFILES_FILE="$(bash "$_repo/scripts/laptop_dds_profile.sh" "$DDS_IFACE_IP")"
    export FASTRTPS_DEFAULT_PROFILES_FILE
    echo "==> DDS bound to $DDS_IFACE_IP only ($FASTRTPS_DEFAULT_PROFILES_FILE); DDS_WHITELIST=0 to undo"
    echo "    NOTE: a ros2 command in ANOTHER terminal on this laptop will not see these nodes unless it"
    echo "    exports the same FASTRTPS_DEFAULT_PROFILES_FILE (a whitelisted and a default participant"
    echo "    on one host do not exchange data -- measured on the Jetson 2026-09-14)."
  else
    echo "    WARNING: no route found to $JETSON_IP, so DDS is left on every interface -- with"
    echo "    Tailscale up the Jetson will send every datagram twice. Set DDS_IFACE_IP=<laptop wifi ip>."
  fi
  unset _jip
fi

# Snap apps leak snap-provided glibc ahead of the system libs, which makes the apt
# rviz2 die with:
#   symbol lookup error: /snap/core20/.../libpthread.so.0:
#   undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
#
# This bit twice. The first fix only scrubbed LD_LIBRARY_PATH and LD_PRELOAD, and a
# 2026-09-02 bring-up crashed with BOTH of those already clean -- so the leak reaches
# the dynamic linker by some other route (GTK/GIO module paths and XDG_DATA_DIRS are
# the usual suspects, and a snap-launched terminal exports a pile of them). Guessing
# which variable it is has now failed once, so stop guessing: report anything that
# mentions /snap/ for diagnosis, then launch RViz under an explicit allow-list
# environment where none of them exist. `env -i` was verified to fix it; the previous
# revision recorded that in a comment but shipped `env -u LD_PRELOAD`, which is not
# the same thing and does not work.
_snappy="$(env | grep -oE '^[A-Za-z_][A-Za-z0-9_]*=' | tr -d '=' | while read -r _n; do
             case "${!_n-}" in */snap/*) echo "  ${_n}";; esac; done || true)"
if [ -n "${_snappy}" ]; then
  echo "==> these environment variables point into /snap/ and are why rviz2 crashes:"
  echo "${_snappy}"
  echo "    (RViz is launched below with a clean environment, so they cannot reach it)"
fi
unset _snappy _n

echo "==> reachability: ping $JETSON_IP"
if ping -c2 -W1 "$JETSON_IP" >/dev/null 2>&1; then
  echo "    ok"
else
  echo "    WARNING: cannot reach $JETSON_IP. Same Wi-Fi? Some APs block client-to-client"
  echo "    traffic (AP/client isolation) — then DDS discovery cannot work."
fi

echo "==> discovered topics (waiting up to 8s)..."
_topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"
if echo "$_topics" | grep -E "cloud_registered|lidar_slam|rog_map|^/tf" ; then
  echo "    discovery OK"
else
  echo "    No SUPER topics seen yet. Common fixes:"
  echo "      - confirm the Jetson stack is running (scripts/run.sh there)"
  echo "      - laptop firewall: sudo ufw disable   (or allow UDP for DDS)"
  echo "      - Wi-Fi multicast blocked -> ask for the FastDDS discovery-server fallback"
  echo "    Launching RViz anyway; displays will populate if/when discovery completes."
fi

if [ "$DTP" = 1 ]; then
  echo "==> DTP view"
  if ! python3 -c "import cv2" 2>/dev/null && ! python3 -c "import PIL" 2>/dev/null; then
    echo "ERROR: the camera panel needs OpenCV or Pillow to decode the Jetson's JPEG:" >&2
    echo "       sudo apt install python3-opencv" >&2
    exit 1
  fi
  for _t in /dtp/image_annotated/compressed /dtp/detections_2d /dtp/viz; do
    if echo "$_topics" | grep -qx "$_t"; then
      echo "    seen: $_t"
    else
      echo "    NOT seen yet: $_t"
    fi
  done
  echo "    The picture needs, on the Jetson: the camera, perception_node (the boxes) and"
  echo "    dtp_image_overlay (draws them) -- dtp_system.launch.py starts all three. Without"
  echo "    perception the panel shows the live camera under an amber 'no boxes' banner."
fi
unset _topics _t

echo "==> launching RViz ($RVIZ_CFG)"
# Allow-list, not deny-list. Start from nothing (`env -i`), carry over only what a GUI
# app genuinely needs, and re-source ROS inside so AMENT_PREFIX_PATH / LD_LIBRARY_PATH /
# PYTHONPATH are rebuilt from /opt/ros alone. Nothing snap-provided survives this, which
# is the whole point -- the failure mode is a variable we did not think to name.
_clean=(env -i
  HOME="$HOME"
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
  ROS_LOCALHOST_ONLY=0
  RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"
  RVIZ_CFG="$RVIZ_CFG"
  DTP="$DTP"
  DECODE_PY="$DECODE_PY"
)
# Only forward these if they are actually set -- `env -i FOO=` sets an empty value,
# which some of them (DISPLAY especially) treat as different from unset.
for _keep in USER LOGNAME DISPLAY XAUTHORITY WAYLAND_DISPLAY XDG_RUNTIME_DIR \
             XDG_SESSION_TYPE QT_QPA_PLATFORM LANG LC_ALL TERM FASTRTPS_DEFAULT_PROFILES_FILE \
             __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME \
             LIBGL_ALWAYS_SOFTWARE MESA_GL_VERSION_OVERRIDE; do
  [ -n "${!_keep-}" ] && _clean+=("${_keep}=${!_keep}")
done
# `bash -c`, not `bash -lc`: a login shell would re-read the profile scripts that put
# the snap paths there in the first place.
# The drone-radius ball is published HERE, on the laptop -- never on the Jetson. RViz
# renders locally, so the marker costs the Jetson exactly nothing, and `frame_locked: true`
# makes RViz re-transform it from /tf every rendered frame. One latched message is enough
# for the ball to track the aircraft forever; the 1 Hz repeat only covers a late RViz start.
#
# It is drawn at `lidar_imu`, which is the MID360's internal IMU -- NOT the drone centre.
# The FC body origin is ~7 cm away, so read the ball as "the planner's robot_r sphere",
# not as a precise airframe outline.
DRONE_R="${DRONE_R:-0.2}"          # metres, matches super_planner robot_r
_diam=$(awk -v r="$DRONE_R" 'BEGIN{printf "%.3f", r*2}')
echo "==> drone radius ball: r=${DRONE_R} m on /viz/drone_radius (published from this laptop)"
_clean+=("MARKER_MSG={header: {frame_id: lidar_imu}, ns: drone, id: 0, type: 2, action: 0, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, scale: {x: ${_diam}, y: ${_diam}, z: ${_diam}}, color: {r: 0.25, g: 0.9, b: 1.0, a: 0.35}, frame_locked: true}")

exec "${_clean[@]}" bash -c '
source /opt/ros/humble/setup.bash
ros2 topic pub -r 1 --qos-durability transient_local --qos-reliability reliable \
  /viz/drone_radius visualization_msgs/msg/Marker "$MARKER_MSG" >/dev/null 2>&1 &
_mk=$!
_dec=
if [ "$DTP" = 1 ]; then
  # JPEG from the Jetson -> raw Image on this laptop, for the "Person boxes" panel.
  python3 "$DECODE_PY" &
  _dec=$!
fi
# RViz in the foreground so the helpers die with it instead of being orphaned.
trap "kill $_mk $_dec 2>/dev/null" EXIT INT TERM
rviz2 -d "$RVIZ_CFG"
'
