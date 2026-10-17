#!/usr/bin/env bash
#
# Bring up the M1 stack: Livox driver -> FAST-LIO2 -> SUPER (ROG-Map).
# The driver runs in the background; the M1 launch runs in the foreground.
# Ctrl-C stops both.
#
#   bash scripts/run.sh            # with RViz
#   RVIZ=false bash scripts/run.sh # headless
#   SUPER_CONFIG=follow_test_ros2.yaml RVIZ=false bash scripts/run.sh   # M5 follow flights
#
# SUPER_CONFIG picks fsm_node's config under super_planner/config/. Default click_smooth_ros2.yaml
# (bench, click goals, 0.1 m/s). The DTP follower launches refuse to start a second fsm_node on
# top of this stack, so the config chosen HERE is the one a follow flight plans on.
#
# CPU pinning (M5.3, 2026-09-13): FAST-LIO runs on FASTLIO_CPUS (default 6,7) and the Livox
# driver + fsm_node on STACK_CPUS (default 0-5); scripts/dtp_follower.sh puts the DTP tree on the
# same 0-5. So the estimator keeps two cores whatever perception and the planner do -- the
# 2026-09-12 field stack ran at load 6.0 with odometry ages past EKF2_DELAY_MAX. taskset only;
# no root, and a thread started later inherits it. FASTLIO_CPUS= (empty) disables both.
#
set -eo pipefail   # not -u: ROS setup.bash references unset vars on a cold shell

SUPER_WS="${SUPER_WS:-$HOME/super_ws}"
WS_LIVOX="${WS_LIVOX:-$HOME/ws_livox}"
SUPER_CONFIG="${SUPER_CONFIG:-click_smooth_ros2.yaml}"
# fsm_node resolves config_name against its compile-time SOURCE dir, not install/. A missing
# name must fail here with a sentence, not as an fsm_node exception after FAST-LIO is up.
SUPER_CFG_PATH="$SUPER_WS/src/SUPER/super_planner/config/$SUPER_CONFIG"
if [ ! -f "$SUPER_CFG_PATH" ]; then
    echo "REFUSING to start: SUPER_CONFIG=$SUPER_CONFIG does not exist ($SUPER_CFG_PATH)." >&2
    exit 1
fi

FASTLIO_CPUS="${FASTLIO_CPUS-6,7}"
FASTLIO_PUB_MODE="${FASTLIO_PUB_MODE:-SYNCHRONOUS}"   # rmw_fastrtps publish mode for FAST-LIO (see the launch file)
# nonblocking (default since 2026-09-13): FAST-LIO's UDP sends never block on a slow Wi-Fi viewer --
# docs/fact-fastdds-blocking-send.md. FASTLIO_DDS_PROFILE= (empty) restores FastDDS defaults.
FASTLIO_DDS_PROFILE="${FASTLIO_DDS_PROFILE-nonblocking}"   # '' | nonblocking | <xml path>
STACK_CPUS="${STACK_CPUS-0-5}"
if [ -n "$FASTLIO_CPUS" ] && [ "$(nproc)" -lt 8 ]; then
    echo "==> $(nproc) cores: not pinning (FASTLIO_CPUS=$FASTLIO_CPUS assumes 8)"
    FASTLIO_CPUS=""; STACK_CPUS=""
fi
[ -n "$FASTLIO_CPUS" ] || STACK_CPUS=""
PIN_STACK=()
[ -n "$STACK_CPUS" ] && PIN_STACK=(taskset -c "$STACK_CPUS")

source /opt/ros/humble/setup.bash
source "$WS_LIVOX/install/setup.bash"
source "$SUPER_WS/install/setup.bash"

# Refuse to start a second copy of anything that owns the LiDAR or the estimate (G0.2).
# On 2026-09-12 a second Livox driver took the MID360's stream from the first, FAST-LIO got IMU
# and no scans for 225 s, and the aircraft was armed on a pose 3.74 km wrong
# (docs/fact-fastlio-divergence-2026-09-12.md). Matched on the EXECUTABLE (argv[0]) -- never
# pgrep -f, which matches this very shell's command line.
running() {
    local p exe
    for p in /proc/[0-9]*; do
        exe=$(tr '\0' '\n' < "$p/cmdline" 2>/dev/null | head -n1) || continue
        [ "${exe##*/}" = "$1" ] && echo "${p#/proc/}"
    done
    return 0
}
for node in livox_ros_driver2_node fastlio_mapping fsm_node; do
    pids=$(running "$node" | tr '\n' ' ')
    if [ -n "$pids" ]; then
        echo "REFUSING to start: $node is already running (pid $pids)." >&2
        echo "A second copy fights the first for the sensor. Stop the running stack first" >&2
        echo "(dashboard STOP ALL, or Ctrl-C its terminal), then start this again." >&2
        exit 1
    fi
done

echo "==> starting Livox MID360 driver (CustomMsg)${STACK_CPUS:+ on cpus $STACK_CPUS}"
"${PIN_STACK[@]}" ros2 launch livox_ros_driver2 msg_MID360_launch.py &
DRV=$!
trap 'echo; echo "stopping..."; kill $DRV 2>/dev/null || true' EXIT
sleep 4

echo "==> starting M1 stack (FAST-LIO${FASTLIO_CPUS:+ on cpus $FASTLIO_CPUS} + SUPER fsm_node on $SUPER_CONFIG${STACK_CPUS:+, cpus $STACK_CPUS})"
# FAST-LIO publishes /cloud_registered (world) + /Odometry (remapped -> /lidar_slam/odom);
# SUPER ingests both via ROG-Map. No simulator. FSM idles in INIT until a /goal_pose.
# ros2 launch rejects `name:=` with an empty value, so optional arguments are passed only when set.
LAUNCH_ARGS=(rviz:="${RVIZ:-true}" config_name:="$SUPER_CONFIG" fastlio_pub_mode:="$FASTLIO_PUB_MODE")
[ -n "$FASTLIO_CPUS" ] && LAUNCH_ARGS+=(fastlio_cpus:="$FASTLIO_CPUS")
[ -n "$STACK_CPUS" ] && LAUNCH_ARGS+=(fsm_cpus:="$STACK_CPUS")
[ -n "$FASTLIO_DDS_PROFILE" ] && LAUNCH_ARGS+=(fastlio_dds_profile:="$FASTLIO_DDS_PROFILE")
ros2 launch super_bringup m1_super_lidar.launch.py "${LAUNCH_ARGS[@]}"
