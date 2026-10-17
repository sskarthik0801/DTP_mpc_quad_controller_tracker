#!/usr/bin/env bash
#
# The DTP follower, riding on an already-running SUPER stack -- what the web
# GCS's "DTP follower" row starts.  It is shell 4 of docs/m5.1-one-graph-runbook.md.
#
#   DTP_MODE=observe bash scripts/dtp_follower.sh
#   DTP_MODE=live SUPER_CONFIG=follow_test_ros2.yaml bash scripts/dtp_follower.sh
#
# observe  observe_only:=true.  Goals and yaw targets go to /dtp/shadow/*, where
#          nothing subscribes, and command_mux's NAV_LAND abort is off.  It
#          commands nothing.
# live     observe_only:=false.  command_mux publishes /planning/follow_goal and
#          holds a /fmu/in/vehicle_command publisher armed to send NAV_LAND.
#
# There is NO default mode: which one you get has to be said out loud.
#
# lidar:=false super:=false, always.  The follower never starts its own Livox
# driver, FAST-LIO or fsm_node -- a second Livox driver is how the aircraft was
# armed on a 3.74 km error on 2026-09-12 (docs/fact-fastlio-divergence-2026-09-12.md).
# Start the stack first (scripts/run.sh, or the GCS's Stack row).
#
# DTP_CPUS (default 0-5) keeps the whole DTP tree -- perception, the tracker,
# the Orbbec driver -- off FAST-LIO's two cores (run.sh's FASTLIO_CPUS, 6,7).
# DTP_CPUS= (empty) runs unpinned.
#
# Ctrl-C stops it.  Started from the dashboard it is stopped by process group
# (scripts/gcs/supervisor.py), which is the same thing done from another shell.
#
set -eo pipefail   # not -u: ROS setup.bash references unset vars on a cold shell

SUPER_WS="${SUPER_WS:-$HOME/super_ws}"
FOLLOW_CONFIG=follow_test_ros2.yaml

# Via the workspace symlink, NOT the repo path: sourced from the repo path it
# finds no workspace (CLAUDE.md, drone-dtp trap 4).
DTP_ENV="$SUPER_WS/src/drone-dtp/ros2_ws/dtp_env_jetson.sh"
if [ ! -f "$DTP_ENV" ]; then
    echo "REFUSING to start: $DTP_ENV not found." >&2
    echo "Is $SUPER_WS/src/drone-dtp a symlink to this repo's drone-dtp/, and is" >&2
    echo "this repo on a branch that has drone-dtp/ (checking out one without it" >&2
    echo "deletes the folder)?" >&2
    exit 1
fi

case "${DTP_MODE:-}" in
    observe) OBSERVE_ONLY=true;  VIZ_GOAL=/dtp/shadow/follow_goal ;;
    live)    OBSERVE_ONLY=false; VIZ_GOAL=/planning/follow_goal ;;
    *)
        echo "REFUSING to start: DTP_MODE must be 'observe' or 'live' (got '${DTP_MODE:-}')." >&2
        exit 1 ;;
esac

# shellcheck disable=SC1090
source "$DTP_ENV"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ "$DTP_MODE" = live ]; then
    # The GCS already read fsm_node's config off its launch parameters before
    # it would start this.  Ask the RUNNING node as well, over ROS: two sources
    # that have to agree, like the FC-reboot interlock.  A live follow on the
    # click config would fly outside the M5.2 first-flight envelope.
    echo "==> live mode: asking /fsm_node which config it loaded"
    loaded="$(timeout 20 ros2 param get /fsm_node config_name 2>/dev/null \
              | sed -n 's/^String value is: //p' || true)"
    if [ "$loaded" != "$FOLLOW_CONFIG" ]; then
        echo "REFUSING live mode: /fsm_node reports config_name='${loaded:-<no answer>}'," >&2
        echo "and a live follower needs $FOLLOW_CONFIG (the M5.2 envelope)." >&2
        echo "Restart the stack on it: SUPER_CONFIG=$FOLLOW_CONFIG RVIZ=false bash scripts/run.sh" >&2
        exit 1
    fi
    echo "==> /fsm_node is on $loaded"
    SUPER_CONFIG="$FOLLOW_CONFIG"
fi

DTP_CPUS="${DTP_CPUS-0-5}"
PIN=()
if [ -n "$DTP_CPUS" ] && [ "$(nproc)" -ge 8 ]; then PIN=(taskset -c "$DTP_CPUS"); else DTP_CPUS=""; fi

echo "==> DTP follower, mode=$DTP_MODE (observe_only:=$OBSERVE_ONLY), config ${SUPER_CONFIG:-<launch default>}${DTP_CPUS:+, cpus $DTP_CPUS}"
exec "${PIN[@]}" ros2 launch dtp_bringup dtp_system.launch.py \
    lidar:=false super:=false \
    "observe_only:=$OBSERVE_ONLY" \
    "viz_goal_topic:=$VIZ_GOAL" \
    ${SUPER_CONFIG:+"config_name:=$SUPER_CONFIG"} \
    "$@"
