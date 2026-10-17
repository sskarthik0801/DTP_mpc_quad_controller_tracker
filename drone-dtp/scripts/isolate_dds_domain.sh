#!/usr/bin/env bash
# Put every ROS 2 participant on the drone NX onto a private DDS domain.
#
# =============================================================================================
# !! DOES NOT WORK ON THIS AIRCRAFT AS-IS. TRIED 2026-09-07 AND REVERTED. !!
#
# `MicroXRCEAgent` -- the PX4 bridge, shipped by ark-os as a raw FastDDS application -- has NO
# DDS domain option. Its `-d/--discovery` flag is the discovery PORT, not the domain, and
# ROS_DOMAIN_ID is an rmw concept the agent never reads. It therefore creates its participant on
# domain 0 no matter what systemd sets.
#
# MEASURED: after running this with DOMAIN=42, `/fmu/out/battery_status_v1` had **Publisher
# count 0** on domain 42 and **Publisher count 1 on domain 0**, still reporting 21.30 V. Every
# PX4 topic -- battery, arming state, vehicle status -- was invisible to our nodes. That is worse
# than the exposure it was meant to fix, so it was reverted.
#
# TO MAKE IT WORK you would have to move the agent's participant with a FastDDS XML profile
# (`FASTRTPS_DEFAULT_PROFILES_FILE` with a <domainId>), and verify PX4 telemetry survives before
# trusting it. Until then the shared domain stands, mitigated by
# `calib_io.check_lidar_source()`.
#
#   sudo bash scripts/isolate_dds_domain.sh --revert     # undo
# =============================================================================================
#
#   sudo bash scripts/isolate_dds_domain.sh [DOMAIN_ID]      # default 42
#
# WHY
# ---
# The drone shares a DDS discovery domain (the default, 0) with other lab robots. On 2026-09-07
# another machine's Livox driver was publishing `/livox/lidar_192_168_1_146` and `_160` onto our
# graph, and a live camera-LiDAR overlay was built on that foreign point cloud. It looked
# entirely plausible: **a point cloud carries no identity**, so nothing in the data says whose it
# is. `calib_io.check_lidar_source()` now catches that specific case, but the exposure is broader
# -- another robot's odometry, TF or camera could appear the same way.
#
# WHY ALL-OR-NOTHING
# ------------------
# Applying this to only some processes is WORSE than not applying it. Measured on this box:
# with `ROS_LOCALHOST_ONLY=1` set for one shell, a fresh participant saw neither the foreign
# topics NOR our own local `/livox/lidar` -- because that driver was started without it. The same
# split applies to ROS_DOMAIN_ID. In particular `dds-agent.service` publishes every `/fmu/*`
# topic, so leaving it behind silently costs all PX4 telemetry.
#
# So this script covers all three places a participant gets its environment:
#   1. interactive shells       -> ~/.bashrc
#   2. our workspace env script -> ros2_ws/dtp_env_jetson.sh   (belt and braces; must MATCH)
#   3. systemd services         -> drop-ins for dds-agent + super-gcs
#
# ⚠️ super-gcs.service runs from a TEAMMATE'S repo (SUPER-Human-DTP). Tell Kevin before running
# this: after it, anything he starts in a shell that has not picked up the new environment will
# not see our nodes, and the symptom is silence, not an error.
#
# TO REACH THE DRONE'S TOPICS FROM ANOTHER MACHINE afterwards, export the same DOMAIN_ID there.
# Chosen over ROS_LOCALHOST_ONLY precisely to keep that possible.

set -euo pipefail

MODE="apply"
if [ "${1:-}" = "--revert" ]; then MODE="revert"; shift; fi
DOMAIN="${1:-42}"
DRONE_USER="ideas-ad"
HOME_DIR="/home/${DRONE_USER}"
ENV_SCRIPT="${HOME_DIR}/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh"
SERVICES=(dds-agent.service super-gcs.service)

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: needs root for the systemd drop-ins. Re-run with sudo." >&2
    exit 1
fi
if ! [[ "$DOMAIN" =~ ^[0-9]+$ ]] || [ "$DOMAIN" -gt 101 ]; then
    # >101 needs kernel port-range tuning and silently fails to discover on some setups.
    echo "ERROR: DOMAIN_ID must be 0-101 (got '$DOMAIN')." >&2
    exit 1
fi

if [ "$MODE" = "revert" ]; then
    echo "=== REVERTING DDS domain isolation (back to the default domain 0) ==="
    sed -i "/^export ROS_DOMAIN_ID=/d" "${HOME_DIR}/.bashrc" 2>/dev/null && echo "  [ok] .bashrc cleaned"
    sed -i "/# Private DDS domain/,+1d" "${HOME_DIR}/.bashrc" 2>/dev/null || true
    [ -f "$ENV_SCRIPT" ] && sed -i "/^export ROS_DOMAIN_ID=/d" "$ENV_SCRIPT" && echo "  [ok] dtp_env_jetson.sh cleaned"
    for svc in "${SERVICES[@]}"; do
        rm -f "/etc/systemd/system/${svc}.d/10-ros-domain.conf" && echo "  [ok] drop-in removed: $svc"
        rmdir "/etc/systemd/system/${svc}.d" 2>/dev/null || true
    done
    systemctl daemon-reload
    for svc in "${SERVICES[@]}"; do
        systemctl restart "$svc" 2>/dev/null && echo "  [ok] restarted $svc"
    done
    echo
    echo "  VERIFY (new shell): ros2 topic echo /fmu/out/battery_status_v1 --once \\"
    echo "                       --qos-reliability best_effort   # must print a voltage"
    echo "  Restart any ROS nodes started on the private domain."
    exit 0
fi

echo "=== isolating the drone onto ROS_DOMAIN_ID=$DOMAIN ==="
echo "!! See the header: this is known to break PX4 telemetry on this aircraft. !!"

# ---------------------------------------------------------------- 1. interactive shells
say_done() { echo "  [ok] $1"; }
if grep -q "^export ROS_DOMAIN_ID=" "${HOME_DIR}/.bashrc" 2>/dev/null; then
    sed -i "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=${DOMAIN}/" "${HOME_DIR}/.bashrc"
    say_done ".bashrc updated"
else
    printf '\n# Private DDS domain -- see scripts/isolate_dds_domain.sh. Other lab robots share\n# the default domain 0 and were appearing on our graph.\nexport ROS_DOMAIN_ID=%s\n' \
        "$DOMAIN" >> "${HOME_DIR}/.bashrc"
    say_done ".bashrc appended"
fi

# ---------------------------------------------------------------- 2. our env script
if [ -f "$ENV_SCRIPT" ]; then
    if grep -q "ROS_DOMAIN_ID" "$ENV_SCRIPT"; then
        sed -i "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=${DOMAIN}/" "$ENV_SCRIPT"
        say_done "dtp_env_jetson.sh updated"
    else
        printf '\n# Must MATCH the systemd drop-ins and ~/.bashrc -- a participant on a different\n# domain sees nothing, and reports no error while doing so.\nexport ROS_DOMAIN_ID=%s\n' \
            "$DOMAIN" >> "$ENV_SCRIPT"
        say_done "dtp_env_jetson.sh appended"
    fi
else
    echo "  [warn] $ENV_SCRIPT not found -- skipped"
fi

# ---------------------------------------------------------------- 3. systemd services
for svc in "${SERVICES[@]}"; do
    if ! systemctl list-unit-files "$svc" >/dev/null 2>&1; then
        echo "  [warn] $svc not installed -- skipped"
        continue
    fi
    d="/etc/systemd/system/${svc}.d"
    mkdir -p "$d"
    cat > "${d}/10-ros-domain.conf" <<EOF
# Generated by scripts/isolate_dds_domain.sh -- keep in step with ~/.bashrc and
# dtp_env_jetson.sh. A service left on another domain goes silent rather than erroring.
[Service]
Environment=ROS_DOMAIN_ID=${DOMAIN}
EOF
    say_done "drop-in written for $svc"
done

systemctl daemon-reload
for svc in "${SERVICES[@]}"; do
    systemctl list-unit-files "$svc" >/dev/null 2>&1 || continue
    systemctl restart "$svc" && say_done "restarted $svc" || echo "  [warn] restart failed: $svc"
done

echo
echo "=== VERIFY (in a NEW shell, so it picks up the environment) ==="
cat <<EOF
  export ROS_DOMAIN_ID=${DOMAIN}
  ros2 daemon stop                       # the daemon caches the OLD domain's graph
  ros2 topic list --no-daemon | grep -c '^/fmu/out/'          # expect > 0
  ros2 topic list --no-daemon | grep -c 'livox/lidar_192'     # expect 0 (foreign robot gone)

  Restart any ROS nodes started before this ran -- they are still on the old domain.
  Anything that must talk to the drone from another machine needs ROS_DOMAIN_ID=${DOMAIN} too.
EOF
