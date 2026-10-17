#!/usr/bin/env bash
#
# Install the web GCS as a systemd service so it comes back on every boot.
#
#   sudo bash scripts/gcs_service.sh install      # enable + start now
#   sudo bash scripts/gcs_service.sh uninstall    # stop + disable + remove
#   bash scripts/gcs_service.sh status            # no sudo needed
#   sudo GCS_TRACKER=mpc bash scripts/gcs_service.sh install
#                     # the trajectory row runs px4_mpc_tracker.py instead of the
#                     # bridge (docs/mpc-tracker-integration.md). Re-install without it
#                     # to go back. Read once, when the GCS starts.
#
# What this does and does NOT do:
#   - It starts the DASHBOARD at boot. It does NOT start the SUPER stack, the
#     bridges, or anything that touches the aircraft. Nothing flies because the
#     Jetson powered on; you still press the buttons.
#   - The service runs as the invoking user (not root), because the ROS
#     workspaces live in that user's home and the token in ~/.cache/super-gcs.
#     Same token as before, so a bookmarked URL keeps working across reboots.
#
set -eo pipefail

ACTION="${1:-}"
UNIT=/etc/systemd/system/super-gcs.service
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# When run under sudo, SUDO_USER is who actually owns the workspaces.
RUN_USER="${SUDO_USER:-$USER}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
PORT="${GCS_PORT:-8080}"
TRACKER="${GCS_TRACKER:-bridge}"

usage() { sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }

case "$ACTION" in
  status)
    systemctl status super-gcs.service --no-pager || true
    echo
    echo "logs:  journalctl -u super-gcs -n 50 --no-pager"
    exit 0
    ;;
  uninstall)
    [ "$(id -u)" -eq 0 ] || { echo "needs sudo"; exit 1; }
    systemctl disable --now super-gcs.service 2>/dev/null || true
    rm -f "$UNIT"
    systemctl daemon-reload
    echo "removed. The dashboard will not start at boot any more."
    echo "Run it by hand with: bash scripts/gcs.sh"
    exit 0
    ;;
  install) ;;
  *) usage ;;
esac

[ "$(id -u)" -eq 0 ] || { echo "needs sudo: sudo bash scripts/gcs_service.sh install"; exit 1; }
[ -n "$RUN_HOME" ] || { echo "cannot resolve home for $RUN_USER"; exit 1; }
[ -f "$REPO/scripts/gcs.sh" ] || { echo "run this from a clone of the repo"; exit 1; }

echo "==> installing super-gcs.service"
echo "    repo : $REPO"
echo "    user : $RUN_USER  (home $RUN_HOME)"
echo "    port : $PORT"
echo "    trajectory row : $TRACKER  (GCS_TRACKER; mpc = px4_mpc_tracker.py)"

cat > "$UNIT" <<UNITEOF
[Unit]
Description=SUPER web GCS (browser control panel for the SUPER stack)
Documentation=file://$REPO/scripts/gcs/README.md
# Deliberately network.target, NOT network-online.target. The GCS binds
# 0.0.0.0 and works before any interface is up, and /api/config re-resolves the
# host IPs on every request, so waiting buys nothing but a boot delay -- up to
# 90 s if anyone ever enables NetworkManager-wait-online (both wait-online
# units are disabled on this board today, so Wants= would be a trap that only
# springs later). At a flying field you want the page the moment you power on.
#
# ARK's dds-agent supplies the /fmu/* telemetry, but the GCS starts fine
# without it and picks it up when it appears -- ordering only, never a
# requirement. The dashboard must never fail to start because the flight
# controller is unplugged.
After=network.target dds-agent.service
# Do not crash-loop forever on a broken install.
StartLimitIntervalSec=120
StartLimitBurst=5

[Service]
Type=exec
User=$RUN_USER
Group=$RUN_USER
Environment=HOME=$RUN_HOME
Environment=GCS_TRACKER=$TRACKER
WorkingDirectory=$REPO
ExecStart=/bin/bash $REPO/scripts/gcs.sh --port $PORT
Restart=on-failure
RestartSec=5

# The GCS handles SIGTERM itself: it shuts the HTTP server down and then stops
# every process group it started, in order. That teardown can take ~30 s with
# the stack and all three bridges up (fsm_node holds a ~2.3 GB allocation), so
# give it room before systemd loses patience.
KillMode=mixed
KillSignal=SIGTERM
TimeoutStopSec=90

[Install]
WantedBy=multi-user.target
UNITEOF

systemctl daemon-reload
systemctl enable super-gcs.service
systemctl restart super-gcs.service
sleep 4

if ! systemctl is-active --quiet super-gcs.service; then
  echo
  echo "FAILED to start. What it said:"
  journalctl -u super-gcs -n 30 --no-pager
  exit 1
fi

echo "==> active, and enabled at boot"
echo
journalctl -u super-gcs -n 12 --no-pager | sed -n '/Open one of these/,$p' || true
cat <<'DONE'

Verify:
  systemctl is-enabled super-gcs        # want: enabled
  bash scripts/gcs_service.sh status
  journalctl -u super-gcs -f            # follow

It starts the DASHBOARD only -- no stack, no bridges, nothing armed.

Note: the unit lives in /etc/systemd/system, so it survives apt upgrades but
NOT a Jetson reflash. Re-run this after any reimage, along with the DTB fix
(docs/xrce-agent-nx.md) and the PX4 power-monitor build (firmware/README.md).
DONE
