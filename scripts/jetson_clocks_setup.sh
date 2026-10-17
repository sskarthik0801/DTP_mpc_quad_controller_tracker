#!/usr/bin/env bash
#
# Pin the Jetson clocks (jetson_clocks) at every boot, so a reboot cannot leave the GPU slow.
#
#   sudo bash scripts/jetson_clocks_setup.sh install    # write the unit, enable it, run it now
#   sudo bash scripts/jetson_clocks_setup.sh uninstall  # disable and remove it (clocks stay until reboot)
#   bash scripts/jetson_clocks_setup.sh show            # no sudo: the unit's state and the GPU clock
#
# Why. jetson_clocks does not survive a reboot, and the GPU then idles at 306 of 918 MHz. On
# 2026-09-21 that happened twice in one field session (reboots at 16:23 and 18:28). Both times
# the live preflight FAILed jetson_clocks, and the first time perception also went intermittently
# BLIND: its single-threaded executor saturated and starved its own tf2 listener, so ~10 % of
# frames published an empty TargetArray and /dtp/sensors_ok held the mux in HOLD|SENSORS
# (docs/dtp-follower-stack.md). The desk check and both preflights still check the clock; this
# only removes the manual step.
#
# Ordered after nvpmodel.service, which sets the power mode (MAXN) at boot: jetson_clocks pins
# the clocks of the mode that is active, so it must run second.
#
# Survives a reboot, not a reflash -- re-run after any reimage, like gcs_service.sh.
#
set -eo pipefail

UNIT=super-dtp-jetson-clocks.service
PATH_UNIT=/etc/systemd/system/$UNIT
GPU=/sys/class/devfreq/17000000.gpu

show() {
    echo "unit: $UNIT  enabled=$(systemctl is-enabled "$UNIT" 2>/dev/null || echo no)" \
         " active=$(systemctl is-active "$UNIT" 2>/dev/null || echo no)"
    local cur max
    cur=$(cat "$GPU/cur_freq" 2>/dev/null) max=$(cat "$GPU/max_freq" 2>/dev/null)
    if [ -n "$cur" ]; then
        echo "GPU $((cur/1000000)) of $((max/1000000)) MHz$([ "$cur" = "$max" ] && echo ' -- pinned' || echo ' -- NOT pinned')"
    else
        echo "GPU: cannot read $GPU"
    fi
}

case "${1:-}" in
    show) show ;;
    install)
        [ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
        cat > "$PATH_UNIT" <<'EOF'
# SUPER-Human-DTP: scripts/jetson_clocks_setup.sh. jetson_clocks does not survive a reboot.
[Unit]
Description=SUPER-Human-DTP: pin the Jetson clocks (jetson_clocks) at boot
After=nvpmodel.service
Wants=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
        systemctl daemon-reload
        systemctl enable "$UNIT"
        systemctl restart "$UNIT"
        show ;;
    uninstall)
        [ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
        systemctl disable --now "$UNIT" 2>/dev/null || true
        rm -f "$PATH_UNIT"
        systemctl daemon-reload
        show ;;
    *) sed -n '2,21p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
