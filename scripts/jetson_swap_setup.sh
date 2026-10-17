#!/usr/bin/env bash
#
# Stop the Jetson parking idle pages in zram swap while the flight stack runs (M5.3, C2).
#
#   sudo bash scripts/jetson_swap_setup.sh install    # write the sysctl drop-in and apply it now
#   sudo bash scripts/jetson_swap_setup.sh uninstall  # remove it and restore the stock value
#   bash scripts/jetson_swap_setup.sh show            # no sudo: current values and swap activity
#
# Why. The M5.3 compute soak on 2026-09-13 (docs/m5-m6-plan.md SS2.2, C2 "swap 0 B") swapped
# 1053 pages out over 20 min with MemAvailable ~7 GB: not memory exhaustion, but the stock
# vm.swappiness=60 letting reclaim push idle anonymous pages into zram while a ros2 bag fills
# the page cache. Swap activity is the M1 regression's signature (RSS growth -> swap thrash ->
# /lidar_slam/odom 10 Hz -> 0.55 Hz), and the soak scores ACTIVITY, so any swap-out is a FAIL
# and, worse, hides a real leak behind a benign one. swappiness=10 keeps zram as the last
# resort instead of the first choice; the eight 1 GB zram devices stay configured.
#
# Survives a reboot (sysctl.d), not a reflash -- re-run after any reimage, like gcs_service.sh.
#
set -eo pipefail

CONF=/etc/sysctl.d/80-super-dtp-swap.conf
VALUE="${SWAPPINESS:-10}"

show() {
    echo "vm.swappiness = $(cat /proc/sys/vm/swappiness)   (stock 60; this repo installs $VALUE)"
    [ -f "$CONF" ] && echo "drop-in: $CONF" || echo "drop-in: not installed ($CONF)"
    awk '/^(pswpin|pswpout) /{printf "%s since boot: %s pages\n", $1, $2}' /proc/vmstat
    awk 'NR>1{u+=$4} END{printf "swap used now: %d kB\n", u}' /proc/swaps
}

case "${1:-}" in
    show) show ;;
    install)
        [ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
        printf '# SUPER-Human-DTP: scripts/jetson_swap_setup.sh (M5.3 C2). Stock is 60.\nvm.swappiness = %s\n' "$VALUE" > "$CONF"
        sysctl -q -p "$CONF"
        show ;;
    uninstall)
        [ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
        rm -f "$CONF"
        sysctl -q -w vm.swappiness=60
        show ;;
    *) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
