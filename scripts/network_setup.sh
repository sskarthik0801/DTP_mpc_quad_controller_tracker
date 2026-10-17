#!/usr/bin/env bash
#
# Persistent static IP for the Livox MID360 via NetworkManager.
# Run with sudo:   sudo bash scripts/network_setup.sh
#
# Why a NetworkManager profile (not `ip addr add`): NM owns the interface and will
# flush a manually-added IP within ~1 min, which breaks the driver ("bind failed").
# A managed profile persists across reboots and is never flushed.
#
# Per-unit: the MID360's IP is 192.168.1.1XX where XX = last two digits of its serial.
# Override with env vars: IFACE, HOST_IP, LIDAR_IP, CON
#
set -euo pipefail

# Auto-detect the wired iface. The name is NOT stable across Jetson boards/JetPack
# images -- it was `enP8p1s0` on one unit and is `eno1` on the Orin Nano Engineering
# Reference devkit. Prefer an interface with a live carrier (the LiDAR plugged in).
detect_iface() {
    local candidates=() n
    for d in /sys/class/net/*; do
        n=$(basename "$d")
        case "$n" in
            lo|docker*|veth*|br-*|usb*|l4tbr*|can*|tailscale*|wl*|p2p-*) continue ;;
        esac
        [ -e "$d/device" ] || continue          # skip virtual devices
        [ "$(cat "$d/carrier" 2>/dev/null)" = "1" ] && { echo "$n"; return; }
        candidates+=("$n")
    done
    [ ${#candidates[@]} -gt 0 ] && echo "${candidates[0]}"
}

IFACE="${IFACE:-$(detect_iface)}"
if [ -z "$IFACE" ]; then
    echo "ERROR: no wired interface found. Is the LiDAR cable plugged in?" >&2
    echo "       Override explicitly:  IFACE=eth0 sudo -E bash $0" >&2
    exit 1
fi
if [ ! -e "/sys/class/net/$IFACE" ]; then
    echo "ERROR: interface '$IFACE' does not exist. Available:" >&2
    ls /sys/class/net >&2
    exit 1
fi
if [ "$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null)" != "1" ]; then
    echo "WARNING: '$IFACE' has no carrier -- cable may be unplugged or LiDAR unpowered." >&2
fi
echo "==> using wired interface: $IFACE"
HOST_IP="${HOST_IP:-192.168.1.50}"
LIDAR_IP="${LIDAR_IP:-192.168.1.154}"
CON="${CON:-livox}"

echo "==> removing any prior livox profiles"
for u in $(nmcli -t -f UUID,NAME connection show | grep -E ':(livox|livox-mid360)$' | cut -d: -f1); do
    nmcli connection delete "$u" || true
done

echo "==> creating profile '$CON' on $IFACE with $HOST_IP/24"
nmcli connection add type ethernet ifname "$IFACE" con-name "$CON"
nmcli connection modify "$CON" ipv4.addresses "$HOST_IP/24"   # address before method
nmcli connection modify "$CON" ipv4.method manual
nmcli connection modify "$CON" ipv4.never-default yes         # keep default route on Wi-Fi
nmcli connection modify "$CON" connection.autoconnect yes
nmcli connection up "$CON"

echo "==> verify"
ip -4 addr show "$IFACE" | grep inet || { echo "ERROR: no IPv4 on $IFACE"; exit 1; }
ping -c3 "$LIDAR_IP"
