#!/usr/bin/env bash
#
# Give the Jetson a stable name, so the web GCS has a URL that survives DHCP.
#
#   sudo bash scripts/set_hostname.sh gypsy
#   bash scripts/set_hostname.sh --show      # no sudo; just report
#
# The field problem this solves: the Wi-Fi IP is DHCP-assigned and changes
# between sessions (CLAUDE.md says as much about laptop_viewer.sh), so a
# bookmarked http://192.168.x.y:8080 goes stale.  Two names do not:
#
#   http://<name>.local:8080          mDNS/avahi -- same Wi-Fi, zero config,
#                                     works with no internet at all
#   http://<name>:8080                Tailscale MagicDNS -- works off-LAN,
#                                     which the RViz/DDS path never can
#
# Why the stock name is not good enough: this board ships as `ubuntu`, which is
# the Ubuntu default.  mDNS names must be unique on a segment, so the moment a
# second stock Ubuntu machine joins the field Wi-Fi one of them is silently
# renamed to `ubuntu-2.local` -- and it may be this one, breaking the bookmark
# exactly when you are standing in a field.  A distinctive name cannot collide.
#
# Everything here is idempotent and survives a reboot.  It does NOT survive a
# Jetson reflash -- re-run it after any reimage, like the DTB fix
# (docs/xrce-agent-nx.md) and the GCS unit (scripts/gcs_service.sh).
set -euo pipefail

show() {
  local h; h="$(hostname)"
  echo "hostname     : $h"
  echo "mDNS         : http://${h}.local:8080     (same Wi-Fi)"
  if command -v tailscale >/dev/null 2>&1; then
    local dns; dns="$(tailscale status --json 2>/dev/null \
      | python3 -c 'import json,sys; print((json.load(sys.stdin).get("Self") or {}).get("DNSName","").rstrip("."))' 2>/dev/null || true)"
    [ -n "$dns" ] && echo "Tailscale    : http://${dns}:8080   (anywhere)"
    [ -n "$dns" ] && echo "               http://${dns%%.*}:8080  (with MagicDNS on the client)"
  fi
  echo "avahi-daemon : $(systemctl is-active avahi-daemon 2>/dev/null || echo unknown)"
}

if [ "${1:-}" = "--show" ] || [ $# -eq 0 ]; then
  show; exit 0
fi

NAME="$1"
# RFC 1123 / mDNS-safe: a wrong character here yields a name that resolves
# nowhere and a dashboard nobody can reach.
if ! [[ "$NAME" =~ ^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$ ]]; then
  echo "ERROR: '$NAME' is not a valid hostname (lowercase letters, digits and" >&2
  echo "       hyphens; must not start or end with a hyphen)." >&2
  exit 1
fi
if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: needs root -- run: sudo bash scripts/set_hostname.sh $NAME" >&2
  exit 1
fi

OLD="$(hostname)"
echo "renaming '$OLD' -> '$NAME'"

hostnamectl set-hostname "$NAME"

# /etc/hosts must follow, or every sudo prints "unable to resolve host". Ubuntu
# puts the hostname on 127.0.1.1, not 127.0.0.1 -- rewrite that line only.
if grep -qE '^127\.0\.1\.1[[:space:]]' /etc/hosts; then
  sed -i -E "s/^(127\.0\.1\.1[[:space:]]+).*/\1$NAME/" /etc/hosts
else
  printf '127.0.1.1\t%s\n' "$NAME" >> /etc/hosts
fi

# Keep <name>.local off the interfaces a laptop cannot reach.  avahi publishes
# on every interface by default, and two of this board's four are dead ends:
# docker0 (172.17.0.1) and eno1 (192.168.1.50), which is the point-to-point
# link dedicated to the MID360 and is NOT routable from a laptop -- the same
# trap CLAUDE.md documents for laptop_viewer.sh's JETSON_IP.  mDNS answers
# per-interface so this is usually harmless, but the failure it prevents ("the
# .local name resolves to something that times out") is one you would be
# debugging in a field.  Denied rather than allow-listed on purpose: the Wi-Fi
# interface name is not stable across boards (CLAUDE.md), these two are.
AVAHI_CONF=/etc/avahi/avahi-daemon.conf
if [ -f "$AVAHI_CONF" ] && ! grep -q '^deny-interfaces=' "$AVAHI_CONF"; then
  cp -n "$AVAHI_CONF" "$AVAHI_CONF.super-bak" || true
  sed -i 's/^\[server\]/[server]\ndeny-interfaces=docker0,eno1/' "$AVAHI_CONF"
  echo "avahi: publishing .local on Wi-Fi and Tailscale only (not docker0/eno1)"
fi

# avahi watches the hostname, but a restart makes the change immediate rather
# than waiting for its next probe -- and applies the deny-interfaces above.
systemctl restart avahi-daemon 2>/dev/null || true

# Tailscale's machine name is set at login and does not follow the system
# hostname on its own.
if command -v tailscale >/dev/null 2>&1; then
  tailscale set --hostname="$NAME" 2>/dev/null \
    && echo "tailscale machine name set to '$NAME'" \
    || echo "WARNING: could not set the Tailscale name (is tailscaled up?)"
fi

# ARK's flight-review derives --allow-websocket-origin from `hostname -f` at
# start (/usr/lib/ark-os/libexec/start_flight_review.sh), so it needs a restart
# to pick the new name up; without it, its websocket is refused on the new URL.
# Nothing in ARK is edited -- it re-derives correctly on its own.
if systemctl is-active --quiet flight-review.service; then
  systemctl restart flight-review.service && echo "flight-review.service restarted"
fi

# So the dashboard's own banner and its bookmark links show the new name.
if systemctl is-active --quiet super-gcs.service; then
  systemctl restart super-gcs.service && echo "super-gcs.service restarted"
fi

echo
show
echo
echo "Bookmark the mDNS URL for the field and the Tailscale one for everywhere"
echo "else. The token is on the dashboard's Host card."
