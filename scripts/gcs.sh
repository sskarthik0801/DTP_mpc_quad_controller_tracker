#!/usr/bin/env bash
#
# Web GCS -- run this ON THE JETSON, then open the printed URL on your laptop.
#
# Serves a browser control panel: start/stop the SUPER stack and the PX4
# bridges, and watch battery, flight mode, estimator health, topic rates and
# swap.  The laptop needs nothing installed but a browser -- the same rule as
# scripts/laptop_viewer.sh, taken one step further (this also works over
# Tailscale, which DDS multicast cannot).
#
#   bash scripts/gcs.sh                     # 0.0.0.0:8080, token auth
#   bash scripts/gcs.sh --port 9000
#   bash scripts/gcs.sh --bind 127.0.0.1 --no-auth
#   GCS_TRACKER=mpc bash scripts/gcs.sh     # trajectory row runs px4_mpc_tracker.py
#
# Ctrl-C stops the server and everything it started.
#
set -eo pipefail   # not -u: ROS setup.bash references unset vars on a cold shell

SUPER_WS="${SUPER_WS:-$HOME/super_ws}"
WS_LIVOX="${WS_LIVOX:-$HOME/ws_livox}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Same source order as scripts/run.sh. The server's children inherit this
# environment, so the supervisor never has to re-source anything.
source /opt/ros/humble/setup.bash
[ -f "$WS_LIVOX/install/setup.bash" ] && source "$WS_LIVOX/install/setup.bash"
[ -f "$SUPER_WS/install/setup.bash" ] && source "$SUPER_WS/install/setup.bash"

cd "$REPO"
exec python3 scripts/gcs/server.py "$@"
