#!/usr/bin/env bash
#
# RViz helper -- run this ON YOUR LAPTOP (optional).
#
# The GCS dashboard itself needs nothing on the laptop but a browser. This
# script only enables its "Open RViz" button: a web page cannot launch a native
# app, so this helper polls the Jetson for the command and starts
# scripts/laptop_viewer.sh here.
#
#   bash scripts/gcs_laptop.sh 'http://192.168.8.130:8080/?k=<token>'
#
# Paste the whole URL the Jetson printed, quoted, token and all -- the helper
# needs the token to talk to the GCS. With no argument it uses $GCS_URL.
#
# It also opens the dashboard in your browser. Ctrl-C stops the helper and any
# RViz it started.
#
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URL="${1:-${GCS_URL:-}}"

if [ -z "$URL" ]; then
  echo "usage: bash scripts/gcs_laptop.sh 'http://<jetson-ip>:8080/?k=<token>'"
  echo
  echo "Paste the URL the Jetson printed when you ran scripts/gcs.sh --"
  echo "quote it, or the shell will eat the '?' and '&'."
  exit 2
fi

if [ ! -d /opt/ros/humble ]; then
  echo "WARNING: /opt/ros/humble not found. The dashboard works in a browser"
  echo "         anyway, but RViz cannot start. Run scripts/laptop_setup.sh first."
fi

# Open the dashboard without blocking or inheriting the helper's stdio.
if command -v xdg-open >/dev/null 2>&1; then
  (xdg-open "$URL" >/dev/null 2>&1 &) || true
elif command -v open >/dev/null 2>&1; then
  (open "$URL" >/dev/null 2>&1 &) || true
fi

exec python3 "$REPO/scripts/gcs/rviz_agent.py" "$URL"
