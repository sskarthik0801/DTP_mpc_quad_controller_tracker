#!/usr/bin/env bash
#
# Laptop RViz for the human-following (DTP) stack: the camera picture with perception's person
# boxes, the 3-D people / lock / forecast markers, and the M1 map view. No build required.
#
#   JETSON_IP=gypsy.local bash scripts/laptop_dtp_viewer.sh
#
# Just laptop_viewer.sh with DTP=1 -- every variable documented there applies here.
exec env DTP=1 bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/laptop_viewer.sh" "$@"
