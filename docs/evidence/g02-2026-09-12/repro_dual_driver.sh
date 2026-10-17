#!/usr/bin/env bash
# G0.2a bench reproduction: a second Livox driver started on top of run.sh, then killed.
# No PX4 bridge runs, so nothing reaches the flight controller.
#
# *** THIS DELIBERATELY DIVERGES FAST-LIO BY KILOMETRES. *** Bench only, aircraft disarmed, and
# NEVER with px4_odom_bridge running unless testing its guard on purpose.
# As archived it will no longer reproduce anything: since the fix, sensors.launch.py refuses to
# start a second driver. To re-run, start the second driver directly with
#   ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=0 \
#     -p user_config_path:=$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/config/MID360_config.json
# Paths below point at the session's scratchpad; edit SP before running.
SP=/tmp/claude-1000/-home-ideas-ad-SUPER-Human-DTP/2fa07642-fdbf-4d5c-a2c8-47a508b105b5/scratchpad
cd /home/ideas-ad/SUPER-Human-DTP
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/super_ws/install/setup.bash
stamp() { echo "$(date +%s.%N | cut -c1-13) $*" | tee -a "$SP/repro2_events.log"; }
: > "$SP/repro2_events.log"

stamp "start run.sh"
RVIZ=false setsid bash scripts/run.sh > "$SP/repro2_runsh.log" 2>&1 < /dev/null &
A=$!
sleep 25
stamp "start recorder"
setsid python3 "$SP/repro_rec.py" "$SP/repro2.csv" > "$SP/repro2_rec.log" 2>&1 < /dev/null &
R=$!
sleep 30
stamp "start SECOND driver (sensors.launch.py camera:=false, xfer_format:=0 = PointCloud2, as dtp_system.launch.py defaults)"
setsid ros2 launch dtp_bringup sensors.launch.py camera:=false xfer_format:=0 > "$SP/repro2_second.log" 2>&1 < /dev/null &
B=$!
sleep 120
stamp "SIGINT second driver pgid $B"
kill -INT -"$B"
sleep 90
stamp "stop recorder and stack"
kill -INT -"$R"
kill -INT -"$A"
sleep 20
for g in $B $R $A; do kill -TERM -"$g" 2>/dev/null; done
sleep 5
for g in $B $R $A; do kill -KILL -"$g" 2>/dev/null; done
stamp "done"
