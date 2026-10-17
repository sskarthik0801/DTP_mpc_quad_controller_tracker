#!/usr/bin/env bash
# bench_bringup.sh -- the full graph, observe-only, on the desk, with every check the runbooks ask
# for and a clean shutdown. What a flight day's `field.sh prep` does, minus the aircraft moving,
# plus the two things only a bench can do: drive the M6 loss oracle with SYNTHETIC coasting targets
# (nobody is walking) and measure each process's CPU with and without it answering.
#
#   bash drone-dtp/scripts/bench_bringup.sh [OUT_DIR]        # ~8 min; default ~/dtp_datasets/<date>_bench_bringup
#   TRACKER=mpc bash drone-dtp/scripts/bench_bringup.sh      # px4_mpc_tracker.py in the offboard bridge's place
#
# Order (docs/m5.1-one-graph-runbook.md SS2, docs/flight-test-plan.md SS1a):
#   1. scripts/run.sh on follow_test_ros2.yaml  (Livox + FAST-LIO + fsm_node, oracle on)
#   2. both PX4 bridges, off FAST-LIO's cores
#   3. the DTP follower, OBSERVE-ONLY (goals to /dtp/shadow/*, abort off)
#   4. checks: odom rate + stamp lag, foreign cloud subscribers, bridge guard, perception,
#      estimator_preflight (60 s), stage_a_preflight (Stage A mode), CPU idle,
#      CPU with the oracle answering synthetic coasts at 8 Hz, a 60 s bag
#   5. stop in reverse, by process GROUP (never pkill -f), verify nothing survives, swap 0
#
# It never arms, never changes a parameter, never sends a command: observe_only:=true and the
# offboard bridge in IDLE. Safe with props on; safer with them off.
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPO=$PWD
OUT=${1:-$HOME/dtp_datasets/$(date +%Y%m%d_%H%M)_bench_bringup}
mkdir -p "$OUT"
# ROS's setup files are not `set -u` clean (AMENT_TRACE_SETUP_FILES); source first, then be strict.
source /opt/ros/humble/setup.bash; source ~/ws_livox/install/setup.bash; source ~/super_ws/install/setup.bash
set -u
PROBE="python3 $REPO/drone-dtp/scripts/bench_probe.py"
PG=()
say() { echo; echo "== $*"; }
start_group() {  # name, cmd...  -> its own session; pgid == pid
    local name=$1; shift
    setsid "$@" > "$OUT/$name.log" 2>&1 < /dev/null &
    local pid=$!
    PG+=("$name:$pid")
    echo "   started $name (pgid $pid) -> $OUT/$name.log"
}
stop_all() {
    say "stopping, in reverse"
    [ -n "${REC_PID:-}" ] && { echo "   SIGINT recorder $REC_PID"; kill -INT "$REC_PID" 2>/dev/null; sleep 4; }
    for ((i=${#PG[@]}-1; i>=0; i--)); do
        local name=${PG[$i]%%:*} pgid=${PG[$i]##*:}
        echo "   SIGINT group $name ($pgid)"; kill -INT -- "-$pgid" 2>/dev/null
        for _ in $(seq 1 30); do kill -0 -- "-$pgid" 2>/dev/null || break; sleep 0.5; done
        kill -0 -- "-$pgid" 2>/dev/null && { echo "   ... SIGTERM $name"; kill -TERM -- "-$pgid" 2>/dev/null; sleep 3; }
        kill -0 -- "-$pgid" 2>/dev/null && { echo "   ... SIGKILL $name"; kill -KILL -- "-$pgid" 2>/dev/null; }
    done
    sleep 2
    local left
    left=$(ps -eo pid,cmd --no-headers | grep -E "fsm_node|fastlio_mapping|livox_ros_driver2_node|px4_odom_bridge|px4_offboard_bridge|px4_mpc_tracker|perception_node|target_manager|follow_planner_node|command_mux|static_calib_tf|sensor_watchdog|fusion_node|gemini|orbbec|ros2 bag" | grep -v grep | grep -v bench_bringup | wc -l)
    echo "   survivors: $left"
    free -m | awk 'NR==3{print "   swap used " $3 " MB"}'
}
trap stop_all EXIT

say "0. preconditions"
ps -eo cmd --no-headers | grep -E "fsm_node|fastlio_mapping|livox_ros_driver2_node" | grep -v grep && { echo "a stack is already running -- refusing"; exit 1; }
ping -c 1 -W 1 192.168.1.154 > /dev/null && echo "   MID360 reachable" || { echo "   MID360 NOT reachable"; exit 1; }
lsusb | grep -q 2bc5 && echo "   Gemini on USB" || echo "   WARNING: no Gemini on USB"
lsusb | grep -q 3185 && echo "   ARK FPV on USB" || echo "   WARNING: no FC on USB"
echo "   swappiness $(sysctl -n vm.swappiness), logloader $(systemctl is-enabled logloader 2>&1), $(df -h / | awk 'NR==2{print $4}') free"

say "1. stack: run.sh on follow_test_ros2.yaml (oracle on), FAST-LIO pinned, nonblocking DDS"
start_group stack env SUPER_CONFIG=follow_test_ros2.yaml RVIZ=false bash "$REPO/scripts/run.sh"
for i in $(seq 1 40); do sleep 3; grep -q "LOSS ORACLE" "$OUT/stack.log" && break; done
grep -o "\-\- \[Fsm\] LOSS ORACLE.*" "$OUT/stack.log" | head -1 | sed 's/^/   /'
grep -o "fsm_node on [a-z_0-9]*\.yaml" "$OUT/stack.log" | head -1 | sed 's/^/   /'
echo "   waiting for odom ..."; sleep 20
$PROBE hz 10 /lidar_slam/odom:nav_msgs/msg/Odometry /cloud_registered:sensor_msgs/msg/PointCloud2 /livox/imu:sensor_msgs/msg/Imu | sed 's/^/   /'
$PROBE lag 5 | sed 's/^/   /'
echo "   pinning: $(for p in $(pgrep -x fastlio_mapping); do echo -n "fastlio_mapping $(grep Cpus_allowed_list /proc/$p/status | awk '{print $2}') "; done; for p in $(pgrep -x fsm_node); do echo -n "fsm_node $(grep Cpus_allowed_list /proc/$p/status | awk '{print $2}')"; done)"
F=$(timeout 8 ros2 topic info -v /cloud_registered 2>/dev/null | awk '/Subscription count/{s=1} s && /Node name:/{print $3}' | grep -vx fsm_node | tr '\n' ' ')
echo "   foreign subscribers on /cloud_registered: ${F:-none}   (a laptop RViz over wifi is the 2026-09-13 stall; nonblocking sends are on)"

say "2. bridges (taskset 0-5)"
start_group odom_bridge taskset -c 0-5 ros2 run super_bringup px4_odom_bridge.py
TRACKER_EXE=px4_offboard_bridge.py
[ "${TRACKER:-bridge}" = mpc ] && TRACKER_EXE=px4_mpc_tracker.py
echo "   trajectory: $TRACKER_EXE"
start_group offboard_bridge taskset -c 0-5 ros2 run super_bringup "$TRACKER_EXE"
sleep 12
timeout 6 ros2 topic echo --once /px4_odom_bridge/status 2>/dev/null | head -3 | sed 's/^/   /'
$PROBE hz 5 /fmu/in/vehicle_visual_odometry:px4_msgs/msg/VehicleOdometry /fmu/in/trajectory_setpoint:px4_msgs/msg/TrajectorySetpoint /fmu/out/vehicle_local_position_v1:px4_msgs/msg/VehicleLocalPosition 2>&1 | sed 's/^/   /'

say "3. DTP follower, OBSERVE-ONLY"
start_group follower env DTP_MODE=observe bash "$REPO/scripts/dtp_follower.sh"
echo "   waiting 75 s for TensorRT + the GPU cloud check ..."; sleep 75
grep -h -o "GPU cloud verified against numpy\|extrinsic verified\|cloud [0-9.]* ms \[[a-z]*\]\|ABORT.*\(armed\|disabled\|off\).*" "$OUT/follower.log" 2>/dev/null | sort | uniq -c | head -5 | sed 's/^/   /'
$PROBE hz 10 /dtp/targets:dtp_msgs/msg/TargetArray /dtp/perception_ok:std_msgs/msg/Bool /dtp/sensors_ok:std_msgs/msg/Bool /camera/color/image_raw/compressed:sensor_msgs/msg/CompressedImage 2>&1 | sed 's/^/   /'
timeout 5 ros2 topic echo --once /dtp/lock_state 2>/dev/null | head -1 | sed 's/^/   lock_state: /'
timeout 5 ros2 topic echo --once /mux/state 2>/dev/null | head -1 | sed 's/^/   mux: /'

say "4a. estimator preflight (60 s, G0.2)"
python3 "$REPO/scripts/px4/estimator_preflight.py" 2>&1 | tail -14 | sed 's/^/   /'

say "4b. stage A preflight (observe-only mode)"
python3 "$REPO/drone-dtp/scripts/stage_a_preflight.py" 2>&1 | tail -40 | sed 's/^/   /'

say "4c. CPU, 30 s, nobody coasting (the oracle idle)"
$PROBE cpu 30 fastlio_mapping fsm_node livox_ros_driver2_node perception_node target_manager follow_planner_node command_mux_node px4_odom_bridge.py "$TRACKER_EXE" static_calib_tf sensor_watchdog | sed 's/^/   /'
uptime | sed 's/.*load/   load/'

say "4d. the oracle LIVE: synthetic coasting targets at 8 Hz for 60 s, 3 m ahead; CPU sampled meanwhile"
REC_PID=""
ros2 bag record -o "$OUT/bag" /planning/loss_event /dtp/follow_target /lidar_slam/odom /dtp/shadow/follow_goal /dtp/shadow/yaw_target /mux/state /dtp/lock_state /fmu/in/vehicle_visual_odometry /fmu/out/vehicle_local_position_v1 /rog_map/occ > "$OUT/bag_record.log" 2>&1 < /dev/null &
REC_PID=$!
sleep 4
($PROBE cpu 45 fastlio_mapping fsm_node perception_node target_manager follow_planner_node command_mux_node > "$OUT/cpu_oracle.txt" 2>&1 &)
$PROBE synth 60 3.0 0.0 0.0 | sed 's/^/   /'
sleep 2; sed 's/^/   /' "$OUT/cpu_oracle.txt"
grep -h "LossOracle\]" "$OUT/stack.log" | tail -2 | sed 's/^/   /'
grep -h "BELOW the lowest" "$OUT/stack.log" | head -1 | cut -c1-160 | sed 's/^/   /'
kill -INT "$REC_PID" 2>/dev/null; sleep 5; REC_PID=""
ls "$OUT/bag" 2>/dev/null | sed 's/^/   bag: /'

say "5. done; shutting down (trap)"
