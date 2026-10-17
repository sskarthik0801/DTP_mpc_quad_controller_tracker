#!/usr/bin/env bash
# replay_perception.sh -- replay a bag through the REAL perception_node and record what it publishes.
#
#   bash scripts/replay_perception.sh <numpy|cuda> <out_dir> [label] [bag]
#   WARM_S=60 RATE=0.25 bash scripts/replay_perception.sh ...   # frame-deterministic (see WARM_S)
#   python3 scripts/compare_replay_targets.py <out_dir> numpyA numpyB cudaA cudaB
#
# Runs perception_node exactly as dtp_params.yaml configures it, except use_sim_time (the bag's
# /clock) and cloud_backend, on a PRIVATE, localhost-only DDS domain (ROS_DOMAIN_ID 87) so nothing
# reaches the aircraft's graph and nothing on it reaches the replay. It opens no camera and no
# LiDAR: only camera topics, /tf, /tf_static and /lidar_slam/odom are played. Writes, per label:
#   targets_<label>.jsonl   every /dtp/targets and /dtp/perception_ok message
#   node_<label>.log        the node's log (backend, self-check verdict, per-frame cloud ms)
#   cpu_<label>.txt         CPU-seconds perception_node's process group used during the bag
#
# READ THIS BEFORE COMPARING TWO RUNS -- a replay is not deterministic, for two separate reasons.
#  1. SCHEDULE (which frames get processed). At 1x the first YOLO inference is a ~2.5 s TensorRT
#     warm-up stall that swallows a run-dependent number of colour frames, and load-dependent
#     TF-wait drops follow; the decimator's phase then differs for the rest of the bag. Measured
#     2026-09-13: two 1x runs of the same backend shared only ~1/3 of their stamps and differed by
#     0.3-0.4 cm median, 2.6-12 cm p95. FIX: WARM_S=60 RATE=0.25 -- every run then processed the
#     SAME 575 stamps.
#  2. INPUTS (what each frame is built from). Even on an identical schedule, two runs of the SAME
#     backend diverge after a few dozen frames: numpy vs numpy first differed at frame 77, then by
#     0.06 cm median / 1.7 cm p95 / 7.5 cm max, with identical track ids throughout. Most likely
#     the colour callback pairs with "the newest depth frame", which depends on executor ordering
#     (a walker moves ~3 cm per depth frame) -- a hypothesis, not proven. Not fixed here.
# So: compare each arm against a SECOND RUN OF ITSELF, never against zero. (docs/m5-m6-plan.md
# M6.2 needs input-deterministic replay for classifier results; this script does not provide it.)
#
# First use, 2026-09-13, cloud_backend numpy vs cuda on 20260910_flightlegal_bench: at 1x numpy
# 110 / 114 CPU-s vs cuda 41 / 41 over 59 s. Deterministic schedule: numpy-vs-numpy p95 1.71 cm,
# numpy-vs-cuda p95 1.65 / 2.18 cm -- the backend adds nothing measurable beyond replay noise.
# Evidence: ~/dtp_datasets/20260913_cloud_backend_replay/.

BACKEND=${1:?usage: replay_perception.sh <numpy|cuda> <out_dir> [label] [bag]}
OUT=${2:?usage: replay_perception.sh <numpy|cuda> <out_dir> [label] [bag]}
LABEL=${3:-$BACKEND}
BAG=${4:-$HOME/dtp_datasets/20260910_flightlegal_bench}
# RATE < 1 slows the bag so the node never drops or ages out a frame; see the note above.
RATE=${RATE:-1.0}
# WARM_S > 0: play the first WARM_S wall-seconds at 0.05x, then switch to RATE. The first YOLO
# inference is a ~2.5 s TensorRT warm-up stall; at RATE it swallows a run-dependent number of
# colour frames and shifts the decimator's phase for the rest of the bag. At 0.05x the stall
# spans ~4 frames, inside the subscription queue, so nothing is lost and the phase is repeatable.
WARM_S=${WARM_S:-0}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"

export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
# Via the ~/super_ws symlink path, not the repo path: from the repo it finds no workspace overlay.
# shellcheck disable=SC1091
source "$HOME/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh" >/dev/null
PARAMS=$HOME/super_ws/install/dtp_bringup/share/dtp_bringup/config/dtp_params.yaml

# setsid: each child leads its own process group, so teardown signals a group we own -- never a
# pattern matched against command lines (the pkill -f trap in CLAUDE.md).
setsid python3 "$HERE/record_targets.py" "$OUT/targets_$LABEL.jsonl" >"$OUT/rec_$LABEL.log" 2>&1 &
REC=$!
setsid ros2 run dtp_perception perception_node --ros-args --params-file "$PARAMS" \
    -p use_sim_time:=true -p cloud_backend:="$BACKEND" >"$OUT/node_$LABEL.log" 2>&1 &
NODE=$!

for _ in $(seq 1 90); do                 # TensorRT load + GPU warm-up + depth readiness
    grep -q "perception_node up" "$OUT/node_$LABEL.log" 2>/dev/null && break
    sleep 1
done
sleep 2

cpu_ticks() {
    local t=0 p
    for p in $(ps -o pid= -g "$NODE"); do
        t=$(( t + $(awk '{print $14 + $15}' "/proc/$p/stat" 2>/dev/null || echo 0) ))
    done
    echo "$t"
}
C0=$(cpu_ticks); T0=$(date +%s.%N)
START_RATE=$RATE
[ "$WARM_S" != "0" ] && START_RATE=0.05
ros2 bag play "$BAG" --clock --rate "$START_RATE" \
    --topics /camera/color/image_raw/compressed /camera/depth/image_raw \
             /camera/color/camera_info /camera/depth/camera_info /tf /tf_static /lidar_slam/odom \
    >"$OUT/play_$LABEL.log" 2>&1 &
PLAY=$!
if [ "$WARM_S" != "0" ]; then
    sleep "$WARM_S"
    ros2 service call /rosbag2_player/set_rate rosbag2_interfaces/srv/SetRate "{rate: $RATE}" \
        >>"$OUT/play_$LABEL.log" 2>&1
fi
wait "$PLAY"
C1=$(cpu_ticks); T1=$(date +%s.%N)
python3 -c "import sys; print('%s cloud_backend=%s cpu_s=%.2f wall_s=%.2f' % (sys.argv[1], \
    sys.argv[2], (int(sys.argv[4]) - int(sys.argv[3])) / float(sys.argv[5]), \
    float(sys.argv[7]) - float(sys.argv[6])))" "$LABEL" "$BACKEND" "$C0" "$C1" \
    "$(getconf CLK_TCK)" "$T0" "$T1" | tee "$OUT/cpu_$LABEL.txt"

sleep 3
kill -INT -- -"$NODE" -"$REC" 2>/dev/null
for _ in $(seq 1 20); do
    kill -0 "$NODE" 2>/dev/null || kill -0 "$REC" 2>/dev/null || break
    sleep 0.5
done
kill -KILL -- -"$NODE" -"$REC" 2>/dev/null
grep -E "verified against numpy|DISAGREES|DISABLED|cannot start" "$OUT/node_$LABEL.log" || true
