#!/usr/bin/env bash
#
# Reproducible M1 setup for SUPER human-follow on a Jetson + Livox MID360 (ROS2 Humble).
# Clones SUPER / FAST-LIO / Livox driver+SDK at PINNED commits, applies the required
# patches and configs from this repo, and builds everything.
#
# Best-effort installer — review it before running on a new machine. It is idempotent
# enough to re-run. It assumes ROS 2 Humble is already installed and on PATH.
#
# Usage:   bash scripts/setup.sh
# Override locations with env vars: SUPER_WS, WS_LIVOX, SDK_DIR
#
set -eo pipefail   # not -u: ROS setup.bash references unset vars on a cold shell

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUPER_WS="${SUPER_WS:-$HOME/super_ws}"
WS_LIVOX="${WS_LIVOX:-$HOME/ws_livox}"
SDK_DIR="${SDK_DIR:-$HOME/Livox-SDK2}"

# --- pinned commits (known-good on Orin Nano + ROS2 Humble, 2026-06) ---------
SUPER_SHA=2ad3419c127a617c6d7df6925e81a14175a9c096      # hku-mars/SUPER  (master)
FASTLIO_SHA=a4743b095409588842a5b30ddfa27e29d2f99164    # hku-mars/FAST_LIO (ROS2 branch)
LIVOX_DRV_SHA=6b9356cadf77084619ba406e6a0eb41163b08039  # Livox-SDK/livox_ros_driver2
LIVOX_SDK_SHA=6a940156dd7151c3ab6a52442d86bc83613bd11b  # Livox-SDK/Livox-SDK2
PX4_MSGS_SHA=86d8239e962f6939e05c3737784f60c02fa884db   # PX4/px4_msgs (release/1.17)

source /opt/ros/humble/setup.bash

echo "==> [1/5] apt dependencies"
sudo apt-get update
# NB: no ros-humble-rosfmt -- that package does not exist for Humble (it was a ROS 1
# package). Only SUPER's ros1.CMakeLists.txt references rosfmt; the ROS 2 build uses the
# fmt/ headers vendored inside super_planner/ and rog_map/. Listing it aborted the whole
# apt transaction with "Unable to locate package", installing nothing.
# ros-humble-mavros-msgs: SUPER's mission_planner find_package()s mavros_msgs. This
#   project does NOT use MAVROS (the FC is PX4 + uXRCE-DDS) -- these are message
#   definitions only, needed so the package compiles. It built on the old Orin Nano
#   purely because MAVROS was left over from the ArduCopter era.
# libglfw3-dev: marsim_render (the simulator renderer) find_package()s glfw3.
sudo apt-get install -y git cmake build-essential python3-colcon-common-extensions \
    libeigen3-dev libyaml-cpp-dev libdw-dev ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-mavros-msgs libglfw3-dev
[ -e /usr/include/Eigen ] || sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

echo "==> [2/5] Livox-SDK2 (build + install)"
[ -d "$SDK_DIR" ] || git clone https://github.com/Livox-SDK/Livox-SDK2.git "$SDK_DIR"
git -C "$SDK_DIR" fetch -q && git -C "$SDK_DIR" checkout -q "$LIVOX_SDK_SHA"
mkdir -p "$SDK_DIR/build"
( cd "$SDK_DIR/build" && cmake .. >/dev/null && make -j"$(nproc)" && sudo make install )

echo "==> [3/5] livox_ros_driver2 (ws_livox overlay)"
mkdir -p "$WS_LIVOX/src"
[ -d "$WS_LIVOX/src/livox_ros_driver2" ] || \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$WS_LIVOX/src/livox_ros_driver2"
git -C "$WS_LIVOX/src/livox_ros_driver2" fetch -q
git -C "$WS_LIVOX/src/livox_ros_driver2" checkout -q "$LIVOX_DRV_SHA"
cp "$REPO_DIR/config/MID360_config.json" "$WS_LIVOX/src/livox_ros_driver2/config/MID360_config.json"
# FAST-LIO needs the Livox CustomMsg (per-point timestamps): xfer_format = 1
sed -i 's/^xfer_format *=.*/xfer_format   = 1    # 1 = CustomMsg (required by FAST-LIO)/' \
    "$WS_LIVOX/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"
( cd "$WS_LIVOX/src/livox_ros_driver2" && ./build.sh humble )

echo "==> [4/5] SUPER + FAST-LIO + super_bringup (super_ws)"
mkdir -p "$SUPER_WS/src"
# SUPER
[ -d "$SUPER_WS/src/SUPER" ] || git clone https://github.com/hku-mars/SUPER.git "$SUPER_WS/src/SUPER"
git -C "$SUPER_WS/src/SUPER" fetch -q && git -C "$SUPER_WS/src/SUPER" checkout -q "$SUPER_SHA"
bash "$SUPER_WS/src/SUPER/scripts/select_ros_version.sh" ROS2
# patch: lower virtual ground so ground-level operation isn't rejected (odom z ~ 0)
sed -i 's/^\( *virtual_ground_height:\).*/\1 -1.0/' \
    "$SUPER_WS/src/SUPER/super_planner/config/click_smooth_ros2.yaml"
# patch: first-flight speed/rotation limits. Upstream ships the benchmark tune
# (max_vel 5.0, max_acc 5.0, max_omg 2.5, yaw_dot_max 3.0). These are the only
# speed limit in the SUPER -> px4_offboard_bridge -> PX4 chain -- the bridge
# forwards whatever SUPER plans and clamps nothing. Raise them deliberately,
# not by forgetting to re-apply this. See docs/flight-test-plan.md.
sed -i -E 's/^( *max_vel:).*/\1 0.1/;      s/^( *max_acc:).*/\1 0.3/;
           s/^( *max_omg:).*/\1 1.5/;      s/^( *yaw_dot_max:).*/\1 0.5/' \
    "$SUPER_WS/src/SUPER/super_planner/config/click_smooth_ros2.yaml"
# patch: robot_r = the MEASURED prop-tip radius (0.25 m, 2026-09-14) + 0.05 m margin. Upstream's
# 0.2 planned corridors 5 cm too tight for this airframe. follow_test_ros2.yaml carries the same
# value inside patch 0001. Re-measure after any prop or arm change.
sed -i -E 's/^( *robot_r:).*/\1 0.30/' \
    "$SUPER_WS/src/SUPER/super_planner/config/click_smooth_ros2.yaml"
# patch: enable the DTP follow-goal input on the click config too.
#
# WHY: upstream's click path (/goal_pose -> setGoalPosiAndYaw) OVERWRITES the z
# of every goal with fsm/click_height (fsm.cpp:188) and SILENTLY DROPS any goal
# within 0.1 m (fsm.cpp:201) -- so the web GCS's altitude control cannot work,
# and a rotation cannot get through, with no error anywhere. setFollowGoal()
# preserves z, has no drop radius, and answers on /planning/goal_status. This
# turns three silent behaviours into reported ones.
#
# click_height is deliberately LEFT ALONE, so an RViz "2D Goal Pose" click --
# which always sends z=0 -- keeps flying at 1.5 m instead of at the floor.
#
# On a clean install this is a NO-OP: the follow path comes from drone-dtp's
# SUPER patch, which this script does not apply, and unpatched SUPER never
# reads the key (LoadParam defaults it to false). Idempotent.
grep -q '^ *follow_goal_en:' \
    "$SUPER_WS/src/SUPER/super_planner/config/click_smooth_ros2.yaml" || \
sed -i '/^ *click_height:/i\  follow_goal_en: true\n  follow_goal_topic: "/planning/follow_goal"\n  goal_status_topic: "/planning/goal_status"' \
    "$SUPER_WS/src/SUPER/super_planner/config/click_smooth_ros2.yaml"
# patch: yaw_mode 3 (YAW_TO_TARGET) on the click config -- a decision, 2026-09-12.
#
# Upstream loads yaw_mode and never reads it, so 1 and 2 behave identically.
# Only the DTP patch reads it, and only tests for 3 (super_planner.cpp:795):
# the heading then aims from the trajectory's END at /planning/yaw_target, and
# with no target fresher than 0.5 s it falls back to heading-to-velocity.
#
# COST: under 3 the goal yaw is never applied -- an RViz 2D-Goal-Pose arrow's
# direction is ignored and the GCS rotate controls are refused (rc.py). Put 1
# back here, not just in the workspace, if you want rotate.
#
# Harmless on a clean install (unpatched SUPER never reads it). Idempotent.
sed -i -E 's/^( *# Yaw mode: 1 heading to velocity, 2 heading to goal)$/\1, 3 heading to followed target/;
           s/^( *yaw_mode:).*/\1 3/' \
    "$SUPER_WS/src/SUPER/super_planner/config/click_smooth_ros2.yaml"
# patch: ROG-Map raycasting ON, p_max 0.95 -- a decision, 2026-09-12.
#
# WHY: raycasting is the only thing that frees a cell. Upstream ships it off,
# and then the map only ever ADDS occupancy -- a person who walks past the
# LiDAR stays in it as a permanent wall. p_max 0.98 -> 0.95 caps how sure a
# cell can get, so one you stood in clears after ~31 rays through it instead
# of ~54 (p_miss 0.49 is very weak; one hit still undoes ~55 misses).
#
# COST: thin real obstacles seen by few rays also fade faster; raycasting
# costs fsm_node CPU. The range is anchored to the raycasting: block because
# the file has several other `enable:` keys. Idempotent.
sed -i -E '/^ *raycasting:/,/^ *enable:/ s/^( *enable:).*/\1 true/;
           s/^( *p_max:).*/\1 0.95/' \
    "$SUPER_WS/src/SUPER/super_planner/config/click_smooth_ros2.yaml"
# FAST-LIO (ROS2 branch) + ikd-Tree submodule
[ -d "$SUPER_WS/src/FAST_LIO_ROS2" ] || \
    git clone --branch ROS2 --recurse-submodules https://github.com/hku-mars/FAST_LIO.git "$SUPER_WS/src/FAST_LIO_ROS2"
git -C "$SUPER_WS/src/FAST_LIO_ROS2" fetch -q
git -C "$SUPER_WS/src/FAST_LIO_ROS2" checkout -q "$FASTLIO_SHA"
git -C "$SUPER_WS/src/FAST_LIO_ROS2" submodule update --init --recursive
# patch: C++14 -> C++17 (Humble rclcpp 16.x headers are C++17-only)
sed -i 's/c++14/c++17/g; s/c++0x/c++17/g; s/CMAKE_CXX_STANDARD 14/CMAKE_CXX_STANDARD 17/g' \
    "$SUPER_WS/src/FAST_LIO_ROS2/CMakeLists.txt"
# config: dense, MID360 extrinsics, CustomMsg
cp "$REPO_DIR/config/mid360.yaml" "$SUPER_WS/src/FAST_LIO_ROS2/config/mid360.yaml"
# patch: fix ~1.35 s odometry latency in the ROS2 port (see docs/fact-fastlio-latency.md).
# Upstream spins a SINGLE-threaded executor with no callback groups and a depth-20
# cloud queue; the always-overdue 100 Hz timer starves the cloud subscription, the
# queue fills once at startup and never drains -> permanently ~1.4 s stale odometry
# with correct timestamps (silent). Idempotent; safe to re-run.
python3 "$REPO_DIR/patches/fastlio_executor.py" \
    "$SUPER_WS/src/FAST_LIO_ROS2/src/laserMapping.cpp"
# patch: rename the TF frames to say what they are (camera_init -> lidar_init,
# body -> lidar_imu). Upstream's "body" is NOT the drone body -- it is the MID360's
# internal IMU, and reading it as the vehicle body is what produces a wrong
# EKF2_EV_POS_* lever arm. "camera_init" has no camera behind it. Nothing downstream
# reads these names (ROG-Map ignores frame_id/TF entirely). Must run AFTER the
# executor patch, which anchors on unrenamed text. Idempotent; safe to re-run.
python3 "$REPO_DIR/patches/fastlio_frames.py" \
    "$SUPER_WS/src/FAST_LIO_ROS2/src/laserMapping.cpp"
# --- upstream build defects at the pinned SHAs (see docs/build-fixes.md) -----
# (a) perfect_drone_sim includes pcl_conversions headers but never find_package()s
#     it, so ament never adds the include path. Patch the ROS2 template *and* the
#     generated CMakeLists, so re-running select_ros_version.sh keeps the fix.
for f in "$SUPER_WS/src/SUPER/mars_uav_sim/perfect_drone_sim/ros/ros2.CMakeLists.txt" \
         "$SUPER_WS/src/SUPER/mars_uav_sim/perfect_drone_sim/CMakeLists.txt"; do
    [ -f "$f" ] || continue
    grep -q 'find_package(pcl_conversions' "$f" && continue
    sed -i 's/^find_package(tf2_ros REQUIRED)$/&\nfind_package(pcl_conversions REQUIRED)/' "$f"
    sed -i 's/^\( *\)tf2_ros$/&\n\1pcl_conversions/' "$f"
done
# (b) Under C++17 (upstream's own -std=c++17) <cmath> no longer guarantees a
#     global ::isnan, and Eigen injects its own overloads -> "not declared" /
#     ambiguous. Qualify it as std::isnan in the files SUPER actually compiles.
#     Scoped to these files ONLY: the vendored fmt/ and glm/ headers *define*
#     their own isnan and must not be touched.
for f in super_planner/src/super_core/fsm.cpp \
         super_planner/src/super_core/super_planner.cpp \
         super_planner/src/utils/polytope.cpp \
         super_planner/src/utils/geometry_utils.cpp \
         super_planner/include/ros_interface/ros2/ros2_adapter.hpp \
         super_planner/include/ros_interface/ros1/ros1_adapter.hpp; do
    p="$SUPER_WS/src/SUPER/$f"
    [ -f "$p" ] || continue
    # negative lookbehind keeps std::isnan / Eigen::isnan / x.isnan() untouched
    perl -pi -e 's/(?<![\w:.>])isnan\(/std::isnan(/g' "$p"
    grep -q '#include <cmath>' "$p" || sed -i '0,/^#include/s//#include <cmath>\n&/' "$p"
done

# px4_msgs -- required by super_bringup/scripts/px4_odom_bridge.py (M2). Must match the
# firmware: the FC runs PX4 v1.17.0, so this tracks release/1.17. Without it the bridge
# cannot even import (`from px4_msgs.msg import VehicleOdometry`).
[ -d "$SUPER_WS/src/px4_msgs" ] || \
    git clone --branch release/1.17 https://github.com/PX4/px4_msgs.git "$SUPER_WS/src/px4_msgs"
git -C "$SUPER_WS/src/px4_msgs" fetch -q
git -C "$SUPER_WS/src/px4_msgs" checkout -q "$PX4_MSGS_SHA"

# super_bringup integration package
rm -rf "$SUPER_WS/src/super_bringup"
cp -r "$REPO_DIR/super_bringup" "$SUPER_WS/src/super_bringup"

# DTP follower stack + the SUPER follow-goal/YAW_TO_TARGET patch (G0.4, 2026-09-13).
# Without this, a clean install yields a SUPER the DTP follower cannot drive: no follow_goal_en,
# no /planning/goal_status, no yaw_mode 3. The patch adds a HARD find_package(dtp_msgs) build dep
# to super_planner, so dtp_msgs must be in the workspace -- hence the symlink. drone-dtp is a
# SYMLINK (not a copy) on purpose: edits in the repo are what the drone runs (drone-dtp/README.md).
# MUST run AFTER the std::isnan fix above -- the patch's context lines carry std::isnan and it will
# not apply to pristine upstream (see drone-dtp/scripts/apply_super_patch.sh).
# Only on a checkout that actually has drone-dtp/ (the dtp-follower-stack branch); an M1-only
# checkout skips it and builds a stock SUPER.
if [ -d "$REPO_DIR/drone-dtp/ros2_ws/src/dtp_msgs" ]; then
    echo "==> [4b/5] DTP follower stack + SUPER follow-goal patch"
    ln -sfn "$REPO_DIR/drone-dtp" "$SUPER_WS/src/drone-dtp"
    bash "$REPO_DIR/drone-dtp/scripts/apply_super_patch.sh" "$SUPER_WS/src/SUPER"
else
    echo "==> [4b/5] no drone-dtp/ in this checkout -- building stock (M1-only) SUPER"
fi

echo "==> [5/5] build super_ws"
source "$WS_LIVOX/install/setup.bash"
( cd "$SUPER_WS" && colcon build --symlink-install )

echo
echo "Done. Next:"
echo "  1) sudo bash $REPO_DIR/scripts/network_setup.sh   # MID360 static IP (edit IPs per unit)"
echo "  2) bash $REPO_DIR/scripts/run.sh                  # bring up driver + FAST-LIO + SUPER"
