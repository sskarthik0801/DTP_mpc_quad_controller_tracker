# Source this (do NOT execute) to enter the DTP dev environment on a WORKSTATION:
#     source ros2_ws/dtp_env.sh
#
# Layers, in order:
#   1. project .venv        -> torch / ultralytics / numpy / pyzmq / etc (see CLAUDE.md SS9 for the
#                               exact dependency list -- create it first with:
#                                 python3 -m venv .venv && source .venv/bin/activate
#                                 pip install torch torchvision ultralytics lap opencv-python numpy \
#                                             pyzmq matplotlib pyyaml
#   2. ROS2 Humble          -> rclpy and friends
#   3. the EXTERNAL colcon workspace overlay (see below -- NOT a local ros2_ws/install/)
#   4. repo root on PYTHONPATH -> so nodes can `import detection_pipeline` / `transport` unchanged
#
# IMPORTANT structural difference from the reference repo this is modeled on: there this
# ros2_ws/ WAS itself the top-level colcon workspace. HERE it is not -- per SUPER's own
# install convention (top-level README: `mkdir -p super_ws/src && cd super_ws/src && git
# clone ... && cd .. && colcon build`), this WHOLE repo (super_planner/, rog_map/,
# mission_planner/, mars_uav_sim/, super_bringup/, AND this ros2_ws/) is cloned as ONE entry
# inside an external workspace's src/ folder. So the actual build output (install/setup.bash)
# lives TWO directories above this repo's root (repo_root/../.. == the external workspace
# root, e.g. super_ws/), not in a local ros2_ws/install/. colcon finds every package.xml
# recursively regardless of this nesting, but sourcing the overlay needs the real path.

_DTP_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_DTP_WS_ROOT="$(cd "${_DTP_REPO_ROOT}/../.." && pwd 2>/dev/null)"

if [ -d "${_DTP_REPO_ROOT}/.venv" ]; then
    source "${_DTP_REPO_ROOT}/.venv/bin/activate"
else
    echo "[dtp_env] WARNING: no .venv/ found at ${_DTP_REPO_ROOT} -- create one first (see this" \
         "file's header comment). Continuing without it; torch/ultralytics/etc will not import."
fi

source /opt/ros/humble/setup.bash

if [ -n "${_DTP_WS_ROOT}" ] && [ -f "${_DTP_WS_ROOT}/install/setup.bash" ]; then
    source "${_DTP_WS_ROOT}/install/setup.bash"
    echo "[dtp_env] overlay: ${_DTP_WS_ROOT}/install/setup.bash"
else
    echo "[dtp_env] WARNING: no built workspace overlay found at " \
         "${_DTP_WS_ROOT:-<repo not nested under .../src/...>}/install/setup.bash -- " \
         "this repo must be cloned as <some_ws>/src/<this-repo> and built with " \
         "'colcon build' from <some_ws>/ before 'ros2 run'/'ros2 launch' will find dtp_msgs, " \
         "dtp_perception, follow_planner, or command_mux."
fi

# `ros2 run`/`ros2 launch` invoke console scripts whose shebang is the SYSTEM python3
# (colcon stamps that regardless of the active venv). Exposing the venv's site-packages on
# PYTHONPATH lets whichever python3 actually runs the node still import torch/ultralytics/etc.
# Appended AFTER the ROS paths so ROS's own python deps win any name clash. Repo root last ->
# `import detection_pipeline` / `import transport` resolve unchanged, matching the reference
# repo's contract (plan/dtp-imp/03 SS1).
if [ -d "${_DTP_REPO_ROOT}/.venv" ]; then
    _DTP_VENV_SITE="${_DTP_REPO_ROOT}/.venv/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages"
    export PYTHONPATH="${PYTHONPATH}:${_DTP_VENV_SITE}:${_DTP_REPO_ROOT}"
else
    export PYTHONPATH="${PYTHONPATH}:${_DTP_REPO_ROOT}"
fi

echo "[dtp_env] venv + ROS2 Humble + workspace overlay ready (repo: ${_DTP_REPO_ROOT})"
