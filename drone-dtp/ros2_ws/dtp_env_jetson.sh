# Source this (do NOT execute) on the target JETSON ORIN NX (JetPack 6, system-python deploy):
#     source ros2_ws/dtp_env_jetson.sh
#
# Adapted from the reference repo's dtp_env_agx.sh for OUR hardware (decision D8/CLAUDE.md SS4:
# Jetson ORIN NX 16GB, not AGX Orin -- same JetPack-6/system-python deploy pattern, different
# module). There is NO project .venv on the Jetson -- torch/ultralytics/etc live in SYSTEM
# python3 (~/.local, the native Jetson pattern: pip3 install --no-cache ..., per plan/dtp-imp/05
# Phase 0 hardware bring-up), so `ros2 run` (whose console-script shebang is system python3)
# imports them directly with no PYTHONPATH bridging needed for those. Layers:
#   1. ROS2 Humble             -> rclpy and friends
#   2. the EXTERNAL colcon workspace overlay (see dtp_env.sh's header comment for why this is
#      NOT a local ros2_ws/install/ -- same reasoning applies here)
#   3. Livox driver overlay    -> a SEPARATE workspace per the Livox SDK2 build instructions
#      (plan/dtp-imp/05 M0.3 Step 0.3.2); needed for `ros2 launch ... livox_mid360...` to find
#      livox_ros_driver2_node
#   4. CUDA on PATH            -> nvcc / TensorRT engine export (CUDA *libs* are already global
#      via ld.so.conf.d once JetPack is installed -- no LD_LIBRARY_PATH needed)
#   5. repo root on PYTHONPATH -> so nodes `import detection_pipeline` / `transport` unchanged

_DTP_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_DTP_WS_ROOT="$(cd "${_DTP_REPO_ROOT}/../.." && pwd 2>/dev/null)"

source /opt/ros/humble/setup.bash

if [ -n "${_DTP_WS_ROOT}" ] && [ -f "${_DTP_WS_ROOT}/install/setup.bash" ]; then
    source "${_DTP_WS_ROOT}/install/setup.bash"
    echo "[dtp_env_jetson] overlay: ${_DTP_WS_ROOT}/install/setup.bash"
else
    echo "[dtp_env_jetson] WARNING: no built workspace overlay found at " \
         "${_DTP_WS_ROOT:-<repo not nested under .../src/...>}/install/setup.bash -- " \
         "build with 'colcon build' from the external workspace root first."
fi

# Livox driver overlay. Two possible layouts exist across machines this has been bootstrapped
# on (see plan/dtp-imp/16 and 17): a SEPARATE ~/ws_livox workspace (the vendor's own build.sh
# does a destructive `rm -rf` two directories up, so early bring-ups isolated it there), or
# livox_ros_driver2 built AS A SIBLING inside the main external workspace (${_DTP_WS_ROOT}/src/,
# per workspace.repos) -- in which case it's already covered by the overlay sourced above and
# there is nothing extra to do here. Check the unified layout first.
if [ -f "${_DTP_WS_ROOT}/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node" ]; then
    echo "[dtp_env_jetson] livox overlay: already covered by ${_DTP_WS_ROOT}/install/setup.bash (unified workspace)"
elif [ -f "${HOME}/ws_livox/install/setup.bash" ]; then
    source "${HOME}/ws_livox/install/setup.bash"
    echo "[dtp_env_jetson] livox overlay: ${HOME}/ws_livox/install/setup.bash (separate legacy workspace)"
else
    echo "[dtp_env_jetson] NOTE: livox_ros_driver2 not found in either the main workspace or" \
         "~/ws_livox -- Mid-360 driver launch will fail until it's built (plan/dtp-imp/05 M0.3 Step 0.3.2)."
fi

export PATH="/usr/local/cuda/bin:${PATH}"
export PYTHONPATH="${PYTHONPATH}:${_DTP_REPO_ROOT}"

# torch>=2.8 (the jp6/cu126 wheel from pypi.jetson-ai-lab.io) dlopen's libcudss.so.0 at import
# time, which JetPack's own CUDA toolkit does not ship -- pulled in instead via the
# `nvidia-cudss-cu12` pip package (user site-packages), so it must be added to the loader path
# by hand. Guarded: skip quietly if not installed (e.g. a future torch build drops this dependency).
_DTP_CUDSS_LIB="$(python3 -c 'import site; print(site.getusersitepackages())' 2>/dev/null)/nvidia/cu12/lib"
if [ -d "${_DTP_CUDSS_LIB}" ]; then
    export LD_LIBRARY_PATH="${_DTP_CUDSS_LIB}:${LD_LIBRARY_PATH}"
fi

echo "[dtp_env_jetson] ROS2 Humble + workspace overlay + repo root ready (system python; repo: ${_DTP_REPO_ROOT})"

# Must MATCH the systemd drop-ins and ~/.bashrc -- a participant on a different
# domain sees nothing, and reports no error while doing so.
