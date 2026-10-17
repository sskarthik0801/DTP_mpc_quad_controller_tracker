#!/usr/bin/env bash
# Install the camera<->LiDAR calibration SOLVER (koide3/direct_visual_lidar_calibration).
#
# WHY A SCRIPT AND NOT A README SECTION: the solver has three heavy from-source dependencies
# with exact pinned versions, and getting any one wrong fails late and confusingly.
# Recalibration must stay cheap (mounts change every airframe revision -- plan/dtp-imp/19),
# so environment setup has to be one command, repeatable, and idempotent.
#
# Run on the WORKSTATION, not the Jetson: solving is offline work on captured bags, so the
# flight computer never needs GTSAM/Ceres/Iridescence.
#
#   bash scripts/install_calib_solver.sh
#
# NO SUDO IS USED. The three from-source deps install into a user-owned prefix
# (~/calib_deps/install) rather than /usr/local. That is deliberate:
#   * it runs unattended -- a "sudo make install" partway through a 40 min build blocks on a
#     password prompt long after you have walked away;
#   * it is removable with "rm -rf ~/calib_deps ~/calib_ws", leaving no root-owned files
#     scattered through /usr/local;
#   * it cannot collide with a system libceres/libgtsam a future apt package might install.
# The only root-requiring step is a few apt packages, which the script detects and prints for
# you to run yourself.
#
# Safe to re-run: each stage is skipped if already satisfied, so a failure resumes rather than
# restarting from scratch.

set -euo pipefail

# --- pinned versions -------------------------------------------------------
# Upstream pins these exactly; drifting from them is a known source of build failures.
GTSAM_TAG="4.2a9"
CERES_COMMIT="e47a42c2957951c9fafcca9995d9927e15557069"

SRC_ROOT="${SRC_ROOT:-$HOME/calib_deps}"
PREFIX="${PREFIX:-$SRC_ROOT/install}"
CALIB_WS="${CALIB_WS:-$HOME/calib_ws}"
ENV_FILE="${CALIB_WS}/calib_env.sh"

# CMake >= 4 hard-refuses projects declaring cmake_minimum_required(VERSION <3.5), which all
# three pinned deps do. Without this they fail immediately on a modern Ubuntu box.
CMAKE_COMPAT="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"

# GTSAM translation units are memory-hungry (~1.5 GB each). Capping jobs avoids an OOM that
# shows up as a bare "c++: fatal error: Killed signal terminated program cc1plus".
NPROC="$(nproc)"
GTSAM_JOBS="${GTSAM_JOBS:-8}"
if [ "$NPROC" -lt 8 ]; then GTSAM_JOBS="$NPROC"; fi

say()  { echo; echo "== $*"; }
skip() { echo "   [have] $*"; }

# --- preflight -------------------------------------------------------------
say "Preflight"
if [ ! -d /opt/ros/humble ]; then echo "ERROR: /opt/ros/humble not found."; exit 1; fi
# ROS 2 setup.bash is not `set -u` safe (it reads AMENT_TRACE_SETUP_FILES et al unguarded), so
# nounset has to come off around it or the script dies on the very first line of the preflight.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
echo "   ROS 2 Humble : ok"
echo "   cores        : ${NPROC} (GTSAM capped to ${GTSAM_JOBS})"
echo "   prefix       : ${PREFIX}   (user-owned, no sudo)"
echo "   solver ws    : ${CALIB_WS}"
cmake --version | head -1 | sed "s/^/   cmake        : /"
mkdir -p "${SRC_ROOT}" "${PREFIX}"

export CMAKE_PREFIX_PATH="${PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# --- apt gate --------------------------------------------------------------
# Only Iridescence needs these; GTSAM and Ceres build with what is already installed. So the
# script does as much as it can BEFORE demanding root, rather than refusing to start.
# These are the Iridescence dependencies ONLY -- this list is what the gate below blocks on, so
# anything added here becomes a hard root requirement. Ceres also wants libgoogle-glog-dev (not in
# upstream's documented list; its configure hard-fails without it), but that is deliberately NOT
# listed here: the Ceres stage falls back to bundled MINIGLOG instead, so a Ceres-only dependency
# must never block Iridescence.
# xvfb: EVERY koide3 tool initialises GLFW and SEGFAULTS without a display -- including the
# non-interactive preprocess/calibrate. Relying on a logged-in desktop session is fragile:
# MEASURED 2026-09-07, the session this was built against (:1) was gone and the only live
# displays belonged to OTHER USERS, so they were (correctly) inaccessible. xvfb-run makes
# the non-interactive stages genuinely headless.
APT_PKGS="libomp-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev libboost-all-dev libeigen3-dev xvfb"
check_apt() {
  MISSING=""
  for p in $APT_PKGS; do
    if ! dpkg -s "$p" >/dev/null 2>&1; then MISSING="$MISSING $p"; fi
  done
}
# Checked here only to warn early; re-checked at the gate below so that installing the packages
# WHILE the GTSAM/Ceres builds run still lets this same invocation continue through to the end.
check_apt
if [ -n "$MISSING" ]; then
  echo "   apt (later)  : missing$MISSING"
  echo "                  install these while GTSAM/Ceres build and this run continues:"
  echo "                  sudo apt-get install -y$MISSING"
fi

# --- 1. GTSAM --------------------------------------------------------------
say "1/4  GTSAM ${GTSAM_TAG}"
if [ -f "${PREFIX}/include/gtsam/geometry/Pose3.h" ]; then
  skip "GTSAM already installed in prefix"
else
  cd "${SRC_ROOT}"
  if [ ! -d gtsam ]; then git clone https://github.com/borglab/gtsam; fi
  cd gtsam
  git checkout "${GTSAM_TAG}"
  mkdir -p build && cd build
  # GTSAM_USE_SYSTEM_EIGEN=ON is REQUIRED on 22.04: the bundled Eigen otherwise collides with
  # the system one PCL/OpenCV already link, producing an ABI crash at runtime, not build time.
  cmake .. ${CMAKE_COMPAT} \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON
  make -j"${GTSAM_JOBS}"
  make install
fi

# --- 2. Ceres --------------------------------------------------------------
say "2/4  Ceres (pinned)"
if [ -f "${PREFIX}/include/ceres/ceres.h" ]; then
  skip "Ceres already installed in prefix"
else
  cd "${SRC_ROOT}"
  if [ ! -d ceres-solver ]; then
    git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver
  fi
  cd ceres-solver
  git checkout "${CERES_COMMIT}"
  mkdir -p build && cd build
  # Ceres refuses to configure without glog. Prefer the system package (upstream-tested), but
  # fall back to Ceres's own bundled MINIGLOG so a missing apt package cannot hard-block a
  # sudo-free install. Without this the script dies here and needs another root round-trip.
  CERES_GLOG=""
  if ! dpkg -s libgoogle-glog-dev >/dev/null 2>&1; then
    echo "   glog not installed -> building Ceres with bundled MINIGLOG"
    CERES_GLOG="-DMINIGLOG=ON"
  fi
  cmake .. ${CMAKE_COMPAT} ${CERES_GLOG} \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
  make -j"${NPROC}"
  make install
fi

# --- apt stop, if needed ---------------------------------------------------
check_apt   # re-check: they may have been installed while GTSAM/Ceres were building
if [ -n "$MISSING" ]; then
  say "STOP -- apt packages required for Iridescence"
  echo "GTSAM and Ceres are built and installed. Iridescence needs these first:"
  echo
  echo "    sudo apt-get install -y$MISSING"
  echo
  echo "Then re-run this script; it resumes from here."
  exit 2
fi

# --- 3. Iridescence (viewer) ----------------------------------------------
say "3/4  Iridescence"
if [ -f "${PREFIX}/include/guik/viewer/light_viewer.hpp" ]; then
  skip "Iridescence already installed in prefix"
else
  cd "${SRC_ROOT}"
  if [ ! -d iridescence ]; then
    git clone https://github.com/koide3/iridescence --recursive
  fi
  cd iridescence
  git submodule update --init --recursive
  mkdir -p build && cd build
  cmake .. ${CMAKE_COMPAT} \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release
  make -j"${NPROC}"
  make install
fi

# --- 4. the solver package -------------------------------------------------
say "4/4  direct_visual_lidar_calibration"
mkdir -p "${CALIB_WS}/src"
cd "${CALIB_WS}/src"
if [ ! -d direct_visual_lidar_calibration ]; then
  git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
fi
cd "${CALIB_WS}"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${PREFIX}"

# --- env file --------------------------------------------------------------
# A custom prefix means the loader will not find these libraries on its own. Rather than leave
# that as a footgun, ship one file that sets up everything the solver needs.
{
  echo "# Generated by scripts/install_calib_solver.sh -- source before running the solver."
  echo "source /opt/ros/humble/setup.bash"
  echo "source ${CALIB_WS}/install/setup.bash"
  echo "export LD_LIBRARY_PATH=${PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
  echo "export CMAKE_PREFIX_PATH=${PREFIX}:\${CMAKE_PREFIX_PATH:-}"
  echo ""
  echo "# EVERY koide3 tool initialises GLFW -- including 'preprocess' and 'calibrate', which are"
  echo "# otherwise non-interactive. Without a display they do not degrade gracefully: they"
  echo "# SEGFAULT after 'failed to initialize GLFW' and produce zero output. Over SSH, DISPLAY is"
  echo "# unset, so default to the console session's X server (hardware GL, no Xvfb needed)."
  echo "# At the console DISPLAY is already set, so these are no-ops there."
  echo "# Find a display we can actually USE. Hardcoding one is fragile: the session number"
  echo "# depends on who logged in when, and another user's display is (correctly) denied to us."
  echo "for _d in \"\$DISPLAY\" :0 :1 :2 :3; do"
  echo "    [ -n \"\$_d\" ] || continue"
  echo "    if DISPLAY=\"\$_d\" timeout 5 xdpyinfo >/dev/null 2>&1; then"
  echo "        export DISPLAY=\"\$_d\"; break"
  echo "    fi"
  echo "    unset DISPLAY"
  echo "done"
  echo "export XAUTHORITY=\${XAUTHORITY:-\$HOME/.Xauthority}"
  echo "if [ -z \"\${DISPLAY:-}\" ]; then"
  echo "    echo \"[calib_env] no usable X display. koide3 tools SEGFAULT headless -- prefix them:\""
  echo "    echo \"[calib_env]   xvfb-run -a --server-args=\\\"-screen 0 1280x1024x24\\\" ros2 run ...\""
  echo "fi"
} > "${ENV_FILE}"

say "Done"
echo "Solver installed (no root; everything under ${PREFIX} and ${CALIB_WS})."
echo
echo "Use it:  source ${ENV_FILE}"
echo
echo "Workflow (see plan/dtp-imp/19-camera-lidar-calibration-runbook.md):"
echo "  ros2 run direct_visual_lidar_calibration preprocess <bags_dir> <out_dir> -av"
echo "  ros2 run direct_visual_lidar_calibration initial_guess_manual <out_dir>"
echo "  ros2 run direct_visual_lidar_calibration calibrate <out_dir>"
echo
echo "Then convert calib.json to our runtime extrinsic:"
echo "  python3 -m dtp_perception.calibration.import_koide3 --calib_json <out_dir>/calib.json --out <run>/T_camera_lidar.txt"
echo
echo "NOTE: koide3 emits T_lidar_camera; we need T_camera_lidar. The importer inverts it for"
echo "you -- do not hand-copy that matrix."
