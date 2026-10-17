#!/usr/bin/env bash
# Deploy the camera<->LiDAR calibration (and the calibration module) to the drone NX.
#
# RUN THIS FROM THE WORKSTATION/LAPTOP, not on the NX:
#     bash scripts/deploy_calib.sh [ssh-host]        # default host: dtp-edge
#
# WHY IT COPIES FILES INSTEAD OF PULLING
# --------------------------------------
# `~/super_ws/src/drone-dtp` on the NX is a **tar-extracted subset** of this repo (packages +
# scripts + patches), NOT a git clone -- there is no `.git` there, so `git pull` cannot work.
# The first version of this script assumed a clone and aborted at step 0. If the NX is ever
# converted to a real clone, the pull path below takes over automatically.
#
# WHY IT VERIFIES BY CONTENT HASH
# --------------------------------
# The previous airframe's 0.117 m extrinsic lives at the *same path* and looks perfectly
# healthy. "The file exists" proves nothing; only the hash tells them apart. Confirmed at the
# 2026-09-07 deploy: the NX really was still serving the stale one, in both source and install.
#
# It verifies the **installed** copy under share/, not just the source: colcon can exit 0 while
# setup.py's glob silently fails to pick the YAML up, leaving a green build and a stale
# extrinsic on the aircraft.

set -u
HOST=${1:-dtp-edge}
REPO_LOCAL=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
P=ros2_ws/src/dtp_perception
R=/home/ideas-ad/super_ws/src/drone-dtp/$P

# What makes up a calibration deploy: the extrinsic, the gate module, and the setup.py that
# registers its entry point. Shipping the module without setup.py leaves it unrunnable.
#
# *** calib_io.py AND write_calib_yaml.py WERE MISSING UNTIL 2026-09-12. ***  The loop below
# checks that write_calib_yaml's entry point exists, so the deploy printed DEPLOY OK while the
# aircraft kept the old module -- which is how a calibration-identity bug would have survived a
# deploy that reported success. calib_io.py is the file that enforces the camera-body and
# foreign-LiDAR gates at node start-up; shipping a new extrinsic without it is shipping half a
# calibration.
FILES=(
    "$P/calib/drone_cam_livox.yaml"
    "$P/dtp_perception/calib_io.py"
    "$P/dtp_perception/calibration/write_calib_yaml.py"
    "$P/dtp_perception/calibration/score_alignment.py"
    "$P/dtp_perception/calibration/compare_calib.py"
    "$P/setup.py"
)

EXPECT_SHA=$(tr -d '\r' < "$REPO_LOCAL/$P/calib/drone_cam_livox.yaml" | sha256sum | cut -d' ' -f1)

echo "=== deploying to $HOST ==="
echo "  from : $REPO_LOCAL"
echo "  yaml : $EXPECT_SHA"

echo
echo "=== 0. is the NX repo a git clone? ==="
if ssh -o BatchMode=yes "$HOST" "[ -d /home/ideas-ad/super_ws/src/drone-dtp/.git ]"; then
    echo "  yes -- fast-forwarding instead of copying"
    if ! ssh -o BatchMode=yes "$HOST" 'cd ~/super_ws/src/drone-dtp && git fetch origin dtp-imp && git merge --ff-only origin/dtp-imp'; then
        echo "  pull failed; falling back to file copy"
        COPY=1
    fi
else
    echo "  no .git -- tar-extracted subset, copying files"
    COPY=1
fi

if [ "${COPY:-0}" = "1" ]; then
    echo
    echo "=== 1. copy ==="
    for f in "${FILES[@]}"; do
        scp -o BatchMode=yes "$REPO_LOCAL/$f" "$HOST:$R/${f#"$P/"}" || { echo "  FAILED: $f"; exit 1; }
        echo "  sent $f"
    done
    # A Windows checkout ships CRLF; a stray \r has broken bash and config parsing on this box.
    ssh -o BatchMode=yes "$HOST" "cd $R && sed -i 's/\r$//' calib/drone_cam_livox.yaml dtp_perception/calibration/*.py setup.py"
    echo "  CRLF stripped"
fi

echo
echo "=== 2. build + verify on the NX ==="
ssh -o BatchMode=yes -o ServerAliveInterval=30 "$HOST" "EXPECT_SHA=$EXPECT_SHA bash -s" <<'REMOTE'
set -u
WS=/home/ideas-ad/super_ws
FAIL=0
say() { printf '\n--- %s ---\n' "$1"; }

say "build"
cd "$WS" || exit 2
set +u
source /opt/ros/humble/setup.bash
[ -f "$WS/src/drone-dtp/ros2_ws/dtp_env_jetson.sh" ] && source "$WS/src/drone-dtp/ros2_ws/dtp_env_jetson.sh" >/dev/null 2>&1
set -u
colcon build --packages-select dtp_perception > /tmp/dtp_build.log 2>&1
BUILD_RC=$?
tail -4 /tmp/dtp_build.log
echo "  colcon exit: $BUILD_RC"
[ "$BUILD_RC" -ne 0 ] && { tail -40 /tmp/dtp_build.log; exit 1; }

say "installed extrinsic (what a running node reads)"
I=$WS/install/dtp_perception/share/dtp_perception/calib/drone_cam_livox.yaml
if [ -f "$I" ]; then
    GOT=$(tr -d '\r' < "$I" | sha256sum | cut -d' ' -f1)
    echo "  got  : $GOT"
    echo "  want : $EXPECT_SHA"
    if [ "$GOT" = "$EXPECT_SHA" ]; then
        echo "  MATCH"
        grep -m1 '^# note' "$I"
    else
        echo "  MISMATCH -- stale extrinsic still installed"
        grep -m1 '^# note' "$I"
        FAIL=1
    fi
else
    echo "  NOT INSTALLED at $I"; FAIL=1
fi

say "entry points"
set +u
source "$WS/install/setup.bash" >/dev/null 2>&1
set -u
EXES=$(ros2 pkg executables dtp_perception 2>/dev/null)
for e in score_alignment compare_calib import_koide3 write_calib_yaml validate_calib capture_calib_data fusion_node; do
    if echo "$EXES" | grep -q " $e\$"; then echo "  ok      $e"; else echo "  MISSING $e"; FAIL=1; fi
done

say "RESULT"
if [ "$FAIL" -eq 0 ]; then echo "DEPLOY OK"; else echo "DEPLOY INCOMPLETE"; fi
exit "$FAIL"
REMOTE
RC=$?

echo
if [ "$RC" -eq 0 ]; then
    echo "=== DEPLOY OK ==="
else
    echo "=== DEPLOY FAILED (rc=$RC) ==="
fi
exit "$RC"
