#!/usr/bin/env bash
# Apply our SUPER additions to a SUPER checkout, in order:
#   patches/0001-super-yaw-to-target-and-follow-goal.patch   YAW_TO_TARGET + setFollowGoal/GoalStatus
#   patches/0002-rog-map-target-exclusion.patch              M5.3a target-point filter (2026-09-13):
#       ROG-Map keeps the followed person's returns out of the map (rog_map/target_exclusion,
#       default OFF). Touches rog_map as well as super_planner, so BOTH need rebuilding. Its
#       fsm_ros2.hpp context comes from 0001, so it only applies on top of 0001.
#   patches/0003-rog-map-virtual-plane-ray-clip.patch        upstream bug fix (2026-09-13): a ray to a
#       return below virtual_ground / above virtual_ceil was clipped to <= 1 m from the sensor
#       instead of to the plane, so it cleared nothing -- a walked-away person's lower body stayed
#       occupied forever whenever the floor is below -1.0 in lidar_init. Always on (a fix, not a
#       feature). rog_map only, but fsm_node links it statically, so rebuild super_planner too.
#   patches/0004-super-loss-oracle.patch                    M6.1/M6.4 loss oracle (2026-09-14):
#       fsm_node classifies WHY the followed person was lost against ROG-Map (OCCLUDED / UNMAPPED /
#       LOST / OUT_OF_VIEW), builds the reappearance belief and a pre-positioning viewpoint, and
#       publishes dtp_msgs/LossEvent on /planning/loss_event (loss_oracle/enable, ON in
#       follow_test_ros2.yaml; it only reads the map). super_planner only: a new header
#       (super_core/loss_oracle.hpp), fsm/config.hpp, fsm_ros2.hpp and the follow config. Needs the
#       LossEvent message, i.e. dtp_msgs rebuilt first. Its fsm_ros2.hpp context is 0002's.
#   patches/0005-super-belief-around-obstacles.patch        M6.4b (2026-09-14): the reappearance
#       rollouts walk AROUND occupied cells instead of through trunks (loss_oracle/rollout_collision,
#       B) and, when most rollouts stay hidden, the yaw target and viewpoint look BEHIND the
#       occluder (loss_oracle/hidden_look, B2). Both OFF in code, ON in follow_test_ros2.yaml. Same
#       four files as 0004 and nothing else; its context is 0004's, so it only applies on top of it.
#       A NEW patch rather than a regenerated 0004: 0004 is skipped once loss_oracle_en exists, and
#       0004's diff base predates the robot_r 0.30 change carried by 0001.
#   patches/0006-super-yaw-hold-on-short-trajectory.patch   yaw hold (2026-09-14, after the first
#       Offboard flight): YawTrajOpt's free-end yaw keeps the current heading when the whole
#       trajectory moves less than 0.3 m. Upstream aligned it to that displacement, which on a
#       re-planned hold goal is the aircraft's own wobble; the aircraft pirouetted at 52 deg/s
#       median for the whole 44 s Offboard hold (bag 20260914_2020_stageB). One file, independent
#       of 0001-0005; applies to pristine upstream too. Gate: scripts/yaw_hold_check.py.
#   patches/0007-rog-map-unhit-decay.patch                 visible-but-unhit decay (2026-09-15):
#       ROG-Map casts rays only to returns, so a cell whose rays all go into the sky is never
#       cleared -- a walker seen from the GROUNDED aircraft stayed in the map for 7 minutes on the
#       2026-09-14 bags. rog_map/unhit_decay: an occupied cell inside the sensor's field of view,
#       within range_max, with no OCCUPIED cell on its line of sight, that gets no hit for window_s
#       (and long enough for min_expected_rays to have crossed it) is decayed as if each expected
#       ray had missed. OFF in code, ON in follow_test_ros2.yaml. rog_map only (config.hpp,
#       prob_map.h/.cpp, rog_map_ros2.hpp) plus the follow config; its rog_map context is
#       0002's + 0003's, so it applies on top of them. Gate: scripts/unhit_decay_check.py.
#   patches/0008-super-visibility-field.patch              visibility field (2026-09-20): for
#       follow_planner's goal_mode nmpc, Stage 2. The loss oracle's viewpoint sweep, run for EVERY
#       locked frame (seen or coasting) and published whole as dtp_msgs/VisibilityField on
#       /planning/visibility. loss_oracle/visibility_enable: off in code, on in
#       follow_test_ros2.yaml. Does not touch evaluate(): the LossEvent stream is unchanged. Its
#       context is 0004's + 0005's, so it applies on top of them, and it needs dtp_msgs REBUILT
#       FIRST (the new message). Gate: scripts/visibility_field_check.py.
#   patches/0009-super-park-on-takeover.patch              park input (2026-09-30): the FSM has no
#       "stop planning"; a trajectory that ends more than 0.1 m from the goal makes it call
#       PlanFromRest on every 10 ms tick until the aircraft gets there. After a pilot takeover it
#       never does: on 2026-09-30 fsm_node re-planned to a stale goal for 45 minutes (76,229
#       failed plans, 144 % of a core) and the GCS stopped answering. A std_msgs/Empty on
#       fsm/park_topic (/planning/park) drops the goal and idles the FSM in WAIT_GOAL, pos_cmd
#       silent, until the next goal. command_mux sends it when its engage gate shuts. super_planner
#       only (fsm.h, fsm.cpp, fsm/config.hpp, fsm_ros2.hpp, the follow config); its context is
#       0001's, and it applies with or without 0008. Gate: scripts/park_check.py.
#
# It edits super_planner's C++ / headers / config and the ros/ros2.* BUILD TEMPLATES, then this
# script separately content-anchors dtp_msgs into the ACTIVE CMakeLists.txt / package.xml.
#
# HISTORY (2026-09-13): this script used to reference two split patches --
# 0001-yaw-to-target.patch and 0002-follow-goal-status.patch -- with a --yaw-only mode for
# external users who wanted the yaw fix without adopting dtp_msgs. Those two files never existed
# in the repo (only the combined patch did), so the script could not run at all. G0.4 regenerated
# the combined patch from the live tree (it had also gone stale -- missing requested_p) and this
# script now uses it. The follower needs BOTH features (full mode), so --yaw-only was dropped; if
# an external yaw-only consumer ever needs it again, re-split the combined patch by hunk.
#
# WHY THIS SCRIPT EXISTS RATHER THAN JUST `git apply`:
# SUPER keeps ROS1/ROS2 build files as TEMPLATES in `super_planner/ros/` which are COPIED into the
# active `CMakeLists.txt` / `package.xml` (see scripts/select_ros_version.sh). Our .patch edits the
# templates, so on any checkout where the templates have already been materialised -- i.e. every
# working build -- the CMake half of the patch is a SILENT NO-OP: the C++ lands, then the build
# fails with `fatal error: dtp_msgs/msg/goal_status.hpp: No such file or directory`.
# MEASURED 2026-09-07 on the drone NX: exactly that, twice, before the cause was found.
#
# And the active files CANNOT be patched by diff: downstream checkouts modify them heavily
# (the drone NX's super_planner/CMakeLists.txt differs from the template by ~163 lines), so a
# line-based hunk would never apply. This script therefore does CONTENT-ANCHORED insertion, which
# works regardless of how the active file has been edited.
#
# BASE THE PATCH APPLIES TO: upstream SUPER 2ad3419 PLUS the std::isnan ARM64 fix (setup.sh step
# 4b). Context lines carry std::isnan, so `git apply --check` FAILS on pristine upstream -- that is
# expected, not a fault. setup.sh applies the isnan fix before calling this, so it lands cleanly.
#
# !! DO NOT "fix" a template/active mismatch by re-running select_ros_version.sh. That copies
# !! templates OVER the active files and would silently destroy downstream packaging work -- and
# !! the build would still succeed, so the damage would not look like damage.
#
# Idempotent: safe to re-run (skips if the marker is already present). Backs up every active file
# it edits as <file>.pre-dtp.
#
#   bash scripts/apply_super_patch.sh [/path/to/SUPER]     (default: ~/super_ws/src/SUPER)

set -euo pipefail

SUPER="${1:-$HOME/super_ws/src/SUPER}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH="$REPO/patches/0001-super-yaw-to-target-and-follow-goal.patch"
PATCH2="$REPO/patches/0002-rog-map-target-exclusion.patch"
PATCH3="$REPO/patches/0003-rog-map-virtual-plane-ray-clip.patch"
PATCH4="$REPO/patches/0004-super-loss-oracle.patch"
PATCH5="$REPO/patches/0005-super-belief-around-obstacles.patch"
PATCH6="$REPO/patches/0006-super-yaw-hold-on-short-trajectory.patch"
PATCH7="$REPO/patches/0007-rog-map-unhit-decay.patch"
PATCH8="$REPO/patches/0008-super-visibility-field.patch"
PATCH9="$REPO/patches/0009-super-park-on-takeover.patch"
SP="$SUPER/super_planner"

say() { echo; echo "== $*"; }

say "Preflight"
[ -d "$SP" ] || { echo "ERROR: no super_planner at $SP"; exit 1; }
[ -f "$PATCH" ] || { echo "ERROR: patch not found at $PATCH"; exit 1; }
[ -f "$PATCH2" ] || { echo "ERROR: patch not found at $PATCH2"; exit 1; }
[ -f "$PATCH3" ] || { echo "ERROR: patch not found at $PATCH3"; exit 1; }
[ -f "$PATCH4" ] || { echo "ERROR: patch not found at $PATCH4"; exit 1; }
[ -f "$PATCH5" ] || { echo "ERROR: patch not found at $PATCH5"; exit 1; }
[ -f "$PATCH6" ] || { echo "ERROR: patch not found at $PATCH6"; exit 1; }
[ -f "$PATCH7" ] || { echo "ERROR: patch not found at $PATCH7"; exit 1; }
[ -f "$PATCH8" ] || { echo "ERROR: patch not found at $PATCH8"; exit 1; }
[ -f "$PATCH9" ] || { echo "ERROR: patch not found at $PATCH9"; exit 1; }
echo "  SUPER checkout : $SUPER"
echo "  patch          : $(basename "$PATCH")  (full: YAW_TO_TARGET + follow-goal/GoalStatus)"
echo "  patch          : $(basename "$PATCH2")  (M5.3a target-point filter, default off)"
echo "  patch          : $(basename "$PATCH3")  (ROG-Map virtual ceil/ground ray-clip bug fix)"
echo "  patch          : $(basename "$PATCH4")  (M6.1/M6.4 loss oracle, on in follow_test_ros2.yaml)"
echo "  patch          : $(basename "$PATCH5")  (M6.4b belief around obstacles + look-behind, on in follow_test_ros2.yaml)"
echo "  patch          : $(basename "$PATCH6")  (yaw hold on a short trajectory)"
echo "  patch          : $(basename "$PATCH7")  (ROG-Map visible-but-unhit decay, on in follow_test_ros2.yaml)"
echo "  patch          : $(basename "$PATCH8")  (visibility field for goal_mode nmpc, on in follow_test_ros2.yaml)"
echo "  patch          : $(basename "$PATCH9")  (park input: a takeover stops the planner)"

# ---------------------------------------------------------------- 1. the code
say "1/3  C++ / config / template changes"
cd "$SUPER"

# marker: setFollowGoal is only introduced by this patch, so its presence means "already applied".
if grep -rq "setFollowGoal" super_planner/include super_planner/src 2>/dev/null; then
    echo "  [have] $(basename "$PATCH") already applied, skipping"
elif git apply --check "$PATCH" 2>/dev/null; then
    git apply "$PATCH"
    echo "  applied $(basename "$PATCH")"
else
    echo "  ERROR: $(basename "$PATCH") does not apply to this checkout."
    echo "  Base is upstream 2ad3419 PLUS the std::isnan ARM64 fix (setup.sh step 4b) -- context"
    echo "  lines contain std::isnan, so it will NOT apply to pristine upstream. Run setup.sh's"
    echo "  isnan fix first, or start from a checkout that has it."
    exit 1
fi

# marker: target_exclusion_en is only introduced by 0002.
if grep -q "target_exclusion_en" rog_map/include/rog_map/rog_map_core/config.hpp 2>/dev/null; then
    echo "  [have] $(basename "$PATCH2") already applied, skipping"
elif git apply --check "$PATCH2" 2>/dev/null; then
    git apply "$PATCH2"
    echo "  applied $(basename "$PATCH2")"
else
    echo "  ERROR: $(basename "$PATCH2") does not apply to this checkout. It needs 0001 applied first"
    echo "  (its fsm_ros2.hpp context is 0001's); on a tree that has 0001 in a DIFFERENT version,"
    echo "  regenerate 0002 against it rather than forcing the hunks."
    exit 1
fi

# marker: the fixed clip keeps the ray's own length; upstream's .normalized() form is the bug.
if grep -qF "p = cur_odom + (p - cur_odom) * (pc / dz);" rog_map/src/rog_map/prob_map.cpp 2>/dev/null; then
    echo "  [have] $(basename "$PATCH3") already applied, skipping"
elif git apply --check "$PATCH3" 2>/dev/null; then
    git apply "$PATCH3"
    echo "  applied $(basename "$PATCH3")"
else
    echo "  ERROR: $(basename "$PATCH3") does not apply to this checkout. Its context is upstream's"
    echo "  raycastProcess(); look for 'p = cur_odom + (p - cur_odom).normalized() * pc / dz;' (twice)"
    echo "  and replace each with 'p = cur_odom + (p - cur_odom) * (pc / dz);'."
    exit 1
fi

# marker: loss_oracle_en is only introduced by 0004.
if grep -q "loss_oracle_en" super_planner/include/fsm/config.hpp 2>/dev/null; then
    echo "  [have] $(basename "$PATCH4") already applied, skipping"
elif git apply --check "$PATCH4" 2>/dev/null; then
    git apply "$PATCH4"
    echo "  applied $(basename "$PATCH4")"
else
    echo "  ERROR: $(basename "$PATCH4") does not apply to this checkout. It needs 0001 and 0002 applied"
    echo "  first (its fsm_ros2.hpp context carries the target-exclusion subscription); on a tree"
    echo "  that has them in a DIFFERENT version, regenerate 0004 against it rather than forcing."
    exit 1
fi

# marker: loss_oracle_rollout_collision is only introduced by 0005.
if grep -q "loss_oracle_rollout_collision" super_planner/include/fsm/config.hpp 2>/dev/null; then
    echo "  [have] $(basename "$PATCH5") already applied, skipping"
elif git apply --check "$PATCH5" 2>/dev/null; then
    git apply "$PATCH5"
    echo "  applied $(basename "$PATCH5")"
else
    echo "  ERROR: $(basename "$PATCH5") does not apply to this checkout. It needs 0004 applied first"
    echo "  (every hunk is inside 0004's loss oracle: loss_oracle.hpp, fsm/config.hpp, fsm_ros2.hpp and"
    echo "  the loss_oracle: block of follow_test_ros2.yaml); on a tree that has 0004 in a DIFFERENT"
    echo "  version, regenerate 0005 against it rather than forcing."
    exit 1
fi

# marker: kMinAlignDispM is only introduced by 0006.
if grep -q "kMinAlignDispM" super_planner/src/traj_opt/yaw_traj_opt.cpp 2>/dev/null; then
    echo "  [have] $(basename "$PATCH6") already applied, skipping"
elif git apply --check "$PATCH6" 2>/dev/null; then
    git apply "$PATCH6"
    echo "  applied $(basename "$PATCH6")"
else
    echo "  ERROR: $(basename "$PATCH6") does not apply to this checkout (one hunk pair in"
    echo "  super_planner/src/traj_opt/yaw_traj_opt.cpp, upstream 2ad3419). Regenerate it against"
    echo "  this tree rather than forcing."
    exit 1
fi

# marker: unhit_decay_en is only introduced by 0007.
if grep -q "unhit_decay_en" rog_map/include/rog_map/rog_map_core/config.hpp 2>/dev/null; then
    echo "  [have] $(basename "$PATCH7") already applied, skipping"
elif git apply --check "$PATCH7" 2>/dev/null; then
    git apply "$PATCH7"
    echo "  applied $(basename "$PATCH7")"
else
    echo "  ERROR: $(basename "$PATCH7") does not apply to this checkout. It needs 0002 and 0003 applied"
    echo "  first (its prob_map / config.hpp / rog_map_ros2.hpp context carries the target exclusion"
    echo "  and the fixed ray clip); on a tree that has them in a DIFFERENT version, regenerate 0007"
    echo "  against it rather than forcing."
    exit 1
fi

# 0008: the visibility field for follow_planner's goal_mode nmpc. Own marker:
# loss_oracle_visibility_en is only introduced by 0008.
if grep -q "loss_oracle_visibility_en" super_planner/include/fsm/config.hpp 2>/dev/null; then
    echo "  [have] $(basename "$PATCH8") already applied, skipping"
elif git apply --check "$PATCH8" 2>/dev/null; then
    git apply "$PATCH8"
    echo "  applied $(basename "$PATCH8")"
    echo "  NOTE: 0008 needs dtp_msgs rebuilt FIRST (msg/VisibilityField.msg), or fsm_ros2.hpp will"
    echo "        not find dtp_msgs/msg/visibility_field.hpp."
else
    echo "  ERROR: $(basename "$PATCH8") does not apply to this checkout. It needs 0004 and 0005 applied"
    echo "  first (its loss_oracle.hpp / fsm_ros2.hpp / config.hpp context is theirs); on a tree that"
    echo "  has them in a DIFFERENT version, regenerate 0008 against it rather than forcing."
    exit 1
fi

# 0009: the park input. Own marker: parkFollow is only introduced by 0009.
if grep -q "parkFollow" super_planner/include/fsm/fsm.h 2>/dev/null; then
    echo "  [have] $(basename "$PATCH9") already applied, skipping"
elif git apply --check "$PATCH9" 2>/dev/null; then
    git apply "$PATCH9"
    echo "  applied $(basename "$PATCH9")"
else
    echo "  ERROR: $(basename "$PATCH9") does not apply to this checkout. It needs 0001 applied first"
    echo "  (its fsm.h / fsm.cpp / fsm_ros2.hpp / config.hpp context carries setFollowGoal and the"
    echo "  follow-goal subscription); on a tree that has 0001 in a DIFFERENT version, regenerate"
    echo "  0009 against it rather than forcing."
    exit 1
fi

# ------------------------------------------------- 2. wire dtp_msgs into ACTIVE build files
say "2/3  wiring dtp_msgs into the ACTIVE build files (the part a diff cannot do)"
python3 - "$SP" <<'PY'
import os, shutil, sys
SP = sys.argv[1]

def safe_write(path, text):
    data = text.encode("utf-8")
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    assert os.path.getsize(tmp) == len(data)
    os.replace(tmp, path)

def backup(p):
    if not os.path.exists(p + ".pre-dtp"):
        shutil.copy(p, p + ".pre-dtp")

# --- CMakeLists.txt: find_package + add to the ros_libs list ---
p = os.path.join(SP, "CMakeLists.txt")
s = open(p, encoding="utf-8").read()
if "dtp_msgs" in s:
    print("  [have] CMakeLists.txt already wired")
else:
    backup(p)
    # anchor on the LAST find_package(...) so we land after the existing block
    import re
    fps = list(re.finditer(r"^find_package\([^\)]*\)\s*$", s, re.M))
    if not fps:
        print("  ERROR: no find_package() found in CMakeLists.txt"); sys.exit(1)
    end = fps[-1].end()
    s = (s[:end]
         + "\n# M0.7 Step 0.7.4 (DTP) -- GoalStatus for the moving-goal ingestion path.\n"
           "# ROS2-ONLY: dtp_msgs has no ROS1 build. Gated behind fsm/follow_goal_en (default false).\n"
           "find_package(dtp_msgs REQUIRED)"
         + s[end:])
    # add to the ros_libs list consumed by ament_target_dependencies
    m = re.search(r"set\(ros_libs\b(.*?)^\)", s, re.S | re.M)
    if not m:
        print("  ERROR: could not find set(ros_libs ...) block"); sys.exit(1)
    block = m.group(0)
    newblock = block.replace("\n)", "\n        dtp_msgs\n)", 1)
    s = s.replace(block, newblock, 1)
    safe_write(p, s)
    print("  CMakeLists.txt: find_package + ros_libs entry added")

# --- package.xml: <depend> ---
p2 = os.path.join(SP, "package.xml")
s2 = open(p2, encoding="utf-8").read()
if "dtp_msgs" in s2:
    print("  [have] package.xml already wired")
else:
    backup(p2)
    anchor = "</package>"
    if anchor not in s2:
        print("  ERROR: malformed package.xml"); sys.exit(1)
    s2 = s2.replace(anchor,
        "    <!-- M0.7 Step 0.7.4 (DTP): GoalStatus for the moving-goal ingestion path.\n"
        "         ROS2-ONLY on purpose - dtp_msgs has no ROS1 build. -->\n"
        "    <depend>dtp_msgs</depend>\n" + anchor, 1)
    safe_write(p2, s2)
    print("  package.xml: <depend>dtp_msgs</depend> added")

print("  verify: CMakeLists has %d dtp_msgs refs, package.xml has %d"
      % (open(os.path.join(SP,"CMakeLists.txt"),encoding="utf-8").read().count("dtp_msgs"),
         open(os.path.join(SP,"package.xml"),encoding="utf-8").read().count("dtp_msgs")))
PY

# ---------------------------------------------------------------- 3. next steps
say "3/3  Done -- now rebuild"
cat <<MSG
  dtp_msgs MUST be built first (it is an unconditional find_package dependency):

    colcon build --packages-select dtp_msgs
    colcon build --packages-select rog_map    # 0002/0003 change ROG-Map; fsm_node links it statically
    rm -rf build/super_planner        # force a fresh cmake configure -- see note below
    colcon build --packages-select super_planner

  NOTE: removing build/super_planner is not optional if super_planner was ever configured
  without dtp_msgs. colcon reuses the CMake cache and the new find_package will not run.

  VERIFY IN THE BINARY, not from the exit code:
    strings install/super_planner/lib/super_planner/fsm_node | grep -c GoalStatus         # expect >0
    ldd     install/super_planner/lib/super_planner/fsm_node | grep -c dtp_msgs           # expect 1
    strings install/super_planner/lib/super_planner/fsm_node | grep -c "TARGET EXCLUSION" # expect >0
    strings install/super_planner/lib/super_planner/fsm_node | grep -c "LOSS ORACLE"      # expect >0
    strings install/super_planner/lib/super_planner/fsm_node | grep -c "look-behind"      # expect >0 (0005)
    strings install/super_planner/lib/super_planner/fsm_node | grep -c "UNHIT DECAY"      # expect >0 (0007)
    strings install/super_planner/lib/super_planner/fsm_node | grep -c "PARKED"           # expect >0 (0009)
  and run the target filter's and the loss oracle's own gates (real fsm_node, synthetic scene,
  ~5 min each, no hardware):
    python3 ~/super_ws/src/drone-dtp/scripts/target_filter_check.py
    python3 ~/super_ws/src/drone-dtp/scripts/loss_oracle_check.py
    python3 ~/super_ws/src/drone-dtp/scripts/yaw_hold_check.py          # 0006, ~2 min
    python3 ~/super_ws/src/drone-dtp/scripts/unhit_decay_check.py       # 0007, ~4 min
    python3 ~/super_ws/src/drone-dtp/scripts/park_check.py              # 0009, ~2 min
    python3 ~/super_ws/src/drone-dtp/scripts/visibility_field_check.py  # 0008, ~6 min

  All features are OFF by default in the CODE (yaw_mode: 1 = YAW_TO_VEL, follow_goal_en: false,
  rog_map/target_exclusion/enable: false, loss_oracle/enable: false, loss_oracle/rollout_collision:
  false, loss_oracle/hidden_look: false, rog_map/unhit_decay/enable: false), so an existing gated
  stack behaves exactly as before until they are explicitly enabled. follow_test_ros2.yaml enables
  the follow path, YAW_TO_TARGET, the unhit decay, the loss oracle and its B/B2 belief switches (the oracle only publishes; nothing acts on it unless
  target_manager / follow_planner run with loss_conditioning:=true).

  To reverse: git apply -R patches/0008-super-visibility-field.patch, then
  git apply -R patches/0007-rog-map-unhit-decay.patch, then
  git apply -R patches/0006-super-yaw-hold-on-short-trajectory.patch, then
  git apply -R patches/0005-super-belief-around-obstacles.patch, then
  git apply -R patches/0004-super-loss-oracle.patch, then
  git apply -R patches/0003-rog-map-virtual-plane-ray-clip.patch, then
  git apply -R patches/0002-rog-map-target-exclusion.patch, then
  git apply -R patches/0001-super-yaw-to-target-and-follow-goal.patch, and restore the
  .pre-dtp backups of CMakeLists.txt / package.xml.
MSG
