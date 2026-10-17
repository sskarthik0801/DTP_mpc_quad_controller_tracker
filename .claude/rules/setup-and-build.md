---
paths:
  - "scripts/setup.sh"
  - "patches/**"
  - "config/**"
  - "docs/build-fixes.md"
  - "docs/fact-fastlio-cpp17.md"
  - "drone-dtp/scripts/apply_super_patch.sh"
  - "drone-dtp/patches/**"
  - "**/CMakeLists.txt"
  - "**/package.xml"
---

# Install, build, and config drift

Scope: `scripts/setup.sh`, what a rebuild silently undoes, and how the SUPER patch series gets applied. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

- **`setup.sh` could not run on a clean machine until 2026-08-30.** Five separate breakages, all
  now fixed (see the `fix(setup)` commit): the Livox `build.sh` path, the two patches from
  `docs/build-fixes.md` that were documented but never scripted (`pcl_conversions`, bare `isnan`
  under C++17), a nonexistent `ros-humble-rosfmt` that aborted the whole apt transaction, and two
  undeclared deps (`ros-humble-mavros-msgs`, `libglfw3-dev`). The last two only ever resolved on
  the Nano because MAVROS was left over from the ArduCopter era — a good example of this repo's
  recurring failure mode: **the old box had state the script never created.**

- **Rebuilds silently revert the repo configs.** A `colcon build` of `fast_lio`, or a Livox driver
  rebuild, restores upstream defaults. Found reverted on 2026-07-29: `MID360_config.json` had stock
  IPs (`.5`/`.12`) so the driver couldn't connect, and `mid360.yaml` was in **ROS 1 format**, which
  made FAST-LIO **crash at startup** (`Cannot have a value before ros__parameters`) — `fsm_node`
  then just logged "No odom" forever, which looks like a sensor problem and isn't. Re-run
  `scripts/setup.sh`, and remember configs must land in **both** `src/` and `install/`.

- **✅ G0.4 CLOSED 2026-09-13 — `scripts/setup.sh` now applies the SUPER follow-goal patch.** Three
  things were broken and are fixed: (1) the patch `drone-dtp/patches/0001-super-yaw-to-target-and-follow-goal.patch`
  was **stale** (no `requested_p`) — **regenerated** from the live tree against the exact base
  (`2ad3419` + the std::isnan fix), so it now `git apply`s clean and carries `requested_p`,
  GoalStatus, setFollowGoal, YAW_TO_TARGET; (2) `apply_super_patch.sh` referenced **two split
  patch files that never existed** (`0001-yaw-to-target.patch`/`0002-follow-goal-status.patch`) so
  it could not run — **rewritten** to use the single combined patch (the `--yaw-only` split was
  dropped; our follower needs full mode); (3) `setup.sh` **now symlinks `drone-dtp` into
  `~/super_ws/src` and runs `apply_super_patch.sh`** after the isnan fix, guarded on drone-dtp being
  present. Verified end-to-end on a fresh base worktree (patch applies, dtp_msgs wired into active
  CMake/package.xml, idempotent). **NOT verified by a full clean colcon build** — that would disturb
  the live workspace; each piece was checked independently. The patch is **content-anchored for the
  CMake/package.xml half, `git apply` for the C++/config half**, and by design does **not** apply to
  pristine upstream (context carries std::isnan) — a failing `git apply --check` against pristine is
  expected, not a fault.

See also: `fast-lio-and-lidar.md` (the C++17 fix and the FAST-LIO patches), `dtp-workspace.md` (colcon and symlink traps for `drone-dtp/`), `px4-bridges-offboard.md` (`super_bringup/` is copied, not linked).
