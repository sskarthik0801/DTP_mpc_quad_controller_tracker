# drone-dtp — the human-following stack (M4), and how it runs from this repo

This folder is the camera-based human-following stack: Gemini 336L colour + depth → YOLO/BoT-SORT →
one locked person in `lidar_init` → a standoff goal and a yaw target into patched SUPER, with a
safety mux in between. **What each node does, and why, is in
[`../docs/dtp-follower-stack.md`](../docs/dtp-follower-stack.md) — read that before changing
behaviour.** This file covers the other half: where the code lives, how the drone ends up running
it, how to edit, build, run and test it, and the traps. Everything below was set up and verified on
the Jetson (`gypsy`, Orin NX) on **2026-09-12**.

**In one sentence:** you edit and run this folder directly. `~/super_ws/src/drone-dtp` is only a
symlink to it, and the ROS build installs links back into it, so there is **one copy of the code,
and it is the one in git**.

---

## Where it lives

| | |
|---|---|
| The code | `~/SUPER-Human-DTP/drone-dtp/` — **this folder** |
| Git branch | **`dtp-follower-stack`** — the folder exists **only on this branch** |
| First commit | `28d366b` — 105 files, byte-identical to the tree it came from minus `__pycache__`/`.pytest_cache` |
| Branched from | `gcs-and-p5-followup` at `c3915b9`, **not `main`** (see below) |
| Pushed? | **Yes** — `origin/dtp-follower-stack`, at `f4df2aa` on 2026-09-13. Commits made after that, and uncommitted work, exist only on this Jetson until pushed |
| What the drone builds | `~/super_ws/src/drone-dtp` → **symlink to this folder** |
| Where it came from | `~/super_ws/src/drone-dtp`, a plain unversioned directory, until 2026-09-12 |
| Backup of that tree | `~/drone-dtp.pre-symlink-2026-09-12` (predates any edit made here) |

**Why the branch is based on `gcs-and-p5-followup` and not `main`:** when it was created that branch
was 13 commits ahead of `main` and the working tree held ~40 uncommitted GCS/doc changes built on
those commits, so a `main`-based branch could not be checked out without disturbing them. The DTP
commit only adds a new directory, so it moves cleanly whenever you want it on `main`:

```bash
git rebase --onto main gcs-and-p5-followup dtp-follower-stack
```

## What is in here

```
drone-dtp/
├── ros2_ws/
│   ├── src/
│   │   ├── dtp_msgs/          ament_cmake   Target, TargetArray, GoalStatus  (SUPER compiles against GoalStatus)
│   │   ├── dtp_perception/    ament_python  perception_node, target_manager, static_calib_tf, sensor_watchdog,
│   │   │                                    fusion_node, dtp_viz, dtp_image_overlay, calibration tools;
│   │   │                                    calib/, rviz/
│   │   ├── dtp_bringup/       ament_python  dtp_system / sensors / super / follower / sensor_watchdog launch files,
│   │   │                                    config/dtp_params.yaml, preflight.py
│   │   ├── command_mux/       ament_python  command_mux_node, abort.py (land-on-blindness)
│   │   └── follow_planner/    ament_cmake   C++ standoff planner (header-only node)
│   ├── dtp_env_jetson.sh      env script for this Jetson (see "Running")
│   └── dtp_env.sh             env script for a workstation with a .venv
├── detection_pipeline/        NOT a ROS package — the frozen predictor, imported via PYTHONPATH
├── transport/                 NOT a ROS package — imported via PYTHONPATH
├── scripts/                   analysis, bench, Stage A, calibration and patch tools
└── patches/
    ├── 0001-super-yaw-to-target-and-follow-goal.patch      the SUPER patch (regenerated 2026-09-13)
    ├── 0002-rog-map-target-exclusion.patch                 M5.3a target-point filter (2026-09-13), on top of 0001
    ├── 0003-rog-map-virtual-plane-ray-clip.patch           ROG-Map ray-clip bug fix (2026-09-13), always on
    ├── 0004-super-loss-oracle.patch                        M6.1/M6.4 loss oracle in fsm_node (2026-09-14), on in follow_test_ros2.yaml
    ├── 0005-super-belief-around-obstacles.patch            M6.4b belief walks round trunks + looks behind them (2026-09-14), on in follow_test_ros2.yaml
    ├── 0006-super-yaw-hold-on-short-trajectory.patch       yaw hold below 0.3 m of trajectory displacement (2026-09-15), always on
    ├── 0007-rog-map-unhit-decay.patch                      ROG-Map visible-but-unhit decay (2026-09-15), on in follow_test_ros2.yaml
    └── 0008-super-visibility-field.patch                   the oracle's visibility field for goal_mode nmpc (2026-09-20) -- NOT applied to the live tree
```

---

## How the drone ends up running this folder

```
~/SUPER-Human-DTP/drone-dtp/                 ← you edit here; tracked by git
            ▲
            │  symlink
~/super_ws/src/drone-dtp ──── colcon build --symlink-install ────▶ ~/super_ws/build/<pkg>/, install/<pkg>/
                                                                  (links that resolve back into the repo)
```

colcon follows the symlink (`colcon_recursive_crawl` walks with `followlinks=True`) and records the
packages at their old path, `src/drone-dtp/ros2_ws/src/<pkg>`, so nothing about the workspace had to
change except the build mode. What each kind of file resolves to, **verified after the switch**:

| Kind | Installed as | Consequence |
|---|---|---|
| Python modules (`dtp_perception`, `dtp_bringup`, `command_mux`) | `install/<pkg>/lib/python3.10/site-packages/` holds only a `.egg-link`; `build/<pkg>/<pkg>` is a **directory symlink** to the package source | an edited **or new** module is live on the next launch |
| Data files (`launch/`, `config/`, `calib/`, `rviz/`) | a **per-file** symlink in `install/<pkg>/share/<pkg>/`, created at build time | edits are live on the next launch; a **new** file needs a build |
| Node executables (`install/<pkg>/lib/<pkg>/<node>`) | generated from `setup.py` entry points at build time | a **new** node needs a build |
| `follow_planner` | `install/…/follow_planner_node` → the compiled binary in `build/follow_planner/` | any C++ change needs a build |
| `dtp_msgs` | generated C/C++/Python code, compiled | any `.msg` change needs a build, and so do its dependents |
| `detection_pipeline/`, `transport/` | not installed at all — put on `PYTHONPATH` from `~/super_ws/src/drone-dtp` (resolves here) | edits are live on the next launch |

`detection_pipeline/` reaches `PYTHONPATH` two ways: `follower.launch.py` sets it from its
`dtp_repo_root` argument (default `~/super_ws/src/drone-dtp`), and `ros2_ws/dtp_env_jetson.sh` adds
the repo root when sourced.

### Why the build still goes through `~/super_ws`

1. **ROS cannot launch source files.** `ros2 launch dtp_bringup …` finds packages, node executables,
   generated message code and the compiled planner through a workspace's `install/`.
2. **SUPER lives in `~/super_ws` and depends on `dtp_msgs`.** The patched
   `super_planner/package.xml` has `<depend>dtp_msgs</depend>` and `fsm_node` links
   `libdtp_msgs__rosidl_typesupport_cpp.so` from `~/super_ws/install/dtp_msgs`, so the messages have
   to be in that workspace or beneath it.
3. **The DTP code was written to be nested inside an outside workspace.** `ros2_ws/dtp_env.sh` says
   so in its header (upstream, the whole repo is cloned as one entry in `super_ws/src/`), both env
   scripts compute the workspace as two directories above the repo root, and several scripts
   hardcode `~/super_ws/src/drone-dtp` (`follower.launch.py`, `stage_a_record.sh`,
   `isolate_dds_domain.sh`, `measure_load.py`, `measure_detect_live.py`, `deploy_calib.sh`).

**Considered and rejected (user's decision, 2026-09-12):** a separate workspace at
`drone-dtp/ros2_ws` layered on top of `~/super_ws`. It would build `dtp_msgs` twice (SUPER still
needs its own), need the env scripts and hardcoded paths changed, and run identically.

---

## Running it

```bash
source ~/super_ws/install/setup.bash          # from any directory
ros2 launch dtp_bringup dtp_system.launch.py                  # sensors + SUPER + follower + watchdog
ros2 launch dtp_bringup dtp_system.launch.py follower:=false  # NAV / click-goal only
ros2 launch dtp_bringup dtp_system.launch.py sensors:=false   # sensors already running
ros2 launch dtp_bringup follower.launch.py observe_only:=true # Stage A, observe-only
# on top of scripts/run.sh (LiDAR + FAST-LIO + fsm_node already up), observe-only:
ros2 launch dtp_bringup dtp_system.launch.py lidar:=false super:=false observe_only:=true
```

To watch it from a laptop — the camera picture with the person boxes, plus the 3-D markers — run
`JETSON_IP=gypsy.local bash scripts/laptop_dtp_viewer.sh` there (repo root; no build). On the
Jetson that needs `perception_node` and `dtp_image_overlay`, which `dtp_viz.launch.py` (and so
`dtp_system.launch.py`) starts; see `docs/dtp-follower-stack.md`, "dtp_viz and dtp_image_overlay".

If you use the DTP env script (it also adds CUDA to `PATH` and the `libcudss` loader path),
**source it through the symlink path**:

```bash
source ~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh     # correct
```

Sourced as `~/SUPER-Human-DTP/drone-dtp/ros2_ws/dtp_env_jetson.sh` instead, it looks for the
workspace two directories up — `/home/ideas-ad/install/setup.bash` — prints
`WARNING: no built workspace overlay found`, and `dtp_perception` does not import. Both were
tested in a clean shell on 2026-09-12.

Before believing any perception result, the checks in `CLAUDE.md` still apply:
`ip -brief link show eno1`, `ping 192.168.1.154`, one Livox driver, and
`ros2 topic hz /lidar_slam/odom`.

## After an edit — what to do

| You changed | To make it take effect |
|---|---|
| any `.py` in `dtp_perception/`, `command_mux/`, `dtp_bringup/` — including a new module | relaunch |
| an existing launch file, `config/dtp_params.yaml`, `calib/*.yaml`, `rviz/*.rviz` | relaunch |
| `detection_pipeline/` or `transport/` | relaunch |
| anything in `scripts/` | nothing — run it |
| a **new** launch/config/calib/rviz file, or `setup.py` (entry points, `data_files`) | build that package, then relaunch |
| `follow_planner` (C++) | build `follow_planner`, then relaunch |
| a `.msg` in `dtp_msgs` | build `dtp_msgs` + `follow_planner`, then `super_planner`, then relaunch everything including `fsm_node` |
| `patches/0001-…` … `patches/0007-…` | nothing — no build applies them; `~/super_ws/src/SUPER` is already patched. A change to the **live SUPER tree** needs the matching patch regenerated in the same commit (base: `2ad3419` + isnan + 0001 for 0002; 0003's context is upstream's; 0004's is 0002's `fsm_ros2.hpp`; 0005's is 0004's four files), then `colcon build --packages-select rog_map super_planner` (0002 touches both, 0003 only `rog_map`, 0004 and 0005 only `super_planner`; `fsm_node` links ROG-Map statically, so relink it either way). 0004 and 0005 were generated with `diff -u` against backups of the edited files — see `docs/m6-loss-oracle.md`. **Never regenerate 0004 to carry a later change**: the applier skips it once `loss_oracle_en` exists, and its diff base predates the `robot_r` 0.30 change. 0006 is one file in `super_planner` (`yaw_traj_opt.cpp`). 0007 is `rog_map` (`config.hpp`, `prob_map.h/.cpp`, `rog_map_ros2.hpp`) plus the follow config, generated with `diff -u` against `~/backups/backup-pre-0007-unhit-decay-*/`; its context is 0002's + 0003's — see `docs/fact-rogmap-unhit-decay.md`. **0008 is the exception to "already patched": it is NOT applied to the live tree** (2026-09-20, by decision — the baseline flight comes first). It is `super_planner` only (`loss_oracle.hpp`, `fsm_ros2.hpp`, `fsm/config.hpp`, the follow config), generated with `diff -u` against the live files from a scratch copy that was built and gated; its context is 0004's + 0005's, and it needs `dtp_msgs` rebuilt first (`msg/VisibilityField.msg`). Applied to a copy of the live files it reproduces the gated source byte for byte — see `docs/fact-nmpc-follow-goal.md` |

"Relaunch" means stop the running launch and start it again; a running Python process never
reloads a file on its own.

## Building

Use a shell that has **not** sourced `~/super_ws/install/setup.bash` if you can (see Traps 6), and
**one build at a time** (Traps 2):

```bash
cd ~/super_ws
source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash

# DTP packages — always --symlink-install
colcon build --symlink-install --packages-select dtp_perception          # one package
colcon build --symlink-install --packages-select dtp_msgs dtp_perception dtp_bringup command_mux follow_planner

# SUPER — WITHOUT --symlink-install: super_planner is a plain build (AMENT_CMAKE_SYMLINK_INSTALL=OFF)
colcon build --packages-select super_planner
```

Measured 2026-09-12: all five DTP packages from clean in **49.7 s**; `follow_planner` alone 34.2 s.
`follow_planner` prints Eigen `note: … eigen_packet_wrapper … declared here` on stderr — a compiler
note, and colcon reports "1 package had stderr output"; the build still succeeds.

**Mixing modes in one workspace is normal**, but a given package must be rebuilt in the mode it was
last built in, or cleaned first (Traps 3). As of 2026-09-12: the five DTP packages are
`--symlink-install`; `super_planner` is not.

## Testing

The unit tests need no hardware and run in under a second:

```bash
cd ~/SUPER-Human-DTP
source ~/super_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  drone-dtp/ros2_ws/src/command_mux/test/test_abort.py \
  drone-dtp/ros2_ws/src/command_mux/test/test_fence.py \
  drone-dtp/ros2_ws/src/dtp_bringup/test/test_preflight_speed.py \
  drone-dtp/ros2_ws/src/dtp_perception/test/test_static_calib_tf.py \
  drone-dtp/ros2_ws/src/dtp_perception/test/test_write_calib_identity.py \
  drone-dtp/ros2_ws/src/dtp_perception/test/test_cuda_cloud.py \
  drone-dtp/ros2_ws/src/dtp_perception/test/test_dtp_image_overlay.py
```

Result on 2026-09-13, with `test_dtp_image_overlay.py` added: **59 passed, 2 skipped**. Before it,
**49 passed, 2 skipped** with libcudss on `LD_LIBRARY_PATH` (source
`dtp_env_jetson.sh`, or prefix
`LD_LIBRARY_PATH="$(python3 -c 'import site; print(site.getusersitepackages())')/nvidia/cu12/lib:$LD_LIBRARY_PATH"`).
Without it, torch cannot load, so `test_cuda_cloud.py`'s 12 GPU tests **skip** — 37 passed, 14
skipped. That is a skip, not a pass: the GPU organized cloud (`cloud_backend: cuda`) is only
tested when the 12 run.
(2026-09-12, before `test_cuda_cloud.py`: 35 passed, 2 skipped.) The two long-standing skips are
`test_preflight_speed.py`'s `follow_test_ros2.yaml not present in this checkout`: that test looks
for `super_planner/config/` *inside* the DTP tree, which exists only in the upstream repo (it bundles
SUPER). The Jetson's copy never had it, so they skipped before the move too.
`PYTHONDONTWRITEBYTECODE=1` and `-p no:cacheprovider` keep `__pycache__`/`.pytest_cache` out of the
source tree. The `test_flake8.py` files are style checks and were not run.

### `follow_planner`: the one C++ package with tests (2026-09-20)

`test/test_nmpc_follow.cpp` tests the `goal_mode: nmpc` solver (`nmpc_follow.hpp`), which is
ROS-free. `ament_cmake_gtest` ships with the Humble install; nothing is added to the aircraft.

```bash
cd ~/super_ws && source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash
colcon build --symlink-install --packages-select dtp_msgs          # FIRST: follow_planner includes its
colcon build --symlink-install --packages-select follow_planner    # VisibilityField header (2026-09-20)
colcon test --packages-select follow_planner && colcon test-result --verbose
```

Result on 2026-09-20: **15 tests, 0 failures**. To test a change WITHOUT replacing the installed
node (that is how this was developed, with a flight pending), build into a scratch workspace and
point the gate at it:

```bash
S=/tmp/fp_scratch && source ~/super_ws/install/setup.bash
for pkg in dtp_msgs follow_planner; do      # one at a time; source the scratch overlay in between
  colcon build --base-paths ~/SUPER-Human-DTP/drone-dtp/ros2_ws/src/$pkg \
    --build-base $S/build --install-base $S/install --packages-select $pkg
  source $S/install/local_setup.bash
done
python3 drone-dtp/scripts/nmpc_check.py --node-exe $S/install/follow_planner/lib/follow_planner/follow_planner_node
```

`nmpc_check.py` (41 checks, ~9 min, ROS_DOMAIN_ID 90) is the gate for ANY `follow_planner`
change, in either `goal_mode`. See `docs/fact-nmpc-follow-goal.md`.

## Checking the wiring

Every path this prints must start with `/home/ideas-ad/SUPER-Human-DTP/drone-dtp`, and the `find`
must print nothing:

```bash
source ~/super_ws/install/setup.bash
readlink ~/super_ws/src/drone-dtp
python3 -c "import importlib.util as u, os; [print(os.path.realpath(u.find_spec(m).origin)) for m in ('dtp_perception.perception_node', 'command_mux.command_mux_node', 'dtp_bringup.preflight')]"
readlink -f ~/super_ws/install/dtp_bringup/share/dtp_bringup/config/dtp_params.yaml
find ~/super_ws/install/{dtp_perception,dtp_bringup,command_mux}/lib/python3.10/site-packages -name '*.py' ! -type l
```

Run it after any rebuild, and whenever a change "doesn't seem to do anything".

---

## Traps — every one of these happened on 2026-09-12

1. **Checking out another branch deletes this folder from disk.** `drone-dtp/` is tracked only on
   `dtp-follower-stack`, so on any other branch `~/super_ws/src/drone-dtp` is a dangling symlink and
   no DTP node can start. **Fly from `dtp-follower-stack`.** Switching back restores everything with
   no rebuild — the install links resolve again. Commit or stash edits here before switching.

2. **Never run two `colcon build`s at once in `~/super_ws`.** Two identical builds started one second
   apart (logs `build_2026-09-12_22-09-55` and `…22-09-56`) shared `build/dtp_msgs` and deleted each
   other's object files mid-compile:
   `/usr/bin/ld: cannot find CMakeFiles/dtp_msgs__rosidl_typesupport_introspection_c.dir/…/target__type_support.c.o: No such file or directory`.
   Fix: `rm -rf build/dtp_msgs install/dtp_msgs`, then one build.

3. **Changing a package between a plain and a `--symlink-install` build needs its `build/` AND
   `install/` removed first.** Without that:
   - `dtp_msgs` failed outright:
     `failed to create symbolic link '…/build/dtp_msgs/ament_cmake_python/dtp_msgs/dtp_msgs' because existing path cannot be removed: Is a directory`.
     `follow_planner` was cleaned before its switch, so it was never tried dirty; treat every
     `ament_cmake` package, `super_planner` included, the same way.
   - `ament_python` is worse: the old **copied** modules left in `site-packages` load in place of
     the edited files, with no error anywhere. The `find` in "Checking the wiring" catches it.

   The switch that was done:
   ```bash
   cd ~/super_ws
   rm -rf build/{dtp_msgs,dtp_perception,dtp_bringup,command_mux,follow_planner} \
          install/{dtp_msgs,dtp_perception,dtp_bringup,command_mux,follow_planner}
   colcon build --symlink-install --packages-select dtp_msgs dtp_perception dtp_bringup command_mux follow_planner
   ```
   While `install/dtp_msgs` is gone, **`fsm_node` cannot run either** — don't start the flight stack
   until the build has finished.

4. **Source `dtp_env_jetson.sh` through `~/super_ws/src/drone-dtp/…`**, never through this folder's
   own path (see Running).

5. **Keep backups and copies of this tree outside `~/super_ws`.** colcon crawls the whole workspace
   (`base_paths=['.']`), not just `src/`, so a second copy anywhere under it is a duplicate package.
   That is why the backup is in `~`.

6. **`AMENT_PREFIX_PATH … doesn't exist` warnings are harmless.** They appear when the shell sourced
   `~/super_ws/install/setup.bash` before those install folders were deleted; a new shell drops
   them. Sourcing the workspace before building also gives
   `'dtp_msgs' is in: /home/ideas-ad/super_ws/install/dtp_msgs … --allow-overriding` — also a warning.

7. **Never delete the link with a trailing slash.** `rm ~/super_ws/src/drone-dtp` removes the link;
   `rm -rf ~/super_ws/src/drone-dtp/` follows it and **deletes the contents of this repo folder**.

8. **Running nodes write `__pycache__/` into this folder.** The repo's `.gitignore` ignores them;
   they never need committing.

9. **Anything that writes into `~/super_ws/src/drone-dtp` now writes into git.** In particular
   `scripts/deploy_calib.sh`, run from another machine, `scp`s calibration files to that path; they
   will show up in `git status` here.

## Undoing it — back to a separate copy in `~/super_ws`

Stop every DTP node and `fsm_node` first.

```bash
rm ~/super_ws/src/drone-dtp                                   # the link only — NO trailing slash
cp -a ~/SUPER-Human-DTP/drone-dtp ~/super_ws/src/drone-dtp    # current code (or mv the 2026-09-12 backup back,
                                                              # which has none of the edits made since)
cd ~/super_ws
rm -rf build/{dtp_msgs,dtp_perception,dtp_bringup,command_mux,follow_planner} \
       install/{dtp_msgs,dtp_perception,dtp_bringup,command_mux,follow_planner}
source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash
colcon build --packages-select dtp_msgs dtp_perception dtp_bringup command_mux follow_planner
```

## Recreating this after a reflash or on another Jetson — NOT run end-to-end

**Since 2026-09-13 (`b78fbc3`) `scripts/setup.sh` does steps 3 and 4 itself** when this folder is
present — it symlinks it into `~/super_ws/src` and runs `apply_super_patch.sh` after the isnan
fix. Each piece was verified; a full clean build from a fresh clone has still not been run. The
pieces, in the order they depend on each other:

1. Clone `SUPER-Human-DTP` and check out `dtp-follower-stack` (pushed; anything not yet pushed is missing).
2. Run `scripts/setup.sh` as usual.
3. `ln -s ~/SUPER-Human-DTP/drone-dtp ~/super_ws/src/drone-dtp`
4. Patch SUPER: `bash drone-dtp/scripts/apply_super_patch.sh` (default target
   `~/super_ws/src/SUPER`) — `setup.sh` runs it. The patch was regenerated on 2026-09-13 and carries
   `requested_p`; the old one lacked it, and `follow_planner` then discarded every `GoalStatus`.
5. Build the five DTP packages `--symlink-install` (`dtp_msgs` first — colcon orders it), then rebuild
   `super_planner`, which now depends on `dtp_msgs`. Note `setup.sh` builds with
   `--symlink-install`, unlike the current `super_planner` on this box; keep whichever mode a package
   was first built in (Traps 3).
6. Restore what git does not hold:
   - **`~/models/yolo11s.engine`** — `perception_node`'s default model. A TensorRT engine is tied to
     the GPU, driver and TensorRT version, so it is exported per machine, not copied.
   - **torch / ultralytics** — per `dtp_env_jetson.sh`'s header, installed in system `python3`
     (`~/.local`), not a venv; torch ≥ 2.8 also needs `nvidia-cudss-cu12`.
   - **`OrbbecSDK_ROS2`** in `~/super_ws/src` — upstream at `8e7cad2`, no local changes.
   - **The calibration** (`dtp_perception/calib/drone_cam_livox.yaml`) is in git, but is valid only
     for camera serial `CPC64630007M` + LiDAR `192.168.1.154` in their current mounts;
     `calib_io` refuses any other pair.
   - EigenTrajectory config/checkpoint (the learned predictor; `eigen_enabled` defaults false).
7. Run "Checking the wiring" and the unit tests.

## Known issues, carried over unchanged

- ~~**`patches/0001-…` is stale against the live SUPER source**~~ — **fixed 2026-09-13**: regenerated
  with `requested_p` and applied by `setup.sh` (`docs/dtp-follower-stack.md`, Open items 1 and 5).
- **`ros2_ws/src/dtp_perception/sensor_watchdog.py`** (package root) is an older duplicate of
  `dtp_perception/sensor_watchdog.py`, missing the minimum-span rate fix and the
  `/dtp/perception_ok` fold-in. `setup.py` uses the inner one; the outer one is dead.
- **Hardcoded home paths**: `/home/ideas-ad/models/yolo11s.engine` (`jpeg_quality_sweep.py`,
  `measure_detect_live.py`, `compare_transport_detect.py`), `/home/ideas-ad/dtp_datasets`
  (`field.sh`), and `/home/krishna/calib_runs/…` in the calibration file's comments.
- **Comments and docstrings still say `camera_init`**; the parameters use `lidar_init`/`lidar_imu`.
  Translate when reading.
- **`auto_reacquire_after_lost`** is `false` in `dtp_params.yaml` since 2026-09-12 and must stay so
  for any flight with a person in the volume (`docs/dtp-follower-stack.md`, Open items 4).
- The design documents the code cites (`plan/dtp-imp/01…21`) and the tree's own `CLAUDE.md` are
  not on this machine.

## Upstream

This tree did not originate here. `scripts/deploy_calib.sh` describes the Jetson's copy as "a
**tar-extracted subset**" of a git repository and, when the target has a `.git`, runs
`git fetch origin dtp-imp`. `ros2_ws/dtp_env.sh` says that upstream repo is a SUPER fork
(`super_planner/`, `rog_map/`, `mission_planner/`, … plus `ros2_ws/`), cloned whole into a
workspace's `src/`. That repository is **not on this machine**. The SUPER patch is by KrishnaRajule
(2026-08-14) and the calibration was captured on another machine, so it most likely lives there.
**Edits made in this folder are not in that repo, and changes there are not in this one.**
