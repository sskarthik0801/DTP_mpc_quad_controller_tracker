---
paths:
  - "drone-dtp/patches/**"
  - "drone-dtp/scripts/apply_super_patch.sh"
  - "drone-dtp/scripts/target_filter_*.py"
  - "drone-dtp/scripts/unhit_decay_check.py"
  - "drone-dtp/scripts/yaw_hold_check.py"
  - "scripts/setup.sh"
  - "super_bringup/launch/m1_super_lidar.launch.py"
  - "rviz/m1_viewer.rviz"
  - "docs/fact-rogmap-*.md"
  - "docs/fact-super-*.md"
---

# SUPER planner and ROG-Map

Scope: the SUPER planner (`fsm_node`), its configs, and ROG-Map's map bounds and clearing. **The SUPER source and its configs (`click_smooth_ros2.yaml`, `follow_test_ros2.yaml`) live in `~/super_ws/src/SUPER`, outside this repo**, so editing them does not load this file. Read it first. Our changes to SUPER are the patch series in `drone-dtp/patches/`. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

## Planner behaviour

- **SUPER FSM stays in `INIT`** ("Wait for goal") until the **first `/goal_pose`** — this is normal,
  not a fault. The map builds regardless. Click "2D Goal Pose" in RViz to make it plan.
- **`click_smooth_ros2.yaml` runs `yaw_mode: 3` (`YAW_TO_TARGET`) since 2026-09-12 — a decision,
  not an accident.** It is set in `src/`, `install/` and `scripts/setup.sh`. Don't re-derive:
  **upstream never reads `yaw_mode`, so 1 and 2 are identical**. Only the DTP patch reads it, and only
  tests `== 3` (`super_planner.cpp:795`). Under 3 the nose aims from the trajectory's end at
  `/planning/yaw_target` (fresher than 0.5 s). With no target — the normal M1 case, since only
  drone-dtp's `follow_planner` publishes it — the heading follows the velocity. **The cost: the goal
  yaw is never applied**, so an RViz 2D-Goal-Pose arrow's direction is ignored and the GCS **Rotate
  is refused**. To undo, set 1 in all three places and restart `fsm_node`. **Unmeasured**, bench or
  flight. Also: **`yaw_dot_max` is a time-allocation hint, not a clamp** — SUPER only warns above
  `yaw_dot_max + 2.0`. See `docs/fact-super-yaw-mode.md`.

- **✅ M5.2 (envelope) DONE 2026-09-13 — `follow_test_ros2.yaml` no longer ships `max_vel: 5.0`.**
  It now carries the **S1 first-flight envelope** (`max_vel 0.5`, `max_acc 0.5`, `max_omg 1.5`,
  `yaw_dot_max 1.0`, `p_max 0.95`) in `src/`, `install/`, **and the regenerated SUPER patch**, so a
  re-clone cannot restore 5.0. The S1→S2→S3 staging (0.5 → 1.0 → 1.5 m/s) is documented in the file;
  bump it there **and** in the patch between flights, never only in `install/`. **Still operator/
  pre-flight items, NOT done here:** ~~`robot_r`~~ (**set 2026-09-14: 0.30 = the user's measured
  0.25 m hub-to-prop-tip + 0.05 m**, in both flight configs, `src/` + `install/`, patch 0001 and
  `setup.sh`), ~~the 30 m × 4 m geofence~~ (built 2026-09-14 in `command_mux`, see `dtp-follow-behaviour.md`), and the target-z + 0.5 m altitude. Takes effect on the next `fsm_node` launch.

**Patch 0006 (yaw hold on a short trajectory):** why it exists and its gate are in the 2026-09-14 night post-mortem in `flight-log.md` and in `docs/fact-super-yaw-hold.md`. SUPER's two goal inputs (click vs `/planning/follow_goal`) are explained in `gcs-remote-control.md`.

## ROG-Map bounds and clearing

- **`virtual_ground_height`** must be `-1.0` (not the upstream `-0.1`) or ground-level/handheld
  operation is rejected as "Odom below virtual ground". **But `-1.0` is not the value that takes
  effect:** ROG-Map insets both bounds by `inflation_step` at init, so the *effective* band is
  **`[-0.5, 2.9]`**, not `[-1.0, 3.5]` — 0.45 m of headroom below the start point, not 1.0 m.
  (Upstream's `-0.1` computes to an effective floor of **+0.3 m**, which is exactly the reported
  symptom — so the rule is right, the number just isn't literal.) Also: `fsm_node`'s **2.03 GB**
  RSS is a bounded A* preallocation, not a leak, and the recurring "`-- [Fsm] No odom.`" log is
  cosmetic (hardcoded 0.1 s threshold vs a 10 Hz source; it does not gate planning).
  See `docs/fact-rogmap-bounds-and-memory.md`.

**ROG-Map voxel clearing — raycasting ON EVERYWHERE since 2026-09-12, at the user's
instruction; clearing still unconfirmed on hardware.** Raycasting is the only thing that frees a
cell; with it off the map only ever *adds* occupancy, so a followed person stamps a permanent wall
behind themselves. `raycasting.enable: true` is now set in **all five** SUPER configs, `src/` and
`install/` (incl. `follow_test_ros2.yaml`, the DTP default), in `drone-dtp/patches/0001-…`
line 201, and by `scripts/setup.sh` for `click_smooth_ros2.yaml`. **It is also SLOW by design —
"on" does not mean "clears promptly":** `p_hit 0.9` vs `p_miss 0.49` makes one hit worth ~55
misses, and a frame with any hit in a cell discards that frame's misses (`prob_map.cpp:564`). A
cell at `p_max` needs `(l_max − l_occ)/|l_miss|` rays through it with no hit to stop being
occupied: **~54 at 0.98, ~31 at 0.95**. `click_smooth_ros2.yaml` is `p_max: 0.95` (user,
2026-09-12, also in `setup.sh`); `follow_test_ros2.yaml` is 0.95 too since M5.2 (2026-09-13).
**Why remnants persist, and the ray-clip bug fixed by patch 0003: see "Found 2026-09-13 (third
session)" below.** The user reported
walking at the LiDAR and back without being cleared, most likely with the click config at 0.98
(the running stack loaded it, but the test time wasn't recorded). Not yet re-tested at 0.95. **CPU:** `fsm_node` 48.2% of a core over 20 s with it on (M1 stack, no goal); there is
no raycasting-off baseline, so the cost is still not isolated.

- **Raycasting is now ON in every config, including the follower's (user's instruction,
  2026-09-12)** — superseding the 2026-09-10 revert. Details, clearing math and the one CPU figure
  are in the raycasting note above. **The raycasting-off CPU baseline exists since 2026-09-13
  (sixth session): `fsm_node` 22.5 % on vs 15.2 % off, full graph, desk scene, no goal** — see
  "Added 2026-09-13 (sixth session)" in `fast-lio-and-lidar.md`. **Still to do:** the hardware walk-in/walk-out test that
  times how long the cells take to clear. Don't turn it back off without asking.

- **ROG-Map sums every miss ray through a cell per frame** (`probabilisticMapFromCache` →
  `missPointUpdate(…, operation_cnt − hit_cnt)`), so the 31/54 figures in the raycasting note above are **rays,
  not frames**. Clearing time depends on range and must be measured. A cell leaving OCCUPIED reads
  **UNKNOWN** until **74 rays (0.95) / 98 (0.98)** make it free. A walked-away subject's trail sits
  on the camera's line of sight — which is why the plan filters the locked target's points out of
  ROG-Map's input.

### Found 2026-09-13 (third session) — why a walked-away person's cells do not clear
The user saw ROG-Map remnants where a person had walked, with raycasting on. Four causes, all
measured; the first is now fixed. **Nothing here was tested with a real person on hardware.**
- **✅ FIXED — upstream ROG-Map bug, `drone-dtp/patches/0003-rog-map-virtual-plane-ray-clip.patch`.**
  - **The bug.** `ProbMap::raycastProcess` (`prob_map.cpp:777/784`) clipped a ray to a return
    above `virtual_ceil_height` or below `virtual_ground_height` with
    `(p - odom).normalized() * pc / dz`. That is a point **within 1 m of the sensor**, so the ray
    cleared nothing.
  - **When it bites.** Whenever the floor is below −1.0 in `lidar_init` (FAST-LIO started > 1 m
    above it: bench, table, terrain dropping away) or a return lies above 3.5 m. A person's lower
    body then **never** clears.
  - **Measured on the real `fsm_node`** (synthetic scene, MID360 ray directions from `run5`, floor
    −1.35, person at 4 m stands 10 s then leaves): 39 s later stock left **27** cells, all below −3°
    elevation and none ever cleared; fixed left **0**.
  - **Deployed.** Applied to the live tree and rebuilt 2026-09-13 16:09.
    - `target_filter_check.py` passed 17/17 on the scratch build and again on the live build.
    - `apply_super_patch.sh` applies it (own marker, idempotent), verified on a fresh
      `2ad3419`+isnan export: the resulting `prob_map.cpp` is byte-identical to the gated one.
    - Previous `fsm_node` and `prob_map.cpp` are at `~/backups/backup-pre-0003-20260913-1609/`.
- **Slow by design (unchanged).** 31 clean rays un-occupy a cell at `p_max 0.95`. The MID360 puts
  ~7 rays/s through a 0.1 m cell at 4 m and ~3 at 6 m, so a standing person clears in **~10 s at 4 m,
  ~20 s at 6 m** (synthetic, floor-level rig, real node's decay matched the offline copy).
- **No return behind ⇒ never clears (unchanged, not fixable by tuning).** ROG-Map casts rays only to
  returns. Sky, open space past ~40 m, glass or black surfaces behind where someone stood give no
  ray. Synthetic, open space behind: 16 of 182 cells never cleared. `target_exclusion` (0002) is the
  answer for the *followed* person; bystanders still leave these.
- **RViz magnifies it.** `rviz/m1_viewer.rviz` shows `/rog_map/inf_occ`, not `/rog_map/occ`, so ONE
  leftover 0.1 m cell draws as a ~1 m blob (33 × 0.2 m boxes). Turn on `rog_map_occ` to see the
  real cells.
- **Don't re-derive — tuning does not fix it.** `run5` replay: current settings average 289 leftover
  cells / 2.8 m³ inflated (peak 6 m³), 92 % below sensor height. `p_miss 0.45` halves the cells
  but cuts inflated volume only ~28 % and drops mapped static structure 99.8 % → 95.4 %; stronger
  settings erode more (OctoMap-like 0.7/0.4: −9.5 m³ of static structure).
- **Harness trap.** `target_filter_check.py` builds `config_name` as a path relative to the *live*
  config dir. A scratch-built `fsm_node` joins it onto its own `ROOT_DIR/config` and dies with
  `YAML::BadFile`. Override `tfc.SUPER_CFG_DIR` to the scratch copy's config dir.

### Added 2026-09-15 — ROG-Map visible-but-unhit decay (patch 0007): the parked-aircraft remnants
**The user's report ("I walked next to the drone and back and remnants of me never went away")
is diagnosed and fixed in code; bench-gated, bag-replayed, and **RAN LIVE on the 2026-09-15 flight**
(`UNHIT DECAY: 10 passes, 124 unobserved cells in clear view, 124 decayed, 15 left occupied; pass
3.9 ms` in the stack log, on the ground before takeoff). Its *effect* on remnants has still not been
scored against a walker.** Full record:
[`docs/fact-rogmap-unhit-decay.md`](docs/fact-rogmap-unhit-decay.md). Don't re-derive:
- **Cause: no return, no ray.** ROG-Map casts a ray only to a point that returned. With the
  aircraft on the ground the MID360 is 0.15 m up, every part of a person is above the horizon, and
  the beams through them go into the sky. GCS bag `bags/2026-09-14_195701` + the Stage A bag
  replayed through the real `fsm_node`: of 457 body cells, **113 were still occupied 7 min after
  the walker left**, median 12 rays through them (30 with none) vs 718 for the cells that cleared.
  Where a backdrop existed (a hedge at 10 m) the same walker's cells cleared in 1.5 s. Not tunable
  (`p_max`/`p_miss` need rays), not recoverable from the sensor (the MID360's no-return points keep
  no direction: neighbouring returns are 47° apart). RViz shows `inf_occ`, so one cell is a 1 m blob.
- **Fix: `rog_map/unhit_decay`** (`ProbMap::unhitDecayPass`, `drone-dtp/patches/0007-…`, in
  `apply_super_patch.sh`, live tree rebuilt, backup `~/backups/backup-pre-0007-unhit-decay-*/`).
  Every 0.5 s: an occupied cell within 8 m, inside the sensor's vertical FOV (−6/+50°, body frame),
  with no *observed* occupied cell on its line of sight, that **no ordinary ray has reached for 3 s,
  hit or miss** (and long enough for 20 expected rays), gets as many misses per pass as rays the
  sensor should have sent through it (10 000 rays/sr/s × res²/r²). OFF in code, **ON in
  `follow_test_ros2.yaml`** (src + install). Three rules were learned on the gate and matter:
  UNKNOWN does not block the view (else tangent cells stall); an unobserved occupied cell is
  transparent to the cells behind it (else 26 s instead of 7); **any** ray resets the clock, not
  only a hit (else far grazing ground is double-counted and eroded, and that was the whole cost);
  a ray the **target exclusion masks still counts as reaching the cell** (else a tube beside the
  followed person reads as sky — `target_filter_check` F2 caught it, 26 tube cells lost).
- **Gate: `python3 drone-dtp/scripts/unhit_decay_check.py` — 10/10** (real `fsm_node`, domain 89,
  synthetic scene, ~4 min): sky-backed walker's 112 cells gone 7 s after leaving; occluded,
  out-of-view (> 52°), beyond-range and real (wall/ground/tube) cells untouched; pass 3–4 ms, max
  9.9 ms. **Re-run it after any ROG-Map change**, with `target_filter_check.py` and
  `loss_oracle_check.py`. **Bag replay, decay on: 338 → 83 cells 2 s after the walk, 10 at 30 s,
  2–3 from 60 s on, 3 at 7 min (was 113).** The walker standing behind the aircraft at the end of
  the bag (411 cells, out of the camera's view) is kept: a real person is hit every scan.
- **Traps:** two replays / gates on one domain collide silently (87 = loss tools, 88 =
  target_filter_check, 89 = unhit_decay_check). `bags/2026-09-14_195701` is only 42 s; the
  7-minute view is the Stage A bag (`/cloud_registered_body` + odom, no `/rog_map/occ`, hence the
  replay). **A Python subscriber must never read a bag while `ros2 bag record` writes it** (the
  20:28 Stage B bag has no `metadata.yaml` — recorder died — `ros2 bag reindex` to recover).

## ROG-Map target exclusion (patch 0002)

- **✅ M5.3a target-point filter BUILT — `drone-dtp/patches/0002-rog-map-target-exclusion.patch`,
  default OFF.**
  - **What it does.** `rog_map/target_exclusion` in `follow_test_ros2.yaml`: ROG-Map drops the
    locked, seen person's returns. Inside a cylinder at `position + velocity × (t_scan −
    t_target)`, r 0.6, z [−1.0, +1.3], a scan inserts **no hits and no misses**. Coasted and held
    frames are never used, and an observation older than 0.5 s is dropped.
  - **Built and in the live tree.** Compiled into the live SUPER tree (`rog_map` + `super_planner`
    rebuilt). The previous `fsm_node` is at `~/backups/backup-pre-m53a-20260913-0327/`.
  - **Don't re-derive the band, radius or centre.**
    - The plan's `z+0.6` band was wrong: perception's target z is the **waist** (0.65 of the box
      from the TOP).
    - The radius, centre and band came from `drone-dtp/scripts/target_filter_study.py` on
      `run5`.
  - **Gates:**
    - `drone-dtp/scripts/target_filter_check.py` — real `fsm_node`, synthetic scene, 17/17, ~5 min;
      **re-run it after any SUPER/ROG-Map change**;
    - `target_filter_replay.py` on `run5`: LoS crossings 2.6 % → 0.7 %.
  - **The consequence.** With it ON the planner does not avoid the followed person.
    `stage_a_preflight.py` FAILs it in Stage A. Enabling it in the flight config waits for the
    hardware walk test.
  - **`apply_super_patch.sh` applies 0002 after 0001** (own marker, idempotent; verified on a fresh
    `2ad3419`+isnan checkout).

M6's loss oracle (patches 0004/0005) also lives in `fsm_node`: see `m6-loss-oracle.md`. So does
**patch 0008 (2026-09-20, the oracle's visibility field for `follow_planner`'s `goal_mode nmpc`) —
which, unlike 0001–0007, is NOT applied to the live SUPER tree**: `apply_super_patch.sh` will
apply it on its next run, and it needs `dtp_msgs` rebuilt first. Gate
`visibility_field_check.py`. → `m6-loss-oracle.md`, `docs/fact-nmpc-follow-goal.md`.
- **⚠️ Hit 2026-09-30: running `apply_super_patch.sh` on the live tree applied 0008.** It was
  reverted at once with `git apply -R` and the tree checked byte for byte; no build happened in
  between. **Do not run that script on the live tree unless 0008 is wanted.** To add one patch,
  `git apply` it alone.

**Patch 0009 (2026-09-30, the park input) IS applied to the live tree and built.** A
`std_msgs/Empty` on `/planning/park` drops SUPER's goal and idles its FSM, so a pilot takeover
no longer leaves it re-planning to a stale goal. It applies with or without 0008. Gate
`park_check.py` (domain 88). → `dtp-follow-behaviour.md`, "the park".
