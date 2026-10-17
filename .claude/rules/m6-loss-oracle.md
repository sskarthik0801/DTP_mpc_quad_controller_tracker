---
paths:
  - "drone-dtp/patches/0004-*"
  - "drone-dtp/patches/0005-*"
  - "drone-dtp/scripts/loss_*.py"
  - "drone-dtp/scripts/m66_*.py"
  - "drone-dtp/scripts/bench_bringup.sh"
  - "drone-dtp/scripts/bench_probe.py"
  - "drone-dtp/detection_pipeline/**"
  - "drone-dtp/weights/**"
  - "drone-dtp/ros2_ws/src/dtp_perception/dtp_perception/target_manager.py"
  - "drone-dtp/ros2_ws/src/dtp_perception/test/test_target_manager_loss.py"
  - "drone-dtp/ros2_ws/src/follow_planner/**"
  - "drone-dtp/ros2_ws/src/dtp_msgs/msg/LossEvent.msg"
  - "docs/m6*.md"
  - "docs/eva-tracker-faster-lio-assessment.md"
  - "docs/paper/**"
---

# M6: the loss oracle, reappearance belief, conditioning, predictor study

Scope: the paper's contribution as built: why the target was lost (patch 0004), the belief around obstacles (patch 0005), loss conditioning, the replay harness, and the M6.6 predictor study. Bench and replay only; never flown. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

## Status

- **⚠️ 2026-09-26: the carry-forward arm's coasting yaw changed (user's decision).** Since then
  `follow_planner`'s coast yaw hold replaces the perception-stamped yaw target from 0.4 s of
  stamp age with one held heading, stamped now. It is the same in both arms after a loss.
  - **The arm as documented below** (the yaw lapses at SUPER's 0.5 s gate) is now
    `coast_yaw_hold:=false` on `follow_planner`. Record which one a run used.
  - Under conditioning, a fresh verdict still aims the camera itself, and the hold after the loss
    keeps the oracle's last aim.
  - `loss_conditioning_check.py` checks both: default, and `--no-coast-yaw-hold`.
  - Details: `dtp-follow-behaviour.md`, "the yaw hold".

- **2026-09-23: M6.3 has a field runbook —
  [`docs/m6.3-handheld-runbook.md`](../../docs/m6.3-handheld-runbook.md). Written, not run.**
  The handheld occlusion dataset → D1, on foot, no flying. New in it, not in the plan:
  - **carry the rig at 1.2 m, not the plan's 1.5 m.** With the waist rows at 0.5 / 1.0 / 1.4 m
    and the view floor at `z_sensor − d·tan 7°`, a 1.5 m carry leaves **only the top row** at 4 m
    (`rays_cast` 5) — run5's failure mode in miniature. 1.2 m leaves two rows, and is also the
    flown follow height.
  - **`rays_cast` is the in-field check.** 15 / 10 / 5 = three / two / one height rows surviving
    the floors. Seeing 5 means lower the rig or lengthen the range *before* recording events.
  - the E1 events must walk **through** the shadow, not stop at its edge: the misses in both
    existing bags are coast points that had not entered the shadow yet, where LOST is correct.
  - record an explicit topic list, not the `stage_b` preset (depth is 6 MB/s and unused here).
  - Verified on this box the same day: the live tree has 0004 + 0005 (`loss_oracle.hpp`,
    `rollout_collision`), **not** 0008 (`visibility_enable` absent); `fsm_node` built 2026-09-15.
  - **Step 0 of it was run 2026-09-23: `loss_oracle_check.py` — 54/54 in 21 min on the flight
    binary.** O6 over 130 events: classify p50 29 / **p99 111 µs**, evaluation p50 297 /
    **p99 2474 µs** (2026-09-14: 31 / 108, 310 / ~2500). Scene outputs reproduce to the last
    decimal — O8 OFF dominant mean (4.01, −0.34) at 0.03 m from a mapped cell, O8 ON min 0.19,
    O9 ON look point (5.33, 0.01) and viewpoint (1.46, −1.03), all identical to the as-built
    record. **This gate is deterministic across binaries and boxes; treat a changed number as a
    real change.**
  - **The gate is 54/54, not 56/56.** The two extra checks in the 2026-09-14 record are the
    `--dump` / `--compare` pairing, which only runs when you pass `--compare` a reference dumped
    from the pre-0005 binary (`~/backups/backup-pre-m64b-20260914-1559/`, no saved dump on this
    box). That was 0005's one-time regression proof. A plain run has 54 checks. Anyone reading
    "56/56" and getting 54 has not lost two checks.
  - Related: the clocks check is `bash scripts/jetson_clocks_setup.sh show` (unit
    `super-dtp-jetson-clocks.service`, enabled + active, GPU 918/918). `systemctl is-enabled
    jetson-clocks` returns "No such file or directory", which looks like a failure and is not.
  - **`bench_bringup.sh` ran the same evening (`~/dtp_datasets/20260923_1656_bench_bringup/`),
    exit 0, 0 survivors, swap 0 MB — and it closes two open items:**
    - **The oracle's CPU on the full graph is measured and passes:** `fsm_node` **22.5 % of a
      core idle → 25.9 % answering** at 8 Hz, **RSS 2064 → 2065 MB** (budgets +5 % / +10 MB).
      464/480 coasts answered, eval p50 27 / p99 161 / max 421 µs, classify p99 59–92 µs. The
      "still unmeasured" line in `docs/m6-loss-oracle.md` is struck out.
    - **`stage_a_preflight` section 4b passes on hardware.** Its target-exclusion line was fixed
      on 2026-09-14 and had never been re-run on hardware since; it now reads
      `[ok] loss oracle /planning/loss_event published by fsm_node`. **estimator_preflight GO**
      (12/12), **stage_a_preflight GO with 4 warnings**, all expected: no `goal_mode` on
      `follow_planner_node` (confirms the nmpc build is NOT installed), `auto_reacquire_after_lost`
      true, `extrinsic_est_en` true, no recorder running yet.
    - **The view floor showed up live: `rays 10`, not 15,** with the rig sitting low on the
      bench — one height row dropped, exactly the §1 arithmetic in the M6.3 runbook, on real
      hardware. `rays_cast` is therefore a working field instrument, not a theory.
    - Health at the same time: odom 10.00 Hz, stamp lag p50 19 / max 29 ms, GPU cloud 12–14 ms
      `[cuda]`, targets 7.8 Hz, no foreign subscribers on `/cloud_registered` or `/livox/lidar`,
      odom bridge TRACKING reset_counter 0. Aircraft was on a pack at 21 % (the user's call).
  - **The M6.2 scoring chain was re-verified the same evening, end to end, on today's binary**
    (it had not run since 2026-09-14). 60 s slice of run5 → `loss_replay.py` **26 events in
    12 episodes** (classify p99 42 µs, eval p99 2393 µs) → `loss_label.py sheets` **12 sheets +
    `labels.csv`** headless → `loss_label.py score` on the 2026-09-14 labels **reproduced the
    documented run5 matrix exactly** (5/1/32, 2/2/4, E1 0.13, E2 0.25). The pipeline is sound;
    only the dataset is missing.
  - **`loss_label.py --gui` cannot run on the NX: `DISPLAY` is unset** over SSH (`cv2` and
    `numpy` are installed, so sheet generation is fine). Label with `ssh -X`, or copy the sheets
    to the laptop and fill `labels.csv` by hand — it ships one row per episode with
    `oracle_first`, `oracle_majority`, `recorded_outcome` and the sheet name already filled in.
  - **The whole applicable gate suite was re-run the same evening, sequentially, and is green:**
    `loss_oracle_check` 54/54, `target_filter_check` 17/17, `unhit_decay_check` PASS (decay cost
    max 3.88 ms), `yaw_hold_check` 7/7, `loss_conditioning_check` 16/16, `m3_bridge_check` 32/32,
    `descent_guard_check` 13/13, `fence_check` 13/13, `terrain_check` 10/10, `engage_gate_check`
    9/9 (see the flaky-teardown note in `dtp-follow-behaviour.md`), unit tests
    `command_mux` 85 / `dtp_perception` 57+12 skipped / `dtp_bringup` 11+2 skipped, and all seven
    GCS suites (`test_render.py` runs the two gjs front-end suites itself). 0 survivors, swap 0 MB.
    - **Two gates are NOT applicable to this binary and must not be read as failures:**
      `nmpc_check.py` (the installed `follow_planner_node` has no `goal_mode` — `stage_a_preflight`
      says so) and `visibility_field_check.py` (0008 unapplied, `visibility_enable` absent).
    - **Harness traps worth not repeating:** run the ament `test_flake8.py` **from inside the
      package** — `pytest` from the repo root lints `docs/evidence/` too and reports ~1526 bogus
      style errors. The GCS suites are **standalone scripts**, not pytest modules (`pytest
      scripts/gcs` dies on `SystemExit` at collection), and the `.js` ones are **gjs**, not node.
    - Pre-existing and untouched: `dtp_perception`'s flake8 is genuinely red, all `E501 line too
      long` (100–107 vs 99).

- **Patch 0008 (2026-09-20), NOT applied to the live tree:** the oracle also publishes
  `dtp_msgs/VisibilityField` on `/planning/visibility` for every locked frame (seen or coasting),
  behind `loss_oracle/visibility_enable`. It is the viewpoint sweep reported whole. **`evaluate()`
  is untouched: 30/30 `LossEvent`s identical, old binary vs patched.** Gate
  `visibility_field_check.py` 13/13 (domain 88 — never with `loss_oracle_check` or
  `target_filter_check`). Trap: `loss_oracle_check.py --compare` pairs only its M6.4b scenes, so
  `--only A --compare` reports "0 matched" and FAILS — dump both and call `pair_compare` yourself.
  Full record: `docs/fact-nmpc-follow-goal.md`, "Stage 2".

- **M6 (the paper's contribution): BUILT 2026-09-14, bench-gated, never flown.** Loss oracle in
  `fsm_node` (patch 0004, `dtp_msgs/LossEvent`), conditioning behind `loss_conditioning`, stock
  baseline, replay harness. See "Added 2026-09-14" below and `docs/m6-loss-oracle.md`.

- **⛔ The paper's contribution does not exist in code.** Nothing in `drone-dtp/` classifies a loss
  as occluded vs lost against the map, estimates reappearance, or conditions the planner.
  `target_manager`'s CTRV coast is exactly the paper's *carry-forward ablation arm*. The plan puts
  the classifier **inside `fsm_node`**, sharing ROG-Map's `map_ptr_` (`isLineFree`,
  `getGridType`); don't build a second map.

*(Superseded 2026-09-14: M6 is now built. See below.)*

### Added 2026-09-14 — M6 BUILT: the loss oracle, the reappearance belief, loss conditioning
**Bench and bag replay only. Nothing has flown; the oracle has never answered a live target.**
The paper's contribution now exists in code. Full as-built record, every parameter, every gate:
[`docs/m6-loss-oracle.md`](docs/m6-loss-oracle.md). Don't re-derive any of it.
- **The oracle lives inside `fsm_node`** (`super_core/loss_oracle.hpp`, SUPER patch
  **`drone-dtp/patches/0004-super-loss-oracle.patch`**, applied by `apply_super_patch.sh`; the live
  tree is patched and rebuilt, backup at `~/backups/backup-pre-m6-20260914-0044/`). It reads
  ROG-Map through the public query API only, answers every COASTING `/dtp/follow_target` with
  **`dtp_msgs/LossEvent` on `/planning/loss_event`**: OCCLUDED / UNMAPPED / LOST / OUT_OF_VIEW
  (Eq. 1 over 15 rays on the target's cylinder, ρ\* 0.5, rays stop 0.4 m short), the occluder's
  edges, a 32-rollout CTRV reappearance belief clustered by side, and a pre-positioning viewpoint
  on the [3, 4] m annulus. **On in `follow_test_ros2.yaml`** (`loss_oracle/enable`), so the next
  observe-only session records it for free. Off in code and in every other config.
- **Gate: `python3 drone-dtp/scripts/loss_oracle_check.py` — 28/28** on the real `fsm_node`,
  synthetic scene (tube → OCCLUDED with edges and the right-side belief; open → LOST; unscanned
  bearing → UNMAPPED; 60° → OUT_OF_VIEW). **Cost: classify p99 81 µs, full evaluation p99 406 µs**
  (budgets 2 / 10 ms). Re-run it after any SUPER/ROG-Map change, alongside `target_filter_check.py`.
- **Trap found and fixed on that gate: the target's own shadow reads UNKNOWN**, and "first
  non-free cell" then says UNMAPPED for a person who walked straight away (both filter arms did).
  `fsm_node` keeps the seen trail (6 s); an UNKNOWN cell within 0.6 m of it is free. OCCUPIED cells
  are never excused, so a walk-away trail with the filter OFF still reads OCCLUDED (the paper's
  documented false verdict) and LOST with it ON.
- **Conditioning (M6.5) is behind ONE switch, `loss_conditioning`, on `target_manager` AND
  `follow_planner`** — set both with `ros2 launch dtp_bringup follower.launch.py
  loss_conditioning:=true` (or via `dtp_system.launch.py`). `dtp_params.yaml` ships **false** =
  the carry-forward arm = today's behaviour. ON: OCCLUDED coasts to the belief's window (cap
  3.0 s, never shorter than the plain coast) with the re-emergence **region** as the
  re-acquisition gate inside the window; LOST/UNMAPPED end the coast **after the verdict has stood
  0.3 s** (a pre-registered deviation from the plan's "immediately": 38 of run5's 48 coasts are
  one-frame LOSTs that re-acquire in 0.1–0.3 s); the goal becomes the oracle's viewpoint and the
  yaw target the re-emergence point, **stamped now** (the one place the perception-stamp rule is
  bypassed, bounded by the coast cap). `stock_baseline` on both nodes = the M6.8 naive follower
  (no coast, no abstain, goal behind the present position; bench/replay only).
- **Gates:** `test_target_manager_loss.py` 27/27 (pure);
  `python3 drone-dtp/scripts/loss_conditioning_check.py` **16/16** on the real
  `target_manager` + `follow_planner`, both arms (the script plays the oracle). Both nodes were
  rebuilt (`follow_planner` C++, `dtp_msgs` for the new message).
- **M6.2 replay harness:** `loss_replay.py BAG --out DIR` (bag → real `fsm_node` → `events.jsonl`
  + `episodes.csv`; rebuilds the map from `/cloud_registered_body` + odom), `loss_label.py sheets`
  (contact sheets ±2 s per coast onset, one-key labelling with `--gui`, `score` = D1 confusion
  matrix), `loss_condition_replay.py BAG events.jsonl --arm off|on|stock` + `--compare`.
  **run5 replayed** (`~/dtp_datasets/20260909_closedloop_run5/loss_replay/`): 82 events in
  48 episodes — **7 OCCLUDED, 3 UNMAPPED, 38 LOST**; the recorded lock re-acquired 47 in
  0.09–0.38 s, and the one it lost is the oracle's clearest OCCLUDED (a partition). Two-arm
  replay through the real `target_manager`: OFF 65/66 re-acquired, ON 58/61 with **3 coasts
  ended by the oracle's LOST** (verdict stood ≥ 0.3 s) and 1 re-lock via the region — on a bag
  whose losses are detector glitches, the conditioned arm's cost is visible and its benefit
  cannot be (no real occlusions). **Unlabelled**; 48 sheets await one keypress each. The ON arm
  is a lower bound (recorded events answer recorded coasts; a closed-loop replay is not written).
- **Traps:** two `loss_condition_replay.py` runs on the same domain (87) collide silently (both
  publish `/dtp/targets`; the coast count went 61 → 554) — run arms sequentially, check
  `pgrep -af target_manager` first. `loss_replay.py` writes its config variant under the SUPER
  source `config/` dir because `fsm_node` resolves `config_name` against `ROOT_DIR`.
- **Second session, same day — closed loop, labels, and two findings that shape M6.3:**
  - **`loss_closed_loop_replay.py`** runs `fsm_node` + `target_manager` + `follow_planner` on a
    bag with the oracle answering live (no more lower bound). run5, both arms: OFF 65/66
    re-acquired, ON 61/64 (3 ended by the oracle, 2 re-locked via the region). `fsm_node` 33 % of
    a core for map + oracle, equal with and without the oracle answering.
  - **run5 labelled, provisionally, by Claude from the contact sheets** (46/48; 38 OCCLUDED
    behind a ~1.7 m cabinet 4 m away, 8 LOST). **D1 fails on it: E1 recall 0.11–0.13.** Not a
    tuning problem — two geometry facts, both now handled and documented:
    (1) the bench rig's `lidar_init` origin is ~1.6 m up, so the subject's waist is a metre
    below ROG-Map's virtual ground and every sample height was dropped; (2) with the ground
    lowered (`--virtual-ground -2.0` on the replay tools) 39/48 read UNMAPPED, because **the
    MID360 looks no lower than 7° under its horizon: at 4 m the map knows nothing below 0.5 m
    under the sensor, ever.** The oracle now drops heights below either floor
    (`loss_oracle/fov_floor_deg` 7.0) and warns loudly; the honest run5 verdict at the only
    knowable height is LOST — the cabinet is below what this sensor can map at 4 m.
    **Paper precondition: an occluder is classifiable only where it rises above the sensor's
    floor at its range.** Trunks and the 1.8 m tubes do.
  - **M6.3 protocol change (in the plan): start FAST-LIO with the rig on the ground, then lift.**
  - Traps: two replays on one domain collide silently (coasts 61 → 554); a chained script that
    waited on `pgrep -f "colcon build"` deadlocked on its own launching shell; a background job's
    outputs are not there until its notification says so.
- **Third session, same day (stopped midway at the user's request — RESUME HERE):**
  - **Every software gate re-run on the M6 build, all green:** unit tests (57 / 22 / 12),
    the four GCS suites, `m3_bridge_check` 32/32, `fence_check` 13/13, `target_filter_check`
    17/17, `loss_oracle_check` 28/28, `loss_conditioning_check` 16/16. FC bench: fault card
    empty, EKF2 `vision-only`, params as expected, clocks pinned, logloader disabled, 341 GB.
  - **`drone-dtp/scripts/bench_bringup.sh`** (+ `bench_probe.py`) = the runbook's bring-up in one
    command, observe-only, with every check and a process-group shutdown. **It ran once**
    (`~/dtp_datasets/20260914_0324_bench_bringup/`): 0 survivors, swap 1 MB. **The oracle
    answered live for the first time** — 478/480 synthetic coasts at 8 Hz, eval p99 254 µs,
    `fsm_node` 25.8 % answering vs 25.1 % idle (the open CPU item: negligible). Odom 10.0 Hz once
    settled (9.3 Hz in the first 10 s window → `estimator_preflight` NO-GO on E3 that early;
    a laptop RViz was subscribed to `/cloud_registered` throughout — the field rule).
  - **`stage_a_preflight.py` FAILED its target-exclusion line on the M6 build, wrongly**: it
    took `fsm_node`'s subscription to `/dtp/follow_target` as "filter ON", but since M6 the
    oracle holds that subscription too. Fixed to look for the filter's own
    `/rog_map/target_exclusion` publisher — **not yet re-run on hardware**; the first thing to
    do next session is `bash drone-dtp/scripts/bench_bringup.sh` and read section 4b. It also
    now checks `loss_conditioning` (must match on both nodes; false for any flight with
    authority) and that `/planning/loss_event` has a publisher.
  - **M6.6 is BLOCKED on this box:** `detection_pipeline/EigenTrajectory/`, its config, its
    checkpoint and any ETH/UCY data are absent. Needs the assets from "the AGX".
  - **Resumed the same day:** `bench_bringup.sh` ran again on the fixed preflight
    (`~/dtp_datasets/20260914_1140_bench_bringup/`): **estimator preflight GO, Stage A preflight
    GO with 2 expected warnings** (the target-exclusion line now passes), oracle live 476/480 at
    p99 89 µs, 0 survivors. **The Stage A bag** (`20260913_1955_stageA`) through the oracle:
    27 episodes, provisionally labelled (24 LOST in plain view at dusk, 2 OCCLUDED behind the
    house wall, 1 OUT_OF_VIEW); **E2 false-OCCLUDED 8/24 because the aircraft sat on the ground
    in tall grass** and the grass in front of the sensor is the mapped occluder — not a
    flight-height evaluation. Neither bag on this box can score D1; the M6.3 protocol in the plan
    now says rig at 1.5 m, FAST-LIO started on the ground, occluders above the sensor's floor,
    walk-behind events that put the CTRV point inside the shadow.

  - *(The G0.1 cycle-count result logged at this point now lives in `px4-link-and-firmware.md`.)*

  - **M6.6 DONE, rebuilt from the public EigenTrajectory sources** (`docs/m6.6-predictor-study.md`;
    the study `drone-dtp/scripts/m66_predictor_study.py`, outputs
    `~/dtp_datasets/m66_predictor_study/`). The public clone + pretrained ET-STGCNN checkpoints
    live in `drone-dtp/detection_pipeline/EigenTrajectory/` (gitignored except the shim
    `eigen.py`), so `eigen_enabled` is now one config key from loading a model — **don't turn it
    on in flight; it is not the flown model and it never was.** Full ETH/UCY run (33 654
    trajectories, 7.8 min on the NX). Four findings for the paper: (1) best-of-1 from the learned
    model is a *selector* problem — the flown adapter's raw sample 0 scores 1.39/2.88 m ADE/FDE,
    worse than constant velocity (0.52/1.14); best-of-20 (0.23/0.38) is an oracle a single-goal
    follower cannot use; (2) the "needs a complete history" sensitivity is the **imputation**, not
    the model — linear interpolation holds 0.54 m ADE from 0 % to 50 % missing, the repo's
    forward-fill degrades to 1.44; (3) the flown CTRV code cannot run verbatim on 0.4 s-spaced
    ETH/UCY (its heading baselines need > 8 points → 100 % fallback), so Table II's CTRV row is
    a gap-native least-squares fit of the same model, labelled as such; (4) CTRV ≈ CV on a 4.8 s
    horizon, and the plan's α schedule changes ≤ 0.07 m. `pip3 install --user tqdm` was the only
    install.
  - **Not started:** the 14 h powered soak.
- **Open:** M6.3 (the 140-event handheld dataset → D1, real labels), flights (→ D2), M6.7. `lost_confirm_s` (0.3) is the knob that trades glitch tolerance for early LOST
  — M6.3 sweeps it.
- **✅ M6.4b "B + B2" BUILT 2026-09-14 (later session), patch
  `drone-dtp/patches/0005-super-belief-around-obstacles.patch`, bench-gated, never flown.** The
  plan was [`docs/m6-belief-around-trees-plan.md`](docs/m6-belief-around-trees-plan.md); the
  as-built record with every number is the **"M6.4b" section of `docs/m6-loss-oracle.md`**. Don't
  re-derive any of it.
  - **B (`loss_oracle/rollout_collision`):** the reappearance rollouts cannot cross an OCCUPIED
    cell of the raw grid — the body is sampled every half cell along its lateral diameter and the
    radius ahead; a blocked sub-step deflects ±15°…±90° (own turn side first) *for that sub-step
    only*, so the rollout slides round and resumes its course; a start inside an obstacle is
    backed off (logged); a cluster mean in occupied cells becomes the medoid. **No new random
    draws: with the switch off the belief is bit-for-bit 0004's (36/36 events identical, old vs
    new binary).**
  - **B2 (`loss_oracle/hidden_look`):** when the still-hidden mass ≥ 0.5 or no rollout exits,
    the yaw target is the hidden rollouts' mean position at 1.5 s and the viewpoint search runs
    about it (the sidestep). `reappear_*` untouched; no message change.
  - **Both OFF in code, ON in `follow_test_ros2.yaml`.** Live tree patched and rebuilt
    (backup `~/backups/backup-pre-m64b-20260914-1559/`, also 0005's diff base). Verified on a
    pristine `2ad3419`+isnan export through `apply_super_patch.sh`: four files byte-identical.
  - **Gate: `loss_oracle_check.py` — new scenes O8 (around) / O8b (start inside) / O9 (behind),
    each OFF and ON, plus `--dump/--compare` pairing; 56/56 with O1–O7.** Cost with the switches
    on: classify p99 108 µs, full evaluation **p99 2.5 ms** (was 406 µs; budget 10 ms).
    `target_filter_check.py` 17/17 on the same binary.
  - **The finding that shaped it — don't re-derive:** ROG-Map holds a trunk as a one-cell shell
    of its *lit* face (six cells per height at 4.2 m); its back half, interior and tangent zones
    are UNKNOWN. The plan's five point probes straddled that shell (every rollout still walked
    through on the first build) — hence lines. And **B keeps the belief out of the *mapped* body
    only**: a rollout that gets round the lit face's edge walks through the unseen back, and the
    map cannot say otherwise. The gate's assertions measure distance to occupied cells (ON ≥ 0.19 m,
    OFF 0.03 m), not to the geometric trunk.
  - **Thresholds were measured, not assumed:** behind an r 0.3 trunk at 1 m/s only 0.31–0.59 of
    rollouts stay hidden (the plan's premise was wrong there, and the cluster viewpoint already
    looks behind it); O9 is an r 0.5 trunk at 0.7 m/s (0.62–0.84 hidden). `hidden_look_min_w` has
    **no hysteresis** — a stream straddling 0.5 flaps between the cluster mean and the look point.
  - **Traps:** ROG-Map cells are floor-indexed with centres at (k+½)·res — a point on a cell
    boundary reads whichever side; a trunk's tangent cells *flicker* (grazing hits vs misses to
    the wall behind), so the viewpoint's clear-ray test and a later read-back can disagree by one
    cell; the harness now stamps scenes from a fixed epoch so seeds repeat — and a pairing
    reference goes stale the moment a scene moves.
- **The AGX's EigenTrajectory assets were checked 2026-09-14: nothing to adopt.**
  - `drone-dtp/weights/EigenTrajectory.tar.xz` (155 MB, untracked and not gitignored — don't
    `git add -A`) holds **byte-identical public weights**.
  - Its wrapper returns NaN for any pedestrian group containing a standing person, and its single
    guess is 2× worse than constant velocity.
  - See `docs/m6.6-predictor-study.md` §8 and `drone-dtp/scripts/m66_agx_wrapper_check.py`.
- **Eva-Tracker / Faster-LIO assessed 2026-09-14 — don't re-derive:**
  [`docs/eva-tracker-faster-lio-assessment.md`](docs/eva-tracker-faster-lio-assessment.md).
  Eva-Tracker (ICRA 2026, same hardware class) is ROS 1 / MAVROS / MonoLoco / AGPL-3.0 — never
  the flight planner; its loss handling is prediction-only (Bézier QP + arc search, the authors
  say short-term only), so it becomes a re-implemented baseline arm (`baseline_mode: eva`, plan
  row 27a) and a predictor arm in M6.6 (row 24a). Faster-LIO is ROS 1 only (one Galactic port,
  5 commits) and FAST-LIO is not the bottleneck; try `extrinsic_est_en: false` (74 % → 31 %,
  row 14a) first, and swap only if the moving-rig latency check fails (rows 14b–14c). `lost_confirm_s` (0.3) is
  the knob that trades glitch tolerance for early LOST — M6.3 sweeps it.

- **The learned predictor is disabled and its weights are outside git.** `detection_pipeline` fuses
  CTRV with EigenTrajectory via `eigen_fusion_alpha`, but `eigen_enabled` defaults **False** and
  the config/checkpoint live outside the tree. Anything claiming the learned branch flew must
  check this first.
- ~~**Minor:** `scripts/px4/goal_forward.py` has no row in `scripts/px4/README.md`'s tool table.~~
  It has one (checked 2026-09-13).

See also: `paper-and-plan.md`, `super-planner-rogmap.md`.
