# CLAUDE.md — orientation for a fresh Claude Code session

Read this first. It's the context a new session (e.g. on a different drone) needs to be
useful here without the original chat history.

## What this repo is
The **integration layer** to run the HKU-MaRS **SUPER** planner on a **Livox MID360 + Jetson
Orin NX (ROS 2 Humble)** as a vision-based human-following drone. It does **not** vendor the
big third-party repos — `scripts/setup.sh` fetches SUPER, FAST-LIO, and the Livox driver at
pinned commits and applies the patches/configs that make them work together.

> **Which board am I on?** Check `cat /proc/device-tree/model`. This repo has run on two:
> an **Orin Nano** with a preinstalled **ARK OS** image, and (current, since 2026-08-30) an
> **Orin NX Engineering Reference Devkit** on **stock JetPack / L4T R36.5.2**. The difference
> matters more than anything else here — see **Board differences** in
> [`.claude/rules/px4-link-and-firmware.md`](.claude/rules/px4-link-and-firmware.md).

Start with `README.md` (the runbook). This file adds the *why* and the gotchas.

## How this file is organised — read this
This file keeps only what every session needs: where things stand, the rules that apply
everywhere, and a map. **Everything topical is in path-scoped rule files in
[`.claude/rules/`](.claude/rules/).** They were split out of this file on 2026-09-17 with the
text unchanged. Claude Code loads a rule automatically the first time you read a file its
`paths:` list matches. Two cases never trigger that, so **read the rule file yourself** when:
- the work is operational and reads no repo file first: bringing up the stack, `ros2 …`,
  `scripts/px4/*.py`, a flight;
- the files are outside this repo: `~/super_ws/src/SUPER` (SUPER and ROG-Map source and
  configs), or `drone-dtp/` opened through the `~/super_ws/src/drone-dtp` symlink.

| Rule | What it holds |
|---|---|
| [`fast-lio-and-lidar.md`](.claude/rules/fast-lio-and-lidar.md) | M1 status. FAST-LIO latency, RAM leak, C++17, frame names, the 10 Hz decision. The 3.74 km divergence (G0.2). The MID360 link. CPU pinning, swap, DDS non-blocking send. |
| [`setup-and-build.md`](.claude/rules/setup-and-build.md) | `setup.sh` history, rebuilds that revert configs, the SUPER patch in `setup.sh` (G0.4). |
| [`px4-link-and-firmware.md`](.claude/rules/px4-link-and-firmware.md) | Who owns the FC links, board differences, the Tegra UART DMA fix, `scripts/px4/` and MAVLink gotchas, the custom firmware and power module, **the `uxrce_dds_client` hardfault (flight blocker)**, the logloader loop. |
| [`px4-estimator.md`](.claude/rules/px4-estimator.md) | The M2 gate, lever arm, `EKF2_EV_DELAY`, FRD yaw, **the unmeasured yaw mount (flight blocker)**, GPS / flow / range, `MPC_ALT_MODE`, the controller-tuning baseline (→ `docs/px4-tuning-notes.md`). |
| [`px4-bridges-offboard.md`](.claude/rules/px4-bridges-offboard.md) | `super_bringup/`, the M3 gate, the odom and offboard bridges and their traps. |
| [`super-planner-rogmap.md`](.claude/rules/super-planner-rogmap.md) | SUPER's FSM and `yaw_mode`, the flight envelope, ROG-Map bounds, raycasting and clearing, patches 0002 / 0003 / 0007 (0008 is pointed to from there). |
| [`dtp-workspace.md`](.claude/rules/dtp-workspace.md) | What `drone-dtp/` is, the symlink, build and branch traps. |
| [`dtp-follow-behaviour.md`](.claude/rules/dtp-follow-behaviour.md) | `max_vel`, follow height, descent guard, geofence, `goal_mode nmpc` (the receding-horizon follow goal), `run.sh` config selection, perception settings, the GPU cloud. |
| [`dtp-calibration.md`](.claude/rules/dtp-calibration.md) | The deployed camera↔LiDAR calibration and what is still wrong with it. |
| [`m6-loss-oracle.md`](.claude/rules/m6-loss-oracle.md) | M6: loss oracle (0004), belief around obstacles (0005), the visibility field (0008, not applied), conditioning, replay harness, predictor study. |
| [`gcs-dashboard.md`](.claude/rules/gcs-dashboard.md) | The web GCS: supervisor, host power, bags, the service, hostname and URL, the follower row. |
| [`gcs-remote-control.md`](.claude/rules/gcs-remote-control.md) | The RC card, and SUPER's two goal inputs. |
| [`gcs-camera.md`](.claude/rules/gcs-camera.md) | The camera panel and the Admin-control gate. |
| [`laptop-rviz.md`](.claude/rules/laptop-rviz.md) | RViz on a laptop, the DTP image view, the snap glibc leak. |
| [`flight-log.md`](.claude/rules/flight-log.md) | Every flight and field session, and the M5.1 / M5.4 / M5.5 procedures. |
| [`paper-and-plan.md`](.claude/rules/paper-and-plan.md) | The M5/M6 plan and the paper draft's rules. |
| [`docs-folder.md`](.claude/rules/docs-folder.md) | Which `docs/` files are current and which are superseded. |

**Where new knowledge goes:** into the rule for that area, dated, not here. This file only takes
a change to the current position, a new flight blocker, or a rule that applies everywhere. A new
area gets a new rule file with a `paths:` list, and a row in the table above.

## Milestones at a glance
- **M1** sensing & state estimation — **done.** → `fast-lio-and-lidar.md`
- **M2** FC link + external odometry — **gate signed off 2026-09-02.** →
  `px4-link-and-firmware.md`, `px4-estimator.md`
- **M3** controller bridge — **gate signed off 2026-09-02**, props off. → `px4-bridges-offboard.md`
- **M4** vision goal adapter — **already exists** as `drone-dtp/`. → `dtp-workspace.md`
- **M5** the follower flies — plan in [`docs/m5-m6-plan.md`](docs/m5-m6-plan.md). First Offboard
  flight 2026-09-14 (44 s, no following in the air). Second 2026-09-15 (87 s in Offboard,
  `FOLLOW|OK` held 103 s). → `flight-log.md`, `dtp-follow-behaviour.md`
- **M6** the paper's contribution — **built 2026-09-14**, bench- and replay-gated, never flown.
  → `m6-loss-oracle.md`, `paper-and-plan.md`

## Where the work is right now — READ BEFORE STARTING
### ▶ NEXT: the first person-following flight — [`docs/flight-day-runbook.md`](docs/flight-day-runbook.md)
The user has the transmitter and wants to fly with a person to follow. The runbook is the day's
script: desk check → venue bring-up → **Sortie 1 Stage A (observe-only, never flown on a sane
estimate)** → **Sortie 2 M5.1 (props off, bench-rehearsed only)** → **Sortie 3 Stage B S1**, each
scored before the next. **The web GCS's "Flight day" card is, since 2026-09-20, the standard
follower bring-up and no longer the runbook row for row:** an Observe only / Live choice, then
twelve buttons in order (stack on follow, bridges, FAST-LIO gates, follower, record, preflight,
Engage FOLLOW, estimator, the stops, fault scan), each with a PASS/FAIL pill; it adds no new way
to reach the aircraft (→ `gcs-dashboard.md`). New checks behind it: `scripts/desk_check.sh`, `rc_switches`,
`field_check` (= `field.sh check`), the Stage A / Stage B preflights; `bagger.PRESETS` stage_a /
m51 / stage_b. Bench pre-check ran 2026-09-14 17:24: estimator GO, Stage A preflight GO, 0
survivors; the transmitter was off so the RC link is unverified (params say ch11 Offboard, ch5
modes). Two orphaned `dtp_viz`/`dtp_image_overlay` nodes from 01:27 were stopped — they would
have blocked the follower's Start. The two flight blockers (hardfault soak, yaw mount) are still
open and are the user's call.

**Latest changes** (full records in the rules):
- **2026-10-17: the MPC trajectory tracker (`px4_mpc_tracker.py`) is integrated, OFF by default,
  NEVER flown, not installed.** It replaces `px4_offboard_bridge.py` (never both) and sends PX4
  acceleration setpoints. It keeps the bridge's Offboard-only setpoints and its yaw servo
  (off), and adds a 1 m far-setpoint clamp. Select it with `GCS_TRACKER=mpc`,
  `deploy_follow.sh --tracker=mpc` or `TRACKER=mpc bench_bringup.sh`; without them everything
  starts the bridge, as before. Gates: `mpc_tracker_sim.py` 8/8, new `mpc_tracker_node_check.py`
  21/21 (ROS stubbed). **Not done:** a real-DDS sandbox and anything on the FC.
  **`m3_gate_check.py`'s drift check passes vacuously against it.**
  → `px4-bridges-offboard.md`, `docs/mpc-tracker.md`, `docs/mpc-tracker-integration.md`
- **2026-10-02 ~21:30: `perception_node` `max_step_m` 2.0 (user; node default stays the
  library's 0.35). Bench-gated by replay, NOT flown.** The frozen predictor's clamp-then-EMA
  capped the published target position at 0.25 × 0.35 = 0.0875 m per frame, **0.9 m/s**: in the
  20:03 flight a jogging walker read a constant 3.7 m while the depth saw them leave to 9.3 m,
  until the track was rebuilt at 9.4 m and LOST. Passed through as a node parameter, no edit to
  `detection_pipeline/`. New gate `max_step_replay_check.py` (the flight's frames through
  the real filter methods). **Later the same evening, `smooth_alpha` 0.5 (user; library 0.25),**
  exposed the same way: the EMA lag drops from 0.71 to 0.24 m median on that walker, at the
  price of 0.58× instead of 0.38× of a standing person's jitter reaching the wire and a 1.0 m
  outlier bound. Gate 9/9. **Relaunch the follower to pick both up.** Undo: `max_step_m: 0.35`,
  `smooth_alpha: 0.25`, or delete the lines.
  → `dtp-follow-behaviour.md`, `flight-log.md`
- **2026-10-02 20:03: a person-following flight, 68 s in Offboard, first on the rollback.** The
  geofence held on both axes, park 0009 worked at the takeover, and there was a second far-side
  close pass at 2.58 m (the 09-29 mechanism; its fixes are not on this branch). → `flight-log.md`
- **2026-10-01 ~02:30: three features ported back onto the rollback (user). Bench-gated, NOT
  flown.**
  - **The 09-30 camera↔LiDAR calibration:** `drone_cam_livox.yaml` is the checkpoint's (+1.09°,
    camera 11.5 cm below the LiDAR). Live through the symlinked install.
  - **The braking envelope** (`brake_envelope_enable: true`): `brake_envelope.py`, the
    speed-widened descent-guard radii, and `follower.launch.py` passing SUPER's `max_acc` (2.0)
    to the mux. **Not ported with it:** follow_planner's speed-scaled standoff, the pass-by
    radius and the flyable HOLD, and the `stage_a_preflight` envelope checks.
  - **The veto's 3.0 m height band** (`proximity_veto_half_height_m: 3.0`).
  - `command_mux` and `dtp_bringup` rebuilt. **Relaunch the follower** to pick all three up.
  - **Gates:**
    - `brake_envelope_check` 13/13;
    - `descent_guard_check` 13/13, both with the envelope off and on;
    - `engage_gate_check` 14/14, `fence_check` 13/13, `terrain_check` 10/10, `park_check` 14/14;
    - the mux unit tests, 76 passed.
  - **Undo:** set either switch back (`false` / 2.0) and relaunch. For the calibration, copy
    `git show c0e3930:drone-dtp/ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml` back.

  → `dtp-follow-behaviour.md`, `dtp-calibration.md`
- **2026-09-30 ~23:00: a pilot takeover now parks SUPER (user). Bench-gated, NOT flown.** It is
  on the rollback branch, on top of the entry below.
  - **Why:** the GCS stopped answering at 18:38. After the takeover SUPER re-planned to its
    stale goal 25 times a second for 45 minutes (`fsm_node` at 144 % of a core, load 10–12).
    SUPER's FSM does that after any takeover that leaves its goal unreached; it is not specific
    to the code that was rolled back.
  - **What:** new SUPER patch **0009** adds a park input (`/planning/park`), and `command_mux`
    sends it when the engage gate shuts. SUPER drops the goal and idles; the next Offboard plans
    from rest at the aircraft.
  - **`super_planner` was rebuilt** (23:06). Old binary: `~/backups/super-pre-park-20260930/`.
  - **Gates:** new `park_check.py` 14/14; `engage_gate_check` 14/14; the other mux and SUPER
    gates unchanged.
  - **The LiDAR ranging was not the CPU load:** it adds 5–8 ms to a perception frame.
  - **Five flights of 2026-09-30 and one of 09-29 (12:07) have no flight-log entry yet.**
  - **⚠️ `apply_super_patch.sh` applies patch 0008 to the live tree.** Hit and reverted.

  → `dtp-follow-behaviour.md`, `super-planner-rogmap.md`
- **⛔ 2026-09-30 ~19:00: THE CODE IS ROLLED BACK to before 2026-09-26 (`c0e3930`, 2026-09-23;
  user's decision).** It is on branch `rollback-pre-0926`. Everything later is in tag
  `checkpoint-2026-09-30` (= `nmpc-follow-goal` at `cdf52cf`).
  - **Why (the user):** go back to the last code that ran with no problem, before the
    2026-09-26 afternoon speed bump (`max_vel` 3.0 / `max_acc` 2.0, made from the Remote
    Control session at 16:48).
  - **What was restored:**
    - every non-markdown file under `drone-dtp/`, `scripts/` and `super_bringup/`;
    - SUPER's envelope in `follow_test_ros2.yaml` (src + install): `max_vel` 3.0 → 1.0,
      `max_acc` 2.0 → 1.0, matching patch 0001 again;
    - `super_bringup` re-copied into `~/super_ws/src`;
    - `super_bringup command_mux dtp_perception dtp_bringup follow_planner` rebuilt
      (`--symlink-install`).
  - **Docs, rules and this file were NOT rolled back.** Every entry below dated 2026-09-26 or
    later describes code that is not on disk now.
  - **Later the same evening the envelope went back to `max_vel` 3.0 / `max_acc` 2.0 (user).**
    Set in `follow_test_ros2.yaml` (src + install) and patch 0001. `--max-vel 3.0` is back in
    `deploy_follow.sh` and `supervisor.py`. These are the checkpoint's versions of those three
    repo files. It needs a relaunch and a `super-gcs` restart. No rebuild is needed.
  - **Carried back on top, the same evening (user), each as it was at the checkpoint:**
    - **the bridge:** acceleration feedforward ON and `setpoint_only_in_offboard` ON. The
      camera yaw servo is in, with both switches OFF (never flown; its replay gate fails).
      `px4_offboard_bridge.py`, `yaw_servo.py` and `offboard_viz.py` are the checkpoint's,
      except one comment. So are the scripts that stopped relying on the IDLE stream
      (`offboard_engage`, `m3_gate_check`, `m51_one_graph_check`, the GCS telemetry chip), and
      the yaw servo's gates and tools;
    - **the yaw hold after a loss** (`lost_yaw_hold_s` 15, `coast_yaw_hold` true), ported
      alone out of 2efbe7c. The speed standoff it was committed with stays out.
      `lost_yaw_hold_check.py` and `loss_conditioning_check.py` are the checkpoint's.
  - **Gone with it:**
    - the speed-scaled standoff and the braking envelope (**the envelope is back since
      2026-10-01**);
    - the 4–5 m band (back to 3–4 m);
    - the three close-pass fixes and the 3.0 m veto band (**the band is back since 2026-10-01**);
    - Part B LiDAR ranging (the caps are back to 10 m);
    - the 09-30 calibration (back to the 09-13 deploy, 3.6° off in pitch; **the 09-30 file is
      back since 2026-10-01**);
    - the 38 × 6 m, −17° geofence (back to 30 × 4 m, x [−2, 28], yaw 0).
  - **Still there (older than 09-23):** the engage gate, terrain-relative height, the HOLD-climb
    fix, auto re-acquire, `goal_mode` (ships geometric).
  - **FC params are unchanged.** None was changed after 2026-09-21.
  - **Relaunch everything to run it.** The nodes running at 19:00 were started on the new code.
    `super-gcs` must be restarted for `supervisor.py`'s `--max-vel 1.0`, and its restart stops
    the stack.
  - **Undo:**
    1. `git switch nmpc-follow-goal`;
    2. copy back `~/backups/super-cfg-pre-rollback-20260930/follow_test_ros2.{src,install}.yaml`;
    3. re-copy `super_bringup`;
    4. rebuild the five packages.
  - **Gates on the plain rollback (before the fixes above):** `m3_bridge_check` 32/32,
    `engage_gate_check` 9/9, `fence_check` 13/13, `descent_guard_check` 13/13,
    `terrain_check` 10/10, `nmpc_check` 41/41, `loss_conditioning_check` 16/16.
  - **Gates with the fixes carried back:**
    - `yaw_servo_unit_check` 139/139, `lost_yaw_hold_check` 26/26, `nmpc_check` 41/41;
    - `loss_conditioning_check` 18/18. The first run was 15/18: one coasting target stamped
      54.81 s after the vanish, under load 10–15 with the live stack up, which failed the three
      S2 ON checks;
    - `m3_bridge_check` 120/120, **run with `ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1`.** On the
      default domain it was 119/120, because "Ya switch off" looks the bridge up by node name
      and found the live `px4_offboard_bridge`. It always does when a bridge is running;
    - `fix1_yaw_replay --gate` 10/18, the known result (the servo ships OFF).
- **2026-09-30 ~16:45: a new camera↔LiDAR calibration is DEPLOYED (user). Motors off, NOT flown.**
  - **Why:** the old file (09-11 solve) was 3.6° off in pitch and put the camera 2.3 cm *above*
    the LiDAR. That is the "~3° pitch" found in every flight bag.
  - **The new one:** the camera's axis is +1.09° above the LiDAR's x-y plane, and the camera is
    11.5 cm below and 13.8 cm ahead of the LiDAR.
  - **How:** solved on the NX, with the drone on a conference table: ICP of the camera's depth
    cloud against the MID360, four viewpoints.
  - **Checks:** the independent tabletop and LiDAR-intensity checks pass (old file: 14 cm and
    21 px off). **`score_alignment` fails both files in that room** (mesh chairs, parallax).
  - **What changes:** every camera-lifted point moves, Δz ≈ 0.063·r − 0.14 m (+37 cm at 8 m).
    `lidar_box_top_extend_px` 45 (the old pitch workaround) and the Part B scores need
    re-scoring.
  - **Relaunch the follower to use it;** no rebuild. Undo: copy back
    `~/backups/calib-pre-20260930-table-deploy/drone_cam_livox.yaml`.

  → `dtp-calibration.md`, `docs/evidence/calib-2026-09-30-table/`
- **2026-09-30 14:55 bench bag (drone on a table indoors): the Part B hybrid read a walker 3–5 m
  too FAR, here and live at 14:22.**
  - **The cause:** its ground rejection takes the ceiling or furniture for "ground" where the
    MID360 cannot see the floor (inside ~7 m here). It deletes the walker, and the wall behind
    becomes the range.
  - **Healthy-timing replay, ON vs OFF:** ON is better at 6–10 m (100 % within 0.3 m at 8–10 m,
    vs 84 % / 0 %). It is worse at 4–6 m (median +3.2 m on a walking walker, vs OFF's +0.85 m
    lag). Overall 29 frames > 1 m far, against OFF's 18.
  - **Left unfixed, the user's decision:** the table setup caused it. Watch bags for a LiDAR
    range far beyond the camera's. A guard is prototyped if it recurs.
  - **The dtp_viz person cylinder is fixed** (it was centred on a knee-height point). The ~2.5°
    camera pitch error is now attributed to the calibration's rotation.
  - ***All topics* recording blinded perception (77 %):** record a preset.

  → `dtp-follow-behaviour.md`, `dtp-calibration.md`, `gcs-dashboard.md`
- **2026-09-30: Part B, LiDAR ranging past 10 m, is built and ON in `dtp_params.yaml`
  (`lidar_range_enable`; caps 15 m). Bench-gated, NOT flown.**
  - **Step 0: GO in daylight, no gain at dusk.** On 09-29 the flown detector boxes the walker in
    100 % of in-FOV frames at 8–17 m (BoT-SORT 98–100 %). At dusk (09-26 20:12) it is 5 % at
    10–12 m and 0 % at 12–15 m, on boxes bigger than the day's: light, not pixels.
  - **The design:** the camera ranges to 8 m. Beyond it a track moves to the LiDAR (whole box,
    0.6 s motion-shifted window, ground rejected, ≥ 3 points to switch, ≥ 2 to stay, back below
    6.5 m), entering before the EMA. The bearing stays the camera's; an attitude-blind LiDAR
    holds the range up to 1.0 s. New `lidar_box_range.py`; unit tests 97 passed.
  - **A cap bug, found and fixed (hybrid only):** the flown `_sample_patch` masked depth beyond
    `max_depth_m` before picking the near surface. So past ~8 m the camera read 1–4 m short, on
    near clutter (median −1.2 m at 8–10 m).
  - **⚠️ The camera and the LiDAR disagree by ~3° in pitch** (2.3–3.5°, all five bags, motors
    off too). The belief was ~1° or less. The camera-lifted target z is ~0.05·r too low. **Fixed
    by the 2026-09-30 re-calibration (top entry).** M5.4b is still owed. → `dtp-calibration.md`
  - **The user's call (D13):** at 15 m the acquisition band holds more bystanders. Auto
    re-acquire after a LOST is held to 10 m (`auto_reacquire_max_range_m`); an operator's engage
    reaches 15 m.
  - **Relaunch the follower to run it;** no rebuild. Undo: `lidar_range_enable: false` and the
    three caps (`max_depth_m`, `prediction_depth_gate_m`, `acquire_max_range_m`) back to 10.0,
    then relaunch.
  - **Step 3 replay scoring, 09-29 bag (two rounds; the second fixed the published position
    lagging the LiDAR and the box starved by the 3° pitch):** ranged frames at 10–15 m **82 %
    vs 9 %** without it, published range within 0.12–0.18 m of the LiDAR truth; **the ≥ 90 %
    bar still fails** because the MID360's −7° floor loses the walker under nose-up pitch.
    Close range unchanged within noise, dusk invents nothing. `lidar_box_top_extend_px` 45 is
    a workaround for the pitch calibration.

  → `dtp-follow-behaviour.md`
- **2026-09-30: the proximity veto's height band is 3.0 m** (`proximity_veto_half_height_m` in
  `dtp_params.yaml`; plan Part A). **Bench-gated, not flown.**
  - **Why:** the band is ± that around the tracked point, which is the walker's lower body
    (z 0.26–0.72 on 09-29), not the centroid. At 2.0 the aircraft at z 2.27–2.73 was outside it,
    and the braking envelope skipped the walker. The node default stays 2.0, as flown.
  - **`close_pass_check.py` 23/23** (was 17/17). At 2.0 a high aircraft is never vetoed, down to
    4.5 m. At 3.0 the pass-by veto fires at 9.28 m and the braking envelope at 5.09–5.30 m.
  - **In the 09-29 replay fix 2 still fires first, at t 454.63, the same frame as at 2.0.** The
    envelope now sees the walker, but the aircraft was still slow, so it is a second, later layer
    from ~456.1 (at 2.0 it was blind until 456.5). The plan expected it from ~454.4.
  - **Relaunch the follower to pick it up;** no rebuild. Undo: set it to 2.0 and relaunch.
  - **Replayed on every flight bag at 2.0 and 3.0 (`veto_height_bag_scan.py`): no new and no
    earlier veto anywhere.** On 09-21 18:41 it covers 35 s of Offboard person time the 2.0
    envelope ignored. One veto on 09-26 20:12 releases 0.56 s later, with no change of goal.

  → `dtp-follow-behaviour.md`
- **2026-09-30: three fixes for the 09-29 close pass, all ON in `dtp_params.yaml` (user).
  Bench-gated, not flown.** `follow_planner` was rebuilt.
  - **Fix 1:** `transit_passby_check_enable`. A goal whose path passes *by* the walker, now or
    on their forecast, closer than the floor is withheld.
  - **Fix 2:** `passby_reach_enable`. For such a path the mux's veto radius grows with the
    closing speed.
  - **Fix 3:** `hold_stop_projection_enable`. A HOLD at speed is latched where a stop can end,
    so SUPER can fly it.
  - **New gate `close_pass_check.py` (domain 95), 17/17.** It replays the real event. Fix 1
    withholds the flown far-side goal, but only by 0.15 m. Fix 2 vetoes 1.6 s earlier than
    flown.
  - **Still open:** SUPER's replans failed before the HOLD, for an unknown reason. (The
    veto's ±2 m height band, measured from a low tracked point, is fixed: entry above.)
  - **Also answered:** the LiDAR cannot simply extend the 10 m range. It puts 1–5 points per
    scan on a walker at 10–17 m, which is too few for the LiDAR path as written. (Since built as
    a hybrid over a longer window: the Part B entry above.)
  - **▶ NEXT (user, 2026-09-30):** both parts of
    **[`docs/plan-veto-height-and-lidar-range.md`](docs/plan-veto-height-and-lidar-range.md)**
    are built, neither flown: Part A (the height fix) and Part B (LiDAR ranging). What remains:
    the user's decisions (D13 and the 10 m auto re-acquire cap; the ~3° pitch is re-calibrated),
    then a Stage A observe-only walk out to 15 m and back, comparing the ranges with the LiDAR
    map (plan Step 4).

  → `dtp-follow-behaviour.md`
- **2026-09-29: the geofence is 38 m long.** `fence_x_max_m` went 28.0 → 36.0 in
  `dtp_params.yaml` (user). **Not flown.** It is sized for that venue, started at the same spot
  and heading.
  - The field rises along +x (+1.1 m at x 36), and SUPER snaps goals above z 2.25. So beyond
    x ≈ 36 it could not hold the 1.2 m follow height.
  - Longer needs SUPER's ceiling raised.
  - `fence_check` 13/13. Relaunch the follower to pick it up.

  → `dtp-follow-behaviour.md`
- **2026-09-29 12:32: a person-following flight, 61.5 s in Offboard.** It was the first flight of
  the 4–5 m band, the speed-scaled standoff, the braking envelope, acceleration feedforward and
  `setpoint_only_in_offboard` (yaw servo off).
  - **Past 10 m the follower is blind by design.** Perception's `max_depth_m` is 10.0 and
    target_manager's `acquire_max_range_m` is 10.0. The walker got past 10 m each time the
    aircraft stood still:
    - after a 1.2 s dropout (LOST, then HOLD in place);
    - **at the geofence.** It clamped every goal to x 27.5 for the last 10 s, while `/mux/state`
      read FOLLOW|OK.
  - **⚠️ A close pass at 3 m/s.** The walker turned back toward the aircraft, and the goal went to
    their far side. The aircraft passed an inferred 0.9–1.5 m from them, ~0.5 m above head height.
    Three causes, none fixed:
    - the goal is always "behind" the walker;
    - both path checks use fixed radii (2.0 m and 1.5 m) against where the walker is now;
    - **a HOLD at speed is a stop point SUPER cannot fly**, so it keeps its old plan. The same
      thing sent the aircraft 3.4 and 5.3 m past two other HOLDs.

    **Recommended until it is fixed (the user's call): the walker does not walk back toward the
    aircraft in Offboard.**

  → `flight-log.md`
- **2026-09-29: the follow band is 4–5 m** (`follow_near_m` 4.0, `follow_far_m` 5.0 in
  `dtp_params.yaml`; user decision). It was 3–4 m on every flight so far. The floor stays
  2.5 m. **Not flown.** No lock has been held in flight beyond 4.4 m, and in a sprint the
  speed-scaled standoff now reaches 5.5–6.5 m. → `dtp-follow-behaviour.md`
- **2026-09-29: the camera yaw servo (fix 1, fix 2) is built in `px4_offboard_bridge`. Both
  switches are OFF, and it is not flown.** The bridge aims the heading from the aircraft's
  position to `/planning/yaw_target`, with an optional turn-rate feed-forward.
  - Unit check 139/139 and `m3_bridge_check.py` **120/120** (was 51). With the switches off
    the bridge's output is byte-identical to before.
  - **⚠️ The replay gate fails, 10/18.** The close-range rule at 2.5 m (the follow floor) froze
    the heading in the 09-26 19:51 replay: off-centre max 38° against 20° as flown.
    `yaw_servo_min_range_m` is the user's call.
  - **The servo alone does not prevent the swing after a loss.** That depends on the
    2026-09-26 yaw hold.
  - **The bridge imports `yaw_servo.py` even when off.** Commit the two together.

  → `px4-bridges-offboard.md`, `docs/plan-camera-yaw-servo.md`
- **2026-09-27/28: four changes after the 09-26 overshoot. All bench-gated, none flown.**
  - **Why.** At the 09-26 engage the aircraft ran 0.8 m past SUPER's reference and closed to
    2.57 m from the walker. The bridge sent no acceleration, so PX4 made it only by falling
    behind.
  - **Acceleration feedforward is ON** in `px4_offboard_bridge` (`publish_acceleration_ff`),
    sent only while tracking SUPER. The aircraft now follows SUPER's path more closely,
    **including its descents**, and z adds up to ±20 % of hover thrust.
  - **The IDLE-bridge interference is fixed on the bench** (`setpoint_only_in_offboard`):
    setpoints go out only while PX4 is in Offboard. **Centre the sticks before engaging:** for
    an estimated 40–70 ms PX4 flies its last manual setpoint. **The props-off check passed
    2026-09-28 (TODO step 1). Until one manual flight also passes, keep the bridge stopped
    for manual flights.** PX4 will not arm from the transmitter in Offboard: arm in
    Position, then ch11. `m3_bridge_check.py` 51/51 then (120/120 since 2026-09-29).
    → `px4-bridges-offboard.md`
  - **A speed-scaled standoff and a braking-envelope veto, ON since 2026-09-28 (user)**
    (`speed_standoff_enable`, `brake_envelope_enable` in `dtp_params.yaml`). The follow
    distance grows with the closing speed, capped at 5.0 m. New gate
    `brake_envelope_check.py` (domain 94). It does not address the 09-26 turnaround approach.
    → `dtp-follow-behaviour.md`
  - **A velocity-loop study and tuning flight plan.** No gain changed. Fly the feedforward
    alone first. → `docs/px4-velocity-tuning-plan.md`, `px4-estimator.md`
- **2026-09-26 19:57 and 20:12: two person-following flights.** These were the first flights of
  `max_vel` 3.0 / `max_acc` 2.0, the engage gate and terrain-relative height.
  - **The engage gate worked** (0.1 m / 0.0 m at the switch).
  - **Terrain height tracked the slope,** but at the latched descent-guard floor (1.8–2.3 m AGL).
  - Flight 1 followed 22.8 m. **⚠️ Then, at the walker's turnaround, SUPER's own trajectory took it
    to 0.58 m AGL, 2.9–3.1 m from the walker.** The cause is not established, and nothing guards
    SUPER's path.
  - Flight 2 lost the walker at 3.1 m in the dusk. **⚠️ SUPER then yawed the camera 83° away.**
  - **⚠️ SUPER snaps goals above z 2.25 m (`lidar_init`)**, which caps terrain height on rising
    ground.
  - **Built the same evening, not flown: a yaw hold.** From 0.4 s into a coast, and after a
    loss, `follow_planner` holds one heading toward where the camera was last aimed, for up to
    15 s (`lost_yaw_hold_s`, `coast_yaw_hold`). **The coast half changes the M6.5 carry-forward
    arm.** The installed binary was rebuilt for it. → `dtp-follow-behaviour.md`

  → `flight-log.md`
- **2026-09-21 20:05: a second person-following flight kept pace, 33 s in Offboard.** The walker
  did 0.61 m/s and the aircraft 0.65 m/s, and the gap closed 4.35 → 3.51 m. SUPER had 0 backup
  OPT_FAILED. It was the first flight of the HOLD-climb fix, and the fix held.
  - **⚠️ At the Offboard engage, `pos_cmd` was 3.0 m from the aircraft.** It closed the gap at
    1.7 m/s (`max_vel` 1.0), with the walker nearby.
    - **Fixed on the bench 2026-09-22, not flown:** `command_mux`'s engage gate sends SUPER
      nothing until armed + Offboard, so its first plan starts from the aircraft
      (`dtp-follow-behaviour.md`).
    - **Still open:** a re-engage within seconds of a takeover, and goals that bypass the mux.
      Those need the bridge to refuse a far setpoint (not built).
  - **The bag was cut by a power loss 10 s after recording stopped.** Use
    `bags/stage_b_2026-09-21_195658_salvaged`. **Run `sync` before pulling the battery.**
  - **The baseline follow is not goal-placement-bound, so `goal_mode nmpc` is recommended
    parked** (the user's call).

  → `flight-log.md`
- **2026-09-21 18:41: a person-following flight followed for 23 m in Offboard, then lost the
  walker because it was too high.** A mux HOLD's +0.75 m climb latched the descent-guard floor at
  2.10 m for the rest of the follow. The field also slopes down ~1.2 m, and the follow height is
  absolute in `lidar_init`. Together they put the aircraft 3.4–3.7 m above the ground, and the
  walker left the bottom of the level camera's frame. **Two fixes built on the bench the same
  evening:**
  - a HOLD's climb no longer raises the floor (only HOLDs flown in Offboard count). **Flown at
    20:05, and it held;**
  - **a terrain-relative follow height, ON since 2026-09-22 (user), not yet flown**
    (`command_mux: terrain_follow_enable` in `dtp_params.yaml`). Engage Offboard near 1.2 m, or
    the descent guard's floor holds the engage height above the ground. It uses a LiDAR ground
    map, not the rangefinder, and starts a new `terrain_estimator` node. Replaying the bag, it
    would have held 1.23–1.51 m above the ground.

  → `flight-log.md`, `dtp-follow-behaviour.md`, `docs/fact-terrain-follow.md`
- **2026-09-21: `max_acc` 0.5 → 1.0** in `follow_test_ros2.yaml` (src + install) and patch
  0001. The S2 envelope (1.0 m/s, 1.0 m/s²) is now complete. **Not flown.** → `dtp-follow-behaviour.md`
- **⚠️ 2026-09-21: `px4_offboard_bridge` in IDLE corrupts Altitude and Position mode.** It
  publishes its "hold here" `trajectory_setpoint` at 50 Hz, even when not in Offboard. PX4's own
  flight task writes that same uORB topic, so `mc_pos_control` alternates between the two. In
  the retune's Flight 2, the attitude setpoint flipped by up to 45° and the aircraft pitched
  against the pilot's stick. It covers every manual flight with the bridge up, **including the
  manual takeover from Offboard.** **Fixed on the bench 2026-09-28, not flown** (entry above).
  Until its props-off check and one manual flight pass, stop the bridge for non-Offboard flights. → `px4-bridges-offboard.md`
- **⚠️ 2026-09-21: `SENS_BAR_AUTOCAL` (1) rewrote the baro calibration in flight.** It stepped
  the baro +4.2 m, and ~1 m leaked into EKF2 height (Flight 3, no LIO). **The user set it to 0,
  and `MC_BAT_SCALE_EN` to 1 (battery scaling, not yet flown).** Dump:
  `firmware/px4-params-2026-09-21-post-batscale-baroautocal.params`.
  → `px4-estimator.md`
- **2026-09-21:** retune Flights 3–4 flew **with no FAST-LIO in EKF2**: baro, ToF, flow and
  compass. They are the first arms without LIO on record; the preflight would have refused them.
  Roll and pitch are critically damped, yaw is fine (an earlier "yaw lags" was not confirmed),
  vertical velocity overshoots ~20 %. The height
  wander was the throttle resting outside PX4's ±0.1 hold band. → `px4-estimator.md`,
  `docs/px4-tuning-notes.md` §9
- **2026-09-21:** retune Flight 2 (wobbles, Altitude) confirmed `IMU_DGYRO_CUTOFF` 40. Roll P:D
  is right and pitch is not overdamped. → `px4-estimator.md`
- **2026-09-21:** retune Flight 1 (Stabilized hover) was flown and read. `IMU_DGYRO_CUTOFF`
  went 55 → 40 because the motor outputs are D-term-noisy. The notches and gyro low-pass are
  unchanged. The rate P:D is unread: Flight 2 must fly sharp wobbles. → `px4-estimator.md`,
  `docs/px4-tuning-notes.md` §9
- **2026-09-19/20:** `goal_mode: nmpc` — a receding-horizon follow goal in `follow_planner`
  (Stage 1), and a visibility cost fed by a new SUPER patch **0008** (Stage 2). Bench-gated,
  **never flown, and 0008 is not applied to the live SUPER tree.** `goal_mode` ships `geometric`.
  **Since 2026-09-26 the installed `follow_planner_node` contains this code**, because it was
  rebuilt for the yaw hold after a loss. It stays dormant under `geometric`: `nmpc_check` N1 shows
  the geometric goals bit-identical to the old binary. **`dtp_msgs` must be rebuilt before
  `follow_planner`.** Branch `nmpc-follow-goal`.
  → `dtp-follow-behaviour.md`, `m6-loss-oracle.md`, `docs/fact-nmpc-follow-goal.md`
- **2026-09-17:** `MPC_ALT_MODE` changed from 2 to 1 (terrain following) for Position and
  Altitude mode. Offboard is untouched. **Not yet flown.** → `px4-estimator.md`
- **2026-09-16/17:** `max_vel` raised to 1.0, every follow goal set to 1.2 m, and the descent
  guard in `command_mux` now latches. **Bench-gated, not flown.** → `dtp-follow-behaviour.md`
- **2026-09-15:** `auto_reacquire_after_lost: true` and `drone-dtp/scripts/deploy_follow.sh`
  were first used in the 20:01 flight. → `flight-log.md`, `dtp-follow-behaviour.md`
- **2026-09-15:** patch 0006 (yaw hold) held in the air. Patch 0007 (ROG-Map unhit decay) ran
  live; its effect on a walker's remnants is not yet scored. → `flight-log.md`,
  `super-planner-rogmap.md`

**Still open, and the user's call: the two flight blockers named above.**
- **The `uxrce_dds_client` hardfault (G0.1).** The 14 h powered soak and the shell-path
  reproducer are still owed. → `px4-link-and-firmware.md`
- **The unmeasured MID360→FC yaw mounting offset.** → `px4-estimator.md`

## Rules that apply everywhere
- **Never open the FC serial devices directly.** ARK's `dds-agent.service` and
  `mavlink-router.service` own both links, and a second reader corrupts the stream. Use MAVLink
  UDP **14569** (`scripts/px4/`) or **14550** (QGroundControl). Details: `px4-link-and-firmware.md`.
- **The FC is an ARK FPV running PX4 v1.17 + uXRCE-DDS.** It is not ArduCopter/MAVROS, and not
  an ARKV6X.
- **After an FC power cycle, run `sudo systemctl restart mavlink-router`.** Otherwise
  `scripts/px4/*` see no PX4 heartbeat.
- **After any JetPack update or reflash, run `bash scripts/uart_loopback_test.sh`.** The Tegra
  UART DMA fix is a DTB override that an update silently removes.
- **After any stock PX4 firmware update, the power-monitor driver is gone.** See
  `firmware/README.md`.
- **Before believing any perception or odometry result, check the sensor chain:**
  - `ip -brief link show eno1` and `ping 192.168.1.154`;
  - exactly one Livox driver, one FAST-LIO and one `fsm_node` are running.

  A second Livox driver is what sent FAST-LIO 3.74 km off (G0.2).
- **`python3 scripts/px4/estimator_preflight.py` must say GO before every arm.**
- **Nobody subscribes the laptop to `/cloud_registered` while flying.** It is a field rule, and
  every gate refuses on it. See `fast-lio-and-lidar.md`.
- **Dump the FC params before any change:**
  `python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F).params`.
- **`super_bringup/` is COPIED into `~/super_ws/src/`, but `drone-dtp/` is SYMLINKED.**
  - Editing `super_bringup/` does nothing until you re-copy and rebuild
    (`px4-bridges-offboard.md`).
  - Editing `drone-dtp/` changes what the drone runs.
- **Build and branch traps:**
  - Never run two `colcon build`s at once.
  - Checking out a branch without `drone-dtp/` deletes it from disk (`dtp-workspace.md`).
  - Rebuilding `fast_lio` or the Livox driver silently reverts the repo configs, so re-run
    `scripts/setup.sh` (`setup-and-build.md`).
- **Two gates or replays on one `ROS_DOMAIN_ID` collide silently.**
  - Domains in use: 86 `descent_guard_check`, 87 the loss replay tools, 88
    `target_filter_check` **and** `loss_oracle_check` **and** `visibility_field_check` **and**
    `yaw_hold_check` **and** `park_check`, 89
    `unhit_decay_check` **and** `fence_check` **and**
    `loss_conditioning_check` (three on one domain: never two of them at once), 90 `nmpc_check`
    and `nmpc_replay`, 91 `terrain_check`, 92 `engage_gate_check`, 93 `lost_yaw_hold_check`,
    94 `brake_envelope_check`, 95 `close_pass_check`.
  - **Different domains still collide under load:** three gates at once on top of the live stack
    failed on discovery timing (2026-09-21). Run gates one at a time.
  - Check `pgrep -af` for a running harness first.
- **Re-run the gate for what you changed:**
  - either bridge: `scripts/px4/m3_bridge_check.py`;
  - the MPC tracker (`px4_mpc_tracker.py`, `mpc_tracker_core.py`):
    `scripts/px4/mpc_tracker_sim.py` and `scripts/px4/mpc_tracker_node_check.py` (no ROS);
  - the yaw servo (`yaw_servo.py`): `scripts/px4/yaw_servo_unit_check.py`, the bridge gate
    above, and `drone-dtp/scripts/fix1_yaw_replay.py --gate`;
  - SUPER / ROG-Map: `drone-dtp/scripts/target_filter_check.py`, `loss_oracle_check.py`,
    `unhit_decay_check.py`, `park_check.py` and (patch 0008) `visibility_field_check.py`;
  - `command_mux`: `fence_check.py`, `descent_guard_check.py`, `terrain_check.py`,
    `engage_gate_check.py`, `brake_envelope_check.py` and `close_pass_check.py`;
  - `follow_planner`: `drone-dtp/scripts/nmpc_check.py`, `loss_conditioning_check.py`,
    `lost_yaw_hold_check.py` and `close_pass_check.py`;
  - `perception_node`'s position filter (`max_step_m`, `smooth_alpha`):
    `drone-dtp/scripts/max_step_replay_check.py` (no ROS graph, no domain).
- **Check the FC's SD card for crash dumps, not the directory in git:**
  `python3 scripts/px4/clear_fault_logs.py` (read-only).
- **Before a flight day:** `systemctl is-enabled logloader` must say `disabled`, and `df -h /`
  must show ≥ 20 GB free.
- **⚠️ Never read a bag's `.db3` (sqlite3, or any of this repo's analysis scripts) while
  `ros2 bag record` is still writing it.** The read locks the database and the recorder dies
  (`SQLite error (5): database is locked`), with no `metadata.yaml`. It killed the 2026-09-13 M5.1
  bench bag. Recover with `ros2 bag reindex <dir> -s sqlite3`.
- **Trap hit again (2026-09-13):** a `ps | grep "<pattern>"` used to find a PID matched the shell
  running it, and `kill` took that shell down. **Use pidfiles, or match argv exactly in Python.**
- **`pkill -f <pattern>` can kill your own shell** if the pattern appears in its command line.
  The general rule, learned again while building `scripts/gcs/`: **never match a pattern against a
  flattened command line.** A `bash -c '<script>'` carries the whole script as one argv token, so a
  shell that merely *mentions* a node counts as running it — the curl command testing the GCS API
  contained `offboard_viz.py`, and the test shell was reported as a live node. Match the **basename
  of an argv token** (`sysinfo.find_by_cmdline`), and kill by **pgid you own**, never by name.
  **The same trap breaks a wait loop.** On 2026-09-17, `until ! pgrep -f "python3 fetch_ulog.py";
  do sleep 5; done` never ended, because the waiting shell's own argv contains the pattern. It sat
  out its full timeout long after the download had finished. Wait on a pid (`tail --pid=<pid> -f
  /dev/null`) or on an output file instead.

## Git
- **`main` is now the trunk** — merged and pushed 2026-09-02, 23 commits, everything through the
  M2 gate. It is GitHub's default. **Branch off `main`.** `fix/fastlio-executor-latency` and
  `nx-module-bringup` are both fully contained in it and are redundant; the user has not yet said
  to delete them, so leave them.

## Layout
- `super_bringup/` — the ROS 2 package: the M1 launch, `px4_odom_bridge.py` (M2),
  `px4_offboard_bridge.py` (M3), `offboard_viz.py`, and `px4_mpc_tracker.py` (the bridge's MPC
  alternative, not flown). Copied into the workspace, not symlinked.
  → `px4-bridges-offboard.md`
- `config/` — `mid360.yaml` (FAST-LIO extrinsics + resource tuning), `MID360_config.json` (driver net).
- `scripts/` — `setup.sh` (install), `network_setup.sh` (static IP, needs sudo, auto-detects the
  wired iface), `run.sh` (bring-up), `gemini_camera.sh` (the Gemini 336L colour publisher — what
  the GCS's Camera panel starts; `gemini_test.sh` is a one-line shim onto it, kept for
  `docs/geminitest.md`). Laptop viewer: `laptop_setup.sh` + `laptop_viewer.sh` +
  `rviz/m1_viewer.rviz`; the follower view is `laptop_dtp_viewer.sh` (= `DTP=1`) +
  `laptop_image_decode.py` + `rviz/dtp_viewer.rviz` (see `.claude/rules/laptop-rviz.md`).
- `scripts/set_hostname.sh` — the Jetson is **`gypsy`**. The dashboard is at
  `http://gypsy.local:8080` on the field Wi-Fi, or `http://gypsy.tail4e9b1c.ts.net:8080` over
  Tailscale. → `gcs-dashboard.md`
- `scripts/gcs/` — the web GCS, started with `bash scripts/gcs.sh`. It cannot arm, change mode or
  write a parameter. Its RC card can move the aircraft, behind Admin control. → `gcs-*.md`
- `scripts/px4/` — FC tooling over MAVLink UDP 14569. `m2_gate_check.py`, `m3_bridge_check.py`
  and `offboard_engage.py` are ROS 2 instead, so source the workspace first.
  → `px4-link-and-firmware.md`
- `firmware/` — the custom `ark_fpv` PX4 build this FC runs, its patches, the crash dumps and the
  FC parameter dumps (indexed in `firmware/README.md`). → `px4-link-and-firmware.md`
- `drone-dtp/` — the human-following stack (M4–M6). `~/super_ws/src/drone-dtp` links to it.
  → `dtp-workspace.md`
- `patches/` — the FAST-LIO source patches `setup.sh` applies. → `fast-lio-and-lidar.md`
- `rviz/` — the laptop RViz configs. → `laptop-rviz.md`
- `docs/` — notes and durable facts, indexed in `docs/README.md`; some are superseded.
  → `docs-folder.md`
- `.claude/rules/` — the topical rules in the table above.

## Environment conventions
- **Two colcon workspaces:** `~/super_ws` (SUPER + FAST_LIO_ROS2 + super_bringup) and `~/ws_livox`
  (the Livox driver, built with its own `./build.sh humble`). Source order:
  `/opt/ros/humble` → `~/ws_livox/install` → `~/super_ws/install`.
- ROS 2 **Humble**. Build SUPER's ROS version with `bash src/SUPER/scripts/select_ros_version.sh ROS2`.
- Re-cloning SUPER **wipes** in-tree additions, so FAST_LIO_ROS2 and super_bringup live as separate
  `src/` packages and are re-applied by `setup.sh`.
