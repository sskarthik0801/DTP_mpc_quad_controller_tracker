---
paths:
  - "drone-dtp/ros2_ws/src/**"
  - "drone-dtp/scripts/field.sh"
  - "drone-dtp/scripts/deploy_follow.sh"
  - "drone-dtp/scripts/indoor_test.sh"
  - "drone-dtp/scripts/descent_guard_check.py"
  - "drone-dtp/scripts/terrain_check.py"
  - "drone-dtp/scripts/terrain_replay.py"
  - "docs/fact-terrain-follow.md"
  - "drone-dtp/scripts/fence_check.py"
  - "drone-dtp/scripts/follow_speed_report.py"
  - "drone-dtp/scripts/nmpc_check.py"
  - "drone-dtp/scripts/nmpc_replay.py"
  - "docs/fact-nmpc-follow-goal.md"
  - "drone-dtp/scripts/stage_a_preflight.py"
  - "drone-dtp/scripts/replay_perception.sh"
  - "drone-dtp/scripts/bench_cloud_backend.py"
  - "scripts/run.sh"
  - "scripts/dtp_follower.sh"
  - "docs/dtp-follower-stack.md"
  - "docs/fact-follow-speed-and-descent-guard.md"
  - "docs/fact-geofence-command-mux.md"
  - "docs/m5-m6-plan.md"
---

# The follower's behaviour: perception, target manager, follow planner, command_mux

Scope: how the DTP nodes turn a person into a goal for SUPER, and the settings and guards on that path (speed, follow height, descent guard, geofence, perception). Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

### Changed 2026-10-02 (later) — `perception_node` `smooth_alpha` 0.5: the EMA behind the clamp (user; bench-gated by replay, NOT flown)
- **Why.** With the clamp out of the way (section below) the published position still trailed
  the 2.5 m/s walker by 0.71 m median / 1.46 m peak on the flight's frames: the EMA
  `new = α·measurement + (1 − α)·old` at the library's α 0.25 lags a steadily moving person by
  about (1 − α)/α = 3 frames of their motion (0.3 s at 10 Hz).
- **What.** `smooth_alpha` is a `HumanXZPredictor` kwarg, so `perception_node` declares it
  (default **0.25**, the library's) and passes it through, next to `max_step_m`. `dtp_params.yaml`
  sets **0.5**: 1 frame of lag. `detection_pipeline/` is not edited.
  - **The height follows too.** `_build_target`'s z EMA reads `self._pred.smooth_alpha`, so the
    published z now moves at the same weight (its 0.20 m `MAX_STEP_Z_M` clamp is unchanged).
  - **`Target.velocity` and the CTRV forecast** are differenced from this smoothed history, so a
    running person also gets a fresher speed and lead. The 3.0 m/s speed clip still applies.
- **What it costs.** Noise: a standing person's frame-to-frame jitter passes at
  √(α/(2−α)) of the raw: **0.58× at 0.5 against 0.38× at 0.25** (a 0.10 m raw jitter publishes
  0.058 m instead of 0.038). A one-frame outlier that gets past `accept_range` moves the output
  **α·max_step_m = 1.0 m** instead of 0.5 (and 0.09 as flown). `accept_range` (4σ, floor 0.35 m,
  ≤ 3 frames) is the line of defence and is unchanged. Watch `follow_planner`'s goal deadband and
  `target_manager`'s 0.4 m acquisition motion gate on a seated or standing person: more jitter
  reaches both.
- **Gate: `max_step_replay_check.py` 9/9** (F8 and F9 added).
  - F8, at 0.5: trails the raw by **0.24 m median / 0.77 m max** (0.25: 0.71 / 1.46); F4's final
    range is now 9.40 vs raw 9.40 m (was 8.98).
  - F9: 0.10 m of white jitter on a standing person → 0.058 m published (0.58×, theory 0.58×).
  - F5: the outlier bound is now α·max_step = 1.00 m, back within 0.25 m two frames later.
  - F7: the slew limit is 8.0 m/s at 8 Hz, 10.5 m/s at 10.5 Hz: no longer a constraint.
  - `dtp_perception` unit tests 69 passed.
- **Takes effect on the next follower launch; no rebuild. Undo:** `smooth_alpha: 0.25`, or
  delete the line, and relaunch. The two parameters are independent.

### Changed 2026-10-02 — `perception_node` `max_step_m` 2.0: the published target position can now follow a running person (user; bench-gated by replay, NOT flown)
- **Why.** Flight 2026-10-02 20:03 (`flight-log.md`): the walker jogged away at ~2 m/s, the
  detector boxed them on every frame and the stereo depth under the box read 4 → 9.3 m, but
  the published range sat at 3.6–3.7 m for three seconds, then jumped to 9.4 m when the track
  was dropped and rebuilt, which cost a LOST and an auto re-acquire.
  - **The cause is the frozen predictor's position filter** (`detection_pipeline/detection.py`,
    `process_frame` ~:788–794). For a *confirmed* track it clamps the WORLD-frame move from the
    previous published position to `max_step_m` (`_limit_motion_step`), then EMA-blends with
    `smooth_alpha` 0.25 (`_smooth_xz`). The published position therefore moves at most
    `0.25 × max_step_m` per processed frame: **0.0875 m at the library default 0.35, i.e.
    0.9 m/s at the ~10.5 Hz actually flown** (`detect_rate_hz` 8.0 configured; `/dtp/targets`
    median interval 0.096 s). A brisk walk is 1.4 m/s.
  - The published x advanced at 0.87 m/s, the slew limit, and the aircraft followed that goal at
    the same speed, so the range did not change. Neither the node's `accept_range` gate nor the
    "unranged" hold logged anything on that track; the forecasts stayed 17–19 points.
  - **The jump:** the tracker dropped id 2 for ~1 s (dusk, conf 0.3–0.5), `max_missed_secs`
    0.5 cleared its history, and the next detection went out raw: the clamp is skipped for an
    unconfirmed track and the EMA on an empty history returns the sample.
  - **Not the depth.** Re-running the flown detector (`yolo11s.engine`, imgsz 640, conf 0.25,
    BoT-SORT) on the bag's frames and sampling the torso patch on the recorded depth gives a
    smooth 4.0 → 9.4 m with 5–75 valid pixels per patch.
- **What.** `max_step_m` is a constructor kwarg of `HumanXZPredictor`, so `perception_node` now
  declares a `max_step_m` parameter (default **0.35**, the library's) and passes it through.
  `detection_pipeline/` is not edited. The node logs the resulting slew limit at startup
  ("position filter: max_step_m … m/s at detect_rate_hz …"). `dtp_params.yaml` sets **2.0**.
  - **Why 2.0 and not 1.2:** 1.2 covers a steady 3 m/s, but at dusk the far walker is boxed only
    every second or third frame, and across a 0.2–0.3 s gap the raw step is up to 1.5 m. On the
    flight's frames 1.2 still clipped those (0.64 m off the clamp-free path); 2.0 clips nothing.
  - **What the clamp still does:** a one-frame outlier moves the output by at most
    0.25 × 2.0 = 0.50 m (was 0.09). The first line of defence against a bad patch is the node's
    velocity-aware range-jump gate (`accept_range`, 4σ, floor 0.35 m, holds ≤ 3 frames), which
    is unchanged.
  - **What remains: the EMA.** `smooth_alpha` 0.25 is a ~3-frame (~0.3 s) time constant, and on
    the flight's frames the clamp-free filter still trails a 2.5 m/s walker by 0.71 m median,
    1.46 m peak. At α 0.5 that would be 0.24 / 0.77 m. `smooth_alpha` is also a kwarg and could
    be exposed the same way; it was NOT changed (noisier position, and the node's height EMA
    reads the same alpha).
- **Gate: `python3 drone-dtp/scripts/max_step_replay_check.py` — 7/7.** No ROS graph. It
  re-execs itself with the pip CUDA libs on `LD_LIBRARY_PATH` (importing `detection.py` imports
  torch). The fixture `max_step_fixture_20261002.json` (37 frames, bag t 42.8–47.0) was made with
  `--regenerate`: the flown detector on the bag's colour frames, the torso patch on the recorded
  depth, lifted through the odometry (camera ≈ body frame; fine for a filter test). The gate feeds
  it through the REAL `_limit_motion_step` / `_smooth_xz` on a model-less predictor instance.
  - F1/F2, at 0.35: range 3.53–4.01 m while the raw opened 5.4 m; ends 5.8 m short. (The flight.)
  - F3, at 2.0: identical to the clamp-free filter (0.00 m) — the clamp no longer binds.
  - F4, at 2.0: the range opens 5.0 of the raw 5.4 m; final 8.98 vs 9.40 m.
  - F5: a 4 m one-frame outlier moves the output 0.50 m and is 0.28 m off two frames later.
  - F6/F7: the node declares and passes it; 2.0 gives 4.0 m/s at 8 Hz, 5.25 m/s at 10.5 Hz.
  - `dtp_perception` unit tests 69 passed (flake8 deselected: it was already red, 35 errors, and
    the two new files are clean at 99 columns).
- **Takes effect on the next follower launch** (develop install + yaml symlink; no rebuild).
  **Undo:** `max_step_m: 0.35`, or delete the line, and relaunch.
- **Downstream, unverified in flight:** `Target.velocity` and the CTRV forecast come from the
  same history, so a running walker now also gets a realistic speed and lead; `target_manager`'s
  re-acquisition band and `command_mux`'s braking envelope see the true closing speed. The
  far-side goal problem is unrelated and still open.

### Ported 2026-10-01 — onto the rollback: the braking envelope and the 3.0 m veto height band (user; bench-gated, NOT flown)
- Branch `rollback-pre-0926`. `brake_envelope_enable: true` and `proximity_veto_half_height_m: 3.0`
  in `dtp_params.yaml`. `brake_envelope.py`, `descent_guard.py`'s `extra_radius_m`, their tests,
  `brake_envelope_check.py` and `descent_guard_check.py --mux-param` are the checkpoint's.
  - The envelope code in `command_mux_node.py` was merged by hand on top of the uncommitted park
    work (0009).
  - `follower.launch.py` passes `super_max_acc_mps2` to the mux only.
- **Not ported:** follow_planner's speed-scaled standoff (`speed_standoff.hpp`), the pass-by radius
  (`passby_reach_enable`), the flyable HOLD (`hold_stop_projection_enable`), the pass-by check
  (`transit_passby_check_enable`), and the `stage_a_preflight` envelope readout.
  - With no standoff, the descent guard's release radius can exceed the follow band in a sprint.
    That only keeps the floor latched longer.
- **Gates:** `brake_envelope_check` 13/13; `descent_guard_check` 13/13 with the envelope off and
  on; `engage_gate_check` 14/14, `fence_check` 13/13, `terrain_check` 10/10, `park_check` 14/14.
- **`speed_track_s` stays 0.4** (no-feedforward overshoot), although the feedforward is ON on this
  branch: it is conservative until the feedforward has flown.

### Added 2026-09-30 — three fixes for the 09-29 close pass (user decision; all ON in `dtp_params.yaml`, bench-gated, NOT flown)
The event and its three causes are in `flight-log.md` (2026-09-29 12:32). All three switches
default to **false** in the nodes, so a launch without `dtp_params.yaml` behaves as flown.
- **Fix 1, `follow_planner`: `transit_passby_check_enable`** (`TransitPassesPerson`).
  - **The rule.** The path to the goal "passes BY" the walker when the walker's projection on it
    lies after the lead-in and strictly before the goal. Such a path must clear `FloorEff()`
    (2.5 m + the speed delta), not 2.0 m. It is tested against the walker's position now **and
    every point of their forecast**.
  - **Why it can use the floor.** The old guard cannot: a legal goal sits at the floor and would
    trip it on its own endpoint. A walker at or beyond the goal is not "passed by", so the
    endpoint never counts here.
  - **On failure** it holds with "transit to the standoff would pass BY the person inside the
    floor (pass-by check)". A `pass-by check:` WARN gives the distance and the forecast time.
- **Fix 2, `command_mux`: `passby_reach_enable`.** For a person the path passes by, the veto
  radius is `reach(closing, ground speed)` = 1.5 m + the stopping distance + the overshoot. It
  is no longer a fixed 1.5 m.
  - Per person, at their own closing speed.
  - A person at or beyond the goal keeps 1.5 m, so a sprint *to* a standing walker is unchanged.
  - The reason says "which passes by them (pass-by radius X m …)".
- **Fix 3, `command_mux`: `hold_stop_projection_enable`.** A HOLD entered at ≥ 0.25 m/s is
  latched `stop(v)` ahead along the velocity (react 0.5 s + 2.0 m/s²), not at the aircraft.
  That is 0.75 / 2.0 / 3.75 m at 1 / 2 / 3 m/s.
  - The re-anchor distance grows by the same amount.
  - `HOLD (…) at X m/s: latched Y m ahead` WARN.
  - Below 0.25 m/s, and when off, the hold is exactly as before.
- **Fixes 2 and 3 need `brake_envelope_enable`** (the velocity and closing speeds come from it).
  Either one without it refuses to start.
- **Gate: `python3 drone-dtp/scripts/close_pass_check.py`, domain 95, 17/17 (2026-09-30).**
  Real `follow_planner_node` and `command_mux_node`, each switch off and on.
  - **P4 and M6 replay the event itself** from `close_pass_fixture_20260929.json`, made with
    `close_pass_extract.py` from bag t 452.8–456.8, so the gate needs no bag.
  - **P4 (the planner, open-loop exact while the aircraft was at rest).** Off: the far-side
    goal (16.71, 1.59) at t 453.48, as flown. **On: no goal.** The forecast at +1.4–1.5 s passed
    at 2.32–2.35 m, under the 2.5 m floor. **The margin is thin (0.15 m):** a walker slightly
    further off the line would not have tripped it. Fix 2 is the second layer.
  - **M6 (the mux, goals as flown).** Off: veto at t 456.21 (flown: 456.20). **On: t 454.63**,
    1.6 s earlier, while the aircraft was still slow: 2.17 m from the path against a pass-by
    radius of 2.31 m.
  - **Synthetic cases.**
    - A hovering aircraft, a walker approaching 0.5 m off the line: off vetoes at 6.81 m, on at
      9.28 m.
    - The hold at 2.5 m/s: 2.61 m ahead (2.81 m minus odom lag), with no re-anchor while braking.
    - No new holds or vetoes in walk-away, a 90° turn, walk–pause–walk, 1 m/s behind a walker,
      or a walker passing 3 m to the side. The goal counts are equal off and on.
- **The existing gates, re-run one at a time on the final build (2026-09-30 02:04–02:14), all
  green.** With the switches at the node defaults (off), these test that nothing else moved.
  - `follow_planner` (rebuilt): `nmpc_check` 52/52 (N1: geometric goals bit-identical),
    `loss_conditioning_check` 18/18, `lost_yaw_hold_check` 26/26.
  - `command_mux`: `fence_check` 13/13, `descent_guard_check` 13/13, `terrain_check` 10/10,
    `engage_gate_check` 9/9, `brake_envelope_check` 13/13.
  - The mux's unit tests and flake8: 125 passed.
  - The pre-change binary is kept at
    `/tmp/claude-1000/.../scratchpad/follow_planner_node.pre-passby`. It will not survive a
    reboot, so undo with the switches, not the binary.
- **The veto's height test** was ±2.0 m around a low tracked point, so an aircraft ≥ 2.0 m above
  it was skipped (the braking envelope missed the 09-29 walker for this). **Fixed 2026-09-30 as
  Part A of the height plan: see the section below.**
- **Not fixed:** SUPER's replans were already failing 1.2 s before the 09-29 HOLD (from t
  455.05, 32 failures), and the cause is not established. While replans fail, SUPER ignores
  every new goal, a flyable HOLD included.
- **Undo, without a rebuild:** set the three switches false in `dtp_params.yaml` and relaunch
  the follower.
- **Found on the way:** `nmpc_check` N13 release in the **nmpc** arm is flaky at its 3.90 m
  limit. The pre-change binary fails it too (3.95 m), and the new one passed on a re-run
  (3.70 m). The flown arm is geometric.

### Added 2026-09-30 — the proximity veto's height band: `proximity_veto_half_height_m` 3.0 (Part A of the veto-height plan; bench-gated, NOT flown)
The plan is `docs/plan-veto-height-and-lidar-range.md`, Part A, option A1 (user-planned). It
closes the height hole named in the close-pass section above.
- **The change: one parameter, no code.** `proximity_veto_half_height_m: 3.0` under
  `command_mux:` in `dtp_params.yaml` (~:363).
  - The node default `DEFAULT_PROXIMITY_HALF_HEIGHT_M` stays **2.0**, as flown. Only its
    comment changed: it said the tracked point is the body centroid, which is wrong (below).
  - The install's `dtp_params.yaml` is a symlink to the source file. **Relaunch the follower to
    pick it up; no rebuild.**
- **Why.** `command_mux` treats each person as a cylinder ±half-height around the tracked point,
  in both the braking envelope and the path check.
  - **In flight the tracked point is the lower body**, not the centroid and not exactly the
    feet. `perception_node`'s `_build_target` (~:1642–1700) takes the near-surface median of a
    torso patch centred 65 % of the way down the box (`PATCH_CENTRE_H_FRAC = 0.65`, ~:225).
    That is hip/thigh height, and it matches the z 0.26–0.72 seen in `lidar_init` on 09-29.
  - Smoothing clamps z steps to 0.20 m (`MAX_STEP_Z_M`). Unranged frames hold the last z.
  - **The fallback for a never-ranged track** (~:1719–1735) publishes the camera's own altitude
    (2.39 m on 09-29), and the first ranged frame replaces it. It is conservative for the veto:
    the aircraft is then always inside the band.
  - On 09-29 the aircraft flew at z 2.27–2.73 against a tracked z of 0.26–0.72. The 2.0 m
    cylinder's top was at the aircraft's height, and the braking envelope skipped the walker
    from t 454.44 to 456.54.
  - At 3.0 the cylinder reaches ~3 m above the ground: head (~1.8 m) plus ~1.2 m of prop
    clearance. The extra 1 m below is harmless.
- **Gate: `close_pass_check.py` (domain 95), now 23/23 (was 17/17).** Passed on two runs,
  ~4.4 min each (M9–M13 add ~96 s). `mux_synthetic` was split into scene functions. M1–M8 are
  unchanged and matched. Each 3.0 case checks that the node's startup log says "+/-3.00 m".
  - **M9 (2.0):** the aircraft and the far-side goal at z 2.7, the walker approaching, the
    pass-by radius on. **Never vetoed, down to 4.50 m.** It reproduces the hole.
  - **M10 (3.0), the same scene:** pass-by veto at 9.28 m.
  - **M11 (3.0), the braking envelope:** the aircraft at z 2.7, 2.5 m/s at a standing walker,
    the goal parked 3 m short. Envelope veto at 5.09 / 5.30 m (formula 5.31).
    - **At 2.0 nothing fired at all, down to 4.0 m.** The path check cannot fire in that scene
      either: the path plus the overshoot ends 2.0 m short.
  - **M12 (3.0), the 09-29 fixture replayed:** the first PROXIMITY at t 454.62–454.63, from the
    pass-by **path check**. That is the same frame as at 2.0 (M6 on: 454.63). Tolerance
    +0.05 s (half a targets frame).
  - **M13 (3.0), no new holds:** 1 m/s behind a walker going away, and a walker passing 3 m to
    the side, with the aircraft 1.0 and 2.0 m above the point (z 1.2, 2.2): 0/0 and 0/0.
- **⚠️ Not what the plan expected: in the replay the braking envelope does not fire first.**
  The plan had it seeing the walker from ~454.4. The walker is inside the band now, but the
  aircraft was still slow (0.1–0.25 m/s at 454.4–454.9), and the walker was > 7 m from the
  braking path. They first come under 1.5 m of it at ~456.09, at 2.05 m/s.
  - So at 3.0 the envelope is a second, later layer, from ~456.1. At 2.0 it was blind from
    ~454.8 to 456.5.
  - The first veto on this event still comes from fix 2 (the pass-by radius), at 454.63.
- **The existing `command_mux` gates, re-run one at a time.** They run the node default (2.0),
  so they test only that nothing else moved.
  - All green on the edited node (the build tree symlinks to the source), 2026-09-30:
    `fence_check` 13/13, `descent_guard_check` 13/13, `terrain_check` 10/10,
    `engage_gate_check` 9/9, `brake_envelope_check` 13/13. None of the five loads
    `dtp_params.yaml` or sets the half-height, so all five ran 2.0; only the new M9–M13 cases
    test the 3.0.
  - The mux's unit tests and flake8: 125 passed (unchanged).
- **Replayed on every flight bag (2026-09-30, `drone-dtp/scripts/veto_height_bag_scan.py`): 3.0
  adds no new and no earlier `HOLD|PROXIMITY` anywhere.** Stage `scan` finds every second where
  the 2.0 and 3.0 bands disagree, offline; stage `replay` runs those windows (plus each whole
  Offboard span) through the real node at both bands, domain 95, deployed switches on. 320 s of
  bag time per band, 11.6 min.
  - Flights that went fine (09-21 18:41 and 20:05, 09-26 19:57): identical veto times at both
    bands (jitter ≤ 0.02 s). 09-29 12:32: identical, first veto 454.62 (pass-by radius).
  - What 3.0 buys is coverage. Offboard person time the 2.0 braking envelope ignored: 35.5 s at
    18:41 (aircraft 2.0–2.7 m above a downhill tracked point, walker at 3.3–6.3 m), 14.6 s at
    20:05, 13.5 s on 09-26 20:12, 5.0 s at 18:30, 2.9 s on 09-29. It never fired first there:
    the closest approach to a braking path in those seconds was 2.01 m (18:41, t 402–404).
  - **One difference: 09-26 20:12, a veto released 0.56 s later** (360.58 → 361.14). The HOLD's
    own climb put the aircraft 2.05–2.37 m above the point, and at 2.0 the veto released on
    height alone while the path still passed 1.38 m from the forecast. The commanded goal did
    not change (both went to HOLD|NO_SOURCE with the same hold point).
  - Found on the way, not changed: while the veto is latched, the release is decided on the
    descent-guard-raised segment only (`_proximity_veto` ~:1786); the requested goal is checked
    again only at the end of the 1 s dwell. And the deployed switches produce vetoes the flown
    record never had, identical at both bands: the envelope at 363.95 in the 09-26 19:57 engage
    sprint, the pass-by radius at 533.66 / 541.91 (20:05) and 440.88 (09-29).
  - Limits: open loop (the aircraft moves as flown); goals rebuilt from `/mux/state`, so a goal
    vetoed on its first tick (09-29 463.72) is not reproduced; PX4 reads not-Offboard, so the
    replay's descent guard latches on HOLD climbs. `close_pass_extract.py` now takes `T0 T1`.
- **The Stage B preflight prints it.** `stage_a_preflight.py`'s "close-pass fixes" warn line
  now adds "veto half-height X.X m", read from `/command_mux` ("not readable" if absent), so a
  bag records what flew.
- **Undo, without a rebuild:** remove the line, or set it to 2.0, in `dtp_params.yaml`, and
  relaunch the follower.

### Added 2026-09-30 — Part B: LiDAR ranging past 10 m, a hybrid range source (camera near, LiDAR far; `lidar_range_enable`; ON in `dtp_params.yaml`; bench-gated, replay-scored below, NOT flown)
The plan is `docs/plan-veto-height-and-lidar-range.md`, Part B, Steps 0–3. The scripts and the
full tables are in `docs/evidence/lidar-range-2026-09-30/README.md` (Step 0, Step 1). All of it
is offline or on the bench, on the bags of the five person-following flights (09-21 18:41 and
20:05, 09-26 19:57 and 20:12, 09-29 12:32). **Nothing here has flown.**

**Step 0 — the detector, GO in daylight** (`detector_range_study.py`; range truth from the LiDAR).
- The flown detector, re-run on the recorded frames: `yolo11s.engine`, imgsz 640, conf 0.25,
  8 Hz, compressed input, `model.track` (BoT-SORT).
- **Daylight, 09-29 (the only day bag with the walker past 8 m):** 100 % of in-FOV frames at
  8–17 m (conf mean 0.73–0.80, box 62–119 px tall at full resolution); 95 % at 17–20 m; 79 % at
  20–30 m. Through BoT-SORT 98–100 % to 17 m, then ~50 %. **GO.**
- **Dusk, 09-26 20:12: it gives out.** 62 % at 8–10 m, 5 % at 10–12 m, 0 % at 12–15 m. The
  boxes there are bigger than the daylight boxes detected at 100 %, so the limit is light and
  blur, not pixels. **LiDAR ranging gains nothing at dusk.** (Pooled with 09-21 20:05 the
  README gives 5–8 % and 0–41 %.)
- The other bags have no walker span at 8–17 m (max walker range 6.4–8 m).
- **Surprise: the flown camera range read 1–4 m SHORT past ~8 m.** Median −1.2 m at 8–10 m, off
  by > 1 m in 59 % of frames. At 09-29 t 441–443 it said 5–6 m for a walker jogging at 9–14 m.
  The cause is the `max_depth_m` mask in `_sample_patch`: it left only near clutter. Confirmed
  in the code and fixed in the build (the cap bug, below).
- **Nose-up pitch blinds the MID360** (its lowest ray is −7°): 09-29 t 501.5–504, flying back
  at up to 6.7 m/s.
- **Auto re-acquire locked the pilot at the landing spot,** 09-29 t 509–512 (track 11), inside
  10 m. That is the D13 case below.

**Step 1 — the measurements behind the design** (`stereo_lidar_bias.py`, `lidar_box_points.py`,
`cpu_cost.py`, `overlay_check.py`; all five bags).
- **Camera − LiDAR horizontal range, pooled:** −0.07 m at 5–6 m, +0.07 at 6–7, −0.07 at 7–8,
  +0.38 at 8–9, +0.73 at 9–10 (few frames).
  - On walkers the camera **lags** 0.5–1 m: the EMA (α 0.25 at 8 Hz) plus the depth/colour
    skew, fitted τ 0.3–1.0 s.
  - It is > 1 m wrong in 2.5 % of frames at 5–7 m and 12 % at 7–10 m.
  - These rows and Step 0's −1.2 m are different frame sets. Here the truth is searched near
    the camera's own fix (the standing-person disc drops camera errors > ~0.8 m; the walker
    truth looks ±4 m of the camera range). Step 0's tracker trusts only the camera's bearing.
- **LiDAR points on the walker, per window** (dynamic points, perfect motion compensation):

  | range | 0.4 s, p50 points | 0.4 s, ≥ 3 points | 0.6 s, ≥ 3 points |
  |---|---|---|---|
  | 8–10 m | 23 | 75 % | 79 % |
  | 10–12 m | 19 | 86 % | 88 % |
  | 12–15 m | 12 | 91 % | 96 % |
  | 15–17 m | 5 | 75 % | 79 % |

  - 100 % whenever the body is above the −7° ray, so nearly every miss is attitude.
  - The walker's motion in the window: p90 1.04 m in 0.4 s, 1.56 m in 0.6 s, max 2.35 m,
    almost all along the line of sight.
- **Background in a whole-body box: mostly ground.**
  - With a perfect calibration the nearest-surface rule with ≥ 2 points is within 0.4 m of truth
    in 85–93 % of windows at 8–15 m, and FAR (too long, the dangerous side) in 0–2 %.
  - The first point behind the walker is p50 1.3–1.4 m behind. The metal fence gave little
    trouble (0.3 %).
- **⚠️ The camera and the LiDAR disagree by ~3° in pitch (2.3–3.5°),** in every bag, including
  on the ground with motors off. The LiDAR points project 20–45 px too high, and the
  camera-lifted hip height falls 0.041–0.060 m per metre of range. The plan's "~1° or less" is
  wrong. → `dtp-calibration.md`
  - With that error and no ground rejection the box rule is OK in only 24–53 % of windows.
    Dropping points < 0.25 m above the local ground restores 79–81 %.
- **The frame chain.** `/cloud_registered_body` is in `lidar_imu`: p_body = p_lidar + t_LI,
  |t_LI| **5.1 cm** (the plan said 4.8).
  - Composed T_camera_body (3×4):
    ```
    [[-0.026158 -0.999585  0.012078 -0.014770]
     [-0.044392 -0.010908 -0.998955  0.070500]
     [ 0.998672 -0.026666 -0.044088 -0.080650]]
    ```
  - The camera's optical centre in `lidar_imu` is (0.083, −0.016, 0.067) m; its axis points
    2.5° down and 1.5° right.
  - The old `LidarDepthSource` disabled itself on |extrinsic_T| = 0.051 m, and projected at the
    sweep's stamp (up to 0.15 s off the image).
  - FAST-LIO's `extrinsic_est_en` is true, so its online estimate can drift. The user's call.
- **CPU:** 19.4 k points per 0.5 s window (3.9 k per scan, not the plan's 7 k); 0.74 ms per
  frame in pure numpy.

**What was built (Steps 1 + 2), all in `dtp_perception`.** Unit tests **97 passed, 12
GPU-skipped**; 40 of them new: `test_lidar_box_range.py` (17), `test_perception_hybrid.py`
(15), `test_target_manager_auto_reacquire.py` (8). ament flake8 36 → 35 errors.
- **New `lidar_box_range.py`, `LidarBoxRange.range_for_box`.** Its module docstring holds the
  reasoning.
  - A 0.6 s ring of scans in `lidar_init`; odom slerped to the colour stamp.
  - Camera←body composed from `lidar_to_body_t/r` and `T_camera_lidar`
    (`depth_source.compose_camera_body`, the inverse of `static_calib_tf`'s own function). It is
    checked against FAST-LIO's live extrinsic (`FastlioExtrinsicCheck`), which replaces the
    "must be identity" check. **So `depth_source: lidar` can run on this aircraft for the first
    time.**
  - A ±55° cull before `cv2.projectPoints`: plumb_bob folds a 65° point to u = 1216.
  - The whole box, shrunk 10 %. A ±3 m gate on the range filter's predicted range.
  - Ground rejection: points < 0.25 m above the 10th-percentile height, in 2 m bins along a
    ±2 m strip on the bearing, with the box cut out. A bin needs ≥ 6 points spread ≥ 0.8 m.
  - Older scans shifted by the track's velocity: 0.6 s shifted when the velocity has history,
    else 0.4 s unshifted.
  - Out: the nearest-surface cluster median, the point count, and an attitude-blind flag (the
    aircraft's pitch against the −7° ray). `select_range_source` holds the switching rule.
  - Synthetic walker at 12 m (truth 11.94): a fence 2 m behind → 11.90; the box slid 3° →
    10.52 (ground) without rejection, 11.95 with; walking at 2 m/s → 11.50 unshifted, 11.94
    shifted.
- **`perception_node.py`: the hybrid, behind `lidar_range_enable`.**
  - **The LiDAR range enters before the EMA,** by wrapping the predictor's
    `_extract_xz_from_pointcloud` on the instance. `detection.py` is not edited. Startup refuses
    if that method's signature changed. A runtime check turns the hybrid off, and clamps the
    caps, if the wrapper is ever bypassed.
  - To the LiDAR beyond 8.0 m with ≥ 3 points; stay with ≥ 2; back to the camera below 6.5 m.
    `accept_range` is source-aware, reset at a switch, with a LiDAR sigma of 0.10 m.
  - The bearing stays the camera's. Only the range changes source.
  - When the LiDAR is attitude-blind, in the far regime only: a 1.0 s hold on the filter's
    rate-clamped prediction (6.5 m/s) along the live bearing, published with an empty forecast.
  - `camera_max_depth_m` 10.0 is how far the stereo is believed, now that `max_depth_m` is 15.
  - Out: `/dtp/range_source` (per-frame JSON), a 1 Hz per-track log line, and a 5 s check of
    its camera pose against the TF lift.
  - CPU: 4.0 ms for the first box + 2.5 ms per further box, ~5 % of a core at 8 Hz.
  - **With the switch off the node is byte-identical to before.** A test compares
    `_sample_patch` on 60 random clouds.
- **Also changed:**
  - `lidar_to_body_t/r` on `perception_node` (yaml [−0.011, −0.02329, 0.04412], identity), a
    third copy of `static_calib_tf`'s and FAST-LIO's;
  - `depth_source.py`'s two stale docstring facts fixed; `static_calib_tf.py`'s docstring;
  - `/dtp/range_source` in `scripts/gcs/bagger.py`'s `stage_b` preset;
  - `stage_a_preflight.py` prints the switch and the caps, and FAILS if the three caps differ
    while it is on.
- **Marginal geometry:** level at 1.5 m, a walker's feet at 12 m sit on the −7° floor. At 12 m
  the margin is ~4° of nose-up pitch.

**The parameters** (`dtp_params.yaml` sets only the rows with a yaml value):

| parameter | node | node default | `dtp_params.yaml` |
|---|---|---|---|
| `lidar_range_enable` | perception_node | false (as flown) | **true** |
| `max_depth_m` | perception_node | 10.0 | **15.0** |
| `prediction_depth_gate_m` | perception_node | 10.0 | **15.0** |
| `acquire_max_range_m` | target_manager | 10.0 | **15.0** |
| `auto_reacquire_max_range_m` (new) | target_manager | 0.0 (= the acquire cap) | **10.0** |
| `lidar_to_body_t` / `lidar_to_body_r` (new) | perception_node | [0, 0, 0] / identity | [−0.011, −0.02329, 0.04412] / identity |
| `camera_max_depth_m` (new) | perception_node | 10.0 | — |
| `lidar_switch_out_m` / `lidar_switch_back_m` (new) | perception_node | 8.0 / 6.5 | — |
| `lidar_min_points_switch` / `lidar_min_points_stay` (new) | perception_node | 3 / 2 | — |
| `lidar_window_s` / `lidar_window_unshifted_s` (new) | perception_node | 0.6 / 0.4 | — |
| `lidar_gate_m` (new) | perception_node | 3.0 | — |
| `lidar_ground_reject_m` (new) | perception_node | 0.25 | — |
| `lidar_range_sigma_m` (new) | perception_node | 0.10 | — |
| `lidar_hold_s` (new) | perception_node | 1.0 | — |
| `range_source_log_period_s` (new) | perception_node | 1.0 | — |

- **The three caps move together, and only with the switch.** With `lidar_range_enable` false,
  `perception_node` clamps its two caps to `camera_max_depth_m` and logs an ERROR.
  `target_manager` cannot see the switch; its docstring names its cap as the third of the set.

**The cap bug — found in Step 0, fixed in the hybrid only** (`_nearest_surface_capped`).
- **The flown `_sample_patch` masks every sample beyond `max_depth_m` BEFORE picking the near
  surface.** For a person near or past the cap, that removes their own far samples and leaves
  near clutter, which then *is* the near surface: a confident range 1–4 m short.
- **The fix,** on the unmasked samples: if more than half the valid samples are beyond the
  camera cap, the patch returns no range. Otherwise the near surface is chosen from all samples
  and the cap is applied to the cluster's median.
  - A person at 11 m with 25 % clutter at 6 m: flown 6.0 m; fixed, no range.
- **Also found: far people were ABSENT from `/dtp/targets`, not unranged.** `process_frame`
  drops a detection with no sample under the cap.

**D13 — the user's call.**
- `auto_reacquire_after_lost` is true (user, 2026-09-15). After a LOST it takes the nearest
  confirmed person in the acquisition band, with nobody choosing. At 15 m that band holds more
  bystanders. On 09-29 it locked the pilot even inside 10 m.
- **New `auto_reacquire_max_range_m`** (yaml 10.0). It is used only when the acquisition was
  started by auto re-acquire after a LOST. An operator's engage and a coast re-lock ignore it.
  So an operator's engage reaches 15 m, an automatic re-acquisition 10 m.

**Relaunch, and undo.**
- The install's `dtp_params.yaml` is a symlink to the source file, so **the next follower launch
  runs the hybrid.** No rebuild.
- **Undo, without a rebuild:** `lidar_range_enable: false` and the three caps back to 10.0 in
  `dtp_params.yaml`, then relaunch.

- **Step 3 replay scoring:**
  - **Setup (2026-09-30):** `replay_perception.sh` now plays `/cloud_registered_body` and takes
    `START_S`/`DUR_S`/`NODE_ARGS`/`PROBE`/`TM`; `record_targets.py` records `/dtp/range_source`;
    new `replay_bag_helper.py`, `replay_perception_probe.py` (records every range-gate verdict
    without editing the package) and `range_source_score.py` (the scorer; the denominator is the
    LiDAR-cluster walker track from `detector_range_study.py`, not `flight_timeline`'s). Domain 87,
    WARM_S=60 RATE=0.25, one replay at a time. A windowed replay does not reproduce the
    decimator's phase: frames are matched to the nearest stamp within 40 ms. OFF vs OFF p95
    0.005 m; final ON vs ON shared all 769 stamps, p95 0.02 m, ranged share within 0.9 points.
  - **Round 1 (the build as first written) failed the 90 % bar:** ranged share at 10–12 m
    71–78 % ON vs 17 % OFF, 12–15 m 62–63 % vs 0; but the *published* position lagged the
    LiDAR by metres (5.5 m short at 12–15 m): the frozen `detection.py` clamps each step to
    0.35 m and EMAs at α 0.25, ~0.9 m/s at 10 Hz. And 59–72 % of the walker's own points
    projected ABOVE the shrunk box (the 3° pitch error, `box_projection_check.py`). Also: the
    LiDAR read 1.0–1.4 m short on a 3.3 m/s sprinter (the shift velocity came from the clamped
    history); a flip at the switch-out range (463.54); a bad camera range taken while
    attitude-blind (465.54); dusk cycling, 10 switches at 6.0–6.7 m.
  - **Round 2 (final build), seven fixes, all behind the same switch;** unit tests 109 passed
    + 12 GPU-skipped (+12 new), `test_flake8` red as before (35 errors, was 36), bit-identical
    with the switch off:
    - the wrapper sets α 0.5 / clamp 1.0 m for a LiDAR or hold sample (`lidar_smooth_alpha`,
      `lidar_max_step_m`) and restores the frozen values after each call; at a camera→LiDAR
      switch, or when the LiDAR is > 1.0 m from the smoothed track, it re-seeds by translating
      the track's whole `tracks_xz` history and cached forecast (no velocity spike).
      `detection.py` is not edited;
    - `lidar_box_top_extend_px` 45 (yaml; default 0) raises the top of the LiDAR mask only: the
      walker's points inside the box 28–38 % → 86–90 % at 8–15 m. **A workaround for the
      pitch calibration, not a fix;** (the pitch was re-calibrated on 2026-09-30, so this now
      lifts the mask 45 px above a correctly projected box. Re-score, and probably set it to 0:
      `dtp-calibration.md`.)
    - the window shift uses the track velocity only if fresh (≤ 1 s old, no gap > 0.3 s in the
      last 7 entries, none older than the last hold); above 3 m/s the read went −0.95 → −0.03 m;
    - the switch-out is decided on the LiDAR's own range (≥ 3 points); 2 empty frames of grace
      (`lidar_stay_grace_frames`) before switching back; a 1.5 s dwell (`lidar_min_dwell_s`);
    - attitude-blind with no LiDAR: hold rather than take a camera range beyond 8 m;
    - `main()` tolerates the rclpy teardown `RCLError` when the context is already shut down.
  - **Final scores, 09-29 t 435–512 (ON ×2 / OFF ×2):**

    | | 8–10 m | 10–12 m | 12–15 m | 15–17 m |
    |---|---|---|---|---|
    | ranged, ON | 80 / 79 % | 86 / 86 % | 78 / 78 % | 0 % (`max_depth_m` 15) |
    | ranged, OFF | 94 % | 17 % | 0 % | 0 % |
    | raw LiDAR within 0.3 m | 89 % | 83 % | 78 % | — |
    | published \|error\| median, ON | 0.18 m | 0.12 m | 0.14 m | — |

    Below 2 m/s the raw LiDAR is within 0.3 m in 88 % of frames (+0.01 m), 2–3 m/s 82 %, above
    3 m/s 55 % (−0.03 m). Switches 6 out / 3 back, 2 reversals; holds 11, max 0.8 s; re-seeds
    10; LOST 3 (OFF 6); gate rejections 0 at 6–10 m, 5/175 past 10 m; gross LiDAR errors 0
    long, 1 short. CPU 100–108 CPU-s vs 90 OFF; frame time 54 vs 49 ms; cloud 13 ms both.
    `flight_timeline` §5 ">10 m" for continuity with the 23 %: as flown 17 %, ON 74–83 %.
  - **Close range (B): ON = OFF within noise** (18:41 p95 0.012 m; 09-26 19:57 0.022 m). 20:05
    improved: at 520.2–520.8 the camera had no depth and the LiDAR ranged the walker at 6.2 m
    (60–80 points, within 0.1 m); 6–8 m ranged 100 % vs 84 %, published error 0.04 m.
  - **Dusk (C, 09-26 20:06):** the detector gives out at 10–12 m (1 of 39 frames) so nothing to
    range there; at 8–10 m the published position is within 0.3 m in 96 % (OFF 19 %); 4
    switches, 0 reversals, no invented range, LOST the same as OFF.
  - **Pass criteria:** past 10 m well above OFF **PASS** (82 % vs 9 %); ≥ 90 % at 10–15 m
    **FAIL at 82 %** — every miss is a frame where the LiDAR has nothing (23 holds, 5
    unranged) plus 9 undetected at 12–15 m; where the LiDAR can see the walker 94 % are ranged
    at 10–12 m and 84 % at 12–15 m, so the limit is the MID360's −7° floor under nose-up
    pitch; no rise in rejected jumps at 6–10 m **PASS**; within 0.3 m in most frames **PASS**;
    no fence/tree range **PASS** (all gross errors short); B/C **PASS**.
  - **Still wrong:** a hold extrapolates the range filter's rate, noisy at σ 0.1 m (469.3–470.1
    ended 4.5 m short, 451.6–451.9 1.8 m long; the next LiDAR frame re-seeds). For ~0.6 s
    after a switch into a sprint, and ~0.7 s after a hold, reads are 0.4–1.0 m short (the safe
    side). `Target.velocity` still comes from the whole history. **The pilot lock at 508.96
    (4.2 m) is unchanged in every run: D13, the user's call.** 45 px is a calibration
    workaround.

### Measured 2026-09-30 14:22 — first LIVE run of Part B: indoor walk, observe-only, aircraft on the ground (plan Step 4, NOT complete)
- **Setup.** Brought up with `deploy_follow.sh --observe`, indoors in the lab. The script died
  at its `sensors_ok` check (below), so the FAST-LIO gates, the recorder and the preflight were
  run by hand. Bag: `~/dtp_datasets/20260930_1422_stageA_range` (9.7 min, 8.0 GB; the stage_b
  preset plus `/dtp/range_source`). Logs: `~/dtp_datasets/20260930_1422_deploy/`.
- **The >10 m question was NOT answered: the room is too short.** The LiDAR puts the far wall
  at ~9–10 m across the camera's view (the doorway reaches 14–20 m). The furthest range was
  **9.27 m (LiDAR)**; the camera's furthest raw reading was 10.05 m. The one gap with no
  detections (t 302–550 s) is the walker **out of sight through the far door**, confirmed on
  the colour frames, not a detector miss. **Repeat outdoors in daylight with ≥ 20 m clear.**
- **The handover works live.** Every walk past 8.0 m switched to the LiDAR with 18–74 points,
  and every return switched back below 6.5 m. There were no reversals past 8 m.

  | bin | ranged by | LiDAR pts (median) | camera − LiDAR, median / \|·\| p90 |
  |---|---|---|---|
  | 6–8 m | camera 117, LiDAR 64 | 43 | −0.11 / 2.16 m |
  | 8–9 m | LiDAR 47/47 | 29 | +0.13 / 0.59 m |
  | 9–10 m | LiDAR 36/36 | 26 | −0.26 / 0.60 m |
- **Seen, not yet explained:**
  - 6 switches to the LiDAR at 4–6 m on "camera range rejected as a jump". In one, the camera
    read 1.96 m against the LiDAR's 7.32 m: probably near clutter in the box.
  - **Track-ID churn:** 10 IDs for one walker in 10 min.
  - At 4 m, only 20–40 % of frames were non-empty for the first ~170 s, against 94–99 % after.
    The frames have not been looked at; the walker may have been moving or partly out of view.
- **The bring-up failure:** `perception_ok` flapped. 8.6 % of frames (482/5585) were published
  blind, on "extrapolation into the future" for `lidar_init ← camera_color_optical_frame`, at
  frame ages of 0.48–0.61 s. The rate was ~5/s at start and ~1.7/s a minute later. Odom latency
  itself was fine (median 35 ms, max 345 ms; FAST-LIO gate p50 0.040 s). So the lag is on
  perception's side (70 % CPU; its tf listener behind). Not fixed.
- **The observe preflight was NO-GO**, on `perception_ok` and on **E7: EKF2 resets in 60 s**
  (xy 4, z 3, heading 3), which is unexplained. Nothing was armed.

### Measured 2026-09-30 14:55 — bench bag, hybrid ON vs OFF by replay; a far-side LiDAR misread; the dtp_viz cylinder fixed
Bag `bags/2026-09-30_145506` (54 s, aircraft on a lab table, observe-only, one walker at
1–9 m). Scripts and the method: `docs/evidence/lidar-range-2026-09-30/bench-1455/`. Replays:
`~/dtp_datasets/20260930_1455_replay/retimed/`.
- **Live, it was 77 % blind.** It was recorded with the GCS's *All topics*: 211 of 331 frames were
  lost on depth/colour skew, and 45 on TF. → `gcs-dashboard.md`
- **Replay method.** The bag does not replay as recorded: the arrival order keeps the skew (395
  frames, 0 people), and only 77 FAST-LIO `/tf` were recorded. So `retime2.py` re-delivers each
  message at its header stamp plus the 14:22 bag's healthy latency, with `/tf` rebuilt from
  `/lidar_slam/odom`. That gives 398 frames, 226–227 with the walker. ON ×2 and OFF ×2 came out
  bit-identical. Truth: dynamic LiDAR points on the published bearing (±5°, ±0.3 s), with the
  floor from the depth camera.
- **Result (|err| of the published range; within 0.3 m; > 1 m FAR):**

  | truth | walker | ON | OFF |
  |---|---|---|---|
  | 6–8 m | moving | 0.22 m, 57 % | 0.48 m, 7 % (the frozen EMA/clamp lag) |
  | 8–10 m | standing / moving | 0.08 / 0.11 m, 100 % | 0.21 / 0.44 m, 84 % / 0 % |
  | 2–4 m | all | 0.17 m, 63 %, 6 far | 0.17 m, 69 %, 0 far |
  | 4–6 m | moving | **+3.17 m median, 17 far** | +0.85 m (lag), 14 far |
  | 0–2 m | standing | +5.33 m, 6 far | +2.72 m, 4 far (a bad first range at the bag start, both) |
  | all | | 0.22 m, 54 %, **29 far** | 0.39 m, 45 %, 18 far |
- **The user's decision (2026-09-30): leave it, and note it in case it becomes a problem.**
  The bench had the drone on a table indoors: the camera 0.90 m above a glossy floor, with a
  ceiling and furniture in the strip. That setup is what produced the misread. No guard was
  added.
  - **Watch for it** in any bag with `/dtp/range_source`: a LiDAR-sourced range well beyond the
    camera's (`src` lidar, `r − cam_r` > 1.5 m). `bench-1455/rs1422.py` scans a bag for exactly
    that.
  - **If it shows up, the fix is ready:** the guard in `bench-1455/proto.py` and
    `guard/replay_perception_probe.py`.
- **⚠️ The bug: the ground rejection deletes the walker indoors, and the wall behind becomes the
  range (FAR, the dangerous side). Not fixed (above).** `strip_ground` takes the 10th
  percentile of z in 2 m bins as "ground". The MID360 cannot see this glossy floor inside about
  7 m, so in those bins the "ground" is the **ceiling** (+1.90 m at 3.6–5.6 m) or furniture
  (+0.53 m), against a floor at −1.1. Every walker point below "ground + 0.25" goes.
  - Reproduced headless with the bag's own scans (`repro.py`, `repro2.py`): at t 40.72, 156 of
    the 190 points in the box were rejected, leaving 8.96 m (12 points). With the rejection off
    it is 5.87 m (105 points); truth 5.74.
  - **The switch-out rule then trusts it.** "The LiDAR's own range beyond 8.0 m" switched a track
    the camera had at 5.6 m (in the gate's ±3 m of the 6.09 forecast).
  - **It happened live at 14:22 too:** 11 of 128 LiDAR-sourced frames > 1 m far, e.g. t 289.2,
    LiDAR 7.32 m for a walker at 2.06 (camera 1.96). The "near clutter" guess in the entry
    above is wrong: the camera was right. Also t 296.0, "beyond 8.0 m" to 8.96 for a walker at
    5.96. (`truth1422` in the evidence README.)
  - **Proposed guard, prototyped offline only:** drop a strip-ground bin higher than 0.30 m
    below the camera. It makes the bug fail short, the safe side. On all 41 detection frames of
    the bag it removes all 7 misreads (+3.2 to +5.7 m) and changes no good frame (`proto.py`).
    A replay wrapper that patches it in at runtime is built (`guard/`) and not run. The
    09-29 day bag must be re-scored with it too. A camera-agreement check on the switch-out is
    a second, independent guard.
  - Outdoors, the same applies wherever something above the ground fills a bin the LiDAR cannot
    see the ground in (trees, overhangs): walkers inside ~10 m at 1.2 m follow height.
- **The person cylinder (fixed, display only).** `dtp_viz` centred a 1.7 m cylinder on the
  reported point, calling it a "torso centroid". It is the patch centre, 65 % down the box (knee
  / lower thigh). Against the floor the bottom sat 0.30–0.57 m under it at 1–9 m, and the top at
  1.13–1.40 m. It now spans (1 − 0.65)·1.7 below the point to 0.65·1.7 above:
  - bottom −0.04 m at 1–2 m, −0.16 at 2–3, −0.32 at 8–9;
  - the rest is the ~2.5° camera pitch error (`dtp-calibration.md`); with it corrected the
    bottom lands within ±0.05 m at every range.
  - The veto cylinder was drawn at the stale ±2.0 m. It is now asked of command_mux
    (`get_parameters`), and was checked live: 1.5 / 3.0.
  - New `test_dtp_viz_geometry.py`, 4 passed.
  - **Relaunch `dtp_viz` (the follower's viz layer) to see it;** no rebuild (symlink install).
- **The published target z is the same point, so it carries the same knee height and pitch
  sag:** 0.44 m above the floor at 2.3 m and 0.28 m at 8.5 m. command_mux's 3.0 m band is
  measured from it (Part A).

### Measured 2026-09-30 — why the LiDAR does not simply extend the 10 m range (superseded by the Part B build above; kept as the measurement)
- **Asked by the user:** why not range the person with the LiDAR too?
  - `perception_node` can already (`depth_source: lidar`). It takes 0.2 s of
    `/cloud_registered_body` through `T_camera_lidar` into the box.
    - *(2026-09-30, later: `/cloud_registered_body` is in `lidar_imu`, 5.1 cm from the LiDAR
      frame. `LidarDepthSource` refused that offset; it now composes it (`compose_camera_body`).
      See the Part B section above.)*
  - The camera won on 2026-09-08, indoors at 2–6 m: 88 % valid pixels against 13.5 % LiDAR
    coverage. The lower third of the image had no LiDAR at all (the MID360 floor is −7°).
  - Past ~10 m the trade reverses. Stereo error grows with range², and in the 09-29 flight it
    already jumped up to 3.3 m between frames at 7–8 m. A far person also sits above the
    LiDAR's floor.
- **Measured on the walker in `bags/stage_b_2026-09-29_122511`,** in FAST-LIO's cloud (1 point
  in 3, which is what the LiDAR path reads):
  - 3–9 m: usually 0 points per 0.1 s scan (51–85 % of scans empty), up to 14–23;
  - **10–17 m (t 450–453, the walker coming back): 1–5 points per scan, at 0.5–1.7 m of body
    height.**
- **So `depth_source: lidar` as written would NOT range them past 10 m.** It wants ≥ 6 samples
  in the torso patch per 0.2 s window (`MIN_ROI_SAMPLES`), and would get ~2–5.
- **What would work (built 2026-09-30 as Part B: the section above):** camera depth to ~8 m,
  LiDAR beyond it, with:
  - a lower minimum count for LiDAR (each point is ~2 cm accurate);
  - a longer window, compensated for the walker's motion;
  - the whole box rather than the torso patch;
  - then raising `max_depth_m`, `prediction_depth_gate_m` and `acquire_max_range_m` together.

  Replay-test it on this bag before it flies.

### Changed 2026-09-29 — the geofence is 38 m long: `fence_x_max_m` 28.0 → 36.0 (user: "make the fence longer"; NOT flown)
- **Why.** In the 12:32 flight the clamp at x 27.5 held the aircraft for the last 10 s while
  the walker walked on, and the lock went past 10 m (`flight-log.md`). The box is now x [−2, 36],
  y [−2, 2]: goals stop at x 35.5, the hard boundary (land) is at x 39.
- **Why 36 and not more: SUPER's ceiling on a rising field.** Sized from that bag's LiDAR
  (`/cloud_registered_body` + odom, `|y|` < 1.5), in `lidar_init`:

  | x (m) | 0 | 10 | 20 | 28 | 36 | 40 | 44 | 48 | 50–54 |
  |---|---|---|---|---|---|---|---|---|---|
  | ground z (m) | −0.13 | +0.14 | +0.5 | +0.8 | +1.1 | +1.3 | +1.5 | +1.7 | a bank/hedge across the corridor |

  - SUPER snaps goals above z 2.25 down to 2.25 (2026-09-26 finding). So the most it can fly
    above the ground is 2.25 minus the ground: ~1.17 m at the new clamp (x 35.5), the 1.2 m
    follow height. At x 44 it would be 0.7 m.
  - **Going further up this slope needs SUPER's ceiling raised** (ROG-Map `virtual_ceil_height`
    / `map_size` z in `follow_test_ros2.yaml` and patch 0001, and the pre-arm gates that pin
    it). That is not done.
  - The corridor: the metal fence at y ≈ −2.2, trees at y ≈ +2.2 from x ≈ 40, and a static
    obstacle across it at x 50–54 (seen from the takeoff spot at 50 m, so not the walker).
- **It holds only for that venue, started at the same spot with the same heading.** The box
  moves with FAST-LIO's origin. Re-cut it anywhere else.
- **Checked.** `dtp_params.yaml` is live through the install symlink, so no rebuild is needed.
  The real `command_mux` started with it on domain 97 logs "GEOFENCE ON: 38.0 x 4.0 m box:
  x [-2.0, 36.0]". `fence_check.py` 13/13 (it tests the clamp logic on its own 10 m box, not
  this value). The NMPC fence mirror is off under `geometric` and was not changed.
  **Relaunch the follower to pick it up.**
- **The clamp is quiet.** `/mux/state` reads FOLLOW|OK while clamped. The only signs are the
  goal's coordinates and a `geofence: FOLLOW goal … clamped` WARN every 2 s in the mux log.

### Measured 2026-09-29 — why the camera does not stay centred on the person: SUPER's heading command, not PX4
`drone-dtp/scripts/yaw_chain_report.py <bag>…` was run on the three follow bags: 09-26 19:51 and
20:06, and 09-21 salvaged. It uses every `/dtp/follow_target` sample in Offboard. It splits the
camera's off-centre angle exactly into three parts:
- **aim** = the person against the `/planning/yaw_target` point;
- **SUPER** = the aim point, seen from the aircraft now, against `pos_cmd.yaw`;
- **PX4** = `pos_cmd.yaw` against the actual heading.

| | 09-26 flight 1 | 09-26 flight 2 | 09-21 |
|---|---|---|---|
| camera off-centre, p50 / p95 / max | 7.7 / 17.4 / 31.0° | 2.1 / 16.3 / 20.6° | 4.5 / 16.2 / 20.6° |
| of that, SUPER's heading command, p95 | 13.9° | 16.7° | 18.2° |
| aim point vs person, p95 | 2.5° | 0.7° | 1.7° |
| PX4 vs SUPER's heading, p95 | 3.3° | 1.9° | 2.1° |
| SUPER's share when off-centre > 20° | 84 % | 98 % | 92 % |

- **Camera: HFOV 92.6° (1280 px, fx 611), so the frame edge is 46° off-centre.** Every flight was
  a walker at ≤ 0.9 m/s, 3.5–7 m away.
- **The camera lags the person's bearing by about 0.8 s** (flight 1: slope 0.81 s, corr 0.73). The
  bearing rate reached 36–60°/s at p99/max. SUPER's commanded `yaw_dot` never went above 23°/s,
  although `yaw_dot_max` is 57°/s.
- **Why, from source** (`super_planner.cpp` ~795, `yaw_traj_opt.cpp`):
  - Under `yaw_mode 3`, SUPER aims the heading at the target from where its trajectory **ends**,
    not from where the aircraft is. That point is up to the planning horizon ahead or to the side.
  - It then reaches that end heading along one smooth polynomial over the whole trajectory, and
    the polynomial is re-solved at each replan.
  - Together these give a heading that trails the bearing, plus a parallax error when the
    aircraft moves sideways.
- **PX4 follows what it is given.** In Offboard, `yaw` and `yawspeed` pass straight to the
  attitude controller, `yawspeed` as feed-forward (`PositionControl.cpp:105,269`).
  `MPC_YAWRAUTO_MAX` applies only to Auto modes.
- **Extrapolated, not measured:** at 5 m/s across the line of sight at 4 m (72°/s), a 0.8 s lag is
  ~57° off-centre. That is outside the frame.
- **Replay of "fix 1" (aim from the aircraft's position NOW), not built:**
  `drone-dtp/scripts/fix1_yaw_replay.py <bag>`.
  - **Method.** A PX4 yaw model is fitted on each flight (P + feed-forward + delay). Its replay
    of the flown heading matches the measurement: p95 15.7° modelled against 16.8° measured, and
    17.2° against 16.9°. The model is then driven by the bearing from the aircraft to
    `/planning/yaw_target`. Positions are as flown.

  | Person off-centre, p50 / p95 / max | as flown | fix 1 | fix 1 + a rate feed-forward ≤ 60°/s |
  |---|---|---|---|
  | 09-26 20:06 bag (the flight at 20:12) | 2.1 / 16.8 / 20.6° | 0.3 / 4.2 / 5.0° | 0.4 / 1.8 / 5.0° |
  | 09-26 19:51 bag (the flight at 19:57) | 7.4 / 16.9 / 20.0° | 1.6 / 8.5 / 11.0° | 1.1 / 5.7 / 7.1° |

  - **It is also gentler.** In the 20:12 flight, with the yaw target fresh (22 s), the aircraft
    yawed at up to 87°/s as flown and would have yawed at most 8°/s under fix 1.
  - **The high rates came after the loss.** There, all three variants reached 94–194°/s: the yaw
    target was stale, so the command fell back to SUPER's swing. The yaw hold of 2026-09-26 keeps
    the yaw target alive through that. A fix 1 must aim at the held point and never fall back to
    SUPER's heading.
  - **It would not have saved that flight's loss** (t 379.2 s). The person was 1.4° off-centre
    just before it, and the cause was the dusk.
- **The build plan: `docs/plan-camera-yaw-servo.md`.** **Built 2026-09-29, both switches off,
  not flown.** The replay on the real module fails on the 19:51 bag because of the close-range
  rule at 2.5 m, and the servo alone does not prevent the swing after a loss. The record is
  `px4-bridges-offboard.md`, "2026-09-29 — the camera yaw servo".

### Added 2026-09-29 — the follow band is 4–5 m (user decision; bench-checked, NOT flown)
- **`follow_near_m` 4.0, `follow_far_m` 5.0 in `dtp_params.yaml`.** It was 3–4 m on every flight
  so far. `min_standoff_m` stays 2.5, and so do the 1.5 m veto and the descent-guard radii.
  **The node's defaults are still 3.0 / 4.0**, so only launches that load `dtp_params.yaml`
  get the new band. Relaunch the follower to pick it up.
- **Measured on the real `follow_planner_node`** (`nmpc_check.py`'s harness and scenes,
  geometric, speed standoff on). Goal-to-person range:

  | Scene | Band 3–4 | Band 4–5 |
  |---|---|---|
  | steady walk, 0.8 m/s | 2.75–2.85 m | 3.34–3.44 m |
  | walk, pause, walk | 2.75–3.50 m | 3.34–4.50 m |
  | turn | 2.75–3.08 m | 3.34–3.83 m |
  | person backs toward the aircraft | 3.66–4.29 m | 4.69–5.29 m |
  | sprint at 2.5 m/s at a standing person | 5.00 m | 6.00 m |
  | 7–9 s after that sprint stops | 3.63 m | 4.60 m |

  - The walk figures are against the person's **current** position. The goal leads by up to
    1.0 m, so it sits closer to a walking person than the band's near edge.
  - 15.0 Hz, no new reversals, goal z 1.20 in both.
- **What to watch on the first flight:**
  - **No lock has been held in flight beyond 4.4 m.** The new band sits at and past that.
  - **With the speed-scaled standoff the band reaches 5.5–6.5 m in a sprint** (it was
    4.5–5.5). The 1.5 m cap was sized so the nominal distance stopped at 5.0 m, "the
    perception cap". `speed_max_delta_m` was not changed.
  - The aircraft trails its goal, so the flown range is larger than the goal's.
- **The gates do not test the new band.** `nmpc_check.py`, `lost_yaw_hold_check.py` and
  `loss_conditioning_check.py` start the node on its defaults, and `nmpc_check.py` hard-codes
  3.0 / 4.0 with the comment "as deployed", which is no longer true.
- **It helps the yaw servo.** A follow at 4–5 m stays clear of the servo's 2.5 m close-range
  rule, which is what failed the 09-26 19:51 replay.

### Added 2026-09-27 — the speed-scaled standoff and the braking envelope ("Option A"; both switches ON since 2026-09-28, bench-gated, NEVER FLOWN)
- **Why.** The 3–4 m band, the 2.5 m floor, the 1.5 m veto and the 3.0/3.5 m descent-guard radii
  were sized for ~1 m/s. `max_vel` is 3.0 and `max_acc` 2.0. Read from the 2026-09-26 bags
  (don't re-derive):
  - **SUPER's reference kept ≥ ~1.2 × v²/(2·max_acc) from its goal in both sprints**: every plan
    ends at rest (`goal_vel_en` false), so SUPER already holds its braking distance *to the goal*.
    It trailed the goal by 0.5–0.9 m at walking pace. The 2.57 m was the aircraft running 0.8 m
    past its reference (no acceleration feed-forward, `px4-bridges-offboard.md`).
  - **Closing speed** (aircraft toward the person, less the person's speed away) was 0–0.3 m/s in
    steady follow and peaked at **1.87 / 2.03 m/s** in the sprints. That is the signal. A standoff
    keyed on the aircraft's own speed would add SUPER's trailing again in a steady follow.
  - **`/lidar_slam/odom`'s twist is all zeros.** Both nodes difference the pose over 0.2 s (0.08–0.10
    m/s median error against EKF2 on those bags).
  - **Flight 1's turnaround closest approach (2.16–2.23 m) came at closing ≤ 0.55 m/s.** No speed
    rule changes it. It is geometry (goal swing, fence clamp), and this does not address it.
- **The formula** (one table asserted by both unit tests, so the C++ and the Python cannot drift):
  `reach = d_min + v_c·react + v_c²/(2·brake) + min(track_s·v_ground, track_max)`,
  `extra = clamp(reach − min_standoff_m, 0, cap)`. Defaults: `d_min` 1.5 (= the veto radius),
  `react` 0.5 s, `brake` 2.0 (SUPER's `max_acc`), **`track_s` 0.4 = the no-feed-forward margin**
  (cap 2.0 m; set 0.25 / 0.25 once the FF has flown), cap **1.5 m → floor 4.0 / nominal 5.0 m**
  (perception: stereo optimal ≤ 6 m, no lock held in flight beyond 4.4 m). Hysteresis: grow at
  once, hold 1.0 s, shrink ≤ 0.5 m/s.
  - At the defaults, extra is 0 up to ~0.89 m/s closing, and hits the cap at ~1.84 m/s.
  - Veto reach ahead at 1/2/3 m/s: 2.65 / 4.30 / 6.45 m. The lateral radius stays 1.5 m.
- **follow_planner** (`speed_standoff.hpp`, ROS-free; `follow_planner_node.hpp`: `NearEff`,
  `FarEff`, `FloorEff`, `UpdateSpeedStandoff`). Switch **`speed_standoff_enable`** (false).
  - The band AND the floor move out by `extra`, so the lead budget is unchanged. It covers the
    geometric and nmpc arms and the M6.5 conditioned viewpoint's floor. **Not `stock_baseline`.**
  - With the switch off, the three helpers return the members themselves: nmpc_check N1
    **worst |Δ| 0.00e+00**.
  - `speed_standoff_signal: closing` (default) | `aircraft` (Option B). The node **refuses to
    start** on `closing` if SUPER's `goal_vel_en` is true, and on `speed_brake_mps2` > SUPER's
    `max_acc`.
  - The person's speed away is the **minimum** of their last 5 reports. A median does not stop a
    3-report glitch burst (09-26 had one, to 2.82 m/s). A coasting target counts as stopped.
- **command_mux** (`command_mux/brake_envelope.py`, pure). Switch **`brake_envelope_enable`** (false).
  - The veto ALSO tests the **braking envelope**: react, then brake to rest along the current
    velocity, plus the overshoot, against each person's forecast *at the same instant*.
  - The goal end of the commanded path is pushed out by the overshoot. At zero speed nothing new
    can veto.
  - The descent guard's trip AND release radii grow by the same `extra` (`DescentGuard.apply(...,
    extra_radius_m)`), keeping "release radius ≥ nominal standoff".
  - `/mux/state` appends `| brake v … closing … reach … extra …` when on.
- **Launch / preflight.** `follower.launch.py` passes SUPER's `max_acc` and `goal_vel_en` to both
  nodes (verified on an isolated domain: `super_max_acc_mps2 2.0`, `super_goal_vel_en false`).
  `stage_a_preflight.py` reports both switches on every run:
  - WARN when on, and WARN if only one is on;
  - **FAIL if the nodes disagree on a shared number**;
  - WARN if `speed_track_s` < 0.4 without the FF.
- **Gates (2026-09-27, all on the installed binary, one at a time):**
  - unit tests: `command_mux` pytest **125** (was 86; +33 `test_brake_envelope.py`, +6
    `test_descent_guard.py`); `follow_planner` gtest **26/26** (+11 `test_speed_standoff`);
  - **`drone-dtp/scripts/brake_envelope_check.py` 13/13** (new, real mux, **domain 94**). B2 = at
    2.5 m/s the envelope vetoes a standing person at 5.20 m (formula 5.31). B6 = with the switch
    OFF, no veto to 4.0 m and the guard latches at 3.0 m, as today;
  - **`nmpc_check.py` 52/52**: new **N13** (11 checks, both arms: sprint goals at 5.00 m not
    3.5; release held ~1 s then ≤ 0.5 m/s; pace — closing leaves delta 0, Option B adds ~0.75 m;
    three refusals). `--only-n13` runs it alone;
  - `loss_conditioning_check` 18/18, 17/17 (`--no-coast-yaw-hold`), 18/18 (`--goal-mode nmpc`);
    `lost_yaw_hold_check` 26/26;
  - `fence_check` 13/13, `descent_guard_check` 13/13 (switch off) and 13/13 with the new
    `--mux-param brake_envelope_enable:=true`, `terrain_check` 10/10, `engage_gate_check` 9/9.
  - Also checked on the real node (scratch harness): the M6.5 OCCLUDED viewpoint 3.00 m from the
    hidden person stays at 3.00 with the switch off and moves out to 4.00 with it on at 2.5 m/s.
- **Bench replay of the two 09-26 bags (open loop, the real code, as shipped):**
  - flight 1: extra hits the cap (floor 4.0 / nominal 5.0) at 363.8 s (closing 1.85 m/s, range
    4.16 m); it is > 0 only for 363.0–368.3, and 0 through the 22 s steady follow and the turnaround.
    The envelope veto would have fired **once, at 363.9** (2.23 m/s, closing 1.89), for 2.4 s;
  - flight 2: the cap at 371.0 s (closing 1.86 m/s, range 5.9 m). Elsewhere extra is ≤ 0.021 m,
    below the goal deadband. **No envelope veto.**
  - follow_planner's delta and the mux's extra peak identically on both bags.
  - Open loop: what the aircraft would then have done is not in the replay.
- **Found on the way:**
  - (a) the gate's B3 caught `PersonVelocities` re-using a track id's old reports after a gap (a
    standing person's v 0 made a walker read as closing at 2.5 m/s). Fixed, with a unit test.
  - (b) N13 release needs the deadband quantization allowed for: the goal moves in 0.167 m steps,
    so 4 can land in 1 s at a true 0.5 m/s. nmpc rides its dead zone's far edge inward, so it is
    judged after 8 s.
  - (c) the rclpy teardown traceback (`engage_gate_check` note above) appears in any harness that
    stops a node while still publishing to it. `brake_envelope_check` quiets its publishers first.
- **Installed 2026-09-27 17:52** (`colcon build --symlink-install --packages-select dtp_msgs
  follow_planner`). The mux is Python (develop install): live at the next launch.
  - **Undo, without a rebuild:** set both switches false in `dtp_params.yaml` and relaunch the
    follower. They are false in code. **The user turned both on in `dtp_params.yaml` on
    2026-09-28.**
  - **Undo, to a binary:** `~/backups/backup-pre-speedstandoff-20260927/follow_planner_node` (the
    09-26 21:45 yaw-hold binary, md5 `533823e0…`).
- **Not verified, needs a flight:**
  - that the aircraft, not just the goal, keeps the floor in a real sprint;
  - that the veto's HOLD at 2–3 m/s stops outside 1.5 m (a HOLD at speed can overrun
    `HOLD_REANCHOR_M` 3.0 and re-anchor mid-stop);
  - SUPER's trailing above 1.9 m/s;
  - that locks hold at 4–5 m.

### Added 2026-09-26 — the yaw hold: at a loss, and through the coast before it (bench-gated, NOT flown)
- **Why.** Whenever SUPER has no fresh yaw target, it falls back to its free-end yaw: the
  heading follows the trajectory. That happened twice per loss.
  - **In the coast (0.5–~1.2 s).** A coasting target keeps the perception stamp of its last seen
    frame, by design (`target_manager.py`, "THE STAMP IS NEVER REWRITTEN", risk R-F10). SUPER
    drops it at 0.5 s.
  - **After the loss.** `follow_planner` fell silent on `/planning/yaw_target`. On the 2026-09-26
    20:12 flight the HOLD after the loss was a ~1.5 m dip-and-climb. That is past patch 0006's
    0.3 m threshold, and the camera turned 83° away in ~2 s (`flight-log.md`).
- **What** (`follow_planner_node.hpp`: `YawHold`, `ReleaseYawHold`, `CoastYawHoldDue`,
  `AimYawAt`). The node latches ONE heading, once per episode, when either:
  - a coasting target's stamp age reaches **0.4 s** (`kCoastYawHoldAgeS`, one tick short of
    SUPER's gate), with `coast_yaw_hold` on; or
  - the target goes stale after having had one (the loss).

  The heading points **toward where the camera was last aimed** (`last_yaw_point_`), or the
  aircraft's own heading if that point is within 1 m. The last aim is the person, the CTRV point,
  or, under `loss_conditioning`, the oracle's re-emergence point. The node publishes the heading
  as a point 100 m out, **stamped now**, for up to `lost_yaw_hold_s` (**15 s**) from the latch.
  - A seen target ends it, and so does a fresh oracle verdict (conditioning aims the camera
    itself).
  - A hold latched in a coast carries through the loss unchanged.
  - `lost_yaw_hold_s: 0` = all off (the flown behaviour). `coast_yaw_hold: false` = the loss
    half only. Both are in `dtp_params.yaml`.
- **The latch fix.** The first build aimed the post-loss hold at the CTRV point. Under
  conditioning, that swung the camera off the re-emergence point it had just been aimed at.
  It now latches on the last point actually published. In the flight configuration the two are
  the same point.
- **⚠️ The coast half changes the M6.5 carry-forward arm** (user's decision, 2026-09-26). Its
  coasting yaw targets used to keep the perception stamp and lapse at 0.5 s. Now they hold a
  heading. For the documented arm, run with `coast_yaw_hold:=false`
  (`loss_conditioning_check.py --no-coast-yaw-hold`).
- **Side effects, inside the window.** An Offboard re-engage turns the nose back to the held
  heading. A NAV goal (the GCS RC card) keeps it. A one-frame flicker never holds (L8).
- **Gates:**
  - **`drone-dtp/scripts/lost_yaw_hold_check.py` 26/26** (real node, **domain 93**, ~2 min).
    L7 = the coast: the hold takes over at 0.44 s of stamp age and carries through the loss.
    L10 = the latch fix: the hold keeps the oracle's 45° after the loss, not the CTRV 0°.
  - `loss_conditioning_check.py` **18/18** by default, **17/17** with `--no-coast-yaw-hold`
    (the old OFF-arm checks, unchanged, still pass), and **18/18** with `--goal-mode nmpc`.
    S2/S4 OFF now expect the held heading, and S2 ON checks the re-emergence bearing after the
    loss (−21.8°).
  - `nmpc_check.py` **41/41**: N1 = the geometric goals bit-identical to the pre-NMPC binary.
  - All of these ran on a scratch build first. `lost_yaw_hold_check` and `loss_conditioning_check`
    were then re-run on the installed binary.
- **Installed 2026-09-26 21:45** (a first version, the loss half only, was installed at 20:59).
  `~/super_ws` was rebuilt with
  `colcon build --symlink-install --packages-select dtp_msgs follow_planner`.
  - **This also puts the dormant NMPC Stage 1/2 code into the flight binary** (see below).
    `dtp_msgs` gains `VisibilityField`, and the existing four messages are byte-identical.
  - `fsm_node` has no unresolved symbols against the rebuilt `dtp_msgs`, and
    `loss_oracle_check.py` passes 54/54 on it (O6 classify p99 89 µs, evaluation p99 2.5 ms).
  - `stage_a_preflight`'s "no `goal_mode` on `follow_planner_node`" warning is expected to stop:
    the node now has the parameter, `geometric`.
  - **Undo, without a rebuild** (relaunch the follower):
    - `lost_yaw_hold_s: 0.0` turns the whole hold off;
    - `coast_yaw_hold: false` turns off the coast half only.
  - **Undo, to a binary:** copy one of these over
    `~/super_ws/build/follow_planner/follow_planner_node`:
    - `~/backups/backup-pre-lostyawhold-20260926/follow_planner_node`, the flown 09-14 binary;
    - `~/backups/backup-lostyawhold-v1-20260926/`, the 20:59 loss-only version.

### Added 2026-09-19/20 — `goal_mode: nmpc`, a receding-horizon follow goal (Stages 1 + 2, bench-gated, NEVER FLOWN)
**Full record: [`docs/fact-nmpc-follow-goal.md`](docs/fact-nmpc-follow-goal.md).** Branch
`nmpc-follow-goal`. Don't re-derive:
- **What it is.** `follow_planner_node` gains `goal_mode: geometric | nmpc`. Only the body of
  `ComputeStandoffGoal` is replaced, behind a dispatcher; every guard still wraps the answer and
  `ApplyHardGuards()` ends both arms. The solver (`nmpc_follow.hpp`, ROS-free) optimises the
  **goal's** positions over a 1.2 s horizon (18 stages of one tick), not the aircraft's, with an
  L-BFGS vendored from its **MIT upstream, not from SUPER's LGPL tree**
  (`follow_planner/THIRDPARTY.md`). Every constraint is a penalty; the guarantee is the hard
  guards. A refused solve publishes that tick's geometric goal — it never Holds or Suspends.
- **~~`~/super_ws` was NOT rebuilt~~ — superseded 2026-09-26:** the installed
  `follow_planner_node` was rebuilt for the yaw hold after a loss (section above), so it contains
  this code, dormant under `geometric`. Until then it was the pre-change binary, by the user's
  decision (baseline flight first). `dtp_params.yaml` ships
  `goal_mode: geometric`; the arm is chosen with `goal_mode:=` on the launch
  (`deploy_follow.sh --goal-mode=nmpc`), and changing it is a relaunch.
- **It does not raise top speed and does not make SUPER plan through the goal**
  (`goal_vel_en` needs the goal > 3.5 m away). Its gain is a goal with a memory, so SUPER's 15 Hz
  replans chain. Whether it cuts the `OPT_FAILED` replans is untested.
- **Gates:** `colcon test --packages-select follow_planner` gtest **15/15**;
  **`python3 drone-dtp/scripts/nmpc_check.py` 41/41** (real node, **domain 90**, both arms; N1 =
  geometric bit-identical to the old binary; N12 = Stage 2); `loss_conditioning_check.py
  --goal-mode nmpc` 16/16. **Re-run `nmpc_check.py` after any `follow_planner` change.**
- **Stage 2 (visibility), built 2026-09-20:** SUPER patch **0008** makes the loss oracle publish
  `dtp_msgs/VisibilityField` on `/planning/visibility` for every locked frame, seen or coasting;
  `follow_planner` uses it when `nmpc_w_vis` / `nmpc_w_clear` > 0 (**default 0 = exactly
  Stage 1**; 10 / 4 when on). A dark trail bearing is moved to the nearest clear one *outside*
  the solver (`nmpc::ClearBearing`) — a visibility cost alone cannot get a goal out of the
  middle of a shadow, and a first attempt that relied on it passed or failed run to run.
  **Patch 0008 is NOT applied to the live SUPER tree and `fsm_node` was NOT rebuilt.**
  Gate: `python3 drone-dtp/scripts/visibility_field_check.py` **13/13** (domain 88; V9 is the
  real `fsm_node` feeding the real `follow_planner`, checked against the scene's geometry: the
  line of sight clears a tube by ~1.0 m). The field knows occlusion only near where the aircraft
  has looked from (open scene: vis 0.19, unk 0.81). It has never seen a real map.
- **Build order changed:** `follow_planner` now includes `dtp_msgs/msg/visibility_field.hpp`, so
  **rebuild `dtp_msgs` before `follow_planner`**, or the build fails.
- **`loss_conditioning_check.py`'s "nodes did not come up" flake is fixed** (a harness race:
  `target_manager` publishes `/dtp/lock_state` only at startup and from its `/dtp/targets`
  callback, and the harness waited in silence). It failed ~1 run in 3 before any of this work.
- **Bag replay (`nmpc_replay.py`, open loop, run5, two runs per arm):** direction reversals
  52 → 15–18, goal step p95 0.73 → 0.33 m, band occupancy unchanged, 1 fallback in 3175 solves.
  **It inherits heading flips:** both arms take the same 27–30 goal jumps of 4–7 m when a slow
  walker's heading flips — Stage 1 does not fix that. Results: `~/dtp_datasets/20260920_nmpc_replay/`.
- **`stage_a_preflight.py`** prints the arm, WARNs on nmpc in Stage A, FAILs on nmpc with
  `stock_baseline`, and under `--stage-b` FAILs on nmpc until `--nmpc-cleared`.
- **Nineteen bench traps are in the fact doc**, ten for Stage 1 (the speed-penalising anchor, `goal_acc_max` vs the
  horizon, the iteration cap, the sanity bracket that caused the jump it existed to prevent, the
  dead zone's far edge, the goal-repelling term, starvation, unoptimised Eigen, a reset rule that
  fired on heading noise — found only by replaying a real bag — and the deadband still
  quantising the wire) and nine for Stage 2 (one ray per bin misplacing a narrow shadow, Gibbs
  ripples, a plateau with no slope, bearing buying range, a goal that never settled beside a
  standing person, the hill-top, UNKNOWN beside a known obstacle). Read them before retuning.
- **Next:** the baseline flight, then Stage A in two legs scored with
  `analyze_shadow_goals.py --goal-topic /dtp/follow_goal`; a second node in shadow is NOT safe
  until the preflight is hardened. Stage 2 needs a replay on a real map before it flies.

### Added 2026-09-22 — the engage gate: no goal to SUPER until Offboard (bench-gated, NEVER FLOWN)
- **Why.** At the 2026-09-21 20:05 Offboard switch, `pos_cmd` was 3.0 m from the aircraft. The
  aircraft closed the gap at 1.7 m/s with the walker nearby (`flight-log.md`).
  - **SUPER plans from odometry only when it starts from rest** (`PlanFromRest`). Every replan
    after that starts from its own committed reference (`generateExpTraj`, `cmd_traj_info_` at
    `replan_state_TT`), and nothing outside `fsm_node` can reset it.
  - The mux had fed it goals since FOLLOW engaged on the ground. So the reference "flew" the
    follow while the pilot hovered in Position.
- **What.** `command_mux/engage_gate.py` (pure, 7 unit tests), wired into `command_mux_node`'s
  single publish point.
  - No goal is published until a **fresh** `vehicle_control_mode` says armed + Offboard.
    SUPER idles until then, and its first plan starts from rest at the aircraft.
  - Arbitration, vetoes, the descent-guard latch and `/mux/state` run as before. The state line
    ends `to SUPER: held until Offboard`.
  - **A stale report never shuts the gate.** That would silence the proximity HOLD in the air.
    It shuts only on a fresh report that says the pilot took over or the aircraft disarmed.
  - A HOLD latched while the gate was shut is dropped when it opens. Otherwise a ground HOLD
    (z 0.85) would be SUPER's first goal.
- **Scope.** On only with `abort_enable` (the PX4-state path). Observe-only is therefore
  unaffected: it has `abort_enable` false and shadow topics. `engage_gate_enable:=false` is for
  benches (ERROR at startup, `ENGAGE GATE OFF` in the state line), and
  **`stage_a_preflight.py --stage-b` FAILs on it**.
- **Not covered.** A re-engage within a few seconds of a takeover, while SUPER finishes its last
  trajectory. Goals that bypass the mux: RViz `/goal_pose` and the GCS RC card. Those need the
  bridge to refuse a far setpoint at engage (Fix 2, not built).
- **Gates.** **`drone-dtp/scripts/engage_gate_check.py` 9/9, twice** (real node, **domain 92**,
  fake mode at 2 Hz). The first goal after the switch took 0.02–0.24 s, and was anchored on the
  aircraft. The check also covers a stale report with a proximity HOLD still reaching SUPER, the
  takeover, the re-engage, the bench switch and observe-only.
  - `descent_guard_check.py` 13/13 and `fence_check.py` 13/13 both now run with the gate off:
    they need goals outside Offboard.
  - `terrain_check.py` 10/10, run with the gate open. Unit tests 86.
  - **⚠️ 2026-09-23: `engage_gate_check.py` intermittently reports 9/**10**, the extra line being
    `[FAIL] traceback in mux_<scenario>.log`. It is a TEARDOWN RACE IN THE HARNESS, not a defect
    in the gate or in `command_mux`.** Ran 4×: traceback in `mux_gate_on` once, in `mux_observe`
    twice, and a clean **9/9** once — i.e. it lands in whichever scenario loses the race, about
    1 s after that scenario's last event, and the error is
    `RuntimeError: Unable to convert call argument to Python object` inside rclpy's
    `_take_subscription`. **All 9 functional checks passed on all 4 runs.** The 10th entry is
    conditional — the script appends it only when it finds a traceback — which is why a clean run
    prints 9/9 and a raced one 9/10.
    - **Not a stale build:** `command_mux` imports only `dtp_msgs/TargetArray` (unchanged since
      2026-07-27) and is pure Python, loading typesupport at runtime, so the 09-14 `dtp_msgs`
      rebuild cannot mismatch it.
    - **Not flight behaviour:** in the same evening's 8-minute `bench_bringup.sh` the real
      `command_mux_node` logged *"process has finished cleanly"*.
    - So: **treat 9/10 with the traceback line as a PASS, and re-run to get the clean 9/9.**
      Worth fixing in the harness (shut the fake publishers down before the node) if it ever
      masks a real failure.
- Takes effect on the next follower launch (`--symlink-install`, no rebuild).

### Added 2026-09-30 — the park: a takeover stops SUPER planning (bench-gated, NEVER FLOWN)
- **Why.** On 2026-09-30 the GCS stopped answering at 18:38 (`/api/state` took 20–25 s). The
  Jetson was at load 10–12, `fsm_node` at 144 % of a core and 4.3 GB, the GCS process at 125 %.
  - After the 18:31:57 takeover SUPER re-planned to its last goal about 25 times a second for
    45 minutes: 76,229 `GeneratePolytopeFromLine failed` in `fsm_node_6512_*.log`.
  - The same loop ran after the 17:28 flight (1,568) and the 18:14 flight (4,359).
  - **The cause is in SUPER's FSM, and it is not specific to the code that was rolled back.**
    A trajectory that ends more than 0.1 m from the goal sends the FSM to `GENERATE_TRAJ`
    (`fsm_ros2.hpp` `pubCmdTimerCallback`). That state calls `PlanFromRest` on every 10 ms tick
    until a plan succeeds or the aircraft is within 0.1 m of the goal (`fsm.cpp`). After a
    takeover neither happens. When the plans succeed instead of failing, SUPER "flies" its
    reference to the stale goal again and again.
  - The engage gate only withholds new goals. It never took the old one away.
- **What.** Two halves, both needed.
  - **SUPER patch `0009-super-park-on-takeover.patch`.** A `std_msgs/Empty` on `fsm/park_topic`
    (`/planning/park`) calls `Fsm::parkFollow()`: the goal is dropped, the FSM goes to
    `WAIT_GOAL`, and `pos_cmd` is silent until the next accepted goal. That goal is planned from
    rest at the aircraft. The subscription exists only with `fsm/follow_goal_en`.
  - **`command_mux`** sends the park when the engage gate shuts: 3 messages 0.3 s apart
    (`ParkBurst` in `engage_gate.py`), because the topic is BEST_EFFORT. A gate that re-opens
    cancels the rest of the burst. Nothing is sent before the first engage.
  - `engage_gate_park_enable:=false` turns it off. It is on by default and not in
    `dtp_params.yaml`, like `engage_gate_enable`.
  - With a SUPER that lacks 0009 nothing subscribes the topic, nothing changes, and the mux logs
    an ERROR at the takeover. **A bag recorder subscribed to `/planning/park` would hide that
    ERROR**, so the topic is not in any recorder preset.
- **What it changes in flight.** After a takeover `pos_cmd` stops at once. The bridge is not in
  Offboard then, so it sends PX4 nothing either way. **A re-engage now starts from rest at the
  aircraft**, where before SUPER's reference was wherever its last trajectory had got to. That
  was the engage gate's first "not covered" case.
- **Still not covered.** Goals that bypass the mux (RViz `/goal_pose`, the GCS RC card): a stale
  one of those still loops. An accepted goal from either un-parks SUPER.
- **Gates.**
  - **New: `drone-dtp/scripts/park_check.py` 14/14** (real `fsm_node`, **domain 88**, ~2 min).
    It reproduces the loop without a park (4 plans and 1,198 `pos_cmd` in 12 s to a stale goal).
    With one park `pos_cmd` stops within 0.5 s and stays silent for 10 s with no planning. The
    next goal's first command is 0.25 m from the aircraft (the harness wobble is 0.20 m).
  - `engage_gate_check.py` **14/14** (was 9): five park cases added. Three of four runs also
    printed the known teardown traceback line (2026-09-23 note above); all 14 functional checks
    passed on every run.
  - `fence_check` 13/13, `descent_guard_check` 13/13, `terrain_check` 10/10 (one run of four
    hit the same teardown traceback), `yaw_hold_check` 7/7, `target_filter_check` 17/17,
    `loss_oracle_check` 54/54, `unhit_decay_check` 10/10. Unit tests 90.
- **Built.** `super_planner` rebuilt 2026-09-30 23:06 (plain build, 2 min 35 s). The old binary
  and the five source files are in `~/backups/super-pre-park-20260930/`.
- **Undo.** Copy that `fsm_node` back to
  `~/super_ws/install/super_planner/lib/super_planner/`, or set `engage_gate_park_enable:=false`.
- Takes effect on the next stack and follower launch.

### Added 2026-09-21 — terrain-relative follow height (ON since 2026-09-22, bench-gated, NEVER FLOWN)
**Full record: [`docs/fact-terrain-follow.md`](docs/fact-terrain-follow.md).** Don't re-derive:
- **Switch:** `command_mux: terrain_follow_enable` in `dtp_params.yaml`. It shipped `false`; the
  user turned it **on 2026-09-22**, after the 20:05 replay (1.25–1.43 m above ground vs 2.5–2.8 m
  flown). **Engage Offboard near 1.2 m**: the descent guard's floor latches at the engage height,
  and in terrain mode it holds that height above the ground for the whole follow. The
  follower launch reads it and starts `terrain_estimator` only when true. Relaunch to change it.
- **What:** a LiDAR ground map (`command_mux/terrain_map.py`, NOT the rangefinder), published on
  `/dtp/terrain` as the higher of the ground under the aircraft and under the goal.
  `command_mux/terrain.py` turns it into an offset: ≤ 0.2 m/s down and 0.3 up in Offboard,
  clamped −1.2 / +1.5, held when stale. FOLLOW z becomes 1.2 + offset. The descent guard and HOLD
  work in height above the ground. `/mux/state` shows `| terrain +X.XX status`.
- **Evidence:** replaying the 18:41 bag, height above ground (ToF-checked) would have been
  **1.23–1.51 m vs 2.4–3.8 m flown**. `terrain_check.py` 10/10 (domain 91). 79 unit tests.
  4.6 % of a core.
- **Traps (seven, in the fact doc), two that generalise:** an rclpy log call site at two
  severities RAISES and killed the mux; with `abort_enable` false the mux never sees Offboard.
- **Limits:** relief about −1.2 / +1.5 m (ROG-Map's absolute virtual ground and ceiling). The
  estimator must start on the ground. Only tracked people are cut out. The camera is still level.
- **Re-run `terrain_check.py` after any `command_mux` or `terrain_estimator` change**, one gate
  at a time. Three gates at once on a loaded box failed on discovery timing.

### Changed 2026-09-21 — a mux HOLD's climb no longer raises the descent-guard floor (FLOWN 2026-09-21 20:05: held)
- **Why.** In the 18:41 flight a 5 s `HOLD|NO_SOURCE` climbed 0.75 m, and the latched floor went
  1.46 → 2.10. It never released in the follow, and it put the aircraft 3.4–3.7 m above a
  sloping field, which lost the walker (`flight-log.md`).
- **What.** `DescentGuard.hold_started / hold_ended / clear_hold`, driven by
  `command_mux_node._report_hold_to_guard`. From a HOLD's start until the aircraft is back within
  `raise_margin_m`, the guard sees the aircraft at its altitude at HOLD entry, for trip and
  raise alike. **HOLDs are counted only while PX4 is armed + Offboard.** Otherwise a ground or
  Position-mode HOLD would register a near-ground altitude, and the guard would never trip all
  flight. The flight's own sequence (HOLD on the ground, Position takeoff, Offboard mid-HOLD)
  would have done that to the first version of this change. Record:
  `docs/fact-follow-speed-and-descent-guard.md` §5.
- **Cost.** After a HOLD the aircraft comes back down to the floor with a person near. That
  includes proximity-veto HOLDs (user asked for all HOLDs).
- **Gates.** `test_descent_guard.py` 26 passed (9 new). **`descent_guard_check.py` 13/13** (new 1b, 4b).
  The node at `f4e31fa` scores 9/13, failing 4b as the flight did. `fence_check.py` 13/13.
  Takes effect on the next follower launch (`--symlink-install`, no rebuild).
- **In the air (2026-09-21 20:05, `flight-log.md`):** the floor latched at 1.67 m before Offboard
  and stayed there through a `HOLD|PROXIMITY` and a `HOLD|NO_SOURCE` (goal 2.47 m). The aircraft
  came back to 1.65 m after each.

### Changed 2026-09-26 — max_vel 3.0, max_acc 2.0 (user; beyond S3; NOT flown)
- **`max_vel` 1.0 → 3.0, `max_acc` 1.0 → 2.0** (user). This skips the plan's S3 (1.5 / 1.5)
  and flies twice its speed. Changed in `follow_test_ros2.yaml` (src + install, identical), in patch
  0001's `+` lines, and in the three pre-arm gates that pin the ceiling: `deploy_follow.sh`,
  `supervisor.py` `live_preflight` and the `flightday.py` text (`--max-vel 3.0`). Also in
  `flight-day-runbook.md`. Takes effect on the next `fsm_node` launch. The GCS server must be
  restarted to pick up `supervisor.py`.
- **Nothing downstream clamps it:** `px4_offboard_bridge` forwards `pos_cmd` as it is. PX4:
  `MPC_XY_VEL_MAX` 12 and `MPC_Z_VEL_MAX_UP` 3.0 do not bind. **`MPC_Z_VEL_MAX_DN` 1.5 does
  bind**, since `max_vel` is 3-D. `px4_odom_bridge` `max_speed_mps` 5.0 is still above it.
- **Risks, stated to the user:** the engage-gap closure seen 2026-09-21 (3.0 m at 1.7 m/s with
  `max_vel` 1.0) would now run up to 3 m/s. A re-engage after a takeover and goals that bypass the
  mux are still unguarded. The mux proximity veto and the 3 m standoff were sized for ≤ 1 m/s.
  At 3 m/s, stopping from 2 m/s² takes 2.25 m.

### Changed 2026-09-21 — max_acc 1.0 (S2 envelope complete; NOT flown)
- **`max_acc` 0.5 → 1.0** (user), in `follow_test_ros2.yaml` (src + install, identical) and
  patch 0001's `+` line. With `max_vel` 1.0 this is the plan's full S2 envelope. Reaching
  1.0 m/s takes 1 s and 0.5 m (was 2 s and 1 m), so a goal ~2 m ahead no longer caps the
  speed below 1.0. Takes effect on the next `fsm_node` launch; no rebuild needed.
- **Checked:** upstream 2ad3419 + the `std::isnan` fix + patches 0001–0007 rebuilds the
  live yaml byte for byte, and 0008 still applies. The patches do **not** apply to raw
  upstream; that is expected (`apply_super_patch.sh`). No gate reads `max_acc`.
- **After the next follow flight:** `follow_speed_report.py BAG --max-vel 1.0` — does the
  commanded speed now reach 1.0, and does the ≥ 39 % `OPT_FAILED` rate change?

### Changed 2026-09-16/17 — max_vel 1.0, a fixed follow height, a latched descent guard (bench-gated, NOT flown)
**Full record: [`docs/fact-follow-speed-and-descent-guard.md`](docs/fact-follow-speed-and-descent-guard.md).**
Every number is re-derivable with `python3 drone-dtp/scripts/follow_speed_report.py BAG --max-vel 0.5
--stack-log …/stack.log`; run it after the next follow flight and compare.

**Why:** the 2026-09-15 20:01 `deploy_follow` flight (bag `~/dtp_datasets/20260915_2001_stageB_salvaged`,
logs `…/20260915_1955_deploy/`) followed for only 13 s in Offboard, at a median 0.36 m/s (commanded
max 0.496). Don't re-derive:
- **`max_vel` is a 3-D cap** (`exp_traj_optimizer_s4.cpp:158`), and every plan ends at rest
  (`goal_vel_en: false`; turning it on acts only on goals > 3.5 m away, `super_planner.cpp:726`).
- **The walker did 0.5–0.8 m/s.** The gap grew 4.4 → 6.5 m and the lock was lost.
- **The goal's altitude flapped.** `follow_planner` put the goal 2.77–3.07 m from the walker at
  z 0.5 (the target z reads 0.03–0.20 m, not the torso the code assumes). The mux descent guard
  (3.0 m radius, re-decided every tick) raised 32 of the 59 follow goals to the aircraft's altitude
  and left 27 at 0.5 (30 height jumps in 13 s), so SUPER spent ±0.2 m/s on vertical motion.
- **SUPER failed ≥ 39 % of its replans** (77 in 13 s at up to 15 Hz; every one at 473–474 s),
  almost all `generateBackupTrajectory return OPT_FAILED`. A failed replan keeps the old
  trajectory, which ends at rest (`fsm.cpp:69`). **Cause not established.** Trap: successful
  replans are an untimestamped, block-buffered `cout` — count only the timestamped failures.
- **Also seen:** after the loss, auto-reacquire locked a *different* track (3) 8 m away behind the
  aircraft. The mux gave SUPER a goal 6 m behind it for 0.6 s, in Offboard.

**What changed** (effective on the next stack + follower launch):
- **`max_vel` 1.0** (user). In `follow_test_ros2.yaml` (src + install), patch 0001,
  `deploy_follow.sh --max-vel 1.0` and the runbook. **`max_acc` stays 0.5 "for now" (user,
  2026-09-17)**, so expect the commanded speed to rarely reach 1.0 with goals ~2 m ahead.
  **Superseded 2026-09-21: `max_acc` is 1.0** (section above).
- **Option A:** `follow_planner_node/goal_z_min_m: 1.2` in `dtp_params.yaml`, so every follow
  goal is at 1.2 m, ~1.35 m above ground (real node: 178/178). **Engage Offboard at about 1.2 m**:
  engaged higher with anyone within 3 m, the latched guard holds the engage height.
- **Option B:** `command_mux`'s descent guard now **latches a floor**
  (`command_mux/descent_guard.py`):
  - trips as before, and holds the aircraft's altitude at the trip;
  - raises the floor only when the aircraft is > 0.3 m above it (largest measured tracking
    error +0.215 m);
  - releases after nobody has been within 3.5 m for 1.0 s, and resets on disarm;
  - `/mux/state` appends `| alt floor Z`.

  **Cost:** a goal may sit up to 0.3 m below the current altitude with a person near (was 0). It
  guards goals only: SUPER's path can still dip under an obstacle, and SUPER can snap a blocked
  goal downward (`fsm.cpp:230`).
- **Gates:**
  - `test_descent_guard.py`: 17 pass (plus fence/abort 21);
  - **`python3 drone-dtp/scripts/descent_guard_check.py` 11/11** (real node, **domain 86**). The
    pre-change node fails 7 of them, with 13 flips in 14 goals;
  - `fence_check.py` 13/13.

  **Re-run `descent_guard_check.py` after any `command_mux` change.**

### M5 started 2026-09-13 (second session) — desk and bench work through M5.5; nothing flown
The user asked for M5 "till M5.5 Stage A". Everything below is bench, desk or bag-replay. **No
M5 gate has passed on hardware.**

- **⚠️ `run.sh` ALWAYS started `fsm_node` on `click_smooth_ros2.yaml`**, and the DTP launch refuses a
  second `fsm_node`. A follow flight riding on it would silently have flown the click config, not
  `follow_test_ros2.yaml` (which carries the M5.2 envelope and the filter).
  - **`SUPER_CONFIG=follow_test_ros2.yaml RVIZ=false bash scripts/run.sh`** now selects it, via
    `m1_super_lidar.launch.py config_name:=`.
  - **The dashboard's Stack row now has a click / follow chip** (default click; 2026-09-13, fifth
    session).
  - `stage_a_preflight.py` now reads the loaded config off `/fsm_node`'s `config_name` parameter.
  - `super_bringup` was re-copied and rebuilt. The deployed copy had also been **missing
    `gemini_camera.launch.py`**.

- **✅ GEOFENCE BUILT 2026-09-14 — in `command_mux`, the user's decision (option 1 of
  `m5-m6-plan.md` M5.2).** PX4's `GF_MAX_HOR_DIST` 50 is inert (`geofence.cpp:354` needs a global
  home; `EKF2_GPS_CTRL=0` gives none), so the box lives at the goal level:
  `drone-dtp/.../command_mux/fence.py` (pure geometry) + eight `fence_*` params in
  `dtp_params.yaml`. **Default: the plan's 30 × 4 m box in `lidar_init`, x ∈ [−2, 28], y ∈ [−2, 2],
  on the boot heading (`fence_yaw_deg` turns it) — re-cut per venue.** Soft boundary = the edge:
  FOLLOW, NAV **and HOLD** goals are clamped 0.5 m inside it (a subject who leaves the box is
  followed to the edge and no further). Hard boundary = edge + 3 m (`fence_breach_land_m`): the
  **aircraft** past it raises a `FENCE` veto that **escalates to NAV_LAND** through the abort
  path (PX4 `GF_ACTION=land` semantics). The node **refuses to start** on a box that excludes the
  origin. **Not covered, by construction:** an RViz `/goal_pose` click and the GCS RC card go
  straight to `fsm_node`. `stage_a_preflight.py` FAILs on `fence_enable: false`.
  **Gate: `python3 drone-dtp/scripts/fence_check.py`** — the real node on a private DDS domain,
  13/13 (clamp, soft breach, hard breach → HOLD|FENCE → ABORT|LAND with NAV_LAND observed,
  latch, refusal, disabled path). **Re-run after any `command_mux` change.** Pure tests:
  `test_fence.py` (10) + `test_abort.py` (12, FENCE escalation added). **The soft clamp has
  flown (2026-09-29 12:32, `flight-log.md`); the hard boundary has not.** Full write-up, the three
  options and why this one: [`docs/fact-geofence-command-mux.md`](docs/fact-geofence-command-mux.md).
  **Since 2026-09-29 `dtp_params.yaml` sets `fence_x_max_m` 36.0** (section at the top of this
  file); the node default is still 28.0.

## Perception and target-manager settings

- **`detect_rate_hz` is 8.0** in the committed `dtp_params.yaml`, since 2026-09-12. Older notes
  saying 10.0 are stale. **`auto_reacquire_after_lost` was false from 2026-09-12 and is TRUE since
  2026-09-15**, at the user's instruction — so `stage_a_preflight --stage-b` now reports **NO-GO**
  on that line every run, by design, and `target_manager` logs a loud ERROR at startup. Neither is
  a fault. It does **not** re-lock the same person: after the coast expires it takes the *nearest
  confirmed* person in the 2.5–10 m band. The reasoning is in the `dtp_params.yaml` comment.
  *(2026-09-30: the acquisition band is 2.5–15 m with `lidar_range_enable` on, and an automatic
  re-acquisition is held to 10 m by `auto_reacquire_max_range_m`. See "Part B: LiDAR ranging
  past 10 m" above.)*
- **The range caps are 15 m since 2026-09-30, and only with the LiDAR hybrid on**
  (`max_depth_m`, `prediction_depth_gate_m`, `acquire_max_range_m`; node defaults 10.0).
  Bench-gated, not flown. → "Part B: LiDAR ranging past 10 m" above.
- **⚡ `perception_node` builds its organized cloud ON THE GPU since 2026-09-13 (`cloud_backend:
  cuda` in `dtp_params.yaml`; the node's own default is still `numpy`). Desk and bag replay only —
  not yet run on the aircraft.**
  - **Don't re-derive what cost the CPU: it was never JPEG decode (5 % of a core).** At 8 Hz the numpy
    depth → colour → z-buffer → 1280×720×3 chain was **90 %** (112 ms CPU / 38 ms wall per frame),
    and BoT-SORT's GMC ~18 %. Now `drone-dtp/.../dtp_perception/cuda_cloud.py` runs the chain in
    torch on the GPU YOLO already owns.
  - **Bit-identical to numpy** on 200/200 real frames (`scripts/bench_cloud_backend.py`).
  - **Replaying a bag through the real node:** **1.9 cores → 0.7** (`scripts/replay_perception.sh`,
    evidence in `~/dtp_datasets/20260913_cloud_backend_replay/`).
  - **Per-frame wall time did NOT improve** (GPU shared with YOLO), so it is not a reason to raise
    `detect_rate_hz`.
  - **The node guards itself:** the first 5 frames are built both ways with numpy flown until they
    agree, and it reverts to numpy at ERROR on disagreement, 3 CUDA errors, or no CUDA at start.
    Without libcudss on `LD_LIBRARY_PATH` torch can't load — that is the "no CUDA at start" case,
    and `test_cuda_cloud.py`'s GPU tests *skip*.
  - **Trap found on the way:** a GPU path that bypasses `DepthSource.range_image()` must still advance
    `last_stamp_s`, or the depth/colour skew guard blinds every frame (`_note_rasterised`).
  - **End to end:** on an identical frame schedule, numpy vs cuda targets differ no more than two
    numpy runs differ from each other (p95 1.65–2.18 cm vs 1.71 cm).
  - **Also found — node-level bag replay is NOT deterministic, and M6.2 depends on replay.**
    - At 1× the first TensorRT inference (~2.5 s stall) shifts the frame schedule;
      `WARM_S=60 RATE=0.25 scripts/replay_perception.sh` fixes that.
    - Even then, two runs of the SAME code diverge after a few dozen frames (likely
      colour→newest-depth pairing).
    - So compare an arm against a second run of itself, never against zero.

    Plan: `docs/m5-m6-plan.md` M5.3 items 4–5 and M6.2.

See also: `flight-log.md` (how these behaved in the air), `super-planner-rogmap.md` (the SUPER envelope in `follow_test_ros2.yaml`).
