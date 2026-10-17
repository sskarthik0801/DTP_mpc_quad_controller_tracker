# Build plan: the veto's height band, and LiDAR ranging past 10 m

Written 2026-09-30 for the next session. **Both parts were built the same day: Part A bench-gated, Part B bench-gated and replay-scored. Neither is flown.** Both came out of the
2026-09-29 12:32 flight (`.claude/rules/flight-log.md`): the 3 m/s close pass, and the walker lost
past 10 m. The three close-pass fixes already built are in `.claude/rules/dtp-follow-behaviour.md`,
"three fixes for the 09-29 close pass".

Do **Part A first**: it is small, it is a safety hole, and fix 2 of the close-pass fixes depends
on it. Part B is a perception feature, and it needs a go/no-go measurement before any code.

Line numbers are as of 2026-09-30. Check them before editing.

---

## Part A — the proximity veto's height band (small, ~30 min)

**Built 2026-09-30, bench-gated, not flown. Record: `dtp-follow-behaviour.md`.**

### The problem
`command_mux` models each person as a vertical cylinder. The radius is 1.5 m, or the pass-by
radius when fix 2 applies. The height is **±`proximity_veto_half_height_m` (2.0) around the point
perception reports**. A path or braking position outside that height range is treated as clear
of the person.
- The constant and its reasoning: `DEFAULT_PROXIMITY_HALF_HEIGHT_M = 2.0`
  (`command_mux_node.py:368`), declared at `:632`, read at `:716`.
- It is used in two places: the braking envelope (`_brake_violation`, `:1367`) and the path
  check (`_proximity_violation`, `:1416–1418`).
- **Its comment assumes the reported point is the body centroid** ("2.0 m above/below the
  centroid covers head height (~0.8 m above it) plus ~1.2 m of prop clearance").
- **In flight the reported point is at foot level.** Close-pass window
  (`drone-dtp/scripts/close_pass_fixture_20260929.json`): tracked z **0.26–0.72**, aircraft z
  **2.27–2.73** (`lidar_init`). The cylinder's top was 2.3–2.7 m, where the aircraft was.
- **Result on 09-29:** the braking envelope skipped the walker from t 454.44 to 456.54. The path
  check only counted them because the goal's z (~2.0) kept the path's z-range inside the band.
- **When it bites:** the aircraft is higher exactly when someone is near. A HOLD climbs 0.75 m.
  The descent guard latches a floor with a person within 3 m. The 09-26 flights flew at
  1.8–2.3 m AGL on that latched floor.

### The change (recommended: option A1)
- **A1 — one parameter, no code.** In `dtp_params.yaml` under `command_mux:`, add
  `proximity_veto_half_height_m: 3.0`, with a comment giving the 09-29 numbers above.
  - With a foot-level point, the cylinder then reaches ~3.0 m above the ground: head (~1.8 m)
    plus ~1.2 m of prop clearance.
  - The extra 1 m below is harmless: the aircraft is never below a person.
  - The node default stays 2.0, as flown, per the switch convention.
- **A2 — asymmetric, in code:** separate `proximity_veto_above_m` (3.0) and
  `proximity_veto_below_m` (2.0). Use A2 only if A1's lower extension causes a veto somewhere
  (it should not).
- **Worth checking while there:** does any perception path report a point that is *not* at the
  feet? `perception_node` logs "track N has never had a height; assuming the torso is at camera
  altitude (2.39 m)" (seen in the 09-29 perception log). With that fallback the point sits at
  the aircraft's own height, and ±3.0 still covers the person, so it is conservative. Note it in
  the record either way.
- **Not in scope:** changing *what* perception reports as z. That is a perception change with
  its own gates.

### Tests
1. **Add cases to `drone-dtp/scripts/close_pass_check.py`** (domain 95). `MuxWorld.z` sets the
   aircraft height, and the person z is 0.2.
   - **M9 at half-height 2.0:** the aircraft at z 2.7 (2.5 m above the point), hovering, the
     far-side goal also at z 2.7, the walker approaching as in M1/M2 with the pass-by radius on.
     **No veto**, which reproduces the hole.
   - **M10 at half-height 3.0:** the same scene. **Veto at > 8.5 m**, as M2.
   - **M11 at 3.0, the braking envelope:** 2.5 m/s at a standing walker, the aircraft at z 2.7,
     the goal parked 3 m short (like `brake_envelope_check` B2 but high). The envelope veto
     fires at ~5.3 m. At 2.0 it does not fire before the path check does.
   - **M6-style replay at 3.0:** replay the fixture with `proximity_veto_half_height_m:=3.0`.
     The first PROXIMITY must be at or before the pass-by-only time (454.63). Record which check
     fired: from ~454.4 the braking envelope should now see the walker.
   - **No new vetoes (M3 at 3.0):** 1 m/s behind a walker going away with the aircraft 1.0 and
     2.0 m above the point; a walker passing 3 m to the side.
2. **Re-run the `command_mux` gates, one at a time:** `fence_check`, `descent_guard_check`,
   `terrain_check`, `engage_gate_check`, `brake_envelope_check`, `close_pass_check`. Also the
   unit tests + flake8 from inside the package (`cd drone-dtp/ros2_ws/src/command_mux && python3
   -m pytest -q test/`). With A1 the existing gates run the node default (2.0), so they only
   prove nothing else moved; the new cases prove the fix.
3. **Print it in the Stage B preflight** (`stage_a_preflight.py`, the "close-pass fixes" line)
   so a bag records the half-height it flew.

### Records to update
`dtp-follow-behaviour.md` (the close-pass section's "Not fixed … the veto's height test" bullet
becomes a record), `CLAUDE.md` Latest changes, and `flight-log.md`'s 09-29 fix list (fix 5).

---

## Part B — ranging the person with the LiDAR past 10 m (bigger; measure first)

**Built 2026-09-30 (Steps 0-3), bench-gated and replay-scored, not flown. Record: `dtp-follow-behaviour.md`.**

### The problem
Past 10 m the follower is blind by design. Three caps, all 10.0, deliberately tied together:
- `perception_node` `max_depth_m` (`perception_node.py:650`) masks every depth sample beyond it
  in `_sample_patch` (`:1533`, the mask at `:1597`);
- `prediction_depth_gate_m` (`:655`) switches the CTRV forecast off past it;
- `target_manager` `acquire_max_range_m` (`target_manager.py:317`); its docstring (~line 85)
  explains why it equals the forecast gate.

Depth comes from the Gemini 336L stereo (`depth_source: camera`). Its error grows with range².
On 09-29 the range jumped up to 3.3 m between frames at 7–8 m, and perception's gate
(`accept_range`, `:505`, used at `:1668`) rejected those frames. Airborne detection on 09-29:
82–85 % of frames at 4–10 m, **23 % past 10 m**. The camera frames show the walker plainly at
11–15 m (bag t 448, 451, 502.5).

### What was measured (2026-09-30, `bags/stage_b_2026-09-29_122511`)
In `/cloud_registered_body` (FAST-LIO keeps 1 point in 3, `point_filter_num: 3`), the points on
the walker per 0.1 s scan:
- 3–9 m: usually 0 (51–85 % of scans empty), up to 14–23;
- **10–17 m: 1–5**, spread over 0.5–1.7 m of body height.

Script: [`evidence/lidar-range-2026-09-30/lidar_on_person.py`](evidence/lidar-range-2026-09-30/README.md).
It odom-transforms each scan to `lidar_init` and counts the points within 0.6 m horizontally of
the walker, 0.3–2.1 m above the local ground. Beyond 10 m the walker was found without the camera,
as the moving cluster in their lane.

### Why the existing LiDAR path will not do it as written
`depth_source: lidar` exists (`depth_source.py:763`, `LidarDepthSource`). It accumulates 0.2 s of
`/cloud_registered_body` (`LIDAR_ACCUM_S`, `:194`) through `T_camera_lidar` into the colour frame.
Three problems:
1. **It needs `MIN_ROI_SAMPLES` = 6 samples in the torso patch** (`:139`,
   `perception_node` `min_patch_samples`). At 10–17 m it would see ~2–5 on the whole body.
2. **It assumes FAST-LIO's body frame IS the LiDAR frame.** That is no longer true: the deployed
   `mid360.yaml` has `extrinsic_T: [-0.011, -0.02329, 0.04412]` (~4.8 cm). Its startup check
   (`_on_body_frame_result`, `:900`) will log the mismatch and tell you to compose the LiDAR→IMU
   extrinsic into `T_camera_lidar`. Do exactly that: `static_calib_tf` already composes the same
   numbers (`lidar_to_body_t` in `dtp_params.yaml`). The module docstring's "`scan_bodyframe_pub_en:
   false`" note is also stale: the topic is published now.
3. **It replaces the camera everywhere.** That would lose close range, where the LiDAR sees
   nothing of the lower body (its floor is −7°).

### Step 0 — GO / NO-GO: does the detector even see a person at 10–17 m?
Ranging does not help if YOLO drops the box first. The 09-29 bag has no 2-D boxes
(`/dtp/detections_2d` is only published while subscribed). Re-run the flown model on the frames
with `drone-dtp/scripts/flight_video_detect.py`: yolo11s TensorRT, conf 0.25, imgsz 640, the flown
settings. Stage 1 of `flight_video.sh` makes its telemetry pickle; see `docs/flight-video.md`.
- **Windows:** bag t 443–453 (the walker 10–17 m away, the aircraft hovering) and 500–506
  (9–16 m).
- **Report:** the detection fraction and confidence by range bin, 8–10 / 10–12 / 12–15 /
  15–17 m. Use the LiDAR cluster above as the range truth past 10 m.
- **GO if ≥ ~70 % of frames detect the walker out to at least 13–15 m.** Otherwise the limit is
  the detector: consider a larger imgsz engine or a crop around the forecast. **Stop and report
  before building the LiDAR part.**

### Step 1 — the design: camera near, LiDAR far (a hybrid, not a switch)
- **Keep the camera's organized cloud as today for close range.** For a detected box whose
  camera range is missing, beyond ~8 m, or rejected by `accept_range`, **take the range from
  the LiDAR**:
  - accumulate `/cloud_registered_body` in `lidar_init` (via odom, as `LidarDepthSource` does)
    over a longer window, ~0.4–0.6 s;
  - project into the frame at its stamp through the **composed** camera←body transform;
  - take the points inside the **whole box** (shrunk ~10 % at the edges), not the torso patch;
  - use the nearest-surface cluster (`CLUSTER_BAND_M` logic, `depth_source.py:157`), then its
    median range.
- **Minimum points for a LiDAR range: 2–3**, not 6. Each point is ~2 cm accurate. The danger is
  a background point inside the box, which the nearest-cluster rule and the forecast gate address.
- **Motion over the window:** at 1.4 m/s the walker moves ≤ 0.8 m in 0.6 s. Either weight the
  newest sweep, or shift older points by the track's velocity before taking the median.
- **Bearing stays the camera's** (it is precise). Only the range comes from the LiDAR.
- **The handover.** The stereo may be biased against the LiDAR at 6–10 m. Measure the bias in
  the overlap on the bag. Hand over with hysteresis (e.g. to LiDAR at > 9 m, back at < 7.5 m),
  so `accept_range` does not reject the switch as a jump.
- **Calibration at range:** the deployed extrinsic's known error is ~1° or less. That is ≤ 0.3 m
  sideways at 15 m, inside a whole box. Re-score with `score_alignment` on far structure if in
  doubt (`.claude/rules/dtp-calibration.md`).
- **CPU:** measure with `measure_load.py`. The camera path is ~0.7 cores on the GPU backend. A
  LiDAR projection of ~7k points per sweep is small, but check it.

### Step 2 — raise the caps together, to 15 m
In one change: `max_depth_m`, `prediction_depth_gate_m`, `acquire_max_range_m` → 15.0. The
sensor caps already say 15: `max_range_m`, `depth_source.MAX_RANGE_M`, `fusion_node.max_depth`.
- Read `target_manager.py`'s docstring (~line 85) first. The acquire cap is pinned to the
  forecast gate on purpose.
- **⚠️ The D13 consequence.** `auto_reacquire_after_lost` is true (user, 2026-09-15). After a
  LOST it takes the **nearest confirmed person in the acquisition band**. At 15 m that band holds
  more bystanders. Say so to the user, and consider a smaller acquire cap for auto re-acquire
  than for a coast re-lock.

### Step 3 — test offline on the 09-29 bag
- Replay through the real perception node: `WARM_S=60 RATE=0.25 scripts/replay_perception.sh`
  (`dtp-follow-behaviour.md` "Perception": node-level replay is not deterministic, so compare
  each arm with a second run of itself).
- **Metrics:** `drone-dtp/scripts/flight_timeline.py` section 5 (detection by range), plus the
  range jumps rejected by `accept_range`. The baseline is 23 % past 10 m.
- **Pass:**
  - past 10 m, well above 23 % (set the bar from Step 0's detector ceiling);
  - no rise in rejected jumps at 6–10 m;
  - the LiDAR range within ~0.3 m of the LiDAR-cluster truth;
  - no range on a background object (the metal fence at y ≈ −2.2 and the tree line are both in
    that bag).
- **Unit tests** for the new source in `dtp_perception/test/`; `test_cuda_cloud.py` is the
  pattern. flake8 is already red in this package (E501), so do not make it worse.

### Step 4 — before it flies
- Bench bring-up (`drone-dtp/scripts/bench_bringup.sh`), and both preflights GO.
- Observe-only first (Stage A): walk out to 15 m and back, and compare the ranges with the LiDAR
  map.
- Then a Stage B flight with the pilot ready at the 10 m mark. Keep the close-pass rule: the
  walker does not walk back at the aircraft in Offboard. See the recommendation in `CLAUDE.md`.

### Records to update
`dtp-follow-behaviour.md` ("Measured 2026-09-30 — why the LiDAR does not simply extend the 10 m
range" becomes a build record), `CLAUDE.md` Latest changes, and the `depth_source.py` docstring
(its two stale config facts).
