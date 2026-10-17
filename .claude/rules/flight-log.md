---
paths:
  - "docs/flight-*.md"
  - "docs/stage-a-flight-results-*.md"
  - "docs/m5.1-one-graph-runbook.md"
  - "docs/m5.4-m5.5-field-runbook.md"
  - "docs/handoff-*.md"
  - "drone-dtp/scripts/field.sh"
  - "drone-dtp/scripts/deploy_follow.sh"
  - "drone-dtp/scripts/bench_bringup.sh"
  - "drone-dtp/scripts/stage_a_*"
  - "drone-dtp/scripts/flight_timeline.py"
  - "drone-dtp/scripts/analyze_*.py"
  - "drone-dtp/scripts/flight_video*"
  - "scripts/desk_check.sh"
  - "scripts/gcs/flightday.py"
  - "scripts/px4/estimator_preflight.py"
  - "scripts/px4/m51_one_graph_check.py"
---

# Flight log and field operations

Scope: what happened on each flight and field session, newest first, and the M5.1 / M5.4 / M5.5 procedures. The day's script is [`docs/flight-day-runbook.md`](../../docs/flight-day-runbook.md). The 2026-09-15 20:01 `deploy_follow` flight is analysed in `dtp-follow-behaviour.md`. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

### Flight 2026-10-02 20:03 EDT — first flight on the rollback; the geofence held on both axes, park 0009 worked at the takeover, and a second far-side close pass (2.6 m)
Read from `bags/stage_b_2026-10-02_200220` (123 s, intact) on 2026-10-02 (20:50 session), with the
node logs of the 20:01 launch (`python3_17407` command_mux, `python3_17400` target_manager,
`python3_17395` perception, `python3_17182` offboard bridge, `follow_planner_node_17405`,
`fsm_node_17160`). Video: `bags/videos/stage_b_2026-10-02_200220_geofence.mp4`. Bag t 0 = 20:02:21.
Before it, `bags/stage_a_2026-10-02_194304` (19:43–19:54, 664 s, 9 GB) holds two 33 s manual hovers
at 1.5 m, observe only, lock held 50 s, detection 100 % in the air.
- **Config, from the node logs.** Branch `rollback-pre-0926` with the 10-01 ports: band 3–4 m,
  `max_vel` 3.0 / `max_acc` 2.0, braking envelope ON, veto band ±3.0 m, terrain height ON,
  geometric, yaw hold ON, park ON, fence 30 × 4 m (x −2…28, y ±2). Bridge: `acc_ff` ON,
  `setpoint_only_in_offboard` ON, yaw servo off. **First flight of patch 0009 (park), of the
  braking envelope and the 3.0 m veto band on this code, and of the 09-30 calibration on it.**
  The 09-30 close-pass fixes (pass-by check, speed-widened veto, flyable HOLD) are not on it.
- **Timeline.** Armed 20:02:37 (t 16.0, Stabilized), Position t 28.8. **Offboard 20:02:56–20:04:04
  (t 35.6–103.6, 68 s).** The pilot took Position at the fence edge, landed at t 109, disarmed
  20:04:12 (t 111.3). Light: mean grey 71/255 in the first minute, 56 in the second; detection
  85 % → 54 %. The frames after t 99 are nearly black.
- **Health.** FAST-LIO stamp lag p95 61 ms, no stall. PX4 local position vs LIO: 6 cm median,
  46 cm worst at the 3 m/s sprint (t 96). `perception_ok` never false. SUPER snapped every goal
  (255/255) but by ≤ 0.22 m in z and ≤ 0.28 m in xy. The engage gate worked: SUPER's first plan
  (PlanFromRest, t 35.67) started 5 cm from the aircraft.
- **t 36–47: a clean follow** from x 0.7 to 10 at a walking pace, range 3.5–4.2 m, z 1.2–1.4,
  ToF 1.3–1.6 m.
- **t 43–47: the walker jogged away, and the published range sat at 3.6–3.7 m the whole time.**
  The LiDAR (`/cloud_registered_body` projected through the odometry) shows a person-height column
  moving from x 12 to 17 in 2.5 s (~2 m/s); the frames show the figure shrinking from ~4 m to
  ~9 m. **The stereo depth saw it:** re-running the detector offline on the bag's frames
  (`yolo11s.engine`, imgsz 640, conf 0.25, BoT-SORT) and sampling the torso patch on the recorded
  depth gives 4.1 m at t 43.0, 5.4 at 44.1, 7.2 at 45.4, 9.3 at 46.4 — a smooth ramp, 5–75 valid
  pixels, no holds. **What pinned the number is the frozen predictor's clamp-then-EMA**
  (`detection.py` `_limit_motion_step` `max_step_m` 0.35, then `_smooth_xz` `smooth_alpha` 0.25,
  both applied in the WORLD frame to a confirmed track): the published position can move at most
  0.25 × 0.35 = **0.0875 m per processed frame, 0.92 m/s at the flown 10.5 Hz** (`detect_rate_hz`
  8.0 configured). A simulation of that filter on the offline raw depth reproduces the published
  track within 0.5 m and a flat ~4 m range. The published x advanced at 0.87 m/s — the slew limit —
  and the aircraft, following that goal, moved at the same 0.8–0.9 m/s, so the gap read a constant
  3.7 m while the real gap opened from 4 to 9.6 m. The range-jump gate (`accept_range`) and the
  "unranged" hold were NOT involved: neither logged a hold on track 2, and the forecasts stayed
  17–19 points. **The jump to 9.4 m:** the tracker dropped id 2 for ~1 s (sparse detections at
  dusk, conf 0.3–0.5), `max_missed_secs` 0.5 cleared its history, and the next detection (new ids
  0/1 at t 46.67, id 2 again at t 47.38) went out raw — the clamp is skipped for an unconfirmed
  track and the EMA on an empty history returns the raw sample. target_manager saw id 2 "reappear
  OUTSIDE the re-acquisition band", LOST at t 47.8, auto re-acquire at t 48.2 at 9.35 m. **That
  re-lock was correct**: the LiDAR column at t 47–50 is at x 18–19, y −1.9. **The limit is
  general:** any walker faster than ~0.9 m/s in the world frame outruns the published position,
  and the shortfall grows until the track is dropped and re-created. A brisk walk is 1.4 m/s.
  **Fixed on the bench the same evening (user): `max_step_m` 2.0 and `smooth_alpha` 0.5 as new
  `perception_node` parameters, replay gate `max_step_replay_check.py` 9/9, not flown**
  → `dtp-follow-behaviour.md`.
- **t 48.2–50.5: a 0.56 s HOLD set the altitude for the next 8 s.** The HOLD between the LOST and
  the re-lock (t 48.15–48.71) commanded +0.75 m (goal z 2.08). SUPER flew it; the aircraft was at
  2.43 m at t 50.5 when the descent guard latched a floor there ("person (track 2) within 3.34 m"
  — the goal was 3.0 m from the walker). The approach below was flown at 2.0–2.4 m until the
  floor released at t 56.8. It is the 09-21 HOLD-climb problem through the re-lock path; the
  09-21 fix covers a floor *raised* by a HOLD, not one *latched* at the HOLD's height.
- **⚠️ t 50.4–54: the far-side close pass, 2.58 m.** The walker had stopped at x ≈ 18.5 and
  turned to face the aircraft (frames t 49.5, 51.0). At t 50.36 the follow goal jumped from
  (16.5, −1.5), 3 m short of them, to (20.2, 1.0), 1.3 m past them (`follow_planner` t 52.07:
  "goal 20.4 0.3"). The aircraft, already at 2.0 m/s toward the first goal, reached 2.9 m/s.
  - **The braking envelope fired at t 51.5** (HOLD|PROXIMITY, "track 2 at 1.30 m from the braking
    path at +1.84 s (2.67 m/s: stop 3.12 m + overshoot 1.07 m)"), 6.0 m from the walker, predicting
    a 4.2 m stop. The HOLD goal was the aircraft's own position (12.75, −0.67).
  - **SUPER did not fly the HOLD.** Its replans had been failing since t 45 (21–31
    `OptimizationExpTrajInPolytopes` failures per 5 s through t 65, 45 backup failures at t 48.8).
    It switched to its backup trajectory at t 52.2 (bridge log) and the aircraft travelled 5.9 m,
    stopping at (18.6, −0.1) at t 54 — 1.7 m further than the envelope's prediction. The mux
    re-anchored the hold at t 52.65 ("drone is 3.2 m from where the hold was latched").
  - **Closest point t 53.1:** aircraft (17.2, −0.3, z 2.06), walker (18.4, −2.6, tracked point
    z 0.55): **2.58 m horizontal, ~1.5 m above the torso point**, ToF 2.5 m AGL. The walker
    side-stepped from y −2.0 to −2.9 during t 52.5–53.5, which opened the gap from ~2.2 m.
  - `follow_planner` withheld the goal ("transit … inside the standoff floor") only from t 51.87,
    after the veto, and then the far-side goal had already been adopted.
  - **The same three causes as 09-29.** Goal "behind" a walker facing the aircraft is their far
    side; the HOLD at speed is a stop SUPER cannot plan; the braking envelope alone shortens the
    pass but does not stop it. Not as close as 09-29 (0.9–1.5 m), because the walker stepped
    aside and the aircraft was 2.4 m up.
- **t 54–75: fence test, sideways — held.** The walker walked out to y −10.8, 9 m beyond the edge.
  125 of 255 goals were clamped (mux WARN every 2 s, e.g. "(15.51, −12.21) is 10.21 m outside …
  clamped to (15.51, −1.50)"); the aircraft held at y −1.3…−1.7, x 17.5–19, yawed −90…−107° to
  keep the camera on them. LOST at t 67.3 (9.3 m, unknown rays) and t 74.7 (7.3 m) while they
  were outside; re-locked at 9.3 m and 4.6 m on their return. The HOLDs climbed +0.75 each time.
- **t 77–89:** follow resumed at 3–4 m, z 1.2–1.4, ToF 1.7–1.9, as the walker came back along
  y −5 → −0.7 and headed +x.
- **t 90.6: LOST at 4.0 m with the walker in frame** (oracle class LOST, bearing −5°; perception
  t 89.17 "unranged, 3 inliers < 6"). Dusk. HOLD + climb to 1.96; re-lock t 93.3 at 9.3 m; LOST
  again t 94.8 at 9.8 m. **The aircraft then sprinted to 2.95 m/s** (t 96) toward the stale goal
  (25.2, −1.5), SUPER switched to backup at t 96.9, and it stopped 4.8 m past the HOLD point
  (20.0) — the same unflyable-HOLD overshoot, with nobody near. Re-lock t 96.9 at 6.7 m.
- **t 97.5–100: fence test, forward — held.** Goals clamped to x 27.5; the aircraft reached x 28.1
  at ~1 m/s (mux t 100.1: "aircraft at (28.01, −1.62) is 0.01 m outside the box (hard boundary at
  3.00 m)"). The walker went on to x 33–34, 5–6 m beyond. **LOST at t 100.1 at 5.4 m**, in frames
  that are nearly black (perception t 101.17 "3 inliers"). HOLD at the edge, +0.75 climb to 2.4 m,
  no re-acquire for 11 s (they were re-locked at 6.3 m only after landing, t 111.7).
- **t 103.6: the takeover parked SUPER — first flight of 0009, and it worked.** Engage gate SHUT
  at 103.61, park sent, `fsm_node` "PARKED in state FOLLOW_TRAJ: goal dropped, pos_cmd silent".
  Last `pos_cmd` 103.60; one fsm WARN line after it; no re-planning until the stack was stopped
  at 20:04:42. The bridge stopped the setpoint stream the same tick.
- **After landing (t 114.9–115.4):** three HOLD|PROXIMITY flickers as the walker came to the
  aircraft (0.82 m). Benign; the gate was shut.
- **Open, the user's call:** the far-side goal / unflyable HOLD pair, now flown twice; the HOLD
  climb latching the floor through a re-lock; following after ~20:00 in October (two in-view
  losses at 4–5 m). None of the 09-30 close-pass fixes is on this branch.

### Flight 2026-09-29 12:32 EDT — first flight of the 4–5 m band and the 09-27/28 changes; the lock went past 10 m whenever the aircraft stood still
Read from `bags/stage_b_2026-09-29_122511` on 2026-09-29 (13:40 session), after the user reported
"the drone lost track of the person after it went beyond 10 m". The bag is intact (metadata written).
Video: `bags/videos/stage_b_2026-09-29_122511.mp4` (made 12:52).
- **Config, from the node logs.** Band 4–5 m, speed-scaled standoff ON, braking envelope ON,
  terrain height ON, geometric, yaw hold ON. Bridge: `acc_ff` ON, `setpoint_only_in_offboard` ON,
  yaw servo off. **This was the first flight of the band, the standoff, the envelope and both bridge
  changes.**
- **Timeline.** Armed 12:32:13 (bag t 421.3, Stabilized), Position t 430.5. **Offboard
  12:32:30–12:33:31 (t 438.0–499.5, 61.5 s).** The pilot took Position at 9.2 m range, flew back to
  x ≈ 5 at up to ~6 m/s, landed, and disarmed at 12:33:44 (t 512.5). Logs: `follow_planner_node_7106`,
  `python3_7098` (target_manager), `python3_7112` (command_mux), `python3_7096` (perception),
  `python3_6533` (offboard bridge).
- **Past 10 m the follower is blind by design.**
  - `perception_node` `max_depth_m` 10.0 drops every depth sample beyond 10 m (camera z) in
    `_sample_patch`. A person there is never ranged, so no target is published.
  - `target_manager` `acquire_max_range_m` 10.0 and `prediction_depth_gate_m` 10.0 are pinned to
    the same number.
  - Depth comes from the Gemini stereo (`depth_source` camera).
  - Airborne detection by range (`flight_timeline.py`): 82–85 % at 4–10 m, 23 % past 10 m (12 of
    52 frames).
  - The colour frames show the walker plainly at ~11–15 m (t 448, 451, 502.5).
- **The walker got past 10 m each time the aircraft stood still.**
  1. **t 442.5–453.5.** A 1.2 s coast at ~6 m ended in LOST at t 443.5, and the mux held
     (HOLD|NO_SOURCE) at x 4.8 for 9.5 s. The walker walked out and back, and was re-locked at
     9.3 m. The cause of that first dropout is not established: the walker is in frame at t 443.5.
  2. **t 470.5–474.** Coasting at 9.2–10.1 m ended in LOST at t 472. The aircraft ran on ~4.6 m on
     its last follow plan (see below) and re-locked at 5.0 m.
  3. **t 489.0–499.3: the geofence.** `fence_x_max_m` is 28.0 and the margin 0.5, so every FOLLOW
     goal was clamped to x 27.50.
     - The walker walked on from x 31.4 to 36.8, and the range grew from 5.1 to 9.2 m.
     - **`/mux/state` read FOLLOW|OK throughout.** The clamp shows only in the goal's x and in a
       `geofence: FOLLOW goal … clamped to (27.50, …)` WARN every 2 s in the mux log.
     - **The same day the fence went to x 36** (user): `dtp-follow-behaviour.md`.
     - The pilot took over at 9.2 m. The lock went at 10–11 m as the walker walked on.
  - **The loop.** A 1.2 s dropout → LOST → HOLD in place. A walker at 1 m/s then passes 10 m within
    ~4–5 s of a 5–6 m follow, and cannot be re-acquired until they come back inside 10 m. In steady
    follow the flown range was 4.0–6.8 m, so the 4–5 m band leaves little margin to the cap.
- **⚠️ A close pass at 3 m/s, t 453.5–458.5 (12:32:45–50).**
  - **The goal.** The walker had turned back and was walking toward the aircraft.
    `follow_planner` put the goal on the walker's far side: (16.7, 1.6) with the walker at
    (14.2, −1.6) and the aircraft at (4.9, −1.0). It withheld the goal ("transit … inside the
    standoff floor") only from t 456.5.
  - **The approach.** SUPER flew at up to 3.0 m/s. The mux went HOLD|PROXIMITY at t 456.5, at
    4.0 m range. The aircraft still travelled ~3.2 m more and stopped at (10.9, −0.3), 2.4–2.6 m up
    in `lidar_init`, then yawed to −118°.
  - **The distance.** The last detection had a horizontal gap of 2.1–2.3 m (t 457.0; the walker's
    head is at the bottom edge of that frame). Nothing is measured after that. The minimum is
    inferred as **0.9 m if the walker kept walking, 1.1–1.5 m if they stopped**. The aircraft was
    ~2.3 m above the ground (ToF), ~0.5 m above head height.
- **Why: three causes stacked** (read the same day from the bag and source; read-only, nothing
  replayed on the real nodes):
  1. **The goal was on the far side.** `follow_planner_node.hpp:1332` puts the viewpoint "behind"
     the walker (`heading + π`). For a walker heading at the aircraft, that is their far side,
     and the near-side bias (±45°, `:1336`) cannot bring it back. The speed-scaled standoff
     pushed it to 5.3–6.0 m, and the fence clamped y to 1.5.
  2. **Both path checks passed until it was too late.**
     - `follow_planner`'s `TransitCrossesKeepOut` (`:1533`) tests the straight path against the
       walker's **current** position with a fixed **2.0 m** (`transit_clearance_m_`, `:279`).
       It does not use the forecast, the 2.5 m floor or the speed-scaled floor. Replayed: 2.46 m
       at t 454.0, which is under 2.5, so a floor-based check would have withheld the first
       goal.
     - The mux's path veto is a fixed **1.5 m** radius. It fired at t 456.2. Its speed-scaled
       `reach` (6.65 m then) feeds only the descent guard and the standoff, not the veto.
     - **The braking envelope never counted the walker.** Its height test is ±2.0 m around the
       tracked point, which sits at foot level (z 0.3–0.6). The aircraft was 2.0–2.26 m above
       it (`command_mux_node.py:1309`).
  3. **HOLD gave SUPER a stop point it could not fly.** `_hold_goal` latches the odom position
     at entry. At 2.5 m/s that point is inside the stopping distance, so every replan failed
     ("ExpOpt … Omg or thr or Pos violation"). SUPER kept its committed plan, then switched to
     its own backup (`trajectory_flag` 2, t 456.46). It stopped ~3.8 m past the HOLD point.
     SUPER's replans had also all been failing since t 455.05 (32 failures), before the HOLD.
     That cause is not established.
- **The same HOLD failure at t 468 and 472.** The aircraft was at 2.2 and 1.75 m/s. SUPER
  finished its last follow plan and stopped at the old follow goal, 3.4 and 5.3 m past the HOLD.
  The goal was accepted (`goal_status` 1). It was not rejected, snapped or deadbanded.
- **Fixes 1–3 were built 2026-09-30 and are ON (user):** `dtp-follow-behaviour.md`, "three fixes
  for the 09-29 close pass". Replayed on this event by `close_pass_check.py`:
  - fix 1 withholds the far-side goal flown at t 453.48, but only by 0.15 m;
  - fix 2 vetoes at t 454.63 instead of 456.21.

  **Fix 5 (the veto's height test) was built 2026-09-30, not flown,** as a wider band rather
  than from the head: `proximity_veto_half_height_m` 3.0. The tracked point is the lower body
  (hip/thigh), not the feet. Replayed: the first veto is unchanged, t 454.63 from fix 2, and the
  braking envelope becomes a second layer from ~456.1 (at 2.0 it was blind until 456.5). Record:
  `dtp-follow-behaviour.md`. **Fix 4 (no far-side goal while the walker approaches) is not
  built.**
- **Candidate fixes as first ranked** (the effects are replay or inference):
  1. The transit check against the forecast, with at least the floor: small. It would have
     withheld the first far-side goal.
  2. The mux's `reach` as the veto radius: small code, but a design choice (more vetoes). It
     would have fired ~1.3 s earlier, at 0.1–0.3 m/s.
  3. A flyable HOLD: odom plus the stopping distance (small), or the bridge braking on
     HOLD|PROXIMITY (design). It cuts the 468/472 overruns to ~1.5–2 m.
  4. No far-side goal while the walker approaches: design.
  5. The veto's height test measured from the walker's head, not the tracked point: small.

### Flights 2026-09-26 19:57 and 20:12 EDT — first flights of max_vel 3.0 / max_acc 2.0, the engage gate and terrain-relative height
Read from the bags on 2026-09-26 (session that evening). Both bags are intact, with metadata written
and the `.db3` page count matching. The Jetson rebooted between flights (battery swap) and ~4 min
after the second. Config: `max_vel` 3.0, `max_acc` 2.0, engage gate ON, `terrain_follow_enable`
ON, geometric goals. **None of the three had flown before.**
- **Both flights, what worked.**
  - **The engage gate.** SUPER published nothing until Offboard, and its first plan started from
    the aircraft: `pos_cmd` was 0.1 m / 0.0 m from the aircraft at the switch (09-21 20:05: 3.0 m).
  - **The terrain-relative height tracked the slope.**
    - Flight 1: the ground rose ~0.5 m along the follow and ToF held 1.78–1.95 m.
    - Flight 2: the ground fell ~0.45 m and ToF held 1.9–2.3 m.
  - **But both flew at the descent guard's floor, not at 1.35 m AGL.** The floor latched before
    Offboard:
    - flight 1 at the engage height (1.76 m);
    - flight 2 during the pilot's Stabilized climb to 2.0 m (floor 1.20 → 1.59 → 1.91). The pilot
      came down to 1.32 m before engaging, but the floor stayed.
- **⚠️ New: SUPER snaps every goal above z 2.25 m (`lidar_init`) down to 2.25.**
  `/planning/goal_status` shows `requested_p` 2.32–2.86 → `goal_p` 2.25 in both flights (ROG-Map's
  virtual ceiling). With terrain-relative height on rising ground this is reachable. In flight 1
  the requested goals reached 2.55 at x ≈ 23 (ground +0.7–0.8), and the aircraft flew 2.27–2.35.
  The allowed height above ground shrinks as the ground rises.
- **Speed.** The follows themselves ran 0.8–0.95 m/s. The new envelope showed at the engage and
  resume, when the walker was 5–7 m away:
  - flight 1 reached 2.3 m/s (EKF) and got 0.7–0.8 m **ahead** of its reference;
  - flight 2 reached 2.57 m/s against a commanded 1.92, and trailed by up to 1.4 m.

  SUPER's replan failures in Offboard: 37 and 48 `GenerateExpTrajectory failed`, plus 6 backup
  `OPT_FAILED` (flight 1, all at the walker's turn) and 8 overtimes (flight 2). On 09-21 20:05 it was
  6 and 0. It may be the envelope; that is not shown.
- **`/planning/follow_goal` has 0 messages again, in both bags.** The recorder started ~2 s before
  `command_mux` and subscribed RELIABLE to a BEST_EFFORT publisher. The mux logged the incompatible
  QoS at startup. `goal_status.requested_p` carries the same goals.
- **The IDLE-bridge interference** (`px4-bridges-offboard.md`) was live in every Stabilized/Position
  segment. No ULog was fetched.

#### Flight 1 — `bags/stage_b_2026-09-26_195108`: followed 22.8 m, then SUPER took it to 0.58 m AGL beside the walker
- **Video (made 2026-09-26):** `bags/videos/flight1_2026-09-26_1957_follow.mp4`. The descent is at
  19:57:39–41. How it was made: `docs/flight-video.md`.
- **Timeline.** Armed 19:56:55.1 (Stabilized), Position 19:57:05.0. Offboard
  19:57:11.0–19:57:42.5 (31.5 s, bag t 361.1–392.6). The pilot took Position and hovered at the far
  end (x 26.8). Landed; disarmed 19:58:12.1 at x 26.6, whose ground is ~1 m above the takeoff.
  Logs: `fsm_node_6484`, `python3_6847` (command_mux), `python3_6534` (offboard bridge),
  `follow_planner_node_6841`, `python3_6837` (target_manager).
- **The follow.** FOLLOW|OK for the whole Offboard window, locked on track 7 with no flicker.
  - At engage the walker was 5.3 m away. SUPER sprinted, closing the range to 2.6 m in 3.5 s.
  - Then 22 s of steady follow: range 3.4–3.9 m, reference gap 0.1–0.3 m, commanded 0.8–0.9 m/s
    against a walker at 0.86 m/s.
  - The walker went out to y −3.6. The goals were clamped at y −1.5, beside a ~1.3 m metal fence
    at y ≈ −2.2 (it is in the LiDAR cloud and the frames).
- **⚠️ The descent at the turnaround (bag t 389.2–391.7).** The walker turned back at x ≈ 25.4,
  near the fence end, and walked past the aircraft.
  - The goals swung 2 m side to side and were snapped from up to 2.55 m to 2.25.
  - SUPER's **primary** trajectory (`trajectory_flag` 1, not the backup) went from z 2.25 to
    **1.13**, and the aircraft followed: **ToF 1.83 → 0.58 m**. The walker was 2.9–3.1 m away
    horizontally (the minimum range, 2.16 m, came at 389.5).
  - SUPER then climbed and accelerated to 2.26 m/s past the walker (lateral gap ~2.8 m), and the
    pilot took over.
  - Nearly every replan failed in this window: `ExpOpt`/`BackOpt` "Omg or thr or Pos violation",
    1 overtime, 1 `SearchPolytopeOnPath` failure. No overhead obstacle is in the cloud.
  - **The cause inside SUPER is not established.** One candidate: the aircraft was already at
    2.33, above the 2.25 goal ceiling, and so at the edge of ROG-Map's inflated virtual ceiling.
  - **Nothing guards it:** the descent guard floors goal z only, not SUPER's path.

#### Flight 2 — `bags/stage_b_2026-09-26_200650`: 11 s of HOLD churn, 10 s of follow, lost at 3.1 m in the dusk
- **Timeline.** Armed 20:12:29.4, Position 20:12:38.7. Offboard 20:12:49.3–20:13:16.7 (27.5 s,
  bag t 358.3–385.8). The pilot took Position, landed and disarmed 20:13:26.4 at x 11. Logs:
  `fsm_node_6427`, `python3_6752` (command_mux), `follow_planner_node_6743`, `python3_6732`
  (target_manager).
- **Engage → 369.6: HOLD, as designed.**
  - The walker had turned back toward the aircraft and stood ~7 m ahead at (8.2, −0.8).
    `follow_planner` put the goal on the walker's far side (x ≈ 10.4, y ≈ −3.6) and withheld it:
    "transit to the standoff would pass inside the standoff floor".
  - Two FOLLOW ticks lasted 0.1–0.4 s each (withheld, then a PROXIMITY veto at 1.21 m from the path).
  - The HOLD climbs asked for 2.07 → 2.85 → 2.86, and SUPER snapped them to 2.25.
  - SUPER's reference wandered ±1.5 m. The aircraft drifted 2.2 m back toward takeoff at up to 1.6 m/s.
- **369.6–379.7: FOLLOW.** The walker walked away at 0.67–0.8 m/s. SUPER sprinted from 6.6 m to a
  steady 3.1–3.5 m, and the goal z followed the falling ground (1.89 → 1.55).
- **The loss.** Five COASTING↔LOCKED flickers from 376.8, then **LOST at 379.3 (20:13:10.3) at
  3.1 m range**. The yaw was on the walker (within 1–4°).
  - The frames show dusk and a walker in dark clothing against the dark tree line, barely visible
    by 378.5. **Light, not height (unlike 09-21 18:41), is the likely cause.** It is not proven.
  - Flight 1, 15 min earlier, was noticeably lighter.
- **After the loss.**
  - The floor released at 379.3 (nobody within 3.5 m). The FOLLOW goal dropped for 0.3 s to z 0.76,
    which is the terrain-relative 1.35 m AGL. The mux then went HOLD (goal 2.37 → 2.25).
  - SUPER dipped first to z 0.95 (ToF 1.49), then climbed to 2.44 (ToF 3.0).
  - **⚠️ SUPER then yawed the camera +83° away from the walker** within ~2 s. It swung back ~100° in
    0.5 s (~140 °/s) at 384.7–385.2. The HOLD trajectory's ~1.5 m vertical move is past patch
    0006's 0.3 m hold threshold, so the free-end yaw followed the trajectory.
  - With the camera turned away, auto re-acquire (ACQUIRING throughout) had nothing to see. The
    pilot took over 6.5 s after the loss.
  - **Built the same evening, not flown:** `follow_planner` now holds one heading, from 0.4 s into
    a coast and after a loss, toward where the camera was last aimed, for up to 15 s
    (`lost_yaw_hold_s`, `coast_yaw_hold`, `dtp-follow-behaviour.md`).

### Flight 2026-09-21 20:05 EDT — person following, 21.6 m in Offboard, kept pace; the bag was cut by a power loss
- **Video (made 2026-09-26):** `bags/videos/stage_b_2026-09-21_195658_salvaged.mp4`, from the
  salvaged bag. The engage step shows at 20:06:00, at 1.6 m/s. `docs/flight-video.md`.
- **Bag.** The original `bags/stage_b_2026-09-21_195658` is **corrupt**: `metadata.yaml` is 0 bytes,
  and the `.db3` header claims 690 more pages than the file holds. **Use
  `bags/stage_b_2026-09-21_195658_salvaged`**: 360 000 messages, 19:56:59–20:07:13, reindexed, and it
  plays with the repo scripts. The original was left untouched. Read it with
  `?mode=ro&immutable=1` + `PRAGMA writable_schema=ON`.
  - **Why it was cut.** The Jetson lost power at ~20:07:25, 34 s after disarm. The journal has no
    shutdown. The recorder had closed ~10 s earlier (`metadata.yaml` was created 20:07:15), but
    the page cache was not yet on disk. **Rule: after stopping a bag, run `sync` (or wait 30 s)
    before pulling the battery.**
  - **`/planning/follow_goal` has 0 messages.** The recorder subscribed RELIABLE before
    `command_mux`'s BEST_EFFORT publisher appeared. It is a race: the 17:45, 18:30 and 18:35 bags
    have it.
  - Logs are in `~/.ros/log/`: `python3_6926_*` (command_mux), `follow_planner_node_6923_*`,
    `python3_6915_*` (target_manager), `python3_6600_*` (offboard bridge), `fsm_node_6546_*`.
- **Timeline.**
  - Armed 20:05:34.6 in Stabilized, climbed to 4.5 m, came down to 1.66 m.
  - Position 20:05:51.3, then **Offboard 20:05:59.0–20:06:32.4 (33.4 s)**.
  - The pilot took Position at x 23.3, near the fence end, and flew back to x 2.4 at up to
    4.1 m/s. Landed; disarmed 20:06:50.0.
  - Config: `terrain_follow_enable` false. **This was the first flight of the HOLD-climb fix.**
    `max_vel` / `max_acc` 1.0.
- **It kept pace** (`follow_speed_report.py … --max-vel 1.0`).
  - FAST-LIO: 21.6 m net at 0.65 m/s.
  - Main FOLLOW window, 21.8 s: walker 0.61 m/s, aircraft 0.65 m/s, range 4.35 → 3.51 m
    (min 3.15). Commanded |v_xy| median 0.71, max 0.984. Altitude error +0.07 m median.
  - **SUPER in Offboard: 0 `generateBackupTrajectory OPT_FAILED`**, 6 `GenerateExpTrajectory`
    failures and 2 overtimes in 33 s. That compares with ≥ 39 % of replans failing on 09-15. On
    the ground before arming: 117.
- **The HOLD-climb fix held.** The floor latched at 1.67 m at 20:05:53 and stayed there through a
  `HOLD|PROXIMITY` (20:06:02) and a `HOLD|NO_SOURCE` (goal 2.47 m). The aircraft came back to
  1.65 m after each. The floor was released at 20:06:33.7.
- **Height above ground still grew.** ToF 1.84 m at engage, 2.83 m at x 23: the same downhill
  slope as 18:41. A person was still detected at the end of Offboard.
- **Lock.** The track hopped 3 → 0 → 8 in the first 2.2 s of Offboard ("sole candidate in band").
  Then it stayed on track 8, with 17 COASTING↔LOCKED flickers of 0.16–0.31 s in the last 8 s. No
  LOST.
- **Fence.** The walker walked along y ≈ +1.5–1.9. The follow goals sat clamped at y 1.50 for
  most of the follow: the 4 m box is narrower than the path walked.
- **⚠️ New: a 3.0 m setpoint step at the Offboard engage.**
  - SUPER was in FOLLOW from 20:05:03, before arming. Its reference ran ahead while the aircraft
    hovered in Position.
  - At the switch, `pos_cmd` was 3.0 m from the aircraft. The aircraft closed the gap at up to
    **1.7 m/s** (EKF2) in ~2 s, against a `max_vel` of 1.0. A `HOLD|PROXIMITY` followed 3 s later,
    with the walker 1.27 m from the commanded path.
  - Same measure earlier today: 18:41 0.55 m, 18:33 0.89 m.
  - Nothing guarded it. The bridge only warns "commanded point is X m from the estimate".
    **Fix 1 was built 2026-09-22 and is not flown:** the mux sends SUPER nothing until Offboard
    (`dtp-follow-behaviour.md`, "the engage gate").
- **The IDLE-bridge interference** (`px4-bridges-offboard.md`) was live for the Stabilized/Position
  takeoff and for the 4 m/s Position return. The ULog was not fetched, so its effect here is
  unmeasured.

### Flight 2026-09-21 18:41 EDT — person following, 23 m in Offboard, then lost because the aircraft was too high
- **Bag** `bags/stage_b_2026-09-21_183532` (in the repo dir, 494 s, metadata present). Armed
  18:41:31. Offboard 18:41:50–18:43:09 (79 s). Pilot took Position, landed, disarmed 18:43:20.
  The follower's ROS logs are `~/.ros/log/{follow_planner_node_11941,python3_11943 (command_mux),
  python3_11936 (target_manager)}_*.log`. Their stamps run ~3 s ahead of bag time.
  An earlier bag, `stage_b_2026-09-21_183022`, also has an armed Offboard segment (~28 s). **It is unscored.**
- **It followed.** The last FOLLOW window was 36 s long. The walker covered 21 m at a median
  0.58 m/s, and the aircraft 23 m at 0.63 m/s. Commanded speed max 0.975 (max_vel 1.0, max_acc
  1.0, the first flight of that envelope). There was no abort, no fence breach and no EKF issue.
  FAST-LIO lag p95 was 69 ms airborne.
- **Why it went high. Two causes that add up:**
  1. **A mux HOLD's climb, then the descent guard's latch.** At 18:41:59, `follow_planner`
     withheld a goal ("transit to the standoff would pass inside the standoff floor"). The mux
     went HOLD, and a HOLD goal is the current z + `hold_climb_m` 0.75 (1.50 → 2.24). The
     aircraft climbed, and the latched floor rose 1.46 → **2.10** ("drone climbed"). **It never
     released**: the walker stayed within 3.5 m for the rest of the follow. So every follow goal
     after that was 2.10, not `goal_z_min_m` 1.2. This matches the descent-guard design ("a HOLD
     climb does raise it"). This flight is the first to show what it costs.
  2. **The field slopes down, and the follow height is absolute in `lidar_init`.** The LiDAR map
     shows the ground at z −0.15 at the start and −1.3 at x = 26–28. The aircraft landed at z −1.2.
     Holding 2.1 m in `lidar_init`, the height above ground rose from 2.4 to **3.4–3.7 m** (ToF
     3.55–3.79, valid).
- **Why tracking failed.** The camera looks level (fy 611 px, half-VFOV 30.5°, horizon near
  mid-frame). At a ~3.4 m standoff, the lowest point in view is ~2 m below the camera. The
  frames show the walker whole-body at 1.7 m AGL, head and shoulders at 3.0 m, and only the top
  of the head at 3.4 m. Then the walker left the bottom of the frame: COASTING at 18:42:55, LOST
  1.1 s later. Frames with a person seen, binned by ToF AGL in Offboard: **100 % below 2.5 m, 96 %
  at 2.5–3.0, 89 % at 3.0–3.4, 24 % above 3.4.** The loss sent the mux to HOLD again (goal
  2.98), until the pilot took over. For whole-body framing at a 3 m standoff, the camera needs
  to stay under ~2 m AGL. For the torso, under ~2.9 m.
- **Also seen.** The target's z stayed at −0.2 to +0.2 while the ground dropped 1.2 m under the
  walker, so perception's person height is off by ~1 m on the slope (`dtp-calibration.md`). The
  lock hopped track IDs 1 → 0 → 2 → 0 → 3 between 396 and 406 s. Each hop was a "sole candidate
  in band" re-acquire, probably the same walker. At the end the walker passed the fence's x_max,
  and the goal was clamped at 27.5, as designed.
- **Changed afterwards (user, same evening), both bench-gated and not flown**
  (`dtp-follow-behaviour.md`):
  - a HOLD's climb no longer raises the floor;
  - a **terrain-relative follow height**, OFF by default (`terrain_follow_enable`,
    `docs/fact-terrain-follow.md`). Replaying this bag, it would have held 1.23–1.51 m above
    the ground.

  The camera tilt is still open.

### Flights 2026-09-21 16:12 and 16:13 EDT — retune Flights 3 (wobbles) and 4 (up and down)
- **Flights.** Both were Stabilized, then Position, then Altitude. Landed and auto-disarmed.
  - **16:12:** 75 s log, wobbles to ~100 °/s, height ~3 m.
  - **16:13:** 130 s log, full-stick climbs and descents to 7.6 m.
- **Logs.** `20_12_25.ulg` and `20_13_44.ulg`, local copies in `~/px4_logs/`. A 19:34 UTC log
  (22.6 MB) is on the card, unread.
- **⚠️ No FAST-LIO in EKF2 on either** (no `cs_ev_*`). They flew on baro, ToF, flow and the
  compass, the first arms without LIO on record. The preflight would have said NO-GO. The
  offboard bridge was off, so the Flight 2 interference is gone.
- **Outcome.** Roll and pitch are critically damped. Yaw lags. Vertical velocity overshoots
  ~20–25 % per step, to be re-flown on LIO. The height wander is the throttle resting outside the
  ±0.1 hold band. → the notes' §9.

### Flight 2026-09-21 15:17 EDT — retune Flight 2: wobbles, then Altitude mode to 9 m
- **Flight.** Stabilized, then Position (12.9–19 s), then Altitude (19–103 s). Climbed to 9 m,
  descended at 0.6 m/s, landed at 104 s. `IMU_DGYRO_CUTOFF` was 40.
- **Log.** `/fs/microsd/log/2026-09-21/19_17_02.ulg`, 47.9 MB. Local copy
  `~/px4_logs/2026-09-21_19_18_48.ulg`. 0.6 s of logger dropouts at 48.4–49.1 s.
- **First flight of `MPC_ALT_MODE=1`.** Over flat ground: EKF height and the ToF agreed, and
  `dist_bottom_valid` held.
- **⚠️ Found:** the offboard bridge was running. Its IDLE hold setpoint was interleaved with the
  Altitude task's, flipping the attitude setpoint by up to 45°: the aircraft pitched against a
  held stick. It is probably also the cause of the descent's vertical-speed oscillation.
  → `px4-bridges-offboard.md`.
- **Tuning result.** Roll P:D is right, pitch is not overdamped, and the D-cutoff change was
  confirmed. → the notes' §9.

### Flight 2026-09-21 14:46 EDT — retune Flight 1, Stabilized hover (gyro tuning)
The first flight of the retune after the 336 → 336L camera swap (`docs/px4-tuning-notes.md`).
- **Flight.** Stabilized only, airborne 4.9 → 91.8 s, disarmed by RC switch. 6S, 24.4 → 22.2 V.
  Flown on the baseline params. It was the first flight with `MPC_ALT_MODE=1` set, but that
  setting only acts in Position and Altitude, **so terrain following is still unflown.**
- **Log.** `/fs/microsd/log/2026-09-21/18_46_55.ulg`, 43.7 MB. Local copy
  `~/px4_logs/2026-09-21_18_48_30.ulg`.
- **Not recorded here.** Whether the stack was up and whether the preflight said GO.
- **Outcome.** A clean hover, no saturation, vibration normal outside takeoff and landing. It
  contained **no sharp wobbles**, so the rate P:D is unread. One change followed:
  `IMU_DGYRO_CUTOFF` 55 → 40 (motor-output noise is D-term-driven). → the notes' §9,
  `px4-estimator.md`.

### Flight 2026-09-15 evening — SECOND Offboard flight, and the 0006 yaw fix HELD
Bring-up at 19:13 from a cold box, aircraft placed at the takeoff spot nose down the corridor.
Everything green first time: odom **10.003 Hz** on the first measurement (no settling wait), FAST-LIO
gates clean (no laptop RViz, stamp lag p50 **57 ms**), odom bridge `TRACKING` 0 rejected, follower
LIVE with GPU cloud verified and the geofence on, **Stage B preflight GO with 4 warnings** (target
filter off, D21 waived, `extrinsic_est_en`, bag not yet running) with E1–E8 all passing inside it
(EKF2 vs FAST-LIO max **0.080 m**, no resets). Bag `~/dtp_datasets/20260915_1921_stageB`, **9.9 GB,
metadata.yaml present, playable**. Fault card empty before and after — no FC reset.
- **THE RESULT: patch 0006 works in the air.** Three Offboard segments (**28 s + 55 s + 4 s = 87 s**,
  vs 44 s on 09-14). `pos_cmd` yaw rate **median 52 °/s → 0.0–1.2 °/s**; net turn +394° → −57° / −231°.
  The hover-wobble pirouette is gone. **`/planning/yaw_target` carried 5643 messages** (09-14: zero),
  so YAW_TO_TARGET had a real target this time — which is also why some turning is expected.
- **`FOLLOW|OK` held 103 consecutive seconds** — the mux arbitrated real follow goals into SUPER
  while armed, which had never happened before. Armed 123 s, max altitude 2.44 m.
- **NOT established, don't claim it:** whether the remaining yaw motion (net −231° over the 55 s
  segment, p95 ~50 °/s, peaks ~135 °/s — above `yaw_dot_max` 57 °/s though under SUPER's warn
  threshold) is legitimate target-tracking or residual wobble. That needs the walker's bearing
  correlated against commanded yaw, from the bag. **The bag is unscored.**
- **The follow-goal altitude is LOW and was flown knowingly.** perception's tracked point on the
  walker read **z = 0.09–0.11 m in `lidar_init`** — with the LiDAR 0.15 m up that is ~0.25 m above
  ground, i.e. the **shin**, not the waist the code intends (the known "0.65 of box height lands on
  the thigh" issue). Goal z = target z + `goal_height_offset_m` (0.5), floored by `goal_z_min_m`
  (0.5) → ~0.6 m in `lidar_init` ≈ **0.75 m above ground**. The operator chose to fly it and hover
  low rather than relaunch. **`follow_planner_node` reads these at construction — `ros2 param set`
  does NOT take; changing them needs a follower relaunch.**
- **The LOST at the end** was the walker closing to 1.1 m: the proximity veto (`HOLD|PROXIMITY`,
  1.5 m radius) firing on the walk-up, which is what it is for. **`HOLD|PROXIMITY` latched once
  before takeoff too** — while it holds, the mux publishes nothing and ch11 does nothing, silently.
  Keep everyone clear of the aircraft before engaging.
- **Trap hit again, and it nearly cost a shell:** `pgrep -f fastlio_mapping` matched the Claude
  Code Bash tool's own snapshot shell. **Resolve pgids from `ps -eo pid,pgid` and verify group
  membership before signalling** — never signal what a `-f` pattern matched.
- **Afterwards, at the user's instruction, two changes — NEITHER HAS BEEN RUN:**
  (a) **`auto_reacquire_after_lost: true`** (see the `dtp_params.yaml` bullet in `dtp-follow-behaviour.md`);
  (b) **`drone-dtp/scripts/deploy_follow.sh`** — the lean bring-up, which unlike `field.sh prep`
  (observe-only, unpinned, stage_a preset) and `bench_bringup.sh` (tears everything down on exit)
  starts a **live** follower on the stage_b preset and **leaves it running**:
  `deploy_follow.sh [--observe|--no-record] | engage | stop | status`. It drops `dtp_viz` and
  `dtp_image_overlay` (`viz:=false`; both passive, nothing in the flight path reads them) and
  **starts the two bridges and the follower in parallel** — they share no state, so ~56 s of serial
  warm-up becomes ~40 s. The recorder goes up **before** the preflight so it can be asserted with
  `--require-record`. **It does not skip the FAST-LIO gates or the pre-arm preflight**, whose 60 s
  estimator window is the floor on bring-up time and the guard against the 3.74 km divergence.
  Estimated ~3 min to ready vs ~8 by hand — **estimate, not a measurement: the bring-up path has
  never been executed.** `status` and `stop` were tested. Two findings while building it:
  **`fusion_node` needs no switch — it starts, logs `DISABLED (enable:=false, the default)` and
  EXITS**; and `fusion:=false` exists only on `follower.launch.py`, as `dtp_system.launch.py`
  neither declares nor forwards it (that file has a documented history of a silent forwarding bug
  that stopped `sensor_watchdog` from ever starting — don't casually edit it).

### Flight day 2026-09-14 night — SORTIE 3 FLEW (first Offboard flight), then the yaw fix (2026-09-15)
The user restarted everything and went straight to Sortie 3 (person following, LIVE follower,
0.5 m/s), skipping Sorties 1 and 2 and the two open flight blockers — their call, stated once.
Bring-up from the dashboard/terminal at ~20:07: stack on the follow config, both bridges, follower
LIVE (`~/dtp_datasets/20260914_sortie3_follower.log`), recorder on the `stage_b` preset. Desk check
PASS, fault card empty, ch11 Offboard on the transmitter confirmed. **The Stage B preflight was
NO-GO on three lines and was overridden by the user's "just stop, skip to recording and start
following":** the laptop RViz on `/cloud_registered` (30 Mbit/s the whole time), the preflight's
0.10 m/s person ceiling vs the 0.5 in the config (`--max-vel` not passed), and the D21 single-IMU
waiver (`--waive-d21` not passed). Neither flag was passed; nothing was loosened in software.
- **The flight (bag `~/dtp_datasets/20260914_2020_stageB`, 224 s, 2.8 GiB, playable):** locked
  the walker at 3.1 m on the ground (295 follow goals to SUPER), pilot armed in Stabilized at
  120 s, **lock LOST 1 s later**, Position-mode takeoff, **Offboard 137–181 s (44 s, the first
  time SUPER/mux/bridge ever flew the aircraft)**, back to Position, landed, disarmed. No abort,
  no fence clamp, no EKF reset, no FC fault; odometry 10 Hz / lag p95 67 ms in flight; EKF2 vs
  FAST-LIO 10 cm median while moving. **No following happened airborne.**
- **Post-mortem (asked for and delivered): two causes, neither the estimator.**
  (1) **Lock loss = depth, not YOLO.** YOLO11s saw the walker in 100 % of frames. At arming the
  torso depth patch read a 3 m range jump toward the camera (grass in the prop wash, aircraft in
  tall grass at night); perception held the frame, `target_manager` coasted ("target occluded"),
  and because the walker was standing still the CTRV forecast was one point and the coast floor
  was the 1.2 s `min_coast_horizon_s` → LOST after 1.1 s. The oracle's verdict on it was LOST
  (7–33 % rays blocked), i.e. no mapped occluder. `auto_reacquire_after_lost` is false and nobody
  re-engaged (`field.sh reengage`); the walker was visible 73 % of frames right after. Perception
  also rejected the walker's range 18 times in the sortie for too few valid depth pixels (night).
  (2) **SUPER pirouetted in Offboard — the serious one.** With the mux in HOLD re-sending its goal
  at 2 Hz, SUPER's own `pos_cmd` yaw rate ran **52 °/s median, 129 °/s peak** for all 44 s with
  zero `/planning/yaw_target` messages; the aircraft orbited ±0.5 m (max 0.94 m from setpoint)
  with the heading. Cause: `yaw_mode: 3` with no target = free-end yaw aligned to the trajectory's
  displacement, which on a hold correction is the hover wobble, re-planned every 0.5 s. Not the
  yaw mount (Position mode held heading within 7° just before). **Fixed 2026-09-15: patch 0006**
  (`drone-dtp/patches/0006-super-yaw-hold-on-short-trajectory.patch`, in `apply_super_patch.sh`,
  live tree rebuilt, backup `~/backups/backup-pre-yawhold-20260914-2356/`): a free-end yaw holds
  the current heading when the whole trajectory moves < 0.3 m. **Gate
  `drone-dtp/scripts/yaw_hold_check.py` 7/7** (real `fsm_node`, closed-loop odom with a 0.2 m
  hover wobble: yaw excursion 0.0° on a 25 s re-planned hold, and still turns 0 → 92° for a 3 m
  move); the **old binary on the same gate: 394° net turn, 45 °/s** — bench reproduces the
  flight. Full record: [`docs/fact-super-yaw-hold.md`](docs/fact-super-yaw-hold.md). **FLOWN
  2026-09-15 and it held: pos_cmd yaw rate median 52 °/s → 0.0–1.2 °/s across two Offboard
  segments. See the 2026-09-15 flight entry above.**
  Harness lessons for anyone reusing the `target_filter_check` harness to make SUPER *plan*: the
  scene must be 360° (the 120° cone leaves the aircraft's own cells UNKNOWN → no corridor), and
  hover at 0.8 m (at 0 the inflated floor leaves a 5 cm corridor → backup optimiser fails).
- **Second attempt never flew:** the aircraft had landed yawed **108°** off the geofence axis
  (the box is fixed along the nose at stack start), so every follow goal was clamped to the box
  edge, and the walker was no longer in front of the camera (it faced a house wall, night). Its
  recorder died at 20:33 with no metadata and a malformed db (unexplained; nothing in it).
- **Runbook additions from this:** after arming say "watch" or run `field.sh watch`; a LOST needs
  `field.sh reengage`; if the aircraft lands yawed, restart the stack at the spot or set
  `fence_yaw_deg`; a walker standing still has a 1.2 s coast, ask them to keep moving.
- **Stack restarted on the 0006 binary at 00:13 on 2026-09-15 (user's request), verified by
  `cmp` against the install binary; follower relaunched LIVE, FOLLOW engaged.** The gate chain
  (`target_filter_check`, `loss_oracle_check` on the new binary) had to be **killed** for that:
  the dashboard refuses to start the stack while a harness `fsm_node` exists on domain 88.
  **Those two gates are still owed on the 0006 binary** (the old-binary yaw gate did finish:
  6/6, 393° spin reproduced). Run them with the stack down.

### Flight day 2026-09-14 evening — Sortie 1 Stage A reached the pre-arm gate, then PAUSED (user)
Venue bring-up redone after a Jetson+FC restart at ~19:27, all from the dashboard/terminal: stack on
the follow config, both bridges, `field.sh prep` PREP DONE (odom 9.97 Hz at the origin, stamp lag
p50 52 ms), Stage A preflight GO with the two expected warnings. `field.sh arm` recorded and locked
the walker at 3.14 m (`~/dtp_datasets/20260914_1943_stageA`, 16 GB, 17 min on the ground, stopped
cleanly, metadata.yaml present). **Never armed.** Two things stopped it:
- **The laptop RViz on `/cloud_registered` came back twice** (closed on request at 19:37, back by
  20:02). Every gate refuses on it, correctly. Field rule stands.
- **Sporadic 0.3–0.9 s odometry stamp-lag spikes, ~1 per 1–2 min, each a stall then 3 scans in
  1 ms**, with p50 steady at 56 ms and clean 60–180 s windows between. NOT the 09-13 cause: no
  Wi-Fi traffic (0 Mbit/s), swap 0, RSS flat 171 MB, FAST-LIO sockets `O_NONBLOCK`, eno1 0 drops.
  Found on the way: `field.sh prep` launches the follower via `indoor_test.sh`, which does **not**
  pin, so perception (66 %), the recorder (30 %) and the camera container sat on FAST-LIO's cores
  6-7 — unlike the 09-13 soak (pinned, 144 ms max). Pinned live with `taskset -acp 0-5` (the
  `-p` form needs the list *after* `-c -p`; the other order fails silently). One 0.34 s spike still
  seen after pinning; the next 60 s + a 30-msg preflight window were clean. Unresolved.
  **Trap:** a Python subscriber on `/livox/lidar` is itself the slow reader — it showed 0.7 s
  "lidar lag" and +3279 UDP RcvbufErrors that were its own; the driver's counter is flat without it.
  `stage_a_preflight` now refuses such subscribers for that reason.
- Transmitter at pause: ch5 Position (slot 4), kill/arm/Offboard all OFF (3) — it started the
  evening on Stabilized with the kill switch engaged.
**Left running:** stack + bridges (dashboard-owned), observe-only follower pgid 11221 (pinned 0-5),
FOLLOW disengaged, recorder stopped. **Resume in place:** close the laptop RViz →
`bash drone-dtp/scripts/field.sh arm` (needs the walker moving at 4–6 m). If the aircraft moved,
restart the stack at the takeoff spot and `field.sh prep` first. Done for the day: `field.sh land`
then STOP ALL.

## Field procedures (M5.1, M5.4/M5.5)

- **M5.1 (props off):**
  - **Runbook:** `docs/m5.1-one-graph-runbook.md`.
  - **Scorer:** `scripts/px4/m51_one_graph_check.py BAG` (S setpoints / G GoalStatus / A abort /
    T takeover). **It exits 3 = INCOMPLETE unless all four sections scored — never read that as a
    pass.**
  - **Bench rehearsal, disarmed** (`~/dtp_datasets/20260913_m51_bench/`):
    - the full graph came up clean;
    - estimator preflight **GO**;
    - Stage B preflight `--target mannequin` **GO with 2 warnings**.
  - **The proximity veto blocked the rehearsal, correctly.** A person seated at the desk ~1 m from
    the rig holds `HOLD|PROXIMITY` from startup, and the mux then publishes nothing: `fsm_node`
    stays in INIT.
  - **PX4 facts from source (don't re-derive):**
    - the ch11 Offboard switch is **edge-triggered**;
    - NAV_LAND is a **forced** mode change;
    - `COM_RC_OVERRIDE=1` lets the sticks take over in AUTO_LAND but not in Offboard.
  - **After an ABORT, never re-engage ch11 without relaunching the follower.**

- **M5.4/M5.5:** `docs/m5.4-m5.5-field-runbook.md`.
  - **Motor-on calibration** comes from the Stage A flight bag via
    `drone-dtp/scripts/bag_to_calib_pairs.py`, not a props-on ground capture (10 s preflight
    disarm, 15 s dwell, PointCloud2 mode starves FAST-LIO).
  - `analyze_shadow_goals.py` section 6 scores M5.5.
  - `outdoor_depth_test.py` needs `sensors.launch.py enable_ir:=true`.

### Still true from 2026-09-02 (second session) — historical: the aircraft has flown since
- **M0–M3 are all signed off. M4 (vision goal adapter) is the next milestone.**
- **▶ NEXT SESSION IS THE FIRST FLIGHT. The plan is written: [`docs/flight-test-plan.md`](docs/flight-test-plan.md).**
  Phases P0 (bench) → P1 (mechanical yaw check) → P2 (venue) → P3 (**Position-mode hover, no
  Offboard, no SUPER — the important one**) → P4 (moving-rig latency + M2 gate recordings) →
  P5 (Offboard hold in the air) → P6 (first SUPER trajectory). P0→P4 is a full successful session;
  P5/P6 only if everything before was boring. The user asked for this plan and said they will work
  from it.
- **⚠️ THE BIG ONE: nothing has ever flown, or even moved.** Every gate to date is bench and
  stationary — M3 armed the aircraft with props off, so it could not translate. Specifically still
  unproven: the bridge's `TRACK` path has never driven a moving vehicle; the frame-matched
  odometry-latency check (`docs/fact-fastlio-latency.md`) has never been run on a moving rig,
  which matters because the latency fix drops the cloud queue to depth 2 so stale scans are
  *dropped* under load and a bench never produces that load; and the **MID360→FC yaw mounting
  offset** is still unmeasured (see the M2 gate in `px4-estimator.md` — it is a mechanical check, and a wrong value
  makes the drone spiral under position control). Do those before adding M4 on top.

- **The stack was STOPPED cleanly at the end of this session.** Nothing of ours is running; swap 0 B.
  The LiDAR itself is still powered and reachable (`ping 192.168.1.154`), and ARK's
  `dds-agent.service` / `mavlink-router.service` are up as always — so the 65 `/fmu/*` topics are
  present with no stack running. That is normal, not a leftover.
  Bring it back with `RVIZ=false bash scripts/run.sh`, then, in two more shells,
  `ros2 run super_bringup px4_odom_bridge.py` and `ros2 run super_bringup px4_offboard_bridge.py`.
  **The FC was left in `AUTO_LOITER` (Hold), disarmed** — it was put into Offboard during the M3
  test and explicitly returned with `offboard_engage.py --hold`. No FC parameter was changed this
  session; the last param dump is still `firmware/px4-params-2026-09-01.params`.
  **Don't use `pkill -f` to stop the stack** — the pattern matches your own shell's command line
  and kills it (it happened again this session, exit 144, with `pkill -f scripts/run.sh`). Kill by
  PID, or use a pattern that cannot appear in your own argv.
