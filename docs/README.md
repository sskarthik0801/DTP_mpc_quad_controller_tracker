# docs/ — index

- [`mpc-tracker-integration.md`](mpc-tracker-integration.md) — **how to install, select, gate
  and fly the MPC trajectory tracker (2026-10-17). Integrated, OFF by default, never flown.**
  What the branch changes, how to push it as a branch, the install, the two offline gates, the
  four ways to select it (`GCS_TRACKER=mpc`, `deploy_follow.sh --tracker=mpc`,
  `bench_bringup.sh TRACKER=mpc`, by hand) and how to go back, which tools accept it unchanged and
  which do not (`m3_gate_check`'s drift check passes vacuously), the log line, and the
  eight-step checklist before it flies.
- [`mpc-tracker.md`](mpc-tracker.md) — **the MPC trajectory tracker's design (2026-10-05,
  updated 2026-10-17).** `px4_mpc_tracker.py` replaces `px4_offboard_bridge.py` and sends PX4
  acceleration setpoints from a condensed linear MPC (numpy, FISTA, a disturbance observer). The
  model, cost, limits, states, the far-setpoint clamp, the yaw servo stage, the parameters, and
  the gate results (sim 8/8, node check 21/21). Not the same thing as `goal_mode: nmpc`.
- [`flight-video.md`](flight-video.md) — **flight-review videos from a bag (2026-09-26):
  `bash drone-dtp/scripts/flight_video.sh bags/<bag>` → `bags/videos/<bag>.mp4`.** Camera with
  person boxes, a HUD (mode, speed, height, range, lock, mux, SUPER goal), a top-down map and
  speed/height/range plots. The boxes are the flown `yolo11s` **re-run offline**, because the
  aircraft's own 2-D boxes are never in a bag. "Locked" comes from the real lock state and the
  projected `follow_target` (88 % within 60 px). It also records how the videos were made, the
  traps (no H.264 in OpenCV here, `libcudss`, stamps, SNAPPED on every goal), and the videos made
  so far.
- [`plan-veto-height-and-lidar-range.md`](plan-veto-height-and-lidar-range.md) — **the build
  plan for the next two fixes (2026-09-30). Both parts built the same day, not flown; the build
  records are in `.claude/rules/dtp-follow-behaviour.md`.**
  - **Part A, small and first:** the proximity veto's ±2 m height band is measured from a
    foot-level tracked point, so an aircraft ≥ 2 m above it is not tested. The fix is
    `proximity_veto_half_height_m: 3.0`, plus new `close_pass_check` cases.
  - **Part B:** LiDAR ranging past 10 m, as a hybrid (camera near, LiDAR far).
    - Step 0 is a go/no-go: does YOLO even detect the walker at 10–17 m? GO in daylight (100 %
      of in-FOV frames at 8–17 m on 09-29); no gain at dusk.
    - The existing `depth_source: lidar` needs ≥ 6 torso points, and gets 1–5 per scan on the
      whole body. It also assumes an identity LiDAR→IMU extrinsic, which is no longer true.
- [`plan-camera-yaw-servo.md`](plan-camera-yaw-servo.md) — **the plan to keep the person centred
  (2026-09-29). Built the same day, both switches off, not flown; the build record and gate
  results are in `.claude/rules/px4-bridges-offboard.md`.** The offboard bridge aims the heading
  from where the aircraft is to `/planning/yaw_target` (fix 1), and can feed that heading's rate
  forward (fix 2). Both
  sit behind switches, default off, with rate limits. The build steps, the parameters, the gates
  (unit, `m3_bridge_check`, replay pass criteria, props-off) and the flights Y1/Y2 are all in the
  file. The evidence is the 09-26 replay: p95 off-centre 16.8° → 4.2° → 1.8°.
- [`px4-velocity-tuning-plan.md`](px4-velocity-tuning-plan.md) — **the horizontal velocity loop:
  findings and a tuning flight plan (2026-09-27). Nothing flown, no gain changed.**
  - **Measured** from the 2026-09-26 Offboard bags and the 2026-09-21 ULogs:
    - the stock loop lags its own velocity setpoint by ~0.55 s and does not overshoot it; the
      overshoot past SUPER's reference came from the stored position error;
    - the attitude loop lags a tilt command by tau 0.15–0.19 s plus 40–60 ms;
    - rotor drag ~0.3 s⁻¹.
  - **Modelled:** PX4 v1.17 `PositionControl` on the fitted plant, validated to 0.10 m/s rms.
    Acceleration feedforward removes most of the error (3 m/s: 1.3 m → ~0.4 m). Tuning then
    halves what is left (0.25 / 0.14 m).
  - **Proposed:** `MPC_XY_VEL_P_ACC` ≈ 3.0, `_D_` 0.1, `_I_` ≈ 0.2 × P, `MPC_XY_P` unchanged.
  - Also: the limits that bind with speed, cited from v1.17 source (`MPC_ACC_HOR` and
    `MPC_JERK_MAX` do not apply in Offboard; `MPC_Z_VEL_MAX_DN` does).
  - The flight plan, in the accepted order: O1 (feedforward alone), the Direct-velocity ladder
    V0–V6, then O2/O3, each with a prediction and a stop rule.
  - Scripts: [`evidence/velocity-loop-2026-09-27/`](evidence/velocity-loop-2026-09-27/README.md).
- [`px4-tuning-notes.md`](px4-tuning-notes.md) — **the PX4 retune after the Gemini 336 → 336L swap,
  as notes (2026-09-20).** The procedure from [`PX4 Tuning Guide.pdf`](PX4%20Tuning%20Guide.pdf)
  (153 slides) with page references: filters → rate (D, K, I, yaw FF) → angle P → vertical then
  horizontal velocity → position P → flight-behaviour limits; a symptom → fix table; the PDF's
  slips. Beside it, this FC's baseline from the 2026-09-17 dump: the guide's filter section is
  already applied at its starting values, rate gains are stock apart from K = 1.2, the velocity
  and position loops are stock, and `CA_ROTOR*` positions are 0.11 / 0.10 **ft** stored as metres.
  Which layers a camera swap invalidates, and this repo's rules that override the guide.
  **Nothing on the FC was changed; nothing flown.**
- [`fact-terrain-follow.md`](fact-terrain-follow.md) — **terrain-relative follow height
  (2026-09-21), ON since 2026-09-22, never flown.** Why (the 18:41 flight: a sloping field put the
  aircraft 3.4–3.7 m up and lost the walker), why a LiDAR ground map and not the downward
  rangefinder (ditches, trees, bushes), the design (`terrain_estimator`, the rate-limited offset,
  the descent guard in height above ground), the replay (1.23–1.51 m above ground vs 2.4–3.8 m
  flown), the gates, seven traps and the limits.
- [`fact-nmpc-follow-goal.md`](fact-nmpc-follow-goal.md) — **`goal_mode: nmpc`, a
  receding-horizon follow goal (Stages 1 and 2 built 2026-09-19/20).** Why an MPC belongs in the goal
  and not in the control loops; why it optimises the goal's positions and not the aircraft's;
  why the solver is vendored from its MIT upstream and not from SUPER; the cost, the reset rule
  and the fallback; nineteen bench traps; the gates (`nmpc_check.py` 41/41, gtest 15/15,
  `visibility_field_check.py` 13/13); what it cannot do (raise top speed, make SUPER plan
  through the goal, know occlusion where the aircraft has not looked); Stage 2 = SUPER patch
  0008 + a visibility-aware trail bearing. **Never flown; no flight binary was rebuilt and 0008
  is not applied to the live SUPER tree.**
- [`fact-follow-speed-and-descent-guard.md`](fact-follow-speed-and-descent-guard.md) — **why the
  2026-09-15 20:01 follow flew slower than 0.5 m/s, and what changed (2026-09-16/17).** 13 s of
  following at a median 0.39 m/s commanded (max 0.496): `max_vel` is a 3-D cap and every plan ends
  at rest ~2 m ahead; the walker outpaced the cap; the goal altitude flapped because the follow
  standoff sits on the descent guard's 3.0 m radius; ≥ 39 % of SUPER's replans failed. User
  decisions: `max_vel` 1.0, `max_acc` 0.5 (1.0 since 2026-09-21), follow height 1.2 m (A), latched descent guard
  (B); how B handles a branch (it pins the goal, not the path). Auto-reacquire locked a different
  person in Offboard. Gates `descent_guard_check.py` 11/11; `follow_speed_report.py` reproduces
  every number. **Nothing flown.**

- [`geminitest.md`](geminitest.md) — Gemini 336L color-image publisher,
  JPEG topics, laptop RViz viewing, installation and live verification.

- [`eva-tracker-faster-lio-assessment.md`](eva-tracker-faster-lio-assessment.md) — **can Eva-Tracker
  (ICRA 2026) or Faster-LIO improve this stack? (2026-09-14)** Eva-Tracker: not as the flight
  planner (ROS 1 / MAVROS / MonoLoco / AGPL), yes as a re-implemented baseline arm and as a
  Bézier-QP predictor arm; its own paper confirms its loss handling is prediction-only. Faster-LIO:
  not now — FAST-LIO is 10 Hz / 50 ms / flat RSS; try `extrinsic_est_en: false` (74 % → 31 %)
  first, swap only if the moving-rig latency check fails. Plan deltas as dated rows.

- [`m5-m6-plan.md`](m5-m6-plan.md) — **the M5/M6 plan (2026-09-12), with numbers.** Gate 0 (safe to
  fly autonomously) → M5 (the DTP follower flies) → M6 (the paper's loss-cause classifier, which
  does not exist in code yet). Includes:
  - a dated schedule from 2026-09-14 (D1 Oct 27, D2 Nov 10, full draft Dec 18);
  - pass thresholds for every gate, sample sizes and a compute budget;
  - a claim-by-claim map of the paper against what is doable;
  - Gate 0 as it stands (2026-09-13):
    - the XRCE hardfault — reproduced; the RX-DMA fix failed; a client-side fix is flashed and
      passed 30/30 reconnects, soak pending;
    - G0.2 closed — a 3.74 km FAST-LIO error from a second Livox driver, now guarded;
    - G0.4 done — the SUPER patch is applied by `setup.sh`;
    - still open: `max_vel: 5.0` in the follower's SUPER config.
  - the GPU organized cloud (M5.3) and the finding that node-level bag replay is not deterministic
    (M6.2).

- [`m6.6-predictor-study.md`](m6.6-predictor-study.md) — **M6.6 offline predictor study for Table II
  (2026-09-14), rebuilt from PUBLIC sources** — the flown learned-predictor assets never reached this
  box. Public ET-STGCNN checkpoints wrapped so the repo's adapter loads them; TrajImpute-style
  ETH/UCY protocol at 0/10/30/50 % missing steps; CTRV (flown code, and a gap-native re-fit), Eigen
  best-of-1 / best-of-20, fused with constant and scheduled α. Findings: best-of-1 from the sampler
  is a selector problem (raw sample 0 is worse than constant velocity); the "needs a complete
  history" sensitivity is the forward-fill imputation, not the model; CTRV ≈ CV on a 4.8 s horizon.
  `drone-dtp/scripts/m66_predictor_study.py`, outputs in `~/dtp_datasets/m66_predictor_study/`.
  §8: the AGX deploy's assets turned out to be the same public weights plus a worse wrapper
  (NaN for groups containing a standing person) — nothing to adopt.
- [`flight-day-runbook.md`](flight-day-runbook.md) — **the first person-following flight, step
  by step (2026-09-14):** three sorties in a fixed order (Stage A observe-only → M5.1 props off →
  Stage B S1), each step as a GCS button, a terminal command, or "ask Claude"; roles, venue, pass
  criteria, the two open blockers, the traps. Since 2026-09-20 the GCS's *Flight day* card is the
  short standard bring-up (Observe only / Live) and no longer renders this file's order.
- [`m6-belief-around-trees-plan.md`](m6-belief-around-trees-plan.md) — **the plan for M6.4b
  (2026-09-14), BUILT the same day as patch 0005; its head lists every deviation.** B = rollouts
  that slide round OCCUPIED cells instead of through them; B2 = when most rollouts stay hidden
  behind the occluder, yaw and pre-position to look behind it. The as-built record (gate numbers,
  what B cannot do, traps) is the "M6.4b" section of `m6-loss-oracle.md`.
- [`m6-loss-oracle.md`](m6-loss-oracle.md) — **M6 as built (2026-09-14): the loss oracle inside
  `fsm_node`** (OCCLUDED / UNMAPPED / LOST / OUT_OF_VIEW against ROG-Map, the reappearance belief,
  a pre-positioning viewpoint; SUPER patch 0004, `dtp_msgs/LossEvent`), **loss conditioning** in
  `target_manager` + `follow_planner` behind one `loss_conditioning` switch, the stock baseline,
  and the M6.2 replay/labelling harness. Gates: 28/28, 16/16, 27/27 on the bench; run5 replayed.
  **Nothing flown.** Since 2026-09-20 also **patch 0008, the oracle's visibility field** (what it
  computes, the one rule `evaluate()` does not have, its gate, and the `--compare` trap) — built
  and gated, NOT applied to the live SUPER tree.
- [`m6.3-handheld-runbook.md`](m6.3-handheld-runbook.md) — **the handheld occlusion dataset →
  D1, as a field day (2026-09-23). Not yet run.** No flying: the rig is carried past 1.8 m
  occluders while a subject walks 140 (or 70) scripted events, then the bag is replayed through
  the real `fsm_node`, labelled and scored. Holds the geometry that decides whether the day is
  worth anything — the MID360's 7° view floor and the map's virtual ground against carry height
  and subject range, and why the rig is carried at **1.2 m, not the plan's 1.5 m** — plus the
  bring-up, the record list, the event script, the live `rays_cast` check, the sweeps, the pass
  lines and the traps. It exists because neither bag on this box can score D1: run5's occluder is
  below the sensor's floor and Stage A's rig sat in grass.
- [`dtp-follower-stack.md`](dtp-follower-stack.md) — **the human-following stack.** Since
  2026-09-12 it is [`../drone-dtp/`](../drone-dtp/) on branch `dtp-follower-stack` (pushed to
  `origin`), symlinked into `~/super_ws/src/drone-dtp` so the drone runs the repo folder; how that
  is wired, built and run is in [`../drone-dtp/README.md`](../drone-dtp/README.md). It is in
  substance the M4 vision goal adapter. What each node does, the SUPER patch it needs, the
  camera/LiDAR calibration, and the state it was found in. **Revised 2026-09-12:** detection
  dropped to 10 Hz and then 8 Hz on measurements, `command_mux` gained an ABORT that **lands the aircraft** on
  sensor loss, a Stage A observe-only flight mode appeared, `static_calib_tf` now publishes the TF
  link that was missing — and the SUPER patch was found **stale against the live source** in a way
  that silently broke the GoalStatus contract on a fresh install (regenerated and applied by
  `setup.sh` on 2026-09-13). **2026-09-13:** the organized
  cloud moved to the GPU (`cloud_backend: cuda`; bit-identical, 1.9 → 0.7 cores in a bag replay,
  no wall-time gain), and the measured per-stage CPU breakdown replaces the "JPEG decode" story.

- [`fact-ark-logloader-loop.md`](fact-ark-logloader-loop.md) — **ARK OS `logloader` re-uploaded one
  ULog ~21,000 times (344 GB, disk at 95 %) until disabled on 2026-09-13.**
  - **Why:** it fetched the log while PX4 was still writing it, and could never mark the finished
    log uploaded.
  - **What was deleted:** 364.8 GB of verified duplicates.
  - **What was kept:** the FC's complete copy of the log.
  - **Backups:** both databases.
  - **Checks before a flight day:** `systemctl is-enabled logloader` and `df -h /`.

- [`paper/`](paper/) — **ICRA draft** (LaTeX). Skeleton + method + system
  characterization written; flight results pending. `\tbd{}` = not written/measured yet,
  `\bench{}` = measured but on the bench, never in flight. See `paper/README.md`.

**Current source of truth** (written for this repo, kept up to date):
- [`../AGENTS.md`](../AGENTS.md) — repository working rules and current contracts;
  resolves conflicting session history using implementation and dated evidence.
- [`../CLAUDE.md`](../CLAUDE.md) — orientation for a fresh Claude Code session; status + gotchas.
- [`../.claude/rules/`](../.claude/rules/) — the per-area detail split out of `CLAUDE.md` on
  2026-09-17 (session history, measurements, traps), path-scoped.
- [`../README.md`](../README.md) — the runbook: install, run, fixes-baked-in, troubleshooting.
- [`ImplementationDoc.md`](ImplementationDoc.md) — the full M0–M5 plan and the **current source of
  truth for M2/M3**. Includes the resolved stationary FAST-LIO regression, the
  outstanding moving-rig gate, and M4 integration work for the existing external DTP stack.
- [`../scripts/gcs/README.md`](../scripts/gcs/README.md) — dashboard operation,
  server-side Admin control for camera/goal commands, bag recording, FC/Jetson
  reboot interlocks and explicit stopping of terminal-started processes.
- [`uart_bringup_log.md`](uart_bringup_log.md) — **FC-link post-mortem (round one).** Why the first
  "UART fault" was really `UXRCE_DDS_CFG=0` plus ARK OS owning the port. **Read before touching any
  serial device on this machine.** *(Note: a second, genuine UART fault was found later — the Tegra
  DMA bug in `xrce-agent-nx.md`. This file predates it. The NX was also reimaged with ARK OS on
  2026-08-31, so the ARK-era mechanism applies again.)*
- [`xrce-agent-nx.md`](xrce-agent-nx.md) — **the Tegra UART DMA fault and its fix (2026-08-31).**
  `ttyTHS1`'s DMA path handed the tty layer **zero-filled buffers** — right byte count, no framing
  errors, payload gone. It blocked M2 for weeks and produced **two wrong "the wire is dead"
  conclusions**, because every capture was taken through the broken receiver. Removing
  `dmas`/`dma-names` from `serial@3100000` forces PIO and the link works.
  **Read before debugging any serial link here.** Also records every eliminated cause.
- [`../scripts/uart_loopback_test.sh`](../scripts/uart_loopback_test.sh) — proves a port can echo
  itself. **Run this first**, and after any system update — the DTB fix does not survive one.
- [`mavlink-setup.md`](mavlink-setup.md) — **MAVLink & QGroundControl.** What was installed, the
  endpoint layout, and how to attach QGC from a laptop. *(Written when the NX ran stock JetPack;
  since the 2026-08-31 ARK OS reimage, ARK's `mavlink-router.service` and `/etc/ark-os/mavlink-router.conf`
  are the active ones — ours are inactive. Endpoint 14569 is unchanged, so `scripts/px4/*.py` work.)*
- [`../firmware/README.md`](../firmware/README.md) — **the custom `ark_fpv` firmware this FC runs**,
  the board patch that produces it, its sha256, and the pre/post-flash param dumps. **Read before
  any FC firmware update** — a stock flash removes the INA238 driver and battery current silently
  goes back to 0.00 A.
- [`../scripts/px4/README.md`](../scripts/px4/README.md) — FC tooling over MAVLink UDP 14569, the
  port-ownership rule per board, and the MAVLink param/heartbeat gotchas.
- [`ark-12s-payload-power-module.md`](ark-12s-payload-power-module.md) — **battery current sensing
  via the ARK 12S Payload Power Module (2026-09-01). WORKING, on a locally built firmware.**
  Stock **ARK FPV** firmware has no `ina238` driver, so `SENS_EN_INA238`/`INA238_SHUNT` don't
  exist and no parameter change can help — ARK's setup page is written for **ARKV6X**. Records the
  two-line board patch, the aarch64 build gotchas (`Tools/setup/ubuntu.sh` does **not** install
  `arm-none-eabi-gcc` on arm64), and the params, including `BAT1_V_CHANNEL=-2`, which ARK's page
  omits. **Read before any FC firmware update — an update removes the driver again.**
- [`flight-test-plan.md`](flight-test-plan.md) — **flight plan and attempt records**, revised
  2026-09-04. P3 on 2026-09-02 did not pass (no takeoff, nine EKF position resets);
  P5 on 2026-09-04 is void (optical-flow hover with the LiDAR stack stopped).
  LiDAR-controlled flight remains unproven. Sequences mechanical yaw checks,
  Position-mode hover, moving-rig recordings, Offboard hold and SUPER trajectories.
  Carries the **abort ladder** and venue rule: **FAST-LIO needs structure**.
- [`../firmware/faults/README.md`](../firmware/faults/README.md) — 35 archived crash dumps
  through 2026-09-13 (22 field crashes of the XRCE serial-driver fault, 2 other field signatures,
  11 induced on the bench), the on-demand reproduction, the FAILED RX-DMA fix, and the client-side
  fix's bench results (30/30 reconnects; flood inconclusive; soak pending).
  Diagnosed does not mean fixed; inspect the actual FC card before updating the history.
- [`calibrate.md`](calibrate.md) — **what to re-measure and re-verify after taking the drone
  apart.** Scoped by what you actually removed, so you don't blanket-recalibrate. Params and
  firmware survive a teardown; **geometry and wiring do not**. Covers the lever arm, the mounting
  tilt check, the mounting-**yaw** problem that no script can measure, and ends at the motion gate
  — the only step that catches an inverted axis.
- [`M1-session-notes.md`](M1-session-notes.md) — what M1 did, decisions, fixes, gate results,
  and the later resource regression.
- [`fact-hardware-pipeline.md`](fact-hardware-pipeline.md) — durable fact: the MID360→FAST-LIO→SUPER pipeline.
- [`fact-fastlio-cpp17.md`](fact-fastlio-cpp17.md) — durable fact: the FAST-LIO C++17 build fix.
- [`fact-fastlio-divergence-2026-09-12.md`](fact-fastlio-divergence-2026-09-12.md) — durable fact:
  **G0.2. The aircraft was armed on a FAST-LIO pose 3.74 km wrong, because a second Livox driver
  starved FAST-LIO of scans for 225 s.** Timeline, bench reproduction, the bridge divergence guard,
  `estimator_preflight.py`, and what it still does not cover.
- [`fact-mpc-alt-mode.md`](fact-mpc-alt-mode.md) — durable fact: **why Position mode did not climb over a raised surface** (2026-09-17). Height is held relative to FAST-LIO, not the ground; the ARK Flow's flow chip gives only horizontal velocity; `MPC_ALT_MODE` 0/1/2 explained. Mode 2 (terrain hold, PX4 default) uses the rangefinder only when stopped, and was never active while crossing the edge in the 09-15 20:10 log. Now 1 (terrain following), not yet flown. `EKF2_MIN_RNG` 0.01 → ~0.15 still open.
- [`fact-fastdds-blocking-send.md`](fact-fastdds-blocking-send.md) — durable fact: **a Wi-Fi subscriber to `/cloud_registered` can stall FAST-LIO** (rmw_fastrtps sends synchronously on blocking sockets; reproduced with `strace` delay injection, 10 Hz → 1.9 Hz); ASYNCHRONOUS mode does not fix it, FastDDS `non_blocking_send` does (`FASTLIO_DDS_PROFILE=nonblocking`). Also: the laptop is sent every datagram twice (Wi-Fi + Tailscale locators).
- [`fact-fastlio-latency.md`](fact-fastlio-latency.md) — durable fact: **the ROS2 port shipped
  ~1.4 s of odometry latency** (correct values, correct stamps, just 1.4 s old — and nothing in
  the stack reports it). Root cause, the fix, and the measurement traps that produced three wrong
  conclusions first. **Read before trusting any latency number measured from Python.**
- [`fact-rogmap-bounds-and-memory.md`](fact-rogmap-bounds-and-memory.md) — durable fact: ROG-Map's
  **effective** height bounds are not the configured ones (`-1.0` -> **-0.5**, `3.5` -> **2.9**),
  `fsm_node`'s **2.03 GB** is a bounded A* preallocation (not a leak), and the recurring
  "`No odom`" log is cosmetic.
- [`fact-ev-delay.md`](fact-ev-delay.md) — durable fact: **`EKF2_EV_DELAY` must stay 0.** PX4
  subtracts it *on top of* `timestamp_sample` (`estimator_interface.cpp:381`), and FAST-LIO stamps
  odometry with `lidar_end_time` — the instant the ESIKF state is valid for — so the ~50 ms is
  already carried. Setting 50 would double-count. Also: `EKF2_DELAY_MAX` must stay ≥ ~100 ms.
- [`fact-ev-frd-yaw-align.md`](fact-ev-frd-yaw-align.md) — durable fact: publishing
  **`POSE_FRAME_FRD` makes PX4 set `yaw_align = false`** (`ev_yaw_control.cpp:179`), hence
  `heading_good_for_control: False`. **Not a fault** — and it costs **only Stabilized and Auto**;
  Offboard/Position/Altitude are unaffected. Records why FRD is kept, the `EKF2_EV_CTRL=3`
  escape hatch, and why switching the bridge to NED is the worst option.
- [`fact-super-cmd-stream.md`](fact-super-cmd-stream.md) — durable fact: the **`/planning/pos_cmd`
  contract**. It is **BEST_EFFORT**, so a default (RELIABLE) subscription never matches and never
  errors; and it goes **completely silent** outside `FOLLOW_TRAJ`/`EMER_STOP` — including every
  time SUPER reaches a goal. Both break a naive consumer. Also: `header.stamp` is unreliable
  (a double-precision bug in `getSimTime`), so use arrival time.
- [`fact-super-yaw-mode.md`](fact-super-yaw-mode.md) — durable fact: **SUPER's `yaw_mode`, and why
  the click config runs 3 (2026-09-12).** Upstream never reads the key, so 1 and 2 are identical.
  Only the DTP patch's `YAW_TO_TARGET` (3) does anything: it aims the nose at `/planning/yaw_target`
  and otherwise follows the velocity, but it **never applies the goal yaw**. That means the RViz
  arrow is ignored and the GCS Rotate is refused. Also: `yaw_dot_max` is a pacing hint, not a clamp.
- [`fact-offboard-rc-arming.md`](fact-offboard-rc-arming.md) — durable fact: **Offboard engages
  without RC; arming does not.** Corrects a claim repeated across these docs. Includes the
  reusable method for answering "why won't it arm" from the command line — `listener
  health_report` decoded against `health_component_t` in
  `~/PX4-Autopilot/src/lib/events/enums.json`.
- [`fact-super-yaw-hold.md`](fact-super-yaw-hold.md) — durable fact: **SUPER pirouettes on a re-planned hold goal under `yaw_mode: 3` with no target** (first Offboard flight 2026-09-14: 52 °/s median for 44 s); the free-end yaw aligned to the hover wobble. Patch 0006 holds the heading below 0.3 m of trajectory displacement; gate `yaw_hold_check.py`, which also reproduces the spin on the old binary.
- [`fact-rogmap-unhit-decay.md`](fact-rogmap-unhit-decay.md) — durable fact: **a person seen from the grounded aircraft never clears from ROG-Map** (2026-09-14 bags: 113 of 457 cells still occupied 7 min after the walker left) because every beam through them goes into the sky and ROG-Map casts rays only to returns; not tunable, not recoverable from the sensor. Patch 0007 (`rog_map/unhit_decay`, on in the follow config) decays an occupied cell in clear view that no ray reaches: gate `unhit_decay_check.py` 10/10, bag replay 113 → 3 survivors, pass 3–4 ms.
- [`fact-geofence-command-mux.md`](fact-geofence-command-mux.md) — durable fact: **the geofence
  lives in `command_mux`, at the goal level** (M5.2, user decision 2026-09-14). PX4's fence is
  inert without a global home (`EKF2_GPS_CTRL=0`). Soft boundary = the edge (FOLLOW/NAV/HOLD goals
  clamped 0.5 m inside); hard boundary = edge + 3 m (aircraft past it → `FENCE` veto → land).
  Why option 1 over GPS fusion, what it does not cover (`/goal_pose` clicks, the GCS RC card), the
  eight parameters, and the 13/13 real-node gate `fence_check.py`. Never flown.
- [`build-fixes.md`](build-fixes.md) — SUPER colcon/C++17/`isnan`/`pcl_conversions` build fixes.

**⛔ SUPERSEDED — historical only.** These describe an **ArduCopter + MAVROS** plan that was
abandoned once the FC was confirmed to run **PX4 v1.17.0 + uXRCE-DDS**. Their parameter names
(`SERIALx_PROTOCOL`, `EK3_SRC1_*`, `ARMING_CHECK`) **do not exist** on this aircraft. Do not
follow them; they are kept only for provenance.
- [`mavros-setup-notes.md`](mavros-setup-notes.md) — MAVROS bring-up notes (superseded).
- [`plantofly.md`](plantofly.md) — ArduCopter EKF3 ext-nav + bridge planning (superseded).
- [`nohardware.md`](nohardware.md) — sim/no-hardware walkthrough (partly superseded).

**Prior-session reference notes** (may describe intended or later-redone states; trust the
source-of-truth files above where they conflict):
- [`livoxconnection.md`](livoxconnection.md) — earlier end-to-end session log (Livox + FAST-LIO + SUPER + FC).
- [`super_ws-CLAUDE.md`](super_ws-CLAUDE.md) — the `super_ws` workspace's CLAUDE.md (SUPER codebase overview).
