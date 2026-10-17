# The DTP follower stack — what it is, where it lives, and how it meets this repo

**Written 2026-09-10, revised 2026-09-12** from a read of the installed source and a live
inspection of the running system. Everything below is either quoted from that code or measured
here; where a number comes from the DTP authors' own notes rather than from a measurement taken in
this session, it says so.

> **The 2026-09-12 revision is substantial — the tree was worked on heavily through the night of
> 09-11.** What changed since the first version of this document: the detection rate dropped from
> 15 Hz to **10 Hz**, and later that day to **8 Hz**, on measurements; `command_mux` gained an
> **ABORT that lands the aircraft**;
> a **Stage A observe-only** flight mode appeared; `static_calib_tf` now publishes the TF link that
> was previously missing (and a 0.051 m lever-arm error in it was fixed); and
> `scripts/outdoor_depth_test.py` — which this document used to say did not exist — now does.
> One new defect was found in the process: **the SUPER patch was stale against the live source**
> (see Open items 1 — regenerated and wired into `setup.sh` on 2026-09-13).

## The one-paragraph version

The human-following perception and goal-generation stack **exists, is built, and runs on this
Jetson** — and since 2026-09-12 it is **`drone-dtp/` in this repository** (branch
`dtp-follower-stack`), reached through a symlink at `~/super_ws/src/drone-dtp`. It sits there as a
separate source tree, compiled into the same `~/super_ws` workspace as SUPER and FAST-LIO. It is,
in substance, the **M4 vision goal adapter** that this repo's `CLAUDE.md` still lists as TODO. It
turns Gemini 336L colour + depth into one locked human target in FAST-LIO's world frame, plans a
standoff viewpoint behind that person, and feeds SUPER a continuously-moving goal plus a yaw
target — through a patched SUPER ingestion path, not the RViz click path.

## Where it is, and the thing to fix first

| | |
|---|---|
| Path | **`drone-dtp/` in this repo** (branch `dtp-follower-stack`); `/home/ideas-ad/super_ws/src/drone-dtp` is a **symlink** to it since 2026-09-12 |
| Version control | Committed 2026-09-12 (`28d366b`, 105 files, identical to the old tree minus caches). Built `--symlink-install`, so edits in the repo are what runs. Old tree backed up at `~/drone-dtp.pre-symlink-2026-09-12` |
| Size | ~12.6k lines Python, ~1.3k lines C++ |
| Built into | `~/super_ws/install/{dtp_msgs,dtp_perception,dtp_bringup,command_mux,follow_planner}` |
| Authorship | The SUPER patch is by KrishnaRajule (2026-08-14); the calibration was captured under `/home/krishna/calib_runs/`, i.e. on another machine |

**How the tree is wired into `~/super_ws`, and how to edit, build, run and test it, is in
[`../drone-dtp/README.md`](../drone-dtp/README.md)** (2026-09-12). This document is about what the
code does.

**It is pushed** — `origin/dtp-follower-stack`, at `f4df2aa` on 2026-09-13. The risk this section
used to describe (twelve thousand lines of flight-relevant code, whose design rationale lives only
in its comments, on a Jetson that gets reflashed) now applies only to work left uncommitted or
unpushed in the tree — which, on 2026-09-13, includes the GPU organized cloud.

The code cites a design-document set — `plan/dtp-imp/01` … `plan/dtp-imp/21`, plus a `CLAUDE.md` of
its own — on nearly every page. **None of those documents are on this box** (re-checked
2026-09-12). The comments are the only surviving statement of why the system is shaped this way.

## How it meets this repo

- **It uses our renamed TF frames.** `dtp_params.yaml` sets `odom_frame: lidar_init` and
  `body_frame: lidar_imu` — the names `patches/fastlio_frames.py` introduced, now **read off a live
  `/lidar_slam/odom` rather than inferred**. But the DTP code's *comments and docstrings still say
  `camera_init`* in places, because they predate the rename. **When reading that tree, translate
  the same way `CLAUDE.md` says to translate upstream FAST-LIO docs.**
- **It depends on our deployed FAST-LIO extrinsic, and this repo's copy is the wrong one.**
  `static_calib_tf` needs `lidar_to_body_t` to equal FAST-LIO's `mapping.extrinsic_T`. The value
  is `[-0.011, -0.02329, 0.04412]` (|T| = 0.051 m) in the config FAST-LIO actually runs from,
  `~/super_ws/install/fast_lio/share/fast_lio/config/mid360.yaml`. **This repo's
  `config/mid360.yaml` says identity and is not the deployed file** — that mismatch is why the node
  reported "WRONG BY 0.051 m" on every launch from 2026-09-08 until it was fixed 2026-09-11. If the
  extrinsic is ever re-tuned, change both; the node reads FAST-LIO's live parameters back and logs
  at ERROR on a disagreement, so a stale copy is loud rather than silent.
- **It is the launch that owns the camera.** `gemini_330_series.launch.py`, serial
  `CPC64630007M`, publishing under `/camera` — exactly the holder the GCS camera panel is built to
  detect and refuse to fight over. The GCS's supervisor deliberately never matches it.
- **It patches SUPER.** `patches/0001-super-yaw-to-target-and-follow-goal.patch`, 10 files,
  creating `super_planner/config/follow_test_ros2.yaml`. **Since 2026-09-13 (`b78fbc3`) our
  `scripts/setup.sh` symlinks `drone-dtp/` into `~/super_ws/src` and applies this patch** (via
  `drone-dtp/scripts/apply_super_patch.sh`), so a clean install of this branch yields a SUPER the
  follower can drive — verified piece by piece, not yet by a full clean build.
- **It does not use our M3 bridge path for goals.** DTP publishes goals to SUPER; our
  `px4_offboard_bridge.py` still owns SUPER → PX4. The two halves have never been run together.
- **As of 2026-09-11 it also talks directly to PX4.** `command_mux` publishes
  `/fmu/in/vehicle_command` for the land-on-blindness ABORT, and subscribes
  `/fmu/out/vehicle_control_mode` and `/fmu/out/vehicle_command_ack`. That is a second writer to
  the FC alongside our two bridges — see the ABORT section.

## The pipeline

```
Gemini 336L colour (MJPEG) ─┐
                            ├─ perception_node ─ target_manager ─ follow_planner ─┐
Gemini depth / MID360 ──────┘       8 Hz          lock/coast        15 Hz         │
                                      ↑                                          ↓
MID360 ─ FAST-LIO ─ /lidar_slam/odom ─┴── static_calib_tf         command_mux (50 Hz, veto+abort)
                                              (the TF bridge)                    │
                       SUPER fsm_node ←── /planning/follow_goal ─────────────────┘
                                     ←── /planning/yaw_target          └→ /fmu/in/vehicle_command
                                     ──→ /planning/goal_status ──→ follow_planner    (NAV_LAND)

sensor_watchdog ──→ /dtp/sensors_ok ──────────────────────────────→ command_mux (hard veto)
```

### `perception_node` (1776 lines) — detect, range, lift, predict

A thin ROS wrapper around `detection_pipeline.HumanXZPredictor.process_frame`, which is ported
unchanged from a ground-robot reference project and is deliberately kept ROS-unaware. Per accepted
frame it builds the organized cloud, looks up `world←camera` **at that frame's own stamp**, runs
inference once, then does the four jobs `process_frame` does not: height, velocity, confidence,
latency compensation.

- **YOLO is `~/models/yolo11s.engine`** (TensorRT, `imgsz` 640, `conf` 0.25). *The first version of
  this document said `yolo26n.engine`; that was wrong.* `dtp_params.yaml` deliberately does not set
  `model`, so `perception_node`'s own default binds, and that default is `yolo11s.engine`. The
  engine is per-machine (GPU + driver + TensorRT version) and is deliberately not shipped.
- **Tracking is BoT-SORT, not ByteTrack** — `detection.py:620` calls `model.track()` with no
  `tracker=` argument, so ultralytics' default `botsort.yaml` (with `sparseOptFlow` GMC) is in
  force. **Its cost is now measured (2026-09-13, `scripts/bench_cloud_backend.py --yolo`, 200 real
  frames):** `track()` is **41 ms CPU / 60 ms wall** per frame against **18.5 ms / 34 ms** for the
  same engine under ByteTrack, so the GMC alone is ≈ 23 ms CPU and 25 ms wall — ~18 % of a core at
  8 Hz. It is the next CPU lever, but it is also what keeps IDs through yaw, so it is not simply
  switched off (see `m5-m6-plan.md` M5.3 item 5).
- `detect_every_n` is **pinned to 1 and the node refuses to start otherwise**. With >1 the previous
  frame's boxes index the current cloud while the track is marked freshly seen, so the torso sample
  walks onto the background with nothing on the wire to say so.
- **`detect_rate_hz` is 8.0 since 2026-09-12** (committed `dtp_params.yaml`). It was 10.0 for one
  day, and both values are measured. The 09-12 re-measure was on the dashboard-launched field stack,
  which is heavier: FAST-LIO with `extrinsic_est_en: true` at **74 %** CPU vs 31 %, this node at
  **137 %**, load 6.0 on 8 cores. At 10 Hz, TF wait was p99 **1393 ms** and 8.0 % of frames aged
  out; at 8 Hz it was p99 **136 ms** and **0.0 %**. Same subject and room: at 10 Hz the lock
  cycled four times and was LOST twice in 30 s, while at 8 Hz one acquisition held for the whole
  90 s. The yaml's comment has both tables. The first decision, **10.0 rather than 15.0**, was
  taken **2026-09-11 against the full navigation stack** (FAST-LIO + `fsm_node` + both PX4 bridges
  + the web GCS, all co-resident; the first time this node ran alongside the real flight stack
  rather than a bare FAST-LIO):

  | `detect_rate_hz` | published | blind | useful lifts | FAST-LIO CPU |
  |---|---|---|---|---|
  | 15.0 | 12.6 Hz | 37.4 % | 7.9 Hz | 44 % |
  | **10.0** | **9.9 Hz** | **2.2 %** | **9.7 Hz** | **31 %** |
  | 8.0 | 8.0 Hz | 0.0 % | 8.0 Hz | 29 % |

  15 Hz was worse on every axis — fewer usable world positions per second, a 17× higher blind rate,
  and more CPU taken from the SLAM the autopilot's position estimate depends on. It was never a
  15 Hz pipeline; it was a 12.6 Hz pipeline discarding a third of its own output.

  **The mechanism matters more than the number, because the symptom points elsewhere.** At 15 Hz
  the node saturates its **single-threaded executor** (~45–79 ms of decode + cloud build +
  inference against a 67 ms budget, at 250 % CPU), and a saturated executor starves **this node's
  own tf2 listener callbacks** — so its transform buffer goes stale while `/tf` is perfectly
  healthy on the wire. Frames then age out and publish blind reporting
  `extrapolation into the future ... latest data is at <476 ms ago>`, which reads exactly like a
  clock-offset or TF-cadence fault and is neither. Measured on the wire at the same moment: odom
  inter-arrival mean 100 ms, p99 117 ms, max 311 ms, **zero** gaps over 450 ms, and `/tf` within
  1–2 ms of the odom topic carrying the same pose. Nothing outside the node was late.
  10 Hz is also the *principled* value: the transform a detection is lifted through only updates at
  `/lidar_slam/odom`'s 10 Hz, so higher rates buy queue depth and contention, not new world-frame
  information. The architectural fix for the residual 2.2 % is a `MultiThreadedExecutor` with tf2
  and odom on their own callback group; it is recorded as a follow-up and deliberately not landed
  the day before a first flight.
- **The organized H×W×3 cloud is built in-process, never sent over DDS.** 1280×720×3 float32 is
  11.06 MB per message; at 15 Hz that is 166 MB/s into an rclpy subscriber, and this box already
  loses ~8 % of 2.76 MB colour frames at 30 Hz. `depth_stride` is 1, not `depth_source`'s default
  2: at stride 2 under 10 % of the cloud is non-NaN and a torso patch at 8 m yields ~13 samples
  instead of ~52.
- **The cloud is built on the GPU since 2026-09-13: `cloud_backend: cuda` in `dtp_params.yaml`**
  (the node's own default stays `numpy`). The chain — depth deprojection, the depth→colour
  transform, the nearest-return z-buffer, the 1280×720×3 build — was the node's single largest
  CPU cost, **112 ms CPU / 38 ms wall per frame (90 % of a core at 8 Hz)**. It now runs in
  `dtp_perception/cuda_cloud.py` (torch, on the GPU YOLO already owns) at 10–14 ms CPU.
  - **Bit-identical to numpy on 200/200 real frames and on every synthetic test scene** — 0
    differing pixels, 0 m. The code still judges it against a tolerance, because identity of the
    float64 transform isn't guaranteed across library versions.
  - **Replaying the same bag through the real node twice per backend:** 1.9 cores → 0.7.
    **Per-frame wall time did not improve** (GPU shared with YOLO), so it buys CPU headroom, not
    latency.
  - **End-to-end target outputs:** on an identical frame schedule, numpy vs cuda differ no more
    than two numpy runs differ from each other (p95 1.65–2.18 cm vs 1.71 cm; track ids identical).
  - **In-node safety net:** the first 5 frames are built both ways and the numpy cloud is flown
    until they agree. A disagreement, 3 consecutive CUDA errors, or no CUDA at start (libcudss
    missing from `LD_LIBRARY_PATH`) reverts to numpy for the life of the node, at ERROR. The
    per-frame log line reads `cloud 28.5 ms [cuda]`.
  - **Tools:** `scripts/bench_cloud_backend.py` (equivalence + cost from a bag),
    `scripts/replay_perception.sh` + `compare_replay_targets.py` (the real node, end to end); tests
    in `dtp_perception/test/test_cuda_cloud.py`.
  - **Not yet run on the aircraft** — that is `m5-m6-plan.md` row 14.
- **Never "latest" TF.** Frames queue up to `max_pair_age_s` (0.45 s, raised from the module's
  0.25 s after frames were measured ageing out at 0.307 s) waiting for odom to cover their stamp.
  A frame TF never covers is **dropped blind, never guessed** — with `T=None`,
  `_camera_xz_to_odom_xz` returns camera-frame coordinates that downstream would consume as world.
- **`tf_at_latest: false` is now flight-legal, and that is new.** From 09-08 to 09-10 it had to be
  `true` on the bench because the Orbbec stamped colour from its own clock (`time_domain=global`):
  53 ms off ROS time on 09-08, 30.5 ms on 09-09, and a stamp-exact lookup never succeeded. **The
  fix was at the source** — `sensors.launch.py` now passes `time_domain:=system` (default in the
  launch file), leaving a +5.6 ms residual that is latency, not skew. Verified 09-10 with SLAM up:
  `perception_ok` true on every frame, `/dtp/targets` 13.3 Hz, stamp age at publish ~0.21 s.
  Setting it true again is only ever a *diagnosis* of a clock problem, never a fix for one.
- `max_depth_colour_skew_s` is **0.20**, not the code's 0.025: the live colour↔depth *delivery*
  skew was measured at ~0.067 s on 09-08, so every frame was being rejected and the node published
  an empty `TargetArray` at a healthy 14 Hz — blind, not quiet.
- **The range statistic deliberately diverges from the frozen module, since 2026-09-09.** It is
  now **nearest-surface**: 20th percentile to locate the near surface, keep samples within 0.45 m
  of it, medians inside. A median rejects outliers *within* one surface but cannot choose *between*
  two, and the patch at 0.65 of box height straddles the silhouette at close range, so it flipped
  between the subject (~2.5 m) and the wall behind (~4 m). Measured over a 194 s walk
  (`bag_follow2`, 09-09): the range gate rejected **62 % of frames** — 117 range-jump holds against
  21 inlier holds — velocity read a median 0.03 m/s while the subject walked at ~1.4 m/s, the
  forecast collapsed to a single point, the coast died in 0.11 s, and `target_manager` sat in LOST
  for the whole run. One statistic, five symptoms. The held-frame rate was 79.8 % at 2–3 m against
  3–5 % at 1–2 m and 5–6 m — **not monotonic in range**, which is what ruled out sparsity and
  pointed at edge-straddling. The patch **geometry** still has to match `detection.py:432-478`
  exactly; only the statistic over it is ours, and `_assert_patch_identity` is the runtime tripwire
  on that (tolerance widened to 1.5 m so surface selection does not trip it).
- Per-track **range gate**: a 2-state (range, range-rate) Kalman filter on the raw patch range,
  σ = 0.05 + 0.02·r, 2 m/s² process accel, reject outside max(4σ, 0.35 m), at most 3 consecutive
  holds then reset. It gates what is *published*; it cannot clean what the frozen predictor already
  folded into its own history.
- `predicted_path[0]` is defined as where the person will be **`latency_compensation_s` = 0.20 s
  after `header.stamp`**. Both consumers are written to that and neither re-compensates. The 0.20 s
  is a *derived* budget (~214 ms mean, ~520 ms worst) and **no leg of it is an end-to-end
  measurement**; over-compensation is the dangerous direction because it shortens the real standoff.
- Publishes `/dtp/targets` **every processed frame, including empty ones** — mandatory, because an
  empty array is a positive "alive and evaluated, nobody here" while a skipped publish is
  indistinguishable from a dead node. Blindness is stated separately on **`/dtp/perception_ok`**,
  which `sensor_watchdog` folds into `/dtp/sensors_ok`.
- Also publishes **`/dtp/detections_2d`** (added 2026-09-13), for viewing only: the same frame's
  boxes in colour-image pixels as JSON in a `std_msgs/String`, one row per detection (`id`, `box`,
  `patch` — the range-sample centre — `conf`, `confirmed`, `held`, and `xyz`/`vel`/`dist` when a
  Target was built), with the colour frame's stamp inside. Blind frames carry `ok: false` and the
  reason. JSON rather than a `dtp_msgs` type so `dtp_msgs` — which SUPER's `fsm_node` links — did
  not need a rebuild. Built **only while something subscribes**, and only after `/dtp/targets`
  and `/dtp/perception_ok` have gone out, so it cannot cost the veto a message.
  `publish_detections_2d: false` removes the publisher. See `dtp_image_overlay` below.
- **Cost: ~250 % CPU measured 2026-09-11** at 15 Hz; 137 % at 8 Hz on 09-12. **The 09-11 note said
  this was "dominated by JPEG decode and the numpy organized-cloud build" — half wrong, per the
  2026-09-13 per-stage measurement:**

  | stage | CPU per frame | % of a core at 8 Hz |
  |---|---|---|
  | organized cloud, numpy | 112 ms | 90 % |
  | YOLO track, BoT-SORT with GMC | 41 ms | 33 % |
  | JPEG decode | 5.8 ms | 5 % |

  The cloud now runs on the GPU (above): **≈ 0.7 cores** in a bag replay, against 1.9 with numpy.

### `target_manager` (1274 lines) — lock one person, never guess

No equivalent in the reference project, which treated every human as a hazard to avoid rather than
one to follow. States: `IDLE → ACQUIRING → LOCKED ⇄ COASTING ⇄ ABSTAIN → LOST`.

Engage is `ros2 param set /target_manager follow_enabled true` — a **rising edge**, not a level, so
LOST latches until the operator re-designates. Disengaging is instant; engaging is hysteretic.

1. **Lock** requires all of: a *confirmed* track, 2.5–10 m, outside `launch_ignore_radius_m` 3.0 of
   the latched launch point, **0.40 m of travel within a 6 s window**, and 3 consecutive frames as
   the best candidate. `acquire_max_range_m` is pinned to `perception_node`'s declared
   `prediction_depth_gate_m` (10.0, not the library's 6.0) because a target with no forecast cannot
   be coasted — acquiring someone we could not coast through one tree is a lock that dies on the
   tree.
2. **The motion gate is new-ish and it has a measurement behind it (2026-09-09).** Over a 194 s
   recording with *one* human present, 66.9 % of frames carried more than one detection, and the
   most persistent track — 535 of 568 sampled frames at confidence 0.85 — was a **humanoid robot
   standing in the lab**, against the real subject's 0.94. Ten of thirteen persistent detection
   clusters in that room never moved. Confidence cannot separate those and neither can appearance;
   movement can. Gated on **acquisition only** — once locked the subject may stand still as long as
   they like. The honest limitation: it cannot distinguish a statue from a person who has not moved
   since the node started, so it converts that into a requirement on the operator (take one step).
3. **Occlusion** → coast the CTRV prediction forward by evaluating the predictor's own polyline at
   the current age, never past its end. Re-acquisition band 1.0 m growing at 1.0 m/s, capped 2.5 m.
4. **Re-acquire**: exactly one candidate in the band *and* confirmed → re-lock. **Two or more →
   ABSTAIN** and publish nothing; still ambiguous after 0.5 s → LOST. The band is a bounded-jump
   gate, not a confidence region — what prevents a wrong lock is the ABSTAIN rule.
5. Non-locked people are neither discarded nor republished — they are already on `/dtp/targets`.

Publishes `/dtp/follow_target` **only** while LOCKED or COASTING. IDLE/ACQUIRING/ABSTAIN/LOST
publish nothing at all. It has **no timer**: one publish per received `TargetArray`, so its
liveness is derived from perception's rather than from a clock that keeps ticking after the pipeline
behind it dies.

Two subtleties worth not re-deriving:

- **`header.stamp` is never rewritten, including while coasting.** SUPER hard-gates
  `/planning/yaw_target` on its age (`kYawTargetTimeoutSec` 0.5 s), and restamping would certify an
  arbitrarily old extrapolation as fresh perception. The accepted cost: past ~0.5 s of coasting
  SUPER falls back to velocity-aligned yaw, exactly when pointing the camera at the predicted
  position would most help.
- **`_rebase_retained_forecast` is load-bearing.** Perception sends an *empty* forecast on a HELD
  frame, so the message arrives carrying a single fallback point — a 0.20 s horizon instead of
  1.5 s. Rebuilt naively that replaces a full polyline with a stub and the next missed frame ends
  the coast immediately. And holds are not independent of occlusions: a patch loses inliers when
  the person is going *behind* something, so "held frame then occluded" is the ordinary onset of
  the event the coast exists to survive. Read that docstring before touching the coast.

The usable coast is the predictor's horizon minus one latency compensation: **1.5 − 0.20 = 1.30 s**,
so `coast_timeout_s` (1.5) never actually fires and tuning it changes nothing. The lever for a
longer coast is the predictor's `prediction_secs`.

> ⚠️ **`auto_reacquire_after_lost` is `false` in the committed `dtp_params.yaml` since 2026-09-12,
> and must stay false for any flight where goals reach SUPER.** A re-acquisition after LOST has no
> forecast left to match against and degenerates to "nearest confirmed body", which is how the
> drone follows a stranger; after a LOST, a human re-designates by toggling `follow_enabled`.
> `stage_a_preflight.py --stage-b` refuses to GO on true. It was `true` from 09-09, for bench and
> observe-only work: on the 09-09 recorded walk the node entered LOST once and stayed there for the
> whole 195 s — 184 `lock_state` messages, one distinct state — and `/dtp/follow_target` and
> `/dtp/follow_goal` both recorded zero messages as a result. The node logs `true` at ERROR on every
> launch and after every live parameter change.

### `follow_planner` (C++, 1262-line header) — the standoff viewpoint

> **Added 2026-09-19/20 — `goal_mode: geometric | nmpc`.** Everything below describes the
> `geometric` goal, which is the default and the only one that has flown. `goal_mode: nmpc`
> replaces ONE function, `ComputeStandoffGoal`, with a receding-horizon optimisation of the
> *goal's* positions over the forecast (`nmpc_follow.hpp`, ROS-free, an L-BFGS vendored from its
> MIT upstream). Every guard described in this section still wraps its answer, both arms end in
> the same `ApplyHardGuards()`, the geometric goal is still computed every tick (it is the cold
> start, the sanity reference and the fallback), and a refused solve publishes that tick's
> geometric goal — it never Holds or Suspends. Stage 2 adds a visibility cost fed by
> `dtp_msgs/VisibilityField` from `fsm_node` (SUPER patch 0008, not applied to the live tree),
> off unless `nmpc_w_vis` > 0. Bench-gated, never flown; the installed binary predates it.
> Full record, 19 bench traps and the gates: [`fact-nmpc-follow-goal.md`](fact-nmpc-follow-goal.md).
> The header is now ~1 900 lines, and `dtp_msgs` must be built before this package.

Turns the locked target plus its predicted path into a moving goal. **The work is in a 15 Hz timer,
not the target callback** — every loss condition it owns is a non-event, defined by something that
stopped arriving, and a callback-driven node cannot detect its own silence.

Geometry: band `follow_near_m` 3.0 – `follow_far_m` 4.0 with a hard `min_standoff_m` 2.5 floor,
commanded at the band **midpoint** 3.5 m (the near edge leaves only 0.5 m over the floor and the
latency compensation already spends ~0.3 m of it). Goal z = target z + 0.5, clamped
[`goal_z_min_m`, 3.0] — outside ROG-Map's virtual ground/ceiling (−1.0, 3.5) a goal comes straight
back DROPPED. **`goal_z_min_m` is 1.2 in `dtp_params.yaml` since 2026-09-17** (node default 0.5):
the tracked point reads near the ground, not the torso the code assumes, so the floor *is* the
follow height — 1.2 in `lidar_init`, ~1.35 m above ground
([`fact-follow-speed-and-descent-guard.md`](fact-follow-speed-and-descent-guard.md)).
Near-side bias ≤ 45°; `goal_deadband_m` 0.15; park guard 0.25 m.

**The lead and the standoff pull on the same axis, so the lead is budgeted.** `lead_distance_m`
1.0 is a *ceiling*, consumed as **arc length** along the CTRV path so it fades to zero by itself
when the person slows. The spendable amount is `standoff − min_standoff_m − 0.25 reserve −
prediction gap` ≈ **0.47 m** at walking pace, putting the commanded goal 2.75 m from the person by
construction. A desk simulation against a first-order drone with τ = 0.3 s settled at a 3.13 m
achieved standoff, never commanded closer than 2.75 m, and never engaged the park guard — desk
numbers off an assumed τ, which say the geometry is self-consistent and nothing more.

Two things its header insists on:

- **The yaw target topic is `/planning/yaw_target`, not `/dtp/target_point`.** `fsm_node` listens
  only to the former. With nothing arriving, SUPER falls through to velocity-aligned yaw and the
  drone flies a perfect standoff **while never looking at the person** — silently. The yaw target
  is published **even on ticks where the goal is withheld**, deliberately: it moves no mass, so the
  camera keeps tracking through a handover and the lock stays alive.
- **Silence is the loss signal in both directions.** When it cannot produce a trustworthy goal it
  **stops publishing** rather than republishing a frozen one — that is what trips `command_mux`.
  It holds on: target stale (0.3 s), odom stale (0.3 s), person outside the ±35° camera cone for
  >1.0 s, suspended, degenerate geometry, or the goal itself inside the keep-out.

It reacts to `/planning/goal_status`: ACCEPTED; SNAPPED (≤ `snap_accept_m` 0.5 m adopted silently —
SNAPPED is the *common* case on hardware, not an edge case); DROPPED, where **the previous goal is
still active and SUPER is still flying to it**. Five consecutive large snaps or drops →
`Suspend()`, going deliberately silent until the target moves 1.0 m. A status that never arrives is
treated as *ambiguous*, not as a rejection — otherwise FOLLOW could never win the mux back after
losing it.

**Correlation is on `requested_p`, not `goal_p`**, and this is the field the SUPER patch is now
missing (Open items 1). `/planning/follow_goal` has several publishers — `follow_planner`,
`command_mux`'s HOLD and NAV goals, and now the web GCS — and `fsm` answers every message on it
with exactly one status. On DROPPED, `goal_p` is the *previous* goal and bears no geometric
relation to the request, so correlating on it let statuses answering mux HOLDs fall into the
SNAPPED branch, read as ~3 m snaps, and suspend the follower mid-follow. `requested_p` was added to
`dtp_msgs/GoalStatus` and echoed in `fsm_ros2.hpp` on 2026-09-08 to fix that.

`TransitCrossesKeepOut` also carries a fix worth knowing: it originally clamped the closest-approach
parameter to `[0,1]`, so `t=0` was the drone's own position and the guard latched true forever
whenever the drone was already near the person — which is the normal state of a follower. It now
skips the first `transit_clearance_m` of travel and tests only the goal and the final approach; the
lead-in belongs to SUPER and to the mux's runtime veto.

### `command_mux` (1167 lines + `abort.py` 157) — arbitration, the veto, and ABORT

Priority **SAFETY > manual RC (enforced by PX4) > FOLLOW > NAV**. 50 Hz, and exactly **one** place
in the file publishes a goal, reached from one timer, because a second path is a path that can skip
the veto.

- The **human-proximity veto** subscribes to `/dtp/targets` directly — every detected person, not
  the locked one and not the goal — so a bug in `follow_planner`'s standoff maths cannot defeat it.
  The keep-out is a **cylinder** (`proximity_veto_radius_m` 1.5 × ±2.0 m), tested as the predicted
  closest approach of the drone→goal segment plus each person's forecast out to 1.5 s. Release
  needs 0.5 m of margin *and* 1.0 s of dwell, or a person walking parallel at exactly the floor
  makes the mux chatter at tick rate.
- It checks **both** the goal the source asked for and the goal after its own rewrites. The descent
  guard raises a goal's z to an altitude floor, which flattens the segment the veto gates on —
  so an unchecked rewrite could switch the proximity veto off on exactly the trigger the two share.
- **The descent guard latches (since 2026-09-17, user decision).** A goal below the drone with any
  person within `descent_guard_radius_m` (3.0 m) of the drone or the goal latches the drone's
  altitude as a floor (`command_mux/descent_guard.py`); the floor rises only when the drone is
  > 0.3 m above it, releases after nobody is within 3.5 m for 1.0 s, and resets on disarm;
  `/mux/state` appends `| alt floor Z`. Re-decided every tick, as it was, it flapped the goal's
  altitude at the follow standoff (2.77–3.07 m from the walker on 2026-09-15). It guards goals,
  not SUPER's path. Gate: `drone-dtp/scripts/descent_guard_check.py` (real node, DDS domain 86,
  11/11); never flown. Full record:
  [`fact-follow-speed-and-descent-guard.md`](fact-follow-speed-and-descent-guard.md).
- **Every veto input fails closed on absence**, not just on a bad value: `/dtp/sensors_ok`,
  `/dtp/targets`, `/lidar_slam/odom`, and any non-finite number anywhere (`nan < radius` is False,
  so a NaN is a fail-*open* bug sitting inside a safety check).
- **Silence is not a stop at the goal level.** `setFollowGoal` stores the goal and SUPER keeps
  flying to it, so every stop condition publishes a **HOLD goal**: latched on entry (a hold point
  recomputed from live odom runs away from a decelerating drone), +0.75 m climb, displaced by at
  least 0.15 m — inside `closeToGoal(0.1)` the FSM returns to WAIT_GOAL *without planning* and the
  drone finishes the trajectory it is already on — and re-anchored if the drone ends up >3 m away.
- Until one fully-healthy tick has passed it publishes **nothing at all**, because publishing is
  what latches SUPER's `started_` flag and hands it a goal to fly.
- NAV is **latched** (RViz publishes one message and means "fly there"); FOLLOW is **streamed** and
  loses authority the moment it goes stale. NAV enters on `/dtp/nav_goal`, never `/goal_pose` —
  subscribing to the topic it publishes would be a feedback loop, not arbitration.
- A veto blocks **NAV as well as FOLLOW**. An operator cannot click the drone away from a person it
  has vetoed on; the independent last line of defence is the RC pilot's mode switch.
- **The geofence lives here (added 2026-09-14, M5.2, user decision).** PX4's own fence is inert
  without a global home, so `command_mux/fence.py` holds a rectangle in `lidar_init` (default the
  plan's 30 × 4 m, x ∈ [−2, 28], y ∈ [−2, 2], `fence_yaw_deg` turns it; **re-cut per venue**). Every
  goal the node publishes — FOLLOW, NAV and HOLD — is clamped 0.5 m inside the edge, where it is
  shaped and again at the publish point. The **aircraft** more than 3 m outside raises a `FENCE`
  veto that escalates to a Land like a sensor loss. A box that excludes the origin is refused at
  startup. It does not cover `/goal_pose` clicks or the GCS RC card, which bypass the mux. Gate:
  `drone-dtp/scripts/fence_check.py` (real node, private DDS domain, 13/13); never flown.

#### ABORT — land on blindness (added 2026-09-11, user decision)

**This is the one place in the DTP stack that changes a PX4 flight mode with no human in the loop,
and it is new since the first version of this document.**

A HOLD is the right answer to a person too close or a follow source gone quiet. It is not the right
answer to the aircraft going blind: four silent sensor losses on the bench in one week — three
camera deaths and one LiDAR link drop, every one *process-alive-and-publishing-nothing* — make
"hold with no perception and possibly no SLAM" a real flight state, and that is a drone hovering on
whatever PX4's EKF still believes, unable to see anyone walking under it.

- **Escalates:** `SENSORS`, `TARGETS`, `ODOM` — the vetoes that mean "we can no longer see or place
  ourselves" — and, since 2026-09-14, `FENCE` (the aircraft past the geofence's hard boundary while
  every goal already reads "inside": not doing what it is told). **Never escalates:** `PROXIMITY` (landing next to the person is worse than holding
  above them), `GOAL_INVALID`, or a missing source (those are the follow chain failing, not the
  aircraft going blind).
- Fires only after the condition **persists `abort_after_s` 2.0 s** *and* the stack has authority:
  the mux has asserted at least one goal **and** PX4 reports **armed + Offboard** on a
  `/fmu/out/vehicle_control_mode` no older than 3 s. Losing authority clears the timer, so a
  condition that began while the pilot had the drone cannot fire the instant Offboard returns.
- Sends `MAV_CMD_NAV_LAND` on `/fmu/in/vehicle_command` — the same command QGC's Land button sends
  — resent every 0.5 s up to 10 times until PX4 acks. `TEMPORARILY_REJECTED` retries;
  `DENIED`/`UNSUPPORTED`/`FAILED` stops and logs *"PILOT, TAKE IT."*
- **Terminal.** After firing the mux stays in HOLD arbitration with an ABORT veto until the node is
  relaunched, so a camera that plugs back in mid-descent cannot quietly resume following.
  `/mux/state` reads `ABORT|LAND|<why>`; `/dtp/abort` is latched for the log.
- The asymmetry is deliberate and mirrors our own bridge's rule: it can take the aircraft **down**,
  and can never arm it, take it up, or put it in Offboard. The RC pilot's mode switch overrides it.
- The whole decision lives in `command_mux/abort.py` with no ROS in it, unit-tested with a fake
  clock. `abort_require_offboard:=false` exists **only** to exercise the path against a disarmed FC
  and is logged at ERROR every 5 s while set. `abort_enable:=false` is for Stage A.

### `static_calib_tf` — the TF link that did not exist (new since 09-10)

Without this node the entire follower is inert, and the first version of this document recorded the
gap as unowned. FAST-LIO publishes `lidar_init → lidar_imu`; the Orbbec driver publishes
`camera_link → camera_color_optical_frame`; **nothing published `body → camera`**, so every
`lookup_transform` failed and `perception_node` published empty arrays at full rate.

It composes the bridge rather than transcribing numbers:

```
T_body__optical = T_body__lidar @ inverse(T_camera__lidar)
```

`T_camera_lidar` is loaded through `calib_io.load_for_node`, which is not a file read but a
**start-up identity gate**: it refuses a calibration belonging to a different camera body (two
Orbbec bodies were attached to this NX on 09-07 and the driver bound whichever enumerated first)
and a LiDAR stream from another robot on the same DDS domain (which also happened). `T_body__lidar`
comes from the `lidar_to_body_*` parameters, which must equal FAST-LIO's deployed extrinsic — the
0.051 m error above.

**The trap it exists for is two parents.** The obvious `static_transform_publisher body
camera_color_optical_frame` is wrong in the worst way: intermittently. The Orbbec driver already
publishes a parent for that frame; tf2 is a tree, its static cache is keyed on the **child**, and
both publishers are TRANSIENT_LOCAL — so each subscriber receives both in whatever order discovery
hands them over. Win the race and the lookup works; lose it and `lidar_init` is unreachable. So the
node finds the **root** of whatever chain already claims the camera frame and bridges to that,
composing the driver's own chain in. Everything in the file uses **zero** tf2 lookup timeouts and
explicit spinning, because a tf2 timeout blocks the calling thread and spins nothing.

The body frame name is a parameter with **no default** — it is `/lidar_slam/odom`'s
`child_frame_id`, nothing in this repository recorded it, and a wrong-but-plausible frame id fails
exactly like a missing one while looking checked. It is then verified against the live topic.

### `sensor_watchdog`

`stale_periods: 10.0`, `rate_floor: 0.7`, `startup_grace_s: 20.0`, 0.2 s tick. Watches only the
**tiny companion topics** — `/camera/color/camera_info` and `/camera/depth/camera_info` at 30 Hz,
`/lidar_slam/odom` at 10 Hz — because `camera_info` tracks the image cadence to 0.01 Hz at
1/2000th the bytes, and a Python subscriber on a 24 MB/s topic would invent the outages it exists
to detect. It distinguishes **STALLED** (was publishing, stopped — the 09-07 Orbbec failure) from
**NEVER PUBLISHED** (start-up ordering), logs on transitions only, and folds in
`/dtp/perception_ok` so an alive-but-blind pipeline stops the drone instead of reading as a calm
one. Started at the **top level** of `dtp_system.launch.py`, not inside the follower layer, so
`follower:=false` cannot disable it — a NAV-only flight is still a flight.

### `dtp_viz` and `dtp_image_overlay` — seeing it from a laptop RViz

Both are passive (nothing in the flight path subscribes to either), both are started by
`dtp_viz.launch.py` and therefore by `dtp_system.launch.py` (`viz:=true`, the default), and the
laptop side needs no build: `JETSON_IP=gypsy.local bash scripts/laptop_dtp_viewer.sh` opens
`rviz/dtp_viewer.rviz` with both.

- **`dtp_viz`** turns the stack's state into one `MarkerArray` on `/dtp/viz`: people as cylinders,
  the proximity-veto keep-out, the locked target's forecast, the standoff goal, and a status text
  block. `scripts/verify_dtp_viz.py` is its gate.
- **`dtp_image_overlay`** (added 2026-09-13) is the camera picture with perception's boxes on it:
  `/dtp/detections_2d` drawn onto `/camera/color/image_raw/compressed`, published as
  `/dtp/image_annotated/compressed` (`sensor_msgs/CompressedImage`, best effort). Locked target
  green, other confirmed tracks blue, unconfirmed grey, a range refused this frame dashed amber, the
  range-sample point magenta, and a header with lock state, mux state, detection rate and how far
  behind live the picture is (red when the frame was blind).
  - **Boxes are drawn on the frame with their own stamp**, not on the newest frame. Boxes arrive
    100–300 ms after their frame, so newest-frame drawing would trail a walking person and look
    exactly like a tracking fault. The node keeps 3 s of JPEGs keyed by stamp. Boxes whose frame
    never arrived within 0.3 s are dropped rather than drawn on another picture; a frame one period
    off is used and labelled "nearest frame, N ms off".
  - **Idle until watched.** It subscribes to nothing — not even the camera — until its output
    has a subscriber, and drops everything 5 s after the last one leaves. While perception is
    silent it publishes the live camera at `live_fps` (4) under an amber "no boxes" banner.
  - **Parameters:** `output_width` (960; 640 decodes at half size, about 2× cheaper),
    `jpeg_quality` (80), `live_fps`, `always_on` (draw with no viewer, e.g. to record it).
  - **Measured on a bag replay (2026-09-13):** 17 % of a core while watched, ~85 kB frames at
    ~10 Hz. It is 40 % if image bytes are assigned to the message as `bytes` rather than
    `array.array('B')` — rclpy then validates a `uint8[]` element by element in Python — and 23 %
    with OpenCV's default 8 threads, hence `cv2.setNumThreads(1)`.
  - **On the laptop**, `scripts/laptop_image_decode.py` turns the JPEG into the raw
    `/dtp/image_annotated/laptop` that RViz shows (7 % of a core). Humble's RViz Image display cannot
    show a CompressedImage, and `image_transport republish` is no substitute: it subscribes RELIABLE
    with no override, so it never matches a best-effort publisher, and it needs a plugin
    `ros-humble-desktop` does not install. The decoder publishes only while every subscriber is an
    RViz/rqt node, because the raw topic is visible on the Jetson too, and a `ros2 bag record -a`
    there would pull ~12 MB/s back over the Wi-Fi.
  - **Tests:** `dtp_perception/test/test_dtp_image_overlay.py` (10) covers the hand-rolled CDR read
    against rclpy's serialiser, own-stamp pairing, late and never-arriving frames, the live
    fallback and box placement.
  - **First thing it showed:** the range patch (0.65 of box height, `PATCH_CENTRE_H_FRAC`) sits on
    a standing person's thigh, not on the torso this document and `perception_node` describe.

### `fusion_node` — off by default, and must stay out of the flight path

A debug publisher of the organized cloud for RViz/bags only. `enable` defaults **false**, and that
gate exists because the other two gates (`publish_rate_hz`, subscriber count) stop it *publishing*
but not *subscribing* — a constructed depth source pulls ~24 MB/s forever and becomes a competitor
to `perception_node` for the same frames.

### The loss oracle and loss conditioning (M6, built 2026-09-14, never flown)

`fsm_node` now answers every COASTING `/dtp/follow_target` with a `dtp_msgs/LossEvent` on
`/planning/loss_event` — OCCLUDED / UNMAPPED / LOST / OUT_OF_VIEW against ROG-Map, plus the
occluder's edges, a reappearance belief (32 CTRV rollouts, clustered by side) and a
pre-positioning viewpoint (SUPER patch `0004`; patch `0005` makes the rollouts slide round mapped
occupancy and adds the look-behind reduction; `loss_oracle:` in `follow_test_ros2.yaml`; it only
reads the map). `target_manager` and `follow_planner` act on it only with `loss_conditioning:=true`
(one launch argument sets both; `dtp_params.yaml` ships false = the paper's carry-forward arm):
an OCCLUDED coast runs to the belief's window (cap 3 s) with the re-emergence *region* as the
re-acquisition gate, LOST/UNMAPPED end the coast after 0.3 s, the goal becomes the viewpoint and
the yaw target the re-emergence point. `stock_baseline` on both nodes is the M6.8 naive follower.
Gates, parameters, replay tooling and the honest open list: [`m6-loss-oracle.md`](m6-loss-oracle.md).

## Why the SUPER patch exists

Upstream SUPER's `/goal_pose` path (`setGoalPosiAndYaw`, `fsm.cpp:184`) does one of three things to
a goal and reports none of them: uses it, **snaps** it to the nearest free cell within a hardcoded
3 m, or **drops** it outright and keeps the previous goal. It also overwrites goal Z with
`click_height` and skips any goal within 0.1 m of the robot. For a continuously-moving standoff
goal that is unusable, so the patch adds:

- `fsm/follow_goal_en` + `follow_goal_topic` (`/planning/follow_goal`) — an ingestion path
  (`setFollowGoal`) that **preserves Z**, applies no `click_height`, sets `started_`, and returns
  what it did;
- `goal_status_topic` publishing `dtp_msgs/GoalStatus` — ACCEPTED / SNAPPED / DROPPED, with the
  adopted point **and the requested point echoed back**;
- **(second patch, `patches/0002-rog-map-target-exclusion.patch`, 2026-09-13)**
  `rog_map/target_exclusion`: the M5.3a target-point filter.
  - **Where it lives:** in ROG-Map (`ProbMap::raycastProcess`), with the `/dtp/follow_target`
    subscription in `fsm_ros2.hpp`, so `rog_map` itself needs no `dtp_msgs`.
  - **Constraints:** default OFF; it applies only on top of 0001; it rebuilds `rog_map` as well as
    `super_planner`.
  - **Record in a bag:** it publishes the cylinder on `rog_map/target_exclusion` (a standard
    `Marker`, stamped with the scan it filtered).
- **(third patch, `patches/0003-rog-map-virtual-plane-ray-clip.patch`, 2026-09-13)** an upstream
  ROG-Map bug fix, always on. `raycastProcess` clipped a ray to a return below
  `virtual_ground_height` / above `virtual_ceil_height` to `(p - odom).normalized() * pc/dz` —
  a point **within 1 m of the sensor** — so that ray cleared nothing. With the floor below −1.0 in
  `lidar_init` a walked-away person's lower body never cleared. Now `(p - odom) * (pc/dz)`.
- **(eighth patch, `patches/0008-super-visibility-field.patch`, 2026-09-20 — NOT applied to the
  live tree)** `loss_oracle/visibility_enable`: the loss oracle also publishes
  `dtp_msgs/VisibilityField` on `/planning/visibility` for every locked frame, seen or coasting —
  per azimuth about the person, how far out a viewpoint clears the inflated map and what
  fraction of the person it would see. It feeds `follow_planner`'s `goal_mode nmpc`. Off in code;
  the oracle's `LossEvent` output is unchanged (30/30 identical, old binary vs patched). Patches
  0004–0007 are described in `m6-loss-oracle.md`, `fact-super-yaw-hold.md` and
  `fact-rogmap-unhit-decay.md`; this one in [`fact-nmpc-follow-goal.md`](fact-nmpc-follow-goal.md).
- a `YAW_TO_TARGET` yaw mode (`yaw_mode: 3`) that aims from the **trajectory's end** at the person,
  and therefore works even when the goal is beyond the planning horizon so `connected_goal` is
  false — which is exactly the case `YAW_TO_GOAL` cannot serve. A target older than
  `kYawTargetTimeoutSec` 0.5 s falls back to velocity-aligned yaw.

**`follow_goal_en` defaults to false** (`fsm/config.hpp`). If it is left false, `fsm_node` creates
neither the subscriber nor the status publisher, the mux publishes into a topic with no listener,
and the drone simply never receives a goal — with no error anywhere. `follow_test_ros2.yaml` is the
only config the patch ships with it true, and it also carries `yaw_mode: 3`. This repo's `setup.sh`
also sets both in `click_smooth_ros2.yaml`: `follow_goal_en` since 2026-09-10, and `yaw_mode: 3`
since 2026-09-12 (see [`fact-super-yaw-mode.md`](fact-super-yaw-mode.md)).

`scripts/apply_super_patch.sh` applies it by **content-anchored insertion**, not `git apply`, for
the CMake/package.xml half: SUPER keeps ROS1/ROS2 build files as templates in `super_planner/ros/`
that are *copied* into the active files, so a diff against the templates is a silent no-op on any
working checkout and the build then fails on a missing `dtp_msgs/msg/goal_status.hpp`. A failing
`git apply --check` against an already-patched tree is the expected state, not a fault.

### The web GCS shares this path (2026-09-10)

The dashboard's Remote control card publishes on **`/planning/follow_goal` too**, and
`scripts/setup.sh` enables `fsm/follow_goal_en: true` in `click_smooth_ros2.yaml` — the bench/click
config, *not* `follow_test_ros2.yaml`. Same reasoning as the patch: the click path overwrites Z and
silently drops a goal under 0.1 m, so a dashboard altitude control cannot work on it.

**The two must not drive at once, and the GCS refuses rather than interleaving.**
`rc.choose_path()` counts publishers on the topic; the GCS creates its own at node init, so **more
than one means the follower or `command_mux` is driving**, and every dashboard command is refused
naming it. The follower is unaffected either way — it never looks at who else publishes. If you are
running the DTP loop, use its controls, not the card. Note the GCS is a *third* publisher on a topic
whose `GoalStatus` echo exists precisely because there was already more than one; nothing needs
changing in `follow_planner`, which correlates on `requested_p`.

## Stage A — the observe-only flight mode (new 2026-09-11)

This is the bridge between the bench and a follow flight, and it did not exist when this document
was first written.

```bash
ros2 launch dtp_bringup follower.launch.py observe_only:=true
```

The navigation stack flies its own mission. The follow stack runs alongside — detecting, locking,
computing standoff goals and yaw targets — and **every one of those is recorded and none is flown**:
both outputs that reach `fsm_node` are redirected under `/dtp/shadow`, where nothing subscribes, and
the ABORT is disabled (in Stage A this node has no authority over an aircraft flying someone else's
mission, and landing it because our camera died would be wrong).

Redirecting the **yaw target** as well as the goal is the part that is easy to forget: with it live,
SUPER turns the aircraft to face the person under `YAW_TO_TARGET`, which is the follow stack
commanding the drone just as surely as a goal would.

It answers the one question a bench cannot: with the aircraft **translating and above all yawing**,
are the standoff goals geometrically correct, and does their quality **degrade with ego-motion**?
`perception_node` lifts through the transform at the frame stamp; the cheap alternative (newest
transform) is wrong by yaw-rate × range — 0.5 m at 3 rad/s and 6 m. **Stationary the two are
identical, so no bench run can tell them apart.** If the stamped lift is right, target jitter and
standoff error stay flat against yaw rate.

Two tools support it, both new:

- `scripts/stage_a_preflight.py` — pre-arm GO/NO-GO that asks the **live ROS graph** who publishes
  to the topics SUPER and PX4 listen on, rather than trusting a launch argument that can be
  forgotten, mistyped, or overridden by a YAML someone edited last week. Refuses on the first thing
  it does not like.
- `scripts/analyze_shadow_goals.py` — post-flight, produces the yaw-rate table above.

## Calibration, and the depth question that is now testable

`dtp_perception/calib/drone_cam_livox.yaml` — **since 2026-09-13 the 09-11 solve**:
- **Source:** capture `~/calib_runs/20260911_2252_bench` (8 pairs), T solved 09-11 23:38.
- **Score:** `score_alignment PASS du+1px dv−1px` on that capture, **motors off on the bench**.
- **What it replaced:** the 09-07 solve (`du+0 dv+1` on its own capture), which scored **du −12 px**
  on 09-09, 09-11 and 09-13 data (`m5.4-m5.5-field-runbook.md` §4).
- **Its translation is physically wrong.** The two differ by 1.44° and 72.8 mm. The LiDAR is
  **above** the camera and a little behind (user, 2026-09-13); the 09-07 solve agrees, while this
  one puts the LiDAR 2.3 cm below.
  - **Kept anyway:** the images only fit its rotation and translation as a pair.
  - **Cost:** estimated ≲ ±3 cm in lifted positions over 2–6 m.
  - **Fix:** a re-capture with near-range structure (`m5.4-m5.5-field-runbook.md` §4).

It is bound to camera serial `CPC64630007M` and LiDAR `192.168.1.154`, and `calib_io` refuses a
mismatched pair. **Any remount of either sensor invalidates it**, and it has still not been
re-validated with motors running.

`depth_source` chooses one sensor, defaulting to **`camera`**. Indoor measurements (2026-09-08, not
re-verified here): Gemini stereo **88.4 % valid pixels at 30 Hz**, registered by factory extrinsics
that survive a remount, versus LiDAR-through-our-extrinsic at **13.5 % coverage in the camera FOV
and nothing in the lower third** (the MID360's vertical floor is about −7°). The caveat their own
docstring raises: the 336L is *active* stereo, and direct sunlight overwhelms its IR projector, so
all of those numbers are indoor numbers.

**`scripts/outdoor_depth_test.py` now EXISTS** (437 lines, written 2026-09-11). This document
previously recorded it as missing; that is no longer true. It has not been run. It measures depth
**on the person**, not frame-wide, because a sunlit scene of tarmac and foliage can score well while
the subject returns nothing. If depth survives outdoors the follower is camera-only and camera
placement becomes a purely mechanical question; if it collapses, the LiDAR path is mandatory and
every remount costs a recalibration.

A related trap, and the tool for it: on 2026-09-08 the **left IR camera was obscured** and camera
depth was 99.7 % empty while RGB looked perfect and the driver reported no error. Re-taping "clear
of the IR windows" did not fix it, because knowing `left_ir` is blocked does not tell you which
physical opening `left_ir` is. `scripts/ir_window_check.py` is a live brightness meter for both IR
cameras — run it and pass a finger over each opening. `std` is the better indicator than `mean`: a
working IR camera under the dot projector shows ~35, a lens behind tape shows ~13 however bright
the room is, because a diffuser destroys structure while still passing light.

## The analysis tooling added 2026-09-10/11

Nine scripts appeared or were rewritten in the two days before this revision. They are the evidence
base for the paper, and several replace numbers this project had been quoting:

| script | what it settles |
|---|---|
| `measure_coresidency.py` | **Replaces** the 27.6 % CPU / 6.5 GB / 66 % GPU / 18.5 W / 71 °C figure quoted since 09-07, which measured M0.1 scaffolds that did no work. The real `perception_node` is ~250 % CPU. Measures the **marginal** cost of adding the follow stack, not the total. |
| `odom_lag_probe.py` | Separates three causes of frames ageing out blind: a steady stamp-base offset, odom stalls, or `/tf` lagging the odom topic. Found ~24 % blind against the full nav stack vs 0 % against a bare FAST-LIO — which is what produced the `detect_rate_hz` change. |
| `compare_lift_strategies.py` | Stamp-exact vs latest-available transform lift, recomputed **paired** from one bag so no calibration enters the comparison (`T_bc` cancels). |
| `depth_aggregation_ablation.py` | Bake-off of range statistics over identical torso patches (median, nearest-surface, NOVA's histogram mode, …). Needs a bag carrying raw depth; `20260910_flightlegal_bench` is the first. |
| `analyze_shadow_goals.py` | Stage A: were the goals we would have flown correct, and do they degrade with yaw rate? |
| `analyze_follow_bag.py` | Offline follow-chain post-mortem. |
| `stage_a_preflight.py` / `stage_a_record.sh` | Stage A pre-arm gate and recording. |
| `outdoor_depth_test.py` | The camera-vs-LiDAR depth decision, on the person, in sunlight. |
| `ir_window_check.py` | Which physical IR window is blocked. |
| `bench_cloud_backend.py` *(2026-09-13)* | Is the GPU organized cloud the same cloud as numpy, frame by frame, and what does each cost (wall + process CPU)? `--yolo` adds decode / predict / BoT-SORT / ByteTrack. Exits 1 on any frame outside tolerance. |
| `follow_speed_report.py` *(2026-09-17)* | How fast did a follow flight fly and what held it back: commanded vs EKF2 vs FAST-LIO speed per Offboard/FOLLOW window, walker speed and range, goal distance, goal-altitude jumps (old descent guard) vs latched floor (new), altitude tracking, every lock taken, and with `--stack-log` SUPER's failed replans per second. |
| `replay_perception.sh` + `record_targets.py` + `compare_replay_targets.py` *(2026-09-13)* | A bag through the **real** `perception_node` on a private localhost-only domain, per `cloud_backend`, with CPU accounting; then pairwise target agreement. **Replay is not deterministic:** at 1× the frame schedule differs between runs (`WARM_S=60 RATE=0.25` fixes that), and even on an identical schedule two runs of the same code differ p95 ~1.7 cm (likely colour→newest-depth pairing). Compare each arm against a second run of itself. |

## Live state, 2026-09-12

Measured at the time of writing. **The 09-10 dead-link condition is gone:**

| Observation | |
|---|---|
| `eno1` | **UP**, `192.168.1.50/24` |
| `ping 192.168.1.154` | **OK** |
| Livox driver | **one** instance (was two on 09-10) |
| `fastlio_mapping` | up 1 h 25 m, 27.8 % CPU, 324 MB RSS |
| `fsm_node` | up 1 h 25 m, 112 % CPU, **1.93 GB RSS** — the bounded A\* preallocation, not a leak |
| DTP nodes | **none running** (no `perception_node`, `target_manager`, `follow_planner`, `command_mux`, `static_calib_tf`) |
| `fsm_node` binary | built 2026-09-09 03:13, from a `fsm_ros2.hpp` of 03:10 — so the running planner **does** echo `requested_p` |

For the record, the state this document described on **2026-09-10** — kept because the failure shape
is instructive: the follower had been up 15 h 57 m and **blind the entire time**. `eno1` was
`DOWN`/`NO-CARRIER` with no IPv4, `/livox/lidar` and `/lidar_slam/odom` had publishers but produced
no messages, and **two** Livox drivers were running (the older holding `192.168.1.50:56xxx`, the
newer bound to the Wi-Fi address instead). Every downstream state was the *correct* response:
`/dtp/sensor_health` said `lidar_slam_odom: NEVER PUBLISHED` and `perception is ALIVE BUT BLIND`,
`/dtp/targets` ran at 15.0 Hz always empty, and `/mux/state` sat in `HOLD|SENSORS+ODOM`. The safety
chain behaved as designed; the faults were the dead link and the duplicate driver.
**Check `ip -brief link show eno1` and `ping 192.168.1.154` before believing any perception result**,
and check for a duplicate driver before starting one.

## What running it looks like

```bash
ros2 launch dtp_bringup dtp_system.launch.py                 # sensors + SUPER + follower + watchdog
ros2 launch dtp_bringup dtp_system.launch.py follower:=false # NAV / click-goal only
ros2 launch dtp_bringup dtp_system.launch.py sensors:=false  # sensors already running
ros2 launch dtp_bringup follower.launch.py observe_only:=true # Stage A (see above)
# on top of scripts/run.sh (LiDAR + FAST-LIO + fsm_node already up), observe-only:
ros2 launch dtp_bringup dtp_system.launch.py lidar:=false super:=false observe_only:=true
```

To watch it: `JETSON_IP=gypsy.local bash scripts/laptop_dtp_viewer.sh` on the laptop (see
`dtp_image_overlay` above).

Source `~/super_ws/install/setup.bash` first. If you use `dtp_env_jetson.sh`, source it as
`~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh` — through the repo path it finds no workspace
(`drone-dtp/README.md`, Running).

Layers (`sensors` / `super` / `follower` / `sensor_watchdog`) each run standalone. There is
deliberately **no start ordering** — `fsm_node` refuses to step while odom is stale, ROG-Map has a
2 s odom timeout, and the watchdog has a 20 s grace, so sequencing would hide a real ordering
dependency rather than fix one.

`super.launch.py` deliberately does **not** include our `m1_super_lidar.launch.py`: that file
hardcodes `click_smooth_ros2.yaml`, which has no follow path, so including it would bring up a
stack that looks completely healthy while the follow chain is open-loop.
`preflight.assert_follow_path_enabled` refuses that at launch time (unless
`require_follow_path:=false`, for a deliberate NAV-only bring-up), and `preflight.assert_wall_clock`
refuses `use_sim_time:=true` outright — a stack half on sim time fails SUPER's yaw gate on every
message, forever, silently.

## Open items

1. **✅ FIXED 2026-09-13 (G0.4) — the SUPER patch was STALE against the live source; it has been
   regenerated and now carries `requested_p`.** (Original finding, kept for context:) live
   `fsm_ros2.hpp:316-318` fills `out.requested_p`; the *old* patch's `followGoalCallback` set only
   `header`, `status` and `goal_p` (`grep -c requested_p` returned **0**), which would have broken
   the GoalStatus contract on a fresh install. The patch was regenerated from the live tree against
   the `2ad3419`+isnan base (`grep -c requested_p` now **3**), `apply_super_patch.sh` was rewritten
   to use the single combined patch, and `setup.sh` now applies it — so a re-clone reproduces the
   working follower. The stale-install failure mode described below no longer occurs:
   `requested_p` then stays `(0,0,0)`, `follow_planner`'s correlation
   `(requested - pending_goal_).norm() > kAcceptedMatchTolM` is true for every message, and it
   `return`s — **discarding every GoalStatus it receives**. `status_link_ok_` never goes true,
   `ExpirePendingStatus` warns "no GoalStatus within 0.20 s" forever, and the SNAPPED/DROPPED
   escalation never runs. Silent, and exactly the class of failure `GoalStatus` was added to
   remove. **Fix: regenerate the patch from the live `~/super_ws/src/SUPER` tree.**
2. **~~Put the tree under version control.~~ Done 2026-09-12:** committed as `drone-dtp/` on
   branch `dtp-follower-stack` (`28d366b`). `~/super_ws/src/drone-dtp` is a symlink to it and the
   packages are built `--symlink-install`, so there is one tree, and it is the one in git.
   **Keep that branch checked out on the aircraft** — any other branch removes the folder.
   **Pushed 2026-09-13** (`origin/dtp-follower-stack`). Wiring and traps: `drone-dtp/README.md`.
3. **The trail problem now has a filter (M5.3a, 2026-09-13), default OFF.**
   - **What it does.** `rog_map/target_exclusion` in `follow_test_ros2.yaml` (`patches/0002-…`)
     keeps the LOCKED person's own returns out of ROG-Map. Inside a cylinder at their
     velocity-extrapolated position, a scan inserts no hits and no misses. Only seen frames count:
     coasted and held frames are excluded, and an observation older than 0.5 s is dropped.
   - **The consequence.** The planner no longer avoids the followed person; the standoff and the
     proximity veto do.
   - **`stage_a_preflight.py` FAILs if it is on in Stage A.**
   - **Evidence:**
     - `drone-dtp/scripts/target_filter_check.py`, 17/17 on the real `fsm_node`;
     - `target_filter_replay.py` on `run5`, line-of-sight crossings 2.6 % → 0.7 %;
     - `target_filter_study.py`, which chose the centre, radius and band.

   **Raycasting is also on in the config this stack loads** (2026-09-12, user's instruction,
   superseding the 2026-09-10 revert). `follow_test_ros2.yaml` line 175 is `enable: true` in `src/`
   and `install/`, and `patches/0001-…` line 201 matches, so a re-clone reproduces it. Its
   `p_max` is still 0.98, so a cell the target stood in needs ~54 clean rays through it before it
   stops being occupied (~31 at the click config's 0.95). **Not yet measured with this stack:**
   the CPU cost alongside perception, and whether a walking person's voxels actually clear.
4. **~~`auto_reacquire_after_lost: true` is live.~~ False since 2026-09-12**, and `tf_at_latest`
   is false. Keep both that way for flight; `abort_require_offboard` must stay `true`. All three log
   at ERROR when unsafe, which is the mitigation, not a fix.
5. **✅ FIXED 2026-09-13 (G0.4) — `scripts/setup.sh` now applies the SUPER follow-goal patch**
   (and symlinks `drone-dtp/` into the workspace), when the folder is present. Before that a clean
   install yielded a SUPER with no `follow_goal_en`, no `/planning/goal_status` and no
   `YAW_TO_TARGET`. **Still unproven:** a full clean colcon build from a fresh clone.
6. **`plan/dtp-imp/*` and the DTP `CLAUDE.md` are still missing** — recover them from wherever they
   were written, or the comments are the only rationale left.
7. **Outdoor depth is still unmeasured**, though the test now exists. **Motor-on calibration
   validation** has still not been done; vibration shifts the extrinsic.
   - **⛔ 2026-09-13: the deployed calibration is already off, motors OFF.** `drone_cam_livox.yaml`
     (09-07) scores **du −12 px (≈ 1.1° of bearing)** on the 09-11 capture, on 09-09 data and on a
     bag from today.
   - **✅ Replaced 2026-09-13 16:52 by the 09-11 22:52 solve** (today du +2 px; its own capture
     +1/−1). Old file in `~/backups/calib-pre-0911-deploy-20260913/`. Its vertical offset is still
     unconfirmed by a tape measure.
   - **Motor-on validation now comes from Stage A flight bags** (`scripts/bag_to_calib_pairs.py`).
   - Full table: [`m5.4-m5.5-field-runbook.md`](m5.4-m5.5-field-runbook.md) §4.
8. **The DTP goal path and our M3 offboard bridge have still never been run together.**
9. **The single-threaded executor in `perception_node`** is the known cause of the residual 2.2 %
   blind rate. The fix is a `MultiThreadedExecutor` with tf2 and odom on their own callback group —
   a threading change to the node that owns every world position in the stack, needing its own
   tests.
10. **`command_mux` is now a third writer to the flight controller** alongside `px4_odom_bridge` and
    `px4_offboard_bridge`. Nothing arbitrates between them, and our bridges' "never command a mode"
    rule no longer holds for the stack as a whole.
    - **Analysed 2026-09-13 against PX4's source (`d6f12ad`, our firmware). The test that settles it
      is written but not yet run.**
    - **Predicted from the source:**
      - NAV_LAND is a forced mode change (`Commander.cpp:1093`), so the bridges' setpoint stream
        cannot hold the aircraft in Offboard against it.
      - The RC Offboard switch is edge-triggered (`ManualControl.cpp:221`). A ch11 left ON does
        not drag the aircraft back after an ABORT, but OFF→ON does, and the terminal mux is still
        publishing its HOLD goal.
      - With `COM_RC_OVERRIDE = 1` the sticks take over during the ABORT's AUTO_LAND, though not in
        Offboard.
    - **The test:** [`m5.1-one-graph-runbook.md`](m5.1-one-graph-runbook.md), scored by
      `scripts/px4/m51_one_graph_check.py` (sections A and T).
11. **The comments say `camera_init`; the params say `lidar_init`.** Translate when reading.
12. **⛔ NEW 2026-09-12 — `follow_test_ros2.yaml`, the SUPER config this stack flies on, carries
    upstream's benchmark `max_vel: 5.0` / `max_acc: 5.0`** in `src/` and `install/`. `setup.sh`'s
    first-flight limits (0.1 m/s) only touch `click_smooth_ros2.yaml`. Staged follow envelopes
    (0.5 → 1.0 → 1.5 m/s) are M5.2 in [`m5-m6-plan.md`](m5-m6-plan.md). They must go into the patch
    so a re-clone cannot restore 5 m/s.
13. **✅ ROOT-CAUSED AND GUARDED 2026-09-13 (G0.2) — the flight stack's FAST-LIO diverged while
    the aircraft was armed, and THIS stack's launch caused it.**
    - **What happened.** In `~/dtp_datasets/20260912_2006_stageA` FAST-LIO was **3.74 km** wrong
      (x −3340, y −1672, z −86.7 m; only z had been read at first), and EKF2 accepted it.
    - **Cause.** `dtp_system.launch.py` started on top of `run.sh` brought a **second Livox
      driver**, which took the MID360's stream as PointCloud2. FAST-LIO received IMU and no scans
      for 225 s. `extrinsic_est_en` was *not* the trigger.
    - **Now guarded:**
      - `sensors.launch.py` / `super.launch.py` / `run.sh` refuse a second driver, FAST-LIO or
        `fsm_node`;
      - `dtp_system.launch.py` defaults `xfer_format` to 1;
      - `px4_odom_bridge` latches DIVERGED;
      - `scripts/px4/estimator_preflight.py` must say GO before arming.
    - **Write-up:** `fact-fastlio-divergence-2026-09-12.md`. None of that bag's four recordings is
      usable for geometry.
14. **The paper's contribution — loss-cause classification, reappearance belief, planner
    conditioning — is not implemented anywhere in this tree.** `target_manager`'s CTRV coast is the
    paper's carry-forward ablation arm. Design, interface (`dtp_msgs/LossEvent` from inside
    `fsm_node`) and schedule: M6 in [`m5-m6-plan.md`](m5-m6-plan.md).
