# Eva-Tracker and Faster-LIO — can they improve this stack, and the plan if so

**Written 2026-09-14.** The user asked whether [Eva-Tracker](https://github.com/Yue-0/Eva-Tracker)
(Lin et al., ICRA 2026, arXiv 2602.12549) and [Faster-LIO](https://github.com/gaoxiang12/faster-lio)
(Bai et al., RA-L 2022) can improve what we are doing, and for an updated plan from where we stand.
Sources: both READMEs, both GitHub trees, the Eva-Tracker paper (arXiv HTML), a GitHub search for
ROS 2 ports, and this repo's `docs/m5-m6-plan.md`, `docs/paper/main.tex`, `config/mid360.yaml`,
`dtp_params.yaml`. **Nothing here was run; no code from either repo has been built on this box.**

## TL;DR

| | Use it? | How | Cost | When |
|---|---|---|---|---|
| **Eva-Tracker as the flight planner** | **No** | — | weeks, unrelated to the thesis (ROS 1 Noetic, MAVROS, MonoLoco monocular target, AGPL-3.0) | never |
| **Eva-Tracker's loss policy as a baseline arm** | **Yes** | third `baseline_mode` in `target_manager` + `follow_planner`: keep planning viewpoints from the *predicted* target with an arc search for a free line of sight; no classification | 2–3 days, bench + replay, no flight dependency | **M6.8, Nov 11–13** (already scheduled; +1 day) |
| **Eva-Tracker's Bézier-QP predictor** | **Yes** | third arm in the M6.6 offline predictor study, and the predictor behind the Eva arm above | 1 day, off-Jetson | **M6.6, Oct 26–Nov 4** (inside the existing 8 days) |
| **Eva-Tracker's own simulator** | **Optional** | run the real thing in a `ros:noetic` container on the laptop, its `benchmark.bag`, to quote *their* numbers on *their* course | 2 days if their build works first time; unbounded if not | after D2, only if a co-author wants Table IV's row 3 to say "real Eva-Tracker" |
| **Eva-Tracker's FoV-ESDF (visibility term in the optimizer)** | **Not now** | SUPER's back end is corridor-constrained MINCO with no visibility cost; adding one is a planner-research task | 2+ weeks | post-paper, if ever |
| **Faster-LIO replacing FAST-LIO** | **Not now — contingency** | swap only if the moving-rig latency check (the current flight blocker) shows FAST-LIO dropping scans or lagging | 1–2 weeks incl. every gate re-run | trigger = a FAIL on `docs/fact-fastlio-latency.md`'s moving-rig check |
| **`extrinsic_est_en: false` in FAST-LIO** | **Yes, try first** | the cheap CPU lever Faster-LIO would otherwise be bought for: 74 % → 31 % measured on this rig | 1 day incl. the handheld M2 gate | **M5.3b, Oct 5–7** |

**Two findings that matter for the paper regardless of the plan:**
1. **The paper's hinge sentence is defensible against Eva-Tracker.** Their loss handling is
   exactly "predict forward and continue": a Bézier fit through the last *m+1* observations at
   0.5 s spacing, then viewpoints at 2.5 m from the *predicted* target with a bidirectional arc
   search for an unoccluded point. There is no map-based reason for the loss, no classification, no
   belief. The authors say so themselves: recovery is "effective only for short-term losses; if the
   target's actual trajectory deviates significantly from the predicted trajectory during prolonged
   occlusion, successful recovery becomes difficult." Quote that in Sec. II-D.
2. **Eva-Tracker removes the target's own points before mapping** ("points within the estimated
   target radius are removed prior to mapping"). That is our patch 0002 (`target_exclusion`).
   Cite it as precedent when the reviewer asks why the followed person is not an obstacle.

## 1. Eva-Tracker

### 1.1 What it is (from the paper and the tree)

- **Platform: the same class as ours** — Jetson Orin NX, Livox Mid-360, Faster-LIO odometry, PX4.
  But **ROS 1 Noetic / Ubuntu 20.04, MAVROS, catkin**, with an RC channel-8 state machine
  (idle / takeoff 1.2 m / follow / hover / land) inside `tracker/scripts/control.py`.
- **Target sensing: monocular.** YOLO11n-Pose keypoints → MonoLoco (a monocular 3-D human
  localiser, `models/MonoLoco.onnx`) via TensorRT (`onnx2trt.py`). The README lists a RealSense,
  but the paper's 3-D target position comes from MonoLoco, not depth. Ours is RGB-D on the
  Gemini 336L, lifted into `lidar_init`.
- **Prediction:** Bézier curve through the last *m+1* observations at τ = 0.5 s, solved as an
  equality-constrained convex QP minimising ∫‖acceleration‖². Extrapolated forward for t > 0.
  (`tracker/include/predictor/{bezier,predictor}.hpp`.)
- **Path generation ("recovery-capable"):** candidate viewpoint at the optimal observation distance
  (2.5 m) on the line to the predicted target; if the line of sight is occluded, a bidirectional
  search along a circular arc around the (predicted) target for the nearest free-LoS point.
  0.02 ± 0.01 ms.
- **FoV-ESDF:** a *precomputed* distance field in the camera frame — distance to the nearest
  boundary of the FoV pyramid, zero outside — so the optimizer's visibility gradient never needs a
  map update. Obstacles use a robo-centric ESDF. Trajectory optimisation is MINCO
  (`planner/minco.hpp`, `optim.hpp`), 10.73 ± 3.89 ms.
- **Results (simulation, 200 s random-obstacle course):** occlusion rate 4.36 % vs Elastic Tracker
  8.71 %, failure rate 6.75 % vs 12.27 %; tracking distance 2.50 ± 0.24 m. Max 2 m/s, 1.5 rad/s in
  the real-world runs.
- **Stated limitations:** short-term loss only (quoted above); fixed-FoV assumption.
- **Licence: AGPL-3.0.** Anything of theirs we link or copy makes the combined work AGPL. Our
  planner patches sit inside SUPER (GPL-2.0 per its repo) — ask before copying code rather than
  re-implementing from the paper.

### 1.2 What it would and would not improve

**Not the flight planner.** `docs/m5-m6-plan.md` §1 row 5 already reached this conclusion on the
assumption that porting is weeks; the sources confirm it. The port would mean: ROS 1 → ROS 2 of a
catkin tree with `quadrotor_msgs`, MAVROS → our uXRCE-DDS bridges, MonoLoco → our RGB-D
target, their RC state machine → our `command_mux`/`offboard_bridge`, and Faster-LIO → our FAST-LIO
topics. None of it is the thesis, and it would be a *second* planner next to SUPER that nothing
in M5 has flown. Cut stays cut.

**Yes as the third baseline arm — and it is a stronger, more honest baseline than we planned.**
Today's M6.8 arms are: `stock_baseline` (goal behind the present position, no coast, no ABSTAIN),
carry-forward (`loss_conditioning:=false`, CTRV coast, goal holds) and ours. A reviewer who reads
Eva-Tracker will note that its loss policy is *not* "hold the last goal": it keeps generating a
2.5 m standoff viewpoint from the **predicted** target and steps around occluders on an arc. Our
oracle already has the annulus viewpoint search (`loss_oracle.hpp`, [3, 4] m annulus, free-LoS
scoring); the Eva arm reuses it with the *predicted* point as the centre and no verdict. That makes
Table IV row 3 "Eva-Tracker's loss policy, re-implemented in our stack" — label it exactly that,
never as the real system.

**Yes as a predictor arm.** M6.6 compares CTRV, EigenTrajectory and fused. Adding the Bézier-QP
predictor is a page of numpy (a small equality-constrained least-squares problem), runs offline,
and closes the obvious question "is CTRV a straw man?". It also gives the Eva arm above its own
predictor, so that arm is faithful in both halves.

**FoV-ESDF: a real gap, not for this paper.** SUPER's trajectory has *no* visibility objective —
between goals the drone can fly behind a trunk on its own. Our conditioning changes the goal and the
yaw target, not the trajectory cost. A FoV-ESDF term inside SUPER's MINCO back end would fix that,
and is a legitimate follow-up paper, but it is planner research on a planner none of M5 has flown.
Write it into Sec. VIII (limitations / future work), citing Eva-Tracker for the construction.

**Their simulator: optional and risky.** `src/simulator/` is a ROS 1 Gazebo-free simulator with a
car target and a `benchmark.bag`. Running the *real* Eva-Tracker there gives their Table I numbers
on their course, which we can quote for row 3 with a footnote — but it does not put it on *our*
tube course or forest, so it never becomes an apples-to-apples row. Only worth two days if a
co-author insists on "run the real thing"; the re-implemented arm is the default.

## 2. Faster-LIO

### 2.1 What it is

- FAST-LIO2's ESIKF with the **ikd-Tree replaced by iVox** (sparse incremental voxels, linear or
  PHC-ordered, LRU-bounded) and TBB parallelism. Claimed 1.5–2× faster than FAST-LIO2; the
  compute win is in the map (nearest-neighbour and insertion), not the filter.
- **ROS 1 only upstream** (melodic/noetic), last substantive activity **February 2022**,
  GPL-2.0. `config/mid360.yaml` exists, and Eva-Tracker ran it on an Orin NX with a Mid-360,
  so the sensor pairing is proven.
- **ROS 2 ports:** one, [RollingOat/faster-lio-ros2](https://github.com/RollingOat/faster-lio-ros2)
  — Galactic, 5 commits, 19 stars, "odometry drifts in the very beginning" in its own README, no
  Humble claim. Nothing maintained. A swap would mean writing our own ROS 2 wrapper around
  `laser_mapping.cc` (which is fairly ROS-light) and re-doing everything `patches/fastlio_*.py`
  did to FAST-LIO.

### 2.2 Would it improve what we are doing?

**Not on the evidence we have.** FAST-LIO is not the bottleneck today:

| FAST-LIO on this rig | Value | Source |
|---|---|---|
| Rate / latency | 10 Hz, 49–57 ms, 0/10 lagged | `docs/fact-fastlio-latency.md` |
| CPU | 39.6 % (M1 bench) … **74 % with `extrinsic_est_en: true`, 31 % with it off** (field stack) | CLAUDE.md M1; `dtp_params.yaml:113` |
| RSS | flat, ~155 MB | every soak since 2026-08-26 |
| Cores | pinned to 6,7 by default; the whole graph sums to ≈ 285–328 % of 800 % | `run.sh`, plan §2.5 |
| 20-min soak | 6/6 PASS, max odom gap 144 ms (with a Wi-Fi subscriber) | `docs/fact-fastdds-blocking-send.md` |

Every FAST-LIO problem this repo has hit was in the **ROS 2 port's plumbing**, not the algorithm:
the 1.4 s executor latency, `/Laser_map` leak, blocking DDS sends, a second Livox driver. Faster-LIO
would inherit none of our fixes and need its own versions of all of them, plus a re-run of every
gate that sits on the odometry: M1 (rate/RSS/latency), **M2 handheld (axis signs)**, G0.2 bridge
guard thresholds, `target_filter_check.py`, `loss_oracle_check.py`, `bag_to_calib_pairs.py` (its
FAST-LIO path), the 20-min soak, and the estimator preflight. One to two weeks, none of it toward
the paper.

**Two things it *would* buy, which is why it stays as a contingency:**
1. **A cheaper map under motion.** The one FAST-LIO test never run is the moving rig, and the
   latency fix dropped the cloud queue to depth 2 so stale scans are *dropped* under load. If that
   check fails (latency > 100 ms, or dropped scans in a moving-rig bag), the fix is either fewer
   points (`point_filter_num` up — bad for ROG-Map density) or a faster map. Faster-LIO is the
   faster map.
2. **Bounded memory by construction.** iVox keeps an LRU of voxels (`ivox_capacity`); ikd-Tree
   only trims at the `cube_side_length` boundary. Our RSS is flat now because of `map_en: false`,
   not because the tree is bounded on a stationary rig.

**The cheap lever first: `extrinsic_est_en: false`.** The extrinsic FAST-LIO estimates online is
LiDAR optical origin → the MID360's *internal* IMU — a factory constant on a rigid sensor. The
measured cost of estimating it is 74 % → 31 % of a core (`dtp_params.yaml:113`), more than
Faster-LIO's whole claimed speed-up. CLAUDE.md records that it is *not* the G0.2 trigger and that
every gate passed with it on; nothing says it must stay on. Turning it off is a config change in
`config/mid360.yaml`, `src/` and `install/`, then the handheld `m2_gate_check.py` (the odometry
frame moves slightly, so the lever arm and signs must be re-confirmed) and a 20-min soak. The
plan's §6 already lists it as the divergence fallback; promote it to a scheduled M5.3b item.

## 3. The updated plan — deltas to `docs/m5-m6-plan.md`

Where we stand (2026-09-14): Gate 0 open on G0.1's 14 h soak and ≥ 500-cycle count; M5 bench-only;
M6.1/M6.4/M6.5/M6.8-stock built and bench-gated; run5 replayed and provisionally labelled;
M6.3 dataset not recorded; nothing has flown. **No row below moves a flight day.** Each is a
desk/replay row that fits inside an existing slot or adds a day to one.

| Row | Item | Who | Days | Dates | After | Exit |
|---|---|---|---|---|---|---|
| **14a** | **`extrinsic_est_en: false`** on the bench: CPU before/after, handheld `m2_gate_check.py` (signs, correlation, lag), 20-min `m53_soak.py`. Keep it if M2 passes and soak 6/6; else revert and record why | B | 1 | Oct 5 (inside M5.3b) | 11 | CPU figure in §2.5; M2 report in `~/dtp_datasets/` |
| **14b** | **Moving-rig latency check** (already the flight blocker; `docs/fact-fastlio-latency.md`) — now also the **Faster-LIO trigger**: FAIL = p95 latency > 100 ms or any dropped scan in a 3-min handheld walk at 1.5 m/s | A | 0.5 | first field day (row 8) | — | PASS closes the Faster-LIO question for the paper; FAIL opens row 14c |
| **14c** | *(only on 14b FAIL)* **Faster-LIO ROS 2 wrapper**: our own `rclcpp` node around `laser_mapping.cc` (Humble, C++17, TBB), same topics/frames as today (`/lidar_slam/odom`, `/cloud_registered`, `/cloud_registered_body`, `lidar_init`/`lidar_imu`), callback groups + depth-2 queue + `nonblocking` DDS profile from day one, `mid360.yaml` extrinsics copied; then M1, M2 handheld, G0.2 guard, `target_filter_check`, `loss_oracle_check`, `bag_to_calib_pairs` control, soak | B | 7–10 | slips rows 23–27 day-for-day | 14b | every gate green on the new odometry; `setup.sh` fetches it at a pinned commit |
| **24a** | **Bézier-QP predictor** (Eva-Tracker's: *m+1* points at τ = 0.5 s, min ∫‖a‖², equality-constrained) as a fourth arm in the M6.6 study; also usable by 27a | A | 1 | Oct 26 (inside M6.6's 8 days) | checkpoint | Table II gains a row; `m`, `τ` swept once |
| **27a** | **Eva arm** — `baseline_mode: eva` in `target_manager` (coast on the Bézier prediction, no ABSTAIN change, no verdict) and `follow_planner` (goal = 2.5 m standoff viewpoint from the *predicted* point, arc search on the oracle's free-LoS scoring, every frame); replaces the `stock_baseline` bool with a three-value enum, keeping `stock` and `off` byte-identical in behaviour (the 16/16 + 27/27 gates must still pass unchanged) | B | 2 | Nov 11–13 (M6.8 + 1 day) | 27 | `loss_conditioning_check.py` gains ≥ 4 Eva cases; `loss_closed_loop_replay.py --arm eva` on run5 and the M6.3 bags; the ON-vs-Eva re-acquisition delta reported alongside ON-vs-OFF |
| **27b** | *(optional)* real Eva-Tracker in its simulator on the laptop (`ros:noetic` Docker, `benchmark.bag`); quote their Table I with a footnote | A | 2 | after D2, only on request | — | numbers or a written "did not build" — either is a paper sentence |
| **28a** | **Paper edits** (author's call, drafted not applied): Sec. II-D quotes Eva-Tracker's own short-term-only limitation; Table I row: prediction = "Bézier QP (min-accel)", occlusion handling = "FoV-ESDF + arc search", loss-cause = no, belief = no; Table IV row 3 relabelled "Eva-Tracker loss policy (re-impl.)"; Sec. VIII: no visibility term in the trajectory cost, FoV-ESDF as the fix; cite Eva-Tracker as precedent for target-point exclusion (patch 0002) | A | 0.5 | with row 28 | 27a | `\note{}` cleared in II-D |

**Flight evaluation (M6.7) gains one arm** if 27a lands: the ABBA schedule becomes four arms, and
§2.3's ≥ 30 usable occlusion events per arm applies to it too. If field days are short, drop the
Eva arm from *flight* and report it on the course replays only — it is a baseline, not the claim.

**Risk rows to add to §6:**

| Risk | Signal | Fallback |
|---|---|---|
| `extrinsic_est_en: false` shifts the odometry frame enough to fail M2 | correlation < 0.99 or any sign flip in 14a | revert; FAST-LIO stays at 74 % and §2.5's worst case stands |
| 14b fails and the Faster-LIO wrapper eats a fortnight | 14c past day 10 | `point_filter_num: 4` and `filter_size_surf: 0.3` as the stop-gap; report the drop rate in the paper's compute section |
| The Eva arm outperforms ours on the course | 27a replay | that is a result, not a failure: it says the belief adds nothing over a good predictor at that occluder scale — report it and lean on the LOST-class (energy/abstain) advantage, which no predictor-only policy has |

## 4. What this does not change

- **Gate 0 is unchanged and still blocks every flight.** Neither repo touches the FC hardfault.
- **The M6.3 handheld dataset is still the critical path to "is there a paper".** Everything in §3
  is desk work that fits around it.
- **The thesis is unchanged.** Eva-Tracker confirms, in its authors' own words, that the strongest
  prior re-acquisition system is prediction-only. That is the paper's hinge, and it is now sourced.
