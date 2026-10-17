# M5 / M6 plan — fly the DTP follower, then build what the paper claims

**Written 2026-09-12; numbers added the same day.** Sources: `docs/paper/main.tex`,
`docs/dtp-follower-stack.md`, `docs/flight-test-plan.md`, the `drone-dtp/` source, the SUPER and
ROG-Map source, the FC fault logs, and today's four Stage A bags. **Measured** means the number was
read off a file or a bag while this was written. **Target** means a pass threshold proposed here.
**Estimate** means a planning assumption: check it on the first day it can be checked.

**Milestone names.** The repo only defines milestones up to M5, and M4 is still open. This plan
re-cuts them:

| | Means | Absorbs |
|---|---|---|
| **Gate 0** | The aircraft is safe to fly autonomously *at all* | open blockers from `flight-test-plan.md`, plus two found today |
| **M5** | **The existing DTP follower flies, end to end, near a person** | the open M4 checklist and the old M5 tuning/safety list |
| **M6** | **The paper's contribution exists, and is evaluated** | `docs/paper/README.md`'s six blockers |

## TL;DR

1. **The paper's contribution does not exist in code.** Nothing labels a loss as occluded or lost,
   predicts where the person reappears, or conditions the planner on it —
   `grep -riE "occlu|raycast|reappear|rog_map"` over `drone-dtp/` finds only comments and the CTRV
   coast. What exists is exactly the paper's **carry-forward comparison arm**, so every M5 flight
   is also baseline data, and M6 is new code.
2. **It is buildable with what we have.** ROG-Map exposes `isLineFree(…, use_unk_as_occ)`,
   `getGridType()` and a voxel raycaster, and `fsm_ros2` holds `map_ptr_`. A small "loss oracle"
   inside `fsm_node` makes "one map serves both the planner and the classifier" literally true.
3. **Most of M6 can be de-risked without flying.** Replay bags into `fsm_node`, and record a
   **handheld** dataset of 140 scripted events on a course of cardboard tubes standing in for
   trunks. That answers "is there a paper" by **2026-10-27**, before any evaluation flight.
4. **Nothing flies until Gate 0 closes** (§3). Status as of 2026-09-13:
   - **`uxrce_dds_client` hardfault.**
     - **The archive:** 35 dumps — 22 classic field crashes, 2 other field signatures, and 11
       induced on the bench.
     - **Reproduced on demand 2026-09-13:** a real session loss resets the FC 5/5, with the exact
       field signature.
     - **`CONFIG_UART7_RXDMA=n` was flashed and FAILED.**
     - **The client-side no-close-on-reconnect fix is flashed** and passed **30/30** reconnects.
       The flood test was inconclusive (the FC stopped reporting after cycle 1).
     - **The 14 h soak and the ≥ 500-cycle gate are still open (§3 G0.1).**
   - In `20260912_2006_stageA` the aircraft was **armed ~2 min while FAST-LIO was 3.74 km wrong
     and EKF2 accepted it as valid**. (**G0.2 closed 2026-09-13**: a second Livox driver; see §3.)
   - The DTP planner config still has upstream's **`max_vel: 5.0`**.
5. **Four paper claims should change** (§1): drop live Eva-Tracker, replace RTK ground truth with
   scripted events plus video, fly CTRV only, and define "search" as hold-and-climb.
   **Full draft by 2026-12-18; submit RA-L ≈ 2027-01-18 or IROS 2027 (deadline typically ~1 March).**
   ICRA's deadline is normally mid-September, so ICRA 2027 is out of reach — check the dates.
6. **Perception's biggest CPU cost now runs on the GPU (2026-09-13, desk + bag replay, not yet on
   the aircraft).** It was the organized-cloud build, not JPEG decode. With `cloud_backend: cuda`
   the cloud is bit-identical on 200/200 real frames, and `perception_node` drops from
   **1.9 cores to 0.7** in a replay of the same bag. Per-frame wall time did **not** improve, so
   this is CPU headroom for FAST-LIO and the §2.5 budget, not latency. The next lever is BoT-SORT's
   camera-motion compensation (§4 M5.3).
7. **Outside this plan (2026-09-19/20): `goal_mode: nmpc`.** `follow_planner` can compute the
   standoff goal with a receding-horizon optimisation, and (Stage 2, SUPER patch 0008) with a
   visibility cost from the loss oracle's map. It is an addition to the follower, not one of the
   rows below, and it changes no claim in §1. **Bench-gated, never flown, not installed**; it
   flies after the M5 baseline, as its own Stage A legs with `goal_mode` as the one variable.
   If it becomes a paper arm it needs its own rows and thresholds here. Record:
   [`fact-nmpc-follow-goal.md`](fact-nmpc-follow-goal.md).

---

## 0. Where things actually stand

| Piece | State | Evidence |
|---|---|---|
| Perception → lock → standoff → mux → patched SUPER | **Built, bench-run, never flown in the loop** | `dtp-follower-stack.md` |
| CTRV coast through occlusion, ABSTAIN on ambiguity | **Exists** — this *is* the carry-forward baseline | `target_manager.py` `COASTING` |
| Proximity veto, HOLD-on-silence, land-on-blindness ABORT | **Exists**; ABORT bench-tested disarmed only | `~/dtp_datasets/20260911_abort_bench` |
| Stage A observe-only mode + analysis | **Exists**; four bags today, none airborne on a sane estimate | §3 G0.2 |
| DTP goal path + our M3 offboard bridge together | **Never run**, props-off or otherwise | `dtp-follower-stack.md` open item 8 |
| LiDAR-position hover (flight plan P3) | **Never passed**; P5 was void (flew on optical flow) | `flight-test-plan.md` |
| MID360→FC yaw mounting offset | **Unmeasured** | flight plan P1 |
| Loss-cause classifier / reappearance / conditioning | **Does not exist** | grep above |
| Learned predictor (EigenTrajectory) | Code exists, **disabled**; no `.pth` on this box ("fetch from the AGX deploy") | `detection_pipeline/eigen.py:27,34` |
| Outdoor depth | Script exists, **not run outdoors** | `~/dtp_datasets/INDOOR_BASELINE_20260911.md` |
| Map clearing with raycasting on | **On everywhere, clearing never timed** | CLAUDE.md M4 bullet |
| SUPER follow-goal patch | **Regenerated 2026-09-13 (G0.4)**: carries `requested_p`, applied by `setup.sh` (which also symlinks `drone-dtp/`); pieces verified, **no full clean colcon build yet** | commit `b78fbc3` |
| Perception organized cloud on the GPU | **Done 2026-09-13 — desk and bag replay only**; `cloud_backend: cuda` in `dtp_params.yaml`, numpy fallback + in-node self-check | `drone-dtp/.../cuda_cloud.py`, `scripts/bench_cloud_backend.py`, `~/dtp_datasets/20260913_cloud_backend_replay` |

**Reusable data** (`~/dtp_datasets/`, 18 GB): `20260909_closedloop_run5` (132 s, has raw
`/livox/lidar`, re-acquisition 0.19–0.29 s over several real occlusions, 149 lock transitions) is
the best replay-harness input. `20260910_flightlegal_bench` has raw depth. The four
`20260912_*_stageA` bags have `/cloud_registered_body` + odom + `/dtp/*` + `/fmu/*`, but their lock
state is LOST in 537 of 628 samples in the longest, so they hold few clean occlusion events.

---

## 1. Paper claims vs. what we can deliver

| # | Claim in `main.tex` | Today | Plan item | Verdict | Paper edit |
|---|---|---|---|---|---|
| 1 | Loss-cause classification against the onboard map (contribution 1, Eq. 1, Alg. 1) | Not implemented | M6.1–M6.3 | **Keep — doable** | Close the Sec. III TBDs with §5.0's decisions: UNMAPPED class, 15-ray bundle, self-exclusion, 8 Hz budget not 15 Hz |
| 2 | Reappearance *belief* + planner conditioning (contribution 2) | Not implemented | M6.4–M6.5 | **Keep — doable, moderate risk** | Write §5.0's goal/yaw reduction into Sec. V-E |
| 3 | Marker-free LiDAR+RGB-D following | True: detection is RGB-D; LiDAR is map and odometry | M5.4 | **Keep**, contingent on outdoor depth | If camera depth fails outdoors, say the LiDAR depth path flew and its coverage |
| 4 | Forest evaluation, uninstrumented subject | Nothing has flown autonomously | Gate 0 → M5 → M6.7 | **Keep — highest schedule risk** | Report the tube course *and* the forest; the course gives the controlled trunk statistics reviewers ask for |
| 5 | Baselines: SUPER stock, carry-forward, **Eva-Tracker**, ours | Carry-forward = today's code; stock is a param mode; Eva-Tracker not present | M6.8 | **Cut Eva-Tracker from flight; add its loss policy as a re-implemented arm** | Code verified 2026-09-14: ROS 1 Noetic + MAVROS + MonoLoco, AGPL-3.0 — porting is weeks unrelated to the thesis. Its loss handling is prediction-only (Bézier QP + 2.5 m arc search, no map reason, no belief; the authors say it is short-term only). Re-implement that policy as a third baseline arm (`baseline_mode: eva`, row 27a) and its predictor as an M6.6 arm (row 24a). See [`eva-tracker-faster-lio-assessment.md`](eva-tracker-faster-lio-assessment.md) |
| 6 | Offline prediction study, TrajImpute protocol | Not started | M6.6 | **Keep — doable off-Jetson** | none |
| 7 | "CTRV ⊕ learned" in Table I; fused predictor in flight | Learned branch disabled, weights off-box | M6.6 | **Change** | Fly **CTRV**; learned/fused is an **offline-evaluated option** — the paper README already demands this |
| 8 | Classification accuracy vs an **RTK tag** | No RTK kit anywhere in this repo's inventory | M6.3, M6.7 | **Change** | Ground truth = **scripted events + synced tripod video**. Consumer GNSS on the subject is ~1–2 m — useless for "behind a 30 cm trunk" |
| 9 | Lost → search ("ascend-and-widen") | mux already does HOLD + 0.75 m climb; LOST latches for re-designation | M6.5 | **Change** | "Search" = hold-and-climb + operator re-designation. Autonomous re-lock after LOST is how the drone follows a stranger |
| 10 | Layered safety architecture + induced-failure experiments | Implemented | M5.7 | **Keep — cheap** | **Add the land-on-blindness ABORT** (new 09-11); Sec. V-H lacks it |
| 11 | Compute table incl. classifier and trajectory planner | Partial, bench | M5.3, M6.1 | **Keep** | `perception_node` measured 137–250 %, not Table III's 60.1 % |
| 12 | Motor-on calibration re-validation | Not done | M5.4 | **Keep — doable** | none |
| 13 | "11 autopilot hard faults" (Sec. VIII) | **22** classic field crashes at one PC (35 dumps incl. bench-induced) | G0.1 | Update | and whether the client-side fix held |
| 14 | "within one 15 Hz perception frame" (Sec. III) | `detect_rate_hz: 8.0` | — | Update | 8 Hz (125 ms) |

---

## 2. The numbers

### 2.1 Schedule

**Assumes:** two people full-time — **A** (aircraft, firmware, field lead) and **B** (software,
M6) — plus **one helper on every field day** (pilot, observer and subject are three people). If B
doubles as subject, B's desk rows slip one day per field day. Working days, Mon–Fri, starting
**Mon 2026-09-14**; Thanksgiving (Nov 26–27) off.

| # | Item | Who | Days | Dates | After | Exit (details in §3–§5) |
|---|---|---|---|---|---|---|
| 1 | Order 8 form tubes | B | 0.5 | Sep 14 | — | delivered by Sep 25 |
| 2 | **G0.2a** root-cause the 86 m divergence — **DONE 2026-09-13** (3.74 km; second Livox driver) | B | 2 | Sep 14–15 | — | cause written in `docs/` |
| 3 | **G0.1a** reproduce the hardfault — **DONE 2026-09-13**: shell path 5/34 (scattered PCs); RX-activity path **5/5, exact field pc:0x08022f60** | A | 3 | Sep 14–16 | — | per-cycle rate p̂ |
| 4 | **M5.4a** outdoor depth, leaves on | B | 1 | Sep 16 | — | camera-vs-LiDAR depth decision |
| 5 | **G0.2b** bridge guard + preflight refusals — **DONE 2026-09-13**, 32/32, hardware-verified | B | 3 | Sep 17–21 | 2 | new `m3_bridge_check` cases pass |
| 6 | **G0.1b** `UART7_RXDMA=n` — **FLASHED & FAILED 2026-09-13** (crash persists, same signature; adds a hang) | A | — | — | 3 | — |
| 6a | **G0.1c** client keeps the UART open across reconnects — **FLASHED 2026-09-13 (01:24 build)**: `--rx none` **30/30, 0 resets**; flood inconclusive (FC stopped reporting after cycle 1). **Open:** shell-path repro on the fix, ≥ 500 cycles, 14 h soak | A | 4 | Sep 14–18 | 6 | §2.2 G0.1 |
| 7 | **G0.4** regenerate SUPER patch, `setup.sh` applies it — **DONE 2026-09-13** (patch regenerated + verified; apply script rewritten; setup.sh wires drone-dtp + applies it) | B | 2 | Sep 22–23 | — | clean-clone build drives the follower |
| 8 | **P1** mechanical yaw check | A | 0.5 | Sep 23 | — | offset known to ±1° |
| 9 | **M5.2** follow envelopes S1–S3 + measured `robot_r` — **envelope DONE 2026-09-13** (S1 baked into config + patch; **`robot_r` 0.30 set 2026-09-14** from the measured 0.25 m; altitude is a pre-flight operator item; **geofence BUILT 2026-09-14 in `command_mux` (option 1, user decision) — `fence_check.py` 13/13, never flown**) | B | 1 | Sep 24 | 7 | configs in the patch |
| 10 | **Field day 1:** P3 + P4 | A | 1 | Sep 24–25 | 5, 6, 8 | P3/P4 thresholds |
| 11 | **M5.3a** target-point filter + erosion test — **BUILT 2026-09-13** (`patches/0002-…`, default off): synthetic gate 17/17 on the real `fsm_node`, `run5` replay LoS crossings 2.6 % → 0.7 %. **Open:** hardware walk-in and tube test, then enable | B | 4 | Sep 25–30 | 7 | filter thresholds |
| 12 | **Field day 2:** P5 on LiDAR | A | 1 | Sep 29–Oct 1 | 10 | P5 thresholds |
| 13 | **M6.3a** handheld dataset, tube course + trees | A+B+helper | 1.5 | Oct 1–2 | 1, 2 | 140 scripted events on disk |
| — | **Gate 0 closes** | | | **Oct 2** (weather buffer: Oct 9) | 6, 7, 10, 12 | |
| 14 | **M5.3b** clearing timing, CPU, FAST-LIO pinning, 20-min soak — **with `cloud_backend: cuda` live**. **Bench rehearsal 2026-09-13** (`m53_soak.py`, full graph, disarmed): odom 10.00 Hz / max gap 126 ms, CPU 384 %, perception 70 %, GPU cloud verified; **swap-out ~4 MB (C2 FAIL)**. **Pinning DONE + raycasting costed + swappiness fix 2026-09-13 (sixth session; see §4 M5.3 item 3).** Clearing timing on hardware not done | B | 3 | Oct 5–7 | 11 | compute budget |
| 14a | **M5.3c** perception cloud on the GPU — **DONE (desk) 2026-09-13**: bit-identical 200/200, replay 1.9 → 0.7 cores. Aircraft check folds into row 14 | B | 0 | — | — | self-check "verified" at every launch; `perception_node` ≤ 80 % |
| 14b | **M5.3d** tracker camera-motion compensation (BoT-SORT GMC, 23 ms CPU + 25 ms wall per frame) — **decide from row 16's yaw data**, only if §2.5 is still over budget | B | 2 | Oct 12–13 | 16 | ID switches under yaw ≤ today's; CPU ≥ 15 % of a core saved |
| 15 | **M5.1** one graph, props off — **runbook + scorer 2026-09-13**; bench rehearsal disarmed: G 2033/2033, S (sandboxed bridge) 99.46 %; A and T need the armed test | A | 3 | Oct 5–7 | 7, 9, 12 | M5.1 thresholds |
| 16 | **Field day 3:** Stage A airborne + analysis — **`analyze_shadow_goals.py` scores S1–S4 since 2026-09-13; the Stage A preflight refuses the M5.3a filter** | A | 2 | Oct 8–9 | 15 | Stage A thresholds |
| 17 | **M6.1** oracle in `fsm_node` + tests + timing | B | 6 | Oct 8–15 | 7 | timing budgets |
| 18 | **M5.4b** motor-on calibration; **M5.7a** induced failures, props off — **M5.4b comes from the Stage A bag** (`bag_to_calib_pairs.py`, validated 2026-09-13). **It found the old calibration ~12 px (≈1.1°) off in u since before 09-09; the 09-11 solve was deployed 2026-09-13** | A | 1.5 | Oct 12–13 | 15 | ≤ 2 px; 4/4 designed responses |
| 19 | **Field day 4:** S1 | A | 1 | Oct 14 | 14, 16 | S1 checklist |
| 20 | **Field day 5:** S2 | A | 1 | Oct 16 | 19 | S2 checklist |
| 21 | **M6.2** replay harness + labelling tool | B | 4 | Oct 16–21 | 17 | replays `run5` + Stage A bags |
| 22 | **Field days 6–7:** S3 + in-air M5.7b | A | 2 | Oct 20–22 | 20 | **M5 gate** |
| — | **M5 gate** | | | **Oct 23** | | |
| 23 | **M6.3b** label 140 events, classifier, sweeps → **D1** | B | 4 | Oct 22–27 | 13, 21 | **D1 Oct 27** |
| 24 | **M6.6** offline predictor study (off-Jetson) | A | 8 | Oct 26–Nov 4 | checkpoint | Table II filled |
| 25 | **M6.4** reappearance belief | B | 5 | Oct 28–Nov 3 | 23 | reappearance thresholds |
| 26 | **M6.5** conditioning + open-loop replay → **D2** | B | 5 | Nov 4–10 | 25 | **D2 Nov 10** |
| 27 | **M6.8** SUPER-stock baseline mode | B | 2 | Nov 11–12 | — | param mode works in replay |
| 28 | Conditioning check, props off + Stage A | A | 1 | Nov 12 | 26 | 3/3 OCCLUDED events pre-position |
| 29 | **Field days 8–10:** tube-course evaluation | all | 3 | Nov 13–19 | 22, 26, 27 | ≥ 30 usable E1 per arm |
| 30 | **Field days 11–12:** forest evaluation (+1 buffer) | all | 2 | Nov 20–25 | 29 | ≥ 15 usable E1 per arm |
| 31 | Analysis, statistics, tables | B | 5 | Nov 30–Dec 4 | 29, 30 | every `\tbd` result cell filled |
| 32 | Teaser + architecture figures | A | 4 | Nov 30–Dec 3 | 29 | 2 figures |
| 33 | Writing | all | 10 | Dec 7–18 | 31 | **full draft Dec 18** |
| 34 | Advisor review → submit | — | — | Jan 4–15 | 33 | **RA-L ≈ Jan 18, 2027** |

**Totals (estimate):** ≈ 115 person-days, 14 weeks to a full draft, **12 flight days + 1.5 handheld
days**. **Critical path:** 3 → 6a → 10 → 12 → 15 → 16 → 19 → 20 → 22 → 29. One lost week on
G0.1 moves every flight row by a week. Track B reaches D1 independently of flying.

**Season:** leaves are normally off here by early November. So rows 4 and 13 (depth and the
handheld trees segment) are placed before mid-October, when the canopy is still on. Row 30
(forest flights) will be **leaf-off** — report the season. If you want under-canopy flight,
row 30 must move ahead of the M5 gate, which is not safe.

### 2.2 Gate thresholds

| Gate | Pass when (all) |
|---|---|
| **G0.1** hardfault | *Status 2026-09-14: baseline measured (shell path 5/34, RX path 5/5); client-side fix flashed, **486 evidential clean `--rx none` cycles, 0 resets** (30 on 09-13, 455 on 09-14 — `firmware/g01/cycles-20260914_115558.csv`, 2.6 h on the bench); the count part of this gate is met; **the 14 h soak and the shell-path reproducer are still open.*** Baseline **≥ 300 session-loss cycles** (stop early at 5 faults) → p̂. After the fix: **0 faults in max(500, ⌈3/p̂⌉) cycles** *and* **0 faults in a 14 h powered soak** with the full stack. Measured rate over 09-11→09-12 was **1 per 4.5 h** wall-clock, so an unchanged bug survives 14 h with probability e^(−14/4.5) ≈ **4 %**. `Payload rx` 1150 B/s ±10 %, `tx` 35.9 kB/s ±10 %, `timesync converged: true` |
| **G0.2** estimator | Step guard rejects **> 0.5 m between consecutive 10 Hz samples** (5 m/s, 3.3× S3's `max_vel`); `reset_counter` increments on FAST-LIO restart. Preflight refuses if: odom **< 9.5 Hz** over 10 s; **\|odom\| > 1.0 m** at bring-up; **\|EKF2 − FRD(odom)\| > 0.20 m** (M2: 7 cm lever-arm offset + 10.4 cm max tracking error); **any EKF reset in the last 60 s** |
| **P1** yaw mount | Offset known to **±1°**; if **≥ 2°**, it goes into `px4_odom_bridge` |
| **P3** hover | **≥ 60 s at 1.0–1.5 m** AGL; `cs_ev_pos` True **100 %**; horizontal error **p95 ≤ 0.25 m** (the flow-only hover in P5 was 0.209 m); **0 EKF resets**; M2 correlations **≥ 0.95**, signs correct; no circling |
| **P4** latency | Frame-matched odometry latency on a moving rig **p95 ≤ 100 ms** (bench: 49–57 ms) |
| **P5** Offboard | **≥ 60 s** armed Offboard hold on LiDAR; setpoint drift **≤ 0.02 m** (bench 0.013 m); tracking **p95 ≤ 0.25 m**; `cs_ev_pos` True throughout |
| **M5.1** one graph | ≥ **99 %** of setpoints equal SUPER's command under the conversion (bench: 910/913); GoalStatus for ≥ **99 %** of follow goals within **0.2 s**, `requested_p` within **1 cm**; ABORT → NAV_LAND acked **≤ 1 s** after its 2.0 s persistence, `nav_state` → AUTO_LAND, while both bridges publish; RC ch11 takeover wins **3/3** |
| **M5.3** map + compute | With the target filter on, a camera→target segment crosses an ex-trail cell in **≤ 2 %** of LOCKED frames at 3.5 m standoff; subject standing **0.3 m from a tube for 10 s** erodes **0** tube voxels; `/lidar_slam/odom` **≥ 9.9 Hz** over **20 min**, no gap > 300 ms; total CPU **≤ 600 %** of 800 %; swap **0 B**; `perception_node` **≤ 80 %** of a core at 8 Hz with its log showing *"GPU cloud verified against numpy"* and `cloud … [cuda]` for the whole soak (any `DISABLED` line fails it) |
| **M5.4** sensors | On-person depth return **≥ 50 % of box pixels at 4 m in direct sun** → camera depth stays default, else LiDAR path. Motor-on `score_alignment` **\|Δu\|, \|Δv\| ≤ 2 px** (bench +1 / −1 for the calibration deployed 2026-09-13; the 09-07 one read 0 / +1 on its own capture) |
| **M5.5** Stage A | Standoff-goal error **p95 ≤ 0.5 m**; lifted-target jitter **p95 ≤ 0.3 m**; error slope vs yaw rate **≤ 0.1 m per rad/s** up to 0.8 rad/s; perception blind **≤ 3 %** |
| **M5 gate** | **≥ 20 min** autonomous following at S3 over **≥ 5 flights**; horizontal distance to subject **≥ 2.5 m 100 %** of LOCKED time and in **[2.5, 4.5] m ≥ 90 %**; bearing error **median ≤ 15°, p95 ≤ 35°** (the ±35° cone); **0** proximity-veto trips from the followed subject walking normally; **0** stack-caused interventions; **0** new FC faults |
| **D1** classifier | E1 OCCLUDED recall **≥ 0.85** (n = 60; 95 % Wilson lower bound ≈ 0.74); E2 false-OCCLUDED **≤ 0.10** (n = 40; upper bound ≈ 0.23); E3 OUT_OF_VIEW correct **≥ 0.90**; timing budgets in §2.4 met |
| **M6.4** reappearance | On E1: correct edge side **≥ 80 %**; median \|t_pred − t_obs\| **≤ 0.5 s**; median position error **≤ 0.5 m**; the 90 %-mass region contains the true re-emergence **≥ 80 %** of the time |
| **D2** conditioning | Open-loop replay on E1: region-gated re-lock rate **≥ band rate + 15 points**; **0** wrong locks in both arms; ABSTAIN count **≤ band's + 10 %** |

### 2.3 Sample sizes

| Study | n | Why |
|---|---|---|
| Handheld (M6.3) | **E1 60, E2 40, E3 20, E4 20 = 140 events**, ≈ 40 s each → **≈ 95 min** of recording | Gives the confidence bounds in D1 |
| **Ablation 1 (primary)** — tube course, conditioning ON vs OFF | **≥ 30 usable E1 per arm** | Two-sided Mann–Whitney, α = 0.05, power 0.8 → detects a standardized effect **d ≥ 0.74**. Pre-register: **primary endpoint = median re-acquisition time on E1** |
| SUPER-stock baseline | **≥ 15 usable E1** | Context row, not a primary test |
| Forest replication | **≥ 15 usable E1 per arm** | Detects d ≥ 1.05 — reported as replication, not as the primary test |
| Flights to get there | **5 E1 attempts per flight × 80 % usable = 4** → 8 ON + 8 OFF + 4 stock + 4 spare = **24 course flights**; 4 + 4 + 4 spare = **12 forest flights** | Arms interleaved **ABBA** by flight |

### 2.4 Classifier and reappearance parameters (starting values; M6.3 sweeps them)

| Parameter | Start | Sweep |
|---|---|---|
| Target cylinder | r **0.25 m**, samples at target z **−0.5 / 0 / +0.4 m** | — |
| Rays N | **15** (3 heights × 5 lateral) | {1, 5, 15, 35} |
| Stop short of target `r_excl` | **0.4 m** | {0.2, 0.4, 0.6} |
| Blocked threshold ρ\* | **0.5** | 0.1 … 0.9, step 0.1 |
| Camera cone for OUT_OF_VIEW | **±35°** (`follow_planner` `camera_fov_half_deg`) | — |
| CTRV rollouts K / horizon / step | **32 / 3.0 s / 0.1 s** | K ∈ {8, 32, 128} |
| Rollout noise | σ_v **0.3 m/s**, σ_ω **0.5 rad/s** (estimate — fit on M6.3 data) | — |
| Coast cap under OCCLUDED | **3.0 s** → predictor horizon **3.2 s** (usable coast = horizon − 0.20 s) | {2, 3, 4} s |
| Re-acquisition window margin δ | **0.5 s** | — |
| Region gate | ellipse, semi-axes **max(0.5 m, 2σ)** of the dominant cluster | — |
| **Budgets** | classification **p99 ≤ 2 ms**; reappearance **p99 ≤ 10 ms**; `LossEvent` out ≤ **30 ms** after `/dtp/follow_target` in (p99); `fsm_node` **+≤ 5 %** of a core, **+≤ 10 MB** | — |

Rough cost check: 15 rays × ≤ 100 cells (10 m at 0.1 m voxels) = 1.5 k cell reads per classification,
and 32 rollouts × 30 steps × one 100-cell ray ≈ 96 k reads for reappearance. Both are milliseconds at
most at 8 Hz (a 125 ms frame).

### 2.5 Compute budget (Orin NX, 8 cores = 800 %)

| Process | Measured | Note |
|---|---|---|
| `perception_node`, numpy cloud | **137 %** (09-12, 8 Hz, field stack) – **250 %** (09-11, 15 Hz); **187–193 %** replaying `20260910_flightlegal_bench` | `dtp_params.yaml` comments; `20260913_cloud_backend_replay` |
| `perception_node`, **cuda cloud (the config since 09-13)** | **≈ 69 %** in the same replay (40.6 / 40.8 CPU-s over 59 s) — not yet measured on the aircraft | same |
| … of which, per frame at 8 Hz | cloud **8–11 %** (numpy: 90 %), YOLO track **33 %** (the same model under ByteTrack 15 %, so BoT-SORT's GMC ≈ **18 %**), JPEG decode **5 %** | `scripts/bench_cloud_backend.py --yolo` |
| `fsm_node` (raycasting on) | **112 %** | `dtp-follower-stack.md` live state |
| FAST-LIO | **74 %** with `extrinsic_est_en: true` (the deployed config) / **31 %** with it off | `dtp_params.yaml` comments |
| `px4_offboard_bridge` / `px4_odom_bridge` | **34 % / 3 %** | CLAUDE.md M3 |
| `command_mux` / `target_manager` / `follow_planner` | **10.4 % / 4.0 % / 0.7 %** | paper Table III |
| GCS with a browser watching | **21 %** | CLAUDE.md GCS |
| Livox driver, loss oracle | **not measured** / budget +5 % | |
| **Sum with the numpy cloud** | **≈ 353 %** (best case) to **≈ 509 %** (worst case) | the 09-12 field stack ran at load 6.0 |
| **Sum with the cuda cloud** | **≈ 285 %** (FAST-LIO at 31 %) to **≈ 328 %** (at 74 %), taking perception at the replay's 69 % | estimate until row 14 measures it live |
| **Budget** | **≤ 600 %**, FAST-LIO pinned to 2 dedicated cores, swap 0 B | M5.3 exit |

### 2.6 Logistics

| | Number |
|---|---|
| Flight days | **12** (P3/P4 1, P5 1, Stage A 1, S1 1, S2 1, S3 2, course 3, forest 2) + **1.5** handheld |
| Flights | ≈ **71** (4 + 4 + 4 + 5 + 6 + 12 + 24 + 12) |
| Airborne time | ≈ **6 h**, assuming **5 min of autonomous following per pack** (estimate — endurance is still `\tbd` in the paper; measure on field day 1) |
| Batteries | **≥ 4 packs** for 8 flights/day with field charging |
| People per field day | **3** — pilot, stack operator, subject |
| Weather limits | wind **≤ 4 m/s** sustained; no rain; LiPo pre-warmed below 10 °C |
| Storage | ≈ **0.5 GB/min** for an explicit topic list (raw LiDAR + compressed colour + `/dtp/*` + `/fmu/*`; estimate — the GCS's *All topics* was measured at ~1 GB/min, so don't use it). Handheld ≈ **50 GB**; flights ≈ **250 GB**. Free on the Jetson today: **277 GB** → **offload to an external SSD after every field day** |
| Purchases | 8 × 12 in × 4 ft cardboard form tubes (≈ $200–300) + sandbags/stakes; a 1 TB external SSD (≈ $100). Everything else is on hand |

---

## 3. Gate 0 — airworthiness. Blocks every flight in M5 and M6.

### G0.1 — The `uxrce_dds_client` hardfault (rows 3, 6, 6a)

> **STATUS 2026-09-13 — step 2's candidate FAILED; a different fix is flashed and half-validated.**
> The rest of this section is the plan as written on 09-12. What happened (full record:
> `firmware/faults/README.md`, CSVs in `firmware/g01/`):
> - **Reproduced (step 1).** Shell `stop`/`start` 5/34 with scattered PCs, which proves memory
>   corruption. The RX-active session loss gave 5/5 at the **exact** field PC.
> - **`CONFIG_UART7_RXDMA=n` (step 2) FAILED.** `--rx none` crashed on cycle 1 with the same
>   signature, and the build also hangs under a flood. The fault is `uart_close` racing
>   `uart_datareceived`; `up_dma_shutdown` was a register value, not the site.
> - **Client-side fix (step 4's "code we own" alternative), flashed:**
>   `ark-fpv-uxrce-no-uart-close-on-reconnect.patch`.
>   - `--rx none`: **30/30, 0 resets, 0 dumps**.
>   - `--rx random`: **inconclusive** — `boot_ms` froze from cycle 2, the flood wedge.
> - **Remaining for the gate:**
>   - `xrce_fault_repro.py` on the fix;
>   - enough cycles to reach ≥ 500 (`--rx none` is the representative test; the flood wedges any
>     build);
>   - the 14 h powered soak;
>   - then `m2_gate_check.py` / `m3_bridge_check.py`.

**Measured:** `firmware/faults/` holds **22** logs; **20** are the same `uxrce_dds_client` fault
(`pc:0x08022f60`, `lr:0x08023263`), **nine of them from 09-11 11:25 to 09-12 23:29**
(≈ 36 h, 1 per 4.5 h). Two are stamped ~20–30 min before today's first Stage A recording, if the
card clock is UTC.

The decode already says what it is: `uart_close()` → `up_dma_shutdown()` tears down **RX DMA**
while `uart_datareceived()` still uses the buffer. **Our board config has RX DMA on exactly that
UART:** `boards/ark/fpv/nuttx-config/nsh/defconfig` sets `CONFIG_UART7_RXDMA=y` (UART7 = TELEM1 =
the XRCE link). We already build this firmware locally.

1. **Reproduce first** (a fix that was never reproduced is not a fix). Bench, FC powered, props
   off. Provoke XRCE session loss ≥ 300 times — restart the agent, unplug TELEM1, or
   `uxrce_dds_client stop/start` from `px4_shell.py` — scanning the card with
   `clear_fault_logs.py`. If none of the three reproduces it, the 14 h soak alone decides.
2. **Candidate fix: `CONFIG_UART7_RXDMA=n`** (PIO receive, the mirror image of the Jetson-side
   fix). Jetson→FC traffic is 1.2–5 kB/s, trivial for interrupt-driven receive. Keep TX DMA
   (FC→Jetson is 35.9 kB/s). Flash is 98.25 % full; this removes code, it doesn't add any.
3. **Pass:** §2.2 G0.1, then `m2_gate_check.py` and `m3_bridge_check.py` still pass.
4. If PIO receive drops bytes at 3 Mbaud, patch NuttX so RX DMA is stopped before `uart_close`
   frees it — and say so in paper Sec. VIII.

### G0.2 — The estimator accepted a 3.74 km error (first read as 86 m) — CLOSED (rows 2, 5)

> **A LEAD FOR THE OPEN ODOMETRY-AGE PROBLEM (found 2026-09-13, second session).** From
> **2026-09-12 19:03** ARK OS's `logloader.service` re-uploaded one ULog every ~2 s: ~500 MB/min,
> 241 GB, ~42 % of a core.
> - **All four 09-12 Stage A bags were recorded during it.**
> - **On the bench full graph while it ran,** `/lidar_slam/odom` was delivered **3.9 s late** once:
>   stamps on time, seen by two independent subscribers, and another ARK service stalled in the same
>   seconds.
> - **With it stopped,** a 20-min soak of the same graph never exceeded a **126 ms** gap.
>
> Not proven to be the 09-12 cause — the 09-12 age tail (p95 up to 571 ms, 17 EKF2 resets) was not
> reproduced with it on — but it is the first candidate with a mechanism.
>
> **Fixed 2026-09-13 16:45.** It looped again after a reboot, reaching 344 GB and a 95 % disk.
> - **The cause:** a logloader bug that fetched the log mid-write.
> - **The fix:** the service disabled, the duplicates deleted (364.8 GB freed), and the FC's
>   complete copy kept.
> - **Record:** [`fact-ark-logloader-loop.md`](fact-ark-logloader-loop.md).
>
> **CLOSED 2026-09-13.** The rest of this section is the plan as written; the outcome is in
> [`fact-fastlio-divergence-2026-09-12.md`](fact-fastlio-divergence-2026-09-12.md). Corrections:
> - **The error was 3.74 km**: x −3340, y −1672, z −86.7 m. Only z had been read.
> - **The cause was a second Livox driver**, not `extrinsic_est_en`. `dtp_system.launch.py` on top
>   of `run.sh` took the MID360's stream as PointCloud2 and starved FAST-LIO of scans for 225 s.
>   Reproduced on the bench.
> - **The resets were 17 horizontal in 220–355 s** (counter 9→26), not 23. They coincide with
>   odometry age past `EKF2_DELAY_MAX`, a separate problem the preflight now refuses (E4).
> - **Delivered:**
>   - the bridge guard (stamp gaps, not arrival gaps, after replaying all bags);
>   - `estimator_preflight.py` E1–E8, also inside `stage_a_preflight.py`;
>   - `m3_bridge_check` T8–T14 (32/32);
>   - duplicate-driver refusals in `run.sh` and the DTP launch files;
>   - flight plan §1a.
> - **Hardware end-to-end on the real FC:** healthy GO, diverged NO-GO, 0.05 m max EV ever sent,
>   EKF2 reset onto a restarted FAST-LIO.

**Measured in `~/dtp_datasets/20260912_2006_stageA` (644 s):**
- **The error.** From the first sample, `/lidar_slam/odom` z ≈ **−86.7 m** and PX4
  `vehicle_local_position.z` ≈ **+86.7 m**, with `z_valid: True` and `eph 0.06`.
- **Armed on it.** Armed **51→174 s** (Position mode 63→172 s), again **246→267 s**.
- **Resets.** **23 EKF position resets** (`xy_reset_counter` 10→26 over 220–355 s; the counters
  restart at 519 s, which is an EKF2 restart or FC reboot), with z briefly at **641 m**.

CLAUDE.md also recorded FAST-LIO at **3.75 Hz** today. Nothing in the preflight caught any of it.
`px4_odom_bridge.py` forwards anything finite: it drops only NaN (line 103) and hardcodes
`out.reset_counter = 0` (line 161).

1. **Root-cause from the bag and `~/.ros/log`.** Lead worth testing first: the deployed
   `mid360.yaml` has **`extrinsic_est_en: true`** (online IMU–LiDAR extrinsic estimation). The DTP
   notes measure it at **74 % CPU vs 31 %** with it off, and `~/dtp_datasets/README.md` says the
   DTP bags were recorded *after* an "extrinsic_est_en fix". Online extrinsic estimation is a
   classic divergence source. Also check for a FAST-LIO restart with the bridge left running.
2. **Bridge guard and preflight refusals:** §2.2 G0.2. Add both to `m3_bridge_check.py`, and the
   refusals to `stage_a_preflight.py` and the flight plan's §1a banner.

### G0.3 — Fly on LiDAR at all (rows 8, 10, 12)

Flight plan **P1**, **P3**, **P4**, **P5**, with §2.2's numbers. None has passed yet. M5 starts
where P5 passes.

### G0.4 — Make SUPER reproducible before adding to it (row 7)

> **DONE 2026-09-13 (`b78fbc3`).**
> - **Patch:** regenerated against `2ad3419` + the isnan fix; it now carries `requested_p`,
>   `GoalStatus`, `setFollowGoal` and `YAW_TO_TARGET`.
> - **`apply_super_patch.sh`:** rewritten for the single combined patch.
> - **`setup.sh`:** symlinks `drone-dtp/` into `~/super_ws/src` and applies the patch.
> - **Verified:** each piece independently.
> - **Not done:** a full clean colcon build from a fresh clone (it would have disturbed the live
>   workspace). The regenerated patch still ships `max_vel: 5.0` — M5.2.

1. Regenerate `drone-dtp/patches/0001-…` from the live `~/super_ws/src/SUPER` (it lacks
   `requested_p`).
2. Have `scripts/setup.sh` run `apply_super_patch.sh`, so a clean install yields a SUPER the
   follower can drive.
3. From here on, every M6 SUPER change lands in the patch in the same commit.

---

## 4. M5 — the DTP follower flies

Everything here runs the **current** follower, i.e. the carry-forward arm.

### M5.1 — One graph, props off (row 15)
Bring up FAST-LIO + SUPER (`follow_test_ros2.yaml`) + DTP + **both** bridges + GCS from one
written runbook. Then, **armed in Offboard with props off**, a person walks in front of the rig and
the §2.2 M5.1 numbers are checked. This settles open item 10 ("third writer to the FC") by test
rather than argument. Today the ABORT has only ever been exercised disarmed.

> **2026-09-13: the runbook and the scorer exist; the graph was rehearsed on the bench, disarmed.**
> - **Runbook:** [`m5.1-one-graph-runbook.md`](m5.1-one-graph-runbook.md).
> - **Scorer:** `scripts/px4/m51_one_graph_check.py`, which exits 3 (INCOMPLETE, not a pass)
>   unless all four sections were scored.
> - **Found while rehearsing — the config trap.** Every stack `run.sh` started planned on
>   `click_smooth_ros2.yaml`, and the DTP launch refuses a second `fsm_node`. So a follow flight
>   would have silently flown the click config. `run.sh` now takes `SUPER_CONFIG`.
> - **Found while rehearsing — the veto.** A person seated beside the rig holds the mux in
>   `HOLD|PROXIMITY`, and the mux then publishes nothing.
> - **Predicted from PX4's source** (runbook §0): NAV_LAND wins against the bridges, and a held
>   ch11 does not re-take the aircraft.
> - **Not done:** the armed props-off trials. They need the RC pilot and a walker, and fit the
>   10 s `COM_DISARM_PRFLT` window per trial.

### M5.2 — Follow-flight planner envelope (row 9)
**`follow_test_ros2.yaml` has upstream's `max_vel: 5.0` / `max_acc: 5.0`, in `src/` and
`install/`.** `setup.sh` applies the 0.1 m/s first-flight limits to `click_smooth_ros2.yaml` only.
`robot_r` was 0.2 m; **0.30 m since 2026-09-14** (measured 0.25 m hub-to-prop-tip + 0.05 m).

| Stage | `max_vel` | `max_acc` | Subject | Flights |
|---|---|---|---|---|
| **S1** | 0.5 m/s | 0.5 m/s² | standing → 0.3 m/s | 5 |
| **S2** | 1.0 m/s | 1.0 m/s² | 0.7 m/s | 6 |
| **S3** | 1.5 m/s | 1.5 m/s² | 1.2 m/s | 12 |

All stages share these settings:
- `yaw_dot_max` **1.0 rad/s** (a subject crossing at 1.2 m/s, 3 m away, needs 0.4 rad/s);
  `max_omg` **1.5**.
- `robot_r` = measured prop-tip radius **+ 0.05 m** — **done: 0.25 + 0.05 = 0.30**.
- Altitude = target z + 0.5 m (≈ 1.5 m AGL).
- Geofence **30 m × 4 m** box.
  - **⚠️ 2026-09-13: nothing enforces a geofence on this aircraft.** PX4's `GF_MAX_HOR_DIST` 50 /
    `GF_MAX_VER_DIST` 50 is **inert** under the vision-only EKF2 profile: `geofence.cpp:354`
    evaluates distance only when `home_global_position_valid()`, and `EKF2_GPS_CTRL=0` gives no
    global position.
  - No node in the stack implemented a box either.
  - **Options, as they stood:**
    - a goal clamp/veto in `command_mux`;
    - GPS fusion (`ekf2_profile.py vision-gps`) so PX4's own fence works;
    - venue + pilot only.
  - **✅ DECIDED AND BUILT 2026-09-14 (user: option 1).** `command_mux/fence.py` + `fence_*`
    params in `dtp_params.yaml`. Why this one: it works under the estimator profile every gate
    was signed off on, changes no FC parameter, and is bench-testable like the rest of the mux.
    GPS fusion stays the long-term answer if venues demand a hard fence, but it is an estimator
    change that would re-open M2 the week of the first follow flight.
    - **Geometry.** A rectangle in `lidar_init` (FAST-LIO's origin at the start spot; E5 keeps the
      aircraft within 1 m of it at arm), x along the boot heading unless `fence_yaw_deg` turns
      it. Default = this table's 30 × 4 m: x ∈ [−2, 28], y ∈ [−2, 2] — the aircraft 2 m from the
      near end facing down the corridor, since the follower flies *behind* the subject. **Re-cut
      per venue.** The node refuses to start on a box that excludes the origin.
    - **Soft boundary = the edge.** Every goal the mux publishes — FOLLOW, NAV and its own HOLD
      goals — is clamped `fence_margin_m` 0.5 inside it, at the shaping step (so the proximity
      veto sees the clamped segment under the same both-goals rule as the descent guard) and
      again at the single publish point (a bite there is an ERROR: a path skipped the clamp). A
      subject who leaves the box is followed to the edge and no further; the state stays FOLLOW.
    - **Hard boundary = edge + `fence_breach_land_m` 3.0.** The *aircraft* past it raises a
      `FENCE` veto (HOLD, hold goal clamped inside — still commanding it back) that escalates to
      `NAV_LAND` through `abort.py` after `abort_after_s` with authority — PX4's own
      `GF_ACTION=land`. At S1's 0.5 m/s the aircraft covers 1 m in the 2 s window, so crossing
      the line *is* the land decision. Between the two boundaries: WARN + `fence: aircraft
      N m outside` in `/mux/state`, no veto. `< 0` disables the hard boundary (clamp only).
    - **Not covered, by construction:** goals that never pass through the mux — an RViz
      `/goal_pose` click and the web GCS's RC card publish straight into `fsm_node`. Operator
      actions with a human watching; the follower's goals are the ones nobody is clicking.
    - **Gates.** `python3 drone-dtp/scripts/fence_check.py` — the real node on a private
      localhost DDS domain, PX4 command topic remapped, **13/13**: clamp, soft breach reported,
      hard breach → `HOLD|FENCE` with the hold inside, latch (no re-anchor churn),
      `ABORT|LAND` + `NAV_LAND` (21) observed, terminal, origin refusal, disabled path logged at
      ERROR. **Re-run after any `command_mux` change.** `test_fence.py` 10/10, `test_abort.py`
      12/12 (FENCE escalation added). `stage_a_preflight.py` FAILs on `fence_enable: false` in
      both stages and prints the box.
    - **Open.** Never run on the aircraft. `HOLD_REANCHOR_M` now measures from the odom the hold
      was *latched* on rather than from the hold goal (a fence-clamped hold can sit metres from
      the aircraft) — same meaning, verified in the gate, not yet seen on hardware. Walk the box
      before arming: it moves with the start point and the boot heading.
- `p_max` **0.95** in the follow config (0.98 today).

Put all of it in the patch, so a re-clone cannot silently restore 5 m/s.

### M5.3 — Map, trail and compute, with everything running (rows 11, 14)

**The trail, in numbers — this is why the target filter is on the critical path.**
- **Filling and clearing.** In ROG-Map one hit takes a cell from unknown to occupied
  (`l_hit` 2.197 > `l_occ` 1.735). Every ray through a cell in a frame counts, but a single hit in
  that frame discards all of that frame's misses (`probabilisticMapFromCache`). From saturation at
  `p_max` 0.95, a cell needs **31 clean rays to stop being OCCUPIED and 74 to become known-free**
  (54 and 98 at 0.98). In between it reads **UNKNOWN**, which the classifier's UNMAPPED rule would
  also trip on.
- **How old the trail is.** When the subject walks straight away, their trail lies on the camera's
  line of sight. It is **3.5 m / 1.2 m/s = 2.9 s** old at S3 and **5 s** at S2.
- **Unknown.** Rays per cell per frame fall with range, and with `point_filter_num: 3` the count is
  unknown. Measure it; don't assume it.

Work:
1. **Target-point filter:** drop LiDAR points within **r 0.5 m** and **z ∈ [target z − 1.2,
   target z + 0.6]** of the *observed* locked target before ROG-Map insertion, **only while
   LOCKED** (never at a coasted position — that is where the occluder is). Exit: §2.2 M5.3.

   > **BUILT 2026-09-13 (`patches/0002-rog-map-target-exclusion.patch`), default OFF. Bench and
   > replay only.**
   >
   > **Two of the numbers above were wrong, and the data changed them.**
   > - **The band.** Perception's target z is the box at 0.65 of its height **from the top** (the
   >   waist), not the torso centre. The person's LiDAR returns span **−0.20 … +1.18 m (p1 … p99)**
   >   around it, so the band is **[z − 1.0, z + 1.3]**; `+0.6` would have left the upper body in.
   > - **The radius.** At `r 0.5` the cylinder held only 41 % of the person's points in the worst
   >   5 % of scans, and **one escaped point is one occupied cell**. It is **`r 0.6`** (71 %).
   >   `r 0.7` reaches 92 %, but blanks more of what the person stands next to.
   >
   > **How the filter is built:**
   > - **Centre:** `position + velocity × (t_scan − t_target)`. It is 0.18 m p50 / 0.52 m p95 from
   >   the person's points, against 0.23 / 0.56 m unextrapolated and 0.24 / 0.55 m for the CTRV
   >   forecast. An extra lead does not help.
   > - **Observations used:** seen frames only, i.e. neither coasted nor held. One older than
   >   **0.5 s** is dropped (past 0.4 s the centre degrades fast).
   > - **Inside the cylinder** a scan inserts no hits **and no misses**. Rays up to it still clear the
   >   line of sight.
   >
   > **Evidence:**
   > - `drone-dtp/scripts/target_filter_study.py` on `20260909_closedloop_run5`;
   > - `target_filter_check.py`, 17/17 on the real `fsm_node`: trail OFF 189–195 cells / ON 0;
   >   0 tube cells eroded inside the cylinder; line-of-sight clearing identical to OFF;
   >   coasted, held and stale targets never filtered;
   > - `target_filter_replay.py` on `run5`: LoS crossings **2.6 % → 0.7 %** of scored frames,
   >   ex-trail cells mean 167 → 99. What remains comes from unlocked, coasted and held periods.
   >
   > **Guard:** `stage_a_preflight.py` FAILs a Stage A with it on.
   >
   > **Still to do:** the walk-in/walk-out and tube tests on hardware, **before** `enable: true`
   > in the flight config. With it on, the planner does not avoid the followed person.
2. Clearing time with the filter off at 2/3.5/5 m, for the record and for Sec. VIII.
3. Raycasting on vs off CPU with perception running; FAST-LIO pinned; `extrinsic_est_en` per G0.2;
   20-min soak. Exit: §2.5 budget.

   > **Bench, 2026-09-13 (sixth session) — pinning built, raycasting costed, the swap FAIL fixed.
   > Full graph (Livox + FAST-LIO + `fsm_node` on `follow_test_ros2.yaml` + both bridges + DTP
   > observe-only, GPU cloud, `jetson_clocks`), disarmed, at the desk, no goal, a laptop RViz
   > watching. Evidence: `~/dtp_datasets/20260913_m53_soak_bench/`.**
   > - **Pinning is in and on by default.** `run.sh` gives FAST-LIO `taskset -c 6,7` (via
   >   `m1_super_lidar.launch.py` args `fastlio_cpus`/`fsm_cpus`) and the Livox driver + `fsm_node`
   >   `0-5`; `dtp_follower.sh` pins the whole DTP tree to `0-5` (`DTP_CPUS`); the GCS starts the
   >   bridges under `taskset -c 0-5`. Verified with `Cpus_allowed_list`: `fastlio_mapping 6-7`,
   >   every other node `0-5`. `FASTLIO_CPUS=` (empty) disables it. No root, no rebuild: the launch
   >   file is a symlink into `~/super_ws/src/super_bringup/` (re-copied).
   > - **Raycasting costs ~7 points of a core in this scene, not the 48 % once attributed to it.**
   >   3-min windows, same graph, config swapped only: `fsm_node` **22.5 %** with raycasting on,
   >   **15.2 %** off (total 280 % vs 250 %). The rehearsal's `fsm_node` 88 % was *planning* —
   >   that run streamed 2033 follow goals and the offboard bridge ran at 42 %; here the FSM sat
   >   in INIT and the bridge at 12 %. Raycasting stays on.
   > - **C2 (swap) fixed at the source: `scripts/jetson_swap_setup.sh install`** writes
   >   `/etc/sysctl.d/80-super-dtp-swap.conf` (`vm.swappiness = 10`, stock 60). The rehearsal's
   >   4 MB swap-out was reclaim parking idle pages in zram while a bag filled the page cache.
   >   Survives a reboot, not a reflash.
   > - **20-min soak (pinned, swappiness 10, the M5.1 recorder running, `~/dtp_datasets/20260913_m53_soak_bench/soak/`):
   >   O1 **10.00 Hz**, C1 mean **292 %** (p95 window 313 %; ~70 % of it desk load), **C2 PASS — 0 pages
   >   swapped**, P1 `perception_node` **65 %**, P2 verified. **O2 FAIL: max gap 944 ms, 4 gaps > 300 ms** —
   >   all in ONE ~10 s episode at minute 5.5 (bag +334…344 s). In that episode `/lidar_slam/odom`, `/tf`
   >   and the bridge's `/fmu/in/vehicle_visual_odometry` all gapped 940 ms while the XRCE agent's
   >   50 Hz `vehicle_local_position` never gapped past 40 ms, FAST-LIO's CPU did **not** rise (21.8 %
   >   vs 23.3 %), and one 300 ms *stamp* gap shows it then dropped two scans (the depth-2 queue). So
   >   FAST-LIO **waited**, on something outside the CPU. **Leading suspect, not proven:** a laptop RViz
   >   was subscribed to `/cloud_registered` over Wi-Fi the whole run — **48 Mbit/s** of transmit,
   >   38 k lifetime qdisc drops — and FastDDS sends over a *blocking* UDP socket, so a Wi-Fi stall
   >   fills the send buffer and blocks FAST-LIO's publish inside its callback. It is the same suspect
   >   CLAUDE.md recorded for the 09-12 3.75 Hz event. **Field rule until it is settled: no laptop
   >   subscription to `/cloud_registered` while flying** (`ros2 topic info /cloud_registered -v` must
   >   list only `fsm_node`). **Candidate fix to test on the bench:** an ASYNCHRONOUS publish mode for
   >   FAST-LIO's participant via a FastDDS XML profile, then re-run this soak with the laptop
   >   subscribed. Pinning is *not* implicated: the episode had no CPU spike, and the other 1190 s were
   >   ≤ 157 ms.
   > - **Same evening: mechanism reproduced, ASYNCHRONOUS rejected, `non_blocking_send` adopted.**
   >   `docs/fact-fastdds-blocking-send.md` has the table. Run the flight stack with
   >   `FASTLIO_DDS_PROFILE=nonblocking`. The laptop also receives every datagram twice (Wi-Fi +
   >   Tailscale locators) — a `laptop_viewer.sh` fix, not done.
   > - **This is still not the M5.3 exit** — that is on the aircraft, with a walker, after the
   >   target filter's hardware test. Field-day CPU will also lack the desk's ~60–90 % of
   >   unattributed load (remote sessions).
4. **Perception's organized cloud on the GPU — built and verified at the desk 2026-09-13; confirm
   it on the aircraft inside item 3's soak.** What was measured, so nobody re-derives it:
   - **The cost was never JPEG decode.** Per frame at 8 Hz, from 200 real frames
     (`scripts/bench_cloud_backend.py --yolo`): the numpy depth → colour → range image → cloud
     chain **112 ms CPU / 38 ms wall (90 % of a core)**; YOLO + BoT-SORT 41 ms CPU; decode
     5.8 ms (5 %).
   - **`cloud_backend: cuda`** (`dtp_perception/cuda_cloud.py`) runs that chain in torch on the
     GPU YOLO already owns: **10–14 ms CPU, bit-identical to numpy on 200/200 real frames**
     (0 differing pixels, 0 m).
   - **In the real node** (`scripts/replay_perception.sh`, the bag replayed twice per backend):
     **110 / 114 CPU-s → 41 / 41 CPU-s** over 59 s, i.e. **1.9 cores → 0.7**.
   - **Target outputs:** on an identical frame schedule, numpy vs cuda differed p95 1.65–2.18 cm,
     against **1.71 cm between two numpy runs**. The backend adds nothing measurable beyond the
     replay's own noise (see M6.2).
   - **Wall time per frame did not improve in the node** (cloud 35 → 28–33 ms, YOLO +5–10 ms on
     the shared GPU). So **this does not by itself make `detect_rate_hz` 10 safe again** — the
     09-12 TF-wait tail was executor time, and that is unchanged. Re-measure before raising it.
   - **The GPU governor matters:** `nvhost_podgov` idles the GPU at 306 MHz, and the same chain
     takes 16 ms back-to-back but 31 ms paced at 8 Hz on an idle GPU. Measure with the full stack
     (and note whether `jetson_clocks` is set) rather than in isolation.
   - **Safety net, in the node:** the first 5 frames are built both ways and the numpy cloud is
     flown until they agree; any disagreement, 3 consecutive CUDA errors, or no CUDA at start
     switches to numpy for the life of the node at ERROR. The per-frame log names the backend in
     use.
5. **Next lever if §2.5 is still over budget: BoT-SORT's camera-motion compensation (row 14b).**
   Measured: `track()` costs **41 ms CPU / 60 ms wall** per frame with the default `botsort.yaml`
   (GMC `sparseOptFlow`) against **18.5 ms / 34 ms** under ByteTrack — GMC is ≈ **23 ms CPU and
   25 ms wall**. Options, in order of preference:
   - feed it the rotation from `/lidar_slam/odom` (the drone already knows its own ego-motion);
   - port the optical flow to the GPU with VPI 3.2, which is installed;
   - raise its downscale.

   **Do not just switch to ByteTrack.** GMC is what keeps track IDs through yaw, and on a follower
   an ID switch is a lost lock. Decide from row 16's airborne Stage A data, and keep ID switches
   under yaw at or below today's.

### M5.4 — Sensors (rows 4, 18)
`outdoor_depth_test.py` in sun, shade and under canopy, on the person at 2/4/6 m. Use `--out_dir
~/dtp_datasets/<date>_outdoor_depth`, not the docstring's `/tmp`. Then the motor-on calibration
check. Exits in §2.2.

> **2026-09-13:** procedure in [`m5.4-m5.5-field-runbook.md`](m5.4-m5.5-field-runbook.md).
> - **Outdoor depth:** the test now defaults to `~/dtp_datasets`, and needs the camera launched with
>   `enable_ir:=true`, or its sun/shade split silently does not run.
> - **Motor-on calibration comes from the Stage A flight bag, not a props-on ground capture.**
>   PX4 disarms 10 s after a no-takeoff arming, capture dwells 15 s, and capture's
>   PointCloud2 LiDAR mode starves FAST-LIO.
>   - `drone-dtp/scripts/bag_to_calib_pairs.py` motion-compensates scans into each colour frame.
>   - The unchanged `score_alignment` then scores the pairs.
> - **⚠️ The same tool found that the DEPLOYED calibration no longer fits the rig.**
>   - The deployed `drone_cam_livox.yaml` (solved 09-07) scores **du −12 px (≈ 1.1° bearing)** on
>     the 09-11 8-viewpoint capture, on 09-09 data, and on a 60 s bag taken today.
>   - Its own 09-07 capture still gives du +0.
>   - The **09-11 22:52 solve** fits today within ~2 px.
>   - **Deployed 2026-09-13 16:52.** Today du +2 px, own capture +1/−1, verified live.
>   - **Its translation is physically wrong:** the LiDAR is above the camera (user); the solve says
>     2.3 cm below.
>   - **Kept:** the images fit only its R+t as a pair, and the estimated cost is ≲ ±3 cm in lifted
>     positions over 2–6 m.
>   - **Open:** a near-range re-capture, and motor-on validation.
>   - Details: [`m5.4-m5.5-field-runbook.md`](m5.4-m5.5-field-runbook.md) §4.

### M5.5 — Stage A in the air (row 16)
Observe-only while the aircraft flies its own mission on a healthy estimate (G0.2 preflight green).
`analyze_shadow_goals.py`, exits in §2.2. Today's four Stage A bags do **not** count.

> **2026-09-13:** `analyze_shadow_goals.py` section 6 now scores S1–S4 and exits 0 / 1 / 3.
> - **"Standoff-goal error"** = the distance **outside** the legal band [2.75, 4.0] m, because the
>   lead legitimately spends up to 0.75 m.
> - **The yaw slope** is fitted on target jitter, and refused below 0.3 rad/s of yaw spread.
> - **Stage A preflight changes:** it reads the loaded SUPER config off `/fsm_node`, and refuses
>   the M5.3a filter.
> - **The 09-12 bags** were also all recorded while ARK's logloader was looping (§3 G0.2 note below).
>
> Procedure: [`m5.4-m5.5-field-runbook.md`](m5.4-m5.5-field-runbook.md).

### M5.6 — Stage B: follow flights (rows 19, 20, 22)
Structured sparse venue, pilot on ch11, observer, subject with a stop signal. S1 → S2 → S3, each
passing before the next. Before every flight: `auto_reacquire_after_lost: false` (it is),
`abort_require_offboard: true`, `cs_ev_pos: True`, fault card empty, G0.2 preflight green.

### M5.7 — Safety layer under induced failure (paper Sec. VI-D) (rows 18, 22)
Four inductions: occlude the subject; kill `perception_node`; put a second person inside the
re-acquisition band; stop `/lidar_slam/odom`. **4/4** must give the designed response props-off /
Stage A. In the air, only occlusion and the second person, **3 trials each**.

---

## 5. M6 — loss-cause classification (the paper)

> **Status 2026-09-14 — M6.1, M6.4, M6.5, M6.8 and the M6.2 harness are BUILT and bench-gated;
> nothing has flown. M6.4b (the belief walks round mapped obstacles and looks behind them,
> patch 0005, two ablation switches) was built later the same day — `loss_oracle_check.py` is
> now 56/56 with the O8/O9 tree scenes.** The as-built record, every parameter and every gate result is in
> [`m6-loss-oracle.md`](m6-loss-oracle.md). In short: the oracle lives in `fsm_node` (SUPER patch
> 0004, `dtp_msgs/LossEvent` on `/planning/loss_event`, on in `follow_test_ros2.yaml`);
> `loss_oracle_check.py` 28/28 (classify p99 81 µs, full evaluation p99 406 µs — inside the §2.4
> budgets by 25×); conditioning behind one `loss_conditioning` switch on `target_manager` +
> `follow_planner`, `loss_conditioning_check.py` 16/16 and 27 unit tests; `loss_replay.py`,
> `loss_label.py`, `loss_condition_replay.py` ran on `run5` (48 coast episodes: 7 OCCLUDED,
> 3 UNMAPPED, 38 LOST — the bag's losses are detector glitches, not occlusions; unlabelled).
> Two pre-registered deviations from the table below: LOST ends the coast after the verdict has
> stood **0.3 s**, not immediately (38/48 of run5's coasts are one-frame LOSTs that re-acquire in
> 0.1–0.3 s), and the OCCLUDED coast is never shorter than the carry-forward one. **Later the
> same day:** the closed-loop replay exists; run5 was provisionally labelled and **D1 fails on it
> (E1 recall 0.11)** for a reason that is a precondition, not a bug — the MID360 sees no lower
> than 7° under its horizon, so at 4 m the map cannot hold an occluder below 0.5 m under the
> sensor, and run5's cabinet is one; the oracle now samples only the heights the map can know
> (`fov_floor_deg`). **Add to §5.0 and the paper: an occluder is classifiable only where it
> rises above the sensor's floor at its range.** Still owed: M6.3 (the labelled dataset → D1),
> flights (→ D2), M6.6, M6.7.

### 5.0 Design decisions (these close the TBDs in paper Sec. III and V-E)

**Where it runs: inside `fsm_node`, sharing `map_ptr_`.** A separate node would need its own map.
That doubles the raycasting CPU (`fsm_node` is already 112 %) and makes the architecture figure's
"one map, two consumers" edge false. The oracle gets its own `MutuallyExclusive` callback group,
alongside the four `fsm_ros2` already has.

**Interface** — `dtp_msgs/LossEvent` on `/planning/loss_event`, answering each
`/dtp/follow_target` with `missed_frames > 0`:
- `header` (stamp of the target it answers), `track_id`
- `cls ∈ {OCCLUDED, UNMAPPED, LOST, OUT_OF_VIEW}`, `onset`
- `blocked_fraction`, `unknown_fraction`
- `query_p` (echoes the point tested — the `requested_p` lesson), `occluder_p`, `edge_left`,
  `edge_right`
- `reappear_p[]`, `reappear_dt[]`, `reappear_w[]`, `still_hidden_w`
- `eval_us`

**Classification (Eq. 1, implementable):**
1. Sample the target cylinder at the CTRV point for the current coast age (§2.4).
2. Cast rays from the **camera** optical centre (via TF) — the camera is what lost them.
3. Walk each ray through ROG-Map, **stopping `r_excl` short of the sample**, and label it by the
   first non-free cell: OCC, UNK or free.
4. Return `OUT_OF_VIEW` first if the point is outside the ±35° cone.
5. Return `OCCLUDED` if `ρ_occ ≥ ρ*`, else `UNMAPPED` if `ρ_unk ≥ ρ*`, else `LOST`.
   - UNMAPPED is reported separately but **conditioned as LOST**.
   - A trunk comes out OCCLUDED because the ray hits the trunk before the unknown space behind it.
   - LOST with a clear line of sight includes detector false negatives. That is correct by design;
     say so.

**Reappearance belief:**
1. **Occluder extent.** From `occluder_p`, march left and right perpendicular to the line of sight
   until 3 free columns → `edge_left`, `edge_right`.
2. **Rollouts.** Run K CTRV rollouts. For each, find the first time its point leaves the occluder's
   angular interval with a free 3-D ray → (p, t). Cluster by edge. Mass that never exits is
   `still_hidden_w`. **Added 2026-09-14 (M6.4b, patch 0005, own switches):** the rollouts slide
   round mapped OCCUPIED cells instead of walking through them (`rollout_collision`), and when
   `still_hidden_w` ≥ 0.5 the reduction below uses the hidden rollouts' position at 1.5 s and a
   viewpoint that can see it (`hidden_look`). See `m6-loss-oracle.md` "M6.4b".
3. **Reduction to what SUPER consumes** (the Sec. V-E TBD):
   - **yaw target** = weighted mean of the dominant cluster;
   - **goal** = the standoff candidate on `follow_planner`'s [3, 4] m annulus whose line of sight
     to that point is free, nearest the drone (the oracle scores the candidates, since
     `follow_planner` has no map);
   - **re-acquisition gate** = §2.4 ellipse, open from `t_min − δ` to `t_max + δ`.

**Conditioning** — in `target_manager` + `follow_planner`, behind **one parameter
`loss_conditioning`**, so both ablation arms are the same binary:

| Class | Coast | Re-acquisition gate | Viewpoint / yaw |
|---|---|---|---|
| OCCLUDED | to `min(t_max + δ, 3.0 s)` | single confirmed track **inside the region during its window**; ABSTAIN unchanged | pre-position for the edge |
| LOST / UNMAPPED | **end immediately** | none — LOST latches, operator re-designates | mux HOLD + 0.75 m climb (= "search") |
| OUT_OF_VIEW | today's coast (usable 1.30 s) | today's band (1.0 m + 1.0 m/s, cap 2.5 m) | yaw to the predicted point |
| `loss_conditioning:=false` | today's behaviour for every loss | today's band | today's — **the carry-forward arm** |

### M6.1 — Loss oracle (row 17)
`LossEvent.msg`, the oracle, and unit tests on a synthetic ROG-Map: a 0.3 m cylinder occluder, a
known unknown region, and a trail of occupied cells behind a walker. Timing → §2.4 budgets and paper
Table III's "Loss classifier" row. Lands in the SUPER patch.

### M6.2 — Replay harness (row 21)
The replay pipeline:
- Republish `/cloud_registered_body` into `lidar_init` using `/lidar_slam/odom`, or re-run FAST-LIO
  where the bag has `/livox/lidar`.
- Feed `fsm_node` with the oracle → `LossEvent`s.
- Add a labelling tool: colour frames ±2 s around each coast onset, one keypress per event.
- Replay `/dtp/targets` + `LossEvent` through `target_manager` with conditioning on and off.

First inputs: `20260909_closedloop_run5` and the Stage A bags.

**Replay is not deterministic, and D1/D2 depend on replay — measured 2026-09-13 while verifying
the GPU cloud** (`drone-dtp/scripts/replay_perception.sh`,
`~/dtp_datasets/20260913_cloud_backend_replay/`):
- **Schedule.** At 1× two runs of the same code processed different frames (only ~⅓ shared
  stamps), differing p95 2.6–12 cm. The first YOLO inference's ~2.5 s TensorRT warm-up stall
  shifts the decimator's phase. **Fixed by `WARM_S=60 RATE=0.25`:** three runs, the same 575
  stamps.
- **Inputs.** Even on that identical schedule, numpy vs numpy first differed at frame 77, then
  p95 1.7 cm / max 7.5 cm. The likely cause — unproven — is `perception_node` pairing each colour
  frame with "the newest depth frame", which depends on executor order.

So for M6.2:
- **Bypass the node's subscriptions.** Drive perception from the bag in-process, pairing colour
  and depth by stamp, the same way `scripts/bench_cloud_backend.py` reads frames.
- **Or run every arm at least twice**, and report the between-arm difference against the
  within-arm one.

A classifier effect smaller than ~2 cm-equivalent of position noise cannot be claimed from a
node-level replay.

### M6.3 — Handheld occlusion dataset → D1 (rows 13, 23)
**The course.**
- **8 tubes, 30 cm diameter, ≥ 1.8 m tall**, in two staggered rows 2.5 m apart, 2.0 m between
  tubes, in a geometry-rich venue.
- Record every tube position to ±5 cm.

**The recording.**
- Carry the powered, disarmed rig at ~1.5 m while the subject walks scripted events.
- **Start FAST-LIO with the rig ON THE GROUND, then lift it** (found 2026-09-14 on run5): ROG-Map's
  virtual ground sits 0.5 m below `lidar_init`'s origin, and everything under it reads OCCUPIED.
  A rig started at 1.5 m puts the subject's waist a metre below that plane; the oracle then
  drops every sample height and classifies on one ray at the plane — head height — where a
  trunk-sized occluder no longer blocks (run5: 32 of 38 labelled occlusions read LOST). `fsm_node`
  now warns loudly when this happens (`target z … is BELOW the map's virtual ground`). The
  alternative is a handheld config with `rog_map/virtual_ground_height: -2.0`; the replay tools
  take `--virtual-ground` for bags already recorded the wrong way.
- Call each event aloud and film from a tripod, synced by a clap.
- Then **one segment among real trees, leaves on**.

| Event | Ground truth | n | Checks |
|---|---|---|---|
| E1 walk behind a tube | OCCLUDED | 60 | recall, reappearance edge + time + position |
| E2 walk away > 10 m in clear view | LOST | 40 | **trail false-OCCLUDED** |
| E3 leave the camera cone | OUT_OF_VIEW | 20 | cone test |
| E4 crouch / turn in clear view | LOST (detector FN) | 20 | clear-LoS behaviour |

**Outputs:**
- Confusion matrix.
- ρ\*/N/`r_excl` sweeps (**ablation 4**).
- Reappearance point-vs-region (**ablation 3**, offline).
- Open-loop region-gate vs band.
- Filter on/off false-OCCLUDED.

**D1 thresholds:** §2.2. Fail → fix the classifier before any M6 flight.

### M6.4 — Reappearance belief (row 25). Exit: §2.2.
Built 2026-09-14 (0004). **M6.4b the same day (0005): obstacle-aware rollouts (B) and the
look-behind reduction (B2)**, each its own ablation switch, `loss_oracle_check.py` 56/56, the
switches-off belief identical to 0004's in 36/36 events; plan and deviations in
[`m6-belief-around-trees-plan.md`](m6-belief-around-trees-plan.md). The §2.2 exit (E1 edge side,
timing, region) still needs the M6.3 dataset.
### M6.5 — Conditioning → D2 (row 26)
If D2 fails, per the paper README this becomes an **RA-L systems paper**. Decide on **Nov 10**,
before the course field days.

### M6.6 — Offline predictor study (row 24)
**Done offline 2026-09-14 with PUBLIC checkpoints, not the flown model — see
[`m6.6-predictor-study.md`](m6.6-predictor-study.md)** (table, protocol, caveats; the α schedule
below was measured and is too gentle to matter; the best constant α falls from 0.8 to 0 across
0–50 % missing under the repo's forward-fill).
TrajImpute protocol on ETH/UCY, comparing CTRV, EigenTrajectory and fused, with complete histories
and with **missing rates of 10 %, 30 % and 50 %** → Table II. Needs the EigenTrajectory checkpoint
(on "the AGX") or retraining, and a GPU desktop. Include a missed-frame-dependent α schedule —
offline only.

### M6.7 — Flight evaluation (rows 29, 30)
- **Protocol.** Course first, then forest. Same scripted events, flown at S3, arms ABBA by flight.
  Sample sizes per §2.3.
- **Metrics from bags:**
  - re-acquisition time (COASTING onset → LOCKED) — **the primary endpoint**;
  - loss duration and visibility rate;
  - bearing error (camera axis from TF vs target);
  - interventions (ch11 exits in `vehicle_control_mode`, with the pilot's stated cause);
  - classification accuracy vs video labels.
- **Forest record.** Trunk diameter and spacing along the route (≥ 20 trunks measured).

### M6.8 — Baselines (row 27)
- **Carry-forward** = `loss_conditioning:=false`.
- **SUPER stock** = a `follow_planner` mode: a goal behind the current position, CV model, no coast
  or ABSTAIN.
- **Eva-Tracker:** not flown (§1 row 5). **Its loss policy is re-implemented as `baseline_mode: eva`**
  (row 27a in [`eva-tracker-faster-lio-assessment.md`](eva-tracker-faster-lio-assessment.md) §3, 2026-09-14):
  Bézier-QP prediction, 2.5 m standoff viewpoint from the *predicted* point, arc search for a free
  line of sight, no verdict. Faster-LIO is a contingency there too (rows 14a–14c), triggered only
  by a FAIL on the moving-rig latency check.

---

## 6. Risks and fallbacks

| Risk | Signal (date) | Fallback |
|---|---|---|
| ~~RX-DMA change doesn't stop the fault~~ — it didn't (09-13). **Client-side fix doesn't hold** | a reset or dump in the ≥ 500 cycles or the 14 h soak (row 6a) | guard `uart_close` against `uart_datareceived` in NuttX's `serial.c` (+1 week); **no autonomous flight until a fix holds** |
| FAST-LIO diverges under the full stack | G0.2a (Sep 15) / 20-min soak (Oct 7) | `extrinsic_est_en: false`, pinning, `detect_rate_hz` down; the G0.2 guard makes it fail safe regardless |
| Outdoor camera depth fails | M5.4a (Sep 16) | LiDAR depth path (13.5 % FOV coverage, recalibration per remount); the classifier is unaffected — it uses the map |
| Trail false-OCCLUDED persists with the filter | D1 (Oct 27) | count only cells occupied before the target passed; widen `r_excl` |
| Ablation 1 doesn't separate | D2 (Nov 10) | RA-L systems paper: classifier accuracy + safety layer + integration lessons |
| S3 isn't stable | M5 gate (Oct 23) | evaluate at S2 with the subject at 0.7 m/s; report it |
| Weather eats field days | any | 2 buffer days are built in; beyond that every row after 29 slips day-for-day |
| GPU cloud misbehaves on the aircraft (driver, CUDA, memory) | a `DISABLED` / `cannot start` line in `perception_node`'s log (row 14) | the node already reverts to numpy by itself; the cost is ~1.2 cores back, so re-check FAST-LIO's rate before flying, or set `cloud_backend: numpy` deliberately |

## 7. Stale facts this plan found

The docs below were corrected in the same change as this plan:

- The `uxrce_dds_client` fault count was **11**; it is **20**, with nine new logs untracked.
- `docs/dtp-follower-stack.md` said `detect_rate_hz` 10.0 and `auto_reacquire_after_lost: true`;
  the committed `dtp_params.yaml` has **8.0** and **false**.
- `docs/paper/README.md` said the follower is "not in this repository" and that raycasting is off
  in the follow config. Both changed on 2026-09-12.
- `~/dtp_datasets/README.md` had no entry for today's four Stage A bags.
- **(2026-09-13)** `docs/dtp-follower-stack.md` said `perception_node`'s ~250 % CPU was "dominated
  by JPEG decode and the numpy organized-cloud build". Decode is 5 % of a core; the cloud build was
  90 % and BoT-SORT's GMC ~18 %. It also said the tracker's cost was unmeasured; it is measured
  now (§2.5).
