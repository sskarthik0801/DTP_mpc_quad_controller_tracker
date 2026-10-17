# Handoff to Kevin — the DTP follower stack, 2026-09-14

**From Krishna.** From today I am on the paper and how we present the results; you own tests,
deployment and flying. This is everything we were planning and where each piece stands, so you can
check it against your own plan and correct anything I have wrong. Reply with edits on this file.

I keep: the analysis tooling (scorer, timelines), the results write-ups, the paper and its figures,
and I will ask you for specific data (section 8). I will keep committing to `drone-dtp` under my
own name for those things; everything that flies is yours to change.

## 1. Where things live

| what | where |
|---|---|
| implementation, the thing that flies | `kleverin/SUPER-Human-DTP`, branch **`drone-dtp`** — both of us commit there; the NX checkout is on it (`~/super_ws/src/drone-dtp` symlinks into it). The follower is `drone-dtp/`. |
| deploy | commit → push → `git pull` on the NX. Python is live through symlink-install; `setup.py`, msg or CMake changes need `colcon build`. No more `scp` into the checkout (that is how a stray `sensor_watchdog.py` got there). |
| planning docs, worklog, results | `john-2424/drone-dtp`, branch `dtp-imp`, folder `plan/dtp-imp/` (not tracked in git — I will hand you files as needed; the ones you need are in `docs/` here). |
| the paper | `john-2424/drone-dtp` `paper/` is the canonical draft. Your `docs/paper/main.tex` (Sec. V-E from the B/B2 build) I will merge into it — please keep writing there for now and tell me when a section is stable. |
| flight results 09-13 | `docs/stage-a-flight-results-2026-09-13.md` (copied here with this handoff), raw reports next to each bag on the NX. |
| runbooks | yours: `docs/flight-day-runbook.md` (the day's master). Mine: the Stage A checklist artifact (link in the worklog); the two agree, see section 6. |

## 2. What the stack is, as of today

Perception → target_manager → follow_planner → command_mux → SUPER (fsm_node via patches 0001–0005) → px4_offboard_bridge → PX4. FAST-LIO on the Mid-360 is the estimator (`/lidar_slam/odom`, frames `lidar_init` / `lidar_imu`), also PX4's external-vision source through `px4_odom_bridge`.

- **perception_node**: YOLO11s TensorRT + BoT-SORT at `detect_rate_hz 8` (10 Hz starved the TF listener under your FAST-LIO), organized cloud on CUDA (`cloud_backend: cuda`, self-checked 5 frames vs numpy then trusted; 1.9 → 0.7 cores), stamp-exact TF lift (`tf_at_latest false`), calibration = the 2026-09-11 22:52 solve (translation physically wrong by your note, kept, ≲ ±3 cm). Publishes `/dtp/targets`; `/dtp/detections_2d` only while your laptop overlay subscribes.
- **target_manager**: acquisition 2.5–10 m, > 3 m from the launch point, 3 confirm frames, 0.4 m of motion in 6 s; **coast floor** `min_coast_horizon_s 1.2` (a fresh lock survives a dropped frame — flew 09-13); re-acquisition band during coast; `auto_reacquire_after_lost` **false** (D13, see section 5); loss-oracle consumer present, `loss_conditioning` **false**.
- **follow_planner**: standoff band 3–4 m, hard floor 2.5 m enforced horizontally against predicted *and* present position, lead budget, z clamp 0.5–3.0 m. Never emitted a goal inside the floor on either flight.
- **command_mux**: proximity veto 1.5 m, HOLD-at-position with no source, NAV_LAND abort (`abort_enable` false in observe-only), and your **geofence** (default ON, 30 × 4 m along the boot heading, goals clamped 0.5 m inside, > 3 m outside → FENCE veto → NAV_LAND). Never flown.
- **sensor_watchdog, dtp_viz, dtp_image_overlay**: as you left them.
- **SUPER**: patch 0001 (yaw-to-target + follow goal, M5.2 envelope max_vel 0.5 / max_acc 0.5 / max_omg 1.5 / yaw_dot_max 1.0, `robot_r 0.30`), 0002 target exclusion (default off; Stage A gate FAILs if on), 0003 ray-clip fix, 0004 loss oracle (publishes `/planning/loss_event`, acts on nothing), 0005 belief around obstacles (switches on in `follow_test_ros2.yaml`, consumed by nothing while `loss_conditioning` is false).
- **Defaults that matter** (`dtp_params.yaml`): `detect_rate_hz 8`, `cloud_backend cuda`, `auto_reacquire_after_lost false`, `loss_conditioning false`, `fence_enable true`, `min_coast_horizon_s 1.2` (node default).

## 3. Tooling — what to run

All on the NX, from `~/super_ws/src/drone-dtp`. `ssh ideas-ad@100.124.49.34` (Tailscale) from anywhere.

| command | what it does |
|---|---|
| `bash scripts/field.sh check` | 10 s: refuses if anyone but `fsm_node`/`laser_mapping` subscribes to `/cloud_registered` or `/livox/lidar` (a laptop RViz), and measures FAST-LIO **stamp** lag (p50/max over 4 s; limit 0.3 s). |
| `bash scripts/field.sh prep` | clocks, wifi, daemon → waits for your odom ≥ 9.5 Hz at the origin → `check` (allows a start-up backlog to drain) → clears anything stale of ours → launches ours observe-only (`indoor_test.sh --keep --no-walk`) → `stage_a_preflight` → launch point and `sensors_ok`. Kills any instance of ours it did not start — **we agreed `field.sh` is the only starter on flight days**. |
| `bash scripts/field.sh arm` | recorder (by PID) → engage → waits for LOCKED → `check` again → pre-arm gate → "GO — Kevin may arm". Re-runnable; restarts the recorder. |
| `bash scripts/field.sh watch` | one line per second: lock, mux, alt, stamp lag, target range, armed. "DROPPED" only for a disarm < 8 s after arming that never left the ground. |
| `bash scripts/field.sh reengage` | after a LOST with auto-reacquire off. |
| `bash scripts/field.sh land` | recorder SIGINT by PID (metadata written), ours stopped, bag printed. Before the battery comes out. |
| `python3 scripts/stage_a_preflight.py [--require-record] [--stage-b --target person\|mannequin --max-vel X --waive-d21]` | the gate. Stage A: authority (nothing of ours on the control topics, shadow topics carrying goals), abort disabled, target exclusion off, `auto_reacquire` WARN; Stage B: mux on `/planning/follow_goal`, abort ENABLED, max_vel read off the running `fsm_node` and checked against `--max-vel`, fence box printed, **D21 needs `--waive-d21`** (my decision, logged), `auto_reacquire` and `loss_conditioning` FAIL if on. Both: FAST-LIO stamp lag < 0.3 s, no foreign subscribers, rates, one publisher per sensor topic, extrinsic verified. |
| `python3 scripts/analyze_shadow_goals.py <bag> [--window airborne\|armed\|all]` | your M5.5 scorer, now windowed on **airborne** by default (odom z > 0.3 m, runs ≥ 10 s); sections 1/2/4 on FOLLOW goals only; HOLD goals reported as person-to-aircraft distance; CSV keeps every row. |
| `python3 scripts/flight_timeline.py <bag>` | arm epochs, airborne time, lock states and runs, stamp-lag stalls, detection vs range, light vs detection per minute, mux while airborne. |
| `bash scripts/indoor_test.sh --live --keep --no-walk` | the live launch (types LIVE). S1 only. |
| `bash scripts/recalib.sh which-tier <event>` / `capture` / `solve` / `deploy` | calibration dispatcher. |

## 4. What was flown and measured (2026-09-13, your place, light 92–101/255)

| | hop 1 (bag 1919) | hop 2 (bag 1955, auto-reacquire on) |
|---|---|---|
| armed / airborne | 216 s / 206 s at ~1.0 m | 103 s / 94 s at ~1.5 m |
| LOCKED while airborne | 8.5 % (LOST at 24 s, nobody re-designated) | **80.1 %**, 3 unaided recoveries in 5–8 s |
| mux FOLLOW\|OK | 8.2 % | 76.5 % |
| detector, frames with a person, 4–6 m / 8–13 m | 99.6 % / ~80 % | 97.9 % / ~84 % |
| FAST-LIO stamp lag airborne p95 / max | 0.115 / 0.97 s | 0.072 / 0.107 s |
| S1 standoff error p95 · S2 jitter p95 · S3 slope · S4 blind | 0.000 m · 0.019 m · n/a (no yaw) · 0.2 % | 0.000 m · 0.061 m · **−0.022 m per rad/s** · 0.0 % |
| **M5.5** | INCOMPLETE (S3) | **PASS** (airborne window) |

S3 is the first in-flight evidence for the stamp-exact lift (A8). Every lock loss was the walker leaving the 4–6 m band. Hop 2's 22 goal steps > 1 m are the trail azimuth swinging round the walker at each turn of the left–right line — real for S1 (a ~2 × standoff replan per reversal), hence "straight line, one turn". Full numbers: `docs/stage-a-flight-results-2026-09-13.md`.

## 5. Decisions and why (so you can overrule knowingly)

- **D13 — `auto_reacquire_after_lost` false in any live mode.** After a LOST the node cannot know the person who reappears is the same person; auto-reacquire locks the nearest moving human. On 09-13 it locked "track 4 at 2.53 m" on whoever walked up to the parked aircraft the moment it was switched on. True for Stage A only (set live after launch), off for M5.1 and S1 until re-ID exists.
- **D21 — single-IMU FC.** The LiDAR IMU feeds FAST-LIO → EKF2 as external vision, not a second IMU in the attitude loop. I decided to fly S1 on the ARK FPV with the waiver logged by `--waive-d21`; the ARKV6X remains the proper fix.
- **Geofence = option 1** (goal clamp + land-in-place in `command_mux`), because PX4's own fence is inert without a global home. Never flown; the box is the boot heading — aircraft nose down the corridor before Start.
- **`loss_conditioning` false** until the M6.5 ablation is run deliberately; the oracle publishes, nobody consumes.
- **`detect_rate_hz` 8**, `cloud_backend cuda`, `robot_r 0.30` (your measurement), `min_coast_horizon_s 1.2`.
- **Calibration**: the 09-11 solve stays deployed (du +1 / dv −1 px on 8 viewpoints; translation physically wrong, cost ≲ ±3 cm). Proper fix = re-capture with structure at 1–2 m, `recalib.sh capture`.
- **One starter**: `field.sh`. The dashboard DTP row is read-only on flight days.

## 6. The rules the two flight days cost us

1. **Clocks before your Start.** A FAST-LIO started at boot clocks ran 27 s behind for 15 min.
2. **No laptop RViz on `/cloud_registered` over wifi while FAST-LIO runs.** 38 s of stamp lag at 10 Hz, twice; PX4 fed stale poses. Your FastDDS non-blocking profile fixes the mechanism; the gates stay.
3. **Rate proves nothing; stamps do.** `field.sh check` and the preflight now gate on stamp lag.
4. **Ours after yours, always.** If your stack restarts, ours relaunches (`prep`).
5. **Walker 4–6 m, inside the fence box (±1.5 m of the nose line for Stage A), never walking away.** In S1 the aircraft keeps the range.
6. **Nose down the corridor before Start**; nobody moves the aircraft afterwards (3.7 km on 09-12).
7. **Recorder stopped by SIGINT by PID only** (`land` does it); `pkill -f` has killed sessions and bags.
8. **Dusk is a different system**: 9 % detection at 6.4 m at 52/255. End flights 20 min before sunset.

Four edits to `flight-day-runbook.md` we agreed tonight: `--waive-d21` on steps 21 and 29; drop step 12 (`arm` records); `auto_reacquire` true for sortie 1 only; hop 2 was a sane estimate, so if it flew in Position on the EV pose, P3 is done.

## 7. The plan you now own

In your order: **G0.3/P3** (Position hover on the EV pose, `estimator_preflight` GO, no resets) → **M5.1** props-off live, 8 trials → **M5.4** depth in sun (2 min in any hover) → **S1** at 0.5 m/s on `follow_test_ros2.yaml`, walker straight down the corridor with one turn, then the trunk occlusion for the M6 events. Score each sortie before the next (`analyze_shadow_goals.py`, `m51_one_graph_check.py`, `flight_timeline.py`); keep the reports next to the bag; copy bags off the NX the same day (the 09-13 bags are being copied to my machine over Tailscale now).

Things I did not get to that you may want: `estimator_preflight.py` should check odom stamp age; `field.sh status` exit code; the fence's `fence_y_*` for a wide Stage A walk (we chose to walk narrow instead); a moving-rig latency check.

## 8. What the paper needs from the flights (my asks)

Per sortie, unchanged: the `stage_a_record` topic set (plus `/planning/loss_event` and the camera for S1), the scorer report, `flight_timeline.txt`, and a two-line note (walker protocol, light, wind, what went wrong). Bag names `<date>_<hhmm>_<stage>` as now.

1. **S1 following**: standoff over time, lock % airborne, goal-step statistics, aircraft speed profile, one full corridor leg each way — the headline result.
2. **Occlusion and reappearance** (M6): walker behind a trunk and out the other side, several times, with `loss_event` recorded; your `loss_label.py` sheets. With `loss_conditioning` off first (baseline), then on (ablation) if the day allows.
3. **Yaw-rate sweep** for the S3 figure: more of what hop 2 gave (±90° at 0.2 / 0.5 / 0.8 rad/s, walker at 4–6 m). One more sortie is plenty.
4. **Detection vs range vs light**: a static 3 / 5 / 7 / 9 m test, three light levels (noon, late afternoon, dusk), 60 s each — this is the envelope figure. No flight needed.
5. **M5.4 depth in sun**: `outdoor_depth_test.py`, standoff-band % in direct sun.
6. **Compute budget**: one `top`/`tegrastats` capture with both stacks up, and the perception frame log (infer / cloud / stamp age) — I have 09-13's numbers, a second day confirms them.
7. **Failure catalogue**: anything that grounded or aborted a flight, one line each, with the bag time.

## 9. Known limitations (going into the paper as such)

Daylight detection envelope ~8 m, dusk 3–6 m; stereo depth on dark clothing thin at 3 m; calibration translation ±3 cm; no re-ID (D13); trail-azimuth swing at walker reversals; single-IMU FC; fence and loss oracle never flown; `extrinsic_est_en` true in flight; walker-side range keeping in Stage A; FAST-LIO stamp lag under any wifi cloud subscriber.

## 10. Small debt

`.gitattributes` for `.msg/.cpp/.hpp/CMakeLists.txt` (git archive made CRLF); flake8 E501 in the scripts; `capture_calib_data` writing `identity.yaml`; MultiThreadedExecutor for perception; `detection_pipeline/` is meant to stay unchanged (your `EigenTrajectory/eigen.py` is fine as an offline addition).

Thanks for the last three days. The stack is in a better place than it was on Friday, and most of that is yours.
