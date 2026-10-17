# FAST-LIO diverged 3.74 km and the aircraft was armed on it — root cause (G0.2)

**Written 2026-09-12/13. Closes G0.2 of [`m5-m6-plan.md`](m5-m6-plan.md).** Evidence:
`~/dtp_datasets/20260912_2006_stageA` (the bag), `~/.ros/log` (every node's log from that
evening), a bench reproduction and a hardware end-to-end test of the fix, both run the same
night. Raw outputs of the tests are in [`evidence/g02-2026-09-12/`](evidence/g02-2026-09-12/).

## TL;DR

- **The error was 3.74 km, not 86 m.** At the start of the bag FAST-LIO reported
  **x −3340.1, y −1672.2, z −86.7 m** on an aircraft sitting at its takeoff spot. Earlier notes
  quoted only z.
- **Cause: a second Livox driver.** At 20:00:31, `dtp_system.launch.py` was started on top of the
  already-running `run.sh` stack. It launched a second `livox_ros_driver2_node`, which took the
  MID360's stream away from the first one. It published `/livox/lidar` as **PointCloud2** (that
  launch file defaulted `xfer_format:=0`), a type FAST-LIO's CustomMsg subscription cannot receive.
  It also published `/livox/imu` as the *same* type FAST-LIO already used.
  - **What FAST-LIO saw:** IMU but no scans, for 225 s.
  - **What it logged:** nothing. With no scans it publishes no odometry and prints no warning.
  - **When the DTP launch was stopped at 20:04:16:** the scans came back. FAST-LIO integrated the
    whole IMU backlog with no LiDAR correction, came back kilometres off, then scan-matched
    around the wrong pose with millimetre covariance.
- **Why nothing caught it:** an estimate that diverged and then settled looks healthy by every
  flag.
  - `px4_odom_bridge` forwarded all of it (`in=8285 out=8285 rejected=0`, running since 19:59:32).
  - EKF2 said `xy_valid`, `z_valid`, `eph 0.06 m`. With GPS off it has nothing absolute to compare
    against, because the vision frame *is* its frame.
  - The aircraft was armed twice on it.
- **Reproduced on demand, on the bench:** a second driver with `xfer_format:=0` for 120 s, then
  killed, put a stationary rig at **(−752, +1957, −62) m**. 90 s later it read
  (−3393, −975, −4561) m, with a position variance still around 6e-6 m².
- **`extrinsic_est_en` (the plan's lead) is not the trigger.** It has been `true` in
  `config/mid360.yaml` since the repo's initial commit, through every gate. The bench
  reproduction diverged with it on, so it is neither necessary nor ruled out as an aggravator.
- **Fixed three ways; each layer tested on hardware:**
  1. The trigger is refused. `run.sh` and the DTP launch files will not start a second driver,
     FAST-LIO or `fsm_node`.
  2. The bridge latches instead of forwarding a jumped, stalled or far-from-origin estimate.
  3. A 60 s estimator preflight refuses to arm.

## Timeline, 2026-09-12 (EDT)

| Time | Event | Source |
|---|---|---|
| 19:59:32 | `px4_odom_bridge` and `px4_offboard_bridge` start | `python3_3482…`, `python3_3510…` |
| 19:59:33 | `run.sh`: Livox driver (pid 3520, CustomMsg) | launch `…19-59-32…-3496` |
| 19:59:37 | FAST-LIO (3571), `fsm_node`; map initialised 19:59:40 | `fastlio_mapping_3571…` |
| **20:00:31** | **`dtp_system.launch.py` on top**: `dtp_livox_driver` (4148, **PointCloud2**), a second FAST-LIO (4152), a second `fsm_node`, camera, perception | launch `…20-00-31…-4125` |
| 20:00:32 | Second FAST-LIO dies, exit −11. Its default config `super_bringup/config/mid360.yaml` does not exist, so it started with no parameters | same |
| 20:00:54 | DTP `sensor_watchdog`: `/lidar_slam/odom: NEVER PUBLISHED`. `static_calib_tf`: TF chain broken, still broken at 20:04:01. **FAST-LIO is silent: no scans** | `python3_4204…`, `python3_4184…` |
| 20:04:16 | DTP launch stopped. `dtp_livox_driver` segfaults on exit (−11) | launch 4125 |
| 20:04:16–:34 | FAST-LIO: ~590 × `No point, skip this scan!` (a burst of near-empty frames). IMU is propagated on every one | `fastlio_mapping_3571…` |
| 20:04:29–:45 | `command_mux` sees the vertical position swing between 17 and 56 m from ground, settling at ~87 m | `python3_5921…` |
| 20:06:59 | Bag starts. Odom **(−3340.11, −1672.20, −86.66)**. EKF2 NED (−3340.1, +1672.2, +86.7), valid, eph 0.06 | bag |
| 20:07:50 / 20:11:06 | **Armed** (bag 51.2 s, Position mode 63–172 s; bag 246.6 s, Position 256–266 s) | bag, `vehicle_status_v1` |
| 20:10:39–20:12:54 | A second, separate problem: FAST-LIO odometry **age** climbs to median 100–311 ms, p95 up to 571 ms. 17 EKF2 horizontal resets (counter 9→26), one 0.4 s excursion to z 641 m | bag |
| 20:15:38 | EKF2 counters restart (FC reboot or `ekf2` restart; **no Jetson log shows which**) | bag |
| 20:15:42 | Stack stopped; FAST-LIO needs SIGKILL | launch 3570 |
| 20:16:02 | FAST-LIO restarted; odometry back at (0,0,0) at bag 543.9 s; EKF2 resets onto it at 546.7 s | bag |

## Mechanism

1. **One MID360, two drivers.** The later driver takes the stream. Measured on the bench
   (repro 2 below):
   - The first driver's CustomMsg dropped to **0 Hz** within a second.
   - `/livox/imu` stayed at **200 Hz** with 2 publishers, but with no duplicated or out-of-order
     stamps, so only one driver was receiving.
   - The socket-level mechanism (Livox SDK2 host binding) was not investigated. The behaviour is
     what matters, and it reproduced every time.
2. **The type split starves FAST-LIO silently.**
   - `mid360.yaml` has `lidar_type: 1`, so FAST-LIO subscribes to CustomMsg. A PointCloud2
     publisher on the same topic never matches.
   - `sync_packages()` returns `false` on an empty lidar buffer (`laserMapping.cpp:388`), so there
     is no odometry, no TF and no log line.
   - `/livox/imu` has the same type from either driver, so the IMU buffer keeps filling.
3. **The driver's exit hands the stream back, and the backlog is integrated blind.** A burst of
   near-empty frames arrives: 12 s of `No point` on the bench, 18 s on 09-12.
   - `p_imu->Process()` runs **before** the empty-scan `return` (`laserMapping.cpp:988–996`), so
     every skipped frame still propagates the state through its IMU samples.
   - A residual of roughly 0.1–0.3 m/s² (1–2° of tilt or bias) integrated twice over minutes is
     kilometres.
4. **It then looks healthy.** Scan matching resumes around the wrong pose, `map_incremental`
   builds a new map there, and the covariance collapses to millimetres.
   - On 09-12 it settled.
   - On the bench it kept running away.
   - Either way nothing downstream can distinguish it from the truth.

**Why the duplicate driver's topic type matters — the control run.** Repro 1 started the second
driver with `xfer_format:=1` (CustomMsg, the `sensors.launch.py` default).
- FAST-LIO kept receiving full frames and odometry stayed within 3 cm.
- Killing that driver still produced a 12 s `No point` burst, but no divergence, because FAST-LIO
  had not been starved beforehand.
- **The starvation needs the PointCloud2 type**, which is what `dtp_system.launch.py` defaulted to.

## Bench reproduction (no PX4 bridge running, FC not involved)

`scripts/run.sh` stack, a 1 Hz recorder (raw subscriptions from repro 2 on), then
`ros2 launch dtp_bringup sensors.launch.py camera:=false xfer_format:=<0|1>` for 120 s, then
SIGINT. The rig did not move.

| | Second driver | During | After SIGINT |
|---|---|---|---|
| repro 1 (control) | CustomMsg | FAST-LIO fed, odom steady | 12 s `No point` burst; drift < 3 cm |
| **repro 2** | **PointCloud2** | **CustomMsg 0 Hz, IMU 200 Hz, odom 0 Hz** | **first pose (−752.6, +1956.8, −62.0) m; (−3393, −975, −4561) m 90 s later; cov ≈ 6e-6 m²** |

The recorder's 1 Hz CSV is `evidence/g02-2026-09-12/repro2_recorder.csv`.

## What was fixed

| Layer | Change | File |
|---|---|---|
| Trigger | Refuses to start if `livox_ros_driver2_node`, `fastlio_mapping` or `fsm_node` is already running (argv[0] match, never `pgrep -f`) | `scripts/run.sh` |
| Trigger | `_livox_driver` refuses a second driver. The docstring used to **recommend** a second driver for PointCloud2 and now says why not | `drone-dtp/…/dtp_bringup/launch/sensors.launch.py` |
| Trigger | Refuses a second FAST-LIO / `fsm_node`. Fails with a sentence instead of a SIGSEGV when the FAST-LIO config is missing | `…/launch/super.launch.py` |
| Trigger | `xfer_format` default **0 → 1**, matching `sensors.launch.py` and `lidar_type: 1` | `…/launch/dtp_system.launch.py` |
| Trigger | `running_pids`, `duplicate_refusal` + tests (a shell that mentions the name, or a `ros2 run` wrapper, does not count) | `…/dtp_bringup/preflight.py`, `…/test/test_preflight_duplicates.py` |
| Bridge | **Divergence guard** (below); `reset_counter` no longer hardcoded 0; state on `/px4_odom_bridge/status` | `super_bringup/scripts/px4_odom_bridge.py` (deployed to `~/super_ws/src/super_bringup/scripts/`) |
| Bridge | T8–T14 sandbox the odom bridge. Gate is now **32/32**. T7 hardened (flaked 1 run in 5) | `scripts/px4/m3_bridge_check.py` |
| Arming | **`estimator_preflight.py`**, E1–E8. Also inside `stage_a_preflight.py`, and a dashboard check | `scripts/px4/estimator_preflight.py`, `drone-dtp/scripts/stage_a_preflight.py`, `scripts/gcs/supervisor.py` |
| Procedure | Preflight before every arm; never a second driver; restart FAST-LIO at the takeoff spot | `docs/flight-test-plan.md` §1a |

### The bridge guard

The bridge forwards a sample only while TRACKING. It latches **DIVERGED**, publishes nothing,
and EKF2 times vision out on the first of these:

- **A step** larger than `max(max_step_m 0.5, max_speed_mps 5.0 × dt)`, with dt taken from the
  stamps.
- **A stamp gap** longer than `max_gap_s` 1.0. A stamp gap is exactly an interval FAST-LIO
  integrated IMU alone.
  - **Arrival gaps are deliberately not a trigger.** The first version used them, and replaying
    the recorded bags showed it would have latched two healthy 09-09 runs. Those had 2.1 s
    arrival gaps whose stamps were 0.1–0.4 s apart: every scan was matched, and the output was
    merely bunched under load.
- **A stamp that does not advance.**
- **A first sample** farther than `max_start_radius_m` 1.0 from the origin.

**It never un-latches on the same FAST-LIO.** The 09-12 estimate *settled*, so "consistent again"
would have re-trusted it. It resumes only for a **new publisher GID** whose first sample is near
the origin, i.e. a restarted FAST-LIO, and then increments `reset_counter` so EKF2 resets onto the
new frame (`ev_control.cpp:52`). Restarting only the bridge far from the origin needs
`max_start_radius_m:=-1.0`, deliberately.

### The preflight (`python3 scripts/px4/estimator_preflight.py`, 60 s, exit 0 = GO)

| | Refuses when | Would it have refused 09-12? |
|---|---|---|
| E1 | ≠ 1 publisher on `/livox/lidar` (and not CustomMsg), `/livox/imu`, `/lidar_slam/odom`; ≠ 1 driver / FAST-LIO process | during 20:00–20:04 yes; after, no |
| E2 | the Livox driver started after FAST-LIO | no (the surviving driver was the older one) |
| E3 | < 9.5 Hz in any 10 s, or a stamp gap > 1.0 s | no (10 Hz by then) |
| E4 | odometry age p95 > 150 ms (`EKF2_DELAY_MAX` is 200 ms) | **yes, 180–350 s** (the reset storm) |
| **E5** | **|odom| > 1.0 m** | **yes: 3736 m** |
| E6 | EKF2 invalid / dead-reckoning, or |EKF2 − FRD(odom)| > 0.20 m | no (EKF2 followed the wrong EV exactly) |
| E7 | any EKF2 reset counter change in the window | **yes, 220–355 s** |
| E8 | bridge guard not TRACKING | yes, with the new bridge |

## Verification

**Offline replay** of every `/lidar_slam/odom` stream on disk (7 bags) through the guard:
- Healthy runs never latch, including the bunched 09-09 runs.
- `20260912_2006` is refused from its first sample and resumes at the FAST-LIO restart.
- `20260910_flightlegal_bench` and `20260912_1947` are refused at "first sample 2.0 / 6.1 m from
  origin". That is correct as a replay: those bags start after the rig had been carried. Live,
  the bridge would have started with the stack, near the origin.

**Hardware end-to-end** (real FC, disarmed, no mode change; outputs in
`evidence/g02-2026-09-12/`):

| Phase | Result |
|---|---|
| 1. Healthy stack + guarded bridge, preflight 60 s | **GO, 14/14**: 10.0 Hz, stamp gap 0.10 s, age p95 57 ms, |p| 0.05 m, |EKF2−EV| 0.080 m, no resets, TRACKING |
| 2. PointCloud2 second driver 60 s, then SIGINT | Bridge **DIVERGED** on the first sample after ("no scan-matched pose for 59.5 s"). FAST-LIO's first pose was (−75.1, +1.3, −0.1) m. **Largest EV position ever sent to PX4 in the whole test: 0.05 m** |
| 3. Preflight on the diverged stack | **NO-GO**: E5 |p| 2642 m, E6 2642 m, E8 DIVERGED |
| 4. Restart FAST-LIO only | Bridge **TRACKING reset_counter=1**. EKF2 had fallen back to flow (eph 14.14) 1 s after starvation began; at resume it reset (xy 18→19, z 2→3) onto (−0.03, 0.05, 0.07) |
| 5. Preflight after restart | All estimate checks GO. **E1 false-failed** on two `fastlio_mapping` processes: the `ros2 run` wrapper matched by argv token. Fixed to match argv[0]; re-verified below |

**Final hardware run with the deployed pieces** (`evidence/g02-2026-09-12/v2_*`):

| Test | Result |
|---|---|
| A second `bash scripts/run.sh` while the stack runs | **exit 1**: `REFUSING to start: livox_ros_driver2_node is already running` |
| `ros2 launch dtp_bringup sensors.launch.py camera:=false xfer_format:=0` on top | **`NOT starting our Livox driver`**; still exactly 1 driver process |
| `ros2 launch dtp_bringup super.launch.py …` on top | **`NOT starting FAST-LIO`**, **`NOT starting fsm_node`**; still 1 of each |
| Deployed bridge via `ros2 run super_bringup px4_odom_bridge.py`; FAST-LIO restarted via `ros2 run` | Guard **TRACKING reset_counter=1** |
| `estimator_preflight.py`, 60 s | **GO, 14/14**: E1 one FAST-LIO (argv[0] fix holds), 10.0 Hz, age p95 60 ms, \|p\| 0.05 m, \|EKF2−EV\| 0.079 m, no resets |

`m3_bridge_check.py` passed 32/32 on three consecutive runs, and 32/32 again after T7 was
hardened. `dtp_bringup` tests: 12 passed, 2 skipped (the skips are the pre-existing SUPER-config
fixtures, which are not in this checkout).

## What this does NOT cover — open

- **Slow divergence in flight with no jump and no stamp gap** (e.g. degenerate geometry). The
  guard only sees discontinuities; E5 only runs on the ground.
- **A step just under the limit** (0.5 m in one frame, 5 m/s) passes.
- **Thresholds on a moving rig.** 5 m/s and 1.0 s are chosen from bench and handheld data. A false
  latch in flight means EKF2 loses vision and falls back to flow, and PX4's behaviour on that in
  the air is untested (flight plan P3/P5).
- **A FAST-LIO killed with SIGKILL** stays in the DDS graph until its lease expires, so the guard
  waits for "exactly one publisher" before resuming. The delay was not measured.
- **The latency surge (E4) is detected, not explained.** Unconfirmed suspects: load 6.0,
  `perception_node` 137 %, `extrinsic_est_en` at 74 % vs 31 % CPU (from `dtp_params.yaml` notes),
  and a laptop RViz on `/cloud_registered`. The earlier "FAST-LIO at 3.75 Hz" observation was not
  reproduced here.
- **Unexplained:** the 0.4 s EKF2 excursion to z 641 m at bag 220.7 s; the EKF2 restart at
  20:15:38; why 09-12 settled while the bench ran away.
- **Not fixed (separate bugs):**
  - DTP `super.launch.py` still publishes the static TF as `world → camera_init`, not `lidar_init`.
  - The Livox driver segfaults on SIGINT (harmless, seen every time).
