---
paths:
  - "scripts/px4/**"
  - "firmware/*.params"
  - "super_bringup/scripts/px4_odom_bridge.py"
  - "docs/fact-ev-*.md"
  - "docs/fact-mpc-alt-mode.md"
  - "docs/px4-tuning-notes.md"
  - "docs/fact-offboard-rc-arming.md"
  - "docs/ImplementationDoc.md"
  - "docs/calibrate.md"
---

# EKF2, the sensors it fuses, and flight-mode parameters

Scope: how PX4's EKF2 uses FAST-LIO, the MID360→FC mounting (the unmeasured yaw offset is a flight blocker), GPS / optical flow / range, and `MPC_ALT_MODE`. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

## M2 gate: EKF2 on FAST-LIO (signed off 2026-09-02)

**GATE SIGNED OFF 2026-09-02 on the NX.** Full chain live and healthy:
`ekf2 status → attitude: 1, local position: 1`; `cs_ev_pos`/`cs_ev_yaw`/`cs_ev_hgt` all True with
**every `reject_*` flag False**; `Payload rx` 1154-1158 B/s. **Hand-motion tracking — the step
that had never been run — passed**: 120 s, ~1 m excursions on all three axes, PX4's
`vehicle_local_position` vs `/lidar_slam/odom` correlated **+0.9935 / +0.9985 / +0.9997 (N/E/D),
all signs correct**, max error 10.4 cm (inside the declared 0.1 m std), **0/1195** invalid
samples, `dead_reckoning` never set, no EKF resets during the moves. Signs correct on all three
axes is the important part — an inverted axis is invisible on a bench and would flip a control
loop in M3. 15-min soak (past the ~8 min where the old gate collapsed): FAST-LIO RSS flat
155.9 → 155.8 MB, swap 0 B, bridge `in=14494 out=14494 rejected=0`.
**The lever arm is independently confirmed** — the constant PX4−FAST-LIO offset measured
`(-0.014, +0.024, +0.064)`, the negative of `EKF2_EV_POS = (+0.011, -0.023, -0.07)`, i.e. EKF2 is
reporting the FC body origin, not the LiDAR IMU. It also survived the FC power cycle.
**Both open questions are now closed — don't re-derive:**
(a) **`EKF2_EV_DELAY` stays `0`.** PX4 subtracts it **on top of** `timestamp_sample`
(`estimator_interface.cpp:381`), and FAST-LIO stamps odometry with `lidar_end_time`
(`laserMapping.cpp:635`) — the instant the ESIKF propagates the state to — so the ~50 ms is
already carried. Setting 50 would double-count. Confirmed empirically: PX4 *leads* FAST-LIO by
~one 10 Hz sample; a double-counted delay would make it lag. See `docs/fact-ev-delay.md`.
(b) **`heading_good_for_control: False` is expected, not a fault.** Publishing `POSE_FRAME_FRD`
makes PX4 set `yaw_align = false` (`ev_yaw_control.cpp:179`) — deliberate, because FRD declares
an *arbitrary* heading offset from True North. It costs **only Stabilized** heading hold and
**Auto/RTL yaw**; **Offboard (M3), Position and Altitude are unaffected**. We keep FRD. The
`EKF2_EV_CTRL=3` (mag-yaw) alternative is rejected while the live flags say
`cs_mag_heading_consistent: False`. See `docs/fact-ev-frd-yaw-align.md`.
**Still open, and it is the flight blocker now: nothing has ever been tested on a *moving/flying*
rig** — stationary and handheld only. The latency fix drops the cloud queue to depth 2, so under
load stale scans are *dropped*; a bench never produces that load. Re-run the frame-matched
latency check (`docs/fact-fastlio-latency.md`) on a moving rig before flight.
**Also open: the MID360→FC *rotation* has no owner anywhere.** `EKF2_EV_POS_X/Y/Z` carry the
translation, but PX4 has **no EV rotation parameter at all** (the complete set is `EKF2_EV_CTRL`,
`_DELAY`, `_NOISE_MD`, `_POS_X/Y/Z`, `_QMIN`), and the bridge does only the FLU→FRD *convention*
flip — not a physical mounting rotation. `mid360.yaml`'s `extrinsic_R` is identity and is the
wrong place regardless (it covers LiDAR optical origin → MID360 internal IMU).
**Tilt measured 2026-09-02 and it is fine: roll −1.55°, pitch +0.09°.** Method, which is
re-usable: `EKF2_EV_CTRL=11` fuses EV **yaw only**, so PX4's roll/pitch come from the FC's own
IMU independently of vision — compare them against FAST-LIO's (both gravity-referenced) and the
difference is the mounting tilt.
**Yaw offset remains unmeasured and is the one that matters.** It cannot be measured this way:
EKF2's yaw is *defined* by the EV yaw we send, so the two agree by construction however the LiDAR
is bolted on. A yaw mounting error leaves positions consistent but makes the attitude estimate
wrong by that angle, so the drone accelerates off-axis under position control — a spiral. It is a
mechanical check: is the MID360 square to the drone's nose? If it is ever rotated (even 90° for
cable routing), that angle must go **into `px4_odom_bridge`** — PX4 has nowhere to put it.

- **EKF2 is left on the `vision-only` profile** (`EKF2_EV_CTRL=11`, `EKF2_HGT_REF=3`,
  `EKF2_GPS_CTRL=0`) with the lever arm set. With the bridge stopped it has no absolute position
  source and will report `local position: 0` — expected, not a fault. Restart the bridge and it
  recovers; no `ekf2 stop`/`start` needed unless you change `EKF2_EV_POS_*`.
- **The M2 gate is re-runnable: `scripts/px4/m2_gate_check.py`.** Correlates
  `/fmu/out/vehicle_local_position_v1` against `/lidar_slam/odom` while you move the rig, and
  reports per-axis sign, correlation, lag and tracking error. It **refuses to pass a run where the
  rig didn't move** (a stationary run cannot prove a sign). Unlike everything else in
  `scripts/px4/`, it is a ROS 2 subscriber, not a MAVLink client — source the workspace.
  This is the tool for the outstanding moving-rig test.

## What EKF2 fuses

- **An optical flow sensor AND a downward range finder are fitted and being fused (found
  2026-09-02).** Nothing in these notes mentioned them before. Measured on the FC:
  **`EKF2_OF_CTRL=1`** (flow fusion on), **`EKF2_RNG_CTRL=2`** (range aiding on),
  `EKF2_OF_POS_Z=0.14`, `EKF2_OF_DELAY=20.0`. Both have live UAVCAN hardware behind them —
  `listener sensor_optical_flow` shows `quality: 159` publishing continuously, `listener
  distance_sensor` a downward (`orientation: 25`) 0.01–25 m ToF at `signal_quality: 100`.
  So EKF2 is fusing **LiDAR vision + optical flow + range**, not vision alone.
  **Keeping flow is now a deliberate decision (2026-09-02)** — it contributes velocity aiding,
  which LiDAR odometry does not provide (`px4_odom_bridge` omits velocity on purpose). Flow and
  range are therefore set **explicitly** in the `vision-only` profile rather than inherited from
  whatever is on the FC. **The profile name is historical**: it means "GPS aiding off", not
  "vision is the sole source". `vision-gps` and `stock` do **not** manage flow/range — switching
  to them leaves both wherever they were; check with `ekf2_profile.py --show`.
- **PX4's "vision" ≠ camera.** `EKF2_EV_CTRL`, `vehicle_visual_odometry` and `EKF2_HGT_REF=3
  (Vision)` mean *any* external non-GPS pose source — here the MID360 via FAST-LIO. No camera is
  involved until M4.

- **A GPS *is* fitted on this airframe (corrected 2026-08-30) — but EKF2 ignores it.** Earlier docs
  said "no GPS fitted"; that is false on this build. `gps status` reports a u-blox on `/dev/ttyS0`
  @115200, `status: OK`, 10 Hz, **`fix_type=4` (DGPS), 9 sats, eph 0.38 m**. What switches GPS off
  is **`EKF2_GPS_CTRL=0`** — a configuration choice, not missing hardware. So the bridge is still
  flight-critical **as configured**, but unlike before there is now a real GPS fallback available
  if you decide to enable it (`scripts/px4/ekf2_profile.py vision-gps`).
  **This bullet used to say `EKF2_GPS_CTRL=0` made FAST-LIO "the only position source" — that was
  wrong on two counts** (corrected 2026-09-02): optical flow and a range finder are also fused
  (next bullet), and FAST-LIO is better described as the only *absolute position* source.

## Position and Altitude mode height

### 2026-09-17 — `MPC_ALT_MODE` 2 → 1 (terrain following), at the user's instruction. NOT YET FLOWN.
The user reported that in Position mode the drone does not climb over a raised surface. The FC log
`/fs/microsd/log/2026-09-16/00_09_45.ulg` (= 09-15 20:10 EDT, POSCTL then ALTCTL) shows why.
- **Height is referenced to FAST-LIO.** `EKF2_HGT_REF=3`, so manual modes hold height in
  `lidar_init`, not above the ground.
- **The ARK Flow's ToF change is absorbed as terrain.** It goes into EKF2's terrain state, not the
  altitude.
- **`MPC_ALT_MODE=2` (terrain hold) barely engaged.** It uses the range reading only with sticks
  centred and speed < 0.4 m/s, and then locks the *current* gap. On that flight it was active ~19%
  of the time, never while crossing the edge.
- **What the drone did.** The range reading fell 0.74 → 0.10 m while the altitude setpoint held at
  0.22 m and the throttle stayed centred. It landed ~0.4 m above its takeoff point.

Full record, including the log-reading method:
[`docs/fact-mpc-alt-mode.md`](docs/fact-mpc-alt-mode.md).

**The three values, don't re-derive:**
- **0 — altitude following:** height relative to the origin (here FAST-LIO).
- **1 — terrain following:** height relative to the ground, always. Falls back to 0 if the ground
  estimate goes invalid.
- **2 — terrain hold, PX4's default, never chosen deliberately here:** relative to the ground only
  when stopped, relative to the origin when moving. It exists for baro drones, whose hover drifts,
  and deliberately ignores bumps while cruising. With FAST-LIO as the height source its benefit is
  small.

Now `MPC_ALT_MODE=1`, set over MAVLink disarmed, verified by read-back and saved (`+`). Backup:
`firmware/px4-params-pre-altmode-2026-09-17.params`. **Affects Position/Altitude mode only.
Offboard is untouched**, since SUPER's setpoints are absolute. **Do not use `EKF2_HGT_REF`=range
instead:** it would put EKF2's z out of step with FAST-LIO's under Offboard.

Caveats of terrain following:
- The ToF beam is 12.4°×6.2°, ~22×11 cm at 1 m, so it cannot see an edge coming. A step taller than
  the gap hits the props first.
- It drops when flying off an edge.
- It climbs over anyone walking underneath.
- Grass reads as ground.

**Still open:** `EKF2_MIN_RNG` is 0.01. The synthetic on-ground range skips the 0.14 m
`EKF2_RNG_POS_Z` offset, so the terrain estimate jumps ~14 cm at takeoff. It should be ~0.15;
unchanged, the user's call.

## Controller tuning (rate, angle, velocity, position)

### 2026-09-20 — retune planned after the Gemini 336 → 336L swap. NOTES ONLY, nothing changed or flown.
The procedure, read out of `docs/PX4 Tuning Guide.pdf`, with this FC's baseline beside each step, is
[`docs/px4-tuning-notes.md`](docs/px4-tuning-notes.md). What the 2026-09-17 dump shows:
- **The filter section of that guide was already applied, at its starting values.**
  `IMU_GYRO_RATEMAX` 800, `DSHOT_BIDIR_EN` 1, `IMU_GYRO_DNF_EN` 1, `DNF_MIN` 100, `DNF_BW` 20
  (exactly 0.2 × `DNF_MIN`, never narrowed), `IMU_GYRO_CUTOFF` 110, `IMU_DGYRO_CUTOFF` 55.
  `SDLOG_PROFILE` is still 273 (raw gyro FIFO), so FC logs are large.
- **Rate gains are stock apart from K = 1.2 on all three axes.** Roll and pitch are identical, so
  they were never tuned per axis. `MC_ROLL_P` / `MC_PITCH_P` are 4.0 (stock 6.5). No yaw FF.
  **Corrected 2026-09-27:** 4.0 *is* stock in v1.17; the 2026-09-27 entry below has the details.
- **The velocity and position loops are stock.** These are the loops Offboard flies on.
- **`CA_ROTOR*_PX/PY` are ±0.0335 / ±0.0305 m: exactly 0.11 ft and 0.10 ft stored as metres.**
  They are not this airframe's arm lengths. Their effect on allocation is not yet checked.
- **The vertical-velocity stage is flown in Altitude mode, so it would be the first flight of
  `MPC_ALT_MODE=1`.**
- **Tuning flights are flown with the LiDAR, FAST-LIO and `px4_odom_bridge` up, the Stabilized
  ones included.** The rate and angle loops do not use LIO (`EKF2_EV_CTRL=11` gives position,
  height and yaw only; roll/pitch are the FC's own). But the mode switch's other two positions
  (Position, Altitude) depend on it, `estimator_preflight.py` says NO-GO without it, the velocity
  and position loops must be tuned on the estimator the follower flies on, and no arm without LIO
  is on record. If LIO dies in the air: stay in Stabilized and land, do not switch to Position.
  Reasoning in the notes' §3.1.
- Gains are changed in QGC's PID Tuning page over UDP 14550. `set_params.py` is a fixed-purpose
  script, not a general setter.

Record the values a tuning session ends on here, dated, with the param dump's filename.

### 2026-09-21 — Flight 1 read; the session ends on `IMU_DGYRO_CUTOFF` = 40 (was 55)
Restore point: `firmware/px4-params-2026-09-21-pre-retune.params`. Everything else is unchanged:
`DNF_MIN` 100, `DNF_BW` 20, `DNF_HMC` 3, `IMU_GYRO_CUTOFF` 110, and all rate gains. Full record,
with the numbers: the notes' §9.
- **The log.** `/fs/microsd/log/2026-09-21/18_46_55.ulg`, Stabilized, 87 s airborne. It had
  no sharp wobbles (setpoints ≤ 17 °/s). **The rate P:D balance is still unread**, and Flight 2
  must carry the wobbles.
- **The notches and the gyro low-pass are right.** An offline replay of v1.17's filter chain on
  the raw FIFO matched the FC at r = 0.9995. In it, a narrower `DNF_BW` or a higher cutoff only
  lets more noise through, for ≤ 0.55 ms less delay.
- **The motor outputs are too noisy.** Above 40 Hz each motor carries 1.4–1.9 % rms, ~87 % of it
  from the D-term, and pitch's is twice roll's. The cause is broadband 30–180 Hz vibration,
  strongest on pitch. The change cuts it by a predicted 18 %. Filtering cannot do much more: the
  rest has to come from D, if pitch proves overdamped, and from the pitch vibration's mechanical
  source.
- **Two corrections to the guide, from the v1.17 source.** The ESC-RPM notch bandwidth is
  constant in Hz, not constant-Q. A motor below `DNF_MIN` parks its notch, it does not switch it
  off.
- **Tool:** `scripts/px4/fetch_ulog.py` downloads the newest log over 14569 (added 2026-09-21).

### 2026-09-21 — Flight 2 read; no change (still `IMU_DGYRO_CUTOFF` = 40)
Log `/fs/microsd/log/2026-09-21/19_17_02.ulg`: Stabilized, then Position, then 84 s of Altitude,
with wobbles. The full record is the notes' §9.
- **The D-cutoff change did what was predicted.** Motor noise ×0.78–0.84 and torque noise ×0.8
  against Flight 1. It is still above target.
- **Roll is critically damped.** Its step response settles at 1.0.
- **Pitch is not overdamped,** so no D cut. **Yaw lags.**
- **`DNF_MIN` 100 stays.** The RPM went below 100 Hz only at liftoff.
- **⚠️ `px4_offboard_bridge` in IDLE interleaves its hold setpoint into Altitude and Position
  mode.** This log's pitch wobbles are partly its doing. Tuning flights run with it stopped.
  → `px4-bridges-offboard.md`.

### 2026-09-21 — Flights 3–4 (20:12 wobbles, 20:13 up and down) read; no change
- **⚠️ Both flew with no FAST-LIO in EKF2.** No `cs_ev_*`, and no `estimator_aid_src_ev_*`
  topics. Height came from baro and ToF, velocity from flow, heading from the compass (the one
  marked inconsistent), with short dead-reckoning patches.
- **The first arms without LIO on record.** PX4 allowed them, and Position mode ran on flow.
  `estimator_preflight.py` would have refused them.
- **Rate loop, clean data** (the offboard bridge was off too):
  - roll is critically damped (peak ratio 1.01, step settles at 1.00);
  - pitch is critical to a touch under (1.05), so pitch D is not cut;
  - yaw: the sag in the deconvolved step was **not confirmed** by held inputs (it reaches
    0.83–0.98 in 80–200 ms, no saturation, tiny integrator). No yaw change is indicated.
- **Vertical velocity overshoots ~20–25 % on every transition** at stock `MPC_Z_VEL_*`. Not acted
  on, because it was flown on baro/ToF: re-fly with LIO up.
- **The wandering height is the throttle sitting outside the ±0.1 hold band.** RC3 is 1065–1932,
  so the band is 1455–1542 µs, and a non-centring stick rarely lands in it.
- Record: the notes' §9.

### 2026-09-21 — `SENS_BAR_AUTOCAL` rewrote the baro calibration in flight; `MC_BAT_SCALE_EN` is 1
- **⚠️ `SENS_BAR_AUTOCAL` = 1 recalibrates the baro against GNSS once per boot, whenever GNSS
  settles, armed or not** (`VehicleAirData.cpp:260`).
  - In Flight 3 it fired at 18.9 s airborne: `CAL_BARO0_OFF` 470 → 519 Pa, baro +4.2 m in one
    step, ~1 m leaked into EKF2's height.
  - It is also why `CAL_BARO0_OFF` differs in every dump.
  - **Set to 0 by the user on 2026-09-21** (read back 0, saved `x +`). `CAL_BARO0_OFF` is now
    frozen at 543.2 Pa. It costs nothing here, since `EKF2_GPS_CTRL` is 0.
- **`MC_BAT_SCALE_EN` 0 → 1, set by the user after Flight 4** (read back 1, saved `x +`). It was
  0 in all four flights, so **no flight has flown with it yet**. The logs support it: hover estimate ÷ battery scale
  is 0.40–0.42 in every flight.
- **The FC now stands at** `IMU_DGYRO_CUTOFF` 40, `MC_BAT_SCALE_EN` 1, `SENS_BAR_AUTOCAL` 0. Rate
  gains, notches, the gyro low-pass, `MPC_THR_HOVER` 0.41 and `MPC_Z_VEL_*` are unchanged from the
  baseline. Dump: `firmware/px4-params-2026-09-21-post-batscale-baroautocal.params`.
- Record: the notes' §9.

### 2026-09-27 — horizontal velocity loop studied; tuning plan accepted. NOTHING flown, no gain changed
Full record: [`docs/px4-velocity-tuning-plan.md`](docs/px4-velocity-tuning-plan.md). Scripts:
`docs/evidence/velocity-loop-2026-09-27/`.
- **The accepted order.** O1 (acceleration feedforward alone, stock gains, Offboard), then the
  Direct-velocity ladder V0–V6 (offboard bridge stopped), then O2/O3 (feedforward with the new
  gains, at 3 and then 4 m/s).
- **Parameters are written by the user, not by an agent.**
- **Proposed end point:** `MPC_XY_VEL_P_ACC` ≈ 3.0, `MPC_XY_VEL_D_ACC` 0.1,
  `MPC_XY_VEL_I_ACC` ≈ 0.2 × P, `MPC_XY_P` unchanged at 0.95. Not the guide's P 4.0.
- **What it rests on.**
  - Measured: the stock loop lags its velocity setpoint by ~0.55 s and does not overshoot it;
    the attitude loop lags by tau 0.15–0.19 s plus 40–60 ms; drag ~0.3 s⁻¹.
  - Modelled: feedforward removes most of the error, and tuning halves the rest (3 m/s:
    0.49 / 0.36 → 0.25 / 0.14 m behind / past).
  - **No horizontal velocity step has ever been flown on LIO,** so the loop's step response is
    modelled only.
- **Corrections to this file and the notes:**
  - **`MC_ROLL_P`/`MC_PITCH_P` 4.0 is stock in v1.17, not "lowered".** Upstream commit
    `c7ef12545c` cut the default from 6.5 for larger vehicles.
  - **The "~20–25 % overshoot" (Flights 3–4) is the vertical loop** (`MPC_Z_VEL_*`, on baro/ToF).
    It says nothing about the horizontal loop.
- **In v1.17, `MPC_ACC_HOR`, `MPC_JERK_MAX` and `MPC_XY_ERR_MAX` do not apply in Offboard.**
  `MPC_TILTMAX_AIR`, `MPC_XY_VEL_MAX` and `MPC_Z_VEL_MAX_DN/UP` do.

See also: `px4-link-and-firmware.md` (link, tools, dumping params first), `fast-lio-and-lidar.md`.
