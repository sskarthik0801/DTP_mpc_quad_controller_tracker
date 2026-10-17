# PX4 tuning notes — for the retune after the Gemini 336 → 336L camera swap

**Written 2026-09-20. 2026-09-21: Flight 1 was flown and read, and one parameter changed —
`IMU_DGYRO_CUTOFF` 55 → 40. Flight 2 confirmed that change and found that `px4_offboard_bridge`
corrupts Altitude mode, so **fly tuning flights with the offboard bridge stopped.** The record is §9.**

Source: [`PX4 Tuning Guide.pdf`](PX4%20Tuning%20Guide.pdf) (153 slides, Chris Rosser / AOS-RC, written
around a MicoAir743 on PX4 v1.16 with QGC v5 on Windows). Page numbers below (`p.N`) are PDF pages.
Everything marked **[repo]** is this drone's context and is not in the PDF. Everything marked
**[my reading]** is a judgement, not a measurement.

The guide's one idea: **tune the cascade from the bottom up, one layer per flight, and read the log
after every flight.** Each layer assumes the one below it is already right.

```
filters → rate (D, then K, then I, then yaw FF) → angle P → vertical velocity
        → horizontal velocity → position P → flight-behaviour limits
```

---

## Start here — the first session  [repo]

**The first session changes no gains.** It measures one thing, takes a baseline, flies one hover
and reads one log. That log decides where the retune starts. Details for every step are in the
numbered sections below.

**At the desk, props off**
1. **Dump the params** — the restore point:
   `python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F)-pre-retune.params`
2. **Measure the motor positions** (§2, p.36). Battery and 336L fitted, balance the aircraft to
   find the centre of mass, then measure each motor's X (forward) and Y (right) from it, in
   metres. Compare with the stored ±0.0335 / ±0.0305 m (= 0.11 / 0.10 ft). **Do not enter new
   values yet:** how v1.17's allocator treats the absolute scale is still unchecked.
3. **Connect QGC over UDP 14550.** Not USB straight to the FC: `mavlink-router` owns that link.
4. **Level horizon calibration** (p.20): upright on a level surface, about a minute.
5. **Desk check:** `bash scripts/desk_check.sh` and `python3 scripts/px4/clear_fault_logs.py`.
   Both are read-only.

**At the field**
6. **Bring the stack up:** `scripts/run.sh` (Livox driver + FAST-LIO), then
   `ros2 run super_bringup px4_odom_bridge.py` — or the GCS "Flight day" card's Start rows (§3.1).
7. **Preflight, aircraft sitting where it will take off. It must say GO:**
   `python3 scripts/px4/estimator_preflight.py`
8. **Safety** (p.47): prop and motor direction, kill switch tested, then arm in Stabilized and tilt
   it by hand on the ground to confirm it fights the right way.

**Flight 1 — a baseline, nothing changed**
9. **Stabilized, about 40 s:** ~20 s steady hover, a few quick climbs and descents, then ~10 s of
   sharp wobbles on each of roll, pitch and yaw. Hold the stick all the way back to centre. Keep
   the moves gentle enough that the rate setpoint does not flat-top at 220 °/s. Land.
10. **Pull the log** (QGC → Analyze Tools → Log Download). `SDLOG_PROFILE` is already 273, so it
    carries everything needed. It can be read offline with `pyulog` (`~/PX4-Autopilot/.venv`);
    nothing has to be uploaded to review.px4.io.

**Read that one log, in this order**

| Plot | Question it answers | Leads to |
|---|---|---|
| Motor RPM | Min and max RPM with the heavier camera. Does `IMU_GYRO_DNF_MIN = 100` still fit (0.9 × min RPM / 60)? | §5.1 |
| Angular Velocity PSD | Any new resonance from the 336L mount? How wide is the motor noise band — can `DNF_BW` come down from 20? | §5.1 |
| **Pitch Angular Rate**, zoomed on the wobbles | **Estimated peaks vs Setpoint peaks — the main question of the retune.** Expected: Estimated lower on pitch → D down 20 %. | §5.2 |
| Roll Angular Rate | The same comparison. | §5.2 |
| Motor Outputs | Signal-to-noise ≥ 15–20? | §5.1, D-term filter |

After that: **one change per flight**, in the order of §5.2 — D, then K, then I, then yaw FF — and
fill in a line of the session sheet (§8) each time.

**Two decisions that are the user's, before step 6:**
- The two flight blockers (hardfault soak, yaw mount) are still open.
- Everything above can be flown in Stabilized. `MPC_ALT_MODE=1` does not have to get its first
  flight until the vertical-velocity stage (§5.4).

---

## 1. What a camera swap actually invalidates  [my reading]

The 336L is a bigger camera on the nose. That changes mass, pitch inertia and the centre of mass.
It does not change the motors, props, ESCs or the frame's stiffness much. So:

| Layer | Redo? | Why |
|---|---|---|
| Motor positions from CoM (p.36) | **Check first** | The CoM moved forward. And the stored values look wrong already — see §2. |
| Level horizon cal (p.20) | Yes, 1 minute | Cheap, and the mount was disturbed. |
| Filters (p.42–67) | **One hover log to check, retune only if the noise moved** | A new mount can add a resonance. RPM notches track the motors by themselves. |
| Rate controller, pitch axis (p.68–105) | **Yes — this is the main job** | Pitch inertia changed. Expect *Estimated < Setpoint* peaks on pitch (softer, overdamped), not instability. |
| Rate controller, roll and yaw | Check in the same log | Probably little change. |
| Angle P (p.106–110) | Check in the same log | Only after rate is right. |
| Vertical velocity (p.111–119) | Check | Heavier aircraft. `MPC_USE_HTE=1`, so hover thrust re-learns by itself. |
| Horizontal velocity, position P (p.120–137) | Check last | Least sensitive to a payload change, but **these are the loops the follower flies on in Offboard.** |
| Flight-behaviour limits (p.138–153) | No | Pilot feel only. SUPER shapes its own trajectory (`max_vel` 1.0, `max_acc` 1.0). |

The guide's own rule for payload changes (p.98): **tune P:D balance at the heaviest configuration
and K at the lightest.** The 336L configuration is the heaviest one, so do P:D with it fitted.

Not part of this retune: the camera↔LiDAR extrinsic. The deployed one is already for the 336L
(`.claude/rules/dtp-calibration.md`).

---

## 2. Baseline — what is on the FC now  [repo]

From `firmware/px4-params-pre-altmode-2026-09-17.params`. **That dump predates the
`MPC_ALT_MODE` 2 → 1 change**, so take a fresh one before touching anything (§3).

| Parameter | On the FC | Stock default | Note |
|---|---|---|---|
| `IMU_GYRO_RATEMAX` | 800 | 400 | Guide's value (p.45). Already set. |
| `SDLOG_PROFILE` | 273 | 1 | = default + high rate + raw gyro FIFO. The **filter-tuning** profile (p.46), still on. Big logs. |
| `DSHOT_BIDIR_EN` / `MOT_POLE_COUNT` | 1 / 14 | 0 / 14 | RPM telemetry is on (p.57). |
| `IMU_GYRO_DNF_EN` / `_HMC` | 1 (ESC RPM) / 3 | 0 / 3 | p.59. `IMU_GYRO_FFT_EN` = 0. |
| `IMU_GYRO_DNF_MIN` | 100 Hz | 25 | Implies a lowest motor speed of about 6 700 RPM (100 / 0.9 × 60). |
| `IMU_GYRO_DNF_BW` | 20 Hz | 15 | **Exactly 0.2 × `DNF_MIN`: the guide's *starting* value (p.59). It was never narrowed (p.60).** |
| `IMU_GYRO_CUTOFF` | 110 Hz | 40 | **Above `DNF_MIN`.** p.65 puts it just *below* the motor peak; p.67 allows higher if the logs stay clean. |
| `IMU_DGYRO_CUTOFF` | 55 Hz | 30 | = 0.5 × gyro cutoff (p.66). **40 Hz since 2026-09-21 (§9).** |
| `MC_BAT_SCALE_EN` | 0 | 0 | **1 since 2026-09-21 (§9)**: thrust and torque scaled by battery state. |
| `SENS_BAR_AUTOCAL` | 1 | 1 | **0 since 2026-09-21 (§9)**: it had recalibrated the baro in flight. |
| `MC_ROLLRATE_K` / `MC_PITCHRATE_K` / `MC_YAWRATE_K` | 1.2 / 1.2 / 1.2 | 1.0 | Raised once, all three alike. |
| `MC_ROLLRATE_P` `_I` `_D` | 0.15 / 0.2 / 0.003 | same | Stock. Pitch identical. |
| `MC_YAWRATE_P` `_I` `_D` `_FF` | 0.2 / 0.1 / 0 / 0 | same | Stock. **No yaw feedforward yet** (p.104–105). |
| `MC_ROLL_P` / `MC_PITCH_P` | 4.0 / 4.0 | 6.5 | **Lowered.** `MC_YAW_P` 2.8 is stock. **Wrong, see the 2026-09-27 correction below: 4.0 is v1.17's stock.** |
| `MPC_XY_VEL_P_ACC` `_I_` `_D_` | 1.8 / 0.4 / 0.2 | same | Stock — p.127 says stock is usually overdamped. |
| `MPC_Z_VEL_P_ACC` `_I_` `_D_` | 4.0 / 2.0 / 0.0 | same | Stock. |
| `MPC_XY_P` / `MPC_Z_P` | 0.95 / 1.0 | same | Stock. |
| `MPC_POS_MODE` | 4 (acceleration based) | 4 | The guide switches this for tuning and back (§5.4). |
| `MPC_VEL_MANUAL` / `MPC_XY_VEL_MAX` | 10 / 12 m/s | 10 / 12 | The guide drops `MPC_VEL_MANUAL` to 2 for tuning. |
| `MPC_Z_VEL_MAX_UP` / `_DN` | 3.0 / 1.5 | same | Already the guide's tuning values (p.114). |
| `MC_AIRMODE` | 0 | 0 | Already what p.78 wants. |
| `MPC_THR_HOVER` / `MPC_USE_HTE` | 0.41 / 1 | 0.5 / 1 | Hover-thrust estimator is on. |
| `CA_ROTOR*_PX` / `_PY` | ±0.0335 / ±0.0305 m | ±0.15 | **Exactly 0.11 ft and 0.10 ft written as metres.** See below. |

**Reading of the baseline [my reading]:** someone ran this guide's filter section and stopped at the
starting values, bumped K once on all axes, and lowered angle P. Roll and pitch carry identical
gains, so they were never tuned per axis (p.90, p.98: an asymmetric drone usually wants different
D and K on roll and pitch). The velocity and position loops are stock.

**Correction, 2026-09-27: angle P was not lowered.** The table and the paragraph above are left as
written on 2026-09-20.
- **What changed upstream.** On this FC's v1.17 (`d6f12ad`), the default of `MC_ROLL_P` and
  `MC_PITCH_P` is **4.0**, not 6.5. Upstream commit `c7ef12545c` (2025-06-04) cut it: "6.5 is a
  relatively high value that was used mostly on smaller, low inertia vehicles … on larger, higher
  inertia vehicles this leads to problems". So 4.0 is stock here, and "lowered angle P" above is
  wrong.
- **What is still open.** The guide's p.110 check on Flights 3–4 reads "raise": estimated peaks
  0.69–0.90 of the setpoint, 160–225 ms late. That makes raising angle P a possible upstream
  lever, not the undoing of someone's change.
- **Two related points,** recorded in
  [`px4-velocity-tuning-plan.md`](px4-velocity-tuning-plan.md) §8:
  - §9's "~20–25 % overshoot on every transition" is the **vertical** loop (`MPC_Z_VEL_*`, flown
    on baro and ToF), not the horizontal one;
  - the horizontal stage (§5.5–5.6) now has its own plan in that file.

**The motor positions.** 3.4 cm and 3.0 cm are not this airframe's arm lengths. They are 0.11 ft
and 0.10 ft to the last digit, which is what QGC stores if the numbers were typed with its units
set to feet. p.36 asks for X (forward) and Y (right) **measured from the centre of mass, in
metres**. I have not checked how much PX4 v1.17's allocator cares about the absolute scale as
opposed to the X:Y ratio, so I am not claiming this has hurt anything — but it is the first thing
to measure, because the camera moved the CoM and these numbers should encode it.

---

## 3. Rules from this repo that override the guide  [repo]

- **A tuning flight is a flight.** Both flight blockers (the `uxrce_dds_client` hardfault soak, the
  unmeasured yaw mount) are still open and are the user's call. `python3
  scripts/px4/estimator_preflight.py` must say GO before every arm, tuning flights included.
- **Dump before any change, and again after each session:**
  `python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F)-pre-retune.params`
- **Connect QGC over UDP 14550.** The guide plugs in USB or Bluetooth; here `mavlink-router` owns
  the FC link. Never open the FC serial device. After an FC power cycle:
  `sudo systemctl restart mavlink-router`.
- **Change gains in QGC's PID Tuning page.** `scripts/px4/set_params.py` is a fixed-purpose script
  (XRCE transport), not a general setter.
- **Fly every tuning flight with the LiDAR and LIO up** — §3.1 has the full reasoning.
- **`MPC_ALT_MODE=1` (terrain following) has never flown.** The vertical-velocity stage (sharp
  full-stick climbs and descents in Altitude mode) would be its first flight. Decide that on
  purpose, not by accident (`docs/fact-mpc-alt-mode.md`).
- **Logs.** `logloader` is disabled, so nothing reaches Flight Review by itself. Fetch with QGC
  (Analyze Tools → Log Download, p.48) or over MAVLink 14569 as in `docs/fact-mpc-alt-mode.md`
  ("How the log was read"); `pyulog` is in `~/PX4-Autopilot/.venv`. The guide reads everything in
  **review.px4.io** (p.49): that is an upload to a public service, "Share by link".
- **Disk and SD.** With `SDLOG_PROFILE=273` a short flight is tens of MB. p.44 says cards over
  32 GB are unsupported (the guide's claim for its board; not checked for the ARK FPV).
- **Flight-safety interlocks are not loosened to make a tuning step easier.**

### 3.1 Do the tuning flights need the LiDAR and LIO?  [repo]

**Asked 2026-09-20. Answer: turn them on for every tuning flight, although the first three
stages do not use them for control.**

**What the FC fuses** (2026-09-17 dump, and `.claude/rules/px4-estimator.md`):

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_EV_CTRL` | 11 | From FAST-LIO: horizontal position, height and **yaw**. Not roll/pitch, not velocity. |
| `EKF2_HGT_REF` | 3 | Height reference is vision, i.e. FAST-LIO. |
| `EKF2_GPS_CTRL` | 0 | A GPS is fitted and healthy, but EKF2 ignores it. FAST-LIO is the only *absolute* position source. |
| `EKF2_OF_CTRL` / `EKF2_RNG_CTRL` | 1 / 2 | Optical flow (velocity aiding) and the downward ToF are fused too. |
| `EKF2_BARO_CTRL` / `EKF2_MAG_TYPE` | 1 / 0 (auto) | Baro is fused. The compass is what yaw falls back to without EV yaw — and it is recorded as inconsistent on this airframe (`cs_mag_heading_consistent: False`). |
| `COM_FLTMODE1` / `4` / `6` | 8 / 2 / 1 | The mode switch is **Stabilized / Position / Altitude**. |

**Stage by stage:**

| Stage | Mode | Does the control loop need LIO? |
|---|---|---|
| 5.1 filters, 5.2 rate, 5.3 angle | Stabilized | **No.** The rate loop runs on the gyro, and roll/pitch come from the FC's own IMU independently of vision. Without LIO only the heading reference changes (to the compass), which Stabilized flying tolerates. |
| 5.4 vertical velocity | Altitude | **Yes.** Height and vertical velocity come from FAST-LIO + range. Without it EKF2 falls back to another height source (probably baro): a different estimator from the one the follower flies on. |
| 5.5 horizontal velocity, 5.6 position | Position | **Yes.** Without LIO there is no absolute position; at best Position mode runs on optical flow alone, at worst it is refused. |

**Why LIO stays on even for the Stabilized stages:**
1. **The fallback switch positions depend on it.** The guide makes Position the safe default
   (p.27). With LIO off, flipping to Position or Altitude mid-wobble lands on a refused mode or on
   a baro/flow estimate — not the recovery behaviour you expect.
2. **The arm gate needs it.** `scripts/px4/estimator_preflight.py` must say GO before every arm,
   and with LIO off it says NO-GO. Its checks:
   - E1 exactly one Livox driver, one FAST-LIO, one publisher each on `/livox/lidar`, `/livox/imu`,
     `/lidar_slam/odom`;
   - E2 FAST-LIO started *after* its driver;
   - E3 odometry ≥ 9.5 Hz in every 10 s, no stamp gap > 1.0 s;
   - E4 odometry age p95 ≤ 150 ms;
   - E5 |odom| ≤ 1.0 m — the aircraft is where FAST-LIO was started (the check that would have
     refused the 3.74 km session);
   - E6 EKF2 within 0.20 m of the vision estimate, `xy_valid`, `z_valid`, not dead-reckoning;
   - E7 no EKF2 resets in the window (use the full 60 s);
   - E8 `/px4_odom_bridge/status` is `TRACKING`.
3. **The outer loops must be tuned on the estimator they fly on.** The velocity and position
   loops close on EKF2's velocity and position, which here come from FAST-LIO + flow + range.
   They are the loops the follower uses in Offboard. Gains found on a baro/flow estimate may not
   transfer.
4. **It is untested the other way.** Every logged flight armed in Stabilized *with the stack up*.
   Nothing in this repo records an arm without LIO, so whether PX4 would even allow it (compass
   inconsistency, no height reference) is unknown.

**The minimum stack for a tuning flight:**
- needed: the Livox driver, FAST-LIO, `px4_odom_bridge` — exactly one of each;
- `scripts/run.sh` also starts SUPER's `fsm_node` alongside FAST-LIO. That is harmless here;
- **not** needed: the DTP follower, the camera, `px4_offboard_bridge`, any Offboard engagement;
- the usual sensor-chain check first: `ip -brief link show eno1`, `ping 192.168.1.154`;
- nobody subscribes the laptop to `/cloud_registered` while flying;
- run the preflight **on the ground where it will take off**, because E5 measures distance from
  where FAST-LIO started.

**If LIO dies or diverges in the air:** in Stabilized the aircraft stays fully controllable — that
mode does not use it. **Do not switch to Position.** Land in Stabilized.

---

## 4. Before the first tuning flight (p.13–37, the parts that matter here)

1. **Motor positions** (p.36): X forward, Y right, from the **CoM**, in metres. Find the CoM by
   balancing the flight-ready aircraft (battery and 336L fitted).
2. **Level horizon** (p.20): upright on a level surface, OK. If it is slow to finish, suspect the
   accelerometer or vibration. Skip the six-side accelerometer cal (p.19: optional, factory cal
   is good). Compass cal (p.15–17) is not needed for a camera swap; `docs/calibrate.md` has this
   drone's compass caveats.
3. **Kill switch verified** (p.28, p.47). Stick disarm is too slow when a tune goes wrong.
4. **Flight modes on the switch** (p.27): Stabilized, Altitude and Position are the three the
   guide tunes in. Position is the safe default position.
5. **Props and motor direction re-checked** (p.47): "by far the biggest cause of crashes".
6. **Stack up and preflight GO** [repo]: Livox driver, FAST-LIO and `px4_odom_bridge` running,
   `estimator_preflight.py` run at the take-off spot (§3.1). This applies to the Stabilized
   flights too.
7. **First hover in Stabilized** (p.47): tilt it on the ground first and confirm it fights the
   right way; lift off smoothly; hover ~20–30 s; land; pull the log. Over grass if possible.

---

## 5. The procedure

### 5.1 Filters (p.42–67)

**Logging for this stage** (p.45–46): `IMU_GYRO_RATEMAX=800`, `SDLOG_PROFILE` = Default + High rate
+ Raw FIFO gyro (= 273), reboot. *Both are already set.* A correct log shows **Angular Velocity
Power Spectral Density** up to about 0.45 × `IMU_GYRO_RATEMAX` = 360 Hz (p.50).

**Reading the PSD** (p.51–52):
- fundamental = motor RPM / 60 — strong with any prop;
- 2nd harmonic — strong with 2-blade props, weak with 3-blade;
- 3rd harmonic — strong with 3-blade, weak with 2-blade.

**RPM notches** (p.53–59). Notches beat a low-pass for motor noise: deep attenuation at one
frequency for little delay. RPM from bidirectional DShot is preferred over the FFT engine (more
accurate, less delay).
1. Fly rapid climbs and descents in the mode you actually use. Read min and max RPM off the
   **Motor RPM** plot (the guide's drone: 5 600–11 000).
2. `IMU_GYRO_DNF_EN = 1` (ESC RPM), `IMU_GYRO_FFT_EN = 0`.
3. `IMU_GYRO_DNF_MIN = 0.9 × (min RPM / 60)`. Example: 5600/60 × 0.9 = 84 Hz.
   **[repo] In v1.17 a motor below `DNF_MIN` does not switch its notch off. The notch is *parked*
   at `DNF_MIN` (`VehicleAngularVelocity.cpp`, `UpdateDynamicNotchEscRpm`).** Setting it higher
   than the RPM really drops to only leaves the notch sitting above the noise.
4. `IMU_GYRO_DNF_BW = 0.2 × DNF_MIN` to start. Example: 17 Hz.
5. `IMU_GYRO_DNF_HMC` = 2 for 2-blade props, 3 for 3-blade. Reboot.

**Narrowing the notch** (p.60–61). Zoom the PSD on the fundamental and estimate how wide the noise
band is (the guide's: ~5 Hz); that is the target `DNF_BW`. The guide says the bandwidth is set at
the *minimum* frequency and grows in proportion above it (constant Q, p.54, p.60). **[repo] Not
on this firmware:** v1.17 (`d6f12ad`) gives every notch, at every harmonic and every frequency,
the same `DNF_BW` in Hz — §7. It also resets a notch whose frequency jumps by more than its
bandwidth between two RPM reports, so a narrow notch can lose track when the RPM moves fast.
Narrower = less delay. **Check:** the *Raw Angular Speed (Gyroscope)* plot should not change. New
high-frequency "fur" on the traces means the notch is now too narrow and noise is leaking past.

**Gyro low-pass, `IMU_GYRO_CUTOFF`** (p.62–65). Real motion is below ~30 Hz; vibration is above
~50 Hz and depends on prop RPM. Lower cutoff = more delay = slower reaction to gusts and sticks.
Push the cutoff as high as it goes without letting noise through; with working notches the usual
spot is **just left of the motor-noise peak** in the *Angular Velocity FFT*.

**D-term low-pass, `IMU_DGYRO_CUTOFF`** (p.66, p.97). Start at **0.5 × `IMU_GYRO_CUTOFF`**. PX4 has
no plot of D-term noise, so judge it on the **Motor Outputs** plot: aim for a signal-to-noise
ratio of about 10 on small quads, **15–20 on larger ones** (big motors and ESCs suffer more from
noise). Raise in 10–20 % steps while that holds.

**Going higher** (p.67). When agility matters, raise both cutoffs further, and after every step
check *Raw Angular Speed* and *Actuator Controls FFT* for noise reaching the PID loop.

### 5.2 Rate controller (p.68–105) — the core of the retune

**Setup** (p.78): `MC_AIRMODE = 0` (stops a bad tune from climbing away); `SDLOG_PROFILE = 17`
(Default + High rate) is enough — 273 is a superset and also works.

**The flight** (p.77, p.79, p.82): Stabilized or Altitude mode, **never Position**. About 10 s of
sharp wobbles per axis. **Hold the stick all the way back to centre** — a released stick bounces
and looks like too much P. **Do not saturate:** the rate setpoint clips at 220 °/s
(`MC_ROLLRATE_MAX`); if the setpoint trace has flat tops, fly it again more gently.

**Order: D → P → I → FF** (p.75). D keeps P from oscillating; P keeps I from oscillating.

**Step 1 — P:D balance, with D** (p.83–90). Zoom into the wobbles on *Roll Angular Rate* and compare
peak heights. A lag between the two traces is normal (PX4 has no derivative feedforward).

| Estimated peaks vs Setpoint peaks | Meaning | Do |
|---|---|---|
| Higher | Underdamped — D too low for this P | Raise `MC_ROLLRATE_D` |
| Equal | Critically damped — done | — |
| Lower | Overdamped — D too high for this P | Lower `MC_ROLLRATE_D` |

Move D in **20 % steps** (×1.2 or ×0.8), not by doubling. Roll and pitch are independent: tune
one, then the other (`MC_PITCHRATE_D`). The step-response plots (Flight Review → *Open PID
Analysis*, p.84–86) say the same thing — overshoot and bounce-back = D too low; slow crawl to 1.0
= D too high — but they can hide problems, so trust the time series first. The target is the
critically damped curve: a trace of overshoot is fine, any oscillation is not.

**Step 2 — overall gain K** (p.91–95, p.98–99). `MC_ROLLRATE_K` scales P, I and D together. Raise
it per axis; the lag between Estimated and Setpoint shrinks (often a big improvement in feel),
then returns diminish, then **micro-oscillations that do not die away** appear in the log and as
a new chittering sound from the motors.
- **A new continuous oscillating sound at take-off after raising K: land immediately.** It can
  kill ESCs and motors (p.95).
- A short burst that dies away is fine. One that persists means K is too high.
- When you find the limit, **back off 20–30 %** (more if payload or build varies).
- Axes are independent: a limit on one says nothing about the others.
- QGC's slider stops at 3.0; the Parameters page allows up to 5.0.
- Re-check P:D as K rises — a higher K makes an imbalance more visible. Raising D lowers the K at
  which D-oscillation starts, because they multiply (K × D).

**If oscillation does not improve when K comes down** (p.96): it is **D-term oscillation** — reduce D
*and* P together in 10 % steps — or **too much filter delay**, so go back to §5.1.

**Step 3 — I** (p.101–102). Usually leave it: K already scaled it. More I does not make the I term
stronger, it makes it *faster*. Too much shows as overshoot built up *during* a fast move, and it
costs angle-controller accuracy. Raise `MC_ROLLRATE_I` / `MC_PITCHRATE_I` only for a
high-performance tune that must fight wind quickly.

**Step 4 — yaw** (p.76, p.100, p.103–105). Yaw has little authority, so it is naturally overdamped:
`MC_YAWRATE_D = 0`, no P:D step.
1. Raise `MC_YAWRATE_K` the same way, back off 20–30 % (or more).
2. If yaw still lags at maximum K, **raise `MC_YAWRATE_I`** — on yaw, I building up during a move
   is wanted.
3. **Feedforward, yaw only:** start `MC_YAWRATE_FF = 0.005`; raise in 20 % steps until the
   estimated yaw-rate peak equals the setpoint peak; lower it if there is overshoot. Do not use FF
   on roll or pitch of a multicopter.

### 5.3 Angle controller (p.106–110)

A plain P controller: angle error → rate setpoint. QGC: PID Tuning → Attitude Controller → manual.
Parameters `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P`. Read *Roll / Pitch / Yaw Angle* in PID review:

| Estimated vs Setpoint angle | Do |
|---|---|
| Peaks fall short, large lag | Raise P |
| Peaks match, small delay | Done |
| Overshoot | Lower P |

### 5.4 Vertical velocity (p.111–119, p.130)

**Setup** (p.114–115): `MPC_Z_VEL_MAX_DN = 1.5`, `MPC_Z_VEL_MAX_UP = 3` (already set). QGC: PID Tuning
→ Velocity Controller → Vertical, and set **position control mode to "Direct velocity"** so the
stick is the velocity setpoint and you can fly near-step inputs.

**The flight** (p.116): **Altitude mode.** Sharp full-stick climbs and descents; reach full speed,
**hold it at least 1 s**, reverse. It needs height and room.

**Tuning** (p.117–118), on the *Velocity* plot, Z against Z setpoint:
1. Raise `MPC_Z_VEL_P_ACC` in 20 % steps until overshoot shows; go back to the last value without.
   (Guide's example: P 4.0 was slow to reach the setpoint; 8.0 with D 0.2 was sharp and clean; 8.0
   with D 0.1 overshot and bounced.)
2. Then raise P, I and D **together** (×1.2) until the response is sharp and accurate. If it turns
   underdamped, drop P a little and continue.

**The P ceiling** (p.119, p.130). The velocity controllers run at only 50 Hz (Nyquist 25 Hz, and a
5 Hz low-pass on the D term — `MPC_VELD_LP = 5.0` on this FC), so above a critical P they show a
fast oscillation that **more D cannot fix**. When it appears: **cut P by at least 30 %**, then lower D at that P for a less damped
response.

### 5.5 Horizontal velocity (p.120–129)

**Setup** (p.123–124): `MPC_VEL_MANUAL = 2.0` m/s (QGC needs *Force Save*: it is below the listed
minimum of 3), `MPC_XY_VEL_I_ACC = 0.2`, position control mode = Direct velocity.

**The flight** (p.125): **Position mode.** Sharp **full-stick** left/right (and optionally
forward/back): full speed one way, brake, full speed the other. The *Velocity* plot should show
near-square setpoints.

**Tuning** (p.127–128):
- Stock is usually **overdamped** (too much D, too little P). If velocity creeps up to the
  setpoint, **lower `MPC_XY_VEL_D_ACC` first**; some airframes go all the way to 0.
- Then raise `MPC_XY_VEL_P_ACC` to the last value before overshoot. (Guide's example: 2.0 / 0.2
  undershoots; **4.0 / 0.05 is good**; 4.0 / 0.0 shows a small overshoot.)
- Then scale P, I, D together ×1.2.
- **Same P ceiling as vertical** (p.129): micro-oscillation that D does not help → cut P by ≥ 30 %.
  (Guide's example of too high: 6.0 / 0.2.)

### 5.6 Position controller (p.131–137)

`MPC_XY_P` and `MPC_Z_P`, Direct velocity mode. Flight: a sharp move, **centre the stick, wait for
it to settle**, repeat. With a tight velocity loop there is little left for position P to do, so
the window is wide and it may need no change. **Raise it until the hold is accurate and stop
there**; more buys nothing and risks oscillation. (Guide's examples: `MPC_Z_P` 0.5 held badly, 1.0
held well, 2.0 was no better; `MPC_XY_P` 0.5, 0.95 and 1.6 all looked fine.)

### 5.7 Put it back, then flight behaviour (p.138–153)

**First: position control mode back to "Acceleration based"** (`MPC_POS_MODE = 4`, p.140). That is
the normal mode. **[repo] Also restore `MPC_VEL_MANUAL` (10), and decide whether `SDLOG_PROFILE`
stays at 273.** The guide sets `MPC_XY_VEL_I_ACC` to 0.2 for tuning and never puts it back: it
scales I up together with P and D instead. The baseline was 0.4.

After §5.1–5.6 the aircraft is sharp (p.139). These limits calm it down for the pilot; they are
not tuning:

| Mode | Parameters |
|---|---|
| Stabilized (p.142) | `MC_MAN_TILT_TAU` and `MPC_MAN_Y_TAU` (stick low-pass: larger = smoother; now 0.0 and 0.08), `MPC_MANTHR_MIN` (**not below 7 %**, motors can stall), `MPC_MAN_TILT_MAX`, `MPC_TILTMAX_AIR`, `MPC_MAN_Y_MAX` |
| Altitude (p.144–145) | `MPC_ACC_UP_MAX`, `MPC_ACC_DOWN_MAX`, `MPC_JERK_MAX` (how fast acceleration builds and releases), `MPC_Z_VEL_MAX_UP/DN`, `MPC_THR_MIN/MAX`, `MPC_HOLD_MAX_Z`, `MPC_ALT_MODE` |
| Position (p.147–148) | `MPC_ACC_HOR`, `MPC_JERK_MAX`, `MPC_XY_VEL_MAX`, `MPC_HOLD_MAX_XY` (speed below which position hold engages) |
| Autonomous (p.150–153) | `MPC_JERK_AUTO`, `MPC_XY_CRUISE`, `MPC_YAWRAUTO_MAX/ACC`, `MPC_YAW_MODE`, `MPC_Z_V_AUTO_UP/DN`, `NAV_ACC_RAD` (waypoint acceptance radius) |

[my reading] These shape stick and mission flight. In Offboard the follower's setpoints are already
a trajectory from SUPER, so this table is not where following gets better; §5.5–5.6 are.

---

## 6. Symptom → cause, in one place

| You see / hear | Cause | Fix | Page |
|---|---|---|---|
| Rate: Estimated peaks > Setpoint | Underdamped | D up 20 % | 88 |
| Rate: Estimated peaks < Setpoint | Overdamped | D down 20 % | 88 |
| Rate: big lag, no oscillation | Undertuned | K up | 92–94 |
| Fine "fur" on the rate trace that never dies; chittering motors | K too high | **Land.** K down 20–30 % | 95 |
| Oscillation that stays when K comes down | D-term oscillation, or filter delay | D and P down 10 % together; revisit filters | 96 |
| Overshoot that builds during a fast roll/pitch move | I too high | I down | 102 |
| Yaw lags even at max K | Yaw needs I, then FF | `MC_YAWRATE_I` up; `MC_YAWRATE_FF` from 0.005 | 103–105 |
| Setpoint trace has flat tops at 220 °/s | Stick moves too hard | Refly gentler | 82 |
| New fur on Raw Angular Speed after narrowing a notch | Notch too narrow | `DNF_BW` up | 61 |
| Noisy Motor Outputs (SNR < 10; < 15–20 on a big quad) | D-term filter too open | `IMU_DGYRO_CUTOFF` down | 97 |
| Angle: peaks short / overshoot | Angle P low / high | `MC_*_P` up / down | 110 |
| Velocity: slow crawl to setpoint | Overdamped | D down, then P up | 117, 127 |
| Velocity: overshoot and bounce | Underdamped | P down a step | 117, 127 |
| Velocity: fast micro-oscillation, D does not help | Past the 50 Hz loop's P ceiling | **P down ≥ 30 %**, then D down | 119, 129 |
| Position hold drifts off the setpoint | Position P low | `MPC_XY_P` / `MPC_Z_P` up, stop when accurate | 136–137 |
| Gyro or level cal slow to complete | Bad sensor or too much vibration | Fix mechanically first | 18, 20 |

---

## 7. Slips in the PDF

- p.50 writes `IMU_GYRO_MAXRATE`; the parameter is **`IMU_GYRO_RATEMAX`** (p.45 has it right).
- p.59 writes `IMU_GYRO_FFT`; the parameter is **`IMU_GYRO_FFT_EN`**.
- p.78 writes `SD_LOG_PROFILE`; it is **`SDLOG_PROFILE`**.
- p.89: "if the current value is 0.001 try 0.008 or 0.0012" — it means **0.0008**.
- p.136 (horizontal position P) says "until *vertical* position hold is accurate" — a paste from
  p.137; it means horizontal.
- p.142 writes `MPC_MAN_TIL_TAU`; on this FC (v1.17) it is **`MC_MAN_TILT_TAU`**.
- p.153 writes `MPC_ACC_DN_MAX`; it is **`MPC_ACC_DOWN_MAX`** (p.144 has it right).
- p.54 and p.60 say a dynamic notch keeps a constant Q (bandwidth growing with frequency). **On
  this FC's v1.17 (`d6f12ad`) the bandwidth is constant:** `UpdateDynamicNotchEscRpm` passes
  `IMU_GYRO_DNF_BW` unchanged to every notch, so a notch at 600 Hz is as wide as one at 200 Hz.
- p.119 and p.130 are two versions of one slide. p.119 is the fuller one (the ≥ 30 % rule).
- The FFT-engine notches, optical flow and proximity sensors are left to the author's paid course
  (p.8, p.55).

---

## 8. A session sheet

```
before:   LiDAR + FAST-LIO + odom bridge up (one of each) · estimator_preflight GO at the
          take-off spot · param dump taken · kill switch tested · props checked
per flight, ONE change:
  param(s) changed ........ from ...... to ......
  mode / manoeuvre ........
  log file ................
  what the plot showed ....
  next step ...............
after:    MPC_POS_MODE back to 4 · MPC_VEL_MANUAL restored · param dump taken · values recorded
          in .claude/rules/px4-estimator.md, dated
```

---

## 9. Session log  [repo]

### 2026-09-21 — Flight 1 (the baseline), read. One change: `IMU_DGYRO_CUTOFF` 55 → 40

**The flight.**
- **The log.** SD `/fs/microsd/log/2026-09-21/18_46_55.ulg` (14:46 EDT, 43.7 MB). Local copy
  `~/px4_logs/2026-09-21_18_48_30.ulg`, fetched with `scripts/px4/fetch_ulog.py`, which names the
  file by the log's last-write time.
- **Mode and battery.** Stabilized throughout. Airborne from 4.9 s to 91.8 s, disarmed by RC switch.
  6S, 24.4 → 22.2 V, ~30 A in the hover, hover thrust 0.41 → 0.43.
- **Params.** Flown on the §2 baseline. Dump `firmware/px4-params-2026-09-21-pre-retune.params`:
  against 09-17 only `MPC_ALT_MODE` and calibration values moved.
- **What it did not contain: step 9's sharp wobbles.** Rate setpoints never went above 17 °/s,
  the sticks stayed within ±25 %, and there were no hard climbs or descents. That was enough to
  answer the filter questions. It was not enough for the rate controller's P:D balance.

**How it was read.** With pyulog, plus an offline replay of v1.17's gyro filter chain
(`VehicleAngularVelocity`). The chain is:
- ESC-RPM notches, with PX4's own update and reset rules;
- `LowPassFilter2p`;
- a derivative through an `AlphaFilter` for the D-term.

The replay ran on the log's raw FIFO gyro and ESC RPM. How closely it matched the FC:
- the FC's `vehicle_angular_velocity`: within 0.11–0.15 °/s rms, r = 0.9995;
- the D-term input: r = 0.99;
- the torque noise it predicts: within 4 % of the logged `vehicle_torque_setpoint`.

So candidate values were scored on this flight's data, without a flight for each one.

**Replay trap.** PX4 designs these filters at the *measured* sample rate,
`vehicle_imu_status.gyro_raw_rate_hz` = 8106 Hz. It does not use 1/`sensor_gyro_fifo.dt`:
that `dt` is 119.47 µs (8370 Hz), because the IIM-42653's FIFO timestamp is quantised to 7 ticks
of 17.07 µs. A replay built on `dt` puts every notch 3 % low.

**Findings**

| Question (Start-here table) | Answer |
|---|---|
| Motor RPM | 8 640–14 100 RPM in the air: a fundamental of 144–235 Hz, with each motor's median at 184–205 Hz. No motor saturated. |
| Does `DNF_MIN = 100` fit? | Yes, and here it never mattered: the RPM never fell below 144 Hz. The guide's 0.9 × min RPM / 60 gives 130 Hz. That would only park the notch above the motor in a hard descent (§5.1). **Unchanged.** |
| `DNF_HMC = 3`? | Right. The 3rd harmonic (~628 Hz) is stronger than the 2nd (~414 Hz), which is what 3-blade props do. |
| A new resonance from the 336L mount? | No narrow line outside the motor harmonics. Instead there is a **broadband floor from ~30 to ~180 Hz**, with about 2.5× the power on pitch that roll has. Body pitch is the raw sensor's x axis. |
| Can `DNF_BW` come down from 20? | **No.** In the replay every step narrower lets more noise through: 15 Hz gives +9 % pitch rate noise, 12 Hz +16 %, 10 Hz +23 %. The saving is only 0.1–0.2 ms of delay. **Unchanged.** |
| `IMU_GYRO_CUTOFF = 110`? | Where p.65 puts it: just left of the motor band, which starts at ~150 Hz. 130 Hz gives +22 % noise for 0.3 ms less delay, 150 Hz +42 % for 0.55 ms. **Unchanged.** |
| Motor Outputs SNR | **Too noisy.** Above 40 Hz each motor carries 1.4–1.9 % rms (≈ 0.08–0.11 peak-to-peak), against 0.8 % rms of real signal in the hover. p.97's example has a ~0.4 swing with ~0.03–0.04 of fuzz, SNR 10. By that measure even a hard wobble here would score only ~3–5, and a quad this size should reach 15–20. |
| Where the noise comes from | **The D-term.** It is ~87 % (rms) of the roll and pitch torque noise above 40 Hz, and pitch's is twice roll's. The gyro filters are not what makes the aircraft slow: the whole chain costs 2.5 ms at 10 Hz. |
| Pitch and roll: Estimated vs Setpoint peaks | **Not measurable from this flight**, because there were no wobbles. A step response deconvolved from the hover settles at 1.3 (roll) and 1.7 (pitch). A loop with an I term cannot do that, so the estimate is biased by the angle loop reacting to disturbances. |

**The change.** Made disarmed, over MAVLink 14569. It read back 40.0, and `param show` marks it `x +`.

`IMU_DGYRO_CUTOFF` **55 → 40 Hz**.
- **Predicted by the replay:** roll and pitch torque noise above 40 Hz down 18 %.
- **Cost:** the D-term low-pass's lag grows from 10° to 14° at 10 Hz, and from 20° to 27° at 20 Hz.
- **Still less lag than stock.** The whole D path lags ~32° at 15 Hz. Stock PX4 (40 / 30 Hz), the
  setting these stock D gains were made for, lags ~58°.
- **Restore:** set 55, or load the dump above.

[my reading] **Filtering alone will not reach an SNR of 15–20.** 30 Hz would give −32 % and 25 Hz
−40 %. Below that the filter eats the D-term's phase lead. The rest has to come from two places:
- **D itself.** If Flight 2 shows pitch overdamped (the expectation in the Start-here table),
  pitch D −20 % takes about another 18 % off pitch's noise.
- **The pitch-axis vibration, found mechanically:** prop balance, the 336L and LiDAR mounts, the
  FC's mounting.

**Flight 2.** The only change on the aircraft is the one above.
1. The same ~20 s hover. Then **step 9's wobbles**: sharp, stick fully back to centre, ~10 s per
   axis, with setpoint peaks of 100–200 °/s. They must not flat-top at 220 °/s.
2. Two or three hard climbs and descents, to find the real minimum RPM for `DNF_MIN`.
3. Read, in this order:
   - motor-output noise above 40 Hz: expect ~1.2–1.6 %;
   - pitch and roll, Estimated vs Setpoint peaks → §5.2 step 1, one axis at a time.

### 2026-09-21 — Flight 2 (`IMU_DGYRO_CUTOFF` 40), read. No change. It found a bridge hazard

**The flight.**
- **The log.** SD `/fs/microsd/log/2026-09-21/19_17_02.ulg` (15:17 EDT, 47.9 MB). Local copy
  `~/px4_logs/2026-09-21_19_18_48.ulg`.
- **Modes.** Stabilized to 12.9 s (takeoff at 4 s), Position 12.9–19 s, **Altitude 19–103 s**, and
  landed at 104 s. The only param different from Flight 1 was `IMU_DGYRO_CUTOFF` 40.
- **What was flown.** Wobbles on all three axes: rate setpoints up to 115 °/s roll, 187 °/s pitch
  and 68 °/s yaw, none flat-topped. Then a slow climb to 9 m, a hold, and a 0.6 m/s descent. The
  hard full-stick climbs and descents of §5.4 were not flown.
- **Dropouts.** The logger dropped 0.6 s at 48.4–49.1 s. That window was excluded.
- **It was the first flight of `MPC_ALT_MODE=1`.** Over flat ground, EKF height and the ToF
  agreed throughout, `dist_bottom_valid` held, and the ToF changed range mode at ~4.5 m and back.
  Terrain following itself (over a step) was not exercised.

**⚠️ The finding that matters most.** In Altitude mode the attitude setpoint flipped between two
values from one sample to the next. At 55.7 s it went +18.8°, −23.2°, +18.6°, −23.1° while the
stick sat still at −0.48. The second stream is `px4_offboard_bridge` in IDLE: it publishes a
"hold here" `trajectory_setpoint` at 50 Hz onto the topic PX4's Altitude task also writes. Full
record and scope: `.claude/rules/px4-bridges-offboard.md`. **For tuning: keep the offboard
bridge stopped.** It also makes this flight's pitch "wobbles" partly the bridge's doing: the
pilot was holding steady pitch pulls, and the bridge's braking samples made the rate setpoint
swing to −150 °/s. **The guide's peak comparison (p.88) cannot be used on this log.** Setpoint
peaks the aircraft is not meant to follow drag the ratio down: it came out 0.81 roll and 0.68
pitch.

**What could still be read.** The rate loop tracks whatever setpoint it gets, so the bridge's
toggling works as excitation. A step response deconvolved over the Altitude section (1 s windows
with a setpoint std > 5 °/s) gives the table below. That is 118 windows for roll, 258 for pitch
and 54 for yaw.

| Axis | Step response | Reading |
|---|---|---|
| Roll | t50 35 ms, t90 58 ms, peak 1.00, settles 0.96–1.0, no bounce | **Critically damped: P:D is right.** It is credible because it settles at 1. |
| Pitch | t50 45 ms, t90 74 ms, peak 1.14 at 110 ms, settles ~1.1 (a little biased) | **Not overdamped.** No slow crawl, a small overshoot, no oscillation. The expected "pitch D −20 %" is not supported. Slower than roll, which fits the camera's inertia. |
| Yaw | t90 64 ms, overshoots to 0.93, sags to ~0.7, creeps back | Lags, as p.103 describes. Its step (K, then I, then FF) comes after roll and pitch. |

**`IMU_DGYRO_CUTOFF` 40 did what the replay predicted.** This is the Stabilized hover 5–12.8 s
against Flight 1's 6–13 s. Only this window is free of the bridge.

| Quantity | Ratio |
|---|---|
| Motor noise above 40 Hz | ×0.78 / ×0.78 / ×0.84 / ×0.84 (now 1.43–1.73 %) |
| Torque noise, roll / pitch | ×0.78 / ×0.81 (predicted ×0.82) |
| Filtered rate noise, roll / pitch | ×0.95 / ×0.99 (unchanged, as it should be) |

Still above the SNR 15–20 target, as §9 expected. **Kept at 40.**

**RPM.** 5 570–15 400 in the air. It went below 100 Hz only at liftoff (4.5–4.7 s, 0.24 s). In
flight the minimum stayed above 110 Hz, so **`DNF_MIN` 100 stays.**

**Next.** Flight 3, with **the offboard bridge not running**:
- Wobbles in Stabilized (or Altitude), roll and pitch, to confirm pitch's P:D on clean data.
- If pitch confirms, the next change is §5.2 step 2: K, one axis per flight. Note that K scales D
  too: roll K ×1.2 would add ~20 % to roll's motor noise.
- §5.4 (vertical velocity) still needs its hard full-stick climbs and descents, in Altitude.

### 2026-09-21 — Flights 3 and 4 (20:12 wobbles, 20:13 up and down), read. No change

**⚠️ Neither flight had FAST-LIO in EKF2.**
- **What was missing.** No `ev_pos`, `ev_hgt` or `ev_yaw` in `estimator_status_flags`, and no
  `estimator_aid_src_ev_*` topics at all. Flight 2 had all three throughout.
- **What EKF2 flew on:**
  - baro and ToF for height;
  - optical flow for horizontal velocity;
  - the **compass for heading** (`cs_mag_hdg` / `cs_mag_3d`), the source §3.1 records as
    inconsistent here;
  - inertial dead reckoning in patches: 5 % and 1.4 % of the two flights.
- **What that makes these flights.** The first arms without LIO on record: PX4 allowed them, and
  Position mode ran on flow. `estimator_preflight.py` would have said NO-GO. Most likely the
  odom bridge was stopped along with the offboard bridge.
- **The offboard bridge was also off.** No `offboard_control_mode` at all. The attitude-setpoint
  flipping is gone: max 5° against Flight 2's 45°.
- **The logs.**
  - `/fs/microsd/log/2026-09-21/20_12_25.ulg` → `~/px4_logs/2026-09-21_20_12_25.ulg`
  - `20_13_44.ulg` → `~/px4_logs/2026-09-21_20_13_44.ulg`
  - `19_34_01.ulg` (22.6 MB) is on the card and was not read.

**Rate loop (20:12, wobbles in Altitude mode, clean data).** Setpoints reached 98 / 106 / 78 °/s.

| Axis | Estimated/setpoint peak ratio (p.88) | Step response | Reading |
|---|---|---|---|
| Roll | 1.01 (IQR 0.95–1.07), 20 peaks | settles 1.00, t90 59 ms, no overshoot | **Critically damped. Done.** |
| Pitch | 1.05 (IQR 0.99–1.09), 30 peaks | peak 1.03 at 95 ms, t90 58 ms, droops to ~0.9 | **Critical to a touch under.** Not overdamped: pitch D stays. +20 % would add pitch noise, the worst axis, for little. |
| Yaw | 1.04, 9 peaks | reaches 0.94, sags to 0.75, creeps back | **Lags on held inputs** (p.103). Next: `MC_YAWRATE_K`, then `_I`, then `_FF` 0.005. Yaw D is 0, so K costs no motor noise. |

**Vertical velocity (20:13, full-stick climbs and descents to 7.6 m, in Altitude mode).**
- **The data is coarse.** `MPC_POS_MODE` 4, so the setpoint is jerk-limited
  (`ManualAltitudeSmoothVel`), not the guide's Direct velocity. The logger keeps
  `trajectory_setpoint` at 5 Hz and `vehicle_local_position` at 10 Hz.
- **Steady climbs and descents track** within ~0.05 m/s, with std 0.01–0.04 m/s. Flight 2's
  descent wobble (std 0.15) is gone.
- **Every transition overshoots once** by 0.1–0.27 m/s (~20–25 % of the step), in both
  directions, and settles in ~0.5 s without ringing. That is mildly underdamped at the stock
  P 4 / I 2 / D 0. p.117 says P down a step, or (p.118's example) some D.
- **Not acted on:** flown on baro and ToF, not FAST-LIO. §3.1 says tune this loop on the
  follower's estimator. Re-fly with the stack up first.

**Why the height wandered (the user's question after Flight 2, confirmed on these flights).**
- **The mechanism.** PX4 locks altitude only while the throttle is within ±`MAN_DEADZONE` (0.1)
  of centre: `FlightTaskManualAltitude::_updateAltitudeLock`, `Sticks.hpp:70`. Outside it, the
  stick is a climb-rate command, with no lock.
- **The band is narrow.** RC3 is calibrated 1065–1932 µs, so zero is 1498.5 µs and the band is
  only 1455–1542 µs.
- **Where the stick actually rested:** 1422 / 1443 / 1525 / 1583 µs. A non-centring throttle
  can't find ±43 µs by feel.
  - 20:12: the stick was in the band 3 % of the Altitude time, and just below it (−0.10 to
    −0.13) 66 %.
  - 20:13: in the band, height held to 0.03–0.08 m. At −0.14 to −0.19 it sank 0.3–0.76 m, as
    commanded.
  - Flight 2: +0.23 for 50 s, a 0.18 m/s climb to 9 m.
- **Fix options (the user's call):**
  - fly with an eye on a centre mark;
  - a sprung or centre-detent throttle gimbal;
  - `MAN_DEADZONE` 0.1 → 0.2. It is shared by roll, pitch and yaw too (`Sticks.hpp:67–70`).

**`IMU_DGYRO_CUTOFF` 40 over longer windows.** Against Flight 1's hover, motor noise is −2 to −14 %
and torque noise −10 % roll, 0 to −11 % pitch. That is smaller than the −18 % seen in Flight 2's
8 s Stabilized window: the vibration input differs between flights. Kept at 40.

**Next.**
- **The stack up (LIO and odom bridge), the offboard bridge off.**
- One change: `MC_YAWRATE_K` ×1.2, with yaw wobbles.
- Re-fly the full-stick climbs and descents on the LIO estimator before touching `MPC_Z_VEL_*`.

### 2026-09-21 — four proposed changes, checked against the logs

The user asked about `MC_YAWRATE_I` 0.2, `MC_BAT_SCALE_EN` 1, `MPC_THR_HOVER` 0.42 and
`SENS_BAR_AUTOCAL` 0. **`MC_BAT_SCALE_EN` already read 1 on the FC** when checked. It was 0 in all
four flights and in `px4-params-2026-09-21-pre-retune.params`, so it was set after 20:13 UTC,
outside a dump.

- **`SENS_BAR_AUTOCAL` 1 → 0: yes.**
  - **What happened.** In Flight 3 the FC rewrote `CAL_BARO0_OFF` 470 → 519 Pa **in the air, at
    18.9 s**. The baro altitude stepped +4.2 m (207.5 → 211.8 m). EKF2 dropped baro fusion for
    1.7 s. Then its height rose 1.4 m while the ToF saw +0.4 m, so ~1 m of the step leaked into
    the height estimate. That flight had no FAST-LIO: baro was the height source.
  - **Why it can fire in flight.** The calibration runs once per boot, as soon as GNSS is good
    and the baro-to-GNSS offset looks stable (`VehicleAirData.cpp:260`,
    `BaroGNSSAltitudeOffset`). Nothing checks armed or landed.
  - **What 0 costs.** Nothing here: `EKF2_GPS_CTRL` is 0, and only relative baro height is used.
  - **Also:** this is why `CAL_BARO0_OFF` differs in every dump.
- **`MC_BAT_SCALE_EN` 1: yes (already set).** The rate controller multiplies thrust and torque
  by `battery_status.scale` (`MulticopterRateControl.cpp:241–253`).
  - **Without it:** the hover thrust estimate climbed 0.41 → 0.47 as the pack sagged, so the
    loops lost ~15 % of their authority by the end of a pack.
  - **The fit:** hover estimate ÷ PX4's scale stays at 0.40–0.42 in all four flights (scale
    1.01–1.17).
  - **The effect:** the whole pack now behaves like the full-pack Flights 1 and 3, which the
    tune was read on.
- **`MPC_THR_HOVER` 0.41 → 0.42: not needed.**
  - **Where it is used.** Only as the hover estimator's start value and Stabilized's fallback;
    Stabilized uses the live estimate when valid (`mc_att_control_main.cpp:101, 226`).
  - **With scaling on,** the estimator converges near 0.41, the pre-scale value. 0.42 is
    harmless but not what the data says.
- **`MC_YAWRATE_I` 0.1 → 0.2: optional, not shown to be needed.** **Correction to Flights 3–4
  above:** the "sag to 0.75" came from a deconvolved step response over 54 short windows.
  - **The direct data shows no sag.** On held yaw inputs, yaw reaches 0.83–0.98 of the setpoint
    by 80–200 ms, then runs slightly over (median 1.09, partly lag at the stick release).
  - **Nothing saturated.** The yaw integrator peaked at 0.026 of its 0.3 limit, and the allocator
    achieved 100 %.
  - **This flight cannot show I's effect.** The holds were 0.4–0.7 s, and I's time constant here
    is ~2 s (I/P = 0.5 s⁻¹).
  - **If it is tried,** fly 2–3 s constant-stick yaw holds so the log can show it.

### 2026-09-21 — the user set `MC_BAT_SCALE_EN` 1 and `SENS_BAR_AUTOCAL` 0

Both were made from QGC after Flight 4, and both were verified on the FC: read back, and saved
(`x +`). `MC_YAWRATE_I` (0.1) and `MPC_THR_HOVER` (0.41) were left unchanged, as recommended
above.

**The FC now stands at:**

| Parameter | Value | Since |
|---|---|---|
| `IMU_DGYRO_CUTOFF` | 40 | Flight 1 |
| `MC_BAT_SCALE_EN` | 1 | after Flight 4 |
| `SENS_BAR_AUTOCAL` | 0 | after Flight 4 |

Everything else is the §2 baseline. The dump is
`firmware/px4-params-2026-09-21-post-batscale-baroautocal.params`. Against the pre-retune dump,
only these three settings differ, plus PX4's own bookkeeping (`CAL_BARO0_OFF` now frozen at
543.2 Pa, accel offsets, flight counters).

**What to check in the next log:**
- **Battery scaling has not flown yet.** Check that `battery_status.scale` moves smoothly, and
  that the hover thrust estimate stays near 0.41 all pack long instead of climbing to 0.47.
  The rate loop at low battery will be up to ~15 % firmer than in Flights 2 and 4. That is what
  full-pack Flights 1 and 3 flew, and they were fine.
- No `CAL_BARO*` parameter should appear in the log's changed-parameters list.
