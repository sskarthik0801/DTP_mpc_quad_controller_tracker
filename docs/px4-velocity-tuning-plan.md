# Horizontal velocity loop: findings and tuning flight plan (2026-09-27)

**Nothing in this document has been flown, and no gain has been changed.** The FC still runs the
stock position-loop gains of `firmware/px4-params-2026-09-21-post-batscale-baroautocal.params`:
`MPC_XY_P` 0.95, `MPC_XY_VEL_P_ACC` 1.8, `MPC_XY_VEL_I_ACC` 0.4, `MPC_XY_VEL_D_ACC` 0.2.
Parameters are written by the user, in QGC, not by an agent.

**Status of the pieces around it:**
- The plan was accepted on 2026-09-27 in this order: **O1** (acceleration feedforward alone, stock
  gains), then the **manual ladder V0–V6**, then **O2/O3** (feedforward with the new gains, at
  3 and then 4 m/s).
- **The acceleration feedforward is ON by default in `px4_offboard_bridge` since 2026-09-27**.
  It is bench-gated (`m3_bridge_check.py` 39/39) and not flown. Still owed: a props-off look at
  `listener trajectory_setpoint` on the FC (`.claude/rules/px4-bridges-offboard.md`).
- The two flight blockers (the `uxrce_dds_client` hardfault soak, the MID360→FC yaw mount) are
  still open and are the user's call.

Labels used below: **[measured]** comes straight from logs; **[modelled]** comes from the
model in §3; **[assumed]** is an input nobody has measured. The scripts that reproduce every
number are in [`evidence/velocity-loop-2026-09-27/`](evidence/velocity-loop-2026-09-27/README.md).

---

## 0. The answer in brief

- **The stock velocity loop is slow, not oscillatory.** Velocity lags PX4's own velocity
  setpoint by ~0.55 s, which is 1/`MPC_XY_VEL_P_ACC`. It does not overshoot its setpoint. What
  overshot SUPER's reference on 2026-09-26 was the stored *position* error feeding the setpoint
  **[measured]**.
- **Most of the fix is the feedforward, not the gains.** On a rest-to-rest move at 3 m/s the
  model gives 1.31 m behind and 1.28 m past the goal without it, and 0.49 / 0.36 m with it
  **[modelled]**.
- **Tuning the velocity loop then roughly halves what is left:** 0.25 / 0.14 m at 3 m/s. That is
  0.1–0.3 m in absolute terms across 2–4 m/s. Disturbance rejection improves by ~40 %
  **[modelled]**.
  - At 2 m/s and below it is marginal. At 3 m/s and above it is worth the ~7 short flights.
- **Proposed end point:** `MPC_XY_VEL_P_ACC` ≈ 3.0, `MPC_XY_VEL_D_ACC` 0.1,
  `MPC_XY_VEL_I_ACC` ≈ 0.2 × P (≈ 0.6), `MPC_XY_P` unchanged at 0.95.
- **Do not go to the guide's example P of 4.0.** On a slower attitude loop it loses its margin,
  and a 2 m/s step would command 39° of tilt. The most ever flown here is 29° **[measured]**.

---

## 1. Sources, and what they cannot show

**The data:**
- **Bags** `bags/stage_b_2026-09-26_195108` and `_200650`, the two person-following Offboard
  flights (`.claude/rules/flight-log.md`).
  - Offboard windows: 31.5 s and 27.5 s.
  - `/fmu/in/trajectory_setpoint` and `/fmu/out/vehicle_local_position_v1` at 50 Hz, plus
    SUPER's `/planning/pos_cmd` and FAST-LIO's `/lidar_slam/odom`.
  - No attitude topic in these bags.
- **ULogs** `~/px4_logs/2026-09-21_*.ulg`, retune Flights 1–4 (`docs/px4-tuning-notes.md` §9).
  - Attitude and its setpoint at ~200 Hz.
  - Local position and its setpoint only at 10 Hz (the default logger profile).
- **PX4 source** `~/PX4-Autopilot` at `d6f12ad`, the v1.17 this FC runs.

**What the data cannot show:**
- **No horizontal velocity step has ever been flown on the LIO estimator** **[measured]**:
  - The ULogs' Position-mode segments are hover only (sticks ≤ 0.44, speed ≤ 0.85 m/s).
  - Flights 3–4 flew on optical flow, with no LIO.
  - **The IDLE-bridge confound.** Flight 2's Position segment (12.9–18.9 s) had the offboard
    bridge up. So did every Position/Altitude segment in both 09-26 bags (before and after
    Offboard). Its interleaved "hold here" setpoints (`.claude/rules/px4-bridges-offboard.md`)
    make them unusable, and the flight task's own setpoint is not in the bags anyway. They were
    excluded.
  - So **the rise time, overshoot and settling of the velocity loop in §4 are model outputs**.
- **The P ceiling** (the guide's fast micro-oscillation from the 50 Hz loop and velocity-estimate
  noise) is not in the model. Only a flight can find it.
- **Nothing above 2.6 m/s horizontal in Offboard and nothing above 29° tilt has been flown.** No
  constant speed above 0.9 m/s was held long enough to measure steady tracking.
- **Wind** on those days is unknown.

The Offboard windows themselves are clean: in TRACK the bridge is the only setpoint source.

---

## 2. What was measured

### 2.1 What PX4 tracks in Offboard (from source)
In v1.17 there is no Offboard flight task. `trajectory_setpoint` goes straight into
`PositionControl` (`MulticopterPositionControl.cpp:438–574`). In `PositionControl.cpp`:
- **The velocity setpoint** (`:127–135`) is `vsp = vel_ff + MPC_XY_P·(pos_sp − pos)`, clamped to
  `MPC_XY_VEL_MAX` with the position term first.
- **The acceleration command** (`:147–150`) is
  `acc = acc_ff + MPC_XY_VEL_P_ACC·(vsp − v) + I − MPC_XY_VEL_D_ACC·v̇`.
  - v̇ is the derivative of the EKF velocity through a 5 Hz low-pass (`MPC_VELD_LP`).
  - `MPC_VEL_LP` is 0, so the velocity itself is unfiltered.
- **The tilt limit** (`:215`) normalises `(−acc_x, −acc_y, g)` to `MPC_TILTMAX_AIR` 45°.
- **Anti-windup** (`:187–196`) applies only when saturated, with gain 2/P.

The bags carry the setpoint and state, so `vsp` and `acc` can be rebuilt sample by sample. The
integrator is rebuilt from zero plus an unknown start value.

### 2.2 The two Offboard windows, stock gains, no feedforward [measured]

| | Flight 1 (19:57) | Flight 2 (20:12) |
|---|---|---|
| Event | engage, walker 5.3 m away | FOLLOW sprint, +9 to +16.5 s |
| Reference peak / aircraft peak | 1.82 / 2.25 m/s | 1.92 / 2.57 m/s |
| Max behind / max ahead of the reference | 0.71–0.73 m / 0.81 m | 1.25 m / 0.52–0.53 m |
| Lag, `vsp` → velocity (cross-correlation) | 553 ms (r 0.963) | 553 ms (r 0.964) |
| \|vsp − v\| rms over the window | 0.53 m/s | 0.59 m/s |
| \|pos − pos_sp\| rms / p95 / max | 0.31 / 0.71 / 0.82 m | 0.40 / 0.95 / 1.27 m |
| Steady tracking at ~0.9 m/s | 22.5–27 s: along-track +0.02 to +0.07 m, \|vsp − v\| 0.02–0.14 m/s | 17.5–19.1 s: along +0.02 m, cross-track 0.03 m, speed −0.09 m/s |

- **Where the overshoot comes from.** In Flight 1's engage, `vsp` reached 2.46 m/s at +2.0 s:
  vel_ff 1.77 plus 0.95 × 0.72 m of lag. The velocity followed it half a second later. The
  velocity loop did not overshoot; the position term asked for more speed than the reference had.
- **Transport.** Local position reaches the bag 8 / 15 ms (median) after its PX4 stamp. The
  reconstruction uses PX4's stamps.
- **EKF2's flow speed limit** (`vxy_max`) was finite only on the ground before takeoff in
  Flight 2 (0.60 m/s at 0.15 m). It never applied in flight.

### 2.3 How fast an acceleration command arrives [measured]
**The attitude loop from the ULogs.** A first-order-plus-delay fit of tilt to tilt setpoint, in
6 s blocks with gain and offset free:

| Flight | Roll: tau / delay / R² | Pitch: tau / delay / R² |
|---|---|---|
| F1 hover | 0.175 s / 53 ms / 0.978 | 0.160 s / 40 ms / 0.980 |
| F2 (bridge up, still valid for this loop) | 0.185–0.215 s / 48–50 ms / 0.93–0.98 | 0.145–0.180 s / 50–60 ms / 0.67–0.99 |
| F3 wobbles (clean) | 0.160 s / 45 ms / 0.985 | 0.180 s / 55 ms / 0.996 |
| F4 (clean) | 0.185 s / 48 ms / 0.983 | 0.155 s / 62 ms / 0.965 |

Tilt maps to horizontal acceleration as g·tan(tilt), so this is the lag between PX4's `acc` and
the acceleration the aircraft makes.

**The same thing from the Offboard bags.** EKF2's `ax/ay` was fitted to the rebuilt command,
as `gain·FOPDT(acc) − drag·v + offset`:
- R² 0.89 and 0.87;
- the fit is a ridge along tau + delay: Flight 1 wants 0.22–0.26 s in total, Flight 2
  0.15–0.20 s;
- the ULog value (0.17 s + 0.05 s = 0.22 s) sits on both ridges, so it is the one adopted.

### 2.4 Drag and trim [measured]
- **Rotor drag.** Thrust lies along body z, so the accelerometer's body x/y reading is
  aerodynamic force. Regressed against body-frame velocity:
  - 0.24–0.34 s⁻¹ on Flight 3, the fastest (up to 3.2 m/s);
  - 0.30 s⁻¹ (x) on Flight 2;
  - the Offboard fit gave 0.17–0.38.
  - **Adopted 0.30 s⁻¹** (bracket 0.2–0.4). It is linear drag; above ~3 m/s real drag grows
    faster **[assumed]**.
- **A constant body-x trim.** Every flight hovers with **+0.44 to +0.55 m/s²** of body-x specific
  force (about 3° of thrust-axis or level offset). The velocity integrator carries it. Don't cut
  `MPC_XY_VEL_I_ACC` below ~0.2 × P.

### 2.5 SUPER's reference [measured]
- **Within its limits:** |acc| ≤ 2.00 m/s² (`max_acc` 2.0), jerk p50 2.3–2.7 and p95 6.4–7.6 m/s³.
- **Replan jumps.** In Flight 2 at +26.0 s, `pos_sp` jumped 0.41 m more than its velocity
  explains. No gain removes a jump in the reference.
- **Vertical reference:** 0.91 m/s down and 1.27 m/s up at most.

### 2.6 The angle loop, by the guide's p.110 check [measured, the reading is judgement]
On Flights 3–4's sharp wobbles, the estimated tilt peak reaches 0.69–0.90 of the setpoint peak
(median by axis), 160–225 ms later. The guide's table reads that as "raise angle P".

**But `MC_ROLL_P`/`MC_PITCH_P` 4.0 is v1.17's own default** (§8). Short wobbles also understate
any first-order loop. So this is a possible upstream lever, not a finding of a mis-tune: a faster
angle loop would let velocity P go higher at the same margin.

### 2.7 Headroom and battery [measured]
- **Motors:** outputs reached 0.63–0.90 at most, with no saturation. The collective thrust
  setpoint peaked at 0.64. Tilt peaked at 29° (Flight 3).
- **Current and consumption.** Hover draws ~30 A (6S, `BAT1_CAPACITY` 3300 mAh), with no rise up
  to 3.2 m/s (29–34 A). Hard manoeuvres peak at 48–53 A. That is ~500 mAh per minute.
- **Voltage sag.** The pack flown in Flights 1–2 reached 20.3 V loaded (3.38 V/cell) at
  ~2000 mAh delivered.
- **Low-battery action.** `COM_LOW_BAT_ACT` is 0: PX4 only warns.

### 2.8 Side finding: the MID360→FC yaw offset, indicatively [measured, indicative only]
- **The idea.** EKF2's `ax/ay` is the FC's IMU rotated by FAST-LIO's heading. FAST-LIO's own
  trajectory does not depend on how the MID360 sits on the FC. A yaw mount error ψ therefore
  shows up as a rotation between the two accelerations.
- **The result.** Over both armed flights it is **+1.6°**. The split halves give +1.7 to
  +2.8°, with 7–9 % residual.
- **A lower bound, not a measurement:** EKF2 can absorb part of a real offset through its
  position corrections. The blocker stays open.
- **What it would cost.** At ~2° the off-axis acceleration is ~3.5 % of the command, which is
  small next to drag. It is not what limits this tuning.

---

## 3. The model

**Controller.** PX4's `PositionControl` as in §2.1, per horizontal axis in vector form:
- 50 Hz update with a zero-order hold **[assumed]**: EKF2 runs at 100 Hz, and 50 Hz is the
  conservative choice;
- D on the derivative of the velocity through a 5 Hz first-order low-pass;
- the tilt limit applied by vector normalisation, and PX4's anti-windup.

**Plant.**
```
a_thrust = 0.95 · FOPDT(acc_produced; tau = 0.17 s, delay = 0.05 s)
a        = a_thrust − 0.30 · v            (+ an optional constant disturbance)
```
**[assumed]** in it: no wind; a perfect velocity estimate; perfect, in-sync feedforward; x and y
decoupled; altitude held; a linear thrust model (`THR_MDL_FAC` 0, unverified).

**Brackets used throughout:**
- fast: tau 0.12 s / 0.03 s;
- adopted: 0.17 / 0.05;
- slow: 0.22 / 0.08;
- drag 0.2–0.4.

**Validation [modelled against measured].** The model was driven by the recorded setpoint
stream, starting from the measured state, and compared with the flights:
- Velocity error over both whole windows: **0.098 m/s rms** for the adopted model. The
  output-error best (tau 0.17, delay 0.03, drag 0.30) scores 0.097; the same model without drag
  scores 0.152.
- Flight 1 engage: modelled 0.84 m behind / 0.75 m ahead / 2.30 m/s peak, against 0.73 / 0.81 /
  2.25 measured (on a 1 ms grid; 0.71 behind on the 50 Hz samples).
- Flight 2 sprint: modelled 1.29 / 0.32 / 2.66, against 1.25 / 0.53 / 2.57.
- Within ~0.1–0.2 m. Validated only to 2.6 m/s, 2 m/s² and 29° of tilt.

**Stability margins.** From the linearised loop (plant above, half-sample delay, D filter): the
inner velocity loop's phase margin, and the position loop's with the inner loop closed.

---

## 4. Predictions [modelled]

### 4.1 The velocity loop on its own: a 0 → 2 m/s step (what Direct-velocity mode flies)

| Gains P / XY_P / I / D | Rise 10–90 % | Overshoot | Settle ±5 % | Inner PM (adopted / slow plant) | Position PM |
|---|---|---|---|---|---|
| stock 1.8 / 0.95 / 0.4 / 0.2 | 1.17 s | 0 % | 1.8 s | 82° / 75° | 57° |
| 1.8, D 0.1 | 1.02 s | 0 % | 1.6 s | 77° / 70° | 58° |
| 2.5 / 0.95 / 0.5 / 0.1 | 0.65 s | 0 % | 0.95 s | 69° / 61° | 66° |
| **3.0 / 0.95 / 0.6 / 0.1** | **0.51 s** | **2.9 %** (slow plant 10.7 %) | **0.75 s** | **64° / 55°** | **69°** |
| 3.0 / 0.95 / 1.0 / 0.1 | 0.48 s | 7.7 % (slow 16 %) | 1.4 s | 61° / 52° | 68° |
| guide's 4.0 / 0.95 / 0.4 / 0.05 | 0.37 s | 9.8 % (slow 20 %) | 1.9 s | 55° / 45° | 75° |
| 3.0 with `MPC_XY_P` 1.2 | 0.51 s | 2.9 % | 0.75 s | 64° | 63° |

Stock is overdamped, as the guide says (p.127). The biggest lever is P, not D.

### 4.2 Rest-to-rest moves like SUPER's (2 m/s², jerk 8): m behind / m past the goal

| Speed | Stock, no feedforward (as flown) | Feedforward, stock gains | Feedforward, 3.0 / 0.95 / 0.6 / 0.1 | Feedforward, guide's 4.0 |
|---|---|---|---|---|
| 2 m/s | 0.96 / 0.94 | 0.36 / 0.27 | 0.22 / 0.15 | 0.16 / 0.05 |
| 3 m/s | 1.31 / 1.28 | 0.49 / 0.36 | 0.25 / 0.14 | 0.18 / 0.07 |
| 4 m/s | 1.55 / 1.52 | 0.61 / 0.46 | 0.30 / 0.18 | 0.24 / 0.10 |

- **Cruise lag from drag**, at the start of braking, with feedforward at 3 / 4 m/s: stock 0.20 /
  0.25 m; tuned 0.14 / 0.18 m.
- **After the stop, stock gains creep.** The integrator holds the cruise's drag compensation and
  unwinds slowly (P/I ≈ 4.5 s): still 0.16 m off 3 s after a 3 m/s stop. With the tuned set it is
  0.11 m.
- **Peak speed** with feedforward exceeds the reference by 0.13–0.36 m/s. Without feedforward it
  is 0.65–1.04 m/s too high.
- **Sensitivity.** Across the bracket (tau 0.12–0.22 s, drag 0.2–0.4), the tuned set's 3 m/s
  result spans 0.19–0.33 m behind / 0.12–0.18 m past. Stock spans 0.38–0.62 / 0.30–0.44. The
  halving holds everywhere.

### 4.3 Replaying the two flown references with feedforward (from `pos_cmd`)

| | Flight 1, whole window: error rms / p95 / max | Flight 2, the same |
|---|---|---|
| As flown (measured, no feedforward) | 0.31 / 0.71 / 0.82 m | 0.40 / 0.95 / 1.27 m |
| Model, no feedforward, stock (the check) | 0.32 / 0.74 / 0.87 m | 0.36 / 0.86 / 1.31 m |
| Model, feedforward, stock | 0.10 / 0.25 / 0.33 m | 0.15 / 0.33 / 0.47 m |
| Model, feedforward, 3.0 / 0.95 / 0.6 / 0.1 | 0.07 / 0.13 / 0.21 m | 0.11 / 0.20 / 0.48 m |

Flight 2's max stays at ~0.48 m for every gain set: that is the 0.41 m replan jump in the
reference (§2.5).

### 4.4 A disturbance: a constant 1 m/s² push from hover (a gust, or unmodelled drag)
Peak excursion:
- stock: 0.51 m;
- 2.5 / 0.5 / 0.1: 0.36 m;
- **3.0 / 0.6 / 0.1: 0.29 m**;
- `MPC_XY_P` 1.2: 0.26 m.

### 4.5 An option, not built: drag feedforward in the bridge
`acc_ff = a_ref + 0.3·v_ref`:
- **With stock gains** it removes the cruise lag (3 m/s: 0.20 → −0.05 m). It does not remove the
  braking overshoot, which is the attitude lag and only P fixes (0.36 → 0.36 m).
- **With the tuned set:** ~0.19 m behind / 0.11–0.15 m past at 4 m/s.
- **If the true drag differs from 0.3,** the aircraft leads or lags by ~0.15 m at cruise.

It is the user's call, and it would need a bridge change and `m3_bridge_check.py`.

### 4.6 How much do the gains matter once feedforward is on?
Honestly, moderately:
- **The feedforward removes ~65–70 % of the error;** the gains remove about half of the rest.
- **Absolute gain:** 0.1–0.15 m at 2 m/s and 0.3 m at 4 m/s, plus ~40 % better gust rejection.
- **For a follower at a 3–4 m standoff,** 0.3 m is ~10 % of the standoff. Worth doing before
  flying at 3 m/s and above. Not urgent at 2 m/s and below.

---

## 5. Recommendation

1. **O1 first:** feedforward with stock gains, in Offboard. One change, and it measures the
   feedforward alone against the model.
2. **Then the manual ladder:**
   - D 0.2 → 0.1;
   - P up in ×1.2 steps: 1.8 → 2.2 → 2.6 → 3.1, with 3.7 only as a probe;
   - I restored to 0.2 × P.

   Expected end point: `MPC_XY_VEL_P_ACC` ≈ 3.0–3.1, `MPC_XY_VEL_D_ACC` 0.1,
   `MPC_XY_VEL_I_ACC` ≈ 0.6.
3. **Then O2/O3:** feedforward with the new gains, at 3 m/s, then 4 m/s.

**Leave unchanged:**
- **`MPC_XY_P` stays 0.95.** 1.2 gains 0.01–0.03 m and costs 6° of position margin; the guide
  found 0.5–1.6 all fine (p.137).
- **`MPC_XY_VEL_D_ACC` not below 0.1.** That is v1.17's listed minimum; the guide's 0.05 or 0
  need a force save, and the model finds no benefit.
- **`MPC_XY_VEL_I_ACC` not above ~0.2 × P.** At P 3, I 1.0 halves the cruise lag (0.08 m) but
  lifts overshoot to 8–16 %.

**Options, the user's call:**
- drag feedforward in the bridge (§4.5);
- angle P above 4.0 (§2.6), flown in Stabilized/Altitude with wobbles, before the ladder,
  because it moves the ladder's optimum;
- a lower `MPC_XY_VEL_MAX` (§6).

---

## 6. Limits as speed rises (v1.17 `d6f12ad`)

| Limit | Applies in Offboard? | Binds? |
|---|---|---|
| `MPC_ACC_HOR` 3.0, `MPC_JERK_MAX` 8.0, `MPC_XY_ERR_MAX` 2.0, `MPC_ACC_HOR_MAX`, `MPC_ACC_UP/DOWN_MAX` | **No.** They are read only by GotoControl (`MulticopterPositionControl.cpp:203–208`, used only for a `goto_setpoint` with no `trajectory_setpoint`), the manual tasks (`StickAccelerationXY.cpp:46,98,160`; `ManualAltitudeSmoothVel.cpp:94`), Auto/Orbit (`FlightTaskAuto.cpp:780–787`) and collision prevention. | Not in Offboard. In Offboard, SUPER's `max_vel`/`max_acc` are the only shaping. In Position mode (`MPC_POS_MODE` 4) they shape the stick, which is why the ladder uses Direct velocity. |
| `MPC_TILTMAX_AIR` 45° | Yes (`PositionControl.cpp:215`) | Not on SUPER-shaped moves: ~18–20° at 2 m/s² plus drag, ~7° cruising at 4 m/s. **It binds on setpoint jumps:** the first command after a jump is P·XY_P·jump. A 3 m jump gives 5.1 m/s² (28°) at stock and 8.5 m/s² (41°) at P 3.0. Higher P makes the unbuilt far-setpoint refusal in the bridge matter more. |
| `MPC_XY_VEL_MAX` 12 m/s | Yes, on the total `vsp` (`PositionControl.cpp:135`, `MulticopterPositionControl.cpp:536–545`) | Never in practice. It is the only speed cap in Offboard. **Tightening it to ~5–6 m/s is worth the user's consideration.** It also caps Position mode. |
| `MPC_Z_VEL_MAX_DN` 1.5 / `_UP` 3.0 | Yes: Offboard forces them (`MulticopterPositionControl.cpp:488–490`), and they clamp `vsp_z` (`PositionControl.cpp:137`) | SUPER's `max_vel` is a 3-D norm (`magnitudeBounds`, `exp_traj_optimizer_s4.cpp:780`). At 4 m/s, any path descending more steeply than ~22° asks for more than 1.5 m/s down: the clamp holds, the height lags, and the position term cannot correct it. Flown so far: 0.91 down, 1.27 up. **Keep the clamp** (vortex ring state); fix it on the planner side if it binds. |
| EKF2 `vxy_max` | Yes (`MulticopterPositionControl.cpp:538–539`) | Only when optical flow is the sole horizontal aid (`ekf_helper.cpp:363–383`: 0.5 × flow rate limit × height), i.e. if LIO drops out. |
| Thrust | — | Not binding. Hover 0.41; 45° needs 0.58 (linear model); outputs peaked at 0.90; `MPC_THR_XY_MARG` 0.3. |
| Battery | — | ~500 mAh/min, 3.38 V/cell loaded at ~2000 mAh, warn-only action. **≤ 3.5 min airborne per full pack.** |
| Space | — | A 4 m/s rest-to-rest leg at 2 m/s² needs 8 m just to reach speed and stop. |
| **IDLE-bridge interference** | Manual modes with the bridge up | Its braking samples are multiplied by `MPC_XY_VEL_P_ACC`, so **P 3.0 makes it ~1.7× harsher,** including the manual takeover from Offboard, until the bridge is fixed. |

---

## 7. The flight plan

### 7.0 Standing rules, every flight
- **Dump params before any change,** and again at the end of the session:
  `python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F)-pre-veltune.params`.
- **`python3 scripts/px4/estimator_preflight.py` must say GO** at the take-off spot before every
  arm.
- **The stack:** Livox driver, FAST-LIO and `px4_odom_bridge`, exactly one of each. First check
  the sensor chain: `ip -brief link show eno1`, `ping 192.168.1.154`.
- **The offboard bridge stays STOPPED for every manual flight** (V0–V6). Check it by argv
  basename (`sysinfo.find_by_cmdline`), never `pgrep -f`.
- **Before a flight day:** `logloader` disabled, at least 20 GB free, nobody on
  `/cloud_registered`.
- **Gains** are changed by the user in QGC's PID Tuning page over UDP 14550. One change per flight.
- **Logs, every flight:**
  - the ULog, afterwards with `scripts/px4/fetch_ulog.py`;
  - for the manual flights, a bag for 50 Hz velocity. The logger keeps local position at only
    10 Hz. Confirm the topic names with `ros2 topic list`, then:
    `ros2 bag record -o bags/veltune_$(date +%F_%H%M%S) /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_attitude /fmu/out/manual_control_setpoint /fmu/out/vehicle_status_v1 /fmu/out/battery_status /lidar_slam/odom`;
  - the Offboard flights use the `stage_b` preset.
  - Never read a `.db3` while it is recording. Run `sync` before pulling the battery.
- **Battery:** ≤ 3.5 min airborne per full pack; land by ~21.5 V loaded.
- **Space:** at least 30 m of clear straight line; fly at 2.5–3 m.
- **The first flight of the session is also `MC_BAT_SCALE_EN` 1's first flight.** Check that
  `battery_status.scale` moves smoothly and the hover estimate stays near 0.41.

### 7.1 O1: feedforward on, stock gains, Offboard (the feedforward alone)
- **Before it:**
  - the props-off look at `listener trajectory_setpoint` on the FC (acceleration finite in TRACK,
    NaN in IDLE/HOLD);
  - `m3_bridge_check.py` 51/51 (39/39 before the 2026-09-28 Offboard-only setpoint change).
- **Stack:** on the **follow config** (`follow_test_ros2.yaml`: `max_vel` 3.0, `max_acc` 2.0;
  the click config caps SUPER at 0.1 m/s), plus both bridges. **The DTP follower is not running:**
  the GCS card refuses to send goals when someone else publishes them.
- **⚠️ Without the follower there is no `command_mux` engage gate.** SUPER plans as soon as it
  gets a goal. **Engage Offboard first** (the bridge holds position), **then** send goals. A goal
  sent before the switch recreates the 09-21 3 m engage jump.
- **Input:** level legs of 10 m (the GCS card's limit), forward and back, 3–4 legs. Keep the
  goals at or below 2.25 m in `lidar_init` (SUPER snaps higher goals down).
- **Predicted, 3 m/s legs:** ≤ 0.49 m behind, ≤ 0.36 m past each goal, peak speed ≤ ~3.2 m/s.
  As flown without feedforward, the model gives 1.3 m for both.
- **Stop rule.** Take over on channel 11 if any of these happens:
  - the aircraft overshoots a goal by more than 0.7 m (2× the prediction);
  - it oscillates about a goal;
  - it accelerates the wrong way at a leg's start (a feedforward sign error).

  Then fly on with feedforward off (`publish_acceleration_ff:=false`) until it is understood.

### 7.2 The manual ladder V0–V6, Position mode, offboard bridge stopped
**Setup, once, logged on the session sheet** (guide p.123–124):
- `MPC_POS_MODE` 4 → **0** (Direct velocity: the stick is the velocity setpoint, no slew,
  `FlightTaskManualPosition`). This also makes Altitude mode Direct velocity vertically.
- `MPC_VEL_MANUAL` 10 → **2.0** (QGC force save: below the listed minimum of 3).
- `MPC_XY_VEL_I_ACC` 0.4 → **0.2**.

**The manoeuvre:**
- Full stick left for ≥ 2 s, so the speed holds for ≥ 1 s. Centre the stick fully and wait ~2 s
  for the stop and the position lock (below `MPC_HOLD_MAX_XY` 0.8 m/s). Then full stick right.
- Then forward and back. 4–6 legs per axis, ~90 s airborne.
- **Never reverse directly.** A 4 m/s swing saturates the tilt at the higher P values.

**How to score each flight, before the next:**
- The velocity setpoint is `vehicle_local_position_setpoint` (ULog, 10 Hz), or the stick ×
  `MPC_VEL_MANUAL` (40 Hz); the velocity is the bag's 50 Hz local position.
- Read the **first second** of each step: rise 10–90 %, first peak, oscillation. Check the
  attitude setpoint for a flat top at 45°.
- **Do not read the tail.** Under I 0.2 plus drag, the velocity creeps to the setpoint over ~9 s.
  That looks exactly like the guide's "overdamped" sign, but it is drag.

| Flight | The one change | Predicted on a 2 m/s step: rise / overshoot / commanded tilt (slow plant) | Stop rule |
|---|---|---|---|
| V0 | setup only | 1.66 s / 0 % / 20° (1.30 s / 0 %) | Any overshoot or oscillation contradicts the model: stop and read the log before going on. |
| V1 | `MPC_XY_VEL_D_ACC` 0.2 → 0.1 | 1.40 s / 0 % (1.06 s / 0 %) | Overshoot or bounce → D back to 0.2, and fly the P steps with 0.2. |
| V2 | `MPC_XY_VEL_P_ACC` 1.8 → 2.2 | 0.92 s / 0 % / 24° | Rule for V2–V5: overshoot above ~5 %, or a bounce → back one step; that is the end point. |
| V3 | P → 2.6 | 0.68 s / 0 % / 28° (0.61 s / 0.9 %) | Fast "fur" on velocity or attitude that D doesn't fix → P down ≥ 30 % (guide p.129) and land. |
| V4 | P → 3.1 | 0.53 s / 0 % / 32° (0.49 s / 6.3 %); the first flight above 30° of tilt | New motor chatter → land immediately. A flat-topped tilt setpoint → re-fly at `MPC_VEL_MANUAL` 1.5 before judging. |
| V5 | P → 3.7, only if V4 was clean | 0.42 s / 3.6 % / 37° (0.41 s / 12.7 %) | Expected to show overshoot. Keep the last clean P. |
| V6 | `MPC_XY_VEL_I_ACC` 0.2 → 0.2 × final P (≈ 0.6 at 3.1) | 0.49 s / 3.5 %, settles in 0.73 s; the creep is gone | Overshoot above 10 %, or oscillation → I back to 0.4. |

**The guide's last step** (scale P, I and D together ×1.2) would reach P 3.7. Fly it only if V5
was clean.

**Position P (§5.6):** fly nothing unless V6's position hold drifts after the stick is centred.

### 7.3 After the ladder
- **Restore** `MPC_POS_MODE` 4 and `MPC_VEL_MANUAL` 10. Keep the new P/I/D.
- **One short Position-mode flight for feel:** acceleration-based mode now runs on a sharper loop.
- **Dump params.** Record the end values, dated with the dump's filename, in
  `.claude/rules/px4-estimator.md` and the tuning notes' §9.

### 7.4 O2 and O3: feedforward with the new gains, Offboard
The same legs and rules as O1. O3 raises `max_vel` to 4.0 in SUPER's follow config: the user's
call, under the staged-envelope rule in that file.
- **Predicted:**
  - O2 at 3 m/s: 0.25 m behind / 0.14 m past;
  - O3 at 4 m/s: 0.30 / 0.18. A 10 m leg only just reaches 4 m/s (~0.5 s at speed).
- **Stop rule:** as O1, with 2× the new prediction.
- **Before any manual flight afterwards,** stop the offboard bridge.

---

## 8. Corrections to other notes (2026-09-27)
- **`MC_ROLL_P`/`MC_PITCH_P` 4.0 is stock in v1.17, not "lowered".** Upstream commit
  `c7ef12545c` (2025-06-04) cut the default from 6.5 to 4: "6.5 … was used mostly on smaller, low
  inertia vehicles … on larger, higher inertia vehicles this leads to problems".
  `docs/px4-tuning-notes.md` §2 and `.claude/rules/px4-estimator.md` said "stock 6.5".
- **The "~20–25 % overshoot on every transition" in the tuning notes is the vertical loop**
  (`MPC_Z_VEL_*`, Flight 4). It was flown on baro and ToF, with no LIO. It says nothing about the
  horizontal loop, which the model finds overdamped (0 % on a step).
- **The guide's horizontal D examples (0.05, 0.0) are below v1.17's listed minimum** for
  `MPC_XY_VEL_D_ACC` (0.1).

---

## 9. Reproducing
[`evidence/velocity-loop-2026-09-27/README.md`](evidence/velocity-loop-2026-09-27/README.md):
`extract_bags.py` (needs ROS sourced), then the measured scripts, then `model_validate.py` and
`model_predict.py`. The ULog scripts need the PX4 venv's `pyulog`. Everything is read-only.
