# Durable fact — `MPC_ALT_MODE`, and why Position mode did not climb over a raised surface

**Established 2026-09-17**, when `MPC_ALT_MODE` was changed from 2 to 1 at the user's instruction.
The diagnosis below comes from the FC's own flight log and from the PX4 source this FC runs
(`~/PX4-Autopilot`, `d6f12ad`, v1.17.0). **Mode 1 has not been flown yet.**

Source:
- `src/modules/mc_pos_control/multicopter_altitude_mode_params.c` (the parameter);
- `src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp`
  (`_updateAltitudeLock`, `_terrainFollowing`);
- `src/modules/ekf2/EKF/aid_sources/range_finder/range_height_control.cpp`.

---

## 1. The report

In Position mode, the drone did not rise when it was flown over a raised surface, although it
carries a flow sensor. The sensor is an **ARK Flow**: DroneCAN node 124 publishes both the flow and
the range. It combines a PAW3902 optical-flow chip with an AFBR-S50LV85D time-of-flight rangefinder
(12.4° × 6.2° beam, up to 30 m).

## 2. What each setting means

The flow chip gives only horizontal velocity. The rangefinder gives the gap to what is directly
below. Neither sets the height the drone holds in Position mode. That is decided by `MPC_ALT_MODE`,
which applies to Position and Altitude mode only (both run `FlightTaskManualAltitude`):

| Value | PX4 name | Height is held relative to |
|---|---|---|
| 0 | Altitude following | the local-frame origin (here FAST-LIO's `lidar_init`, via `EKF2_HGT_REF=3`) |
| 1 | Terrain following | the ground, always. It falls back to 0 if the distance-to-ground estimate goes invalid |
| 2 | Terrain hold (**PX4's default**) | the ground when stationary, the origin when moving |

**Mode 2 in detail:**
- **Hold engages** with the roll/pitch sticks centred, horizontal speed below
  ½ `MPC_HOLD_MAX_XY` (0.4 m/s) and vertical speed below `MPC_HOLD_MAX_Z` (0.6 m/s).
- **Hold releases** on any stick input, or above `MPC_HOLD_MAX_XY` (0.8 m/s).
- **The gap it holds is the one at the moment it engages.** It never climbs back to an earlier gap.
- **Switching between the two is smoothed**, so the setpoint does not jump.

**What mode 2 is for:** barometer-referenced drones. A baro drifts, so a low hover creeps; holding
the measured gap stops that. Meanwhile cruising ignores the rangefinder, so the drone does not bob
over bushes and cars. Here the height comes from FAST-LIO, which does not drift like a baro, so that
benefit is small. The case the user cared about, crossing onto a raised surface, happens while
moving, which is exactly when mode 2 ignores the sensor.

**The estimator does not do it either.** `EKF2_RNG_CTRL=2` fuses the range, but a drop in range is
absorbed by EKF2's *terrain* state, not its altitude. On the flight below, range innovations had a
median of 0.01 m and a maximum of 0.19 m: EKF2 believed both the rangefinder and FAST-LIO.

## 3. The flight that shows it

FC log `/fs/microsd/log/2026-09-16/00_09_45.ulg` (= 2026-09-15 20:10 EDT, 44 s). The drone took off
in Stabilized, switched to Position at 11.8 s and to Altitude at 18.2 s. It was never in Offboard.

- **Throttle:** the stick sat at +0.07 throughout, inside `MAN_DEADZONE` 0.1. The altitude setpoint
  was therefore locked the whole time: 0.36 m in Position mode, then re-locked at the sagged
  0.22 m on the switch to Altitude mode.
- **Crossing onto the surface (21–24 s):** speed was 1.5–2.5 m/s. The range reading fell
  0.74 → 0.18 m while the altitude stayed at its setpoint.
- **Over the surface:** the drone flew with 0.10–0.25 m of range reading. It landed at +0.13 m in
  `lidar_init`, having taken off at −0.25 m, so the surface is **~0.4 m above the takeoff spot**.
  That height is inferred from FAST-LIO and the rangefinder; nobody measured it.
- **Terrain hold:** replaying PX4's mode-2 logic on the logged sticks and velocity, with
  `MAN_DEADZONE` applied, shows it active **~19%** of 12–36 s: 12.1–12.5 s, 17.5–18.6 s and
  32.8–36.0 s. It was **never** active while crossing the edge. When it engaged over the surface,
  it held the 0.10 m gap the drone had then.
- **Also in the log:** at full stick the altitude rose +0.27 m while accelerating
  (0.36 → 0.63 m) and sagged −0.14 m while braking. All of this happened within 0.6 m of the
  ground.

**How the log was read:**
1. **Download.** Use MAVLink `LOG_REQUEST_LIST` / `LOG_REQUEST_DATA` over UDP 14569, matching the
   file by byte size from `ls -l /fs/microsd/log/<date>` in `px4_shell.py`. It took about 50 s for
   28 MB. `logloader` is disabled, so FC logs no longer reach flight-review by themselves.
2. **Parse.** `pyulog` is in `~/PX4-Autopilot/.venv`.
3. **Trap:** some topics carry a first sample from before the log's start. Subtract timestamps as
   floats, or the unsigned subtraction wraps.

## 4. The change

`MPC_ALT_MODE` 2 → 1, set over MAVLink with the vehicle disarmed. It was verified by read-back,
and `param show` marks it saved (`+`). Restore point:
`firmware/px4-params-pre-altmode-2026-09-17.params`. To undo, set 2 or load that file.

**What it affects:**
- **Offboard is untouched.** v1.17 has no Offboard flight task, and SUPER's setpoints are absolute
  in `lidar_init`. SUPER clears terrain from ROG-Map.
- **Do not use `EKF2_HGT_REF` = range instead.** It would make EKF2's z terrain-relative for every
  mode, so Offboard setpoints would be off by the terrain height under the aircraft.

**What mode 1 costs:**
- **It cannot see an edge coming.** The beam covers ~22 × 11 cm at 1 m, directly below. A step
  taller than the current gap hits the props before the sensor is over it, so fly at ≥ 1 m.
- **It drops by the step height** when flying off an edge.
- **It climbs over anyone walking underneath.** That matters on a person-following aircraft,
  though only in the manual modes.
- **Tall grass reads as ground.**

## 5. Still open: `EKF2_MIN_RNG` is 0.01 and should be ~0.15

PX4 describes this parameter as the expected range reading on the ground. Two things interact:
- **On the ground,** when the reading is invalid, PX4 substitutes `EKF2_MIN_RNG` and skips the
  0.14 m `EKF2_RNG_POS_Z` offset. Right now the ToF reads 0.006 m, below `UAVCAN_RNG_MIN` 0.01.
- **In the air,** it uses range + 0.14 m.

So the EKF's height above ground reads 0.01 m before takeoff but 0.15 m on the ground after
landing (flight at 20:09). That is a ~14 cm jump in the ground estimate at takeoff, which mode 1
will act on. **Not changed; the user's call.**
