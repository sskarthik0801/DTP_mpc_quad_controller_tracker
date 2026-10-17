# Plan — aim the camera from where the aircraft is (fix 1) and feed its turn rate forward (fix 2)

**Written 2026-09-29, for a separate build session. Nothing here is built or flown.** Branch at
the time: `nmpc-follow-goal`. Read this whole file before touching code. Then read
`.claude/rules/px4-bridges-offboard.md` (the bridge and its traps) and the "Measured 2026-09-29"
section at the top of `.claude/rules/dtp-follow-behaviour.md` (the evidence).

## 0. In one paragraph
The camera trails the person by ~0.8 s. When the person was far off-centre, 84–98 % of the error
came from **SUPER's heading command**, not from PX4 or the person prediction. Under
`yaw_mode 3`, SUPER aims the nose at `/planning/yaw_target` from where its trajectory **ends**,
and gets there along one smooth polynomial that it re-solves at every replan. The fix:
- the **offboard bridge** computes the heading itself, straight from the aircraft's current
  position to `/planning/yaw_target` (**fix 1**);
- optionally, it sends that heading's rate to PX4 as `yawspeed` (**fix 2**).

SUPER still plans and flies the position. The replay on the 09-26 20:12 flight showed:
- person off-centre, p95: 16.8° → 4.2° (fix 1) → 1.8° (fix 1 + 2);
- turning while following: at most 8°/s against 87°/s as flown.

Both parts go behind switches, default **off**, with hard rate limits.

## 1. Evidence (don't re-derive)
- Tools, both read-only on finished bags:
  - `drone-dtp/scripts/yaw_chain_report.py <bag>…` splits the off-centre angle into aim /
    SUPER / PX4;
  - `drone-dtp/scripts/fix1_yaw_replay.py <bag>…` replays fix 1 and fix 1 + 2 through a PX4 yaw
    model fitted on the same flight.
- The replay's numbers, from `.claude/rules/dtp-follow-behaviour.md`:

  | p50 / p95 / max | as flown | fix 1 | fix 1 + 2 |
  |---|---|---|---|
  | `stage_b_2026-09-26_200650` | 2.1 / 16.8 / 20.6° | 0.3 / 4.2 / 5.0° | 0.4 / 1.8 / 5.0° |
  | `stage_b_2026-09-26_195108` | 7.4 / 16.9 / 20.0° | 1.6 / 8.5 / 11.0° | 1.1 / 5.7 / 7.1° |

- **How PX4 uses what we send (v1.17 `d6f12ad`):**
  - In Offboard, `trajectory_setpoint.yaw` and `.yawspeed` go straight to the attitude
    controller (`PositionControl.cpp:105,269`).
  - The controller's yaw rate = `MC_YAW_P`·error + `yawspeed` (`AttitudeControl.cpp`). The
    yaw weight 0.4 is divided back out of the gain (`setProportionalGain`), so for small errors
    the gain is 2.8 s⁻¹. The result is capped at `MC_YAWRATE_MAX` 200°/s.
  - `MPC_YAWRAUTO_MAX` does **not** apply in Offboard.
- **Why the lagged heading is SUPER's** (`super_planner.cpp` ~795, `yaw_traj_opt.cpp`): the
  heading aims from the trajectory end, and one polynomial runs from the start yaw to the end
  yaw. SUPER's commanded `yaw_dot` never went above 23°/s, though `yaw_dot_max` is 57°/s.
- **The dangerous moments were after a loss, not during the follow.** After the 20:12 loss the
  yaw target went stale, and all variants fell back to SUPER's swing (94–194°/s modelled). The
  2026-09-26 yaw hold keeps the yaw target alive through a loss. The servo must aim at that held
  point, and must rate-limit any fall-back to SUPER's heading.

## 2. Design

### 2.1 Where
- **In `super_bringup/scripts/px4_offboard_bridge.py`,** as a stage that rewrites only `yaw` and
  `yawspeed` of the setpoint just before it is published.
- It needs no SUPER change and no `follow_planner` change.
- **The algorithm goes in one pure module with no ROS dependency,** e.g.
  `super_bringup/scripts/yaw_servo.py`. The bridge and `fix1_yaw_replay.py` both import it, so
  the replay gate tests the code that flies. There must not be two copies.
  - `super_bringup/CMakeLists.txt` installs scripts by name (`install(PROGRAMS …)`), so add the
    module there.
  - The installed scripts are symlinks into `~/super_ws/src/super_bringup/scripts/`, and Python
    resolves the symlink for `sys.path[0]`. Check that the installed bridge can import the
    module.
- **`super_bringup/` is COPIED, not symlinked.** After editing, re-copy and rebuild:
  `rm -rf ~/super_ws/src/super_bringup && cp -r super_bringup ~/super_ws/src/ && (cd ~/super_ws
  && colcon build --packages-select super_bringup --symlink-install)`. Then restart the bridge.
  Never run two `colcon build`s at once.

### 2.2 Inputs
- `/planning/yaw_target`, `geometry_msgs/PointStamped`, in `lidar_init` (FLU, z up). It is a new
  bridge parameter `yaw_target_topic`, so that `m3_bridge_check.py` can remap it.
  - `follow_planner` publishes it directly. It does **not** pass through `command_mux`, so the
    engage gate does not hold it back.
  - **In observe-only mode it is `/dtp/shadow/yaw_target`** (`follower.launch.py:135`). There
    the bridge never sees a target, and the servo stays inactive by construction. Keep it that
    way.
  - **Its stamp is the perception stamp**, copied verbatim by `target_manager`. The yaw hold
    stamps it with the current time.
- The aircraft's position: the `vehicle_local_position` the bridge already has (NED, and EKF2
  holds it within ~0.08 m of FRD(odom), estimator preflight E6).

### 2.3 The algorithm, per 50 Hz tick
Let `ψ_super` = SUPER's heading as the bridge sends it today (`−msg.yaw`, FRD), and `ψ_out` the
heading sent last tick.
1. **Is the servo active?** All of these must hold:
   - `yaw_servo_enable` is on;
   - the bridge is **engaged** (armed and in Offboard) and in **TRACK or HOLD**, never IDLE;
   - the latest yaw target is **fresh**: stamp age ≤ `yaw_servo_target_timeout` (0.5 s, the
     same gate as SUPER's `kYawTargetTimeoutSec`) and not more than 0.2 s in the future;
   - its frame is `lidar_init` or `world` (identity TF in this stack). Any other frame is
     ignored, with one warning logged;
   - the target and `lpos` are finite, and `lpos.xy_valid`.
2. **The desired heading, `ψ_des`.**
   - Convert the target FLU → NED: `(x, −y)`.
   - `ψ_des = atan2(t_y − p_y, t_x − p_x)`, where `p` is the aircraft's current position in
     `lpos`.
   - This equals the FRD negation of the FLU bearing, the same `ψ → −ψ` rule the bridge already
     uses. **The sign is the classic spiral trap: the gate must assert it (§4.2 b).**
3. **Close range.** If the horizontal range is below `yaw_servo_min_range_m` (2.5 m), keep
   `ψ_des` at its last value and set the feed-forward to 0. At close range, a person stepping
   sideways demands huge rates (rate = speed ÷ range).
4. **Otherwise, not active** (the fall-back): `ψ_des = ψ_super`.
5. **Slew limit, whenever the switch is on, whatever the source:**
   `ψ_out += clamp(wrap(ψ_des − ψ_out), ±yaw_servo_max_rate·dt)`, with the rate at **90°/s**.
   - This covers a target jump (a track switch, the yaw hold latching its point, the
     re-emergence point) and the switch between servo and SUPER.
   - It must never bind in a normal follow (as flown, ≤ 8°/s under fix 1).
   - On the first active tick after IDLE, initialise `ψ_out` to the vehicle's heading.
6. **`yawspeed`:**
   - servo active, fix 2 off: **0** (replayed as "fix 1");
   - servo active, fix 2 on (`yaw_servo_ff_enable`): the feed-forward, computed as follows
     (replayed as "fix 1 + 2"):
     - `ff_raw = wrap(ψ_des − ψ_des_prev)/dt`, averaged over 0.2 s (the target arrives at
       15 Hz);
     - **a step in `ψ_des` over 5° in one tick resets the average to 0** (a target switch must
       not become a rate spike);
     - capped at ±`yaw_servo_ff_max_rate` (60°/s);
     - slewed at ≤ `yaw_servo_ff_max_accel` (90°/s²);
     - 0 below the close-range limit, and decaying at the slew rate once the servo goes
       inactive;
   - fall-back (switch on, servo inactive): SUPER's `−yaw_dot` while the slew limit is not
     binding (|wrap(ψ_super − ψ_out)| < 2°), else 0.
7. **Position, velocity and acceleration are untouched.** With the switch off, the output must
   be **bit-identical** to today's bridge.

### 2.4 When it acts

| Bridge state | Switch off | Switch on |
|---|---|---|
| Not in Offboard | no setpoints (2026-09-28 rule) | same |
| IDLE (disarmed Offboard) | latch, yaw = heading | same: **no servo** |
| TRACK | SUPER's yaw and `yaw_dot` | servo heading and ff; fall-back when the target is stale |
| HOLD (SUPER silent, armed Offboard) | frozen yaw, `yawspeed` 0 | **servo keeps aiming** (position still frozen); fall-back = the frozen yaw |

- **One new behaviour to be aware of:** at the engage, the bridge HOLDs until SUPER's first
  command. The servo already turns the nose to the person then (slew-limited).
- Decide in the build session whether that is wanted. The alternative is a parameter that
  delays the servo until the first TRACK.

### 2.5 Parameters (ROS parameters of the bridge)

| Name | Default | Meaning |
|---|---|---|
| `yaw_servo_enable` | **false** | fix 1 |
| `yaw_servo_ff_enable` | **false** | fix 2; has no effect unless `yaw_servo_enable` |
| `yaw_target_topic` | `/planning/yaw_target` | the input |
| `yaw_servo_target_timeout` | 0.5 s | freshness; matches SUPER |
| `yaw_servo_max_rate` | 90°/s | heading slew limit |
| `yaw_servo_ff_max_rate` | 60°/s | feed-forward cap |
| `yaw_servo_ff_max_accel` | 90°/s² | feed-forward slew |
| `yaw_servo_min_range_m` | 2.5 m | below it: hold heading, ff 0 |
| `yaw_servo_camera_offset_deg` | 0 | optional: the camera's yaw relative to the body, see §6 |

- **How a switch reaches a flight.** Every launch path uses a bare `ros2 run` (the GCS row in
  `scripts/gcs/supervisor.py` `PROCS`, `deploy_follow.sh`, `bench_bringup.sh`), so the
  **defaults decide**.
- When the user approves a flight, flip the default in code, as was done for
  `publish_acceleration_ff` on 2026-09-27, and restart the bridge.
- Log the switches in the startup line.

### 2.6 Logging
- Add a debug topic `/bridge/yaw_servo` (`std_msgs/Float32MultiArray`, 50 Hz, only while
  engaged). Suggested layout: `[active, ψ_des, ψ_out, yawspeed, range_m, target_age_s,
  source(0 super / 1 servo / 2 close-range hold)]`. Document the layout in the node.
- Add it to `_M51` in `scripts/gcs/bagger.py`, so the m51 and stage_b presets record it, and to
  `deploy_follow.sh`'s topic list.
- Add a line to the 5 s status report: servo on/off/fall-back, and its worst slew in the window.
- Log each servo on/off edge once.

## 3. What this does not change
- SUPER, `follow_planner`, `command_mux` and the perception stack.
- The position, velocity and acceleration setpoints.
- Flights without a follower (TODO step 3, O1): there is no yaw target, so the servo is inactive,
  and with it the whole change.
- Speed. Following at 5 m/s still needs the velocity tuning ladder, a longer SUPER planning
  horizon (7 m is 1.4 s of look-ahead at 5 m/s), more `max_vel` and `max_acc`, and the battery
  work. This plan only keeps the person in frame.

## 4. Gates, all before any flight

### 4.1 Unit tests of the pure module
A small script, no ROS. Cover:
- the sign (b below);
- wrap across ±180°;
- the slew limit;
- the ff cap, slew and step reset;
- close range;
- freshness, including a future stamp;
- fall-back without a jump;
- NaN inputs;
- switch off = identity.

### 4.2 `scripts/px4/m3_bridge_check.py`: new checks
It runs the real node in a sandbox with the topics under `/m3_check/`. Start extra bridges with
`start_offboard_bridge(suffix, extra=...)`. **All 51 current checks must still pass with the
switches off.** New checks:
- a. Switches off: `yaw` and `yawspeed` are identical to SUPER's.
- b. **Sign.** A target at FLU +y (to the left) gives an FRD yaw < 0 (a left turn), equal to
  −atan2 within 0.5°. It is not the π/2 variant.
- c. A target bearing sweeping through ±180°: the output is continuous and the rate stays within
  the limit.
- d. A 180° target jump: every tick's change is ≤ 90°/s·dt, and the heading arrives in ≤ 2.2 s.
- e. A stale target (> 0.5 s) → fall-back to SUPER's yaw through the slew limit, with no jump.
  A future-stamped target is ignored.
- f. A target at 1.5 m moving sideways fast: the heading is held and `yawspeed` is 0.
- g. The ff:
  - a target orbiting at 30°/s → `yawspeed` 30 ± 5°/s once settled;
  - at 100°/s → capped at 60°/s, with the slew ≤ 90°/s²;
  - a 5 m target jump in one message → no step above the slew.
- h. IDLE (disarmed Offboard): no servo, the latch as today. Not in Offboard: no setpoints (the
  existing contract).
- i. HOLD with a fresh target: it aims, and the position stays frozen.
- j. Wrong `frame_id`, NaN target, or `lpos` not valid → fall-back.
- k. Position, velocity and acceleration are identical with the switches on and off.
- Update the check counts in `.claude/rules/px4-bridges-offboard.md`, `CLAUDE.md` and the GCS gate
  label.

### 4.3 The replay gate
- Point `drone-dtp/scripts/fix1_yaw_replay.py` at the shared module (§2.1), so it replays the
  real algorithm with its limits. Run it on `bags/stage_b_2026-09-26_200650`,
  `bags/stage_b_2026-09-26_195108` and `bags/stage_b_2026-09-21_195658_salvaged`.
- **Pass** (margins over the 2026-09-29 replay, which had no slew limit):
  - 200650: servo p95 ≤ 5°, max ≤ 6°; servo + ff p95 ≤ 2.5°;
  - 195108: servo p95 ≤ 9.5°; servo + ff p95 ≤ 6.5°;
  - 09-21: servo no worse than as flown at p95;
  - with the target fresh, the heading rate stays ≤ 15°/s;
  - over the whole Offboard window, including after the loss, it **never exceeds 90°/s**.
- If the slew limit binds during a normal follow, stop and report. The limit is wrong, or the
  input is.

### 4.4 Props off, on the aircraft
- Follower **Live**, bridge with both switches on (for this test only, via `--ros-args -p`, not a
  default).
- Arm in Position, then ch11 (Offboard). PX4 will not arm from the transmitter in Offboard.
- A person walks a circle at ~4 m. Check that `listener trajectory_setpoint` on the FC shows
  `yaw` following the person's bearing, and `/bridge/yaw_servo` shows `active`.
- Also check the takeover: the stream stops as before.
- Restart the stack afterwards: a bench goal left in SUPER is a stale plan for the next engage.

## 5. Flights (the user's call; after TODO step 4)
Fly TODO step 4 first, with the servo **off**. It is the baseline for `yaw_chain_report.py` on
today's stack. Then:
- **Y1: servo on, ff off, walking pace.** The walker:
  - walks the line;
  - side-steps 2–3 m;
  - walks a slow arc around the aircraft at ~5 m;
  - does a turnaround.
  - Score with `yaw_chain_report.py` and `fix1_yaw_replay.py`. Expect p95 off-centre ≤ ~6°.
- **Y2: add the ff.** Same course.
- **Stop rule (ch11 takeover):** the nose swings more than ~45° away from the person, oscillates
  in yaw, or turns continuously. The slew limit caps the rate at 90°/s, so a runaway would show
  as a steady turn, not a snap.
- Raise speed only with the velocity tuning plan (`docs/px4-velocity-tuning-plan.md`).

## 6. Open questions for the build session
- **The camera's yaw against the body.** The servo points the **body** x-axis at the person. If
  the Gemini's optical axis is yawed by δ against it, the person sits δ off-centre.
  - Check `.claude/rules/dtp-calibration.md` (the camera↔LiDAR extrinsic) and whether perception
    or `follow_planner` already compensates. Otherwise leave `yaw_servo_camera_offset_deg` at 0.
  - Measure δ on Y1 from the detections' pixel x.
- **Aim during the engage HOLD, or wait for the first TRACK?** See §2.4.
- **The unmeasured MID360→FC yaw mount (an open flight blocker).** The servo turns less than
  SUPER while following, so it does not add exposure there. It stays the user's call either way.

## 7. Records to update after the build
- A dated entry in `.claude/rules/px4-bridges-offboard.md`: what was built, the switches and
  gate results.
- A pointer from the "Measured 2026-09-29" section of `dtp-follow-behaviour.md`.
- `CLAUDE.md` "Latest changes": one line.
- `TODO.md`: the Y1/Y2 flights.
- Commit only when the user asks. No `Co-Authored-By` trailer in this repo.
