# MPC trajectory tracker (`px4_mpc_tracker`): what it is and how it works

**Built 2026-10-05 on `nmpc-follow-goal`; ported onto `rollback-pre-0926` and brought in line
with its bridge 2026-10-17. Never flown, not installed in `~/super_ws`.** Offline simulator 8/8,
node check (ROS stubbed) 21/21. It has **not** run with the real `px4_msgs` / DDS, nor on the
flight controller.

**How to install, select and test it, step by step: [`mpc-tracker-integration.md`](mpc-tracker-integration.md).**
This file is the design.

This is an MPC for **flying the drone along SUPER's trajectory**. It is separate from
`goal_mode: nmpc` in `follow_planner`, which decides *where the goal is*
([`fact-nmpc-follow-goal.md`](fact-nmpc-follow-goal.md)).

---

## 1. What it replaces

```
Today:   SUPER --pos_cmd--> px4_offboard_bridge --POSITION (+vel, +acc ff)--> PX4 position+velocity PID --> attitude --> motors
New:     SUPER --pos_cmd--> px4_mpc_tracker     --ACCELERATION setpoint----> PX4 ---------------------------> attitude --> motors
                                   ^                                                                         |
                                   +----------------- /fmu/out/vehicle_local_position_v1 --------------------+
```

- **Today:** the bridge forwards SUPER's position, velocity and (since 2026-09-27) acceleration
  as feedforward, and PX4's own PID loops do the tracking.
- **With the tracker:** the MPC does the tracking and sends PX4 an **acceleration** setpoint
  (`offboard_control_mode.acceleration`, position and velocity NaN).
- **What PX4 still does:** attitude, rate and motor control. Those are unchanged.

`px4_mpc_tracker.py` is a **replacement** for `px4_offboard_bridge.py`. Run one or the other,
never both. The tracker refuses to start if `/fmu/in/trajectory_setpoint` already has a
publisher, and logs an error every 5 s if a second one appears later.

## 2. Files

| File | What it is |
|---|---|
| `super_bringup/scripts/mpc_tracker_core.py` | The controller maths. Numpy only, no ROS, so it can be tested offline. Installed as a module (`install(FILES)`), like `yaw_servo.py`. |
| `super_bringup/scripts/px4_mpc_tracker.py` | The ROS 2 node that runs on the drone. |
| `super_bringup/scripts/yaw_servo.py` | Unchanged. The tracker imports it, as the bridge does. |
| `scripts/px4/mpc_tracker_sim.py` | Bench gate 1: a simulated drone flies the real maths. PASS/FAIL. |
| `scripts/px4/mpc_tracker_node_check.py` | Bench gate 2: the real node with ROS stubbed, 21 checks. PASS/FAIL. |

The launcher and GCS changes that select it are listed in the integration doc, §1.

## 3. How the controller works

### The model: the same 1D drone as the `mpc_learn` lessons, once per axis

For each axis (x, y, z, in PX4's frame, NED-like), with model step `h`:

```
p[k+1] = p[k] + h*v[k] + h²/2 * (a[k] + d)
v[k+1] = v[k] + h*(a[k] + d)
a[k+1] = α*a[k] + (1-α)*u[k]          α = exp(-h/tau)
```

| Symbol | Meaning |
|---|---|
| `u` | The acceleration we command. This is what the MPC chooses. |
| `a` | The acceleration the drone actually has. It lags `u` by `tau`, because the drone must tilt before it accelerates sideways. |
| `d` | A constant unknown push: wind, a wrong hover thrust, a tilted mount. The controller estimates it while flying, which gives offset-free hover. |

### The cost, summed over a 1 s horizon (20 steps of 0.05 s)

```
qp·(p − p_ref)²  +  qv·(v − v_ref)²  +  r·(u − (a_ref − d))²  +  rdu·(u − u_prev)²
```

The last step's position and velocity terms are multiplied by `terminal`.

### The limits

- **Horizontal:** `|a_xy| ≤ 2.5 m/s²`, a disc. That is about 14° of tilt.
- **Vertical:** up ≤ 2.0 m/s², down ≤ 1.5 m/s².

### How it is solved

The model is linear and the cost is quadratic, so the whole horizon becomes one small QP:
`min ½UᵀHU + GᵀU` subject to the limits.

1. **Once at startup:** `H` is built. It never changes.
2. **Every tick:** `G` is rebuilt from the current state and reference.
3. **Solve, unconstrained first:** `U = −H⁻¹G`. If that already respects the limits, it is the exact answer.
4. **If it does not:** run projected gradient descent (FISTA), up to 60 iterations, starting
   from last tick's plan shifted by one step.
5. **Send only `U[0]`.** Re-plan next tick. That is receding horizon.

No solver library is needed, which matters because the Orin has no CasADi, OSQP or acados.

### The reference

SUPER's `pos_cmd` is a single sample: position, velocity and acceleration for "now". The tracker
extrapolates it over the horizon: constant acceleration for 0.3 s, then constant velocity.
SUPER's acceleration is used as the input reference (`a_ref`), so the tracker always uses the
feedforward the bridge sends behind `publish_acceleration_ff`.

### The disturbance observer

Each tick it compares the velocity the model predicted with the velocity measured. The
difference, divided by `d_time_const` (0.6 s), is added to `d`. The estimate is clamped to
±1.5 m/s².

## 4. The node's states

| State | When | What is published |
|---|---|---|
| `IDLE` | PX4 not in Offboard | **Heartbeat only, no setpoint.** The same contract as the bridge's `setpoint_only_in_offboard` (2026-09-28): PX4's own Position, Altitude and Auto-land tasks write that uORB topic. |
| `POS_HOLD` | In Offboard but disarmed (engage-then-arm) | Position setpoint at the current estimate. The MPC never runs on the ground. |
| `TRACK` | Armed + Offboard + fresh `pos_cmd` (< 0.3 s old) | MPC acceleration setpoint following SUPER. |
| `HOLD` | Armed + Offboard, `pos_cmd` stale or never received | MPC holding the last commanded point, or this tick's estimate at the engage. With `command_mux`'s engage gate SUPER is silent until armed + Offboard, so every engage starts here. |
| `FALLBACK` | Estimate stale (> 0.2 s) or invalid, or the MPC fails | Position setpoint at the last reference, so PX4's own controller takes over. |

### Behaviour kept identical to the bridge (rollback-pre-0926, 2026-10-17)

- **The Offboard flag changes only on a received `vehicle_control_mode`.** A report stream that
  goes silent never stops the setpoints (it would starve PX4 in the air); after 2 s it is
  warned about, once.
- **The stream's edges are logged:** `setpoint stream started (…)` and `PX4 left Offboard:
  setpoint stream stopped (heartbeat continues)`. The 5 s line says `(withheld: PX4 not in
  Offboard)` while nothing is sent.
- **SUPER's backup trajectory** (`trajectory_flag` EMER) is logged on the edge, both ways.
- **Every parameter is read-only.** `ros2 param set` on a running tracker is refused, not
  silently ignored.

### The camera yaw servo (default OFF)

`yaw_servo.py` is the one copy of the algorithm, used by both nodes. The tracker takes **the
bridge's parameter names and defaults** (`yaw_servo_enable`, `yaw_servo_ff_enable`,
`yaw_servo_min_range_m`, …; table in `.claude/rules/px4-bridges-offboard.md`), so a launch line
written for the bridge works unchanged.

- Off: no `/planning/yaw_target` subscription, no `/bridge/yaw_servo` publisher, yaw is SUPER's.
- On: it rewrites only `yaw` and `yawspeed` in TRACK and HOLD. **It never touches the MPC's
  acceleration** (node check N20). POS_HOLD and FALLBACK are handed to the servo as IDLE, so it
  resets and passes the yaw through. `/bridge/yaw_servo` is published while engaged, as from the
  bridge, so the bag presets record it.
- The servo's replay gate still fails (10/18) for the reasons in the rule file. Its state is the
  same here as in the bridge: **not ready to fly as configured.**

### Other safety behaviour

- **Far setpoint clamp.** If SUPER's present point is more than `max_pos_err` (1 m) from the
  drone, the whole horizon is shifted in to 1 m. In simulation, a 3 m offset at engage (the
  2026-09-21 case) gave a 0.97 m/s approach instead of a lunge. This is the "bridge refuses a far
  setpoint" protection the 2026-09-21 flight log asked for, which the bridge itself still lacks.
  - Only the *present* offset is clamped. Clamping every step also bent normal trajectories:
    this was a bug found and fixed during testing.
- **Frames.** It converts FLU to NED exactly as the bridge does: `(x, y, z) → (x, −y, −z)`,
  `yaw → −yaw`. It does **not** use `π/2 − yaw`.
- **QoS.** `pos_cmd` and `yaw_target` are subscribed BEST_EFFORT/VOLATILE, the same as the bridge.
- **Logging.** A log line every 5 s shows the state, tracking error, solve time, the command `u`,
  the disturbance estimate `d_hat`, and the servo's ticks per source when it is on.

## 5. Test results

### Offline simulator: `python3 scripts/px4/mpc_tracker_sim.py` (8/8, re-run 2026-10-17)

The simulated drone is deliberately different from the MPC's model:

- the true lag is 0.16 s (the model assumes 0.12 s);
- there is an extra 20 ms delay;
- there is a constant (0.6, −0.4, 0.5) m/s² wind;
- the position and velocity it sees are noisy.

| Scenario | Result |
|---|---|
| S1 hover in wind | Max error 1.2 cm after 8 s; wind estimate (0.73, −0.36, 0.57) |
| S2 figure-eight, 1.1 m/s peak | 3.4 cm RMS, 5.1 cm max |
| S3 reference 3 m away at engage | Peak speed 0.97 m/s, 0.8 cm overshoot |
| S4 limits | Tilt disc and vertical box never exceeded |
| Solve time | p99 0.24 ms on the PC (expect ~3× slower on the Orin) |

The simulator also prints a "PX4 cascade" comparison (22.7 cm RMS). That is a rough copy of
PX4's loops **without** the acceleration feedforward the bridge now sends, and it is **not** PX4
itself. Do not quote it as what the aircraft does. Add `--plot` to write `mpc_tracker_sim.png`.

### Node check: `python3 scripts/px4/mpc_tracker_node_check.py` (21/21, 2026-10-17)

The real `px4_mpc_tracker.py`, with rclpy and the message packages replaced by stand-ins, on a
fake clock, with a lagged point-mass plant where the loop must close.

| | Check |
|---|---|
| N1 | refuses to start beside another setpoint publisher |
| N2 | not in Offboard: heartbeat only, no setpoint |
| N3 | Offboard + disarmed: POS_HOLD, position setpoint at the estimate |
| N4–N5 | armed + Offboard with SUPER silent: HOLD, acceleration-only, holds the engage point (3 s, drift 0.000 m) |
| N6–N7 | FLU→NED: left is −y, up is −z; yaw → −yaw (not `π/2 − yaw`) |
| N8–N9 | 3 m far setpoint: inside the tilt disc, peak 0.86 m/s |
| N10 | stale `pos_cmd`: TRACK → HOLD |
| N11–N13 | invalid or stale estimate: FALLBACK to a PX4 position setpoint; recovery to TRACK |
| N14–N15 | takeover stops the stream at once; a silent mode report does not (warned after 2 s) |
| N16–N21 | yaw servo: off = no topics; bad parameter stops the node; on = turns to the target at ≤ 90°/s, never changes the acceleration, debug only while engaged |

**Mutation-tested 2026-10-17:** y not flipped fails N6; `π/2 − yaw` fails N7; setpoints sent
outside Offboard fail N2, N14 and N15; the servo scaling the acceleration fails N20; the engage
holding the origin instead of the estimate fails N5.

## 6. Tuning parameters

Every parameter is a read-only ROS parameter, set on the launch line,
e.g. `--ros-args -p mpc_tau:=0.15`. The MPC ones carry the `mpc_` prefix.

| Parameter | Default | Meaning |
|---|---|---|
| `mpc_h` / `mpc_N` | 0.05 s / 20 | Model step and number of steps (1 s horizon) |
| `mpc_tau` | 0.12 s | Tilt lag. **A guess. Measure it** (integration doc §8). The 2026-09-27 velocity study measured 0.15–0.19 s + 40–60 ms delay. |
| `mpc_qp` / `mpc_qv` | 10 / 3 | Position and velocity weights. Higher = tighter, more aggressive. |
| `mpc_r` | 0.5 | Weight on deviating from SUPER's feedforward acceleration |
| `mpc_rdu` | 2.0 | Smoothness: penalises changes in the command |
| `mpc_terminal` | 5.0 | Extra weight on the last step |
| `mpc_a_xy_max` | 2.5 m/s² | Horizontal limit (~14° tilt) |
| `mpc_a_up_max` / `mpc_a_down_max` | 2.0 / 1.5 m/s² | Vertical limits |
| `mpc_max_pos_err` | 1.0 m | Far-setpoint clamp |
| `mpc_v_ref_max` | 1.5 m/s | Reference speed cap. **See the note below.** |
| `mpc_acc_extrap_s` | 0.3 s | How long the reference's acceleration is extrapolated |
| `mpc_d_time_const` / `mpc_d_max` | 0.6 s / 1.5 m/s² | Disturbance observer speed and clamp |
| `mpc_max_iters` | 60 | Solver iterations when the limits are active |

Node parameters: `publish_rate` (50 Hz), `cmd_timeout` (0.3 s), `lpos_timeout` (0.2 s), and the
yaw servo's. The topic names are the same as the bridge's.

> **⚠️ `mpc_v_ref_max` vs SUPER's envelope on this branch.** The defaults were chosen for the S2
> envelope (`max_vel` 1.0, `max_acc` 1.0). Since 2026-09-30 this branch flies `max_vel` 3.0 /
> `max_acc` 2.0. With the defaults the tracker caps the *reference* at 1.5 m/s and the command at
> 2.5 m/s² horizontally, so it will lag SUPER above 1.5 m/s. Raise `mpc_v_ref_max` only after the
> hover and short-goal flights, and re-run the simulator after any default change in
> `mpc_tracker_core.py`.

## 7. What to expect

The repo's flight data says tracking was **not** what limited person-following: the median
altitude error in Offboard was +0.07 m, and SUPER's `max_vel` and `max_acc` set the speed. Since
2026-09-27 the bridge also feeds SUPER's acceleration forward, which the 2026-09-27 model says
removes most of the sprint overshoot. So this tracker should give tighter tracking, better wind
rejection, and a bounded approach to a far setpoint, but it is **not** expected to make the drone
keep up with a walker any better.
