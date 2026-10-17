# Horizontal velocity loop evidence, 2026-09-27

The scripts behind [`../../px4-velocity-tuning-plan.md`](../../px4-velocity-tuning-plan.md). All of
them are read-only. They read two bags and four ULogs, never ROS topics, DDS or the flight
controller. **Nothing here was flown and no gain was changed.**

## Inputs
- `bags/stage_b_2026-09-26_195108` and `bags/stage_b_2026-09-26_200650`: the two 2026-09-26
  Offboard follow flights (`.claude/rules/flight-log.md`). Bags are gitignored and live only on the
  Jetson.
- `~/px4_logs/2026-09-21_{18_48_30,19_18_48,20_12_25,20_13_44}.ulg`: retune Flights 1–4
  (`docs/px4-tuning-notes.md` §9).
- The cache goes to `/tmp/velocity-loop-2026-09-27/` (override with `VELTUNE_CACHE`), outside
  the repo. The bag pickles are ~10 MB and are not kept in git.

## How to run
```bash
cd docs/evidence/velocity-loop-2026-09-27
# 1. bags -> cache. Needs ROS for px4_msgs / mars_quadrotor_msgs. Refuses while any
#    `ros2 bag record` runs (reading a .db3 that is being written kills the recorder).
source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
python3 extract_bags.py
# 2. measured, from the bags (numpy only)
python3 offboard_score.py      # vsp reconstruction, lag, steady tracking, events, the reference
python3 offboard_accfit.py     # commanded vs achieved acceleration: lag + delay + drag
python3 yaw_rotation.py        # indicative MID360->FC yaw (lower bound, not a measurement)
# 3. measured, from the ULogs (pyulog is only in the PX4 venv)
~/PX4-Autopilot/.venv/bin/python ulog_attitude.py      # attitude-loop lag, the p.110 angle check
~/PX4-Autopilot/.venv/bin/python ulog_drag_limits.py   # drag, modes, headroom, battery
# 4. modelled (numpy only, minutes each)
python3 model_validate.py      # the model against both flights
python3 model_predict.py       # all prediction tables; or name sections: steps moves ladder ...
```
Run them with `nice -n 19` if gates are running: the fits are single-threaded but take minutes.

## Files

| File | What it is |
|---|---|
| `common.py` | Paths, filters, Offboard-window detection (armed + Offboard flags), and the reconstruction of PX4's velocity setpoint `vsp = vel_ff + MPC_XY_P·(pos_sp − pos)` on PX4's own sample times |
| `extract_bags.py` | Bags → cache pickle: setpoints, local position (with PX4 stamps), control mode, status, SUPER `pos_cmd`, FAST-LIO odometry |
| `sim_model.py` | The model: PX4 v1.17 `PositionControl` (xy) from source, on a fitted plant. Also the S-curve reference |
| `offboard_score.py` | Measured Offboard scores, stock gains, no feedforward |
| `offboard_accfit.py` | Fit of EKF2's `ax/ay` to the rebuilt acceleration command, with the R² ridge |
| `yaw_rotation.py` | Rotation between EKF2's and FAST-LIO's accelerations |
| `ulog_attitude.py` | Tilt setpoint → tilt fit per axis and flight; estimated/setpoint peak ratios on wobbles |
| `ulog_drag_limits.py` | Rotor drag from body-frame specific force; what the Position segments contain; motor, tilt and thrust headroom; battery |
| `model_validate.py` | Output-error fit of the model to both flights, and the event metrics |
| `model_predict.py` | Step responses, stability margins, rest-to-rest moves at 2/3/4 m/s, disturbance, replays with feedforward, drag feedforward, sensitivity, the tuning ladder, setpoint jumps |

## Traps
- **Two clocks.** Bag receive time lags PX4's stamp by 8–15 ms (median) for local position. The
  reconstruction puts local position on `timestamp_sample` and the setpoint on the bridge's stamp
  + 5 ms. Using bag time for both biases the rebuilt position error by a few cm at 2 m/s.
- **`pos_cmd` is `lidar_init` FLU.** Into NED it is `(x, −y)`. The check: its acceleration
  correlates 0.95–0.96 with d(vel_ff)/dt.
- **The Offboard window is detected, not typed.** Times are seconds after the first message of the
  extracted topics, so they differ slightly from other notes' bag times.
- **pytest from the repo root lints `docs/evidence/`** (`.claude/rules/m6-loss-oracle.md`). These
  are scripts, not tests.
