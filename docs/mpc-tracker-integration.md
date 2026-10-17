# Integrating the MPC trajectory tracker: install, select, test, fly

**Status (2026-10-17): integrated into the code, OFF by default, never flown.** Nothing changes
unless you select the tracker. With no selection, every launcher still starts
`px4_offboard_bridge.py` exactly as before.

The design is in [`mpc-tracker.md`](mpc-tracker.md). This file is the how-to:

1. what this branch changes;
2. how to put it on GitHub as a new branch;
3. how to install it on the drone;
4. how to gate it;
5. how to select it, and how to go back;
6. what works with it unchanged, and what does not;
7. how to read its log;
8. the steps before it flies.

---

## 1. What this branch changes

Every change is additive, or sits behind a switch whose default is the old behaviour.

| File | New / edited | What |
|---|---|---|
| `super_bringup/scripts/px4_mpc_tracker.py` | **new** | The node. A replacement for `px4_offboard_bridge.py`, never run beside it. |
| `super_bringup/scripts/mpc_tracker_core.py` | **new** | The MPC maths, numpy only. |
| `super_bringup/CMakeLists.txt` | edited | Installs the node (`PROGRAMS`) and the core (`FILES`, beside `yaw_servo.py`). |
| `scripts/px4/mpc_tracker_sim.py` | **new** | Offline gate: simulated drone, 8 checks. |
| `scripts/px4/mpc_tracker_node_check.py` | **new** | Offline gate: the real node with ROS stubbed, 21 checks. |
| `scripts/gcs/supervisor.py` | edited | `GCS_TRACKER=mpc` makes the trajectory row run the tracker. The row recognises either node as running. |
| `scripts/gcs.sh`, `scripts/gcs_service.sh` | edited | Document `GCS_TRACKER`; the service installer writes it into the unit. |
| `drone-dtp/scripts/deploy_follow.sh` | edited | `--tracker=mpc`. Its survivor check now also matches `px4_mpc_tracker` and the 15-character `px4_offboard_br` that `ps -o comm` actually prints. |
| `drone-dtp/scripts/bench_bringup.sh` | edited | `TRACKER=mpc`. |
| `docs/mpc-tracker.md`, `docs/mpc-tracker-integration.md` | **new** | The design and this guide. |
| `docs/README.md`, `scripts/px4/README.md`, `.claude/rules/px4-bridges-offboard.md`, `CLAUDE.md` | edited | Index entries and the dated record. |

**Not changed:** `px4_offboard_bridge.py`, `yaw_servo.py`, `px4_odom_bridge.py`, every
`drone-dtp` node, every FC parameter, SUPER and its configs.

### Why it was changed from the 2026-10-05 version
The tracker was first written on `nmpc-follow-goal`, against the bridge as it was then. The
bridge on `rollback-pre-0926` has moved on, so the tracker was brought in line with it:

| Bridge change | In the tracker |
|---|---|
| Setpoints only while PX4 reports Offboard (2026-09-28) | Already the tracker's IDLE rule. The edge logs, the "silent report keeps the stream" rule and its 2 s warning were added. |
| Camera yaw servo, default off (2026-09-29) | Added, with the bridge's parameter names and defaults, through the same `yaw_servo.py`. |
| Acceleration feedforward on (2026-09-27) | Already built in: SUPER's acceleration is the MPC's input reference. |
| Read-only, validated servo parameters | All the tracker's parameters are now read-only. A bad servo value stops the node at start-up. |
| `yaw_servo.py` is imported from beside the node only | The same import rule, for both modules. |

## 2. Putting it on GitHub as a new branch

This folder has no `.git`. It is the full working tree of `rollback-pre-0926` plus the changes
above. To push it as a new branch, copy it over a fresh clone of that branch:

```bash
# 1. a clean clone, on the branch this was built on
git clone <your-repo-url> SUPER-Human-DTP
cd SUPER-Human-DTP
git switch rollback-pre-0926
git switch -c mpc-tracker            # the new branch

# 2. copy this folder over it. --exclude .git keeps the clone's history.
#    No --delete: the clone may hold files this copy never had.
rsync -a --exclude .git /path/to/updated/SUPER-Human-DTP-rollback-pre-0926/ ./

# 3. check that ONLY the files in section 1 changed
git status --short
git diff --stat

# 4. the executable bits matter (`ros2 run` needs them; see the 2026-09-02 odom-bridge bug)
git update-index --chmod=+x super_bringup/scripts/px4_mpc_tracker.py \
    scripts/px4/mpc_tracker_sim.py scripts/px4/mpc_tracker_node_check.py

# 5. commit and push
git add -A
git commit -m "Add px4_mpc_tracker: MPC alternative to the offboard bridge (not flown)"
git push -u origin mpc-tracker
```

If `git status` shows files you did not expect (anything under `docs/evidence/`, `firmware/` or
`bags/`), the uploaded copy was older or newer than the branch. Restore them with
`git checkout -- <path>` before committing.

**`drone-dtp/` trap (`dtp-workspace.md`):** checking out a branch without `drone-dtp/` deletes it
from disk. This branch has it, so switching to and from `rollback-pre-0926` is safe.

## 3. Installing on the drone

`super_bringup` is **copied** into the workspace, not symlinked, so nothing reaches the drone
until you re-copy and rebuild. `drone-dtp/` *is* symlinked, so `deploy_follow.sh` and
`bench_bringup.sh` are live as soon as the branch is checked out.

```bash
cd ~/vla_learn/SUPER-Human-DTP                   # wherever the repo is on the drone
git switch mpc-tracker

# the two offline gates first (no ROS needed, ~10 s together)
python3 scripts/px4/mpc_tracker_sim.py           # must end PASS (8/8)
python3 scripts/px4/mpc_tracker_node_check.py    # must end PASS (21/21)

# re-copy and rebuild super_bringup (never two colcon builds at once)
rm -rf ~/super_ws/src/super_bringup && cp -r super_bringup ~/super_ws/src/
(cd ~/super_ws && colcon build --packages-select super_bringup --symlink-install)
source ~/super_ws/install/setup.bash

# all three files must be installed side by side
ls ~/super_ws/install/super_bringup/lib/super_bringup/
#   ... mpc_tracker_core.py  px4_mpc_tracker.py  px4_offboard_bridge.py  yaw_servo.py ...

# and the INSTALLED node must pass the node check too
NODE=~/super_ws/install/super_bringup/lib/super_bringup/px4_mpc_tracker.py \
    python3 scripts/px4/mpc_tracker_node_check.py
```

Re-copying also re-installs the bridge, which is unchanged. Re-run
`python3 scripts/px4/m3_bridge_check.py` anyway (`ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1` while a
live bridge is up, per the 2026-09-30 note), so that the install is known to be good.

## 4. The gates

| Gate | What it proves | Needs |
|---|---|---|
| `scripts/px4/mpc_tracker_sim.py` | The maths tracks a plant that is deliberately not its model: hover in wind, figure-eight, far setpoint, limits, solve time. | numpy |
| `scripts/px4/mpc_tracker_node_check.py` | The node: states, frames, what is on the wire in each state, takeover, silent mode report, fallback, the yaw servo. Mutation-tested. | numpy |
| **Not built yet:** a real-DDS sandbox, like `m3_bridge_check.py` | QoS matching, real `px4_msgs` field types, discovery, the "refuses a second publisher" check against a real bridge. | the drone |

Run both offline gates after any change to `px4_mpc_tracker.py`, `mpc_tracker_core.py` or
`yaw_servo.py`. `m3_bridge_check.py` tests only the bridge. It does not test the tracker.

## 5. Selecting the tracker, and going back

**The rule:** exactly one publisher on `/fmu/in/trajectory_setpoint`. The tracker refuses to start
if the bridge is up. The bridge does not check for the tracker, so **always stop one before
starting the other**, then confirm:

```bash
ros2 topic info /fmu/in/trajectory_setpoint      # Publisher count: 1
ros2 node list | grep -E "px4_offboard_bridge|px4_mpc_tracker"   # exactly one line
```

### a. The web GCS (the standard bring-up)
The trajectory row is the second bridge row on the Stack card (key `offboard_bridge`). Which node
it runs is chosen when the GCS starts:

```bash
# by hand
GCS_TRACKER=mpc bash scripts/gcs.sh

# the service, so it survives a reboot
sudo GCS_TRACKER=mpc bash scripts/gcs_service.sh install
```

The row's label then reads **"MPC trajectory tracker (SUPER -> PX4 acceleration)"**. The Flight
day card's "Start the trajectory bridge" button starts whichever the row runs. Everything else on
the card is unchanged.

- **Restarting `super-gcs` stops the dashboard-owned stack and bridges** (its ordered teardown).
  Switch between flights only.
- The row recognises **both** node names as "running". A hand-started bridge therefore shows as an
  outside process on the MPC row, and its Stop stops it. That is deliberate: the two must never run
  together.
- **To go back:** `sudo bash scripts/gcs_service.sh install` without `GCS_TRACKER`, or
  `GCS_TRACKER=bridge`.

### b. `deploy_follow.sh` (the lean field bring-up)
```bash
bash drone-dtp/scripts/deploy_follow.sh --tracker=mpc            # live follower, MPC tracker
bash drone-dtp/scripts/deploy_follow.sh --observe --tracker=mpc  # observe-only
bash drone-dtp/scripts/deploy_follow.sh stop                     # unchanged
```
It uses the same `OFFBOARD` group and the same `offboard_bridge.log`, so `stop` and `status` work
as before. It prints `trajectory: px4_mpc_tracker.py` in step 2. Leave out `--tracker` to get the
bridge.

### c. `bench_bringup.sh` (desk pre-check)
```bash
TRACKER=mpc bash drone-dtp/scripts/bench_bringup.sh
```

### d. By hand
```bash
# stop the bridge first (its own terminal: Ctrl-C), then:
ros2 run super_bringup px4_mpc_tracker.py
ros2 run super_bringup px4_mpc_tracker.py --ros-args -p mpc_tau:=0.17       # a tuning value
ros2 run super_bringup px4_mpc_tracker.py --ros-args -p yaw_servo_enable:=true   # bench only
```

## 6. What works with it unchanged, and what does not

These check `/fmu/in/trajectory_setpoint` and `/fmu/in/offboard_control_mode` by **publisher
count** or **heartbeat rate**, not by node name, so they accept the tracker as it is:

- `stage_a_preflight.py` (both the Stage A and the live / Stage B forms) — "nav stack:
  `/fmu/in/trajectory_setpoint` … publishing". The message names the bridge, but the check is the
  topic;
- `estimator_preflight.py`, `field.sh check`, `goal_forward.py`;
- `offboard_engage.py`: it triggers on the 50 Hz heartbeat, which the tracker sends in every
  state, and after the switch it counts setpoints;
- `m51_one_graph_check.py` A and T: the heartbeat, and the setpoints stopping after a takeover;
- the bag presets (`stage_a`, `m51`, `stage_b`): the same topics, plus `/bridge/yaw_servo` when
  the servo is on;
- the GCS telemetry chips (setpoint and heartbeat rates) and the descent guard / geofence /
  engage gate in `command_mux`, which act on SUPER's goal, upstream of the tracker.

These do **not** fit the tracker. Know it before you read their output:

| Tool | Why | What to use instead |
|---|---|---|
| `scripts/px4/m3_gate_check.py` | Its "setpoint stayed frozen" reads the setpoint's **position**. The tracker sends NaN there in TRACK and HOLD, so the check passes **vacuously** (max drift 0.000). | Read the vehicle's own drift in HOLD from `vehicle_local_position`, or the tracker's `err` in its 5 s line. |
| `super_bringup/scripts/offboard_viz.py` | Draws the setpoint's position. NaN → no setpoint marker in TRACK/HOLD (it shows only in POS_HOLD and FALLBACK). | The tracker's log, or `/planning/pos_cmd` in RViz. |
| `scripts/px4/m3_bridge_check.py`, `yaw_servo_soak.py`, `yaw_servo_bag_bench.py`, `fix1_yaw_replay.py` | They start or model `px4_offboard_bridge.py` specifically. | The two tracker gates (§4). |
| Velocity-loop study (`docs/px4-velocity-tuning-plan.md`) | The tracker bypasses PX4's position and velocity loops, so `MPC_XY_VEL_*` and `MPC_XY_P` do not act on it. `MPC_TILTMAX_AIR` and `MPC_THR_HOVER` still do. | Tune the tracker's own weights (`mpc-tracker.md` §6). |
| `measure_coresidency.py`, `m53_soak.py` | They name `px4_offboard_bridge` in their process lists. | Read the tracker's CPU with `top -p $(pgrep -x px4_mpc_tracker)` (its comm name is exactly 15 characters; never `pgrep -f`, `CLAUDE.md`). |

## 7. Reading the log

At start-up it prints the horizon, `tau`, the limits, the clamp, and whether the servo is on. Then
it prints one line every 5 s:

```
TRACK cmd_in=4512 bad=0 sp_out=250 fallbacks=0 armed=True offboard=True | err mean 0.04 max 0.09 m |
  solve p50 0.31 max 0.80 ms iters 0 | u [+0.42 -0.10 -0.05] d_hat [+0.08 -0.03 +0.21] | yaw servo off
```

| Field | Healthy | Worry when |
|---|---|---|
| state | IDLE on the ground, POS_HOLD / HOLD at the engage, TRACK while SUPER commands | any `FALLBACK` in the air: read the ERROR line before it |
| `err mean/max` | ≤ 0.1 m in HOLD, ≤ 0.2 m in TRACK | growing in HOLD (`tau` or `qp` wrong, or a drift) |
| `solve p50/max` | < 2 ms on the Orin | max > 10 ms (half a tick) |
| `iters` | 0 most of the time (the unconstrained answer fitted) | stuck at 60: the limits bind all the time |
| `d_hat` | settles within ~3 s of the engage, small in calm air | at ±1.5 (clamped): a real hover-thrust or mount error |
| `(withheld: PX4 not in Offboard)` | on the ground and in manual modes | in Offboard: the mode report says Offboard is off |

## 8. Before it flies: the checklist

Each step must pass before the next. **The two open flight blockers in `CLAUDE.md` (the hardfault
soak and the yaw mount) apply to this tracker exactly as to the bridge.** Whether it flies at all
is the user's call.

1. [ ] **Both offline gates on the drone, against the installed node** (§3).
2. [ ] **A real-DDS sandbox, props off, no FC needed.** Not built yet. Write it like
       `m3_bridge_check.py`, with every topic remapped under a sandbox prefix on its own
       `ROS_DOMAIN_ID`. It must check frames, the states, the heartbeat at 50 Hz, the refusal beside
       a real bridge, and QoS matching against SUPER's BEST_EFFORT `pos_cmd`.
3. [ ] **Props off, on the aircraft.** With the tracker up, in both engage orders:
       - `listener offboard_control_mode` on the FC shows `acceleration: True` in HOLD/TRACK;
       - `listener trajectory_setpoint` shows NaN position and velocity and a finite acceleration;
       - outside Offboard, `listener trajectory_setpoint` shows only PX4's own task;
       - the takeover stops the stream (`setpoint stream stopped` in the log).
       This also settles the open question: does PX4 v1.17 accept acceleration-only Offboard on
       this build?
4. [ ] **Measure `tau` from a ULog.** Compare `vehicle_acceleration` with the acceleration setpoint
       during a Position-mode step. The 2026-09-27 study measured 0.15–0.19 s plus 40–60 ms of
       delay, against the default 0.12. Set `mpc_tau` and re-run the simulator.
5. [ ] **The engage window.** What does `mc_pos_control` fly between the Offboard switch and the
       tracker's first sample? The bridge's answer (`px4-bridges-offboard.md`, 2026-09-28) applies:
       ~40–70 ms of the flight task's last setpoint. **Centre the sticks before engaging.**
6. [ ] **First flight: hover only (HOLD, no goal).** The RC Offboard switch (ch11) is the takeover.
       Arm in Position, then ch11. Watch the 5 s line: `d_hat` should settle and `err` stay small.
       Record the `stage_b` preset.
7. [ ] **Then a short SUPER goal from the GCS RC card,** inside the S2 envelope (1 m/s, 1 m/s²).
       Raise `mpc_v_ref_max` only after this, for the 3.0 m/s envelope.
8. [ ] **Then a Stage B follow**, the yaw servo off.

## 9. Rolling back

Nothing outside the tracker depends on it. To go back to the bridge:

- stop the tracker and start `px4_offboard_bridge.py` (GCS: re-install without `GCS_TRACKER`;
  `deploy_follow.sh`: drop `--tracker`);
- or `git switch rollback-pre-0926`, re-copy `super_bringup`, and rebuild it.

The bridge, its parameters and its gates are untouched by this branch.
