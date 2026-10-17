# PX4 diagnostic scripts

Talk to the flight controller **without touching a serial port**. A `mavlink-router` process owns
the USB-C link and fans MAVLink out over UDP; port **14569** (`companion_aux`) is reserved for a
companion computer — the MAVLink tools use `udpin:127.0.0.1:14569`.
ROS bridge gates and goal/control helpers use DDS instead, as marked below.

**Never open the FC serial devices directly** — a second reader corrupts the stream, and `stty` is
per-device (not per-fd), so changing baud breaks the owning process too. This is what made the
original UART bring-up look broken for weeks.

**Which process owns the links depends on the board:**

| Board | MAVLink (USB-C, `/dev/ttyACM0` @ 2000000) | uXRCE-DDS (`/dev/ttyTHS1` @ 3000000) |
|---|---|---|
| Orin Nano, **ARK OS** image | ARK's `mavlink-router.service` | ARK's `dds-agent.service` (`MicroXRCEAgent serial -b 3000000 -D /dev/ttyTHS1`) |
| **Orin NX, ARK OS since 2026-08-31** (current recorded image) | ARK's `mavlink-router.service`, `/etc/ark-os/mavlink-router.conf`; UDP 14569 unchanged | ARK's `dds-agent.service`, hardcoded 3000000 baud |
| Orin NX, stock JetPack (**superseded installation**) | Our manually installed router, `/etc/mavlink-router/main.conf`; see [`../../docs/mavlink-setup.md`](../../docs/mavlink-setup.md) | Manual agent setup described in historical notes |

Verify current ownership before operational work. On the current ARK OS image,
our older router configuration is inactive and our `xrce-agent.service` is disabled.

On the NX the device path is the same stable by-id symlink,
`/dev/serial/by-id/usb-ARK_ARK_FPV.x_0-if00 → /dev/ttyACM0`.

## Scripts

| Script | Does |
|---|---|
| `px4_shell.py [cmd ...]` | Run commands on the PX4 NuttX shell over MAVLink `SERIAL_CONTROL` (same mechanism as QGC's MAVLink Console). Defaults to `uxrce_dds_client status` + `mavlink status`. |
| `probe_params.py` | Read the params that matter for M2/M3, grouped and annotated. |
| `set_params.py [--apply]` | Phase-1 transport params. Dry-run by default; `--apply` writes. Refuses if armed, verifies by read-back. |
| `power_module_params.py [--apply]` | ARK 12S Payload Power Module (INA238) config. **Aborts on stock ARK FPV firmware — the driver isn't compiled in**; see [`../../docs/ark-12s-payload-power-module.md`](../../docs/ark-12s-payload-power-module.md). |
| `diag_health.py` | Battery, sensor health, estimator flags — for "why won't it arm / why is it UNINIT". |
| `fetch_ulog.py [--list \| --id N \| --size B] [-o out]` | Download a ULog off the FC's SD card over MAVLink (`LOG_REQUEST_DATA`); the newest by default, into `~/px4_logs/`. ~80 s for 44 MB. **Refuses the newest log while armed** — a copy taken mid-write is truncated (the logloader loop, `docs/fact-ark-logloader-loop.md`). Read it with `pyulog` from `~/PX4-Autopilot/.venv`. Read-only. |
| `dump_params.py <out.params>` | **Full** read-only dump of every param, QGC format. Take one before any change; `probe_params.py`'s shortlist is not a backup. Dumps land in [`../../firmware/`](../../firmware/). |
| `ev_lever_arm.py [--apply]` | External-vision lever arm (`EKF2_EV_POS_X/Y/Z`) — where the MID360's **IMU** sits relative to the FC IMU, in body FRD. Dry-run by default. Applied 2026-09-02; the derivation is in the script docstring. |
| `m2_gate_check.py [secs] [csv]` | **The M2 gate.** Correlates PX4's `vehicle_local_position` against `/lidar_slam/odom` while you move the rig by hand, and reports per-axis **sign**, correlation, lag and tracking error. **Not a MAVLink script** — it's a ROS 2 subscriber, so source the workspace instead of needing `pymavlink`. Refuses to pass a run where the rig didn't actually move. |
| `m3_bridge_check.py` | **The M3 bench gate.** Runs the real `px4_offboard_bridge` **and `px4_odom_bridge`** in a sandbox (every topic remapped under `/m3_check/`, synthetic PX4 state) and asserts 51 things (39 before 2026-09-28, 32 before 2026-09-27). Offboard (T1–T7): the FLU→FRD conversion, the yaw sign **and that it is not the `π/2−ψ` trap**, yaw wrapping, the >2 Hz heartbeat, NaN rejection, HOLD-on-timeout and IDLE re-latch. Since 2026-09-27 it also checks acceleration feedforward: forwarded as (x,−y,−z) by the bridge's default, NaN with `publish_acceleration_ff:=false` (a second sandboxed bridge), and NaN in HOLD, in IDLE and after dropped commands. Since 2026-09-28 it checks that `trajectory_setpoint` is sent **only while PX4 reports Offboard**: nothing when disarmed or armed in a manual mode, the heartbeat still at rate, the first sample after the Offboard report = the current estimate for both engage orders, the stream kept up when the mode report goes silent in Offboard, stopped by a fresh Offboard-off report, and the old always-on stream from a third bridge with `setpoint_only_in_offboard:=false`. **Touches no flight controller and cannot arm anything** — run it after any change to the bridge. Also a ROS 2 script. |
| `estimator_preflight.py [--window S] [--no-bridge]` | **G0.2 — run before every arm; must print `GO`.** 60 s, read-only (subscribes, reads `/proc`). Refuses on: a second Livox driver / FAST-LIO or a non-CustomMsg `/livox/lidar` (E1); a driver newer than FAST-LIO (E2); odom < 9.5 Hz or a stamp gap > 1 s (E3); odom age p95 > 150 ms (E4); **\|odom\| > 1.0 m from FAST-LIO's origin (E5), the check that refuses the 09-12 3.74 km divergence**; EKF2 invalid or > 0.20 m from FRD(odom) (E6); any EKF2 reset in the window (E7); `/px4_odom_bridge/status` not TRACKING (E8). Also run inside `drone-dtp/scripts/stage_a_preflight.py` and from the dashboard. ROS 2, not MAVLink. See `docs/fact-fastlio-divergence-2026-09-12.md`. |
| `m3_gate_check.py [secs] [csv]` | **The M3 gate.** A passive recorder — commands nothing, cannot arm — while *you* drive engage/arm/takeover from the RC. 9 checks: armed-in-Offboard time, setpoint drift (the bench meaning of "position holds"), manual takeover, position validity, dead-reckoning, EKF resets, **`/lidar_slam/odom` live**, and **`cs_ev_pos` true throughout**. Invalid/dead-reckoning/no-vision are reported as *time spans*, and the CSV carries `xy_valid,z_valid,dead_reck,eph,epv,ev_pos` per sample. **Spends 4 s on preflight and exits (rc 2) if nothing publishes `/lidar_slam/odom`** — the offboard bridge derives its setpoint from PX4's own estimate, so a run with the LiDAR dead looks perfectly healthy and means nothing (it voided the 2026-09-04 P5 flight); `ALLOW_NO_LIDAR=1` overrides for a deliberate flow-only baseline. Run with **no `/goal_pose`** so SUPER stays silent and the bridge sits in HOLD. Default 40 s; ~10 s max armed per cycle because `COM_DISARM_PRFLT` auto-disarms with props off. ROS 2, not MAVLink. |
| `mpc_tracker_sim.py [--plot]` | **Offline gate for the MPC tracker's maths** (2026-10-05). Flies `super_bringup/scripts/mpc_tracker_core.py` against a simulated quadrotor that is deliberately not its model (wrong lag, 20 ms delay, wind, noise): hover in wind, a figure-eight, a 3 m far setpoint at engage, the tilt/vertical limits, solve time. 8 checks, numpy only, no ROS. See [`../../docs/mpc-tracker.md`](../../docs/mpc-tracker.md). |
| `mpc_tracker_node_check.py` | **Offline gate for the `px4_mpc_tracker` node** (2026-10-17). The real node with rclpy and the message packages stubbed, on a fake clock: states, FLU→NED and the yaw sign, heartbeat-only outside Offboard, takeover, the silent-mode-report rule, FALLBACK, the far-setpoint clamp, and the yaw servo stage. 21 checks, mutation-tested, no ROS. `NODE=<path>` tests the installed copy. It does **not** test DDS or real `px4_msgs`. See [`../../docs/mpc-tracker-integration.md`](../../docs/mpc-tracker-integration.md). |
| `clear_fault_logs.py [--clear]` | **Crash dumps on the SD card.** Read-only by default: lists `/fs/microsd/fault_*.log`. `--clear` downloads each into `firmware/faults/` over MAVLink FTP, verifies it, and only then `rm`s it from the card — a hardfault log is the only evidence the FC **reset itself**. Also commits a fault still pending in BBSRAM first, so the newest crash is not dropped. `--reset-counter` clears the boot-time reboot counter. |
| `fc_reboot.py [--yes]` | **Reboot the flight controller** (`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`). Dry-run by default, like `set_params.py`; `--yes` sends it. **Refuses while armed or while the FC reports IN AIR** — PX4's commander refuses an armed reboot too, but a tool whose job is to reset the FC must not rely on the thing it is resetting to say no. Confirms the reboot by watching the heartbeat drop and return *and* `SYSTEM_TIME.time_boot_ms` going backwards; a lost `COMMAND_ACK` is normal (PX4 answers and then reboots) and is not treated as failure. `--yes` rather than a prompt because the web GCS runs it with `stdin=DEVNULL`. **This is the close-and-reopen of the XRCE client's UART behind the known NuttX hardfault** — scan the card afterwards. |
| `offboard_engage.py [--status\|--arm\|--disarm\|--hold]` | **The human half of M3.** The bridge never arms and never changes mode; this does. Refuses to command Offboard unless the heartbeat is already >2 Hz and EKF2 has a valid non-dead-reckoned position. Since 2026-09-28 the setpoint stream is checked **after** the switch instead (the bridge sends none before it): with none, it exits 1 and does not arm. `--status` is read-only. Also a ROS 2 script (uXRCE-DDS `VehicleCommand`), not MAVLink. |
| `m51_one_graph_check.py BAG [--only S,G,A,T]` | **The M5.1 scorer** (2026-09-13). Reads a bag of the props-off one-graph test (`docs/m5.1-one-graph-runbook.md`) and scores `m5-m6-plan.md` §2.2 M5.1: **S** ≥ 99 % of TRACK setpoints equal SUPER's command under (x,−y,−z)/ψ→−ψ; **G** ≥ 99 % of `/planning/follow_goal` answered on `/planning/goal_status` ≤ 0.2 s with `requested_p` ≤ 1 cm (a status may be *recorded* up to 50 ms before its goal — measured 19/495 on `run5`); **A** `/dtp/abort` → NAV_LAND ACCEPTED ≤ 1 s → AUTO_LAND with both bridges still publishing (the offboard bridge by its heartbeat since 2026-09-28); **T** 3/3 ch11 takeovers held ≥ 2 s with the bridge's heartbeat up, and (since 2026-09-28) its `trajectory_setpoint` stopped ≤ 0.25 s after each. **Exit 0 only if every requested section was scored and passed; 3 = INCOMPLETE (not a pass).** `--setpoint-topic/--control-mode-topic` score a sandboxed bridge for bench evidence (S only). Reads bags; commands nothing; needs `px4_msgs`/`dtp_msgs`/`mars_quadrotor_msgs` sourced. |
| `goal_forward.py [--dry-run] [--distance M] [--height M] [--allow-disarmed]` | **Publishes one real `/goal_pose`**, relative to current FAST-LIO position and nose heading. Default request is 1 m forward at 0.5 m; the installed `click_height` must match, so the recorded 1.5 m click config refuses that default. Checks live odometry, planner/bridge presence, valid PX4 position and armed+Offboard; `--allow-disarmed` permits deliberate bench work. `--dry-run` sends no goal. SUPER may snap the goal up to 3 m; inspect its accepted goal. Ctrl-C does not stop the aircraft; abort with the RC. ROS 2, not MAVLink. |

Requires `pymavlink` (system-wide). Not in apt for jammy — `sudo pip3 install pymavlink`.
The ROS 2 scripts above (`m2_gate_check.py`, `m3_bridge_check.py`, `m3_gate_check.py`,
`offboard_engage.py`, `goal_forward.py`, `estimator_preflight.py`, `m51_one_graph_check.py`)
need ROS/workspaces sourced in order: `/opt/ros/humble/setup.bash`,
`~/ws_livox/install/setup.bash`, then `~/super_ws/install/setup.bash`.

**Correction (2026-09-02):** earlier revisions of this file and of `ImplementationDoc.md` said PX4
refuses **Offboard** without an RC link or `COM_RCL_EXCEPT` bit 2. Measured on the aircraft, that
is wrong — the mode switch was ACCEPTED with `COM_RCL_EXCEPT = 0` and no receiver ever bound. RC
blocks **arming**, not the mode switch. See [`../../docs/fact-offboard-rc-arming.md`](../../docs/fact-offboard-rc-arming.md).

```bash
python3 scripts/px4/px4_shell.py "uxrce_dds_client status"
python3 scripts/px4/probe_params.py
```

For example, the M2 gate uses ROS 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/super_ws/install/setup.bash
python3 scripts/px4/m2_gate_check.py 120 /tmp/motion.csv    # then move the rig, props off
```

**Why a motion test and not a bench check:** a sign error on any axis is invisible while the rig sits
still — both sources hover near zero and every "is it working?" check passes — and it would flip a
control loop the first time M3 commands a setpoint. Correlating against real motion is the only way
to catch it. Results from the 2026-09-02 run are in [`../../docs/ImplementationDoc.md`](../../docs/ImplementationDoc.md) §M2.

## Two gotchas these scripts encode

**Heartbeat targeting.** ARK OS components (comp 191/192/236, `autopilot=8`) also heartbeat on the
shared UDP bus. (On a stock-JetPack board those components are absent, so the bus is quieter — but
keep the filter: it costs nothing and the scripts must work on both boards.) `wait_heartbeat()` grabs whichever lands first and yields sysid 0 (broadcast),
which makes param reads silently return `None`. Always filter for
`autopilot == MAV_AUTOPILOT_PX4 and srcComponent == 1`.

**INT32 params.** PX4 ships integer params bit-packed into MAVLink's float `param_value` field.
Reading `.param_value` directly shows denormal garbage (`1.4153e-43` is really `101`). Decode with
`struct.unpack("<i", struct.pack("<f", raw))` when `param_type` is an integer type.

## Benign states

On USB power with no battery the FC reports `MAV_STATE_UNINIT`, battery `65535 mV`, and EKF
`CONST_POS_MODE` (flags `0xa5`) — because there's no power module and no horizontal aiding
source. Gyro/accel/mag/baro are healthy and it settles to `STANDBY`. Not a fault.
