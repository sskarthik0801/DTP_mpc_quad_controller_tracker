# Repository guidance

## Read first and resolve conflicting history

This is the integration layer for HKU-MaRS SUPER human following on a Livox
MID360, Jetson Orin, ROS 2 Humble, and an ARK FPV flight controller running
PX4 v1.17.0 with uXRCE-DDS. It does not vendor the planner, estimator, driver,
SDK, or PX4 source. M4's camera-based goal stack already exists, in
`drone-dtp/` on branch `dtp-follower-stack` (symlinked into
`~/super_ws/src/drone-dtp`); its integration and flight validation remain open. PX4's "external vision" is LiDAR odometry, not a camera.

Read [CLAUDE.md](CLAUDE.md) as well as this file, and the topic files it indexes in
[.claude/rules/](.claude/rules/), where its session history now lives (split out
2026-09-17). That history, its measurements, rejected approaches, and
explanations remain relevant. Use
[README.md](README.md) for installation and operation,
[docs/ImplementationDoc.md](docs/ImplementationDoc.md) for M0–M5 and bridge
design, and [docs/README.md](docs/README.md) to find the detailed evidence.

Documentation contains historical contradictions. Prefer the implementation
for current software behavior and the latest dated evidence for hardware
history. Do not assume a previous session's running processes, FC mode,
parameter values, IPs, or workspace builds are still current. In particular:

- The NX was reimaged with **ARK OS on 2026-08-31**. Statements that the NX
  has stock JetPack, no ARK services, or uses our own router configuration
  describe the preceding installation.
- M0–M3 have recorded sign-offs, but M2 hand-motion and M3 props-off gates
  are not proof of successful LiDAR-controlled flight.
- The latest flight-plan revisions supersede "nothing has flown": P3 on
  2026-09-02 did not pass (no takeoff, nine EKF position resets); P5 on
  2026-09-04 is **void** because the LiDAR stack was stopped and the aircraft
  hovered on optical flow. Read [docs/flight-test-plan.md](docs/flight-test-plan.md).
- "Root cause resolved" in the fault archive means diagnosed, **not fixed**.
  The archived FC hardfaults remain a documented flight blocker.
- **M5/M6 are planned in [docs/m5-m6-plan.md](docs/m5-m6-plan.md)** (2026-09-12):
  Gate 0 (safe to fly autonomously) → M5 (the DTP follower flies) → M6 (the
  paper's loss-cause classifier, which does not exist in code yet). It has the
  dated schedule, pass thresholds and sample sizes. Gate 0 state, 2026-09-13:
  the `uxrce_dds_client` hardfault is reproduced; `CONFIG_UART7_RXDMA=n`
  FAILED; the client-side no-close-on-reconnect fix is flashed and passed
  30/30 reconnects, but the ≥ 500-cycle count and 14 h soak are open, so it is
  still a blocker. G0.2 (a 3.74 km FAST-LIO error from a second Livox driver)
  is closed and guarded. G0.4 (SUPER patch in `setup.sh`) is done. Still open:
  `max_vel: 5.0` in the follower's `follow_test_ros2.yaml`.
- `px4_bridge.py` was never recovered; its replacement is
  `super_bringup/scripts/px4_offboard_bridge.py`. Do not search for the old
  file or implement its incorrect ENU yaw conversion again.
- The 2026-09-10 DTP findings supersede "M4 is unwritten" and blanket
  "raycasting is disabled" claims. Since 2026-09-12 every SUPER config enables
  it, the follow config included. Read the M4 section below before edits.
- The dashboard now publishes goals behind Admin control and can explicitly
  stop terminal-started nodes. Older prohibitions describe earlier software;
  preserve the current gates and process-ownership rules below.

## Working practices

- Check `git status` before editing; preserve unrelated edits and untracked
  work. `main` is the documented trunk. Do not delete historical branches
  merely because they have been merged.
- Make durable changes here, including reproducible third-party fixes in
  `setup.sh`/`patches/`; a fix made only in an external workspace is lost on
  the next install. Keep runbooks consistent with changed behavior and mark
  historical conclusions as superseded rather than silently reusing them.
- Follow the existing Python 3/rclpy, Bash, ament CMake, and vanilla JS style.
  Keep ROS executable scripts executable and register new installed scripts
  in `super_bringup/CMakeLists.txt`; declare runtime dependencies in
  `super_bringup/package.xml`.
- ROS-sourcing shell wrappers use `set -eo pipefail`, deliberately without
  `set -u`: ROS setup scripts reference unset variables.
- Installation, networking, hardware tests, parameter writes, mode changes,
  and firmware flashing are operational actions, not routine code checks.
  Do not run them just to validate a documentation or syntax change.

## Code and data map

| Path | Responsibility |
| --- | --- |
| `super_bringup/launch/m1_super_lidar.launch.py` | Real FAST-LIO + identity world TF + SUPER FSM; optional RViz; no simulator |
| `super_bringup/scripts/px4_odom_bridge.py` | M2: FAST-LIO pose to PX4 external odometry |
| `super_bringup/scripts/px4_offboard_bridge.py` | M3: SUPER commands to PX4 setpoints and heartbeat |
| `super_bringup/scripts/offboard_viz.py` | PX4 estimate/setpoint as standard `MarkerArray` on `/viz/offboard` |
| `config/mid360.yaml` | ROS 2 FAST-LIO tuning, sensor configuration, internal extrinsics |
| `config/MID360_config.json` | Livox driver sensor/host IPs and UDP ports |
| `scripts/setup.sh` | Fetch pinned dependencies, patch, copy integration package, build |
| `scripts/network_setup.sh` | Persistent NetworkManager LiDAR profile; detects interface; `IFACE` override |
| `scripts/run.sh` | Livox driver followed by M1 launch |
| `scripts/laptop_setup.sh`, `scripts/laptop_viewer.sh`, `rviz/` | Build-free laptop ROS/RViz viewer |
| `scripts/laptop_dtp_viewer.sh`, `scripts/laptop_image_decode.py`, `rviz/dtp_viewer.rviz` | Laptop follower view: person boxes on the camera (from `dtp_image_overlay`) plus DTP markers |
| `scripts/gcs.sh`, `scripts/gcs/` | Jetson browser dashboard, telemetry, process supervision, frontend tests |
| `scripts/gcs/rc.py`, `scripts/gcs/hostpower.py` | Admin-gated goal controls; Jetson power/reboot interlocks |
| `scripts/gcs/bagger.py`, `bags/` | ROS bag naming/listing/disk guards; gitignored recordings |
| `scripts/gemini_camera.sh`, `scripts/gcs/camera.py`, `scripts/gcs/admin.py` | Gemini publisher, existing-stream viewer, server-side Admin control |
| `scripts/set_hostname.sh` | Persistent host/Tailscale name and mDNS interface configuration |
| `scripts/gcs_laptop.sh`, `scripts/gcs/rviz_agent.py` | Optional laptop polling helper to launch RViz |
| `scripts/gcs_service.sh` | Install/status/uninstall dashboard systemd service |
| `scripts/px4/` | MAVLink diagnostics/parameter tools and ROS bridge gates/control helpers |
| `scripts/uart_loopback_test.sh`, `scripts/01_test_uart.sh` | UART diagnostics; establish port ownership before use |
| `patches/fastlio_executor.py` | Idempotent executor, callback-group, queue, locking latency fix |
| `patches/fastlio_frames.py` | Idempotent TF string renames; runs after executor patch |
| `firmware/` | Custom PX4 image, board patch, full parameter restore points |
| `firmware/faults/` | Archived FC crash evidence and decoding notes |
| `docs/` | Runbooks, implementation plan, durable facts, historical session notes |
| `docs/dtp-follower-stack.md` | External M4 source, interfaces, calibration, and 2026-09-10 inspection |
| `docs/paper/` | Uncompiled ICRA/RA-L draft; bench evidence and unmeasured placeholders |

## Build and deployment

Two separate colcon workspaces are used. Source them in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/super_ws/install/setup.bash
```

`bash scripts/setup.sh` is the full installer, with `SUPER_WS`, `WS_LIVOX`,
and `SDK_DIR` overrides. It uses network access and sudo, checks out pinned
commits, and replaces the workspace's `super_bringup` directory. Review it
before running on an existing installation. Pins currently encoded there:

| Dependency | Commit |
| --- | --- |
| `hku-mars/SUPER` | `2ad3419c127a617c6d7df6925e81a14175a9c096` |
| `hku-mars/FAST_LIO` ROS2 | `a4743b095409588842a5b30ddfa27e29d2f99164` |
| `Livox-SDK/livox_ros_driver2` | `6b9356cadf77084619ba406e6a0eb41163b08039` |
| `Livox-SDK/Livox-SDK2` | `6a940156dd7151c3ab6a52442d86bc83613bd11b` |
| `PX4/px4_msgs` release/1.17 | `86d8239e962f6939e05c3737784f60c02fa884db` |

The driver builds from its repository directory with `./build.sh humble`.
SUPER requires `bash src/SUPER/scripts/select_ros_version.sh ROS2` and
`colcon build --symlink-install` in `super_ws`. FAST-LIO and `super_bringup`
are sibling `src/` packages, not additions inside SUPER's checkout.

**The integration package is copied, not symlinked from this repository.**
After editing it, synchronize the relevant files into
`~/super_ws/src/super_bringup/` (check for workspace-only edits first), then:

```bash
cd ~/super_ws
colcon build --packages-select super_bringup --symlink-install
```

`--symlink-install` does not make the original repository copy live.
Rebuilds can restore upstream configuration; check both source and installed
copies of FAST-LIO, Livox, and planner configs before claiming deployment.
FAST-LIO YAML must retain the ROS 2 `/**: / ros__parameters:` structure.

`setup.sh` does not install DTP or apply its
`patches/0001-super-yaw-to-target-and-follow-goal.patch`. A fresh install lacks
the follow-goal subscriber, `GoalStatus`, and `YAW_TO_TARGET`. The external
tree's `scripts/apply_super_patch.sh` uses content anchors; a failing
`git apply --check` against an already modified SUPER is not proof of a broken
installation. `setup.sh` enables raycasting and sets `p_max: 0.95` in
`click_smooth_ros2.yaml`; the follow config's raycasting comes from the DTP
patch. Do not assume a clean install reproduces the current workspace.

Preserve the installer fixes: C++17 for Humble rclcpp; `pcl_conversions` in
both generated and template simulator CMake files; scoped `std::isnan` and
`<cmath>` fixes; `ros-humble-mavros-msgs` (build-time message dependency,
not a MAVROS architecture); `libglfw3-dev`. `ros-humble-rosfmt` does not
exist. Do not broadly rewrite vendored fmt/GLM code.

## Runtime contracts

The sensor pipeline is MID360 → `livox_ros_driver2` → FAST-LIO2 →
`/cloud_registered` and `/lidar_slam/odom` → SUPER/ROG-Map. FAST-LIO alone
owns `/lidar_slam/odom`; never add a competing publisher or run the simulator
in this real-hardware launch. Confirm publisher count is one when relevant.
SUPER normally waits in `INIT` until its first `/goal_pose`; mapping runs
without a goal.

The TF tree is `world → lidar_init → lidar_imu`. Upstream names were
`camera_init` and `body`; the latter is the MID360 **internal IMU**, not the
FC/body origin. ROG-Map ignores TF/frame IDs. Physical mounting geometry is
distinct from a coordinate-convention conversion.

M2 contracts:

- Input `/lidar_slam/odom`; output `/fmu/in/vehicle_visual_odometry`.
- FAST-LIO has an arbitrary gravity-aligned heading, not a North-aligned ENU
  frame. Publish `POSE_FRAME_FRD`: position `(x,y,z) → (x,-y,-z)` and
  quaternion `(w,x,y,z) → (w,x,-y,-z)`.
- Reject nonfinite poses. Preserve measurement time in `timestamp_sample`.
  `EKF2_EV_DELAY=0`: adding ~50 ms double-counts latency already in the stamp.
  The documented `EKF2_DELAY_MAX` must remain at least approximately 100 ms.
- Velocity is deliberately disabled until its frame is verified. Use
  `EKF2_EV_CTRL=11`, not 15. Variance floors are position/orientation 0.01,
  velocity 0.04; do not pass through implausible millimetre-level confidence.
- `/fmu/` bridge QoS uses BEST_EFFORT and TRANSIENT_LOCAL. Match actual
  versioned output topic names, including `vehicle_local_position_v1`.

M3 contracts:

- `/planning/pos_cmd` is `mars_quadrotor_msgs/PositionCommand`, BEST_EFFORT,
  VOLATILE, depth 1. A default RELIABLE subscriber silently fails to match.
- The planner intentionally goes silent outside `FOLLOW_TRAJ`/`EMER_STOP`,
  including at goal completion. Use arrival time for freshness; its header
  timestamp has a documented precision bug.
- Publish `TrajectorySetpoint` and `OffboardControlMode` on a separate
  50 Hz timer (configurable), default command timeout 0.3 s.
- Convert vectors `(x,-y,-z)`, yaw `-psi` wrapped to pi, yaw rate `-yaw_dot`.
  **Never use `pi/2-psi`**: EKF2 and SUPER already share the flipped heading.
- IDLE re-latches the current valid PX4 estimate while not armed+Offboard;
  TRACK uses fresh commands; HOLD freezes the last point when commands stop.
  HOLD must not follow a drifting estimate. Remove feedforward in HOLD.
- Position is the enabled control level. Velocity feedforward defaults on,
  acceleration feedforward off; unused numeric fields use NaN.
- The bridge never arms or changes mode and does not clamp planner speed.
  Current setup/flight-plan values are max velocity 0.1 m/s, acceleration
  0.3 m/s², angular rate 1.5 rad/s, yaw rate 0.5 rad/s. Planner optimization
  limits are soft penalties, not guaranteed hard clamps.

## Hardware, tuning, and known blockers

Treat these as documented configuration/history to verify on the target:

- Current recorded board is a 16 GB Orin NX; the earlier Nano had 8 GB.
  Identify hardware with `/proc/device-tree/model` before assuming a board.
- ARK FPV is **not ARKV6X**. XRCE is TELEM1, FC `/dev/ttyS6` UART7 ↔ Jetson
  `/dev/ttyTHS1`, 3000000 baud, `UXRCE_DDS_CFG=101`. MAVLink is separate USB-C.
  `px4_msgs` must match the firmware release.
- ARK `dds-agent.service` and `mavlink-router.service` own the serial links;
  active router config is `/etc/ark-os/mavlink-router.conf`, not our older
  `/etc/mavlink-router/main.conf`. Use MAVLink UDP 14569 for companion tools
  and 14550 for QGC. Never open or `stty` an owned serial device.
- Tegra HSUART DMA zeroed received payloads. The boot DTB override removes
  `dmas`/`dma-names` from `serial@3100000` to force PIO. JetPack updates or
  reflashes can undo it. Follow `docs/xrce-agent-nx.md` and prove loopback
  with the owner stopped and the physical jumper installed before trusting
  serial captures. `ttyTHS2` needs its own fix if used.
- MID360 is configured as `192.168.1.154`, host `192.168.1.50`; sensor ports
  56100–56500 correspond to host ports 56101–56501. Use persistent NM setup;
  manual IPs can be flushed. Interface names are not portable.
- CustomMsg `xfer_format=1`, scan rate 10 Hz, `point_filter_num=3`, surface
  filter 0.2, map filter 0.5, cube side 200, and `publish.map_en=false` are
  deliberate. `/Laser_map`'s never-cleared buffer caused the RAM leak;
  obsolete comments blaming PCD saving or only the ikd-Tree are superseded.
  First-generation Livox driver/SDK does not support MID360. 100 Hz LiDAR
  destabilized estimation; 50 Hz undid the CPU savings; keep 10 Hz.
- Preserve executor patch callback groups, multithreading, cloud queue
  depth 2, and `sync_packages()` buffer lock. Historical latency improved
  from ~1.4 s to 49–57 ms. Moving-rig load/latency remains unproven; a bench
  does not exercise stale scans being dropped.
- ROG-Map configured ground/ceiling -1.0/3.5 becomes approximately
  **-0.5/2.9** after inflation. Its ~2.03 GB A* allocation is bounded.
  Recurring FSM `No odom` can be its cosmetic 0.1 s threshold at a 10 Hz
  source; independently verify live odometry rather than dismiss all logs.
- M4 exists outside the repo; integration and M5 validation remain open.
  **Raycasting is on in every SUPER config** since 2026-09-12 (user's
  instruction), in source, install, the DTP SUPER patch, and `setup.sh`.
  Clearing is slow by design: one hit outweighs ~55 misses, so an occupied cell
  needs ~31 clean rays through it at `p_max: 0.95` (click config) or ~54 at
  0.98 (follow config). CPU cost has no off-baseline and hardware clearing time
  is unmeasured. **Ask before turning it off again.**
- "vision-only" means GPS aiding off: `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=3`,
  `EKF2_EV_CTRL=11`. GPS hardware exists. Optical flow (`EKF2_OF_CTRL=1`)
  and downward range (`EKF2_RNG_CTRL=2`) are fitted and fused. Other profiles
  do not manage those two settings. Valid local position alone does not
  prove live LiDAR aiding; inspect `cs_ev_pos` and bridge input/output.
- Lever arm `EKF2_EV_POS=(+0.011,-0.023,-0.07)` is MID360 IMU relative to FC
  in body FRD. Internal `extrinsic_T=(-0.011,-0.02329,0.04412)` is different.
  Measured mounting tilt was roll -1.55°, pitch +0.09°; yaw is unmeasured.
  PX4 has no EV mounting-rotation parameter. Physical yaw correction belongs
  in the odometry bridge, not FAST-LIO's internal sensor extrinsics. Compare
  gravity-based roll/pitch independently; EV-defined yaw cannot validate itself.
- FRD makes `heading_good_for_control=False` expected. Recorded PX4 analysis
  says Offboard/Position/Altitude are unaffected. Do not change to NED to
  suppress it. Offboard mode can engage without RC; RC gates arming.
  Dedicated `RC_MAP_OFFB_SW=11` was required for the M3 gate. Props-off
  Offboard arming can immediately command hover thrust and auto-disarm after
  ~10 s (`COM_DISARM_PRFLT`); neither observation proves flight control.
- Custom INA238 firmware enables current sensing: `SENS_EN_INA238=1`,
  `INA238_SHUNT=0.0001`, `BAT1_V_CHANNEL=-2` avoids duplicate battery sources.
  Stock ARK FPV firmware lacks the driver. Read `firmware/README.md` before
  updates; flash was ~98.25% full and current accuracy remains uncalibrated.
- The fault archive holds 35 dumps: 22 field crashes of the XRCE serial-driver
  fault (`uart_close` racing `uart_datareceived`, newest 2026-09-13), one
  SPI1/uORB and one `logger` field fault, and 11 induced on the bench. Disabling
  UART7 RX DMA did not fix it; the client-side fix now on the FC is bench-tested
  but not through the G0.1 gate (`firmware/faults/README.md`). The 3 Mbaud flood
  test wedges the FC on any firmware — do not use it as a regression. Inspect the
  actual FC SD card, not just the repository archive.

## External M4 stack and paper

Read [docs/dtp-follower-stack.md](docs/dtp-follower-stack.md) before M4 work.
Since 2026-09-12 the DTP tree is `drone-dtp/` in this repo, on branch
`dtp-follower-stack` only (pushed to `origin/dtp-follower-stack` (at `f4df2aa` on 2026-09-13; later uncommitted work is not) — push new
commits before trusting them to survive a reflash). `~/super_ws/src/drone-dtp` is a symlink to it and the
packages are built `--symlink-install`, so the repo folder is what the drone
runs; read [drone-dtp/README.md](drone-dtp/README.md) before building or moving
anything. Its ROS packages live under `ros2_ws/src/`: `dtp_msgs`,
`dtp_perception`, `dtp_bringup`, `command_mux`, `follow_planner`. Its
referenced `plan/dtp-imp/*` and its own `CLAUDE.md` were missing.

The pipeline is Gemini 336L + FAST-LIO → YOLO/BoT-SORT perception → target
manager → standoff planner → command mux → patched SUPER → M3 bridge.

Since 2026-09-19/20 the standoff planner (`follow_planner_node`) has
`goal_mode: geometric | nmpc`. `geometric` is the default and the only arm that
has flown. `nmpc` replaces one function with a receding-horizon optimisation of
the goal, optionally with a visibility cost fed by `dtp_msgs/VisibilityField`
from `fsm_node` (SUPER patch 0008). It is bench-gated only: `~/super_ws` was not
rebuilt, and 0008 is not applied to the live SUPER tree (unlike 0001–0007).
Build `dtp_msgs` before `follow_planner`. Two ROS domains are shared by several
gates and must not be used twice at once: 88 (`target_filter_check`,
`loss_oracle_check`, `visibility_field_check`) and 89 (`fence_check`,
`unhit_decay_check`, `loss_conditioning_check`); 90 is `nmpc_check` and
`nmpc_replay`. Read [docs/fact-nmpc-follow-goal.md](docs/fact-nmpc-follow-goal.md)
before changing or retuning it.
The DTP goal path and this repo's M3 bridge have no recorded combined test.
Key contracts:

- Configured frames are `lidar_init` and `lidar_imu`; DTP comments still using
  `camera_init` are historical. Transform detections at their frame stamp.
- `/dtp/targets` publishes even empty arrays; target manager publishes
  `/dtp/follow_target` only while LOCKED/COASTING. Ambiguous reacquisition
  abstains rather than selecting a different person.
- DTP uses `/planning/follow_goal`, `/planning/yaw_target`, and feedback on
  `/planning/goal_status` (`dtp_msgs/GoalStatus`). `follow_goal_en` must be on.
  The follow path preserves Z; the click `/goal_pose` path overwrites/snaps or
  drops goals. NAV enters the mux on `/dtp/nav_goal`.
- `perception_node` builds its organized cloud on the GPU (`cloud_backend: cuda`
  in `dtp_params.yaml`, since 2026-09-13; node default `numpy`). It must stay
  equivalent to the numpy path: `cuda_cloud.compare`/`within_camera_tolerance`
  judge it in `test_cuda_cloud.py`, `scripts/bench_cloud_backend.py` and the
  node's own first-frames self-check. Any GPU path must advance
  `DepthSource.last_stamp_s` or the skew guard blinds every frame.
- The mux's proximity veto reads all `/dtp/targets`, independently of the
  locked target. Goal silence leaves SUPER's stored goal active; DTP stop
  conditions send a nearby HOLD goal. Do not confuse this with M3 HOLD after
  the planner's command stream goes silent.
- The sensor watchdog lives at the top-level launch even with
  `follower:=false`. The debug organized-cloud `fusion_node` defaults off;
  enabling it adds expensive depth subscriptions even without viewers.
- Camera depth is the default; outdoor depth remains unvalidated and the
  referenced `scripts/outdoor_depth_test.py` is missing. Camera/LiDAR
  calibration is bench-only and invalidated by remounting either sensor.
  The EigenTrajectory branch defaults off (`eigen_enabled=False`) and its
  config/checkpoint are outside version control.

During the 2026-09-10 inspection, the follower had run blind for 15 h 57 m:
the LiDAR Ethernet link was down, two drivers existed, and advertised
`/livox/lidar` and `/lidar_slam/odom` produced no messages. Empty targets and
mux `HOLD|SENSORS+ODOM` were the expected response. Recheck the actual wired
interface, sensor reachability, driver ownership and message flow before
interpreting perception results; topic registration alone proves no liveness.

[docs/paper/README.md](docs/paper/README.md) tracks an uncompiled ICRA/RA-L
draft with no flight results. Preserve `\tbd{}` for unmeasured/unwritten work
and `\bench{}` for bench/handheld measurements. Remove a `\bench{}` wrapper
only when replacing its value with flight evidence. Do not promote the
disabled learned predictor, indoor depth measurements, or draft novelty claims
to validated flight results.

## Diagnostics and validation

There is no repository-wide package-manager test command or CI configuration.
Use checks appropriate to the files changed; report missing ROS/hardware
prerequisites rather than treating unavailable gates as passes.

```bash
# Required after changing the Offboard bridge; source ROS/workspaces first.
python3 scripts/px4/m3_bridge_check.py

# Required after changing drone-dtp follow_planner (either goal_mode); real node, domain 90, ~9 min.
python3 drone-dtp/scripts/nmpc_check.py
python3 drone-dtp/scripts/loss_conditioning_check.py --goal-mode nmpc   # and --goal-mode geometric
# Required after SUPER patch 0008 (the loss oracle's visibility field); real fsm_node, domain 88.
python3 drone-dtp/scripts/visibility_field_check.py

# GCS frontend render behavior against a running API; requires gjs.
python3 scripts/gcs/test_render.py

# Targeted GCS checks; inspect each harness before use.
python3 scripts/gcs/test_supervisor.py  # stand-ins, no ROS/FC; includes adopt regression
python3 scripts/gcs/test_camera.py     # camera interlock/JPEG, no camera required
python3 scripts/gcs/test_admin.py      # server-side camera and recording gates
python3 scripts/gcs/test_rc.py         # goal signs/admin gate; own server, --no-ros
python3 scripts/gcs/test_hostpower.py  # own server, GCS_POWER_DRY_RUN=1

# Read-only FC diagnostics; require the configured MAVLink link.
python3 scripts/px4/probe_params.py
python3 scripts/px4/ekf2_profile.py --show
python3 scripts/px4/diag_health.py
python3 scripts/px4/clear_fault_logs.py
python3 scripts/px4/px4_shell.py "uxrce_dds_client status"
```

The M3 bridge check exercises the real node with topics under `/m3_check/`,
needs ROS but no FC, cannot arm, and historically passes 16/16 in ~15 s.
`m2_gate_check.py` is a passive ROS motion comparison and refuses a stationary
pass; rerun when frames/mounting/aiding change. `m3_gate_check.py` is passive
hardware recording with nine current checks, including live LiDAR and EV
position fusion; old 7/7 results predate these additions. It rejects missing
LiDAR after preflight; `ALLOW_NO_LIDAR=1` is only a deliberate flow baseline.
Gate output does not replace reviewing motion, resets, aiding, or the flight
plan. Use `docs/fact-fastlio-latency.md`'s frame-matched measurement method.

`dump_params.py` is the full QGC-format backup; take one into `firmware/`
before any parameter change. `probe_params.py` is only a shortlist. Read
parameters back after FC reboot; an immediate read-back did not always mean
persistence. `set_params.py`, `ev_lever_arm.py`, `power_module_params.py`,
and `ekf2_profile.py` manage their respective settings; inspect modes before
invoking them. `offboard_engage.py` sends real mode/arm commands except in
status mode. `fake_odom_pub.py` publishes synthetic odometry; it is not a
read-only diagnostic. `goal_forward.py` publishes a real relative goal unless
`--dry-run`; it checks click height and defaults to requiring armed+Offboard.
`--allow-disarmed` is for deliberate bench work. Inspect helpers before use;
Ctrl-C on a goal publisher does not stop the aircraft.

`clear_fault_logs.py` defaults to read-only. Its `--clear` path archives,
verifies, then deletes from the card and preserves pending BBSRAM evidence.
Retain and commit archived evidence; do not bypass it with `--no-archive`.

MAVLink clients must filter heartbeats to PX4 autopilot/component 1, rather
than accepting ARK component heartbeats. Decode integer parameters with
`struct.unpack("<i", struct.pack("<f", raw))` when the type is integer.
Use `listener trajectory_setpoint`, `offboard_control_mode`, `health_report`
through `px4_shell.py` to inspect what the FC actually receives.

## Dashboard and remote operation

`RVIZ=false bash scripts/run.sh` starts M1 headlessly; explicitly set it
because the current wrapper defaults `RVIZ` to true. Start the odom bridge,
Offboard bridge, and visualization separately as appropriate to the task.
Use the flight plan's dependency order and reverse shutdown order; do not
leave the odom bridge attached while restarting FAST-LIO into a new origin.

`bash scripts/gcs.sh` starts the stdlib HTTP/vanilla JS dashboard. Preserve
its offline/no-pip/no-npm/no-CDN design. It manages stack processes, telemetry,
checks, camera viewing, ROS bags, and interlocked FC/Jetson reboots or Jetson
poweroff. **Admin control now permits `/goal_pose` publication, which can move
the aircraft.** It cannot arm/disarm, change mode/parameters, or engage
Offboard. Archive-and-clear crash dumps remains explicitly confirmed in the UI.

The API uses a persistent token at `~/.cache/super-gcs/token`, via header or
`?k=`. The browser remembers it in `localStorage`, offers a paste-once gate
when absent, and forgets rejected tokens; 401 must not appear as link loss.
Do not expose tokens in logs or documentation. It works over
Tailscale; RViz's multicast DDS path normally does not. The optional laptop
RViz helper polls outward, not through a browser-to-localhost server.

The recorded host name is `gypsy`: `http://gypsy.local:8080` on field Wi-Fi,
or its Tailscale MagicDNS name on port 8080. `set_hostname.sh` also configures
Tailscale, `/etc/hosts`, and avahi to exclude the LiDAR interface. This survives
reboots, not reflashes. ARK nginx owns port 80; the 2026-09-07 Tailscale Serve
attempt did not replace it, and tailnet HTTPS certificates were disabled.

Admin control is a separate server-side session gate for camera and RC routes;
locked state/config omit restricted data. Password overrides are at
`~/.cache/super-gcs/admin`, set through `admin.py --set`; never document the
plaintext. Sessions are tab-scoped, expire after 30 min idle, and disappear on
GCS restart. Wrong admin passwords return `200` with `ok:false`, not `401`.

RC controls send one goal relative to current FAST-LIO pose and nose heading.
Fresh odometry is enforced; planner-listener, armed and Offboard indications
are informational, permitting bench goals. There is no browser flight STOP:
abort uses RC channel 11 Offboard OFF, with channel 5 left on Position.
`rc.py` reads the click config (install first, source fallback), not an
arbitrary externally launched DTP config. `click_height: 1.5` disables altitude
controls; enabling message Z changes flight behavior and RViz clicks send Z=0.
Rotate adds a 0.15 m translation to clear SUPER's 0.1 m goal-drop threshold;
it requires both yaw flags and is refused if the inspected `yaw_mode` is 3.
Since 2026-09-12 the click config ships `yaw_mode: 3` (set in `setup.sh`), so
Rotate is refused by default. Mode 3 skips the goal yaw even with no yaw target.
Upstream never reads `yaw_mode`, so 1 and 2 are identical; see
`docs/fact-super-yaw-mode.md`.
Goals can snap up to 3 m; inspect the planner's accepted goal.

FC reboot requires fresh, disarmed telemetry with no in-air report and a
second check in `fc_reboot.py`. Host power/reboot has two browser confirmations,
`?confirm=<mode>`, and `host_power_blocked()`: armed (including stale last-armed)
or fresh in-air evidence blocks, but absent telemetry does not. Preserve the
busy-operation and passwordless-sudo checks, delayed `sudo -n systemctl`
execution and `GCS_POWER_DRY_RUN=1` test path. A Jetson reboot restores only
the dashboard service.

Recordings go to gitignored `bags/`. Start and running disk guards use a 5 GB
free floor. SIGINT the recorder and allow metadata finalization before stack
teardown; missing `metadata.yaml` must remain visible. Restart ends a recording
without resuming it. Locked Admin control hides/refuses image topics and adds
an image-name exclusion to All topics; camera info/depth clouds stay available.
Humble rejects `--exclude` with an explicit topic list. Topic discovery is
on-demand/cached, never part of the regular state poll.

Preserve supervisor invariants: start children with `start_new_session=True`,
`stdin=DEVNULL`; signal owned process groups (SIGINT, 15 s, SIGTERM, 5 s,
SIGKILL), never `pkill -f` or command-line substring matches. `run.sh`'s trap
alone does not reliably stop remote descendants. Match executable argv-token
basenames for detection. Serialize start/stop with `Managed._op`, snapshot
`self.proc` locally in readers, and keep `status()` from clearing it during
stop. Exercise restarts while API polling is active to detect races.

Restart restores only the previously owned running set, with DDS settling and
a stack head start. Shutdown/restart sweeps use `Managed.stop(adopt=False)`;
the 2026-09-10 regression fix prevents closing a test GCS from killing a
terminal-started stack. Explicit row Stop, Stop outside nodes, and STOP ALL
may stop recognized external processes. Adopt a group only if its leader is
recognized; otherwise signal matched PIDs/children, never a login shell or our
own group. The camera row matches only our Gemini wrapper/launch, never DTP's
`gemini_330_series.launch.py`. Keep systemd dashboard-only startup,
`After=network.target`, `KillMode=mixed`, `TimeoutStopSec=90`; it survives apt
updates but must be reinstalled after reimaging.

Telemetry is ROS, not a competing UDP 14569 client. Preserve raw subscriptions,
lazy decoding, browser-activity gating, and hidden-tab polling suspension.
Avoid redundant 100/200 Hz topics or point-cloud subscriptions. Documented
GCS CPU is ~1.5% idle and ~21% of one core watched with the stack; measure on
a dedicated port and wait beyond `IDLE_AFTER=15 s` before measuring idle.
The RC attitude display is a deliberate on-demand 100 Hz exception: the 5 Hz
card poll stops off screen and the attitude subscription expires after 15 s
(measured +8.2% of one core while viewed). Camera MJPEG forwards existing
`CompressedImage` bytes without decoding; disconnect the stream when hidden
and drop the image subscription 6 s after the last viewer. Its 10 fps browser
cap saves bandwidth, not source-subscription CPU. A Gemini has one owner:
watch an existing `/camera` stream instead of launching a competing driver.

Laptop setup uses Ubuntu 22.04/ROS Humble desktop with no colcon build or
`px4_msgs`; expose PX4 visual data as standard messages on the Jetson.
RViz uses fixed frame `world`, shared ROS domain/RMW, and same-L2 Wi-Fi
multicast. `JETSON_IP` is a reachability check, not a DDS destination; use
the Wi-Fi IP, not the dedicated LiDAR Ethernet address. Preserve the viewer's
`env -i` GUI-variable allow-list and ROS re-sourcing: Snap library pollution
can arrive via PATH/XDG/GIO, so merely unsetting `LD_LIBRARY_PATH` is inadequate.
Stale topic listings can come from a remote RViz discovery cache; inspect
nodes and actual publishers.

The follower view (`scripts/laptop_dtp_viewer.sh`) shows perception's boxes on
the camera picture. `perception_node` publishes `/dtp/detections_2d` only while
subscribed; `dtp_image_overlay` draws each set on the frame with the same stamp
and sends a JPEG; `scripts/laptop_image_decode.py` makes the raw Image RViz
needs on the laptop. Humble's RViz cannot show CompressedImage, and
`image_transport republish` subscribes RELIABLE, so it never matches a
best-effort image. Keep the decoder's viewer-only publishing: a Jetson-side
`ros2 bag record -a` would otherwise pull raw frames back over Wi-Fi. Assign
image bytes to `uint8[]` fields as `array.array('B')`, never `bytes`. Non-root `/proc/*/fd` scans can miss ARK-owned
ports; absence there is not proof that a device is unowned.

## Further reading by task

- Sensor pipeline/build: `docs/fact-hardware-pipeline.md`,
  `docs/M1-session-notes.md`, `docs/build-fixes.md`, `docs/fact-fastlio-cpp17.md`.
- Timing/maps: `docs/fact-fastlio-latency.md`,
  `docs/fact-rogmap-bounds-and-memory.md`, `docs/fact-super-cmd-stream.md`.
- EKF/frames/control: `docs/fact-ev-delay.md`,
  `docs/fact-ev-frd-yaw-align.md`, `docs/fact-offboard-rc-arming.md`.
- Serial/network: `docs/uart_bringup_log.md`, `docs/xrce-agent-nx.md`,
  `docs/mavlink-setup.md` (apply the ARK OS correction above).
- Reassembly/flight: `docs/calibrate.md`, `docs/flight-test-plan.md`.
- Power/firmware: `docs/ark-12s-payload-power-module.md`,
  `firmware/README.md`, `firmware/faults/README.md`.
- Dashboard/tool details: `scripts/gcs/README.md`, `scripts/px4/README.md`.
- M4 integration: `docs/dtp-follower-stack.md`; source is `drone-dtp/` on
  branch `dtp-follower-stack`. M5/M6 plan: `docs/m5-m6-plan.md`.
  Paper/evidence rules: `docs/paper/README.md`.
- `docs/super_ws-CLAUDE.md` gives upstream workspace orientation;
  `docs/livoxconnection.md` is historical session reference.
- `docs/plantofly.md` and `docs/mavros-setup-notes.md` are superseded
  ArduCopter/MAVROS plans; `docs/nohardware.md` is partly superseded. Do not
  apply their FC parameters or simulator assumptions to this aircraft.
