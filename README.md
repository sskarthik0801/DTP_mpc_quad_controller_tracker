# SUPER Human-Follow — Bringup

Reproducible setup to run the [HKU-MaRS **SUPER**](https://github.com/hku-mars/SUPER)
planner as the local obstacle-avoiding planner on a **Livox MID360 + NVIDIA Jetson
Orin NX (ROS 2 Humble)** — the first milestone of a vision-based human-following drone.

This repo is the **integration layer only**. It does not vendor SUPER, FAST-LIO, or the
Livox driver; `scripts/setup.sh` fetches those at pinned commits and applies the few
patches/configs needed to make them work together on the Jetson.

## Status / roadmap

| Milestone | What | State |
|---|---|---|
| **M1** | Sensing & state estimation: MID360 → FAST-LIO2 → SUPER/ROG-Map | ✅ verified — **re-verified from scratch on the Orin NX 2026-08-30** (odom 9.98 Hz, RSS flat 5 min); moving-rig endurance still open |
| **M2** | FC link + external-vision odometry into PX4 EKF2 | ✅ **GATE SIGNED OFF 2026-09-02** — EKF2 tracks hand motion against `/lidar_slam/odom` at corr +0.9935/+0.9985/+0.9997 (N/E/D), signs correct on all three axes, 0/1195 invalid samples, 15-min soak flat |
| **M3** | Trajectory bridge (SUPER → PX4 Offboard setpoints) | ✅ **Props-off gate signed off 2026-09-02, historical 7/7**; bench gate 16/16, now 32/32 including the G0.2 odom-bridge guard. Current hardware gate has nine checks. LiDAR-controlled flight remains unproven. |
| M4 | Vision goal adapter (human follow) | Implemented in [`drone-dtp/`](drone-dtp/) on branch `dtp-follower-stack` (since 2026-09-12; pushed to `origin`), symlinked into `~/super_ws/src/drone-dtp`; `setup.sh` now wires it and applies its SUPER patch (a clean-install build has not been run end to end); combined M3 validation remains open. |
| M5 | The DTP follower flies (integration, envelope, safety) | ⏳ planned — [`docs/m5-m6-plan.md`](docs/m5-m6-plan.md). Behind Gate 0: the FC hardfault (client-side fix flashed, 30/30 reconnects, soak pending) and a LiDAR hover; the estimator-divergence guard (G0.2) and SUPER patch install (G0.4) are done. |
| M6 | Loss-cause classification + evaluation (the paper) | ⬜ planned, not implemented — same plan; D1 decision 2026-10-27, full draft 2026-12-18. |

**Update 2026-09-10:** the old "nothing has flown" statement is superseded by
the [flight-test record](docs/flight-test-plan.md). P3 on 2026-09-02 did not
pass: no takeoff and nine EKF position resets. P5 on 2026-09-04 is **void**:
the aircraft hovered on optical flow while the LiDAR stack was stopped.
Neither establishes LiDAR-controlled flight. 35 FC crash dumps are archived
(22 field crashes of the XRCE serial-driver fault through 2026-09-13, 2 other
field signatures, 11 induced on the bench); a client-side fix is flashed and
bench-tested but not yet soak-proven.

**Historical M2 snapshot, 2026-07-29** — real MID360→FAST-LIO odometry reached
PX4. The original "sole position source" interpretation was superseded when
live optical-flow and range aiding were identified on 2026-09-02:

```
uxrce_dds_client: Running, connected · Payload rx 1157 B/s · timesync converged
ekf2 status:      attitude: 1, local position: 1
vehicle_local_position: xy_valid True · z_valid True · heading_var 0.0007
```

⛔ **EKF2 later dropped back to `local position: 0`** because **FAST-LIO's RAM leak** degraded
odometry from 10 Hz to 0.55 Hz. With `EKF2_GPS_CTRL=0` there's **no GPS fallback**, so this was a
flight-safety issue, not a benchmark curiosity.

✅ **RAM leak resolved 2026-08-26** — root cause was `/Laser_map`, a topic with **zero
subscribers**, whose backing buffer (`pcl_wait_pub` in `laserMapping.cpp`) grew every second
forever. Fix: `publish.map_en: false` in `config/mid360.yaml`. An 18-minute re-run (matching the
original crisis window) held RSS flat (net **−3.5 MB**) with `/lidar_slam/odom` steady at 10 Hz
the whole time — see [`docs/ImplementationDoc.md`](docs/ImplementationDoc.md) §M1 regression.
**Later evidence:** the M2 hand-motion gate passed on 2026-09-02. Sustained
moving-rig FAST-LIO endurance and frame-matched latency remain unproven.

> ⚠️ **FC note for M2+:** the target flight controller is an **ARK FPV running
> PX4 v1.17.0 + uXRCE-DDS** (`/dev/ttyTHS1` @ **3000000**, `MicroXRCEAgent` + `px4_msgs` at
> `release/1.17`). ARK FPV and ARKV6X are different board targets.

### External human-following stack (M4)

Read [the DTP integration notes](docs/dtp-follower-stack.md) before adding M4.
The existing Gemini 336L → YOLO/BoT-SORT → target manager → standoff planner
→ command mux stack uses `/planning/follow_goal`, `/planning/yaw_target`, and
`/planning/goal_status` in patched SUPER. Since 2026-09-12 it lives in
[`drone-dtp/`](drone-dtp/) on branch `dtp-follower-stack` (pushed to
`origin`); `~/super_ws/src/drone-dtp` is a symlink to it and its packages are
built `--symlink-install`, so edits there are what the drone runs — see
[`drone-dtp/README.md`](drone-dtp/README.md) for the wiring, build commands
and traps. Since 2026-09-13 `scripts/setup.sh` symlinks `drone-dtp/` into
the workspace and applies its SUPER follow-goal patch (checked piece by piece;
a full clean install has not been run end to end).
The DTP path and this repo's M3 bridge have no recorded combined test.

**`goal_mode: nmpc` (2026-09-19/20, bench-gated, never flown, not installed).** The standoff
planner can compute its goal with a receding-horizon optimisation instead of the closed form
(`follow_planner_node` `goal_mode: geometric | nmpc`; `geometric` is the default and the only one
that has flown), optionally with a visibility cost fed from SUPER's map by a new patch 0008. No
flight binary was rebuilt and 0008 is not applied to the live SUPER tree. `dtp_msgs` must now be
built before `follow_planner`. See [`docs/fact-nmpc-follow-goal.md`](docs/fact-nmpc-follow-goal.md).

**Map clearing: raycasting is on in every SUPER config** (2026-09-12) —
source, install, the DTP SUPER patch, and `setup.sh` for
`click_smooth_ros2.yaml`, which also sets `p_max: 0.95`. Clearing is still
slow by design: one hit outweighs ~55 misses, so a cell someone stood in needs
~31 clean rays through it (~54 at the follow config's `p_max: 0.98`). Its CPU
cost and real-world clearing time are not yet measured.

The 2026-09-10 inspection found a follower running blind for almost 16 hours:
the LiDAR link was down and two drivers existed, despite advertised topics.
Check the wired link, sensor reachability, driver ownership and actual
`/livox/lidar` and `/lidar_slam/odom` messages before trusting perception.
Camera/LiDAR calibration is bench-only, outdoor depth is unvalidated, and
the learned predictor defaults off with weights outside version control.

### ⚠️ The Tegra UART DMA fault (fixed 2026-08-31) — read this before debugging any serial link

**`/dev/ttyTHS1` could not send data to itself.** Tegra HSUART **DMA** on `serial@3100000`
delivered **zero-filled buffers** to the tty layer: correct byte count, zero framing errors,
payload replaced by `0x00`. Writes of ≤4 bytes echoed fine; ≥8 bytes came back all zeros, at
115200 / 921600 / 3000000 alike.

This blocked M2 for weeks and produced **two wrong "the wire is dead" conclusions**, because every
capture was taken *through the broken receiver*. The FC and the harness were fine the whole time.

**Fix — force PIO by removing DMA from that one DT node:**

```bash
cp /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb pio.dtb
fdtput -d pio.dtb /bus@0/serial@3100000 dmas
fdtput -d pio.dtb /bus@0/serial@3100000 dma-names
sudo cp pio.dtb /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv-pio.dtb
# add to LABEL primary in /boot/extlinux/extlinux.conf, right after INITRD:
#   FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv-pio.dtb
sudo reboot
```

Verify: `/proc/device-tree/bus@0/serial@3100000/` must have **no `dmas`**, and

```bash
sudo systemctl stop dds-agent
bash scripts/uart_loopback_test.sh      # jumper TX->RX first: 40-pin header pin 8 -> pin 10
```

must print **PASS**. PIO costs little here — 1286 IRQ/s, agent 7.3% CPU, load 0.66.

> 🔁 **This fix does NOT survive a JetPack upgrade or reflash.** `nv-update-extlinux` rewrites
> `extlinux.conf`, and the stock (DMA-enabled) DTB is still in `/boot/dtb`. An update silently
> restores the broken path — and the symptom looks exactly like a dead wire.
> **Run `bash scripts/uart_loopback_test.sh` after any system update.**
> `ttyTHS2` still declares `dmas`; same fix if the link ever moves there.

**Always loopback-test a UART before trusting anything measured through it.**
Full write-up: [`docs/xrce-agent-nx.md`](docs/xrce-agent-nx.md).

## Architecture (M1)

```
Livox MID360 ──livox_ros_driver2──> /livox/lidar (CustomMsg) + /livox/imu (~200 Hz)
                                              │
                                              ▼
                                     FAST-LIO2 (ROS 2 Humble)
                                     ├─> /cloud_registered  (PointCloud2, WORLD frame, 10 Hz)
                                     └─> /Odometry ─remap─> /lidar_slam/odom (nav_msgs/Odometry)
                                              │                       │
                                              ▼                       ▼
                                  SUPER fsm_node + ROG-Map  (rog_map/occ, rog_map/inf_occ)
```

ROG-Map **ignores `frame_id`/TF** and assumes the cloud is already in the world frame —
FAST-LIO's `/cloud_registered` satisfies this directly. The ROG-Map occupancy grid is a
**local, rolling** window (50×50×6 m, slides with the drone), not a saved global map.

## Hardware

| Role | Part |
|---|---|
| Companion | Jetson **Orin NX** Engineering Reference Devkit, 16 GB, **ARK OS since the 2026-08-31 reimage** (Ubuntu 22.04, ROS 2 Humble). Earlier stock-JetPack notes are superseded. Recorded wired iface: `eno1` @ `192.168.1.50`; `network_setup.sh` auto-detects it. Verify the board with `/proc/device-tree/model`. |
| LiDAR | Livox MID360 @ `192.168.1.154` (internal IMU used by FAST-LIO) |
| Flight controller | **ARK FPV**, PX4 v1.17.0 with the custom INA238 build — XRCE on TELEM1 (FC `/dev/ttyS6`) ↔ Jetson `/dev/ttyTHS1` @ **3000000**, owned by ARK `dds-agent.service`; MAVLink on USB-C, owned by ARK `mavlink-router.service` using `/etc/ark-os/mavlink-router.conf`. |
| Camera | Orbbec **Gemini 336L**, used by the external DTP stack; the dashboard can view its existing `/camera` MJPEG publisher. Only one driver may own the camera. |
| ESC / motors / battery | ARK 4-in-1 AM32 · T-Motor X2807 1300KV · **6S**. **Corrected 2026-08-30:** pack voltage *is* being read — `BAT1_SOURCE=0` (power module ADC, not ESC telemetry), `BAT1_N_CELLS=6`, observed 21.65 V / 28%. Earlier notes claiming no pack voltage / no low-battery failsafe are out of date. |
| Power monitoring | **Updated 2026-09-01:** ARK 12S Payload Power Module with INA238 supplies voltage/current through the custom firmware; `BAT1_V_CHANNEL=-2` disables the earlier duplicate board-ADC source. Current accuracy remains uncalibrated. See [`firmware/README.md`](firmware/README.md) before firmware updates. |
| GPS | **Fitted and working — corrected 2026-08-30.** u-blox UBX on `/dev/ttyS0` @115200, `status: OK`, 10 Hz, `fix_type=4` (DGPS), 9 sats, `eph 0.38 m`. Earlier notes said "none fitted". GPS aiding is off by config (**`EKF2_GPS_CTRL=0`**), not absent hardware; a fallback is available via `scripts/px4/ekf2_profile.py vision-gps`. **Corrected 2026-09-02:** LiDAR odometry is *not* the sole position source — see the optical flow row. |
| Optical flow + range | **Fitted and fused — found 2026-09-02.** Flow and a downward ToF, both on UAVCAN (`sensor_optical_flow` quality 159; `distance_sensor` 0.01–25 m, `orientation: 25`). `EKF2_OF_CTRL=1`, `EKF2_RNG_CTRL=2`, `EKF2_OF_POS_Z=0.14`. Flow supplies **velocity** aiding, which the bridge deliberately does not; it is not a position fallback. Kept on purpose — see [`docs/ImplementationDoc.md`](docs/ImplementationDoc.md) §M2. |

## FC parameter backups

The flight controller's parameters live on the **FC**, not the Jetson, so they survive a board
swap or a Jetson reflash — and are lost by a PX4 reflash or a factory reset. Full dumps are kept
in [`firmware/`](firmware/) (see [`firmware/README.md`](firmware/README.md) for the index):

```bash
python3 scripts/px4/dump_params.py firmware/px4-params-$(date +%F).params
```

Read-only, QGC format, loads back via QGC → Parameters → Tools → Load from file. **Take one before
any parameter change** — `probe_params.py` reads a fixed shortlist and is not a backup. The most
recent restore points are `px4-params-pre-evpos-2026-09-02.params` (before the external-vision
lever arm was set) and `px4-params-post-evpos-2026-09-02.params` (after).

## FC crash dumps (hardfault logs)

A hardfault log is the only evidence that the flight controller **reset itself** — on the bench an
annoyance, in flight a fall. Clear the SD card before a flight day and read it again afterwards, so
any dump that appears in between belongs to *that* flight:

```bash
python3 scripts/px4/clear_fault_logs.py            # read-only: what is on /fs/microsd?
python3 scripts/px4/clear_fault_logs.py --clear    # archive to firmware/faults/, verify, then rm
```

`--clear` downloads each `fault_*.log` into [`firmware/faults/`](firmware/faults/) over MAVLink FTP
and verifies the local copy **before** deleting it from the card. It also commits a fault still
pending in BBSRAM first (`hardfault_log check`/`commit`), so the newest crash isn't silently
dropped. Two rarer flags: `--clear --no-archive` deletes without keeping a copy (it asks first), and
`--reset-counter` clears the boot-time *reboot counter* — separate from deleting the files.

> ⚠️ **Check the card, not the directory.** An empty `firmware/faults/` proves nothing: three dumps
> were missed exactly that way on 2026-09-02. Only the read-only run above says what is actually on
> `/fs/microsd`. As of 2026-09-13 there are **35 archived dumps**: 22 field
> `uxrce_dds_client` serial-driver crashes (newest 2026-09-13), one `wq:SPI1` uORB
> null callback, one `logger` null semaphore, and 11 induced on purpose on the bench.
> A client-side fix is flashed (30/30 reconnects clean) but the soak that would
> clear it as a flight blocker has not run. Evidence is in
> [`firmware/faults/README.md`](firmware/faults/README.md).

## Pinned dependencies (fetched by `setup.sh`)

| Repo | Commit |
|---|---|
| `hku-mars/SUPER` (master) | `2ad3419` |
| `hku-mars/FAST_LIO` (**ROS2** branch) | `a4743b0` |
| `Livox-SDK/livox_ros_driver2` | `6b9356c` |
| `Livox-SDK/Livox-SDK2` | `6a94015` |

## Install on a fresh Jetson

ROS 2 Humble must already be installed.

This installer fetches dependencies, uses sudo and replaces the workspace's
`super_bringup` copy. Review it before running on an existing installation,
especially the locally patched DTP workspace described above.

```bash
git clone <this-repo> ~/super-human-follow
cd ~/super-human-follow
bash scripts/setup.sh                 # clone+patch+build deps, super_ws + ws_livox
sudo bash scripts/network_setup.sh    # persistent MID360 static IP (edit IPs per unit)
bash scripts/run.sh                   # driver + FAST-LIO + SUPER (+ RViz)
#   RVIZ=false            headless (no X server on the Jetson)
#   SUPER_CONFIG=follow_test_ros2.yaml   the M5 follow config (default click_smooth_ros2.yaml)
#   FASTLIO_CPUS=6,7 STACK_CPUS=0-5      CPU pinning, the defaults (M5.3); FASTLIO_CPUS= disables
#   FASTLIO_DDS_PROFILE=nonblocking    the default: FAST-LIO's UDP sends never block on a slow Wi-Fi viewer (docs/fact-fastdds-blocking-send.md)
```

**On a board without ARK OS** (a stock JetPack installation) nothing bridges
MAVLink to the network and `scripts/px4/*.py` cannot reach the FC. Set that up once — build
`mavlink-router` v4 from source, install `pymavlink`, drop in `/etc/mavlink-router/main.conf`:
see **[`docs/mavlink-setup.md`](docs/mavlink-setup.md)**, which also covers connecting
QGroundControl from a laptop.

The current NX has ARK OS: its services already own both serial links and
provide UDP 14569 for companion tools and 14550 for QGC. The manual router
setup above describes the earlier installation; do not install a competing
router on the current image.

### After taking the drone apart
See [`docs/calibrate.md`](docs/calibrate.md) — the scoped re-calibration and re-verification
runbook. Short version: FC params and firmware survive a teardown, **geometry and wiring do not**,
and the motion gate (`scripts/px4/m2_gate_check.py`) is the only step that catches an inverted axis.

### Per-unit changes for a different drone
- **MID360 IP** in `config/MID360_config.json` (`lidar_ip`) — it is `192.168.1.1XX` where
  `XX` = the last two digits of that unit's serial. Pass `LIDAR_IP=...` to `network_setup.sh`.
- **Mounting geometry:** `config/mid360.yaml`'s `extrinsic_T/extrinsic_R` describe
  the MID360's internal LiDAR↔IMU transform. A remount relative to the airframe
  instead requires checking the FC lever arm/mounting rotation and DTP
  camera/LiDAR calibration; follow [`docs/calibrate.md`](docs/calibrate.md).
- **`robot_r`** (robot radius incl. props) and speed limits in SUPER's config for the airframe.

## Run & verify (Gates A/B/C)

After `run.sh`, in RViz set **Fixed Frame `world`** and add `PointCloud2` displays for
`rog_map/inf_occ`, `rog_map/occ`, and `/cloud_registered`.

| Gate | Check | Expected |
|---|---|---|
| **A** driver | `ros2 topic hz /livox/imu` / `/livox/lidar` | ~200 Hz / ~10 Hz |
| **B** odometry | `/cloud_registered` & `/lidar_slam/odom` publishing; pose finite, non-divergent | 10 Hz; cloud world-stable as you walk the rig |
| **C** map ingest | `ros2 topic echo --once --field width /rog_map/occ` (and `inf_occ`) | width > 0 (thousands of voxels); no odom-timeout / below-ground warnings |

The SUPER FSM stays in `INIT` ("Wait for goal") until the **first `/goal_pose`** arrives
(`started_` is set in the goal callback). The map builds regardless — that is M1 done.
To exercise planning, click **2D Goal Pose** in RViz.

## Web GCS (start the stack from a browser)

The stack normally needs three SSH terminals plus a fourth to read battery or
flight mode. Instead, on the Jetson:

```bash
bash scripts/gcs.sh                   # prints a URL with an access token
```

Open that URL on a laptop or phone. Buttons start, stop and **restart** the
SUPER stack and both PX4 bridges; the page shows **flight mode, arming, battery
(voltage, current, %, mAh, temperature), EKF2 health (`xy_valid`,
`dead_reckoning`, `cs_ev_*`, rejects), per-process CPU/RSS, topic rates and
swap** — plus the checks (`m3_bridge_check`, `probe_params`, `diag_health`, the
FC hardfault scan, and **archive + clear the FC's crash dumps**) with their
output tailed live.

The Stack row has a **config chip, click or follow**: which SUPER planner config
`fsm_node` loads at start. **click** (`click_smooth_ros2.yaml`, the default) is
for bench work and manual goals from RViz or the page's Remote control card.
**follow** (`follow_test_ros2.yaml`) is the M5 follow-flight config with the
first-flight envelope and the target filter, and the only one the DTP follower
will fly on. The row shows the config the running node actually loaded.

The **DTP follower** row starts the human-following stack on top of the running
SUPER stack, in one of two modes. **Observe only** (the default) is a dry run:
the follower's goals go to shadow topics that nothing flies. **Live** sends them
to SUPER, arms the NAV_LAND abort and turns the nose toward the person; it needs
Admin control, a confirmed request, a fresh link saying disarmed, `fsm_node` on
the follow config and odometry at rate. Live never arms or changes mode — the
pilot does that on RC channel 11, which is also the takeover. **Live has not yet
been started from the page on the aircraft** (M5.1 is the first use).

*Restart cleanly* restarts the processes this GCS owns, with DDS settling and
the stack started before the bridges. Row Stop, *Stop outside nodes* and
*STOP ALL* can explicitly stop recognized terminal-started nodes. Closing the
GCS or restarting it leaves those outside processes alone; a regression in
that rule was fixed on 2026-09-10.

The **Camera** panel — behind its own password, *Admin control* in the header —
starts the attached Gemini 336L and shows its live picture in the page (MJPEG —
nothing to install on the viewing machine, and it works over Tailscale). The
gate is enforced by the server: a client that has not unlocked it is sent no
camera data at all. Set the password with
`python3 scripts/gcs/admin.py --set`. It offers Start only when the camera is actually free: a Gemini
can be opened by one process at a time, so when something already owns it the
panel names that topic or pid and lets you watch the existing stream instead.
See [the camera guide](docs/geminitest.md) for the launch itself, laptop RViz
and the JPEG topics.

The **Remote control** card also requires Admin control. Each press publishes
one `/goal_pose` relative to current position and nose heading; SUPER plans
the trajectory. It can move an armed aircraft in Offboard. Fresh FAST-LIO
odometry is required; the armed/Offboard/listener indicators allow bench
goal publication. The click config fixes altitude at 1.5 m, so altitude
controls are refused. Rotate also moves 0.15 m to clear SUPER's close-goal
threshold. The abort is **RC channel 11 Offboard OFF** (leave channel 5 on
Position); the browser has no flight STOP command.

**Recording** saves all or selected ROS topics to gitignored `bags/`, with a
5 GB free-space floor. Stop cleanly to finalize `metadata.yaml`; restart or
GCS shutdown stops the recording first and does not resume it. Locked Admin
control excludes camera imagery. **Reboot FC** requires fresh disarmed
telemetry; **Power off/Reboot Jetson** has two confirmations and blocks on
armed or fresh in-air evidence, including a stale last-armed report. Host
shutdown allows absent telemetry for the FC-already-off case.

Cost on the Jetson: **~1.5% of one core when no tab is open or visible, ~21%
while you are watching it with the stack running** — against ~165% for the
stack itself. A hidden tab stops polling and drops back to 1.5%. An open camera
feed adds ~3.4 points on top, and only while it is on screen: the image
subscription exists solely while somebody is watching.
The RC attitude display was measured separately at **+8.2% of one core**
(14.4% → 22.7%) while visible; its subscription expires 15 s after the card
goes off screen. These are bench measurements, not a combined flight budget.

**The laptop needs nothing installed but a browser** — no ROS, no `colcon
build`, no npm. Unlike the RViz path below it also works over **Tailscale**,
because it is HTTP rather than DDS multicast. Telemetry comes off `/fmu/out/*`
rather than MAVLink, so every `scripts/px4/*.py` tool keeps working alongside it.

The dashboard cannot arm/disarm, engage Offboard, change mode or write FC
parameters. Mode and arming actions stay on the RC and
`scripts/px4/offboard_engage.py`.

The recorded host name is **`gypsy`**: use `http://gypsy.local:8080` on field
Wi-Fi or `http://gypsy.tail4e9b1c.ts.net:8080` over Tailscale. After one token
entry the browser remembers it; a bare URL offers a token box. ARK nginx
owns port 80, so retain `:8080` with the recorded configuration.

To have it come back on every boot (dashboard only — it never starts the stack
or anything that flies):

```bash
sudo bash scripts/gcs_service.sh install
```

For the "Open RViz" button, run this on the laptop you want RViz to open on —
quote it, and use the URL the Jetson printed:

```bash
bash scripts/gcs_laptop.sh 'http://192.168.8.130:8080/?k=<token>'
```

That helper polls the Jetson and drives `scripts/laptop_viewer.sh` for you.
Details, CPU numbers and troubleshooting: **[`scripts/gcs/README.md`](scripts/gcs/README.md)**.

## View remotely (laptop RViz)

The M1 stack runs on the Jetson; you can watch it live from a laptop on the **same Wi-Fi**.
ROS 2 auto-discovers over DDS (same `ROS_DOMAIN_ID`, no master). The displayed topics are
standard message types, so the laptop needs only **ROS 2 Humble + RViz — no `colcon build`**.

```bash
git clone <this-repo> ~/super-human-follow && cd ~/super-human-follow
bash scripts/laptop_setup.sh      # one-time: install ROS 2 Humble + RViz (Ubuntu 22.04)
bash scripts/laptop_viewer.sh     # check discovery + open RViz with rviz/m1_viewer.rviz
```

`laptop_viewer.sh` matches the Jetson's DDS settings (`ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`,
`rmw_fastrtps_cpp`) and pings the Jetson (`JETSON_IP`, default `192.168.8.130`) before launching;
override any as env vars. If no topics are discovered it is almost always the network: same Wi-Fi,
AP client-isolation off, laptop firewall off (`sudo ufw disable`), or Wi-Fi multicast blocked
(then use a FastDDS discovery-server). Windows/macOS: use WSL2 or a `ros:humble` Docker image.

### The human-following view: person boxes on the camera picture

```bash
JETSON_IP=gypsy.local bash scripts/laptop_dtp_viewer.sh   # = DTP=1 bash scripts/laptop_viewer.sh
```

Opens `rviz/dtp_viewer.rviz`: everything in the M1 view, plus a **Person boxes** panel (the colour
camera with YOLO's boxes, track IDs, range, confidence, the range-sample point, and a header with
lock state, mux state, detection rate and lag), `dtp_viz`'s 3-D people / lock / forecast markers,
and the follow goal sent to SUPER. Still no build on the laptop; it needs OpenCV or Pillow to
decode the picture (`sudo apt install python3-opencv` if the script says so).

On the Jetson the picture needs the camera, `perception_node` and `dtp_image_overlay`, which
`dtp_system.launch.py` starts. On top of an already-running `scripts/run.sh` stack, bring up only
the DTP half, observe-only so nothing can send SUPER a goal:

```bash
source ~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh
ros2 launch dtp_bringup dtp_system.launch.py lidar:=false super:=false observe_only:=true
```

How the picture gets to RViz, and why it takes this route:

- `perception_node` publishes its boxes on `/dtp/detections_2d` (JSON, pixel coordinates, the
  colour frame's own stamp), but only while something subscribes.
- `dtp_image_overlay` (Jetson) draws each set of boxes **on the frame with the same stamp**, so a
  box never trails a walking person, and publishes `/dtp/image_annotated/compressed` (960 px JPEG,
  ~85 kB, ~10 Hz). It subscribes to nothing until a viewer appears.
- `scripts/laptop_image_decode.py` (laptop, started by the viewer script) turns the JPEG back into
  the raw `/dtp/image_annotated/laptop` that RViz displays. Humble's RViz cannot show a
  CompressedImage, and a raw 720p frame is 2.7 MB — too much for field Wi-Fi.

With perception not running, the panel shows the live camera under an amber "no boxes" banner.
Verified end to end on a bag replay (2026-09-13), not yet in a real laptop RViz window.

## Fixes baked in (and why)

| Fix | Where | Why |
|---|---|---|
| `-std=c++14` → **c++17** | FAST-LIO `CMakeLists.txt` | Humble `rclcpp` 16.x headers are C++17-only; C++14 fails with `invalid field 'callback_variant_'` |
| `xfer_format = 1` (CustomMsg) | livox `msg_MID360_launch.py` | FAST-LIO (`lidar_type:1`) needs per-point timestamps |
| `virtual_ground_height: -1.0` | SUPER `click_smooth_ros2.yaml` | avoid "Odom below virtual ground" rejection at ground level (odom z ≈ 0) |
| Persistent NM profile + **wired-iface auto-detect** | `network_setup.sh` | NM flushes a manual IP → driver `bind failed`. The iface name is **not stable across boards** (`enP8p1s0` on one unit, **`eno1`** on the Orin Nano Engineering Reference devkit) — the script now detects it (override with `IFACE=`). |
| `point_filter_num: 1 → 3`, `cube_side_length: 1000 → 200`, `det_range: 100 → 70` | `config/mid360.yaml` | **2026-07-29:** at `point_filter_num:1` FAST-LIO pegged a core at 100% and grew to 2.97 GB in 18 min, swap-thrashing the Jetson and collapsing odom to 0.55 Hz. Cut CPU to ~64% but RAM kept growing ~0.55 MB/s — see the next row for why. |
| `publish.map_en: true → false` | `config/mid360.yaml` | **2026-08-26:** the residual RAM growth above was `/Laser_map` — a topic with **zero subscribers**, whose buffer (`pcl_wait_pub` in `laserMapping.cpp`) is appended to every second and never cleared. Disabling it held RSS flat over an 18-min re-run. **This closes the M1 RAM regression** (stationary case). |
| `UXRCE_DDS_CFG=101`, `SER_TEL1_BAUD=3000000`, `MAV_0_CONFIG=0` | PX4 params (flash) | Starts the XRCE client on TELEM1 at the baud ARK OS's agent already uses. **This was the real M2 blocker** — it had been `0` the whole time. |
| Drop `ros-humble-rosfmt`; add `ros-humble-mavros-msgs` + `libglfw3-dev`; correct the Livox `build.sh` path; script the two `build-fixes.md` patches | `scripts/setup.sh` | **2026-08-30:** `setup.sh` could not complete on a clean machine. `rosfmt` does not exist for Humble and aborted the whole apt transaction; `mavros_msgs`/`glfw3` are undeclared deps of `mission_planner`/`marsim_render` that only resolved on the Nano because MAVROS was left over from the ArduCopter era. |
| `EKF2_EV_CTRL=11`, `EKF2_HGT_REF=3`, `EKF2_GPS_CTRL=0` | PX4 params (flash) | LiDAR external position/yaw/height aiding, GPS aiding off. Flow and range are also fused by the `vision-only` profile. `11` not `15` — the bridge deliberately omits velocity. Switch with `scripts/px4/ekf2_profile.py`. |

> ⚠️ **A `colcon build` of `fast_lio`, or a Livox driver rebuild, silently reverts
> `config/mid360.yaml` and `config/MID360_config.json` to upstream defaults.** Both were found
> reverted on 2026-07-29 — the Livox config had stock IPs (`.5`/`.12`) so the driver could never
> connect, and `mid360.yaml` was in **ROS 1 format**, which made FAST-LIO *crash at startup*
> (`Cannot have a value before ros__parameters`). Re-run `scripts/setup.sh` after any such build.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| **Odom rate decays over minutes** (10 Hz → 1 Hz) while `/livox/lidar` stays 10 Hz; system swapping | **FAST-LIO unbounded RAM growth — resolved 2026-08-26** for the stationary case: `publish.map_en` was leaking `/Laser_map`'s buffer (zero subscribers). Confirm `config/mid360.yaml` has `map_en: false` and it's synced to `~/super_ws/install/.../mid360.yaml`. If you still see this with that config in place, it's a new issue — check `ps -o rss -p $(pgrep -x fastlio_mapping)` and see `docs/ImplementationDoc.md` §M1 regression. |
| **PX4 loses position**, or position looks valid with no LiDAR | Check actual LiDAR/odom message flow, bridge input/output and `cs_ev_pos`. Flow/range can keep local position valid while LiDAR aiding is absent; validity alone does not prove the integration works. |
| Follower publishes empty targets despite topics being listed | Check sensor-link carrier/IP, reachability, duplicate Livox drivers and actual topic rates. The 2026-09-10 inspection found registered publishers with no messages; see [`docs/dtp-follower-stack.md`](docs/dtp-follower-stack.md). |
| FAST-LIO exits instantly: `Couldn't parse params file ... Cannot have a value before ros__parameters` | `mid360.yaml` got reverted to the upstream **ROS 1** format by a rebuild. Re-run `scripts/setup.sh` (or copy `config/mid360.yaml` into `~/super_ws/src/FAST_LIO_ROS2/config/` **and** `~/super_ws/install/fast_lio/share/fast_lio/config/`). |
| Livox driver connects to nothing / no `/livox/*` topics | Workspace `MID360_config.json` reverted to stock IPs (`192.168.1.5` host, `.12` lidar). Copy `config/MID360_config.json` into `~/ws_livox/src/...` **and** `~/ws_livox/install/...`. |
| `ros2 topic list \| grep fmu` is empty | The XRCE client isn't connected. `python3 scripts/px4/px4_shell.py "uxrce_dds_client status"` — if `Running, disconnected`, **first prove the UART works at all**: stop `dds-agent`, jumper TX→RX (header pin 8 → pin 10) and run `bash scripts/uart_loopback_test.sh`. If that FAILs, the Tegra DMA fault is back (see the callout above / [`docs/xrce-agent-nx.md`](docs/xrce-agent-nx.md)) — most likely a JetPack update reverted `extlinux.conf`. Only if the loopback PASSes is it worth checking wiring and `UXRCE_DDS_CFG`. **Do not** open `/dev/ttyTHS1` while an agent is running. |
| **Serial capture is nearly all `0x00`** with a correct byte count and **zero framing errors** | Classic Tegra UART DMA fault — *nothing* measured through that port can be trusted, including any conclusion about the wire or the far end. Run `bash scripts/uart_loopback_test.sh`. See the callout above. |
| `device reports readiness to read but returned no data` on `/dev/ttyACM0` | Literal: `mavlink-routerd` already owns that port. Use the MAVLink UDP endpoint **14569** instead. |
| QGroundControl can't find the FC over the network | On a stock-JetPack board nothing routes MAVLink until you set up `mavlink-router` — [`docs/mavlink-setup.md`](docs/mavlink-setup.md). The `qgc` endpoint is a UDP **server**, so QGC must add the Jetson as a *target host*; auto-connect alone won't find it. Use the Jetson's Wi-Fi or Tailscale address, **not** `192.168.1.50` (that's the LiDAR link). |
| Odom blows up to huge values; `cur_pose out of map range`; `/cloud_registered` stalls | MID360 **point stream stalled** — check `ros2 topic hz /livox/lidar` (dead while `/livox/imu` still ~200 Hz). Restart the Livox driver, then restart FAST-LIO. |
| `rviz2: undefined symbol __libc_pthread_init` (`/snap/.../libpthread`) | Snap library pollution can arrive through PATH/XDG/GIO as well as library variables. Use `laptop_viewer.sh`, which launches with an `env -i` GUI allow-list and re-sources ROS; the older LD_LIBRARY_PATH-only workaround is superseded. |
| Laptop discovers no topics | Network: same Wi-Fi? AP client-isolation? laptop `ufw`? Wi-Fi multicast blocked → use a FastDDS discovery server. |
| **Person boxes** panel stays black | Walk the chain: `ros2 topic hz /dtp/image_annotated/compressed` on the laptop (nothing → `dtp_image_overlay` or the camera is not running on the Jetson; the overlay log says `a viewer subscribed -- drawing` once it sees you), then `/dtp/image_annotated/laptop` (nothing → the decoder; its log names a non-RViz subscriber if it has paused itself, see next row). Amber "no boxes" banner → perception is not publishing `/dtp/detections_2d`. |
| Decoder log: `NOT publishing: <node> subscribes ... and is not RViz/rqt` | Deliberate. Something other than a viewer (typically `ros2 bag record -a`, possibly on the Jetson) subscribed to the raw picture, which would send ~12 MB/s back over the Wi-Fi. Output resumes when it leaves; `--any-subscriber` overrides. |
| Driver `bind failed` | Host IP missing on the LiDAR subnet (NetworkManager flushed a manual IP). Re-run `scripts/network_setup.sh`. |
| FSM stuck in `INIT` | Normal — it waits for the first `/goal_pose`. Click "2D Goal Pose" in RViz. |

## Layout

```
super_bringup/                 ROS 2 package: launches + bridge nodes
  launch/m1_super_lidar.launch.py   M1 stack (FAST-LIO + TF + fsm_node, no sim)
  scripts/px4_odom_bridge.py        M2: /lidar_slam/odom -> /fmu/in/vehicle_visual_odometry
  scripts/px4_offboard_bridge.py    M3: /planning/pos_cmd -> /fmu/in/trajectory_setpoint
  scripts/offboard_viz.py           setpoint vs estimate as markers, so laptop RViz can see it
config/mid360.yaml             FAST-LIO MID360 profile (extrinsics + resource tuning)
config/MID360_config.json      Livox driver network config (set lidar IP per unit)
scripts/setup.sh               reproducible installer (pinned deps + patches + build)
scripts/network_setup.sh       persistent MID360 static IP (auto-detects wired iface)
scripts/run.sh                 bring-up: driver + M1 stack
scripts/gcs.sh                 web GCS: browser control panel (start/stop + telemetry)
scripts/gcs_laptop.sh          laptop: open the GCS in a browser + enable its RViz button
scripts/gcs_service.sh         install the GCS as a systemd service (starts at boot)
scripts/gcs/                   the GCS itself -- stdlib HTTP server, rclpy node, supervisor
  rc.py / hostpower.py           goal controls and Jetson power interlocks
  bagger.py                      bag naming/listing and disk guard helpers
bags/                          ROS recordings (gitignored)
scripts/laptop_setup.sh        laptop: install ROS 2 Humble + RViz (Ubuntu 22.04)
scripts/laptop_viewer.sh       laptop: open RViz against the Jetson's live topics (DTP=1: follower view)
scripts/laptop_dtp_viewer.sh   laptop: the follower view -- person boxes on the camera + DTP markers
scripts/laptop_image_decode.py laptop: JPEG from the Jetson -> raw Image for RViz (started by the above)
rviz/                          m1_viewer.rviz (M1), dtp_viewer.rviz (follower), gemini_viewer.rviz
scripts/px4/                   FC tools over MAVLink UDP 14569, plus ROS bridge/goal helpers
                               (current NX: ARK router, /etc/ark-os/mavlink-router.conf)
  px4_shell.py                   run commands on PX4's NuttX shell (like QGC's console)
  probe_params.py                read the params that matter for M2/M3
  ekf2_profile.py                switch EKF2 aiding: vision-only | vision-gps | stock
  diag_health.py                 battery / sensor health / estimator flags
  fake_odom_pub.py               synthetic odom, to bench-test the bridge without a LiDAR
  set_params.py                  phase-1 uXRCE-DDS transport params
  m2_gate_check.py               THE M2 GATE: does EKF2 track the LiDAR? (ROS 2, not MAVLink)
  m3_bridge_check.py             THE BRIDGE BENCH GATE: sandboxes both PX4 bridges, 32 asserts
  estimator_preflight.py         G0.2: 60 s GO/NO-GO on the estimate -- run before every arm
  offboard_engage.py             engage Offboard / arm / disarm -- the human half of M3
  m3_gate_check.py               THE M3 GATE: records Offboard engage/arm/hold/takeover (passive)
  clear_fault_logs.py            archive FC crash dumps to firmware/faults/, then clear the SD card
  goal_forward.py                one relative forward goal; --dry-run sends nothing
rviz/m1_viewer.rviz            RViz config for the remote viewer
```

> ⚠️ **Never open the FC serial devices directly** — a second reader corrupts the stream, and
> `stty` is per-device, not per-fd. On the current **Orin NX, reimaged with
> ARK OS on 2026-08-31**, ARK's `dds-agent.service` and
> `mavlink-router.service` own the links. Earlier stock-JetPack instructions
> using our router config are superseded for this installation. Use the MAVLink UDP
> endpoints — **14569 (`companion_aux`)** for `scripts/px4/`, **14550** for QGroundControl.
> See [`scripts/px4/README.md`](scripts/px4/README.md).

## Documentation

- [`CLAUDE.md`](CLAUDE.md) — orientation for a fresh Claude Code session (status + gotchas).
- [`.claude/rules/`](.claude/rules/) — the topical detail behind it (FAST-LIO, PX4, SUPER, the
  follower, the GCS, the flight log), one file per area. Claude Code loads each one when a
  matching file is read.
- [`AGENTS.md`](AGENTS.md) — repository working rules and current integration contracts.
- [`docs/dtp-follower-stack.md`](docs/dtp-follower-stack.md) — the M4 implementation,
  follow-goal patch, calibration and open integration work.
- [`drone-dtp/README.md`](drone-dtp/README.md) — how the DTP tree is symlinked into `~/super_ws`,
  and how to edit, build, run and test it (branch `dtp-follower-stack`).
- [`docs/paper/README.md`](docs/paper/README.md) — uncompiled ICRA/RA-L draft; preserve
  `\tbd{}` placeholders and `\bench{}` distinctions from flight evidence.
- [`docs/ImplementationDoc.md`](docs/ImplementationDoc.md) — the M0–M5 plan and **current source
  of truth for M2/M3**.
- [`docs/xrce-agent-nx.md`](docs/xrce-agent-nx.md) — **the Tegra UART DMA fault and its fix**, plus
  every eliminated cause. Read before debugging any serial link here.
- [`scripts/uart_loopback_test.sh`](scripts/uart_loopback_test.sh) — proves a port can echo itself.
  **Run this first**, and after any system update.
- [`docs/uart_bringup_log.md`](docs/uart_bringup_log.md) — the FC-link post-mortem. Worth reading
  before touching any serial device on this machine.
- [`docs/mavlink-setup.md`](docs/mavlink-setup.md) — MAVLink/QGC endpoint layout and historical
  manual installation; apply the current ARK OS ownership correction above.
- [`scripts/px4/README.md`](scripts/px4/README.md) — FC tooling and the port-ownership rule.
- [`firmware/faults/README.md`](firmware/faults/README.md) — the 35 archived FC crash dumps and the G0.1 fix record,
  decoded, and how to pull/clear new ones off the SD card.
- [`docs/`](docs/README.md) — full index, including superseded ArduCopter/MAVROS-era notes.

## Credits & licenses

Built on [SUPER](https://github.com/hku-mars/SUPER) and [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
(HKU-MaRS, GPL-2.0) and the [Livox SDK/driver](https://github.com/Livox-SDK). Original
integration code here is GPL-2.0 (see `LICENSE`); third-party sources keep their own licenses.
