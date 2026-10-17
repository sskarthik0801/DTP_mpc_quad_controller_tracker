> ⚠️ **PRIOR-SESSION LOG — partly superseded (flagged 2026-07-29).**
> The Livox connection / FAST-LIO / SUPER material here is still a useful reference, but the
> **ArduCopter bridge** sections are obsolete: the FC runs **PX4 v1.17.0 + uXRCE-DDS**, and the
> odometry bridge is `px4_odom_bridge` → `/fmu/in/vehicle_visual_odometry`.
>
> Two specifics that are now wrong: the wired iface is **`eno1`** on this board (not `enP8p1s0`),
> and the LiDAR IP must be set via the persistent NetworkManager profile in
> `scripts/network_setup.sh` — a manual `ip addr add` is flushed within ~1 min.
>
> **Current source of truth:** [`ImplementationDoc.md`](ImplementationDoc.md), [`../README.md`](../README.md), [`../CLAUDE.md`](../CLAUDE.md).

# Livox MID-360 — Full Session Log
## Connection, FAST-LIO2, SUPER Planner, Hybrid Simulation & ArduCopter Bridge

**Hardware:** Jetson Orin Nano (ROS2 Humble) + Livox MID-360 via Ethernet + ARK FPV FC (ArduCopter latest, May 2026)

---

## What Was Built (End State)

| Component | Location | Status |
|-----------|----------|--------|
| Livox ROS2 driver | `/home/ideas-ad/ws_livox` | Built, working |
| FAST-LIO2 (ROS2 branch) | `/home/ideas-ad/super_ws/src/FAST_LIO_ROS2` | Built, working |
| SUPER planner | `/home/ideas-ad/super_ws/src/SUPER/super_planner` | Built, working |
| Hybrid sim launch | `/home/ideas-ad/super_ws/src/SUPER/mission_planner/launch/real_lidar_sim.launch.py` | Working |
| RViz config | `/home/ideas-ad/livox_view.rviz` | Full pipeline view |
| MAVROS launch + config | `/home/ideas-ad/jetson_px4_setup/` | Created, **UART not wired yet** |
| Bridge node (SUPER → FC) | `/home/ideas-ad/super_ws/src/` | **NOT YET WRITTEN** |

---

## One-Time Setup (Already Done)

### 1. Ethernet — Jetson interface is `enP8p1s0`, NOT `eth0`

The guide uses `eth0` but the Jetson Orin Nano uses predictable names.
Check with: `ip link show`

Assign IP temporarily (resets on reboot):
```bash
sudo ip addr add 192.168.1.50/24 dev enP8p1s0
sudo ip link set enP8p1s0 up
```

Make permanent via netplan:
```bash
sudo bash -c 'cat > /etc/netplan/02-livox.yaml << EOF
network:
  version: 2
  ethernets:
    enP8p1s0:
      addresses: [192.168.1.50/24]
      dhcp4: false
EOF'
sudo netplan apply
```

Verify:
```bash
ip addr show enP8p1s0   # must show: inet 192.168.1.50/24
ping -c 3 192.168.1.154 # must show: 0% packet loss
```

> **Note:** `sudo ip addr add` requires an interactive desktop terminal — it cannot be run via Claude Code (no password prompt). Open a terminal on the Jetson desktop.

---

### 2. Fix ws_livox build conflict

A leftover directory blocked `--symlink-install`. Fix and rebuild:
```bash
rm -rf /home/ideas-ad/ws_livox/build/livox_ros_driver2/ament_cmake_python/livox_ros_driver2/livox_ros_driver2

source /opt/ros/humble/setup.bash
cd /home/ideas-ad/ws_livox
colcon build --symlink-install
# Expected: Summary: 1 packages finished
```

---

### 3. xfer_format — must be 1 (CustomMsg) for FAST-LIO2

**Issue found and corrected:** The nohardware.md guide said to set `xfer_format = 0` (PointCloud2). This was wrong. FAST-LIO2 with `lidar_type: 1` subscribes to `livox_ros_driver2/msg/CustomMsg`, not `sensor_msgs/msg/PointCloud2`. With mismatched types, no data flows.

**Correct setting:** `xfer_format = 1`

File: `/home/ideas-ad/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`
```python
xfer_format   = 1    # 1 = CustomMsg (required for FAST-LIO2 lidar_type: 1)
```

Verify:
```bash
grep "xfer_format" /home/ideas-ad/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
# Expected: xfer_format   = 1
```

---

### 4. Install FAST-LIO2 (ROS2 branch)

**Note:** FAST-LIVO2 was considered but rejected — it is ROS1 only (catkin) and requires a camera. Used `FAST_LIO` ROS2 branch instead.

```bash
cd /home/ideas-ad/super_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git FAST_LIO_ROS2 --branch ROS2 --depth 1

# Init submodule (ikd-Tree — not included in --depth 1 clone)
cd FAST_LIO_ROS2
git submodule update --init --recursive

# Build
source /opt/ros/humble/setup.bash
source /home/ideas-ad/ws_livox/install/setup.bash
cd /home/ideas-ad/super_ws
colcon build --symlink-install --packages-select fast_lio
# Expected: Summary: 1 packages finished
```

**Issues hit during build:**
- First build failed: stale CMake cache from a previous path → `rm -rf build/fast_lio install/fast_lio` then rebuild
- Second build failed: `ikd_Tree.cpp` not found → submodule was empty because `--depth 1` skips submodules → fixed with `git submodule update --init --recursive`

---

### 5. Topic remap in FAST-LIO2 launch file

SUPER expects `/lidar_slam/odom` but FAST-LIO2 publishes `/Odometry`.
Remap added to `/home/ideas-ad/super_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py`:

```python
fast_lio_node = Node(
    package='fast_lio',
    executable='fastlio_mapping',
    parameters=[...],
    remappings=[('/Odometry', '/lidar_slam/odom')],  # added
    output='screen'
)
```

---

### 6. Dense point cloud settings in FAST-LIO2

Default settings produced a sparse `/cloud_registered`. Changed in
`/home/ideas-ad/super_ws/src/FAST_LIO_ROS2/config/mid360.yaml`:

| Parameter | Before | After | Effect |
|-----------|--------|-------|--------|
| `point_filter_num` | 3 | 1 | Keep every point (not every 3rd) |
| `filter_size_surf` | 0.5 | 0.2 | Finer voxel grid |
| `dense_publish_en` | false | true | Publish full scan, not downsampled map cloud |

---

### 7. SUPER planner virtual_ground_height fix

The drone sits at z ≈ 0. The ROG-Map's effective ground (after inflation) was 0.25 m, causing constant `Odom below virtual ground` warnings and map rejection.

Changed in `/home/ideas-ad/super_ws/src/SUPER/super_planner/config/static_dense.yaml`:
```yaml
virtual_ground_height: -1.0   # was -0.1
```

---

### 8. super_ws Build Fixes (4 issues before colcon succeeded)

All fixes recorded in full in `/home/ideas-ad/super_ws_fixes/fixederror.md`.

#### Fix 8a — Duplicate package names (SUPER tree at workspace root)

**Error:**
```
ERROR:colcon: Duplicate package names not supported:
- marsim_render: SUPER/mars_uav_sim/marsim_render AND src/SUPER/mars_uav_sim/marsim_render
- mission_planner: SUPER/mission_planner AND src/SUPER/mission_planner
```
**Root cause:** A copy of the SUPER tree existed at `super_ws/SUPER/` (workspace root) alongside the correct `super_ws/src/SUPER/`. colcon found both.

**Fix:**
```bash
touch /home/ideas-ad/super_ws/SUPER/COLCON_IGNORE
```

#### Fix 8b — ROS1 catkin `FAST_LIO` at workspace root

**Error:**
```
CMake Error: Could not find a package configuration file provided by "catkin"
Failed <<< fast_lio
```
**Root cause:** `super_ws/FAST_LIO/` is the original ROS1 catkin package placed at the workspace root. colcon (ROS2) does not provide catkin.

**Fix:**
```bash
touch /home/ideas-ad/super_ws/FAST_LIO/COLCON_IGNORE
```

#### Fix 8c — `pcl_conversions/pcl_conversions.h` not found in `perfect_drone_sim`

**Error:**
```
fatal error: pcl_conversions/pcl_conversions.h: No such file or directory
```
**Root cause:** `perfect_drone_sim/CMakeLists.txt` never called `find_package(pcl_conversions REQUIRED)`.

**Fix:** Added to `super_ws/src/SUPER/mars_uav_sim/perfect_drone_sim/CMakeLists.txt`:
```cmake
find_package(pcl_conversions REQUIRED)   # line 34
# and added pcl_conversions to ament_target_dependencies block
```

#### Fix 8d — `isnan` undeclared / ambiguous in C++17

**Error:**
```
error: 'isnan' was not declared in this scope
note: suggested alternatives: 'std::isnan', 'Eigen::isnan', 'Eigen::numext::isnan'
```
**Root cause:** C++17 no longer guarantees `<cmath>` functions in the global namespace. Eigen also injects its own `isnan` into multiple namespaces, making bare `isnan(x)` ambiguous.

**Fix:** Added `#include <cmath>` and replaced all bare `isnan(` with `std::isnan(` across:
| File | Calls fixed |
|------|-------------|
| `super_planner/src/super_core/fsm.cpp` | 4 |
| `super_planner/src/super_core/super_planner.cpp` | 4 |
| `super_planner/include/ros_interface/ros2/ros2_adapter.hpp` | 5 |
| `super_planner/include/ros_interface/ros1/ros1_adapter.hpp` | 5 |
| `super_planner/src/utils/polytope.cpp` | 1 |

**Final build result after all 4 fixes:**
```
Summary: 6 packages finished
  mars_quadrotor_msgs  ✓   marsim_render  ✓   mission_planner  ✓
  rog_map  ✓   perfect_drone_sim  ✓   super_planner  ✓
```

---

### 9. Hybrid simulation launch file

Created `/home/ideas-ad/super_ws/src/SUPER/mission_planner/launch/real_lidar_sim.launch.py`

**What it does:**
- Uses **real Livox LiDAR** (via FAST-LIO2) as the occupancy map
- Uses **perfect_drone_sim** (SUPER's built-in simulator) for virtual drone dynamics
- Connects them cleanly via topic remaps so they don't conflict

**Topic routing:**
| Topic | Source | Used by |
|-------|--------|---------|
| `/cloud_registered` | FAST-LIO2 (real) | SUPER planner map |
| `/lidar_slam/odom` | perfect_drone_sim | SUPER planner drone position |
| `/lidar_slam/odom_real` | FAST-LIO2 | Info only, not used by planner |
| `/sim/cloud_registered` | perfect_drone_sim | Suppressed (real map used instead) |
| `/planning/pos_cmd` | SUPER planner | Commands virtual drone |

**Static TF:** `world → lidar_init` (identity) links the sim frame to FAST-LIO2's frame so both appear in RViz together. *(Renamed from upstream `camera_init` on 2026-09-02 by `patches/fastlio_frames.py`.)*

---

### 10. MAVROS / ArduCopter FC Integration (infrastructure only — UART not wired yet)

**Hardware:** ARK FPV FC running ArduCopter latest (May 2026). SUPER planner output must eventually reach this FC via MAVROS.

**Full reference:** `/home/ideas-ad/jetson_px4_setup/setup_notes.md` and `plantofly.md`

**Data flow (target state):**
```
SUPER /planning/pos_cmd (PositionCommand @ 100 Hz)
          ↓
  [Bridge Node — NOT YET WRITTEN]
          ↓
  /mavros/setpoint_raw/local (PositionTarget)
  /mavros/vision_pose/pose   (PoseStamped — FAST-LIO2 odom → EKF3 ext nav)
          ↓
  MAVROS 2.14.0 (/dev/ttyTHS1 @ 921600 baud)
          ↓
  ArduCopter GUIDED mode → motors
```

**Two bugs pre-existing on this machine (must fix before first MAVROS launch):**

| Bug | Symptom | Fix |
|-----|---------|-----|
| GeographicLib geoid missing | MAVROS crashes silently at startup — `global_position` plugin throws `std::runtime_error` | `sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh` |
| `ideas-ad` not in `dialout` group | `/dev/ttyTHS1` permission denied — MAVROS prints "Opening link..." then stops silently | `sudo usermod -aG dialout ideas-ad` then re-login |

**Quick prerequisite fix:**
```bash
bash /home/ideas-ad/jetson_px4_setup/scripts/00_fix_prerequisites.sh
# Then log out / log in to apply dialout group
```

**Files created in `jetson_px4_setup/`:**

| File | Purpose |
|------|---------|
| `launch/px4_mavros_jetson.launch.py` | MAVROS launch — ttyTHS1 @ 921600, PX4 protocol v2 |
| `config/px4_jetson_params.yaml` | MAVROS params tuned for SUPER (200 Hz IMU, relative alt, timesync via MAVLink) |
| `config/px4_pluginlists_super.yaml` | Plugin whitelist — only what SUPER needs |
| `scripts/00_fix_prerequisites.sh` | Installs geoid dataset + adds dialout in one shot |
| `scripts/01_test_uart.sh` | Tests physical wire before MAVROS (counts MAVLink HEARTBEAT bytes) |
| `scripts/02_verify_mavros.sh` | Verifies MAVROS topics are live after launch |
| `scripts/03_mavros_autostart.sh` | Installs systemd service so MAVROS starts on boot |
| `netplan/01-pixhawk-ethernet.yaml` | Optional Ethernet path to FC (not used — UART chosen) |

**What is still missing:**
- UART wire: Pixhawk TELEM2 TX→RX/RX→TX/GND → Jetson 40-pin header (ttyTHS1)
- ArduCopter EKF3 external nav parameters (see `plantofly.md` Section 4)
- Bridge node that converts `/planning/pos_cmd` → `/mavros/setpoint_raw/local`

---

## Every Time — Full Hybrid Pipeline

### Step 1 — Check IP and Livox connection
```bash
ip addr show enP8p1s0        # must show inet 192.168.1.50/24
ping -c 3 192.168.1.154      # must show 0% packet loss
```

If IP missing: `sudo ip addr add 192.168.1.50/24 dev enP8p1s0` (run in desktop terminal)

### Step 2 — Start Livox driver (Terminal 1)
```bash
source /opt/ros/humble/setup.bash
source /home/ideas-ad/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
Wait for: `livox/lidar publish use livox custom format`

### Step 3 — Launch full hybrid stack (Terminal 2)
```bash
source /opt/ros/humble/setup.bash
source /home/ideas-ad/ws_livox/install/setup.bash
source /home/ideas-ad/super_ws/install/setup.bash
ros2 launch mission_planner real_lidar_sim.launch.py
```

RViz opens automatically. Wait ~30 seconds for the FSM to transition from `INIT` → `WAIT_GOAL` (map needs to build up).

### Step 4 — Send a goal
In RViz: click **"2D Goal Pose"** in the toolbar, then click a point in the point cloud ~3–5 m away.

The FSM will cycle: `WAIT_GOAL` → `GENERATE_TRAJ` → `FOLLOW_TRAJ` → `WAIT_GOAL`

Verify trajectory is executing:
```bash
source /opt/ros/humble/setup.bash
source /home/ideas-ad/super_ws/install/setup.bash
ros2 topic hz /planning/pos_cmd   # ~100 Hz while flying
```

### Step 5 — Reset and fly again
Kill all nodes and relaunch:
```bash
kill -9 $(pgrep -f "fastlio_mapping|perfect_drone_node|fsm_node|rviz2")
# Then repeat Steps 2–4
```

---

## Key Facts

| Item | Value |
|------|-------|
| Jetson Ethernet interface | `enP8p1s0` (not `eth0`) |
| Jetson IP (Livox) | `192.168.1.50/24` |
| Livox MID-360 IP | `192.168.1.154` |
| LiDAR ROS frame | `livox_frame` (raw) / `lidar_init` (FAST-LIO2 world, upstream `camera_init`); odom child is `lidar_imu` (upstream `body`) |
| Sim drone frame | `world` |
| TF link | `world → lidar_init` (identity, static) |
| xfer_format | `1` (CustomMsg — required for FAST-LIO2 lidar_type:1) |
| FAST-LIO2 odom topic (normal) | `/lidar_slam/odom` |
| FAST-LIO2 odom topic (hybrid) | `/lidar_slam/odom_real` |
| RViz config | `/home/ideas-ad/livox_view.rviz` |
| Hybrid launch file | `ros2 launch mission_planner real_lidar_sim.launch.py` |
| Flight controller | ARK FPV FC — ArduCopter latest (May 2026) |
| FC UART port | `/dev/ttyTHS1` @ 921600 baud (ttyTHS1 = Jetson 40-pin header) |
| FC MAVROS connection | **Not yet wired** — wire TELEM2 TX→RX / RX→TX / GND |
| MAVROS launch | `ros2 launch jetson_px4_setup px4_mavros_jetson.launch.py` |
| Bridge node | **Not yet written** — converts `/planning/pos_cmd` → MAVROS setpoints |

---

## All Issues Encountered & Fixes

| # | Issue | Root Cause | Fix |
|---|-------|-----------|-----|
| 1 | `cannot find device "eth0"` | Jetson uses predictable interface names | Use `enP8p1s0` |
| 2 | `ping` fails after IP assigned | `sudo ip addr add` wasn't run (needs desktop terminal) | Run in Jetson desktop terminal, not via Claude |
| 3 | `librclcpp.so: cannot open shared object file` | ws_livox build conflict (leftover directory blocked symlink) | `rm -rf` the conflicting dir, rebuild |
| 4 | FAST-LIO2 first build: CMake cache mismatch | Previous build used a different source path | `rm -rf build/fast_lio install/fast_lio`, rebuild |
| 5 | FAST-LIO2 second build: `ikd_Tree.cpp` not found | `--depth 1` clone doesn't pull submodules | `git submodule update --init --recursive` |
| 6 | FAST-LIO2 receives no LiDAR data | `xfer_format=0` publishes PointCloud2 but `lidar_type=1` subscribes to CustomMsg — type mismatch | Change `xfer_format` back to `1` |
| 7 | Point cloud sparse in hybrid sim | `dense_publish_en: false`, `point_filter_num: 3`, `filter_size_surf: 0.5` | Enable dense publish, set filter_num=1, filter_size=0.2 |
| 8 | `Odom below virtual ground` warning, map rejected | Drone at z≈0, virtual_ground_height effective value was +0.25 m | Set `virtual_ground_height: -1.0` in static_dense.yaml |
| 9 | FAST-LIVO2 rejected (user request) | ROS1 only (catkin), requires camera hardware | Switched to FAST_LIO ROS2 branch |
| 10 | Duplicate node instances after restarts | `pkill` exit code 144 (some processes not found) left old nodes running | Use `kill -9 $(pgrep -f ...)` for reliable cleanup |
| 11 | FSM stuck in INIT for ~30s | Normal — ROG-Map needs time to build up enough voxels | Wait for `WAIT_GOAL` before sending goal |
| 12 | Goal sent during INIT was ignored | FSM discards goals received before map is ready | Resend goal after FSM reaches `WAIT_GOAL` |
| 13 | `colcon build` fails: Duplicate package names | SUPER tree copied to `super_ws/SUPER/` (workspace root) in addition to `super_ws/src/SUPER/` | `touch super_ws/SUPER/COLCON_IGNORE` |
| 14 | `colcon build` fails: catkin not found | ROS1 `FAST_LIO/` at workspace root (`super_ws/FAST_LIO/`) — colcon does not provide catkin | `touch super_ws/FAST_LIO/COLCON_IGNORE` |
| 15 | `perfect_drone_sim` fails: `pcl_conversions.h` not found | `CMakeLists.txt` missing `find_package(pcl_conversions REQUIRED)` | Added to CMakeLists.txt line 34 + ament_target_dependencies |
| 16 | `super_planner` fails: `isnan` undeclared in C++17 | C++17 removes global `isnan`; Eigen adds competing overloads, making bare `isnan()` ambiguous | Added `#include <cmath>`, replaced all `isnan(` with `std::isnan(` in 5 files |
| 17 | MAVROS crashes silently at startup | GeographicLib egm96-5 geoid dataset not installed — `global_position` plugin throws `std::runtime_error` | `sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh` |
| 18 | MAVROS: serial port permission denied | User `ideas-ad` not in `dialout` group — `/dev/ttyTHS1` mode 660 blocks open() | `sudo usermod -aG dialout ideas-ad` then re-login |
| 19 | SUPER commands never reach ArduCopter | Bridge node between `/planning/pos_cmd` and `/mavros/setpoint_raw/local` not yet written | Must write bridge node (see `plantofly.md`) |

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| No `inet` on `enP8p1s0` | `sudo ip addr add 192.168.1.50/24 dev enP8p1s0` in desktop terminal |
| Ping to 192.168.1.154 fails | Check Livox is powered on and Ethernet cable is connected |
| `livox/lidar` not publishing | Restart Livox driver; check xfer_format=1 |
| FAST-LIO2 receives no data | Verify `xfer_format=1` and `lidar_type=1` in mid360.yaml |
| Point cloud is sparse | Check `dense_publish_en: true`, `point_filter_num: 1` in mid360.yaml |
| FSM stays in INIT forever | Check `/lidar_slam/odom` and `/cloud_registered` are both publishing |
| FSM in WAIT_GOAL, goal ignored | Goal sent too early (during INIT) — resend after WAIT_GOAL |
| Duplicate nodes running | `kill -9 $(pgrep -f "fastlio_mapping\|perfect_drone_node\|fsm_node\|rviz2")` |
| RViz shows no point cloud | Press `F` to reset camera; check Fixed Frame is `world` |
| MAVROS crashes on startup (silent) | GeographicLib geoid missing — run `bash jetson_px4_setup/scripts/00_fix_prerequisites.sh` |
| MAVROS: no FCU heartbeat, "Opening link..." then stops | User not in `dialout` — run `sudo usermod -aG dialout ideas-ad` then re-login |
| `/dev/ttyTHS1` not found | JetPack pinmux not set — run `sudo /opt/nvidia/jetson-io/jetson-io.py` to enable UART |
| MAVROS starts but no IMU data | PX4 TELEM2 baud rate mismatch — set `SER_TEL2_BAUD=921600` in QGroundControl |
| SUPER plans but drone doesn't move | Bridge node not written yet — see `plantofly.md` |
