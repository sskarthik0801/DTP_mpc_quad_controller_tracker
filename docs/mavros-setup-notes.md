> ⛔ **SUPERSEDED — HISTORICAL ONLY (flagged 2026-07-29).**
> This document plans M2/M3 around **ArduCopter + MAVROS**, which was abandoned.
>
> The flight controller runs **PX4 v1.17.0 + uXRCE-DDS**, confirmed live over MAVLink
> (`MAV_AUTOPILOT_PX4`, firmware `v1.17.0`). The ArduCopter/MAVROS parameters used below
> (`SERIALx_PROTOCOL`, `SERIALx_BAUD`, `EK3_SRC1_*`, `ARMING_CHECK`) **do not exist on this
> aircraft**. MAVROS is not used; the bridge is `px4_odom_bridge` →
> `/fmu/in/vehicle_visual_odometry` over uXRCE-DDS.
>
> **Current source of truth:** [`ImplementationDoc.md`](ImplementationDoc.md),
> [`uart_bringup_log.md`](uart_bringup_log.md), [`../CLAUDE.md`](../CLAUDE.md).
> Kept only for provenance — do not follow these steps.

# Jetson Orin Nano + PX4 Setup Notes

Same format as the build-fix log and Jackson project notes:
every entry gives the exact file path, line, what it does, and why it matters.

---

## System State (confirmed on this machine)

| Item | Status |
|------|--------|
| Hardware | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit |
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| Kernel | 5.15.185-tegra (aarch64) |
| ROS 2 | Humble |
| MAVROS | 2.14.0 (already installed) |
| Serial ports | `/dev/ttyTHS1`, `/dev/ttyTHS2` (confirmed present) |
| GeographicLib geoid | **NOT INSTALLED** — fix required before MAVROS will start |
| `ideas-ad` in dialout | **NO** — fix required before serial port access works |

---

## Two Real Bugs Found on This Machine

### Bug 1 — GeographicLib geoid dataset missing

**What breaks:** MAVROS `global_position` plugin converts GPS (WGS84) to local ENU coordinates using the egm96-5 geoid model. If the dataset file is absent the plugin throws `std::runtime_error` at startup and the entire MAVROS node exits. There is no clear error message — it just looks like MAVROS silently failed.

**Why it happens:** The ROS package `ros-humble-mavros` does not include the dataset because it is too large for a deb package. It must be downloaded separately.

**Fix:** Run as root:
```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```
Or run the prerequisite fix script in this repo:
```bash
bash jetson_px4_setup/scripts/00_fix_prerequisites.sh
```

**Verify:** `ls /usr/share/GeographicLib/geoids/egm96-5.pgm` should return the file.

---

### Bug 2 — User `ideas-ad` not in `dialout` group

**What breaks:** `/dev/ttyTHS1` is owned `root:dialout` with permissions `660`. MAVROS calls `open("/dev/ttyTHS1", O_RDWR)` as the normal user and gets `EACCES`. The node starts, prints "Opening link...", then silently stops with no FCU heartbeat. The connection never establishes.

**Fix:**
```bash
sudo usermod -aG dialout ideas-ad
# Log out and back in, or in the current shell:
newgrp dialout
```

**Verify:** `groups ideas-ad | grep dialout` should show `dialout`.

---

## Architecture: Three Methods to Bridge PX4 ↔ ROS2

| Method | What it does | Best for |
|--------|-------------|----------|
| **MAVROS** | Translates MAVLink ↔ ROS2 topics/services. Full bidirectional control. | Complex planners like SUPER that publish setpoints and read state continuously |
| **MAVSDK** | C++/Python SDK that wraps MAVLink. High-level actions (arm, takeoff, mission). | Simple scripts, one-shot commands, Python prototyping |
| **PX4 DDS / micro-XRCE** | Native PX4 uORB topics exposed directly as ROS2 topics via DDS middleware. | Highest performance, but requires PX4 firmware recompile with DDS enabled |

**Recommendation for SUPER planner: MAVROS.** SUPER publishes position/velocity setpoints and subscribes to IMU + odometry continuously. MAVROS exposes exactly these as standard ROS2 topics out of the box.

---

## File Reference

---

### `scripts/00_fix_prerequisites.sh`

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/scripts/00_fix_prerequisites.sh` | 24–35 | Checks for `egm96-5.pgm` and installs it if absent. This is the geoid dataset fix. Must be run before the first MAVROS launch or the global_position plugin crashes on startup. |
| `jetson_px4_setup/scripts/00_fix_prerequisites.sh` | 38–47 | Checks dialout group membership and adds the user if missing. `usermod -aG dialout` appends dialout to the user's supplementary groups. The change is not active until the next login — that is why the script also prints the `newgrp` workaround for the current shell. |
| `jetson_px4_setup/scripts/00_fix_prerequisites.sh` | 50–57 | Verification block — re-checks all three conditions and prints a clear PRESENT/MISSING/FOUND/NOT FOUND status. Run this again after fixing to confirm. |

---

### `scripts/01_test_uart.sh`

Test the physical UART wire before spending time debugging MAVROS config.

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/scripts/01_test_uart.sh` | 27–31 | Checks `/dev/ttyTHS1` exists. If absent the JetPack pinmux has not assigned THS1 to the UART function — fix with Jetson-IO tool: `sudo /opt/nvidia/jetson-io/jetson-io.py`. |
| `jetson_px4_setup/scripts/01_test_uart.sh` | 34–38 | Read-permission check. Catches the dialout group problem at the shell level before MAVROS tries to open the port. |
| `jetson_px4_setup/scripts/01_test_uart.sh` | 41–45 | Sets baud rate with `stty`. The Jetson kernel does not persist baud rate across reboots for THS ports, so MAVROS (and this test) must set it explicitly every time. |
| `jetson_px4_setup/scripts/01_test_uart.sh` | 48–57 | Runs `cat /dev/ttyTHS1` for 3 seconds and counts bytes. MAVLink sends HEARTBEAT messages at 1 Hz regardless of whether anything is listening. If any bytes arrive, the physical wire is correct. If zero bytes: check baud rate, TX/RX swap, and whether the flight controller is powered. |
| `jetson_px4_setup/scripts/01_test_uart.sh` | 59–61 | Checks if MAVROS already holds the port with `lsof`. Two processes cannot open the same serial device — if MAVROS is running, this test will get no data even if the wire is fine. |

---

### `scripts/02_verify_mavros.sh`

Run after `ros2 launch ... px4_mavros_jetson.launch.py` to confirm the full stack.

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/scripts/02_verify_mavros.sh` | 31–36 | Checks `ros2 node list` for `/mavros`. If absent MAVROS crashed at startup — most likely the geoid dataset bug (Bug 1 above). |
| `jetson_px4_setup/scripts/02_verify_mavros.sh` | 38–44 | Reads `/mavros/state` topic. `connected: true` means a MAVLink HEARTBEAT was received from PX4. This is the single most important confirmation: FCU is alive and talking. |
| `jetson_px4_setup/scripts/02_verify_mavros.sh` | 46–52 | Measures IMU topic rate with `ros2 topic hz`. FAST-LIO2 needs IMU at 200 Hz. If rate is ~0 the `imu` plugin is in the denylist or PX4 is not streaming `IMU_DATA` messages. |
| `jetson_px4_setup/scripts/02_verify_mavros.sh` | 54–58 | Single-shot read of `/mavros/local_position/pose`. Required by SUPER planner for state estimation feedback. |
| `jetson_px4_setup/scripts/02_verify_mavros.sh` | 61–64 | Re-checks geoid dataset even after MAVROS is running. The plugin silently degraded if the dataset was missing, so this catches that case. |
| `jetson_px4_setup/scripts/02_verify_mavros.sh` | 66–73 | Confirms all five topics that SUPER planner needs are advertised. A missing topic here means the corresponding plugin is in the denylist in `px4_pluginlists_super.yaml`. |

---

### `scripts/03_mavros_autostart.sh`

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/scripts/03_mavros_autostart.sh` | 18–19 | `After=dev-ttyTHS1.device` + `Requires=dev-ttyTHS1.device` — systemd will not start the MAVROS service until the kernel has enumerated `/dev/ttyTHS1`. Without this the service starts, fails to open the port, and exits before the hardware is ready. |
| `jetson_px4_setup/scripts/03_mavros_autostart.sh` | 25–26 | `Restart=on-failure` + `RestartSec=5s` — if the flight controller reboots mid-flight (power glitch), the UART disappears and MAVROS exits. Systemd restarts it 5 seconds later without any manual intervention. |

---

### `launch/px4_mavros_jetson.launch.py`

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/launch/px4_mavros_jetson.launch.py` | 26–29 | `fcu_url="/dev/ttyTHS1:921600"` — `ttyTHS1` is the hardware UART on the Jetson Orin Nano 40-pin header, connected to Pixhawk TELEM2. 921600 baud provides ~90 KB/s — enough for IMU at 200 Hz plus position and setpoint traffic. Drop to 57600 only if you see framing errors. |
| `jetson_px4_setup/launch/px4_mavros_jetson.launch.py` | 33–35 | `gcs_url=""` — empty means no forwarding to QGroundControl. Set to `udp://:14550@0.0.0.0:14550` when you need simultaneous QGC connection (e.g. during bench testing). Leave empty in flight to reduce CPU load. |
| `jetson_px4_setup/launch/px4_mavros_jetson.launch.py` | 49–53 | Loads three parameter sources: `px4_jetson_params.yaml` (SUPER-tuned params), `px4_pluginlists_super.yaml` (bandwidth-optimised plugin list), and inline overrides. Inline overrides win over YAML if there is a conflict. |

---

### `config/px4_jetson_params.yaml`

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/config/px4_jetson_params.yaml` | 16 | `min_voltage: [14.8]` — 3.7 V × 4 cells = 14.8 V low-battery threshold for a 4S pack. The default (10.0 V) would not trigger an alarm until the battery is dangerously depleted for a 4S. |
| `jetson_px4_setup/config/px4_jetson_params.yaml` | 25–26 | `timesync_mode: MAVLINK` — synchronises the Jetson clock to PX4's internal clock over UART using MAVLink TIMESYNC messages. This keeps sensor timestamps aligned without needing GPS lock or NTP. FAST-LIO2 will diverge if IMU and LiDAR timestamps differ by more than ~10 ms. |
| `jetson_px4_setup/config/px4_jetson_params.yaml` | 40 | `use_relative_alt: true` — for GPS-denied forest flights the absolute altitude from GPS is unavailable. Relative altitude from the barometer is used instead. SUPER's `rog_map` works in a local ENU frame and does not need absolute altitude. |
| `jetson_px4_setup/config/px4_jetson_params.yaml` | 47–48 | `tf.send: true` on local_position — MAVROS publishes the `map → base_link` TF transform. SUPER's corridor generator and A* search use this transform to convert between world-frame goals and body-frame commands. |

---

### `config/px4_pluginlists_super.yaml`

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/config/px4_pluginlists_super.yaml` | 15 | `image_pub` denied — image data is published directly by the camera ROS2 driver, not through MAVLink. Enabling this would add zero value and waste UART bandwidth. |
| `jetson_px4_setup/config/px4_pluginlists_super.yaml` | 16 | `vision_speed_estimate` denied — SUPER sends position estimates (`vision_pose_estimate`) not velocity estimates. Enabling both would create conflicting EKF inputs in PX4. |
| `jetson_px4_setup/config/px4_pluginlists_super.yaml` | 18–20 | `distance_sensor`, `rangefinder`, `wheel_odometry` denied — these sensors are handled by their own dedicated ROS2 drivers (Livox driver, etc.), not by MAVLink. |
| `jetson_px4_setup/config/px4_pluginlists_super.yaml` | 24 | `hil` denied — hardware-in-the-loop simulation plugin. Leaving it enabled on a real vehicle is a safety hazard: HiL mode overrides actuator outputs with simulated values. |

---

### `netplan/01-pixhawk-ethernet.yaml`

Used only when the Holybro Pixhawk-Jetson Baseboard is connected via Ethernet cable
(higher bandwidth alternative to UART).

| File | Line | What it does and why it matters |
|------|------|---------------------------------|
| `jetson_px4_setup/netplan/01-pixhawk-ethernet.yaml` | 16 | `addresses: [192.168.0.1/24]` — static IP on the Pixhawk-facing interface. Static is required because there is no DHCP server on this point-to-point link. PX4 defaults to `192.168.0.2`; Jetson takes `192.168.0.1`. |
| `jetson_px4_setup/netplan/01-pixhawk-ethernet.yaml` | 18 | `dhcp4: false` — if left as `true`, NetworkManager would send DHCP broadcasts that never get a response, adding 30–90 s to boot time while waiting for the lease timeout. |
| `jetson_px4_setup/netplan/01-pixhawk-ethernet.yaml` | 20 | `routes: []` — no default gateway on this interface. The Jetson's normal internet traffic still goes through WiFi or another Ethernet port. Without this, Netplan might route all traffic through the Pixhawk link, breaking internet connectivity. |

---

## VSLAM vs. LiDAR+YOLO+BoT-SORT (GPS-Denied Navigation Comparison)

The Hackster.io guide uses **Isaac ROS VSLAM** on Jetson Orin Nano with an Intel D435i depth camera. The SUPER planner stack uses **FAST-LIO2** (LiDAR odometry) + **YOLO + BoT-SORT** (object tracking).

| Aspect | VSLAM (D435i) | LiDAR + YOLO + BoT-SORT |
|--------|---------------|------------------------|
| **Sensor** | Intel D435i RGB-D camera | Livox Mid-360 or similar LiDAR |
| **Range** | 0.3–10 m (indoor-biased) | 0.1–40 m (outdoor capable) |
| **Dense forest** | Fails — foliage is texture-poor and reflective | Better — LiDAR returns structure even without visual texture |
| **Low light** | Fails without IR fill | Works — LiDAR is active sensing, unaffected by lighting |
| **Object tracking** | Not native — needs separate model | BoT-SORT tracks dynamic obstacles (people, vehicles) |
| **Compute** | GPU-accelerated via Isaac ROS | CPU-heavy (FAST-LIO2) + GPU (YOLO) |
| **Map output** | Dense point cloud (limited range) | rog_map occupancy grid, suitable for path planning |
| **Conclusion** | Good for indoor structured environments | Better for outdoor, GPS-denied, dynamic-obstacle scenarios |

**For your dense-forest use case: the LiDAR + YOLO + BoT-SORT stack is the right choice.**
VSLAM with a depth camera would lose tracking among tree trunks (poor visual features,
variable lighting, depth noise from leaves). FAST-LIO2 on a spinning or solid-state LiDAR
handles exactly this environment.

---

## Quick-Start Order

```bash
# Step 1 — Fix prerequisites (once, after fresh install)
bash jetson_px4_setup/scripts/00_fix_prerequisites.sh
# Then log out and back in

# Step 2 — Test the UART wire before touching MAVROS
bash jetson_px4_setup/scripts/01_test_uart.sh /dev/ttyTHS1 921600

# Step 3 — Launch MAVROS
source /opt/ros/humble/setup.bash
ros2 launch jetson_px4_setup/launch/px4_mavros_jetson.launch.py

# Step 4 — Verify everything is working (in a second terminal)
bash jetson_px4_setup/scripts/02_verify_mavros.sh

# Step 5 — (Optional) Set up autostart on boot
bash jetson_px4_setup/scripts/03_mavros_autostart.sh
```

## PX4 Parameters to Set in QGroundControl

Before running MAVROS, configure PX4 via QGroundControl (USB connection):

| Parameter | Value | Why |
|-----------|-------|-----|
| `MAV_1_CONFIG` | `TELEM2` | Routes MAVLink stream 1 to TELEM2 port |
| `SER_TEL2_BAUD` | `921600` | Matches `fcu_url` baud in launch file |
| `MAV_1_MODE` | `Onboard` | Sets message rates appropriate for a companion computer |
| `EKF2_AID_MASK` | `24` (vision pos + yaw) | Tells EKF2 to use external vision instead of GPS |
| `EKF2_HGT_MODE` | `3` (vision) | Use vision altitude estimate in GPS-denied flight |
