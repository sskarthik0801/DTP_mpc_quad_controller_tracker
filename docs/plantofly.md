> ⛔ **SUPERSEDED — HISTORICAL ONLY (flagged 2026-07-29).**
> This document plans ArduCopter EKF3 external-nav and a MAVROS bridge, which was abandoned.
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

# Plan to Fly
### SUPER UAV — Autonomous GPS-Denied Navigation Readiness Document
**Hardware confirmed:** Jetson Orin Nano · ARK FPV FC (ArduCopter latest, May 2026) · Livox MID-360

---

## Confirmed Answers

| Question | Answer | Impact |
|----------|--------|--------|
| Flight controller | ARK FPV, ArduCopter latest (May 2026) | Parameters documented below |
| UART wired? | ❌ Not yet | Section 3 covers this fully |
| EKF3 ext. nav? | ❌ Not configured | Section 4 covers every parameter |
| Livox MID-360 mounted? | ✅ Yes | Ready for driver launch |
| External IMU? | ❌ None — using MID-360 built-in IMU | FAST-LIO2 config confirmed for this |

---

## The Complete Picture — What This Drone Needs to Do

```
You give it: a destination (x, y, z)

It must:
  1. Know where it is            ← FAST-LIO2  (Livox MID-360 + built-in IMU)
  2. Build a map of obstacles    ← ROG-Map    (from Livox point cloud)
  3. Plan a safe path            ← SUPER Planner (A* + CIRI corridors + MINCO traj)
  4. Send commands to the FC     ← Bridge Node (MISSING — must be written)
  5. Execute the flight          ← ArduCopter GUIDED mode (via MAVROS)
  6. Not crash                   ← Backup trajectory + collision check at 15 Hz
```

---

## Full Data-Flow Diagram

```
╔══════════════════════════════════════════════════════════════════════════╗
║                         JETSON ORIN NANO                                 ║
║                                                                          ║
║  ┌─────────────────┐                                                     ║
║  │  Livox MID-360  │──── Ethernet (192.168.1.50 ↔ 192.168.1.154) ────┐  ║
║  │  (on drone)     │                                                  │  ║
║  └─────────────────┘                                                  │  ║
║                                                                       ▼ ║
║                                              ┌────────────────────────┐  ║
║                                              │  livox_ros_driver2     │  ║
║                                              │                        │  ║
║                                              │  Publishes:            │  ║
║                                              │  /livox/lidar  10 Hz   │  ║
║                                              │  /livox/imu   200 Hz   │  ║
║                                              └──────────┬─────────────┘  ║
║                                                         │                 ║
║                                                        ▼                 ║
║                                              ┌────────────────────────┐  ║
║                                              │  FAST-LIO2  (ROS 2)    │  ║
║                                              │  IMU: MID-360 built-in │  ║
║                                              │                        │  ║
║                                              │  Subscribes:           │  ║
║                                              │    /livox/lidar        │  ║
║                                              │    /livox/imu          │  ║
║                                              │                        │  ║
║                                              │  Publishes:            │  ║
║                                              │  /Odometry ──── ⚠ remap│  ║
║                                              │  /cloud_registered     │  ║
║                                              └──────────┬─────────────┘  ║
║                                          topic remap ▼  │cloud           ║
║                        /lidar_slam/odom ◀─────────────n┤                ║
║                                          ┌──────────────┘                ║
║                                          ▼                                ║
║  ┌───────────────────────────────────────────────────────────────────┐   ║
║  │                   SUPER PLANNER  (ROS 2 Humble)                   │   ║
║  │                                                                   │   ║
║  │   ROG-Map (/cloud_registered + /lidar_slam/odom)                  │   ║
║  │      ↓  3D voxel occupancy grid (0.05m resolution)                │   ║
║  │   A* path search                                                  │   ║
║  │      ↓  waypoints through free space                              │   ║
║  │   CIRI corridor generation                                        │   ║
║  │      ↓  safe convex polytopes around path                         │   ║
║  │   MINCO trajectory optimisation                                   │   ║
║  │      ↓  pos + yaw smooth polynomial trajectory                    │   ║
║  │   Backup trajectory (pre-computed emergency escape)               │   ║
║  │                                                                   │   ║
║  │   Publishes /planning/pos_cmd @ 100 Hz ────────────────────────┐  │   ║
║  └───────────────────────────────────────────────────────────────┘ │  │   ║
║                                                                     │      ║
║  ┌──────────────────────────────────────────────────────────────┐  │      ║
║  │   ⚠  BRIDGE NODE  (does not exist yet — must be written)     │◀─┘      ║
║  │                                                              │          ║
║  │   Input:  /planning/pos_cmd  (PositionCommand)               │          ║
║  │           /lidar_slam/odom   (Odometry)                      │          ║
║  │                                                              │          ║
║  │   Output: /mavros/setpoint_raw/local  (PositionTarget)       │          ║
║  │           /mavros/vision_pose/pose    (PoseStamped)          │          ║
║  └─────────────────────────┬────────────────────────────────────┘          ║
║                            │                                               ║
║  ┌─────────────────────────▼────────────────────────────────────┐          ║
║  │   MAVROS 2.14.0  (ROS 2)                                     │          ║
║  │   /dev/ttyTHS1  @  921600 baud  (40-pin header)              │          ║
║  │   Translates ROS2 topics ↔ MAVLink protocol                  │          ║
║  └─────────────────────────┬────────────────────────────────────┘          ║
║                            │                                               ║
╚════════════════════════════│═══════════════════════════════════════════════╝
                             │ UART Serial (3.3V logic, 921600 baud)
                             ▼
            ╔══════════════════════════════════════════╗
            ║    ARK FPV FLIGHT CONTROLLER             ║
            ║    ArduCopter (GUIDED mode)              ║
            ║                                          ║
            ║    Receives via MAVLink:                 ║
            ║    SET_POSITION_TARGET_LOCAL_NED         ║
            ║      (position + velocity setpoint)      ║
            ║    VISION_POSITION_ESTIMATE              ║
            ║      (odometry → EKF3 localisation)      ║
            ║                                          ║
            ║    EKF3 fuses:                           ║
            ║      - External vision (FAST-LIO2 odom)  ║
            ║      - Barometer (altitude backup)       ║
            ║      - Built-in IMU (IMC-42688-P)        ║
            ║                                          ║
            ║    PID loops → ESC PWM → Motors          ║
            ╚══════════════════════════════════════════╝
```

---

## Section 1 — What Is Already Working

| Component | Status | Evidence |
|-----------|--------|----------|
| Jetson Orin Nano (Ubuntu 22.04, ROS2 Humble) | ✅ Running | Confirmed in session |
| MAVROS 2.14.0 | ✅ Installed | `dpkg -l ros-humble-mavros` |
| `/dev/ttyTHS1` port | ✅ Present | `ls /dev/ttyTHS*` |
| SUPER planner (all 6 packages) | ✅ Builds | `colcon build` passed |
| Livox ROS Driver 2 (source) | ✅ Present | `ws_livox/src/livox_ros_driver2/` |
| Livox MID-360 mounted on drone | ✅ Confirmed | Your answer Q4 |
| ARK FPV FC (basic calibration in QGC) | ✅ Calibrated | Your answer Q3 |

---

## Section 2 — What Is Missing (In Priority Order)

```
PRIORITY 1 — Without these, nothing flies
─────────────────────────────────────────
  ❌  UART not wired (Jetson ↔ ARK FPV)          → Section 3
  ❌  Bridge node does not exist                 → Section 7
  ❌  FAST-LIO2 is ROS1 catkin in workspace      → Section 5
  ❌  EKF3 external nav not configured           → Section 4

PRIORITY 2 — Without these, it flies but poorly
────────────────────────────────────────────────
  ❌  GeographicLib geoid dataset missing         → run Section 6 fix
  ❌  ideas-ad not in dialout group               → run Section 6 fix
  ❌  Livox Ethernet IP not statically set        → Section 6
  ❌  Topic remap /Odometry → /lidar_slam/odom    → Section 5

PRIORITY 3 — Tuning
────────────────────
  ⚠   robot_r not set to actual drone size        → Section 8
  ⚠   max_vel at 8 m/s for first flight is risky → start at 2 m/s
  ⚠   MID-360 extrinsic mounting angle not set    → Section 5
```

---

## Section 3 — UART Wiring: ARK FPV ↔ Jetson Orin Nano

### 3.1 Connector on ARK FPV — JST-GH 6-pin (TELEM2)

The ARK FPV uses the Pixhawk Dronecode standard for TELEM connectors.
Use **TELEM2** (leaves TELEM1 free for a radio link to QGC on the bench).

```
ARK FPV — TELEM2 port (JST-GH 6-pin, looking INTO connector)
┌─────────────────────────────────────────────────────────┐
│  Pin 1   Pin 2   Pin 3   Pin 4   Pin 5   Pin 6          │
│  +5V     TX      RX      CTS     RTS     GND            │
│  (out)  (out)   (in)   (opt)   (opt)   (ref)            │
└─────────────────────────────────────────────────────────┘
           │        │                       │
           │        │                       │
```

### 3.2 Connector on Jetson Orin Nano — 40-pin Header

`/dev/ttyTHS1` maps to the hardware UART on the Jetson Orin Nano 40-pin expansion header.

```
Jetson Orin Nano 40-pin Header (top view, pin 1 top-left)

 3V3 [ 1] [ 2] 5V
     [ 3] [ 4] 5V
     [ 5] [ 6] GND  ◀────── connect to ARK FPV Pin 6 (GND)
     [ 7] [ 8] UART1_TX  ◀── connect to ARK FPV Pin 3 (RX)
 GND [ 9] [10] UART1_RX  ◀── connect to ARK FPV Pin 2 (TX)
     [11] [12]
     ...  ...
```

### 3.3 Wiring Diagram (3 wires only)

```
ARK FPV TELEM2                          Jetson Orin Nano
JST-GH 6-pin                            40-pin header

Pin 1  +5V    ──── DO NOT CONNECT ────  (would damage Jetson 3.3V GPIO)
               ╳
Pin 2  FC TX  ──────────────────────── Pin 10  UART1_RX  (/dev/ttyTHS1)
               ──────────────────────
Pin 3  FC RX  ──────────────────────── Pin 8   UART1_TX  (/dev/ttyTHS1)
               ──────────────────────
Pin 4  CTS    ──── NOT CONNECTED ────  (hardware flow control not needed)
Pin 5  RTS    ──── NOT CONNECTED ────

Pin 6  GND    ──────────────────────── Pin 6   GND
               ──────────────────────

TOTAL: 3 wires — TX, RX, GND
Logic level: 3.3V on BOTH sides — no level shifter needed ✅
```

> **⚠ CRITICAL:** Never connect the FC 5V (Pin 1) to any Jetson pin.
> The Jetson must be powered from its own supply, not from the flight controller.

### 3.4 Verify the Wire After Connecting

```bash
# On Jetson — before launching MAVROS:
bash /home/ideas-ad/jetson_px4_setup/scripts/01_test_uart.sh /dev/ttyTHS1 921600

# Expected output if wired correctly and FC is powered:
#   [OK] Received N bytes — MAVLink traffic detected.
#
# If 0 bytes: TX/RX are swapped — swap Pin 8 and Pin 10 connections.
```

---

## Section 4 — ArduPilot Parameters to Set

Connect ARK FPV to QGroundControl via USB. Set these parameters in the Parameter Editor.
**Reboot the FC after setting all parameters.**

### 4.1 UART / MAVLink for TELEM2

```
SERIAL2_PROTOCOL    = 2         MAVLink2 (not 1 — must be v2 for MAVROS)
SERIAL2_BAUD        = 921       921600 baud (matches MAVROS fcu_url)

Stream rates on TELEM2 (how often ArduPilot sends data to Jetson):
SR2_POSITION        = 10        Position @ 10 Hz
SR2_EXTRA1          = 10        Attitude @ 10 Hz
SR2_EXTRA2          = 10        VFR HUD  @ 10 Hz
SR2_EXTRA3          = 2
SR2_RAW_SENS        = 10        IMU raw  @ 10 Hz
SR2_RC_CHANNELS     = 5
```

### 4.2 GPS — Disable (We Are GPS-Denied)

```
GPS_TYPE            = 0         No GPS — external nav only
GPS_TYPE2           = 0
```

### 4.3 EKF3 — External Vision Odometry (FAST-LIO2 as position source)

This is the most important section. It tells ArduCopter's EKF3 to use
FAST-LIO2 odometry (sent via MAVROS vision_pose) as its position source
instead of GPS.

```
EK3_ENABLE          = 1         Enable EKF3 (should already be 1)
EK3_IMU_MASK        = 3         Use both built-in IMUs (redundancy)

# Primary source = ExternalNav (value 6)
EK3_SRC1_POSXY      = 6         XY position from external vision
EK3_SRC1_POSZ       = 1         Z from barometer (safer for first flights)
                                 Change to 6 (ExternalNav) once confident
EK3_SRC1_VELXY      = 6         XY velocity from external vision
EK3_SRC1_VELZ       = 6         Z velocity from external vision
EK3_SRC1_YAW        = 6         Yaw from external vision
                                 Change to 1 (compass) if yaw drifts

# External nav settings
VISO_TYPE           = 1         Enable visual/LiDAR odometry input
VISO_DELAY_MS       = 50        Latency compensation (tune if needed)
VISO_VEL_M_NSE      = 0.05      Velocity measurement noise (m/s)
VISO_POS_M_NSE      = 0.1       Position measurement noise (m)
VISO_YAW_M_NSE      = 0.1       Yaw measurement noise (rad)
```

### 4.4 GUIDED Mode and Arming

```
ARMING_CHECK        = 388892     Disable GPS-required check only
                                 (or set to 0 for bench testing — NOT for flight)

FENCE_ENABLE        = 0          Disable geofence (no GPS = no fence possible)
GUID_OPTIONS        = 0          Standard GUIDED mode
GUID_TIMEOUT        = 3          Seconds before GUIDED stops if setpoints stop
                                 (safety: drone hovers if Jetson crashes)

# Failsafe
FS_EKF_ACTION       = 2          Land if EKF fails (not RTL — no GPS for RTL)
FS_EKF_THRESH       = 0.8        EKF variance threshold for failsafe trigger
FS_THR_VALUE        = 975        Throttle failsafe PWM value
```

### 4.5 Arming and First Takeoff Sequence

```
# Via MAVROS from Jetson:
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{custom_mode: 'GUIDED'}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"

# Then the bridge node starts publishing setpoints
# ArduPilot will fly to them
```

---

## Section 5 — FAST-LIO2: Install ROS2 Version + Configure for MID-360

### 5.1 Problem: Current FAST-LIO is ROS1

The `super_ws/FAST_LIO/` package uses `catkin` (ROS1). It cannot be used
with SUPER (ROS2). A `COLCON_IGNORE` file was already placed there to stop
the build from failing. Now we need the ROS2 version.

### 5.2 Install FAST-LIO2 ROS2 Branch

```bash
cd /home/ideas-ad/super_ws/src

# Clone the ROS2-compatible branch
git clone https://github.com/hku-mars/FAST_LIO.git FAST_LIO_ROS2 --depth 1

# FAST-LIO2 needs livox_ros_driver2 — link the already-built ws_livox install
source /home/ideas-ad/ws_livox/install/setup.bash

# Build everything together
cd /home/ideas-ad/super_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fast_lio
```

### 5.3 FAST-LIO2 Configuration for MID-360 + Built-In IMU

The MID-360 has an integrated IMU. FAST-LIO2 uses it for tightly-coupled
LiDAR-IMU odometry — no external IMU board is needed.

The factory extrinsic calibration (LiDAR frame → IMU frame) is already
in `mid360.yaml`. Do not change these unless you have re-calibrated:

```yaml
# super_ws/FAST_LIO/config/mid360.yaml — what each setting means:

common:
    lid_topic:  "/livox/lidar"        # Livox driver publishes here
    imu_topic:  "/livox/imu"          # MID-360 built-in IMU

preprocess:
    lidar_type: 1                     # 1 = Livox series (correct for MID-360)
    scan_line: 4                      # MID-360 has 4 scan lines
    blind: 0.5                        # Ignore returns within 0.5m (close reflections)

mapping:
    acc_cov: 0.1                      # Accelerometer noise — MID-360 built-in IMU
    gyr_cov: 0.1                      # Gyroscope noise — adequate for FAST-LIO
    b_acc_cov: 0.0001                 # Accelerometer bias noise
    b_gyr_cov: 0.0001                 # Gyroscope bias noise
    fov_degree: 360                   # MID-360 is 360° horizontal ✅
    det_range: 100.0                  # Max LiDAR range used
    extrinsic_est_en: false           # Use factory calibration (correct)
    extrinsic_T: [-0.011, -0.02329, 0.04412]   # Factory LiDAR→IMU translation (m)
    extrinsic_R: [1,0,0, 0,1,0, 0,0,1]         # Factory rotation (identity)

publish:
    scan_publish_en: true
    dense_publish_en: true
    scan_bodyframe_pub_en: false      # ← CHANGE THIS TO false
                                      #   SUPER needs world-frame cloud only
```

> **⚠ CHANGE REQUIRED:** Set `scan_bodyframe_pub_en: false`.
> SUPER's ROG-Map requires the world-frame `/cloud_registered`.
> The body-frame cloud is not needed and wastes bandwidth.

### 5.4 MID-360 Mounting Angle — Update Extrinsic if Tilted

If the MID-360 is mounted at an angle (tilted forward, rotated), the
`extrinsic_R` matrix must reflect the physical rotation.

Example — if MID-360 is mounted rotated 90° around Z-axis:
```yaml
extrinsic_R: [0, -1, 0,
               1,  0, 0,
               0,  0, 1]
```

**Ask yourself: Is the MID-360 mounted perfectly level and facing forward?**
If yes → identity matrix is correct.
If no → measure the angle and update.

### 5.5 Topic Remap — Fix /Odometry → /lidar_slam/odom

FAST-LIO2 publishes to `/Odometry`. SUPER subscribes to `/lidar_slam/odom`.
Fix this in the FAST-LIO2 launch file:

```python
# In fast_lio launch file, add remappings to the node:
Node(
    package='fast_lio',
    executable='fastlio_mapping',
    name='laserMapping',
    output='screen',
    remappings=[
        ('/Odometry', '/lidar_slam/odom'),        # ← critical remap
    ],
    parameters=[...],
)
```

---

## Section 6 — One-Time Jetson Fixes (Run Once, Then Done)

```bash
# Fix 1 — MAVROS geoid dataset (prevents silent crash on startup)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Fix 2 — Serial port permissions
sudo usermod -aG dialout ideas-ad
# Log out and back in after this

# Fix 3 — Livox Ethernet IP (Livox is at 192.168.1.154, Jetson must be .50)
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up
# Verify:
ping -c 3 192.168.1.154

# Make the IP permanent:
sudo bash -c 'cat > /etc/netplan/02-livox.yaml << EOF
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.50/24]
      dhcp4: false
EOF'
sudo netplan apply

# Fix 4 — Livox driver: change xfer_format to PointCloud2 (needed by FAST-LIO2)
# Edit: ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
# Change:  xfer_format = 1
# To:      xfer_format = 0
```

---

## Section 7 — The Bridge Node (Must Be Written)

This is the only piece of code that does not exist and cannot be skipped.

### What It Does

```
Input  → /planning/pos_cmd       (mars_quadrotor_msgs/PositionCommand)
       → /lidar_slam/odom        (nav_msgs/Odometry)

Output → /mavros/setpoint_raw/local   (mavros_msgs/PositionTarget)
       → /mavros/vision_pose/pose     (geometry_msgs/PoseStamped)
```

### Why Two Outputs Are Needed

```
/mavros/setpoint_raw/local  = "where to fly to" (from SUPER planner)
/mavros/vision_pose/pose    = "where you currently are" (from FAST-LIO2)

ArduPilot needs BOTH:
  - Without the setpoint:    FC does not know where to go
  - Without the vision pose: EKF3 has no position — refuses to arm/fly
```

### Bridge Node — Suggested Location

```
super_ws/src/SUPER/misc/ardu_bridge/
├── CMakeLists.txt
├── package.xml
└── src/
    └── ardu_bridge_node.cpp   (or .py)
```

### Bridge Logic (Python pseudocode — ~100 lines)

```python
import rclpy
from rclpy.node import Node
from mars_quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class ArduBridge(Node):
    def __init__(self):
        super().__init__('ardu_bridge')

        # Subscribe to SUPER planner output
        self.cmd_sub = self.create_subscription(
            PositionCommand, '/planning/pos_cmd',
            self.cmd_callback, 10)

        # Subscribe to FAST-LIO2 odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/lidar_slam/odom',
            self.odom_callback, 10)

        # Publish to MAVROS — where to fly
        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)

        # Publish to MAVROS — where we are (feeds ArduPilot EKF3)
        self.vision_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10)

    def cmd_callback(self, msg):
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Use position + velocity, ignore acceleration
        # ArduPilot GUIDED mode does NOT use acceleration feedforward
        target.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE   # use yaw angle, not rate
        )

        # SUPER uses ENU frame, MAVROS LOCAL_NED uses NED
        # ENU → NED:  x_ned = y_enu,  y_ned = x_enu,  z_ned = -z_enu
        target.position.x =  msg.position.y
        target.position.y =  msg.position.x
        target.position.z = -msg.position.z

        target.velocity.x =  msg.velocity.y
        target.velocity.y =  msg.velocity.x
        target.velocity.z = -msg.velocity.z

        # Yaw: ENU → NED yaw = pi/2 - yaw_enu
        import math
        target.yaw = math.pi / 2.0 - msg.yaw

        self.setpoint_pub.publish(target)

    def odom_callback(self, msg):
        # Forward FAST-LIO2 pose to ArduPilot EKF3 as vision estimate
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.vision_pub.publish(pose)
```

> **⚠ ENU vs NED Frame:**
> SUPER planner works in ENU (East-North-Up).
> MAVROS `LOCAL_NED` frame uses NED (North-East-Down).
> The bridge MUST convert between them or the drone flies sideways.

---

## Section 8 — SUPER Planner Config for Real Flight

Edit `/home/ideas-ad/super_ws/src/SUPER/super_planner/config/static_dense.yaml`
before first flight:

```yaml
fsm:
  replan_rate: 10.0          # Reduce from 15 → 10 Hz for first flights

super_planner:
  robot_r: 0.35              # SET THIS: measure your drone arm-to-arm diagonal
                             # divide by 2, add 0.1m safety margin
                             # Example: 350mm diagonal → 0.175 + 0.1 = 0.275m
  max_vel: 2.0               # START HERE — not 8.0 m/s — increase gradually
  max_acc: 3.0               # START HERE — not 15.0 m/s²
  corridor_bound_dis: 0.8    # Keep for forest
  planning_horizon: 5.0      # Keep for forest

rog_map:
  resolution: 0.1            # 0.1m for first flights (faster than 0.05m)
  map_size: [20, 20, 6]      # Adjust to your flying area
  virtual_ceil_height: 4.0   # Set to your max altitude + 1m
  virtual_ground_height: -0.3
  cloud_topic: "/cloud_registered"   # World-frame cloud from FAST-LIO2 ✅
  odom_topic: "/lidar_slam/odom"     # Remapped from FAST-LIO2 ✅
```

---

## Section 9 — Launch Order (All Terminals, Every Flight)

```
TERMINAL 1 — Livox LiDAR driver
─────────────────────────────────
source /opt/ros/humble/setup.bash
source /home/ideas-ad/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

Verify: ros2 topic hz /livox/lidar   → ~10 Hz
        ros2 topic hz /livox/imu     → ~200 Hz


TERMINAL 2 — FAST-LIO2 (LiDAR odometry + point cloud)
───────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source /home/ideas-ad/super_ws/install/setup.bash
ros2 launch fast_lio mapping_mid360.launch.py

Verify: ros2 topic echo /lidar_slam/odom    → pose changing
        ros2 topic hz /cloud_registered     → ~10 Hz
        In RViz2: move drone by hand, cloud should rotate with it


TERMINAL 3 — MAVROS (FC bridge)
─────────────────────────────────
source /opt/ros/humble/setup.bash
ros2 launch jetson_px4_setup/launch/px4_mavros_jetson.launch.py \
  fcu_url:=/dev/ttyTHS1:921600

Verify: ros2 topic echo /mavros/state  →  connected: true, mode: GUIDED
        (if connected: false, check UART wiring + FC parameters)


TERMINAL 4 — Bridge node (pos_cmd → MAVROS)
─────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source /home/ideas-ad/super_ws/install/setup.bash
ros2 run ardu_bridge ardu_bridge_node

Verify: ros2 topic list | grep mavros/setpoint      → topic exists
        ros2 topic list | grep mavros/vision_pose   → topic exists


TERMINAL 5 — SUPER planner
───────────────────────────
source /opt/ros/humble/setup.bash
source /home/ideas-ad/super_ws/install/setup.bash
ros2 run super_planner fsm_node \
  --ros-args -p config_name:=static_dense.yaml

Verify: ros2 topic hz /planning/pos_cmd  (appears only after goal is sent)
        RViz2: rog_map/grid_vis shows voxel occupancy building up


TERMINAL 6 — RViz2 (monitoring + goal sending)
────────────────────────────────────────────────
ros2 run rviz2 rviz2
  Add displays: /cloud_registered, /rog_map/grid_vis,
                /planning/pos_cmd, /mavros/local_position/pose
  Use "2D Goal Pose" plugin → publishes to /planning/click_goal
```

---

## Section 10 — Pre-Flight Checklist (Before Every Flight)

```
HARDWARE
[ ] Props OFF for all software testing — fit them LAST
[ ] Battery charged ≥ 80%
[ ] Livox MID-360 Ethernet cable connected (Jetson eth0 ↔ MID-360)
[ ] UART cable connected (Jetson 40-pin ↔ ARK FPV TELEM2) — 3 wires
[ ] All propellers secure and in correct rotation direction
[ ] Drone on flat surface, clear of obstacles (for EKF3 initialisation)

JETSON
[ ] ping 192.168.1.154  succeeds (Livox reachable)
[ ] groups ideas-ad | grep dialout  (dialout group active)
[ ] ls /usr/share/GeographicLib/geoids/egm96-5.pgm  (geoid present)

FAST-LIO2
[ ] /livox/lidar publishing at ~10 Hz
[ ] /livox/imu publishing at ~200 Hz
[ ] /lidar_slam/odom publishing (after FAST-LIO2 starts)
[ ] Move drone 0.5m in X → odom.pose.pose.position.x changes by ~0.5

MAVROS + ARDUPILOT
[ ] /mavros/state: connected=true
[ ] /mavros/state: mode=GUIDED (set via QGC or service call)
[ ] /mavros/local_position/pose: z > 0 (barometer initialised)
[ ] EKF3 status in QGC: "using ExternalNav" for position

PLANNER
[ ] /rog_map/grid_vis visible in RViz2 (map building)
[ ] Send a test goal 3m away at current altitude
[ ] /planning/pos_cmd publishes at ~100 Hz
[ ] /mavros/setpoint_raw/local receives translated command

ARMING (props still OFF for first test)
[ ] Switch FC to GUIDED mode
[ ] Arm via MAVROS service
[ ] Motor output in QGC should show non-zero → drone wants to move
[ ] Disarm immediately
[ ] Fit props only when above checks pass on 3 consecutive runs
```

---

## Section 11 — Frame Convention Reference

Understanding this prevents sideways flights.

```
FAST-LIO2 / SUPER output frame: ENU (East-North-Up)
ArduPilot / MAVROS LOCAL_NED:   NED (North-East-Down)

Conversion:
  NED_x = ENU_y     (north  = east-axis in ENU)
  NED_y = ENU_x     (east   = north-axis in ENU)
  NED_z = -ENU_z    (down   = negative up)

Yaw:
  NED yaw = π/2 − ENU yaw   (NED 0° = north; ENU 0° = east)

The bridge node in Section 7 applies this conversion.
Get it wrong and the drone flies 90° off from the goal.
```

---

## Section 12 — Summary: The 4 Steps to First Autonomous Flight

```
STEP 1 — Wire the hardware          (Section 3)
  3 wires: TX, RX, GND
  ARK FPV TELEM2 JST-GH → Jetson 40-pin header

STEP 2 — Configure ArduPilot        (Section 4)
  Set EKF3 to use ExternalNav
  Disable GPS
  Configure TELEM2 for MAVLink2

STEP 3 — Fix the software stack     (Sections 5 & 6)
  Install FAST-LIO2 ROS2 branch
  Fix topic remap: /Odometry → /lidar_slam/odom
  Fix geoid, dialout, Livox IP, xfer_format

STEP 4 — Write the bridge node      (Section 7)
  ~100 lines of Python
  Translates PositionCommand → PositionTarget (ENU→NED)
  Forwards odometry to ArduPilot EKF3

After these 4 steps:
  Launch all 6 terminals (Section 9)
  Send a 2D goal in RViz2
  Drone flies autonomously to destination
  Replans in real time around obstacles from Livox point cloud
```

---

*plantofly.md — updated with confirmed hardware (ARK FPV, MID-360 built-in IMU)*
*Last updated: based on full codebase read of super_ws + ws_livox + confirmed answers*
