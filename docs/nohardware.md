> ⚠️ **PARTLY SUPERSEDED — read with care (flagged 2026-07-29).**
> The LiDAR/FAST-LIO/SUPER sim walkthrough below is still broadly useful, but every reference to
> **ArduCopter, MAVROS, or "skip the ARK FPV"** is out of date: the FC runs **PX4 v1.17.0 +
> uXRCE-DDS**, the link is live, and M2's odometry bridge is built and hardware-verified.
> Hardware is no longer the blocker it was when this was written.
>
> Also note `point_filter_num: 1` (dense tuning) is now known to peg a CPU core on the Orin Nano —
> the repo config uses `3`. See [`ImplementationDoc.md`](ImplementationDoc.md) §M1 regression.
>
> **Current source of truth:** [`ImplementationDoc.md`](ImplementationDoc.md), [`../README.md`](../README.md).

# No-Hardware Test — SUPER Planner Verification
**Goal:** Confirm the full software pipeline works (LiDAR → odometry → mapping → planning)
**What is powered:** Jetson Orin Nano + Livox MID-360 (via battery or bench supply)
**What is skipped:** UART wiring, ARK FPV, MAVROS, Bridge node

---

## What This Test Proves

```
Livox MID-360
    │  Ethernet
    ▼
livox_ros_driver2   →  /livox/lidar  /livox/imu
    ▼
FAST-LIO2           →  /lidar_slam/odom  /cloud_registered
    ▼
SUPER (ROG-Map)     →  3D voxel occupancy grid
    ▼
SUPER (Planner)     →  /planning/pos_cmd  (trajectory computed — not sent anywhere)
```

The planner output goes nowhere without the bridge node — that is fine.
You are verifying that path planning works end-to-end in software.

---

## One-Time Setup (Run Once Per Machine)

### Step 1 — Fix Livox xfer_format (must be PointCloud2 for FAST-LIO2)

```bash
# Change xfer_format from 1 to 0 in the MID-360 launch file
sed -i 's/xfer_format   = 1/xfer_format   = 0/' \
  /home/ideas-ad/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py

# Verify the change:
grep "xfer_format" /home/ideas-ad/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
# Expected: xfer_format   = 0
```

### Step 2 — Set Livox Ethernet IP on Jetson (Livox is at 192.168.1.154)

```bash
# Temporary (resets on reboot — use for first test):
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up

# Verify Livox is reachable:
ping -c 3 192.168.1.154
# Must see: 3 packets transmitted, 3 received, 0% packet loss
```

To make the IP permanent across reboots:
```bash
sudo bash -c 'cat > /etc/netplan/02-livox.yaml << EOF
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.50/24]
      dhcp4: false
EOF'
sudo netplan apply
```

### Step 3 — Install FAST-LIO2 ROS2 Branch (one time only)

```bash
cd /home/ideas-ad/super_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git FAST_LIO_ROS2 --depth 1

# Source the livox driver (FAST-LIO2 depends on its messages)
source /home/ideas-ad/ws_livox/install/setup.bash
source /opt/ros/humble/setup.bash

cd /home/ideas-ad/super_ws
colcon build --symlink-install --packages-select fast_lio

# Verify build succeeded:
# Expected: Summary: 1 packages finished
```

### Step 4 — Fix Topic Remap in FAST-LIO2 Launch File

FAST-LIO2 publishes `/Odometry`, but SUPER listens to `/lidar_slam/odom`.
Find the launch file and add the remap:

```bash
# Find the mid360 launch file in the newly cloned package:
find /home/ideas-ad/super_ws/src/FAST_LIO_ROS2 -name "*.launch.py" | grep -i mid360
```

Open that file and add `remappings` to the Node definition:

```python
# Add this line inside the Node(...) call:
remappings=[('/Odometry', '/lidar_slam/odom')],
```

If the launch file uses XML (.launch) instead of Python:
```xml
<!-- Add inside the <node> tag: -->
<remap from="/Odometry" to="/lidar_slam/odom"/>
```

---

## Launch Sequence (4 Terminals — Every Test)

Open 4 separate terminals on the Jetson.

### Terminal 1 — Livox LiDAR Driver

```bash
source /opt/ros/humble/setup.bash
source /home/ideas-ad/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**Wait for:** No error messages. Then in a spare terminal verify:
```bash
ros2 topic hz /livox/lidar   # must show ~10 Hz
ros2 topic hz /livox/imu     # must show ~200 Hz
```

If `/livox/lidar` does not appear: Livox is not reachable — re-run Step 2.

---

### Terminal 2 — FAST-LIO2 (Odometry + Point Cloud)

```bash
source /opt/ros/humble/setup.bash
source /home/ideas-ad/ws_livox/install/setup.bash
source /home/ideas-ad/super_ws/install/setup.bash
ros2 launch fast_lio mapping_mid360.launch.py
```

**Wait for:** FAST-LIO2 prints initialisation messages and goes quiet.

Verify odometry is running:
```bash
ros2 topic echo /lidar_slam/odom --once
# Must show a pose with position and orientation fields
```

**Functional test:** Pick up the drone (props OFF) and slowly move it 0.5 m in any direction.
```bash
ros2 topic echo /lidar_slam/odom | grep -A3 "position:"
# x/y/z values must change as you move the drone
```

If `/lidar_slam/odom` is missing: the remap in Step 4 was not applied — check the launch file.

---

### Terminal 3 — SUPER Planner

```bash
source /opt/ros/humble/setup.bash
source /home/ideas-ad/super_ws/install/setup.bash
ros2 run super_planner fsm_node \
  --ros-args -p config_name:=static_dense.yaml
```

**Wait for:** Node starts without crash. It will be idle (no goal yet).

Verify ROG-Map is receiving data:
```bash
ros2 topic hz /cloud_registered   # must show ~10 Hz
```

---

### Terminal 4 — RViz2 (Visualisation + Goal Sending)

```bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set **Fixed Frame** to `lidar_init` (FAST-LIO2 world frame; upstream calls it `camera_init` — renamed 2026-09-02 by `patches/fastlio_frames.py`)
2. Add display → **PointCloud2** → topic `/cloud_registered`
   - You should see a 3D point cloud of the room building up
3. Add display → **Odometry** → topic `/lidar_slam/odom`
   - You should see an arrow moving as you move the drone
4. Add display → **MarkerArray** → topic `/rog_map/grid_vis`
   - You should see voxel blocks appearing around obstacles

---

## Sending a Test Goal (Confirms Planner End-to-End)

Once the point cloud and odom are visible in RViz2:

1. In RViz2 toolbar, click **"2D Goal Pose"**
2. Click a point on the floor ~3 m away from the drone
3. Within 1–2 seconds you should see:
   - A trajectory line appear in RViz2
   - `/planning/pos_cmd` start publishing at ~100 Hz:

```bash
ros2 topic hz /planning/pos_cmd
# Expected: ~100 Hz
```

This confirms the full pipeline is working:
**LiDAR → odometry → occupancy map → path search → trajectory optimisation → position commands**

---

## What Success Looks Like

| Check | Command | Expected |
|-------|---------|----------|
| Livox connected | `ping 192.168.1.154` | 0% packet loss |
| LiDAR data | `ros2 topic hz /livox/lidar` | ~10 Hz |
| IMU data | `ros2 topic hz /livox/imu` | ~200 Hz |
| Odometry | `ros2 topic echo /lidar_slam/odom --once` | pose fields populated |
| Point cloud | `ros2 topic hz /cloud_registered` | ~10 Hz |
| Voxel map | RViz2 `/rog_map/grid_vis` | coloured voxels visible |
| Planner output | `ros2 topic hz /planning/pos_cmd` | ~100 Hz after goal sent |

---

## Common Failures

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `/livox/lidar` never appears | Ethernet IP not set | Re-run Step 2 |
| FAST-LIO2 crashes immediately | `xfer_format` still 1 | Re-run Step 1 |
| `/lidar_slam/odom` missing | Topic remap not applied | Re-check Step 4 launch file |
| Point cloud builds but odom drifts | MID-360 tilted, extrinsic_R wrong | See plantofly.md §5.4 |
| Planner starts but no trajectory | ROG-Map not receiving cloud | Check `/cloud_registered` Hz |
| RViz2 Fixed Frame error | Wrong frame name | Try `lidar_imu` or `map` if `lidar_init` fails (upstream names are `body` / `camera_init`) |

---

## What Is NOT Tested Here

- UART wiring (no FC connected)
- MAVROS / ArduPilot communication
- Bridge node (not written yet)
- Actual flight — drone stays on ground throughout

Once parts arrive, continue from plantofly.md §3 (UART wiring).
