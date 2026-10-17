---
paths:
  - "scripts/laptop_*"
  - "scripts/gcs_laptop.sh"
  - "scripts/gcs/rviz_agent.py"
  - "scripts/set_hostname.sh"
  - "rviz/**"
  - "super_bringup/scripts/offboard_viz.py"
  - "drone-dtp/ros2_ws/src/dtp_perception/dtp_perception/dtp_image_overlay.py"
  - "drone-dtp/ros2_ws/src/dtp_perception/dtp_perception/dtp_viz.py"
  - "drone-dtp/ros2_ws/src/dtp_perception/launch/dtp_viz.launch.py"
---

# Viewing the live stack from a laptop

Scope: RViz on a laptop over DDS, the DTP image view, and what breaks it. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

## Viewing the live stack in RViz from a laptop
The Jetson normally runs headless (`RVIZ=false bash scripts/run.sh`, or just `bash scripts/run.sh`
with no `$DISPLAY` — no X server on this box). To watch `/cloud_registered`, `/lidar_slam/odom`, and
`/rog_map/*` build live from another machine:
1. On the laptop (Ubuntu 22.04 only — WSL2/Docker `ros:humble` otherwise), one-time:
   `bash scripts/laptop_setup.sh` (installs `ros-humble-desktop`, needs sudo + internet).
2. Put the laptop on the **same Wi-Fi** as the Jetson — DDS discovery is multicast, so it needs the
   same L2 segment; an AP with client isolation enabled will silently break it.
3. `JETSON_IP=<jetson-wifi-ip> bash scripts/laptop_viewer.sh` — this matches `ROS_DOMAIN_ID`/RMW to
   the Jetson and launches RViz with `rviz/m1_viewer.rviz`. `JETSON_IP` is only used for a
   reachability ping, not connection, but get it right or the ping check falsely says "unreachable".
   **Use the Wi-Fi IP (`wlP1p1s0`/similar), not the wired `eno1` IP (`192.168.1.50`)** — that
   interface is a point-to-point link dedicated to the MID360 and isn't routable from a laptop. Find
   it with `hostname -I` or `ip -4 addr show` on the Jetson. It's DHCP-assigned and can change
   between sessions — or just pass **`JETSON_IP=gypsy.local`**, which cannot go stale
   (`scripts/set_hostname.sh`); it is only used for a reachability ping, and mDNS now resolves it
   to the Wi-Fi address rather than the LiDAR link.
4. No topics? Confirm the Jetson stack is actually up, `sudo ufw disable` on the laptop, or ask
   about a FastDDS discovery-server fallback if the Wi-Fi blocks multicast outright.

**The laptop deliberately needs no colcon build**, so it can only display *standard* ROS 2 types.
It has no `px4_msgs`, which means **every `/fmu/...` topic is invisible there** — it cannot even
deserialise them. Anything from PX4 you want to see on the laptop has to be republished as a
standard type by a node on the Jetson. That is what `super_bringup/scripts/offboard_viz.py` does
for the Offboard setpoint (flight-test P5): cyan sphere = commanded setpoint, orange = EKF2's
estimate, line + text = the gap in cm, green/amber/red at 20/50 cm. Start it alongside the
bridges; the display is already in `rviz/m1_viewer.rviz`.

**The DTP view — person boxes on the camera picture (added 2026-09-13, bag replay only, not yet
seen on a real laptop RViz):** `JETSON_IP=gypsy.local bash scripts/laptop_dtp_viewer.sh` (=
`DTP=1 laptop_viewer.sh`) opens `rviz/dtp_viewer.rviz`: the M1 view + `/dtp/viz` + a
"Person boxes" image panel. Node detail: `docs/dtp-follower-stack.md`, "dtp_viz and
dtp_image_overlay". On top of a running `scripts/run.sh`, bring up the DTP half observe-only with
`ros2 launch dtp_bringup dtp_system.launch.py lidar:=false super:=false observe_only:=true`.
- **The chain:** `perception_node` publishes `/dtp/detections_2d` (JSON, pixel boxes, the colour
  frame's stamp; built only while subscribed) → `dtp_image_overlay` (Jetson, started by
  `dtp_viz.launch.py`, i.e. by `dtp_system.launch.py`) draws them **on the frame with the same
  stamp** and publishes `/dtp/image_annotated/compressed` (960 px JPEG, ~85 kB, ~10 Hz) →
  `scripts/laptop_image_decode.py` (laptop) makes the raw Image RViz needs.
- **Don't re-derive:** Humble's RViz Image display cannot show a CompressedImage, and
  `image_transport republish` subscribes RELIABLE with no override, so it never matches a
  best-effort image. The decoder publishes only while every subscriber is RViz/rqt, so a Jetson-side
  `ros2 bag record -a` cannot pull raw frames back over the Wi-Fi.
- **Cost, measured on replay:** overlay 17 % of a core while watched (nothing at all while not),
  decoder 7 % on the laptop side. **Assign image bytes to a `uint8[]` field as `array.array('B')`,
  never `bytes`** — rclpy then validates element by element in Python (40 % → 17 %, and 100 % of a
  core at 3.7 Hz in the decoder).
- **Seen on the first replay:** the range patch (0.65 of box height) sits on the thigh of a
  standing person, not the torso the perception comments describe.

### Added 2026-09-13 (fourth session) — laptop RViz view of the person boxes
`scripts/laptop_dtp_viewer.sh`: perception's YOLO boxes drawn on the camera picture, in RViz, plus
the DTP markers. New pieces: `/dtp/detections_2d` from `perception_node`, the `dtp_image_overlay`
node, `scripts/laptop_image_decode.py`, `rviz/dtp_viewer.rviz`. Verified on a bag replay only,
**never yet opened in a real laptop RViz**. A web-dashboard version was built first and removed at
the user's request (they want RViz). Details: "Viewing the live stack in RViz from a laptop" above.

- **An RViz was connected from the laptop** (`ros2 node list` showed `/rviz` with no local
  process). Stale topic listings with `Publisher count: 0` after a shutdown are that laptop's
  discovery cache, not a surviving node — check `ros2 node list`, not `ros2 topic list`.

- **Snap leaks its glibc into the laptop's RViz** — `rviz2: symbol lookup error:
  /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol:
  __libc_pthread_init, version GLIBC_PRIVATE`. This bit twice, and the second time the guard
  that was supposed to prevent it did nothing, because it only scrubbed `LD_LIBRARY_PATH` and
  `LD_PRELOAD` and the leak came in elsewhere — **`PATH` (`/snap/bin`), `XDG_DATA_DIRS` and
  `GIO_MODULE_DIR` are the actual carriers on stock Ubuntu.** Worse, the fix that had been
  *verified* (`env -i`) was written into a comment while the code still shipped
  `env -u LD_PRELOAD`, which is not the same thing. `scripts/laptop_viewer.sh` now launches RViz
  under an **allow-list** environment (`env -i` + only the GUI vars, ROS re-sourced inside) and
  prints every variable containing `/snap/` first, so the cause is visible rather than guessed.
  **Deny-listing environment variables does not work when you cannot enumerate the threats.**
