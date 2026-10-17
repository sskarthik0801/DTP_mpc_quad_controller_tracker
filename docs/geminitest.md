# Gemini 336L camera test

The attached camera is an **Orbbec Gemini 336L**, USB ID `2bc5:0807`,
confirmed on this Jetson on 2026-09-07. This integration runs the official
Orbbec ROS 2 camera node with a color-only profile and exposes standard ROS
image messages for the laptop. It is independent of the LiDAR/flight stack.

## Start it from the dashboard

The web GCS has a **Camera** panel, behind its own password (**Admin control**
in the header — see `scripts/gcs/admin.py`; the gate is enforced server-side,
so a locked client is sent no camera data at all). It starts this same launch,
and shows the live picture in the page over MJPEG — no ROS, no plugin and no build step on
the viewing machine, and it works over Tailscale, which the RViz/DDS path
cannot. It offers **Start only when the camera is actually free**, and when it
is not it names the topic or the pid that has it — and it can **subscribe to
whatever is already publishing** (the `/camera` case below) as a first-class
action, with a chip per source. See
[`scripts/gcs/README.md`](../scripts/gcs/README.md) § *The camera*.

```bash
bash scripts/gcs.sh            # then open the printed URL
```

## Start on the Jetson

From this repository:

```bash
bash scripts/gemini_camera.sh
```

(`scripts/gemini_test.sh` still works — it is a one-line shim onto the same
script. The launcher was renamed when the dashboard started running it, and
there is only ever one implementation.)

Ctrl-C in that terminal stops this camera launch. The script sources Humble
and `${SUPER_WS:-$HOME/super_ws}`, then launches the **repository copy** of
`super_bringup/launch/gemini_camera.launch.py`; no integration-package rebuild
is needed for this command. The driver must already be installed (it is on
the tested Jetson; fresh-install instructions are below).

The camera node is `/gemini/gemini`, hosted by `/gemini/camera_container`.
Defaults are 640×480, 15 FPS, MJPEG over USB, using the SDK's `libuvc`
backend. `/dev/video*` is not required. Depth, IR, IMU and point clouds are
disabled to keep this viewing test small. No camera-to-drone mounting
transform is invented; this is a 2D image viewer, not the M4 target adapter.

```bash
# Alternative profile, or select one camera by its actual serial number:
bash scripts/gemini_camera.sh color_width:=1280 color_height:=720 color_fps:=30
bash scripts/gemini_camera.sh serial_number:=YOUR_CAMERA_SERIAL
COLOR_WIDTH=1280 COLOR_HEIGHT=720 COLOR_FPS=30 bash scripts/gemini_camera.sh
```

Unsupported profiles are reported by the driver; use one from its startup
profile list. Resolution/FPS changes require restarting the launch.

| Topic | Message | Use |
| --- | --- | --- |
| `/gemini/color/image_raw` | `sensor_msgs/msg/Image` | Decoded color pixels; expensive over Wi-Fi |
| `/gemini/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG frames; preferred for the laptop |
| `/gemini/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration from the driver |

Image QoS is BEST_EFFORT/VOLATILE. Raw and compressed publishers are
subscriber-driven. The pinned driver forwards camera MJPEG directly as
`CompressedImage`, avoiding a Python decode/re-encode loop on the Jetson.
The laptop only needs standard ROS messages and a compressed image transport
plugin, not the Orbbec SDK, `px4_msgs`, or a colcon build.

### A camera node was already running during implementation

On 2026-09-07 another process already owned the USB camera, under namespace
`/camera`. It was left running. **You can view that existing stream now** at
`/camera/color/image_raw/compressed` (base topic `/camera/color/image_raw`).
Do not launch a second USB reader: it fails with `uvc_open ... Return Code:
-6` (busy). Stop the original launch in its own terminal before starting
`gemini_camera.sh`, or keep using its existing topics — the dashboard's Camera
panel will happily display whatever is already publishing, and refuses to start
a second driver on top of it.

The live check against that existing stream decoded **60 JPEG frames at
29.99 Hz, 1280×720**, averaging **168,212 bytes/frame** (~40.4 Mbit/s payload).
That verifies the connected hardware, driver, DDS reception, JPEG contents,
and increasing timestamps on the Jetson. The new launch loaded successfully,
but its default 640×480/15 profile could not acquire the occupied camera and
has **not yet been live-validated**. Laptop network reception has not been
tested from this session.

## View on the laptop

Use Ubuntu 22.04/ROS 2 Humble on the same LAN/Wi-Fi as the Jetson, with
multicast/client-to-client traffic allowed. For a fresh laptop run the
existing `bash scripts/laptop_setup.sh`, then install the viewing plugin:

```bash
sudo apt-get install ros-humble-compressed-image-transport
```

From the laptop's copy of this repository:

```bash
JETSON_IP=<jetson-wifi-ip> RVIZ_CFG="$PWD/rviz/gemini_viewer.rviz" \
  bash scripts/laptop_viewer.sh
```

This reuses the existing viewer's clean environment handling for the Snap
glibc problem. Its discovery message checks SUPER topics and may report no
SUPER stack; that is expected when only the camera is running. RViz still
opens. The `Gemini color` Image display uses base topic
`/gemini/color/image_raw` with **Transport Hint = compressed**, Reliability
**Best Effort**, Durability **Volatile**, queue depth 1.

**For the already-running `/camera` publisher**, change the Image display's
Topic to `/camera/color/image_raw`. Do not put `/compressed` into RViz's
base-topic field; select the compressed transport separately. If needed set
Global Options → Fixed Frame to `camera_color_optical_frame`; an Image
display needs no world transform. A 3D Camera display does.

Both machines must share `ROS_DOMAIN_ID` (default 0) and
`RMW_IMPLEMENTATION` (default `rmw_fastrtps_cpp`), with `ROS_LOCALHOST_ONLY=0`.
The wrappers set these defaults and accept environment overrides. Use the
Jetson's Wi-Fi IP, not LiDAR Ethernet `192.168.1.50`. Tailscale reachability
alone does not provide multicast DDS discovery.

## Verify without opening a GUI

The receive check needs `python3-opencv`, `python3-numpy`, ROS Humble and
`sensor_msgs`. These are already available on the tested Jetson. On a
laptop without OpenCV, install `python3-opencv python3-numpy` first.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/gemini_check.py

# Check the existing publisher instead:
python3 scripts/gemini_check.py --ros-args \
  -p topic:=/camera/color/image_raw/compressed
```

It receives up to 60 frames over 20 seconds, decodes JPEG data, checks frame
IDs and increasing timestamps, and prints shape/rate/bytes per frame. No
frames, corrupt data or non-increasing timestamps fail with a nonzero exit.
It does not start a camera. Run it on the laptop to prove network delivery,
not just local publication. For a renamed camera, override its `topic`.

## Install after a reimage

The camera dependency is separate from the main LiDAR installer. The tested
Orbbec checkout is clean, package version **2.9.3**, at pinned commit
`8e7cad2bfa2c4a6ac4e779be99c64e72166043af`. Do not substitute a moving branch
without checking its launch options and MJPEG publisher behavior.

For a fresh workspace (do not overwrite an existing modified driver):

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/super_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git \
  ~/super_ws/src/OrbbecSDK_ROS2
git -C ~/super_ws/src/OrbbecSDK_ROS2 checkout \
  8e7cad2bfa2c4a6ac4e779be99c64e72166043af
git -C ~/super_ws/src/OrbbecSDK_ROS2 submodule update --init --recursive
sudo apt-get install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
  libusb-1.0-0-dev libeigen3-dev
rosdep install --from-paths ~/super_ws/src/OrbbecSDK_ROS2 \
  --ignore-src -r -y --rosdistro humble
cd ~/super_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
cd ~/super_ws
colcon build --packages-select orbbec_camera_msgs orbbec_camera orbbec_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If rosdep is not initialized, follow the upstream installation guide below.
Reconnect the camera after installing USB access rules. Inspect build errors;
do not treat the existence of `install/` as proof a build completed.

The new launch is automatically installed by `super_bringup`'s existing
`install(DIRECTORY launch ...)`. To use
`ros2 launch super_bringup gemini_camera.launch.py`, first synchronize this
repository's package changes into `~/super_ws/src/super_bringup` (preserve any
workspace-only edits), build with
`colcon build --packages-select super_bringup --symlink-install`, and source
the resulting install. The direct `gemini_camera.sh` command avoids that copy.

## Troubleshooting

- USB busy / return code -6: another SDK/ROS/viewer process owns the camera.
  Run **`python3 scripts/gcs/camera.py`** — it names the USB device, the pid
  holding it and the topic it is publishing, which is the same `fuser
  /dev/bus/usb/<bus>/<device>` check without the lsusb arithmetic. (It only
  sees file descriptors belonging to your own user.) Stop it through its
  original owner; do not kill arbitrary process names.
- Permission denied: check the Orbbec udev rules and reconnect USB. Do not
  run the entire ROS stack as root to mask missing device permissions.
- No image: verify a USB 3 connection, driver startup logs and selected
  profile. Check actual message reception; a listed topic alone proves little.
- Local check passes but laptop fails: check domain/RMW, localhost-only,
  Wi-Fi client isolation, firewall DDS access, and multicast discovery.
- Large delay: use compressed transport, reduce FPS/resolution, close extra
  raw-image subscribers. The measured 720p/30 stream is already ~40 Mbit/s
  before network overhead, so bandwidth matters alongside LiDAR viewing.
- No `compressed` transport in RViz: install the laptop plugin and restart
  RViz. Do not add SDK packages to the laptop just to decode JPEG.

Sources: [Orbbec ROS 2 driver and installation guide](https://github.com/orbbec/OrbbecSDK_ROS2),
[pinned Gemini 330-series launch](https://github.com/orbbec/OrbbecSDK_ROS2/blob/8e7cad2bfa2c4a6ac4e779be99c64e72166043af/orbbec_camera/launch/gemini_330_series.launch.py).
