# Flight-review videos from a bag (2026-09-26)

**The command** (Jetson, flight stack down):

```
bash drone-dtp/scripts/flight_video.sh bags/<bag_dir>                  # → bags/videos/<bag_dir>.mp4
bash drone-dtp/scripts/flight_video.sh bags/<bag_dir> out.mp4          # another output path
bash drone-dtp/scripts/flight_video.sh bags/<bag_dir> stills --png 361,390   # PNGs at BAG times
```

It writes a 1920×1080, 30 fps, H.264 MP4 covering first arm − 10 s to disarm + 5 s, in real
time. For a 90 s flight it takes ~9 min: ~7 s extract, ~1 min detect (GPU), ~7 min render and
encode (CPU). The output is ~80 MB. `bags/` is gitignored, and so are the videos. To pull one to
a laptop:
`scp ideas-ad@gypsy.local:~/SUPER-Human-DTP/bags/videos/<name>.mp4 .`

## 1. What is on screen

- **Camera (left, 1280×720, the recorded `/camera/color/image_raw/compressed`).**
  - Person boxes. The locked walker is green (`LOCKED #<track> <range>`), amber while
    COASTING. Everyone else is blue, labelled with the detector confidence.
  - A ◆ marks the follower's own 3-D estimate of the walker (`/dtp/follow_target`), projected into
    the frame.
  - The top bar shows the wall clock, T± seconds from the arm, the PX4 mode, armed/disarmed and
    the lock state.
  - A red `LOW: x m above ground` banner appears when the ToF reads < 1.0 m **in Offboard**.
- **Tiles (top right).**
  - Ground speed (EKF2 `vx,vy`). The sub-line is SUPER's commanded horizontal speed
    (`pos_cmd`), marked "not flown" outside Offboard.
  - Height above ground (ToF `dist_bottom`) and climb rate.
  - Range to the walker, and the walker's speed (`follow_target`).
  - "Camera off the walker": the drone's FAST-LIO yaw against the bearing to the walker.
- **State lines.**
  - `/mux/state`: the state and reason, plus goal, alt floor and terrain.
  - `/dtp/lock_state`: the state and reason.
  - SUPER's goal (`/planning/goal_status`), in amber with "asked X → clipped" when the requested
    z exceeded the used z by more than 0.1 m.
- **Map (right).** The whole flight's path is shown faint. The path flown so far is drawn on top
  of it, green in Offboard. There is an 8 s walker trail, SUPER's goal (×), other people, the
  camera's field of view (±46°, 5 m), H for take-off, and a 5 m grid in `lidar_init`.
- **Plots (bottom).** Speed (EKF, SUPER commanded, walker), height (ToF, FAST-LIO z) and range to
  the walker, over the whole window. They are shaded by armed PX4 mode (green Offboard, blue
  Position, amber Stabilized). A white cursor moves with the video. The axes grow when the data
  exceeds 3 m/s, 3.2 m or 8 m.

## 2. How it is made: three stages, stages 1–2 cached

`flight_video.sh` refuses:
- a bag with no `metadata.yaml`: either it is still being recorded (reading its `.db3` kills the
  recorder, see `CLAUDE.md`), or it was cut and not reindexed;
- a bag with more than one `.db3`.

It sources `dtp_env_jetson.sh`, then runs the stages below. The caches are in
`~/.cache/flight_video/<bag>/`; delete that directory to redo stages 1–2.

1. **`flight_video_extract.py` → `tele.pkl`.** Reads the `.db3` read-only with sqlite and
   `rclpy` deserialisation. It takes every series the video needs, keyed by **bag time** (receive
   time since the bag's first message, the "bag t" the flight log quotes), plus header stamps.
   - It also takes `/tf_static` and the colour `camera_info`.
   - Optional topics may be missing (e.g. `/dtp/terrain` before 2026-09-22).
   - It sets the window from `vehicle_status` arming. More than one arm gives one window
     spanning them all. A bag that never armed gets the whole bag.
2. **`flight_video_detect.py` → `dets.pkl`.** Re-runs **`~/models/yolo11s.engine` with the flown
   settings** (conf 0.25, imgsz 640, class person) on every frame in the window, at ~47 fps.
3. **`flight_video_render.py`** draws each frame, with numpy/OpenCV for shapes and PIL
   (DejaVu fonts) for text. It pipes raw BGR into `gst-launch-1.0 … x264enc bitrate=10000
   speed-preset=fast ! mp4mux faststart=true`. The plots are drawn once with matplotlib, and
   only the cursor moves. Output frame *k* shows the newest camera frame with bag time
   ≤ window start + k/30.

## 3. Why the boxes are re-detected, and how "locked" is decided

**The aircraft's own 2-D boxes are not in any stage_b bag.** `perception_node` publishes
`/dtp/detections_2d` only while something subscribes (`laptop-rviz.md`). The boxes in the video are
therefore what the flown detector finds on each recorded frame at 30 Hz, not what the aircraft saw
at its 8 Hz. The video says so in its caption.

**Which box is the locked one comes from the aircraft's real state.**
- The box has to be the one nearest the projected `follow_target`, while `/dtp/lock_state` is
  LOCKED or COASTING. The projection's column must lie within 90 px of the box's edge.
- **The projection:** `lidar_init` → `lidar_imu` comes from `/lidar_slam/odom`, interpolated
  (position lerp, orientation nlerp) to the image's header stamp. `lidar_imu` →
  `camera_color_optical_frame` is composed from the bag's own `/tf_static` (the deployed
  calibration, via `static_calib_tf`). The camera model is `cv2.projectPoints` with the recorded
  K and plumb_bob distortion.
- **The estimate is carried forward on its own velocity** by (image stamp − target stamp, median
  0.23 s), because `follow_target` is ~0.23 s older than the frame it is drawn on.
- **Measured on the 2026-09-26 19:57 bag**, over 1725 frames with exactly one box and a fresh
  target:
  - the projected column minus the box centre has median 0 px;
  - without the velocity step, p10/p90 are −43/+75 px and 81 % are within 60 px;
  - with it, −35/+60 px and **88 % within 60 px**.

  The ◆ still trails a walker who crosses the frame sideways. With the walker standing still it
  sits on the thigh, the range patch.

## 4. Traps met building it

- **This OpenCV's FFMPEG backend cannot write H.264.** `avc1`/`H264` fail (`h264_v4l2m2m`: no
  device), and only `mp4v` opens, which browsers and phones do not play. There is no `ffmpeg`
  binary either. GStreamer's `x264enc` is installed and is what the renderer uses.
- **`import ultralytics` fails outside `dtp_env_jetson.sh`:** `libcudss.so.0` is not found.
  The env script adds it to `LD_LIBRARY_PATH`.
- **`mp4mux faststart=true` leaves the output at 0 bytes until the very end.** That is normal
  while it renders.
- **Run the wrapper through a real path, or it resolves wrongly.**
  `~/super_ws/src/drone-dtp` is a symlink, so the wrapper uses `pwd -P`. Otherwise the default
  output would land in `~/super_ws/src/bags/`.
- **Stamps (2026-09-26 19:57 bag):**
  - the colour image header stamp ≈ its receive time (median 0.001 s);
  - `/lidar_slam/odom` stamps are 0.055 s behind receive;
  - `/dtp/follow_target` stamps are 0.19 s behind receive, and never equal any image's stamp
    exactly.
- **`goal_status` says SNAPPED on every goal:** 158/158 in that bag, median move 0.065 m, which
  is grid rounding. So the video does not show the status word. It flags only a z cut of more
  than 0.1 m (26/158 there: the 2.25 m ceiling, `flight-log.md`).
- **`pos_cmd` keeps coming after a manual takeover,** swinging up to ~2.8 m/s. PX4 is not in
  Offboard, so none of it is flown. The plot draws it faint, and the tile says "not flown".
  Before the 2026-09-22 engage gate, it also runs before arming.

## 5. Videos made

| Video (`bags/videos/`) | Bag | Flight |
|---|---|---|
| `flight1_2026-09-26_1957_follow.mp4` (made before the wrapper; the same drawing apart from the title) | `stage_b_2026-09-26_195108` | 2026-09-26 19:57, Flight 1: follow, then 0.58 m AGL at the turnaround (~19:57:39–41) |
| `stage_b_2026-09-21_195658_salvaged.mp4` | `stage_b_2026-09-21_195658_salvaged` | 2026-09-21 20:05: kept pace, 3.0 m engage step (1.6 m/s at 20:06:00) |
