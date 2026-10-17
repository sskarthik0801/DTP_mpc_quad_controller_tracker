# Evidence: LiDAR points on the walker, and the venue's ground profile (2026-09-30)

These are the scripts behind two sets of numbers from `bags/stage_b_2026-09-29_122511`: the
LiDAR ranging numbers in `docs/plan-veto-height-and-lidar-range.md` Part B, and the geofence
sizing in `.claude/rules/dtp-follow-behaviour.md` ("the geofence is 38 m long"). Each takes the
bag directory as its only argument. Source the workspace first:
`source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash`.

- `lidar_on_person.py` counts the points on the walker per 0.1 s scan of
  `/cloud_registered_body`, in two parts:
  - **(a)** with the walker located by the camera track, by range bin;
  - **(b)** at t 443.6–453.4, with no track, as the cluster in the walker's lane.
  - Result: 1–5 points per scan at 10–17 m, usually 0 at 3–9 m.
  - **Its ground profile (`GX`/`GZ`) is this venue's**, from `ground.py`.
- `ground.py` gives the ground height along +x in `lidar_init` (the 5th percentile of points
  with |y| < 1.5, per 2 m). Result: −0.13 at x 0, +0.8 at x 28, +1.1 at x 36, +1.5 at x 44,
  and a bank past x 48.
- `corridor.py` draws an ASCII top-down obstacle map, x −4…70, y ±8. It shows the metal fence
  at y ≈ −2.2, the trees at y ≈ +2.2 from x ≈ 40 and the bank at x 50–54.
- `_load.py` is shared by the last two. It takes every 4th scan and every 2nd point, and
  odom-transforms them into `lidar_init`.

`ground.py`, `corridor.py` and `_load.py` were **re-created** from the session's code after its
scratchpad was reset. The logic is the same as what produced the numbers, but these files have
not been re-run. `lidar_on_person.py` is the file that ran.

**Read-only; each reads the whole 8.7 GB bag (a few minutes).** Never run them while
`ros2 bag record` is writing that bag.

## Step 1 pre-build measurements (2026-09-30, second session)

The numbers behind Part B Step 1's design ("camera near, LiDAR far"), from the five flight bags
(`stage_b_2026-09-29_122511`, `-09-26_195108`, `-09-26_200650`, `-09-21_183532`,
`-09-21_195658_salvaged`). They are CPU-only and read-only on the bags. Every scan and every
camera frame is used, with no subsampling.

### Reproduce
```
source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
cd docs/evidence/lidar-range-2026-09-30
python3 _step1_cache.py ../../../bags/<bag>        # once per bag, ~15 s each: odom, clouds in lidar_init, targets -> .npz
python3 stereo_lidar_bias.py                        # (1) ~80 s; writes bias_rows.csv (read by the next script)
python3 lidar_box_points.py                         # (2)+(3) ~130 s; writes box_rows.csv
python3 cpu_cost.py                                 # (5) <10 s
python3 overlay_check.py stage_b_2026-09-29_122511 476.12 449.99   # (4) visual check of the chain
```
The cache and the outputs go to the session scratchpad (`DEFAULT_CACHE` in `_step1_cache.py`,
`OUT` in each script). Change them if that directory is gone.

- `_step1_lib.py` holds the shared geometry:
  - a local ground grid (2 m cells, 5th percentile);
  - a static/dynamic voxel test (0.25 m voxels; a point is static if its voxel was hit in ≥ 4
    one-second bins more than 3 s away);
  - depth_source's nearest-surface rule;
  - the MID360 −7° floor rotated by the odom attitude.
- `_walker_track.py` follows the walker's dynamic points past the camera's 10 m cap, seeded where
  camera and LiDAR agree.

### What each script found
- **`stereo_lidar_bias.py` (1): camera − LiDAR horizontal range, 3–10 m.**
  - Standing people (the disc gate, no lag): median −0.07…+0.18 m at 3–8 m, MAD 0.08–0.36;
    +0.36 at 8–9 m (one track).
  - Walkers: the published camera position lags by τ ≈ 0.3–1.0 s. That is the EMA α 0.25 at
    8 Hz plus the depth/colour skew. At 7–10 m the difference is −0.69 m receding and +0.98 m
    approaching.
  - The camera has gross errors: |error| > 1 m in 2.5 % of frames at 5–7 m and 12 % at 7–10 m.
    Example: 09-29 t 475–477 and 487–489, where the camera said 5.5 m and the walker was at
    7.7–8.2 m.
- **`lidar_box_points.py` (2): points on the walker.** These are dynamic points within 0.5 m,
  counted with perfect motion compensation, pooled over 1,847 windows.

  | range | 0.4 s ≥ 3 points | 0.6 s ≥ 3 points |
  |---|---|---|
  | 8–10 m | 75 % | 79 % |
  | 10–12 m | 86 % | 88 % |
  | 12–15 m | 91 % | 96 % |
  | 15–17 m | 75 % | 79 % |
  | 17–25 m | 58 % | 69 % |

  - With the body above the LiDAR floor, 10–17 m is 100 %.
  - The misses are the aircraft's attitude. The floor is −7° plus the nose-up pitch. At 09-29
    t 501.5–505 the pitch was +15…25° and the walker was invisible.
  - The walker moves p50 0.47 / p90 1.04 m in 0.4 s and 0.70 / 1.56 m in 0.6 s, almost all of
    it along the line of sight. They ran at up to 3.3 m/s.
- **`lidar_box_points.py` (3): the whole-body box.** The box is 0.48 × 1.44 m, the gate ±3 m,
  and the rule is nearest-surface with ≥ 2 points.
  - With a perfect calibration, the first background point behind the walker is p50 1.3–2.1 m
    behind. It is within 0.6 m in 0–7 % of windows (8–17 m).
  - The rule is right in 84–94 % of windows at 8–15 m and says FAR in ≤ 2 %.
  - With the ~3° vertical misprojection measured here, ground in front of the walker enters the
    box. The rule then says NEAR in 23–60 % of windows.
  - Dropping points < 0.25 m above the ground restores 77–81 % correct.
- **Frame chain (4).**
  - `/cloud_registered_body` is in `lidar_imu`. Camera←body = `T_camera_lidar` · [I | −extrinsic_T].
  - `overlay_check.py` shows the LiDAR walker points ~20–45 px ABOVE the walker in the image.
  - The camera's own lifted patch height above the LiDAR ground falls 0.041–0.060 m per metre
    of range, in all five bags, including on the ground with motors off.
  - Both point to a ~2.3–3.5° pitch disagreement between the camera and the LiDAR.
  - `dv_sweep.py` (score_alignment widened to ±60 px on single outdoor frames) was
    **inconclusive**: the peaks are scattered on both axes. Do not use it as evidence either way.
- **`cpu_cost.py` (5).** The 0.5 s window is 19.4 k points (3.9 k per scan, not 7 k).
  - World→camera, project and box mask: 0.74 ms per frame, about 1 % of a core at 15 Hz.
  - With the 5 scans re-transformed every frame: 1.1 ms.
  - A full z-buffered raster: 1.5 ms.
  - Measured at MAXN, single thread.

**Caveats.**
- The truth is the LiDAR's dynamic cluster. It cannot see a walker who stands still for more than
  about 4 s, or one below the floor.
- The tracker has a selection bias: spans where the walker was invisible for more than 1.5 s are
  not windows at all.
- Only 09-29 and 09-26 20:06 have 10–17 m spans.

## Step 0: does the flown detector see the walker at 10–17 m? (2026-09-30, `detector_range_study.py`)

The Part B Step 0 go/no-go. It re-runs the flown detector on the recorded colour frames of the
five flight bags: `~/models/yolo11s.engine`, conf 0.25, imgsz 640, class person. It runs
`model.predict` on every frame, and `model.track` (BoT-SORT, the flown call) on every 4th frame.
The range truth comes from the LiDAR, not the camera.

### Reproduce
The flight stack must be down, because this needs the GPU.
```
source drone-dtp/ros2_ws/dtp_env_jetson.sh           # ROS + libcudss (ultralytics)
python3 docs/evidence/lidar-range-2026-09-30/detector_range_study.py OUT_DIR \
    bags/stage_b_2026-09-29_122511 bags/stage_b_2026-09-26_195108 bags/stage_b_2026-09-26_200650 \
    bags/stage_b_2026-09-21_183532 bags/stage_b_2026-09-21_195658_salvaged
python3 docs/evidence/lidar-range-2026-09-30/detector_range_study.py OUT_DIR --report   # tables again
```
- **Runtime.** About 13 min from cold for all five bags, all frames, none subsampled:
  - ~3.5 min extracting telemetry and LiDAR, ~40 s per bag;
  - ~8 min detecting, 14.6 k frames at ~30 fps (every 2nd frame in the unarmed spans of 09-29);
  - seconds for everything else.
- **Caches.** Each stage is cached in `OUT_DIR/<bag>/`: `tele`, `lidar`, `walker`, `dets.pkl`.
  Delete a file to redo that stage.
- **Output.** One row per frame and person goes to `OUT_DIR/<bag>/frames.csv` and
  `OUT_DIR/frames_all.csv`.
- **Read-only on the bags** (sqlite `mode=ro`). It refuses a bag without a `metadata.yaml`.

### How the truth is built
- **The walker is tracked on the DYNAMIC LiDAR points**, forward and backward over the whole bag.
  "Dynamic" means points 0.3–2.1 m above the local ground whose 0.25 m cell was not hit 15–60 s
  away. The tracker is seeded and steered by the camera's **bearing** only.
- **The camera's range is not used as truth past the LiDAR floor.** Past ~8 m it reads short:
  on 09-29 it said 5–6 m for a walker the LiDAR put at 9–13 m.
- **A person is counted only when the whole 1.75 m body projects inside the image.** The
  projection uses the bag's own `/tf_static` and `camera_info`.
- **A box counts as the walker's only if it lies on the projected column** and overlaps the
  projected body.
- **Validation (09-29).** A matched box's centre is a median 3–8 px from the projected column,
  and its height is 0.92–0.95 of the geometric height in every bin.
- **`EXTRA_PERSONS`.** One more person was found in the 09-21 20:05 bag, walking in from 20 m
  after the landing. It came from an independent scan for moving person-sized clusters and was
  confirmed by eye. Every other candidate that scan found was trees.

### Result
**Daylight** (09-29, the only day bag with the walker past 8 m):
- **100 % of in-FOV frames detected at 8–17 m**, hovering and moving alike. The frame counts are
  434 / 351 / 297 / 34 airborne.
- 95 % at 17–20 m and 79 % at 20–30 m. The median box is 62 px tall at 15–17 m and 41 px at
  20–30 m (full resolution; the engine sees half that).
- BoT-SORT at ~8 Hz keeps 98–100 % to 17 m, and ~50 % past it.

**Dusk** (09-26 20:12, 09-21 20:05):
- 62 % at 8–10 m, 5–8 % at 10–12 m, and 0–41 % at 12–15 m.
- The boxes there are larger than the day boxes that were detected, so the limit at dusk is
  light and blur, not pixels.

## Step 3: the hybrid range source scored on replays (2026-09-30, third session)

The camera-near / LiDAR-far hybrid (`lidar_box_range.py`, `perception_node` behind
`lidar_range_enable`) replayed through the real `perception_node` and `target_manager`, and scored
against the Step 0 LiDAR-cluster walker truth (`detector_range_study.py`'s caches). **Not flown.**

### Reproduce
The flight stack must be down (GPU), and only one replay may run at a time (domain 87).
```
cd ~/SUPER-Human-DTP
# ON = dtp_params.yaml as it is; OFF = the same plus a param override (the node clamps its caps to 10 m)
START_S=428 DUR_S=88 WARM_S=60 RATE=0.25 PROBE=1 TM=1 \
    bash drone-dtp/scripts/replay_perception.sh cuda OUT A_on1 bags/stage_b_2026-09-29_122511
START_S=428 DUR_S=88 WARM_S=60 RATE=0.25 PROBE=1 TM=1 NODE_ARGS="-p lidar_range_enable:=false" \
    bash drone-dtp/scripts/replay_perception.sh cuda OUT A_off1 bags/stage_b_2026-09-29_122511
source /opt/ros/humble/setup.bash
python3 drone-dtp/scripts/range_source_score.py --runs OUT --truth STEP0_DIR \
    --bag stage_b_2026-09-29_122511 --span 435 512 --static-cache NPZ_DIR \
    --pair A_on1 A_on2 --pair A_off1 A_off2 --csv frames.csv A_on1 A_on2 A_off1 A_off2
python3 docs/evidence/lidar-range-2026-09-30/box_projection_check.py STEP0_DIR stage_b_2026-09-29_122511
```
- `replay_perception.sh` gained opt-in env vars: `START_S`/`DUR_S` (a window;
  `replay_bag_helper.py` republishes the bag's `/tf_static`, which a window skips), `NODE_ARGS`,
  `PROBE=1` (`replay_perception_probe.py`: the same entry point plus the range gate's verdict per
  track per frame) and `TM=1` (`target_manager` with `follow_enabled` true, its launch point latched
  from the bag's first odom). It now also plays `/cloud_registered_body`. `record_targets.py` also
  records `/dtp/range_source`, `/dtp/lock_state` and `/dtp/follow_target`.
- Runs used (bag s; all WARM_S 60, RATE 0.25; 260–440 s each): A 09-29 428+88 (scored 435–512),
  ON ×2 and OFF ×2; B1 09-21 18:35 369+92 (375–460); B2 09-21 20:05 salvaged 514+63 (520–576);
  B3 09-26 19:51 349+47 (355–395); C 09-26 20:06 334+57 (340–390); ON and OFF once each.
- The denominator is every processed frame with the whole walker in the colour FOV, binned by the
  LiDAR-cluster range. "Ranged" = the walker's target carries a fresh accepted camera or LiDAR range
  (not the bearing hold, not "published at its last position").

### Replay noise
- **A windowed replay does not reproduce the decimator's phase.** A_on1 shared no stamp with the
  other three A runs, and B2's two runs shared none, so frames are matched within 40 ms.
- OFF vs OFF (same phase): identical, walker position p95 0.005 m.
- ON vs ON (a different phase): walker position median 0.07 m, p95 2.7 m (during the sprint, where
  the step clamp below amplifies a timing change). Ranged share per bin within 6.4 points.

### Result, A (09-29, day, walker 4–27 m)
| in-FOV frames | 8–10 m | 10–12 m | 12–15 m | 15–17 m |
|---|---|---|---|---|
| OFF ranged (×2, identical) | 94 % | 17 % | 0 % | 0 % |
| ON ranged, run 1 / run 2 | 87 / 85 % | 71 / 78 % | 62 / 63 % | 0 / 0 % |
| ON ranged, LiDAR-visible frames only | 99 / 98 % | 78 / 85 % | 68 / 69 % | 0 / 0 % |

- **10–15 m ranged: ON 67 % / 71 % against OFF 9 %.** The 90 % bar is not met.
  - The misses are the LiDAR's: nose-up attitude-blind frames, and at 449–451.7 and 502.7–503.9
    zero points in the box.
  - **`box_projection_check.py`: 59–72 % of the walker's own LiDAR points project ABOVE the
    10 %-shrunk YOLO box at 8–15 m, and 98 % at 15–20 m.** The median point sits at the box's
    top edge (−0.01…+0.07 box heights below it); points on the body belong inside the box. Same on
    09-26 20:06 (38–58 % above). That is the ~3° pitch disagreement of Step 1, through the
    deployed calibration (the pose cross-check shows the node and the TF lift use one geometry).
  - Past 15 m nothing is ranged: `max_depth_m` 15 rejects the LiDAR range as designed.
- **The LiDAR range is accurate before the EMA:** median |e| 0.06–0.27 m at 6–15 m, and 83 / 66 /
  65 / 55 % within 0.3 m at 6–8 / 8–10 / 10–12 / 12–15 m. At walker speeds < 2 m/s, 75–77 % within
  0.3 m. At 2–3 m/s it reads 0.44 m short, and > 3 m/s 0.90 m short (the sprint at 442–443.5,
  1.0–1.4 m short). The 0.6 s window's shift uses the track velocity, which comes from the
  clamped history below.
- **What is PUBLISHED is not what the LiDAR measured.** At 12–15 m the published position is a
  median 5.5 m short. `detection.py` (frozen) clamps each raw step to `max_step_m` 0.35 m against
  its own smoothed position, then applies the EMA (α 0.25). So the published position moves at
  most 0.0875 m per processed frame, ~0.9 m/s at 10 Hz.
  - At 443.3 the LiDAR said 12.4 m, the truth was 13.4 m, and the published position was 6.2 m.
  - On the walk back (487.5–490.3, 1.3 m/s) the ON arm published the walker **1.2–1.8 m FARTHER**
    than the truth, with the raw LiDAR within 0.1 m. OFF was nearer than the truth there: its own
    lag, from the camera's dropouts past 10 m.
- **Range gate rejections at 6–10 m: 0 walker rejections in either arm** (ON 0/253 and 0/253, OFF
  0/274 ×2; all tracks 1 in A_on1). ON has 6–10 LiDAR-range rejections at > 10 m.
- **No LiDAR range longer than the truth by > 1 m** (the background side) in any run. The 7–11
  gross LiDAR errors are all SHORT (the sprint). The camera has 13–20 ranges > 1 m LONG in both
  arms (e.g. 465.5–466.0, 469.5–469.9, walker at the fence). The fence test itself is ambiguous in
  this venue: past x 20 the walker walks 0.3–0.5 m from the fence line.
- **Switching:** 7–8 camera→LiDAR, 6 back. 3–5 reverse within 2 s:
  - at 463.54/463.64, a flip at 7.3–8.0 m. The camera read 0.7 m longer than the LiDAR, so
    "beyond 8.0 m" fired at a LiDAR range of 7.3 m, and one zero-point LiDAR frame sent it back;
  - at 465.54, back to the camera at 9.56 m (truth 7.9 m). "LiDAR has 0 points; the camera has a
    range and the track is inside 8.0 m": a 1.7 m-long camera range was taken past 8 m.
  - Holds: 13–16 episodes, max 0.9 s (the 1.0 s bound held). Attitude-blind in 25–27 % of far rows.
- **Continuity (`target_manager`):** LOST in the span 3 times ON (446, 458, 503.6) against 6 OFF
  (443, 458, 467, 471, 480, 503.6). **The pilot lock still happens,** at 509.0 in all four runs,
  at 4.1–4.2 m (inside `auto_reacquire_max_range_m` 10).
- **Log:** the pose cross-check matched (0.0000 m, 0.001°) in every ON run. The FAST-LIO extrinsic
  check is unverified (no fast_lio in a replay), as expected. No hybrid error and no disable. The
  frame time median rose +4.5 ms (53.5 / 53.9 against 49.0 / 49.3 ms).
  - CPU-s over the window: ON 111.8 / 98.9, OFF 90.2 / 90.8. The ON-vs-ON spread is as large as
    the effect.

### Result, B and C
- **B, close range (never past 8 m), ON = OFF:**
  - B1 (09-21 18:35): same stamps, identical categories, walker position p95 0.012 m.
  - B3 (09-26 19:51): p95 0.022 m. 3 frames differ in the bearing association only (1 cm apart).
  - B2 (09-21 20:05): different phase, p95 0.22 m. One real difference: at 520.2–520.8 the camera
    had no depth, and ON ranged the walker by LiDAR at 6.2 m (60–80 points, within 0.1 m) where
    OFF dropped the detection (6–8 m: 100 % against 84 %).
  - 0 rejections at 6–10 m in all six runs.
- **C, dusk (09-26 20:06):** no invented ranges.
  - 0 gross LiDAR errors, no holds. The detector gives out at 10–12 m (ON 1 of 39 frames).
  - The LiDAR ranged 63 frames at 6–10 m (raw within 0.3 m in 89–100 %). At 8–10 m, published
    within 0.3 m in 63 % against 18.5 % OFF.
  - **Cycling:** 10 switches in 50 s, 6 reversed within 2 s, at 6.0–6.7 m (341.6–347). The camera
    depth comes and goes at dusk, and "camera missing" moves the track to the LiDAR below the 8 m
    switch-out. The two ranges agreed within 0.1–0.4 m, so the published walker moved ≤ 0.68 m.
- **Teardown artefact:** B1_on's log ends in an rclpy `RCLError` ("context is not valid")
  traceback. It is in `main()` after the bag ended (SIGINT during spin), not the hybrid.

### Verdict against the plan's pass criteria
- Past 10 m well above OFF: **yes** (67–71 % against 9 %). ≥ 90 % at 10–15 m: **FAIL**.
- No rise in rejected jumps at 6–10 m: **PASS** (0 ON, 0 OFF).
- LiDAR within 0.3 m in most frames: **PASS, marginally** (55–83 % by bin). It fails for a walker
  faster than 2 m/s.
- No range on the fence or trees: **PASS** for the LiDAR (no long gross error). The camera's long
  errors are unchanged from OFF.
- B equal within noise: **PASS**, with one improvement at B2 520.2–520.8. C invents no range:
  **PASS**.
- **The published position does not follow the LiDAR range** (the frozen step clamp). Until that is
  addressed, the hybrid's range reaches `target_manager` and `command_mux` metres late, on both
  sides.

### Second round (2026-09-30, fourth session): the defects fixed, re-scored
Same bags, spans, `WARM_S 60 RATE 0.25`, domain 87, one replay at a time. New runs: A ON ×2, B2 ON,
C ON. The round-1 OFF runs and the truth caches are reused. **Not flown. B1 and B3 were not re-run.**

**What changed** (`dtp_perception`; unit tests **121 passed** (was 109), ament flake8 **35**,
unchanged):
1. **The published position now follows the LiDAR** (`perception_node._source_smoothing`,
   `_reseed_history`). `detection.py` is still not edited. The frozen convention is in
   `perception_node`'s module docstring ("ported UNCHANGED and stays that way"), in
   `drone-dtp/README.md` ("the frozen predictor") and in `docs/dtp-follower-stack.md`.
   `detection.py` itself carries no header note; its only commit is the verbatim port.
   - `process_frame` reads the clamp (`max_step_m`, `max_step_m_sq`) and the EMA (`smooth_alpha`,
     `one_minus_alpha`) off the instance, per detection, right after the wrapped extractor. So on a
     LiDAR or hold sample the wrapper sets them for that one detection. It restores them at the
     next call and in `_process`'s `finally`.
   - The LiDAR values are α 0.5 and a 1.0 m clamp. The frozen values were α 0.25 and 0.35 m.
   - A **re-seed** happens at a camera→LiDAR switch, and whenever the LiDAR is more than the clamp
     from the smoothed track. It translates the track's whole `tracks_xz` history and its cached
     forecast. The newest entry then continues one step of its own motion into the LiDAR
     position, and that detection passes unclamped with α 1.
   - Translating keeps the history's shape. Overwriting the last entry would put a 10+ m/s spike
     into the velocity.
   - A LiDAR sample the range gate is about to reject keeps the frozen clamp and EMA, and is never
     re-seeded. `lidar_smooth_alpha: 0` turns all of this off.
2. **The LiDAR box's top edge is raised 45 px** (`lidar_box_range.in_box(top_extend_px)`,
   `LidarBoxRange(box_top_extend_px)`, the node parameter `lidar_box_top_extend_px`). The node
   default is 0 and the yaml sets 45. It applies to the mask and the ground-strip cut-out only.
   It is the calibration's pitch workaround; the fix is a re-calibration (`dtp-calibration.md`).
3. **The LiDAR window's motion shift uses only a fresh, measured velocity**
   (`SHIFT_VEL_MAX_AGE_S` 1.0, `SHIFT_VEL_MAX_GAP_S` 0.3, `_history_contiguous`,
   `_TrackState.vel_break_t`). Otherwise the window is the 0.4 s unshifted one. A velocity counts
   as fresh when all of these hold:
   - it was computed on a ranged frame ≤ 1 s ago;
   - its last 7 history entries have no gap > 0.3 s;
   - all 7 of those entries are newer than the track's last hold.

   Fixing item 1 alone (the "v1" runs) fixed the sprint's short read (below). It also exposed two
   stale-velocity failures:
   - **451.0–452.2:** the first LiDAR frames after the walker came back inside 15 m were shifted
     with the 5 s-old outbound velocity. They read ~14.2 m while the truth fell 14.4 → 12.9 m, one
     read 1.5 m LONG. The gate then refused the good reads.
   - **469–471:** a hold extrapolated −2.2 m/s, and the re-seed kept that slope. The next windows
     were shifted the wrong way and read ~11.0 m against a truth of 11.1 → 13.1 m, and the track
     went LOST at 471.1 (v1 A_on1).

   A first version also broke the velocity at every re-seed. That left the switch into the sprint
   unshifted and read 0.65 m short above 3 m/s. It now breaks at holds only.
4. **The switch-out uses the LiDAR's own range** when it has ≥ 3 points (`select_range_source`,
   unconditional). **On the LiDAR, 2 empty frames are ridden out** (held, when a hold is allowed)
   before the camera (`lidar_stay_grace_frames` 2).
5. **On the LiDAR with no LiDAR range,** a camera range is switched to only when all of these hold:
   - it is inside `lidar_switch_out_m`;
   - the gate would accept it;
   - the track is inside `lidar_switch_out_m`.

   Otherwise the track holds. After the hold, it takes the camera if the gate accepts it, and
   nothing if not.
6. **The optional switch back to the camera waits 1.5 s** (`lidar_min_dwell_s`). The time runs from
   the later of the switch to the LiDAR and the camera's last missing frame.
7. **`perception_node.main()`** now swallows an exception from `spin` only when the context is
   already shut down (the SIGINT race). It re-raises otherwise. It is the package's own code, and
   a unit test covers it.

Tooling:
- `range_source_score.py`'s frames CSV gained `truth`, `smooth` and `reseed`, and its switching
  line gained `reseeds`.
- `box_projection_check.py` takes `TOP_EXTEND_PX`.
- `/dtp/range_source` rows carry `smooth` (frozen / lidar / reseed) and `reseed` (m).

**Replay noise.** The two final A ON runs processed the same 769 stamps. Walker position p95 is
0.02 m, and the ranged share differs by ≤ 0.9 points per bin (0.0 at 10–15 m). With the step
clamp's amplification gone, ON-vs-ON noise is now far below the round-1 differences.

**A (09-29), final ON ×2 against round 1 ON ×2 and OFF ×2** (from `range_source_score.py`'s
frames CSV):

| | 8–10 m | 10–12 m | 12–15 m | 15–17 m |
|---|---|---|---|---|
| ranged, final ON | 80 / 79 % | 86 / 86 % | 78 / 78 % | 0 / 0 % |
| ranged, round 1 ON | 87 / 85 % | 71 / 78 % | 62 / 63 % | 0 / 0 % |
| ranged, OFF | 94 % | 17 % | 0 % | 0 % |
| raw LiDAR within 0.3 m, final (round 1) | 89 / 88 % (66 / 65) | 83 / 83 % (65 / 71) | 78 / 78 % (55 / 55) | — |
| published \|error\| median, final (round 1) | 0.18 / 0.19 m (1.40 / 1.19) | 0.12 / 0.12 m (2.45 / 0.89) | 0.14 / 0.14 m (5.53 / 5.44) | — |
| published within 0.3 m of truth, final (round 1) | 71 / 70 % (6 / 3) | 72 / 72 % (18 / 20) | 66 / 64 % (0 / 0) | — |

- **10–15 m ranged: 82 % / 82 %,** against round 1's 67 / 71 % and OFF's 9 %.
- **The published position tracks the raw LiDAR.** At walker speeds < 2 m/s, beyond 8 m:
  - final: median 0.12 m, 85 % within 0.3 m;
  - round 1: 1.61 / 1.06 m, 14 / 15 %.

  At 2–3 m/s it is 0.26 m (68 %), and above 3 m/s 0.25 m (55 %). Round 1 was 2.7–3.7 m.
- **Raw LiDAR vs truth by walker speed, beyond 8 m:**

  | walker speed | final: within 0.3 m, signed median | round 1 | v1 |
  |---|---|---|---|
  | < 2 m/s | 88 %, +0.01 m | 78–79 % | — |
  | 2–3 m/s | 82 %, +0.10 m | 30–35 %, −0.37 / −0.48 m | — |
  | > 3 m/s | 55 %, −0.03 m | 14–15 %, −0.79 / −0.95 m | 48 / 55 % |

  **The sprint's short read is gone once the track is on the LiDAR.** At 442.5–443.5 the read is
  within 0.1–0.4 m. What is left: 0.6 s of 0.8–0.9 m short reads right after the switch into the
  sprint (442.0–442.5, the camera history's clamped slope), then a re-seed.
- **Switches:** 6 camera→LiDAR and 3 back (round 1: 8/6 and 7/6). Reversals within 2 s: 2 (round
  1: 5 / 3). Both are at 453.2–454.4. The first is a hold that expired, then the camera at 8.86 m,
  then the LiDAR at 8.44 m. The second is the LiDAR empty for 3 frames at 7.23 m, then the camera.
  **The 463.54 flip and the 465.54 long camera range are gone.**
- **Holds:** 11 episodes, 62 / 63 frames, max 0.8 s (round 1: 13–16, 68–74, 0.9 s). Re-seeds: 10
  per run.
- **LOST in the span: 3** (445.95, 458.23, 503.64), both runs, the same as round 1 (OFF: 6).
- **The pilot lock is unchanged:** 508.96, 4.17–4.18 m, both runs (D13, the user's call).
- **Range-gate rejections:** at 6–10 m, 0 in either ON run (OFF: 0). Beyond 10 m, 5 / 175 (round 1:
  10 / 140, 6 / 154).
- **Gross errors:**
  - **LiDAR ranges > 1 m LONG: 0.** v1 had one, +1.5 m at 451.53, removed by item 3.
  - LiDAR > 1 m short: 1 (round 1: 11 / 7).
  - Camera > 1 m long: 11 (round 1: 13 / 16; OFF: 19–20).
  - The "non-walker on the fence" rows at 443.9–444.8 are the walker's own track leaving 15 m
    along the fence, where the truth has no fix.
- **CPU:** 107.6 / 100.0 CPU-s (round 1: 111.8 / 98.9; OFF: 90.2 / 90.8). Frame time median 54.0 /
  54.2 ms (round 1: 53.5 / 53.9; OFF: 49.0 / 49.3). The pose cross-check matched, and there were no
  tracebacks, no hybrid errors and no disable.
- **The 8–10 m ranged share fell** 87 / 85 → 80 / 79 %. The grace frames and the camera-beyond-8 m
  rule hold where round 1 took a camera range. Ranged AND within 1 m of the truth there went
  30 / 32 → 80 / 79 %.

**`box_projection_check.py`, 45 px:**

| bag | range | inside the box, 0 px → 45 px | above the box, 0 px → 45 px |
|---|---|---|---|
| 09-29 | 8–10 m | 38 → 90 % | 59 → 0 % |
| 09-29 | 10–12 m | 31 → 90 % | 67 → 0 % |
| 09-29 | 12–15 m | 28 → 86 % | 72 → 0 % |
| 09-29 | 15–20 m | 1 → 65 % | 98 → 9 % |
| 09-26 20:06 | 8–10 m | 48 → 83 % | 39 → 0 % |

**B2 (09-21 20:05):**
- 6–8 m ranged 100 % against OFF 84 %; 4–6 m 97.3 / 97.9 %.
- Published \|error\| at 6–8 m: median 0.04 m, 100 % within 0.3 m. Round 1 ON: 0.12 m, 98 %.
  OFF: 0.28 m, 65 %.
- 1 switch, 0 reversals (round 1: 2, 1). 0 rejections.
- LOST as OFF (527.45, 574.34).
- CPU 71.5 against 66.5 CPU-s; frame 47.8 against 46.6 ms.

**C (09-26 20:06, dusk):**
- **4 switches, 0 reversals** (round 1: 10, 6). 341.64 went to the LiDAR, and 347.35 came back to
  the camera once the dwell had run.
- 1 hold of 0.1 s.
- Raw LiDAR within 0.3 m in 100 % at 6–10 m. At 8–10 m, published within 0.3 m in 96 % (round 1:
  63 %; OFF: 19 %).
- No invented range: no gross LiDAR error.
- The detector still gives out at 10–12 m (1 of 39).
- LOST as OFF (379.19). CPU 66.8 against 59.9 CPU-s; frame 50.1 against 43.8 ms.

**Verdict, round 2:**
- Past 10 m well above OFF: **yes** (82 % against 9 %).
- ≥ 90 % at 10–15 m: **FAIL** at 82 / 82 %; the ON-vs-ON difference is 0.0 points. The misses
  are LiDAR-empty frames: 23 holds and 5 unranged in the two bins, and 9 undetected at 12–15 m.
  LiDAR-visible frames are ranged 94 % at 10–12 m and 84 % at 12–15 m.
- No rise in rejected jumps at 6–10 m: **PASS** (0 and 0).
- LiDAR within 0.3 m in most frames: **PASS** (78–89 % past 8 m, 55 % above 3 m/s).
- No range on the fence or trees: **PASS** (no long LiDAR error).
- B2 equal or better: **PASS**. C invents no range: **PASS**.
- **The published position follows the LiDAR: PASS** (median 0.12 m at walking pace).

**Still wrong:**
- **A hold extrapolates the range filter's rate, which is noisy at the LiDAR's 0.1 m sigma.**
  - At 469.3–470.1 it held −2.6 m/s while the walker turned and receded, and ended 4.5 m short.
  - At 451.56–451.86 it lagged an approach at 3.1 m/s, and ended 1.8 m on the FAR side.
  - A re-seed corrects each at the next LiDAR frame.
- **After a hold, a receding sprinter reads 0.4–1.0 m short for ~0.7 s** (470.5–471.2; the window
  is unshifted), with 2 gate rejections. It is the safe side.
- **`Target.velocity` still comes from the whole history,** holds and pre-gap entries included.
  Only the LiDAR shift uses the freshness rule. The height (`MAX_STEP_Z_M` clamp + EMA) takes a
  few frames to follow a re-seed.
- The 90 % bar is set by the Mid-360's −7° floor under nose-up pitch, not by the software.
- **The pilot lock (D13) is unchanged.**
- 45 px is a workaround for the pitch calibration.

Scratch (not in git): `…/scratchpad/step3/runs2` (final), `runs2/v1`, `runs2/v2`, `r2score/`,
`r1score/` (round 1 re-scored with the new columns), `compare2.py`, `facts.py`.
