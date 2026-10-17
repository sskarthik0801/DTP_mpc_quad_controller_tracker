# Camera ↔ LiDAR re-calibration on the conference table, 2026-09-30

The deployed `drone_cam_livox.yaml` was replaced on 2026-09-30 with this solve. The record,
with the downstream consequences, is in `.claude/rules/dtp-calibration.md` (the 2026-09-30 entry).
This folder holds the scripts and the numbers.

**Motors off, bench. Not flown. M5.4b (motor-on validation) is still owed.**

## Capture

- The aircraft sat on a long conference table (camera 14–15 cm above the tabletop), motors off.
- Sensors: `ros2 launch dtp_perception calib_sensors.launch.py` (the Livox publishing PointCloud2,
  our `dtp_livox_driver`; the 336L `CPC64630007M` on USB 3).
- Four static viewpoints, 21 s each, `ros2 bag record` of colour, depth, both camera_infos,
  `/livox/lidar`, `/livox/imu`, `/tf_static`, `/fmu/out/vehicle_attitude`:

  | vp | view |
  |---|---|
  | 0 | down the table: chairs at 1–3 m, the TV wall at ~5.5 m, the ceiling |
  | 1 | turned ~70° right: a concrete wall at ~1.8 m, one chair (weak geometry) |
  | 2 | the other end: a whiteboard at ~1.5–2 m, a door, glass partitions |
  | 3 | as vp0, with a black box on the table ~1.5–2 m ahead |

- Bags, pairs and solver copies: `~/calib_runs/20260930_table/` (8.8 GB, on the NX only).
  `pair_000..003.npz` + `camera_info.yaml` are in the `score_alignment` / `write_calib_yaml`
  layout. `T_camera_lidar.txt` is the deployed solve.

## Method (`icp_solve.py`)

The koide3 workstation (`dtp-lab-ws`) does not resolve from the NX, so the solve runs on the NX
with numpy/scipy only.

- `extract.py`: one viewpoint → the median of 126 depth frames (a pixel is kept if valid in
  ≥ 60 % of them), one colour frame, all ~215 LiDAR sweeps accumulated (3.7 M points), the
  driver's depth→colour TF (`T_col_dep`: −23.8 mm in colour x, 0.37° roll), PX4 attitude.
- The Gemini's own depth cloud goes through the factory depth→colour TF (tier 1) into the colour
  optical frame. It is then registered against the MID360 cloud: robust (Tukey) point-to-plane
  ICP, with LiDAR normals from a 1.5 cm voxel map. Weights come from a range-dependent stereo σ
  (0.0025·z²). Depth-discontinuity pixels are dropped. The four viewpoints are solved jointly,
  starting from the deployed file.
- It is independent of image features and of gravity. **It needs near structure.** A far-only
  solve (depth > 2.5 m) collapses to elevation −0.29° and a camera level with the LiDAR
  (z +0.3 cm). That is the rotation/translation trade the 09-11 solve fell into.

## Result

| | 2026-09-13 deploy (09-11 solve) | **2026-09-30 table** |
|---|---|---|
| optical axis elevation above the LiDAR x-y plane | −2.53° | **+1.09°** |
| optical axis azimuth in the LiDAR frame | −1.53° | **−1.24°** |
| camera origin in the LiDAR frame (x fwd, y left, z up) | (+9.4, +0.7, **+2.3**) cm | (**+13.8, +1.3, −11.5**) cm |
| rotation change | | 3.64° (3.62° of it pitch) |
| published `lidar_imu → camera_link` | (+0.084, +0.008, +0.067) m | (+0.128, +0.014, −0.071) m |

The LiDAR is 11.5 cm above the camera and 13.8 cm behind it. That matches the user's description
(2026-09-13: "the LiDAR is ABOVE the camera and a little behind"). The tabletop gives 11.9 cm
independently: the LiDAR is 26.2 cm above it and the camera 14.3 cm, each measured by that
sensor alone.

### Robustness

- vp0 alone converges to the same answer from four starts (up to 3° and 12 cm off).
- The single-view solves against the joint solve:

  | | rotation | translation |
  |---|---|---|
  | vp0 | 0.26° | ≤ 0.6 cm |
  | vp1 | 0.29° | ≤ 0.6 cm |
  | vp3 | 0.19° | ≤ 0.6 cm |
  | vp2 | 1.49° (roll; a plane square-on to the camera does not fix roll) | 1.9 cm in z |

- Leaving vp2 out moves the answer 0.07° and < 1 mm.
- Depth scale set free: 1.0028. Joint residual: median 6 mm, rms 1.7 cm.

## Independent checks

- **Tabletop plane, each sensor on its own** (`table_plane.py`, vp0 and vp3; the camera's
  near-range tabletop depth against the LiDAR's tabletop returns at 1.9–4 m). The "via T" height
  is the camera height the calibration implies; "camera" is what the camera's own depth says.

  | | normal angle, old → new | height via T vs camera, old | height via T vs camera, new |
  |---|---|---|---|
  | vp0 | 3.88° → **0.26°** | 0.286 vs 0.143 m | **0.149 vs 0.143 m** |
  | vp3 | 2.56° → **1.10°** | 0.289 vs 0.153 m | **0.152 vs 0.153 m** |

  In vp1 and vp2 the check picks a wall, not the table; there it is a distance check only (new
  within 6 mm, old 4.2 cm off).

- **LiDAR intensity vs image grey, NCC shift sweep** (`intensity_ncc.py`, z-buffered, whole
  image, ±40 px). The peak is where the LiDAR image must move to match; + dv means the LiDAR
  projects too high.

  | | old | new |
  |---|---|---|
  | vp0 | du +6.9, **dv +21.3** px | du +3.0, dv +1.6 px |
  | vp3 | du +6.5, **dv +17.4** px | du +2.2, dv +0.6 px |

  vp1 (bare wall) and vp2 (whiteboard + glass) have no intensity structure, so they are
  inconclusive for both files.

- **Overlay** (`box_tv_overlay_vp3.jpg`: LiDAR intensity, image edges in green). With the old
  file the TV rectangle (~5.5 m) and the near box (~1.5–2 m) both project ~15–25 px high. With
  the new file both sit on their image outlines. Near and far fitting at once is what separates
  rotation from translation.

- **`score_alignment` FAILS on this capture, for BOTH files.** New: du +3, dv ≤ −6 (a wide
  sweep gives dv −16, on a broad plateau from −16 to −4). Old: du +6, dv −6 (wide: +10 / −8).
  Its dv cannot be trusted in this room, for two reasons:
  - It reads the old file as dv −8. The NCC, the TV and the flight bags all put the old file at
    +17 to +21, and the old file has almost no vertical baseline, so this is not parallax. The
    cause is the mesh chair backs: their horizontal slats are strong |dI/dy| edges, and the LiDAR
    partly sees through them, which scatters range steps across every chair.
  - It has no occlusion handling. With the LiDAR 12 cm above the camera, it sees wall points just
    above each chair top that the camera cannot. Those project onto the chair and drag
    "horizontal" edges downward.

  **The du +2–3 px residual is seen by both methods:** ~0.25° of yaw, ≈ 1.5 cm at 4 m. It is
  most likely the factory depth→colour registration (the ICP aligns the depth cloud; the image
  checks test the colour camera). It is not corrected.

## What changed downstream (not re-scored)

A camera-lifted point moves by, in the LiDAR frame:
- **Δz ≈ 0.063·r − 0.14 m**: −7 cm at 1 m, 0 at 2.2 m, +11 cm at 4 m, +37 cm at 8 m, +49 cm at
  10 m;
- **+1 to +6 cm left** (Δy);
- **+4 to +25 cm forward** (Δx), more for rows below the image centre.

This removes the "camera-lifted target z is ~0.05·r too low" error.

## Gravity (not used)

- PX4 pitch on the table was +1.29° to +1.68° across the viewpoints.
- The Livox IMU's mean acceleration says −0.40° (x −0.0069 g, y +0.0123 g). That ~1.7°
  disagreement is IMU bias at this level, so gravity was not used as a reference.
