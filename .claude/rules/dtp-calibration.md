---
paths:
  - "drone-dtp/ros2_ws/src/dtp_perception/calib/**"
  - "drone-dtp/ros2_ws/src/dtp_perception/dtp_perception/calibration/**"
  - "drone-dtp/ros2_ws/src/dtp_perception/dtp_perception/calib_io.py"
  - "drone-dtp/ros2_ws/src/dtp_perception/dtp_perception/static_calib_tf.py"
  - "drone-dtp/scripts/*calib*"
  - "docs/calibrate.md"
  - "docs/evidence/calib-*/**"
---

# Camera ↔ LiDAR calibration

Scope: the deployed Gemini 336L ↔ MID360 extrinsic, how it was chosen, and what is still wrong with it. The teardown/re-measure runbook is `docs/calibrate.md`. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

- **✅ 2026-09-30: the table re-calibration is DEPLOYED (user). It replaces the 09-11 solve (the
  next entry, now history). Motors off. Not flown.** Record:
  `docs/evidence/calib-2026-09-30-table/README.md`.
  - **What changed:**
    - The camera's optical axis is **+1.09°** above the LiDAR's x-y plane (was −2.53°). The
      rotation moved 3.64°, 3.62° of it in pitch.
    - The camera origin is **(+13.8, +1.3, −11.5) cm** in the LiDAR frame (was +2.3 cm *above*
      it). So the LiDAR sits 11.5 cm above and 13.8 cm behind the camera, which matches the
      user's 09-13 description.
    - `static_calib_tf` now publishes `lidar_imu → camera_link` = (+0.128, +0.014, −0.071) m
      (was +0.084, +0.008, +0.067).
  - **How:** the koide3 workstation `dtp-lab-ws` does not resolve from the NX, so it was solved
    on the NX. Robust point-to-plane ICP of the Gemini's own depth cloud (through the factory
    depth→colour TF) against the accumulated MID360 cloud, four static 21 s viewpoints solved
    jointly. Scripts: the evidence folder. Data: `~/calib_runs/20260930_table/` (bags, pairs,
    `solver/`).
  - **It needs near structure.** A far-only solve (> 2.5 m) collapses to "camera level with the
    LiDAR, same height", the trade the 09-11 solve made.
  - **Robustness:**
    - four starting guesses give the same answer;
    - three single-view solves are within 0.3° and 0.6 cm of the joint solve, and the fourth
      (a square-on whiteboard) is 1.5° off in roll only;
    - leaving that view out moves the answer 0.07°;
    - depth scale set free: 1.0028.
  - **Independent checks, all pass:**
    - **Tabletop plane**, each sensor alone: the camera height the calibration implies is
      within 0.6 cm of the camera's own, where the old file was 14 cm off. The normal agrees to
      0.26° (vp0) and 1.10° (vp3), against 3.88° / 2.56° for the old file.
    - **LiDAR-intensity NCC**: the peak moves from (+7, **+21**) px (old) to (+3, +2) px, and
      from (+7, +17) px to (+2, +1) px on a second view.
    - **Overlay:** a near box (1.5–2 m) and the TV (5.5 m) both land on their outlines.
  - **⚠️ `score_alignment` FAILS on this capture, for BOTH files** (new: du +3 / dv ≤ −6; old:
    du +6 / dv −6). Its dv is unusable in this room: it reads the *old* file as dv −8 when
    everything else says +17 to +21. The causes are mesh chair backs and no occlusion handling
    against a 12 cm vertical baseline. It was deployed on the other checks at the user's
    instruction. `recalib.sh verify` would have refused it.
  - **Residual:** du +2–3 px on both image methods, ≈ 0.25° yaw, ≈ 1.5 cm at 4 m. Probably the
    factory depth→colour registration. Not corrected.
  - **What it changes downstream (none re-scored):** a camera-lifted point moves by
    **Δz ≈ 0.063·r − 0.14 m** (−7 cm at 1 m, 0 at 2.2 m, +11 cm at 4 m, +37 cm at 8 m, +49 cm at
    10 m), +1–6 cm left and +4–25 cm forward.
    - `lidar_box_top_extend_px` 45 was a workaround for the old pitch error. It now raises the
      LiDAR mask 45 px above a correctly projected box: re-score (Part B Step 3) and probably
      set it back to 0. The user's call.
    - `proximity_veto_half_height_m` 3.0 was sized on the old, low tracked z. The tracked point
      now sits higher at range.
    - The Part B replay scores, the 14:55 bench numbers and the dtp_viz cylinder heights were all
      computed with the old file.
  - **Tests:** 76 passed (write_calib_identity, static_calib_tf, lidar_box_range,
    perception_hybrid, dtp_viz). `test_lidar_box_range`'s composition check now pins the 09-13
    matrix as a literal, because it tests the brief's hand-computed numbers. It is live-loaded
    by `static_calib_tf`, with the camera and LiDAR identity verified.
  - **Undo:** `cp ~/backups/calib-pre-20260930-table-deploy/drone_cam_livox.yaml
    drone-dtp/ros2_ws/src/dtp_perception/calib/`, then relaunch. The install copy is a symlink
    to the source, so no rebuild is needed.
  - **Still owed:** M5.4b motor-on validation, and a Stage A look at the lifted heights.
- **✅ (superseded 2026-09-30, entry above) The 09-11 camera↔LiDAR calibration is DEPLOYED
  (2026-09-13 16:52, at the user's instruction).** The 09-07 solve it replaced had stopped fitting
  the rig.
  - **Why it was replaced.** The old `drone_cam_livox.yaml` (09-07) scored **du −12 px ≈ 1.1° of
    bearing** (≈ 8 cm lateral on a person at 4 m). It shows on:
    - the 09-11 22:52 8-viewpoint motors-off capture;
    - `run5` (09-09);
    - a 60 s stationary bag from 2026-09-13 (`~/dtp_datasets/20260913_calib_check`).

    It still passed its own 09-07 capture, so the geometry moved after 09-07.
  - **What is deployed.** `~/calib_runs/20260911_2252_bench/T_camera_lidar.txt`, written with
    `write_calib_yaml` (camera `CPC64630007M` and LiDAR `192.168.1.154` identity gates armed).
    - Its own capture: **du +1, dv −1, PASS**.
    - Today's bag: **du +2** (old: −12). dv reads −6 at 4 % contrast, but dv has no peak in that
      single desk viewpoint for either file.
    - Old file backed up at `~/backups/calib-pre-0911-deploy-20260913/`; it is also in git history.
  - **Verified live, observe-only bring-up** (`~/dtp_datasets/20260913_calib_deploy_check/`):
    - `static_calib_tf` and `perception_node` loaded it; camera and LiDAR identity verified;
    - LiDAR→body matches FAST-LIO;
    - `lidar_imu → camera_link` = (+0.084, +0.008, +0.067) m;
    - `perception_ok` and `sensors_ok` True;
    - `test_write_calib_identity` + `test_static_calib_tf`: 18 passed.
  - **⚠️ Two things still open:**
    - **The deployed translation is physically WRONG (user, 2026-09-13: the LiDAR is ABOVE the
      camera and a little behind).**
      - **In the MID360 frame:** the 09-07 solve put the LiDAR 4.2 cm above / 7.6 cm behind the
        camera (matches); the deployed 09-11 solve puts it 2.3 cm **below** / 9.4 cm behind.
      - **Kept anyway, deliberately — the images only fit the 09-11 R+t as a pair:**
        - 09-11 rotation + 09-07 translation scored WORSE on the 8-viewpoint capture (du −4,
          dv no peak, against +1/−1);
        - a pitch/yaw re-fit (±2°) with the physical translation fixed found no both-axes peak.
        - The solver's pitch absorbed the offset at capture range (~4 m).
      - **Cost of keeping it (analytic estimate):** ≲ ±3 cm in lifted target positions over
        2–6 m. The old file was ~1.1° of bearing, ≈ 8 cm at 4 m.
      - **The real fix:** a tier-2 re-capture with structure at 1–2 m as well as far (near range
        separates rotation from translation), and/or a solve with the translation fixed to a
        tape-measured offset.
    - **Nothing records who solved it, or why it was not deployed on 09-11.**
    - **Motor-on validation is still owed** (M5.4b, from the Stage A bag).
  - **How it was measured:** the stock `score_alignment` sweep stops at ±6 px and reads "−6, FAIL";
    a wider sweep shows a clear peak at −12.
  - **The tool:** `drone-dtp/scripts/bag_to_calib_pairs.py` (validated: its FAST-LIO path matches a
    raw-`/livox/lidar` control).
- **⚠️ 2026-09-30: the camera and the LiDAR disagree by ~3° in pitch (2.3–3.5°), in every
  flight bag, motors off too. Fixed the same day by the table re-calibration (first entry): it
  moved 3.62° in pitch.** Found in the Part B pre-build measurements
  (`docs/evidence/lidar-range-2026-09-30/README.md`, "Frame chain (4)"; the build record is
  `dtp-follow-behaviour.md`, "Part B: LiDAR ranging past 10 m").
  - **The evidence:**
    - `overlay_check.py`: the LiDAR's points on the walker project **20–45 px too high** in the
      colour image.
    - The camera-lifted hip (patch) height above the LiDAR ground **falls 0.041–0.060 m per
      metre of range**, in all five flight bags (09-21 ×2, 09-26 ×2, 09-29), **including on the
      ground with motors off.**
    - Both point to the same 2.3–3.5° pitch disagreement.
  - **It contradicts the belief that the deployed file's error is ~1° or less** (Part B's plan,
    Step 1). The file's own capture scored dv −1, but dv had no peak on the desk viewpoint (the
    entry above).
  - **A widened `score_alignment` sweep was inconclusive** (`dv_sweep.py`, ±60 px on single
    outdoor frames: the peaks are scattered on both axes). It is evidence neither way.
  - **What it costs:**
    - **The camera-lifted target z is ~0.05·r too low** (~0.5 m at 10 m). Part A's 3.0 m veto
      band was sized on the flown tracked z (0.26–0.72 m on 09-29), which already carries it at
      those ranges; the band's top sits ~0.05·r lower than a true point would put it.
    - **A LiDAR box mask needs ground rejection.** With this error the ground in front of the
      walker enters the box. The nearest-surface rule is then OK in only 24–53 % of windows;
      dropping points < 0.25 m above the local ground restores 79–81 %. `lidar_box_range.py`
      does that.
  - **Still owed:** M5.4b motor-on validation. Whether the deployed translation error above plays
    a part is not tested.
  - **2026-09-30 14:55, a third, independent measurement: the error is in the camera's
    calibrated rotation, ~2.5°, and the camera really looks level with the LiDAR.** Bench bag
    `bags/2026-09-30_145506`, aircraft on a lab table, motors off. Scripts:
    `docs/evidence/lidar-range-2026-09-30/bench-1455/` (`floor_cam.py`, `att.py`).
    - **Floor plane from the depth camera, in the camera's own frame** (RANSAC on the bottom of 22
      depth frames): camera 0.897 m above the floor (p10–p90 0.878–0.915), optical axis
      **2.67° above** the floor plane.
    - **PX4's attitude (gravity):** pitch **+2.67° nose-up**, roll −0.13°. With the 09-02 LiDAR↔FC
      mount tilt (pitch +0.09°), the LiDAR's x axis is also ~2.7° above level. So the camera axis
      is parallel to the LiDAR's x axis (to ~0.3°).
    - **The deployed chain** (`lidar_imu → camera_link → … → optical`) puts the camera axis
      **2.54° below** the LiDAR's x axis. So the chain is ~2.5° nose-down too far, which
      matches the 2.3–3.5° above.
    - **Check:** with the chain's rotation, the camera-measured floor lands 5.2° from level in
      `lidar_init`, 0.4 m below the LiDAR's own sparse floor returns at 8–9 m. With PX4's 2.67°
      it passes through 2152 of them (±6 cm), and the LiDAR's walker head-top then reads a flat
      1.54–1.79 m above it from 2 to 9 m.
    - **What a fix would buy (computed on this bag):** the camera-lifted walker point sits 0.44 m
      above the floor at 2.3 m and 0.28 m at 8.5 m. Adding r·tan 2.5° makes it flat, and puts
      the dtp_viz cylinder's bottom within ±0.05 m of the floor at 1–9 m. It would also remove
      the reason for `lidar_box_top_extend_px` 45 and most of the ground-in-box problem.
    - **Not applied: the user's call.** It changes every camera-lifted z, and so the veto
      band's reference. The re-solve or a pitch-only correction, then the gates and a Stage A
      look, are M5.4b.
    - **Two things this relies on:** the floor is level, and PX4's pitch is the LiDAR's tilt. A
      LiDAR-box head-top projection on the same bag (`headproj.py`) gives only +0.5° median.
      But the LiDAR's top return sits below the true head top, so that is a lower bound.
### Ported 2026-10-01 — the 09-30 table calibration is deployed again on the rollback (user; motors off, NOT flown)
`drone_cam_livox.yaml` is the checkpoint's 2026-09-30 table-ICP solve. The rollback had put back
the 09-13 deploy (3.6° off in pitch). The install links to the source, so relaunching the follower
picks it up. `lidar_box_top_extend_px` (the old pitch workaround) is Part B only and is not on
this branch. M5.4b is still owed.

