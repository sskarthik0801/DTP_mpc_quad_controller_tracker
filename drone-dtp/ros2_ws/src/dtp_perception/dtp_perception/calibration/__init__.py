"""Reusable camera<->LiDAR calibration tooling.

WHY THIS EXISTS AS A MODULE, not a one-off procedure: the sensor mounts change with every
airframe revision, and each change invalidates T_camera_lidar. A calibration you cannot
cheaply repeat is a calibration that silently goes stale -- and a stale extrinsic does not
announce itself, it just puts the range frustum on a tree trunk instead of the person
(plan/dtp-imp/14 A8, risk R-F12).

Design rule: CAPTURE and VALIDATE are ours and solver-agnostic; SOLVE is pluggable.
That way swapping the solver (direct_visual_lidar_calibration, a checkerboard method, or a
manual refinement) does not cost us the surrounding workflow.

Stages
------
1. capture_calib_data  -- record synchronised image + point-cloud pairs from live sensors
2. <solve>             -- external solver produces a 4x4 T_camera_lidar
3. validate_calib      -- project LiDAR into the image with the candidate extrinsic and
                          report reprojection/depth residuals; this is the go/no-go gate

Full runbook: plan/dtp-imp/19-camera-lidar-calibration-runbook.md
"""
