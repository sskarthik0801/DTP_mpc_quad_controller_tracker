"""import_koide3 -- convert direct_visual_lidar_calibration's calib.json into our extrinsic.

Bridges the pluggable SOLVE stage back into our workflow (see calibration/__init__.py).

WHY THIS EXISTS RATHER THAN A COPY-PASTE: koide3 reports **T_lidar_camera**, and everything
downstream of us (fusion_node, validate_calib) wants **T_camera_lidar** -- the inverse. Hand-
copying that matrix silently produces a mirrored extrinsic that still projects plausibly
in-view, so it survives the cheap checks and fails as a wrong RANGE on a human. Doing the
inversion in code, once, with the direction printed, removes the chance to get it backwards.

Its value order is translation-first with an XYZW quaternion:
    results.T_lidar_camera = [tx, ty, tz, qx, qy, qz, qw]

Usage:
    python3 -m dtp_perception.calibration.import_koide3 \
        --calib_json ~/calib_out/calib.json \
        --out ~/calib_runs/20260823_1800/T_camera_lidar.txt
"""
import argparse
import json
import os
import sys

import numpy as np


def quat_xyzw_to_R(qx, qy, qz, qw):
    """Quaternion -> 3x3 rotation. Written out rather than pulled from scipy so this stays
    dependency-free on the Jetson, where the solver itself is deliberately absent."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        raise ValueError("degenerate quaternion in calib.json")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ], dtype=float)


def find_transform(doc):
    """Tolerate both {'results': {...}} and a flattened layout across upstream versions."""
    for holder in (doc.get('results', {}), doc):
        if isinstance(holder, dict) and 'T_lidar_camera' in holder:
            v = holder['T_lidar_camera']
            if len(v) != 7:
                raise ValueError("expected 7 values [tx ty tz qx qy qz qw], got %d" % len(v))
            return [float(x) for x in v]
    raise KeyError("no 'T_lidar_camera' in calib.json -- did `calibrate` finish?")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib_json', required=True)
    ap.add_argument('--out', required=True, help='4x4 T_camera_lidar text file')
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    doc = json.load(open(os.path.expanduser(a.calib_json)))
    tx, ty, tz, qx, qy, qz, qw = find_transform(doc)

    T_lidar_camera = np.eye(4)
    T_lidar_camera[:3, :3] = quat_xyzw_to_R(qx, qy, qz, qw)
    T_lidar_camera[:3, 3] = [tx, ty, tz]

    # THE inversion. Orthonormal rotation, so transpose is the inverse -- but use the general
    # form so a non-rigid matrix would surface as a numerical problem rather than silently pass.
    T_camera_lidar = np.linalg.inv(T_lidar_camera)

    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    with open(out, 'w') as f:
        f.write("\n".join(" ".join("%.9f" % v for v in row) for row in T_camera_lidar) + "\n")

    np.set_printoptions(precision=5, suppress=True)
    print("koide3 reported T_lidar_camera (LiDAR <- camera):")
    print(T_lidar_camera)
    print("")
    print("INVERTED to T_camera_lidar (camera <- LiDAR)  <-- what we use:")
    print(T_camera_lidar)
    print("")
    print("baseline (camera origin in LiDAR frame): %.3f m" % np.linalg.norm(T_lidar_camera[:3, 3]))
    print("wrote %s" % out)
    print("")
    print("NEXT -- do not skip: validate before trusting this.")
    print("  ros2 run dtp_perception validate_calib --run_dir <run> --calib_file %s" % out)
    print("Check the overlay PNGs: LiDAR depth edges must land on image edges. A mirrored")
    print("extrinsic still projects in-view, so in-view % alone will NOT catch it.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
