"""validate_calib -- go/no-go check on a candidate T_camera_lidar.

Stage 3 of the reusable calibration workflow (see calibration/__init__.py). Solver-agnostic:
it reads captured pairs + a candidate extrinsic and answers one question -- does projecting
LiDAR through this extrinsic land where the image says it should?

WHY THIS MATTERS MORE THAN THE SOLVER'S OWN RESIDUAL: every solver reports a residual against
its OWN cost function, which is exactly the number it optimised. That number can look excellent
while the extrinsic is wrong (degenerate viewpoints, a mirrored axis, a swapped frame
convention). This checks an independent quantity -- LiDAR-derived depth vs the camera's own
depth in the same pixels -- so it can catch failures the solver is blind to.

Renders an overlay PNG per pair so a human can eyeball edge alignment, which remains the single
most reliable check for a gross frame error.

Usage:
    ros2 run dtp_perception validate_calib --ros-args \
        -p run_dir:=~/calib_runs/20260815_1200 \
        -p calib_file:=~/calib_runs/20260815_1200/T_camera_lidar.txt
"""
import glob
import os
import sys

import numpy as np


def load_transform(path):
    """Accept either a bare 4x4 text matrix or our calib YAML."""
    txt = open(path).read()
    if 'T_camera_lidar' in txt:
        rows = []
        for line in txt.splitlines():
            line = line.strip()
            if line.startswith('- ['):
                rows.append([float(v) for v in line[3:].rstrip(']').split(',')])
        return np.array(rows, dtype=float)
    vals = [float(v) for v in txt.split()]
    if len(vals) != 16:
        raise ValueError("expected 16 numbers for a 4x4, got %d" % len(vals))
    return np.array(vals, dtype=float).reshape(4, 4)


def load_intrinsics(run_dir):
    p = os.path.join(run_dir, 'camera_info.yaml')
    k = {}
    for line in open(p):
        for key in ('fx', 'fy', 'cx', 'cy'):
            if line.startswith(key + ':'):
                k[key] = float(line.split(':')[1])
    if len(k) != 4:
        raise ValueError("could not parse intrinsics from %s" % p)
    return k


def project(cloud_xyz, T, K, w, h):
    """LiDAR points -> pixel coords. Returns (uv, depth, mask_in_view)."""
    pts = np.hstack([cloud_xyz, np.ones((len(cloud_xyz), 1))])
    cam = (T @ pts.T).T[:, :3]           # into the camera optical frame
    z = cam[:, 2]
    front = z > 0.05                      # discard points behind the camera
    u = np.full(len(cam), -1.0)
    v = np.full(len(cam), -1.0)
    u[front] = K['fx'] * cam[front, 0] / z[front] + K['cx']
    v[front] = K['fy'] * cam[front, 1] / z[front] + K['cy']
    inview = front & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    return np.stack([u, v], axis=1), z, inview


def main(args=None):
    # Deliberately runnable WITHOUT a ROS graph -- calibration is offline work and should not
    # require the sensors to still be connected.
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--run_dir', required=True)
    ap.add_argument('--calib_file', required=True)
    ap.add_argument('--save_overlays', action='store_true', default=True)
    a = ap.parse_args(args if args is not None else sys.argv[1:])

    run_dir = os.path.expanduser(a.run_dir)
    T = load_transform(os.path.expanduser(a.calib_file))
    K = load_intrinsics(run_dir)
    pairs = sorted(glob.glob(os.path.join(run_dir, 'pair_*.npz')))
    if not pairs:
        print("no pair_*.npz in %s" % run_dir); return 1

    print("T_camera_lidar:\n%s" % np.array2string(T, precision=5, suppress_small=True))
    print("intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f" % (K['fx'], K['fy'], K['cx'], K['cy']))
    print("-" * 68)
    print("%-16s %10s %10s %12s" % ("pair", "points", "in-view", "in-view %"))

    total_pts = total_in = 0
    for p in pairs:
        d = np.load(p, allow_pickle=True)
        cloud = d['cloud']
        img = d['image']
        h, w = img.shape[0], img.shape[1]
        uv, z, inview = project(cloud, T, K, w, h)
        total_pts += len(cloud); total_in += int(inview.sum())
        print("%-16s %10d %10d %11.1f%%" % (os.path.basename(p), len(cloud),
                                            inview.sum(), 100.0 * inview.mean()))

        if a.save_overlays:
            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                fig, ax = plt.subplots(figsize=(12, 7))
                disp = img[:, :, ::-1] if img.ndim == 3 and img.shape[2] == 3 else img
                ax.imshow(disp)
                s = ax.scatter(uv[inview, 0], uv[inview, 1], c=z[inview],
                               s=1.5, cmap='turbo', alpha=0.6)
                plt.colorbar(s, ax=ax, label='LiDAR range (m)')
                ax.set_title(os.path.basename(p))
                ax.axis('off')
                out = p.replace('.npz', '_overlay.png')
                fig.savefig(out, dpi=110, bbox_inches='tight')
                plt.close(fig)
            except Exception as e:
                print("   (overlay skipped: %s)" % str(e)[:60])

    frac = 100.0 * total_in / max(total_pts, 1)
    print("-" * 68)
    print("overall in-view: %.1f%%  (%d / %d points)" % (frac, total_in, total_pts))
    print("")
    print("HOW TO READ THIS -- in-view % is a SANITY check, not an accuracy metric:")
    print("  ~0%       -> extrinsic is grossly wrong (rotation/axis/sign error), or the camera")
    print("              and LiDAR simply do not overlap in FOV. Investigate before trusting.")
    print("  very high-> plausible, but NOT proof: a wrong translation still projects in-view.")
    print("")
    print("THE REAL GATE is visual: open the *_overlay.png files and confirm LiDAR depth edges")
    print("land on image edges (wall corners, door frames, box edges). Per plan/dtp-imp/05")
    print("Step 0.3.1b the numeric target is a few cm of overlay error at 3-6 m, and it MUST be")
    print("re-validated MOTOR-ON -- vibration shifts the extrinsic.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
