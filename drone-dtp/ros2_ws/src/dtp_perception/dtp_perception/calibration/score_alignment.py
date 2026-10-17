"""Quantitative gate on a solved extrinsic: does edge alignment PEAK at zero pixel offset?

WHY THIS EXISTS
---------------
`validate_calib` reports in-view % and draws overlays. Neither is an accuracy metric:

  * in-view % cannot see a wrong translation -- a mirrored or shifted extrinsic still
    projects plausibly in-view, and on this rig the geometric ceiling is only ~22% anyway
    (Mid-360 is 360x59 deg, the Gemini colour FOV is ~86x55 deg), so "low" is normal here.
  * the overlay PNGs are downscaled and judged by eye, which cannot separate a good solve
    from one biased by a few pixels.
  * solver residuals measure the quantity the solver itself optimised, so they cannot be an
    independent check on that same solve.

This module is independent of all three. It projects the cloud with the final
T_camera_lidar, finds LiDAR depth discontinuities, and scores them against image intensity
gradients -- then RE-SCORES at deliberately wrong pixel offsets. If the extrinsic is right,
the score must be maximal at (0, 0). If it peaks elsewhere, the extrinsic is biased by that
many pixels and the sign tells you which way.

THE ORIENTATION TRAP (this produced a wrong verdict on the 2026-09-07 run)
--------------------------------------------------------------------------
Scoring every depth edge against the TOTAL image gradient makes the vertical axis look
almost flat, and a naive argmax then reports a bogus multi-pixel bias from what is really
noise. The cause is geometric, not numerical: an indoor scene's depth edges are
overwhelmingly VERTICAL structure (walls, panels, door frames, pillars), and a vertical
edge slid vertically still lands on itself -- it carries essentially no dv information.

So the axes are decomposed by edge orientation:

  * range gradient mostly horizontal -> vertical structure   -> constrains du, scored on |dI/dx|
  * range gradient mostly vertical   -> horizontal structure -> constrains dv, scored on |dI/dy|

Each axis is swept using only the edges that carry information about it. Read CONTRAST
alongside the peak: a peak location is only meaningful if the sweep has enough contrast to
distinguish offsets at all. Below MIN_CONTRAST the axis is under-determined by the captured
scene (add viewpoints with that edge orientation) rather than wrong.

Usage:
    ros2 run dtp_perception score_alignment --run_dir ~/calib_runs/<stamp>
    # or: python3 -m dtp_perception.calibration.score_alignment --run_dir ...
"""
import argparse
import glob
import os
import sys

import numpy as np

# Sub-pixel peaks are a PASS: at fx~611 one pixel is ~8 mm of lateral error at 5 m, well
# inside the "few cm at 3-6 m" target in plan/dtp-imp/05 Step 0.3.1b.
PEAK_TOL_PX = 1.0
# Below this, the sweep cannot distinguish offsets and the peak location means nothing.
MIN_CONTRAST = 0.02
# A range spread this large within the window counts as a depth discontinuity.
STEP_M = 0.30
# How strongly a range gradient must favour one axis before the edge is assigned to it.
ORIENT_RATIO = 2.0

SHIFTS = [-6, -4, -3, -2, -1, 0, 1, 2, 3, 4, 6]


def _load_intrinsics(run_dir):
    import yaml
    with open(os.path.join(run_dir, 'camera_info.yaml')) as f:
        ci = yaml.safe_load(f)
    return ci['fx'], ci['fy'], ci['cx'], ci['cy']


def _sweep_one_pair(npz_path, T, intr, ndi):
    """Return (du_scores, dv_scores, n_vert, n_horz) for one capture pair, or None."""
    fx, fy, cx, cy = intr
    d = np.load(npz_path)
    img = d['image']
    gray = img.mean(2).astype(float) if img.ndim == 3 else img.astype(float)
    H, W = gray.shape[:2]

    # Blur the gradients so the score varies smoothly over a few pixels; without this the
    # sweep is dominated by whether an edge happens to land on one bright pixel.
    gx = ndi.gaussian_filter(np.abs(ndi.sobel(gray, 1)), 2.0)
    gy = ndi.gaussian_filter(np.abs(ndi.sobel(gray, 0)), 2.0)
    gx /= gx.max() + 1e-9
    gy /= gy.max() + 1e-9

    pts = d['cloud'][:, :3].astype(float)
    cam = (T[:3, :3] @ pts.T).T + T[:3, 3]
    z = cam[:, 2]
    keep = z > 0.3
    cam, z = cam[keep], z[keep]
    if cam.shape[0] < 500:
        return None
    u = fx * cam[:, 0] / z + cx
    v = fy * cam[:, 1] / z + cy
    rng = np.linalg.norm(cam, axis=1)

    # Rasterise range into the image plane so neighbourhood statistics are cheap.
    grid = np.full((H, W), np.nan)
    ui, vi = u.astype(int), v.astype(int)
    ok = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
    grid[vi[ok], ui[ok]] = rng[ok]

    filled = np.nan_to_num(grid)
    mask = (~np.isnan(grid)).astype(float)
    # Coverage-normalised local variance: the cloud is sparse in the image plane, so a plain
    # box filter would read empty pixels as range 0 and call every gap a depth edge.
    s1 = ndi.uniform_filter(filled, 7)
    s2 = ndi.uniform_filter(filled ** 2, 7)
    cov = ndi.uniform_filter(mask, 7) + 1e-6
    step = np.sqrt(np.clip(s2 / cov - (s1 / cov) ** 2, 0.0, None))

    smooth = ndi.uniform_filter(filled, 5)
    rx = ndi.sobel(smooth, 1)
    ry = ndi.sobel(smooth, 0)

    is_edge = (~np.isnan(grid)) & (step > STEP_M)
    vert = is_edge & (np.abs(rx) > ORIENT_RATIO * np.abs(ry))
    horz = is_edge & (np.abs(ry) > ORIENT_RATIO * np.abs(rx))

    out = {}
    for key, sel, gimg, axis in (('du', vert, gx, 0), ('dv', horz, gy, 1)):
        ev, eu = np.nonzero(sel)
        if len(eu) < 100:
            out[key] = None
            continue
        scores = {}
        for s in SHIFTS:
            uu = eu + (s if axis == 0 else 0)
            vv = ev + (s if axis == 1 else 0)
            m = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
            scores[s] = float(gimg[vv[m], uu[m]].mean()) if m.any() else 0.0
        out[key] = scores
    return out['du'], out['dv'], int(vert.sum()), int(horz.sum())


def _report_axis(label, per_pair):
    """Print one swept axis. Returns True (pass), False (fail) or None (under-determined)."""
    if not per_pair:
        print("%s: not enough oriented edges in this capture -- axis UNDETERMINED." % label)
        print("   Add viewpoints containing that edge orientation and re-run.\n")
        return None
    vals = {s: float(np.mean([p[s] for p in per_pair])) for s in SHIFTS}
    peak = max(vals, key=vals.get)
    lo = min(vals.values())
    contrast = (vals[0] - lo) / (vals[0] + 1e-9)

    print("%s sweep  (%d pairs):" % (label, len(per_pair)))
    print("   " + "".join("%8d" % s for s in SHIFTS))
    print("   " + "".join("%8.4f" % vals[s] for s in SHIFTS))
    print("   peak %+d px | score(0)=%.4f | contrast %.1f%%" % (peak, vals[0], contrast * 100))

    if contrast < MIN_CONTRAST:
        print("   -> UNDETERMINED: contrast below %.0f%%, the sweep cannot distinguish offsets."
              % (MIN_CONTRAST * 100))
        print("      The peak location here is noise. Capture more structure on this axis.\n")
        return None
    if abs(peak) <= PEAK_TOL_PX:
        print("   -> PASS (within +/-%.0f px; ~%.0f mm lateral at 5 m)\n"
              % (PEAK_TOL_PX, PEAK_TOL_PX * 5000.0 / 611.0))
        return True
    print("   -> FAIL: biased %+d px. Re-solve; do NOT fly this extrinsic.\n" % peak)
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--run_dir', required=True, help='capture run directory (holds pair_*.npz)')
    ap.add_argument('--calib_file', default=None,
                    help='4x4 T_camera_lidar (default: <run_dir>/T_camera_lidar.txt)')
    args = ap.parse_args(argv)

    run_dir = os.path.expanduser(args.run_dir)
    calib = os.path.expanduser(args.calib_file) if args.calib_file \
        else os.path.join(run_dir, 'T_camera_lidar.txt')

    try:
        from scipy import ndimage as ndi
    except ImportError:
        print("ERROR: needs scipy.  pip3 install --user scipy", file=sys.stderr)
        return 2

    if not os.path.exists(calib):
        print("ERROR: no extrinsic at %s" % calib, file=sys.stderr)
        return 2
    T = np.loadtxt(calib)
    if T.shape != (4, 4):
        print("ERROR: expected a 4x4 transform, got %s" % (T.shape,), file=sys.stderr)
        return 2

    pairs = sorted(glob.glob(os.path.join(run_dir, 'pair_*.npz')))
    if not pairs:
        print("ERROR: no pair_*.npz in %s" % run_dir, file=sys.stderr)
        return 2
    intr = _load_intrinsics(run_dir)

    du_all, dv_all, nv, nh = [], [], 0, 0
    for p in pairs:
        r = _sweep_one_pair(p, T, intr, ndi)
        if r is None:
            continue
        du, dv, a, b = r
        if du:
            du_all.append(du)
        if dv:
            dv_all.append(dv)
        nv += a
        nh += b

    print("depth-edge vs image-edge alignment, swept over pixel offsets")
    print("edge pixels: vertical-structure %d (du), horizontal-structure %d (dv)\n" % (nv, nh))
    res = [_report_axis("du (horizontal)", du_all), _report_axis("dv (vertical)", dv_all)]

    if False in res:
        print("VERDICT: FAIL -- alignment does not peak at zero offset.")
        return 1
    if True not in res:
        print("VERDICT: INCONCLUSIVE -- neither axis was determined by this capture.")
        return 1
    if None in res:
        print("VERDICT: PARTIAL PASS -- one axis confirmed, the other under-determined by the")
        print("scene. Acceptable to proceed, but the unconfirmed axis is unverified.")
        return 0
    print("VERDICT: PASS -- both axes peak at zero offset.")
    print("NOTE: this is a BENCH result. plan/dtp-imp/05 Step 0.3.1b still requires re-validation")
    print("MOTOR-ON; vibration shifts the extrinsic and no static capture can predict it.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
