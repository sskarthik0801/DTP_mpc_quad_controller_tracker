"""Compare two extrinsics and decide whether the difference MATTERS.

WHY THIS EXISTS
---------------
plan/dtp-imp/05 Step 0.3.1b requires re-validating the calibration with **motors running**,
and the runbook says to "compare overlay residuals against the motors-off run". Nothing
actually did that comparison, and "a material change" was never defined -- so the gate could
not be decided, only eyeballed.

The problem with eyeballing two extrinsics is that raw numbers are not commensurable. A 4 mm
translation and a 0.3 deg rotation are not obviously comparable, yet at 5 m the rotation is
~4x worse. So this reduces both to ONE operational number: **how far, in pixels, does the
same LiDAR point land differently** under the two extrinsics.

Pixels are the right unit because that is what the perception stack consumes -- a LiDAR
return is associated with a detection box by where it projects. It is also directly
comparable to `score_alignment`, whose PASS band is +/-1 px on this rig.

TWO MODES
---------
* EMPIRICAL (preferred) -- give it a capture run. Every real point is projected under both
  extrinsics and the actual displacement distribution is measured. No approximations.
* ANALYTIC (fallback) -- with no capture data, estimate the shift at nominal ranges from the
  rotation angle and the perpendicular translation. Small-angle, and it ignores where points
  actually are, so it is a sanity estimate and says so.

WHAT VIBRATION LOOKS LIKE
-------------------------
A mount that flexes under motor load usually shows up as ROTATION, not translation, and
rotation is the dangerous one: its pixel error is range-INDEPENDENT, so it does not shrink as
the target gets closer. A pure translation error shrinks as 1/range. The report separates the
two contributions so the failure mode is legible, not just the total.

Usage:
    ros2 run dtp_perception compare_calib \\
        --baseline <motors-off T_camera_lidar.txt or calib yaml> \\
        --test     <motors-on  T_camera_lidar.txt or calib yaml> \\
        [--run_dir <capture run with pair_*.npz>]      # enables empirical mode
"""
import argparse
import glob
import os
import sys

import numpy as np

# Thresholds in pixels, tied to the precision we have actually demonstrated on this rig:
# score_alignment resolves the extrinsic to +/-1 px, so a shift at or below 1 px is
# indistinguishable from simply re-solving the same rig twice.
PASS_PX = 1.0
# At f~611, 3 px is ~25 mm at 5 m -- the outer edge of plan 05's "a few cm at 3-6 m".
MARGINAL_PX = 3.0

# Ranges used for the analytic fallback and for the per-range breakdown.
REPORT_RANGES_M = (3.0, 5.0, 10.0)


def _load_transform(path):
    """Accept either a bare 4x4 matrix file or a calib YAML, and say which was read."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        raise SystemExit("ERROR: no such file: %s" % path)
    if path.endswith(('.yaml', '.yml')):
        import yaml
        with open(path) as f:
            d = yaml.safe_load(f)
        if 'T_camera_lidar' not in d:
            raise SystemExit("ERROR: %s has no T_camera_lidar key" % path)
        T = np.array(d['T_camera_lidar'], float)
        note = d.get('note')
        if note is None:  # write_calib_yaml puts provenance in comments, not keys
            note = _grep_note(path)
        return T, note
    return np.loadtxt(path), _grep_note(path)


def _grep_note(path):
    """Pull the '# note' provenance line if the file carries one."""
    try:
        with open(path) as f:
            for line in f:
                if line.startswith('# note'):
                    return line.split(':', 1)[-1].strip()
                if not line.startswith('#'):
                    break
    except OSError:
        pass
    return None


def _rotation_angle_deg(Ra, Rb):
    """Angle of the relative rotation Ra^T Rb, in degrees."""
    R = Ra.T @ Rb
    # clip guards against acos domain error from accumulated float noise
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def _load_intrinsics(run_dir, fallback_yaml=None):
    import yaml
    p = os.path.join(run_dir, 'camera_info.yaml') if run_dir else None
    if p and os.path.exists(p):
        with open(p) as f:
            ci = yaml.safe_load(f)
        return ci['fx'], ci['fy'], ci['cx'], ci['cy']
    if fallback_yaml and os.path.exists(fallback_yaml):
        with open(fallback_yaml) as f:
            d = yaml.safe_load(f)
        i = d.get('intrinsics')
        if i:
            return i['fx'], i['fy'], i['cx'], i['cy']
    return None


def _project(T, pts, intr):
    """Project ALL points; return u, v, z UNFILTERED and index-aligned with `pts`.

    Deliberately does no z-filtering. Filtering inside here would return different-length
    arrays for the two extrinsics, and pairing them up by position would then compare
    DIFFERENT physical points -- a silent wrong answer, worst near the z cutoff, which is
    exactly where a marginal extrinsic difference shows up. The caller masks once, on the
    intersection, so both arrays stay aligned to the same points.
    """
    fx, fy, cx, cy = intr
    cam = (T[:3, :3] @ pts.T).T + T[:3, 3]
    z = cam[:, 2]
    safe = np.where(np.abs(z) < 1e-6, 1e-6, z)      # avoid divide-by-zero; masked off later
    u = fx * cam[:, 0] / safe + cx
    v = fy * cam[:, 1] / safe + cy
    return u, v, z


def _empirical(A, B, run_dir, intr):
    """Actual per-point pixel displacement between the two extrinsics."""
    pairs = sorted(glob.glob(os.path.join(run_dir, 'pair_*.npz')))
    if not pairs:
        return None
    shifts, ranges = [], []
    for p in pairs:
        d = np.load(p)
        pts = d['cloud'][:, :3].astype(float)
        ua, va, za = _project(A, pts, intr)
        ub, vb, zb = _project(B, pts, intr)
        # Comparable only where the point is in front of the camera under BOTH extrinsics.
        both = (za > 0.3) & (zb > 0.3)
        if both.sum() < 100:
            continue
        shifts.append(np.hypot(ua[both] - ub[both], va[both] - vb[both]))
        ranges.append(np.linalg.norm(pts[both], axis=1))
    if not shifts:
        return None
    return np.concatenate(shifts), np.concatenate(ranges)


def _verdict(px):
    if px <= PASS_PX:
        return 'PASS', 'within the +/-%.0f px the solver itself resolves - indistinguishable from re-solve noise' % PASS_PX
    if px <= MARGINAL_PX:
        return 'MARGINAL', 'the mount moved measurably but stays inside plan 05\'s "few cm at 3-6 m"'
    return 'FAIL', 'exceeds the calibration tolerance - STIFFEN THE MOUNT, do not just re-solve'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--baseline', required=True, help='motors-off extrinsic (4x4 txt or calib yaml)')
    ap.add_argument('--test', required=True, help='motors-on extrinsic (4x4 txt or calib yaml)')
    ap.add_argument('--run_dir', default=None,
                    help='capture run with pair_*.npz -> enables exact empirical comparison')
    args = ap.parse_args(argv)

    A, note_a = _load_transform(args.baseline)
    B, note_b = _load_transform(args.test)
    for T, n in ((A, args.baseline), (B, args.test)):
        if T.shape != (4, 4):
            raise SystemExit("ERROR: %s is not a 4x4 transform (got %s)" % (n, T.shape))

    print("baseline : %s" % args.baseline)
    if note_a:
        print("           %s" % note_a)
    print("test     : %s" % args.test)
    if note_b:
        print("           %s" % note_b)

    # --- raw geometric difference ------------------------------------------
    dt = B[:3, 3] - A[:3, 3]
    dt_mm = dt * 1000.0
    dtheta = _rotation_angle_deg(A[:3, :3], B[:3, :3])
    print("\n--- geometric difference ---")
    print("  translation : dx=%+.1f  dy=%+.1f  dz=%+.1f mm   (norm %.1f mm)"
          % (dt_mm[0], dt_mm[1], dt_mm[2], np.linalg.norm(dt_mm)))
    print("  rotation    : %.4f deg" % dtheta)

    run_dir = os.path.expanduser(args.run_dir) if args.run_dir else None
    intr = _load_intrinsics(run_dir, args.test if args.test.endswith(('.yaml', '.yml')) else None)
    if intr is None:
        intr = _load_intrinsics(run_dir, args.baseline if args.baseline.endswith(('.yaml', '.yml')) else None)
    if intr is None:
        print("\nERROR: no intrinsics found. Pass --run_dir (needs camera_info.yaml), or point")
        print("--baseline/--test at a calib YAML that carries an `intrinsics:` block.",
              file=sys.stderr)
        return 2
    fx, fy, cx, cy = intr
    print("  intrinsics  : fx=%.2f fy=%.2f" % (fx, fy))

    # --- which error source dominates --------------------------------------
    # Rotation shift is range-independent; translation shift falls off as 1/range.
    rot_px = fx * np.tan(np.radians(dtheta))
    perp = float(np.linalg.norm(dt[:2]))       # component perpendicular to the optical axis
    print("\n--- contribution breakdown (analytic) ---")
    print("  rotation      -> %6.2f px at EVERY range (does not shrink as the target nears)" % rot_px)
    for r in REPORT_RANGES_M:
        print("  translation   -> %6.2f px at %4.1f m" % (fx * perp / r, r))

    # --- the operational number --------------------------------------------
    emp = _empirical(A, B, run_dir, intr) if run_dir else None
    if emp is not None:
        shift, rng = emp
        p50, p95, mx = np.percentile(shift, 50), np.percentile(shift, 95), shift.max()
        print("\n--- EMPIRICAL reprojection shift (%d real points) ---" % len(shift))
        print("  median %.2f px | p95 %.2f px | max %.2f px" % (p50, p95, mx))
        for lo, hi in ((0, 3), (3, 6), (6, 12)):
            m = (rng >= lo) & (rng < hi)
            if m.sum() > 50:
                print("    %2d-%2dm : median %5.2f px  p95 %5.2f px  (%d pts)"
                      % (lo, hi, np.percentile(shift[m], 50), np.percentile(shift[m], 95), m.sum()))
        decide_px, basis = p95, 'p95 of the empirical shift'
    else:
        if run_dir:
            print("\n  (no usable pair_*.npz in %s -- falling back to analytic)" % run_dir)
        worst = rot_px + fx * perp / min(REPORT_RANGES_M)
        print("\n--- ANALYTIC estimate (no capture data) ---")
        print("  worst-case %.2f px at %.1f m" % (worst, min(REPORT_RANGES_M)))
        print("  This ignores where points actually are. Pass --run_dir for the exact number.")
        decide_px, basis = worst, 'analytic worst case at %.1f m' % min(REPORT_RANGES_M)

    v, why = _verdict(decide_px)
    print("\n--- VERDICT: %s ---" % v)
    print("  %.2f px (%s)" % (decide_px, basis))
    print("  %s" % why)
    if v != 'PASS' and rot_px > fx * perp / 5.0:
        print("\n  NOTE: the difference is ROTATION-dominated, which is the signature of a mount")
        print("  flexing under load. Rotation error does not shrink with range, so it will not")
        print("  improve as the drone closes on a person. Stiffen the mount.")
    return 0 if v == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
