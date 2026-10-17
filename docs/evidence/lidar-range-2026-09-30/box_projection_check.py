#!/usr/bin/env python3
"""Step 3 diagnostic: where do the walker's own LiDAR points land relative to the flown detector's box?

    python3 box_projection_check.py STEP0_DIR BAG_NAME [T0 T1 ...]     (bag-second spans, pairs)
    TOP_EXTEND_PX=45 python3 box_projection_check.py ...   (the box top raised, as the hybrid's
                                                            lidar_box_top_extend_px does)

Uses detector_range_study.py's caches only (tele.pkl, lidar.pkl, walker.pkl, dets.pkl in
STEP0_DIR/BAG_NAME). Per in-FOV colour frame with a matched YOLO box and the walker 8-20 m away:
the walker's DYNAMIC LiDAR points (within 0.6 m of the LiDAR-cluster truth, scans within 0.25 s)
are projected through the bag's own /tf_static chain and colour intrinsics -- the chain
perception_node's TF lift uses, which the hybrid's pose cross-check found identical to its own --
and compared with the box shrunk 10 % per edge (lidar_box_range.BOX_SHRINK_FRAC).
Prints, per range bin: points per frame, the share inside the shrunk box, the share ABOVE it, and
the median point's height below the box top as a fraction of the box height.
No motion compensation: the walker's horizontal smear does not move a point vertically.
TOP_EXTEND_PX (added 2026-09-30, Step 3 second round): the shrunk box's TOP edge is raised by that
many pixels before the test (lidar_box_range.in_box's top_extend_px); sides and bottom unchanged.
CPU only, read-only, a few seconds.
"""
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detector_range_study as drs      # noqa: E402


def main():
    ext = float(os.environ.get('TOP_EXTEND_PX', '0') or 0.0)
    step0, bag = sys.argv[1], sys.argv[2]
    sp = [float(x) for x in sys.argv[3:]] or [440.0, 453.0, 486.0, 506.0]
    spans = list(zip(sp[0::2], sp[1::2]))
    d = os.path.join(step0, bag)
    D = pickle.load(open(os.path.join(d, 'tele.pkl'), 'rb'))
    L = pickle.load(open(os.path.join(d, 'lidar.pkl'), 'rb'))
    W = pickle.load(open(os.path.join(d, 'walker.pkl'), 'rb'))
    dets = pickle.load(open(os.path.join(d, 'dets.pkl'), 'rb'))
    geo = drs.Geo(D, L)
    fx = W['fixes']
    fst = [f[0] for f in fx]
    scans = L['scans']
    sst = np.array([s[1] for s in scans])
    fr = D['frames']
    res = []
    for i in sorted(dets):
        st, boxes, _trk, _label = dets[i]
        tb = fr[i, 1]
        if boxes is None or not any(a <= tb <= b for a, b in spans):
            continue
        tr = drs.truth_at(fst, fx, st)
        if tr is None:
            continue
        Tc, p, q = geo.cam(st)
        row = drs.person_row(tr, Tc, p, q, geo, boxes, None)
        if row.get('fov') != 'in' or not row.get('det') or row['range_m'] < 8.0:
            continue
        b = [float(x) for x in row['box'].split()]
        bh, bw = b[3] - b[1], b[2] - b[0]
        sb = (b[0] + 0.1 * bw, b[1] + 0.1 * bh - ext, b[2] - 0.1 * bw, b[3] - 0.1 * bh)
        j = int(np.searchsorted(sst, st))
        us, vs = [], []
        for k in range(max(0, j - 3), min(len(scans), j + 3)):
            if abs(sst[k] - st) > 0.25 or len(scans[k][3]) == 0:
                continue
            pts = scans[k][3]                       # (x, y, height above the local ground)
            for P in pts[np.hypot(pts[:, 0] - tr[0], pts[:, 1] - tr[1]) < 0.6]:
                uv, _ = geo.project(Tc, (P[0], P[1], P[2] + geo.ground(P[0], P[1])))
                if uv:
                    us.append(uv[0])
                    vs.append(uv[1])
        if not vs:
            continue
        us, vs = np.array(us), np.array(vs)
        inside = float(np.mean((vs >= sb[1]) & (vs <= sb[3]) & (us >= sb[0]) & (us <= sb[2])))
        res.append((tb, row['range_m'], len(vs), inside, float(np.mean(vs < sb[1])),
                    float(np.median((vs - b[1]) / bh)), bh))
    res = np.array(res)
    print('%s, spans %s: %d frames; box top raised %.0f px' % (bag, spans, len(res), ext))
    for lo, hi in [(8, 10), (10, 12), (12, 15), (15, 20)]:
        m = (res[:, 1] >= lo) & (res[:, 1] < hi)
        if m.sum():
            print('  %2d-%2d m  frames %3d  walker points/frame %5.1f  inside shrunk box %3.0f %%  ABOVE it %3.0f %%'
                  '  median point %+.2f box heights below the top  (box %3.0f px)' % (
                      lo, hi, m.sum(), res[m, 2].mean(), 100 * res[m, 3].mean(), 100 * res[m, 4].mean(),
                      float(np.median(res[m, 5])), float(np.median(res[m, 6]))))
    return 0 if len(res) else 1


if __name__ == '__main__':
    sys.exit(main())
