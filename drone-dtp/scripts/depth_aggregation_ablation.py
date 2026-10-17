#!/usr/bin/env python3
"""Depth-aggregation bake-off over a recorded bag: which statistic over the torso patch should
give the range to a person?

WHY THIS EXISTS, AND WHY IT IS RUN OFFLINE
------------------------------------------
perception_node replaced detection.py's plain median over the torso patch with a nearest-surface
rule (20th percentile + 0.45 m band, medians inside) on 2026-09-09, after a live run in which the
median flipped between the person and the wall behind them and tripped the range gate on 62 % of
frames. That was ONE alternative measured against ONE baseline on ONE run, and the live pipeline
can only run one rule at a time. The related-work pass also found that NOVA uses a histogram-mode
range (0.15 m bins), so "ours" is one of several reasonable rules, not a discovery. The honest
thing to publish is a comparison of all of them on identical patches -- which is what a recorded
bag with RAW depth allows and the live pipeline does not.

20260910_flightlegal_bench is the first bag carrying /camera/depth/image_raw (424x240 16UC1), so
it is the first on which this can be done at all.

WHAT IS HELD FIXED
------------------
Everything except the statistic. The depth -> colour registration, the range image, the organized
cloud and the patch GEOMETRY are the pipeline's own code paths (depth_source constants and
organized_cloud.build are imported, not re-derived; the registration maths is a line-for-line copy
of CameraDepthSource._points_in_colour_frame + DepthSource.range_image because those need a live
node). The detector is the same engine. No tracker: the bag has exactly one person and zero
multi-detection frames, so the highest-confidence person box per frame IS the subject.

THE RULES COMPARED
------------------
    median       plain median of z over the valid patch            (detection.py:478, upstream)
    p20band      20th pct, keep z <= p20 + 0.45 m, median inside    (perception_node since 09-09)
    hist_mode    0.15 m histogram of z, mode bin, median inside     (NOVA-style; bin width from
                                                                     the paper, the rest assumed)
    p10          10th percentile                                   (a robust minimum)
    min          nearest valid sample                              (the speckle-prone extreme)
    iqr_mean     mean of the middle 50 %                           (a trimmed mean)

METRICS -- THERE IS NO GROUND-TRUTH RANGE, SO THESE ARE WHAT CAN HONESTLY BE CLAIMED
------------------------------------------------------------------------------------
  * jump rate: fraction of consecutive-frame pairs (dt < 0.2 s) with |dr| > 0.5 m and > 1.0 m.
    A walking person moves ~0.14 m per frame at 10 Hz, so these are not motion; this is the
    failure that produced the 2026-09-09 holds, measured directly.
  * jitter: robust std (1.4826 * MAD) of the frame-to-frame differences. A smooth trajectory has
    small dr; an estimator that hops has a wide dr distribution regardless of any bias.
  * background landings: fraction of frames where the estimate sits > 1.0 m behind the patch's
    nearest sample. A person is ~0.3 m deep; a metre behind the front of the patch is the wall.
  * offset behind the front: median (estimate - p02 of the patch), i.e. where on the body each
    rule lands. NOT an error -- a torso has depth -- but it decides what "range" means downstream.
  * everything also by range bucket, because the 2026-09-09 failure was range-dependent
    (79.8 % held at 2-3 m, 3-5 % at 1-2 m and 5-6 m) and a scalar hides that.

Writes a per-frame CSV next to the bag so the paper figure can be drawn without re-running YOLO.

    python3 scripts/depth_aggregation_ablation.py ~/dtp_datasets/20260910_flightlegal_bench
"""
import argparse
import csv
import glob
import os
import sqlite3
import sys

CONF = 0.25
MAX_SKEW_S = 0.20          # perception_node's max_depth_colour_skew_s
# perception_node's max_depth_m (declared 10.0). The predictor's own default is 6.0, and a first
# run with 6.0 emptied 94 patches at exactly 6 m -- an artefact of this script, not of the data.
MAX_DEPTH_M = 10.0
HALF_PATCH_W = 0.08        # HumanXZPredictor half_patch_scale_w / _h
HALF_PATCH_H = 0.08
CENTRE_W = 0.50            # perception_node PATCH_CENTRE_W_FRAC / _H_FRAC
CENTRE_H = 0.65
NEAR_PCT = 20.0
NEAR_BAND_M = 0.45
HIST_BIN_M = 0.15
JUMP_DT_S = 0.20
BUCKETS = ((1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 6.0), (6.0, 9.0))


def _bag_rows(con, tid, topic):
    return con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid[topic],)).fetchall()


def _nearest(stamps, t):
    """index of the nearest stamp (stamps sorted ascending)."""
    import bisect
    i = bisect.bisect_left(stamps, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(stamps):
            if best is None or abs(stamps[j] - t) < abs(stamps[best] - t):
                best = j
    return best


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--model', default=os.path.expanduser('~/models/yolo11s.engine'))
    ap.add_argument('--stride', type=int, default=1, help='use every Nth colour frame')
    ap.add_argument('--out', default=None, help='CSV path (default: <bag>/depth_ablation.csv)')
    args = ap.parse_args(argv)

    import numpy as np
    import cv2
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from tf2_msgs.msg import TFMessage
    from ultralytics import YOLO
    from dtp_perception import depth_source, organized_cloud

    files = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % args.bag)
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}
    for t in ('/camera/color/image_raw/compressed', '/camera/depth/image_raw',
              '/camera/depth/camera_info', '/camera/color/camera_info', '/tf_static'):
        if t not in tid:
            raise SystemExit('bag has no %s -- this ablation needs raw depth' % t)

    # -- intrinsics and the factory depth->colour extrinsic, from the bag itself ---------------
    ci = deserialize_message(bytes(_bag_rows(con, tid, '/camera/color/camera_info')[0][1]),
                             CameraInfo)
    K = np.array(ci.k, dtype=np.float64).reshape(3, 3)
    W, H = int(ci.width), int(ci.height)
    di = deserialize_message(bytes(_bag_rows(con, tid, '/camera/depth/camera_info')[0][1]),
                             CameraInfo)
    Kd = np.array(di.k, dtype=np.float64).reshape(3, 3)
    depth_frame = di.header.frame_id
    R_cd = t_cd = None
    for _ts, d in _bag_rows(con, tid, '/tf_static'):
        for tf in deserialize_message(bytes(d), TFMessage).transforms:
            if (tf.child_frame_id == depth_frame
                    and tf.header.frame_id == depth_source.COLOUR_OPTICAL_FRAME):
                R_cd = depth_source.quat_to_R(tf.transform.rotation)
                tr = tf.transform.translation
                t_cd = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
    if R_cd is None:
        # The driver publishes camera_link -> each optical frame; compose through camera_link.
        chain = {}
        for _ts, d in _bag_rows(con, tid, '/tf_static'):
            for tf in deserialize_message(bytes(d), TFMessage).transforms:
                chain[tf.child_frame_id] = tf

        def T_of(frame):
            T = np.eye(4)
            while frame in chain:
                tf = chain[frame]
                M = np.eye(4)
                M[:3, :3] = depth_source.quat_to_R(tf.transform.rotation)
                tr = tf.transform.translation
                M[:3, 3] = [tr.x, tr.y, tr.z]
                T = M @ T
                frame = tf.header.frame_id
            return T, frame
        Td, rd = T_of(depth_frame)
        Tc, rc = T_of(depth_source.COLOUR_OPTICAL_FRAME)
        if rd != rc:
            raise SystemExit('depth frame %r and colour frame %r do not share a root in /tf_static'
                             % (depth_frame, depth_source.COLOUR_OPTICAL_FRAME))
        T_cd = np.linalg.inv(Tc) @ Td
        R_cd, t_cd = T_cd[:3, :3], T_cd[:3, 3]
    print('colour %dx%d  fx %.1f | depth %dx%d fx %.1f | depth->colour baseline %.4f m (%s)'
          % (W, H, K[0, 0], di.width, di.height, Kd[0, 0], float(np.linalg.norm(t_cd)),
             depth_frame))

    # -- frames -----------------------------------------------------------------------------------
    col_rows = _bag_rows(con, tid, '/camera/color/image_raw/compressed')
    dep_rows = _bag_rows(con, tid, '/camera/depth/image_raw')
    dep_msgs = [deserialize_message(bytes(d), Image) for _ts, d in dep_rows]
    dep_stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in dep_msgs]
    print('colour frames %d, depth frames %d, stride %d' % (len(col_rows), len(dep_msgs),
                                                            args.stride))

    model = YOLO(args.model, task='detect')
    stride = max(1, int(depth_source.DEPTH_STRIDE))
    rules = ('median', 'p20band', 'hist_mode', 'p10', 'min', 'iqr_mean')

    def range_image_from(depth_msg):
        """Line-for-line the pipeline's registration + z-buffer (depth_source.py)."""
        dw, dh = int(depth_msg.width), int(depth_msg.height)
        row = depth_msg.step // 2 if depth_msg.step else dw
        depth_m = (np.frombuffer(depth_msg.data, np.uint16).reshape(dh, row)[:, :dw]
                   .astype(np.float32) * depth_source.DEPTH_SCALE_MM)
        vv, uu = np.mgrid[0:dh:stride, 0:dw:stride]
        zz = depth_m[0:dh:stride, 0:dw:stride]
        ok = np.isfinite(zz) & (zz > 0)
        buf = np.full((H, W), np.nan, np.float32)
        if not ok.any():
            return buf
        u = uu[ok].astype(np.float32)
        v = vv[ok].astype(np.float32)
        z = zz[ok]
        pts_d = np.stack([(u - Kd[0, 2]) * z / Kd[0, 0], (v - Kd[1, 2]) * z / Kd[1, 1], z], axis=1)
        pts = (pts_d @ R_cd.T + t_cd).astype(np.float32)
        zc = pts[:, 2]
        rng = np.linalg.norm(pts, axis=1)
        keep = (zc > 0.0) & (rng >= depth_source.MIN_RANGE_M) & (rng <= depth_source.MAX_RANGE_M)
        p, rng = pts[keep], rng[keep]
        if p.size:
            uu2 = K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]
            vv2 = K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]
            inb = (uu2 >= 0) & (uu2 < W) & (vv2 >= 0) & (vv2 < H)
            uu2 = uu2[inb].astype(np.intp)
            vv2 = vv2[inb].astype(np.intp)
            rng = rng[inb]
            order = np.argsort(-rng)
            buf[vv2[order], uu2[order]] = rng[order]
        return buf

    def patch_z(cloud, box):
        """Valid z samples of the torso patch -- perception_node._sample_patch's GEOMETRY."""
        x1, y1, x2, y2 = map(int, box)
        bw, bh = max(1, x2 - x1), max(1, y2 - y1)
        cx, cy = int(x1 + CENTRE_W * bw), int(y1 + CENTRE_H * bh)
        half_w, half_h = max(1, int(HALF_PATCH_W * bw)), max(1, int(HALF_PATCH_H * bh))
        h, w = cloud.shape[:2]
        cx, cy = max(0, min(cx, w - 1)), max(0, min(cy, h - 1))
        patch = cloud[max(0, cy - half_h):min(h, cy + half_h + 1),
                      max(0, cx - half_w):min(w, cx + half_w + 1), :3].reshape(-1, 3)
        xv, zv = patch[:, 0], patch[:, 2]
        mask = np.isfinite(xv) & np.isfinite(zv) & (zv > 0.0) & (zv <= MAX_DEPTH_M)
        return zv[mask], patch.shape[0]

    def estimates(z):
        out = {}
        out['median'] = float(np.median(z))
        p20 = float(np.percentile(z, NEAR_PCT))
        out['p20band'] = float(np.median(z[z <= p20 + NEAR_BAND_M]))
        edges = np.arange(0.0, MAX_DEPTH_M + HIST_BIN_M, HIST_BIN_M)
        counts, _ = np.histogram(z, bins=edges)
        b = int(np.argmax(counts))
        inbin = z[(z >= edges[b]) & (z < edges[b + 1])]
        out['hist_mode'] = float(np.median(inbin)) if inbin.size else float('nan')
        out['p10'] = float(np.percentile(z, 10.0))
        out['min'] = float(z.min())
        q1, q3 = np.percentile(z, [25.0, 75.0])
        mid = z[(z >= q1) & (z <= q3)]
        out['iqr_mean'] = float(mid.mean()) if mid.size else float(np.mean(z))
        out['front'] = float(np.percentile(z, 2.0))     # "front of the patch", speckle-robust
        # p90 - p10 of the patch: > 1.0 m means TWO surfaces are in it (a body is ~0.3 m deep).
        # This is the situation the nearest-surface rule was written for; the rules can only
        # differ on these frames, so they are reported separately.
        out['spread'] = float(np.percentile(z, 90.0) - np.percentile(z, 10.0))
        return out

    rows = []
    n_nodet = n_noskew = n_nopatch = 0
    for i in range(0, len(col_rows), args.stride):
        _ts, d = col_rows[i]
        cmsg = deserialize_message(bytes(d), CompressedImage)
        t = cmsg.header.stamp.sec + cmsg.header.stamp.nanosec * 1e-9
        j = _nearest(dep_stamps, t)
        if j is None or abs(dep_stamps[j] - t) > MAX_SKEW_S:
            n_noskew += 1
            continue
        img = cv2.imdecode(np.frombuffer(cmsg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.shape[1] != W or img.shape[0] != H:
            n_nodet += 1
            continue
        res = model.predict(img, verbose=False, conf=CONF, classes=[0])[0]
        if not len(res.boxes):
            n_nodet += 1
            continue
        k = int(np.argmax(res.boxes.conf.cpu().numpy()))
        box = res.boxes.xyxy[k].cpu().numpy().tolist()
        conf = float(res.boxes.conf[k])
        cloud = organized_cloud.build(range_image_from(dep_msgs[j]), K)
        z, n_px = patch_z(cloud, box)
        if z.size == 0:
            n_nopatch += 1
            continue
        e = estimates(z)
        rows.append(dict(t=t, conf=conf, n_valid=int(z.size), n_px=int(n_px),
                         box_h=float(box[3] - box[1]), **e))
        if len(rows) % 200 == 0:
            print('  %d frames ranged ...' % len(rows))

    print('ranged %d frames | no detection %d | no depth within %.2f s %d | empty patch %d'
          % (len(rows), n_nodet, MAX_SKEW_S, n_noskew, n_nopatch))
    if len(rows) < 10:
        raise SystemExit('too few frames to compare')

    out_csv = args.out or os.path.join(args.bag, 'depth_ablation.csv')
    with open(out_csv, 'w', newline='') as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print('per-frame estimates -> %s' % out_csv)

    # -- metrics -------------------------------------------------------------------------------
    t_arr = np.array([r['t'] for r in rows])
    front = np.array([r['front'] for r in rows])
    ref = np.array([r['p20band'] for r in rows])          # only for bucketing, not as truth
    spread = np.array([r['spread'] for r in rows])
    dt = np.diff(t_arr)
    consecutive = dt < JUMP_DT_S

    def metrics(sel):
        out = {}
        for rule in rules:
            v = np.array([r[rule] for r in rows])
            vs = v[sel]
            pair = consecutive & sel[:-1] & sel[1:]
            dv = np.abs(np.diff(v))[pair]
            mad = float(np.median(np.abs(dv - np.median(dv)))) if dv.size else float('nan')
            out[rule] = dict(
                n=int(vs.size),
                jump05=100.0 * float(np.mean(dv > 0.5)) if dv.size else float('nan'),
                jump10=100.0 * float(np.mean(dv > 1.0)) if dv.size else float('nan'),
                jitter=1.4826 * mad,
                p95dr=float(np.percentile(dv, 95)) if dv.size else float('nan'),
                wall=100.0 * float(np.mean((vs - front[sel]) > 1.0)) if vs.size else float('nan'),
                behind=float(np.median(vs - front[sel])) if vs.size else float('nan'),
            )
        return out

    def table(title, m):
        print()
        print(title)
        print('  rule        n    jump>0.5m%  jump>1.0m%  jitter(m)  p95|dr|(m)  wall%'
              '   behind-front(m)')
        for rule in rules:
            r = m[rule]
            print('  %-9s %5d   %8.1f   %8.1f     %6.3f     %6.3f    %5.1f     %+.3f'
                  % (rule, r['n'], r['jump05'], r['jump10'], r['jitter'], r['p95dr'],
                     r['wall'], r['behind']))

    table('ALL FRAMES (%d)' % len(rows), metrics(np.ones(len(rows), bool)))
    two = spread > 1.0
    print()
    print('PATCH SPREAD (p90-p10 of z): median %.2f m, p90 %.2f m; %d / %d frames (%.1f %%) have'
          % (float(np.median(spread)), float(np.percentile(spread, 90)), int(two.sum()),
             len(rows), 100.0 * two.mean()))
    print('two surfaces in the patch (> 1.0 m). The rules can only disagree on those frames.')
    if two.sum() >= 10:
        table('TWO-SURFACE FRAMES ONLY (%d)' % two.sum(), metrics(two))
    else:
        print('Too few two-surface frames to compare the rules on -- on this bag the choice of')
        print('statistic is NOT exercised, and any difference in the tables is noise.')
    for lo, hi in BUCKETS:
        sel = (ref >= lo) & (ref < hi)
        if sel.sum() >= 20:
            table('RANGE %.0f-%.0f m (%d frames, bucketed on p20band)' % (lo, hi, sel.sum()),
                  metrics(sel))

    print()
    print('READING THIS: jump% and wall% are the failure that produced the 2026-09-09 holds;')
    print('jitter is noise on a smooth path; behind-front is WHERE ON THE BODY the rule lands,')
    print('not an error (a torso is ~0.3 m deep). There is no ground truth in this bag, so no')
    print('column here is an accuracy. A rule that wins jump/wall/jitter while sitting 0.1-0.2 m')
    print('behind the front is measuring the person; one that sits >1 m behind is measuring the')
    print('wall. hist_mode is NOVA-style with only the bin width taken from the paper.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
