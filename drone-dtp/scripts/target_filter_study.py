#!/usr/bin/env python3
"""M5.3a: where must the target-point filter's cylinder sit to actually cover the person?

THE QUESTION
------------
fsm_node's target-point filter (m5-m6-plan.md M5.3 item 1) drops the followed person's LiDAR
returns before ROG-Map inserts them, so a walking subject stops stamping a trail of occupied cells
behind themselves. It centres a cylinder on the locked target from /dtp/follow_target. Three
things make "the target's position" the wrong centre if taken naively, and none of them shows on a
stationary bench:

  * /dtp/follow_target reaches fsm_node ~0.27 s after its header.stamp, while a LiDAR scan
    reaches it ~0.05 s after its own. At cloud-arrival time the newest target describes the person
    as they were ~0.2-0.35 s EARLIER than the scan being inserted.
  * Its position is EMA-smoothed (detection.py smooth_alpha 0.25), which lags a walking person by
    roughly v * frame_period * (1 - a) / a -- ~0.45 m at 1.2 m/s and 8 Hz, if nothing else
    compensates.
  * A HELD frame (perception's range gate rejected the torso sample) republishes the LAST position
    under the NEW stamp. Its forecast has one point; that is the only mark it carries.

So this replays the bag CAUSALLY -- for each scan, only targets that had arrived by the scan's own
arrival are usable, exactly as in fsm_node -- and scores candidate centres against where the
person's points actually are in that scan:

  C0  position as published (no extrapolation)
  C1  position + velocity * (t_scan - t_target)
  C2  target_manager's own polyline -- (0, position) then predicted_path -- evaluated at that age
  L1, L2  C1 with an extra lead of 0.15 / 0.30 s along the velocity (candidate EMA-lag compensation)

The person's points are found around C1 within the filter's z band, and their median is taken as
the ground truth for that scan. That is only meaningful where the subject stands clear of other
structure, so the cluster must be compact and populated or the scan is not scored (the counts say
how many were).

    python3 scripts/target_filter_study.py ~/dtp_datasets/20260909_closedloop_run5
"""
import argparse
import bisect
import glob
import math
import os
import sqlite3

import numpy as np

# The z band the person's points are searched in, relative to the target's z. Deliberately wider
# than any filter band under test: the plan's [z - 1.2, z + 0.6] assumed z is a torso centre, but
# perception samples the box at 0.65 of its height FROM THE TOP (detection.py:444), i.e. near the
# waist, so a head is ~1.1-1.2 m above z. The profile printed below is what sets the filter's band.
Z_BELOW_M = 1.5
Z_ABOVE_M = 1.6
# Search radius for the subject's points around C1. Wider than any filter radius under test, so
# the ground truth is not defined by the answer.
SEARCH_R_M = 1.0
# After the first median, keep points this close to it; this is the "person" whose coverage counts.
PERSON_R_M = 0.6
MIN_PERSON_PTS = 20
# A person seen from one side is at most ~0.3 m of horizontal spread; a wall segment is not.
MAX_SPREAD_M = 0.30
# Points this close to the floor are ground, not person (only applied when a floor is visible).
FLOOR_CLEAR_M = 0.15
RADII = (0.4, 0.5, 0.6, 0.7)
# An observation older than this at scan time is not used (the filter's max_extrapolation_s).
MAX_AGE_S = 0.5
SPEED_BINS = ((0.0, 0.3), (0.3, 0.8), (0.8, 9.9))
# Static background: a 0.2 m voxel returning in at least this fraction of scans is furniture, not
# the subject, and its points are removed before the subject is searched for. Without this the
# first run of this study reported a 0.8 m p95 "error" that was a fixed object at (3.80, 1.75) in
# run5 -- found at the same world point at t = 48, 61, 75, 93, 104 and 110 s while the subject
# walked 0.8 m from it. VALID ONLY FOR A STATIONARY RIG (the script refuses a moving one).
BG_VOXEL_M = 0.2
BG_FRACTION = 0.3


def _rows(con, tid, topic):
    if topic not in tid:
        return []
    return con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid[topic],)).fetchall()


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _quat_to_R(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _xyz(cloud):
    step = cloud.point_step
    n = cloud.width * cloud.height
    off = {f.name: f.offset for f in cloud.fields}
    buf = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(n, step)
    out = np.empty((n, 3), dtype=np.float64)
    for i, k in enumerate('xyz'):
        out[:, i] = buf[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0]
    return out[np.isfinite(out).all(axis=1)]


def _polyline_at(ts, pts, t):
    if t <= ts[0]:
        return pts[0]
    for i in range(1, len(ts)):
        if t <= ts[i]:
            f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
            return pts[i - 1] + f * (pts[i] - pts[i - 1])
    return pts[-1]


def _pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float('nan')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[1])
    ap.add_argument('bag')
    ap.add_argument('--cloud', default='/cloud_registered_body')
    ap.add_argument('--max-age', type=float, default=MAX_AGE_S,
                    help='oldest observation used at scan time (the filter\'s max_age), s')
    ap.add_argument('--worst', type=int, default=0,
                    help='print the N scans where C1 is furthest from the person, for inspection')
    args = ap.parse_args(argv)

    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from dtp_msgs.msg import Target

    files = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % args.bag)
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}

    od = {}
    for _ts, d in _rows(con, tid, '/lidar_slam/odom'):
        m = deserialize_message(bytes(d), Odometry)
        p = m.pose.pose.position
        od[round(_stamp(m.header), 4)] = (np.array([p.x, p.y, p.z]), _quat_to_R(m.pose.pose.orientation))
    od_keys = sorted(od)

    # Targets, in ARRIVAL order (bag time), with everything the filter could read.
    tg = []
    n_held = n_coast = 0
    for ts, d in _rows(con, tid, '/dtp/follow_target'):
        m = deserialize_message(bytes(d), Target)
        if m.missed_frames > 0:
            n_coast += 1
            continue
        if len(m.predicted_path) < 2:
            n_held += 1
            continue
        p = np.array([m.position.x, m.position.y, m.position.z])
        v = np.array([m.velocity.x, m.velocity.y, 0.0])
        ts_poly = [0.0] + [float(x) for x in m.predicted_path_dt]
        pts_poly = [p] + [np.array([q.x, q.y, q.z]) for q in m.predicted_path]
        tg.append((ts * 1e-9, _stamp(m.header), p, v, ts_poly, pts_poly))
    if len(tg) < 20:
        raise SystemExit('only %d seen /dtp/follow_target samples' % len(tg))
    tg_arrival = [t[0] for t in tg]

    clouds = _rows(con, tid, args.cloud)
    print()
    print('=' * 78)
    print('TARGET-FILTER CENTRE STUDY   %s' % os.path.basename(args.bag.rstrip(os.sep)))
    print('=' * 78)
    print('scans %d on %s; seen targets %d (coasted %d and held %d are never used)'
          % (len(clouds), args.cloud, len(tg), n_coast, n_held))

    # Floor height: the lowest well-populated z layer over the first scans, in the world frame.
    zs = []
    for _ts, d in clouds[:60]:
        c = deserialize_message(bytes(d), PointCloud2)
        k = min(od_keys, key=lambda x: abs(x - _stamp(c.header)))
        pos, R = od[k]
        w = _xyz(c) @ R.T + pos
        zs.append(w[np.hypot(w[:, 0] - pos[0], w[:, 1] - pos[1]) < 8.0, 2])
    zs = np.concatenate(zs)
    hist, edges = np.histogram(zs, bins=np.arange(zs.min(), zs.max() + 0.05, 0.05))
    # The MID360 sees nothing below -7 deg, so from a raised bench rig the floor is often not in
    # view at all within 8 m. Take the lowest layer holding >= 2 % of the points, and only use it
    # if it is genuinely below every target (checked per scan).
    idx = np.nonzero(hist >= 0.02 * hist.sum())[0]
    floor_z = float(edges[idx[0]] + 0.025) if len(idx) else -1e9
    print('lowest populated z layer %.2f m in lidar_init (used as floor only where below z - %.1f)'
          % (floor_z, Z_BELOW_M))

    # -- pass 1: world points per scan, and the static background ------------------------------
    world = []
    pos0 = None
    max_travel = 0.0
    for ts, d in clouds:
        c = deserialize_message(bytes(d), PointCloud2)
        t_c = _stamp(c.header)
        j = bisect.bisect_left(od_keys, round(t_c, 4))
        cand_k = [od_keys[x] for x in (j - 1, j) if 0 <= x < len(od_keys)]
        k = min(cand_k, key=lambda x: abs(x - t_c)) if cand_k else None
        if k is None or abs(k - t_c) > 0.01:
            world.append((ts, t_c, None, None))
            continue
        pos, R = od[k]
        pos0 = pos if pos0 is None else pos0
        max_travel = max(max_travel, float(np.linalg.norm(pos - pos0)))
        world.append((ts, t_c, pos, _xyz(c) @ R.T + pos))
    if max_travel > 0.3:
        raise SystemExit('the rig moved %.2f m: the static-background mask assumes a stationary '
                         'rig and would be wrong here' % max_travel)
    counts = {}
    n_valid = 0
    for _ts, _t, _pos, w in world:
        if w is None:
            continue
        n_valid += 1
        for key in set(map(tuple, np.floor(w / BG_VOXEL_M).astype(np.int64))):
            counts[key] = counts.get(key, 0) + 1
    static = {key for key, n in counts.items() if n >= BG_FRACTION * n_valid}
    print('static background: %d of %d voxels (>= %.0f %% of %d scans), rig travel %.2f m'
          % (len(static), len(counts), 100 * BG_FRACTION, n_valid, max_travel))

    ages, speeds, zrel, detail = [], [], [], []
    NAMES = ('C0', 'C1', 'C2', 'L1', 'L2')
    off = {k: [] for k in NAMES}
    along = {k: [] for k in NAMES}
    cov = {k: {r: [] for r in RADII} for k in NAMES}
    skipped = {'no target yet': 0, 'target too old': 0, 'no odom': 0, 'too few points': 0,
               'not compact': 0}
    for ts, t_c, pos, w in world:
        arrival = ts * 1e-9
        i = bisect.bisect_right(tg_arrival, arrival) - 1
        if i < 0:
            skipped['no target yet'] += 1
            continue
        _arr, t_obs, p, v, ts_poly, pts_poly = tg[i]
        age = t_c - t_obs
        if abs(age) > args.max_age:
            skipped['target too old'] += 1
            continue
        if w is None:
            skipped['no odom'] += 1
            continue
        keys = np.floor(w / BG_VOXEL_M).astype(np.int64)
        w = w[np.fromiter((tuple(q) not in static for q in keys), dtype=bool, count=len(keys))]

        centres = {'C0': p, 'C1': p + v * age, 'C2': _polyline_at(ts_poly, pts_poly, age),
                   'L1': p + v * (age + 0.15), 'L2': p + v * (age + 0.30)}
        z_lo = p[2] - Z_BELOW_M
        if floor_z + FLOOR_CLEAR_M < p[2] - 0.3:
            z_lo = max(z_lo, floor_z + FLOOR_CLEAR_M)
        z_hi = p[2] + Z_ABOVE_M
        band = w[(w[:, 2] >= z_lo) & (w[:, 2] <= z_hi)]
        c1 = centres['C1']
        near = band[np.hypot(band[:, 0] - c1[0], band[:, 1] - c1[1]) <= SEARCH_R_M]
        if len(near) < MIN_PERSON_PTS:
            skipped['too few points'] += 1
            continue
        med = np.median(near[:, :2], axis=0)
        person = band[np.hypot(band[:, 0] - med[0], band[:, 1] - med[1]) <= PERSON_R_M]
        if len(person) < MIN_PERSON_PTS:
            skipped['too few points'] += 1
            continue
        med = np.median(person[:, :2], axis=0)
        spread = float(np.sqrt(np.mean(np.sum((person[:, :2] - med) ** 2, axis=1))))
        if spread > MAX_SPREAD_M:
            skipped['not compact'] += 1
            continue

        ages.append(age)
        zrel.append(person[:, 2] - p[2])
        # a second, independent object near the person inflates the median; count what the 1.0 m
        # search saw beyond the person's own 0.6 m so the tail can be attributed
        ray = (med - pos[:2]) / max(1e-6, float(np.hypot(*(med - pos[:2]))))
        d1 = c1[:2] - med
        detail.append((t_c, float(np.hypot(*d1)), len(person), spread,
                       float(np.hypot(p[0] - pos[0], p[1] - pos[1])), int(len(near) - len(person)),
                       float(np.hypot(v[0], v[1])), age, float(d1 @ ray),
                       float(med[0]), float(med[1]), float(c1[0]), float(c1[1])))
        sp = float(np.hypot(v[0], v[1]))
        speeds.append(sp)
        heading = v[:2] / sp if sp > 0.3 else None
        for name, ctr in centres.items():
            dxy = ctr[:2] - med
            off[name].append(float(np.hypot(*dxy)))
            # positive = the centre is AHEAD of the person along their velocity
            along[name].append(float(dxy @ heading) if heading is not None else float('nan'))
            dist = np.hypot(person[:, 0] - ctr[0], person[:, 1] - ctr[1])
            for r in RADII:
                cov[name][r].append(float(np.mean(dist <= r)))

    n = len(ages)
    print('scored %d scans; skipped %s' % (n, ', '.join('%s %d' % kv for kv in skipped.items())))
    if n == 0:
        return 1
    ages = np.array(ages)
    speeds = np.array(speeds)
    print('observation age at scan time (t_scan - t_target): p5 %.3f  p50 %.3f  p95 %.3f s'
          % (_pct(ages, 5), _pct(ages, 50), _pct(ages, 95)))
    print('subject speed (published velocity): p50 %.2f  p95 %.2f  max %.2f m/s'
          % (_pct(speeds, 50), _pct(speeds, 95), float(speeds.max())))
    zr = np.concatenate(zrel)
    print('person points\' z relative to target z (pooled, n %d): p1 %+.2f  p5 %+.2f  p50 %+.2f  '
          'p95 %+.2f  p99 %+.2f  max %+.2f m' % (len(zr), _pct(zr, 1), _pct(zr, 5), _pct(zr, 50),
                                               _pct(zr, 95), _pct(zr, 99), float(zr.max())))
    print()
    print('horizontal offset of each candidate centre from the person\'s LiDAR median, m')
    print('%-4s %8s %8s %8s   %s' % ('', 'p50', 'p95', 'max', 'along-track p50 when moving > 0.3 m/s (+ = ahead)'))
    for name in NAMES:
        a = np.array(off[name])
        al = np.array([x for x in along[name] if math.isfinite(x)])
        print('%-4s %8.3f %8.3f %8.3f   %s' % (name, _pct(a, 50), _pct(a, 95), float(a.max()),
                                             ('%+.3f  (n %d)' % (_pct(al, 50), len(al))) if len(al) else 'n/a'))
    print()
    print('fraction of the person\'s points inside the cylinder (p50 / p5 over scans)')
    print('%-4s ' % '' + ''.join('   r=%.1f      ' % r for r in RADII))
    for name in NAMES:
        print('%-4s ' % name + ''.join('%5.2f / %4.2f  ' % (_pct(cov[name][r], 50), _pct(cov[name][r], 5))
                                       for r in RADII))
    if args.worst:
        t0 = clouds[0][0] * 1e-9
        print()
        print('worst %d scans by C1 offset: t(s)  offset  n_person  spread  range  n_beyond_0.6m  speed  '
              'age  radial(+=target beyond person)  lidar_med_xy  C1_xy' % args.worst)
        for row in sorted(detail, key=lambda r: -r[1])[:args.worst]:
            print('  %6.1f  %5.2f  %4d  %4.2f  %5.2f  %4d  %4.2f  %4.2f  %+5.2f  (%5.2f,%5.2f)  (%5.2f,%5.2f)'
                  % ((row[0] - t0,) + row[1:]))
        rad = np.array([r[8] for r in detail])
        rng = np.array([r[4] for r in detail])
        print('radial component of C1 offset by target range (+ = target placed BEYOND the LiDAR person):')
        for lo_r, hi_r in ((0, 3), (3, 3.5), (3.5, 4), (4, 4.5), (4.5, 99)):
            sel = (rng >= lo_r) & (rng < hi_r)
            if sel.any():
                print('  %.1f-%.1f m  n %3d  radial p50 %+.2f  p95 %+.2f  |offset| p50 %.2f'
                      % (lo_r, hi_r, int(sel.sum()), _pct(rad[sel], 50), _pct(rad[sel], 95),
                         _pct(np.array([r[1] for r in detail])[sel], 50)))
        print('C1 offset p95 per 10 s of bag:')
        tt = np.array([r[0] - t0 for r in detail])
        oo = np.array([r[1] for r in detail])
        for b in range(0, int(tt.max()) + 10, 10):
            sel = (tt >= b) & (tt < b + 10)
            if sel.any():
                print('  %3d-%3d s  n %3d  p50 %.2f  p95 %.2f' % (b, b + 10, int(sel.sum()),
                                                                 _pct(oo[sel], 50), _pct(oo[sel], 95)))
    print()
    print('C1 by observation age (how far the filter can extrapolate)')
    off1 = np.array(off['C1'])
    cov1 = np.array(cov['C1'][0.6])
    for lo_a, hi_a in ((0.0, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 0.7), (0.7, 1.0), (1.0, 9.9)):
        sel = (ages >= lo_a) & (ages < hi_a)
        if sel.any():
            print('  %.1f-%.1f s  n %4d   offset p50 %.3f p95 %.3f   cov@0.6 p50 %.2f p5 %.2f'
                  % (lo_a, hi_a, int(sel.sum()), _pct(off1[sel], 50), _pct(off1[sel], 95),
                     _pct(cov1[sel], 50), _pct(cov1[sel], 5)))
    print()
    print('C1 offset p95 by subject speed')
    for lo_s, hi_s in SPEED_BINS:
        sel = (speeds >= lo_s) & (speeds < hi_s)
        a = np.array(off['C1'])[sel]
        a0 = np.array(off['C0'])[sel]
        print('  %.1f-%.1f m/s  n %4d   C0 p95 %.3f   C1 p95 %.3f   C1 cov@0.5 p5 %.2f'
              % (lo_s, hi_s, int(sel.sum()), _pct(a0, 95), _pct(a, 95),
                 _pct(np.array(cov['C1'][0.5])[sel], 5)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
