#!/usr/bin/env python3
"""Part B Step 0 (docs/plan-veto-height-and-lidar-range.md): does the FLOWN detector see the
walker at 10-17 m?  Range truth from the LiDAR, detections re-run on the recorded frames.

    source drone-dtp/ros2_ws/dtp_env_jetson.sh      # ROS + libcudss for ultralytics
    python3 detector_range_study.py OUT_DIR BAG_DIR [BAG_DIR ...] [--every N] [--no-detect]

Read-only on the bags (sqlite mode=ro). Never run it on a bag that `ros2 bag record` is still
writing (CLAUDE.md). Stages are cached per bag in OUT_DIR/<bag>/ (delete to redo):

1. tele.pkl  -- odom, PX4 status/speed/ToF, /dtp/follow_target, /dtp/targets, lock state,
                /tf_static, camera_info; the camera frame index.
2. lidar.pkl -- every /cloud_registered_body scan, odom-transformed into lidar_init; a ground
                grid (1 m cells, 5th percentile of z, 3x3 median-smoothed); the "body band"
                (0.3-2.1 m above ground, > 1.5 m from the aircraft); and each body-band point's
                BACKGROUND flag: its 0.25 m cell was also hit in >= BG_MIN distinct 0.5 s bins
                that lie 15-60 s away from this scan. A walker who stands still < 15 s, or walks
                back over their own path, stays dynamic; the fence, trees and parked things go.
3. walker.pkl -- the walker's position, by a LiDAR-first alpha-beta tracker on the DYNAMIC
                body-band points, run forward and backward over the bag and merged (a scan where
                the two passes disagree by > 1 m is dropped). Seeds: the locked
                /dtp/follow_target (missed_frames == 0). Only the camera fix's BEARING is trusted:
                the tracker takes the LiDAR cluster along it (0.6-2.5x the camera range). The
                camera range reads short past ~8 m (perception masks every depth sample beyond
                max_depth_m 10, so what is left in the patch is nearer clutter) and lags a walker
                who speeds up; on 09-29 it read 5-6 m for a walker the LiDAR put at 9-13 m. The
                camera POSITION is used only where the MID360 cannot see the walker (a point
                1.35 m above their ground below the sensor's -7 deg floor, body frame -- close in
                from high up, or with the nose pitched up) and inside the track's gate. There is
                no reset on a bearing disagreement with the locked target: the locked target is
                not always the walker (09-29 t 510: auto re-acquire locked the pilot). Truth is
                interpolated between fixes <= 1 s apart, or <= 3 s apart at <= 2.5 m/s.
                EXTRA_PERSONS adds other people found and confirmed by eye (LiDAR only).
4. dets.pkl  -- the flown detector on each frame of the chosen spans: ~/models/yolo11s.engine,
                conf 0.25, imgsz 640, classes [person], model.predict (the flown node calls
                model.track -- BoT-SORT -- on top; predict is the detector's ceiling). Also a
                second pass at ~8 Hz (every 4th frame) through model.track(persist=True), the
                flown call, so the BoT-SORT loss can be seen.
5. frames.csv -- one row per frame: truth range and source, whether the walker projects inside
                the image (through the bag's own /tf_static chain and camera_info), the expected
                box height for a 1.75 m person, and the detected box that matches the walker's
                projected column (if any) with its confidence and height.

The spans detected are the armed window (first arm - 10 s .. disarm + 5 s) plus any other span
where the LiDAR walker is 7-20 m from the aircraft and inside the camera's azimuth (every 2nd
frame there). The report prints detection by range bin per bag, per window (WINDOWS) and for
day / dusk, and the flown camera range against the LiDAR range of the same person.

    python3 detector_range_study.py OUT_DIR --report      # the tables again, from the CSVs

Validation built in: for a matched box, `du` (box centre - projected column) and box_h/h_exp.
On 09-29 du is a median 3-8 px (p10-p90 within +-18 px) and box_h/h_exp 0.92-0.95 in every bin,
so the truth is on the person and the match is not a loose one.
"""
import argparse
import bisect
import csv
import math
import os
import pickle
import sqlite3
import struct
import sys
import time

import numpy as np

# ---------------------------------------------------------------- parameters
BAND_LO, BAND_HI = 0.3, 2.1        # body band, m above local ground
SELF_R = 1.5                       # drop points this close to the aircraft (props, legs)
MAX_R = 45.0                       # and beyond this
CELL = 0.25                        # background cell, m
BIN = 0.5                          # background time bin, s
BG_NEAR, BG_FAR, BG_MIN = 15.0, 60.0, 4
GATE0, GATE_RATE, GATE_MAX = 0.8, 3.0, 3.0
COAST_MAX = 3.0                    # s without an update -> lost
VMAX = 4.5                         # m/s
ALPHA, BETA = 0.7, 0.3
PERSON_H = 1.75                    # m, for the expected box height
FILL_GAP = 1.0                     # s: interpolate truth between fixes at most this far apart,
FILL_GAP_SLOW = 3.0                # or this far when the walker moved <= 2.5 m/s between them
BINS = [(4, 6), (6, 8), (8, 10), (10, 12), (12, 15), (15, 17), (17, 20), (20, 30)]
# People other than the locked walker, 8-20 m away and in the camera's azimuth, found with an
# independent scan for moving person-sized LiDAR clusters and CONFIRMED BY EYE on the frames
# (2026-09-30). Every other candidate that scan found in the five bags was trees or bushes (a
# moving sensor sees vegetation only briefly, so it passes the background test). (bag t, x, y).
EXTRA_PERSONS = {
    'stage_b_2026-09-21_195658_salvaged': [(590.0, 18.0, 2.2)],   # walks in 20 -> 13 m after landing, dusk
}
# Windows reported separately (bag t). 09-29: the aircraft held position while the walker went
# out and back (443-453), and the pilot's retreat at up to 6 m/s (500-506).
WINDOWS = {
    'stage_b_2026-09-29_122511': [(443.0, 453.0, 'hover 443-453'), (500.0, 506.0, 'retreat 500-506')],
}
LIGHT = {
    'stage_b_2026-09-29_122511': 'day, sun (12:32)',
    'stage_b_2026-09-21_183532': 'day, overcast (18:41)',
    'stage_b_2026-09-26_195108': 'dusk (19:57)',
    'stage_b_2026-09-26_200650': 'dusk, dark (20:12)',
    'stage_b_2026-09-21_195658_salvaged': 'dusk, dark (20:05)',
}


def hs(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_of(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_R(q)
    T[:3, 3] = p
    return T


def db3_of(bag):
    fs = [f for f in os.listdir(bag) if f.endswith('.db3')]
    if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        sys.exit('refusing: %s has no metadata.yaml (still recording, or cut and not reindexed)' % bag)
    if len(fs) != 1:
        sys.exit('refusing: expected one .db3 in %s' % bag)
    return os.path.join(bag, fs[0])


# ---------------------------------------------------------------- stage 1: telemetry
def stage_tele(db3):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    c = sqlite3.connect('file:%s?mode=ro' % db3, uri=True)
    topics = {r[1]: (r[0], r[2]) for r in c.execute('select id,name,type from topics')}
    t0 = c.execute('select min(timestamp) from messages').fetchone()[0]

    def msgs(name):
        if name not in topics:
            return
        tid, ty = topics[name]
        cls = get_message(ty)
        for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
            yield (ts - t0) / 1e9, deserialize_message(d, cls)

    D = {'t0_ns': t0}
    D['status'] = np.array([(t, m.nav_state, m.arming_state) for t, m in msgs('/fmu/out/vehicle_status_v1')])
    D['lpos'] = np.array([(t, m.vx, m.vy, m.dist_bottom, float(m.dist_bottom_valid))
                          for t, m in msgs('/fmu/out/vehicle_local_position_v1')])
    D['odom'] = np.array([(t, hs(m.header), m.pose.pose.position.x, m.pose.pose.position.y,
                           m.pose.pose.position.z, m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                           m.pose.pose.orientation.z, m.pose.pose.orientation.w)
                          for t, m in msgs('/lidar_slam/odom')])
    D['follow'] = np.array([(t, hs(m.header), m.track_id, m.position.x, m.position.y, m.position.z,
                             m.distance_m, m.missed_frames, m.confidence)
                            for t, m in msgs('/dtp/follow_target')], dtype=float).reshape(-1, 9)
    D['targets'] = [(t, hs(m.header), [(g.track_id, g.position.x, g.position.y, g.position.z,
                                         g.is_locked_target, g.distance_m, g.confidence, g.missed_frames)
                                        for g in m.targets]) for t, m in msgs('/dtp/targets')]
    D['lock'] = [(t, m.data) for t, m in msgs('/dtp/lock_state')]
    D['tf_static'] = []
    for t, m in msgs('/tf_static'):
        for tr in m.transforms:
            q, p = tr.transform.rotation, tr.transform.translation
            D['tf_static'].append((tr.header.frame_id, tr.child_frame_id, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
    for t, m in msgs('/camera/color/camera_info'):
        D['K'], D['dist'], D['img_wh'] = np.array(m.k).reshape(3, 3), np.array(m.d), (m.width, m.height)
        break
    tid = topics['/camera/color/image_raw/compressed'][0]
    D['frames'] = np.array([(r[0], (r[1] - t0) / 1e9) for r in
                            c.execute('select id,timestamp from messages where topic_id=? order by timestamp', (tid,))])
    st = D['status']
    armed = st[st[:, 2] == 2, 0] if len(st) else []
    D['arm_win'] = (armed[0] - 10.0, armed[-1] + 5.0) if len(armed) else None
    return D


# ---------------------------------------------------------------- stage 2: LiDAR
def stage_lidar(db3, D):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2 as pc2
    c = sqlite3.connect('file:%s?mode=ro' % db3, uri=True)
    topics = {r[1]: (r[0], r[2]) for r in c.execute('select id,name,type from topics')}
    t0 = D['t0_ns']
    od = D['odom']
    tid, ty = topics['/cloud_registered_body']
    cls = get_message(ty)
    scans = []
    for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        m = deserialize_message(d, cls)
        st = hs(m.header)
        i = int(np.searchsorted(od[:, 1], st))
        j = min([k for k in (i - 1, i) if 0 <= k < len(od)], key=lambda k: abs(od[k, 1] - st))
        if abs(od[j, 1] - st) > 0.02:
            continue
        a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        w = a @ quat_R(od[j, 5:9]).T + od[j, 2:5]
        scans.append(((ts - t0) / 1e9, st, od[j, 2:5].copy(), w.astype(np.float32)))
    # ground grid: 1 m cells, 5th percentile of z (>= 10 points), then 3x3 median of valid cells
    allp = np.vstack([s[3] for s in scans])
    gx, gy = np.floor(allp[:, 0]).astype(np.int64), np.floor(allp[:, 1]).astype(np.int64)
    key = (gx + 2000) * 4000 + (gy + 2000)
    o = np.argsort(key, kind='stable')
    ks, zs = key[o], allp[o, 2]
    u, s0 = np.unique(ks, return_index=True)
    e0 = np.r_[s0[1:], len(ks)]
    raw = {}
    for k, a, b in zip(u, s0, e0):
        if b - a >= 10:
            raw[int(k)] = float(np.percentile(zs[a:b], 5))
    ground = {}
    for k in raw:
        nb = [raw.get(k + dx * 4000 + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
        ground[k] = float(np.median([v for v in nb if v is not None]))
    gkeys = np.array(sorted(ground))
    gvals = np.array([ground[k] for k in gkeys])
    del allp

    def ground_at(x, y):
        k = (np.floor(x).astype(np.int64) + 2000) * 4000 + (np.floor(y).astype(np.int64) + 2000)
        i = np.clip(np.searchsorted(gkeys, k), 0, len(gkeys) - 1)
        return np.where(gkeys[i] == k, gvals[i], np.nan)

    band = []
    for tb, st, ac, w in scans:
        g = ground_at(w[:, 0], w[:, 1])
        h = w[:, 2] - g
        dh = np.hypot(w[:, 0] - ac[0], w[:, 1] - ac[1])
        m = np.isfinite(h) & (h > BAND_LO) & (h < BAND_HI) & (dh > SELF_R) & (dh < MAX_R)
        band.append(np.c_[w[m, :2], h[m]].astype(np.float32))
    # background: cell hit in >= BG_MIN distinct bins that are BG_NEAR..BG_FAR s away from now
    cx = [np.floor(b[:, 0] / CELL).astype(np.int64) + 100000 for b in band]
    cy = [np.floor(b[:, 1] / CELL).astype(np.int64) + 100000 for b in band]
    bins = [int(s[0] / BIN) for s in scans]
    NB = 1 << 14
    ck = [(x * 200000 + y) for x, y in zip(cx, cy)]
    allk = np.unique(np.concatenate([k * NB + b for k, b in zip(ck, bins)]))
    near, far = int(BG_NEAR / BIN), int(BG_FAR / BIN)
    out = []
    for (tb, st, ac, w), b, k, pts in zip(scans, bins, ck, band):
        base = k * NB

        def cnt(lo, hi):
            return np.searchsorted(allk, base + np.clip(hi, 0, NB - 1), 'right') - \
                np.searchsorted(allk, base + np.clip(lo, 0, NB - 1), 'left')
        n = cnt(b - far, b - near) + cnt(b + near, b + far)
        dyn = n < BG_MIN
        out.append((tb, st, ac.astype(np.float32), pts[dyn], int(len(pts))))
    return {'scans': out, 'gkeys': gkeys, 'gvals': gvals}


# ---------------------------------------------------------------- stage 3: walker tracker
def camera_meas(D):
    """Camera fixes of the walker, keyed by header stamp: (stamp, x, y, locked, range, conf)."""
    M = []
    for r in D['follow']:
        if r[7] == 0:
            M.append((r[1], r[3], r[4], True, r[6], r[8]))
    for t, st, gs in D['targets']:
        for g in gs:
            if not g[4] and g[7] == 0:
                M.append((st, g[1], g[2], False, g[5], g[6]))
    M.sort()
    return M


def pose_interp(od, st):
    hsv = od[:, 1]
    i = int(np.clip(np.searchsorted(hsv, st) - 1, 0, len(hsv) - 2))
    a = float(np.clip((st - hsv[i]) / max(1e-6, hsv[i + 1] - hsv[i]), 0, 1))
    p = (1 - a) * od[i, 2:5] + a * od[i + 1, 2:5]
    q0, q1 = od[i, 5:9], od[i + 1, 5:9].copy()
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1 - a) * q0 + a * q1
    return p, q / np.linalg.norm(q)


def clusters(p, eps=0.6):
    """Connected components in xy (small N): list of index arrays."""
    n = len(p)
    if n == 0:
        return []
    lab = -np.ones(n, int)
    L = 0
    for i in range(n):
        if lab[i] >= 0:
            continue
        stack = [i]
        lab[i] = L
        while stack:
            j = stack.pop()
            d = np.hypot(p[:, 0] - p[j, 0], p[:, 1] - p[j, 1])
            for k in np.where((d < eps) & (lab < 0))[0]:
                lab[k] = L
                stack.append(k)
        L += 1
    return [np.where(lab == k)[0] for k in range(L)]


def yaw_of(q):
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def angdiff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def lidar_blind(od, st, wx, wy, g):
    """True when the MID360 cannot see more than the top ~0.4 m of a 1.75 m walker at (wx, wy):
    a point 1.35 m above their ground lies below the sensor's -7 deg floor (body frame)."""
    p, q = pose_interp(od, st)
    v = quat_R(q).T @ (np.array([wx, wy, g + 1.35]) - p)
    return math.degrees(math.atan2(v[2], math.hypot(v[0], v[1]))) < -7.0


def wedge(pts, ac, brg, rc):
    """Dynamic points along a camera bearing: within max(3 deg, 0.6 m) of it, at 0.6-2.5x the
    camera's range (the camera range past ~8 m reads short; see the module docstring)."""
    if len(pts) == 0:
        return pts
    rp = np.hypot(pts[:, 0] - ac[0], pts[:, 1] - ac[1])
    bp = np.arctan2(pts[:, 1] - ac[1], pts[:, 0] - ac[0])
    db = np.abs((bp - brg + np.pi) % (2 * np.pi) - np.pi)
    return pts[(db < np.maximum(math.radians(3.0), np.arctan2(0.6, rp))) & (rp > 0.6 * rc) & (rp < 2.5 * rc)]


def run_pass(scans, cams, od, ground, direction, seeds=()):
    """One LiDAR-first alpha-beta pass over the scans, forward (+1) or backward (-1) in time.
    Returns {scan index: (x, y, src, npts)} for the scans where the track was updated."""
    order = range(len(scans)) if direction > 0 else range(len(scans) - 1, -1, -1)
    cam_st = np.array([c[0] for c in cams]) if cams else np.zeros(0)
    fixes = {}
    state = None            # (p at t_any, forward-time velocity, t_lidar, t_any)
    for k in order:
        tb, st, ac, pts, _ = scans[k]
        lo, hi = np.searchsorted(cam_st, st - 0.05), np.searchsorted(cam_st, st + 0.05)
        cam_here = []
        for i in range(lo, hi):
            c = cams[i]
            cp, _ = pose_interp(od, c[0])
            cam_here.append((c, math.atan2(c[2] - cp[1], c[1] - cp[0]), math.hypot(c[1] - cp[0], c[2] - cp[1])))
        if state is not None and abs(st - state[3]) > COAST_MAX:
            state = None
        if state is None:
            for sst, sx, sy in seeds:          # a manual seed (EXTRA_PERSONS): LiDAR cluster near it
                if abs(sst - st) < 0.05 and len(pts):
                    q = pts[np.hypot(pts[:, 0] - sx, pts[:, 1] - sy) < 1.5]
                    if len(q):
                        z = np.median(q[:, :2], axis=0)
                        state = (z, np.zeros(2), st, st)
                        fixes[k] = (z[0], z[1], 'lidar', len(q))
            if state is not None:
                continue
            for c, brg, rc in cam_here:
                if not c[3]:
                    continue
                q = wedge(pts, ac, brg, rc)
                cl = max(clusters(q), key=len) if len(q) >= 2 else []
                if len(cl) >= 2:
                    z = np.median(q[cl, :2], axis=0)
                    state = (z, np.zeros(2), st, st)
                    fixes[k] = (z[0], z[1], 'lidar', len(cl))
                    break
                cp = pose_interp(od, c[0])[0]
                far = cp[:2] + 1.5 * (np.array([c[1], c[2]]) - cp[:2])      # the camera reads short
                if lidar_blind(od, st, c[1], c[2], ground(c[1], c[2])) and \
                        lidar_blind(od, st, far[0], far[1], ground(far[0], far[1])):
                    z = np.array([c[1], c[2]])
                    state = (z, np.zeros(2), -1e18 * direction, st)
                    fixes[k] = (z[0], z[1], 'cam', 0)
                    break
            continue
        p0, v, tl, ta = state
        dt = st - ta
        if abs(dt) < 1e-6:
            continue
        pred = p0 + v * dt
        pb = math.atan2(pred[1] - ac[1], pred[0] - ac[0])
        since_l = abs(st - tl)
        gate = min(GATE0 + GATE_RATE * since_l, GATE_MAX)
        # camera fixes that are the walker: the locked one, or any target on the track's bearing
        mine = [(c, brg, rc) for c, brg, rc in cam_here if c[3] or angdiff(brg, pb) < math.radians(10)]
        z, src, n = None, None, 0
        for c, brg, rc in mine:             # LiDAR along the camera bearing first
            q = wedge(pts, ac, brg, rc)
            if len(q):
                q = q[np.hypot(q[:, 0] - pred[0], q[:, 1] - pred[1]) < GATE_MAX + 1.0]
            if len(q) >= 2:
                cl = max(clusters(q, 0.5), key=len)
                if len(cl) >= 2:
                    z, src, n = np.median(q[cl, :2], axis=0), 'lidar', len(cl)
                    break
        if z is None and len(pts):
            q = pts[np.hypot(pts[:, 0] - pred[0], pts[:, 1] - pred[1]) < gate]
            if len(q):
                best = min(clusters(q, 0.5), key=lambda cl: np.hypot(*(np.median(q[cl, :2], axis=0) - pred)))
                zc = np.median(q[best, :2], axis=0)
                if len(best) >= 2 or (np.hypot(*(zc - pred)) < 0.8 and since_l < 0.6):
                    z, src, n = zc, 'lidar', len(best)
        # No bearing-disagreement reset: the locked target is not always the walker (09-29 t 510:
        # auto re-acquire locked the pilot at the landing spot). The camera POSITION is used only
        # where the MID360 cannot see the walker (judged at the track's own position, not at the
        # camera's range, which reads short), and only inside the gate.
        if z is None and lidar_blind(od, st, pred[0], pred[1], ground(pred[0], pred[1])):
            for c, brg, rc in mine:
                if math.hypot(c[1] - pred[0], c[2] - pred[1]) < max(gate, 1.5):
                    z, src, n = np.array([c[1], c[2]]), 'cam', 0
                    break
        if z is None:
            continue
        r = z - pred
        pn = pred + (ALPHA if src == 'lidar' else 0.5) * r
        vn = v + BETA * r / (dt if abs(dt) > 0.05 else math.copysign(0.05, dt))
        sp = float(np.hypot(*vn))
        if sp > VMAX:
            vn = vn * VMAX / sp
        state = (pn, vn, st if src == 'lidar' else tl, st)
        fixes[k] = (pn[0], pn[1], src, n)
    return fixes


def stage_walker(L, D, name):
    scans = L['scans']
    cams = camera_meas(D)
    od = D['odom']
    gk, gv = L['gkeys'], L['gvals']

    def ground(x, y):
        for r in range(6):
            vals = []
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    kk = (int(math.floor(x)) + dx + 2000) * 4000 + (int(math.floor(y)) + dy + 2000)
                    i = int(np.searchsorted(gk, kk))
                    if i < len(gk) and gk[i] == kk:
                        vals.append(gv[i])
            if vals:
                return float(np.median(vals))
        return 0.0
    F = run_pass(scans, cams, od, ground, +1)
    B = run_pass(scans, cams, od, ground, -1)
    merged, conflicts = merge_passes(scans, F, B)
    extra = []
    t0 = D['t0_ns'] * 1e-9
    for tb, x, y in EXTRA_PERSONS.get(name, []):
        # a second person, LiDAR only: seeded at every scan within +-0.3 s of the given time so
        # that one sparse scan does not lose the seed
        sd = [(t0 + tb + d, x, y) for d in np.arange(-0.3, 0.31, 0.1)]
        ef, _ = merge_passes(scans, run_pass(scans, [], od, ground, +1, sd), run_pass(scans, [], od, ground, -1, sd))
        extra.append(ef)
    return {'fixes': merged, 'n_fwd': len(F), 'n_bwd': len(B), 'conflicts': conflicts, 'extra': extra}


def merge_passes(scans, F, B):
    merged, conflicts = [], 0
    for k in sorted(set(F) | set(B)):
        f, b = F.get(k), B.get(k)
        st = scans[k][1]
        if f and b:
            if math.hypot(f[0] - b[0], f[1] - b[1]) > (1.0 if f[2] == b[2] == 'lidar' else 1.5):
                conflicts += 1
                continue
            if f[2] == b[2]:
                merged.append((st, (f[0] + b[0]) / 2, (f[1] + b[1]) / 2, f[2], max(f[3], b[3])))
            else:
                x = f if f[2] == 'lidar' else b
                merged.append((st, x[0], x[1], 'lidar', x[3]))
        else:
            x = f or b
            merged.append((st, x[0], x[1], x[2], x[3]))
    return merged, conflicts


# ---------------------------------------------------------------- geometry helpers
class Geo:
    def __init__(self, D, L):
        parent = {c: (p, T_of(t, q)) for p, c, t, q in D['tf_static']}
        T, f = np.eye(4), 'camera_color_optical_frame'
        while f != 'lidar_imu':
            p, M = parent[f]
            T, f = M @ T, p
        self.T_opt_imu = np.linalg.inv(T)
        self.K, self.dist = D['K'], D['dist'][:5]
        self.W, self.H = D['img_wh']
        self.od = D['odom']
        self.gk, self.gv = L['gkeys'], L['gvals']

    def ground(self, x, y):
        for r in (0, 1, 2, 3, 4, 5):
            vals = []
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    k = (int(math.floor(x)) + dx + 2000) * 4000 + (int(math.floor(y)) + dy + 2000)
                    i = int(np.searchsorted(self.gk, k))
                    if i < len(self.gk) and self.gk[i] == k:
                        vals.append(self.gv[i])
            if vals:
                return float(np.median(vals))
        return float('nan')

    def cam(self, st):
        p, q = pose_interp(self.od, st)
        return self.T_opt_imu @ np.linalg.inv(T_of(p, q)), p, q

    def project(self, Tc, P):
        import cv2
        pc = Tc @ np.array([P[0], P[1], P[2], 1.0])
        # Outside the lens's field the plumb_bob polynomial folds a point back INTO the image
        # (09-29 t 460, yawing at 33 deg/s). The colour camera is about +-46 x +-30 deg; refuse
        # rays beyond +-55 x +-40 deg before distorting.
        if pc[2] < 0.3 or abs(pc[0] / pc[2]) > 1.43 or abs(pc[1] / pc[2]) > 0.84:
            return None, pc[2]
        uv, _ = cv2.projectPoints(pc[None, :3], np.zeros(3), np.zeros(3), self.K, self.dist)
        return (float(uv[0, 0, 0]), float(uv[0, 0, 1])), pc[2]


def truth_at(fx_st, fixes, st):
    """Walker (x, y, src, gap) at stamp st from the merged fixes, or None."""
    i = bisect.bisect_right(fx_st, st)
    a = fixes[i - 1] if i > 0 else None
    b = fixes[i] if i < len(fixes) else None
    if a and b and (b[0] - a[0] <= FILL_GAP or (b[0] - a[0] <= FILL_GAP_SLOW and
                    math.hypot(b[1] - a[1], b[2] - a[2]) <= 2.5 * (b[0] - a[0]))):
        w = (st - a[0]) / max(1e-6, b[0] - a[0])
        src = a[3] if a[3] == b[3] else 'mixed'
        return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), src, b[0] - a[0])
    for c in (a, b):
        if c and abs(c[0] - st) <= 0.15:
            return (c[1], c[2], c[3], abs(c[0] - st))
    return None


# ---------------------------------------------------------------- stage 4: spans + detector
def choose_spans(D, L, Wk, geo):
    spans = []
    if D['arm_win']:
        spans.append((D['arm_win'][0], D['arm_win'][1], 1, 'armed'))
    fixes = Wk['fixes']
    fx_st = [f[0] for f in fixes]
    t0 = D['t0_ns'] * 1e-9
    fr = D['frames']
    hit = []
    for tb in np.arange(fr[0, 1], fr[-1, 1], 0.25):
        st = t0 + tb
        tr = truth_at(fx_st, fixes, st)
        if tr is None:
            continue
        Tc, p, q = geo.cam(st)
        rng = math.hypot(tr[0] - p[0], tr[1] - p[1])
        if not (7.0 <= rng <= 20.0):
            continue
        uv, z = geo.project(Tc, (tr[0], tr[1], p[2]))
        if uv is not None and 0 <= uv[0] < geo.W:
            hit.append(tb)
    # group into spans, grown 1 s, minus the armed window
    extra = []
    for tb in hit:
        if D['arm_win'] and D['arm_win'][0] <= tb <= D['arm_win'][1]:
            continue
        if extra and tb - extra[-1][1] <= 2.0:
            extra[-1][1] = tb
        else:
            extra.append([tb, tb])
    for a, b in extra:
        a, b = a - 1.0, b + 1.0
        if D['arm_win']:
            if a < D['arm_win'][1] and b > D['arm_win'][0]:
                if a < D['arm_win'][0]:
                    spans.append((a, D['arm_win'][0], 2, 'ground/other'))
                if b > D['arm_win'][1]:
                    spans.append((D['arm_win'][1], b, 2, 'ground/other'))
                continue
        spans.append((a, b, 2, 'ground/other'))
    return sorted(spans)


def stage_detect(db3, D, spans, every_override=None):
    import cv2
    from ultralytics import YOLO
    model_path = os.path.expanduser('~/models/yolo11s.engine')
    c = sqlite3.connect('file:%s?mode=ro' % db3, uri=True)
    det_model = YOLO(model_path, task='detect')
    fr = D['frames']
    out = {}
    t_start = time.time()
    n = 0
    for a, b, every, label in spans:
        if every_override:
            every = every_override
        trk_model = YOLO(model_path, task='detect')   # fresh BoT-SORT per span
        sel = np.where((fr[:, 1] >= a) & (fr[:, 1] <= b))[0]
        for j, i in enumerate(sel):
            on_grid = (j % 4 == 0)
            if j % every and not on_grid:
                continue
            rid = int(fr[i, 0])
            raw = c.execute('select data from messages where id=?', (rid,)).fetchone()[0]
            sec, nsec = struct.unpack_from('<iI', raw, 4)
            img = cv2.imdecode(np.frombuffer(raw[raw.find(b'\xff\xd8\xff'):], np.uint8), cv2.IMREAD_COLOR)
            boxes = None
            if j % every == 0:
                r = det_model.predict(img, imgsz=640, conf=0.25, classes=[0], verbose=False)[0]
                bx = r.boxes
                boxes = np.hstack([bx.xyxy.cpu().numpy(), bx.conf.cpu().numpy()[:, None]]) if len(bx) else np.zeros((0, 5))
            trk = None
            if on_grid:
                r = trk_model.track(img, persist=True, imgsz=640, conf=0.25, classes=[0], verbose=False)[0]
                bx = r.boxes
                if bx.id is not None and len(bx):
                    trk = np.hstack([bx.xyxy.cpu().numpy(), bx.conf.cpu().numpy()[:, None], bx.id.cpu().numpy()[:, None]])
                else:
                    trk = np.zeros((0, 6))
            out[int(i)] = (sec + nsec * 1e-9, boxes, trk, label)
            n += 1
            if n % 500 == 0:
                print('    %d frames, %.1f fps' % (n, n / (time.time() - t_start)), flush=True)
        del trk_model
    print('    detector: %d frames in %.1f s' % (n, time.time() - t_start))
    return out


# ---------------------------------------------------------------- stage 5: per-frame rows
def lock_at(D, tb):
    ts = [x[0] for x in D['lock']]
    i = bisect.bisect_right(ts, tb) - 1
    return D['lock'][i][1].split('|')[0] if i >= 0 else ''


def nav_at(D, tb):
    st = D['status']
    i = int(np.searchsorted(st[:, 0], tb, 'right')) - 1
    return (int(st[i, 1]), int(st[i, 2])) if i >= 0 else (-1, -1)


def match_box(boxes, u, v_head, v_feet, depth, fx):
    if boxes is None or len(boxes) == 0:
        return None
    h = max(1.0, v_feet - v_head)
    tol = 15.0 + fx * 0.4 / max(depth, 0.5)
    best = None
    for b in boxes:
        cu = (b[0] + b[2]) / 2
        du = cu - u
        if abs(du) > (b[2] - b[0]) / 2 + tol:
            continue
        if b[3] < v_head - 0.3 * h or b[1] > v_feet + 0.3 * h:
            continue
        if best is None or abs(du) < abs(best[1]):
            best = (b, du)
    return best


def stage_rows(name, D, L, Wk, dets, geo):
    persons = [('walker', Wk['fixes'])] + [('extra%d' % (j + 1), ef) for j, ef in enumerate(Wk.get('extra', []))]
    persons = [(pn, fx, [f[0] for f in fx]) for pn, fx in persons]
    fr = D['frames']
    lp = D['lpos']
    rows = []
    for i in sorted(dets):
        st, boxes, trk, label = dets[i]
        tb = fr[i, 1]
        nav, arm = nav_at(D, tb)
        li = int(np.searchsorted(lp[:, 0], tb, 'right')) - 1
        spd = float(np.hypot(lp[li, 1], lp[li, 2])) if li >= 0 else float('nan')
        agl = float(lp[li, 3]) if li >= 0 and lp[li, 4] > 0.5 else float('nan')
        Tc, p, q = geo.cam(st)
        _, q2 = pose_interp(D['odom'], st + 0.05)
        _, q1 = pose_interp(D['odom'], st - 0.05)
        yr = math.degrees(angdiff(yaw_of(q2), yaw_of(q1))) / 0.1      # |yaw rate|, deg/s: blur
        for pname, fixes, fx_st in persons:
            row = dict(bag=name, person=pname, t_bag=round(tb, 3), stamp=round(st, 3), span=label, nav=nav,
                       armed=int(arm == 2), ac_speed=round(spd, 2), ac_agl=round(agl, 2), lock=lock_at(D, tb),
                       n_boxes=-1 if boxes is None else len(boxes), ac_yawrate=round(yr, 1),
                       ac_x=round(p[0], 2), ac_y=round(p[1], 2), ac_z=round(p[2], 2))
            tr = truth_at(fx_st, fixes, st)
            if tr is None:
                if pname == 'walker':
                    row.update(truth='none')
                    rows.append(row)
                continue
            row.update(person_row(tr, Tc, p, q, geo, boxes, trk))
            rows.append(row)
    return rows


def person_row(tr, Tc, p, q, geo, boxes, trk):
    wx, wy, src, gap = tr
    g = geo.ground(wx, wy)
    rng = math.hypot(wx - p[0], wy - p[1])
    brg = math.atan2(wy - p[1], wx - p[0])
    off = math.degrees((brg - yaw_of(q) + math.pi) % (2 * math.pi) - math.pi)
    row = dict(truth=src, truth_gap=round(gap, 2), walker_x=round(wx, 2), walker_y=round(wy, 2),
               walker_ground=round(g, 2), range_m=round(rng, 2), bearing_off_deg=round(off, 1))
    uvc, zc = geo.project(Tc, (wx, wy, g + 0.9))
    uvf, _ = geo.project(Tc, (wx, wy, g))
    uvh, _ = geo.project(Tc, (wx, wy, g + PERSON_H))
    if uvc is None or uvf is None or uvh is None:
        row.update(fov='out')
        return row
    h_exp = uvf[1] - uvh[1]
    w_exp = 0.45 * geo.K[0, 0] / zc
    u = uvc[0]
    inside_h = (u - w_exp / 2 >= 0) and (u + w_exp / 2 <= geo.W)
    inside_v = (uvh[1] >= -0.1 * h_exp) and (uvf[1] <= geo.H + 0.1 * h_exp)
    any_v = uvf[1] > 0 and uvh[1] < geo.H
    fov = 'in' if (inside_h and inside_v) else ('edge' if (0 <= u < geo.W and any_v) else 'out')
    row.update(fov=fov, u_proj=round(u, 1), v_head=round(uvh[1], 1), v_feet=round(uvf[1], 1),
               h_exp_px=round(h_exp, 1), depth_m=round(zc, 2))
    if boxes is not None:
        m = match_box(boxes, u, uvh[1], uvf[1], zc, geo.K[0, 0])
        if m:
            b, du = m
            row.update(det=1, conf=round(float(b[4]), 3), box_h=round(float(b[3] - b[1]), 1),
                       box_w=round(float(b[2] - b[0]), 1), du=round(du, 1),
                       box=' '.join('%.0f' % x for x in b[:4]))
        else:
            row.update(det=0)
    if trk is not None:
        row.update(trk8=1 if match_box(trk, u, uvh[1], uvf[1], zc, geo.K[0, 0]) else 0)
    return row


COLS = ['bag', 'person', 't_bag', 'stamp', 'span', 'nav', 'armed', 'ac_speed', 'ac_yawrate', 'ac_agl', 'lock', 'ac_x', 'ac_y', 'ac_z',
        'truth', 'truth_gap', 'walker_x', 'walker_y', 'walker_ground', 'range_m', 'bearing_off_deg', 'fov',
        'u_proj', 'v_head', 'v_feet', 'h_exp_px', 'depth_m', 'n_boxes', 'det', 'conf', 'box_h', 'box_w', 'du',
        'box', 'trk8']


NUM = {'t_bag', 'stamp', 'nav', 'armed', 'ac_speed', 'ac_yawrate', 'ac_agl', 'ac_x', 'ac_y', 'ac_z', 'truth_gap', 'walker_x',
       'walker_y', 'walker_ground', 'range_m', 'bearing_off_deg', 'u_proj', 'v_head', 'v_feet', 'h_exp_px',
       'depth_m', 'n_boxes', 'det', 'conf', 'box_h', 'box_w', 'du', 'trk8'}


def load_csv(path):
    out = []
    for r in csv.DictReader(open(path)):
        out.append({k: (float(v) if k in NUM and v != '' else (v if v != '' else None)) for k, v in r.items()})
    return out


def table(rows, title):
    """Detection by range bin over the frames where the person is wholly inside the image."""
    rows = [r for r in rows if r.get('range_m') is not None and r.get('det') is not None]
    if not rows:
        return
    print('\n  %s' % title)
    print('    %-7s %6s %6s %6s %9s %9s %11s %11s %9s %6s %7s %8s' % (
        'range m', 'inFOV', 'det', 'det %', 'conf mean', 'conf med', 'box h med', 'h_exp med', 'trk8 %', 'edge',
        'ac m/s', 'yaw d/s'))
    for lo, hi in BINS:
        sel = [r for r in rows if lo <= r['range_m'] < hi]
        inn = [r for r in sel if r['fov'] == 'in']
        edge = sum(1 for r in sel if r['fov'] == 'edge')
        if not inn:
            if edge:
                print('    %2d-%-4d %6d %6s %6s %9s %9s %11s %11s %9s %6d' % (lo, hi, 0, '', '', '', '', '', '', '', edge))
            continue
        d = [r for r in inn if r['det'] == 1]
        cf = [r['conf'] for r in d]
        bh = [r['box_h'] for r in d]
        t8 = [r['trk8'] for r in inn if r.get('trk8') is not None]
        print('    %2d-%-4d %6d %6d %5.0f%% %9s %9s %11s %11.0f %9s %6d %7.1f %8.0f' % (
            lo, hi, len(inn), len(d), 100.0 * len(d) / len(inn),
            '%.2f' % np.mean(cf) if cf else '-', '%.2f' % np.median(cf) if cf else '-',
            '%.0f' % np.median(bh) if bh else '-', np.median([r['h_exp_px'] for r in inn]),
            ('%.0f%%/%d' % (100.0 * np.mean(t8), len(t8))) if t8 else '-', edge,
            np.nanmedian([r['ac_speed'] for r in inn]), np.nanmedian([r.get('ac_yawrate') or 0 for r in inn])))


def report(rows):
    bags = sorted(set(r['bag'] for r in rows), key=lambda b: b[8:])
    print('\n' + '=' * 100)
    print('REPORT. det = the flown detector (yolo11s.engine, conf 0.25, imgsz 640, person) found a box on the')
    print('LiDAR-tracked person; inFOV = the whole 1.75 m person projects inside the 1280x720 image; box h in')
    print('full-res px (the engine sees half: 640 wide); h_exp = geometric height of a 1.75 m person there;')
    print('trk8 = matched by model.track (BoT-SORT, the flown call) on every 4th frame (~7.5 Hz).')
    for b in bags:
        R = [r for r in rows if r['bag'] == b]
        print('\n--- %s  [%s]' % (b, LIGHT.get(b, '?')))
        for pn in sorted(set(r['person'] for r in R)):
            P = [r for r in R if r['person'] == pn]
            table([r for r in P if r['armed'] == 1], '%s, armed' % pn)
            table([r for r in P if r['armed'] != 1], '%s, not armed (on the ground)' % pn)
        for lo, hi, lab in WINDOWS.get(b, []):
            table([r for r in R if r['person'] == 'walker' and lo <= r['t_bag'] <= hi], 'window: %s' % lab)
        if WINDOWS.get(b):
            rest = [r for r in R if r['person'] == 'walker' and r['armed'] == 1 and
                    not any(lo <= r['t_bag'] <= hi for lo, hi, _ in WINDOWS[b])]
            table(rest, 'armed, outside those windows (moving follow)')
            table([r for r in R if r['truth'] == 'lidar' and r['armed'] == 1],
                  'robustness: armed, LiDAR-only truth (no camera fixes, no mixed interpolation)')
    day = [r for r in rows if LIGHT.get(r['bag'], '').startswith('day')]
    dusk = [r for r in rows if LIGHT.get(r['bag'], '').startswith('dusk')]
    print('\n--- ALL BAGS')
    table([r for r in day if r['armed'] == 1], 'day, armed (airborne or on the pad while armed)')
    table([r for r in day if r['armed'] != 1], 'day, not armed')
    table([r for r in dusk if r['armed'] == 1], 'dusk, armed')
    table([r for r in dusk if r['armed'] != 1], 'dusk, not armed')


def camera_vs_lidar(D, Wk):
    """The flown camera range (locked /dtp/follow_target, fresh) against the LiDAR fix of the same
    person (within 0.06 s and 5 deg of bearing), by LiDAR range. Positive = camera reads long.
    A by-product only: stereo_lidar_bias.py (Step 1) is the measurement of this."""
    lf = [f for f in Wk['fixes'] if f[3] == 'lidar']
    ls = np.array([f[0] for f in lf])
    if not len(ls):
        return
    out = {}
    for r in D['follow']:
        if r[7] != 0:
            continue
        i = int(np.argmin(np.abs(ls - r[1])))
        if abs(ls[i] - r[1]) > 0.06:
            continue
        p, _ = pose_interp(D['odom'], r[1])
        rl = math.hypot(lf[i][1] - p[0], lf[i][2] - p[1])
        rc = math.hypot(r[3] - p[0], r[4] - p[1])
        # the same person: on the same bearing (09-29 t 509-512 the lock was on the pilot)
        if angdiff(math.atan2(r[4] - p[1], r[3] - p[0]), math.atan2(lf[i][2] - p[1], lf[i][1] - p[0])) > math.radians(5):
            continue
        for lo, hi in BINS:
            if lo <= rl < hi:
                out.setdefault((lo, hi), []).append(rc - rl)
    if out:
        print('  camera range - LiDAR range (m), locked target, by LiDAR range:')
        for k in sorted(out):
            v = np.array(out[k])
            print('    %2d-%-3d n %4d  median %+5.2f  p10 %+5.2f  p90 %+5.2f  |err|>1 m %3.0f%%' % (
                k[0], k[1], len(v), np.median(v), np.percentile(v, 10), np.percentile(v, 90), 100 * np.mean(np.abs(v) > 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('bags', nargs='*')
    ap.add_argument('--every', type=int, default=None, help='detect every Nth frame in every span')
    ap.add_argument('--no-detect', action='store_true')
    ap.add_argument('--report', action='store_true', help='only print the report from OUT/*/frames.csv')
    a = ap.parse_args()
    if a.report:
        rows = []
        for d in sorted(os.listdir(a.out)):
            f = os.path.join(a.out, d, 'frames.csv')
            if os.path.isfile(f):
                rows += load_csv(f)
        report(rows)
        return
    for bag in a.bags:
        bag = os.path.abspath(bag)
        name = os.path.basename(bag.rstrip('/'))
        db3 = db3_of(bag)
        wd = os.path.join(a.out, name)
        os.makedirs(wd, exist_ok=True)
        print('== %s' % name, flush=True)

        def cached(fn, f, *args):
            p = os.path.join(wd, fn)
            if os.path.isfile(p):
                return pickle.load(open(p, 'rb'))
            t = time.time()
            v = f(*args)
            pickle.dump(v, open(p, 'wb'))
            print('  %s: %.1f s' % (fn, time.time() - t), flush=True)
            return v
        D = cached('tele.pkl', stage_tele, db3)
        L = cached('lidar.pkl', stage_lidar, db3, D)
        Wk = cached('walker.pkl', stage_walker, L, D, name)
        src = {}
        for f in Wk['fixes']:
            src[f[3]] = src.get(f[3], 0) + 1
        print('  walker fixes %s (fwd %d, bwd %d updates, %d conflicts dropped); extra persons: %s' % (
            src, Wk['n_fwd'], Wk['n_bwd'], Wk['conflicts'], [len(e) for e in Wk.get('extra', [])]))
        camera_vs_lidar(D, Wk)
        geo = Geo(D, L)
        spans = choose_spans(D, L, Wk, geo)
        print('  spans: %s' % ', '.join('%.1f-%.1f (%s, every %d)' % (s[0], s[1], s[3], a.every or s[2]) for s in spans))
        if a.no_detect:
            continue
        dets = cached('dets.pkl', stage_detect, db3, D, spans, a.every)
        rows = stage_rows(name, D, L, Wk, dets, geo)
        with open(os.path.join(wd, 'frames.csv'), 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=COLS, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
    rows = []
    for d in sorted(os.listdir(a.out)):
        f = os.path.join(a.out, d, 'frames.csv')
        if os.path.isfile(f):
            rows += load_csv(f)
    if rows and not a.no_detect:
        with open(os.path.join(a.out, 'frames_all.csv'), 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=COLS, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
        report(rows)


if __name__ == '__main__':
    main()
