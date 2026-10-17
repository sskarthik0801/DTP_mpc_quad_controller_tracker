#!/usr/bin/env python3
"""M5.3a on REAL data: replay a bag through fsm_node with the target-point filter OFF and ON.

WHAT IT MEASURES
----------------
The M5.3 exit metric (docs/m5-m6-plan.md SS2.2): "a camera->target segment crosses an ex-trail
cell in <= 2 % of LOCKED frames". The bag's own /cloud_registered_body (put into lidar_init with the
bag's odometry), /lidar_slam/odom and /dtp/follow_target are republished at 1x, in bag order, into
the real fsm_node on a private localhost-only domain, once per arm. ROG-Map's occupancy is read off
/rog_map/occ (2 Hz) and, for every SEEN target sample, the segment from the sensor to the target is
walked through the newest snapshot:

  ex-trail cell   occupied, within TRAIL_R_M of where the subject stood >= TRAIL_AGE_S ago, more
                  than CURRENT_R_M from where they stand now, and NOT static background.
  crossing frame  a seen target whose sensor->target segment (stopping CURRENT_R_M short of them)
                  passes through at least one ex-trail cell.

Static background is every 0.2 m voxel returning in >= 30 % of scans (target_filter_study.py's
mask), so furniture next to the walked path is not counted as trail in either arm. That assumes a
stationary rig; the script refuses a bag where the rig moved.

The sensor is taken as the odometry origin (the MID360's IMU); the camera is centimetres from it,
which is below one cell at the ranges involved.

A replay is NOT bit-deterministic (1x pacing, executor order), so compare arms, not decimals.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/target_filter_replay.py ~/dtp_datasets/20260909_closedloop_run5
"""
import argparse
import bisect
import glob
import math
import os
import re
import sqlite3
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import target_filter_check as tfc          # noqa: E402  (fsm_node sandbox + config generation)
from target_filter_study import _quat_to_R, _stamp, _xyz, BG_VOXEL_M, BG_FRACTION  # noqa: E402

RES = 0.1
TRAIL_R_M = 0.5
TRAIL_AGE_S = 1.5
CURRENT_R_M = 0.8
SENSOR_SKIP_M = 0.5        # ROG-Map's ray_range minimum: nothing closer is ever updated
WARMUP_S = 5.0             # the map needs a few seconds before a crossing means anything


def load(bag):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from dtp_msgs.msg import Target
    files = sorted(glob.glob(os.path.join(bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % bag)
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}
    need = ('/cloud_registered_body', '/lidar_slam/odom', '/dtp/follow_target')
    missing = [t for t in need if t not in tid]
    if missing:
        raise SystemExit('bag lacks %s' % ', '.join(missing))
    names = {v: k for k, v in tid.items()}
    rows = con.execute('SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) '
                       'ORDER BY timestamp' % ','.join(str(tid[t]) for t in need)).fetchall()
    od = {}
    events = []
    for topic_id, ts, data in rows:
        name = names[topic_id]
        if name == '/lidar_slam/odom':
            m = deserialize_message(bytes(data), Odometry)
            p = m.pose.pose.position
            od[round(_stamp(m.header), 4)] = (np.array([p.x, p.y, p.z]), _quat_to_R(m.pose.pose.orientation))
            events.append((ts * 1e-9, 'odom', m))
        elif name == '/dtp/follow_target':
            events.append((ts * 1e-9, 'target', deserialize_message(bytes(data), Target)))
        else:
            events.append((ts * 1e-9, 'cloud', deserialize_message(bytes(data), PointCloud2)))
    od_keys = sorted(od)
    return events, od, od_keys


def pose_at(od, od_keys, t):
    j = bisect.bisect_left(od_keys, round(t, 4))
    cand = [od_keys[x] for x in (j - 1, j) if 0 <= x < len(od_keys)]
    k = min(cand, key=lambda x: abs(x - t)) if cand else None
    return od[k] if k is not None and abs(k - t) <= 0.01 else None


def static_cells(events, od, od_keys):
    counts, n = {}, 0
    pos0, travel = None, 0.0
    for _t, kind, m in events:
        if kind != 'cloud':
            continue
        pr = pose_at(od, od_keys, _stamp(m.header))
        if pr is None:
            continue
        pos, R = pr
        pos0 = pos if pos0 is None else pos0
        travel = max(travel, float(np.linalg.norm(pos - pos0)))
        n += 1
        for key in set(map(tuple, np.floor((_xyz(m) @ R.T + pos) / BG_VOXEL_M).astype(np.int64))):
            counts[key] = counts.get(key, 0) + 1
    if travel > 0.3:
        raise SystemExit('the rig moved %.2f m; the static-background mask assumes a stationary rig' % travel)
    return {k for k, c in counts.items() if c >= BG_FRACTION * n}


def replay(h, events, od, od_keys, rate):
    """Republish at `rate` x bag time. Returns [(wall_t, occ_cells)] and [(wall_t, target, sensor_pos)]."""
    from sensor_msgs_py import point_cloud2
    from sensor_msgs.msg import PointField
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
    snaps, seen = [], []
    last_occ_t = 0.0
    t_bag0 = events[0][0]
    t_wall0 = time.time() + 0.5
    for t_bag, kind, m in events:
        due = t_wall0 + (t_bag - t_bag0) / rate
        while time.time() < due:
            h.rclpy.spin_once(h.node, timeout_sec=max(0.0, min(0.01, due - time.time())))
            if h.occ_t > last_occ_t:
                last_occ_t = h.occ_t
                snaps.append((h.occ_t, {tuple(np.floor(p / RES + 0.5).astype(int)) for p in h.occ}))
        if kind == 'odom':
            h.odom_pub.publish(m)
        elif kind == 'target':
            h.target_pub.publish(m)
            if m.is_locked_target and m.missed_frames == 0 and len(m.predicted_path) >= 2:
                pr = pose_at(od, od_keys, _stamp(m.header))
                if pr is not None:
                    seen.append((time.time(), np.array([m.position.x, m.position.y, m.position.z]), pr[0]))
        else:
            pr = pose_at(od, od_keys, _stamp(m.header))
            if pr is None:
                continue
            pos, R = pr
            w = (_xyz(m) @ R.T + pos).astype(np.float32)
            data = np.hstack([w, np.full((len(w), 1), 100.0, dtype=np.float32)])
            m.header.frame_id = 'world'
            h.cloud_pub.publish(point_cloud2.create_cloud(m.header, fields, data))
    h.spin(1.0)
    return snaps, seen


def score(snaps, seen, static):
    def is_static(c):
        return tuple(np.floor(np.array(c) * RES / BG_VOXEL_M).astype(np.int64)) in static

    snap_t = [s[0] for s in snaps]
    t0 = seen[0][0] if seen else 0.0
    frames = crossing = 0
    trail_counts = []
    for i, (t, p, sensor) in enumerate(seen):
        if t - t0 < WARMUP_S:
            continue
        j = bisect.bisect_right(snap_t, t) - 1
        if j < 0:
            continue
        occ = snaps[j][1]
        past = [q for (tq, q, _s) in seen[:i] if t - tq >= TRAIL_AGE_S and math.hypot(q[0] - p[0], q[1] - p[1]) > CURRENT_R_M]
        if not past:
            continue
        past_xy = np.array([[q[0], q[1]] for q in past])

        def ex_trail(c):
            x, y = c[0] * RES, c[1] * RES
            if math.hypot(x - p[0], y - p[1]) <= CURRENT_R_M or is_static(c):
                return False
            return bool(np.min(np.hypot(past_xy[:, 0] - x, past_xy[:, 1] - y)) <= TRAIL_R_M)

        frames += 1
        seg = p - sensor
        L = float(np.linalg.norm(seg))
        hit = False
        if L > SENSOR_SKIP_M + CURRENT_R_M:
            u = seg / L
            for s in np.arange(SENSOR_SKIP_M, L - CURRENT_R_M, 0.05):
                c = tuple(np.floor((sensor + u * s) / RES + 0.5).astype(int))
                if c in occ and ex_trail(c):
                    hit = True
                    break
        crossing += hit
        # all ex-trail cells in the map at this instant, within the subject's vertical extent
        trail_counts.append(sum(1 for c in occ if abs(c[2] * RES - p[2] - 0.15) <= 1.2 and ex_trail(c)))
    return frames, crossing, trail_counts


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--logs', default=None)
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = tfc.DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    events, od, od_keys = load(args.bag)
    static = static_cells(events, od, od_keys)
    print('\ntarget_filter_replay -- %s: %d messages, %d static background voxels'
          % (os.path.basename(args.bag.rstrip(os.sep)), len(events), len(static)))

    tmpdir = args.logs or tempfile.mkdtemp(prefix='target_filter_replay_')
    os.makedirs(tmpdir, exist_ok=True)
    base = open(tfc.BASE_CFG).read()
    cfgs = {}
    for arm in ('off', 'on'):
        cfgs[arm] = os.path.join(tmpdir, 'replay_%s.yaml' % arm)
        open(cfgs[arm], 'w').write(re.sub(r'^(  target_exclusion:\n    enable:) (true|false)',
                                          r'\1 %s' % ('true' if arm == 'on' else 'false'), base, flags=re.M))
    h = tfc.Harness()
    out = {}
    try:
        for arm in ('off', 'on'):
            log_path = os.path.join(tmpdir, 'replay_%s.log' % arm)
            proc, log = tfc.start_fsm(cfgs[arm], log_path)
            try:
                h.occ, h.occ_t, h.markers = None, 0.0, []
                time.sleep(6.0)          # A* preallocation; the map starts with the first scan
                snaps, seen = replay(h, events, od, od_keys, args.rate)
            finally:
                tfc.stop_fsm(proc, log)
            frames, crossing, trail = score(snaps, seen, static)
            filt = re.findall(r'target exclusion: filtered (\d+) scans', open(log_path).read())
            out[arm] = (frames, crossing, trail, sum(int(x) for x in filt))
            print('  %-3s  seen frames scored %4d   crossing an ex-trail cell %4d (%.1f %%)   '
                  'ex-trail cells in map: mean %.1f  p95 %.0f  max %d   scans filtered %d'
                  % (arm.upper(), frames, crossing, 100.0 * crossing / max(1, frames),
                     float(np.mean(trail)) if trail else 0.0,
                     float(np.percentile(trail, 95)) if trail else 0.0, max(trail) if trail else 0,
                     out[arm][3]))
    finally:
        h.close()
    print('\nM5.3 exit (on the aircraft, at 3.5 m standoff): crossing <= 2 %% of LOCKED frames. '
          'This is a bench replay (stationary rig) -- the number is %s evidence, not the gate.'
          % ('\\bench{}'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
