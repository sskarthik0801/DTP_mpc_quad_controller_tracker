"""Shared by the Step 1 measurement scripts (stereo_lidar_bias.py, lidar_box_points.py,
background_margin.py): read one flight bag ONCE and cache what they need as an .npz.

    python3 _step1_cache.py <bag_dir> [<cache_dir>]      # builds <cache_dir>/<bag>.npz

Cached (all stamps are header stamps, epoch seconds; `start` is the bag's start time):
  od_*   every /lidar_slam/odom (lidar_init <- lidar_imu)
  sc_*   every /cloud_registered_body scan, odom-transformed into lidar_init (float32), with the
         aircraft position at the scan stamp. FAST-LIO stamps the cloud and the odom from the same
         iteration (lidar_end_time), so the match is exact (a scan with no odom within 20 ms is
         dropped and counted).
  tg_*   every Target in /dtp/targets (the camera measurement; position is lidar_init)
  ft_*   /dtp/follow_target (only its track_id and state are used: which track is the walker)
  lk_*   /dtp/lock_state, mx_* /mux/state
  lag_*  median (bag receive time - header stamp) per topic, to spot a clock offset.
Read-only on the bag. NEVER run it on a bag that `ros2 bag record` is still writing.
"""
import os
import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DEFAULT_CACHE = ('/tmp/claude-1000/-home-ideas-ad-SUPER-Human-DTP/'
                 'd04c92ae-44af-44ea-8c75-b858015fae68/scratchpad/step1_measure/cache')
TOPICS = ['/lidar_slam/odom', '/cloud_registered_body', '/dtp/targets', '/dtp/follow_target',
          '/dtp/lock_state', '/mux/state', '/camera/color/camera_info']


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def xyz_view(m):
    """PointCloud2 -> (N,3) float32 via one buffer view (depth_source.cloud_xyz's method)."""
    off = {f.name: f.offset for f in m.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [np.float32] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
    n = len(m.data) // m.point_step
    a = np.frombuffer(memoryview(m.data)[:n * m.point_step], dtype=dt)
    p = np.stack([a['x'], a['y'], a['z']], axis=1)
    return p[np.isfinite(p).all(axis=1)]


def build(bag, cache_dir=DEFAULT_CACHE):
    bag = bag.rstrip('/')
    os.makedirs(cache_dir, exist_ok=True)
    out = os.path.join(cache_dir, os.path.basename(bag) + '.npz')
    if os.path.exists(out):
        return out
    info = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    start = info['starting_time']['nanoseconds_since_epoch'] * 1e-9
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    od, clouds, tg, ft, lk, mx, lag, K = [], [], [], [], [], [], {}, None
    while r.has_next():
        t, d, ts = r.read_next()
        m = deserialize_message(d, get_message(types[t]))
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 if hasattr(m, 'header') else None
        if st is not None:
            lag.setdefault(t, []).append(ts * 1e-9 - st)
        if t == '/lidar_slam/odom':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            od.append((st, p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        elif t == '/cloud_registered_body':
            clouds.append((st, xyz_view(m)))
        elif t == '/dtp/targets':
            for g in m.targets:
                s = g.header.stamp.sec + g.header.stamp.nanosec * 1e-9
                tg.append((s, ts * 1e-9, g.track_id, g.position.x, g.position.y, g.position.z,
                           g.velocity.x, g.velocity.y, g.confidence, g.missed_frames,
                           float(g.track_confirmed), g.distance_m))
        elif t == '/dtp/follow_target':
            ft.append((ts * 1e-9, st, m.track_id, m.position.x, m.position.y, m.position.z,
                       m.missed_frames, m.confidence))
        elif t == '/dtp/lock_state':
            lk.append((ts * 1e-9, m.data))
        elif t == '/mux/state':
            mx.append((ts * 1e-9, m.data))
        elif t == '/camera/color/camera_info' and K is None:
            K = (m.width, m.height) + tuple(m.k)
    od = np.array(od)
    od_t = od[:, 0]
    pts, offs, sc_t, sc_pos, dropped = [], [0], [], [], 0
    for st, a in clouds:
        i = int(np.searchsorted(od_t, st))
        cand = [j for j in (i - 1, i) if 0 <= j < len(od_t)]
        j = min(cand, key=lambda j: abs(od_t[j] - st))
        if abs(od_t[j] - st) > 0.02:
            dropped += 1
            continue
        R = quat_R(*od[j, 4:8])
        T = od[j, 1:4]
        pts.append((a.astype(np.float64) @ R.T + T).astype(np.float32))
        offs.append(offs[-1] + len(a))
        sc_t.append(st)
        sc_pos.append(T)
    np.savez(out, start=start, od=od, sc_t=np.array(sc_t), sc_off=np.array(offs),
             sc_pts=np.vstack(pts), sc_pos=np.array(sc_pos), sc_dropped=dropped,
             tg=np.array(tg, dtype=np.float64).reshape(-1, 12),
             ft=np.array(ft, dtype=np.float64).reshape(-1, 8),
             lk_t=np.array([x[0] for x in lk]), lk_s=np.array([x[1] for x in lk]),
             mx_t=np.array([x[0] for x in mx]), mx_s=np.array([x[1] for x in mx]),
             K=np.array(K if K else [np.nan] * 11),
             lag_topics=np.array(list(lag.keys())),
             lag_med=np.array([np.median(v) for v in lag.values()]))
    return out


def load(bag, cache_dir=DEFAULT_CACHE):
    return dict(np.load(build(bag, cache_dir), allow_pickle=False))


if __name__ == '__main__':
    p = build(sys.argv[1], *(sys.argv[2:3]))
    c = np.load(p)
    print(p, 'scans', len(c['sc_t']), 'dropped', int(c['sc_dropped']), 'points', len(c['sc_pts']),
          'targets', len(c['tg']))
    for t, v in zip(c['lag_topics'], c['lag_med']):
        print('   receive - stamp  %-28s %+.3f s' % (t, v))
