"""How many FAST-LIO body-frame points land on the walker, by range: from the 09-29 bag."""
import sys, os, bisect, math, numpy as np, rosbag2_py, yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
bag = sys.argv[1]
start = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch'] * 1e-9
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_slam/odom', '/cloud_registered_body', '/dtp/follow_target', '/dtp/lock_state']))
GX = [-2, 10, 20, 28, 36, 40, 44, 48]; GZ = [-0.13, 0.14, 0.5, 0.8, 1.1, 1.3, 1.5, 1.7]
ground = lambda x: np.interp(x, GX, GZ)
od_t, od, scans, tgt, lock = [], [], [], [], []
while r.has_next():
    t, d, ts = r.read_next(); tb = ts * 1e-9 - start
    if tb < 438 or tb > 505: continue
    m = deserialize_message(d, get_message(types[t]))
    if t == '/lidar_slam/odom':
        p = m.pose.pose.position; q = m.pose.pose.orientation
        od_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); od.append((p.x, p.y, p.z, q.w, q.x, q.y, q.z))
    elif t == '/cloud_registered_body':
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        scans.append((tb, st, pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)))
    elif t == '/dtp/follow_target':
        tgt.append((tb, m.position.x, m.position.y))
    else:
        lock.append((tb, m.data.split('|')[0]))
def pose(st):
    i = bisect.bisect_left(od_t, st)
    if i >= len(od_t) or abs(od_t[i] - st) > 0.02: return None
    x, y, z, w, qx, qy, qz = od[i]
    R = np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-w*qz), 2*(qx*qz+w*qy)], [2*(qx*qy+w*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-w*qx)], [2*(qx*qz-w*qy), 2*(qy*qz+w*qx), 1-2*(qx*qx+qy*qy)]])
    return R, np.array([x, y, z])
W = []
for tb, st, a in scans:
    ps = pose(st)
    if ps is None: continue
    R, T = ps; w = a @ R.T + T
    W.append((tb, T, w))
tt = [x[0] for x in tgt]; lt = [x[0] for x in lock]
def locked(tb):
    i = bisect.bisect_right(lt, tb) - 1
    return i >= 0 and lock[i][1] == 'LOCKED'
# (a) tracked walker: points within 0.6 m horizontally of the camera-derived position, 0.3-2.1 m above ground
print('(a) walker located by the camera track (LOCKED only), per 0.1 s scan (FAST-LIO keeps 1 point in 3)')
rows = []
for tb, T, w in W:
    i = bisect.bisect_right(tt, tb) - 1
    if i < 0 or tb - tgt[i][0] > 0.15 or not locked(tb): continue
    px, py = tgt[i][1], tgt[i][2]
    g = ground(px)
    dh = np.hypot(w[:, 0] - px, w[:, 1] - py)
    m = (dh < 0.6) & (w[:, 2] > g + 0.3) & (w[:, 2] < g + 2.1)
    rng = math.hypot(px - T[0], py - T[1])
    hts = w[m, 2] - g
    rows.append((rng, int(m.sum()), hts, T[2] - ground(T[0])))
for lo, hi in [(3, 5), (5, 7), (7, 9), (9, 11)]:
    sel = [x for x in rows if lo <= x[0] < hi]
    if not sel: continue
    n = np.array([x[1] for x in sel]); h = np.concatenate([x[2] for x in sel]) if n.sum() else np.array([np.nan])
    agl = np.median([x[3] for x in sel])
    print(f'  {lo:2d}-{hi:2d} m: scans {len(sel):3d}  pts/scan p10 {np.percentile(n,10):.0f} p50 {np.median(n):.0f} p90 {np.percentile(n,90):.0f}  zero-scans {np.mean(n==0)*100:.0f} %  | body heights hit p10/p50/p90 {np.nanpercentile(h,10):.2f}/{np.nanpercentile(h,50):.2f}/{np.nanpercentile(h,90):.2f} m | aircraft {agl:.1f} m AGL')
# (b) walker NOT tracked (t 443.5-453.5): find the cluster in the corridor strip beyond 8 m
print('\n(b) t 443.6-453.4, walker not tracked: points 0.3-2.1 m above ground in x 8-22, y -2.0..-0.4 (the walker\'s lane), per scan')
for tb, T, w in W:
    if not (443.6 <= tb <= 453.4): continue
    g = ground(w[:, 0])
    m = (w[:, 0] > 8) & (w[:, 0] < 22) & (w[:, 1] > -2.0) & (w[:, 1] < -0.4) & (w[:, 2] > g + 0.3) & (w[:, 2] < g + 2.1)
    if round(tb * 10) % 5 == 0 or m.sum():
        c = w[m]
        if len(c):
            cx, cy = np.median(c[:, 0]), np.median(c[:, 1]); rng = math.hypot(cx - T[0], cy - T[1])
            print(f'  t {tb:6.1f}: {int(m.sum()):3d} pts  centre ({cx:5.1f},{cy:5.1f})  range {rng:5.1f} m  heights {np.min(c[:,2]-ground(c[:,0])):.2f}-{np.max(c[:,2]-ground(c[:,0])):.2f}')
        else:
            print(f'  t {tb:6.1f}:   0 pts')
