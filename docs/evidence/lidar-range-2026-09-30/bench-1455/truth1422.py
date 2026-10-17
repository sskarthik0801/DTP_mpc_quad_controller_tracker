import sqlite3, json, glob, math, collections
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
BAG = glob.glob('/home/ideas-ad/dtp_datasets/20260930_1422_stageA_range/*.db3')[0]
db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
topics = {r[0]: (r[1], r[2]) for r in db.execute('select id,name,type from topics')}
name2id = {v[0]: k for k, v in topics.items()}
t0 = db.execute('select min(timestamp) from messages').fetchone()[0] / 1e9
def read(name, every=1):
    tid = name2id[name]; cls = get_message(topics[tid][1])
    for i, (ts, data) in enumerate(db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,))):
        if i % every == 0: yield ts / 1e9, deserialize_message(data, cls)
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
od = [(st(m.header), [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z], [m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w]) for _, m in read('/lidar_slam/odom')]
ot = np.array([o[0] for o in od]); P_ = np.array([o[1] for o in od])
print('odom spread (m):', np.ptp(P_, 0).round(3))
def to_world(t, P):
    i = min(np.searchsorted(ot, t), len(ot) - 1)
    return P @ R.from_quat(od[i][2]).as_matrix().T + np.array(od[i][1])
scans = []
for _, m in read('/cloud_registered_body'):
    t = st(m.header); a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z')).astype(np.float32)
    scans.append((t, to_world(t, a).astype(np.float32)))
T = np.concatenate([np.full(len(a), t) for t, a in scans]); P = np.concatenate([a for _, a in scans])
V = np.floor(P / 0.2).astype(np.int64); B = np.floor((T - T.min()) / 2.0).astype(np.int64); nb = B.max() + 1
key = (V[:, 0] * 100003 + V[:, 1]) * 100003 + V[:, 2]
kb = np.unique(key * 1000 + B); kk, cnt = np.unique(kb // 1000, return_counts=True)
occ = dict(zip(kk.tolist(), cnt.tolist())); nbins = np.array([occ[x] for x in key.tolist()])
dyn = nbins < 0.25 * nb
Pd, Td = P[dyn], T[dyn]
print('scans', len(scans), 'dynamic frac %.3f' % dyn.mean())
tg = {}
for _, m in read('/dtp/targets'):
    for g in m.targets: tg[(round(st(m.header), 2), g.track_id)] = (g.position.x, g.position.y)
tid = name2id['/dtp/range_source']; out = []
for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
    j = json.loads(deserialize_message(d, get_message(topics[tid][1])).data)
    s = j['stamp'][0] + j['stamp'][1] * 1e-9
    for tr in j['tracks']:
        if tr['src'] != 'lidar' or tr.get('cam_r') is None: continue
        xy = tg.get((round(s, 2), tr['id']))
        if xy is None: continue
        b = math.atan2(xy[1], xy[0]); w = np.abs(Td - s) < 0.3; Q = Pd[w]
        az = np.arctan2(Q[:, 1], Q[:, 0]); rr = np.hypot(Q[:, 0], Q[:, 1])
        m = (np.abs(np.angle(np.exp(1j * (az - b)))) < math.radians(5)) & (rr > 0.8) & (rr < 14) & (Q[:, 2] > -0.8) & (Q[:, 2] < 1.2)
        truth = None
        if m.sum() >= 3:
            rs = np.sort(rr[m]); best = None
            for i in range(len(rs)):
                k = np.searchsorted(rs, rs[i] + 0.6)
                if best is None or k - i > best[0] + 2: best = (k - i, rs[i:k])
            if best[0] >= 3: truth = float(np.median(best[1]))
        out.append((s - t0, tr['id'], tr['r'], tr['cam_r'], truth))
a = [(t, i, l, c, tr) for t, i, l, c, tr in out if tr is not None]
print('LiDAR-sourced frames with a truth: %d of %d' % (len(a), len(out)))
far = [x for x in a if x[2] - x[4] > 1.0]; camok = [x for x in far if abs(x[3] - x[4]) < 0.6]
print('LiDAR > truth + 1 m (FAR): %d; of those the camera was within 0.6 m of truth: %d' % (len(far), len(camok)))
for x in far[:30]: print('  t %6.1f id %d  lidar %.2f  cam %.2f  truth %.2f' % x)
good = [x for x in a if abs(x[2] - x[4]) <= 0.3]
print('LiDAR within 0.3 m of truth: %d of %d' % (len(good), len(a)))
