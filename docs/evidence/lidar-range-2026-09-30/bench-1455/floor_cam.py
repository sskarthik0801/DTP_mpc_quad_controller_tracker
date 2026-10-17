"""Floor plane from the Orbbec depth image, in the camera's own frame: camera height above the floor
and the floor normal, independent of any camera<->LiDAR extrinsic."""
import sqlite3, sys, json
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG = sys.argv[1]
db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
topics = {r[0]: (r[1], r[2]) for r in db.execute('select id,name,type from topics')}
name2id = {v[0]: k for k, v in topics.items()}
def rows(name, lim=None):
    tid = name2id[name]; cls = get_message(topics[tid][1])
    q = 'select timestamp,data from messages where topic_id=? order by timestamp'
    for ts, data in db.execute(q, (tid,)):
        yield ts, deserialize_message(data, cls)
ci = next(rows('/camera/depth/camera_info'))[1]
K = np.array(ci.k).reshape(3, 3); print('depth K', K.round(1).tolist(), ci.width, ci.height, ci.distortion_model, np.round(ci.d, 4).tolist(), ci.header.frame_id)
res = []
for i, (ts, m) in enumerate(rows('/camera/depth/image_raw')):
    if i % 40: continue
    d = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).astype(np.float64) / 1000.0
    v, u = np.mgrid[0:m.height:4, 0:m.width:4]
    z = d[::4, ::4]
    sel = (z > 0.3) & (z < 6.0) & (v > m.height * 0.62)       # bottom part of the image: floor
    x = (u[sel] - K[0, 2]) * z[sel] / K[0, 0]; y = (v[sel] - K[1, 2]) * z[sel] / K[1, 1]
    Q = np.stack([x, y, z[sel]], 1)
    best = None; rng = np.random.default_rng(0)
    for _ in range(300):
        s = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9: continue
        n /= nn; dd = -n @ s[0]
        inl = np.abs(Q @ n + dd) < 0.03
        if best is None or inl.sum() > best[0]: best = (inl.sum(), n, dd, inl)
    cnt, n, dd, inl = best
    # refine by least squares
    A = Q[inl]; c = A.mean(0); _, _, vt = np.linalg.svd(A - c); n = vt[2]; dd = -n @ c
    if n[1] > 0: n, dd = -n, -dd          # normal pointing up = -y in optical (y is down)
    h = abs(dd)                            # distance from the optical centre to the plane
    pitch = np.degrees(np.arctan2(n[2], -n[1]))   # >0: optical axis tilted toward... sign checked below
    res.append((h, pitch, cnt / len(Q), n))
    print('frame %3d  inliers %.2f  cam height above floor %.3f m  floor normal (optical) %s' % (i, cnt / len(Q), h, n.round(4)))
H = np.array([r[0] for r in res]); print('camera height above floor: median %.3f  p10 %.3f p90 %.3f' % (np.median(H), *np.percentile(H, [10, 90])))
N = np.median(np.array([r[3] for r in res]), 0); N /= np.linalg.norm(N); print('median up-normal in depth optical frame', N.round(4), ' axis elevation vs floor plane %.2f deg' % np.degrees(np.arcsin(N[2])))
np.save('floor_normal_depthopt.npy', np.r_[N, np.median(H)])
