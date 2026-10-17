"""Extract a static calibration viewpoint from a bag: median depth, one colour frame, the accumulated
LiDAR cloud (LiDAR frame), intrinsics, the driver's depth->colour TF, PX4 attitude and Livox gravity."""
import sqlite3, sys, glob
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot

bagdir, out = sys.argv[1], sys.argv[2]
db = sqlite3.connect('file:%s?mode=ro' % glob.glob(bagdir + '/*.db3')[0], uri=True)
topics = {r[1]: (r[0], r[2]) for r in db.execute('select id,name,type from topics')}

def rows(name):
    tid, typ = topics[name]; cls = get_message(typ)
    for ts, data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        yield ts, deserialize_message(data, cls)

ci = next(rows('/camera/color/camera_info'))[1]
di = next(rows('/camera/depth/camera_info'))[1]
print('colour', ci.width, ci.height, ci.header.frame_id, np.round(ci.k, 2)[[0, 2, 4, 5]])
print('depth ', di.width, di.height, di.header.frame_id, np.round(di.k, 2)[[0, 2, 4, 5]], di.distortion_model, np.round(di.d, 4))

# tf_static: build a little tree and compose depth optical -> colour optical
edges = {}
for _, m in rows('/tf_static'):
    for t in m.transforms:
        q = t.transform.rotation; p = t.transform.translation
        T = np.eye(4); T[:3, :3] = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix(); T[:3, 3] = [p.x, p.y, p.z]
        edges[t.child_frame_id] = (t.header.frame_id, T)
        print('tf', t.header.frame_id, '->', t.child_frame_id, np.round([p.x, p.y, p.z], 4), np.round(Rot.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True), 3))

def to_root(f):
    T = np.eye(4)
    while f in edges:
        par, Tp = edges[f]; T = Tp @ T; f = par
    return f, T
rc, T_root_col = to_root(ci.header.frame_id)
rd, T_root_dep = to_root(di.header.frame_id)
assert rc == rd, (rc, rd)
T_col_dep = np.linalg.inv(T_root_col) @ T_root_dep
print('T_colour_depth t', T_col_dep[:3, 3].round(4), 'rot deg', Rot.from_matrix(T_col_dep[:3, :3]).as_rotvec(degrees=True).round(3))

img = None
for i, (ts, m) in enumerate(rows('/camera/color/image_raw')):
    if i == 300:
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1).copy(); enc = m.encoding; break
deps = []
for i, (ts, m) in enumerate(rows('/camera/depth/image_raw')):
    if i % 5 == 0:
        deps.append(np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).copy())
D = np.stack(deps).astype(np.float32); D[D == 0] = np.nan
valid_frac = np.mean(~np.isnan(D), 0)
dmed = np.nanmedian(D, 0); dmed[valid_frac < 0.6] = np.nan
dstd = np.nanstd(D, 0)
print('depth frames', len(deps), 'valid px', np.mean(~np.isnan(dmed)).round(3))

pts = []
for ts, m in rows('/livox/lidar'):
    off = {f.name: f.offset for f in m.fields}
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
    xyz = np.stack([a[:, off[k]:off[k] + 4].copy().view('<f4')[:, 0] for k in 'xyz'], 1)
    it = a[:, off['intensity']:off['intensity'] + 4].copy().view('<f4')[:, 0] if 'intensity' in off else np.zeros(len(a), np.float32)
    pts.append(np.c_[xyz, it])
cloud = np.vstack(pts); r = np.linalg.norm(cloud[:, :3], axis=1); cloud = cloud[(r > 0.15) & np.isfinite(r)]
print('lidar sweeps', len(pts), 'points', len(cloud), 'frame', m.header.frame_id)

q = np.array([[*m.q] for _, m in rows('/fmu/out/vehicle_attitude')])  # PX4 w,x,y,z, FRD->NED
qm = q[len(q) // 2]
eul = Rot.from_quat([qm[1], qm[2], qm[3], qm[0]]).as_euler('ZYX', degrees=True)
print('PX4 attitude yaw %.2f pitch %.3f roll %.3f (deg, FRD/NED)' % tuple(eul))
acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in rows('/livox/imu')]).mean(0)
print('Livox IMU mean accel', acc.round(4), '|a|', np.linalg.norm(acc).round(4))

np.savez_compressed(out, image=img, encoding=enc, depth_med_mm=dmed, depth_std_mm=dstd,
                    K_col=np.array(ci.k).reshape(3, 3), D_col=np.array(ci.d), K_dep=np.array(di.k).reshape(3, 3),
                    D_dep=np.array(di.d), T_col_dep=T_col_dep, cloud=cloud, px4_q=qm, livox_acc=acc,
                    col_wh=[ci.width, ci.height], dep_wh=[di.width, di.height])
print('wrote', out)
