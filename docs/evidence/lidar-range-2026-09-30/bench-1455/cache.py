"""Cache the latest bag's LiDAR scans (lidar_init), detections, range_source, targets. Read-only."""
import sqlite3, sys, json, pickle
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
BAG = sys.argv[1]; OUT = sys.argv[2]
db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
topics = {r[0]: (r[1], r[2]) for r in db.execute('select id,name,type from topics')}
name2id = {v[0]: k for k, v in topics.items()}
t0 = db.execute('select min(timestamp) from messages').fetchone()[0] / 1e9
def read(name):
    tid = name2id[name]; cls = get_message(topics[tid][1])
    for ts, data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        yield ts / 1e9, deserialize_message(data, cls)
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
scans = []
for tr, m in read('/cloud_registered'):
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z')).astype(np.float32)
    scans.append((st(m.header), a))
dets = []
for tr, m in read('/dtp/detections_2d'):
    j = json.loads(m.data); j['t_rx'] = tr
    j['t_img'] = j['stamp'][0] + j['stamp'][1] * 1e-9 if j.get('stamp') else None
    dets.append(j)
rs = []
for tr, m in read('/dtp/range_source'):
    j = json.loads(m.data); j['t_rx'] = tr; j['t_img'] = j['stamp'][0] + j['stamp'][1] * 1e-9
    rs.append(j)
tg = []
for tr, m in read('/dtp/targets'):
    for g in m.targets:
        tg.append(dict(t_rx=tr, t_img=st(m.header), id=g.track_id, x=g.position.x, y=g.position.y, z=g.position.z,
                       dist=g.distance_m, conf=g.confidence, confirmed=g.track_confirmed, missed=g.missed_frames))
odom = [(st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z) for tr, m in read('/lidar_slam/odom')]
ci = None
for tr, m in read('/camera/color/camera_info'):
    ci = dict(K=list(m.k), D=list(m.d), w=m.width, h=m.height, model=m.distortion_model); break
tfs = []
for tr, m in read('/tf_static'):
    for x in m.transforms:
        tfs.append((x.header.frame_id, x.child_frame_id, [x.transform.translation.x, x.transform.translation.y, x.transform.translation.z],
                    [x.transform.rotation.x, x.transform.rotation.y, x.transform.rotation.z, x.transform.rotation.w]))
tf_dyn = []
for tr, m in read('/tf'):
    for x in m.transforms:
        if x.child_frame_id == 'lidar_imu':
            tf_dyn.append((st(x.header), [x.transform.translation.x, x.transform.translation.y, x.transform.translation.z],
                           [x.transform.rotation.x, x.transform.rotation.y, x.transform.rotation.z, x.transform.rotation.w]))
pickle.dump(dict(t0=t0, scans=scans, dets=dets, rs=rs, tg=tg, odom=odom, ci=ci, tfs=tfs, tf_dyn=tf_dyn), open(OUT, 'wb'))
print('scans', len(scans), 'dets', len(dets), 'rs', len(rs), 'tg', len(tg), 'ci', ci and ci['w'], ci and ci['h'])
