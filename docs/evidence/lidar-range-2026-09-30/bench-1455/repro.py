"""Re-run LidarBoxRange.range_for_box headless on the bag's own scans for the frames around t 40.7."""
import sqlite3, sys, json, pickle, math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
sys.path.insert(0, '/home/ideas-ad/SUPER-Human-DTP/drone-dtp/ros2_ws/src/dtp_perception')
from dtp_perception import calib_io, lidar_box_range as lbr
BAG = '/home/ideas-ad/SUPER-Human-DTP/bags/2026-09-30_145506/2026-09-30_145506_0.db3'
db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
topics = {r[0]: (r[1], r[2]) for r in db.execute('select id,name,type from topics')}
name2id = {v[0]: k for k, v in topics.items()}
t0 = db.execute('select min(timestamp) from messages').fetchone()[0] / 1e9
def read(name):
    tid = name2id[name]; cls = get_message(topics[tid][1])
    for ts, data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        yield ts / 1e9, deserialize_message(data, cls)
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
T, intr, _ = calib_io.load_calib('/home/ideas-ad/SUPER-Human-DTP/drone-dtp/ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml')
D = pickle.load(open('bag1455.pkl', 'rb'))
K = np.array(D['ci']['K']).reshape(3, 3); Dd = np.array(D['ci']['D'])
scans = [(st(m.header), pc2.read_points_numpy(m, field_names=('x', 'y', 'z')).astype(np.float32)) for _, m in read('/cloud_registered_body')]
odom = [(st(m.header), np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]),
         (m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w)) for _, m in read('/lidar_slam/odom')]
# the detections around the bad frames, with the node's own velocity and forecast
rsd = {}
for j in D['rs']:
    for tr in j['tracks']: rsd[(round(j['t_img'], 3), tr['id'])] = tr
frames = []
for j in D['dets']:
    if j.get('ok') and j.get('dets'):
        for d in j['dets']:
            tb = j['t_img'] - t0
            if 36.0 < tb < 41.0:
                frames.append((j['t_img'], d, rsd.get((round(j['t_img'], 3), d['id']))))
def run(stamp, box, fc, vel, top_ext=45.0, ground=0.25):
    src = lbr.LidarBoxRange(None, T, [-0.011, -0.02329, 0.04412], [1, 0, 0, 0, 1, 0, 0, 0, 1], ground_reject_m=ground, box_top_extend_px=top_ext)
    src.set_intrinsics(K, Dd, (1280, 720))
    for t, p, q in odom:
        if t < stamp + 2.0: src.add_odom(t, p, q)
    for t, P in scans:
        if stamp - 2.5 < t < stamp + 1.0: src.add_scan(t, P)
    return src.range_for_box(box, stamp, fc, vel)
for stamp, d, tr in frames:
    fc = tr.get('fc_r') if tr else None
    fc = float('nan') if fc is None else fc
    vel = d.get('vel')
    s = run(stamp, d['box'], fc, vel if vel and any(vel) else None)
    s0 = run(stamp, d['box'], fc, None)
    print('t %6.2f id %d live src %-6s live r %-6s cam %-6s fc %-6s vel %s | repro r %.2f n %d (box %d gated %d ground %d, win %.1f shifted %s) | unshifted r %.2f n %d' % (
        stamp - t0, d['id'], tr and tr['src'], tr and tr['r'], tr and tr['cam_r'], tr and tr.get('fc_r'), vel, s.range_m if s.valid else float('nan'), s.n_points, s.n_box, s.n_gated, s.n_ground, s.window_s, s.shifted,
        s0.range_m if s0.valid else float('nan'), s0.n_points))
