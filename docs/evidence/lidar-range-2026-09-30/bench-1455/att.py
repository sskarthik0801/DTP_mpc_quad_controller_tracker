import sqlite3, sys, math, pickle
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
BAG = sys.argv[1]
db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
topics = {r[0]: (r[1], r[2]) for r in db.execute('select id,name,type from topics')}
name2id = {v[0]: k for k, v in topics.items()}
def rows(name):
    tid = name2id[name]; cls = get_message(topics[tid][1])
    for ts, data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        yield ts, deserialize_message(data, cls)
q = np.array([m.q for _, m in rows('/fmu/out/vehicle_attitude')])     # w x y z, FRD body -> NED
eul = R.from_quat(q[:, [1, 2, 3, 0]]).as_euler('ZYX', degrees=True)
print('PX4 attitude: roll %.2f pitch %.2f (deg, median), yaw %.1f' % (np.median(eul[:, 2]), np.median(eul[:, 1]), np.median(eul[:, 0])))
o = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w] for _, m in rows('/lidar_slam/odom')])
e2 = R.from_quat(o).as_euler('ZYX', degrees=True)
print('FAST-LIO odom (lidar_init<-body): roll %.2f pitch %.2f yaw %.2f' % tuple(np.median(e2[:, ::-1], 0)))
# TF chain lidar_init -> camera_depth_optical_frame
D = pickle.load(open('bag1455.pkl', 'rb'))
tf = {(p, c): (np.array(t), np.array(qq)) for p, c, t, qq in D['tfs']}
def T(tq):
    M = np.eye(4); M[:3, :3] = R.from_quat(tq[1]).as_matrix(); M[:3, 3] = tq[0]; return M
_, t_li, q_li = D['tf_dyn'][len(D['tf_dyn']) // 2]
M = T((np.array(t_li), np.array(q_li)))
for pc in [('lidar_imu', 'camera_link'), ('camera_link', 'camera_depth_frame'), ('camera_depth_frame', 'camera_depth_optical_frame')]:
    M = M @ T(tf[pc])
print('depth optical centre in lidar_init', M[:3, 3].round(3))
ax = M[:3, :3] @ np.array([0, 0, 1.0]); print('optical axis in lidar_init', ax.round(4), 'elevation %.2f deg' % np.degrees(np.arcsin(ax[2])))
Nf = np.load('floor_normal_depthopt.npy'); n_opt, h = Nf[:3], Nf[3]
n_li = M[:3, :3] @ n_opt
print('camera-measured floor up-normal in lidar_init', n_li.round(4), 'tilt from lidar_init z %.2f deg (pitch-ish %.2f, roll-ish %.2f)' % (np.degrees(np.arccos(n_li[2])), np.degrees(np.arctan2(n_li[0], n_li[2])), np.degrees(np.arctan2(n_li[1], n_li[2]))))
# floor plane in lidar_init: points p with n_li.(p - c) = -h, c = camera centre
c = M[:3, 3]
print('floor z at the origin (camera-derived plane) %.3f; directly below the camera %.3f' % (c[2] - (h + n_li[:2] @ (np.zeros(2) - c[:2])) / n_li[2], c[2] - h / n_li[2]))
np.save('floor_plane_li.npy', np.r_[n_li, -(n_li @ c) - h])
