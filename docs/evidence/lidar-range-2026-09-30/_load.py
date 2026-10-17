"""Shared by corridor.py and ground.py: every 4th /cloud_registered_body scan (every 2nd point),
odom-transformed into lidar_init. Returns (points Nx3, odom times, odom poses)."""
import bisect

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2

EVERY = 4


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_slam/odom', '/cloud_registered_body']))
    od_t, od, clouds, n = [], [], [], 0
    while r.has_next():
        t, d, ts = r.read_next()
        m = deserialize_message(d, get_message(types[t]))
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t == '/lidar_slam/odom':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            od_t.append(st)
            od.append((p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        else:
            n += 1
            if n % EVERY:
                continue
            a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
            clouds.append((st, a[::2].astype(np.float64)))
    pts = []
    for st, a in clouds:
        i = bisect.bisect_left(od_t, st)
        if i >= len(od_t) or abs(od_t[i] - st) > 0.02:
            continue
        x, y, z, w, qx, qy, qz = od[i]
        R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - w * qz), 2 * (qx * qz + w * qy)],
                      [2 * (qx * qy + w * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - w * qx)],
                      [2 * (qx * qz - w * qy), 2 * (qy * qz + w * qx), 1 - 2 * (qx * qx + qy * qy)]])
        pts.append(a @ R.T + np.array([x, y, z]))
    return np.vstack(pts), od_t, od
