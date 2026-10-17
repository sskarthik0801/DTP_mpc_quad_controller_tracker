#!/usr/bin/env python3
"""Per-second recorder for the dual-Livox-driver reproduction. Writes CSV to argv[1]."""
import csv, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

out = open(sys.argv[1], 'w', newline='')
w = csv.writer(out)
w.writerow(['wall', 'odom_n', 'x', 'y', 'z', 'cov0', 'imu_n', 'imu_backwards', 'imu_dup',
            'cmsg_n', 'cmsg_pts_mean', 'pub_lidar', 'pub_imu', 'pub_odom'])

rclpy.init()
n = Node('g02_repro_rec')
st = dict(odom_n=0, pos=(float('nan'),) * 3, cov0=float('nan'), imu_n=0, back=0, dup=0,
          last_imu=None, cmsg_n=0, pts=0)


def on_odom(m):
    st['odom_n'] += 1
    p = m.pose.pose.position
    st['pos'] = (p.x, p.y, p.z)
    st['cov0'] = m.pose.covariance[0]


def on_imu(m):
    st['imu_n'] += 1
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if st['last_imu'] is not None:
        if t < st['last_imu']:
            st['back'] += 1
        elif t == st['last_imu']:
            st['dup'] += 1
    st['last_imu'] = t


def on_cmsg(m):
    st['cmsg_n'] += 1
    st['pts'] += m.point_num


import struct
def on_imu_raw(b):
    sec, nsec = struct.unpack_from('<iI', b, 4)
    class M: pass
    m = M(); m.header = M(); m.header.stamp = M(); m.header.stamp.sec = sec; m.header.stamp.nanosec = nsec
    on_imu(m)
def on_cmsg_raw(b):
    st['cmsg_n'] += 1
    st['pts'] += max(0, (len(b) - 40)) // 18   # CustomPoint serialises to ~18 bytes


n.create_subscription(Odometry, '/lidar_slam/odom', on_odom, 50)
n.create_subscription(Imu, '/livox/imu', on_imu_raw, qos_profile_sensor_data, raw=True)
n.create_subscription(CustomMsg, '/livox/lidar', on_cmsg_raw, qos_profile_sensor_data, raw=True)

t_next = time.time() + 1.0
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        if time.time() >= t_next:
            t_next += 1.0
            w.writerow(['%.2f' % time.time(), st['odom_n'], *('%.3f' % v for v in st['pos']),
                        '%.3g' % st['cov0'], st['imu_n'], st['back'], st['dup'], st['cmsg_n'],
                        '%.0f' % (st['pts'] / st['cmsg_n']) if st['cmsg_n'] else 0,
                        len(n.get_publishers_info_by_topic('/livox/lidar')),
                        len(n.get_publishers_info_by_topic('/livox/imu')),
                        len(n.get_publishers_info_by_topic('/lidar_slam/odom'))])
            out.flush()
            for k in ('odom_n', 'imu_n', 'back', 'dup', 'cmsg_n', 'pts'):
                st[k] = 0
except KeyboardInterrupt:
    pass
