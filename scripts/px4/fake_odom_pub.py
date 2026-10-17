#!/usr/bin/env python3
"""Bench test fixture: publish synthetic nav_msgs/Odometry on /lidar_slam/odom
so px4_odom_bridge can be exercised without the LiDAR or FAST-LIO running.

Traces a slow circle so you can see the values move on the FC side.

SAFETY: only meaningful to run while EKF2_EV_CTRL=0, in which case PX4 receives
the odometry but does not fuse it. Once external vision is enabled this fixture
would be feeding EKF2 fabricated position -- do not run it then.

    python3 scripts/px4/fake_odom_pub.py [--rate 30] [--radius 1.0]
"""
import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FakeOdom(Node):

    def __init__(self, rate, radius, period):
        super().__init__('fake_odom_pub')
        self.pub = self.create_publisher(Odometry, '/lidar_slam/odom', 10)
        self.radius = radius
        self.period = period
        self.t = 0.0
        self.dt = 1.0 / rate
        self.n = 0
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f'publishing synthetic /lidar_slam/odom at {rate} Hz '
            f'(circle r={radius} m, period={period} s)')

    def tick(self):
        self.t += self.dt
        ang = 2.0 * math.pi * self.t / self.period

        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'lidar_init'   # upstream FAST-LIO: camera_init
        m.child_frame_id = 'lidar_imu'     # upstream FAST-LIO: body (the MID360 IMU)

        m.pose.pose.position.x = self.radius * math.cos(ang)
        m.pose.pose.position.y = self.radius * math.sin(ang)
        m.pose.pose.position.z = 0.5

        # yaw tangent to the circle
        yaw = ang + math.pi / 2.0
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        m.pose.pose.orientation.x = 0.0
        m.pose.pose.orientation.y = 0.0
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)

        v = 2.0 * math.pi * self.radius / self.period
        m.twist.twist.linear.x = -v * math.sin(ang)
        m.twist.twist.linear.y = v * math.cos(ang)

        self.pub.publish(m)
        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info(f'published {self.n}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--radius', type=float, default=1.0)
    ap.add_argument('--period', type=float, default=20.0)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = FakeOdom(args.rate, args.radius, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
