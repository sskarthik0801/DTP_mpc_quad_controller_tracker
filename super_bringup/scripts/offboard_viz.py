#!/usr/bin/env python3
"""Make the Offboard hold visible in RViz on a laptop (flight-test phase P5).

Subscribes  /fmu/in/trajectory_setpoint         (px4_msgs, NED)  -- what we command
            /fmu/out/vehicle_local_position_v1  (px4_msgs, NED)  -- what EKF2 believes
Publishes   /viz/offboard  (visualization_msgs/MarkerArray, `world` frame, FLU)

Why this node exists
--------------------
`scripts/laptop_viewer.sh` deliberately requires **no colcon build on the laptop**: every
topic it shows (`/cloud_registered`, `/rog_map/*`, `/lidar_slam/odom`, `/tf`) is a standard
ROS 2 type. The two topics that matter for P5 are neither standard nor viewable:
`TrajectorySetpoint` and `VehicleLocalPosition` are `px4_msgs` types, so a laptop without
`px4_msgs` built cannot even deserialise them, and RViz has no display for either.

So this runs on the **Jetson** (where `px4_msgs` exists), does the NED->FLU flip, and emits
plain markers. The laptop stays build-free.

What you see
------------
    cyan sphere    the commanded setpoint
    orange sphere  where EKF2 thinks the vehicle is
    line + text    the gap between them, in centimetres

During P5 with SUPER idle the bridge is in HOLD, so the cyan sphere should be **nailed in
place** and the orange one should sit on top of it. A growing line is the failure this phase
exists to catch, and it is readable from across a field.

**Only in Offboard (2026-09-28).** The bridge now sends setpoints only while PX4 reports
Offboard, so outside it there is no cyan sphere, line or text: just the orange estimate.
A setpoint older than `setpoint_stale_s` is removed, or the last Offboard point would stay
drawn after a takeover and the line would grow red on a healthy aircraft.

Frame note
----------
NED -> FLU is the same `(x, -y, -z)` flip the bridges use in the other direction (it is its
own inverse). The two spheres are ~7 cm apart even when perfect: EKF2 reports the **FC body
origin** while FAST-LIO tracks the **MID360's IMU**, and `EKF2_EV_POS_*` carries that lever
arm. Do not read a constant few-cm offset as drift.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Point
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from visualization_msgs.msg import Marker, MarkerArray


def ned_to_flu(x, y, z):
    return (float(x), float(-y), float(-z))


def _finite(*v):
    return all(math.isfinite(c) for c in v)


class OffboardViz(Node):

    def __init__(self):
        super().__init__('offboard_viz')
        self.declare_parameter('setpoint_topic', '/fmu/in/trajectory_setpoint')
        self.declare_parameter('local_position_topic',
                               '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('output_topic', '/viz/offboard')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_rate', 10.0)
        # The bridge streams at 50 Hz while in Offboard; 0.5 s of silence means it stopped.
        self.declare_parameter('setpoint_stale_s', 0.5)

        sp_topic = self.get_parameter('setpoint_topic').value
        lp_topic = self.get_parameter('local_position_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.frame = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate').value)
        self.sp_stale = float(self.get_parameter('setpoint_stale_s').value)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

        self.sp = None
        self.sp_t = 0.0           # time.monotonic() of the last setpoint
        self.est = None
        self.create_subscription(TrajectorySetpoint, sp_topic, self.on_sp, px4_qos)
        self.create_subscription(VehicleLocalPosition, lp_topic, self.on_est, px4_qos)
        # RViz subscribers come and go across the Wi-Fi; TRANSIENT_LOCAL means a laptop
        # that connects late still gets the last frame instead of an empty display.
        self.pub = self.create_publisher(
            MarkerArray, out_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'offboard_viz: {sp_topic} + {lp_topic} -> {out_topic} '
            f'(NED->FLU, frame={self.frame}) @ {rate:.0f} Hz')

    def on_sp(self, m):
        if _finite(*m.position):
            self.sp = ned_to_flu(*m.position)
            self.sp_t = time.monotonic()

    def on_est(self, m):
        if m.xy_valid and m.z_valid and _finite(m.x, m.y, m.z):
            self.est = ned_to_flu(m.x, m.y, m.z)

    def _marker(self, mid, mtype, scale, rgba):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'offboard'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def tick(self):
        arr = MarkerArray()

        if self.sp is not None and time.monotonic() - self.sp_t > self.sp_stale:
            # Out of Offboard (the bridge stopped): take the setpoint, line and text down.
            self.sp = None
            for mid in (0, 2, 3):
                d = self._marker(mid, Marker.SPHERE, (0.1, 0.1, 0.1), (0.0, 0.0, 0.0, 0.0))
                d.action = Marker.DELETE
                arr.markers.append(d)

        if self.sp is not None:
            s = self._marker(0, Marker.SPHERE, (0.22, 0.22, 0.22), (0.20, 0.85, 0.90, 0.95))
            s.pose.position.x, s.pose.position.y, s.pose.position.z = self.sp
            arr.markers.append(s)

        if self.est is not None:
            e = self._marker(1, Marker.SPHERE, (0.18, 0.18, 0.18), (1.0, 0.55, 0.15, 0.95))
            e.pose.position.x, e.pose.position.y, e.pose.position.z = self.est
            arr.markers.append(e)

        if self.sp is not None and self.est is not None:
            d = math.dist(self.sp, self.est)
            # Green while the hold is tight, amber past 20 cm, red past 50 cm -- the same
            # semantics as the test card, so a glance from across the field is enough.
            rgba = (0.30, 0.80, 0.45, 0.9) if d < 0.20 else \
                   (0.95, 0.70, 0.20, 0.9) if d < 0.50 else (0.95, 0.30, 0.30, 0.95)

            line = self._marker(2, Marker.LINE_STRIP, (0.03, 0.0, 0.0), rgba)
            line.points = [Point(x=self.sp[0], y=self.sp[1], z=self.sp[2]),
                           Point(x=self.est[0], y=self.est[1], z=self.est[2])]
            arr.markers.append(line)

            txt = self._marker(3, Marker.TEXT_VIEW_FACING, (0.0, 0.0, 0.22), rgba)
            txt.pose.position.x = (self.sp[0] + self.est[0]) / 2.0
            txt.pose.position.y = (self.sp[1] + self.est[1]) / 2.0
            txt.pose.position.z = (self.sp[2] + self.est[2]) / 2.0 + 0.25
            txt.text = f'{d * 100:.0f} cm'
            arr.markers.append(txt)

        if arr.markers:
            self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardViz()
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
