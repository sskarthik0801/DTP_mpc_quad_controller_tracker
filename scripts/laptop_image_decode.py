#!/usr/bin/env python3
"""Decode a CompressedImage from the Jetson into a raw Image on THIS laptop, for RViz.

  python3 scripts/laptop_image_decode.py [in_topic] [out_topic]
  defaults: /dtp/image_annotated/compressed  ->  /dtp/image_annotated/laptop

laptop_viewer.sh starts this for you with DTP=1. No build: plain rclpy, standard messages.

WHY IT EXISTS
  RViz on Humble has no way to show a sensor_msgs/CompressedImage, and a raw 720p frame is 2.7 MB,
  which field Wi-Fi will not carry. So the Jetson sends a JPEG (~50-80 kB, dtp_image_overlay) and
  this turns it into a raw Image here, where the raw frame never touches the network.
  `ros2 run image_transport republish` would be the textbook tool, but on Humble it subscribes
  RELIABLE with no override, which never matches a best-effort image publisher -- it would sit
  there receiving nothing, without an error -- and it needs compressed_image_transport, which
  ros-humble-desktop does not install.

THE ONE TRAP: THE RAW IMAGE MUST NOT GO BACK OVER THE WI-FI
  This laptop shares a DDS domain with the Jetson, so the raw topic published here is visible
  there too. Nothing on the Jetson normally subscribes to it -- but `ros2 bag record -a` there
  (the dashboard's "All topics" recording is exactly that) would, and would then pull ~12 MB/s
  back across the same Wi-Fi the flight stack's telemetry uses. So frames are only published
  while every subscriber is an RViz or rqt node; anything else pauses output and says so.
  --any-subscriber turns that off.
"""
import argparse
import array
import io
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
    import numpy as np
except ImportError:             # PIL comes with ros-humble-desktop (via matplotlib)
    cv2 = None
    try:
        from PIL import Image as PILImage
    except ImportError:
        PILImage = None

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=1)
VIEWER_PREFIXES = ('rviz', 'rqt')


class Decoder(Node):

    def __init__(self, in_topic, out_topic, any_subscriber):
        super().__init__('laptop_image_decode')
        self._out_topic = out_topic
        self._any = any_subscriber
        self._pub = self.create_publisher(Image, out_topic, QOS)
        self.create_subscription(CompressedImage, in_topic, self._on_image, QOS)
        self._paused_for = None
        self._frames = 0
        self._check_every = 0
        self._ok = False
        self.get_logger().info('%s -> %s (%s)' % (in_topic, out_topic,
                                                  'OpenCV' if cv2 else 'PIL'))

    def _viewers_only(self):
        """True while every subscriber to the raw output is RViz/rqt (see the module docstring)."""
        if self._any:
            return True
        infos = self.get_subscriptions_info_by_topic(self._out_topic)
        others = sorted({i.node_name for i in infos
                         if not i.node_name.startswith(VIEWER_PREFIXES)})
        if others != (self._paused_for or []):
            if others:
                self.get_logger().warn(
                    'NOT publishing: %s subscribes to %s and is not RViz/rqt -- it may be on the '
                    'Jetson, and raw frames would cross the Wi-Fi (e.g. `ros2 bag record -a`). '
                    'Output resumes when it leaves; --any-subscriber overrides.'
                    % (', '.join(others), self._out_topic))
            elif self._paused_for:
                self.get_logger().info('only viewers subscribe again -- publishing')
            self._paused_for = others
        return not others

    def _on_image(self, msg):
        # Re-checked every few frames, not every frame: a graph query is not free.
        self._check_every -= 1
        if self._check_every <= 0:
            self._check_every = 8
            self._ok = self._pub.get_subscription_count() > 0 and self._viewers_only()
        if not self._ok:
            return
        out = Image()
        out.header = msg.header
        try:
            if cv2 is not None:
                img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    raise ValueError('not an image')
                out.height, out.width = img.shape[:2]
                out.encoding = 'bgr8'
                out.data = array.array('B', img.tobytes())
            else:
                img = PILImage.open(io.BytesIO(bytes(msg.data))).convert('RGB')
                out.width, out.height = img.size
                out.encoding = 'rgb8'
                out.data = array.array('B', img.tobytes())
        except Exception as exc:                                        # noqa: BLE001
            self.get_logger().warn('could not decode a frame: %s' % exc,
                                   throttle_duration_sec=5.0)
            return
        # (array.array above, never bytes: rclpy validates a bytes value for a uint8[] field one
        # element at a time in Python -- 1.5 M items a frame, which pinned a core at 3.7 Hz.)
        out.step = out.width * 3
        self._pub.publish(out)
        self._frames += 1
        if self._frames == 1:
            self.get_logger().info('first frame: %dx%d' % (out.width, out.height))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('in_topic', nargs='?', default='/dtp/image_annotated/compressed')
    ap.add_argument('out_topic', nargs='?', default='/dtp/image_annotated/laptop')
    ap.add_argument('--any-subscriber', action='store_true',
                    help='publish whoever subscribes (see the module docstring first)')
    args = ap.parse_args()
    if cv2 is None and PILImage is None:
        sys.exit('Neither OpenCV nor Pillow is installed. Run: sudo apt install python3-opencv')
    rclpy.init()
    node = Decoder(args.in_topic, args.out_topic, args.any_subscriber)
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
