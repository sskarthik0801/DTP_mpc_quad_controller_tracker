#!/usr/bin/env python3
"""Receive and decode live Gemini JPEG frames; exits nonzero on timeout/bad data."""
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    rclpy.init()
    node = rclpy.create_node('gemini_frame_check')
    node.declare_parameter('topic', '/gemini/color/image_raw/compressed')
    samples = []
    failures = []

    def receive(msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None or not msg.header.frame_id:
            failures.append('Invalid JPEG or empty frame_id')
            return
        samples.append((time.monotonic(), len(msg.data), frame.shape,
                        msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec))

    node.create_subscription(CompressedImage, node.get_parameter('topic').value,
                             receive, qos_profile_sensor_data)
    deadline = time.monotonic() + 20
    try:
        while rclpy.ok() and time.monotonic() < deadline and len(samples) < 60:
            rclpy.spin_once(node, timeout_sec=0.5)
        if failures or len(samples) < 2:
            raise RuntimeError(f'Camera check failed: {len(samples)} frames; {failures[:3]}')
        if any(b[3] <= a[3] for a, b in zip(samples, samples[1:])):
            raise RuntimeError('Camera timestamps are not increasing')
        elapsed = samples[-1][0] - samples[0][0]
        print(f'PASS: {len(samples)} decoded frames, {samples[-1][2]}, '
              f'{(len(samples)-1)/elapsed:.2f} Hz, '
              f'{sum(s[1] for s in samples)/len(samples):.0f} JPEG bytes/frame')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
