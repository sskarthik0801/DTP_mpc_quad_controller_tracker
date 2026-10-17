#!/usr/bin/env python3
"""record_targets.py -- write /dtp/targets and /dtp/perception_ok to JSON lines until SIGINT.

Used by replay_perception.sh. One line per message:
  {"k": "t", "s": <stamp, s>, "targets": [[track_id, x, y, z, distance_m, confidence], ...]}
  {"k": "ok", "v": <bool>}
"""
import json
import sys

import rclpy
from dtp_msgs.msg import TargetArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


class Recorder(Node):

    def __init__(self, path):
        super().__init__('replay_target_recorder')
        self._f = open(path, 'w')
        self.create_subscription(TargetArray, '/dtp/targets', self._on_targets,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/dtp/perception_ok', self._on_ok, 10)

    def _on_targets(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        targets = [[t.track_id, t.position.x, t.position.y, t.position.z, t.distance_m,
                    t.confidence] for t in msg.targets]
        self._f.write(json.dumps({'k': 't', 's': stamp, 'targets': targets}) + '\n')

    def _on_ok(self, msg):
        self._f.write(json.dumps({'k': 'ok', 'v': bool(msg.data)}) + '\n')

    def close(self):
        self._f.close()


def main():
    rclpy.init()
    node = Recorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()


if __name__ == '__main__':
    main()
