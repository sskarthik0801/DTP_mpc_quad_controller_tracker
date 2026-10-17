#!/usr/bin/env python3
"""terrain_estimator_node: the ground height under the aircraft and under the follow goal.

It folds FAST-LIO's scans into a TerrainMap (terrain_map.py: why a map and not the downward
rangefinder, and what each step rejects), samples the ground under the aircraft and under the
latest /dtp/follow_goal, and publishes the HIGHER of the two on /dtp/terrain for command_mux
(terrain.py: climb early, descend late). Started by follower.launch.py only when
command_mux terrain_follow_enable is true.

It commands nothing. If it dies, /dtp/terrain goes stale and command_mux HOLDS the offset it
had: it never jumps back to the absolute height.

Topics:
  Subscribes: /cloud_registered_body (sensor_msgs/PointCloud2, FAST-LIO, the lidar_imu frame).
                NOT /cloud_registered: the preflights refuse any subscriber on that topic but
                fsm_node. The recorder already takes this one.
              /lidar_slam/odom (nav_msgs/Odometry): the pose each scan is transformed by,
                matched on header stamp -- FAST-LIO stamps a scan and its odometry alike.
              /dtp/follow_goal (geometry_msgs/PoseStamped, follow_planner): where the aircraft
                is about to go. Ignored once older than goal_timeout_s.
              /dtp/targets (dtp_msgs/TargetArray, perception): every person seen. Their points
                are cut out of each scan (terrain_map.py, PEOPLE ARE CUT OUT) -- the walker is
                otherwise read as the ground. Ignored once older than targets_timeout_s.
  Publishes:  /dtp/terrain (geometry_msgs/PointStamped): point.z = the ground height to follow,
                in lidar_init, NaN when neither patch is known; point.x/y = the aircraft's
                position it was sampled at. One message per processed scan (5 Hz at stride 2).
"""
import collections
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from dtp_msgs.msg import TargetArray

from command_mux.terrain_map import TerrainMap


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _rotation(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class TerrainEstimator(Node):

    def __init__(self):
        super().__init__('terrain_estimator')
        self.declare_parameter('cloud_topic', '/cloud_registered_body')
        self.declare_parameter('odom_topic', '/lidar_slam/odom')
        self.declare_parameter('goal_topic', '/dtp/follow_goal')
        self.declare_parameter('targets_topic', '/dtp/targets')
        self.declare_parameter('terrain_topic', '/dtp/terrain')
        self.declare_parameter('world_frame', 'world')
        # Every N-th scan. The ground does not change at 10 Hz, and at 2 the node costs ~2 ms of
        # CPU per processed scan on the NX (terrain_replay.py on the 2026-09-21 bag).
        self.declare_parameter('scan_stride', 2)
        self.declare_parameter('odom_match_s', 0.05)
        self.declare_parameter('goal_timeout_s', 1.0)
        self.declare_parameter('targets_timeout_s', 0.5)
        self.declare_parameter('cell_m', 0.5)
        self.declare_parameter('point_history', 32)
        self.declare_parameter('cell_percentile', 0.2)
        self.declare_parameter('min_points_per_cell', 3)
        self.declare_parameter('patch_radius_m', 1.5)
        self.declare_parameter('min_cells', 8)
        self.declare_parameter('max_range_m', 30.0)
        self.declare_parameter('min_below_sensor_m', 0.1)
        self.declare_parameter('exclude_radius_m', 1.0)

        def p(name):
            return self.get_parameter(name).value

        self._map = TerrainMap(
            cell_m=float(p('cell_m')), point_history=int(p('point_history')),
            cell_percentile=float(p('cell_percentile')),
            min_points_per_cell=int(p('min_points_per_cell')),
            patch_radius_m=float(p('patch_radius_m')), min_cells=int(p('min_cells')),
            max_range_m=float(p('max_range_m')),
            min_below_sensor_m=float(p('min_below_sensor_m')),
            exclude_radius_m=float(p('exclude_radius_m')))
        self._stride = max(1, int(p('scan_stride')))
        self._match_s = float(p('odom_match_s'))
        self._goal_timeout = float(p('goal_timeout_s'))
        self._targets_timeout = float(p('targets_timeout_s'))
        self._people = ((), None)     # ((x, y), ...), arrival
        self._frame = str(p('world_frame'))
        self._odoms = collections.deque(maxlen=50)
        self._goal = None             # (x, y, arrival)
        self._n_scans = 0
        self._n_used = 0
        self._n_unmatched = 0
        self._pending = None          # a scan that arrived before its odometry
        self._cpu = collections.deque(maxlen=100)
        self._last = (None, None)     # (ground under the aircraft, ground under the goal)

        self.create_subscription(PointCloud2, str(p('cloud_topic')), self._on_cloud,
                                 QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                            history=QoSHistoryPolicy.KEEP_LAST, depth=2))
        self.create_subscription(Odometry, str(p('odom_topic')), self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, str(p('goal_topic')), self._on_goal,
                                 qos_profile_sensor_data)
        self.create_subscription(TargetArray, str(p('targets_topic')), self._on_targets,
                                 qos_profile_sensor_data)
        self._pub = self.create_publisher(PointStamped, str(p('terrain_topic')), 10)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            'terrain estimator: %s + %s -> %s, every %d scan(s); cells %.2f m, %d points each, '
            'p%.0f; patch %.1f m, >= %d cells; range %.0f m; people from %s cut out to %.1f m'
            % (p('cloud_topic'), p('odom_topic'), p('terrain_topic'), self._stride,
               self._map.cell_m, self._map.point_history, 100 * self._map.cell_percentile,
               self._map.patch_radius_m, self._map.min_cells, self._map.max_range_m,
               p('targets_topic'), self._map.exclude_radius_m))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z):
            self._odoms.append((_stamp(msg.header), msg.pose.pose))
            if self._pending is not None:
                pending, self._pending = self._pending, None
                self._process(pending, retry=False)

    def _on_goal(self, msg):
        p = msg.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            self._goal = (p.x, p.y, time.monotonic())

    def _on_targets(self, msg):
        self._people = (tuple((t.position.x, t.position.y) for t in msg.targets),
                        time.monotonic())

    def _on_cloud(self, msg):
        self._n_scans += 1
        if self._n_scans % self._stride:
            return
        self._process(msg, retry=True)

    def _process(self, msg, retry):
        cs = _stamp(msg.header)
        match = min(self._odoms, key=lambda o: abs(o[0] - cs)) if self._odoms else None
        if match is None or abs(match[0] - cs) > self._match_s:
            if retry and (match is None or match[0] < cs):
                # The scan beat its odometry here (they share a stamp): try again when the next
                # odometry arrives, instead of dropping it. Replaying the 2026-09-21 bag dropped
                # ~25 % of scans before this.
                self._pending = msg
            else:
                self._n_unmatched += 1
            return
        pose = match[1]
        c0 = time.process_time()
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pos = pose.position
        world = pts.astype(np.float64) @ _rotation(pose.orientation).T + np.array(
            [pos.x, pos.y, pos.z])
        people, arrival = self._people
        if arrival is None or time.monotonic() - arrival > self._targets_timeout:
            people = ()
        self._map.add_scan(world, (pos.x, pos.y, pos.z), exclude=people)
        g_drone, _ = self._map.ground_at(pos.x, pos.y)
        g_goal = None
        if self._goal is not None and time.monotonic() - self._goal[2] <= self._goal_timeout:
            g_goal, _ = self._map.ground_at(self._goal[0], self._goal[1])
        self._last = (g_drone, g_goal)
        known = [g for g in self._last if g is not None]
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame
        out.point.x, out.point.y = pos.x, pos.y
        out.point.z = max(known) if known else math.nan
        self._pub.publish(out)
        self._n_used += 1
        self._cpu.append(time.process_time() - c0)

    def _report(self):
        g_drone, g_goal = self._last

        def f(v):
            return 'unknown' if v is None else '%.2f' % v
        self.get_logger().info(
            'terrain: ground under aircraft %s, under goal %s | %d cells | %d/%d scans used, '
            '%d unmatched to odom | %.1f ms CPU per scan'
            % (f(g_drone), f(g_goal), len(self._map), self._n_used, self._n_scans,
               self._n_unmatched, 1e3 * (np.median(self._cpu) if self._cpu else 0.0)))


def main(args=None):
    rclpy.init(args=args)
    node = TerrainEstimator()
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
