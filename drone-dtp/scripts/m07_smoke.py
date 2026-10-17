#!/usr/bin/env python3
"""M0.7 cross-language contract smoke test -- and a regression test for the QoS trap.

WHY THIS EXISTS IN THE REPO: this gate was declared PASSED on 2026-08-14 while the script lived
only in /tmp on one machine. /tmp was later cleared on both machines and the script was gone --
a "passed" gate with nothing behind it. Anything that gates a milestone belongs in version control.

WHAT IT ACTUALLY CHECKS (two things, and the second is the important one):

  1. LIVENESS -- a Python node can drive SUPER (a C++ stack) and get commands back. This is the
     ROS2 cross-language contract from plan/dtp-imp/05 Step 0.1.2 / M0.7.

  2. THE QoS TRAP -- SUPER's fsm_node publishes /planning/pos_cmd as BEST_EFFORT. A subscriber
     using rclpy's DEFAULT QoS is RELIABLE, and a RELIABLE subscriber CANNOT match a BEST_EFFORT
     publisher. The subscription is created without error and receives nothing forever. This is the
     most likely silent-integration failure in this codebase (CLAUDE.md section 10).

     NOTE (measured 2026-08-24, correcting an earlier claim in this repo): rclpy DOES emit
         "New publisher discovered on topic '/planning/pos_cmd', offering incompatible QoS.
          No messages will be received from it. Last incompatible policy: RELIABILITY"
     at WARN level. So the trap is not literally silent -- but the warning is easy to miss in a
     busy launch, and is invisible when a node is launched with output='log'. When a node "sees no
     data", GREP ITS LOG FOR "incompatible QoS" FIRST; it names the exact failure.

     So we subscribe to the SAME topic TWICE -- once BEST_EFFORT, once RELIABLE -- and assert:
         best_effort_count > 0   AND   reliable_count == 0
     The second half is what makes this a regression test rather than a liveness check: if the
     RELIABLE subscriber ever starts receiving, the publisher's QoS changed and every downstream
     assumption in the plan needs revisiting.

NO LIDAR AND NO FC ARE NEEDED. Odometry and the registered cloud are synthesised, so this runs on
any machine with the workspace built.

REQUIRES SUPER TO BE RUNNING. GitHub CI builds only dtp_msgs / dtp_perception / follow_planner /
command_mux, not super_planner -- so this is a HARDWARE/INTEGRATION gate, not a CI job. Run it
alongside a live fsm_node.

Usage:
    # terminal 1: bring SUPER up (no sensors needed for this test)
    ros2 launch super_bringup m1_super_lidar.launch.py
    # terminal 2:
    python3 scripts/m07_smoke.py --duration 15
"""
import argparse
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

ODOM_TOPIC = '/lidar_slam/odom'
CLOUD_TOPIC = '/cloud_registered'
CMD_TOPIC = '/planning/pos_cmd'
# SUPER's fsm sits in WAIT_GOAL and publishes NOTHING until it is given a goal (see the comment in
# super_bringup/launch/m1_super_lidar.launch.py). Feeding odom + cloud alone is not enough to make
# this test meaningful -- without a goal there is no pos_cmd and the QoS assertion never fires.
GOAL_TOPIC = '/goal_pose'

# Hover pose the synthetic odometry reports. Height is above the synthetic ground plane so the
# map does not think the vehicle is underground (which would suppress planning).
HOVER_XYZ = (0.0, 0.0, 1.5)


def qos(reliability, depth=10):
    return QoSProfile(
        reliability=reliability,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_ground_grid(half_extent=6.0, step=0.15, z=0.0):
    """Flat world-frame ground plane as XYZ float32. ROG-Map needs *some* occupied geometry to
    build a map from; a plane is the minimum that is both trivially synthesised and unambiguous."""
    pts = []
    n = int((2 * half_extent) / step) + 1
    for i in range(n):
        x = -half_extent + i * step
        for j in range(n):
            y = -half_extent + j * step
            pts.append((x, y, z))
    return pts


def cloud_msg(stamp, frame_id, pts):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *p) for p in pts)
    return msg


class SmokeNode(Node):
    def __init__(self, frame_id, goal_xyz):
        super().__init__('m07_smoke')
        self.frame_id = frame_id
        self.goal_xyz = goal_xyz
        self.goals_sent = 0

        # Publishers: BEST_EFFORT matches what the real sensor pipeline uses. A RELIABLE publisher
        # would also satisfy a BEST_EFFORT subscriber, but we mirror production deliberately.
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, qos(ReliabilityPolicy.BEST_EFFORT))
        self.cloud_pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, qos(ReliabilityPolicy.BEST_EFFORT))
        # RViz publishes goals RELIABLE/TRANSIENT-less; match the common case.
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, qos(ReliabilityPolicy.RELIABLE))

        self.be_count = 0
        self.rel_count = 0

        # The whole point: same topic, two reliabilities.
        try:
            from mars_quadrotor_msgs.msg import PositionCommand
            self.cmd_type = PositionCommand
        except ImportError:
            self.get_logger().error(
                "mars_quadrotor_msgs not found -- source the workspace that builds SUPER.")
            raise

        self.create_subscription(self.cmd_type, CMD_TOPIC, self._on_be,
                                 qos(ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(self.cmd_type, CMD_TOPIC, self._on_rel,
                                 qos(ReliabilityPolicy.RELIABLE))

        self.ground = make_ground_grid()
        self.create_timer(0.01, self._tick_odom)    # 100 Hz
        self.create_timer(0.10, self._tick_cloud)   # 10 Hz

    def _on_be(self, _):
        self.be_count += 1

    def _on_rel(self, _):
        self.rel_count += 1

    def _tick_odom(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.child_frame_id = 'body'
        m.pose.pose.position.x = HOVER_XYZ[0]
        m.pose.pose.position.y = HOVER_XYZ[1]
        m.pose.pose.position.z = HOVER_XYZ[2]
        m.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(m)

    def _tick_cloud(self):
        self.cloud_pub.publish(
            cloud_msg(self.get_clock().now().to_msg(), self.frame_id, self.ground))

    def send_goal(self):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = self.frame_id
        g.pose.position.x = self.goal_xyz[0]
        g.pose.position.y = self.goal_xyz[1]
        g.pose.position.z = self.goal_xyz[2]
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)
        self.goals_sent += 1


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=15.0,
                    help='seconds to run before evaluating (default 15)')
    ap.add_argument('--frame_id', default='world',
                    help='frame the synthetic odom/cloud are published in (default world)')
    ap.add_argument('--min_cmds', type=int, default=10,
                    help='minimum BEST_EFFORT pos_cmd messages required to pass (default 10)')
    ap.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                    default=[3.0, 0.0, 1.5],
                    help='goal sent on %s to take SUPER out of WAIT_GOAL (default 3 0 1.5)' % GOAL_TOPIC)
    ap.add_argument('--settle', type=float, default=4.0,
                    help='seconds of odom+cloud before sending the goal, so ROG-Map has a map to '
                         'plan through (default 4)')
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = SmokeNode(a.frame_id, a.goal)
    print("publishing %s @100Hz and %s @10Hz for %.0fs ..." % (ODOM_TOPIC, CLOUD_TOPIC, a.duration))
    print("goal %s on %s after %.0fs settle" % (a.goal, GOAL_TOPIC, a.settle))
    print("subscribing %s twice (BEST_EFFORT and RELIABLE)" % CMD_TOPIC)
    print("SUPER must already be running -- this test synthesises sensors, not the planner.\n")

    t0 = time.time()
    goal_deadline = t0 + a.settle
    next_goal_retry = 0.0
    try:
        while time.time() - t0 < a.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            # Resend periodically until commands appear: the first goal can be published before the
            # subscription has matched, and a dropped goal looks identical to a broken planner.
            if now >= goal_deadline and node.be_count == 0 and now >= next_goal_retry:
                node.send_goal()
                next_goal_retry = now + 2.0
    except KeyboardInterrupt:
        pass

    be, rel, goals = node.be_count, node.rel_count, node.goals_sent
    node.destroy_node()
    rclpy.shutdown()

    print("-" * 62)
    print("  goals sent on %-18s : %6d" % (GOAL_TOPIC, goals))
    print("  BEST_EFFORT subscriber            : %6d msgs" % be)
    print("  RELIABLE   subscriber             : %6d msgs" % rel)
    print("-" * 62)

    ok = True
    if be < a.min_cmds:
        print("FAIL: BEST_EFFORT received %d < %d. SUPER is not publishing %s."
              % (be, a.min_cmds, CMD_TOPIC))
        print("      Check SUPER is running and actually planning (it needs a goal to command).")
        ok = False
    else:
        print("PASS: cross-language contract live -- Python drove SUPER and got commands back.")

    if rel != 0:
        print("FAIL: RELIABLE received %d msgs, expected 0." % rel)
        print("      %s is no longer BEST_EFFORT. This CHANGES THE CONTRACT: re-check every" % CMD_TOPIC)
        print("      subscriber's QoS and update CLAUDE.md section 10 and plan/dtp-imp/05 Step 0.7.2.")
        ok = False
    elif be == 0:
        # Do NOT report this as a pass. With no publisher at all, "RELIABLE got nothing" is
        # vacuously true and says nothing about QoS -- a green light here would be actively
        # misleading, which is the exact failure mode this whole test exists to prevent.
        print("SKIP: QoS assertion NOT EXERCISED -- nothing published, so zero on the RELIABLE")
        print("      subscriber proves nothing. Re-run with SUPER live to actually test it.")
    else:
        print("PASS: RELIABLE subscriber received nothing while BEST_EFFORT received %d --" % be)
        print("      QoS trap still behaves as documented. (A default-QoS rclpy subscriber WILL")
        print("      silently see zero messages on this topic.)")

    print("\nRESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
