"""playback_node -- replays a recorded session (color + cloud + odom) for offline,
deterministic non-regression testing. ADAPTED from the reference repo's record-once/
replay-many harness (plan/dtp-imp/03 SS5, plan/dtp-imp/06 M1.1 exit gate).

SCAFFOLD ONLY (Phase 0 M0.1). Real logic lands in Phase 1 M1.1: load a recorded session
(color frames + organized cloud + odom, captured once) and republish it at a controllable
rate so the SAME recording can be run through the live ROS2 graph repeatedly -- e.g. to
compare a code change's output against a pinned baseline, or to build the
perception-in-the-loop harness needed because MARSIM has no humans (plan/dtp-imp/07 M2.3.2,
risk A3). Gate per plan/dtp-imp/06: BOUNDED parity (detection IoU / range within tolerance)
against the reference repo's recordings, not bit-identity -- the camera/geometry layers are
being adapted, so exact reproduction isn't the right bar.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


class PlaybackNode(Node):
    def __init__(self):
        super().__init__('playback_node')

        self.declare_parameter('session', '')  # path to a recorded session directory
        self.declare_parameter('rate', 8.0)     # playback rate, Hz

        self._session = self.get_parameter('session').value
        self._rate = float(self.get_parameter('rate').value)

        # Republishes onto the SAME topics the real camera/fusion nodes would use, so
        # perception_node (and everything downstream) cannot tell the difference.
        self._color_pub = self.create_publisher(
            Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self._cloud_pub = self.create_publisher(
            PointCloud2, '/dtp/points_fused', qos_profile_sensor_data)

        if not self._session:
            self.get_logger().warn(
                "playback_node up but no 'session' param set -- nothing to replay. "
                "Run with --ros-args -p session:=<path> -p rate:=<hz>.")
        else:
            self.get_logger().info(
                f'playback_node up (SCAFFOLD -- session={self._session} rate={self._rate}, '
                'no replay loop yet, see plan/dtp-imp/06 M1.1).')

        # TODO Phase 1 M1.1: load the session, create a timer at self._rate that pops the
        # next recorded (color, cloud, odom) tuple and republishes it with a fresh stamp.


def main(args=None):
    rclpy.init(args=args)
    node = PlaybackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how SIGTERM arrives -- the normal stop path for systemd
        # and `ros2 launch`. Without it, every managed shutdown exits with a stack trace.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
