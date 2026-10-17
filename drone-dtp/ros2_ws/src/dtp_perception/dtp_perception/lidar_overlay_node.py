"""lidar_overlay_node -- validates the camera<->LiDAR extrinsic calibration (T_camera_lidar).

ADAPTED from the reference repo's lidar_overlay_node.py (plan/dtp-imp/03 SS5, "Reuse as the
T_camera_lidar calib gate"). SCAFFOLD ONLY (Phase 0 M0.1) -- real logic lands in Phase 0 M0.3/
Phase 1 M1.2 (plan/dtp-imp/05, plan/dtp-imp/06): project LiDAR into the camera image using the
calibrated extrinsic, compare the median LiDAR-derived depth against camera stereo depth in a
central patch, and report the residual -- this is the go/no-go check for a calibration, and
per plan/dtp-imp/05 Step 0.3.1b must be re-run MOTOR-ON (vibration changes the extrinsic).

Not part of the runtime follow pipeline -- run standalone during calibration, not launched by
amr.launch.py-equivalent bringup files.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

from dtp_perception import calib_io


class LidarOverlayNode(Node):
    def __init__(self):
        super().__init__('lidar_overlay_node')

        # Bare filename by design: calib_io resolves it against the package SHARE directory.
        # The old relative default 'calib/drone_cam_livox.yaml' resolved against the process
        # working directory and so never found the installed file. Absolute paths still
        # override, which is what comparing two candidate extrinsics on the bench wants.
        self.declare_parameter('calib_file', calib_io.DEFAULT_CALIB)

        # This node exists to judge an extrinsic, so running it without one validates nothing.
        self._T_camera_lidar, self._intrinsics = calib_io.load_for_node(
            self, self.get_parameter('calib_file').value)

        self._lidar_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self._on_lidar, qos_profile_sensor_data)
        self._depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self._on_depth, qos_profile_sensor_data)

        self.get_logger().info(
            'lidar_overlay_node up (SCAFFOLD -- no overlay comparison yet, '
            'see plan/dtp-imp/06 M1.2).')

    def _on_lidar(self, msg: PointCloud2) -> None:
        pass  # TODO Phase 1 M1.2: cache latest LiDAR for comparison against camera depth

    def _on_depth(self, msg: Image) -> None:
        pass  # TODO Phase 1 M1.2: compute + log median LiDAR-vs-camera depth residual


def main(args=None):
    rclpy.init(args=args)
    node = LidarOverlayNode()
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
