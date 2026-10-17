"""Gemini 336L color publisher using the Orbbec SDK's USB backend."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    options = {
        'camera_name': ('gemini', 'Camera namespace and frame prefix'),
        'serial_number': ('', 'Optional camera serial when more than one is connected'),
        'color_width': ('640', 'Color width in pixels'),
        'color_height': ('480', 'Color height in pixels'),
        'color_fps': ('15', 'Camera frame rate; choose a supported stream profile'),
    }
    arguments = {name: LaunchConfiguration(name) for name in options}
    arguments.update({
        'uvc_backend': 'libuvc',
        'color_format': 'MJPG',
        'enable_color': 'true',
        'color_qos': 'SENSOR_DATA',
        'color_camera_info_qos': 'SENSOR_DATA',
        'enable_depth': 'false',
        'enable_left_ir': 'false',
        'enable_right_ir': 'false',
        'enable_point_cloud': 'false',
        'enable_colored_point_cloud': 'false',
        'enable_accel': 'false',
        'enable_gyro': 'false',
        'enable_sync_output_accel_gyro': 'false',
        'enable_frame_sync': 'false',
        'enumerate_net_device': 'false',
        'publish_tf': 'false',
    })
    driver_launch = os.path.join(get_package_share_directory('orbbec_camera'),
                                 'launch', 'gemini_330_series.launch.py')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in options.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(driver_launch),
                                 launch_arguments=arguments.items()),
    ])
