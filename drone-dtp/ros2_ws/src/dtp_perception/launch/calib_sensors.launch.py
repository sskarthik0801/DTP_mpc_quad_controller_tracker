"""Sensor bring-up for camera<->LiDAR CALIBRATION only.

WHY THIS EXISTS SEPARATELY FROM THE FLIGHT LAUNCH:
The Livox driver's `xfer_format` decides the message type on `/livox/lidar`, and the two consumers
disagree:

  * FAST-LIO (flight)      -> xfer_format 1 = `livox_ros_driver2/msg/CustomMsg`
  * calibration + koide3   -> xfer_format 0 = `sensor_msgs/msg/PointCloud2`

A ROS 2 subscriber of the wrong type simply NEVER CONNECTS. There is no error, no warning: the
capture node sits there recording zero points and the run looks like a hardware fault. This is the
same silent type-mismatch that cost a Gate C run on the bench NX
(see plan/dtp-imp/11 R-F22, and the `super_bringup` launch docstring).

So calibration gets its own launch that requests PointCloud2, and the flight launch is left alone.
**Do NOT "fix" the flight launch to match this one** -- FAST-LIO needs CustomMsg.

Everything else mirrors `livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` so the LiDAR behaves
identically (same config json, frame_id, publish rate).

Usage:
    ros2 launch dtp_perception calib_sensors.launch.py
    # then, in another shell:
    ros2 run dtp_perception capture_calib_data --ros-args \
        -p out_dir:=~/calib_runs/$(date +%Y%m%d_%H%M) -p target_pairs:=8 -p bag_secs:=15
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    livox_share = get_package_share_directory('livox_ros_driver2')
    user_config_path = os.path.join(livox_share, 'config', 'MID360_config.json')

    declare_camera = DeclareLaunchArgument(
        'camera', default_value='true',
        description='also start the Orbbec camera (set false if it is already running)')

    # PIN THE CAMERA BY SERIAL. Two Orbbec bodies are plugged into this NX:
    #   Gemini 336   CP913630009A  on USB 2.0 (480 Mbps)
    #   Gemini 336L  CPC64630007M  on USB 3.0 (5 Gbps)   <- the one we calibrated
    # With no selector the driver binds whichever it enumerates first, and after a reboot on
    # 2026-09-07 it took the 336. The visible symptom was harmless -- on USB 2.0 the 336 offers
    # 1280x720 only at 6/10/15 fps MJPG, so the stream ran at 10 Hz instead of 30.
    # The REAL hazard is silent: calib/drone_cam_livox.yaml is stamped "Gemini 336L", so binding
    # the 336 applies another camera's intrinsics AND extrinsic. calib_io loads it happily,
    # overlays still look plausible, and every range to a person is wrong. No existing gate
    # catches this -- the calibration file is perfectly valid, just for a different body.
    declare_serial = DeclareLaunchArgument(
        'camera_serial', default_value='CPC64630007M',
        description='Orbbec serial to bind. Defaults to the 336L we calibrated; override ONLY '
                    'with a matching calibration for that body.')

    # xfer_format 0 == PointCloud2. This is the ONLY intentional difference from the flight launch.
    livox_params = [
        {'xfer_format': 0},
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': user_config_path},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        # *** NAMED OURS, and that matters MORE here than on the flight launch. ***
        #
        # 'livox_lidar_publisher' is the upstream driver's default name. On 2026-09-07
        # ANOTHER LAB'S ROBOT was publishing under that exact name on our DDS domain, and
        # our stack bound THEIR LiDAR -- plausibly, at the right rate, with no error.
        # dtp_bringup/sensors.launch.py was renamed to fix that; THIS launch was missed and
        # kept the colliding name until 2026-09-11.
        #
        # Why it is worse here: calib_io.check_lidar_source identifies the publisher by the
        # node name on the topic endpoint. With our node ALSO called
        # 'livox_lidar_publisher', that gate cannot tell our LiDAR from theirs. And the
        # output of this launch is not a flight -- it is an EXTRINSIC, which gets written to
        # a YAML and then silently turns every detected person's pixels into a range for
        # weeks afterwards. A calibration solved against someone else's LiDAR is wrong in a
        # way nothing downstream can detect.
        name='dtp_livox_driver',
        output='screen',
        parameters=livox_params,
    )

    orbbec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orbbec_camera'),
            'launch', 'gemini_330_series.launch.py')),
        launch_arguments={
            'serial_number': LaunchConfiguration('camera_serial'),
            # Pin MJPG to the driver's maximum. The knob is a TARGET SIZE, not a quality factor.
            #
            # This said '100' until 2026-09-08, and the driver on the NX REJECTS that:
            #   "color MJPEG quality value is out of range[1,80]"
            # It logs the error, ignores the value, and streams on -- so we were asserting a
            # setting that never applied. Measured on the aircraft: 80 and the default both give
            # a uniform 135456 B/frame, and 80 IS the default, so nothing was actually lost. The
            # 168212 B/frame recorded earlier in plan/dtp-imp/21 came from a driver build that
            # accepted 100; do not expect to reproduce it here.
            #
            # Detection parity against raw was measured over 1180 person-frames: zero
            # disagreements, box IoU 0.995 -- and the quality sweep found detection lossless down
            # to q40, so 135 KB/frame has ample margin. See scripts/compare_transport_detect.py.
            'color_mjpeg_quality': '80',
        }.items(),
        condition=IfCondition(LaunchConfiguration('camera')),
    )

    return LaunchDescription([declare_camera, declare_serial, livox_driver, orbbec])
