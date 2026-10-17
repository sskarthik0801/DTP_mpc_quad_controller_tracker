"""Sensor layer for FLIGHT: Livox Mid-360 + Orbbec Gemini 336L.

Sibling of ros2_ws/src/dtp_perception/launch/calib_sensors.launch.py, which brings the same two
sensors up for camera<->LiDAR CALIBRATION. Read that file too; the camera pinning below is
lifted from it deliberately so the two cannot drift.

    ros2 launch dtp_bringup sensors.launch.py
    ros2 launch dtp_bringup sensors.launch.py camera:=false     # LiDAR only
    ros2 launch dtp_bringup sensors.launch.py lidar:=false      # camera only

THE ONE THING PEOPLE GET WRONG HERE: xfer_format
------------------------------------------------
`xfer_format` decides the MESSAGE TYPE on /livox/lidar, and a ROS 2 subscriber of the wrong
type never connects -- no error, no warning, on either side.

*** THERE IS NO CONSTANT RIGHT ANSWER. IT MUST MATCH fast_lio's `lidar_type`. ***
That is why this setting has been "corrected" in both directions in this repo and will be again
unless the pairing is checked rather than the number remembered:

    preprocess.h:  enum LID_TYPE { AVIA = 1, VELO16 = 2, OUST64 = 3, MID360 = 4 }
    laserMapping.cpp:930   if (p_pre->lidar_type == AVIA) -> CustomMsg subscription
                           else                           -> PointCloud2 subscription

  lidar_type: 1 (AVIA)   -> fast_lio wants CustomMsg    -> xfer_format 1
  lidar_type: 4 (MID360) -> fast_lio wants PointCloud2  -> xfer_format 0

**The config we actually ship and load, FAST_LIO_ROS2/config/mid360.yaml, sets `lidar_type: 1`**
(verified on the aircraft 2026-09-08), despite the file being named for the MID360 -- 1 is the
"Livox serials" family, not the model. So the default here is **1 = CustomMsg**.

VERIFY, DO NOT TRUST THIS COMMENT: `grep lidar_type <the config fast_lio actually loads>`. A
previous version of this docstring asserted mid360.yaml set lidar_type 4 and therefore defaulted
xfer_format to 0. It does not, and that default would have left fast_lio silent -- publishing no
odometry and no /cloud_registered, with nothing logged on either side -- on the first bench run
of the full chain.

IF YOU NEED PointCloud2 AS WELL (depth_source:=lidar, or fusion_node's debug cloud), do NOT flip
this -- and do NOT run a second driver instance either. This file used to recommend one. MEASURED
2026-09-12 on this aircraft: a second livox_ros_driver2_node on the same MID360 takes the sensor's
UDP stream away from the first, whatever topic it publishes on. The first driver's CustomMsg went
to 0 Hz while /livox/imu kept flowing from the second, so FAST-LIO integrated IMU alone and came
back 2.1 km off after 120 s on a stationary bench. That is how the aircraft was armed on a 3.74 km
error that evening. Convert CustomMsg to PointCloud2 in a node instead. _livox_driver below now
refuses to start a second driver (preflight.duplicate_refusal).
WHY THE HOST IP IS NOT IN THIS FILE
-----------------------------------
The Mid-360's host/sensor IPs live in livox_ros_driver2's own MID360_config.json, which is
per-device and belongs to that sibling clone, not to this repo (it had to be changed from .12
to .146 on this aircraft). Worse, colcon COPIES config into install/share/ rather than
symlinking it -- this repo's builds have never used --symlink-install -- so editing the source
json without rebuilding livox_ros_driver2 changes nothing. The `livox_config` argument below
exists so you can point at an edited file without that rebuild.

WHY BOTH DRIVERS ARE RESOLVED INSIDE OpaqueFunctions
-----------------------------------------------------
livox_ros_driver2 and orbbec_camera live in a separate overlay on the aircraft and are simply
absent on a workstation. get_package_share_directory raises when the package is missing, and at
generate_launch_description() time that kills the whole launch file -- including
`camera:=false`, which is exactly the invocation someone without a camera would reach for.
Resolving each inside its own conditioned OpaqueFunction means a disabled layer is never looked
up at all.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dtp_bringup import preflight


def _livox_driver(context, *args, **kwargs):
    """Mirrors livox_ros_driver2/launch_ROS2/msg_MID360_launch.py.

    Same json, frame_id and publish rate as every other bring-up on this aircraft, so the LiDAR
    behaves identically whichever launch started it.

    An empty `livox_config` resolves to the driver's own installed json HERE rather than on the
    DeclareLaunchArgument, so that a parent launch can forward the argument unconditionally. If
    the default lived on the declaration, a parent forwarding an unset value would pass ''
    straight through, and an empty user_config_path makes the driver read no config at all -- it
    then never finds the sensor, which presents as a network fault rather than a launch bug.

    xfer_format goes through ParameterValue(value_type=int) because a LaunchConfiguration
    substitution is a STRING. The driver declares xfer_format as an integer, and handing a
    string to an integer parameter fails at node start-up with an InvalidParameterTypeException
    rather than being coerced.
    """
    # One driver per MID360. See preflight.duplicate_refusal: a second one is what starved
    # FAST-LIO into a 3.74 km divergence on 2026-09-12, and the aircraft was armed on it.
    refusal = preflight.duplicate_refusal('livox_ros_driver2_node', 'our Livox driver')
    if refusal:
        return [LogInfo(msg=refusal)]

    user_config_path = LaunchConfiguration('livox_config').perform(context)
    if not user_config_path:
        user_config_path = os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json')

    return [Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        # NOT 'livox_lidar_publisher'. That is the upstream default AND the name the foreign lab
        # driver on this DDS domain uses, and ROS 2 does not enforce unique node names.
        # MEASURED CONSEQUENCE, 2026-09-08: calib_io.check_lidar_source resolved the publisher of
        # /livox/lidar to that shared name, its parameter query was answered by the FOREIGN node,
        # that node's user_config_path does not exist on this machine, and perception_node
        # refused to start with "LIDAR SOURCE IS NOT THIS MACHINE". The data was ours; the name
        # was not. A unique name is the whole fix.
        name='dtp_livox_driver',
        output='screen',
        parameters=[
            {'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'), value_type=int)},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': user_config_path},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )]


def _orbbec_camera(context, *args, **kwargs):
    """Depth is left at the driver's default (on) on purpose.

    sensor_watchdog proxies the depth stream's health through /camera/depth/camera_info -- 0.41
    KB at 30 Hz, tracking the image cadence to within 0.01 Hz at 1/2000th the bytes. Turning
    depth off here would take a watchdog channel offline as a side effect, and the watchdog
    would report it as a sensor that has never published rather than as a configuration choice.
    """
    del context  # only needed to satisfy the OpaqueFunction signature
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orbbec_camera'),
            'launch', 'gemini_330_series.launch.py')),
        launch_arguments={
            'serial_number': LaunchConfiguration('camera_serial'),
            'color_mjpeg_quality': LaunchConfiguration('color_mjpeg_quality'),
            'depth_width': LaunchConfiguration('depth_width'),
            'depth_height': LaunchConfiguration('depth_height'),
            'depth_fps': LaunchConfiguration('depth_fps'),
            'time_domain': LaunchConfiguration('time_domain'),
            'enable_left_ir': LaunchConfiguration('enable_ir'),
            'enable_right_ir': LaunchConfiguration('enable_ir'),
            # *** THE IR GEOMETRY MUST EQUAL THE DEPTH GEOMETRY, so it is DERIVED from the
            # same launch arguments instead of getting knobs of its own. ***
            #
            # MEASURED 2026-09-11, and it cost nothing only because it was found indoors the
            # night before the outdoor test instead of in a field. The Orbbec driver defaults
            # these to 0, meaning "driver chooses", and what it chooses does NOT match depth
            # at 424x240. The SDK then refuses to start the pipeline AT ALL:
            #
            #   Failed to start pipeline: The depth/ir streams must have the same resolution
            #   and frame rate. {type: Left IR, ...}
            #   try to disable ir stream and try again
            #   Failed to initialize device ... Resetting device UID: 2-1.4-3
            #
            # The camera then resets and publishes NOTHING -- not colour, not depth, not IR.
            # So enabling IR without this does not merely lose the IR streams, it takes the
            # WHOLE CAMERA down, and the only symptom on the topics is total silence, which
            # looks exactly like the three silent camera deaths already in the worklog.
            # Deriving these makes the constraint unbreakable rather than documented.
            'left_ir_width': LaunchConfiguration('depth_width'),
            'left_ir_height': LaunchConfiguration('depth_height'),
            'left_ir_fps': LaunchConfiguration('depth_fps'),
            'right_ir_width': LaunchConfiguration('depth_width'),
            'right_ir_height': LaunchConfiguration('depth_height'),
            'right_ir_fps': LaunchConfiguration('depth_fps'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='start the Livox Mid-360 driver'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='start the Orbbec camera (set false if it is already running)'),

        # See the docstring. Exposed as an argument only so the failure is diagnosable from a
        # command line rather than requiring an edit; 0 is the only value that flies.
        DeclareLaunchArgument(
            'xfer_format', default_value='1',
            description='1 = Livox CustomMsg, what fast_lio needs with the shipped mid360.yaml '
                        '(lidar_type 1) -- see this file\'s docstring. 0 = PointCloud2, which '
                        'fast_lio cannot receive: it then gets /livox/imu and no scans, and '
                        'integrates IMU alone until it diverges (2026-09-12, 3.74 km).'),

        DeclareLaunchArgument(
            'livox_config', default_value='',
            description='MID360_config.json holding the per-device host/sensor IPs; empty uses '
                        'the copy installed with livox_ros_driver2'),

        # PIN THE CAMERA BY SERIAL -- same reasoning, verbatim, as calib_sensors.launch.py, and
        # the hazard is worse here than there. Two Orbbec bodies are plugged into this NX (Gemini
        # 336 CP913630009A on USB 2.0, Gemini 336L CPC64630007M on USB 3.0) and with no selector
        # the driver binds whichever it enumerates first; after a reboot on 2026-09-07 it took
        # the 336. calib/drone_cam_livox.yaml is stamped for the 336L, so binding the 336 applies
        # another body's intrinsics AND extrinsic. calib_io loads it happily, overlays still look
        # plausible, and every range to a person is wrong -- which in the follow stack means a
        # standoff computed off a wrong range. No existing gate catches this.
        DeclareLaunchArgument(
            'camera_serial', default_value='CPC64630007M',
            description='Orbbec serial to bind. Defaults to the 336L we calibrated; override '
                        'ONLY with a matching calibration for that body.'),

        # 80 is the driver MAXIMUM, not a round number chosen for headroom. Anything higher is
        # rejected outright -- the NX driver logs "color MJPEG quality value is out of
        # range[1,80]", ignores the value and streams on, so a launch asserting 100 was asserting
        # nothing. 80 is also the default, and measured on the aircraft both give a uniform
        # 135456 B/frame. Detection parity against raw over 1180 person-frames: zero
        # disagreements, box IoU 0.995, and the quality sweep found detection lossless down to
        # q40 -- ample margin.
        DeclareLaunchArgument(
            'depth_width', default_value='424',
            description='Depth width. 424x240 NOT 848x480, and this is a MEASURED throughput '
                        'decision, not a quality compromise. 848x480 16UC1 at 30 Hz is ~24 MB/s, '
                        'and perception_node ingests it through rclpy, which cannot keep up: '
                        'delivered depth falls behind, colour<->depth skew grows past its gate, '
                        'and frames are rejected. MEASURED 2026-09-09: 848x480 gave /dtp/targets '
                        '5.6 Hz with frames rejected at 0.234 s skew; 424x240 gave 10.9 Hz with '
                        'ZERO skew rejections and ZERO blind frames. A person at 2-6 m is '
                        'hundreds of pixels tall at 424x240 -- the torso patch is not the '
                        'limiting factor, the transport is.'),
        DeclareLaunchArgument(
            'depth_height', default_value='240',
            description='See depth_width.'),
        # Declared so the IR streams can be pinned to the SAME rate -- the driver requires
        # depth and IR to agree on frame rate as well as resolution (see the include above).
        DeclareLaunchArgument(
            'depth_fps', default_value='30',
            description='Depth frame rate. The IR streams inherit it when '
                        'enable_ir is true, because the SDK refuses to start '
                        'if depth and IR disagree on rate.'),
        DeclareLaunchArgument(
            'color_mjpeg_quality', default_value='80',
            description='Orbbec MJPG quality. Valid range is [1,80]; >80 is REJECTED, ignored.'),
        # 'system' stamps every frame from the HOST clock at receive time; the driver default
        # ('global', device clock mapped onto host time) put colour stamps +30.5 ms off ROS
        # time as MEASURED on the aircraft 2026-09-09, which is why perception_node ever needed
        # its tf_at_latest escape hatch. With 'system' the residual is +5.6 ms and it is
        # capture-to-publish latency, not a clock mismatch -- inside one frame period, so a
        # stamp-exact TF lookup succeeds on every frame (VERIFIED 2026-09-10, SLAM up,
        # tf_at_latest:=false, perception_ok true, 13.3 Hz). This is what makes the stack
        # flight-legal: a latest-available TF is wrong by yaw-rate x range once airborne.
        # Do not change this to buy a lower stamp offset on paper -- the number that matters
        # is the one measured against /lidar_slam/odom, and 'system' is the one that closes it.
        # OFF by default because it costs depth RATE: enabling both IR streams adds ~2.5 MB/s of
        # USB traffic and takes depth from 30 Hz to roughly 20 Hz. It does NOT affect depth
        # VALIDITY, which is what the outdoor test measures -- so the trade is free for that test
        # and not worth paying in normal operation.
        #
        # TURN IT ON FOR: scripts/outdoor_depth_test.py, which needs the IR pair to separate sun
        # from shade by saturation (without it the sun/shade classification silently does not
        # work, and a bad depth result has no explanation attached); and
        # scripts/ir_window_check.py, which is how the blocked-IR-window fault of 2026-09-08 was
        # found -- depth was 0.2 % valid while colour looked perfect and the driver logged
        # nothing. Brightness called that camera healthy; only IR CONTRAST exposed it.
        DeclareLaunchArgument(
            'enable_ir', default_value='false',
            description='publish /camera/{left,right}_ir/image_raw. Needed by '
                        'outdoor_depth_test.py (sun/shade by IR saturation) and '
                        'ir_window_check.py. Costs ~10 Hz of depth rate, no effect on validity.'),
        DeclareLaunchArgument(
            'time_domain', default_value='system',
            description='Orbbec stamp clock: system (host, REQUIRED for stamped TF lookups) | '
                        'global | device. See the comment above; global costs ~30 ms of skew.'),

        OpaqueFunction(function=_livox_driver,
                       condition=IfCondition(LaunchConfiguration('lidar'))),
        OpaqueFunction(function=_orbbec_camera,
                       condition=IfCondition(LaunchConfiguration('camera'))),
    ])
