"""sensor_watchdog on its own, so no configuration can accidentally fly without it.

    ros2 launch dtp_bringup sensor_watchdog.launch.py

It is a separate file rather than three lines inside follower.launch.py for one reason: the
watchdog must survive `follower:=false`. A NAV-only or click-goal-only flight has no follower
layer and still needs stall detection -- arguably more, since there is no target lock whose
disappearance would hint that perception died. dtp_system.launch.py therefore starts this
directly and tells follower.launch.py not to, so the two toggles are genuinely independent.

WHAT IT CATCHES, IN ONE SENTENCE: a sensor that has stopped publishing while its process,
its USB enumeration and its topic registrations all still look perfectly healthy -- how the
Orbbec driver failed on 2026-09-07 after 1175 s of uptime with nothing logged, recovering
fully on restart. Nothing else in this stack notices; check_camera_serial and
check_lidar_source both verify identity at start-up and never look again.

THE TRAP ITS CONSUMER MUST HANDLE (command_mux, not this launch): /dtp/sensors_ok reads TRUE
during the 20 s startup grace even when no sensor has ever published, because a topic that has
NEVER published is only WARN in that window. Treating boot-time True as health is a real
hazard; the consumer must require a message received, its age under a timeout, AND data==True.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dtp_bringup import preflight


def _launch_setup(context, *args, **kwargs):
    preflight.assert_wall_clock(LaunchConfiguration('use_sim_time').perform(context))

    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory('dtp_bringup'), 'config', 'dtp_params.yaml')

    return [Node(
        package='dtp_perception',
        executable='sensor_watchdog',
        name='sensor_watchdog',
        parameters=[params_file, {'use_sim_time': False}],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='parameter YAML; defaults to dtp_bringup share config/dtp_params.yaml'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='must be false -- see dtp_bringup.preflight.assert_wall_clock'),
        OpaqueFunction(function=_launch_setup),
    ])
