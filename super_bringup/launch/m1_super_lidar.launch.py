"""
M1 - Sensing & State Estimation: real Livox MID360 -> FAST-LIO2 -> SUPER (ROG-Map).

This is PURE state estimation: FAST-LIO2 provides BOTH the registered cloud and the
odometry that drive ROG-Map. There is NO perfect_drone_sim here (it would clobber
/cloud_registered and /lidar_slam/odom). No flight; this only proves perception -> map.

Run the Livox driver separately first (it lives in the ws_livox overlay):
    source /opt/ros/humble/setup.bash
    source ~/ws_livox/install/setup.bash
    ros2 launch livox_ros_driver2 msg_MID360_launch.py      # xfer_format=1 (CustomMsg)

Then, in another terminal:
    source /opt/ros/humble/setup.bash
    source ~/ws_livox/install/setup.bash
    source ~/super_ws/install/setup.bash
    ros2 launch super_bringup m1_super_lidar.launch.py

Gate C (M1 done): in RViz (Fixed Frame: world) subscribe to rog_map/occ and
rog_map/inf_occ and confirm occupied/inflated voxels appear from the live cloud, with
no "odom timeout" / "below virtual ground" warnings in the fsm_node console.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    mid360_cfg = os.path.join(fast_lio_share, 'config', 'mid360.yaml')

    use_rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    fastlio_cpus = LaunchConfiguration('fastlio_cpus')
    fsm_cpus = LaunchConfiguration('fsm_cpus')
    fastlio_pub_mode = LaunchConfiguration('fastlio_pub_mode')
    fastlio_dds_profile = LaunchConfiguration('fastlio_dds_profile')

    # M5.3 (2026-09-13): CPU pinning, the plan's "FAST-LIO pinned to 2 dedicated cores"
    # (docs/m5-m6-plan.md SS2.5). taskset needs no root and is inherited by every thread the
    # process starts, so FAST-LIO's 3-thread executor lands on its own cores and fsm_node
    # (ROG-Map raycasting, ~90-120 % of a core) on the others. "Dedicated" only holds if the
    # rest of the graph is kept OFF FAST-LIO's cores: run.sh pins the Livox driver and
    # dtp_follower.sh the whole DTP tree to the same complement it passes here as fsm_cpus.
    # An empty list means no pinning (launch drops an empty prefix), which is what a box with
    # fewer cores, or a bench check that wants the old behaviour, should pass.
    def pinned(cpus):
        return PythonExpression(["'taskset -c ' + '", cpus, "' if '", cpus, "' else ''"])

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false', description='Launch RViz')
    declare_rviz_cfg = DeclareLaunchArgument(
        'rviz_cfg', default_value='',
        description='RViz config file (full-pipeline view from the prior session)')
    # M5 (2026-09-13): the follow flights plan on follow_test_ros2.yaml, which carries the staged
    # follow envelope (M5.2) and the target-point filter (M5.3a). Before this argument existed the
    # fsm_node started here was ALWAYS click_smooth_ros2.yaml -- and dtp_bringup's super.launch.py
    # deliberately refuses to start a second fsm_node on top of this stack, so a follower riding on
    # it silently planned on the click config whatever it was launched with. run.sh forwards
    # $SUPER_CONFIG here. Resolved by fsm_node against its SOURCE config/ dir (ROOT_DIR).
    config_name = LaunchConfiguration('config_name')
    declare_config_name = DeclareLaunchArgument(
        'config_name', default_value='click_smooth_ros2.yaml',
        description='fsm_node config under super_planner/config/: click_smooth_ros2.yaml (bench, '
                    'click goals, 0.1 m/s) or follow_test_ros2.yaml (M5 follow flights)')

    declare_fastlio_cpus = DeclareLaunchArgument(
        'fastlio_cpus', default_value='',
        description="taskset -c list for fastlio_mapping (e.g. '6,7'); empty = not pinned")
    declare_fsm_cpus = DeclareLaunchArgument(
        'fsm_cpus', default_value='',
        description="taskset -c list for fsm_node (e.g. '0-5'); empty = not pinned")
    # rmw_fastrtps publishes SYNCHRONOUSLY by default: publish() does the UDP sendto in the
    # calling thread, i.e. inside FAST-LIO's scan callback. With a Wi-Fi subscriber to the dense
    # /cloud_registered, a stalled link fills the socket's send buffer and the callback blocks --
    # the suspect for the 2026-09-13 soak's 944 ms odometry gap (docs/m5-m6-plan.md M5.3 item 3).
    # ASYNCHRONOUS moves the sends to FastDDS's own thread. Applied to FAST-LIO's process only.
    declare_fastlio_pub_mode = DeclareLaunchArgument(
        'fastlio_pub_mode', default_value='SYNCHRONOUS',
        description='RMW_FASTRTPS_PUBLICATION_MODE for fastlio_mapping: SYNCHRONOUS | ASYNCHRONOUS')
    # A FastDDS XML profile for FAST-LIO's process only (FASTRTPS_DEFAULT_PROFILES_FILE).
    # 'nonblocking' = super_bringup/config/fastdds_nonblocking.xml (UDP sends never block, see the
    # file); '' = FastDDS defaults; anything else = a path.
    declare_fastlio_dds_profile = DeclareLaunchArgument(
        'fastlio_dds_profile', default_value='',
        description="FastDDS XML profile for fastlio_mapping: '' | nonblocking | <path>")

    # FAST-LIO2: real LiDAR-inertial odometry. Publishes /cloud_registered (world frame,
    # which ROG-Map consumes directly) and /Odometry, remapped to /lidar_slam/odom.
    def make_fast_lio(context):
        env = {'RMW_FASTRTPS_PUBLICATION_MODE': fastlio_pub_mode.perform(context)}
        profile = fastlio_dds_profile.perform(context)
        if profile == 'nonblocking':
            profile = os.path.join(get_package_share_directory('super_bringup'), 'config',
                                   'fastdds_nonblocking.xml')
        if profile:
            if not os.path.isfile(profile):
                raise RuntimeError('fastlio_dds_profile: no such file: %s' % profile)
            # An empty FASTRTPS_DEFAULT_PROFILES_FILE is not "unset" to FastDDS, hence the branch.
            env['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile
        return [Node(
            package='fast_lio',
            executable='fastlio_mapping',
            exec_name='fastlio_mapping',   # keep the log label; with a prefix launch would print [taskset-N]
            prefix=pinned(fastlio_cpus),
            additional_env=env,
            parameters=[mid360_cfg, {'use_sim_time': False}],
            remappings=[('/Odometry', '/lidar_slam/odom')],
            output='screen',
        )]
    fast_lio_node = OpaqueFunction(function=make_fast_lio)

    # FAST-LIO's world origin frame is `lidar_init`; SUPER/ROG-Map visualize in `world`.
    # Identity static TF so RViz (Fixed Frame: world) can also display the raw cloud/path.
    # (ROG-Map itself ignores frame_id/TF and treats the odom pose as world-frame.)
    #
    # `lidar_init` / `lidar_imu` are OUR names, applied to the FAST-LIO source by
    # patches/fastlio_frames.py. Upstream calls them `camera_init` / `body`. The odom
    # child frame is the MID360's internal IMU -- NOT the drone body and NOT the LiDAR
    # optical origin (4.41 cm apart, mid360.yaml extrinsic_T).
    world_to_lidar_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'lidar_init'],
        output='log',
    )

    # SUPER flight FSM + ROG-Map. Ingests /cloud_registered + /lidar_slam/odom.
    # No goal is sent in M1, so the planner stays in WAIT_GOAL; we only check the map.
    super_fsm = Node(
        package='super_planner',
        executable='fsm_node',
        exec_name='fsm_node',
        prefix=pinned(fsm_cpus),
        parameters=[{'config_name': config_name}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz),
        output='log',
    )

    return LaunchDescription([
        declare_rviz,
        declare_rviz_cfg,
        declare_config_name,
        declare_fastlio_cpus,
        declare_fsm_cpus,
        declare_fastlio_pub_mode,
        declare_fastlio_dds_profile,
        fast_lio_node,
        world_to_lidar_init,
        super_fsm,
        rviz_node,
    ])
