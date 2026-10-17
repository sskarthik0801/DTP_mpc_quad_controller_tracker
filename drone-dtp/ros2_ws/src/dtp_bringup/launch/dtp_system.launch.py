"""The whole DTP stack in one command: sensors + SUPER + follower + watchdog.

    ros2 launch dtp_bringup dtp_system.launch.py                     # everything
    ros2 launch dtp_bringup dtp_system.launch.py follower:=false     # NAV / click-goal only
    ros2 launch dtp_bringup dtp_system.launch.py sensors:=false      # sensors already running
    ros2 launch dtp_bringup dtp_system.launch.py super:=false        # perception bring-up only
    ros2 launch dtp_bringup dtp_system.launch.py viz:=false          # no marker publisher

    # indoor integrated test -- see scripts/indoor_test.sh, which wraps this with the
    # config and the checks rather than leaving them to be remembered:
    ros2 launch dtp_bringup dtp_system.launch.py \
        config_name:=click_smooth_ros2.yaml

Each layer is a real launch file in this package that also runs standalone, so a bench session
can bring the layers up in separate terminals and watch each one's console independently:

    sensors.launch.py         Livox Mid-360 + Orbbec Gemini 336L
    super.launch.py           FAST-LIO2 -> ROG-Map -> fsm_node (ROG-Map is inside fsm_node)
    follower.launch.py        fusion -> perception -> target_manager -> follow_planner -> mux
    sensor_watchdog.launch.py stall detection, started HERE and not inside follower (below)
    dtp_viz.launch.py         MarkerArray converter + dtp_image_overlay (boxes on the camera
                              picture), both dtp_perception; RViz NOT started here

WHY THE VIZ LAYER IS HERE AND DEFAULTS ON
-----------------------------------------
dtp_viz is the only way to SEE what this stack believes -- RViz cannot render
dtp_msgs/TargetArray, the lock and arbitration state are std_msgs/String, and the proximity
keep-out is a rule inside command_mux rather than a published shape. It is also strictly
passive: it publishes /dtp/viz and nothing in the flight path subscribes, so the cost of having
it on is one small node and the cost of having it off is a test you cannot watch. The same layer
starts dtp_image_overlay, which costs nothing until a laptop subscribes to the picture
(scripts/laptop_dtp_viewer.sh) and ~17 % of a core while one does.

It starts the CONVERTER, never RViz (dtp_viz.launch.py rviz:=false is its own default). The
Jetson has no display and rendering competes for the GPU with the detector, which already
saturates its executor. RViz belongs on the operator laptop, pointed at this same DDS domain.

WHY THE WATCHDOG IS NOT PART OF THE FOLLOWER LAYER HERE
-------------------------------------------------------
`watchdog` is a top-level toggle, defaulted true, and this file explicitly passes
`watchdog:=false` down to follower.launch.py so the node is started exactly once. The point is
that turning the follower OFF must not turn stall detection off with it. A NAV-only flight is
still a flight, and a silently stalled camera or a dead FAST-LIO is exactly as dangerous there.
If the watchdog rode along inside the follower layer, `follower:=false` would quietly remove the
only thing in the system that notices a sensor that stopped publishing while its process, its
USB enumeration and its topic registrations all still look healthy (2026-09-07, Orbbec).

ORDERING: there is none, deliberately
--------------------------------------
No delays or event handlers sequence these layers. ROS 2 nodes come up in any order and the
stack is built for it -- fsm_node refuses to step the FSM while odom is older than 0.1 s
(fsm.cpp:100), rog_map has a 2.0 s odom timeout, and sensor_watchdog has a 20 s startup grace
precisely so start-up ordering does not read as a fault. Adding artificial sequencing here
would hide a real ordering dependency rather than fix it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(launch_file, condition_arg, forward, package='dtp_bringup'):
    """Include a layer, resolving its package share dir ONLY IF the layer is enabled.

    *** FindPackageShare, NOT get_package_share_directory. ***  The eager form resolved the
    path while the LaunchDescription was being BUILT, which is before any condition is
    evaluated -- so a missing package raised PackageNotFoundError and killed the entire
    bring-up even with that layer turned off. `viz:=false` could not protect anything.
    FindPackageShare is a substitution, performed only when the action actually executes,
    which is exactly when the condition is true. That matters because viz reaches into a
    SECOND package (dtp_perception) whose install tree can legitimately be older than
    dtp_bringup after a --packages-select rebuild.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=forward,
        condition=IfCondition(LaunchConfiguration(condition_arg)),
    )


def generate_launch_description():
    # Every layer gets use_sim_time forwarded rather than each defaulting it independently.
    # SUPER's yaw-target staleness gate compares fsm_node's clock against follow_planner's
    # stamps with a 0.5 s budget, so a stack half on sim time and half on wall clock fails that
    # gate on every message, forever, and silently reverts to velocity-aligned yaw. One knob,
    # forwarded everywhere, is the only way that cannot happen by accident. (preflight refuses
    # true outright -- see dtp_bringup.preflight.assert_wall_clock for why it is not supportable.)
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_name = LaunchConfiguration('config_name')
    super_config_file = LaunchConfiguration('super_config_file')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true',
                              description='Livox + Orbbec drivers'),
        DeclareLaunchArgument('super', default_value='true',
                              description='FAST-LIO2 + ROG-Map + fsm_node'),
        DeclareLaunchArgument('follower', default_value='true',
                              description='fusion, perception, target_manager, follow_planner, '
                                          'command_mux'),
        DeclareLaunchArgument('watchdog', default_value='true',
                              description='sensor_watchdog; independent of `follower` on '
                                          'purpose -- see this file\'s docstring'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='dtp_viz MarkerArray converter. Passive: publishes '
                                          '/dtp/viz and nothing in the flight path reads it. '
                                          'Does NOT start RViz -- run that on the laptop'),

        # *** THIS WAS MISSING, AND ITS ABSENCE MADE A SAFETY CLAIM FALSE. ***
        #
        # follower.launch.py has had observe_only since Stage A: it redirects the arbitrated
        # goal to /dtp/shadow/follow_goal AND sets command_mux abort_enable false, which is
        # what removes the /fmu/in/vehicle_command publisher. But dtp_system.launch.py neither
        # declared nor forwarded it, so EVERY bring-up through this file was live: command_mux
        # publishing real goals on /planning/follow_goal (which the deployed
        # click_smooth_ros2.yaml subscribes, follow_goal_en true since 2026-09-10) and holding
        # a VehicleCommand publisher armed to send NAV_LAND.
        #
        # scripts/indoor_test.sh told its operator the opposite in capital letters. Anything
        # that is meant to observe rather than command MUST pass observe_only:=true here --
        # and, because an assertion is not a check, verify it with scripts/stage_a_preflight.py.
        DeclareLaunchArgument('observe_only', default_value='false',
                              description='forward to follower.launch.py: send goals to '
                                          '/dtp/shadow/* instead of SUPER and disable the '
                                          'NAV_LAND abort. Use true for any run that must '
                                          'not be able to command the aircraft'),
        DeclareLaunchArgument('loss_conditioning', default_value='',
                              description='forward to follower.launch.py: the M6.5 ablation '
                                          'switch on target_manager AND follow_planner_node '
                                          '(true / false; empty leaves dtp_params.yaml in '
                                          'charge, i.e. false = the carry-forward arm)'),
        DeclareLaunchArgument('goal_mode', default_value='',
                              description='forward to follower.launch.py: follow_planner_node '
                                          'goal_mode (geometric / nmpc; empty leaves '
                                          'dtp_params.yaml in charge, i.e. geometric)'),
        DeclareLaunchArgument('viz_goal_topic', default_value='/dtp/shadow/follow_goal',
                              description='which goal stream dtp_viz draws. Default is the '
                                          'Stage A shadow topic; use /planning/follow_goal '
                                          'for a live follow'),

        # Forwarded to sensors.launch.py, which already gates each driver on them. Needed
        # because the field configuration is Kevin's stack underneath ours, and his run.sh
        # launches ONLY the Livox driver: riding on it means lidar:=false with the camera
        # still ours. sensors:=false is the wrong knob for that -- it takes the camera down
        # too, and the first live run (2026-09-12) spent 60 s waiting for images that
        # nothing was going to publish.
        DeclareLaunchArgument('lidar', default_value='true',
                              description='start our Livox driver. false when riding on an '
                                          'already-running one (Kevin\'s run.sh)'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='start the Orbbec driver'),
        DeclareLaunchArgument('camera_serial', default_value='CPC64630007M'),
        DeclareLaunchArgument('color_mjpeg_quality', default_value='80'),
        # 1, NOT 0 -- sensors.launch.py's docstring has why. This default was '0' and overrode
        # sensors.launch.py's verified 1. On 2026-09-12 this file was launched on top of
        # run.sh's stack: its driver took the MID360's stream and published PointCloud2 --
        # which FAST-LIO (lidar_type 1, CustomMsg) cannot receive -- while /livox/imu kept flowing.
        # FAST-LIO integrated IMU alone for 225 s; the aircraft was armed on a 3.74 km error.
        DeclareLaunchArgument('xfer_format', default_value='1'),
        DeclareLaunchArgument('livox_config', default_value=''),

        DeclareLaunchArgument('config_name', default_value='follow_test_ros2.yaml'),
        DeclareLaunchArgument('super_config_file', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('calib_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('skip_preflight', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_cfg', default_value=''),

        # Empty strings are forwarded freely because each sub-launch resolves its own default
        # from an empty value at launch time. That is why sensors.launch.py resolves
        # livox_config in an OpaqueFunction instead of on the DeclareLaunchArgument: an empty
        # user_config_path reaching the driver makes it read no config at all and never find the
        # sensor, which presents as a network fault rather than a launch bug.
        _include('sensors.launch.py', 'sensors', {
            'lidar': LaunchConfiguration('lidar'),
            'camera': LaunchConfiguration('camera'),
            'camera_serial': LaunchConfiguration('camera_serial'),
            'color_mjpeg_quality': LaunchConfiguration('color_mjpeg_quality'),
            'xfer_format': LaunchConfiguration('xfer_format'),
            'livox_config': LaunchConfiguration('livox_config'),
        }.items()),

        # require_follow_path is the `follower` toggle, forwarded. The gate that refuses a
        # config with fsm/follow_goal_en false exists to protect the follow chain, so it should
        # fire exactly when a follower was asked for -- and not turn a deliberate NAV-only
        # bring-up into a launch failure.
        _include('super.launch.py', 'super', {
            'config_name': config_name,
            'super_config_file': super_config_file,
            'use_sim_time': use_sim_time,
            'require_follow_path': LaunchConfiguration('follower'),
            'skip_preflight': LaunchConfiguration('skip_preflight'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_cfg': LaunchConfiguration('rviz_cfg'),
        }.items()),

        # watchdog:=false is hardcoded, not forwarded: this file owns the watchdog so it is
        # started exactly once regardless of the `follower` toggle.
        #
        # *** SCOPED, AND THAT IS THE WHOLE FIX. ***  Launch configurations are GLOBAL across
        # includes: the launch_arguments below are applied as SetLaunchConfiguration in the
        # shared context, not in a private one. So this include set watchdog=false for
        # everyone, and the top-level watchdog include a few lines down -- evaluated AFTER
        # this one -- read that false in its IfCondition and never started. The mechanism
        # meant to start the watchdog exactly once started it zero times, in every bring-up
        # through this file since it was written. Nothing logged it: a skipped include is
        # silent, and the symptom was one line in the mux state -- "/dtp/sensors_ok never
        # received (sensor_watchdog down?)" -- which read like a startup race.
        #
        # Found 2026-09-12 by scripts/indoor_test.sh, confirmed by follower:=false making the
        # watchdog appear. GroupAction(scoped=True) pushes a configuration scope for the
        # include; forwarding=True still lets it see the parent values it needs.
        GroupAction([
            _include('follower.launch.py', 'follower', {
                'params_file': params_file,
                'calib_file': LaunchConfiguration('calib_file'),
                'config_name': config_name,
                'super_config_file': super_config_file,
                'use_sim_time': use_sim_time,
                'watchdog': 'false',
                'observe_only': LaunchConfiguration('observe_only'),
                'loss_conditioning': LaunchConfiguration('loss_conditioning'),
                'goal_mode': LaunchConfiguration('goal_mode'),
            }.items()),
        ], scoped=True, forwarding=True),

        _include('sensor_watchdog.launch.py', 'watchdog', {
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items()),

        # dtp_perception, not dtp_bringup -- the converter lives with the messages it
        # converts. rviz is NOT forwarded: this launch never starts a renderer (see the
        # docstring), so the only thing to pass down is which goal stream to draw.
        _include('dtp_viz.launch.py', 'viz', {
            'goal_topic': LaunchConfiguration('viz_goal_topic'),
            'rviz': 'false',
        }.items(), package='dtp_perception'),
    ])
