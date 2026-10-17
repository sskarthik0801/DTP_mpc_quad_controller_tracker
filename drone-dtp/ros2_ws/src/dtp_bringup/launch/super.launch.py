"""SUPER layer: FAST-LIO2 -> ROG-Map -> fsm_node.

    ros2 launch dtp_bringup super.launch.py
    # THIS repo copy of click_smooth_ros2.yaml has no follow path (the DEPLOYED one does, see
    # below), so for a bench click-goal run against our copy, say so:
    ros2 launch dtp_bringup super.launch.py \
        config_name:=click_smooth_ros2.yaml require_follow_path:=false

Assumes the sensor layer is already up (dtp_bringup sensors.launch.py). ROG-Map is NOT a
separate node -- it is compiled into fsm_node and configured from the same YAML, so there is
nothing to launch for it.

WHY THIS DOES NOT JUST INCLUDE super_bringup/launch/m1_super_lidar.launch.py
----------------------------------------------------------------------------
That file is the M1 state-estimation launch and it hardcodes `config_name: click_smooth_ros2.yaml`
for fsm_node. THIS REPO copy of click_smooth_ros2.yaml has no `fsm/follow_goal_en` at all, so it
defaults to false
(fsm/config.hpp:73) -- and with it false fsm_node creates neither the /planning/follow_goal
subscription nor the /planning/goal_status publisher. Including that launch would therefore
bring up a stack that looks completely healthy (map builds, FSM waits for a goal, RViz clicking
works) while the follow chain is open-loop: goals published into the void, no status ever
returned, every DROPPED/SNAPPED code path in follow_planner unreachable. That is precisely the
failure `preflight.assert_follow_path_enabled` refuses to launch into, so silently inheriting it
from an include would be the worst of both.

The FAST-LIO half IS duplicated from m1_super_lidar.launch.py rather than factored out, because
super_bringup is a different package with a different owner and the two launches legitimately
diverge at fsm_node. If the FAST-LIO half ever needs to change, it needs to change in both.

Gate check, same as M1's Gate C: in RViz with Fixed Frame `world`, rog_map/occ and
rog_map/inf_occ should fill in from the live cloud with no "odom timeout" or "below virtual
ground" warnings in the fsm_node console.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from dtp_bringup import preflight


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config_name = arg('config_name')
    actions = []

    # Preflight runs here, inside an OpaqueFunction, because launch arguments do not have
    # values until launch time -- generate_launch_description() only builds the description.
    if preflight.as_bool(arg('skip_preflight')):
        actions.append(LogInfo(msg='[dtp_bringup] preflight SKIPPED by request.'))
    else:
        preflight.assert_wall_clock(arg('use_sim_time'))

        cfg_path, searched, stale_risk = preflight.resolve_super_config(
            config_name, arg('super_config_file'))
        if cfg_path is None:
            # Not fatal. The follow chain may be perfectly fine; we just cannot prove it, and
            # saying so is more useful than either failing or staying quiet. The one thing not
            # to do is let the operator believe a check ran when it did not.
            actions.append(LogInfo(
                msg='[dtp_bringup] preflight COULD NOT CHECK %s -- fsm/follow_goal_en is '
                    'UNVERIFIED. Looked in: %s. Pass super_config_file:=<abs path> or set $%s '
                    'to the directory fsm_node reads.'
                    % (config_name, ', '.join(searched), preflight.SUPER_CONFIG_ENV)))
        else:
            cfg = preflight.load_super_config(cfg_path)
            # The gate fires only when a follower is actually expected. A click-goal or NAV-only
            # bench run legitimately uses click_smooth_ros2.yaml, which has no follow path at
            # all -- refusing that would just teach people to pass skip_preflight and lose the
            # wall-clock check with it. dtp_system.launch.py forwards its own `follower` toggle
            # here, so the two can never disagree about whether following was asked for.
            try:
                # Before the config check: a missing Livox overlay kills fast_lio with a
                # bare SIGSEGV that names nothing. Fail here with a sentence instead.
                preflight.assert_livox_typesupport()
                preflight.assert_follow_path_enabled(cfg, cfg_path)
            except preflight.PreflightError:
                if preflight.as_bool(arg('require_follow_path')):
                    raise
                actions.append(LogInfo(
                    msg='[dtp_bringup] %s has no follow path (fsm/follow_goal_en is not true). '
                        'Allowed because require_follow_path:=false -- this stack can accept '
                        'click and NAV goals but cannot follow anybody.' % cfg_path))
            # DELIBERATELY OUTSIDE the try/except above. require_follow_path:=false says this
            # bring-up is not following anybody; it does not say the speed limit stopped
            # mattering. See preflight.MAX_VEL_CEILING_DEFAULT for what this protects against
            # -- our own preflight message used to recommend a 50x max_vel increase.
            ceiling = float(arg('max_vel_ceiling'))
            preflight.assert_speed_envelope(cfg, cfg_path, ceiling)
            actions.append(LogInfo(
                msg='[dtp_bringup] speed envelope OK (ceiling %g m/s): %s'
                    % (ceiling, preflight.format_speed_envelope(cfg))))
            if ceiling > preflight.MAX_VEL_CEILING_DEFAULT:
                actions.append(LogInfo(
                    msg='[dtp_bringup] *** max_vel_ceiling RAISED to %g m/s, above the %g m/s '
                        'default. Nothing downstream clamps SUPER. ***'
                        % (ceiling, preflight.MAX_VEL_CEILING_DEFAULT)))
            if stale_risk:
                actions.append(LogInfo(
                    msg='[dtp_bringup] preflight read the INSTALLED copy %s. fsm_node reads the '
                        'source tree (ROOT_DIR), so this check passes against a file that may '
                        'be older than the one actually loaded.' % cfg_path))
            for warning in preflight.collect_follow_warnings(cfg):
                actions.append(LogInfo(msg='[dtp_bringup] WARNING: ' + warning))

    # NOT fast_lio's own share dir -- that holds the vendor fork's default mid360.yaml, a
    # different file with different (wrong for us) values. Ours is the repo-root config/,
    # installed under super_bringup's share dir, with the ros__parameters wrapper and
    # lidar_type 4 (MID360); lidar_type was 3 (OUST64, an Ouster) until 2026-08-08.
    fastlio_config = arg('fastlio_config')
    if not fastlio_config:
        fastlio_config = os.path.join(
            get_package_share_directory('super_bringup'), 'config', 'mid360.yaml')
    # One estimator, one planner. Two FAST-LIOs interleave two estimates on /lidar_slam/odom
    # (seen 2026-09-11 at "20 Hz"); two fsm_nodes double the 2 GB A* preallocation and publish
    # competing pos_cmd. When the M1 stack is already up, use it.
    for basename, what in (('fastlio_mapping', 'FAST-LIO'), ('fsm_node', 'fsm_node')):
        refusal = preflight.duplicate_refusal(basename, what)
        if refusal:
            actions.append(LogInfo(msg=refusal))
    fastlio_running = bool(preflight.running_pids('fastlio_mapping'))
    fsm_running = bool(preflight.running_pids('fsm_node'))

    # *** FAIL HERE, NOT AS A SIGSEGV. *** super_bringup installs no config/ on the aircraft, so
    # this default does not exist there. launch_ros only WARNS "Parameter file path is not a
    # file", starts fast_lio with no parameters at all, and it dies with exit code -11 in 0.6 s
    # (2026-09-12 20:00:32, the launch that also started the duplicate Livox driver). Only when
    # we would actually start one: riding on a running FAST-LIO needs no config.
    if not fastlio_running and not os.path.isfile(fastlio_config):
        raise preflight.PreflightError(
            'FAST-LIO config %s does not exist; launched without it fast_lio segfaults. Pass '
            'fastlio_config:=<file>. The one the flown M1 stack loads (m1_super_lidar.launch.py) '
            'is $(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml.' % fastlio_config)

    # use_sim_time is pinned false rather than forwarded: preflight has already refused any
    # other value, and passing it explicitly stops a stray global from re-introducing the epoch
    # mismatch described in preflight.assert_wall_clock.
    if not fastlio_running:
        actions.append(Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[fastlio_config, {'use_sim_time': False}],
            remappings=[('/Odometry', '/lidar_slam/odom')],
            output='screen',
        ))

    # FAST-LIO's world origin frame is `lidar_init` (upstream `camera_init`, renamed by
    # patches/fastlio_frames.py -- this line still said camera_init until 2026-09-13, which left
    # `world` a root with no path to anything FAST-LIO publishes); SUPER/ROG-Map visualise in
    # `world` (follow_test_ros2.yaml frame_id: "world"). Identity static TF so RViz on Fixed Frame
    # `world` can display the raw cloud and path. ROG-Map itself ignores frame_id and TF
    # entirely and treats the odom pose as world-frame, so this transform is for humans only --
    # which also means a wrong frame_id anywhere in the goal chain is accepted silently.
    # Only when we start FAST-LIO: riding on run.sh's stack, m1_super_lidar.launch.py already
    # publishes the same transform.
    if not fastlio_running:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'lidar_init'],
            output='log',
        ))

    if not fsm_running:
        actions.append(Node(
            package='super_planner',
            executable='fsm_node',
            name='fsm_node',
            parameters=[{'config_name': config_name, 'use_sim_time': False}],
            output='screen',
        ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # follow_test_ros2.yaml is click_smooth_ros2.yaml plus the DTP follow-goal path and
        # yaw_mode 3 (YAW_TO_TARGET). It is the M0.7 Step 0.7.4 fixture that passed on hardware
        # 2026-08-14, and the only config in THIS REPO with fsm/follow_goal_en: true.
        #
        # *** IT IS NOT THE CONFIG TO FLY. *** It carries max_vel 5.0 / yaw_dot_max 3.0, while
        # the DEPLOYED click_smooth_ros2.yaml on the aircraft carries max_vel 0.1 AND -- since
        # Kevin added it on 2026-09-10 -- fsm/follow_goal_en: true. The safe config is now also
        # the capable one. This default stays as the repo fixture because it is what a
        # workstation bench run has; preflight.assert_speed_envelope is what stops it reaching
        # the air.
        DeclareLaunchArgument(
            'config_name', default_value='follow_test_ros2.yaml',
            description='fsm_node config, resolved by SUPER against its own source config/ dir'),
        DeclareLaunchArgument(
            'super_config_file', default_value='',
            description='absolute path to the same file, used ONLY by preflight (fsm_node '
                        'resolves its own via the compile-time ROOT_DIR, which we cannot read)'),
        DeclareLaunchArgument(
            'fastlio_config', default_value='',
            description='override the FAST-LIO config; empty uses super_bringup mid360.yaml'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='must be false -- see dtp_bringup.preflight.assert_wall_clock'),
        DeclareLaunchArgument(
            'require_follow_path', default_value='true',
            description='fail the launch if fsm/follow_goal_en is not true; set false for a '
                        'deliberate click-goal or NAV-only bring-up'),
        DeclareLaunchArgument(
            'max_vel_ceiling', default_value=str(preflight.MAX_VEL_CEILING_DEFAULT),
            description='refuse to launch if traj_opt/boundary/max_vel exceeds this, in m/s. '
                        'Default is the project pre-Stage-B ceiling; raising it is logged'),
        DeclareLaunchArgument(
            'skip_preflight', default_value='false',
            description='bypass the launch-time config gates (bench use only)'),
        DeclareLaunchArgument('rviz', default_value='false', description='launch RViz'),
        DeclareLaunchArgument('rviz_cfg', default_value='', description='RViz config file'),
        OpaqueFunction(function=_launch_setup),
    ])
