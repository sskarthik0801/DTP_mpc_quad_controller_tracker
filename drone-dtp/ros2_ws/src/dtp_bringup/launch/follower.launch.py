"""Follower layer: fusion -> perception -> target_manager -> follow_planner -> command_mux.

    ros2 launch dtp_bringup follower.launch.py
    ros2 launch dtp_bringup follower.launch.py perception:=false     # replay/bench

Assumes the sensor and SUPER layers are already up. sensor_watchdog comes up with this layer by
default (`watchdog:=false` to suppress it, e.g. when dtp_system.launch.py is already running one).

THE THREE TOPIC NAMES THAT MUST AGREE WITH SUPER, AND WHY THIS LAUNCH SETS THEM
-------------------------------------------------------------------------------
command_mux and follow_planner both default their topics correctly on their own; this launch
does not fix a break. What it does is bind them to the SAME SOURCE OF TRUTH fsm_node uses --
the SUPER config -- so the two sides cannot drift apart later. Three names have to match, and
every one of them fails silently when it does not:

  fsm/follow_goal_topic  -> command_mux `arbitrated_goal_topic`
      fsm_node's /goal_pose subscriber is the RViz CLICK path, setGoalPosiAndYaw: it OVERWRITES
      goal Z with cfg_.click_height (1.5 m, destroying the D6 matched-height trail), snaps
      within 3 m, silently returns on a "deeply occupied" goal, skips any goal within 0.1 m of
      the robot, and publishes NO GoalStatus at all. Only setFollowGoal, on
      fsm/follow_goal_topic, preserves Z and answers. Click goals keep working regardless:
      fsm/click_goal_en stays true and /goal_pose stays live for bench RViz clicking.
  fsm/goal_status_topic  -> follow_planner `goal_status_topic`
      Listening on the wrong name is indistinguishable from fsm_node never answering, which is
      the same symptom as fsm/follow_goal_en being false.
  fsm/yaw_target_topic   -> follow_planner `yaw_target_topic`
      With nothing arriving, gi_.yaw_target_valid stays false and generateExpTraj falls through
      to velocity-aligned yaw: the drone flies a perfect standoff and never once looks at the
      person, with no warning printed. That is risk R-F10, the exact failure YAW_TO_TARGET
      exists to prevent.

These go in as PARAMETERS, not remaps. Both nodes expose the topics as parameters deliberately
(command_mux_node.py's arbitrated_goal_topic exists so a bench rig can point the output back at
/goal_pose), and a launch-level remap would silently override that choice instead of honouring
it. Pass `mux_goal_topic:=` / `yaw_target_topic:=` explicitly to break the coupling to SUPER's
config on purpose.

WHAT IS STILL MISSING FOR PERCEPTION, AND IS NOT THIS FILE'S TO INVENT
----------------------------------------------------------------------
perception_node looks up odom -> camera at the CLOUD timestamp to build T_odom_camera. Nothing
in this repository publishes that chain today: FAST-LIO stamps its odometry in `camera_init`
(not `odom`), and there is no base_link -> camera static transform anywhere, because the
calibration we have is T_camera_lidar (Livox frame -> camera optical frame) rather than a
body-frame mount. `odom_frame` is set to camera_init in config/dtp_params.yaml, which is
verified; `camera_frame` is deliberately NOT set, because guessing the Orbbec optical frame
name would produce a lookup that fails in exactly the same way as the current wrong default
while looking like it had been checked. Read it off the aircraft:
    ros2 topic echo --field header.frame_id /lidar_slam/odom
    ros2 run tf2_tools view_frames
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

from dtp_bringup import preflight

# 'auto' means "take it from SUPER's config". A sentinel rather than the literal default so an
# operator who passes the topic explicitly is never silently overridden by a config file.
AUTO = 'auto'


# Stage A (observe-only flight). Everything the follow stack would send to SUPER is redirected
# under this namespace, where nothing subscribes. The stack runs, locks, computes goals and yaw
# targets, and every one of them is RECORDED and none of them is FLOWN. Chosen over disabling
# command_mux / follow_planner because the point of Stage A is to see exactly what the aircraft
# WOULD have been told, on a flying platform, with the real latency and the real odometry --
# which means the whole chain has to run. scripts/stage_a_preflight.py verifies on the live graph
# that no node of ours publishes to any /planning or /fmu/in topic before anyone arms.
SHADOW_NS = '/dtp/shadow'


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    actions = []
    observe_only = str(arg('observe_only')).strip().lower() in ('1', 'true', 'yes', 'on')

    # detection_pipeline/ IS NOT A ROS PACKAGE. It sits at the repository root, ported unchanged
    # from the reference repo (CLAUDE.md section 7) so upstream fixes can be pulled in without a
    # merge. colcon therefore never installs it, and an installed perception_node dies on import
    # with ModuleNotFoundError -- which is what happened on the first end-to-end launch attempt,
    # 2026-09-08. Put the repo root on PYTHONPATH rather than vendoring a second copy into the
    # package, because a second copy is how the "ported unchanged" guarantee quietly dies.
    repo_root = arg('dtp_repo_root') or os.path.expanduser('~/super_ws/src/drone-dtp')
    if os.path.isdir(os.path.join(repo_root, 'detection_pipeline')):
        actions.append(SetEnvironmentVariable(
            'PYTHONPATH', repo_root + os.pathsep + os.environ.get('PYTHONPATH', '')))
    else:
        actions.append(LogInfo(
            msg='[dtp_bringup] detection_pipeline NOT found under %s -- perception_node will die '
                'on import. Pass dtp_repo_root:=<path to the drone-dtp checkout>.' % repo_root))

    params_file = arg('params_file')
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory('dtp_bringup'), 'config', 'dtp_params.yaml')

    preflight.assert_wall_clock(arg('use_sim_time'))

    goal_topic = arg('mux_goal_topic')
    yaw_topic = arg('yaw_target_topic')
    status_topic = arg('goal_status_topic')
    if AUTO in (goal_topic, yaw_topic, status_topic):
        cfg_path, searched, _stale = preflight.resolve_super_config(
            arg('config_name'), arg('super_config_file'))
        if cfg_path is None:
            actions.append(LogInfo(
                msg='[dtp_bringup] could not read SUPER config %s (looked in: %s); falling back '
                    'to the M0.7 fixture topic names. If fsm_node was configured with different '
                    'ones, the follow chain will be silently open-loop.'
                    % (arg('config_name'), ', '.join(searched))))
            cfg = {}
        else:
            cfg = preflight.load_super_config(cfg_path)
        auto_goal, auto_status, auto_yaw = preflight.follow_topics_from(cfg)
        goal_topic = auto_goal if goal_topic == AUTO else goal_topic
        yaw_topic = auto_yaw if yaw_topic == AUTO else yaw_topic
        status_topic = auto_status if status_topic == AUTO else status_topic

    mux_overrides = {'arbitrated_goal_topic': goal_topic}
    if observe_only:
        # Redirect BOTH outputs that reach fsm_node. The yaw target is the one that is easy to
        # forget: with it live, SUPER turns the aircraft to face the person (YAW_TO_TARGET), which
        # is the follow stack commanding the drone just as surely as a goal would.
        goal_topic = SHADOW_NS + '/follow_goal'
        yaw_topic = SHADOW_NS + '/yaw_target'
        # No land-on-sensor-loss either: in Stage A this node has no authority over the aircraft
        # (someone else's mission is flying it) and landing that mission because OUR camera died
        # would be wrong. command_mux logs this choice at startup.
        mux_overrides = {'arbitrated_goal_topic': goal_topic, 'abort_enable': False}
        actions.append(LogInfo(
            msg='[dtp_bringup] *** OBSERVE-ONLY (Stage A) *** the follow stack COMMANDS NOTHING: '
                'goals -> %s, yaw -> %s, abort disabled. Run scripts/stage_a_preflight.py on the '
                'live graph before arming; it refuses if anything of ours reaches /planning or '
                '/fmu/in.' % (goal_topic, yaw_topic)))

    actions.append(LogInfo(
        msg='[dtp_bringup] follow goal -> %s ; goal status <- %s ; yaw target -> %s'
            % (goal_topic, status_topic, yaw_topic)))

    # THE BRAKING ENVELOPE (2026-09-27; command_mux's brake_envelope_enable in dtp_params.yaml).
    # command_mux assumes SUPER never plans a stop harder than its own max_acc and refuses to start
    # if its braking deceleration says otherwise. The number is read HERE, from the same config
    # fsm_node reads, for the same reason as the topics above: one source of truth. Unreadable: the
    # node is told nothing, logs the envelope as NOT KNOWN, and skips that check.
    mux_super = []
    env_path, _searched, _stale = preflight.resolve_super_config(
        arg('config_name'), arg('super_config_file'))
    if env_path is not None:
        env_cfg = preflight.load_super_config(env_path)
        max_acc = dict(preflight.read_speed_envelope(env_cfg)).get('traj_opt/boundary/max_acc')
        if max_acc is not None and max_acc > 0.0:
            mux_super = [{'super_max_acc_mps2': float(max_acc)}]
        actions.append(LogInfo(
            msg='[dtp_bringup] SUPER max_acc for the braking envelope: %s (from %s)'
                % (max_acc if max_acc is not None else 'NOT READABLE', env_path)))

    # `name=` is pinned on every Node below to the name that node gives itself in code. It looks
    # redundant and is not: config/dtp_params.yaml is keyed by node name, and a ROS 2 parameter
    # file whose key does not match the running node's name is ignored SILENTLY -- every value
    # in it quietly reverts to the node's own default. Pinning the name here makes that coupling
    # explicit in one place instead of depending on two files agreeing by accident.
    common = [params_file, {'use_sim_time': False}]

    # calib_file is appended only when non-empty. Passing '' would OVERRIDE calib_io's
    # share-relative default with an empty string, and fusion_node refuses to start without a
    # calibration by design -- it would rather die than run with an implicit identity extrinsic.
    fusion_params = list(common)
    if arg('calib_file'):
        fusion_params.append({'calib_file': arg('calib_file')})
    actions.append(Node(
        package='dtp_perception', executable='fusion_node', name='fusion_node',
        parameters=fusion_params, output='screen',
        condition=IfCondition(LaunchConfiguration('fusion')),
    ))

    # static_calib_tf BRIDGES THE ONLY GAP IN THE TF CHAIN, and without it the entire follower is
    # inert. FAST-LIO publishes camera_init -> <body>; the Orbbec driver publishes
    # camera_link -> camera_color_optical_frame; NOTHING joins them. perception_node's lookup then
    # raises LookupException on every frame and it publishes an EMPTY TargetArray at 15 Hz -- which
    # is indistinguishable on the wire from "nobody is here", so the drone reads a blind pipeline
    # as a calm one. It takes the same calib params as fusion_node because it derives the bridge
    # from T_camera_lidar through calib_io, gates included.
    #
    # ORDER MATTERS: it must come up after the camera driver, because it cross-checks its own
    # frames against the live TF tree at start-up. Launching it here (before perception_node)
    # rather than in sensors.launch.py keeps it with the consumer that needs it.
    actions.append(Node(
        package='dtp_perception', executable='static_calib_tf', name='static_calib_tf',
        parameters=fusion_params, output='screen',
        condition=IfCondition(LaunchConfiguration('perception')),
    ))

    actions.append(Node(
        package='dtp_perception', executable='perception_node', name='perception_node',
        parameters=common, output='screen',
        condition=IfCondition(LaunchConfiguration('perception')),
    ))

    # M6.5: ONE launch argument sets the ablation switch on BOTH nodes that read it. An empty
    # value leaves dtp_params.yaml in charge (both false there). Setting it here rather than in
    # the file is what keeps the two arms one binary: the file could be edited on one node only.
    conditioning_override = []
    if arg('loss_conditioning').strip():
        conditioning_on = arg('loss_conditioning').strip().lower() in ('1', 'true', 'yes', 'on')
        conditioning_override = [{'loss_conditioning': conditioning_on}]
        actions.append(LogInfo(
            msg='[dtp_bringup] loss_conditioning:=%s on target_manager AND follow_planner_node '
                '(M6.5 arm %s)'
                % (arg('loss_conditioning'), 'ON' if conditioning_on else 'OFF -- carry-forward')))

    # goal_mode: which implementation answers follow_planner's standoff goal. Same shape as the
    # switch above and for the same reason -- one binary, the arm chosen on the command line and
    # printed here, so a bag can be attributed to the arm that flew it. follow_planner reads it
    # at construction: changing the arm is a relaunch, `ros2 param set` does nothing.
    goal_mode_override = []
    if arg('goal_mode').strip():
        goal_mode_override = [{'goal_mode': arg('goal_mode').strip()}]
        actions.append(LogInfo(
            msg='[dtp_bringup] goal_mode:=%s on follow_planner_node' % arg('goal_mode').strip()))

    actions.append(Node(
        package='dtp_perception', executable='target_manager', name='target_manager',
        parameters=common + conditioning_override, output='screen',
        condition=IfCondition(LaunchConfiguration('target_manager')),
    ))

    # follow_planner's own goal output stays on its default /dtp/follow_goal -- command_mux must
    # arbitrate it, not SUPER receive it directly. Only the two topics shared with fsm_node are
    # pinned here. Note these overrides come AFTER the params file in the list, which is how ROS
    # 2 resolves them: later entries win, so an operator's explicit :=  argument beats the YAML.
    actions.append(Node(
        package='follow_planner', executable='follow_planner_node', name='follow_planner_node',
        parameters=common + conditioning_override + goal_mode_override + [
            {'yaw_target_topic': yaw_topic, 'goal_status_topic': status_topic}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('follow_planner')),
    ))

    actions.append(Node(
        package='command_mux', executable='command_mux_node', name='command_mux',
        parameters=common + [mux_overrides] + mux_super, output='screen',
        condition=IfCondition(LaunchConfiguration('command_mux')),
    ))

    # terrain_estimator runs only when command_mux will use it: ONE switch, command_mux's
    # terrain_follow_enable in the params file, read here rather than duplicated as a launch
    # argument that could disagree with it. Passive -- it publishes /dtp/terrain and nothing else.
    try:
        with open(params_file) as f:
            doc = yaml.safe_load(f) or {}
        mux_params = doc.get('command_mux', {}).get('ros__parameters', {})
        terrain_on = bool(mux_params.get('terrain_follow_enable', False))
    except (OSError, yaml.YAMLError) as exc:
        terrain_on = False
        actions.append(LogInfo(
            msg='[dtp_bringup] could not read terrain_follow_enable from %s (%s); '
                'terrain_estimator not started' % (params_file, exc)))
    if terrain_on and arg('command_mux').strip().lower() in ('1', 'true', 'yes', 'on'):
        actions.append(LogInfo(
            msg='[dtp_bringup] terrain_follow_enable true: FOLLOW height above the LOCAL ground; '
                'starting terrain_estimator (bench-gated, never flown)'))
        actions.append(Node(
            package='command_mux', executable='terrain_estimator', name='terrain_estimator',
            parameters=common, output='screen',
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('dtp_bringup'), 'launch', 'sensor_watchdog.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': arg('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('watchdog')),
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fusion', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('target_manager', default_value='true'),
        DeclareLaunchArgument('follow_planner', default_value='true'),
        DeclareLaunchArgument('command_mux', default_value='true'),
        # Defaults true even standalone: sensor_watchdog is the only thing in the stack that
        # notices a sensor that has gone silent while its process still looks healthy, which is
        # how the Orbbec driver failed on 2026-09-07 (topics registered, USB enumerated, uptime
        # climbing, nothing logged). A follower with a stalled camera keeps commanding from its
        # last known target while the person walks away.
        DeclareLaunchArgument(
            'watchdog', default_value='true',
            description='start sensor_watchdog; leave true for anything that flies'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='parameter YAML; defaults to dtp_bringup share config/dtp_params.yaml'),
        DeclareLaunchArgument(
            'dtp_repo_root', default_value='',
            description='drone-dtp checkout root, put on PYTHONPATH so perception_node can import '
                        'detection_pipeline/ (not a ROS package, so colcon never installs it). '
                        'Empty uses ~/super_ws/src/drone-dtp'),
        DeclareLaunchArgument(
            'calib_file', default_value='',
            description='camera<->LiDAR calibration for fusion_node; empty uses calib_io\'s '
                        'share-relative default (a BARE FILENAME -- a relative path resolves '
                        'against the process working directory and will not be found on the '
                        'aircraft)'),
        DeclareLaunchArgument(
            'mux_goal_topic', default_value=AUTO,
            description='where command_mux\'s arbitrated goal goes; "auto" reads '
                        'fsm/follow_goal_topic from the SUPER config'),
        DeclareLaunchArgument(
            'yaw_target_topic', default_value=AUTO,
            description='where follow_planner\'s yaw target goes; "auto" reads '
                        'fsm/yaw_target_topic from the SUPER config'),
        DeclareLaunchArgument(
            'goal_status_topic', default_value=AUTO,
            description='where follow_planner listens for GoalStatus; "auto" reads '
                        'fsm/goal_status_topic from the SUPER config'),
        DeclareLaunchArgument('config_name', default_value='follow_test_ros2.yaml'),
        DeclareLaunchArgument('super_config_file', default_value=''),
        DeclareLaunchArgument(
            'observe_only', default_value='false',
            description='Stage A: run the whole follow stack but redirect every output that '
                        'reaches SUPER (goal AND yaw target) under ' + SHADOW_NS + ' and disable '
                        'the land-on-sensor-loss abort. The stack records what it WOULD have '
                        'commanded and commands nothing. Verify with '
                        'scripts/stage_a_preflight.py.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='must be false -- see dtp_bringup.preflight.assert_wall_clock'),
        DeclareLaunchArgument(
            'loss_conditioning', default_value='',
            description='M6.5 ablation switch, applied to target_manager AND follow_planner_node '
                        'together: true = condition the coast, re-acquisition gate, yaw and '
                        'viewpoint on fsm_node\'s /planning/loss_event; false = the carry-forward '
                        'arm. Empty (default) leaves dtp_params.yaml in charge.'),
        DeclareLaunchArgument(
            'goal_mode', default_value='',
            description='follow_planner_node goal_mode: geometric (the closed-form standoff that '
                        'has flown) or nmpc (docs/fact-nmpc-follow-goal.md). Empty (default) '
                        'leaves dtp_params.yaml in charge, i.e. geometric.'),
        OpaqueFunction(function=_launch_setup),
    ])
