#!/usr/bin/env python3
"""Stage A (observe-only flight) pre-arm check. Run on the aircraft with BOTH stacks up. GO / NO-GO.

WHAT STAGE A IS
---------------
The navigation stack flies its own mission. The follow stack runs alongside -- detects, locks,
computes standoff goals and yaw targets -- and every one of those is RECORDED and NONE is flown.
The question it answers is the one the bench cannot: do the goals we would have sent make sense
on a moving platform, with real latency and real odometry? Everything else about Stage A exists to
make sure the answer is obtained without the follow stack ever touching the aircraft.

WHY A SCRIPT AND NOT A CHECKLIST ITEM
-------------------------------------
"observe_only:=true" is a launch argument. A launch argument can be forgotten, mistyped, or
overridden by a YAML that someone edited last week -- and the consequence is the follow stack
commanding a drone that is flying someone else's mission. The only place the truth is visible is
the live ROS graph: WHO publishes to the topics SUPER and PX4 listen on. So this asks the graph,
not the config, and it refuses on the first thing it does not like. It also checks the handful of
settings that turn a flight-legal stack back into a bench one, because every one of them has been
flipped at least once this week.

Exit code 0 = GO. Anything else = NO-GO, with every failing line printed.

    python3 scripts/stage_a_preflight.py
    python3 scripts/stage_a_preflight.py --require-record   # also insist a bag is recording
"""
import argparse
import os
import subprocess
import sys
import time

# Nodes that belong to the follow stack. A publisher from any of these on a control topic is a
# NO-GO regardless of what the launch file claims.
OUR_NODES = ('command_mux', 'follow_planner_node', 'perception_node', 'target_manager',
             'sensor_watchdog', 'static_calib_tf', 'fusion_node')
# Topics through which a node could command the aircraft. /goal_pose and /dtp/nav_goal are the
# click path and the NAV source; /fmu/in/* is the autopilot directly.
CONTROL_TOPICS = ('/planning/follow_goal', '/planning/yaw_target', '/goal_pose', '/dtp/nav_goal',
                  '/fmu/in/vehicle_command', '/fmu/in/trajectory_setpoint',
                  '/fmu/in/offboard_control_mode', '/fmu/in/vehicle_visual_odometry')
SHADOW_NS = '/dtp/shadow'
# Exactly ONE publisher each, or two stacks are fighting over a sensor. MEASURED on the first
# dry run 2026-09-11: Kevin's stack had come up from the web GCS after a reboot, our sensors
# launch added a second Livox driver and a second FAST-LIO, /lidar_slam/odom read 20 Hz from two
# interleaved estimators, and static_calib_tf + perception_node refused to start (correctly).
# FAST-LIO's STAMPS, not its rate (2026-09-13, both flights' pre-arm): a laptop RViz subscribed
# to /cloud_registered over wifi (-72 dBm) blocked FAST-LIO's publisher; odom kept coming at 10 Hz
# while each stamp was up to 38 s old and growing, every camera frame failed the world lift and
# PX4 was fed stale poses -- and the rate check above passed. A Start at boot clocks does the
# same for a few minutes (backlog). So: nobody but the consumers we know may sit on FAST-LIO's
# outputs, and the stamp lag over the rate window must be small.
FASTLIO_ALLOWED_SUBS = {'/cloud_registered': ('fsm_node',), '/livox/lidar': ('laser_mapping',)}
ODOM_STAMP_LAG_MAX_S = 0.30

SINGLE_PUBLISHER_TOPICS = ('/livox/lidar', '/lidar_slam/odom', '/camera/color/camera_info',
                           '/camera/depth/camera_info', '/planning/pos_cmd',
                           '/fmu/in/trajectory_setpoint', '/fmu/in/vehicle_visual_odometry')
RATE_WINDOW_S = 3.0
# DDS discovery is not instantaneous and it is not monotonic under load. MEASURED 2026-09-11:
# two consecutive runs of this script against an UNCHANGED, healthy graph disagreed -- one
# reported 5 failures, the next none -- because a 2 s settle sampled the graph mid-discovery.
# A pre-arm check that flaps is WORSE THAN NO CHECK: the operator re-runs it until it says GO,
# which is a fail-open gate wearing a fail-closed costume. So every graph-dependent check now
# POLLS for up to DISCOVERY_WINDOW_S and fails only if the thing is still absent at the end.
# Conservative in the right direction: a genuinely missing publisher still fails, it just
# costs the window to say so.
DISCOVERY_WINDOW_S = 12.0
DISCOVERY_POLL_S = 0.5
GPU_DEVFREQ = '/sys/class/devfreq/17000000.gpu'
MIN_FREE_GB = 5.0
# fsm_node joins its config_name parameter onto its compile-time SOURCE config dir (ROOT_DIR).
SUPER_CFG_DIR = os.path.expanduser('~/super_ws/src/SUPER/super_planner/config')
# M5.3a: the topic fsm_node subscribes to when rog_map/target_exclusion is enabled.
TARGET_EXCLUSION_TOPIC_DEFAULT = '/dtp/follow_target'


class Report:
    def __init__(self):
        self.rows = []
        self.fails = 0
        self.warns = 0

    def ok(self, what, detail=''):
        self.rows.append(('PASS', what, detail))

    def warn(self, what, detail=''):
        self.rows.append(('WARN', what, detail))
        self.warns += 1

    def fail(self, what, detail=''):
        self.rows.append(('FAIL', what, detail))
        self.fails += 1

    def dump(self):
        w = max(len(r[1]) for r in self.rows) if self.rows else 20
        for state, what, detail in self.rows:
            mark = {'PASS': ' ok ', 'WARN': 'WARN', 'FAIL': 'FAIL'}[state]
            print('  [%s] %-*s  %s' % (mark, w, what, detail))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--require-record', action='store_true',
                    help='fail unless a `ros2 bag record` process is running')
    ap.add_argument('--datasets', default=os.path.expanduser('~/dtp_datasets'))
    ap.add_argument('--stage-b', action='store_true',
                    help='the follow stack HAS authority: goals reach SUPER. INVERTS the '
                         'authority section (our publishers must be present on the follow '
                         'topics, absent everywhere else), requires abort_enable TRUE, '
                         'auto_reacquire FALSE, and reads the limits from the config the '
                         'running fsm_node loaded. Requires --target.')
    ap.add_argument('--target', choices=('mannequin', 'person'),
                    help='what is being followed. Sets the max_vel ceiling and, for a person, '
                         'requires --waive-d21.')
    ap.add_argument('--max-vel', type=float, default=None,
                    help='ceiling for traj_opt/boundary/max_vel in the loaded SUPER config. '
                         'Default: 0.1 for --target person (Kevin gate-signed first-flight '
                         'value), 1.5 for mannequin (project pre-Stage-B ceiling).')
    ap.add_argument('--nmpc-cleared', action='store_true',
                    help='with --stage-b: goal_mode nmpc may have follow authority, because its '
                         'Stage A observe-only legs have been flown and scored '
                         '(analyze_shadow_goals.py exit 0 on both arms). Record where.')
    ap.add_argument('--waive-d21', action='store_true',
                    help='RECORD that the operator waives D21 (single-IMU FC, person in the '
                         'flight volume). Required with --target person. This is a decision, '
                         'and the gate makes it a logged one rather than a silent one.')
    ap.add_argument('--super-config', default=None,
                    help='the file fsm_node LOADED (it reads the source tree via ROOT_DIR, '
                         'so this is the truth about max_vel and yaw_mode, not install/share). '
                         'Default: ask the running /fsm_node for its config_name parameter and '
                         'resolve it under %s; a given path must agree with that' % SUPER_CFG_DIR)
    ap.add_argument('--estimator-window', type=float, default=60.0,
                    help='seconds the G0.2 estimator checks observe (default 60, the plan\'s '
                         '"no EKF2 reset in the last 60 s"; below 60 that check only WARNs)')
    args = ap.parse_args(argv)
    if args.stage_b and not args.target:
        ap.error('--stage-b requires --target mannequin|person')
    if args.target == 'person' and not args.stage_b:
        ap.error('--target person only means something with --stage-b')
    if args.max_vel is None:
        args.max_vel = 0.1 if args.target == 'person' else 1.5

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import Bool

    rep = Report()
    rclpy.init()
    node = Node('stage_a_preflight')

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def get_params(target, names, timeout=3.0):
        cli = node.create_client(GetParameters, target.rstrip('/') + '/get_parameters')
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        req = GetParameters.Request()
        req.names = list(names)
        fut = cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not fut.done() or fut.result() is None:
            return None
        out = {}
        for name, v in zip(names, fut.result().values):
            # rcl_interfaces/ParameterType: 1 BOOL, 2 INTEGER, 3 DOUBLE, 4 STRING, 8 DOUBLE_ARRAY
            out[name] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value,
                         4: v.string_value, 8: list(v.double_array_value)}.get(v.type)
        return out

    def await_graph(predicate):
        """True as soon as `predicate()` holds; False if it never does inside the window."""
        end = time.time() + DISCOVERY_WINDOW_S
        while time.time() < end:
            if predicate():
                return True
            spin(DISCOVERY_POLL_S)
        return False

    spin(2.0)   # a first settle; the polls below do the real waiting

    # -- 1. our nodes are up ------------------------------------------------------------------
    WANT = ('command_mux', 'follow_planner_node', 'perception_node', 'target_manager',
            'sensor_watchdog', 'static_calib_tf')
    await_graph(lambda: set(WANT).issubset(
        {nm for nm, _ns in node.get_node_names_and_namespaces()}))
    names = {nm for nm, _ns in node.get_node_names_and_namespaces()}
    for n in WANT:
        if n in names:
            rep.ok('node %s' % n, 'running')
        else:
            rep.fail('node %s' % n, 'NOT in the graph -- the follow stack is not fully up')

    # -- 2. THE authority check ------------------------------------------------------------------
    #
    # Stage A: nothing of ours publishes where the aircraft listens, and the shadow topics carry
    # our goals. Stage B/C is the INVERSE on exactly two topics -- command_mux MUST be on
    # /planning/follow_goal and follow_planner_node MUST be on /planning/yaw_target, or the
    # follow stack has no authority and the flight tests nothing -- plus the abort path,
    # command_mux on /fmu/in/vehicle_command (NAV_LAND on sensor loss). Everything ELSE stays
    # forbidden in both stages: we never go around SUPER to the click path or the autopilot.
    if args.stage_b:
        MUST_HAVE = {
            '/planning/follow_goal': 'command_mux',
            '/planning/yaw_target': 'follow_planner_node',
            '/fmu/in/vehicle_command': 'command_mux',
        }
        for topic, who in MUST_HAVE.items():
            await_graph(lambda tp=topic, w=who: w in
                        {p.node_name for p in node.get_publishers_info_by_topic(tp)})
        for topic in CONTROL_TOPICS:
            pubs = node.get_publishers_info_by_topic(topic)
            ours = sorted({p.node_name for p in pubs if p.node_name in OUR_NODES})
            theirs = sorted({p.node_name for p in pubs if p.node_name not in OUR_NODES})
            want = MUST_HAVE.get(topic)
            if want:
                if want in ours:
                    rep.ok('publishers on %s' % topic,
                           '%s -- the follow stack HAS AUTHORITY here%s'
                           % (want, (' (also: %s)' % ', '.join(theirs)) if theirs else ''))
                else:
                    rep.fail('publishers on %s' % topic,
                             '%s is NOT publishing -- no authority. Was it launched with '
                             'observe_only:=true?' % want)
            elif ours:
                rep.fail('publishers on %s' % topic,
                         'OURS: %s -- this path bypasses SUPER and is forbidden in every stage'
                         % ', '.join(ours))
            else:
                rep.ok('publishers on %s' % topic,
                       'none of ours' + (' (others: %s)' % ', '.join(theirs) if theirs else ''))
        # the shadow topics must be SILENT: a publisher there means the redirect is still on
        for topic in (SHADOW_NS + '/follow_goal', SHADOW_NS + '/yaw_target'):
            pubs = {p.node_name for p in node.get_publishers_info_by_topic(topic)}
            leak = sorted(pubs & set(OUR_NODES))
            if leak:
                rep.fail('shadow %s' % topic,
                         'still published by %s -- observe_only redirect is STILL IN EFFECT; '
                         'goals are going to the shadow, not to SUPER' % ', '.join(leak))
            else:
                rep.ok('shadow %s' % topic, 'silent -- goals are not being shadowed')
    else:
        for topic in CONTROL_TOPICS:
            pubs = node.get_publishers_info_by_topic(topic)
            ours = sorted({p.node_name for p in pubs if p.node_name in OUR_NODES})
            theirs = sorted({p.node_name for p in pubs if p.node_name not in OUR_NODES})
            if ours:
                rep.fail('publishers on %s' % topic,
                         'OURS: %s -- the follow stack CAN COMMAND THE AIRCRAFT. Relaunch with '
                         'observe_only:=true.' % ', '.join(ours))
            else:
                rep.ok('publishers on %s' % topic,
                       'none of ours' + (' (others: %s)' % ', '.join(theirs) if theirs else ''))
        # and the shadow topics DO have our publishers, proving the redirect is where goals went
        for topic, who in ((SHADOW_NS + '/follow_goal', 'command_mux'),
                           (SHADOW_NS + '/yaw_target', 'follow_planner_node')):
            await_graph(lambda tp=topic, w=who: w in
                        {p.node_name for p in node.get_publishers_info_by_topic(tp)})
            pubs = {p.node_name for p in node.get_publishers_info_by_topic(topic)}
            if who in pubs:
                rep.ok('shadow %s' % topic,
                       'published by %s -- goals go here, nowhere else' % who)
            else:
                rep.fail('shadow %s' % topic,
                         'no publisher from %s -- observe_only redirect not in effect' % who)

    # -- 2b. one stack per sensor ---------------------------------------------------------------
    for topic in SINGLE_PUBLISHER_TOPICS:
        await_graph(lambda tp=topic: len(node.get_publishers_info_by_topic(tp)) >= 1)
        pubs = node.get_publishers_info_by_topic(topic)
        who = sorted(p.node_name for p in pubs)
        if len(pubs) == 1:
            rep.ok('single publisher %s' % topic, who[0])
        elif not pubs:
            rep.fail('single publisher %s' % topic, 'NO publisher')
        else:
            rep.fail('single publisher %s' % topic,
                     '%d publishers (%s) -- two stacks own this sensor; kill the duplicate'
                     % (len(pubs), ', '.join(who)))

    # -- 3. the settings that make a stack flight-legal -----------------------------------------
    mux = get_params('/command_mux', ['abort_enable', 'arbitrated_goal_topic',
                                      'abort_require_offboard', 'engage_gate_enable'])
    if mux is None:
        rep.fail('command_mux parameters', 'parameter service not answering')
    else:
        if args.stage_b:
            if mux['abort_enable'] is True:
                rep.ok('command_mux abort_enable',
                       'TRUE -- sensor loss lands the aircraft (at the standoff, 3-4 m from '
                       'the target). Subject: step AWAY if it descends.')
            else:
                rep.fail('command_mux abort_enable',
                         'FALSE -- no land-on-sensor-loss with the follow stack in authority. '
                         'Was it launched with observe_only:=true?')
            if str(mux['arbitrated_goal_topic']) == '/planning/follow_goal':
                rep.ok('command_mux arbitrated_goal_topic', mux['arbitrated_goal_topic'])
            else:
                rep.fail('command_mux arbitrated_goal_topic',
                         '%s -- goals are not going to SUPER' % mux['arbitrated_goal_topic'])
            # 2026-09-22: goals reach SUPER only once PX4 is armed + Offboard, so SUPER's first
            # plan starts from the aircraft (the 20:05 engage flew a 3.0 m step). None = a mux
            # that predates the gate.
            if mux['engage_gate_enable'] is True:
                rep.ok('command_mux engage gate',
                       'ON -- nothing reaches SUPER until Offboard; expect a brief hover at the '
                       'switch')
            else:
                rep.fail('command_mux engage gate',
                         'OFF (engage_gate_enable %s) -- SUPER plans before Offboard and the '
                         'aircraft can lurch at the switch' % mux['engage_gate_enable'])
        else:
            if mux['abort_enable'] is False:
                rep.ok('command_mux abort_enable', 'false -- correct for Stage A (no authority)')
            else:
                rep.fail('command_mux abort_enable',
                         'TRUE -- the follow stack could LAND a mission it does not own')
            if str(mux['arbitrated_goal_topic']).startswith(SHADOW_NS):
                rep.ok('command_mux arbitrated_goal_topic', mux['arbitrated_goal_topic'])
            else:
                rep.fail('command_mux arbitrated_goal_topic',
                         '%s is NOT under %s' % (mux['arbitrated_goal_topic'], SHADOW_NS))
        if mux['abort_require_offboard'] is not True:
            rep.fail('command_mux abort_require_offboard',
                     'bench flag is DOWN -- never fly with this')

    # M5.2 geofence (2026-09-14): the ONLY fence on a follow flight -- PX4's is inert without a
    # global home. Checked in BOTH stages: in Stage A the shadow goals should show the clamp
    # behaving, and a stack that flew Stage A with it off would not be the stack flown in Stage B.
    fence = get_params('/command_mux', ['fence_enable', 'fence_x_min_m', 'fence_x_max_m',
                                        'fence_y_min_m', 'fence_y_max_m', 'fence_yaw_deg',
                                        'fence_margin_m', 'fence_breach_land_m'])
    if fence is None or fence.get('fence_enable') is None:
        rep.fail('command_mux geofence',
                 'no fence_* parameters -- this command_mux predates M5.2 (2026-09-14); rebuild')
    else:
        box = ('%.1f x %.1f m, x [%.1f, %.1f] y [%.1f, %.1f], yaw %.1f deg, margin %.2f m'
               % (fence['fence_x_max_m'] - fence['fence_x_min_m'],
                  fence['fence_y_max_m'] - fence['fence_y_min_m'],
                  fence['fence_x_min_m'], fence['fence_x_max_m'],
                  fence['fence_y_min_m'], fence['fence_y_max_m'],
                  fence['fence_yaw_deg'], fence['fence_margin_m']))
        if fence['fence_enable'] is not True:
            rep.fail('command_mux geofence', 'DISABLED (fence_enable false) -- box would be ' + box)
        elif fence['fence_breach_land_m'] < 0.0:
            rep.warn('command_mux geofence',
                     box + '; NO hard boundary (fence_breach_land_m < 0): goals are clamped but '
                     'the aircraft is never landed for leaving the box')
        else:
            rep.ok('command_mux geofence',
                   box + '; lands beyond +%.1f m. Walk the box before arming: it is in '
                   'lidar_init, so it moves with the start point and the boot heading.'
                   % fence['fence_breach_land_m'])

    per = get_params('/perception_node', ['tf_at_latest', 'odom_frame', 'max_pair_age_s'])
    if per is None:
        rep.fail('perception_node parameters', 'parameter service not answering')
    else:
        if per['tf_at_latest'] is False:
            rep.ok('perception tf_at_latest', 'false (stamped lookups)')
        else:
            rep.fail('perception tf_at_latest',
                     'TRUE -- bench shortcut; wrong by yaw-rate x range in flight')
        if per['odom_frame'] == 'lidar_init':
            rep.ok('perception odom_frame', 'lidar_init')
        else:
            rep.fail('perception odom_frame', '%r != lidar_init' % per['odom_frame'])

    # -- target_manager: the two settings that decide WHO gets followed ----------------------
    #
    # auto_reacquire_after_lost is the one flag this stack's own node refuses to stay quiet
    # about: target_manager logs an ERROR at startup when it is true -- "after a LOST this node
    # will re-acquire the nearest confirmed person WITHOUT operator re-designation. That
    # defeats D13/A10. Never fly this with a person in the volume." Until 2026-09-12 this gate
    # did not read it at all, so the one setting the node calls unflyable was the one setting
    # nobody checked before arming.
    #
    # It is TRUE in dtp_params.yaml on purpose, and the purpose is real: on 09-09 a single 0.2 s
    # forecast glitch became a 195 s permanent LOST because re-acquisition needed an operator
    # false->true edge that never came, and the whole run recorded zero follow goals. So the
    # verdict depends on who has authority:
    #   Stage A  -- goals go to the shadow topic, nothing we re-acquire onto is flown. WARN,
    #               loudly, so the flag does not ride into Stage B unnoticed.
    #   Stage B  -- goals reach SUPER. Re-acquiring onto "the nearest confirmed person" with no
    #               human choosing is exactly the failure D13 forbids. FAIL. (--stage-b)
    # -- 3a. which SUPER config the RUNNING fsm_node loaded ---------------------------------------
    #
    # Until 2026-09-13 this assumed click_smooth_ros2.yaml, which was true only because run.sh
    # could start nothing else. run.sh now takes SUPER_CONFIG (the M5 follow flights run
    # follow_test_ros2.yaml), so the file is read back off fsm_node's own config_name parameter.
    # fsm_node reads its config from the SOURCE tree via the compile-time ROOT_DIR, so that file is
    # the truth and the install/share copy can be arbitrarily stale.
    super_config = args.super_config
    fsm_p = get_params('/fsm_node', ['config_name'])
    if fsm_p and fsm_p.get('config_name'):
        loaded = os.path.normpath(os.path.join(SUPER_CFG_DIR, fsm_p['config_name']))
        if super_config and os.path.realpath(super_config) != os.path.realpath(loaded):
            rep.fail('SUPER config', '--super-config %s but fsm_node loaded %s -- checking the '
                     'wrong file proves nothing' % (super_config, loaded))
        super_config = loaded
        rep.ok('SUPER config', 'fsm_node loaded %s' % fsm_p['config_name'])
    else:
        super_config = super_config or os.path.join(SUPER_CFG_DIR, 'click_smooth_ros2.yaml')
        rep.warn('SUPER config', '/fsm_node config_name not readable; checking %s on trust'
                 % super_config)
    try:
        import yaml as _yaml
        with open(super_config) as fh:
            scfg = _yaml.safe_load(fh) or {}
    except Exception as exc:
        scfg = None
        rep.fail('SUPER config', 'cannot read %s: %s' % (super_config, exc))

    # -- 3a'. M5.3a target-point filter: who is kept out of the planner's map -------------------
    #
    # With rog_map/target_exclusion on, fsm_node drops the LOCKED person's returns before they
    # reach ROG-Map, so the planner does not avoid them. In a follow flight that is the design
    # (follow_planner's standoff and command_mux's proximity veto keep the drone off them). In
    # Stage A the NAVIGATION mission owns the aircraft and nothing of ours keeps it off the walker
    # -- the planner's map is the only thing that does. Checked two ways, because the config file
    # and the running node can disagree: the loaded file, and the running fsm_node. The node-side
    # evidence is the filter's OWN publisher, /rog_map/target_exclusion (ROGMapROS creates it only
    # when target_exclusion is on) -- NOT fsm_node's subscription to the target topic, which since
    # M6 (2026-09-14) the loss oracle holds too, with the filter off. The first bench run of the
    # M6 build failed this line on exactly that (docs/m6-loss-oracle.md).
    tex = ((scfg or {}).get('rog_map') or {}).get('target_exclusion') or {}
    tex_topic = str(tex.get('topic') or TARGET_EXCLUSION_TOPIC_DEFAULT)
    tex_cfg_on = bool(tex.get('enable', False))
    tex_subscribed = 'fsm_node' in {p.node_name for p in node.get_publishers_info_by_topic('/rog_map/target_exclusion')}
    if args.stage_b:
        if tex_cfg_on or tex_subscribed:
            rep.ok('target exclusion (M5.3a)',
                   'ON (%s) -- the followed person is NOT in the planner\'s map; the standoff and '
                   'the proximity veto keep the drone off them' % tex_topic)
        else:
            rep.warn('target exclusion (M5.3a)',
                     'off -- the followed person stamps occupied cells into ROG-Map as they walk '
                     '(docs/m5-m6-plan.md M5.3)')
    elif tex_cfg_on or tex_subscribed:
        rep.fail('target exclusion (M5.3a)',
                 'ON in Stage A (config %s, fsm_node publishing its marker %s): the walker is kept OUT of the '
                 'navigation planner\'s map, so the mission would fly through them. Set '
                 'rog_map/target_exclusion/enable false and restart fsm_node.'
                 % (tex_cfg_on, tex_subscribed))
    else:
        rep.ok('target exclusion (M5.3a)', 'off -- the walker is an obstacle to the mission planner')

    # -- 3b. Stage B/C: the limits the RUNNING fsm_node loaded, and the D21 decision -------------
    if args.stage_b:
        if scfg is not None:
            bnd = (scfg.get('traj_opt') or {}).get('boundary') or {}
            spl = scfg.get('super_planner') or {}
            fsm = scfg.get('fsm') or {}
            try:
                mv = float(bnd.get('max_vel'))
            except (TypeError, ValueError):
                mv = None
            if mv is None:
                rep.fail('SUPER max_vel', 'unreadable in %s' % super_config)
            elif mv <= args.max_vel:
                rep.ok('SUPER max_vel', '%.2f m/s <= %.2f ceiling for target=%s  '
                       '(acc %s omg %s yaw_dot %s)' % (mv, args.max_vel, args.target,
                                                       bnd.get('max_acc'), bnd.get('max_omg'),
                                                       spl.get('yaw_dot_max')))
            else:
                rep.fail('SUPER max_vel',
                         '%.2f m/s EXCEEDS the %.2f ceiling for target=%s. Nothing downstream '
                         'clamps it. Lower it in %s, or pass --max-vel deliberately.'
                         % (mv, args.max_vel, args.target, super_config))
            if int(spl.get('yaw_mode', 0) or 0) == 3:
                rep.ok('SUPER yaw_mode', '3 (YAW_TO_TARGET) -- the aircraft turns to face the target')
            else:
                rep.fail('SUPER yaw_mode',
                         '%s -- must be 3 (YAW_TO_TARGET) or the aircraft follows without ever '
                         'turning to face the target (R-F10). Kevin\'s file, Kevin\'s edit; then '
                         'dashboard Stop -> Start.' % spl.get('yaw_mode'))
            if bool(fsm.get('follow_goal_en', False)):
                rep.ok('SUPER follow_goal_en', 'true')
            else:
                rep.fail('SUPER follow_goal_en', 'not true -- fsm_node is not listening for goals')

        if args.target == 'person':
            if args.waive_d21:
                rep.warn('D21 WAIVED by operator',
                         'single-IMU FC with a PERSON in the flight volume. The LiDAR IMU feeds '
                         'FAST-LIO -> EKF2 as external vision (a pose measurement); it is NOT a '
                         'second IMU in the attitude loop. Mitigations that must hold: max_vel '
                         '<= %.2f, altitude <= 1.5 m, pilot hands-on with kill switch tested, '
                         'subject in PPE, nobody else within 15 m, subject steps AWAY on descent.'
                         % args.max_vel)
            else:
                rep.fail('D21',
                         'single-IMU FC and --target person. This is a go/no-go gate in the risk '
                         'register. Pass --waive-d21 to record the waiver explicitly, or use '
                         '--target mannequin.')

    # -- M6.5: the loss-conditioning arm has NEVER flown. Both nodes must agree, and only the
    #    carry-forward arm (false) may reach SUPER until it has (docs/m6-loss-oracle.md).
    lc_tm = get_params('/target_manager', ['loss_conditioning'])
    lc_fp = get_params('/follow_planner_node', ['loss_conditioning'])
    if lc_tm is None or lc_fp is None:
        rep.warn('loss_conditioning (M6.5)',
                 'could not read it from %s -- an older build without the M6 switch?'
                 % ('target_manager' if lc_tm is None else 'follow_planner_node'))
    else:
        a, b = lc_tm['loss_conditioning'], lc_fp['loss_conditioning']
        if a != b:
            rep.fail('loss_conditioning (M6.5)',
                     'target_manager %s but follow_planner_node %s -- the switch must be set on both '
                     '(launch with loss_conditioning:=true|false, never edit one node)' % (a, b))
        elif a is False:
            rep.ok('loss_conditioning (M6.5)',
                   'false on both -- the carry-forward arm; LossEvents are recorded, not acted on')
        elif args.stage_b:
            rep.fail('loss_conditioning (M6.5)',
                     'TRUE with follow authority. The conditioned arm has never flown (bench and '
                     'replay only, 2026-09-14); it is not a first-flight setting.')
        else:
            rep.warn('loss_conditioning (M6.5)',
                     'TRUE. Tolerable in Stage A only because goals are shadowed; it changes the '
                     'coast, the re-acquisition gate and the yaw target. --stage-b fails on it.')
    # -- goal_mode: which goal implementation is loaded. Printed on every run so a bag can be
    #    attributed to its arm. nmpc is bench-gated only (nmpc_check.py): harmless in Stage A,
    #    where goals are shadowed, and refused with authority until its Stage A legs are scored.
    gm = get_params('/follow_planner_node', ['goal_mode', 'stock_baseline', 'nmpc_fence_enable',
                                             'nmpc_fence_x_min_m', 'nmpc_fence_x_max_m',
                                             'nmpc_fence_y_min_m', 'nmpc_fence_y_max_m',
                                             'nmpc_fence_yaw_deg', 'nmpc_fence_margin_m'])
    if gm is None or gm.get('goal_mode') is None:
        rep.warn('goal_mode', 'follow_planner_node has no goal_mode -- a build from before the '
                              'switch existed; the goal is the geometric one')
    elif gm['goal_mode'] != 'nmpc':
        rep.ok('goal_mode', '%s -- the closed-form standoff goal' % gm['goal_mode'])
    elif gm.get('stock_baseline'):
        rep.fail('goal_mode', 'nmpc together with stock_baseline: the baseline wins and the goal '
                              'is geometric, so the run is not the arm its launch line says')
    elif args.stage_b and not args.nmpc_cleared:
        rep.fail('goal_mode', 'NMPC with follow authority. It is bench-gated only; fly its Stage A '
                              'observe-only legs, score them, then pass --nmpc-cleared.')
    else:
        rep.warn('goal_mode', 'NMPC%s' % (' (cleared by the operator for Stage B)' if args.stage_b
                                          else ' -- shadowed in Stage A, commands nothing'))
    if gm is not None and gm.get('nmpc_fence_enable') and gm.get('goal_mode') == 'nmpc':
        # The mirror exists so the solver stops asking for goals the mux will clamp. A mirror
        # that disagrees with the mux is a second source of truth, which is worse than none.
        pairs = [('nmpc_fence_x_min_m', 'fence_x_min_m'), ('nmpc_fence_x_max_m', 'fence_x_max_m'),
                 ('nmpc_fence_y_min_m', 'fence_y_min_m'), ('nmpc_fence_y_max_m', 'fence_y_max_m'),
                 ('nmpc_fence_yaw_deg', 'fence_yaw_deg'), ('nmpc_fence_margin_m', 'fence_margin_m')]
        bad = [a for a, b in pairs if fence is None or fence.get(b) is None or
               abs(gm[a] - fence[b]) > 1e-6]
        if bad:
            rep.fail('nmpc fence mirror', 'differs from command_mux on: %s' % ', '.join(bad))
        else:
            rep.ok('nmpc fence mirror', "equal to command_mux's box")
    # -- M6.1: fsm_node's oracle should be publishing, or the flight records no LossEvents.
    lo_pubs = node.get_publishers_info_by_topic('/planning/loss_event')
    if lo_pubs:
        rep.ok('loss oracle /planning/loss_event', 'published by %s' % ', '.join(p.node_name for p in lo_pubs))
    else:
        rep.warn('loss oracle /planning/loss_event',
                 'no publisher -- fsm_node was not started with loss_oracle/enable (follow_test_ros2.yaml '
                 'has it on). The flight will record no loss classifications.')

    tm = get_params('/target_manager', ['auto_reacquire_after_lost', 'launch_ignore_radius_m'])
    if tm is None:
        rep.fail('target_manager parameters', 'parameter service not answering')
    else:
        if tm['auto_reacquire_after_lost'] is False:
            rep.ok('target_manager auto_reacquire_after_lost',
                   'false -- a LOST needs an operator to re-designate (D13)')
        elif args.stage_b:
            rep.fail('target_manager auto_reacquire_after_lost',
                     'TRUE with follow authority -- after a LOST it will pick the nearest '
                     'person with no human choosing. Set false before Stage B.')
        else:
            rep.warn('target_manager auto_reacquire_after_lost',
                     'TRUE. Tolerable in Stage A only because goals are shadowed. MUST be '
                     'false before any run where goals reach SUPER (--stage-b fails on it).')
        r = tm['launch_ignore_radius_m']
        try:
            r = float(r)
        except (TypeError, ValueError):
            r = None
        if r is not None and r >= 3.0:
            rep.ok('target_manager launch_ignore_radius_m',
                   '%.1f m -- the safety pilot beside the aircraft cannot be locked (A-F13)' % r)
        else:
            rep.fail('target_manager launch_ignore_radius_m',
                     '%r -- below 3.0 m the first CONFIRMED person after engage can be whoever '
                     'is standing next to the aircraft' % (tm['launch_ignore_radius_m'],))

    # camera stamps from the host clock
    cam_full = None
    for n, ns in node.get_node_names_and_namespaces():
        if ns.startswith('/camera') and n not in ('camera_container',):
            cam_full = (ns.rstrip('/') + '/' + n)
            break
    if cam_full is None:
        rep.fail('camera node', 'no node under /camera -- is the Orbbec driver up?')
    else:
        cam = get_params(cam_full, ['time_domain'])
        if cam and cam.get('time_domain') == 'system':
            rep.ok('camera time_domain', 'system (host clock) on %s' % cam_full)
        elif cam is None:
            rep.warn('camera time_domain', 'could not read from %s' % cam_full)
        else:
            rep.fail('camera time_domain',
                     '%r -- must be system, or stamps are ~30 ms off SLAM' % cam.get('time_domain'))

    # the composed extrinsic matches what FAST-LIO runs
    sc = get_params('/static_calib_tf', ['lidar_to_body_t', 'lidar_to_body_r'])
    lm = get_params('/laser_mapping', ['mapping.extrinsic_T', 'mapping.extrinsic_R'])
    if sc is None or lm is None:
        rep.fail('LiDAR->body extrinsic', 'static_calib_tf or laser_mapping not answering')
    else:
        dt = max(abs(a - b) for a, b in zip(sc['lidar_to_body_t'], lm['mapping.extrinsic_T']))
        dr = max(abs(a - b) for a, b in zip(sc['lidar_to_body_r'], lm['mapping.extrinsic_R']))
        if dt < 1e-4 and dr < 1e-4:
            rep.ok('LiDAR->body extrinsic', 'static_calib_tf == FAST-LIO (|T| %.3f m)'
                   % sum(v * v for v in lm['mapping.extrinsic_T']) ** 0.5)
        else:
            rep.fail('LiDAR->body extrinsic', 'static_calib_tf and FAST-LIO DISAGREE (dT %.4f m, '
                     'dR %.2e) -- every world position is off by that' % (dt, dr))
    if lm is not None:
        ee = get_params('/laser_mapping', ['mapping.extrinsic_est_en'])
        if ee and ee.get('mapping.extrinsic_est_en') is True:
            rep.warn('FAST-LIO extrinsic_est_en', 'TRUE -- fine once airborne and moving (Kevin '
                     'flies with it); the composed body->camera transform is exact only to the '
                     'extent the online estimate stays near the datasheet value')

    # -- 4. sensors alive and the chain healthy -------------------------------------------------
    counts = {}
    last = {}

    def mk(topic):
        def cb(msg):
            counts[topic] = counts.get(topic, 0) + 1
            last[topic] = msg
        return cb
    from sensor_msgs.msg import CameraInfo
    from nav_msgs.msg import Odometry
    from dtp_msgs.msg import TargetArray
    from std_msgs.msg import String
    odom_lags = []
    odom_cb = mk('/lidar_slam/odom')

    def odom_lag_cb(msg):
        odom_cb(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom_lags.append(node.get_clock().now().nanoseconds * 1e-9 - stamp)
    keep_alive = [   # subscriptions must outlive the spin below
        node.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 mk('/camera/color/camera_info'), qos_profile_sensor_data),
        node.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 mk('/camera/depth/camera_info'), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/lidar_slam/odom', odom_lag_cb,
                                 qos_profile_sensor_data),
        node.create_subscription(TargetArray, '/dtp/targets', mk('/dtp/targets'),
                                 qos_profile_sensor_data),
        node.create_subscription(Bool, '/dtp/sensors_ok', mk('/dtp/sensors_ok'),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(Bool, '/dtp/perception_ok', mk('/dtp/perception_ok'),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(String, '/mux/state', mk('/mux/state'), 10),
    ]
    spin(RATE_WINDOW_S)
    del keep_alive
    for topic, nominal, floor in (('/camera/color/camera_info', 30.0, 20.0),
                                  ('/camera/depth/camera_info', 30.0, 20.0),
                                  ('/lidar_slam/odom', 10.0, 8.0),
                                  ('/dtp/targets', 10.0, 5.0)):
        hz = counts.get(topic, 0) / RATE_WINDOW_S
        if hz >= floor:
            rep.ok('rate %s' % topic, '%.1f Hz (nominal %.0f)' % (hz, nominal))
        else:
            rep.fail('rate %s' % topic, '%.1f Hz < %.0f floor' % (hz, floor))
    # FAST-LIO gate 1: its stamps. See FASTLIO_ALLOWED_SUBS.
    if odom_lags:
        lag_sorted = sorted(odom_lags)
        lag_p50, lag_max = lag_sorted[len(lag_sorted) // 2], lag_sorted[-1]
        if lag_max <= ODOM_STAMP_LAG_MAX_S:
            rep.ok('FAST-LIO stamp lag', 'p50 %.3f s, max %.3f s over %d msgs (limit %.2f)'
                   % (lag_p50, lag_max, len(odom_lags), ODOM_STAMP_LAG_MAX_S))
        else:
            rep.fail('FAST-LIO stamp lag',
                     'p50 %.3f s, MAX %.3f s (limit %.2f) -- FAST-LIO is BEHIND while its rate '
                     'reads fine: every camera frame fails the world lift and PX4 gets stale '
                     'poses. Seen '
                     'causes: a laptop RViz on /cloud_registered (close it, wait 20 s), or a '
                     'Start at '
                     'boot clocks (jetson_clocks, then Stop -> Start with the aircraft untouched)'
                     % (lag_p50, lag_max, ODOM_STAMP_LAG_MAX_S))
    else:
        rep.fail('FAST-LIO stamp lag', 'no /lidar_slam/odom in %.0f s' % RATE_WINDOW_S)
    # FAST-LIO gate 2: who is reading it. A subscriber we do not know on the cloud or the raw
    # scans is a laptop over wifi, and that is what stalled it.
    for topic, allowed in FASTLIO_ALLOWED_SUBS.items():
        subs = sorted({s.node_name for s in node.get_subscriptions_info_by_topic(topic)}
                      - set(allowed))
        if subs:
            rep.fail('foreign subscriber on %s' % topic,
                     '%s -- a laptop RViz over wifi stalls FAST-LIO (38 s of stamp lag on '
                     '2026-09-13). Close it; never draw the cloud over wifi in flight'
                     % ', '.join(subs))
        else:
            rep.ok('foreign subscriber on %s' % topic, 'none (only %s)' % ', '.join(allowed))
    for topic in ('/dtp/sensors_ok', '/dtp/perception_ok'):
        m = last.get(topic)
        if m is None:
            rep.fail(topic, 'nothing received in %.0f s' % RATE_WINDOW_S)
        elif bool(m.data):
            rep.ok(topic, 'True')
        else:
            rep.fail(topic, 'FALSE -- a sensor is dead or perception is blind')
    m = last.get('/mux/state')
    if m is None:
        rep.fail('/mux/state', 'silent')
    else:
        state = m.data.split('|')[0]
        if state == 'ABORT':
            rep.fail('/mux/state', m.data[:120])
        else:
            rep.ok('/mux/state', m.data[:120])

    # -- 4b. the estimate itself: G0.2 --------------------------------------------------------
    #
    # *** Everything above passed on 2026-09-12 20:06 while FAST-LIO was 3.74 km wrong. ***
    # Rate 10 Hz, one publisher, perception ok -- and the aircraft was armed on a pose at
    # (-3340, -1672, -86.7) m that EKF2 called valid with eph 0.06 m. Health flags cannot see a
    # diverged-then-settled estimate; only WHERE it is can. These are the same checks as
    # scripts/px4/estimator_preflight.py (one copy of the logic, imported from the repo root
    # this tree lives in), and they FAIL CLOSED if that file cannot be found.
    est_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__)))), 'scripts', 'px4', 'estimator_preflight.py')
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location('estimator_preflight', est_path)
        est = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(est)
    except (OSError, ImportError, AttributeError) as exc:
        est = None
        rep.fail('G0.2 estimator checks', 'cannot load %s (%s) -- not flying without them'
                 % (est_path, exc))
    if est is not None:
        for state, what, detail in est.run_checks(node, spin, args.estimator_window,
                                                  require_bridge=True):
            {'PASS': rep.ok, 'WARN': rep.warn, 'FAIL': rep.fail}[state](what, detail)

    # -- 5. the PX4 side is present (Kevin's stack) -----------------------------------------------
    for topic, who in (('/fmu/out/vehicle_status_v1', 'uXRCE-DDS agent'),
                       ('/fmu/in/vehicle_visual_odometry', 'px4_odom_bridge (EKF position source)'),
                       ('/fmu/in/trajectory_setpoint', 'px4_offboard_bridge'),
                       ('/planning/pos_cmd', 'SUPER fsm_node')):
        await_graph(lambda tp=topic: len(node.get_publishers_info_by_topic(tp)) >= 1)
        pubs = node.get_publishers_info_by_topic(topic)
        if pubs:
            rep.ok('nav stack: %s' % topic, '%s publishing' % who)
        else:
            rep.warn('nav stack: %s' % topic, 'no publisher -- %s not up yet' % who)

    node.destroy_node()
    rclpy.shutdown()

    # -- 6. host: clocks, disk, recording ---------------------------------------------------
    try:
        cur = int(open(os.path.join(GPU_DEVFREQ, 'cur_freq')).read())
        mx = int(open(os.path.join(GPU_DEVFREQ, 'max_freq')).read())
        if cur >= mx:
            rep.ok('jetson_clocks', 'GPU pinned at %d MHz' % (mx // 1000000))
        else:
            rep.fail('jetson_clocks', 'GPU at %d of %d MHz -- run `sudo jetson_clocks` (does not '
                     'survive reboot)' % (cur // 1000000, mx // 1000000))
    except OSError:
        rep.warn('jetson_clocks', 'could not read %s' % GPU_DEVFREQ)
    try:
        st = os.statvfs(args.datasets)
        free_gb = st.f_bavail * st.f_frsize / 1e9
        if free_gb >= MIN_FREE_GB:
            rep.ok('disk for recording', '%.1f GB free under %s' % (free_gb, args.datasets))
        else:
            rep.fail('disk for recording', 'only %.1f GB free under %s' % (free_gb, args.datasets))
    except OSError:
        rep.fail('disk for recording', '%s does not exist' % args.datasets)
    ps = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True).stdout
    recording = [line for line in ps.splitlines()
                 if 'ros2 bag record' in line and 'stage_a_preflight' not in line]
    if recording:
        rep.ok('bag recording', recording[0][:110])
    elif args.require_record:
        rep.fail('bag recording', 'no `ros2 bag record` running -- scripts/stage_a_record.sh')
    else:
        rep.warn('bag recording',
                 'not running yet (start scripts/stage_a_record.sh before takeoff)')

    print()
    if args.stage_b:
        print('STAGE B/C PRE-ARM CHECK -- follow stack HAS AUTHORITY -- target: %s%s'
              % (args.target, '  [D21 WAIVED]' if args.waive_d21 else ''))
    else:
        print('STAGE A PRE-ARM CHECK')
    rep.dump()
    print()
    if rep.fails:
        print('NO-GO: %d failing check(s). Do not arm.' % rep.fails)
        return 2
    if rep.warns:
        print('GO with %d warning(s) -- read them.' % rep.warns)
    else:
        print('GO.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
