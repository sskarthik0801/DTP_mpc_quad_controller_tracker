"""Launch-time gates for the DTP follow stack.

WHY THIS IS A MODULE AND NOT A FEW LINES INSIDE A LAUNCH FILE
-------------------------------------------------------------
Every check in here exists because the misconfiguration it catches is SILENT. Each one produces
a stack that starts cleanly, prints nothing unusual, publishes on every topic you would think
to check, and simply does not follow anybody. Two of the three have already cost real bench
sessions on this project (see the dated entries in CLAUDE.md). A launch file is the last place
where the whole configuration is visible at once, so it is the right place to refuse.

Keeping them here rather than inline also makes them testable without a ROS graph and lets
`follower.launch.py` and `super.launch.py` share the config resolution instead of each growing
its own slightly-different copy.
"""
import os

import yaml

# Where SUPER's fsm_node actually reads its config from, and why we cannot simply ask it.
#
# fsm_node_ros2.cpp:81-87 resolves `config_name` against the COMPILE-TIME macro ROOT_DIR:
#     #define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/" + (name)))
# and super_planner/ros/ros2.CMakeLists.txt:15 sets ROOT_DIR to CMAKE_CURRENT_SOURCE_DIR. So
# fsm_node reads the SOURCE tree copy, not the installed one -- editing the YAML takes effect
# without a rebuild, and `share/super_planner/config/` can be arbitrarily stale relative to it
# (this repo's builds have never used --symlink-install; see CLAUDE.md's note that colcon
# copies config into install/share/). ROOT_DIR is not recoverable at runtime, so we search.
#
# Order is deliberate: most-authoritative first. The share copy is LAST and reported as
# possibly-stale, because passing this check against a stale copy is worse than not checking.
SUPER_CONFIG_ENV = 'DTP_SUPER_CONFIG_DIR'

# Fallbacks used only when the SUPER config cannot be located at all. These are the values in
# super_planner/config/follow_test_ros2.yaml:7-9, the M0.7 Step 0.7.4 fixture that passed on
# hardware 2026-08-14 -- not invented defaults.
DEFAULT_FOLLOW_GOAL_TOPIC = '/planning/follow_goal'
DEFAULT_GOAL_STATUS_TOPIC = '/planning/goal_status'
DEFAULT_YAW_TARGET_TOPIC = '/planning/yaw_target'

# super_planner/yaw_mode 3 == YAW_TO_TARGET (1 = YAW_TO_VEL, 2 = YAW_TO_GOAL). Anything else
# and the drone flies a correct standoff while never turning to face the person.
YAW_MODE_TO_TARGET = 3

# *** THIS GATE EXISTS BECAUSE OUR OWN TOOLING USED TO RECOMMEND A 50x SPEED INCREASE. ***
#
# Until 2026-09-12, assert_follow_path_enabled below told the operator to fix a missing follow
# path by switching to config_name:=follow_test_ros2.yaml. That file carries SUPER benchmark
# tuning -- max_vel 5.0, max_acc 5.0, max_omg 2.5, yaw_dot_max 3.0 -- while the config the
# aircraft actually flies (click_smooth_ros2.yaml, hardcoded for fsm_node in super_bringup
# m1_super_lidar.launch.py) carries the gate-signed first-flight limits max_vel 0.1,
# max_acc 0.3, max_omg 1.5, yaw_dot_max 0.5, re-applied by super_bringup/scripts/setup.sh on
# every install. Taking our advice therefore raised max_vel 50x and yaw rate 6x, and NOTHING
# downstream clamps either -- the offboard bridge forwards whatever SUPER plans. The advice was
# also unnecessary: follow_goal_en was added to the DEPLOYED click_smooth_ros2.yaml on
# 2026-09-10, so the config with the safe limits already has the follow path.
#
# The lesson is that an envelope recorded in a comment is not an envelope. So it is a gate.
# Note where it is called from: OUTSIDE super.launch.py require_follow_path try/except. That
# flag means "this bring-up is not following anybody", which is no reason at all to stop
# checking how fast it may fly.
#
# 1.5 m/s is not invented here -- it is this project own written pre-Stage-B ceiling
# (plan/dtp-imp/21 TODO "lower max_vel 5.0 to <=1.5 before Stage B", risk R-F07). Raising it
# is a deliberate and logged act: super.launch.py max_vel_ceiling:=<N>.
MAX_VEL_CEILING_DEFAULT = 1.5


class PreflightError(RuntimeError):
    """Raised to abort a launch whose configuration cannot produce a working follow chain."""


def assert_wall_clock(use_sim_time):
    """Refuse to launch the DTP stack on simulated time.

    This is not a style preference. fsm_node COERCES itself to wall clock: fsm_node_ros2.cpp
    :56-77 spins a loop that reads `use_sim_time`, sets it back to false if it is true, sleeps
    a second, and refuses to initialise the FSM until it reads false. So fsm_node is always on
    wall clock no matter what you pass it.

    Meanwhile SUPER gates the yaw target on staleness:
        (ros_ptr_->getSimTime() - rclcpp::Time(msg->header.stamp).seconds()) > kYawTargetTimeoutSec
    with kYawTargetTimeoutSec = 0.5 s, a compile-time constant (super_planner.h:102). If
    follow_planner stamps on sim time while fsm_node compares on wall clock, that difference is
    an epoch gap, not a latency -- the gate is exceeded on every single message, forever. Yaw
    silently falls back to velocity-aligned and the drone never looks at the person. Nothing is
    logged. That is exactly the R-F10 failure YAW_TO_TARGET exists to prevent.

    So the only self-consistent setting for this stack is false everywhere, and a `true` here
    is always a mistake worth stopping for rather than a mode worth supporting.
    """
    if as_bool(use_sim_time):
        raise PreflightError(
            'use_sim_time:=true is not supported by the DTP follow stack. fsm_node forces '
            'itself back to wall clock (fsm_node_ros2.cpp:56-77), so a sim-time follow_planner '
            'would fail SUPER\'s 0.5 s yaw-target staleness gate on every message and yaw would '
            'silently revert to velocity-aligned. Relaunch with use_sim_time:=false.')


def resolve_super_config(config_name, explicit_path=''):
    """Locate the YAML that fsm_node will actually read.

    Returns (path_or_None, searched, stale_risk) where `searched` lists every location tried
    (so a failure message can say where it looked) and `stale_risk` is True when the file we
    found is the installed copy rather than the source copy fsm_node reads.
    """
    searched = []

    if explicit_path:
        searched.append(explicit_path)
        if os.path.isfile(explicit_path):
            return explicit_path, searched, False

    env_dir = os.environ.get(SUPER_CONFIG_ENV, '')
    if env_dir:
        cand = os.path.join(env_dir, config_name)
        searched.append(cand)
        if os.path.isfile(cand):
            return cand, searched, False

    # Installed copy. Found last and flagged, per the module comment above.
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('super_planner'), 'config', config_name)
        searched.append(cand)
        if os.path.isfile(cand):
            return cand, searched, True
    except Exception:  # noqa: BLE001 - package simply may not be on the overlay yet
        searched.append('<share/super_planner not on the ament index>')

    return None, searched, False


def load_super_config(path):
    with open(path, 'r') as handle:
        return yaml.safe_load(handle) or {}


def assert_livox_typesupport():
    """Fail with a sentence instead of a SIGSEGV when the Livox overlay is not sourced.

    MEASURED 2026-09-09, and it cost most of an evening. fast_lio subscribes the Livox-native
    CustomMsg whenever `preprocess.lidar_type == AVIA`, which is what our shipped mid360.yaml
    sets (lidar_type: 1 -- AVIA is the Livox FAMILY, not a model, despite the file being named
    for the MID360). Creating that subscription needs
    liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so, which lives in ~/ws_livox/install
    and is NOT on the library path of a shell that sourced only /opt/ros and super_ws.

    What that looks like when it goes wrong is the problem. Through one launch path it is
    `process has died ... exit code -11` -- a bare SIGSEGV. Through another it is
    `invalid allocator, at ./src/rcl/subscription.c:219`. Neither names the missing library, and
    neither suggests an overlay. The real message is buried several lines earlier in a
    `Could not load library` line that scrolls past.

    This is the same failure family as the libcudss trap in bench_yolo.py: a non-interactive
    shell missing an overlay that an interactive one would have had. Catch it here, where we can
    say what to do about it.
    """
    import ctypes
    lib = 'liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so'
    try:
        ctypes.CDLL(lib)
    except OSError:
        raise PreflightError(
            '%s is not loadable, so fast_lio will die creating its CustomMsg subscription -- '
            'as a SIGSEGV or as "invalid allocator", neither of which names this library. '
            'Source the Livox overlay before launching: source ~/ws_livox/install/setup.bash'
            % lib)


def running_pids(basename):
    """pids of live processes whose EXECUTABLE (argv[0]) has exactly this basename.

    Never a flattened command line: `pgrep -f fastlio_mapping` matches the shell that ran it
    (CLAUDE.md), and a `bash -c` that merely MENTIONS a node would count as running it. Not any
    argv token either: `ros2 run fast_lio fastlio_mapping` is a python3 wrapper whose argv carries
    the name, and it would count as a second copy of the node it started.
    """
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open('/proc/%s/cmdline' % entry, 'rb') as handle:
                argv = [a.decode(errors='replace') for a in handle.read().split(b'\0') if a]
            with open('/proc/%s/stat' % entry) as handle:
                stat = handle.read()
        except OSError:
            continue
        if stat[stat.rfind(')') + 2:].startswith('Z'):
            continue
        if argv and os.path.basename(argv[0]) == basename:
            pids.append(int(entry))
    return pids


def duplicate_refusal(basename, what):
    """A refusal sentence if `basename` is already running, else None.

    *** A SECOND LIVOX DRIVER IS HOW THE AIRCRAFT GOT ARMED ON A 3.74 km ERROR (2026-09-12). ***
    dtp_system.launch.py was started on top of run.sh's stack. Its driver took the MID360's UDP
    stream and published it as PointCloud2, a type FAST-LIO's CustomMsg subscription cannot
    receive -- while its /livox/imu, the SAME type, kept FAST-LIO's IMU flowing. FAST-LIO got
    IMU and no scans for 225 s, integrated IMU alone, and came back 3.74 km off with millimetre
    covariance. Reproduced on the bench the same night (+2.1 km in 120 s on a stationary rig).
    docs/fact-fastlio-divergence-2026-09-12.md in the repo root has the evidence.

    So a launch that would start a second copy of a sensor-owning node does not start it.
    """
    pids = running_pids(basename)
    if not pids:
        return None
    return ('[dtp_bringup] NOT starting %s: %s is already running (pid %s). Two copies '
            'fight over one sensor, estimate or planner -- on 2026-09-12 a second Livox driver '
            'starved FAST-LIO of scans for 225 s and it diverged 3.74 km. Using the running one. '
            'If that is not what you want, stop it first, then relaunch.'
            % (what, basename, ', '.join(str(p) for p in pids)))


def assert_follow_path_enabled(cfg, path):
    """The check that replaces plan/dtp-imp/07's obsolete `click_height <= -5` assertion (R-F17).

    R-F17 was written before setFollowGoal existed, when the plan was to reuse SUPER's click
    path and force click_height to -100 so goal Z would pass through. setFollowGoal deliberately
    omits the click_height override (fsm.cpp:226-227), so click_height is now irrelevant to the
    follow goal and forcing it negative would only break RViz clicking. The config error that
    actually breaks following today, and breaks it invisibly, is this one:

    fsm/follow_goal_en defaults to FALSE (fsm/config.hpp:73). With it false, fsm_node creates
    neither the /planning/follow_goal subscription nor the /planning/goal_status publisher. The
    arbitrated goal then lands on a topic nobody is listening to, follow_planner's GoalStatus
    subscriber never fires, and the entire DROPPED/SNAPPED handling contract is unreachable --
    with no error on either side, because a ROS 2 publisher with no subscriber is normal.
    """
    fsm = cfg.get('fsm') or {}
    if not as_bool(fsm.get('follow_goal_en', False)):
        raise PreflightError(
            'fsm/follow_goal_en is not true in %s. Without it fsm_node never creates the '
            '/planning/follow_goal subscription or the /planning/goal_status publisher, so the '
            'follow chain is silently open-loop: goals go nowhere and no status ever comes '
            'back. Set fsm/follow_goal_en: true in THIS file. Do NOT switch to '
            'follow_test_ros2.yaml to get it: that file also carries the 5.0 m/s benchmark '
            'tune (see MAX_VEL_CEILING_DEFAULT above), and the deployed click_smooth_ros2.yaml '
            'has had follow_goal_en since 2026-09-10 anyway.'
            % path)


# Distinguishes "key absent" from "key present but junk" in assert_speed_envelope. A bare None
# cannot, because _as_float maps both to None on purpose.
_MISSING = object()


def read_speed_envelope(cfg):
    """The four config values that decide how fast SUPER may fly, with their YAML paths.

    Returns (label, value) pairs, value None when the key is absent. Absent is NOT the same as
    safe: fsm/config.hpp supplies its own compiled default, which is not readable from here, so
    an absent max_vel is reported as unknown and gated, never passed.
    """
    boundary = (cfg.get('traj_opt') or {}).get('boundary') or {}
    planner = cfg.get('super_planner') or {}
    return [
        ('traj_opt/boundary/max_vel', _as_float(boundary.get('max_vel'))),
        ('traj_opt/boundary/max_acc', _as_float(boundary.get('max_acc'))),
        ('traj_opt/boundary/max_omg', _as_float(boundary.get('max_omg'))),
        ('super_planner/yaw_dot_max', _as_float(planner.get('yaw_dot_max'))),
    ]


def format_speed_envelope(cfg):
    """One line naming every limit, so the whole envelope is visible before anyone arms."""
    return ', '.join(
        '%s=%s' % (label.rsplit('/', 1)[-1], 'ABSENT' if v is None else '%g' % v)
        for label, v in read_speed_envelope(cfg))


def assert_speed_envelope(cfg, path, ceiling=MAX_VEL_CEILING_DEFAULT):
    """Refuse to launch a planner configured to fly faster than this project has cleared.

    Gates max_vel and REPORTS the other three. Not laziness: max_vel is the only one of the four
    this project has a written ceiling for, and inventing three more numbers here would lend
    them the same authority as the documented one. Gating max_vel is enough to stop the
    configuration that matters -- the benchmark tune fails on max_vel first -- and the refusal
    prints all four, so the acc/omg/yaw excursions are visible in the same message.
    """
    envelope = read_speed_envelope(cfg)
    max_vel = dict(envelope)['traj_opt/boundary/max_vel']
    shown = format_speed_envelope(cfg)
    if max_vel is None:
        # Absent and unreadable both land here, but they need DIFFERENT instructions: telling
        # an operator to "set the key" when the key is set and malformed sends them looking
        # for the wrong thing under time pressure. So say which, and quote it back.
        raw = ((cfg.get('traj_opt') or {}).get('boundary') or {}).get('max_vel', _MISSING)
        if raw is _MISSING:
            how = 'the key is absent; add it'
        else:
            how = 'the key is present but unreadable as a number (%r); correct it' % (raw,)
        raise PreflightError(
            'traj_opt/boundary/max_vel cannot be read from %s, so the speed SUPER may plan is '
            'unverified before flight (fsm/config.hpp would supply its own compiled default, '
            'which is not readable from here): %s. Envelope read: %s. Or pass '
            'max_vel_ceiling:=<N> to accept the unknown deliberately.' % (path, how, shown))
    if max_vel > ceiling:
        raise PreflightError(
            'traj_opt/boundary/max_vel is %g m/s in %s, above this stack ceiling of %g m/s. '
            'Nothing downstream clamps it -- the offboard bridge forwards whatever SUPER plans. '
            'Full envelope: %s. Either fly the gate-signed config (the DEPLOYED '
            'click_smooth_ros2.yaml: max_vel 0.1, and it has had fsm/follow_goal_en since '
            '2026-09-10), or lower max_vel in this file, or raise the ceiling on purpose with '
            'max_vel_ceiling:=%g.' % (max_vel, path, ceiling, shown, max_vel))


def collect_follow_warnings(cfg):
    """Non-fatal config observations worth printing where the operator will see them.

    Deliberately warnings and not errors: each one describes a stack that still flies, just
    not the way the follow design intends. Refusing to launch over them would make bench work
    (NAV-only, click-goal-only) needlessly painful.
    """
    warnings = []
    sp = cfg.get('super_planner') or {}
    yaw_mode = sp.get('yaw_mode')
    if yaw_mode is not None and int(yaw_mode) != YAW_MODE_TO_TARGET:
        warnings.append(
            'super_planner/yaw_mode is %s, not %d (YAW_TO_TARGET). The drone will fly the '
            'standoff correctly and never turn to face the person (risk R-F10).'
            % (yaw_mode, YAW_MODE_TO_TARGET))

    fsm = cfg.get('fsm') or {}
    if not as_bool(fsm.get('click_goal_en', True)):
        warnings.append(
            'fsm/click_goal_en is false: RViz "2D Goal Pose" clicking on /goal_pose is off. '
            'That is fine in flight but removes the usual bench fallback.')
    return warnings


def follow_topics_from(cfg):
    """The three topic names fsm_node will actually bind, with the M0.7 fixture as fallback.

    Reading them from SUPER's own config rather than hardcoding them is the fix for the two
    blocking wiring breaks recorded in the interface contract: follow_planner publishes
    /dtp/target_point while fsm_node listens on fsm/yaw_target_topic, and command_mux publishes
    /goal_pose (the click path, which overwrites Z and answers nothing) while the only ingest
    path that preserves Z and emits GoalStatus is fsm/follow_goal_topic. Both are one-line
    remaps in the launch -- but only if the launch knows the names, and the config is the one
    place they are stated.
    """
    fsm = cfg.get('fsm') or {}
    return (
        str(fsm.get('follow_goal_topic', DEFAULT_FOLLOW_GOAL_TOPIC)),
        str(fsm.get('goal_status_topic', DEFAULT_GOAL_STATUS_TOPIC)),
        str(fsm.get('yaw_target_topic', DEFAULT_YAW_TARGET_TOPIC)),
    )


def _as_float(value):
    """YAML numbers, launch-substitution strings and absent keys, reduced to float or None.

    Returns None rather than raising on junk. A gate that crashes on a malformed limit tells
    the operator less than one that reports it could not read the limit and refuses for that
    stated reason.
    """
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def as_bool(value):
    """Launch substitutions arrive as strings; YAML gives real bools. Accept both."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')
