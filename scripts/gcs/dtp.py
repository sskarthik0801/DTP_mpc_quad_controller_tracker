#!/usr/bin/env python3
"""The DTP follower row: what has to be true before it starts, and which SUPER
config the stack underneath it is really running.

The follower is the one row on this dashboard that can COMMAND the aircraft.  In
live mode command_mux publishes SUPER's follow goal and holds a VehicleCommand
publisher armed to send NAV_LAND; in observe mode everything goes to
/dtp/shadow/* and the abort is off (drone-dtp/.../follower.launch.py).  So Start
is refused, with a sentence, unless all of this holds:

  both modes  not already running (here or in a terminal); drone-dtp installed;
              the SUPER stack up (the follower rides on it and never starts its
              own Livox driver -- a second one is the 2026-09-12 divergence);
              exactly one fsm_node; the Gemini free (a second opener dies with
              uvc_open -6); and the FC not reporting ARMED (even on a link that
              has since gone stale) or IN AIR.  That last one holds for
              observe mode too: perception's first TensorRT
              inference stalls ~2.5 s, and a whole-Jetson stall is what put
              /lidar_slam/odom 3.9 s late on 2026-09-13.
  live only   a fresh FC link that says disarmed and landed (no link is not
              good enough); fsm_node on follow_test_ros2.yaml, the M5.2
              envelope; and /lidar_slam/odom at >= 9.5 Hz.

Stop is never refused here.  Stopping is the direction an operator reaches for
when something looks wrong, and the page already says what a stop does to an
armed aircraft before it sends one.

Which config fsm_node loaded, and which mode a running follower is in, are both
read off the processes' own launch parameters -- the `--params-file` YAML a
`ros2 launch` writes for every node -- rather than remembered from what this
dashboard asked for.  That makes them true for a stack or follower started from
a terminal too, and it is the same file the node itself read.  (The shell
wrapper asks /fsm_node over ROS as a second, independent check before a live
start: see scripts/dtp_follower.sh.)
"""
import os
import time

import yaml

import sysinfo

CLICK_CONFIG = 'click_smooth_ros2.yaml'
FOLLOW_CONFIG = 'follow_test_ros2.yaml'
# The only two configs the Stack row may start.  A whitelist, not a filename
# from the request: SUPER_CONFIG ends up in a path inside run.sh.
STACK_CONFIGS = {'click': CLICK_CONFIG, 'follow': FOLLOW_CONFIG}
MODES = ('observe', 'live')
SHADOW_NS = '/dtp/shadow/'

STACK_NODES = ('livox_ros_driver2_node', 'fastlio_mapping', 'fsm_node')
# M5.1 runbook section 2: "odom >= 9.5 Hz" before the follower comes up.
ODOM_MIN_HZ = 9.5
ODOM_TOPIC = '/lidar_slam/odom'

SUPER_WS = os.path.expanduser(os.environ.get('SUPER_WS', '~/super_ws'))
DTP_ENV = os.path.join(SUPER_WS, 'src', 'drone-dtp', 'ros2_ws', 'dtp_env_jetson.sh')
DTP_INSTALL = os.path.join(SUPER_WS, 'install', 'dtp_bringup')

CACHE_TTL = 2.0
_cache = {}


def _cached(key, fn):
    now = time.monotonic()
    hit = _cache.get(key)
    if hit is None or now - hit[0] > CACHE_TTL:
        hit = (now, fn())
        _cache[key] = hit
    return hit[1]


# -- launch parameters -------------------------------------------------------

def _node_name(tokens):
    for t in tokens:
        if t.startswith('__node:='):
            return t.split(':=', 1)[1]
    return None


def launch_param(tokens, name):
    """A parameter's value as the node's own argv handed it to the node.

    `tokens` is the node's argv.  `ros2 launch` passes parameters as one or
    more `--params-file <yaml>` (later files win) and occasionally `-p
    name:=value`.  A shared params file is keyed by node name -- dtp_params.yaml
    has a block per node -- so only `/**` and THIS node's own block count, or a
    same-named parameter of another node would be read as this one's.

    Returns (found, value).  A file that cannot be read (another user's 0600
    temp file, or launch already gone) is skipped, not an error: the caller
    decides what "not found" means.
    """
    node = _node_name(tokens)
    keys = {'/**', '**'} | ({node, '/' + node} if node else set())
    found, value = False, None
    for i, t in enumerate(tokens):
        nxt = tokens[i + 1] if i + 1 < len(tokens) else None
        if t == '--params-file' and nxt:
            try:
                with open(nxt) as f:
                    doc = yaml.safe_load(f) or {}
            except (OSError, yaml.YAMLError):
                continue
            if not isinstance(doc, dict):
                continue
            for key, block in doc.items():
                if key not in keys or not isinstance(block, dict):
                    continue
                params = block.get('ros__parameters') or {}
                if isinstance(params, dict) and name in params:
                    found, value = True, params[name]
        elif t == '-p' and nxt and nxt.startswith(name + ':='):
            found, value = True, nxt.split(':=', 1)[1]
    return found, value


def _procs(pattern):
    hits = sysinfo.find_by_cmdline([pattern])
    return sorted(hits.get(pattern, []))


def fsm_config():
    """Which SUPER config the running fsm_node loaded.

    {'name': str|None, 'pids': [...], 'error': str|None}.  `name` is None when
    no fsm_node runs, and also when one runs but its config cannot be read --
    `error` tells those apart, because "not running" and "cannot tell" call for
    different sentences on the page.
    """
    return _cached('fsm', _fsm_config)


def _fsm_config():
    pids = _procs('fsm_node')
    out = {'name': None, 'pids': pids, 'error': None}
    if not pids:
        return out
    if len(pids) > 1:
        out['error'] = (f'{len(pids)} fsm_node processes are running '
                        f'(pids {", ".join(map(str, pids))}) -- there must be one')
    names = set()
    for pid in pids:
        found, value = launch_param(sysinfo.argv(pid), 'config_name')
        if found and isinstance(value, str) and value:
            names.add(os.path.basename(value))
    if len(names) == 1:
        out['name'] = names.pop()
    elif not names:
        out['error'] = out['error'] or (
            f'cannot read config_name from fsm_node\'s launch parameters '
            f'(pid {pids[0]})')
    else:
        out['error'] = out['error'] or f'fsm_node processes disagree: {sorted(names)}'
    return out


def follower_mode():
    """'observe', 'live', 'mixed' or None, from what the running nodes were given.

    command_mux's arbitrated_goal_topic and follow_planner's yaw_target_topic
    are the two outputs that reach fsm_node; observe mode moves BOTH under
    /dtp/shadow/.  Reading them off the processes rather than remembering the
    mode we asked for is what makes this true for a follower started in a
    terminal -- and a follower half-redirected reports 'mixed' instead of
    whichever half was checked first.
    """
    return _cached('mode', _follower_mode)


def _follower_mode():
    seen = []
    for pattern, param in (('command_mux_node', 'arbitrated_goal_topic'),
                           ('follow_planner_node', 'yaw_target_topic')):
        for pid in _procs(pattern):
            found, value = launch_param(sysinfo.argv(pid), param)
            if found and isinstance(value, str):
                seen.append(value.startswith(SHADOW_NS))
    if not seen:
        return None
    if all(seen):
        return 'observe'
    if not any(seen):
        return 'live'
    return 'mixed'


def installed():
    """Why drone-dtp cannot be started from here, or None.

    Checking out a branch without drone-dtp/ deletes the folder the workspace
    symlink points at (CLAUDE.md, drone-dtp trap 1), and nothing else notices.
    """
    if not os.path.isfile(DTP_ENV):
        return (f'drone-dtp is not installed here: {DTP_ENV} is missing. Is '
                f'{SUPER_WS}/src/drone-dtp a symlink to this repo\'s drone-dtp/, '
                'on a branch that has it?')
    if not os.path.isdir(DTP_INSTALL):
        return (f'dtp_bringup is not built ({DTP_INSTALL} is missing) -- see '
                'drone-dtp/README.md')
    return None


def stack_missing():
    """The stack nodes that are NOT running, by any owner."""
    hits = sysinfo.find_by_cmdline(list(STACK_NODES))
    return [n for n in STACK_NODES if not hits.get(n)]


# -- the interlock -----------------------------------------------------------

def _camera_owner(camera_row, cam):
    if camera_row and camera_row['state'] != 'stopped':
        return ('the dashboard\'s own camera (the Camera panel) has the Gemini -- '
                'stop it first; the follower opens the camera itself, and a '
                'second opener dies with uvc_open -6')
    if cam is None:
        return None
    if not cam.get('devices'):
        return ('no Orbbec camera is on USB -- perception has nothing to look '
                'at. Check the cable, then `lsusb | grep 2bc5`')
    # /dtp/* is our own overlay's output, not a camera driver: it cannot hold
    # the USB device, so it must not be read as somebody owning it.
    src = [s['topic'] for s in cam.get('sources') or []
           if not s['topic'].startswith('/dtp/')]
    if src:
        return (f'another process is already publishing {src[0]} -- the Gemini '
                'can only be opened once, so the follower\'s camera driver would '
                'die with uvc_open -6. Stop that launch first')
    holders = cam.get('holders') or []
    if holders:
        h = holders[0]
        return (f'the camera\'s USB device is held by pid {h["pid"]} '
                f'({(h.get("cmd") or "").split(" ")[0] or "unknown"})')
    return None


def start_blocked(mode, *, row, aircraft, ros_ok, fsm, missing, camera_row,
                  cam, odom_hz, install_error=None):
    """Why the follower must not be started in `mode` right now, or None.

    Pure: every fact comes in as an argument, so test_dtp.py can walk each
    refusal without ROS, a camera or an aircraft.

      row        this row's supervisor status
      aircraft   telemetry's aircraft block, or None with no ROS
      ros_ok     whether this dashboard has ROS telemetry at all
      fsm        fsm_config()
      missing    stack_missing()
      camera_row the Camera panel's supervisor status (None while locked)
      cam        server.camera_state(), or None
      odom_hz    measured /lidar_slam/odom rate, or None if not measured
    """
    if mode not in MODES:
        return f'unknown mode {mode!r} -- it is one of {", ".join(MODES)}'
    if row and row['state'] == 'running':
        return 'the DTP follower is already running'
    if row and row['state'] == 'external':
        return (f'a DTP follower is already running outside this dashboard '
                f'(pid {row.get("pid")}) -- stop it first')
    if install_error:
        return install_error

    a = aircraft or {}
    fresh = bool(a.get('fresh'))
    # Same asymmetry as hostpower.flight_blocked(): a never-seen FC does not
    # block observe mode (the bench), but a link that said ARMED and then went
    # quiet does -- that is the one shape that could be an aircraft in the air.
    if a.get('armed'):
        return (('the vehicle is ARMED' if fresh else
                 'the last thing the FC said was ARMED and the link has since '
                 'gone stale')
                + ' -- start the follower on the ground, disarmed. Its first '
                'TensorRT inference stalls the Jetson for seconds, and so does '
                'opening the camera')
    if fresh and a.get('landed') is False:
        return 'the FC reports the vehicle is IN AIR'
    if mode == 'live':
        if not ros_ok:
            return ('this dashboard has no ROS 2 telemetry, so it cannot verify '
                    'the vehicle is disarmed -- a live follower is not started '
                    'blind')
        if not fresh:
            return ('no fresh /fmu/out/vehicle_status_v1 -- a live follower '
                    'needs a working FC link that says disarmed, not silence')

    if missing:
        return ('the SUPER stack is not running (no '
                + ', '.join(missing) + ') -- start it first. The follower '
                'rides on its Livox driver, FAST-LIO and fsm_node, and never '
                'starts its own')
    if fsm and len(fsm.get('pids') or []) > 1:
        return fsm['error']

    if mode == 'live':
        name = (fsm or {}).get('name')
        if name is None:
            return ((fsm or {}).get('error') or 'cannot tell which config '
                    'fsm_node loaded') + (' -- a live follower is not started '
                                          'without knowing it')
        if name != FOLLOW_CONFIG:
            return (f'fsm_node is running {name}. A live follower needs '
                    f'{FOLLOW_CONFIG} (the M5.2 first-flight envelope) -- stop '
                    'the stack and start it again on the follow config')
        if odom_hz is None or odom_hz < ODOM_MIN_HZ:
            return (f'{ODOM_TOPIC} is at '
                    + ('an unknown rate' if odom_hz is None else f'{odom_hz} Hz')
                    + f', below {ODOM_MIN_HZ} Hz -- the follower would be '
                    'placing people in a frame that is not updating')

    return _camera_owner(camera_row, cam)


FOLLOW_ACTIONS = ('on', 'off')


def follow_blocked(action, *, row, ros_ok, state):
    """Why FOLLOW must not be engaged (or disengaged) from the page right now, or None.

    Pure, like start_blocked(). This is the one button on the page that DESIGNATES a
    target: on a LIVE follower with the aircraft armed in Offboard, pressing it makes
    the aircraft move toward a person. That is exactly what it is for -- a lock lost
    in flight needs a re-engage in seconds, not a shell (2026-09-14, Sortie 3: LOST one
    second after arming, never re-engaged, 44 s of Offboard flown as a hold). So being
    armed does NOT refuse here. What does:

      row      this row's supervisor status (running / external / stopped)
      ros_ok   whether this dashboard has ROS 2 telemetry at all
      state    telemetry's dtp block: lock, mux, available (the parameter service)
    """
    if action not in FOLLOW_ACTIONS:
        return f'unknown action {action!r} -- it is one of {", ".join(FOLLOW_ACTIONS)}'
    if not ros_ok:
        return ('this dashboard has no ROS 2 telemetry, so it cannot reach '
                'target_manager -- use field.sh arm / reengage in a shell')
    if not row or row.get('state') == 'stopped':
        return 'no DTP follower is running -- start one first'
    st = state or {}
    if not st.get('available'):
        return ('target_manager is not offering its parameter service yet -- the '
                'follower is still coming up, or its tracker has died')
    mux = st.get('mux') or ''
    if action == 'on' and mux.startswith('ABORT'):
        return ('command_mux has ABORTED (terminal until relaunch) -- stop and '
                'start the follower before engaging again')
    return None


def stop_note(row, aircraft):
    """What stopping the follower does right now, for the page's confirm.

    Never a refusal -- see the module docstring -- but a live follower on an
    armed aircraft is not the same button press as an observe-only one on the
    bench, and the question should say which of the two this is.
    """
    mode = (row or {}).get('mode')
    a = aircraft or {}
    if not a.get('fresh') or not a.get('armed'):
        return None
    if mode == 'observe':
        return ('The aircraft is ARMED. This follower is observe-only, so it '
                'commands nothing and the flight is unaffected; only the /dtp/* '
                'topics stop.')
    return ('The aircraft is ARMED' + (' and in OFFBOARD' if a.get('offboard') else '')
            + '. Stopping a live follower stops its goals: SUPER flies to the '
            'last one it was given and holds there, and command_mux\'s NAV_LAND '
            'abort stops with it. Be ready on channel 11.')
