#!/usr/bin/env python3
"""Remote control: nudge SUPER's goal from the dashboard.

The dashboard does NOT fly the aircraft here.  Every button on the RC card
publishes ONE goal message -- the same shape RViz's *2D Goal Pose* tool sends
-- and SUPER plans a collision-free trajectory to it, which `px4_offboard_bridge`
streams to PX4 only while the pilot has it armed and in Offboard.  So the
planner keeps obstacle avoidance in the loop, a dropped Wi-Fi packet means "no
new goal" rather than "no setpoints", and the abort stays where it belongs:

    ABORT IS THE RC.  Channel-11 Offboard switch OFF (you then get whatever
    channel 5 is on -- leave it on Position).  SUPER's FSM has exactly three
    inputs (click goal, follow goal, yaw target) and NONE of them is a stop, so
    there is deliberately no STOP button here to imply otherwise.

Relative moves are relative to WHERE THE AIRCRAFT IS NOW, not to the last goal
sent, so tapping forward three times while it is moving does not queue up three
metres.  Same rule as scripts/px4/goal_forward.py, and for the same reason.

TWO GOAL PATHS, AND THE CARD PICKS ONE
--------------------------------------
This SUPER has two goal inputs, and which one is live decides what the controls
can do.  `choose_path()` asks the ROS graph rather than trusting the config,
because what matters is the node that is actually running:

  * **click** -- `/goal_pose` -> `Fsm::setGoalPosiAndYaw()` (`fsm.cpp:186`).
    Upstream's path, the one RViz clicks.  It **overwrites the z** of every goal
    with `fsm/click_height` (`fsm.cpp:188`), and it **silently drops** any goal
    within 0.1 m of the aircraft (`fsm.cpp:201`).  Neither is reported anywhere.

  * **follow** -- `/planning/follow_goal` -> `Fsm::setFollowGoal()`
    (`fsm.cpp:226`), the DTP patch's moving-goal ingestion, enabled by
    `fsm/follow_goal_en`.  It **preserves z** (deliberately: "a follow goal's Z
    is meaningful ... and must survive"), has **no** 0.1 m drop, and **always
    answers** on `/planning/goal_status` with ACCEPTED / SNAPPED / DROPPED plus
    the point it adopted and the request echoed back.

The follow path is preferred wherever it is live.  It costs nothing to the RViz
click path -- `click_height` stays as it is, so a click still flies at 1.5 m --
and it turns three silent behaviours into reported ones.  On a clean install of
this repo (no DTP patch) the key is never read and the click path is all there
is, which is why this is detected and not assumed.

Both paths still share two behaviours worth knowing:

* **Goal yaw is honoured only when the planner is willing** -- `goal_yaw_en` and
  a trajectory that reaches the goal (`super_planner.cpp:814`, `connected_goal`),
  which short steps do.  Under `yaw_mode: 3` (YAW_TO_TARGET) an external target
  wins over it entirely.  The click path additionally gates it on `click_yaw_en`;
  the follow path reads the quaternion straight (`fsm_ros2.hpp:297`).
* **The goal is snapped** to the nearest non-inflated-occupied cell up to 3 m
  away, so what SUPER flies to is not necessarily what was sent.  On the follow
  path the status message says when that happened; on the click path it does not.

Altitude is clamped to ROG-Map's *effective* band, which is not the configured
one: it insets both bounds at init, so the shipped [-1.0, 3.5] is really
[-0.5, 2.9] (CLAUDE.md, measured).  A goal outside it is refused by the planner
with "Odom below virtual ground", which looks like a sensor fault and is not.
"""
import math
import os
import threading
import time

import yaml

# The config the GUI's own Stack row launches by default (run.sh's SUPER_CONFIG
# default).  Only the fallback: server.py passes the name the RUNNING fsm_node
# was launched with (dtp.fsm_config()), because the Stack row can also start
# follow_test_ros2.yaml, whose speed limits are not this file's.  install/
# first: CLAUDE.md's standing warning is that a colcon build silently reverts
# src/ and install/ independently.
CONFIG_NAME = 'click_smooth_ros2.yaml'
CONFIG_DIRS = (
    os.path.expanduser('~/super_ws/install/super_planner/share/super_planner/config'),
    os.path.expanduser('~/super_ws/src/SUPER/super_planner/config'),
)

GOAL_TOPIC_DEFAULT = '/goal_pose'
FOLLOW_GOAL_TOPIC = '/planning/follow_goal'
GOAL_STATUS_TOPIC = '/planning/goal_status'
GOAL_FRAME = 'lidar_init'        # cosmetic: the FSM ignores the header

LINEAR_STEPS = (0.25, 0.5, 1.0, 2.0, 3.0)
YAW_STEPS = (15, 45, 90)
DEFAULT_STEP = 1.0
DEFAULT_YAW_STEP = 45
STEP_MIN, STEP_MAX = 0.25, 3.0
# Hoisted out of plan_move() so the number the panel puts in an <input min/max>
# and the number that validates the request are the same one.
YAW_MIN, YAW_MAX = 1, 180
# Clear of the 0.1 m drop threshold in fsm.cpp:201 with room for the snap, but
# small enough that "rotate" is a rotation and not a flight.  Click path only:
# the follow path has no such threshold, so a rotation there is a pure rotation.
YAW_NUDGE_M = 0.15
CLICK_DROP_M = 0.1               # fsm.cpp:201, click path only
SNAP_RADIUS_M = 3.0              # getNearestInfCellNot(), both paths
ODOM_STALE_S = 1.0               # /lidar_slam/odom is 10 Hz

# How far one TYPED coordinate may send it.  The sliding ROG-Map is 50x50 m, so
# ~25 m is the physical limit; this is well inside it, and at the first-flight
# max_vel of 0.1 m/s it is already about 100 s of flight.  A typo should not be
# able to send the aircraft across a field.
GOTO_MAX_M = 10.0
# Not a gate, just the distance above which the panel asks first.
GOTO_CONFIRM_M = 5.0

# Arrival.  0.1 m is not a taste: it is SUPER's OWN test -- closeToGoal(0.1)
# (fsm.cpp:134), the distance at which the FSM declares the goal reached and
# returns to WAIT_GOAL.  Using the same number means "arrived" on this page and
# "arrived" in the planner are the same event, rather than two answers that
# disagree by centimetres.
ARRIVE_M = 0.1
# Flying and not getting closer.  At the first-flight max_vel of 0.1 m/s a 1 m
# step takes 10 s, and SUPER replans at 15 Hz, so a genuine advance shows up in
# a couple of seconds; 20 s of nothing is a stall, not a slow patch.
STALL_AFTER_S = 20.0
STALL_IMPROVE_M = 0.05     # smaller than this is noise, not progress

# ROG-Map insets both z bounds by more than the configured value at init; these
# are the measured insets that turn the shipped [-1.0, 3.5] into the effective
# [-0.5, 2.9] (CLAUDE.md, docs/fact-rogmap-bounds-and-memory.md).  Plus 0.1 m of
# margin, so a slider at its end stop is inside the band rather than on it.
GROUND_INSET, CEIL_INSET, EDGE_MARGIN = 0.5, 0.6, 0.1

AXES = ('forward', 'back', 'left', 'right', 'up', 'down',
        'yaw_left', 'yaw_right')


class RCError(Exception):
    """Refused, with a sentence the panel can show verbatim."""


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def to_px4_ned(p):
    """The same point as PX4/QGroundControl would report it.

    `px4_odom_bridge` declares POSE_FRAME_FRD and EKF2 adopts that frame
    wholesale, so EKF2's local frame IS lidar_init with y and z negated
    (docs/ImplementationDoc.md 8).  One helper, so the front end cannot invent
    its own sign convention for the read-only echo next to the coordinate box --
    which is exactly the kind of mistake that flies it to the wrong place.
    """
    if p is None:
        return None
    return {'x': round(float(p['x']), 3),
            'y': round(-float(p['y']), 3),
            'z': round(-float(p['z']), 3)}


def dist3(pose, x, y, z):
    return math.sqrt((x - pose['x']) ** 2 + (y - pose['y']) ** 2
                     + (z - pose['z']) ** 2)


_CFG = {'at': 0.0, 'path': None, 'mtime': None, 'data': None, 'name': None}
_CFG_LOCK = threading.Lock()
CONFIG_TTL = 5.0


def _find_config(name=CONFIG_NAME):
    for d in CONFIG_DIRS:
        p = os.path.join(d, name)
        if os.path.isfile(p):
            return p
    return None


def read_config(force=False, name=None):
    """What the planner will do with a goal, read off the file it loads.

    `name` is the config fsm_node was launched with, when known; None means
    the Stack row's default.  Only a bare file name is honoured -- it is joined
    onto the config directories, so a path in it would read somewhere else.

    Re-read when the file changes rather than once at startup: editing a config
    and restarting the stack is a two-minute loop here, and a panel that kept
    insisting the old value was in force would be worse than no panel.
    """
    name = name if name and os.path.basename(name) == name else CONFIG_NAME
    now = time.time()
    with _CFG_LOCK:
        fresh = (not force and _CFG['data'] is not None
                 and _CFG['name'] == name
                 and now - _CFG['at'] < CONFIG_TTL)
        if fresh:
            return _CFG['data']
    path = _find_config(name)
    data = {'path': path, 'name': name, 'error': None, 'click_height': None,
            'yaw_mode': None, 'click_yaw_en': None, 'goal_yaw_en': None,
            'goal_topic': GOAL_TOPIC_DEFAULT, 'max_vel': None,
            'yaw_dot_max': None, 'ground': None, 'ceil': None,
            'follow_goal_en': None, 'follow_goal_topic': FOLLOW_GOAL_TOPIC,
            'goal_status_topic': GOAL_STATUS_TOPIC}
    if path is None:
        data['error'] = (f'no {name} found -- is SUPER installed? '
                         f'looked in {", ".join(CONFIG_DIRS)}')
    else:
        try:
            with open(path) as f:
                y = yaml.safe_load(f) or {}
            fsm = y.get('fsm') or {}
            sp = y.get('super_planner') or {}
            rog = y.get('rog_map') or {}
            data.update(
                click_height=fsm.get('click_height'),
                click_yaw_en=fsm.get('click_yaw_en'),
                goal_topic=fsm.get('click_goal_topic') or GOAL_TOPIC_DEFAULT,
                # The DTP patch's second goal input. Absent from upstream and
                # from a clean setup.sh install, where LoadParam never reads it.
                follow_goal_en=fsm.get('follow_goal_en'),
                follow_goal_topic=(fsm.get('follow_goal_topic')
                                   or FOLLOW_GOAL_TOPIC),
                goal_status_topic=(fsm.get('goal_status_topic')
                                   or GOAL_STATUS_TOPIC),
                yaw_mode=sp.get('yaw_mode'),
                goal_yaw_en=sp.get('goal_yaw_en'),
                yaw_dot_max=sp.get('yaw_dot_max'),
                # traj_opt/boundary, not super_planner: the speed limit is
                # the optimiser's, and it is 0.1 m/s here on purpose (the
                # first-flight limits, commit 001f71f). A 1 m step is ~10 s.
                max_vel=((y.get('traj_opt') or {}).get('boundary') or {}
                         ).get('max_vel'),
                ground=rog.get('virtual_ground_height'),
                ceil=rog.get('virtual_ceil_height'))
        except Exception as exc:
            data['error'] = f'cannot read {path}: {type(exc).__name__}: {exc}'
    data['altitude'] = altitude_band(data)
    with _CFG_LOCK:
        _CFG.update(at=now, path=path, data=data, name=name)
    return data


def altitude_band(cfg):
    """The z range a goal can actually use, or None if the config is unreadable."""
    g, c = cfg.get('ground'), cfg.get('ceil')
    if g is None or c is None:
        return None
    lo = g + GROUND_INSET + EDGE_MARGIN
    hi = c - CEIL_INSET - EDGE_MARGIN
    return None if hi <= lo else {'min': round(lo, 2), 'max': round(hi, 2),
                                  'configured': [g, c]}


def choose_path(cfg, follow_subs=None, click_subs=None, follow_pubs_other=0):
    """Which of SUPER's two goal inputs this card will use, and what that costs.

    `follow_subs`/`click_subs` are subscriber counts from the ROS graph -- the
    question is which topic the RUNNING planner is listening on, which a config
    file cannot answer (someone may have launched a different one).  Pass None
    for both when there is no ROS at all: the config's own declaration is then
    the best available answer, and `why` says so rather than claiming it was
    observed.

    `follow_pubs_other` is how many publishers on the follow topic are NOT us.
    It is counted BY NODE NAME in telemetry.goal_graph(), deliberately, rather
    than as "total minus one": our own publisher being exactly one of the total
    is an assumption, and an off-by-one here either invents contention that is
    not there or -- much worse -- misses the case where the DTP follower and
    this page are both driving the same goal.
    """
    others = max(0, int(follow_pubs_other or 0))
    contended = None
    if others:
        contended = (
            f'{others} other publisher(s) on {cfg.get("follow_goal_topic")} -- '
            'the DTP follower or command_mux is driving the goal, and two '
            'sources interleaving goals is worse than either alone. Stop the '
            'follower, or use its own controls')
    observed = follow_subs is not None or click_subs is not None
    if observed:
        use_follow = bool(follow_subs)
    else:
        use_follow = bool(cfg.get('follow_goal_en'))

    ch = cfg.get('click_height')
    if use_follow:
        return {'name': 'follow', 'topic': cfg.get('follow_goal_topic'),
                'status_topic': cfg.get('goal_status_topic'),
                'keeps_z': True, 'reports': True, 'drop_radius': 0.0,
                'contended': contended, 'observed': observed,
                'why': ('follow goal -- z is kept and SUPER reports what it did '
                        'on ' + str(cfg.get('goal_status_topic'))
                        + ('' if observed else
                           ' (from the config; no planner seen yet)'))}
    keeps_z = ch is not None and ch <= -5
    return {'name': 'click', 'topic': cfg.get('goal_topic'),
            'status_topic': None,
            'keeps_z': keeps_z, 'reports': False, 'drop_radius': CLICK_DROP_M,
            'contended': None, 'observed': observed,
            'why': ('click goal -- ' + ('z is kept' if keeps_z else
                    f'z is forced to {ch} m by fsm/click_height')
                    + ', and a goal under 0.1 m away is dropped without a word'
                    + ('' if observed else
                       ' (from the config; no planner seen yet)'))}


def capabilities(cfg, path=None):
    """Which controls this config+path will actually honour, and why not.

    The panel renders these as the reason under a greyed-out control.  A button
    that looks live and does nothing is the worst outcome available here: it
    reads as a broken aircraft rather than a config setting.
    """
    if path is None:
        path = choose_path(cfg)
    out = {}
    ch = cfg.get('click_height')
    if path['keeps_z']:
        out['altitude'] = (True, None)
    elif ch is None:
        out['altitude'] = (False, 'cannot read fsm/click_height from the config')
    else:
        out['altitude'] = (False, (
            f'fsm/click_height = {ch} overrides the z in every goal on '
            f'{cfg.get("goal_topic")} (fsm.cpp:188), so every goal flies at '
            f'{ch} m. Either set fsm/follow_goal_en: true in '
            f'{os.path.basename(cfg.get("path") or "")} -- which keeps the z '
            'and leaves RViz clicks alone -- or set fsm/click_height: -6, '
            'which also makes an RViz click (always z=0) fly to the floor. '
            'Restart the stack either way'))

    if cfg.get('yaw_mode') == 3:
        out['yaw'] = (False, 'yaw_mode: 3 (YAW_TO_TARGET) -- an external yaw '
                             'target wins over the goal yaw')
    elif path['name'] == 'click' and not cfg.get('click_yaw_en'):
        out['yaw'] = (False, 'fsm/click_yaw_en is off, so the goal yaw is '
                             'discarded (fsm.cpp:216)')
    elif not cfg.get('goal_yaw_en'):
        out['yaw'] = (False, 'super_planner/goal_yaw_en is off, so the heading '
                             'follows the velocity (super_planner.cpp:814)')
    else:
        out['yaw'] = (True, None)
    return out


def clamp_altitude(z, cfg):
    band = cfg.get('altitude')
    if band is None:
        return z, None
    if z < band['min']:
        return band['min'], (f'altitude clamped up to {band["min"]} m -- below '
                             "that ROG-Map calls it 'below virtual ground'")
    if z > band['max']:
        return band['max'], (f'altitude clamped down to {band["max"]} m -- the '
                             'effective ceiling of the map')
    return z, None


def effective_z(z, cfg, path):
    """The z SUPER will actually use, which is not always the one we send."""
    if path['keeps_z']:
        return z
    ch = cfg.get('click_height')
    return z if ch is None else float(ch)


def require_fresh(pose, what='there is no frame to put a goal in'):
    """Every command needs a live pose: for the frame, and to predict the
    thresholds SUPER applies silently."""
    if pose is None:
        raise RCError('no /lidar_slam/odom -- FAST-LIO is not publishing, so '
                      + what)
    age = pose.get('age')
    # `age or 99` would be a bug here and was one: an age of exactly 0.0 is the
    # FRESHEST possible reading and is also falsy, so the slider refused every
    # command that arrived in the same 10 ms as its odometry. Check for None.
    if age is None or age > ODOM_STALE_S:
        raise RCError(f'/lidar_slam/odom is stale ({age if age is None else round(age, 1)}s) '
                      '-- check the LiDAR link before commanding anything')


def _finalise(x, y, z, yaw, pose, cfg, path, notes):
    """The tail every command shares: clamp z into the band, work out what
    SUPER will really fly to, and refuse what it would drop without a word.

    One place, because the interesting bugs here are all "two call sites
    disagreed about which z counts".
    """
    if path.get('contended'):
        raise RCError(path['contended'])
    z, why = clamp_altitude(float(z), cfg)
    if why:
        notes.append(why)
    eff = effective_z(z, cfg, path)
    if eff != z:
        notes.append(f'fsm/click_height overrides the z: this will fly at '
                     f'{eff} m, not {round(z, 2)} m')
    drop = path.get('drop_radius') or 0.0
    if drop:
        d = dist3(pose, x, y, eff)
        if d < drop:
            raise RCError(
                f'that is only {d:.2f} m from where it is now, and on '
                f'{path["topic"]} SUPER silently drops any goal closer than '
                f'{drop} m (fsm.cpp:201)')
    return {'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3),
            'yaw': round(wrap_pi(yaw), 4)}, notes


def plan_move(axis, step, pose, cfg, path=None, hold_z=None):
    """Where the next goal goes.  Pure arithmetic: no ROS, no I/O.

    `pose` is the aircraft NOW, in lidar_init: {'x','y','z','yaw','age'}.
    Returns (goal, notes) with goal = {'x','y','z','yaw'}.
    """
    if axis not in AXES:
        raise RCError(f'unknown axis {axis!r}')
    if path is None:
        path = choose_path(cfg)
    require_fresh(pose)
    caps = capabilities(cfg, path)
    notes = []
    if axis.startswith('yaw'):
        if step < YAW_MIN or step > YAW_MAX:
            raise RCError(f'yaw step must be between {YAW_MIN} and {YAW_MAX} degrees')
        d = math.radians(step) * (1.0 if axis == 'yaw_left' else -1.0)
        yaw = wrap_pi(pose['yaw'] + d)
        x, y = pose['x'], pose['y']
        if path['drop_radius']:
            # A pure rotation is dropped by fsm.cpp:201, so the goal is placed
            # just past that threshold along the NEW heading: the aircraft turns
            # and creeps, rather than the button doing nothing at all.  The
            # follow path has no such threshold, so it gets a pure rotation.
            x += YAW_NUDGE_M * math.cos(yaw)
            y += YAW_NUDGE_M * math.sin(yaw)
            notes.append(f'a pure rotation is dropped on {path["topic"]}, so '
                         f'the goal sits {YAW_NUDGE_M} m along the new heading')
    else:
        if step < STEP_MIN or step > STEP_MAX:
            raise RCError(f'step must be between {STEP_MIN} and {STEP_MAX} m')
        yaw = pose['yaw']
        dx = {'forward': step, 'back': -step}.get(axis, 0.0)
        dy = {'left': step, 'right': -step}.get(axis, 0.0)
        # Body frame -> lidar_init. "Forward" is where the nose points, which is
        # what forward means to whoever is looking at the aircraft.
        x = pose['x'] + dx * math.cos(yaw) - dy * math.sin(yaw)
        y = pose['y'] + dx * math.sin(yaw) + dy * math.cos(yaw)

    z = pose['z'] if hold_z is None else hold_z
    if axis in ('up', 'down'):
        x, y = pose['x'], pose['y']
        z = z + (step if axis == 'up' else -step)
    # Refused, not merely flagged. With click_height in force an up/down goal
    # keeps x and y and has its z overwritten, so "up" and "down" produce the
    # SAME goal -- i.e. the button would do nothing, silently, which is the
    # single worst thing a control on this page can do. Same for a yaw the
    # config discards: the aircraft would creep 15 cm and not turn.
    need = ('altitude' if axis in ('up', 'down')
            else 'yaw' if axis.startswith('yaw') else None)
    if need:
        ok, reason = caps.get(need, (True, None))
        if not ok:
            raise RCError(reason)
    return _finalise(x, y, z, yaw, pose, cfg, path, notes)


def plan_altitude(z, pose, cfg, path=None):
    """The altitude slider and the altitude box: an absolute z, position held."""
    if path is None:
        path = choose_path(cfg)
    require_fresh(pose, 'there is nothing to hold position at')
    ok, reason = capabilities(cfg, path).get('altitude', (True, None))
    if not ok:
        raise RCError(reason)
    # No special-casing of a small climb here any more: _finalise() applies the
    # path's own drop radius, which is 0.1 m on the click path and nothing at
    # all on the follow path -- where a 5 cm climb is a perfectly good goal.
    return _finalise(pose['x'], pose['y'], float(z), pose['yaw'],
                     pose, cfg, path, [])


def plan_goto(x, y, z, yaw_deg, pose, cfg, path=None):
    """The coordinate bar: an absolute point in lidar_init.

    Unlike up/down this is NOT refused when the config would override the z --
    an x/y goal is still a perfectly meaningful thing to ask for, and the note
    says what height it will really fly at.  The asymmetry is the point: up/down
    would become a no-op, this does not.
    """
    if path is None:
        path = choose_path(cfg)
    require_fresh(pose)
    notes = []
    try:
        x, y = float(x), float(y)
        z = pose['z'] if z is None or z == '' else float(z)
        # Blank yaw holds the current heading. NOT zero: zero is a real command
        # to turn to map north, and a blank box must never mean "rotate".
        if yaw_deg is None or yaw_deg == '':
            yaw = pose['yaw']
            notes.append('yaw left blank, so the current heading is held')
        else:
            yaw = math.radians(float(yaw_deg))
    except (TypeError, ValueError):
        raise RCError('x, y, z and yaw must be numbers -- metres in '
                      'lidar_init, and degrees. Leave z or yaw blank to hold '
                      'the current one') from None
    if not all(math.isfinite(v) for v in (x, y, z, yaw)):
        raise RCError('x, y, z and yaw must all be finite numbers')

    d = dist3(pose, x, y, effective_z(z, cfg, path))
    if d > GOTO_MAX_M:
        raise RCError(
            f'that goal is {d:.1f} m away and this bar is capped at '
            f'{GOTO_MAX_M} m -- at max_vel {cfg.get("max_vel")} m/s it would '
            f'already be {d / (cfg.get("max_vel") or 0.1):.0f} s of flight. '
            'Send it in steps')
    ok, _reason = capabilities(cfg, path).get('yaw', (True, None))
    if not ok and (yaw_deg is not None and yaw_deg != ''):
        notes.append('the goal yaw will be ignored: ' + str(_reason))
    return _finalise(x, y, z, yaw, pose, cfg, path, notes)


def goal_target(last, status, cfg, path):
    """The point the aircraft is actually flying to, which is often not the
    one that was typed.

    Two corrections, in order of authority:

    1. If SUPER answered on the follow path and the answer is about OUR goal,
       `adopted` is the truth -- it is the post-snap point, and the snap can
       move a goal by up to 3 m.
    2. Otherwise use what we sent, with `click_height` applied if the click
       path is in force.  Note this is an INFERENCE: the click path snaps too
       and never says so, so on that path "arrived" can be wrong by up to the
       snap radius.  The card says which path it is on.
    """
    goal = (last or {}).get('goal')
    if not goal:
        return None, False
    if status and status.get('ours') and status.get('adopted'):
        a = status['adopted']
        if all(a.get(k) is not None for k in ('x', 'y', 'z')):
            return {'x': a['x'], 'y': a['y'], 'z': a['z']}, True
    return {'x': goal['x'], 'y': goal['y'],
            'z': effective_z(goal['z'], cfg, path)}, False


def track_goal(last, pose, aircraft, status, cfg, path, now=None):
    """Did the last goal actually happen?  Mutates `last`'s tracking fields.

    The states are the questions an operator asks in order: did it go, did
    SUPER take it, is it moving, did it get there -- and the two that a naive
    "moving..." spinner gets wrong. Sending a goal DISARMED is the normal bench
    case and nothing will move, so saying "moving" would be a lie that never
    resolves; and a planner with no free path just sits there, which is a
    stall, not slow progress.
    """
    now = time.time() if now is None else now
    if not (last or {}).get('goal'):
        return None
    target, exact = goal_target(last, status, cfg, path)
    elapsed = now - (last.get('at') or now)
    out = {'target': target, 'exact': exact, 'elapsed': round(elapsed, 1),
           'what': last.get('what'), 'remaining': None, 'total': None,
           'frac': None, 'state': 'unknown',
           'label': 'no odometry',
           'detail': 'no fresh /lidar_slam/odom, so there is no way to tell '
                     'whether the aircraft moved'}

    # SUPER refused it outright. Only the follow path can tell us this; on the
    # click path a rejected goal is indistinguishable from a slow one.
    if status and status.get('ours') and status.get('name') == 'DROPPED':
        out.update(state='rejected', label='SUPER REFUSED the goal',
                   detail='the goal is deeply occupied, so it was dropped and '
                          'the PREVIOUS goal is still active (fsm.cpp:230)')
        last['state'] = 'rejected'
        return out

    age = None if pose is None else pose.get('age')
    if age is None or age > ODOM_STALE_S:
        last['state'] = 'unknown'
        return out

    remaining = dist3(pose, target['x'], target['y'], target['z'])
    origin = last.get('origin')
    total = (dist3(origin, target['x'], target['y'], target['z'])
             if origin else None)
    out['remaining'] = round(remaining, 2)
    if total and total > 1e-6:
        out['total'] = round(total, 2)
        out['frac'] = round(max(0.0, min(1.0, 1.0 - remaining / total)), 3)

    # Best approach so far, which is what "is it still making progress" means.
    best = last.get('best')
    if best is None or remaining < best - STALL_IMPROVE_M:
        last['best'] = remaining
        last['best_at'] = now
    stuck_for = now - (last.get('best_at') or now)

    where = f'[{target["x"]:.2f}, {target["y"]:.2f}, {target["z"]:.2f}]'
    if remaining <= ARRIVE_M:
        # Three different confidences, and saying the wrong one is worse than
        # saying nothing: on the follow path with a matching status this IS
        # the point SUPER flew to; without one we only know what we sent; on
        # the click path nothing is ever reported and the snap is invisible.
        caveat = ('' if exact
                  else ' (the point we sent -- SUPER did not confirm one)'
                  if path['reports']
                  else ' (the point we sent: the click path never reports '
                       'what it adopted, and it can snap by up to '
                       f'{SNAP_RADIUS_M} m)')
        out.update(state='arrived', label='Arrived',
                   detail=f'at {where}, {remaining * 100:.0f} cm off, '
                          f'{elapsed:.0f} s after the goal went out' + caveat)
        last['state'] = 'arrived'
        return out

    armed = bool((aircraft or {}).get('armed'))
    offb = bool((aircraft or {}).get('offboard'))
    if not (armed and offb):
        why = ('disarmed' if not armed else 'not in Offboard')
        if not armed and not offb:
            why = 'disarmed and not in Offboard'
        out.update(state='idle', label='Sent — but nothing is flying',
                   detail=f'SUPER has the goal and is planning to {where}, but '
                          f'the aircraft is {why}, so it will not move. '
                          f'{remaining:.2f} m away.')
        last['state'] = 'idle'
        return out

    vel = cfg.get('max_vel')
    eta = f', about {remaining / vel:.0f} s at {vel} m/s' if vel else ''
    if stuck_for > STALL_AFTER_S:
        out.update(state='stalled', label='No progress',
                   detail=f'armed in Offboard but no closer for {stuck_for:.0f} s, '
                          f'still {remaining:.2f} m from {where} -- SUPER may '
                          'have no free path to it')
        last['state'] = 'stalled'
        return out

    out.update(state='moving', label='Moving',
               detail=f'{remaining:.2f} m to {where}{eta}')
    last['state'] = 'moving'
    return out


if __name__ == '__main__':
    cfg = read_config()
    path = choose_path(cfg)
    print(f'config: {cfg["path"]}')
    if cfg['error']:
        print(f'  ERROR {cfg["error"]}')
    for k in ('click_height', 'click_yaw_en', 'yaw_mode', 'goal_yaw_en',
              'max_vel', 'yaw_dot_max', 'goal_topic', 'follow_goal_en',
              'follow_goal_topic', 'goal_status_topic'):
        print(f'  {k:18s} {cfg[k]}')
    print(f'  altitude band      {cfg["altitude"]}')
    print(f'\ngoal path: {path["name"]} -> {path["topic"]}')
    print(f'  {path["why"]}')
    for name, (ok, why) in capabilities(cfg, path).items():
        print(f'  {name:9s} {"LIVE" if ok else "inert"}' + (f' -- {why}' if why else ''))
