#!/usr/bin/env python3
"""Test the DTP follower's Start interlock and the Stack row's config choice.

    python3 scripts/gcs/test_dtp.py

Two halves, and neither can start or stop anything real:

  unit    dtp.start_blocked() walked refusal by refusal with no ROS, camera or
          aircraft; the launch-parameter reader against YAML files it writes
          itself; supervisor.start_options()' whitelists; and Restart's refusal
          with the process checks stubbed out.
  server  its OWN server on a free port, --no-ros --no-auth, with HOME pointed
          at a scratch directory holding a test admin password (SUPER_WS still
          points at the real workspace).  It asserts the locked shape, then the
          unlocked one.  It never POSTs a follower start that the server's own
          state did not already say is refused, and it never POSTs a stop with
          an admin session -- a stop adopts, and on this aircraft a follower is
          often running in somebody's terminal.
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import dtp                      # noqa: E402
import supervisor as sup        # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  -- {detail}' if detail else ''))
    if not cond:
        fails.append(name)


# -- unit: the rule -----------------------------------------------------------

STOPPED = {'key': 'dtp', 'state': 'stopped', 'pid': None}
GROUND = {'fresh': True, 'armed': False, 'landed': True}
FSM_FOLLOW = {'name': dtp.FOLLOW_CONFIG, 'pids': [100], 'error': None}
FSM_CLICK = {'name': dtp.CLICK_CONFIG, 'pids': [100], 'error': None}
CAM_FREE = {'devices': [{'id': '2bc5:0800', 'node': '/dev/bus/usb/002/003'}],
            'sources': [], 'holders': []}


def blocked(mode, **over):
    """start_blocked() with everything fine, except what the case overrides."""
    kw = dict(row=STOPPED, aircraft=GROUND, ros_ok=True, fsm=FSM_FOLLOW,
              missing=[], camera_row={'key': 'camera', 'state': 'stopped'},
              cam=CAM_FREE, odom_hz=10.0, install_error=None)
    kw.update(over)
    return dtp.start_blocked(mode, **kw)


def unit_rule():
    print('the interlock  (dtp.start_blocked)')
    for mode in dtp.MODES:
        check(f'{mode}: everything fine starts', blocked(mode) is None,
              str(blocked(mode)))
    check('an unknown mode is refused', bool(blocked('follow')))

    for mode in dtp.MODES:
        check(f'{mode}: already running here',
              'already running' in (blocked(mode, row=dict(STOPPED, state='running')) or ''))
        check(f'{mode}: already running in a terminal names the pid',
              '4242' in (blocked(mode, row=dict(STOPPED, state='external', pid=4242)) or ''))
        check(f'{mode}: not installed', blocked(mode, install_error='no dtp') == 'no dtp')
        why = blocked(mode, aircraft={'fresh': True, 'armed': True, 'landed': True})
        check(f'{mode}: ARMED refuses', 'ARMED' in (why or ''), str(why))
        why = blocked(mode, aircraft={'fresh': False, 'armed': True})
        check(f'{mode}: armed-then-link-lost still refuses', 'stale' in (why or ''), str(why))
        why = blocked(mode, aircraft={'fresh': True, 'armed': False, 'landed': False})
        check(f'{mode}: IN AIR refuses', 'IN AIR' in (why or ''), str(why))
        why = blocked(mode, missing=['fsm_node'])
        check(f'{mode}: no stack refuses, naming what is missing',
              'fsm_node' in (why or '') and 'stack' in (why or ''), str(why))
        why = blocked(mode, fsm=dict(FSM_FOLLOW, pids=[1, 2], error='2 fsm_node'))
        check(f'{mode}: two fsm_nodes refuse', why == '2 fsm_node', str(why))
        why = blocked(mode, camera_row={'key': 'camera', 'state': 'running'})
        check(f'{mode}: the dashboard camera holding the Gemini refuses',
              'Camera panel' in (why or ''), str(why))
        why = blocked(mode, cam=dict(CAM_FREE, devices=[]))
        check(f'{mode}: no camera on USB refuses', 'USB' in (why or ''), str(why))
        why = blocked(mode, cam=dict(CAM_FREE, sources=[
            {'topic': '/camera/color/image_raw/compressed'}]))
        check(f'{mode}: another camera publisher refuses, naming it',
              '/camera/color' in (why or ''), str(why))
        why = blocked(mode, cam=dict(CAM_FREE, holders=[{'pid': 77, 'cmd': 'x y'}]))
        check(f'{mode}: a USB holder refuses, naming it', '77' in (why or ''), str(why))
        check(f'{mode}: our own /dtp/ overlay is not a camera owner',
              blocked(mode, cam=dict(CAM_FREE, sources=[
                  {'topic': '/dtp/image_annotated/compressed'}])) is None)

    print('\n  observe only')
    check('an FC never seen does not block the bench',
          blocked('observe', aircraft=None, ros_ok=False) is None)
    check('the click config is fine: it commands nothing',
          blocked('observe', fsm=FSM_CLICK) is None)
    check('a slow or unmeasured odometry rate is fine',
          blocked('observe', odom_hz=None) is None)

    print('\n  live only')
    why = blocked('live', ros_ok=False, aircraft=None)
    check('no ROS telemetry refuses', 'telemetry' in (why or ''), str(why))
    why = blocked('live', aircraft={'fresh': False, 'armed': None})
    check('a silent FC refuses: silence is not "disarmed"',
          'vehicle_status' in (why or ''), str(why))
    why = blocked('live', fsm=FSM_CLICK)
    check('the click config refuses, naming both configs',
          dtp.CLICK_CONFIG in (why or '') and dtp.FOLLOW_CONFIG in (why or ''), str(why))
    why = blocked('live', fsm={'name': None, 'pids': [1], 'error': 'cannot read'})
    check('an unreadable config refuses', 'cannot read' in (why or ''), str(why))
    why = blocked('live', odom_hz=4.0)
    check('odometry below 9.5 Hz refuses', '4.0 Hz' in (why or ''), str(why))
    why = blocked('live', odom_hz=None)
    check('unmeasured odometry refuses', 'unknown rate' in (why or ''), str(why))

    print('\n  stop note  (never a refusal)')
    check('nothing to say on the ground',
          dtp.stop_note({'mode': 'live'}, GROUND) is None)
    note = dtp.stop_note({'mode': 'live'}, {'fresh': True, 'armed': True, 'offboard': True})
    check('armed + live warns about the last goal and the abort',
          'OFFBOARD' in note and 'NAV_LAND' in note and 'channel 11' in note, note)
    note = dtp.stop_note({'mode': 'observe'}, {'fresh': True, 'armed': True})
    check('armed + observe says the flight is unaffected',
          'unaffected' in note, note)


# -- unit: reading what the processes were given -----------------------------

def unit_follow_rule():
    print('\nfollow_blocked (pure)')
    run = {'state': 'running'}
    ok_state = {'lock': 'LOST|-|forecast ran out', 'mux': 'HOLD|NO_SOURCE|...', 'available': True}
    check('a running follower with telemetry and a served parameter service may engage',
          dtp.follow_blocked('on', row=run, ros_ok=True, state=ok_state) is None)
    check('...and disengage',
          dtp.follow_blocked('off', row=run, ros_ok=True, state=ok_state) is None)
    check('an unknown action is refused by name',
          'unknown action' in (dtp.follow_blocked('go', row=run, ros_ok=True, state=ok_state) or ''))
    check('no telemetry refuses, naming the shell alternative',
          'field.sh' in (dtp.follow_blocked('on', row=run, ros_ok=False, state=ok_state) or ''))
    check('no follower refuses',
          'no DTP follower' in (dtp.follow_blocked('on', row={'state': 'stopped'}, ros_ok=True, state=ok_state) or ''))
    check('...and so does a missing row',
          'no DTP follower' in (dtp.follow_blocked('on', row=None, ros_ok=True, state=ok_state) or ''))
    check('an external (terminal-started) follower is fine',
          dtp.follow_blocked('on', row={'state': 'external'}, ros_ok=True, state=ok_state) is None)
    check('no parameter service yet refuses',
          'parameter service' in (dtp.follow_blocked('on', row=run, ros_ok=True,
                                                     state=dict(ok_state, available=False)) or ''))
    aborted = dict(ok_state, mux='ABORT|LAND|sensors lost')
    check('after a mux ABORT, engage is refused until relaunch',
          'ABORT' in (dtp.follow_blocked('on', row=run, ros_ok=True, state=aborted) or ''))
    check('...but disengage is not',
          dtp.follow_blocked('off', row=run, ros_ok=True, state=aborted) is None)
    # The deliberate non-rule: being armed does not refuse. There is no aircraft
    # argument at all, so a future edit cannot quietly add one without a test.
    import inspect
    check('the rule takes no aircraft argument: armed never refuses a re-engage',
          'aircraft' not in inspect.signature(dtp.follow_blocked).parameters)


def unit_params(tmp):
    print('\nlaunch parameters  (dtp.launch_param)')

    def yml(name, text):
        path = os.path.join(tmp, name)
        with open(path, 'w') as f:
            f.write(text)
        return path

    shared = yml('dtp_params.yaml',
                 'command_mux:\n  ros__parameters:\n    arbitrated_goal_topic: /from/shared\n'
                 'follow_planner_node:\n  ros__parameters:\n    yaw_target_topic: /other/node\n')
    launch = yml('launch_a.yaml',
                 '/**:\n  ros__parameters:\n    arbitrated_goal_topic: /dtp/shadow/follow_goal\n')
    fsm = yml('launch_b.yaml', '/**:\n  ros__parameters:\n    config_name: follow_test_ros2.yaml\n')

    argv = ['/x/command_mux_node', '--ros-args', '-r', '__node:=command_mux',
            '--params-file', shared, '--params-file', launch]
    check('a later params file wins',
          dtp.launch_param(argv, 'arbitrated_goal_topic')
          == (True, '/dtp/shadow/follow_goal'))
    check('this node\'s own block in a shared file counts',
          dtp.launch_param(argv[:6], 'arbitrated_goal_topic') == (True, '/from/shared'))
    fp = ['/x/follow_planner_node', '--ros-args', '-r', '__node:=command_mux',
          '--params-file', shared]
    check('ANOTHER node\'s block in a shared file does not',
          dtp.launch_param(fp, 'yaw_target_topic') == (False, None))
    check('-p name:=value is read',
          dtp.launch_param(argv + ['-p', 'arbitrated_goal_topic:=/planning/follow_goal'],
                           'arbitrated_goal_topic') == (True, '/planning/follow_goal'))
    check('an unreadable file is skipped, not fatal',
          dtp.launch_param(['n', '--params-file', os.path.join(tmp, 'gone.yaml')],
                           'config_name') == (False, None))

    # The two process readers, with the process table stubbed out.
    real_procs, real_argv = dtp._procs, dtp.sysinfo.argv
    table = {}
    dtp._procs = lambda pattern: sorted(table.get(pattern, {}))
    dtp.sysinfo.argv = lambda pid: next(
        (a for t in table.values() for p, a in t.items() if p == pid), [])
    try:
        table = {'fsm_node': {10: ['/x/fsm_node', '--ros-args', '--params-file', fsm]}}
        got = dtp._fsm_config()
        check('fsm_config reads the loaded config',
              got['name'] == dtp.FOLLOW_CONFIG and got['error'] is None, json.dumps(got))
        table = {'fsm_node': {10: ['/x/fsm_node'], 11: ['/x/fsm_node']}}
        got = dtp._fsm_config()
        check('two fsm_nodes are an error, not a pick',
              '2 fsm_node' in (got['error'] or ''), json.dumps(got))
        table = {}
        got = dtp._fsm_config()
        check('no fsm_node is "not running", with no error',
              got['name'] is None and got['error'] is None and not got['pids'],
              json.dumps(got))

        shadow = yml('fp_shadow.yaml',
                     '/**:\n  ros__parameters:\n    yaw_target_topic: /dtp/shadow/yaw_target\n')
        live_fp = yml('fp_live.yaml',
                      '/**:\n  ros__parameters:\n    yaw_target_topic: /planning/yaw_target\n')
        live_mux = yml('mux_live.yaml',
                       '/**:\n  ros__parameters:\n    arbitrated_goal_topic: /planning/follow_goal\n')
        mux = lambda f: {20: ['/x/command_mux_node', '--params-file', f]}   # noqa: E731
        fpl = lambda f: {21: ['/x/follow_planner_node', '--params-file', f]}  # noqa: E731
        table = {'command_mux_node': mux(launch), 'follow_planner_node': fpl(shadow)}
        check('both outputs shadowed reads as observe', dtp._follower_mode() == 'observe')
        table = {'command_mux_node': mux(live_mux), 'follow_planner_node': fpl(live_fp)}
        check('both outputs to SUPER reads as live', dtp._follower_mode() == 'live')
        table = {'command_mux_node': mux(live_mux), 'follow_planner_node': fpl(shadow)}
        check('half-redirected reads as mixed, not whichever came first',
              dtp._follower_mode() == 'mixed')
        table = {}
        check('no follower nodes is no mode', dtp._follower_mode() is None)
    finally:
        dtp._procs, dtp.sysinfo.argv = real_procs, real_argv


# -- unit: the supervisor -----------------------------------------------------

def unit_supervisor():
    print('\nstart options  (supervisor.start_options)')
    opts, env = sup.start_options('stack', None)
    check('the stack defaults to the click config',
          env == {'SUPER_CONFIG': dtp.CLICK_CONFIG} and opts == {'config': 'click'})
    opts, env = sup.start_options('stack', {'config': 'follow'})
    check('config=follow selects the follow config',
          env['SUPER_CONFIG'] == dtp.FOLLOW_CONFIG)
    for bad in ({'config': '../../etc/passwd'}, {'config': dtp.FOLLOW_CONFIG},
                {'config': 'follow', 'x': 1}):
        try:
            sup.start_options('stack', bad)
            check(f'stack refuses {bad}', False)
        except ValueError:
            check(f'stack refuses {bad}', True)
    try:
        sup.start_options('dtp', None)
        check('the follower has NO default mode', False)
    except ValueError as exc:
        check('the follower has NO default mode', 'mode' in str(exc), str(exc))
    opts, env = sup.start_options('dtp', {'mode': 'live', 'config': dtp.FOLLOW_CONFIG})
    check('mode and config reach the wrapper',
          env == {'DTP_MODE': 'live', 'SUPER_CONFIG': dtp.FOLLOW_CONFIG}, json.dumps(env))
    for bad in ({'mode': 'LIVE'}, {'mode': 'observe', 'config': 'x.yaml'}):
        try:
            sup.start_options('dtp', bad)
            check(f'follower refuses {bad}', False)
        except ValueError:
            check(f'follower refuses {bad}', True)
    try:
        sup.start_options('odom_bridge', {'config': 'follow'})
        check('a row without options refuses them', False)
    except ValueError:
        check('a row without options refuses them', True)
    check('the follower row exists and stops before the bridges',
          sup.ORDER.index('dtp') > sup.ORDER.index('offboard_bridge'))
    check('...and never matches the generic Orbbec container',
          'component_container' not in sup.PROCS['dtp'][3])

    print('\nrestart')
    s = sup.Supervisor(os.path.dirname(os.path.dirname(HERE)))
    started = []
    # Stub every process check: this must not depend on what runs on this box,
    # and must never reach a real start.
    for p in s.procs.values():
        p.alive = lambda: False
        p.external_pids = lambda: []
    s._restart = lambda keys, opts: (started.append((keys, opts)), s._busy.release())
    s.procs['dtp'].external_pids = lambda: [4242]
    ok, msg = s.restart()
    check('Restart is refused while a follower runs in a terminal',
          not ok and 'follower' in msg, msg)
    s.procs['dtp'].external_pids = lambda: []
    s.procs['dtp'].alive = lambda: True
    ok, msg = s.restart()
    check('...and while ours runs', not ok and 'follower' in msg, msg)
    check('...and the busy lock was released both times', not s._busy.locked())
    s.procs['dtp'].alive = lambda: False
    s.procs['stack'].alive = lambda: True
    s.procs['stack'].opts = {'config': 'follow'}
    ok, msg = s.restart()
    time.sleep(0.2)
    check('otherwise it restarts what was up', ok and started
          and started[0][0] == ['stack'], f'{msg} {started}')
    check('...on the config it was on',
          bool(started) and started[0][1] == {'stack': {'config': 'follow'}},
          json.dumps(started[0][1] if started else None))


# -- server -------------------------------------------------------------------

def call(base, path, method='GET', admin=None, body=None):
    headers = {'X-GCS-Admin': admin} if admin else {}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        headers['Content-Type'] = 'application/json'
    req = urllib.request.Request(base + path, method=method, headers=headers, data=data)
    try:
        with urllib.request.urlopen(req, timeout=20) as r:
            return r.status, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read() or b'{}')
        except ValueError:
            return e.code, {}


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def server_half(tmp):
    import admin
    password = 'test-only-' + os.urandom(6).hex()
    home = os.path.join(tmp, 'home')
    os.makedirs(os.path.join(home, '.cache', 'super-gcs'))
    with open(os.path.join(home, '.cache', 'super-gcs', 'admin'), 'w') as f:
        f.write(admin.encode(password) + '\n')

    port = free_port()
    env = dict(os.environ, HOME=home, SUPER_WS=dtp.SUPER_WS)
    proc = subprocess.Popen(
        [sys.executable, os.path.join(HERE, 'server.py'), '--port', str(port),
         '--bind', '127.0.0.1', '--no-auth', '--no-ros'],
        env=env, cwd=os.path.dirname(os.path.dirname(HERE)),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL)
    base = f'http://127.0.0.1:{port}'
    try:
        for _ in range(50):
            try:
                call(base, '/api/state')
                break
            except Exception:
                time.sleep(0.2)
        else:
            check('the test server came up', False, f'port {port}')
            return

        print('\nserver, locked')
        _code, st = call(base, '/api/state')
        row = next((r for r in st.get('procs', []) if r['key'] == 'dtp'), None)
        check('the follower row is shown to a locked client', row is not None)
        if row:
            check('...marked locked', row.get('locked') is True, json.dumps(row))
            check('...with no reasons, which name the camera',
                  'blocked' not in row and 'stop_note' not in row, sorted(row))
        stack = next((r for r in st.get('procs', []) if r['key'] == 'stack'), {})
        check('the stack row says which config fsm_node is on',
              'config' in stack, sorted(stack))
        _code, cfg = call(base, '/api/config')
        check('no follower log tab while locked', 'dtp' not in cfg.get('procs', {}))
        check('the stack configs are offered', cfg.get('stack_configs') == dtp.STACK_CONFIGS)
        check('the locked row carries the lock state field (None without ROS), never the reasons',
              'follow' in (row or {}) and 'follow_blocked' not in (row or {}), sorted(row or {}))
        for path, method in (('/api/proc/dtp/start?mode=observe', 'POST'),
                             ('/api/proc/dtp/start?mode=live&confirm=live', 'POST'),
                             ('/api/proc/dtp/stop', 'POST'),
                             ('/api/dtp/follow?set=on', 'POST'),
                             ('/api/dtp/follow?set=off', 'POST'),
                             ('/api/logs/dtp', 'GET')):
            code, body = call(base, path, method)
            check(f'{method} {path} needs Admin control',
                  code == 403 and body.get('admin_required') is True, str(code))

        print('\nserver, unlocked')
        _code, body = call(base, '/api/admin/login', 'POST', body={'password': password})
        session = body.get('session')
        check('the test password unlocks', bool(session), json.dumps(body))
        if not session:
            return
        _code, st = call(base, '/api/state', admin=session)
        row = next((r for r in st.get('procs', []) if r['key'] == 'dtp'), {})
        check('the follower row carries one reason per mode',
              set((row.get('blocked') or {})) == set(dtp.MODES), json.dumps(row))
        _code, cfg = call(base, '/api/config', admin=session)
        check('the follower log tab appears', 'dtp' in cfg.get('procs', {}))
        code, body = call(base, '/api/logs/dtp', admin=session)
        check('...and its log is readable', code == 200, str(code))

        code, body = call(base, '/api/proc/dtp/start', 'POST', admin=session)
        check('a start with no mode is a 400, not a default', code == 400,
              f'{code} {body.get("message")}')
        check('the unlocked row carries one follow reason per action',
              set((row.get('follow_blocked') or {})) == set(dtp.FOLLOW_ACTIONS), json.dumps(row))
        code, body = call(base, '/api/dtp/follow', 'POST', admin=session)
        check('a follow with no ?set is a 400, not a default', code == 400,
              f'{code} {body.get("message")}')
        code, body = call(base, '/api/dtp/follow?set=on', 'POST', admin=session)
        check('with no telemetry an engage is refused, naming the shell alternative',
              code == 409 and 'field.sh' in body.get('message', ''),
              f'{code} {body.get("message")}')
        check('...with the reason the row showed',
              body.get('message') == 'refused: ' + (row.get('follow_blocked') or {}).get('on', '?'),
              f'{body.get("message")}')
        # Safe to send whatever the aircraft is doing: the confirm gate answers
        # before the interlock is consulted.
        for q in ('mode=live', 'mode=live&confirm=yes', 'mode=live&confirm=observe'):
            code, body = call(base, f'/api/proc/dtp/start?{q}', 'POST', admin=session)
            check(f'?{q} is refused for want of confirm=live',
                  code == 409 and 'confirm=live' in body.get('message', ''),
                  f'{code} {body.get("message")}')
        # With --no-ros there is no telemetry, so a confirmed live start is
        # refused by the interlock -- say so, and never start one to find out.
        live_why = (row.get('blocked') or {}).get('live')
        if live_why:
            code, body = call(base, '/api/proc/dtp/start?mode=live&confirm=live',
                              'POST', admin=session)
            check('a confirmed live start is refused with the reason the row showed',
                  code == 409 and body.get('message') == 'refused: ' + live_why,
                  f'{code} {body.get("message")}')
        else:
            check('with no telemetry, live must be blocked', False, json.dumps(row))
        obs_why = (row.get('blocked') or {}).get('observe')
        if obs_why:
            code, body = call(base, '/api/proc/dtp/start?mode=observe', 'POST',
                              admin=session)
            check('an observe start the row refused is refused by the POST too',
                  code == 409 and body.get('message', '').startswith('refused: '),
                  f'{code} {body.get("message")}')
        else:
            print('  SKIP  observe start: the row says it would START here, and this '
                  'test does not start followers')
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()


def main():
    tmp = tempfile.mkdtemp(prefix='gcs-dtp-test-')
    try:
        unit_rule()
        unit_follow_rule()
        unit_params(tmp)
        unit_supervisor()
        server_half(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('\n' + (f'{len(fails)} FAILED: ' + ', '.join(fails) if fails else 'all passed'))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())
