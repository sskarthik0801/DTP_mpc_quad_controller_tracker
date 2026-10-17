#!/usr/bin/env python3
"""Test the remote-control goal maths and the gates around it.

    python3 scripts/gcs/test_rc.py

Two halves.  The first is pure arithmetic -- where a goal lands for each
button, in the frame SUPER plans in -- and needs nothing running.  The second
spawns its own server on a free port to check the admin gate and the refusals.

That server is started with **--no-ros on purpose, and this test must stay that
way**: with telemetry live, a passing "forward" case would publish a real goal,
and if the aircraft happened to be armed in Offboard that is a test that flies a
drone.  The arithmetic is verified directly instead, which is where the mistakes
would be anyway -- a sign error in the body-frame rotation sends it backwards,
and no HTTP status code would show that.
"""
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
fails = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  -- {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def near(a, b, tol=1e-3):
    return abs(a - b) <= tol


def call(base, path, method='GET', admin=None, body=None):
    req = urllib.request.Request(
        base + path, method=method,
        data=json.dumps(body).encode() if body is not None else None,
        headers={**({'X-GCS-Admin': admin} if admin else {}),
                 **({'Content-Type': 'application/json'} if body is not None else {})})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
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


def main():
    import rc

    live = {'x': 10.0, 'y': -4.0, 'z': 1.5, 'yaw': 0.0, 'age': 0.1}

    def mkcfg(**over):
        """A config with everything enabled, so the ARITHMETIC is tested rather
        than whatever this machine's config happens to say today."""
        cfg = {'click_height': 1.5, 'click_yaw_en': True, 'goal_yaw_en': True,
               'yaw_mode': 1, 'ground': -1.0, 'ceil': 3.5, 'path': 'test.yaml',
               'goal_topic': '/goal_pose', 'max_vel': 0.1,
               'follow_goal_en': False,
               'follow_goal_topic': rc.FOLLOW_GOAL_TOPIC,
               'goal_status_topic': rc.GOAL_STATUS_TOPIC}
        cfg.update(over)
        cfg['altitude'] = rc.altitude_band(cfg)
        return cfg

    # The two paths, as the panel will see them: the follow path observed live
    # on the graph, and the click path with click_height in force.
    cfg = mkcfg()
    follow = rc.choose_path(cfg, follow_subs=1, click_subs=1)
    click = rc.choose_path(cfg, follow_subs=0, click_subs=1)

    print('the two goal paths  (which one is live decides what the buttons do)')
    check('a subscribed follow topic wins', follow['name'] == 'follow',
          follow['topic'])
    check('...and no subscriber on it falls back to the click path',
          click['name'] == 'click', click['topic'])
    check('the follow path keeps z DESPITE click_height 1.5',
          follow['keeps_z'] is True,
          'setFollowGoal() has no override -- fsm.cpp:227 says so in a comment')
    check('...and the click path does not', click['keeps_z'] is False)
    check('only the follow path has no silent drop radius',
          follow['drop_radius'] == 0.0 and click['drop_radius'] == 0.1)
    check('with no graph at all the config decides',
          rc.choose_path(mkcfg(follow_goal_en=True))['name'] == 'follow'
          and rc.choose_path(mkcfg())['name'] == 'click')
    check('...and says the answer was not observed',
          'no planner seen yet' in rc.choose_path(mkcfg())['why'])

    print('\nwhat each path lets the controls do')
    fcap = rc.capabilities(cfg, follow)
    ccap = rc.capabilities(cfg, click)
    check('ALTITUDE IS LIVE on the follow path with click_height still 1.5',
          fcap['altitude'][0] is True,
          'this is the whole reason the follow path is preferred')
    check('...and inert on the click path', ccap['altitude'][0] is False,
          str(ccap['altitude'][1])[:60])
    check('...with a reason naming both ways out',
          'follow_goal_en' in ccap['altitude'][1]
          and 'click_height: -6' in ccap['altitude'][1])
    check('click_yaw_en gates yaw only on the CLICK path',
          rc.capabilities(mkcfg(click_yaw_en=False), follow)['yaw'][0] is True
          and rc.capabilities(mkcfg(click_yaw_en=False), click)['yaw'][0] is False,
          'followGoalCallback reads the quaternion straight (fsm_ros2.hpp:297)')
    check('goal_yaw_en gates yaw on BOTH',
          rc.capabilities(mkcfg(goal_yaw_en=False), follow)['yaw'][0] is False)

    print('\nbody frame  (the sign errors that would fly it the wrong way)')
    g, _ = rc.plan_move('forward', 1.0, live, cfg, follow)
    check('nose-forward at yaw 0 is +x', near(g['x'], 11.0) and near(g['y'], -4.0),
          json.dumps(g))
    g, _ = rc.plan_move('left', 1.0, live, cfg, follow)
    check('left at yaw 0 is +y', near(g['y'], -3.0) and near(g['x'], 10.0),
          json.dumps(g))
    g, _ = rc.plan_move('right', 1.0, live, cfg, follow)
    check('right at yaw 0 is -y', near(g['y'], -5.0), json.dumps(g))
    g, _ = rc.plan_move('back', 1.0, live, cfg, follow)
    check('back at yaw 0 is -x', near(g['x'], 9.0), json.dumps(g))

    turned = dict(live, yaw=math.pi / 2)
    g, _ = rc.plan_move('forward', 2.0, turned, cfg, follow)
    check('forward follows the NOSE, not the map',
          near(g['x'], 10.0) and near(g['y'], -2.0), json.dumps(g))
    g, _ = rc.plan_move('right', 1.0, turned, cfg, follow)
    check('...and so does sideways', near(g['x'], 11.0) and near(g['y'], -4.0),
          json.dumps(g))

    print('\nrotation  (a nudge on the click path, a pure turn on the follow one)')
    g, notes = rc.plan_move('yaw_left', 90, live, cfg, click)
    d = math.hypot(g['x'] - live['x'], g['y'] - live['y'])
    check('a rotation clears the click path 0.1 m drop radius', d > 0.1,
          f'{d:.3f} m -- fsm.cpp:201 drops anything closer, silently')
    check('...and it is a rotation, not a flight', d < 0.3, f'{d:.3f} m')
    check('...to the requested heading', near(g['yaw'], math.pi / 2),
          f'{math.degrees(g["yaw"]):.1f} deg')
    g, notes = rc.plan_move('yaw_left', 90, live, cfg, follow)
    d = math.hypot(g['x'] - live['x'], g['y'] - live['y'])
    check('the follow path turns in place -- no nudge at all', near(d, 0.0),
          f'{d:.3f} m; setFollowGoal has no drop radius to clear')
    check('...and does not claim it added one', not any('heading' in n for n in notes),
          json.dumps(notes))
    g, _ = rc.plan_move('yaw_right', 90, live, cfg, follow)
    check('rotating right is negative yaw', near(g['yaw'], -math.pi / 2),
          f'{math.degrees(g["yaw"]):.1f} deg')

    print('\naltitude')
    g, _ = rc.plan_altitude(2.0, live, cfg, follow)
    check('an absolute height holds x and y',
          near(g['x'], 10.0) and near(g['y'], -4.0) and near(g['z'], 2.0),
          json.dumps(g))
    try:
        rc.plan_altitude(live['z'] + 0.05, live, cfg, click)
        check('a sub-0.1 m climb is refused on the CLICK path, not dropped', False)
    except rc.RCError as exc:
        check('a sub-0.1 m climb is refused on the CLICK path, not dropped',
              True, str(exc)[:64])
    try:
        g, _ = rc.plan_altitude(live['z'] + 0.05, live, cfg, follow)
        check('...and allowed on the follow path, which has no such rule',
              near(g['z'], 1.55), json.dumps(g))
    except rc.RCError as exc:
        check('...and allowed on the follow path, which has no such rule',
              False, str(exc))

    # Regression: age 0.0 is the freshest reading there is, and it is falsy.
    # `age or 99` refused every command that arrived in the same tick as its
    # odometry -- which is most of them, and only showed up end to end.
    try:
        g, _ = rc.plan_altitude(2.0, dict(live, age=0.0), cfg, follow)
        check('odometry aged exactly 0.0 s is FRESH, not stale',
              near(g['z'], 2.0), json.dumps(g))
    except rc.RCError as exc:
        check('odometry aged exactly 0.0 s is FRESH, not stale', False, str(exc))

    print('\nthe altitude band  (ROG-Map insets it -- the configured one is a lie)')
    band = cfg['altitude']
    check('band is inside the configured [-1.0, 3.5]',
          band['min'] > -1.0 and band['max'] < 3.5, json.dumps(band))
    g, notes = rc.plan_move('up', 3.0, live, cfg, follow)
    check('climbing past the ceiling clamps', near(g['z'], band['max']),
          json.dumps(g))
    check('...and says so', any('clamp' in n for n in notes), json.dumps(notes))
    g, notes = rc.plan_move('down', 3.0, live, cfg, follow)
    check('descending past the floor clamps', near(g['z'], band['min']),
          json.dumps(g))

    print('\nthe coordinate bar  (one absolute goal, not a nudge)')
    g, notes = rc.plan_goto(14.0, -4.0, 2.0, 90, live, cfg, follow)
    check('an absolute point is sent verbatim',
          near(g['x'], 14.0) and near(g['y'], -4.0) and near(g['z'], 2.0),
          json.dumps(g))
    check('...with yaw in DEGREES in and radians out', near(g['yaw'], math.pi / 2),
          f'{math.degrees(g["yaw"]):.1f} deg')
    g, notes = rc.plan_goto(14.0, -4.0, 2.0, '', live, cfg, follow)
    check('a BLANK yaw holds the current heading, it does not mean zero',
          near(g['yaw'], live['yaw']) and any('blank' in n for n in notes),
          json.dumps(g) + json.dumps(notes))
    g, _ = rc.plan_goto(14.0, -4.0, '', '', live, cfg, follow)
    check('a blank z holds the current height', near(g['z'], live['z']),
          json.dumps(g))
    g, _ = rc.plan_goto('12.5', '-4', '2', '45', live, cfg, follow)
    check('strings off a query string are numbers here',
          near(g['x'], 12.5) and near(g['yaw'], math.pi / 4), json.dumps(g))

    try:
        rc.plan_goto(live['x'] + rc.GOTO_MAX_M + 1, live['y'], live['z'], '',
                     live, cfg, follow)
        check(f'a goal past the {rc.GOTO_MAX_M} m cap is refused', False,
              'a mistyped digit must not send it across a field')
    except rc.RCError as exc:
        check(f'a goal past the {rc.GOTO_MAX_M} m cap is refused',
              'capped' in str(exc), str(exc)[:70])
    g, _ = rc.plan_goto(live['x'] + rc.GOTO_MAX_M - 0.1, live['y'], live['z'],
                        '', live, cfg, follow)
    check('...and one just inside it is not', g is not None, json.dumps(g))

    # The asymmetry that matters: up/down under click_height is a NO-OP and is
    # refused; an x/y goal under click_height is still a real goal.
    g, notes = rc.plan_goto(14.0, -4.0, 2.0, '', live, cfg, click)
    check('a coordinate goal is ALLOWED when click_height would override z',
          near(g['x'], 14.0), json.dumps(g))
    check('...but says what height it will really fly at',
          any('click_height' in n and '1.5' in n for n in notes),
          json.dumps(notes))

    print('\ngoal tracking  ("sent" is the easy one; these are the rest)')
    flying = {'armed': True, 'offboard': True}
    sent_at = 1000.0

    def rec(goal, origin=None, **over):
        r = {'goal': goal, 'at': sent_at, 'what': 'goto', 'best': None,
             'best_at': sent_at, 'origin': origin or dict(live), 'state': 'sent'}
        r.update(over)
        return r

    goal = {'x': 14.0, 'y': -4.0, 'z': 1.5, 'yaw': 0.0}
    # Still 4 m out, armed and in Offboard: moving.
    pr = rc.track_goal(rec(goal), dict(live, x=10.0), flying, None, cfg, follow,
                       now=sent_at + 5)
    check('4 m out and flying reads as MOVING', pr['state'] == 'moving',
          pr['detail'][:60])
    check('...with the distance an operator watches', near(pr['remaining'], 4.0),
          str(pr['remaining']))
    check('...and a fraction covered', pr['frac'] is not None and pr['frac'] >= 0,
          str(pr['frac']))

    # Inside SUPER's own closeToGoal(0.1): arrived, and by the same test the
    # planner uses to decide it is done.
    pr = rc.track_goal(rec(goal), dict(live, x=14.0, y=-4.0, z=1.44), flying,
                       None, cfg, follow, now=sent_at + 40)
    check(f'within SUPER\'s own {rc.ARRIVE_M} m reads as ARRIVED',
          pr['state'] == 'arrived', pr['detail'][:70])
    check('...and says how far off and how long it took',
          'cm off' in pr['detail'] and 's after' in pr['detail'])
    pr = rc.track_goal(rec(goal), dict(live, x=14.0, y=-4.0, z=1.29), flying,
                       None, cfg, follow, now=sent_at + 40)
    check('...and 0.21 m away is NOT arrived', pr['state'] == 'moving',
          str(pr['remaining']))

    # THE ONE A SPINNER GETS WRONG: disarmed, so it will never move at all.
    for who, air in (('disarmed', {'armed': False, 'offboard': True}),
                     ('not in Offboard', {'armed': True, 'offboard': False}),
                     ('neither', {'armed': False, 'offboard': False})):
        pr = rc.track_goal(rec(goal), dict(live, x=10.0), air, None, cfg,
                           follow, now=sent_at + 30)
        check(f'{who}: reported as NOT FLYING, never as moving',
              pr['state'] == 'idle' and 'not move' in pr['detail'],
              pr['detail'][:72])

    # The other one: flying, but getting no closer.
    stuck = rec(goal)
    # best_at is set on the FIRST observation, not at send, so the clock starts
    # at t=1 here -- t=25 is unambiguously past STALL_AFTER_S.
    for t in (1, 5, 10, 25):
        pr = rc.track_goal(stuck, dict(live, x=10.0), flying, None, cfg,
                           follow, now=sent_at + t)
    check('flying but no closer for 20 s reads as STALLED',
          pr['state'] == 'stalled', pr['detail'][:70])
    # ...and real progress must reset that, or a slow flight reads as a stall.
    creep = rec(goal)
    for t, x in ((1, 10.0), (8, 11.0), (16, 12.0), (23, 13.0)):
        pr = rc.track_goal(creep, dict(live, x=x), flying, None, cfg, follow,
                           now=sent_at + t)
    check('...but steady progress is never called a stall',
          pr['state'] == 'moving', pr['detail'][:60])

    pr = rc.track_goal(rec(goal), dict(live, age=9.0), flying, None, cfg,
                       follow, now=sent_at + 5)
    check('stale odometry admits it cannot tell, rather than guessing',
          pr['state'] == 'unknown', pr['detail'][:60])
    check('no goal ever sent means no progress block',
          rc.track_goal({'goal': None}, live, flying, None, cfg, follow) is None)

    # SUPER's own answer beats our arithmetic: it is the post-snap point, and
    # the snap moves a goal by up to 3 m.
    snapped = {'ours': True, 'name': 'SNAPPED',
               'adopted': {'x': 12.0, 'y': -4.0, 'z': 1.5},
               'requested': {'x': 14.0, 'y': -4.0, 'z': 1.5}}
    pr = rc.track_goal(rec(goal), dict(live, x=12.0, y=-4.0, z=1.5), flying,
                       snapped, cfg, follow, now=sent_at + 20)
    check('arrival is judged against the point SUPER ADOPTED, not the one sent',
          pr['state'] == 'arrived' and pr['exact'] is True,
          f'adopted {pr["target"]}')
    dropped = {'ours': True, 'name': 'DROPPED',
               'adopted': {'x': 0.0, 'y': 0.0, 'z': 0.0},
               'requested': {'x': 14.0, 'y': -4.0, 'z': 1.5}}
    pr = rc.track_goal(rec(goal), live, flying, dropped, cfg, follow,
                       now=sent_at + 3)
    check('a goal SUPER dropped is reported as refused, not as moving',
          pr['state'] == 'rejected', pr['detail'][:70])

    # On the click path z is overwritten, so arrival must be measured there.
    pr = rc.track_goal(rec({'x': 14.0, 'y': -4.0, 'z': 2.6, 'yaw': 0.0}),
                       dict(live, x=14.0, y=-4.0, z=1.5), flying, None, cfg,
                       click, now=sent_at + 20)
    check('on the click path arrival uses the OVERRIDDEN z, not the sent one',
          pr['state'] == 'arrived' and near(pr['target']['z'], 1.5),
          f'target {pr["target"]}')
    check('...and admits the click path never confirmed it',
          'never reports what it adopted' in pr['detail'], pr['detail'][-70:])
    pr = rc.track_goal(rec(goal), dict(live, x=14.0, y=-4.0, z=1.5), flying,
                       None, cfg, follow, now=sent_at + 20)
    check('a follow goal with no status says so, rather than claiming SUPER '
          'confirmed it', 'did not confirm' in pr['detail'], pr['detail'][-55:])
    pr = rc.track_goal(rec(goal), dict(live, x=14.0, y=-4.0, z=1.5), flying,
                       {'ours': True, 'name': 'ACCEPTED',
                        'adopted': {'x': 14.0, 'y': -4.0, 'z': 1.5},
                        'requested': {'x': 14.0, 'y': -4.0, 'z': 1.5}},
                       cfg, follow, now=sent_at + 20)
    check('...and a confirmed one carries no caveat at all',
          'did not confirm' not in pr['detail'] and pr['exact'] is True,
          pr['detail'][-55:])

    print('\nthe PX4 echo  (the frame mix-up that flies it to the wrong place)')
    ned = rc.to_px4_ned(live)
    check('PX4 NED negates y and z and keeps x',
          near(ned['x'], 10.0) and near(ned['y'], 4.0) and near(ned['z'], -1.5),
          json.dumps(ned))
    check('to_px4_ned(None) is None, not a crash', rc.to_px4_ned(None) is None)

    print('\ntyped values  (the bounds the boxes advertise are the ones enforced)')
    for step, ok in ((rc.STEP_MIN, True), (rc.STEP_MAX, True),
                     (rc.STEP_MIN - 0.01, False), (rc.STEP_MAX + 0.01, False)):
        try:
            rc.plan_move('forward', step, live, cfg, follow)
            got = True
        except rc.RCError:
            got = False
        check(f'a {step} m step is {"accepted" if ok else "refused"}', got == ok)
    for yaw, ok in ((rc.YAW_MIN, True), (rc.YAW_MAX, True), (0, False),
                    (rc.YAW_MAX + 1, False)):
        try:
            rc.plan_move('yaw_left', yaw, live, cfg, follow)
            got = True
        except rc.RCError:
            got = False
        check(f'a {yaw} deg turn is {"accepted" if ok else "refused"}', got == ok)

    print('\nrefusals  (each of these is a button that would otherwise do nothing)')
    for name, args in (
            ('no odometry at all', ('forward', 1.0, None, cfg, follow)),
            ('stale odometry', ('forward', 1.0, dict(live, age=9.0), cfg, follow)),
            ('an unknown axis', ('sideways', 1.0, live, cfg, follow)),
            ('an absurd step', ('forward', 99.0, live, cfg, follow))):
        try:
            rc.plan_move(*args)
            check(f'{name} is refused', False)
        except rc.RCError as exc:
            check(f'{name} is refused', True, str(exc)[:64])

    inert = mkcfg(goal_yaw_en=False)
    for axis, why in (('up', 'click_height'), ('yaw_left', 'goal_yaw_en')):
        try:
            rc.plan_move(axis, 1.0 if axis == 'up' else 45, live, inert, click)
            check(f'{axis} is refused when the config would ignore it', False,
                  'a button that silently does nothing reads as a broken aircraft')
        except rc.RCError as exc:
            check(f'{axis} is refused when the config would ignore it',
                  why in str(exc), str(exc)[:70])

    print('\ncontention  (the DTP follower and this page must not both drive)')
    busy = rc.choose_path(cfg, follow_subs=1, click_subs=1, follow_pubs_other=1)
    check('another publisher on the follow topic is detected',
          bool(busy['contended']), str(busy['contended'])[:70])
    check('...and being alone on it is not contention',
          rc.choose_path(cfg, follow_subs=1, click_subs=1,
                         follow_pubs_other=0)['contended'] is None,
          'counted by node name in goal_graph(), not total-minus-one')
    for fn, args in ((rc.plan_move, ('forward', 1.0, live, cfg, busy)),
                     (rc.plan_altitude, (2.0, live, cfg, busy)),
                     (rc.plan_goto, (14.0, -4.0, 2.0, '', live, cfg, busy))):
        try:
            fn(*args)
            check(f'{fn.__name__} is refused while contended', False)
        except rc.RCError as exc:
            check(f'{fn.__name__} is refused while contended',
                  'other publisher' in str(exc), str(exc)[:50])

    print('\nthe installed config  (whatever it happens to say today)')
    real = rc.read_config(force=True)
    check('a config was found', real['path'] is not None, str(real['path']))
    check('...and parsed', real['error'] is None, str(real['error']))
    real_path = rc.choose_path(real)
    check('capabilities are reported for both controls',
          set(rc.capabilities(real, real_path)) == {'altitude', 'yaw'},
          json.dumps({k: v[0] for k, v in
                      rc.capabilities(real, real_path).items()}))

    # -- the gate --------------------------------------------------------
    print('\nthe gate  (server on its own port, --no-ros: it CANNOT send a goal)')
    with tempfile.TemporaryDirectory() as home:
        import admin
        os.makedirs(os.path.join(home, '.cache', 'super-gcs'))
        pw = 'test-only-password'
        with open(os.path.join(home, '.cache', 'super-gcs', 'admin'), 'w') as f:
            f.write(admin.encode(pw, iterations=1000) + '\n')
        port = free_port()
        proc = subprocess.Popen(
            [sys.executable, os.path.join(HERE, 'server.py'), '--port', str(port),
             '--bind', '127.0.0.1', '--no-auth', '--no-ros'],
            env=dict(os.environ, HOME=home),
            cwd=os.path.dirname(os.path.dirname(HERE)),
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
                sys.exit(f'the test server never came up on {port}')

            code, body = call(base, '/api/rc')
            check('GET /api/rc is refused while locked', code == 403, str(code))
            check('...as an admin problem', body.get('admin_required') is True,
                  json.dumps(body))
            for ep in ('/api/rc/move?axis=forward&step=1',
                       '/api/rc/altitude?z=2',
                       '/api/rc/goto?x=1&y=2&z=1.5'):
                code, _ = call(base, ep, 'POST')
                check(f'POST {ep.split("?")[0]} is refused while locked',
                      code == 403, str(code))
            _code, st = call(base, '/api/state')
            check('a locked client gets no RC data at all', 'rc' not in st,
                  'the panel is absent because the data is')

            code, body = call(base, '/api/admin/login', 'POST', body={'password': pw})
            sess = body.get('session')
            check('unlocking works', code == 200 and bool(sess), json.dumps(body))

            code, st = call(base, '/api/rc', admin=sess)
            check('GET /api/rc answers an unlocked client', code == 200, str(code))
            for k in ('config', 'capabilities', 'limits', 'ready', 'available',
                      'path'):
                check(f'...and carries {k}', k in st, json.dumps(list(st))[:70])
            for k in ('step_min', 'step_max', 'yaw_min', 'yaw_max', 'goto_max'):
                check(f'...and the typed boxes get {k} from the server',
                      k in (st.get('limits') or {}),
                      'a bound duplicated in app.js is a second bound')
            check('the readiness list covers all four failures',
                  {r['key'] for r in st.get('ready') or []} ==
                  {'odom', 'planner', 'armed', 'offboard'},
                  json.dumps([r['key'] for r in st.get('ready') or []]))

            for ep in ('/api/rc/move?axis=forward&step=1',
                       '/api/rc/goto?x=1&y=2&z=1.5'):
                code, body = call(base, ep, 'POST', admin=sess)
                check(f'{ep.split("?")[0]} with no telemetry is refused, not '
                      'silently dropped', code == 409, json.dumps(body))
            code, _ = call(base, '/api/rc/fly?axis=forward', 'POST', admin=sess)
            check('an invented RC endpoint is a 404', code == 404, str(code))
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                proc.kill()

    print()
    print(f'{len(fails)} FAILED: ' + ', '.join(fails) if fails else 'all passed')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
