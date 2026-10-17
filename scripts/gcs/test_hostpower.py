#!/usr/bin/env python3
"""Test the gates on powering the JETSON off, without ever powering it off.

    python3 scripts/gcs/test_hostpower.py

It starts its OWN server on a free port with GCS_POWER_DRY_RUN=1, --no-ros and
--no-auth, so a gate that turned out to be broken echoes a line instead of
shutting a drone's computer down mid-test.  Nothing here touches the GCS you
have running, and it starts and stops no ROS nodes: with --no-ros the
supervisor's shutdown sweep is the fixed one that adopts nothing (CLAUDE.md,
2026-09-10), and it has nothing of its own to stop anyway.

The three things asserted, in the order they would fail in real life:
  * the endpoint refuses a POST with no ?confirm=<mode>, or the wrong one --
    the page asks twice, but the page is not the gate; curl does not run app.js
  * an unknown mode is a 404, not a shrug
  * a confirmed POST schedules exactly one action, and a second is refused
"""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
fails = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  -- {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def call(base, path, method='GET'):
    req = urllib.request.Request(base + path, method=method)
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
    # Unit half first: no server, no network, just the module.
    sys.path.insert(0, HERE)
    os.environ['GCS_POWER_DRY_RUN'] = '1'
    import hostpower

    print('the flight interlock  (the half that would hurt somebody)')
    fb = hostpower.flight_blocked
    check('ARMED blocks it', bool(fb({'fresh': True, 'armed': True})))
    check('IN AIR blocks it',
          bool(fb({'fresh': True, 'armed': False, 'landed': False})))
    check('...and both say which', 'ARMED' in fb({'fresh': True, 'armed': True})
          and 'IN AIR' in fb({'fresh': True, 'armed': False, 'landed': False}))
    check('armed-then-link-lost still blocks',
          bool(fb({'fresh': False, 'armed': True})),
          'a vehicle that stopped talking while armed may be in the air')
    check('disarmed and landed does not block',
          fb({'fresh': True, 'armed': False, 'landed': True}) is None)
    # The deliberate difference from reboot_blocked(): no FC at all is the
    # ordinary end-of-day case, and refusing then only teaches people to ssh in
    # and sudo, which skips every gate here.
    check('an FC that was never seen does not block the bench case',
          fb({'fresh': False, 'mode': None, 'armed': None}) is None)
    check('no telemetry at all does not block', fb(None) is None)

    print('\nmodule')
    check('every mode has a command', set(hostpower.CMD) == set(hostpower.MODES))
    check('an unknown mode is refused, not run',
          hostpower.schedule('halt')[0] is False)
    check('nothing is scheduled by a refusal',
          hostpower.pending()['mode'] is None)
    ok, msg = hostpower.schedule('reboot', log=lambda _l: None, delay=0.0)
    check('a known mode schedules', ok, msg)
    check('a second one is refused while the first is pending',
          hostpower.schedule('poweroff')[0] is False)
    time.sleep(0.6)
    st = hostpower.status()
    check('dry run is reported, not hidden', st['dry_run'] is True)
    # The successful path deliberately leaves `pending` set: the box is on its
    # way down, and the page must keep saying so rather than reporting the link
    # it is about to lose as a fault.
    check('a successful command leaves the action pending',
          st['pending'] == 'reboot' and st['error'] is None, json.dumps(st))

    # A shutdown that systemd accepted and then did not perform must not leave
    # the page saying "going down" for ever with the buttons dead.
    hostpower.STUCK_AFTER = 0.2
    time.sleep(0.3)
    st = hostpower.status()
    check('a shutdown that never happened un-sticks itself',
          st['pending'] is None and 'still running' in (st['error'] or ''),
          json.dumps(st))
    hostpower.STUCK_AFTER = 120.0
    hostpower._STATE.update(mode=None, at=0.0, error=None)

    port = free_port()
    env = dict(os.environ, GCS_POWER_DRY_RUN='1')
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
            sys.exit(f'the test server never came up on {port}')

        print('\nserver')
        code, st = call(base, '/api/state')
        p = st.get('power')
        check('/api/state carries the power block', code == 200 and p is not None,
              str(code))
        check('...and says whether it could actually do it',
              bool(p) and 'available' in p and 'blocked' in p)
        check('...and reports dry run', bool(p) and p.get('dry_run') is True)

        code, body = call(base, '/api/host/poweroff', 'POST')
        check('a POST with no confirm is refused', code == 409, str(code))
        check('...and says what is missing',
              'confirm=poweroff' in (body.get('message') or ''),
              json.dumps(body))
        code, _ = call(base, '/api/host/poweroff?confirm=yes', 'POST')
        check('a POST with the WRONG confirm is refused', code == 409, str(code))
        code, _ = call(base, '/api/host/poweroff?confirm=reboot', 'POST')
        check("...including another mode's confirm", code == 409, str(code))
        code, _ = call(base, '/api/host/halt?confirm=halt', 'POST')
        check('an unknown mode is a 404', code == 404, str(code))

        _code, st = call(base, '/api/state')
        check('none of that scheduled anything',
              st['power']['pending'] is None, json.dumps(st['power']))

        code, body = call(base, '/api/host/reboot?confirm=reboot', 'POST')
        check('a confirmed POST is accepted', code == 200, json.dumps(body))
        _code, st = call(base, '/api/state')
        check('...and the state says it is going down',
              st['power']['pending'] == 'reboot', json.dumps(st['power']))
        code, body = call(base, '/api/host/poweroff?confirm=poweroff', 'POST')
        check('a second action is refused while one is pending', code == 409,
              json.dumps(body))
        time.sleep(2.5)
        _code, st = call(base, '/api/state')
        check('the dry-run command reported no error',
              st['power']['error'] is None, json.dumps(st['power']))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()

    print()
    if fails:
        print(f'{len(fails)} FAILED: ' + ', '.join(fails))
    else:
        print('all passed')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
