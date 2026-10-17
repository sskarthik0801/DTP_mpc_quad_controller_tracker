#!/usr/bin/env python3
"""Fetch a live /api/state and run the front-end tests against it.

    python3 scripts/gcs/test_render.py [http://127.0.0.1:8080/?k=<token>]

With no argument it uses 127.0.0.1:8080 and the token in ~/.cache/super-gcs.
Needs `gjs` (present on Ubuntu desktops and on this Jetson) -- there is no
browser here, and this is the only thing that executes app.js's render path.
"""
import json
import os
import subprocess
import sys
import tempfile
import urllib.request
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))


def fetch(base, token, path, admin=None):
    req = urllib.request.Request(
        base + path, headers={'X-GCS-Token': token,
                              **({'X-GCS-Admin': admin} if admin else {})})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


def admin_login(base, token, password):
    """An admin session, so the UNLOCKED camera shape gets exercised too.

    Without one this tests the locked shape, which is the default a browser
    lands on and the one that matters most -- but it means the camera panel's
    own render path is never run. Set GCS_ADMIN_PASSWORD to cover both.
    """
    req = urllib.request.Request(
        base + '/api/admin/login', method='POST',
        data=json.dumps({'password': password}).encode(),
        headers={'X-GCS-Token': token, 'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=15) as r:
        body = json.loads(r.read().decode())
    if not body.get('ok'):
        print(f'admin login refused ({body.get("message")}) -- '
              'testing the locked shape only')
    return body.get('session')


def main():
    raw = sys.argv[1] if len(sys.argv) > 1 else 'http://127.0.0.1:8080/'
    u = urlparse(raw)
    token = (parse_qs(u.query).get('k') or [''])[0]
    if not token:
        try:
            with open(os.path.expanduser('~/.cache/super-gcs/token')) as f:
                token = f.read().strip()
        except OSError:
            sys.exit('no token: pass the full GCS URL including ?k=<token>')
    base = f'{u.scheme}://{u.hostname}:{u.port or 8080}'

    if not any(os.access(os.path.join(d, 'gjs'), os.X_OK)
               for d in os.environ.get('PATH', '').split(':')):
        sys.exit('gjs not found -- install it (apt install gjs) or skip this test')

    admin = None
    if os.environ.get('GCS_ADMIN_PASSWORD'):
        try:
            admin = admin_login(base, token, os.environ['GCS_ADMIN_PASSWORD'])
        except Exception as exc:
            print(f'admin login failed ({exc}) -- testing the locked shape only')

    try:
        config = fetch(base, token, '/api/config', admin)
        # /fmu/out/vehicle_status_v1 is 2 Hz and the high-rate subscriptions are
        # created on demand, so a snapshot taken the instant the GCS starts is
        # legitimately empty. Give it a few seconds to fill before judging it.
        import time
        for _ in range(10):
            state = fetch(base, token, '/api/state', admin)
            if (state.get('ros', {}).get('aircraft') or {}).get('fresh'):
                break
            time.sleep(1)
    except Exception as exc:
        sys.exit(f'cannot reach the GCS at {base}: {exc}\n'
                 'Start it with: bash scripts/gcs.sh')

    # The RC card renders from its OWN endpoint, so a /api/state snapshot
    # cannot exercise it. Fetch it too when there is an admin session; without
    # one the js falls back to asserting the locked shape.
    rc_state = None
    if admin:
        try:
            rc_state = fetch(base, token, '/api/rc', admin)
        except Exception as exc:
            print(f'/api/rc unavailable ({exc}) -- skipping the RC render half')

    with tempfile.TemporaryDirectory() as tmp:
        if rc_state is not None:
            with open(os.path.join(tmp, 'rc.json'), 'w') as f:
                json.dump(rc_state, f)
        for name, obj in (('state.json', state), ('config.json', config)):
            with open(os.path.join(tmp, name), 'w') as f:
                json.dump(obj, f)
        rc = 0
        # Two passes over the same snapshot: the normal render, and the auth
        # path (no ?k= in the URL, 401 from everything, then a paste). Both are
        # run even if the first fails -- knowing only that "the render test
        # failed" hides whether the gate still works.
        runs = [('test_render.js', 'state.json'), ('test_gate.js', 'state.json')]
        # Then the follower and Stack rows in the shapes this box is probably
        # NOT in right now. Whether a follower happens to be running when the
        # test is run should not decide which half of its render path gets
        # tested -- the stopped half is where Start LIVE lives.
        for label, variant in follower_variants(state):
            name = f'state-{label}.json'
            with open(os.path.join(tmp, name), 'w') as f:
                json.dump(variant, f)
            runs.append(('test_render.js', name))
        for script, state_file in runs:
            if state_file != 'state.json':
                print(f'--- synthetic: {state_file} ---')
            argv = ['gjs', os.path.join(HERE, script),
                    os.path.join(tmp, state_file),
                    os.path.join(tmp, 'config.json'),
                    os.path.join(HERE, 'static')]
            if rc_state is not None:
                argv.append(os.path.join(tmp, 'rc.json'))
            rc |= subprocess.call(argv)
            print()
    sys.exit(rc)


def follower_variants(state):
    """The live snapshot with the Stack and DTP rows swapped for other shapes.

    Only those two rows change, and only in fields the server really sends, so
    every other assertion in test_render.js still holds against the variant.
    A GCS older than the follower row sends none of this and gets no variants.
    """
    import copy
    rows = {r['key']: r for r in state.get('procs') or []}
    if 'dtp' not in rows or 'stack' not in rows:
        return []
    unlocked = bool((state.get('admin') or {}).get('unlocked'))
    out = []

    stopped = copy.deepcopy(state)
    for r in stopped['procs']:
        if r['key'] in ('stack', 'dtp'):
            r.update(state='stopped', managed=False, pid=None, procs=0,
                     rss_mb=0.0, cpu_pct=0.0, uptime_s=None, opts={})
        if r['key'] == 'stack':
            r.update(config=None, config_error=None)
        if r['key'] == 'dtp':
            r.update(mode=None)
            if unlocked:
                r.update(stop_note=None, blocked={
                    'observe': None,
                    'live': 'fsm_node is running click_smooth_ros2.yaml (synthetic)'})
    out.append(('stopped', stopped))

    running = copy.deepcopy(state)
    for r in running['procs']:
        if r['key'] in ('stack', 'dtp'):
            r.update(state='running', managed=True, pid=4242, procs=3,
                     rss_mb=100.0, cpu_pct=10.0, uptime_s=60)
        if r['key'] == 'stack':
            r.update(config='follow_test_ros2.yaml', config_error=None,
                     opts={'config': 'follow'})
        if r['key'] == 'dtp':
            r.update(mode='live', opts={'mode': 'live'})
            r.pop('blocked', None)
            if unlocked:
                r.update(stop_note='The aircraft is ARMED (synthetic).')
    out.append(('running', running))
    return out


if __name__ == '__main__':
    main()
