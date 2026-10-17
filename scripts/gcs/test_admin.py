#!/usr/bin/env python3
"""Test that the camera gate is enforced by the SERVER, not by the page.

Both doors to the same data: watching the feed (/api/camera/*) and recording it
into a bag (/api/bag/*).  A gate on only the first one is a hidden panel with
an open back door -- `ros2 bag record` is a subscriber like any other.

    python3 scripts/gcs/test_admin.py                       # locked half only
    GCS_ADMIN_PASSWORD='…' python3 scripts/gcs/test_admin.py # both halves
    python3 scripts/gcs/test_admin.py http://127.0.0.1:8099/?k=<token>

Needs a running GCS.  With no password it still runs the half that matters --
that a client holding only the dashboard token is refused everywhere and is
never sent any camera data at all.  That is the assertion a hidden panel cannot
make for itself: `curl` does not run app.js.

The rate-limit case runs LAST because it deliberately trips the server's
cooldown, which lasts ~30 s for everybody.
"""
import json
import os
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import urlparse, parse_qs

fails = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  -- {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def call(base, token, path, admin=None, body=None, method=None):
    """(status, parsed-or-raw). Never raises on an HTTP error status."""
    req = urllib.request.Request(
        base + path, method=method or ('POST' if body is not None else 'GET'),
        data=json.dumps(body).encode() if body is not None else None,
        headers={'X-GCS-Token': token,
                 **({'X-GCS-Admin': admin} if admin else {}),
                 **({'Content-Type': 'application/json'} if body is not None else {})})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            raw, code, ctype = r.read(), r.status, r.headers.get('Content-Type', '')
    except urllib.error.HTTPError as e:
        raw, code, ctype = e.read(), e.code, e.headers.get('Content-Type', '')
    if 'json' in ctype:
        try:
            return code, json.loads(raw or b'{}')
        except ValueError:
            pass
    return code, raw


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
    try:
        call(base, token, '/api/state')
    except Exception as exc:
        sys.exit(f'cannot reach the GCS at {base}: {exc}\n'
                 'Start it with: bash scripts/gcs.sh')

    print('locked  (dashboard token only -- no admin session)')
    code, st = call(base, token, '/api/state')
    check('/api/state answers', code == 200, str(code))
    check('...and carries NO camera data', 'camera' not in st,
          'a gate the page enforces is not a gate: curl does not run app.js')
    check('...and says it is locked', st.get('admin', {}).get('unlocked') is False,
          str(st.get('admin')))
    check('...and hides the camera process row',
          not any(p['key'] == 'camera' for p in st.get('procs', [])),
          str([p['key'] for p in st.get('procs', [])]))
    code, cfg = call(base, token, '/api/config')
    check('/api/config hides the camera too, so no log tab appears',
          'camera' not in (cfg.get('procs') or {}), str(list(cfg.get('procs') or {})))

    # The DTP follower opens the camera and, live, commands the aircraft: its
    # row is shown to everyone, but not its log, its reasons or its controls.
    row = next((p for p in st.get('procs', []) if p['key'] == 'dtp'), None)
    if row is not None:
        check('the follower row is marked locked, with no reasons',
              row.get('locked') is True and 'blocked' not in row, str(sorted(row)))
        check('/api/config hides the follower log tab',
              'dtp' not in (cfg.get('procs') or {}))

    # Every POST here is refused before it is looked at, so none of them can
    # start or stop anything -- which is why they are safe against a live GCS.
    for path, method in (('/api/camera/stream', 'GET'),
                         ('/api/camera/snapshot', 'GET'),
                         ('/api/logs/camera', 'GET'),
                         ('/api/camera/source?topic=/x', 'POST'),
                         ('/api/proc/camera/start', 'POST'),
                         ('/api/proc/camera/stop', 'POST'),
                         ('/api/logs/dtp', 'GET'),
                         ('/api/proc/dtp/start?mode=observe', 'POST'),
                         ('/api/proc/dtp/stop', 'POST')):
        code, body = call(base, token, path, method=method)
        check(f'{method} {path.split("?")[0]} is refused', code == 403, str(code))
        if code == 403:
            check('  ...as an admin problem, not a broken endpoint',
                  isinstance(body, dict) and body.get('admin_required') is True,
                  str(body))

    # The bag recorder is the other way to reach a camera: `ros2 bag record`
    # is a subscriber like any other, so the same gate has to hold there or the
    # redaction above is just a hidden panel with an open back door.
    code, tp = call(base, token, '/api/bag/topics')
    check('/api/bag/topics answers a locked client', code == 200, str(code))
    imgs = [t['topic'] for t in (tp.get('topics') or [])
            if t.get('kind') == 'image']
    check('...with no image topic in the list', not imgs, str(imgs))
    check('...and says an exclusion is in force for `record all`',
          bool(tp.get('excluded')), str(tp.get('excluded')))
    code, body = call(base, token, '/api/bag/start',
                      body={'all': False,
                            'topics': ['/camera/color/image_raw/compressed']})
    check('recording an image topic by name is refused', code == 403, str(code))
    check('  ...as an admin problem, not a bad request',
          isinstance(body, dict) and body.get('admin_required') is True, str(body))

    code, body = call(base, token, '/api/admin/login', body={'password': 'not-it'})
    check('a wrong password is rejected',
          code == 200 and body.get('ok') is False, f'{code} {body}')
    # 200 and not 401 on purpose: app.js treats a 401 as "the DASHBOARD token
    # has gone bad" and throws it away. Two credentials, two failure paths.
    check('...with 200, so the page does not discard the dashboard token',
          code == 200, str(code))

    password = os.environ.get('GCS_ADMIN_PASSWORD')
    if not password:
        print('\nunlocked  -- SKIPPED (set GCS_ADMIN_PASSWORD to run this half)')
    else:
        print('\nunlocked  (with an admin session)')
        code, body = call(base, token, '/api/admin/login', body={'password': password})
        session = body.get('session') if isinstance(body, dict) else None
        check('the right password returns a session', bool(session), str(body))
        if session:
            code, st = call(base, token, '/api/state', admin=session)
            check('/api/state now carries camera data', 'camera' in st)
            check('...and the camera process row is back',
                  any(p['key'] == 'camera' for p in st.get('procs', [])))
            code, shot = call(base, token, '/api/camera/snapshot', admin=session)
            # 503 is a legitimate pass here: it means the gate opened and there
            # simply is no camera publishing right now.
            check('the snapshot is reachable', code in (200, 503), str(code))
            if code == 200:
                check('...and it really is a JPEG', shot[:3] == b'\xff\xd8\xff',
                      shot[:8].hex())
            code, _ = call(base, token, '/api/admin/logout', admin=session,
                           method='POST')
            check('logout is accepted', code == 200, str(code))
            code, tp = call(base, token, '/api/bag/topics', admin=session)
            check('an unlocked client may see image topics for recording',
                  code == 200 and tp.get('excluded') is None, str(code))
            code, _ = call(base, token, '/api/camera/snapshot', admin=session)
            check('the session is dead immediately after logout', code == 403,
                  str(code))

    print('\nbrute force  (this trips the server cooldown for ~30 s)')
    seen = []
    for _ in range(6):
        _code, body = call(base, token, '/api/admin/login',
                           body={'password': 'still-not-it'})
        seen.append(body.get('message', '') if isinstance(body, dict) else '')
        time.sleep(0.1)
    check('repeated wrong passwords end in a cooldown, not endless retries',
          any('wait' in m for m in seen), ' | '.join(seen[-2:]))

    print('\n' + (f'{len(fails)} FAILED: ' + ', '.join(fails) if fails else 'all passed'))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())
