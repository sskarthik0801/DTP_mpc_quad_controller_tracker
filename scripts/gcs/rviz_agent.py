#!/usr/bin/env python3
"""Laptop-side helper that opens RViz when you press the button in the GCS.

Run this ON THE LAPTOP (scripts/gcs_laptop.sh does it for you).

It is a pure CLIENT: it polls the Jetson for a pending command and reports
whether RViz is up.  It listens on nothing.

That is deliberate.  The obvious design -- a small HTTP server on
127.0.0.1 that the page calls -- does not survive contact with browsers: a
page served from 192.168.x.x calling 127.0.0.1 is a private-network request,
and Chrome blocks it unless the preflight carries
Access-Control-Allow-Private-Network, with more restrictions planned.  It also
could never work from a phone, where 127.0.0.1 is the phone.  Polling outward
has no cross-origin story at all, so there is nothing for a browser to block.

RViz itself is started by running `bash scripts/laptop_viewer.sh` UNCHANGED,
and stopped by process group.  That script carries the verified `env -i`
allow-list for the snap-glibc bug (rviz2: undefined symbol
__libc_pthread_init, which bit twice) and the /viz/drone_radius marker
publisher with its own EXIT trap.  None of it is reimplemented here --
reimplementing it is how that bug came back the second time.
"""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import urlparse, parse_qs

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
POLL_S = 1.0
STATE = {'proc': None, 'pgid': None, 'started': None, 'note': None}


def running():
    p = STATE['proc']
    return p is not None and p.poll() is None


def reap():
    """Notice RViz dying on its own and say so.

    Without this the dashboard just flips back to "not running" and the button
    looks like it did nothing -- which is indistinguishable from the bug this
    replaced. The usual cause is a real one worth naming: no DISPLAY, or the
    snap-glibc crash laptop_viewer.sh guards against.
    """
    p = STATE['proc']
    if p is None or p.poll() is None:
        return
    rc, up = p.returncode, time.time() - (STATE['started'] or time.time())
    STATE['proc'] = None
    STATE['note'] = (f'RViz exited immediately (code {rc}) -- see this '
                     f'helper\'s terminal for why'
                     if up < 20 else f'RViz closed (code {rc}) after {int(up)}s')
    print(STATE['note'], flush=True)


def start(env_extra):
    if running():
        return 'already running'
    argv = ['bash', os.path.join('scripts', 'laptop_viewer.sh')]
    try:
        proc = subprocess.Popen(argv, cwd=REPO,
                                env=dict(os.environ, **env_extra),
                                start_new_session=True)
    except OSError as exc:
        return f'could not start: {exc}'
    STATE.update(proc=proc, pgid=os.getpgid(proc.pid), started=time.time())
    return f'started (pid {proc.pid})'


def stop():
    if not running():
        STATE['proc'] = None
        return 'not running'
    pgid = STATE['pgid']
    for sig, grace in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        deadline = time.time() + grace
        while time.time() < deadline:
            if STATE['proc'].poll() is not None:
                STATE['proc'] = None
                return 'stopped'
            time.sleep(0.2)
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    STATE['proc'] = None
    return 'killed'


def hello(url, token, timeout=4.0):
    """Report our state, collect any pending command."""
    body = json.dumps({
        'host': socket.gethostname(),
        'running': running(),
        'pid': STATE['proc'].pid if running() else None,
        'uptime_s': int(time.time() - STATE['started']) if running() else None,
        'note': STATE['note'],
    }).encode()
    req = urllib.request.Request(url, data=body, method='POST',
                                 headers={'Content-Type': 'application/json',
                                          'X-GCS-Token': token})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def split_url(raw):
    """Accept the whole URL the Jetson printed, token and all."""
    u = urlparse(raw if '//' in raw else 'http://' + raw)
    if not u.hostname:
        sys.exit(f'ERROR: cannot parse {raw!r} as a URL')
    token = (parse_qs(u.query).get('k') or [''])[0]
    base = f'{u.scheme or "http"}://{u.hostname}:{u.port or 8080}'
    return base, token, u.hostname


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('url', help='the GCS URL the Jetson printed, including ?k=<token>')
    ap.add_argument('--token', default=None, help='override the token from the URL')
    ap.add_argument('--jetson-ip', default=None,
                    help='passed to laptop_viewer.sh as JETSON_IP (default: from url)')
    args = ap.parse_args()

    base, token, host = split_url(args.url)
    token = args.token or token
    if not token:
        sys.exit('ERROR: no token. Paste the full URL the Jetson printed, '
                 'the one ending in ?k=<token>.')

    viewer = os.path.join(REPO, 'scripts', 'laptop_viewer.sh')
    if not os.path.isfile(viewer):
        sys.exit(f'ERROR: {viewer} not found -- run this from a clone of the repo.')
    if not os.path.isdir('/opt/ros/humble'):
        print('WARNING: /opt/ros/humble missing -- run scripts/laptop_setup.sh '
              'first, or RViz will not start.', file=sys.stderr)

    env_extra = {'JETSON_IP': args.jetson_ip or host}
    endpoint = base + '/api/rviz/hello'
    print(f'RViz helper: polling {base} every {POLL_S:.0f}s '
          f'(JETSON_IP={env_extra["JETSON_IP"]})', flush=True)

    warned = False
    try:
        while True:
            reap()
            try:
                reply = hello(endpoint, token)
                if warned:
                    print('reconnected to the GCS', flush=True)
                    warned = False
                cmd = reply.get('command')
                if cmd == 'start':
                    STATE['note'] = start(env_extra)
                    print(f'start -> {STATE["note"]}', flush=True)
                elif cmd == 'stop':
                    STATE['note'] = stop()
                    print(f'stop -> {STATE["note"]}', flush=True)
            except urllib.error.HTTPError as exc:
                if exc.code == 401:
                    sys.exit('ERROR: the GCS rejected the token. Re-copy the '
                             'URL the Jetson printed.')
                if not warned:
                    print(f'GCS returned HTTP {exc.code}; retrying...', flush=True)
                    warned = True
            except (urllib.error.URLError, socket.timeout, OSError) as exc:
                if not warned:
                    print(f'cannot reach the GCS at {base} ({exc}); retrying...',
                          flush=True)
                    warned = True
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        print()
    finally:
        if running():
            print('stopping RViz...', flush=True)
            stop()


if __name__ == '__main__':
    main()
