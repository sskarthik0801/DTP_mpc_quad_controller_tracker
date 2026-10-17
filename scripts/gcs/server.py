#!/usr/bin/env python3
"""Web GCS for the SUPER human-following drone -- stdlib only.

Serves a browser control panel on the Jetson so a laptop (or a phone) can bring
the stack up, stop it, and watch battery / flight mode / estimator health with
nothing installed on the laptop but a browser.  Unlike the DDS path used by
laptop_viewer.sh this works over Tailscale as well as same-L2 Wi-Fi.

Start it with `bash scripts/gcs.sh` so the ROS workspaces are sourced first;
children inherit that environment, so nothing here re-sources anything.

No web framework is installed on this board and there is no internet at a
flying field, so this is http.server + vanilla JS, no build step, no CDN.

The GUI starts and stops SOFTWARE only.  It cannot arm, change flight mode, or
write a parameter -- those stay on the RC and scripts/px4/offboard_engage.py,
where the pilot is looking at the aircraft.

Two endpoints reach past that line, both by request and both interlocked:
/api/check/faults_clear archives the FC's crash dumps before deleting them, and
/api/fc/reboot resets the flight controller.  The reboot is refused unless live
telemetry says the vehicle is disarmed and on the ground -- see reboot_blocked().

/api/host/poweroff and /api/host/reboot stop the JETSON ITSELF, which is the
one action here that ends the flight stack, this dashboard and the odometry
EKF2 is holding position on, all at once.  The page asks twice, the POST must
carry ?confirm=<mode>, and host_power_blocked() refuses while the FC reports
ARMED or IN AIR -- see hostpower.py for why that interlock is deliberately not
the same rule as reboot_blocked()'s.

/api/rc/* is the remote control: every button, and the coordinate bar, publishes
ONE goal -- the same message RViz's 2D Goal Pose tool sends -- and SUPER plans
the path.  Which of SUPER's two goal inputs it goes to is decided from the ROS
graph, not from a config file: see rc.choose_path().  The GUI
still cannot arm, change mode or write a parameter, and there is deliberately
no stop button -- SUPER's FSM has no stop input, so the abort is the RC.  Like
the camera it sits behind Admin control: "may watch telemetry" and "may move
the aircraft" are not the same permission.  See rc.py.

/api/camera/stream is an MJPEG feed of whatever camera topic is live, so the
Gemini can be watched from the same page over Tailscale.  It only ever READS a
ROS topic; starting the camera is an ordinary /api/proc/camera/start, and it is
refused when something already owns the USB device -- see camera_blocked().

/api/bag/* records ROS bags into <repo>/bags/ (gitignored) -- all topics or a
chosen set.  It is `ros2 bag record` in a process group we own, stopped with
SIGINT so rosbag2 gets to write the metadata that makes the bag playable.

/api/proc/dtp/start?mode=observe|live starts the DTP follower on top of the
running stack (scripts/dtp_follower.sh).  It is behind Admin control -- a live
follower moves the aircraft and every follower opens the camera -- and refused
by dtp_blocked() unless the stack is up, the camera is free and the FC is not
armed; a live start also needs ?confirm=live, a fresh link saying disarmed,
fsm_node on follow_test_ros2.yaml and odometry at rate.  See dtp.py.  Its row
is shown to everyone, because whether a follower is commanding the aircraft is
not something a locked dashboard should hide; only its controls, its log and
its reasons (which name the camera) are withheld.

Everything camera-shaped sits behind a SECOND password (Admin control), because
"may fly this aircraft" and "may switch on a camera and watch it" are not the
same permission.  While there is no admin session, the camera endpoints refuse
and /api/state and /api/config do not mention the camera at all -- see admin.py
and redact().
"""
import argparse
import errno
import json
import math
import os
import re
import secrets
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
STATIC = os.path.join(HERE, 'static')
sys.path.insert(0, HERE)

import admin                      # noqa: E402
import bagger                     # noqa: E402
import flightday
import camera                     # noqa: E402
import dtp                        # noqa: E402
import hostpower                  # noqa: E402
import rc                         # noqa: E402
import supervisor as sup          # noqa: E402
import sysinfo                    # noqa: E402

TOKEN_FILE = os.path.expanduser('~/.cache/super-gcs/token')
MIME = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
        '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml',
        '.ico': 'image/x-icon', '.woff2': 'font/woff2'}

STATE = {}          # filled in by main()
SHUTDOWN = threading.Event()      # set on SIGINT/SIGTERM; ends open MJPEG streams

# -- camera feed -----------------------------------------------------------
# A viewer is a thread parked on a socket, and each one keeps the camera
# subscription alive, so this is bounded: three tabs at 720p/30 is already
# ~120 Mbit/s of field Wi-Fi, and the fourth would only make the first three
# stutter.  DEFAULT_FPS decimates for the NETWORK, not for the Jetson -- the
# frames arrive either way, this only decides how many are forwarded.
BOUNDARY = 'superframe'
MAX_STREAMS = 3
DEFAULT_FPS = 10.0
FIRST_FRAME_S = 8.0       # driver start + DDS matching, worst case
STREAM_IDLE_S = 10.0      # no frame for this long -> close, let the <img> retry
STREAMS = {'n': 0}
STREAM_LOCK = threading.Lock()

# Process rows whose controls and log need Admin control.  The camera's for the
# reason the camera gate exists; the follower's because a live one moves the
# aircraft (the RC card's permission), every one opens the camera, and its log
# prints the camera driver's identity checks.  Claude's because a session there
# is a shell on this box.
RESTRICTED_PROCS = ('camera', 'dtp', 'claude')
RESTRICTED_LOGS = RESTRICTED_PROCS

# Admin sessions for the camera gate.  ADMIN_QS exists for the same reason the
# token has one: an <img> tag cannot send a header, so the MJPEG stream has to
# carry its credential in the URL.
ADMIN_HEADER = 'X-GCS-Admin'
ADMIN_QS = 'a'
SESSIONS = admin.Sessions()

# The laptop-side RViz helper is a polling client, not a server: it POSTs here
# to say what it is doing and to collect a pending command.  See the long note
# in rviz_agent.py for why it is not the other way round (browsers block a page
# on a private address from calling 127.0.0.1, and a phone has no laptop
# loopback at all).
# The last goal this dashboard sent, so the panel can show what it asked for
# next to where the aircraft actually is.  Not authoritative: SUPER snaps a
# goal to the nearest free cell up to 3 m away, and may drop it entirely.
RC_LAST = {'goal': None, 'at': 0.0, 'what': None, 'notes': [],
           'origin': None, 'best': None, 'best_at': 0.0, 'state': None,
           'announced': None}
RC_LOCK = threading.Lock()
RC_GOAL_TTL = 120.0        # after this, a lateral move uses the live altitude

RVIZ = {'last_seen': 0.0, 'running': False, 'pid': None, 'host': None,
        'uptime_s': None, 'note': None, 'command': None}
RVIZ_LOCK = threading.Lock()
HELPER_TIMEOUT = 6.0        # no hello for this long -> helper considered gone


def rviz_public():
    with RVIZ_LOCK:
        age = time.time() - RVIZ['last_seen']
        present = RVIZ['last_seen'] > 0 and age < HELPER_TIMEOUT
        return {'helper': present,
                'running': bool(present and RVIZ['running']),
                'pid': RVIZ['pid'] if present else None,
                'uptime_s': RVIZ['uptime_s'] if present else None,
                'host': RVIZ['host'] if present else None,
                'note': RVIZ['note'] if present else None,
                'pending': RVIZ['command'] if present else None}


def load_token():
    """Persist the token so a bookmarked URL keeps working across restarts."""
    try:
        with open(TOKEN_FILE) as f:
            tok = f.read().strip()
        if tok:
            return tok
    except OSError:
        pass
    tok = secrets.token_urlsafe(12)
    os.makedirs(os.path.dirname(TOKEN_FILE), exist_ok=True)
    with open(TOKEN_FILE, 'w') as f:
        f.write(tok + '\n')
    os.chmod(TOKEN_FILE, 0o600)
    return tok


def host_ips():
    out = []
    try:
        import subprocess
        for line in subprocess.run(['hostname', '-I'], capture_output=True,
                                   text=True, timeout=5).stdout.split():
            if not line.startswith('172.17.'):        # docker0
                out.append(line)
    except Exception:
        pass
    return out


_TS = {'t': 0.0, 'name': None}


def tailscale_name():
    """This machine's Tailscale MagicDNS name, or None.

    Cached: it only changes on a rename, and scripts/set_hostname.sh restarts
    this service when it does.  Uncached it would fork `tailscale` on every
    /api/config, which the page fetches on load and the RViz helper polls.
    """
    if time.time() - _TS['t'] < 300:
        return _TS['name']
    name = None
    try:
        import subprocess
        r = subprocess.run(['tailscale', 'status', '--json'],
                           capture_output=True, text=True, timeout=5)
        if r.returncode == 0:
            me = json.loads(r.stdout).get('Self') or {}
            name = (me.get('DNSName') or '').rstrip('.') or None
    except Exception:
        pass
    _TS.update(t=time.time(), name=name)
    return name


def stable_urls():
    """URLs that survive a DHCP lease change -- the ones worth bookmarking.

    The Wi-Fi address is reassigned between sessions (the same thing that makes
    laptop_viewer.sh's JETSON_IP a recurring annoyance), so a bookmarked
    http://192.168.x.y:8080 goes stale at exactly the wrong moment.  A name does
    not: mDNS on the field Wi-Fi with no internet at all, MagicDNS from
    anywhere.  Both carry the token, because a URL without it just says
    "cannot reach the GCS API".
    """
    host = socket.gethostname().split('.')[0]
    tail = f'/?k={STATE["token"]}' if STATE['token'] else '/'
    urls = [{'name': f'{host}.local', 'note': 'same Wi-Fi',
             'url': f'http://{host}.local:{STATE["port"]}{tail}'}]
    ts = tailscale_name()
    if ts:
        urls.append({'name': ts, 'note': 'anywhere, over Tailscale',
                     'url': f'http://{ts}:{STATE["port"]}{tail}'})
    return urls


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'SuperGCS/1.0'

    # -- plumbing ----------------------------------------------------------
    def log_message(self, fmt, *args):
        pass          # the access log is noise; process logs are what matter

    def _send(self, code, body, ctype='application/json'):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _authed(self, query):
        if STATE['token'] is None:
            return True
        given = self.headers.get('X-GCS-Token') or (query.get('k') or [''])[0]
        return secrets.compare_digest(given, STATE['token'])

    def _admin(self, query):
        """Is this request carrying a live admin session?

        Checked per request rather than remembered per connection: HTTP/1.1
        keep-alive means one socket carries many requests, and a session that
        expired between two of them must actually expire.
        """
        tok = (self.headers.get(ADMIN_HEADER)
               or (query.get(ADMIN_QS) or [''])[0])
        return SESSIONS.check(tok)

    def _need_admin(self):
        # Names nothing: this is the answer a client that has NOT unlocked
        # gets, so spelling out the feature here would give back exactly what
        # the redaction withholds.
        return self._send(403, {'ok': False, 'admin_required': True,
                                'message': 'restricted -- unlock it with '
                                           'Admin control'})

    # -- routes ------------------------------------------------------------
    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        path = u.path

        if path.startswith('/api/'):
            if not self._authed(q):
                return self._send(401, {'error': 'bad or missing token'})
            return self._api_get(path, q)

        if path == '/':
            path = '/static/index.html'
        if not path.startswith('/static/'):
            return self._send(404, 'not found', 'text/plain')
        name = os.path.basename(path)
        full = os.path.join(STATIC, name)
        if not os.path.isfile(full):
            return self._send(404, 'not found', 'text/plain')
        with open(full, 'rb') as f:
            body = f.read()
        ctype = MIME.get(os.path.splitext(name)[1], 'application/octet-stream')
        return self._send(200, body, ctype)

    def _api_get(self, path, q):
        if path == '/api/state':
            return self._send(200, snapshot(self._admin(q)))
        if path == '/api/config':
            is_admin = self._admin(q)
            return self._send(200, {
                'host': socket.gethostname(), 'ips': host_ips(),
                'repo': REPO, 'auth': STATE['token'] is not None,
                # Keys here become log tabs, so a restricted row is left out
                # while locked even when (like dtp) its state row is shown.
                'procs': {k: v[0] for k, v in sup.PROCS.items()
                          if k not in RESTRICTED_LOGS or is_admin},
                'stack_configs': dict(dtp.STACK_CONFIGS),
                'admin': {'unlocked': is_admin, 'sessions': SESSIONS.count()},
                'urls': stable_urls(),
                'checks': {k: {'label': v.label, 'destructive': v.destructive}
                           for k, v in sup.CHECKS.items()},
                # The Flight day card: docs/flight-day-runbook.md as steps.
                'flight': flightday.config(),
                'bag_presets': {k: {'topics': len(v), 'images': any(camera_topic(t) for t in v)}
                                for k, v in bagger.PRESETS.items()},
                # The exact line to paste on the laptop, token included -- the
                # helper needs it, and this is the copy the user already has.
                # Built from the mDNS name, NOT host_ips()[0]: that was eno1
                # (192.168.1.50), the point-to-point link dedicated to the
                # MID360, which is not routable from a laptop -- so the line
                # the panel told you to paste could not have worked. Same trap
                # CLAUDE.md documents for laptop_viewer.sh's JETSON_IP.
                'helper_cmd': (f'bash scripts/gcs_laptop.sh '
                               f'\'{stable_urls()[0]["url"]}\''),
            })
        if path == '/api/bag/topics':
            return self._send(200, bag_topics(self._admin(q)))
        if path == '/api/bag/list':
            return self._send(200, {
                'dir': bagger.BAG_DIRNAME,
                'free_gb': bagger.free_gb(REPO),
                'bags': bagger.list_bags(
                    REPO, active=STATE['sup'].bag.active_path())})
        if path.startswith('/api/logs/'):
            key = path[len('/api/logs/'):]
            if key in RESTRICTED_LOGS and not self._admin(q):
                return self._need_admin()
            since = int((q.get('since') or ['0'])[0])
            return self._send(200, STATE['sup'].logs(key, since))
        if path == '/api/rc':
            if not self._admin(q):
                return self._need_admin()
            return self._send(200, rc_state())
        if path == '/api/camera/stream':
            if not self._admin(q):
                return self._need_admin()
            return self._camera_stream(q)
        if path == '/api/camera/snapshot':
            if not self._admin(q):
                return self._need_admin()
            return self._camera_snapshot()
        return self._send(404, {'error': 'no such endpoint'})

    # -- camera ------------------------------------------------------------
    # An <img> tag cannot send a header, so these two authenticate off the ?k=
    # in the URL like every other GET -- which is also why the page builds the
    # src from its stored token rather than assuming one is in location.search.

    def _feed(self):
        tel = STATE['tel']
        return getattr(tel, 'camera', None) if tel is not None else None

    def _camera_stream(self, q):
        """MJPEG (multipart/x-mixed-replace) straight from the camera topic.

        The bytes are the camera's own MJPEG, forwarded untouched: no decode,
        no re-encode, no image library on the Jetson.  Every browser has
        rendered this into an <img> for twenty years, so the page needs no
        player, no websocket and no build step -- which is the only kind of
        video this dashboard could have had.
        """
        feed = self._feed()
        if feed is None:
            return self._send(503, {'error': STATE.get(
                'ros_error', 'this dashboard has no ROS 2 telemetry')})
        try:
            fps = float((q.get('fps') or [DEFAULT_FPS])[0])
        except (TypeError, ValueError):
            fps = DEFAULT_FPS
        with STREAM_LOCK:
            if STREAMS['n'] >= MAX_STREAMS:
                return self._send(503, {'error': f'{MAX_STREAMS} viewers are '
                                        'already watching -- close one first'})
            STREAMS['n'] += 1
        try:
            with feed.viewer():
                first = feed.wait(0, FIRST_FRAME_S)
                if first is None:
                    return self._send(503, {'error': 'no camera frames -- is '
                                            'anything publishing?'})
                self._pump_frames(feed, first, 1.0 / fps if fps > 0 else 0.0)
        finally:
            with STREAM_LOCK:
                STREAMS['n'] -= 1

    def _pump_frames(self, feed, frame, min_gap):
        self.close_connection = True        # no content-length is possible here
        self.send_response(200)
        self.send_header('Content-Type',
                         f'multipart/x-mixed-replace; boundary={BOUNDARY}')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Connection', 'close')
        self.end_headers()
        seq, last = 0, 0.0
        while not SHUTDOWN.is_set():
            if frame is None:
                # Give up rather than hold the socket for ever: the camera may
                # have been stopped, and a closed stream makes the <img> retry,
                # which is how the feed comes back on its own.
                frame = feed.wait(seq, STREAM_IDLE_S)
                if frame is None:
                    return
            seq, data, _stamp, ctype = frame
            frame = None
            now = time.monotonic()
            if min_gap and now - last < min_gap:
                continue                    # dropped for the network only
            last = now
            try:
                self.wfile.write(
                    f'--{BOUNDARY}\r\nContent-Type: {ctype}\r\n'
                    f'Content-Length: {len(data)}\r\n\r\n'.encode()
                    + data + b'\r\n')
            except (BrokenPipeError, ConnectionResetError, OSError):
                return                      # tab closed, or the Wi-Fi went

    def _camera_snapshot(self):
        """One still frame.  What a phone on a bad link can actually load, and
        what `curl -o` gets you for a bug report."""
        feed = self._feed()
        if feed is None:
            return self._send(503, {'error': STATE.get(
                'ros_error', 'this dashboard has no ROS 2 telemetry')})
        feed.touch()
        frame = feed.wait(0, FIRST_FRAME_S)
        if frame is None:
            return self._send(503, {'error': 'no camera frames -- is anything '
                                    'publishing?'})
        return self._send(200, frame[1], frame[3])

    def do_POST(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if not self._authed(q):
            return self._send(401, {'error': 'bad or missing token'})
        parts = [p for p in u.path.split('/') if p]
        s = STATE['sup']
        try:
            if parts == ['api', 'admin', 'login']:
                return self._admin_login()
            if parts == ['api', 'admin', 'logout']:
                SESSIONS.logout(self.headers.get(ADMIN_HEADER)
                                or (q.get(ADMIN_QS) or [''])[0])
                return self._send(200, {'ok': True, 'message': 'locked'})
            # One gate for every way to reach the camera: the panel's buttons,
            # the Stack row's, and the source picker all land here.
            if ((parts[:2] == ['api', 'proc'] and len(parts) >= 3
                 and parts[2] in RESTRICTED_PROCS)
                    or parts[:2] == ['api', 'camera']
                    or parts[:2] == ['api', 'dtp']
                    or parts[:2] == ['api', 'rc']) and not self._admin(q):
                return self._need_admin()
            if parts == ['api', 'dtp', 'follow']:
                return self._dtp_follow(q)
            if parts[:2] == ['api', 'proc'] and len(parts) == 4:
                key, action = parts[2], parts[3]
                if action == 'start' and key == 'camera':
                    why = camera_blocked(camera_state(),
                                         STATE['sup'].procs['camera'].status(),
                                         STATE['sup'].procs['dtp'].status())
                    if why:
                        return self._send(409, {'ok': False,
                                                'message': 'refused: ' + why})
                if action == 'start' and key == 'dtp':
                    return self._dtp_start(q)
                if action == 'start' and key == 'stack':
                    ok, msg = s.start(key, {'config': (q.get('config') or
                                                       ['click'])[0]})
                elif action == 'start':
                    ok, msg = s.start(key)
                elif action == 'stop':
                    ok, msg = s.stop(key)
                else:
                    ok, msg = False, 'unknown action'
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if parts == ['api', 'stop_all']:
                # external=True: pressing STOP ALL means everything, including
                # a stack someone started in a terminal. The server's own
                # shutdown path calls stop_all() without it -- closing the
                # dashboard must not kill what it never started.
                ok, msg = s.stop_all(external=True)
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if parts == ['api', 'stop_external']:
                ok, msg = s.stop_external()
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if parts == ['api', 'fc', 'reboot']:
                why = reboot_blocked()
                if why:
                    return self._send(409, {'ok': False,
                                            'message': 'refused: ' + why})
                ok, msg = s.run_action('fc_reboot')
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if (parts[:2] == ['api', 'host'] and len(parts) == 3
                    and parts[2] in hostpower.MODES):
                mode = parts[2]
                why = host_power_blocked()
                if why:
                    return self._send(409, {'ok': False,
                                            'message': 'refused: ' + why})
                # The page asks twice; this is the gate a curl, a replayed URL
                # or a click on a stale tab has to pass as well.  It is the
                # only endpoint here that cannot be reached by accident.
                if (q.get('confirm') or [''])[0] != mode:
                    return self._send(409, {'ok': False, 'message': (
                        f'refused: powering the Jetson off is not something '
                        f'to do by accident -- this endpoint needs '
                        f'?confirm={mode}')})
                ok, msg = hostpower.schedule(
                    mode, log=STATE['sup'].note)
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if parts[:2] == ['api', 'rc'] and len(parts) == 3:
                return self._rc_command(parts[2], q)
            if parts == ['api', 'restart']:
                ok, msg = s.restart()
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if parts[:2] == ['api', 'check'] and len(parts) == 3:
                ok, msg = s.run_check(parts[2])
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if parts == ['api', 'camera', 'source']:
                # Watching an already-published topic is a SEPARATE action from
                # starting the driver, and this is it: no process is touched,
                # we just subscribe somewhere else.  ?topic= empty means "back
                # to automatic" (ours first, then whatever is publishing).
                feed = getattr(STATE['tel'], 'camera', None) if STATE['tel'] else None
                if feed is None:
                    return self._send(409, {'ok': False, 'message': STATE.get(
                        'ros_error', 'this dashboard has no ROS 2 telemetry')})
                topic = (q.get('topic') or [''])[0].strip()
                pinned = feed.pin(topic)
                return self._send(200, {'ok': True, 'message': (
                    f'watching {pinned}' if pinned
                    else 'source back to automatic')})
            if parts == ['api', 'bag', 'start']:
                return self._bag_start(q)
            if parts == ['api', 'bag', 'stop']:
                ok, msg = s.stop_bag()
                return self._send(200 if ok else 409, {'ok': ok, 'message': msg})
            if parts[:2] == ['api', 'rviz'] and len(parts) == 3:
                return self._rviz(parts[2])
        except Exception as exc:
            return self._send(500, {'ok': False, 'message': f'{type(exc).__name__}: {exc}'})
        return self._send(404, {'error': 'no such endpoint'})


    def _admin_login(self):
        """Password in a JSON BODY, never a query string.

        A query string ends up in every access log, in shell history and in
        the browser's address bar; the token already lives there because an
        <img> tag leaves no choice, but a password does not have to.
        """
        n = int(self.headers.get('Content-Length') or 0)
        try:
            body = json.loads(self.rfile.read(min(n, 4096)) or b'{}')
        except ValueError:
            body = {}
        token, why = SESSIONS.login(body.get('password'))
        if token is None:
            # 200 with ok:false, not 401: a 401 here would make the page's
            # generic handler throw away the DASHBOARD token and show the
            # wrong gate. Different credential, different failure.
            return self._send(200, {'ok': False, 'message': why})
        return self._send(200, {'ok': True, 'session': token,
                                'idle_timeout_s': SESSIONS.idle,
                                'message': 'unlocked'})

    def _rc_command(self, action, q):
        """Move the goal.  Every path here ends in one /goal_pose or a refusal.

        The refusal text is the product, not an afterthought: the interesting
        failures are all "SUPER will ignore this and not tell you" (a goal
        under 0.1 m, an altitude click_height overrides, a yaw the config
        discards), and each of those looks like a broken aircraft if the panel
        just reports success.
        """
        # Route first, telemetry second: /api/rc/fly is a 404 whether or not
        # there is a link, and answering "no telemetry" to it would send
        # somebody debugging DDS over a typo.
        if action not in ('move', 'altitude', 'goto'):
            return self._send(404, {'error': 'no such endpoint'})
        tel = STATE['tel']
        if tel is None:
            return self._send(409, {'ok': False, 'message': STATE.get(
                'ros_error', 'this dashboard has no ROS 2 telemetry')})
        cfg = rc.read_config(name=dtp.fsm_config()['name'])
        snap = tel.rc_snapshot()
        pose = snap.get('pose')
        try:
            graph = tel.goal_graph()
        except Exception:
            # A graph query that fails must not 500 a command: fall back to
            # what the config declares, which is what rc_state() renders too.
            graph = {}
        path = rc.choose_path(cfg, **graph)
        try:
            if action == 'move':
                axis = (q.get('axis') or [''])[0]
                step = float((q.get('step') or ['0'])[0] or 0)
                goal, notes = rc.plan_move(axis, step, pose, cfg, path,
                                           hold_z=_rc_hold_z(pose))
                what = f'{axis} {step:g}' + ('°' if axis.startswith('yaw') else ' m')
            elif action == 'altitude':
                z = float((q.get('z') or ['0'])[0] or 0)
                goal, notes = rc.plan_altitude(z, pose, cfg, path)
                what = f'altitude {goal["z"]:g} m'
            elif action == 'goto':
                # An empty yaw is not zero -- it means "hold the current
                # heading", so it has to survive as '' all the way into
                # plan_goto() rather than being floated here.
                goal, notes = rc.plan_goto(
                    (q.get('x') or [''])[0], (q.get('y') or [''])[0],
                    (q.get('z') or [''])[0], (q.get('yaw') or [''])[0],
                    pose, cfg, path)
                what = (f'goto {goal["x"]:g}, {goal["y"]:g}, {goal["z"]:g} '
                        f'({rc.dist3(pose, goal["x"], goal["y"], goal["z"]):.1f} m away)')
            else:                                  # unreachable: routed above
                return self._send(404, {'error': 'no such endpoint'})
        except rc.RCError as exc:
            return self._send(409, {'ok': False, 'message': f'refused: {exc}'})
        except (TypeError, ValueError) as exc:
            return self._send(400, {'ok': False, 'message': f'bad request: {exc}'})

        listeners = tel.goal_listeners(path['topic'])
        tel.publish_goal(goal, path['topic'])
        with RC_LOCK:
            # origin is what turns "1.2 m to go" into a progress fraction, and
            # best/best_at are the stall detector's memory. All three reset
            # here: a new goal is a new journey, not a continuation.
            RC_LAST.update(goal=goal, at=time.time(), what=what, notes=notes,
                           topic=path['topic'], origin=dict(pose),
                           best=None, best_at=time.time(), state='sent',
                           announced=None)
        msg = (f'goal {what}: [{goal["x"]:.2f}, {goal["y"]:.2f}, {goal["z"]:.2f}] '
               f'yaw {math.degrees(goal["yaw"]):.0f}° on {path["topic"]}')
        if not listeners:
            # Not an error -- the message went out. But nothing is listening,
            # and reporting plain success would be a lie of omission.
            msg += (f' -- WARNING: nothing is subscribed to {path["topic"]}, so '
                    'no planner received it. Is fsm_node running?')
        for n in notes:
            msg += f' -- {n}'
        STATE['sup'].note(f'[rc] {msg}')
        return self._send(200, {'ok': True, 'message': msg, 'goal': goal})

    def _dtp_start(self, q):
        """Start the follower -- the only process start on this page that can
        end with something commanding the aircraft, so it has its own route.

        The mode is required, never defaulted; the interlock is dtp_blocked();
        and a LIVE start must also carry ?confirm=live, the host-power rule: the
        page asks first, and this is the gate a curl, a replayed URL or a click
        on a stale tab has to pass as well.
        """
        mode = (q.get('mode') or [''])[0]
        if mode not in dtp.MODES:
            return self._send(400, {'ok': False, 'message': (
                'say which mode to start the follower in: ?mode='
                + ' or ?mode='.join(dtp.MODES))})
        # Before the interlock, not after: the answer to a request that did
        # not say confirm=live must not depend on the aircraft's state, and
        # test_dtp.py can then prove this gate without an aircraft to satisfy.
        if mode == 'live' and (q.get('confirm') or [''])[0] != 'live':
            return self._send(409, {'ok': False, 'message': (
                'refused: a live follower commands the aircraft, and that is '
                'not something to start by accident -- this endpoint needs '
                '?confirm=live')})
        why = dtp_blocked(mode)
        if why:
            return self._send(409, {'ok': False, 'message': 'refused: ' + why})
        # The config fsm_node is ACTUALLY on, so the follower resolves its
        # topic names from the same file.  Only a whitelisted name is passed.
        name = dtp.fsm_config()['name']
        config = name if name in dtp.STACK_CONFIGS.values() else None
        ok, msg = STATE['sup'].start('dtp', {'mode': mode, 'config': config})
        if ok:
            STATE['sup'].note(f'[dtp] follower started in {mode.upper()} mode'
                              + (f' on {config}' if config else ''))
        return self._send(200 if ok else 409, {'ok': ok, 'message': msg})

    def _dtp_follow(self, q):
        """Engage (?set=on) or disengage (?set=off) FOLLOW on the running follower.

        The one button that designates a target.  Admin control gates it
        (above); dtp.follow_blocked() refuses without a follower, without
        telemetry, or after a mux ABORT; and it is deliberately NOT refused on
        an armed aircraft, because a lock lost in flight is when it is needed.
        The page asks nothing first for the same reason.  ?set=on is always a
        false -> true cycle (field.sh reengage), so it re-designates from LOST.
        """
        action = (q.get('set') or [''])[0]
        if action not in dtp.FOLLOW_ACTIONS:
            return self._send(400, {'ok': False, 'message': (
                'say what to do: ?set=on (engage / re-engage) or ?set=off')})
        rows = {r['key']: r for r in STATE['sup'].status()}
        ros = _ros_state()
        why = dtp.follow_blocked(action, row=rows.get('dtp'),
                                 ros_ok=bool(ros.get('available')),
                                 state=ros.get('dtp'))
        if why:
            return self._send(409, {'ok': False, 'message': 'refused: ' + why})
        ok, err = STATE['tel'].follow(action == 'on')
        if not ok:
            return self._send(409, {'ok': False, 'message': 'failed: ' + err})
        msg = ('FOLLOW engaged -- watch the lock state; LOCKED needs a person at '
               '2.5-10 m who is moving' if action == 'on' else 'FOLLOW disengaged')
        STATE['sup'].note('[dtp] ' + msg)
        return self._send(200, {'ok': True, 'message': msg})

    def _bag_start(self, q):
        """Start a recording.  Body: {all: bool, topics: [...], label: str}.

        The topic list is a JSON body rather than a query string for the same
        reason the admin password is: a request line ends up in logs and in the
        address bar, and there can be a hundred topics in it.
        """
        n = int(self.headers.get('Content-Length') or 0)
        try:
            body = json.loads(self.rfile.read(min(n, 65536)) or b'{}')
        except ValueError:
            body = {}
        topics = [str(t) for t in (body.get('topics') or [])]
        # A Flight day preset is just a topic list chosen server-side, so it
        # passes through exactly the same camera gate as a hand-picked list.
        preset = str(body.get('preset') or '')
        if preset:
            if preset not in bagger.PRESETS:
                return self._send(400, {'ok': False, 'message': (
                    'unknown preset; one of ' + ', '.join(bagger.PRESETS))})
            topics = list(bagger.PRESETS[preset])
        all_topics = bool(body.get('all'))
        label = str(body.get('label') or preset)
        exclude = ''
        if not self._admin(q):
            # The camera gate, applied to the other way of reading a camera.
            # Naming an image topic is refused outright; `record all` gets an
            # --exclude so it cannot sweep one up by accident.  An unlocked
            # client records exactly what it asked for.
            if any(camera_topic(t) for t in topics):
                return self._need_admin()
            exclude = bagger.IMAGE_EXCLUDE
        ok, msg = STATE['sup'].start_bag(topics=topics, all_topics=all_topics,
                                         label=label, exclude=exclude)
        return self._send(200 if ok else 409, {'ok': ok, 'message': msg})

    def _rviz(self, action):
        if action == 'hello':
            n = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                body = {}
            with RVIZ_LOCK:
                RVIZ.update(last_seen=time.time(),
                            running=bool(body.get('running')),
                            pid=body.get('pid'), host=body.get('host'),
                            uptime_s=body.get('uptime_s'),
                            note=body.get('note'))
                cmd, RVIZ['command'] = RVIZ['command'], None
            return self._send(200, {'command': cmd})

        if action in ('start', 'stop'):
            state = rviz_public()
            if not state['helper']:
                return self._send(409, {
                    'ok': False,
                    'message': ('no RViz helper is running on your laptop -- '
                                'run scripts/gcs_laptop.sh there')})
            with RVIZ_LOCK:
                RVIZ['command'] = action
            return self._send(200, {'ok': True,
                                    'message': f'RViz {action} sent to '
                                               f'{state["host"] or "the laptop"}'})
        return self._send(404, {'error': 'no such endpoint'})


def reboot_blocked():
    """Why a flight-controller reboot must not be sent right now, or None.

    Deliberately duplicated: fc_reboot.py asks the FC itself over MAVLink
    whether it is armed, and this asks the uXRCE-DDS telemetry we already have.
    Two independent sources have to agree the aircraft is on the ground before
    anything resets the flight controller.  With no telemetry at all (--no-ros)
    this refuses rather than falling back to the script's single check: a
    dashboard that cannot see the vehicle should not be resetting it.
    """
    tel = STATE['tel']
    if tel is None:
        return ('this dashboard has no ROS 2 telemetry, so it cannot verify the '
                'vehicle is disarmed -- run scripts/px4/fc_reboot.py --yes in a '
                'shell instead')
    try:
        a = tel.snapshot()['aircraft']
    except Exception as exc:
        return f'cannot read vehicle status ({exc})'
    if not a.get('fresh'):
        return ('no fresh /fmu/out/vehicle_status_v1 -- the link to the FC is '
                'already down, so there is nothing to reboot over it')
    if a.get('armed'):
        return 'the vehicle is ARMED. Disarm it first'
    if a.get('landed') is False:
        return 'the FC reports the vehicle is IN AIR'
    return None


def host_power_blocked():
    """Why the JETSON must not be powered off or rebooted right now, or None.

    Only POSITIVE evidence that the aircraft is flying refuses here.  That is a
    weaker rule than reboot_blocked()'s on purpose: a dashboard with no FC link
    should not reset the flight controller, but "no telemetry" is the ordinary
    end-of-day shape for the Jetson -- the FC is off and you want the computer
    off too.  Refusing then would only teach people to ssh in and sudo, which
    skips this check entirely.
    """
    ok, why = hostpower.sudo_ready()
    if not ok:
        return why
    p = hostpower.pending()
    if p['mode']:
        return f'a {p["mode"]} is already scheduled'
    if STATE['sup'].busy:
        return f'{STATE["sup"].busy} in progress'
    tel = STATE['tel']
    if tel is None:
        return None
    try:
        a = tel.snapshot()['aircraft']
    except Exception:
        return None          # a failed poll is not a reason to stay powered on
    return hostpower.flight_blocked(a)


def power_state():
    """hostpower's own state plus the reason the buttons are greyed out.

    Same rule as Reboot FC and the camera's Start: the page must not OFFER an
    action the server would refuse, and it must say why rather than just going
    grey -- a button that fails when pressed reads as a broken page.
    """
    return dict(hostpower.status(), blocked=host_power_blocked())


def camera_state():
    """What the camera panel needs: the live topic, and who owns the hardware.

    The USB half works with no ROS at all, on purpose -- it is what turns a
    greyed-out Start button into a sentence saying which pid has the camera.
    """
    feed = getattr(STATE['tel'], 'camera', None) if STATE['tel'] else None
    if feed is None:
        out = {'available': False, 'source': None, 'sources': [], 'viewers': 0,
               'error': STATE.get('ros_error',
                                  'this dashboard has no ROS 2 telemetry, so it '
                                  'cannot show the feed')}
    else:
        out = feed.status()
    devices = camera.usb_devices()
    out['devices'] = devices
    # Only worth a /proc/*/fd walk when nothing is publishing: if a ROS topic is
    # live we already know who has the camera, and this scan cannot see another
    # user's fds anyway (CLAUDE.md).
    out['holders'] = ([] if out.get('source')
                      else camera.usb_holders([d['node'] for d in devices]))
    out['max_streams'] = MAX_STREAMS
    out['default_fps'] = DEFAULT_FPS
    return out


def rc_state():
    """Everything the RC card renders, in one 5 Hz poll.

    Deliberately its own endpoint rather than part of /api/state: this is
    polled fast enough to make an attitude display readable, and /api/state
    carries the battery, the estimator, every process row and the bag listing.
    """
    tel = STATE['tel']
    cfg = rc.read_config(name=dtp.fsm_config()['name'])
    # The path is decided from the ROS graph where there is one, so the panel
    # describes the planner that is RUNNING rather than the config on disk.
    graph = {}
    if tel is not None:
        try:
            graph = tel.goal_graph()
        except Exception:
            graph = {}
    path = rc.choose_path(cfg, **graph)
    out = {'t': time.time(),
           'config': {k: cfg.get(k) for k in
                      ('path', 'error', 'click_height', 'yaw_mode',
                       'click_yaw_en', 'goal_yaw_en', 'max_vel', 'yaw_dot_max',
                       'goal_topic', 'altitude', 'follow_goal_en',
                       'follow_goal_topic', 'goal_status_topic')},
           'path': path,
           'graph': graph,
           'capabilities': {k: {'ok': v[0], 'why': v[1]}
                            for k, v in rc.capabilities(cfg, path).items()},
           # The bounds the typed boxes put in their min/max attributes come
           # from here rather than from a constant in app.js: two copies of a
           # limit is two limits, and the server's is the one that refuses.
           'limits': {'steps': list(rc.LINEAR_STEPS),
                      'yaw_steps': list(rc.YAW_STEPS),
                      'default_step': rc.DEFAULT_STEP,
                      'default_yaw_step': rc.DEFAULT_YAW_STEP,
                      'step_min': rc.STEP_MIN, 'step_max': rc.STEP_MAX,
                      'yaw_min': rc.YAW_MIN, 'yaw_max': rc.YAW_MAX,
                      'goto_max': rc.GOTO_MAX_M,
                      'goto_confirm': rc.GOTO_CONFIRM_M,
                      'nudge': rc.YAW_NUDGE_M}}
    with RC_LOCK:
        out['last'] = dict(RC_LAST)
    if tel is None:
        out['available'] = False
        out['error'] = STATE.get('ros_error',
                                 'this dashboard has no ROS 2 telemetry')
    else:
        out['available'] = True
        out.update(tel.rc_snapshot())
        out['goal_listeners'] = tel.goal_listeners(path['topic'])
        try:
            out['aircraft'] = tel.aircraft_only()
        except Exception:
            out['aircraft'] = None
        out['goal_status'] = _rc_status_for_us(out.get('goal_status'))
        out['progress'] = _rc_progress(out, cfg, path)
    # Computed even with no telemetry at all: a checklist is most useful in
    # exactly the case where nothing works, and a panel that shows four blanks
    # there is a panel that stops being trusted.
    out['ready'] = rc_ready(out)
    return out


def rc_ready(st):
    """The checklist under the D-pad: what has to be true for a press to fly.

    Four independent things, each of which fails on its own and none of which
    is visible from the others -- a goal is accepted by a planner that is not
    running, and flown by a bridge PX4 is ignoring, without a single error.
    """
    pose = st.get('pose')
    a = st.get('aircraft') or {}
    rows = []
    fresh = bool(pose and (pose.get('age') or 99) <= rc.ODOM_STALE_S)
    rows.append({'key': 'odom', 'ok': fresh, 'label': 'FAST-LIO odometry',
                 'note': ('lidar_init pose is live' if fresh else
                          'no fresh /lidar_slam/odom -- goals have no frame')})
    path = st.get('path') or {}
    topic = path.get('topic') or rc.GOAL_TOPIC_DEFAULT
    listening = bool(st.get('goal_listeners')) and not path.get('contended')
    rows.append({'key': 'planner', 'ok': listening, 'label': 'SUPER listening',
                 'note': (path['contended'] if path.get('contended') else
                          f'{st.get("goal_listeners")} subscriber(s) on {topic}'
                          if st.get('goal_listeners') else
                          f'nothing subscribes to {topic} -- fsm_node is not '
                          'running, so a goal goes nowhere')})
    armed = bool(a.get('armed'))
    rows.append({'key': 'armed', 'ok': armed, 'label': 'Armed',
                 'note': 'armed' if armed else
                         'disarmed -- goals still plan, nothing flies'})
    offb = bool(a.get('offboard'))
    rows.append({'key': 'offboard', 'ok': offb, 'label': 'Offboard',
                 'note': 'PX4 is following the bridge' if offb else
                         'not in Offboard -- PX4 ignores the setpoints '
                         '(RC channel 11)'})
    return rows


def _rc_status_for_us(status):
    """Tag the last GoalStatus with whether it answers OUR last goal.

    /planning/follow_goal has several legitimate publishers and fsm answers
    every message on it, so a status arriving here may be about somebody else's
    goal entirely.  `requested_p` is the only correlation key that works --
    `goal_p` is the PREVIOUS goal when one was DROPPED, which is exactly the
    case a caller most needs to identify (GoalStatus.msg says so at length).
    """
    if not status:
        return status
    with RC_LOCK:
        mine = RC_LAST.get('goal')
    req = status.get('requested') or {}
    status['ours'] = bool(mine and all(
        req.get(k) is not None and abs(req[k] - mine[k]) < 1e-3
        for k in ('x', 'y', 'z')))
    return status


def _rc_progress(st, cfg, path):
    """Track the last goal to its conclusion, and say so ONCE in the log.

    The page is polled at 5 Hz, so the interesting thing here is not the
    computation -- it is that a state change is announced exactly once. A log
    that repeated "arrived" three hundred times would bury the flight it is
    supposed to record.
    """
    with RC_LOCK:
        prog = rc.track_goal(RC_LAST, st.get('pose'), st.get('aircraft'),
                             st.get('goal_status'), cfg, path)
        if prog and prog['state'] in ('arrived', 'rejected', 'stalled'):
            if RC_LAST.get('announced') != prog['state']:
                RC_LAST['announced'] = prog['state']
                note = f'[rc] {prog["label"]}: {prog["detail"]}'
            else:
                note = None
        else:
            note = None
    if note:
        STATE['sup'].note(note)
    return prog


def _rc_hold_z(pose):
    """The altitude a lateral move keeps: the last one commanded, else the
    aircraft's own.  Without this, every sideways step would also re-assert
    whatever z the aircraft had drifted to."""
    with RC_LOCK:
        goal = RC_LAST.get('goal')
        at = RC_LAST.get('at') or 0.0
    if goal and time.time() - at < RC_GOAL_TTL:
        return goal['z']
    return None


def dtp_blocked(mode, ros=None, cam=None, procs=None):
    """Why the DTP follower must not be started in `mode` now, or None.

    The plumbing half: gathers the facts and hands them to dtp.start_blocked(),
    which is the rule and is tested on its own.  snapshot() passes in what it
    has already computed, so rendering the reason twice a second costs no
    second telemetry snapshot and no second camera scan.
    """
    procs = STATE['sup'].status() if procs is None else procs
    rows = {r['key']: r for r in procs}
    ros = _ros_state() if ros is None else ros
    cam = camera_state() if cam is None else cam
    available = bool(ros.get('available'))
    odom = (ros.get('rates') or {}).get(dtp.ODOM_TOPIC) if available else None
    # The rate subscriptions only exist while someone polls; until they are up
    # a 0.0 is "not measured yet", not "FAST-LIO is dead".
    odom_hz = odom['hz'] if odom and ros.get('hot') else None
    return dtp.start_blocked(
        mode, row=rows.get('dtp'),
        aircraft=ros.get('aircraft') if available else None,
        ros_ok=available, fsm=dtp.fsm_config(), missing=dtp.stack_missing(),
        camera_row=rows.get('camera'), cam=cam, odom_hz=odom_hz,
        install_error=dtp.installed())


def camera_blocked(cam, row, dtp_row=None):
    """Why the camera must not be started right now, or None.

    A Gemini can be opened by exactly ONE process; a second reader dies with
    `uvc_open ... Return Code: -6` (docs/geminitest.md).  On this Jetson the
    camera is regularly already owned by another launch under /camera, so
    'Start' has to mean "start it if it is free", never "start a second driver
    and take somebody else's perception stack down with it".
    """
    if row and row['state'] != 'stopped':
        return 'the camera is already running here'
    # Checked before the topic: a follower that is still starting has opened
    # nothing yet, so there is no publisher to find -- and in a few seconds it
    # will open the camera itself.
    if dtp_row and dtp_row['state'] != 'stopped':
        return ('the DTP follower is running, and it opens the Gemini itself -- '
                'watch its /camera stream instead')
    if not cam.get('devices'):
        return ('no Orbbec camera is on USB -- check the cable, then `lsusb | '
                'grep 2bc5`')
    src = cam.get('source')
    if src:
        return (f'another process is already publishing {src}. A Gemini can '
                'only be opened once, so watch that stream instead of starting '
                'a second driver on top of it')
    holders = cam.get('holders') or []
    if holders:
        h = holders[0]
        return (f'the USB device is held by pid {h["pid"]} '
                f'({h["cmd"].split(" ")[0] or "unknown"})')
    return None


def camera_topic(name):
    """Is this topic a camera image, by name alone?

    Used where the graph is not available or not trusted: refusing a POSTed
    topic list.  Name-based because that is what a client sends, and because
    rosbag2's own --exclude can only match names -- the two have to agree or a
    topic refused here could still be swept up by `record all`.
    """
    return bool(re.search(bagger.IMAGE_EXCLUDE, name or ''))


def bag_topics(is_admin=False):
    """What the recorder's topic picker offers.

    Image topics are withheld from a locked client for the same reason
    /api/state omits the camera block: the redaction is about not disclosing
    that a camera is here and publishing, and a topic picker is a list of
    everything that is publishing.  Filtered by TYPE here (the graph knows the
    types) and by NAME in _bag_start (a POSTed list is just strings) -- both
    ends of the same gate.
    """
    tel = STATE['tel']
    if tel is None:
        return {'ok': False, 'topics': [], 'error': STATE.get(
            'ros_error', 'this dashboard has no ROS 2 telemetry, so it cannot '
                         'list topics -- "All topics" still records')}
    try:
        rows = tel.topics()
    except Exception as exc:
        return {'ok': False, 'topics': [], 'error': f'{type(exc).__name__}: {exc}'}
    out = []
    for r in rows:
        kind = bagger.classify(r['type'])
        if kind == 'image' and not is_admin:
            continue
        out.append({'topic': r['topic'], 'type': r['type'],
                    'publishers': r['publishers'], 'kind': kind})
    return {'ok': True, 'topics': out, 'admin': is_admin,
            'excluded': None if is_admin else bagger.IMAGE_EXCLUDE}


def _stack_rows(procs, ros, is_admin):
    """Annotate the Stack and DTP rows with what they are actually running.

    The stack's config and the follower's mode are read off the live processes
    (dtp.py), so both are right for a copy started in a terminal as well.
    """
    rows = {r['key']: r for r in procs}
    fsm = dtp.fsm_config()
    stack = rows.get('stack')
    if stack is not None:
        stack['config'] = fsm['name']
        stack['config_error'] = fsm['error'] if fsm['pids'] else None
    row = rows.get('dtp')
    if row is None:
        return
    row['mode'] = dtp.follower_mode() if row['state'] != 'stopped' else None
    if row['mode'] is None and row['state'] == 'running':
        # Ours, still coming up: command_mux has not started yet. Say what it
        # was asked for rather than nothing.
        row['mode'] = (row.get('opts') or {}).get('mode')
        row['mode_pending'] = True
    # The lock and mux state are shown to everyone, like the mode: whether the
    # follower is tracking someone is not something to hide. The button is not.
    st = ros.get('dtp') if ros.get('available') else None
    row['follow'] = ({'lock': st.get('lock'), 'mux': st.get('mux'),
                      'fresh': bool(st.get('lock_fresh') or st.get('mux_fresh'))}
                     if st else None)
    if not is_admin:
        # State only. The reasons name the camera, and the controls are not
        # this client's to press.
        row['locked'] = True
        return
    a = ros.get('aircraft') if ros.get('available') else None
    row['stop_note'] = dtp.stop_note(row, a)
    row['follow_blocked'] = {
        act: dtp.follow_blocked(act, row=row, ros_ok=bool(ros.get('available')),
                                state=st)
        for act in dtp.FOLLOW_ACTIONS}


def snapshot(is_admin=False):
    procs = STATE['sup'].status()
    ros = _ros_state()
    _stack_rows(procs, ros, is_admin)
    if not is_admin:
        # Redaction, not just a hidden button. Without this the topic name, the
        # device serial and whether the camera is running would all still be on
        # the wire for anyone holding only the dashboard token -- and the page
        # would be hiding a panel whose contents it had already received.
        return {'t': time.time(),
                'procs': [dict(r, locked=True) if r['key'] in RESTRICTED_PROCS
                          else r for r in procs if r['key'] != 'camera'],
                'admin': {'unlocked': False},
                'host': sysinfo.host(),
                'check_running': STATE['sup'].check_running,
                'checks': STATE['sup'].check_results,
                'busy': STATE['sup'].busy, 'error': STATE['sup'].last_error,
                # Counts and sizes only, never the topic names -- see the note
                # in Recorder.status().
                'bag': STATE['sup'].bag_status(),
                'power': power_state(),
                'rviz': rviz_public(), 'ros': ros}
    cam = camera_state()
    rows = {r['key']: r for r in procs}
    for row in procs:
        if row['key'] == 'camera':
            # Rendered as the row's tooltip and as the panel's reason. The
            # server refuses anyway, but a button you can press and that then
            # says no reads as a broken page -- same rule as Reboot FC.
            row['start_blocked'] = camera_blocked(cam, row, rows.get('dtp'))
        elif row['key'] == 'dtp':
            # One reason per mode: the page shows the one for the mode chip
            # that is selected, and the same function refuses the POST.
            row['blocked'] = {m: dtp_blocked(m, ros, cam, procs)
                              for m in dtp.MODES}
    return {'t': time.time(), 'procs': procs, 'camera': cam,
            'admin': {'unlocked': True, 'idle_timeout_s': SESSIONS.idle},
            'host': sysinfo.host(),
            'check_running': STATE['sup'].check_running,
            'checks': STATE['sup'].check_results,
            'busy': STATE['sup'].busy, 'error': STATE['sup'].last_error,
            'bag': STATE['sup'].bag_status(),
            'power': power_state(),
            'rviz': rviz_public(), 'ros': ros}


def _ros_state():
    tel = STATE['tel']
    if tel is None:
        return {'available': False,
                'error': STATE.get('ros_error', 'ROS 2 not initialised')}
    try:
        return dict(tel.snapshot(), available=True)
    except Exception as exc:
        return {'available': False, 'error': str(exc)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--bind', default='0.0.0.0')
    ap.add_argument('--no-auth', action='store_true',
                    help='disable the access token (only sane with --bind 127.0.0.1)')
    ap.add_argument('--no-ros', action='store_true',
                    help='process control only; skip rclpy (for testing)')
    args = ap.parse_args()

    STATE['sup'] = sup.Supervisor(REPO)
    STATE['port'] = args.port
    STATE['token'] = None if args.no_auth else load_token()
    STATE['tel'] = None

    if not args.no_ros:
        try:
            import telemetry
            STATE['tel'] = telemetry.TelemetryThread()
        except Exception as exc:
            STATE['ros_error'] = f'{type(exc).__name__}: {exc}'
            print(f'WARNING: telemetry disabled -- {STATE["ros_error"]}\n'
                  '         Did you start this via scripts/gcs.sh?', file=sys.stderr)

    # Restarting the GCS is routine, and the outgoing one holds the port for
    # the few seconds its teardown takes -- so wait for it rather than making
    # the user guess how long "a moment" is.
    httpd = None
    deadline = time.time() + 20
    while True:
        try:
            httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
            break
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE or time.time() > deadline:
                sys.exit(f'ERROR: cannot bind {args.bind}:{args.port} -- {exc}\n'
                         '       Another GCS is already running here. Stop it, '
                         'or use --port.')
            print(f'{args.bind}:{args.port} still busy, waiting...', flush=True)
            time.sleep(2)
    httpd.daemon_threads = True

    suffix = '' if STATE['token'] is None else f'/?k={STATE["token"]}'
    print('=' * 68)
    print('  SUPER web GCS is up.  Open this on your laptop or phone:')
    # Names first and IPs second, deliberately: the IP is the one that goes
    # stale between sessions, and whichever line is printed first is the one
    # that gets bookmarked.
    rows = [(u['url'], u['note']) for u in stable_urls()]
    rows.append(('', ''))
    rows += [(f'http://{ip}:{args.port}{suffix}', 'IP — changes with DHCP')
             for ip in (host_ips() or ['127.0.0.1'])]
    wide = max(len(u) for u, _ in rows)
    for url, note in rows:
        print(f'      {url:<{wide}}  {note}'.rstrip())
    print()
    print('  Ctrl-C stops the server AND everything it started.')
    print('=' * 68, flush=True)

    # Explicit handlers rather than relying on KeyboardInterrupt: teardown
    # below owns real process groups and must run even if something has
    # replaced the default SIGINT behaviour.  shutdown() has to be called from
    # another thread or it deadlocks against serve_forever().
    def _bye(signum, _frame):
        print(f'\nreceived {signal.Signals(signum).name}, shutting down...',
              flush=True)
        # Open MJPEG streams are threads parked on feed.wait(); without this
        # they would each hold a socket until their idle timeout expired.
        SHUTDOWN.set()
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _bye)
    signal.signal(signal.SIGTERM, _bye)

    try:
        httpd.serve_forever()
    finally:
        # Never leave an orphaned stack behind: we own these process groups.
        ok, msg = STATE['sup'].stop_all()
        print(f'stack teardown: {msg}', flush=True)
        if STATE['tel']:
            STATE['tel'].shutdown()
        httpd.server_close()


if __name__ == '__main__':
    main()
