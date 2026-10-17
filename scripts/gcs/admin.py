#!/usr/bin/env python3
"""Password gate for the dashboard's restricted panels.

What it guards today is the camera -- the one thing here that watches PEOPLE.
Nothing the WEB UI shows ever says so: the gate names no feature, because that
text is rendered for a client that has not unlocked it.

The dashboard's own token says "you are allowed to fly this aircraft".  This
says something narrower: "you are allowed to switch on a camera and watch what
it sees".  Those are not the same permission, and a field Wi-Fi with a shared
bookmark is exactly where they come apart.

ENFORCED IN THE SERVER, NOT IN THE PAGE.  A JavaScript check would be theatre:
`curl /api/camera/stream` would still hand out video, and the password would be
sitting in app.js for anyone who pressed View Source.  So the camera endpoints
require an admin session, and while there is none, `/api/state` and
`/api/config` do not even mention the camera -- not its topic, not the device
serial, not whether it is running.  The page hides the panel because the data
is not there, which is the right way round.

WHAT THIS IS AND IS NOT.  It is a second factor on one feature of a LAN
dashboard.  It is not a defence against someone who can read this repository:
the shipped default is a hash rather than the word, but a short known password
is a short known password.  Change it on any machine that matters:

    python3 scripts/gcs/admin.py --set        # prompts, writes ~/.cache/super-gcs/admin

The file wins over the built-in default, so a fresh clone works out of the box
and a machine that has been locked down stays locked down across a git pull.
"""
import argparse
import getpass
import hashlib
import hmac
import os
import secrets
import sys
import threading
import time

ADMIN_FILE = os.path.expanduser('~/.cache/super-gcs/admin')
SCHEME = 'pbkdf2_sha256'
ITERATIONS = 200_000          # ~93 ms on this Jetson: unnoticeable to type
                              # through, 10 000x slower than a bare sha256 to
                              # grind through.

# Hash of the password this was set up with, so the plaintext is never in git.
# Overridden by ADMIN_FILE when that exists.
DEFAULT = ('pbkdf2_sha256$200000$206c6b437cd5aa9a7c4a7e70d1df85a0'
           '$4a0ebb7d5fafca208c7c40d1ee347c5f249bed4eda75f5a4bc78b0aa179fbbed')

SESSION_IDLE = 1800.0         # 30 min with no request -> re-enter the password
MAX_FAILURES = 5              # then a cooldown, so this cannot be ground down
FAILURE_WINDOW = 300.0
LOCKOUT = 30.0


def encode(password, iterations=ITERATIONS, salt=None):
    salt = salt or os.urandom(16)
    h = hashlib.pbkdf2_hmac('sha256', password.encode(), salt, iterations)
    return f'{SCHEME}${iterations}${salt.hex()}${h.hex()}'


def verify(password, stored):
    """Constant-time check of a password against a stored hash."""
    try:
        scheme, iters, salt, want = stored.split('$')
        if scheme != SCHEME:
            return False
        got = hashlib.pbkdf2_hmac('sha256', password.encode(),
                                  bytes.fromhex(salt), int(iters))
    except (ValueError, AttributeError):
        return False
    return hmac.compare_digest(got.hex(), want)


def stored_hash():
    """The hash in force: the file if there is one, else the shipped default."""
    try:
        with open(ADMIN_FILE) as f:
            line = f.read().strip()
        if line:
            return line
    except OSError:
        pass
    return DEFAULT


def set_password(password):
    os.makedirs(os.path.dirname(ADMIN_FILE), exist_ok=True)
    # Create with 0600 from the start rather than chmod after: for a moment in
    # between, the hash would be world-readable.
    fd = os.open(ADMIN_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f:
        f.write(encode(password) + '\n')
    return ADMIN_FILE


class Sessions:
    """Admin sessions, in memory only.

    In memory means a GCS restart logs everybody out, which is the behaviour
    you want from a gate on a camera: nothing survives that you did not
    deliberately re-enter.
    """

    def __init__(self, idle=SESSION_IDLE):
        self._lock = threading.Lock()
        self._live = {}                 # token -> last seen (monotonic)
        self._failures = []             # monotonic stamps of wrong passwords
        self.idle = idle

    def locked_for(self):
        """Seconds until login is allowed again, or 0."""
        now = time.monotonic()
        with self._lock:
            self._failures = [t for t in self._failures
                              if now - t < FAILURE_WINDOW]
            if len(self._failures) < MAX_FAILURES:
                return 0.0
            return max(0.0, LOCKOUT - (now - self._failures[-1]))

    def login(self, password):
        """(token, None) on success, (None, reason) on failure."""
        wait = self.locked_for()
        if wait:
            return None, (f'too many wrong passwords -- wait {wait:.0f}s')
        if not verify(password or '', stored_hash()):
            with self._lock:
                self._failures.append(time.monotonic())
            return None, 'wrong password'
        token = secrets.token_urlsafe(24)
        with self._lock:
            self._failures.clear()
            self._live[token] = time.monotonic()
        return token, None

    def check(self, token):
        """True if this token is a live session; refreshes its idle timer."""
        if not token:
            return False
        now = time.monotonic()
        with self._lock:
            for dead in [t for t, seen in self._live.items()
                         if now - seen > self.idle]:
                del self._live[dead]
            # Compare against every live token in constant time: a plain dict
            # lookup leaks nothing much here, but there is no reason to make
            # the timing of a miss depend on the token's prefix.
            for live, _seen in list(self._live.items()):
                if hmac.compare_digest(live, token):
                    self._live[live] = now
                    return True
        return False

    def logout(self, token):
        with self._lock:
            self._live.pop(token, None)

    def count(self):
        with self._lock:
            return len(self._live)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--set', action='store_true',
                    help='set the admin password (prompts twice)')
    ap.add_argument('--check', action='store_true',
                    help='prompt for a password and say whether it is accepted')
    args = ap.parse_args()

    if args.set:
        pw = getpass.getpass('New admin password: ')
        if len(pw) < 6:
            sys.exit('too short -- use at least 6 characters')
        if pw != getpass.getpass('Again: '):
            sys.exit('they do not match')
        print(f'written to {set_password(pw)} (mode 0600)')
        print('Restart the GCS for it to take effect: '
              'sudo systemctl restart super-gcs')
        return 0
    if args.check:
        ok = verify(getpass.getpass('Password: '), stored_hash())
        print('accepted' if ok else 'REJECTED')
        return 0 if ok else 1

    where = ADMIN_FILE if os.path.exists(ADMIN_FILE) else 'built-in default'
    print(f'admin password source: {where}')
    if where != ADMIN_FILE:
        print('  This machine is using the password that ships in the repo. '
              'Change it with --set if that matters here.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
