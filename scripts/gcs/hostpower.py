#!/usr/bin/env python3
"""Power off / reboot the JETSON ITSELF -- the computer this dashboard runs on.

Everything else in this GUI stops software.  These two stop the machine, and on
this project that machine is bolted to an aircraft: the absolute position EKF2
is holding is FAST-LIO's, computed on this board, so cutting it in the air is a
fall rather than an inconvenience.  Hence three independent gates, none of
which is the other's fallback:

  * the page asks TWICE (app.js) -- one misplaced tap on a phone is not a
    shutdown, and the second question names the host so it has to be read;
  * the POST must carry ?confirm=<mode>, so nothing reaches this by accident --
    a stray curl, a replayed URL, or a click on a stale tab;
  * server.py's host_power_blocked() refuses while live /fmu/* telemetry says
    ARMED or IN AIR, the same shape as reboot_blocked() for the FC.

Deliberately NOT the same rule as reboot_blocked() in one respect: that one
refuses when it cannot see the aircraft at all, because a dashboard that has
lost the FC has no business resetting it.  Here "no telemetry" is the ordinary
end-of-day case -- the FC is off and you want the Jetson off too -- so this
refuses only on POSITIVE evidence that the vehicle is flying.

`sudo -n` so it can never sit on a password prompt nobody will ever see: a
systemd service has no terminal, and plain `sudo` would block until its own
timeout with the button apparently doing nothing.  `systemctl` rather than
`shutdown -h now` so the ordinary ordered shutdown runs -- that is what stops
super-gcs.service, and its teardown SIGINTs a running rosbag2 so the bag keeps
the metadata.yaml without which it is not playable at all (CLAUDE.md).

To check the sudo path by hand without turning the drone off:

    python3 scripts/gcs/hostpower.py          # is it available, and what runs
    sudo -n systemctl --dry-run poweroff      # systemd prints what it would do

This module never powers anything down from its own CLI.  If you are already in
a shell on the Jetson, `sudo systemctl poweroff` is shorter than any flag here.
"""
import os
import subprocess
import threading
import time

MODES = ('poweroff', 'reboot')
VERB = {'poweroff': 'powering off', 'reboot': 'rebooting'}
CMD = {
    'poweroff': ['sudo', '-n', 'systemctl', 'poweroff'],
    'reboot': ['sudo', '-n', 'systemctl', 'reboot'],
}

# The one test seam, and it can only ever make this LESS destructive: with
# GCS_POWER_DRY_RUN set the commands become a no-op that logs what it would
# have run, so the endpoint's three gates can be tested end to end -- by
# test_hostpower.py, which spawns its own server -- without any chance of
# turning a drone's computer off to prove that a refusal works.  status()
# reports it, and the page says so: a dashboard whose power buttons quietly do
# nothing would be worse than one without them.
DRY_RUN = bool(os.environ.get('GCS_POWER_DRY_RUN'))
if DRY_RUN:
    CMD = {m: ['echo', f'DRY RUN: would run {" ".join(c)}']
           for m, c in CMD.items()}

# Long enough for the JSON reply to reach the browser and be rendered. A page
# whose request dies mid-flight just says "lost the Jetson", which is exactly
# what a crash looks like too -- the one ambiguity worth two seconds.
DELAY_S = 2.0

_LOCK = threading.Lock()
_STATE = {'mode': None, 'at': 0.0, 'error': None}
# A working sudo is cached for an hour; a broken one for 30 s, because the fix
# is a sudoers line you make while looking at the page and it should show up
# without restarting the GCS.
_SUDO = {'ok': None, 'why': None, 'at': 0.0}
SUDO_TTL_OK = 3600.0
SUDO_TTL_BAD = 30.0
# Still running this long after systemd took the job means it did not take --
# an inhibitor, or a stop job sitting on its own timeout.  Say so and put the
# buttons back: a page insisting it is going down, for ever, with nothing but a
# GCS restart to clear it, is a bad thing to hand somebody at a field.
STUCK_AFTER = 120.0

NO_SUDO = ('this dashboard cannot run sudo without a password, so it cannot '
           'power the Jetson down -- do it in a shell instead: '
           'sudo systemctl poweroff')


def _probe_sudo():
    if DRY_RUN:
        return True, None
    try:
        p = subprocess.run(['sudo', '-n', 'true'], stdin=subprocess.DEVNULL,
                           capture_output=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, f'cannot run sudo here ({type(exc).__name__}: {exc})'
    return (True, None) if p.returncode == 0 else (False, NO_SUDO)


def sudo_ready():
    """Can this user run the command with no password prompt?

    Cached, because /api/state is polled at 2 Hz and each probe is a fork plus
    a line in auth.log.
    """
    now = time.time()
    with _LOCK:
        if _SUDO['ok'] is not None:
            ttl = SUDO_TTL_OK if _SUDO['ok'] else SUDO_TTL_BAD
            if now - _SUDO['at'] < ttl:
                return _SUDO['ok'], _SUDO['why']
    ok, why = _probe_sudo()
    with _LOCK:
        _SUDO.update(ok=ok, why=why, at=now)
    return ok, why


def flight_blocked(aircraft):
    """What the FC's own view of the vehicle says about this, or None.

    Takes the telemetry dict rather than reaching for it: this is the rule, not
    the plumbing, and the rule is the half that would hurt somebody if it were
    wrong -- so it is testable with no flight controller in the room.

    A never-seen FC has armed=None and does not block: that is the bench case
    the module docstring describes.  But a link that HAS been seen and said
    ARMED blocks even once it goes stale, because "we stopped hearing from the
    aircraft while it was armed" is the one shape here that could be a vehicle
    in the air with a broken link.
    """
    a = aircraft or {}
    if a.get('armed'):
        return ('the vehicle is ARMED. Disarm it first' if a.get('fresh') else
                'the last thing the FC said was ARMED and the link has since '
                'gone stale -- check the aircraft')
    if a.get('fresh') and a.get('landed') is False:
        return 'the FC reports the vehicle is IN AIR'
    return None


def pending():
    """The scheduled action, if any: {'mode', 'at', 'error'}."""
    with _LOCK:
        return dict(_STATE)


def status():
    """What /api/state carries. `pending` is what makes the page stop calling a
    dead link a lost link."""
    st = pending()
    if st['mode'] and time.time() - st['at'] > STUCK_AFTER:
        with _LOCK:
            if _STATE['mode'] and time.time() - _STATE['at'] > STUCK_AFTER:
                waited = int(time.time() - _STATE['at'])
                _STATE.update(mode=None, at=0.0, error=(
                    f'{st["mode"]} was accepted but this box is still running '
                    f'{waited}s later -- something is holding the shutdown up '
                    '(systemd-inhibit --list, or journalctl -b)'))
        st = pending()
    ok, _why = sudo_ready()
    return {
        'available': ok,
        'dry_run': DRY_RUN,
        'pending': st['mode'],
        'in_s': (None if not st['mode']
                 else round(max(0.0, st['at'] - time.time()), 1)),
        'error': st['error'],
        'modes': list(MODES),
    }


def schedule(mode, log=None, delay=DELAY_S):
    """Run the command after `delay`, so the HTTP reply gets out first.

    Returns (ok, message) about the SCHEDULING; whether systemd accepted the
    job lands in the log and, on failure, in status()['error'] -- by which time
    this response is long delivered.
    """
    if mode not in MODES:
        return False, f'unknown power action {mode!r}'
    ok, why = sudo_ready()
    if not ok:
        return False, why
    with _LOCK:
        if _STATE['mode']:
            return False, f'a {_STATE["mode"]} is already scheduled'
        _STATE.update(mode=mode, at=time.time() + delay, error=None)
    threading.Thread(target=_run, args=(mode, delay, log),
                     name=f'hostpower-{mode}', daemon=True).start()
    return True, (f'{VERB[mode]} the Jetson in {delay:g} s -- '
                  + ('it will come back on its own'
                     if mode == 'reboot'
                     else 'someone has to power it back on by hand'))


def _run(mode, delay, log):
    say = log or (lambda m: print(m, flush=True))
    time.sleep(delay)
    say(f'[gcs] {" ".join(CMD[mode])}')
    try:
        p = subprocess.run(CMD[mode], stdin=subprocess.DEVNULL,
                           capture_output=True, timeout=60)
        err = None if p.returncode == 0 else (
            (p.stderr or b'').decode('utf-8', 'replace').strip()
            or f'exit {p.returncode}')
    except (OSError, subprocess.SubprocessError) as exc:
        err = f'{type(exc).__name__}: {exc}'
    if err is None:
        # systemctl returns as soon as systemd has queued the job, so the box
        # is on its way down and nothing after this line is guaranteed to run.
        # `mode` is left set on purpose: the page keeps saying "going down"
        # rather than reporting the link it is about to lose as a fault.
        return say(f'[gcs] {mode} accepted by systemd -- going down')
    with _LOCK:
        _STATE.update(mode=None, at=0.0, error=f'{mode} failed: {err}')
    say(f'[gcs] {mode} FAILED: {err}')


if __name__ == '__main__':
    ok, why = sudo_ready()
    if DRY_RUN:
        print('GCS_POWER_DRY_RUN is set: nothing here would actually run')
    print(f'sudo without a password: {"yes" if ok else "NO"}')
    if why:
        print(f'  {why}')
    for m in MODES:
        print(f'  {m:9s} -> {" ".join(CMD[m])}')
    print('\nThis CLI reports only. To do it: sudo systemctl poweroff|reboot')
