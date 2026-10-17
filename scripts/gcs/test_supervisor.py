#!/usr/bin/env python3
"""Test the adopt-and-stop rules -- the part of the GCS that signals processes
it did not start.

    python3 scripts/gcs/test_supervisor.py

Needs no ROS, no flight controller and no running GCS: it spawns its own
harmless stand-ins under a pattern name (`gcs_test_node.py`) that matches
nothing real, so it is safe to run with the stack up.

What it is actually protecting.  Escalating from a matched pid to its process
group is what makes "Stop outside nodes" able to clean up a stack started by
hand -- and it is also, done carelessly, how you kill the operator's shell.
The rule is that a group is adopted only when its LEADER is recognisably ours.
These cases are the three shapes that show up in practice:

  1. `ros2 run ... node` with its own session   -> leader IS the node    -> adopt
  2. `bash scripts/run.sh` as a shell job       -> leader is run.sh      -> adopt
  3. a node started from a `bash -c '...'`      -> leader is a SHELL     -> refuse
                                                   the group, kill the pid

Case 3 is the one with teeth, so it asserts both halves: the node dies and the
shell is still there afterwards.

Case 5 covers the other half of the rule -- WHEN adopting is allowed at all.
A row's own Stop button may adopt; a sweep (the Ctrl-C teardown, and restart)
may not, or closing the dashboard kills a stack it never started.
"""
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import supervisor as sup       # noqa: E402
import sysinfo                 # noqa: E402

PATTERN = 'gcs_test_node.py'   # deliberately matches nothing in this repo
NODE = '''import signal, sys, time
# Exits on SIGINT like a real node, so the first escalation step is enough and
# the test does not sit through the SIGTERM/SIGKILL grace periods.
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
time.sleep(120)
'''
RUN_SH = '''#!/bin/bash
trap 'kill 0' EXIT
python3 "$(dirname "$0")/gcs_test_node.py" &
wait
'''

fails = []
log_lines = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  -- {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def log(line):
    log_lines.append(line)


def alive(pid):
    rows = {p.pid: p for p in sysinfo.scan(ttl=0.0)}
    return pid in rows and rows[pid].state != 'Z'


def matched(tmp):
    """The pids find_by_cmdline would report -- the same matcher the GUI uses."""
    hits = sysinfo.find_by_cmdline([PATTERN])
    sysinfo.scan(ttl=0.0)
    return sorted({p for pids in hits.values() for p in pids})


def wait_for(fn, secs=5.0):
    end = time.time() + secs
    while time.time() < end:
        if fn():
            return True
        time.sleep(0.1)
    return False


def main():
    tmp = tempfile.mkdtemp(prefix='gcs-test-')
    node = os.path.join(tmp, PATTERN)
    runsh = os.path.join(tmp, 'run.sh')
    with open(node, 'w') as f:
        f.write(NODE)
    with open(runsh, 'w') as f:
        f.write(RUN_SH)
    os.chmod(runsh, 0o755)
    procs = []

    def spawn(argv, **kw):
        p = subprocess.Popen(argv, cwd=tmp, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             **kw)
        procs.append(p)
        return p

    try:
        # -- case 1: the node is its own group leader -----------------------
        print('\ncase 1: node in its own session (like `ros2 run` from a terminal)')
        p1 = spawn(['python3', node])
        wait_for(lambda: p1.pid in matched(tmp))
        groups, loose = sysinfo.kill_targets([p1.pid], [PATTERN])
        check('its process group is adopted', groups == [p1.pid],
              f'groups={groups} loose={loose}')
        ok, msg = sup.stop_foreign([p1.pid], [PATTERN], log)
        check('stop_foreign reports success', ok, msg)
        check('the node is gone', wait_for(lambda: not alive(p1.pid)))

        # -- case 2: run.sh is the group leader -----------------------------
        print('\ncase 2: `bash run.sh` as a shell job, node is its child')
        p2 = spawn(['bash', runsh])
        wait_for(lambda: len(matched(tmp)) == 1)
        kid = matched(tmp)
        check('the child node is found by the argv matcher', len(kid) == 1, str(kid))
        groups, loose = sysinfo.kill_targets(kid, [PATTERN])
        check('run.sh\'s group is adopted, not just the child',
              groups == [p2.pid], f'groups={groups} vs run.sh pgid {p2.pid}')
        ok, msg = sup.stop_foreign(kid, [PATTERN], log)
        check('stop_foreign reports success', ok, msg)
        check('run.sh went too, not just its child',
              wait_for(lambda: not alive(p2.pid) and not alive(kid[0])))

        # -- case 3: THE ONE THAT MATTERS -----------------------------------
        print('\ncase 3: node started from `bash -c` -- the group leader is a SHELL')
        p3 = spawn(['bash', '-c', f'python3 {node} & sleep 120'])
        wait_for(lambda: len(matched(tmp)) == 1)
        kid = matched(tmp)
        check('the node is found', len(kid) == 1, str(kid))
        rows = {p.pid: p for p in sysinfo.scan(ttl=0.0)}
        check('it really is sharing the shell\'s process group',
              kid and rows[kid[0]].pgid == p3.pid,
              f'node pgid={rows[kid[0]].pgid if kid else "?"} shell pid={p3.pid}')
        groups, loose = sysinfo.kill_targets(kid, [PATTERN])
        check('the shell\'s group is REFUSED', groups == [],
              f'groups={groups} -- adopting this would kill the shell')
        check('the node is targeted individually instead', loose == kid, str(loose))
        ok, msg = sup.stop_foreign(kid, [PATTERN], log)
        check('stop_foreign reports success', ok, msg)
        check('the node is gone', wait_for(lambda: not alive(kid[0])))
        check('the SHELL SURVIVED', alive(p3.pid),
              'a stop that takes the operator\'s terminal with it is the bug '
              'this whole rule exists to prevent')

        # -- case 4: our own group is never a target ------------------------
        print('\ncase 4: this process\'s own group')
        groups, loose = sysinfo.kill_targets([os.getpid()], [PATTERN])
        check('never adopts our own process group', os.getpgrp() not in groups,
              f'groups={groups}')

        # -- case 5: a SWEEP must not adopt ---------------------------------
        # The regression this exists for: Ctrl-C on the GCS calls
        # stop_all(external=False), which walks every row -- and every row's
        # stop() used to fall through to stop_foreign() when it had nothing of
        # its own running. Closing the dashboard therefore killed a stack
        # started in somebody's terminal, which is precisely what stop_all()'s
        # own docstring promises it will not do. Observed for real: a test GCS
        # shut down and took a hand-started Livox driver, FAST-LIO and three
        # PX4 bridges with it.
        print('\ncase 5: a sweep leaves outside processes alone; a row Stop does not')
        p5 = spawn(['python3', node])
        wait_for(lambda: p5.pid in matched(tmp))
        row = sup.Managed('test', 'test row', ['true'], {}, [PATTERN])
        ok, msg = row.stop(adopt=False)
        check('the sweep declines to touch it', not ok and msg == 'not running', msg)
        check('...and it is still alive', alive(p5.pid),
              'this is the shutdown path -- it must stop only what it started')
        ok, msg = row.stop(adopt=True)
        check('the row\'s own Stop button still adopts it', ok, msg)
        check('...and it is gone', wait_for(lambda: not alive(p5.pid)))

    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), 9)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                p.wait(timeout=2)
            except Exception:
                pass
        for f in (node, runsh):
            try:
                os.unlink(f)
            except OSError:
                pass
        try:
            os.rmdir(tmp)
        except OSError:
            pass

    print('\n--- stop_foreign log ---')
    for line in log_lines:
        print('  ' + line)
    print(f'\n{len(fails)} FAILED: {fails}' if fails else '\nall passed')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
