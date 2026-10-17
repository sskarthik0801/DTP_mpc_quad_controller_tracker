#!/usr/bin/env python3
"""Host stats straight from /proc -- psutil is not installed on this Jetson.

Swap is here on purpose: swap != 0 B is the signature of the FAST-LIO RAM leak
this repo chased for a session (CLAUDE.md, M1 regression), so it belongs on the
dashboard next to RAM rather than buried.
"""
import os
import time
from typing import NamedTuple

PAGE_KB = os.sysconf('SC_PAGE_SIZE') // 1024
TICKS = os.sysconf('SC_CLK_TCK')

_cpu_prev = {}      # pid -> (ticks, monotonic) for CPU% between polls
_scan = {'t': 0.0, 'rows': []}
SCAN_TTL = 0.8      # seconds; a /proc walk is ~750 file reads on this box
CPU_HISTORY_S = 60  # forget a pid's CPU baseline this long after its last sample


def _stat_fields(pid):
    """Parse /proc/<pid>/stat.  comm can contain spaces and ')', so split on
    the LAST ')' -- field 1 is pid, field 2 is comm, fields 3+ follow."""
    with open(f'/proc/{pid}/stat', 'rb') as f:
        raw = f.read().decode('utf-8', 'replace')
    rest = raw[raw.rindex(')') + 2:].split()
    return rest       # rest[0] is field 3 (state); field N is rest[N-3]


class Proc(NamedTuple):
    """One row of scan().  ppid/sid are here for adopt-and-stop: deciding
    whether a process group may be signalled needs the group leader's session,
    and killing a stray without its group needs the child list."""
    pid: int
    ppid: int
    pgid: int
    sid: int
    state: str
    bases: frozenset


def scan(ttl=SCAN_TTL):
    """One walk of /proc, cached.

    Every caller here used to walk /proc for itself, so a single /api/state --
    four processes, each asking "is something of mine running?" -- cost four
    full walks of 376 processes, eight per second at the poll rate.  That was
    9% of a core spent on nothing.  One shared, cached pass instead.

    Returns a list of Proc.
    """
    now = time.monotonic()
    if now - _scan['t'] < ttl:
        return _scan['rows']
    rows = []
    me = os.getpid()
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            f = _stat_fields(name)
            state, ppid, pgid, sid = f[0], int(f[1]), int(f[2]), int(f[3])
        except (OSError, ValueError, IndexError):
            continue
        bases = frozenset() if pid == me else frozenset(
            os.path.basename(t) for t in argv(name) if _names_a_program(t))
        rows.append(Proc(pid, ppid, pgid, sid, state, bases))
    _scan.update(t=now, rows=rows)
    return rows


def pids_in_group(pgid):
    """Every live pid whose process group is pgid.

    A zombie is not running: counting one would stall a stop's wait loop into
    needless SIGKILL escalation while its parent gets round to reaping it.
    """
    return [p.pid for p in scan() if p.pgid == pgid and p.state != 'Z']


def tree_usage(pids):
    """(rss_mb, cpu_percent) summed over pids.  CPU is measured between calls,
    so the first call for a pid returns 0.0 -- that is correct, not a bug."""
    now = time.monotonic()
    rss_kb = 0
    cpu = 0.0
    for pid in pids:
        try:
            f = _stat_fields(pid)
            utime, stime = int(f[11]), int(f[12])     # fields 14, 15
            rss_kb += int(f[21]) * PAGE_KB            # field 24, in pages
        except (OSError, ValueError, IndexError):
            continue
        ticks = utime + stime
        prev = _cpu_prev.get(pid)
        _cpu_prev[pid] = (ticks, now)
        if prev and now > prev[1]:
            cpu += (ticks - prev[0]) / TICKS / (now - prev[1]) * 100.0
    # Age entries out by time, never by "absent from this call": each managed
    # process asks about its own pids, so pruning to the current call would
    # make every process wipe the others' baseline and report 0% forever.
    for stale in [k for k, (_t, when) in _cpu_prev.items()
                  if now - when > CPU_HISTORY_S]:
        _cpu_prev.pop(stale, None)
    return round(rss_kb / 1024.0, 1), round(cpu, 1)


def _meminfo():
    out = {}
    with open('/proc/meminfo') as f:
        for line in f:
            k, _, v = line.partition(':')
            out[k] = int(v.split()[0])      # kB
    return out


def host():
    m = _meminfo()
    load = os.getloadavg()
    st = os.statvfs('/')
    mem_total = m['MemTotal'] / 1048576.0
    swap_total = m.get('SwapTotal', 0) / 1048576.0
    return {
        'cores': os.cpu_count(),
        'load1': round(load[0], 2),
        'load_pct': round(load[0] / max(os.cpu_count(), 1) * 100, 0),
        'mem_total_gb': round(mem_total, 1),
        'mem_used_gb': round(mem_total - m['MemAvailable'] / 1048576.0, 1),
        'swap_total_gb': round(swap_total, 1),
        # The number that matters: anything above 0 means we are paging.
        'swap_used_gb': round(swap_total - m.get('SwapFree', 0) / 1048576.0, 2),
        'disk_free_gb': round(st.f_bavail * st.f_frsize / 1073741824.0, 1),
        'uptime_s': int(float(open('/proc/uptime').read().split()[0])),
    }


def argv(pid):
    """/proc/<pid>/cmdline split into its real argv tokens."""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            raw = f.read().decode('utf-8', 'replace')
    except OSError:
        return []
    return [t for t in raw.split('\0') if t]


def cmdline(pid):
    return ' '.join(argv(pid))


def _names_a_program(token):
    """True if this argv token is plausibly a program or script path.

    The guard that matters: a shell invoked as `bash -c '<script>'` carries the
    whole script as ONE argv token, and that script may well mention the very
    program we are looking for.  Matching a pattern against a flattened command
    line is exactly the mistake behind `pkill -f` killing the shell that ran it
    (CLAUDE.md), and it bit this file during development -- the curl command
    testing the API contained "offboard_viz.py", so the test shell was reported
    as a running node.  Real program paths have no whitespace and are short.
    """
    return len(token) <= 512 and not any(c.isspace() for c in token)


def find_by_cmdline(patterns, exclude_pgids=()):
    """pids running one of these programs.

    `patterns` are basenames (`fsm_node`, `px4_odom_bridge.py`); a pid matches
    only if some argv token's BASENAME equals one exactly.  That catches both
    a compiled node (argv[0] = .../lib/super_planner/fsm_node) and a Python
    node started by `ros2 run` (argv = [python3, .../px4_odom_bridge.py]),
    while never matching a shell that merely mentions the name.
    """
    exclude_pgids = set(exclude_pgids)
    hits = {}
    for p in scan():
        if not p.bases or p.pgid in exclude_pgids or p.state == 'Z':
            continue
        for pat in patterns:
            if pat in p.bases:
                hits.setdefault(pat, []).append(p.pid)
    return hits


# ---------------------------------------------------------------------------
# Adopt-and-stop: signalling processes this server did NOT start.
#
# The house rule is "signal a process group you own, never a name you matched"
# -- it is why pkill -f keeps killing the shell that ran it (CLAUDE.md).  This
# section is the one place that has to reach outside what we own, so it keeps
# the half of the rule that actually protects anything: the MATCHING is still
# find_by_cmdline's argv-basename match, never a flattened command line.
#
# What is new is the escalation from a matched pid to its process group, and
# that is where a shell gets killed if you are careless.  A job started under
# job control (`bash scripts/run.sh` at a prompt) gets its own group whose
# leader is run.sh, so signalling the group takes down run.sh, ros2 launch,
# fastlio_mapping, fsm_node and the Livox driver in one go -- exactly what
# Ctrl-C in that terminal would have done.  A process started WITHOUT job
# control sits in the shell's own group, and signalling that group kills the
# shell.
#
# So the group is only adopted when its LEADER is recognisably ours: the
# leader's own argv basenames must contain one of the patterns we matched, or
# a wrapper that starts them.  A login shell's argv is just {bash}, which
# matches neither, so it can never be adopted.  When the group is refused the
# matched pids are still stopped individually, together with their children --
# less tidy, never dangerous.
# ---------------------------------------------------------------------------

# Leaders that count as "ours" beyond the node names themselves: run.sh is the
# stack's entry point, and `ros2 run` / `ros2 launch` are argv[1] of the python
# wrapper that starts every node here.
WRAPPERS = frozenset({'run.sh', 'ros2'})


def descendants(pids, rows=None):
    """Every live descendant of these pids, at any depth."""
    rows = scan() if rows is None else rows
    kids = {}
    for p in rows:
        kids.setdefault(p.ppid, []).append(p.pid)
    out, stack = set(), list(pids)
    while stack:
        for child in kids.get(stack.pop(), ()):
            if child not in out:
                out.add(child)
                stack.append(child)
    return out - set(pids)


def kill_targets(pids, patterns=()):
    """Split matched pids into (process groups, individual pids) to signal.

    A group is only returned when its leader is recognisably one of ours --
    see the note above.  Everything else falls back to individual pids plus
    their descendants, which cannot take a shell with it.
    """
    rows = scan(ttl=0.0)
    by_pid = {p.pid: p for p in rows}
    ours = frozenset(patterns) | WRAPPERS
    mine = {os.getpgrp(), os.getsid(0), 0, 1}

    groups, loose = set(), set()
    for pid in pids:
        proc = by_pid.get(pid)
        if proc is None:
            continue
        leader = by_pid.get(proc.pgid)
        if (proc.pgid not in mine and leader is not None
                and leader.bases & ours):
            groups.add(proc.pgid)
        else:
            loose.add(pid)
    loose |= descendants(loose, rows)
    # Anything already covered by an adopted group needs no separate signal.
    loose = {pid for pid in loose
             if by_pid.get(pid) is not None and by_pid[pid].pgid not in groups}
    return sorted(groups), sorted(loose)


def survivors(pgids, pids, ttl=0.3):
    """Which of these groups/pids still have a live, non-zombie process.

    Groups are re-read rather than remembered: SIGINT to a group can spawn
    nothing but can certainly leave a child that was not in the original match.
    """
    pgids, pids = set(pgids), set(pids)
    return sorted(p.pid for p in scan(ttl)
                  if p.state != 'Z' and (p.pgid in pgids or p.pid in pids))
