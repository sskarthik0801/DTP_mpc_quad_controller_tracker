#!/usr/bin/env python3
"""Start/stop the stack and the bridges as process GROUPS.

Why groups.  `scripts/run.sh` backgrounds the Livox driver and foregrounds the
M1 launch; its EXIT trap kills only the driver PID, so killing run.sh from
another session leaves fastlio_mapping / fsm_node / livox_ros_driver2_node
alive.  That is also why CLAUDE.md keeps warning about `pkill -f` -- the
pattern matches the killing shell's own argv.  Here every child gets its own
session (start_new_session=True) and is stopped with os.killpg(), so neither
problem can happen: we signal exactly the tree we started, by pgid, never by
name.

SIGINT first (not SIGTERM), so run.sh's own trap and ros2 launch's shutdown
both run; escalate only if they do not exit.

stop_foreign() is the one place that reaches outside what we started -- the
stack begun by hand in a terminal, which the dashboard could previously only
report and not stop.  It keeps the half of the rule that protects anything:
the matching is still sysinfo.find_by_cmdline's argv-basename match, and a
process group is only adopted when its leader is recognisably ours.  See the
long note above sysinfo.kill_targets.
"""
import os
import signal
import subprocess
import threading
import time
from collections import deque
from typing import NamedTuple

import bagger
import dtp
import sysinfo

LOG_LINES = 500          # per-process ring buffer
RESTART_SETTLE = 3.0     # after a stop, before starting again
RESTART_STACK_WAIT = 6.0 # the stack needs a head start before the bridges
# Canonical start order. Stop order is the reverse: the bridges consume the
# stack's topics, so they come down first and go up last -- and the DTP
# follower, which consumes everything and commands SUPER, comes down before
# any of them.
ORDER = ('stack', 'odom_bridge', 'offboard_bridge', 'offboard_viz', 'dtp',
         'camera', 'claude')
# Rows that are never brought back by Restart.  See Supervisor.restart().
NO_RESTART = ('dtp',)
# Rows Restart neither stops nor starts: they are not part of the stack, and
# cycling the Claude session would drop whoever is driving it from a phone.
# Stop, STOP ALL and the server's own shutdown still stop them.
RESTART_IGNORES = ('claude',)
# Generous, deliberately: fsm_node holds a ~2.3 GB A* preallocation and
# FAST-LIO an ikd-Tree, so a graceful unwind under load can take >8 s. One run
# out of four needed SIGKILL at 8 s, and a SIGKILLed ROS 2 node can leave DDS
# shared-memory segments in /dev/shm. Waiting longer is the cheaper mistake.
SIGINT_GRACE = 15.0
SIGTERM_GRACE = 5.0

# M5.3 (2026-09-13): the bridges share the six cores everything but FAST-LIO runs on
# (run.sh STACK_CPUS / dtp_follower.sh DTP_CPUS = 0-5, FAST-LIO on 6,7). taskset execs
# the real command in the same pid, so the row's leader still carries ros2's argv and
# find_by_cmdline / group adoption see exactly what they saw before.
BRIDGE_CPUS = '0-5' if (os.cpu_count() or 0) >= 8 else ''


def _pinned(argv):
    return (['taskset', '-c', BRIDGE_CPUS] if BRIDGE_CPUS else []) + argv


# Which node the 'offboard_bridge' row runs (docs/mpc-tracker-integration.md). Chosen once,
# when the GCS starts: GCS_TRACKER=mpc runs px4_mpc_tracker.py, anything else (the default)
# runs px4_offboard_bridge.py. The row keeps its key, so the Flight day card, ORDER and the
# tests do not change. Its patterns name BOTH nodes: they must never run together (two
# publishers on /fmu/in/trajectory_setpoint), so whichever is running counts as this row.
TRACKER = 'mpc' if os.environ.get('GCS_TRACKER', '').strip().lower() == 'mpc' else 'bridge'
_TRAJECTORY_ROW = {
    'bridge': ('M3 trajectory bridge (SUPER -> PX4)', 'px4_offboard_bridge.py'),
    'mpc': ('MPC trajectory tracker (SUPER -> PX4 acceleration)', 'px4_mpc_tracker.py'),
}[TRACKER]


# id -> (label, argv, extra env, argv substrings that prove it is running)
PROCS = {
    'stack': (
        'SUPER stack (Livox + FAST-LIO + fsm_node)',
        ['bash', 'scripts/run.sh'],
        {'RVIZ': 'false'},
        ['fastlio_mapping', 'fsm_node', 'livox_ros_driver2_node'],
    ),
    'odom_bridge': (
        'M2 odometry bridge (FAST-LIO -> PX4)',
        _pinned(['ros2', 'run', 'super_bringup', 'px4_odom_bridge.py']), {},
        ['px4_odom_bridge.py'],
    ),
    'offboard_bridge': (
        _TRAJECTORY_ROW[0],
        _pinned(['ros2', 'run', 'super_bringup', _TRAJECTORY_ROW[1]]), {},
        ['px4_offboard_bridge.py', 'px4_mpc_tracker.py'],
    ),
    'offboard_viz': (
        'Offboard setpoint markers (for laptop RViz)',
        ['ros2', 'run', 'super_bringup', 'offboard_viz.py'], {},
        ['offboard_viz.py'],
    ),
    # The human-following stack from drone-dtp/, riding on the row above it.
    # Its mode (observe | live) is chosen per start -- see start_options().
    #
    # The patterns are the DTP nodes' own executables, and deliberately NOT the
    # Orbbec driver: that runs as a generic rclcpp `component_container`, which
    # any other launch may be using, and matching it would let STOP ALL take
    # down somebody else's camera -- the camera row's rule, below.  Stopping
    # the follower's process group takes its camera driver with it anyway.
    'dtp': (
        'DTP follower (perception, target_manager, follow_planner, command_mux)',
        ['bash', 'scripts/dtp_follower.sh'], {},
        ['dtp_follower.sh', 'perception_node', 'target_manager',
         'follow_planner_node', 'command_mux_node', 'fusion_node',
         'static_calib_tf', 'sensor_watchdog', 'dtp_viz', 'dtp_image_overlay'],
    ),
    # Independent of the flight stack: it neither feeds nor reads anything the
    # planner uses, so it is last in ORDER and first to be stopped.
    #
    # The patterns name OUR launcher only, deliberately.  A Gemini can be
    # opened by exactly one process, and on this Jetson it is routinely already
    # owned by somebody else's `ros2 launch orbbec_camera
    # gemini_330_series.launch.py` under the /camera namespace.  Matching that
    # too would make STOP ALL quietly kill another team's perception stack, so
    # this row only ever adopts a camera started from THIS repo.  A foreign
    # owner is reported by the camera panel instead (camera.py finds it by
    # publisher count), and blocks Start rather than being killed.
    'camera': (
        'Gemini 336L camera (color feed for the dashboard)',
        ['bash', 'scripts/gemini_camera.sh'], {},
        ['gemini_camera.sh', 'gemini_camera.launch.py'],
    ),
    # Claude Code Remote Control, on the PERSONAL profile (~/.claude-personal),
    # driven from the Claude app or claude.ai/code.  Nothing in the flight
    # stack reads or feeds it.  Behind Admin control (server.py): a session
    # here is a shell on the flight computer, which is far more than this
    # dashboard lets anyone do.
    #
    # The pattern is the launcher's basename ONLY.  `claude` would match every
    # Claude Code session on this box -- including one run from a terminal --
    # and STOP ALL would kill it.
    'claude': (
        'Claude Remote Control (personal profile, via the Claude app)',
        ['bash', 'scripts/claude_remote.sh'], {},
        ['claude_remote.sh'],
    ),
}

# One-shot diagnostics.  All read-only.  Nothing here can arm, change a mode,
# or write a parameter -- clear_fault_logs is listed WITHOUT --clear on purpose.
class Check(NamedTuple):
    label: str
    argv: list
    timeout: float
    timeout_ok: bool = False    # hitting the timeout is the normal ending
    destructive: bool = False   # the UI must confirm before running it


CHECKS = {
    # ~3 min since the yaw servo checks (2026-09-29: 166-171 s with round two's Y2 block); it
    # was ~50 s with a 150 s timeout.
    'm3_bridge': Check('Bridge bench gate (expect 120/120)',
                       ['python3', 'scripts/px4/m3_bridge_check.py'], 300),
    # G0.2. Read-only (subscribes, reads /proc) and 60 s by design: "no EKF2 reset in the
    # last 60 s" cannot be answered faster. Run before every arm -- on 2026-09-12 every
    # health readout on this page said fine while FAST-LIO was 3.74 km wrong.
    'estimator': Check('Estimator preflight before arming (60 s)',
                       ['python3', 'scripts/px4/estimator_preflight.py'], 120),
    'probe_params': Check('Probe FC params',
                          ['python3', 'scripts/px4/probe_params.py'], 90),
    'ekf2_show': Check('EKF2 profile (--show)',
                       ['python3', 'scripts/px4/ekf2_profile.py', '--show'], 60),
    'diag_health': Check('FC health / battery / STATUSTEXT',
                         ['python3', 'scripts/px4/diag_health.py'], 60),
    'faults': Check('Scan FC SD card for hardfaults',
                    ['python3', 'scripts/px4/clear_fault_logs.py'], 120),
    # The only check that changes anything. It is safe by construction --
    # clear_fault_logs.py copies each dump into firmware/faults/ and verifies
    # the local copy BEFORE removing it from the card -- but a hardfault log is
    # the only evidence the FC reset itself, so the UI confirms first.
    # Never pass --no-archive here: that path prompts on stdin, which does not
    # exist for a subprocess, and it deletes the only copy.
    'faults_clear': Check('Archive + clear FC crash dumps',
                          ['python3', 'scripts/px4/clear_fault_logs.py', '--clear'],
                          180, destructive=True),
    'xrce': Check('uxrce_dds_client status on the FC',
                  ['python3', 'scripts/px4/px4_shell.py', 'uxrce_dds_client status'], 60),
    # ---- field-day checks. All read-only. The Flight day card (flightday.py) presses
    # field_check, stage_a_preflight, live_preflight, estimator and faults; the rest are
    # chips on the Checks card only.
    'desk': Check('Desk check (log loader, disk, clocks, swap, links)',
                  ['bash', 'scripts/desk_check.sh'], 30),
    'rc_switches': Check('RC switches on the FC (transmitter must be on)',
                         ['python3', 'scripts/px4/px4_shell.py', 'listener manual_control_switches'], 45),
    # field.sh sources its own ROS env. `check` = the FAST-LIO gates only:
    # foreign subscribers on the cloud, stamp lag. ~15 s.
    'field_check': Check('FAST-LIO gates: foreign cloud subscribers, stamp lag',
                         ['bash', 'drone-dtp/scripts/field.sh', 'check'], 60),
    # The Stage A / Stage B preflights carry their own 60 s estimator window.
    'stage_a_preflight': Check('Stage A preflight (observe-only mode, ~90 s)',
                               ['python3', 'drone-dtp/scripts/stage_a_preflight.py'], 180),
    'stage_b_preflight_mannequin': Check('Stage B preflight, --target mannequin (props off)',
                                         ['python3', 'drone-dtp/scripts/stage_a_preflight.py',
                                          '--stage-b', '--target', 'mannequin'], 180),
    'stage_b_preflight': Check('Stage B preflight, --target person',
                               ['python3', 'drone-dtp/scripts/stage_a_preflight.py',
                                '--stage-b', '--target', 'person'], 180),
    # What the Flight day card's Live path runs: the flags deploy_follow.sh passes, which are
    # the operator's two recorded decisions -- max_vel 3.0 (follow_test_ros2.yaml since
    # 2026-09-26; keep the three in step) and the single-IMU waiver. The strict one above
    # FAILs on both every run, so it cannot be the standard gate for a live follower.
    'live_preflight': Check('Live follower preflight (as deploy_follow.sh: --max-vel 3.0 --waive-d21)',
                            ['python3', 'drone-dtp/scripts/stage_a_preflight.py',
                             '--stage-b', '--target', 'person',
                             '--max-vel', '3.0', '--waive-d21'], 180),
    # ros2 topic hz never exits on its own -- the timeout IS how it finishes,
    # so its non-zero exit code is not a failure and must not read like one.
    'cloud_hz': Check('Deep check: /cloud_registered rate',
                      ['ros2', 'topic', 'hz', '/cloud_registered', '--window', '20'],
                      12, timeout_ok=True),
}

# Not read-only, and deliberately NOT offered as a check chip: each of these
# has its own endpoint, its own confirmation and its own interlock, so none of
# them can be reached by the generic /api/check/<name> route.  They still run
# through the same single-slot runner as the checks, because they are MAVLink
# clients too and only one process at a time may hold udpin:127.0.0.1:14569.
ACTIONS = {
    'fc_reboot': Check('Reboot the flight controller',
                       # --yes because a subprocess has no stdin to prompt on.
                       # fc_reboot.py refuses while armed or in air on its own;
                       # the GUI refuses again from live telemetry first.
                       ['python3', 'scripts/px4/fc_reboot.py', '--yes'],
                       90, destructive=True),
}


def start_options(key, opts=None):
    """(opts, extra env) for one start of `key`, or ValueError with a sentence.

    Two rows take a choice, and both are whitelists rather than strings passed
    through: the Stack row's config ends up in a path inside run.sh, and the
    follower's mode decides whether it can command the aircraft.  Every other
    row takes nothing, and saying so beats silently ignoring a query string.

    The follower has NO default mode.  Which of the two you get is the one
    thing about it that must never be decided by omission.
    """
    opts = {k: v for k, v in (opts or {}).items() if v not in (None, '')}
    if key == 'stack':
        choice = opts.pop('config', 'click')
        if opts:
            raise ValueError(f'the stack takes only config=, not {sorted(opts)}')
        if choice not in dtp.STACK_CONFIGS:
            raise ValueError(f'unknown stack config {choice!r} -- it is one of '
                             + ', '.join(dtp.STACK_CONFIGS))
        return ({'config': choice},
                {'SUPER_CONFIG': dtp.STACK_CONFIGS[choice]})
    if key == 'dtp':
        mode = opts.pop('mode', None)
        config = opts.pop('config', None)
        if opts:
            raise ValueError(f'the follower takes mode= and config=, not {sorted(opts)}')
        if mode not in dtp.MODES:
            raise ValueError('say which mode to start the follower in: '
                             + ' or '.join(dtp.MODES))
        if config is not None and config not in dtp.STACK_CONFIGS.values():
            raise ValueError(f'unknown SUPER config {config!r}')
        return ({'mode': mode, 'config': config},
                {'DTP_MODE': mode, 'SUPER_CONFIG': config or ''})
    if opts:
        raise ValueError(f'{key} takes no options')
    return {}, {}


def stop_foreign(pids, patterns, log):
    """Adopt and stop processes this server did not start.

    Same escalation as Managed._stop -- SIGINT, then SIGTERM, then SIGKILL --
    because these are the same ROS 2 nodes and need the same grace: SIGINT lets
    run.sh's EXIT trap and ros2 launch's shutdown run, and fsm_node's ~2.3 GB
    unwind is not instant.  Liveness comes from /proc rather than waitpid():
    you cannot wait on a process that is not your child.
    """
    groups, loose = sysinfo.kill_targets(pids, patterns)
    live = sysinfo.survivors(groups, loose, ttl=0.0)
    if not live:
        return False, 'they exited before we could signal them'
    log(f'[gcs] adopting {len(live)} process(es) started outside this GUI: '
        f'group(s) {groups or "none"}, loose pid(s) {loose or "none"}')
    n = len(live)

    def blast(sig):
        for pgid in groups:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                log(f'[gcs] not permitted to signal group {pgid} '
                    '(started by another user?)')
        for pid in sysinfo.survivors((), loose, ttl=0.0):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                log(f'[gcs] not permitted to signal pid {pid}')

    for sig, grace in ((signal.SIGINT, SIGINT_GRACE),
                       (signal.SIGTERM, SIGTERM_GRACE)):
        if not sysinfo.survivors(groups, loose):
            break
        log(f'[gcs] {sig.name} -> {len(sysinfo.survivors(groups, loose))} '
            'outside process(es)')
        blast(sig)
        deadline = time.time() + grace
        while time.time() < deadline:
            if not sysinfo.survivors(groups, loose):
                log('[gcs] outside processes stopped cleanly')
                return True, f'stopped {n} process(es) started outside this GUI'
            time.sleep(0.2)
    if sysinfo.survivors(groups, loose):
        log('[gcs] SIGKILL -> outside process(es)')
        blast(signal.SIGKILL)
        time.sleep(0.5)
    left = sysinfo.survivors(groups, loose, ttl=0.0)
    if left:
        # Almost always a permission problem: everything else dies to SIGKILL.
        log(f'[gcs] still alive after SIGKILL: {left}')
        return False, (f'{len(left)} process(es) survived SIGKILL (pid {left[0]}) '
                       '-- stop them in the terminal that started them')
    return True, f'killed {n} process(es) started outside this GUI'


class Managed:
    def __init__(self, pid_key, label, argv, env, patterns):
        self.owner = None
        self.key, self.label = pid_key, label
        self.argv, self.env, self.patterns = argv, env, patterns
        self.proc = None
        self.pgid = None
        self.started = None
        self.opts = {}            # the choices the running copy was started with
        self.log = deque(maxlen=LOG_LINES)
        self.seq = 0
        self.lock = threading.Lock()
        # start()/stop() mutate self.proc; status() runs concurrently on every
        # HTTP poll. Serialise the mutators, and have every reader take a local
        # snapshot -- self.proc can go None between two statements otherwise.
        self._op = threading.Lock()
        self._stopping = False

    # -- log ring ----------------------------------------------------------
    def _append(self, line):
        with self.lock:
            self.seq += 1
            self.log.append((self.seq, line.rstrip('\n')))

    def logs(self, since=0):
        with self.lock:
            rows = [(s, t) for s, t in self.log if s > since]
        return {'lines': [t for _s, t in rows],
                'seq': rows[-1][0] if rows else since}

    def _pump(self, stream):
        try:
            for line in iter(stream.readline, ''):
                self._append(line)
        except Exception as exc:
            self._append(f'[gcs] log reader stopped: {exc}')
        finally:
            try:
                stream.close()
            except Exception:
                pass

    # -- lifecycle ---------------------------------------------------------
    def alive(self):
        proc = self.proc
        return proc is not None and proc.poll() is None

    def start(self, cwd, opts=None):
        with self._op:
            return self._start(cwd, opts)

    def _start(self, cwd, opts=None):
        if self.alive():
            return False, 'already running'
        ext = self.external_pids()
        if ext:
            return False, (f'already running outside this GUI (pid {ext[0]}). '
                           'Stop it in its own terminal first.')
        try:
            opts, extra = start_options(self.key, opts)
        except ValueError as exc:
            return False, str(exc)
        env = dict(os.environ, **self.env, **extra)
        env.setdefault('PYTHONUNBUFFERED', '1')
        self._append(f'[gcs] $ {" ".join(f"{k}={v}" for k, v in extra.items())}'
                     f'{" " if extra else ""}{" ".join(self.argv)}')
        # stdin=DEVNULL: nothing here is interactive, and a tool that stops to
        # ask a question would otherwise hang until its timeout with no clue why
        # (clear_fault_logs.py --no-archive has exactly such a prompt).
        self.proc = subprocess.Popen(
            self.argv, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True)
        self.pgid = os.getpgid(self.proc.pid)
        self.started = time.time()
        self.opts = opts
        threading.Thread(target=self._pump, args=(self.proc.stdout,),
                         daemon=True).start()
        what = ', '.join(f'{k} {v}' for k, v in opts.items() if v)
        return True, (f'started (pid {self.proc.pid}, pgid {self.pgid})'
                      + (f' -- {what}' if what else ''))

    def stop(self, adopt=True):
        with self._op:
            return self._stop(adopt)

    def _stop(self, adopt=True):
        """Stop this process.  `adopt` decides what happens when it is not ours.

        The row's own Stop button passes adopt=True: a stack started in a
        terminal IS stoppable from the dashboard, which is the whole point of
        stop_foreign().  Every SWEEP passes adopt=False, because a sweep that
        silently adopted would break the promise stop_all() makes in its own
        docstring -- and did: this server's Ctrl-C teardown calls stop_all()
        with external=False, which walked every row, found each row's outside
        processes here, and killed a stack it had never started.  Closing the
        dashboard must not take down someone else's terminal session.  STOP ALL
        still reaches them, through stop_external(), which is the explicit ask.
        """
        proc = self.proc                    # snapshot: status() may clear it
        if proc is None or proc.poll() is not None:
            self.proc = None
            ext = self.external_pids() if adopt else []
            if ext:
                # Started from a terminal rather than here. We do not own its
                # process group, so stop_foreign() has to work out what is safe
                # to signal -- see the note above sysinfo.kill_targets.
                return stop_foreign(ext, self.patterns, self._append)
            return False, 'not running'
        pgid = self.pgid
        self._stopping = True               # keep status() from logging an exit
        try:
            for sig, grace in ((signal.SIGINT, SIGINT_GRACE),
                               (signal.SIGTERM, SIGTERM_GRACE)):
                self._append(f'[gcs] {sig.name} -> process group {pgid}')
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                deadline = time.time() + grace
                while time.time() < deadline:
                    if proc.poll() is not None and not sysinfo.pids_in_group(pgid):
                        self._append('[gcs] stopped cleanly')
                        return True, 'stopped'
                    time.sleep(0.2)
            self._append(f'[gcs] SIGKILL -> process group {pgid}')
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            time.sleep(0.5)
            return True, 'killed'
        finally:
            self._stopping = False
            self.proc = None

    # -- status ------------------------------------------------------------
    def external_pids(self):
        """Stack processes running that this server did not start.

        Our own children match our own patterns, so their group is excluded --
        but only while it is actually alive, because a pgid is reusable the
        moment it frees up.  Excluding it by pgid rather than short-circuiting
        on alive() is what lets a sweep find a SECOND, hand-started copy of a
        node while we are running our own.
        """
        skip = []
        if self.owner and self.owner.check_pgid:
            # m3_bridge_check.py runs a real px4_offboard_bridge in a sandbox.
            skip.append(self.owner.check_pgid)
        if self.alive() and self.pgid:
            skip.append(self.pgid)
        hits = sysinfo.find_by_cmdline(self.patterns, exclude_pgids=skip)
        return sorted({p for pids in hits.values() for p in pids})

    def status(self):
        proc = self.proc                    # snapshot: stop() may clear it
        if proc is not None and proc.poll() is None:
            pids = sysinfo.pids_in_group(self.pgid) or [proc.pid]
            rss, cpu = sysinfo.tree_usage(pids)
            return {'key': self.key, 'label': self.label, 'state': 'running',
                    'managed': True, 'pid': proc.pid, 'procs': len(pids),
                    'rss_mb': rss, 'cpu_pct': cpu, 'opts': dict(self.opts),
                    'uptime_s': int(time.time() - (self.started or time.time()))}
        rc = proc.poll() if proc is not None else None
        if rc is not None and not self._stopping:
            # Surface a crash instead of quietly showing "stopped". Not while a
            # stop we asked for is in flight -- that is not news, and clearing
            # self.proc from here would pull it out from under stop().
            self._append(f'[gcs] exited with code {rc}')
            self.proc = None
        ext = self.external_pids()
        if ext:
            rss, cpu = sysinfo.tree_usage(ext)
            return {'key': self.key, 'label': self.label, 'state': 'external',
                    'managed': False, 'pid': ext[0], 'procs': len(ext),
                    'rss_mb': rss, 'cpu_pct': cpu, 'uptime_s': None}
        return {'key': self.key, 'label': self.label, 'state': 'stopped',
                'managed': False, 'pid': None, 'procs': 0,
                'rss_mb': 0.0, 'cpu_pct': 0.0, 'uptime_s': None,
                'exit_code': rc}


DISK_POLL = 5.0          # how often a running recording re-checks free space


class Recorder(Managed):
    """One `ros2 bag record`, started and stopped like any other process group.

    Not a row in PROCS, because every other managed process has a fixed argv
    and a recording's argv IS the question the user answered: all topics, or
    the ones they picked.  What it inherits from Managed is the part that must
    not be reimplemented -- start_new_session + os.killpg, and the
    SIGINT -> SIGTERM -> SIGKILL escalation.

    SIGINT first matters more here than anywhere else in this file.  rosbag2
    writes metadata.yaml when the recorder SHUTS DOWN; a bag whose recorder was
    killed has every message on disk and no way to read them back, because
    `ros2 bag play` needs that file.  Managed already gives SIGINT 15 s before
    escalating, which is why this subclasses it instead of rolling its own --
    and when SIGKILL is reached anyway, stop() says the bag is unfinalised
    rather than reporting a clean stop.

    The disk guard is the other addition.  Recording everything on this
    aircraft is a few hundred MB a minute (the MID360 alone is ~200k points/s
    across /livox/lidar and /cloud_registered), and bags land on the same
    rootfs the flight stack swaps against -- CLAUDE.md's M1 regression is
    "swap != 0 and the Jetson thrashes".  So a running recording watches free
    space and stops ITSELF, cleanly, while there is still room to write the
    metadata that makes the bag readable.
    """

    def __init__(self, root):
        # No patterns, i.e. this never adopts a recording started elsewhere.
        # `ros2 bag record` is argv [python3, .../ros2, bag, record, ...] and
        # its only program-shaped basename is `ros2`, which every `ros2 run`
        # node in PROCS shares.  Matching on that would be exactly the
        # flattened-command-line mistake the top of this file exists to avoid,
        # so an outside recording is left alone rather than half-adopted.
        super().__init__('bag', 'ROS bag recording', [], {}, [])
        self.root = root
        self.path = None          # absolute, while recording
        self.name = None
        self.topics = []
        self.all = False
        self.last = None          # summary of the recording that just finished
        self._size = (0.0, 0, 0.0, 0.0)   # (mb, files, monotonic, mb_per_s)

    # -- lifecycle ---------------------------------------------------------
    def start(self, topics=(), all_topics=False, label='', exclude=''):
        topics = [t.strip() for t in topics if t and t.strip()]
        with self._op:
            if self.alive():
                return False, f'already recording into {bagger.BAG_DIRNAME}/{self.name}'
            if not all_topics and not topics:
                return False, ('nothing selected -- pick at least one topic, '
                               'or choose "All topics"')
            bad = [t for t in topics if not bagger.valid_topic(t)]
            if bad:
                # Belt and braces: Popen takes a list so there is no shell to
                # inject into, but a value starting with '-' would be read by
                # ros2 as an option rather than a topic.
                return False, f'not a topic name: {bad[0]!r}'
            free = bagger.free_gb(self.root)
            if free < bagger.MIN_FREE_GB:
                return False, (f'only {free} GB free on this filesystem -- '
                               f'refusing to record below {bagger.MIN_FREE_GB:.0f} GB. '
                               'Delete some bags first.')
            path, name = bagger.new_bag_path(self.root, label)
            try:
                os.makedirs(bagger.bag_root(self.root), exist_ok=True)
            except OSError as exc:
                return False, f'cannot create {bagger.BAG_DIRNAME}/: {exc}'
            self.argv = bagger.record_argv(
                os.path.relpath(path, self.root), topics, all_topics, exclude)
            ok, msg = self._start(self.root)
            if not ok:
                return ok, msg
            self.path, self.name = path, name
            self.topics, self.all = topics, all_topics
            self.last = None
            # Seeded with `now`, not 0: the first poll then reports the average
            # rate since the recording began rather than a placeholder 0 MB/s,
            # which is the number someone reads to decide whether to let it run.
            self._size = (0.0, 0, time.monotonic(), 0.0)
            threading.Thread(target=self._disk_watch, args=(self.proc,),
                             daemon=True).start()
            what = 'all topics' if all_topics else f'{len(topics)} topic(s)'
            if all_topics and exclude:
                what += ' (images excluded)'
            return True, f'recording {what} into {bagger.BAG_DIRNAME}/{name}'

    def stop(self):
        with self._op:
            if not self.alive():
                # Nothing of ours is running.  There is deliberately no sweep
                # for an outside recorder here -- see the note in __init__.
                return False, 'no recording is running'
            path, name = self.path, self.name
            ok, msg = self._stop(adopt=False)
            summary = self._summarise(path, name, killed=(msg == 'killed'))
            self.last = summary
            self.path = self.name = None
            return True, summary['message']

    def _summarise(self, path, name, killed=False):
        mb, files = bagger.dir_size(path) if path else (0.0, 0)
        meta = bagger.read_metadata(path) if path else {'finalized': False}
        row = {'name': name, 'dir': f'{bagger.BAG_DIRNAME}/{name}',
               'size_mb': mb, 'files': files, **meta}
        if meta.get('finalized'):
            row['message'] = (f'stopped -- {bagger.BAG_DIRNAME}/{name}: {mb} MB'
                              + (f', {meta["messages"]} messages'
                                 if meta.get('messages') else '')
                              + (f', {meta["duration_s"]}s'
                                 if meta.get('duration_s') else ''))
        else:
            # The one outcome worth shouting about: the messages are on disk
            # and `ros2 bag play` cannot read them.
            row['message'] = (
                f'stopped -- {bagger.BAG_DIRNAME}/{name} is {mb} MB but has NO '
                'metadata.yaml, so `ros2 bag play` cannot read it'
                + (' (the recorder had to be SIGKILLed)' if killed else '')
                + '. `ros2 bag reindex` can sometimes rebuild it.')
        self._append('[gcs] ' + row['message'])
        return row

    def _disk_watch(self, proc):
        """Stop the recording rather than fill the rootfs.

        Takes the Popen it was started for, so a watchdog left over from a
        previous recording can never stop the next one: if self.proc has moved
        on, this thread is stale and exits.
        """
        while True:
            time.sleep(DISK_POLL)
            if self.proc is not proc or proc.poll() is not None:
                return
            free = bagger.free_gb(self.root)
            if free >= bagger.MIN_FREE_GB:
                continue
            self._append(
                f'[gcs] only {free} GB free -- stopping the recording now, '
                'while there is still room to write the metadata that makes '
                'the bag readable')
            self.stop()
            return

    # -- status ------------------------------------------------------------
    def active_path(self):
        """The bag being written right now, or None -- never a stale one.

        self.path is only cleared by a stop WE performed, so it outlives a
        recorder that died on its own (a bad topic, a full disk under it).
        Asked from a reader thread, so it must not clear anything: mutating
        state from the polling path is the concurrency bug this file already
        carries a note about.
        """
        return self.path if self.alive() else None

    def _live_size(self, ttl=1.0):
        """(MB, files, MB/s) of the recording in progress, cached.

        MB/s is measured between polls rather than derived from a message
        count: it is what answers "how long can I keep this running", which is
        the only question anyone asks of it mid-test.
        """
        mb, files, when, rate = self._size
        now = time.monotonic()
        if now - when < ttl:
            return mb, files, rate
        new_mb, new_files = bagger.dir_size(self.path) if self.path else (0.0, 0)
        if when and now > when:
            rate = round(max(0.0, new_mb - mb) / (now - when), 2)
        self._size = (new_mb, new_files, now, rate)
        return new_mb, new_files, rate

    def status(self):
        row = super().status()
        free = bagger.free_gb(self.root)
        row.update({'recording': row['state'] == 'running',
                    'name': self.name, 'all': self.all,
                    'topics_n': len(self.topics),
                    'dir': bagger.BAG_DIRNAME,
                    'free_gb': free, 'min_free_gb': bagger.MIN_FREE_GB,
                    'low_free': free < bagger.LOW_FREE_GB,
                    'last': self.last, 'size_mb': 0.0, 'files': 0,
                    'rate_mbs': 0.0, 'elapsed_s': None})
        # Deliberately NOT the topic names: a locked client must not learn from
        # here that a camera topic is being recorded, which is precisely what
        # /api/state redacts everywhere else.
        if row['recording']:
            mb, files, rate = self._live_size()
            row.update({'size_mb': mb, 'files': files, 'rate_mbs': rate,
                        'elapsed_s': row['uptime_s']})
        row['start_blocked'] = (
            None if row['recording'] else
            (f'only {free} GB free -- below the {bagger.MIN_FREE_GB:.0f} GB floor'
             if free < bagger.MIN_FREE_GB else None))
        return row


class Supervisor:
    def __init__(self, repo_root):
        self.root = repo_root
        self.procs = {k: Managed(k, lbl, argv, env, pats)
                      for k, (lbl, argv, env, pats) in PROCS.items()}
        # Separate from procs: its argv is chosen per recording, it is never
        # brought back by a restart, and it is not something the stack needs.
        self.bag = Recorder(repo_root)
        # One slot for one-shot checks: several of them are MAVLink clients and
        # only one process at a time may hold udpin:127.0.0.1:14569.
        self.check_lock = threading.Lock()
        self.check = Managed('check', 'diagnostic', [], {}, [])
        self.check_running = None
        # name -> {'ok', 't', 'label', 'detail'} of the last run of each check,
        # so the Flight day card can show a verdict next to its button.
        self.check_results = {}
        # m3_bridge_check.py starts a real px4_offboard_bridge in a sandbox;
        # ignore that pgid so it is not reported as an external bridge.
        self.check_pgid = None
        # One long-running operation at a time. Without this, a double-clicked
        # Restart would race itself: two threads stopping and starting the same
        # process groups, and the second start would find the first's children
        # half dead and refuse.
        self.busy = None
        self.last_error = None
        self._busy = threading.Lock()
        # A sweep walks process groups it does not own; two at once would have
        # the second one signalling pids the first had already reaped.
        self._sweep = threading.Lock()
        for proc in self.procs.values():
            proc.owner = self

    def start(self, key, opts=None):
        if self.busy:
            return False, f'{self.busy} in progress'
        p = self.procs.get(key)
        return (False, 'unknown process') if p is None else p.start(self.root, opts)

    def stop(self, key):
        if self.busy:
            return False, f'{self.busy} in progress'
        p = self.procs.get(key)
        return (False, 'unknown process') if p is None else p.stop()

    def stop_all(self, external=False):
        """Stop everything we started, and optionally the strays too.

        `external` is False for the shutdown and restart paths and True for the
        STOP ALL button.  Ctrl-C on the server must stop what it started and
        nothing else -- silently killing a stack someone had running in another
        terminal, just because the dashboard was closed, would be a nasty
        surprise.  Pressing STOP ALL is an explicit "stop everything" and is
        the reason this sweep exists.
        """
        if self.busy:
            return False, f'{self.busy} in progress'
        msgs = self._stop_msgs()
        if external:
            ok, msg = self.stop_external()
            if ok:
                msgs.append(f'outside: {msg}')
        return True, '; '.join(msgs) if msgs else 'nothing was running'

    def _stop_all(self, skip=()):
        msgs = self._stop_msgs(skip)
        return True, '; '.join(msgs) if msgs else 'nothing was running'

    def _stop_msgs(self, skip=()):
        msgs = []
        # First: a bag has to be SIGINTed and given time to write metadata.yaml,
        # and doing that while fsm_node is unwinding 2.3 GB is asking for the
        # escalation to SIGKILL that loses the bag. Nothing brings it back
        # afterwards -- a restart resumes processes, not a recording.
        ok, msg = self.bag.stop()
        if ok:
            msgs.append(f'bag: {msg}')
        for key in reversed(ORDER):
            if key in skip:
                continue
            # adopt=False: this walk is the shutdown and restart path, and it
            # must touch only what we started. STOP ALL reaches the rest by
            # calling stop_external() as a separate, explicit step.
            ok, msg = self.procs[key].stop(adopt=False)
            if ok:
                msgs.append(f'{key}: {msg}')
        return msgs

    def external(self):
        """Every stack process on this box that this server did not start."""
        pids, patterns = set(), set()
        for key in reversed(ORDER):
            proc = self.procs[key]
            found = proc.external_pids()
            if found:
                pids.update(found)
                patterns.update(proc.patterns)
        return sorted(pids), sorted(patterns)

    def stop_external(self):
        """Adopt and stop the nodes someone started from a terminal.

        Swept in ONE pass rather than row by row: a hand-started stack puts
        fastlio_mapping, fsm_node and the Livox driver in a single process
        group, and stopping it per row would signal that same group three times
        and wait out the full grace on each.
        """
        if self.busy:
            return False, f'{self.busy} in progress'
        if not self._sweep.acquire(blocking=False):
            return False, 'already stopping outside processes'
        try:
            pids, patterns = self.external()
            if not pids:
                return False, ('nothing is running outside this GUI -- '
                               'everything that is up was started here')
            # The detail goes to the checks pane (one-shot operations), with a
            # pointer in each affected row's own log so it is not a mystery
            # there either.
            for key in ORDER:
                if set(self.procs[key].external_pids()) & set(pids):
                    self.procs[key]._append(
                        '[gcs] stopping outside processes -- see the checks log')
            return stop_foreign(pids, patterns, self.check._append)
        finally:
            self._sweep.release()

    def restart(self):
        """Stop everything that is running, then bring the same set back.

        Restarts what WAS running rather than a fixed list: if you were only
        running the stack, you get the stack back, not three surprise nodes --
        and on the config it was on, not the default.

        Refused outright while the DTP follower runs, from here or a terminal.
        Its Start has checks this thread cannot re-run halfway through (the FC
        says disarmed, the camera is free, fsm_node is on the follow config
        before a live start), so it could not honestly bring the follower
        back; and taking the stack down under a running follower removes the
        odometry it places people in, which on an armed aircraft is exactly
        the sensor loss that trips command_mux's NAV_LAND abort.
        """
        if not self._busy.acquire(blocking=False):
            return False, f'{self.busy} in progress'
        for key in NO_RESTART:
            proc = self.procs[key]
            if proc.alive() or proc.external_pids():
                self._busy.release()
                return False, (f'the {proc.label.split(" (")[0]} is running -- '
                               'stop it first. Restart never brings it back on '
                               'its own: its Start has checks a restart cannot '
                               're-run, and restarting the stack under it takes '
                               'its odometry away')
        keys = [k for k in ORDER
                if k not in RESTART_IGNORES and self.procs[k].alive()]
        if not keys:
            self._busy.release()
            return False, ('nothing is running -- use Start. Restart brings '
                           'back whatever was up, and nothing is.')
        self.busy = 'restart'
        opts = {k: dict(self.procs[k].opts) for k in keys}
        threading.Thread(target=self._restart, args=(keys, opts),
                         daemon=True).start()
        return True, 'restarting ' + ', '.join(keys)

    def _restart(self, keys, opts=None):
        """Stop, settle, start again.

        Every start is attempted even if an earlier one failed: the worst
        outcome here is stopping everything and bringing nothing back, so a
        failure must never abort the remaining starts, and it must be visible
        rather than leaving a silently empty stack.
        """
        self.last_error = None
        failed = []
        try:
            self._stop_all(skip=RESTART_IGNORES)
            # DDS needs a moment to drop the old participants; starting a new
            # fastlio_mapping while the old one's discovery data is still live
            # is how you get two publishers on /lidar_slam/odom for a second.
            time.sleep(RESTART_SETTLE)
            for key in keys:
                try:
                    ok, msg = self.procs[key].start(self.root,
                                                    (opts or {}).get(key))
                except Exception as exc:
                    ok, msg = False, f'{type(exc).__name__}: {exc}'
                if not ok:
                    failed.append(f'{key} ({msg})')
                self.procs[key]._append(
                    f'[gcs] restart: {msg}' if not ok else '[gcs] restarted')
                time.sleep(RESTART_STACK_WAIT
                           if key == 'stack' and len(keys) > 1 else 0.5)
        except Exception as exc:
            failed.append(f'{type(exc).__name__}: {exc}')
        finally:
            if failed:
                self.last_error = 'restart could not bring back: ' + ', '.join(failed)
                self.check._append(f'[gcs] {self.last_error}')
            self.busy = None
            self._busy.release()

    def status(self):
        return [self.procs[k].status() for k in ORDER]

    def start_bag(self, **kw):
        if self.busy:
            return False, f'{self.busy} in progress'
        return self.bag.start(**kw)

    def stop_bag(self):
        # Never gated on self.busy: finishing a bag cleanly is the one thing
        # that must not have to wait for a restart to walk five process groups.
        return self.bag.stop()

    def bag_status(self):
        return self.bag.status()

    def run_action(self, name):
        """Run one of ACTIONS.  Separate from run_check() on purpose: these are
        not read-only, and keeping them out of the CHECKS dict is what stops
        /api/check/<name> from reaching them without their own interlock."""
        return self._launch(name, ACTIONS.get(name))

    def run_check(self, name):
        return self._launch(name, CHECKS.get(name))

    def _launch(self, name, spec):
        if spec is None:
            return False, 'unknown check'
        if self.busy:
            return False, f'{self.busy} in progress'
        if not self.check_lock.acquire(blocking=False):
            return False, f'{self.check_running} is still running'
        self.check_running = spec.label
        threading.Thread(target=self._run_check,
                         args=(name, spec.label, spec.argv, spec.timeout, spec.timeout_ok),
                         daemon=True).start()
        return True, f'running: {spec.label}'

    def _run_check(self, name, label, argv, timeout, timeout_ok=False):
        c = self.check
        result = {'ok': False, 't': time.time(), 'label': label, 'detail': 'did not run'}
        try:
            c._append('')
            c._append(f'===== {label} =====')
            c._append(f'[gcs] $ {" ".join(argv)}')
            proc = subprocess.Popen(
                argv, cwd=self.root,
                env=dict(os.environ, PYTHONUNBUFFERED='1'),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
            pgid = self.check_pgid = os.getpgid(proc.pid)
            hit = threading.Event()

            def expire():
                hit.set()
                try:
                    os.killpg(pgid, signal.SIGINT)
                except ProcessLookupError:
                    pass

            timer = threading.Timer(timeout, expire)
            timer.start()
            try:
                for line in iter(proc.stdout.readline, ''):
                    c._append(line)
                proc.wait()
            finally:
                timer.cancel()
                try:
                    proc.stdout.close()
                except Exception:
                    pass
            if hit.is_set():
                c._append(f'[gcs] stopped after {timeout}s'
                          + ('' if timeout_ok else ' (timed out)'))
                result.update(ok=timeout_ok,
                              detail='finished' if timeout_ok else f'timed out after {timeout:.0f} s')
            else:
                c._append(f'[gcs] exit code {proc.returncode}')
                result.update(ok=proc.returncode == 0,
                              detail='exit 0' if proc.returncode == 0 else f'exit {proc.returncode}')
        except Exception as exc:
            c._append(f'[gcs] check failed: {exc}')
            result['detail'] = str(exc)
        finally:
            result['t'] = time.time()
            self.check_results[name] = result
            self.check_running = None
            self.check_pgid = None
            self.check_lock.release()

    def note(self, line):
        """Write one line into the CHECKS pane -- the log for one-shot
        operations.  Public because the pane is where the user is already
        looking after pressing a button, and things outside this module
        (hostpower's shutdown thread) have something to say there.
        """
        self.check._append(line)

    def logs(self, key, since=0):
        if key == 'check':
            return self.check.logs(since)
        if key == 'bag':
            return self.bag.logs(since)
        p = self.procs.get(key)
        return p.logs(since) if p else {'lines': [], 'seq': since}
