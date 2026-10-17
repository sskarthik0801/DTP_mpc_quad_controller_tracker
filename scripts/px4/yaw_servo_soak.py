#!/usr/bin/env python3
"""Synthetic soak of the REAL, INSTALLED px4_offboard_bridge with the camera yaw servo.

    source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash \
        && source ~/super_ws/install/setup.bash
    python3 scripts/px4/yaw_servo_soak.py --out /tmp/soak                  # 45 min, a fresh seed
    python3 scripts/px4/yaw_servo_soak.py --out /tmp/soak --seed 1234      # reproduce a scenario
    python3 scripts/px4/yaw_servo_soak.py --analyze /tmp/soak              # re-analyse a finished soak

Two bridges run side by side on the same sandboxed inputs, with separate outputs:
  servo  yaw_servo_enable:=true yaw_servo_ff_enable:=true   (fix 1 + 2, the one under test)
  ctrl   both switches off                                  (the control)
Both are ~/super_ws/install/super_bringup/lib/super_bringup/px4_offboard_bridge.py (env BRIDGE),
run with python3 as `ros2 run` runs it, at their default parameters except the topics and the
switches.

A driver (this script, --role driver) publishes, from a seeded scenario (--seed; logged in
meta.json and at the top of the report; the timeline and every message's content are functions
of the seed and the scheduled time, so a seed reproduces the scenario up to timing jitter):
* vehicle_local_position, 50 Hz: a slowly moving aircraft (sums of sinusoids, <= ~3 m/s), its
  heading drifting through +-180 deg (<= ~56 deg/s); xy_valid / z_valid false for 0.2-2 s now and
  then; NaN or inf in x / y / z / heading for single messages or short bursts.
* vehicle_control_mode, 2 Hz plus on every change: cycles of disarmed not-Offboard; disarmed
  Offboard (engage-then-arm) or armed not-Offboard (arm-then-engage); armed Offboard 20-120 s,
  sometimes with a quick takeover and re-engage inside it, sometimes with 2.2-3.5 s of report
  SILENCE (the link drops the report; the bridge must keep streaming); then a takeover (armed,
  not Offboard) or a disarm in Offboard (the latch is sent again); then disarmed.
* pos_cmd, 100 Hz in bursts of 1-15 s with silences of 0.5-5 s (TRACK <-> HOLD): SUPER-like yaw
  (holds, smooth turns, fast turns at 60-120 deg/s, through +-180 deg, sent wrapped or unwrapped),
  position / velocity / acceleration / jerk near the aircraft, now and then a non-finite field
  (position or yaw: the bridge drops it; velocity or acceleration: the bridge sends NaN), now
  and then SUPER's EMER flag.
* yaw_target, 15 Hz, stamped 150-400 ms before publication (as flown): a person walking,
  orbiting at up to 3 m/s, passing inside 2.5 m and out past 2.8 m, hovering across the
  hysteresis band, jumping (track switches); dropouts of 0.1-3 s, sometimes ending in a burst of
  back-to-back messages; stamps going stale while messages keep arriving; future stamps (inside
  and outside the 0.2 s tolerance); wrong frame_ids; NaN / inf points; frozen stamps (a coast);
  'lidar_init' as well as 'world'.
A recorder (--role recorder) records every output of both bridges (setpoints with all fields
and their own timestamps, offboard_control_mode, the servo's /bridge/yaw_servo) in npz chunks
every 5 s; the driver logs every input it published the same way. The orchestrator samples each
bridge's CPU (utime+stime delta from /proc/<pid>/stat) and RSS (/proc/<pid>/status) and the
machine's load and memory every 10 s, writes progress.json every 10 s, and runs the invariant
analysis on the data so far every 5 min (in a niced subprocess), so a harness crash loses
nothing already on disk.

Isolation, as yaw_servo_bag_bench.py: ROS_DOMAIN_ID 96 (--domain) and ROS_LOCALHOST_ONLY=1 for
everything started; every bridge topic remapped under /soak/ through its topic parameters; a
refusal to start if any readable process already has that domain or a gate harness runs; every
child in its own session, stopped by the process group it owns and waited on by PID; at the end
it verifies by PID that nothing it started is alive and that no process is left on the domain.

The invariants (every recorded message; violations counted with the worst value) and the trend
checks are listed in the report it prints; see analyse().
"""
import argparse
import glob
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True

BRIDGE = os.environ.get('BRIDGE', os.path.expanduser(
    '~/super_ws/install/super_bringup/lib/super_bringup/px4_offboard_bridge.py'))
NS = '/soak'
BRIDGES = {
    'servo': ['yaw_servo_enable:=true', 'yaw_servo_ff_enable:=true'],
    'ctrl': [],
}
HARNESS_NAMES = ('yaw_servo_soak.py', 'yaw_servo_bag_bench.py', 'fix1_yaw_replay.py',
                 'yaw_chain_report.py', 'nmpc_replay.py')
FRAMES = ['world', 'lidar_init', 'map', 'base_link', '', 'odom', 'camera_link']
MAX_RATE = math.radians(90.0)
FF_MAX = math.radians(60.0)
FF_ACC = math.radians(90.0)
SLEW_TOL = math.radians(0.001)
FF_TOL = math.radians(0.001)
F32_PI = 3.1415927410125732
EDGE_ALLOW_NS = 25_000_000   # report -> stream: one 20 ms tick + 5 ms delivery (measured, reported)
HB_MAX_MS = 100.0


# ---------------------------------------------------------------------------------------------
# environment, isolation, processes (same rules as yaw_servo_bag_bench.py)
# ---------------------------------------------------------------------------------------------

def env_setup(domain, logdir):
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    os.environ['PYTHONUNBUFFERED'] = '1'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '0'
    os.makedirs(logdir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = logdir


def _ancestors():
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])
        except Exception:
            break
    return out


def _argv(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except Exception:
        return []


def domain_users(domain, exclude=()):
    key = f'ROS_DOMAIN_ID={domain}'.encode()
    hits = []
    for p in os.listdir('/proc'):
        if not p.isdigit() or int(p) in exclude:
            continue
        try:
            with open(f'/proc/{p}/environ', 'rb') as f:
                env = f.read().split(b'\0')
        except Exception:
            continue
        if key in env:
            hits.append((int(p), _argv(int(p))))
    return hits


def harnesses(exclude=()):
    hits = []
    for p in os.listdir('/proc'):
        if not p.isdigit() or int(p) in exclude:
            continue
        argv = _argv(int(p))
        base = [os.path.basename(a) for a in argv[:4]]
        if any(b.endswith('_check.py') or b in HARNESS_NAMES for b in base):
            hits.append((int(p), argv))
    return hits


def isolation_check(domain):
    ex = _ancestors()
    bad = [f'pid {p} has ROS_DOMAIN_ID={domain}: {" ".join(a)[:160]}' for p, a in domain_users(domain, ex)]
    bad += [f'pid {p} is a gate/replay harness: {" ".join(a)[:160]}' for p, a in harnesses(ex)]
    return bad


def start_proc(argv, log, nice=0):
    f = open(log, 'wb')
    pre = (lambda: os.nice(nice)) if nice else None
    p = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, start_new_session=True,
                         preexec_fn=pre)
    f.close()
    return p


def stop_proc(p, grace=6.0):
    if p.poll() is not None:
        return p.returncode
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3.0), (signal.SIGKILL, 3.0)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            break
        try:
            return p.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue
    return p.poll()


def bridge_argv(name, extra):
    argv = [sys.executable, BRIDGE, '--ros-args', '-r', f'__node:=soak_bridge_{name}',
            '-p', f'cmd_topic:={NS}/in/pos_cmd',
            '-p', f'local_position_topic:={NS}/in/vehicle_local_position',
            '-p', f'vehicle_control_mode_topic:={NS}/in/vehicle_control_mode',
            '-p', f'yaw_target_topic:={NS}/in/yaw_target',
            '-p', f'setpoint_topic:={NS}/{name}/trajectory_setpoint',
            '-p', f'control_mode_topic:={NS}/{name}/offboard_control_mode',
            '-p', f'yaw_servo_debug_topic:={NS}/{name}/yaw_servo']
    for x in extra:
        argv += ['-p', x]
    return argv


def proc_stat(pid):
    """(utime+stime in s, RSS in MB) of one process, or None."""
    try:
        with open(f'/proc/{pid}/stat') as f:
            fields = f.read().rsplit(')', 1)[1].split()
        cpu = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
        rss = None
        with open(f'/proc/{pid}/status') as f:
            for ln in f:
                if ln.startswith('VmRSS:'):
                    rss = int(ln.split()[1]) / 1024.0
        return cpu, rss
    except Exception:
        return None


def machine_stat():
    with open('/proc/loadavg') as f:
        la = [float(x) for x in f.read().split()[:3]]
    mem = {}
    with open('/proc/meminfo') as f:
        for ln in f:
            k, v = ln.split(':', 1)
            mem[k] = int(v.split()[0]) / 1024.0
    return la, mem.get('MemAvailable'), mem.get('MemFree')


# ---------------------------------------------------------------------------------------------
# the scenario
# ---------------------------------------------------------------------------------------------

def _segments(rs, T, mean_gap, lo, hi, start=5.0):
    out, t = [], start + rs.exponential(mean_gap)
    while t < T:
        d = rs.uniform(lo, hi)
        out.append((t, t + d))
        t += d + rs.exponential(mean_gap)
    return out


def _in(segs, t):
    for a, b in segs:
        if a <= t < b:
            return (a, b)
    return None


class Scenario:
    """Everything the driver publishes, as functions of the seed and the scheduled time t (s)."""
    DT = 0.01

    def __init__(self, seed, duration):
        import numpy as np
        rs = np.random.RandomState(seed)
        self.seed, self.duration = seed, duration
        T = self.T = duration + 30.0
        n = self.n = int(T / self.DT) + 2
        tw = 2 * math.pi
        # ---- vehicle (NED): sums of sinusoids; heading drifts through +-180 deg
        self.vx = [(rs.uniform(1, 6), rs.uniform(0.01, 0.1), rs.uniform(0, tw)) for _ in range(4)]
        self.vy = [(rs.uniform(1, 6), rs.uniform(0.01, 0.1), rs.uniform(0, tw)) for _ in range(4)]
        self.vz = (rs.uniform(0.1, 0.4), rs.uniform(0.05, 0.2), rs.uniform(0, tw))
        self.h0 = rs.uniform(-math.pi, math.pi)
        self.hdr = math.radians(rs.uniform(2, 8)) * rs.choice([-1, 1])
        self.hs = [(math.radians(rs.uniform(5, 40)), rs.uniform(0.05, 0.4), rs.uniform(0, tw)) for _ in range(3)]
        # lpos faults
        self.f_xy = _segments(rs, T, 60.0, 0.2, 2.0)
        self.f_z = _segments(rs, T, 120.0, 0.2, 1.0)
        self.f_nan = [(a, b, rs.randint(4), rs.rand() < 0.2)
                      for a, b in _segments(rs, T, 30.0, 0.015, 0.2)]      # field 0..3, inf?
        # ---- mode cycle
        segs, sil = [], []
        t = 0.0
        while t < T:
            segs.append((t, False, False))
            t += rs.uniform(2, 8)
            if rs.rand() < 0.4:
                segs.append((t, False, True))       # engage, then arm
                t += rs.uniform(2, 6)
            else:
                segs.append((t, True, False))       # arm, then engage
                t += rs.uniform(1, 5)
            L = rs.uniform(20, 120)
            segs.append((t, True, True))
            quick = rs.rand() < 0.25
            silent = rs.rand() < 0.2
            if quick:
                tq = t + rs.uniform(4, L / 2 - 2)
                segs.append((tq, True, False))
                segs.append((tq + rs.uniform(0.3, 1.5), True, True))
            if silent:
                ts0 = t + L / 2 + rs.uniform(1, L / 2 - 5)
                sil.append((ts0, ts0 + rs.uniform(2.2, 3.5)))
            t += L
            if rs.rand() < 0.15:
                segs.append((t, False, True))       # disarmed in Offboard (landed): the latch again
                t += rs.uniform(2, 5)
            else:
                segs.append((t, True, False))       # the pilot's takeover
                t += rs.uniform(2, 8)
        self.modes = segs
        self.mode_t = np.array([s[0] for s in segs])
        self.silences = sil
        # IDLE with an invalid position: an xy-invalid fault inside 60 % of the disarmed-Offboard
        # phases (SUPER's stored command then goes out, acceleration NaN); and across 25 % of the
        # engages a NaN heading (the servo starts from SUPER's yaw), across 15 % an invalid xy
        for i, (ta_, a_, o_) in enumerate(segs):
            tb_ = segs[i + 1][0] if i + 1 < len(segs) else T
            if o_ and not a_ and tb_ - ta_ > 1.5 and rs.rand() < 0.6:
                s0 = rs.uniform(ta_ + 0.3, tb_ - 1.0)
                self.f_xy.append((s0, s0 + rs.uniform(0.5, min(1.5, tb_ - s0 - 0.1))))
            if a_ and o_ and (i == 0 or not (segs[i - 1][1] and segs[i - 1][2])):
                u_ = rs.rand()
                if u_ < 0.25:
                    self.f_nan.append((ta_ - 0.06, ta_ + 0.06, 3, False))
                elif u_ < 0.4:
                    self.f_xy.append((ta_ - 0.1, ta_ + 0.5))
        # ---- pos_cmd bursts and SUPER's yaw on the grid
        bursts, t = [], rs.uniform(0, 3)
        while t < T:
            L = rs.uniform(1, 15)
            # wrapped?, EMER?, yaw_dot gain (30 %: SUPER's yaw_dot inconsistent with its own yaw
            # sequence, x1.2-1.6, so the fall-back's clamp of SUPER's rate to 90 deg/s is reached)
            bursts.append((t, t + L, rs.rand() < 0.7, rs.rand() < 0.05,
                           rs.uniform(1.2, 1.6) if rs.rand() < 0.3 else 1.0))
            t += L + rs.uniform(0.5, 5)
        self.bursts = bursts
        self.burst_t = np.array([b[0] for b in bursts])
        rate = np.zeros(n)
        i = 0
        while i < n:
            u = rs.rand()
            if u < 0.3:
                r, L = 0.0, rs.uniform(0.5, 6)
            elif u < 0.75:
                r, L = math.radians(rs.uniform(-30, 30)), rs.uniform(0.5, 6)
            else:
                r, L = math.radians(rs.uniform(60, 120)) * rs.choice([-1, 1]), rs.uniform(0.3, 2)
            m = max(1, int(L / self.DT))
            rate[i:i + m] = r
            i += m
        from scipy.signal import lfilter
        a = self.DT / 0.2
        self.ydot = lfilter([a], [1, -(1 - a)], rate)
        self.yaw = rs.uniform(-math.pi, math.pi) + np.cumsum(self.ydot) * self.DT
        self.c_bad = [(a_, rs.randint(6)) for a_, _ in _segments(rs, T, 120.0, 0.0, 0.0)]  # which field
        self.c_off = [(rs.uniform(0.2, 1.0), rs.uniform(0.02, 0.2), rs.uniform(0, tw)) for _ in range(3)]
        # ---- the person, relative to the aircraft: range r and FLU bearing th, on the grid
        r = np.empty(n)
        th = np.empty(n)
        rc, tc = 5.0, rs.uniform(-math.pi, math.pi)
        i = 0
        kinds = ['walk', 'orbit', 'fast', 'close', 'boundary', 'jump']
        self.p_segs = []
        while i < n:
            k = kinds[rs.choice(6, p=[0.3, 0.2, 0.1, 0.2, 0.1, 0.1])]
            L = rs.uniform(2, 15)
            m = max(1, int(L / self.DT))
            self.p_segs.append((i * self.DT, k))
            if k == 'jump':
                rc, tc = rs.uniform(2, 8), tc + rs.uniform(-math.pi, math.pi)
                k = 'walk'
            rg = rs.uniform(3, 8) if k == 'walk' else rs.uniform(2.55, 2.85) if k == 'fast' else rs.uniform(2.3, 6)
            vt = (rs.uniform(-1.5, 1.5) if k in ('walk', 'close', 'boundary')
                  else rs.uniform(2.7, 3.0) * rs.choice([-1, 1]) if k == 'fast'
                  else rs.uniform(1, 3) * rs.choice([-1, 1]))
            rmin, rout = rs.uniform(1.8, 2.45), rs.uniform(2.9, 4.0)
            per = rs.uniform(0.8, 3.0)
            for q in range(m):
                if i + q >= n:
                    break
                if k in ('walk', 'orbit', 'fast'):
                    rc += max(-self.DT, min(self.DT, rg - rc))
                elif k == 'close':
                    goal = rmin if q < m / 2 else rout
                    rc += max(-1.5 * self.DT, min(1.5 * self.DT, goal - rc))
                else:
                    rc = 2.5 + 0.35 * math.sin(tw * q * self.DT / per)
                tc += vt / max(rc, 0.5) * self.DT
                r[i + q], th[i + q] = rc, tc
            i += m
        self.pr, self.pth = r, th
        # ---- yaw target pathologies
        self.y_drop = [(a_, b_, rs.rand() < 0.4, rs.randint(3, 11)) for a_, b_ in _segments(rs, T, 25.0, 0.1, 3.0)]
        self.y_stale = [(a_, b_, rs.uniform(0.6, 1.5)) for a_, b_ in _segments(rs, T, 60.0, 2.0, 4.0)]
        self.y_future = [(a_, b_, rs.uniform(0.05, 0.15) if rs.rand() < 0.5 else rs.uniform(0.25, 1.0))
                         for a_, b_ in _segments(rs, T, 90.0, 0.5, 2.0)]
        self.y_frame = [(a_, b_, FRAMES[2 + rs.randint(len(FRAMES) - 2)]) for a_, b_ in _segments(rs, T, 90.0, 0.5, 3.0)]
        self.y_lidar = _segments(rs, T, 60.0, 1.0, 5.0)
        self.y_nan = [(a_, b_, rs.randint(2), rs.rand() < 0.3) for a_, b_ in _segments(rs, T, 60.0, 0.03, 1.0)]
        self.y_dup = _segments(rs, T, 40.0, 0.5, 3.0)
        self.u = rs.uniform(-1, 1, 200_000)

    # ---- evaluators ------------------------------------------------------------------------
    def _g(self, t):
        return min(self.n - 1, max(0, int(t / self.DT)))

    def vehicle(self, t):
        x = sum(a * math.sin(w * t + p) for a, w, p in self.vx)
        y = sum(a * math.sin(w * t + p) for a, w, p in self.vy)
        a, w, p = self.vz
        z = -1.5 + a * math.sin(w * t + p)
        h = self.h0 + self.hdr * t + sum(b * math.sin(v * t + c) for b, v, c in self.hs)
        return x, y, z, math.atan2(math.sin(h), math.cos(h))

    def vehicle_vel(self, t):
        vx = sum(a * w * math.cos(w * t + p) for a, w, p in self.vx)
        vy = sum(a * w * math.cos(w * t + p) for a, w, p in self.vy)
        ax = -sum(a * w * w * math.sin(w * t + p) for a, w, p in self.vx)
        ay = -sum(a * w * w * math.sin(w * t + p) for a, w, p in self.vy)
        return vx, vy, ax, ay

    def lpos(self, t):
        x, y, z, h = self.vehicle(t)
        xy_valid = _in(self.f_xy, t) is None
        z_valid = _in(self.f_z, t) is None
        for a, b, field, inf in self.f_nan:
            if a <= t < b:
                bad = math.inf if inf else math.nan
                if field == 0:
                    x = bad
                elif field == 1:
                    y = bad
                elif field == 2:
                    z = bad
                else:
                    h = bad
        return x, y, z, h, xy_valid, z_valid

    def mode(self, t):
        import numpy as np
        k = int(np.searchsorted(self.mode_t, t, 'right')) - 1
        _, a, o = self.modes[max(k, 0)]
        return a, o

    def silent(self, t):
        return _in(self.silences, t) is not None

    def next_edge(self, t):
        import numpy as np
        k = int(np.searchsorted(self.mode_t, t, 'right'))
        return self.modes[k][0] if k < len(self.modes) else math.inf

    def burst(self, t):
        import numpy as np
        k = int(np.searchsorted(self.burst_t, t, 'right')) - 1
        if k < 0:
            return None
        b = self.bursts[k]
        return b if t < b[1] else None

    def cmd(self, t, seq):
        """SUPER's command at t (FLU): 14 floats + flag, or None outside a burst."""
        b = self.burst(t)
        if b is None:
            return None
        g = self._g(t)
        x, y, z, _ = self.vehicle(t)
        vx, vy, ax, ay = self.vehicle_vel(t)
        o = [a * math.sin(w * t + p) for a, w, p in self.c_off]
        u = self.u
        px, py, pz = x + o[0] + 1e-4 * u[seq % 199_991], -y + o[1] + 1e-4 * u[(seq * 7) % 199_993], -z + 0.1 * o[2]
        v = [vx + 0.1 * u[(seq * 3) % 199_999], -vy + 0.1 * u[(seq * 5) % 199_997], 0.2 * u[(seq * 11) % 199_989]]
        acc = [max(-2.5, min(2.5, ax + 0.3 * u[(seq * 13) % 199_981])),
               max(-2.5, min(2.5, -ay + 0.3 * u[(seq * 17) % 199_979])), 0.5 * u[(seq * 19) % 199_977]]
        j = [3 * u[(seq * 23) % 199_973], 3 * u[(seq * 29) % 199_967], 3 * u[(seq * 31) % 199_963]]
        yaw = float(self.yaw[g])
        if b[2]:
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        row = [px, py, pz, *v, *acc, *j, yaw, float(self.ydot[g]) * b[4]]
        for a_, field in self.c_bad:
            if a_ <= t < a_ + 0.0101:        # one command
                idx = (0, 1, 2, 12, 3, 6)[field]  # position x/y/z, yaw: dropped; velocity / acc: NaN out
                row[idx] = math.nan
        return row, (2 if b[3] else 1)

    def person(self, t):
        """The person, FLU world: the aircraft (FLU) + r (cos th, sin th)."""
        g = self._g(t)
        x, y, _, _ = self.vehicle(t)
        r, th = self.pr[g], self.pth[g]
        return x + r * math.cos(th), -y + r * math.sin(th)

    def base_age(self, t, k):
        return 0.25 + 0.06 * math.sin(2 * math.pi * t / 37.0) + 0.05 * self.u[(k * 37) % 199_951]

    def yaw_targets(self, t, k, t_last):
        """The yaw target messages to publish at scheduled time t (the k-th slot):
        [(age_s, x, y, frame, flags)], or a frozen stamp as ('dup', stamp_t, x, y, frame, flags);
        flags: bit0 burst, bit1 stale ramp, bit2 future, bit3 bad frame, bit4 nan, bit5 dup,
        bit6 lidar_init. Empty = a dropout."""
        if _in([(a, b) for a, b, _, _ in self.y_drop], t):
            return []
        out = []
        # a burst after a dropout that ended since the last slot
        for a, b, burst, nb in self.y_drop:
            if burst and t_last < b <= t:
                for q in range(nb - 1, 0, -1):
                    ts = t - q / 15.0
                    px, py = self.person(ts)
                    out.append((self.base_age(ts, k) + q / 15.0, px, py, 'world', 1))
        age = self.base_age(t, k)
        flags = 0
        px, py = self.person(t)
        frame = 'world'
        s = next(((a, b, m) for a, b, m in self.y_stale if a <= t < b), None)
        if s:
            f = (t - s[0]) / (s[1] - s[0])
            age = age + (s[2] - 0.25) * min(1.0, 2 * f) if f < 0.5 else age + (s[2] - 0.25)
            flags |= 2
        fu = next((m for a, b, m in self.y_future if a <= t < b), None)
        if fu is not None:
            age = -fu
            flags |= 4
        if _in(self.y_lidar, t):
            frame = 'lidar_init'
            flags |= 64
        fr = next((m for a, b, m in self.y_frame if a <= t < b), None)
        if fr is not None:
            frame = fr
            flags |= 8
        nn = next(((f_, inf) for a, b, f_, inf in self.y_nan if a <= t < b), None)
        if nn is not None:
            bad = math.inf if nn[1] else math.nan
            if nn[0] == 0:
                px = bad
            else:
                py = bad
            flags |= 16
        d = _in(self.y_dup, t)
        if d is not None:
            # a coast: the stamp and the point frozen at the segment's start
            px0, py0 = self.person(d[0])
            out.append(('dup', d[0] - self.base_age(d[0], 0), px0, py0, frame, flags | 32))
            return out
        out.append((age, px, py, frame, flags))
        return out


# ---------------------------------------------------------------------------------------------
# driver (subprocess role)
# ---------------------------------------------------------------------------------------------

def role_driver(cfg_path):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PointStamped
    from mars_quadrotor_msgs.msg import PositionCommand
    from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition

    cfg = json.load(open(cfg_path))
    out = cfg['out']
    sc = Scenario(cfg['seed'], cfg['duration'])
    rclpy.init()
    node = Node('soak_driver')
    px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
    super_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
    p_cmd = node.create_publisher(PositionCommand, f'{NS}/in/pos_cmd', super_qos)
    p_yt = node.create_publisher(PointStamped, f'{NS}/in/yaw_target', super_qos)
    p_lp = node.create_publisher(VehicleLocalPosition, f'{NS}/in/vehicle_local_position', px4_qos)
    p_vc = node.create_publisher(VehicleControlMode, f'{NS}/in/vehicle_control_mode', px4_qos)
    want = cfg['want']
    t_wait = time.time() + 40
    while time.time() < t_wait:
        m = {'pos_cmd': p_cmd.get_subscription_count(), 'yaw_target': p_yt.get_subscription_count(),
             'vehicle_local_position': p_lp.get_subscription_count(),
             'vehicle_control_mode': p_vc.get_subscription_count()}
        if m == want:
            break
        time.sleep(0.2)
    with open(os.path.join(out, 'driver_ready.json'), 'w') as f:
        json.dump({'matched': m, 'want': want, 't': time.time()}, f)
    if m != want:
        sys.exit(f'driver: inputs not matched {m} want {want}')
    # wait for the go file (the orchestrator checks the recorder side first)
    while not os.path.exists(os.path.join(out, 'go')):
        time.sleep(0.05)
    t0 = time.time_ns() + 200_000_000
    with open(os.path.join(out, 'driver_start.json'), 'w') as f:
        json.dump({'t0_ns': t0}, f)
    bufs = {k: [] for k in ('vcm', 'lpos', 'cmd', 'yt')}
    chunk = [0]
    counts = {k: 0 for k in bufs}

    def flush():
        arrs = {}
        for k, rows in bufs.items():
            if rows:
                arrs[k + '__t'] = np.array([r[0] for r in rows], np.int64)
                arrs[k + '__f'] = np.array([r[1:] for r in rows], np.float64)
                rows.clear()
        if arrs:
            tmp = os.path.join(out, f'.drv_{chunk[0]:05d}.npz')
            np.savez(tmp, **arrs)
            os.replace(tmp, os.path.join(out, f'drv_{chunk[0]:05d}.npz'))
            chunk[0] += 1
        with open(os.path.join(out, '.drv_status.json'), 'w') as f:
            json.dump({'t': time.time(), 'counts': counts, 'chunks': chunk[0]}, f)
        os.replace(os.path.join(out, '.drv_status.json'), os.path.join(out, 'drv_status.json'))

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    ns = lambda t: t0 + int(t * 1e9)      # noqa: E731
    nxt = {'lpos': 0.0, 'cmd': 0.0, 'vcm': 0.0, 'yt': 0.0}
    k_cmd = k_yt = 0
    t_last_yt = -1.0
    last_mode = None
    t_flush = time.time()
    lp = VehicleLocalPosition()
    vc = VehicleControlMode()
    try:
        while True:
            t_next = min(nxt.values())
            if t_next > sc.duration:
                break
            d = ns(t_next) - time.time_ns()
            if d > 0:
                time.sleep(d / 1e9)
            now = time.time_ns()
            for ch in [c for c, t in nxt.items() if t <= t_next]:
                t = nxt[ch]
                if ch == 'lpos':
                    x, y, z, h, xv, zv = sc.lpos(t)
                    lp.timestamp = now // 1000
                    lp.x, lp.y, lp.z, lp.heading = float(x), float(y), float(z), float(h)
                    lp.xy_valid, lp.z_valid = bool(xv), bool(zv)
                    tp = time.time_ns()
                    p_lp.publish(lp)
                    bufs['lpos'].append((tp, x, y, z, h, float(xv), float(zv)))
                    counts['lpos'] += 1
                    nxt['lpos'] = t + 0.02
                elif ch == 'vcm':
                    a, o = sc.mode(t)
                    if not sc.silent(t):
                        vc.timestamp = now // 1000
                        vc.flag_armed, vc.flag_control_offboard_enabled = bool(a), bool(o)
                        tp = time.time_ns()
                        p_vc.publish(vc)
                        bufs['vcm'].append((tp, float(a), float(o)))
                        counts['vcm'] += 1
                    last_mode = (a, o)
                    # 2 Hz, and exactly at the next change
                    nxt['vcm'] = min(t + 0.5, sc.next_edge(t + 1e-9))
                elif ch == 'cmd':
                    c = sc.cmd(t, k_cmd)
                    if c is not None:
                        row, flag = c
                        m = PositionCommand()
                        m.header.stamp.sec, m.header.stamp.nanosec = divmod(now, 1_000_000_000)
                        m.header.frame_id = 'world'
                        (m.position.x, m.position.y, m.position.z, m.velocity.x, m.velocity.y, m.velocity.z,
                         m.acceleration.x, m.acceleration.y, m.acceleration.z, m.jerk.x, m.jerk.y, m.jerk.z,
                         m.yaw, m.yaw_dot) = [float(v) for v in row]
                        m.trajectory_flag = flag
                        tp = time.time_ns()
                        p_cmd.publish(m)
                        bufs['cmd'].append((tp, *row, float(flag)))
                        counts['cmd'] += 1
                        k_cmd += 1
                        nxt['cmd'] = t + 0.01
                    else:
                        b = sc.bursts[min(len(sc.bursts) - 1, max(0, int(np.searchsorted(sc.burst_t, t, 'right'))))]
                        nxt['cmd'] = max(t + 0.01, b[0]) if b[0] > t else t + 0.01
                elif ch == 'yt':
                    for item in sc.yaw_targets(t, k_yt, t_last_yt):
                        m = PointStamped()
                        tp = time.time_ns()
                        if item[0] == 'dup':
                            _, st_t, x, y, frame, flags = item
                            stamp = ns(st_t)
                        else:
                            age, x, y, frame, flags = item
                            stamp = tp - int(age * 1e9)
                        m.header.stamp.sec, m.header.stamp.nanosec = divmod(int(stamp), 1_000_000_000)
                        m.header.frame_id = frame
                        m.point.x, m.point.y, m.point.z = float(x), float(y), 1.2
                        tp = time.time_ns()
                        p_yt.publish(m)
                        bufs['yt'].append((tp, float(stamp), x, y, float(FRAMES.index(frame)), float(flags)))
                        counts['yt'] += 1
                    t_last_yt = t
                    k_yt += 1
                    nxt['yt'] = t + 1.0 / 15.0 + 0.005 * sc.u[(k_yt * 41) % 199_931]
            if time.time() - t_flush > 5.0:
                flush()
                t_flush = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        flush()
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------------------------
# recorder (subprocess role)
# ---------------------------------------------------------------------------------------------

def role_recorder(cfg_path):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_check_compatible, QoSCompatibility)
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
    from std_msgs.msg import Float32MultiArray

    cfg = json.load(open(cfg_path))
    out = cfg['out']
    rclpy.init()
    node = Node('soak_recorder')
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=500)
    bufs, counts = {}, {}

    def add(key, row):
        bufs[key].append(row)
        counts[key] += 1

    for name, t in cfg['bridges'].items():
        for kind in ('sp', 'ocm', 'dbg'):
            if t.get(kind):
                bufs[f'{kind}_{name}'] = []
                counts[f'{kind}_{name}'] = 0
        node.create_subscription(
            TrajectorySetpoint, t['sp'],
            lambda m, k=f'sp_{name}': add(k, (time.time_ns(), m.timestamp, *m.position.tolist(),
                                              *m.velocity.tolist(), *m.acceleration.tolist(),
                                              *m.jerk.tolist(), m.yaw, m.yawspeed)), qos)
        node.create_subscription(
            OffboardControlMode, t['ocm'],
            lambda m, k=f'ocm_{name}': add(k, (time.time_ns(), m.timestamp)), qos)
        if t.get('dbg'):
            node.create_subscription(
                Float32MultiArray, t['dbg'],
                lambda m, k=f'dbg_{name}': add(k, (time.time_ns(), *list(m.data)[:7])), qos)
    chunk = [0]

    def flush():
        arrs = {}
        for k, rows in bufs.items():
            if not rows:
                continue
            arrs[k + '__recv'] = np.array([r[0] for r in rows], np.int64)
            if k.startswith('dbg_'):
                arrs[k + '__data'] = np.array([r[1:] for r in rows], np.float64)
            else:
                arrs[k + '__ts'] = np.array([r[1] for r in rows], np.int64)
                if k.startswith('sp_'):
                    arrs[k + '__f'] = np.array([r[2:] for r in rows], np.float64)
            rows.clear()
        if arrs:
            tmp = os.path.join(out, f'.rec_{chunk[0]:05d}.npz')
            np.savez(tmp, **arrs)
            os.replace(tmp, os.path.join(out, f'rec_{chunk[0]:05d}.npz'))
            chunk[0] += 1

    def graph():
        g = {}
        for name, t in cfg['bridges'].items():
            for kind in ('sp', 'ocm', 'dbg'):
                top = t.get(kind)
                if not top:
                    continue
                infos = node.get_publishers_info_by_topic(top)
                mine = [i.qos_profile for i in node.get_subscriptions_info_by_topic(top)
                        if i.node_name == node.get_name()]
                g[f'{kind}_{name}'] = [bool(mine) and qos_check_compatible(i.qos_profile, mine[0])[0]
                                      != QoSCompatibility.ERROR for i in infos]
        return g

    def status():
        try:
            g = graph()
        except Exception as e:
            g = {'error': repr(e)[:200]}
        with open(os.path.join(out, '.rec_status.json'), 'w') as f:
            json.dump({'t': time.time(), 'counts': counts, 'graph': g, 'chunks': chunk[0]}, f)
        os.replace(os.path.join(out, '.rec_status.json'), os.path.join(out, 'rec_status.json'))

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    t_flush = t_stat = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - t_stat > 1.0:
                status()
                t_stat = now
            if now - t_flush > 5.0:
                flush()
                t_flush = now
    except KeyboardInterrupt:
        pass
    finally:
        flush()
        status()
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def load_chunks(out, prefix):
    import numpy as np
    parts = {}
    for f in sorted(glob.glob(os.path.join(out, f'{prefix}_*.npz'))):
        try:
            z = np.load(f)
            for k in z.files:
                parts.setdefault(k, []).append(z[k])
        except Exception:
            continue
    return {k: np.concatenate(v) for k, v in parts.items()}


# ---------------------------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------------------------

def _u32(a):
    import numpy as np
    return np.ascontiguousarray(np.asarray(a, np.float32)).view(np.uint32)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pct(x, q):
    import numpy as np
    x = np.asarray(x, float)
    return float(np.percentile(x, q)) if len(x) else math.nan


def norm(s):
    return re.sub(r'[-+]?\d+(\.\d+)?', '#', s)


def analyse(out, partial=False):
    """Every invariant over everything recorded so far. Returns (lines, summary)."""
    import numpy as np
    meta = json.load(open(os.path.join(out, 'meta.json')))
    D = load_chunks(out, 'drv')
    R = load_chunks(out, 'rec')
    L = []
    S = {'partial': partial, 'viol': {}, 'info': {}}

    def say(s=''):
        L.append(s)

    def inv(name, n, worst, txt):
        S['viol'][name] = {'n': int(n), 'worst': worst}
        say(f'  [{"PASS" if n == 0 else "FAIL"}] {name}: {n} violations; {txt}')

    if 'vcm__t' not in D or 'sp_servo__ts' not in R:
        say('no data yet')
        return L, S
    t0 = json.load(open(os.path.join(out, 'driver_start.json')))['t0_ns']
    cut = min(int(D['lpos__t'].max()), int(R['ocm_servo__recv'].max()), int(R['ocm_ctrl__recv'].max()))
    cut -= 2_000_000_000 if partial else 0
    rel = lambda t: (np.asarray(t, np.int64) - t0) / 1e9   # noqa: E731
    # ---- inputs ------------------------------------------------------------------------------
    vt, vf = D['vcm__t'], D['vcm__f']
    lt, lf = D['lpos__t'], D['lpos__f']
    ct, cf = D['cmd__t'], D['cmd__f']
    yt_t, yt_f = D.get('yt__t', np.zeros(0, np.int64)), D.get('yt__f', np.zeros((0, 6)))
    o = np.argsort(vt, kind='stable'); vt, vf = vt[o], vf[o]
    o = np.argsort(lt, kind='stable'); lt, lf = lt[o], lf[o]
    o = np.argsort(ct, kind='stable'); ct, cf = ct[o], cf[o]
    # mode edges as published
    edges = []
    prev = None
    for t, (a, ob) in zip(vt, vf):
        s_ = (bool(a), bool(ob))
        if s_ != prev:
            edges.append((int(t), s_[0], s_[1]))
            prev = s_
    e_t = np.array([e[0] for e in edges], np.int64)
    e_off = np.array([e[2] for e in edges], bool)
    e_eng = np.array([e[1] and e[2] for e in edges], bool)

    def state_at(ts):
        """The last published report at each ts: (offboard, engaged, ns since that report's edge,
        offboard before that edge, engaged before that edge)."""
        k = np.searchsorted(e_t, ts, 'right') - 1
        ok = k >= 0
        kc = np.clip(k, 0, None)
        kp = np.clip(k - 1, 0, None)
        ob = np.where(ok, e_off[kc], False)
        en = np.where(ok, e_eng[kc], False)
        since = np.where(ok, ts - e_t[kc], 10**18)
        pob = np.where(k >= 1, e_off[kp], False)
        pen = np.where(k >= 1, e_eng[kp], False)
        return ob, en, since, pob, pen

    n_engage = int(sum(1 for i in range(len(edges)) if e_eng[i] and (i == 0 or not e_eng[i - 1])))
    n_offb = int(sum(1 for i in range(len(edges)) if e_off[i] and (i == 0 or not e_off[i - 1])))
    n_dis_off = int(sum(1 for i in range(len(edges)) if e_off[i] and not edges[i][1]))
    S['info']['engagements'] = n_engage
    say(f'soak {out}: seed {meta["seed"]}, {rel(cut):.1f} s analysed{" (partial)" if partial else ""}')
    say(f'inputs published: vehicle_control_mode {len(vt)} ({len(edges)} changes: {n_offb} Offboard entries, '
        f'{n_engage} engagements (armed+Offboard), {n_dis_off} disarmed-Offboard phases), vehicle_local_position '
        f'{len(lt)} (xy invalid {int(np.sum(lf[:, 4] == 0))}, non-finite {int(np.sum(~np.isfinite(lf[:, :4]).all(1)))}), '
        f'pos_cmd {len(ct)} (non-finite position/yaw {int(np.sum(~np.isfinite(cf[:, [0, 1, 2, 12, 13]]).all(1)))}, '
        f'EMER {int(np.sum(cf[:, 14] == 2))}), yaw_target {len(yt_t)}')
    # the bridge's conversions (its own arithmetic)
    c_ok = np.isfinite(cf[:, [0, 1, 2, 12, 13]]).all(1)
    fin3 = lambda a: np.where(np.isfinite(a).all(1)[:, None], a, np.nan)   # noqa: E731
    c_pos = _u32(np.stack([cf[:, 0], -cf[:, 1], -cf[:, 2]], 1))
    c_vel = _u32(fin3(np.stack([cf[:, 3], -cf[:, 4], -cf[:, 5]], 1)))
    c_acc = _u32(fin3(np.stack([cf[:, 6], -cf[:, 7], -cf[:, 8]], 1)))
    c_jerk = _u32(fin3(np.stack([cf[:, 9], -cf[:, 10], -cf[:, 11]], 1)))
    c_yaw = _u32(np.array([_wrap(-y) if math.isfinite(y) else math.nan for y in cf[:, 12]]))
    c_ys = _u32(-cf[:, 13])
    l_ok = (lf[:, 4] == 1) & (lf[:, 5] == 1) & np.isfinite(lf[:, :4]).all(1)
    l_pos = _u32(lf[:, 0:3])
    l_yaw = _u32(np.array([_wrap(h) if math.isfinite(h) else math.nan for h in lf[:, 3]]))
    NAN3 = _u32([np.nan] * 3)
    ZERO = _u32([0.0])[0]
    cmap, lmap = {}, {}
    for j in np.flatnonzero(c_ok):
        cmap.setdefault(c_pos[j].tobytes(), []).append(j)
    for j in np.flatnonzero(l_ok):
        lmap.setdefault(l_pos[j].tobytes(), []).append(j)
    cv_idx = np.flatnonzero(c_ok)
    lv_idx = np.flatnonzero(l_ok)

    class Sp:
        pass

    def load_sp(name):
        s = Sp()
        k = f'sp_{name}'
        o = np.argsort(R[k + '__ts'], kind='stable')
        s.recv, s.ts = R[k + '__recv'][o], R[k + '__ts'][o] * 1000
        f = R[k + '__f'][o]
        keep = s.ts <= cut
        s.recv, s.ts, f = s.recv[keep], s.ts[keep], f[keep]
        s.pos, s.vel, s.acc, s.jerk, s.yaw, s.ys = (_u32(f[:, 0:3]), _u32(f[:, 3:6]), _u32(f[:, 6:9]),
                                                    _u32(f[:, 9:12]), f[:, 12], f[:, 13])
        s.yaw_u, s.ys_u = _u32(f[:, 12]), _u32(f[:, 13])
        s.n = len(s.ts)
        ok_ = f'ocm_{name}'
        s.ocm_ts = np.sort(R[ok_ + '__ts']) * 1000
        s.ocm_recv = np.sort(R[ok_ + '__recv'])
        s.ocm_ts = s.ocm_ts[s.ocm_ts <= cut]
        s.ocm_recv = s.ocm_recv[s.ocm_recv <= cut]
        return s

    def classify(s, with_yaw):
        """kind: 0 none, 1 TRACK, 2 HOLD, 3 LATCH, 4 IDLE_CMD (IDLE, lpos invalid: SUPER's command with
        acceleration NaN); src index; back = inputs of that kind published between it and the tick."""
        kind = np.zeros(s.n, np.int8)
        src = np.full(s.n, -1)
        back = np.full(s.n, -1)
        for i in range(s.n):
            key = s.pos[i].tobytes()
            t = s.ts[i]
            for j in reversed(cmap.get(key, ())):
                if ct[j] > t:
                    continue
                ye = (not with_yaw) or (s.yaw_u[i] == c_yaw[j])
                ve = np.array_equal(s.vel[i], c_vel[j])
                ae = np.array_equal(s.acc[i], c_acc[j])
                je = np.array_equal(s.jerk[i], c_jerk[j])
                vn = np.array_equal(s.vel[i], NAN3)
                an = np.array_equal(s.acc[i], NAN3)
                jn = np.array_equal(s.jerk[i], NAN3)
                se = (not with_yaw) or (s.ys_u[i] == c_ys[j])
                s0 = (not with_yaw) or (s.ys_u[i] == ZERO)
                if ve and ae and je and ye and se:
                    kind[i] = 1
                elif vn and an and jn and ye and s0:
                    kind[i] = 2
                elif ve and an and je and ye and se:
                    kind[i] = 4
                if kind[i]:
                    src[i] = j
                    back[i] = int(np.searchsorted(ct[cv_idx], t, 'right')) - 1 - int(np.searchsorted(cv_idx, j))
                    break
            if kind[i]:
                continue
            for j in reversed(lmap.get(key, ())):
                if lt[j] > t:
                    continue
                if (np.array_equal(s.vel[i], NAN3) and np.array_equal(s.acc[i], NAN3)
                        and np.array_equal(s.jerk[i], NAN3)
                        and ((not with_yaw) or (s.yaw_u[i] == l_yaw[j] and s.ys_u[i] == ZERO))):
                    kind[i] = 3
                    src[i] = j
                    back[i] = int(np.searchsorted(lt[lv_idx], t, 'right')) - 1 - int(np.searchsorted(lv_idx, j))
                    break
        return kind, src, back

    sp = {n: load_sp(n) for n in ('servo', 'ctrl')}
    K = {}
    for n, s in sp.items():
        K[n] = classify(s, with_yaw=(n == 'ctrl'))
    say(f'outputs recorded (to the analysis cut): servo {sp["servo"].n} setpoints, ctrl {sp["ctrl"].n}; '
        f'heartbeats servo {len(sp["servo"].ocm_ts)}, ctrl {len(sp["ctrl"].ocm_ts)}')
    # ---- servo debug, paired with its tick's setpoint ------------------------------------------
    s = sp['servo']
    drecv, dd = R['dbg_servo__recv'], R['dbg_servo__data']
    o = np.argsort(drecv, kind='stable')
    drecv, dd = drecv[o], dd[o]
    # a setpoint ticked just before the cut has its debug RECEIVED just after it: pair debug received up
    # to 0.1 s past the cut, then drop only the unpaired ones past it
    keep = drecv <= cut + 100_000_000
    drecv, dd = drecv[keep], dd[keep]
    orr = np.argsort(s.recv, kind='stable')
    srecv = s.recv[orr]
    sp_idx = np.full(len(dd), -1)
    db2, db3 = _u32(dd[:, 2]), _u32(dd[:, 3])
    for j in range(len(dd)):
        q = int(np.searchsorted(srecv, drecv[j]))
        best = None
        for r_ in range(max(0, q - 4), min(len(srecv), q + 5)):
            i = orr[r_]
            if s.yaw_u[i] == db2[j] and s.ys_u[i] == db3[j] and abs(int(s.recv[i]) - int(drecv[j])) < 15_000_000:
                if best is None or abs(int(s.recv[i]) - int(drecv[j])) < abs(int(s.recv[best]) - int(drecv[j])):
                    best = i
        if best is not None:
            sp_idx[j] = best
    keep = (sp_idx >= 0) | (drecv <= cut)
    drecv, dd, sp_idx = drecv[keep], dd[keep], sp_idx[keep]
    unp = int(np.sum(sp_idx < 0))
    dts = np.where(sp_idx >= 0, s.ts[np.clip(sp_idx, 0, None)], 0)
    o = np.argsort(dts, kind='stable')
    dd, dts, drecv, sp_idx = dd[o], dts[o], drecv[o], sp_idx[o]
    good = sp_idx >= 0
    src = dd[:, 6]
    say(f'servo debug: {len(dd)} messages, {len(dd) - unp} paired to their tick\'s setpoint, {unp} unpaired')
    # ---- tick dt from the heartbeat (every tick) --------------------------------------------
    ocm_ts = s.ocm_ts

    def tick_dt(ts):
        k = np.searchsorted(ocm_ts, ts, 'left')
        prev = np.where(k > 0, ocm_ts[np.clip(k - 1, 0, None)], ts - 20_000_000)
        return (ts - prev) / 1e9

    # Two setpoints are CONSECUTIVE only if no bridge tick (heartbeat) lies between them: a takeover
    # and re-engage leaves the setpoint array contiguous across a gap of unpublished ticks, in which
    # the servo was reset (the first engaged tick after it is S11's, not a slew pair).
    k_of = np.searchsorted(ocm_ts, s.ts, 'left')
    adj = np.zeros(s.n, bool)          # adj[a]: setpoint a+1 is the tick right after setpoint a
    if s.n > 1:
        adj[:-1] = (k_of[1:] - k_of[:-1]) <= 1

    def consecutive(a, b):
        return b == a + 1 and bool(adj[a])

    say('\ninvariants (servo bridge unless named)')
    # S1 slew per pair over engaged ticks
    eng_sp = np.zeros(s.n, bool)
    eng_sp[sp_idx[good]] = True
    ia = np.flatnonzero(eng_sp)
    pairs = [(a, b) for a, b in zip(ia, ia[1:]) if consecutive(a, b)]
    n_gap = sum(1 for a, b in zip(ia, ia[1:]) if b == a + 1 and not adj[a])
    worst, nv, worst_t = 0.0, 0, None
    for a, b in pairs:
        dt = (s.ts[b] - s.ts[a]) / 1e9
        dy = abs(_wrap(float(s.yaw[b]) - float(s.yaw[a])))
        if dt <= 0 or dy > MAX_RATE * dt + SLEW_TOL:
            nv += 1
        if dt > 0 and dy / dt > worst:
            worst, worst_t = dy / dt, s.ts[b]
    inv('S1 heading slew |dyaw| <= 90 deg/s x pair dt + 0.001 deg (consecutive engaged setpoints)', nv,
        math.degrees(worst), f'{len(pairs)} pairs ({n_gap} setpoint pairs across a disengaged gap left out: S11); '
        f'worst {math.degrees(worst):.4f} deg/s'
        + (f' at t {rel(worst_t):.2f} s' if worst_t is not None else ''))
    # S2 |yawspeed| <= 90 deg/s on every setpoint
    ysa = np.abs(s.ys)
    inv('S2 |yawspeed| <= 90 deg/s (every setpoint)', int(np.sum(~(ysa <= MAX_RATE + 1e-6))),
        math.degrees(float(np.nanmax(ysa))) if s.n else 0.0, f'max {math.degrees(float(np.nanmax(ysa))):.4f} deg/s')
    # S3 ff slew in source 1 (both ticks)
    worst, nv, npair = 0.0, 0, 0
    for j in range(1, len(dd)):
        if not (good[j] and good[j - 1] and consecutive(sp_idx[j - 1], sp_idx[j]) and src[j] == 1 and src[j - 1] == 1):
            continue
        dt = (dts[j] - dts[j - 1]) / 1e9
        da = abs(float(dd[j, 3]) - float(dd[j - 1, 3]))
        npair += 1
        if dt <= 0 or da > FF_ACC * dt + FF_TOL:
            nv += 1
        if dt > 0:
            worst = max(worst, da / dt)
    inv('S3 feed-forward slew <= 90 deg/s^2 x dt + 0.001 deg/s (pairs in source 1)', nv, math.degrees(worst),
        f'{npair} pairs; worst {math.degrees(worst):.4f} deg/s^2')
    # S3b source 1/3 yawspeed within the ff cap; source 2 exactly 0
    s13 = np.isin(src, (1.0, 3.0))
    inv('S3b source 1/3: |yawspeed| <= ff cap 60 deg/s; source 2: yawspeed exactly 0',
        int(np.sum(np.abs(dd[s13, 3]) > FF_MAX + 1e-6) + np.sum(dd[src == 2, 3] != 0.0)),
        math.degrees(float(np.max(np.abs(dd[s13, 3])))) if s13.any() else 0.0,
        f'source 1/3 max {math.degrees(float(np.max(np.abs(dd[s13, 3])))) if s13.any() else 0:.3f} deg/s; '
        f'source 2 ticks {int(np.sum(src == 2))}, non-zero {int(np.sum(dd[src == 2, 3] != 0.0))}')
    # S4/S5 finite and range, both bridges
    for n_, s_ in sp.items():
        fin = np.isfinite(s_.yaw) & np.isfinite(s_.ys)
        rng_ok = np.abs(s_.yaw) <= F32_PI
        inv(f'S4 {n_}: yaw and yawspeed finite, |yaw| <= f32(pi)', int(np.sum(~fin) + np.sum(~rng_ok & fin)), None,
            f'{s_.n} setpoints; non-finite {int(np.sum(~fin))}; yaw min/max {math.degrees(np.nanmin(s_.yaw)):.3f} / '
            f'{math.degrees(np.nanmax(s_.yaw)):.3f} deg; exactly -f32(pi) {int(np.sum(s_.yaw == -F32_PI))}')
    # S6 setpoints only in Offboard (both), with the measured report -> stream latency
    for n_, s_ in sp.items():
        ob, en, since, pob, _ = state_at(s_.ts)
        # excused only: a tick within the allowance after an Offboard EXIT report
        bad = ~ob & ~(pob & (since <= EDGE_ALLOW_NS))
        late = since[~ob] / 1e6
        # latency of the first setpoint after each Offboard entry, and of the last one after each exit
        lat_on, lat_off = [], []
        for i in range(len(edges)):
            if e_off[i] and (i == 0 or not e_off[i - 1]):
                q = np.searchsorted(s_.ts, e_t[i], 'left')
                if q < s_.n and (i + 1 >= len(edges) or s_.ts[q] < e_t[i + 1]):
                    lat_on.append((s_.ts[q] - e_t[i]) / 1e6)
            if not e_off[i] and i > 0 and e_off[i - 1]:
                q = np.searchsorted(s_.ts, e_t[i], 'right') - 1
                if q >= 0:
                    lat_off.append((s_.ts[q] - e_t[i]) / 1e6)
        S['info'][f'lat_on_{n_}'] = (pct(lat_on, 50), max(lat_on) if lat_on else None)
        S['info'][f'lat_off_{n_}'] = (min(lat_off) if lat_off else None, max(lat_off) if lat_off else None)
        inv(f'S6 {n_}: no setpoint outside Offboard (allowance {EDGE_ALLOW_NS / 1e6:.0f} ms after an Offboard-off '
            f'report)', int(bad.sum()), float(late.max()) if len(late) else None,
            f'{int(np.sum(~ob))} setpoints ticked after an Offboard-off report was published, all within '
            f'{late.max() if len(late) else 0:.2f} ms of it; first setpoint after each of {len(lat_on)} Offboard entries: '
            f'p50 {pct(lat_on, 50):.1f} max {max(lat_on) if lat_on else math.nan:.1f} ms; last setpoint relative to each '
            f'of {len(lat_off)} Offboard exits: {min(lat_off) if lat_off else math.nan:+.1f} .. '
            f'{max(lat_off) if lat_off else math.nan:+.1f} ms')
    # S7 debug only while engaged
    ob, en, since, _, pen = state_at(dts[good])
    bad = ~en & ~(pen & (since <= EDGE_ALLOW_NS))
    lat_d = since[~en] / 1e6
    ob_s, en_s, since_s, _, _ = state_at(s.ts)
    miss = int(np.sum(en_s & (since_s > EDGE_ALLOW_NS) & ~eng_sp))
    inv('S7 debug only while engaged, and on every engaged tick', int(bad.sum()) + miss + unp,
        float(lat_d.max()) if len(lat_d) else None,
        f'{int(np.sum(~en))} debug ticks after a disengaging report, all within '
        f'{lat_d.max() if len(lat_d) else 0:.2f} ms; engaged setpoints (> {EDGE_ALLOW_NS / 1e6:.0f} ms after the engage '
        f'report) without a debug message: {miss}; unpaired debug {unp}')
    # S8 IDLE in disarmed Offboard: the latch (both bridges)
    for n_, s_ in sp.items():
        kind, srcj, back = K[n_]
        ob, en, since, _, _ = state_at(s_.ts)
        idle = ob & ~en & (since > EDGE_ALLOW_NS)
        ii = np.flatnonzero(idle)
        nl = int(np.sum(kind[ii] == 3))
        ni = int(np.sum(kind[ii] == 4))
        bad_kind = int(np.sum(~np.isin(kind[ii], (3, 4))))
        # a LATCH must be the latest valid lpos published before the tick or the one before it; an
        # IDLE_CMD only when the latest lpos published before the tick was not valid
        stale_latch = 0
        yaw_bad = 0
        bk = []
        for i in ii:
            if kind[i] == 3:
                bk.append(back[i])
                stale_latch += back[i] > 1
                yaw_bad += not (s_.yaw_u[i] == l_yaw[srcj[i]] and s_.ys_u[i] == ZERO)
            elif kind[i] == 4:
                kl = np.searchsorted(lt, s_.ts[i], 'right') - 1
                stale_latch += bool(l_ok[kl]) and bool(l_ok[max(0, kl - 1)])
                yaw_bad += not (s_.yaw_u[i] == c_yaw[srcj[i]] and s_.ys_u[i] == c_ys[srcj[i]])
        stale_latch += yaw_bad
        yawchk = f'; yaw/yawspeed not exactly the latch\'s (or the command\'s) on {yaw_bad}'
        inv(f'S8 {n_}: IDLE in disarmed Offboard sends the latch (yaw = f32(wrap(heading)) of the latest valid lpos, '
            f'yawspeed 0, the rest NaN)', bad_kind + stale_latch, None,
            f'{len(ii)} IDLE setpoints: {nl} latch ({np.bincount(np.array(bk, int), minlength=2)[:3].tolist() if bk else []} '
            f'by lpos messages behind the latest valid), {ni} SUPER\'s command with acceleration NaN while the lpos was '
            f'invalid, {bad_kind} other, {stale_latch} with a stale source{yawchk}')
    # S9 servo == ctrl for position/velocity/acceleration/jerk (same input, same kind); all classified
    ks, ss_, _ = K['servo']
    kc, sc_, _ = K['ctrl']
    ref = {}
    sc_sp = sp['ctrl']
    for i in range(sc_sp.n):
        if kc[i]:
            ref.setdefault((int(kc[i]), int(sc_[i])), sc_sp.pos[i].tobytes() + sc_sp.vel[i].tobytes()
                           + sc_sp.acc[i].tobytes() + sc_sp.jerk[i].tobytes())
    ncmp = ndiff = 0
    for i in range(s.n):
        kk = (int(ks[i]), int(ss_[i]))
        if ks[i] and kk in ref:
            ncmp += 1
            ndiff += (s.pos[i].tobytes() + s.vel[i].tobytes() + s.acc[i].tobytes() + s.jerk[i].tobytes()) != ref[kk]
    nun_s, nun_c = int(np.sum(ks == 0)), int(np.sum(kc == 0))
    kinds_s = np.bincount(ks, minlength=5).tolist()
    kinds_c = np.bincount(kc, minlength=5).tolist()
    inv('S9 servo vs ctrl: position/velocity/acceleration/jerk identical for the same input; every setpoint of '
        'both is the exact conversion of an input (servo ignoring yaw/yawspeed)', ndiff + nun_s, None,
        f'{ncmp} servo setpoints with a (kind, input) the ctrl bridge also sent, {ndiff} differ; unclassified servo '
        f'{nun_s} of {s.n} (kinds none/TRACK/HOLD/LATCH/IDLE_CMD {kinds_s})')
    inv('S10 ctrl (switch off): yaw and yawspeed exactly SUPER\'s (-yaw wrapped, -yaw_dot) or the latch/HOLD value',
        nun_c, None, f'unclassified {nun_c} of {sc_sp.n} (kinds {kinds_c})')
    # S11 the first engaged setpoint after each engage: within one slew step of the vehicle heading
    nfirst = nv = n_nan_h = 0
    worst = -math.inf
    det = []
    for i in range(len(edges)):
        if not (e_eng[i] and (i == 0 or not e_eng[i - 1])):
            continue
        q = np.searchsorted(dts[good], e_t[i], 'left')
        dgt = dts[good]
        if q >= len(dgt) or (i + 1 < len(edges) and dgt[q] >= e_t[i + 1]):
            continue
        ti = dgt[q]
        isp = int(np.searchsorted(s.ts, ti, 'left'))
        dt = float(tick_dt(np.array([ti]))[0])
        lim = MAX_RATE * min(dt, 0.1) + SLEW_TOL
        kl = int(np.searchsorted(lt, ti, 'right')) - 1
        # the bridge held the latest lpos published before the tick, or (delivery) the one before
        hs = [lf[q_, 3] for q_ in (kl, kl - 1) if q_ >= 0]
        refs = [(h, 'heading') for h in hs if math.isfinite(h)]
        if any(not math.isfinite(h) for h in hs):
            # a NaN heading: the servo starts from SUPER's yaw (the setpoint's own source)
            kk = int(ks[isp])
            j = int(ss_[isp])
            ref_y = (float(np.asarray(c_yaw[j]).view(np.float32)) if kk in (1, 2, 4)
                     else float(np.asarray(l_yaw[j]).view(np.float32)) if kk == 3 else math.nan)
            if math.isfinite(ref_y):
                refs.append((ref_y, 'SUPER yaw (heading NaN)'))
            n_nan_h += 1
        y = float(s.yaw[isp])
        nfirst += 1
        err, what = min(((abs(_wrap(y - r_)), w_) for r_, w_ in refs), default=(math.inf, 'nothing finite'))
        ok = err <= lim
        worst = max(worst, err - lim)
        if not ok:
            nv += 1
            det.append(f't {rel(ti):.2f}: |yaw - {what}| {math.degrees(err):.3f} deg > {math.degrees(lim):.3f}')
    inv('S11 first engaged setpoint after each engage within one slew step (90 deg/s x tick dt) of the vehicle heading',
        nv, math.degrees(worst), f'{nfirst} engages ({n_nan_h} with the heading NaN: checked against SUPER\'s yaw, '
        f'the servo\'s start then); smallest margin {math.degrees(-worst):+.5f} deg'
        + ('; ' + '; '.join(det[:5]) if det else ''))
    # S12 stale hold <= 0.5 s + one tick
    runs = []
    j = 0
    while j < len(dd):
        if src[j] == 3.0 and good[j]:
            j2 = j
            while j2 < len(dd) and src[j2] == 3.0 and good[j2] and (j2 == j or consecutive(sp_idx[j2 - 1], sp_idx[j2])):
                j2 += 1
            prev_t = dts[j - 1] if j > 0 and good[j - 1] and consecutive(sp_idx[j - 1], sp_idx[j]) else None
            acc_t = (dts[j2 - 1] - prev_t) / 1e9 if prev_t is not None else math.nan
            nxt_src = src[j2] if j2 < len(dd) and good[j2] and consecutive(sp_idx[j2 - 1], sp_idx[j2]) else -1
            runs.append((j2 - j, acc_t, dts[j], nxt_src))
            j = j2
        else:
            j += 1
    acc = [r[1] for r in runs if math.isfinite(r[1])]
    nlong = sum(1 for r in runs if (math.isfinite(r[1]) and r[1] > 0.5 + 0.02 + 1e-3) or r[0] > 26)
    ends = {}
    for r in runs:
        ends[int(r[3])] = ends.get(int(r[3]), 0) + 1
    inv('S12 stale hold (source 3) never longer than 0.5 s + one tick', nlong, max(acc) if acc else None,
        f'{len(runs)} holds; longest {max((r[0] for r in runs), default=0)} ticks, {max(acc) if acc else 0:.4f} s from '
        f'the last aimed tick to the last hold tick; ended in source {ends} (-1: the engagement ended during it)')
    # S13 close range with hysteresis
    rng = dd[:, 4]
    n1 = int(np.sum((src == 1) & ~(rng >= 2.5 - 1e-6)))
    n2 = int(np.sum((src == 2) & ~(rng < 2.8 + 1e-6)))
    t12 = t21 = b12 = b21 = 0
    for j in range(1, len(dd)):
        if not (good[j] and good[j - 1] and consecutive(sp_idx[j - 1], sp_idx[j])):
            continue
        if src[j - 1] in (0.0, 1.0) and src[j] == 2.0:
            t12 += 1
            b12 += not (rng[j] < 2.5 + 1e-6)
        if src[j - 1] == 2.0 and src[j] == 1.0:
            t21 += 1
            b21 += not (rng[j] >= 2.8 - 1e-6)
    inv('S13 close range: entered below 2.5 m, left at >= 2.8 m (source 1 => range >= 2.5; source 2 => < 2.8)',
        n1 + n2 + b12 + b21, None, f'source-1 ticks under 2.5 m {n1}, source-2 ticks at >= 2.8 m {n2}; entries '
        f'{t12} ({b12} not below 2.5), exits to source 1 {t21} ({b21} below 2.8); tolerance 1e-6 m (float32 range)')
    # S14 heartbeat gaps (both)
    for n_, s_ in sp.items():
        g1 = np.diff(s_.ocm_ts) / 1e6
        g2 = np.diff(s_.ocm_recv) / 1e6
        inv(f'S14 {n_}: offboard_control_mode never lapses more than {HB_MAX_MS:.0f} ms',
            int(np.sum(g1 > HB_MAX_MS) + np.sum(g2 > HB_MAX_MS)), float(max(g1.max(), g2.max())),
            f'{len(s_.ocm_ts)} heartbeats, {(len(s_.ocm_ts) - 1) / ((s_.ocm_ts[-1] - s_.ocm_ts[0]) / 1e9):.3f} Hz; '
            f'longest gap {g1.max():.2f} ms by its stamps, {g2.max():.2f} ms by receive time')
    # ---- what the scenario exercised ---------------------------------------------------------
    secs = {int(k_): float(np.sum(src == k_)) * 0.02 for k_ in (0, 1, 2, 3)}
    ntr = int(np.sum(src[1:] != src[:-1]))
    fb_rate = dd[(src == 0), 3]
    say(f'\nexercised: servo seconds per source 0/1/2/3 {secs[0]:.1f} / {secs[1]:.1f} / {secs[2]:.1f} / {secs[3]:.1f} '
        f'({ntr} changes); fall-back ticks passing SUPER\'s rate {int(np.sum(fb_rate != 0))}, at the 90 deg/s clamp '
        f'{int(np.sum(np.abs(np.abs(fb_rate) - MAX_RATE) < 1e-6))}; |ff| at the 60 deg/s cap on '
        f'{int(np.sum(np.abs(np.abs(dd[src == 1, 3]) - FF_MAX) < 1e-6))} source-1 ticks; slew-binding pairs '
        f'{sum(1 for a, b in pairs if abs(_wrap(float(s.yaw[b]) - float(s.yaw[a]))) > MAX_RATE * (s.ts[b] - s.ts[a]) / 1e9 - 1e-6)}')
    S['info']['secs'] = secs
    # ---- ticks: first vs last 5 min --------------------------------------------------------------
    for n_, s_ in sp.items():
        g = np.diff(s_.ocm_ts) / 1e6
        tt = s_.ocm_ts[1:]
        a_ = g[tt < s_.ocm_ts[0] + 300e9]
        b_ = g[tt > s_.ocm_ts[-1] - 300e9]
        S['info'][f'tick_{n_}'] = ((pct(a_, 50), pct(a_, 99), float(a_.max())), (pct(b_, 50), pct(b_, 99), float(b_.max())))
        say(f'tick {n_} (heartbeat stamps, every tick): first 5 min p50/p99/max {pct(a_, 50):.3f} / {pct(a_, 99):.3f} / '
            f'{a_.max():.3f} ms; last 5 min {pct(b_, 50):.3f} / {pct(b_, 99):.3f} / {b_.max():.3f} ms')
    return L, S


def trends(out):
    """CPU / RSS / load from samples.jsonl, and the bridge logs."""
    import numpy as np
    L = []
    rows = [json.loads(x) for x in open(os.path.join(out, 'samples.jsonl'))]
    if len(rows) < 3:
        return ['too few samples'], {}
    S = {}
    t = np.array([r['t'] for r in rows]) - rows[0]['t']
    for n in ('servo', 'ctrl', 'driver', 'recorder'):
        cpu = np.array([r['procs'].get(n, {}).get('cpu_pct', np.nan) for r in rows], float)
        rss = np.array([r['procs'].get(n, {}).get('rss_mb', np.nan) for r in rows], float)
        ok = np.isfinite(rss)
        if not ok.any():
            continue
        m = ok & (t >= 60.0)
        if m.sum() > 3:
            A = np.vstack([t[m] / 3600.0, np.ones(m.sum())]).T
            coef, res, *_ = np.linalg.lstsq(A, rss[m], rcond=None)
            resid = rss[m] - A @ coef
            slope, sd = coef[0], float(np.std(resid))
        else:
            slope, sd = math.nan, math.nan
        r1 = rss[np.argmin(np.abs(t - 60.0))]
        S[n] = dict(cpu_mean=float(np.nanmean(cpu[1:])), cpu_max=float(np.nanmax(cpu[1:])), rss_1min=float(r1),
                    rss_end=float(rss[ok][-1]), rss_min=float(np.nanmin(rss)), rss_max=float(np.nanmax(rss)),
                    slope=float(slope), sd=sd)
        L.append(f'{n:8s}: CPU % of one core mean {S[n]["cpu_mean"]:.2f} max {S[n]["cpu_max"]:.2f}; RSS MB at 1 min '
                 f'{r1:.2f}, at the end {S[n]["rss_end"]:.2f} (min {S[n]["rss_min"]:.2f} max {S[n]["rss_max"]:.2f}); '
                 f'linear fit from 1 min: {slope:+.3f} MB/hour, residual sd {sd:.3f} MB')
    if 'servo' in S and 'ctrl' in S:
        d = np.array([r['procs'].get('servo', {}).get('cpu_pct', np.nan) - r['procs'].get('ctrl', {}).get('cpu_pct', np.nan)
                      for r in rows[1:]], float)
        L.append(f'CPU servo - ctrl: mean {np.nanmean(d):+.2f} p95 {np.nanpercentile(d, 95):+.2f} max {np.nanmax(d):+.2f} '
                 f'percentage points of one core')
        S['cpu_diff'] = float(np.nanmean(d))
    la = np.array([r['load'][0] for r in rows])
    ma = np.array([r['mem_avail_mb'] for r in rows])
    L.append(f'machine: load (1 min) mean {la.mean():.2f} min {la.min():.2f} max {la.max():.2f} (8 cores); MemAvailable '
             f'MB mean {ma.mean():.0f} min {ma.min():.0f} max {ma.max():.0f}')
    S['load'] = (float(la.mean()), float(la.max()))
    # logs
    rx = re.compile(r'^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[(\d+)\.(\d+)\] \[([^\]]+)\]: (.*)$')
    for n in ('servo', 'ctrl'):
        sev = {}
        nl = 0
        for ln in open(os.path.join(out, f'bridge_{n}.log'), errors='replace').read().splitlines():
            nl += 1
            m = rx.match(ln)
            if m:
                if m.group(1) in ('WARN', 'ERROR', 'FATAL'):
                    sev.setdefault((m.group(1), norm(m.group(5))), []).append(m.group(5))
            elif ln.strip():
                sev.setdefault(('STDERR', norm(ln)), []).append(ln)
        L.append(f'log {n}: {nl} lines; WARN/ERROR/FATAL/other stderr, distinct {len(sev)}:')
        for (sv, _), x in sorted(sev.items(), key=lambda kv: -len(kv[1])):
            L.append(f'    {len(x):5d} x {sv}: "{x[0][:170]}"')
        S[f'log_{n}'] = {f'{a} {b}': len(x) for (a, b), x in sev.items()}
    return L, S


# ---------------------------------------------------------------------------------------------
# orchestrator
# ---------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--role', default='main', help=argparse.SUPPRESS)
    ap.add_argument('--config', help=argparse.SUPPRESS)
    ap.add_argument('--out', default=os.path.join(os.getcwd(), 'yaw_servo_soak_out'))
    ap.add_argument('--duration', type=float, default=2700.0, help='s of soak (default 45 min)')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--domain', type=int, default=96)
    ap.add_argument('--analyze', action='store_true', help='(re-)analyse a finished soak in --out')
    ap.add_argument('--partial', action='store_true', help=argparse.SUPPRESS)
    ap.add_argument('--analysis-every', type=float, default=300.0,
                    help='s between the invariant analyses of the data so far (progress.json)')
    a = ap.parse_args()
    if a.role == 'driver':
        role_driver(a.config)
        return 0
    if a.role == 'recorder':
        role_recorder(a.config)
        return 0
    out = a.out
    if a.analyze:
        L, S = analyse(out, partial=a.partial)
        name = 'analysis_partial' if a.partial else 'analysis'
        if not a.partial:
            L2, S2 = trends(out)
            L += ['', 'trends'] + L2
            S['trends'] = S2
        with open(os.path.join(out, name + '.txt'), 'w') as f:
            f.write('\n'.join(L) + '\n')
        with open(os.path.join(out, name + '.json'), 'w') as f:
            json.dump(S, f, indent=1, default=str)
        if not a.partial:
            print('\n'.join(L))
        return 0
    os.makedirs(out, exist_ok=True)
    if glob.glob(os.path.join(out, 'rec_*.npz')) or glob.glob(os.path.join(out, 'drv_*.npz')):
        sys.exit(f'{out} already holds a soak; use a fresh --out')
    env_setup(a.domain, os.path.join(out, 'roslog'))
    if not os.path.isfile(BRIDGE):
        sys.exit(f'bridge not found: {BRIDGE}')
    bad = isolation_check(a.domain)
    if bad:
        sys.exit('isolation check failed:\n  ' + '\n  '.join(bad))
    seed = a.seed if a.seed is not None else int(time.time()) % 1_000_000
    meta = {'seed': seed, 'duration': a.duration, 'bridge': BRIDGE, 'bridge_realpath': os.path.realpath(BRIDGE),
            'domain': a.domain, 'ns': NS, 'bridges': BRIDGES, 'started': time.time()}
    with open(os.path.join(out, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f'soak: seed {seed}, {a.duration:.0f} s, bridge {BRIDGE} -> {os.path.realpath(BRIDGE)}, domain '
          f'{a.domain}, ROS_LOCALHOST_ONLY=1, sandbox {NS}, out {out}', flush=True)
    t_build = time.time()
    Scenario(seed, a.duration)       # build once here, so a scenario error stops us before anything runs
    print(f'  scenario built in {time.time() - t_build:.1f} s', flush=True)
    procs = {}
    rcfg = {'out': out, 'bridges': {n: {'sp': f'{NS}/{n}/trajectory_setpoint', 'ocm': f'{NS}/{n}/offboard_control_mode',
                                        'dbg': f'{NS}/{n}/yaw_servo' if BRIDGES[n] else None} for n in BRIDGES}}
    with open(os.path.join(out, 'recorder.json'), 'w') as f:
        json.dump(rcfg, f)
    dcfg = {'out': out, 'seed': seed, 'duration': a.duration,
            'want': {'pos_cmd': 2, 'yaw_target': 1, 'vehicle_local_position': 2, 'vehicle_control_mode': 2}}
    with open(os.path.join(out, 'driver.json'), 'w') as f:
        json.dump(dcfg, f)
    me = os.path.realpath(__file__)
    samples = open(os.path.join(out, 'samples.jsonl'), 'a')
    prog = {'seed': seed, 'duration_s': a.duration, 'status': 'starting'}
    last_an = None
    an_proc = None
    ok_run = False

    def write_progress():
        tmp = os.path.join(out, '.progress.json')
        with open(tmp, 'w') as f:
            json.dump(prog, f, indent=1, default=str)
        os.replace(tmp, os.path.join(out, 'progress.json'))

    try:
        for n, extra in BRIDGES.items():
            procs[n] = start_proc(bridge_argv(n, extra), os.path.join(out, f'bridge_{n}.log'))
        procs['recorder'] = start_proc([sys.executable, me, '--role', 'recorder', '--config',
                                        os.path.join(out, 'recorder.json')], os.path.join(out, 'recorder.log'))
        procs['driver'] = start_proc([sys.executable, me, '--role', 'driver', '--config',
                                      os.path.join(out, 'driver.json')], os.path.join(out, 'driver.log'))
        meta['pids'] = {k: p.pid for k, p in procs.items()}
        with open(os.path.join(out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        # discovery: the driver waits for its inputs to match; here: the recorder side
        t_end = time.time() + 45
        while time.time() < t_end:
            dead = [k for k, p in procs.items() if p.poll() is not None]
            if dead:
                raise RuntimeError(f'died during start-up: {dead}')
            try:
                st = json.load(open(os.path.join(out, 'rec_status.json')))
                dr = json.load(open(os.path.join(out, 'driver_ready.json')))
            except Exception:
                time.sleep(0.2)
                continue
            c, g = st.get('counts', {}), st.get('graph', {})
            if (all(c.get(f'ocm_{n}', 0) >= 25 for n in BRIDGES) and g.get('dbg_servo') == [True]
                    and g.get('sp_servo') == [True] and g.get('sp_ctrl') == [True]
                    and dr['matched'] == dr['want']):
                break
            time.sleep(0.2)
        else:
            raise RuntimeError('discovery timed out')
        # the sandbox: no /fmu, /planning, /bridge topic on the domain; each bridge's own topics
        import rclpy
        from rclpy.node import Node
        rclpy.init()
        gnode = Node('soak_graph_check')
        t_g = time.time() + 15.0
        while time.time() < t_g and not all(f'soak_bridge_{n}' in gnode.get_node_names() for n in BRIDGES):
            time.sleep(0.2)
        time.sleep(0.5)
        topics = sorted(t for t, _ in gnode.get_topic_names_and_types())
        gr = {n: sorted(t for t, _ in gnode.get_subscriber_names_and_types_by_node(f'soak_bridge_{n}', '/'))
              + sorted(t for t, _ in gnode.get_publisher_names_and_types_by_node(f'soak_bridge_{n}', '/'))
              for n in BRIDGES}
        gnode.destroy_node()
        rclpy.shutdown()
        leak = [t for t in topics if t.startswith(('/fmu', '/planning', '/bridge'))]
        foreign = [t for v in gr.values() for t in v if not t.startswith(NS + '/')
                   and t not in ('/rosout', '/parameter_events')]
        meta['graph'] = {'topics': topics, 'bridges': gr, 'driver_ready': dr, 'recorder': st}
        with open(os.path.join(out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        if leak or foreign:
            raise RuntimeError(f'sandbox check failed: leak {leak} foreign {foreign}')
        print(f'  discovery OK: driver inputs matched {dr["matched"]}; recorder {st["counts"]}; no /fmu /planning '
              f'/bridge topic on domain {a.domain}', flush=True)
        open(os.path.join(out, 'go'), 'w').close()
        # ---- the soak --------------------------------------------------------------------
        t_start = time.time()
        prev = {}
        next_sample = t_start
        next_an = t_start + a.analysis_every
        prog['status'] = 'running'
        while True:
            now = time.time()
            el = now - t_start
            dead = [k for k, p in procs.items() if p.poll() is not None]
            if 'driver' in dead and el > a.duration:
                break
            if dead:
                prog['status'] = f'FAILED: died {dead} at {el:.0f} s'
                write_progress()
                raise RuntimeError(prog['status'])
            if now >= next_sample:
                row = {'t': now, 'el': el, 'procs': {}}
                for k, p in procs.items():
                    st_ = proc_stat(p.pid)
                    if st_ is None:
                        continue
                    cpu, rss = st_
                    d = {'rss_mb': rss, 'cpu_s': cpu}
                    if k in prev:
                        d['cpu_pct'] = 100.0 * (cpu - prev[k][0]) / (now - prev[k][1])
                    prev[k] = (cpu, now)
                    row['procs'][k] = d
                la, mav, mfree = machine_stat()
                row['load'], row['mem_avail_mb'], row['mem_free_mb'] = la, mav, mfree
                samples.write(json.dumps(row) + '\n')
                samples.flush()
                try:
                    rc = json.load(open(os.path.join(out, 'rec_status.json')))['counts']
                    dc = json.load(open(os.path.join(out, 'drv_status.json')))['counts']
                except Exception:
                    rc, dc = {}, {}
                prog.update({'elapsed_s': round(el, 1), 'recorded': rc, 'published': dc,
                             'alive': {k: p.poll() is None for k, p in procs.items()},
                             'last_sample': row})
                if an_proc is not None and an_proc.poll() is not None:
                    try:
                        S = json.load(open(os.path.join(out, 'analysis_partial.json')))
                        prog['violations_so_far'] = {k: v['n'] for k, v in S['viol'].items()}
                        prog['violations_total'] = sum(v['n'] for v in S['viol'].values())
                        prog['analysed_at_s'] = last_an
                    except Exception as e:
                        prog['analysis_error'] = repr(e)
                    an_proc = None
                write_progress()
                next_sample += 10.0
            if now >= next_an and an_proc is None:
                an_proc = start_proc([sys.executable, me, '--analyze', '--partial', '--out', out],
                                     os.path.join(out, 'analysis_partial.log'), nice=10)
                last_an = round(el, 1)
                next_an += a.analysis_every
            time.sleep(0.5)
        ok_run = True
        prog['status'] = 'soak done; stopping'
        write_progress()
    finally:
        rc = {}
        if an_proc is not None:
            an_proc.wait()
        for k in ('driver', 'recorder', 'servo', 'ctrl'):
            if k in procs:
                if k == 'recorder':
                    time.sleep(1.5)
                rc[k] = stop_proc(procs[k])
        alive = [k for k, p in procs.items() if p.poll() is None or os.path.exists(f'/proc/{p.pid}')]
        left = domain_users(a.domain, _ancestors())
        meta['exit_codes'] = rc
        meta['alive_after_stop'] = alive
        meta['domain_left'] = left
        meta['ended'] = time.time()
        with open(os.path.join(out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        samples.close()
        prog['status'] = ('completed' if ok_run else prog.get('status', 'FAILED'))
        prog['exit_codes'] = rc
        prog['alive_after_stop'] = alive
        prog['domain_left'] = left
        write_progress()
        print(f'  stopped: exit codes {rc}; still alive by PID: {alive or "none"}; processes on domain '
              f'{a.domain}: {left or "none"}', flush=True)
    L, S = analyse(out)
    L2, S2 = trends(out)
    L += ['', 'trends'] + L2
    S['trends'] = S2
    with open(os.path.join(out, 'analysis.txt'), 'w') as f:
        f.write('\n'.join(L) + '\n')
    with open(os.path.join(out, 'analysis.json'), 'w') as f:
        json.dump(S, f, indent=1, default=str)
    print('\n'.join(L))
    return 0


if __name__ == '__main__':
    sys.exit(main())
