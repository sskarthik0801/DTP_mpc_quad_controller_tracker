#!/usr/bin/env python3
"""What does adding the follow stack to the navigation stack actually cost on one Orin NX?

WHY THIS REPLACES concurrent_stack_test.py's NUMBERS RATHER THAN REFINING THEM
-----------------------------------------------------------------------------
The 27.6 % CPU / 6.5 GB / 66 % GPU / 18.5 W / 71 C figure this project has been quoting since
2026-09-07 was not measuring the real system, and its own docstring says so:

    "fusion_node / perception_node are still M0.1 scaffolds -- they store parameters and do no
     work. Measuring them would measure nothing. So the follower side is represented by its two
     real costs ... an honest lower bound on the real thing, not the real thing."

So the follower half was a standalone YOLO benchmark standing in for nodes that did nothing. The
real perception_node was MEASURED at ~250 % CPU on 2026-09-11 -- two and a half cores -- because
its cost is dominated by JPEG decode and the organized-cloud build in numpy, not by inference.
The old number therefore understates our half structurally, not just imprecisely. It was also
taken with GPU clocks unpinned, and every unpinned number on this project has turned out to be
about half the truth.

THE NUMBER THE PAPER ACTUALLY NEEDS IS MARGINAL, NOT TOTAL
----------------------------------------------------------
"The whole stack uses N %" is not a useful claim: most of it is Kevin's navigation stack, which
would be flying anyway. The claim worth publishing is what OUR stack adds to a platform that is
already doing SLAM and planning. So this is run twice:

    phase A   navigation stack only          (baseline)
    phase B   navigation + follow stack      (what we actually fly)

and `--compare A.json B.json` prints the delta. Run A with the follow stack STOPPED and B with it
running; nothing else about the box may change between them.

"IT FITS" IS A STATEMENT ABOUT RATES, NOT ABOUT CPU
---------------------------------------------------
Kept from the original script, because it is the right criterion: a stack can sit at 70 % CPU and
be broken. A component below 85 % of its nominal rate is failing whatever the CPU says -- and on
2026-09-11 that was exactly the failure mode (perception saturating its executor and starving its
own transform callbacks while every CPU number looked fine).

EVERY NUMBER NAMES ITS INSTRUMENT, AND AN UNAVAILABLE ONE IS NOT A ZERO
----------------------------------------------------------------------
Three measuring instruments have given this project confidently wrong answers (a rate tool that
under-reports on a multi-MB/s payload under load, a delivery counter mistaken for a capture
counter, a failure proxy that scored a standing person as a blind camera). So each reading is
printed with the file or tool it came from, and anything unreadable prints "unavailable (tried:
...)" rather than 0.0 -- a zero that looks like a measurement is how the first two of those got
into the record.

    python3 scripts/measure_coresidency.py --phase A --seconds 60 --out /tmp/phaseA.json
    python3 scripts/measure_coresidency.py --phase B --seconds 60 --out /tmp/phaseB.json
    python3 scripts/measure_coresidency.py --compare /tmp/phaseA.json /tmp/phaseB.json
"""
import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

# Process attribution. comm is truncated to 15 chars in /proc/<pid>/stat, and the PX4 bridges are
# python3 scripts whose comm is just "python3" -- so everything is matched on the full cmdline and
# the first pattern that hits wins. Order matters: OURS before NAV before OTHER.
GROUPS = (
    ('follow', (
        'dtp_perception/perception_node', 'dtp_perception/target_manager',
        'dtp_perception/sensor_watchdog', 'dtp_perception/static_calib_tf',
        'dtp_perception/fusion_node', 'follow_planner/follow_planner_node',
        'command_mux/command_mux_node',
    )),
    ('camera', ('camera_container', 'OBCameraNode', 'orbbec_camera')),
    ('nav', (
        'fast_lio/fastlio_mapping', 'super_planner/fsm_node',
        'livox_ros_driver2/livox_ros_driver2_node',
        'px4_odom_bridge', 'px4_offboard_bridge',
    )),
    ('other_ros', ('/opt/ros/', 'ros2 launch', 'ros2 run', 'Micro-XRCE', 'mavlink-router')),
)
# Nominal rates and the 85 % compliance floor. A component under its floor is failing.
#
# *** ONLY SMALL TOPICS ARE SUBSCRIBED, AND THAT IS A CORRECTNESS REQUIREMENT. ***
# A first version also subscribed /livox/lidar (a full scan) and /cloud_registered_body (a
# multi-megabyte PointCloud2 at 10 Hz). Deserialising those in Python saturated THIS PROCESS'S
# executor and cost it ~28 % of ALL messages on ALL topics -- it reported /lidar_slam/odom at
# 7.2 Hz while `ros2 topic hz` measured 10.000 Hz in the same minute. The measuring instrument
# was the thing that was overloaded, which is the trap this project has now hit four times and
# already wrote down: probe the small COMPANION topic, never the heavy payload.
#
# /livox/imu stands in for 'the LiDAR driver is alive and keeping time' at ~200 Hz and ~50 B a
# message. It does NOT prove the scan rate; nothing small can, so the scan rate is reported as
# not-measured-here rather than guessed.
RATES = (
    ('/livox/imu', 200.0),
    ('/lidar_slam/odom', 10.0),
    ('/camera/color/camera_info', 30.0),
    ('/camera/depth/camera_info', 30.0),
    ('/dtp/targets', 10.0),
)
# Subscribing to these would corrupt the measurement (see above). Named so the report can say
# they were deliberately skipped instead of silently omitting them.
HEAVY_TOPICS_NOT_SUBSCRIBED = ('/livox/lidar', '/cloud_registered_body')
RATE_FLOOR = 0.85
GPU_DEVFREQ = '/sys/class/devfreq/17000000.gpu'
# GPU utilisation lives in a different place on every JetPack. Probed in order; the one that
# answers is named in the report so the number is traceable.
GPU_LOAD_PATHS = (
    '/sys/devices/gpu.0/load',
    '/sys/devices/platform/gpu.0/load',
    '/sys/class/devfreq/17000000.gpu/device/load',
)
CLK = 100.0   # USER_HZ; os.sysconf('SC_CLK_TCK') is read at runtime and this is only a fallback


def read_first(paths):
    """(text, path) from the first readable path, else (None, tried-list)."""
    for p in paths:
        try:
            with open(p) as f:
                return f.read().strip(), p
        except OSError:
            continue
    return None, list(paths)


def proc_table():
    """{pid: (group, cmdline)} for every process we care to attribute."""
    out = {}
    for d in glob.glob('/proc/[0-9]*'):
        pid = os.path.basename(d)
        try:
            with open(os.path.join(d, 'cmdline'), 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()
        except OSError:
            continue
        if not cmd:
            continue
        for group, pats in GROUPS:
            if any(pat in cmd for pat in pats):
                out[pid] = (group, cmd)
                break
    return out


def proc_cpu_ticks(pid):
    """utime+stime in ticks, or None if the process is gone."""
    try:
        with open('/proc/%s/stat' % pid) as f:
            parts = f.read().rsplit(')', 1)[1].split()
        return int(parts[11]) + int(parts[12])      # utime, stime (fields 14,15 1-indexed)
    except (OSError, IndexError, ValueError):
        return None


def proc_rss_kb(pid):
    try:
        with open('/proc/%s/status' % pid) as f:
            for line in f:
                if line.startswith('VmRSS:'):
                    return int(line.split()[1])
    except OSError:
        pass
    return None


def sys_cpu_ticks():
    """(busy, total) from /proc/stat."""
    with open('/proc/stat') as f:
        parts = [int(x) for x in f.readline().split()[1:]]
    idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
    return sum(parts) - idle, sum(parts)


def mem_used_gb():
    total = avail = None
    with open('/proc/meminfo') as f:
        for line in f:
            if line.startswith('MemTotal:'):
                total = int(line.split()[1])
            elif line.startswith('MemAvailable:'):
                avail = int(line.split()[1])
    if total is None or avail is None:
        return None, None
    return (total - avail) / 1048576.0, total / 1048576.0


def thermals():
    out = {}
    for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        try:
            with open(os.path.join(z, 'type')) as f:
                name = f.read().strip()
            with open(os.path.join(z, 'temp')) as f:
                out[name] = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
    return out


def power_rails_mw():
    """{label: mW} from the INA3221 hwmon nodes, plus where it was read from."""
    out = {}
    src = None
    for h in sorted(glob.glob('/sys/class/hwmon/hwmon*')):
        try:
            with open(os.path.join(h, 'name')) as f:
                if 'ina' not in f.read().strip().lower():
                    continue
        except OSError:
            continue
        for lab in sorted(glob.glob(os.path.join(h, 'in*_label'))):
            try:
                with open(lab) as f:
                    label = f.read().strip()
                idx = re.search(r'in(\d+)_label', lab).group(1)
                cur = os.path.join(h, 'curr%s_input' % idx)
                volt = os.path.join(h, 'in%s_input' % idx)
                with open(cur) as f:
                    ma = int(f.read().strip())
                with open(volt) as f:
                    mv = int(f.read().strip())
                out[label] = ma * mv / 1000.0
                src = h
            except (OSError, ValueError, AttributeError):
                continue
    return out, src


def tegrastats_sample(seconds):
    """Mean GPU %, total power mW and peak temp from tegrastats, or None if unavailable.

    tegrastats is preferred for GPU and power because it reports both in one place and is the
    tool NVIDIA supports; the sysfs paths differ between JetPack releases.
    """
    exe = shutil.which('tegrastats')
    if exe is None:
        return None
    try:
        p = subprocess.Popen([exe, '--interval', '1000'], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, text=True)
    except OSError:
        return None
    gpu, vdd_in, temps = [], [], []
    end = time.time() + seconds
    try:
        while time.time() < end:
            line = p.stdout.readline()
            if not line:
                break
            m = re.search(r'GR3D_FREQ (\d+)%', line)
            if m:
                gpu.append(float(m.group(1)))
            m = re.search(r'VDD_IN (\d+)mW', line)
            if m:
                vdd_in.append(float(m.group(1)))
            t = [float(x) for x in re.findall(r'(?:cpu|soc\d|tj)@([\d.]+)C', line)]
            if t:
                temps.append(max(t))
    finally:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
    if not gpu and not vdd_in:
        return None
    return dict(
        gpu_pct_mean=(sum(gpu) / len(gpu)) if gpu else None,
        gpu_pct_max=max(gpu) if gpu else None,
        power_mw_mean=(sum(vdd_in) / len(vdd_in)) if vdd_in else None,
        power_mw_max=max(vdd_in) if vdd_in else None,
        temp_c_max=max(temps) if temps else None,
        samples=max(len(gpu), len(vdd_in)),
    )


class RateCounter:
    """Counts messages on the rate topics for as long as it is open.

    *** THE SPIN MUST RUN ON ITS OWN EXECUTOR THREAD, AND THAT IS THE WHOLE POINT. ***
    The first version of this counted inside `while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)`. spin_once services AT MOST ONE ready callback
    per call and blocks up to its timeout when nothing is ready, so that loop has a hard
    ceiling near 1/timeout = 20 callbacks per second. Against ~30 msg/s of real traffic it
    reported /lidar_slam/odom at 7.6 Hz while `ros2 topic hz` measured 10.010 Hz in the same
    minute -- i.e. it would have declared a healthy SLAM stack BELOW ITS RATE FLOOR. That is
    the fourth instrument on this project to under-report under load, and the reason every
    number in this report names the tool that produced it.

    A dedicated SingleThreadedExecutor in a background thread services every callback without
    busy-waiting, so it neither drops messages nor inflates the CPU figure it sits beside.

    *** AND THE RATE IS COMPUTED FROM ARRIVAL TIMES, NOT FROM THE WINDOW. *** Fixing the spin
    loop alone still read 6.8 Hz against `ros2 topic hz`'s 10.003 Hz -- a uniform 0.68 across
    all three topics, which is the signature of DDS DISCOVERY LATENCY: the subscriptions take
    seconds to match their publishers while the window clock is already running, so ~15 s of a
    45 s window counted nothing. Dividing by the window therefore measures discovery, not the
    topic. Each topic's rate is now (n-1) / (last_arrival - first_arrival), which is immune to
    both discovery and the window edges. A topic that never publishes still reports 0.0.
    """

    def __init__(self):
        self.ok = True
        self.note = None
        self.counts = {t: 0 for t, _ in RATES}
        self.first = {t: None for t, _ in RATES}
        self.last = {t: None for t, _ in RATES}
        self._t0 = None
        self._thread = None
        self._executor = None
        self._node = None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CameraInfo, Imu
            from nav_msgs.msg import Odometry
            from dtp_msgs.msg import TargetArray
        except ImportError as exc:
            self.ok = False
            self.note = 'rclpy/msgs not importable (%s)' % exc
            return

        types = {
            '/livox/imu': Imu,
            '/lidar_slam/odom': Odometry,
            '/camera/color/camera_info': CameraInfo,
            '/camera/depth/camera_info': CameraInfo,
            '/dtp/targets': TargetArray,
        }

        import rclpy as _rclpy
        if not _rclpy.ok():
            _rclpy.init()
        self._rclpy = _rclpy
        self._node = Node('coresidency_probe')
        self._subs = []
        for topic, _nom in RATES:
            mt = types.get(topic)
            if mt is None:
                continue
            self._subs.append(self._node.create_subscription(
                mt, topic, self._make(topic), qos_profile_sensor_data))
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

    def _make(self, topic):
        def cb(_msg):
            now = time.time()
            self.counts[topic] += 1
            if self.first[topic] is None:
                self.first[topic] = now
            self.last[topic] = now
        return cb

    def start(self):
        if not self.ok:
            return
        import threading
        for k in self.counts:
            self.counts[k] = 0
            self.first[k] = None
            self.last[k] = None
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._t0 = time.time()

    def stop(self):
        """{topic: hz} from observed inter-arrival spans, or None if unusable."""
        if not self.ok or self._t0 is None:
            return None
        self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        out = {}
        for topic, _nom in RATES:
            n = self.counts[topic]
            a, b = self.first[topic], self.last[topic]
            if n >= 2 and a is not None and b is not None and b > a:
                # (n-1) intervals span (b-a) seconds. Excludes the dead time before the
                # subscription matched, which is what made the first two versions under-report.
                out[topic] = (n - 1) / (b - a)
            else:
                out[topic] = 0.0
        self._node.destroy_node()
        self._rclpy.shutdown()
        return out


def sample(seconds, phase):
    rec = {'phase': phase, 'seconds': seconds}

    # clocks first: an unpinned measurement is not worth taking
    cur, _ = read_first([os.path.join(GPU_DEVFREQ, 'cur_freq')])
    mx, _ = read_first([os.path.join(GPU_DEVFREQ, 'max_freq')])
    rec['gpu_cur_hz'] = int(cur) if cur and cur.isdigit() else None
    rec['gpu_max_hz'] = int(mx) if mx and mx.isdigit() else None
    rec['clocks_pinned'] = (rec['gpu_cur_hz'] is not None
                            and rec['gpu_cur_hz'] == rec['gpu_max_hz'])

    # Rates are counted across the SAME window as CPU and power. Measuring them afterwards
    # would report the rates of a differently-loaded machine than the one whose CPU we quote.
    rates = RateCounter()
    rates.start()

    procs = proc_table()
    t0_ticks = {pid: proc_cpu_ticks(pid) for pid in procs}
    busy0, tot0 = sys_cpu_ticks()

    teg = tegrastats_sample(seconds)           # this is the blocking wall-clock window
    if teg is None:
        time.sleep(max(0.0, seconds - 1.0))
    rec['tegrastats'] = teg

    busy1, tot1 = sys_cpu_ticks()
    clk = os.sysconf('SC_CLK_TCK') or CLK
    per_proc = {}
    for pid, (group, cmd) in procs.items():
        a, b = t0_ticks.get(pid), proc_cpu_ticks(pid)
        if a is None or b is None:
            continue
        per_proc[pid] = dict(group=group, cmd=cmd[:110],
                             cpu_pct=100.0 * (b - a) / clk / seconds,
                             rss_mb=(proc_rss_kb(pid) or 0) / 1024.0)
    rec['per_proc'] = per_proc
    rec['sys_cpu_pct'] = (100.0 * (busy1 - busy0) / (tot1 - tot0)) if tot1 > tot0 else None
    rec['ncpu'] = os.cpu_count()
    used, total = mem_used_gb()
    rec['mem_used_gb'] = used
    rec['mem_total_gb'] = total
    rec['thermals_c'] = thermals()
    # Fallback GPU utilisation for boxes where tegrastats is absent. Tegra reports this in
    # per-mille on some JetPacks and percent on others, so the raw value and its source are
    # both recorded and the interpretation is made at print time rather than guessed here.
    gpu_raw, gpu_src = read_first(GPU_LOAD_PATHS)
    rec['gpu_load_raw'] = gpu_raw
    rec['gpu_load_src'] = gpu_src
    rails, rail_src = power_rails_mw()
    rec['power_rails_mw'] = rails
    rec['power_rails_src'] = rail_src
    rec['rates_hz'] = rates.stop()
    rec['rates_note'] = rates.note
    return rec


def group_totals(rec):
    out = {}
    for info in rec['per_proc'].values():
        g = out.setdefault(info['group'], {'cpu_pct': 0.0, 'rss_mb': 0.0, 'n': 0})
        g['cpu_pct'] += info['cpu_pct']
        g['rss_mb'] += info['rss_mb']
        g['n'] += 1
    return out


def report(rec):
    print()
    print('=' * 78)
    print('CO-RESIDENCY MEASUREMENT   phase %s   %.0f s   %d CPUs'
          % (rec['phase'], rec['seconds'], rec['ncpu'] or 0))
    print('=' * 78)
    if rec['clocks_pinned']:
        print('clocks      : PINNED at %d MHz  [%s/cur_freq]'
              % ((rec['gpu_cur_hz'] or 0) // 1000000, GPU_DEVFREQ))
    else:
        print('clocks      : *** NOT PINNED (%s of %s MHz) -- run `sudo jetson_clocks` and REDO.'
              % ((rec['gpu_cur_hz'] or 0) // 1000000, (rec['gpu_max_hz'] or 0) // 1000000))
        print('              Every unpinned number on this project has been about half the truth.')

    print()
    print('CPU BY GROUP (%% of ONE core; %d cores available => %d %% is full)'
          % (rec['ncpu'] or 0, 100 * (rec['ncpu'] or 1)))
    tot = group_totals(rec)
    for g in ('follow', 'camera', 'nav', 'other_ros'):
        if g in tot:
            t = tot[g]
            print('  %-10s %7.1f %%   %7.0f MB   %d process(es)'
                  % (g, t['cpu_pct'], t['rss_mb'], t['n']))
    ours = sum(tot.get(g, {}).get('cpu_pct', 0.0) for g in ('follow', 'camera'))
    theirs = tot.get('nav', {}).get('cpu_pct', 0.0)
    print('  ' + '-' * 52)
    print('  OURS       %7.1f %%   (follow + camera)' % ours)
    print('  NAV        %7.1f %%   (FAST-LIO + planner + drivers + bridges)' % theirs)
    if rec['sys_cpu_pct'] is not None:
        print('  system-wide busy %.1f %% of all cores  [/proc/stat]' % rec['sys_cpu_pct'])

    print()
    print('TOP PROCESSES')
    for pid, info in sorted(rec['per_proc'].items(),
                            key=lambda kv: -kv[1]['cpu_pct'])[:8]:
        print('  %-9s %6.1f %%  %6.0f MB  %s'
              % (info['group'], info['cpu_pct'], info['rss_mb'], info['cmd'][:70]))

    print()
    if rec['mem_used_gb'] is not None:
        print('memory      : %.2f of %.2f GB used  [/proc/meminfo MemTotal-MemAvailable]'
              % (rec['mem_used_gb'], rec['mem_total_gb']))
    teg = rec['tegrastats']
    if teg:
        def fmt(v, unit, scale=1.0):
            return 'unavailable' if v is None else '%.1f %s' % (v / scale, unit)
        print('GPU         : mean %s, peak %s  [tegrastats GR3D_FREQ, %d samples]'
              % (fmt(teg['gpu_pct_mean'], '%'), fmt(teg['gpu_pct_max'], '%'), teg['samples']))
        print('power       : mean %s, peak %s  [tegrastats VDD_IN -- TOTAL MODULE INPUT]'
              % (fmt(teg['power_mw_mean'], 'W', 1000.0), fmt(teg['power_mw_max'], 'W', 1000.0)))
        print('temperature : peak %s  [tegrastats]' % fmt(teg['temp_c_max'], 'C'))
    else:
        print('GPU/power   : tegrastats unavailable (not present, or no parsable lines).')
        raw, src = rec.get('gpu_load_raw'), rec.get('gpu_load_src')
        if raw is not None and str(raw).strip().isdigit():
            v = int(str(raw).strip())
            # >100 cannot be a percentage, so it is per-mille.
            pct = v / 10.0 if v > 100 else float(v)
            print('              GPU %.1f %% (raw %s)  [%s]' % (pct, raw, src))
        else:
            print('              GPU load unavailable (tried: %s)'
                  % ', '.join(GPU_LOAD_PATHS))
        if rec['power_rails_mw']:
            tot_mw = sum(rec['power_rails_mw'].values())
            print('              rails from %s: %.1f W total (%s)'
                  % (rec['power_rails_src'], tot_mw / 1000.0,
                     ', '.join('%s %.1fW' % (k, v / 1000.0)
                               for k, v in sorted(rec['power_rails_mw'].items()))))
        else:
            print('              no INA3221 hwmon rails either -- power is NOT measured here.')
    th = rec['thermals_c']
    if th:
        hot = sorted(th.items(), key=lambda kv: -kv[1])[:3]
        print('thermal zones: %s  [/sys/class/thermal]'
              % ', '.join('%s %.1f C' % (k, v) for k, v in hot))

    print()
    print('RATE COMPLIANCE -- the real "does it fit" test (floor = %.0f %% of nominal)'
          % (100 * RATE_FLOOR))
    if rec['rates_hz'] is None:
        print('  not measured: %s' % rec['rates_note'])
    else:
        for topic, nom in RATES:
            hz = rec['rates_hz'].get(topic)
            if hz is None:
                continue
            if hz <= 0.01:
                verdict = 'SILENT'
            elif hz >= RATE_FLOOR * nom:
                verdict = 'ok'
            else:
                verdict = '*** BELOW FLOOR ***'
            print('  %-32s %6.1f Hz / %4.0f nominal   %s' % (topic, hz, nom, verdict))
        if rec['rates_note']:
            print('  note: %s' % rec['rates_note'])
        print('  NOT subscribed here (doing so corrupts the measurement -- see RATES): %s'
              % ', '.join(HEAVY_TOPICS_NOT_SUBSCRIBED))
    print('=' * 78)


def compare(a, b):
    print()
    print('=' * 78)
    print('MARGINAL COST OF THE FOLLOW STACK   (phase %s -> phase %s)'
          % (a['phase'], b['phase']))
    print('=' * 78)
    for rec, name in ((a, 'A'), (b, 'B')):
        if not rec['clocks_pinned']:
            print('*** phase %s was measured with clocks UNPINNED -- the delta is not valid. ***'
                  % name)
    ta, tb = group_totals(a), group_totals(b)
    print()
    print('%-14s %12s %12s %12s' % ('', 'phase A', 'phase B', 'delta'))
    for g in ('follow', 'camera', 'nav', 'other_ros'):
        va = ta.get(g, {}).get('cpu_pct', 0.0)
        vb = tb.get(g, {}).get('cpu_pct', 0.0)
        print('%-14s %11.1f%% %11.1f%% %11.1f%%' % ('CPU ' + g, va, vb, vb - va))
    print('%-14s %11.1f%% %11.1f%% %11.1f%%'
          % ('CPU total', a['sys_cpu_pct'] or 0.0, b['sys_cpu_pct'] or 0.0,
             (b['sys_cpu_pct'] or 0.0) - (a['sys_cpu_pct'] or 0.0)))
    if a['mem_used_gb'] and b['mem_used_gb']:
        print('%-14s %10.2fGB %10.2fGB %10.2fGB'
              % ('memory', a['mem_used_gb'], b['mem_used_gb'],
                 b['mem_used_gb'] - a['mem_used_gb']))
    for key, label, scale, unit in (('gpu_pct_mean', 'GPU mean', 1.0, '%'),
                                    ('power_mw_mean', 'power mean', 1000.0, 'W'),
                                    ('temp_c_max', 'temp peak', 1.0, 'C')):
        va = (a['tegrastats'] or {}).get(key)
        vb = (b['tegrastats'] or {}).get(key)
        if va is None or vb is None:
            print('%-14s %12s %12s %12s' % (label, 'n/a', 'n/a', 'not measured'))
        else:
            print('%-14s %11.1f%s %11.1f%s %11.1f%s'
                  % (label, va / scale, unit, vb / scale, unit, (vb - va) / scale, unit))

    print()
    print('RATE COMPLIANCE -- did adding our stack cost the NAV stack its rates?')
    for topic, nom in RATES:
        ha = (a['rates_hz'] or {}).get(topic)
        hb = (b['rates_hz'] or {}).get(topic)
        if ha is None or hb is None:
            continue
        # ALWAYS show the change for a topic that was already publishing in phase A. An
        # earlier version only flagged drops over 5 %, which silently hid /lidar_slam/odom
        # falling 10.0 -> 9.6 Hz -- and that topic is the autopilot's own position source, so
        # a 4 % drop caused by our stack is exactly the thing this comparison exists to show.
        note = ''
        if hb > 0.01 and hb < RATE_FLOOR * nom:
            note = '  *** BELOW FLOOR IN B ***'
        elif ha > 0.01 and hb > 0.01 and abs(hb - ha) / ha > 0.01:
            note = '  %+.1f %% vs A' % (100.0 * (hb / ha - 1.0))
        elif ha <= 0.01 and hb > 0.01:
            note = '  (ours -- silent in A, as expected)'
        print('  %-32s A %6.1f   B %6.1f   nominal %4.0f%s' % (topic, ha, hb, nom, note))
    print()
    print('Read the CPU delta as "what our stack adds to a platform already doing SLAM and')
    print('planning". The NAV row should be ~unchanged; if it ROSE, our stack is making their')
    print('stack work harder, which is the 2026-09-11 starvation finding and belongs in the paper.')
    print('=' * 78)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--phase', default='B', help='label for this sample (A = nav only, B = both)')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--out', default=None, help='write the raw sample as JSON here')
    ap.add_argument('--compare', nargs=2, metavar=('A.json', 'B.json'),
                    help='print the marginal cost between two saved samples and exit')
    args = ap.parse_args(argv)

    if args.compare:
        with open(args.compare[0]) as f:
            a = json.load(f)
        with open(args.compare[1]) as f:
            b = json.load(f)
        compare(a, b)
        return 0

    rec = sample(args.seconds, args.phase)
    report(rec)
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(rec, f, indent=1)
        print('raw sample -> %s' % args.out)
    return 0 if rec['clocks_pinned'] else 2


if __name__ == '__main__':
    sys.exit(main())
