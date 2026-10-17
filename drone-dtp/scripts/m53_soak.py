#!/usr/bin/env python3
"""M5.3 compute soak: does the whole graph fit, for 20 minutes, without starving FAST-LIO?

THE EXIT IT SCORES (docs/m5-m6-plan.md SS2.2, M5.3 -- the compute half)
-------------------------------------------------------------------------
  O1  /lidar_slam/odom >= 9.9 Hz over the whole soak
  O2  no /lidar_slam/odom gap > 300 ms (arrival time -- what EKF2 and ROG-Map actually see)
  C1  total CPU <= 600 % of 800 % (mean over the soak; the p95 window is printed too)
  C2  nothing swaps: no pages swapped OUT during the soak and swap use does not grow. (The plan says
      "swap 0 B"; that is its intent -- the M1 regression thrashed swap -- but a box with a few MB
      parked in swap from an earlier job would fail "0 B" while swapping nothing, so the absolute
      value is printed and the ACTIVITY is scored.)
  P1  perception_node <= 80 % of a core
  P2  perception_node logged "GPU cloud verified against numpy" and never a cloud-backend DISABLED
      line (checked in --perception-log, when given)

The map half of M5.3 -- trail crossings and tube erosion -- needs a person walking and is scored by
target_filter_check.py (synthetic) and target_filter_replay.py (bags), not here.

It commands nothing. It reads /proc and subscribes to one topic, BEST_EFFORT (a RELIABLE
subscriber to FAST-LIO's odometry never matches and never errors -- flight-test-plan SS1a).

WHY PER-PROCESS FROM /proc AND NOT top
--------------------------------------
Two readings of the same stack disagreed by 40 points on 2026-09-11 because `top` averages over
its own refresh and attributes a python node's threads to whichever sample caught them. Here each
process's utime+stime is differenced over the same wall interval as /proc/stat's total, so the
per-process numbers add up to the total by construction.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/m53_soak.py --minutes 20 --out ~/dtp_datasets/<date>_m53_soak \\
        --perception-log <the launch log that contains perception_node's output>

Exit 0 = every scored check passes; 1 = one failed; 3 = too short to score (see --minutes).
"""
import argparse
import csv
import math
import os
import re
import sys
import time

# The plan's thresholds (SS2.2 M5.3)
ODOM_MIN_HZ = 9.9
ODOM_MAX_GAP_S = 0.300
TOTAL_CPU_MAX_PCT = 600.0
PERCEPTION_MAX_PCT = 80.0
MIN_SOAK_S = 20 * 60

# Processes worth a row, by argv basename (never a flattened cmdline -- CLAUDE.md, pkill -f trap).
WATCH = {
    'fastlio_mapping': 'FAST-LIO',
    'fsm_node': 'SUPER fsm_node',
    'livox_ros_driver2_node': 'Livox driver',
    'perception_node': 'perception_node',
    'target_manager': 'target_manager',
    'follow_planner_node': 'follow_planner',
    'command_mux_node': 'command_mux',
    'sensor_watchdog': 'sensor_watchdog',
    'static_calib_tf': 'static_calib_tf',
    'dtp_viz': 'dtp_viz',
    'component_container': 'Orbbec driver (container)',
    'orbbec_camera_node': 'Orbbec driver',
    'px4_odom_bridge.py': 'px4_odom_bridge',
    'px4_offboard_bridge.py': 'px4_offboard_bridge',
    'server.py': 'web GCS',
    'MicroXRCEAgent': 'XRCE agent',
    'mavlink-routerd': 'mavlink-router',
    'ros2': 'ros2 bag / cli',
}
CLK_TCK = os.sysconf('SC_CLK_TCK')
NCPU = os.cpu_count() or 8


def proc_rows():
    """{pid: (label, utime+stime ticks)} for every watched process."""
    out = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            argv = open('/proc/%s/cmdline' % d, 'rb').read().split(b'\0')
            stat = open('/proc/%s/stat' % d).read()
        except OSError:
            continue
        label = None
        for tok in argv[:3]:
            base = os.path.basename(tok.decode(errors='replace'))
            if base in WATCH:
                label = WATCH[base]
                break
        if label is None:
            continue
        # comm may contain spaces/parens: fields after the LAST ')' are fixed
        rest = stat[stat.rfind(')') + 2:].split()
        out[int(d)] = (label, int(rest[11]) + int(rest[12]))
    return out


def total_ticks():
    f = open('/proc/stat').readline().split()[1:]
    vals = [int(x) for x in f]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals), idle


def swap_used_kb():
    info = dict(line.split(':', 1) for line in open('/proc/meminfo'))
    return int(info['SwapTotal'].split()[0]) - int(info['SwapFree'].split()[0])


def pswpout():
    for line in open('/proc/vmstat'):
        if line.startswith('pswpout '):
            return int(line.split()[1])
    return 0


def gpu_freq_mhz():
    try:
        return int(open('/sys/class/devfreq/17000000.gpu/cur_freq').read()) // 1000000
    except OSError:
        return -1


def tj_c():
    best = None
    for z in os.listdir('/sys/devices/virtual/thermal'):
        if not z.startswith('thermal_zone'):
            continue
        try:
            t = int(open('/sys/devices/virtual/thermal/%s/temp' % z).read()) / 1000.0
        except (OSError, ValueError):
            continue
        best = t if best is None else max(best, t)
    return best


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--minutes', type=float, default=20.0)
    ap.add_argument('--window', type=float, default=10.0, help='sample period, s')
    ap.add_argument('--out', required=True, help='directory for soak.csv and summary.txt')
    ap.add_argument('--perception-log', default=None)
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('m53_soak')
    arrivals = []
    node.create_subscription(Odometry, '/lidar_slam/odom', lambda _m: arrivals.append(time.monotonic()),
                             QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                        history=HistoryPolicy.KEEP_LAST, depth=10))

    duration = args.minutes * 60.0
    csv_path = os.path.join(args.out, 'soak.csv')
    fh = open(csv_path, 'w', newline='')
    w = csv.writer(fh)
    w.writerow(['t_s', 'total_cpu_pct', 'swap_kb', 'gpu_mhz', 'tj_c', 'odom_hz', 'odom_max_gap_ms']
               + ['cpu_pct:%s' % lbl for lbl in sorted(set(WATCH.values()))])

    t_start = time.monotonic()
    swap0, pswpout0 = swap_used_kb(), pswpout()
    prev_tot, prev_idle = total_ticks()
    prev_proc = proc_rows()
    next_sample = t_start + args.window
    samples = []
    per_label = {}
    max_swap = 0
    print('m53_soak: %.0f min, %.0f s windows -> %s' % (args.minutes, args.window, csv_path))
    try:
        while time.monotonic() - t_start < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now < next_sample:
                continue
            next_sample += args.window
            tot, idle = total_ticks()
            cur_proc = proc_rows()
            dt_ticks = max(1, tot - prev_tot)
            total_pct = 100.0 * NCPU * (1.0 - (idle - prev_idle) / dt_ticks)
            wall = dt_ticks / CLK_TCK / NCPU
            by_label = {}
            for pid, (lbl, ticks) in cur_proc.items():
                if pid in prev_proc:
                    by_label[lbl] = by_label.get(lbl, 0.0) + 100.0 * (ticks - prev_proc[pid][1]) / CLK_TCK / wall
            win = [a for a in arrivals if a >= now - args.window]
            gaps = [b - a for a, b in zip(win, win[1:])]
            if win and arrivals.index(win[0]) > 0:
                gaps.append(win[0] - arrivals[arrivals.index(win[0]) - 1])
            odom_hz = len(win) / args.window
            max_gap = max(gaps) if gaps else float('inf')
            swap = swap_used_kb()
            max_swap = max(max_swap, swap)
            row = [round(now - t_start, 1), round(total_pct, 1), swap, gpu_freq_mhz(), tj_c(),
                   round(odom_hz, 2), round(1e3 * max_gap, 1) if math.isfinite(max_gap) else '']
            row += [round(by_label.get(lbl, 0.0), 1) for lbl in sorted(set(WATCH.values()))]
            w.writerow(row)
            fh.flush()
            samples.append((now - t_start, total_pct, odom_hz, max_gap, sum(by_label.values())))
            for lbl, pct in by_label.items():
                per_label.setdefault(lbl, []).append(pct)
            print('  t=%5.0f s  CPU %5.0f %%  odom %5.2f Hz (max gap %4.0f ms)  swap %d kB  GPU %d MHz  '
                  'perception %4.0f %%  FAST-LIO %4.0f %%  fsm %4.0f %%'
                  % (now - t_start, total_pct, odom_hz, 1e3 * max_gap if math.isfinite(max_gap) else -1,
                     swap, gpu_freq_mhz(), by_label.get('perception_node', 0.0),
                     by_label.get('FAST-LIO', 0.0), by_label.get('SUPER fsm_node', 0.0)))
            prev_tot, prev_idle, prev_proc = tot, idle, cur_proc
    except KeyboardInterrupt:
        print('interrupted -- scoring what was collected')
    finally:
        fh.close()
        node.destroy_node()
        if rclpy.ok():          # a SIGINT has already shut the context down
            rclpy.shutdown()

    elapsed = time.monotonic() - t_start
    lines = []

    def say(s=''):
        print(s)
        lines.append(s)

    fails = 0

    def verdict(code, ok, what, detail):
        nonlocal fails
        fails += (ok is False)
        say('  %s  %-3s %-44s %s' % ({True: 'PASS', False: 'FAIL', None: 'n/a '}[ok], code, what, detail))

    say()
    say('M5.3 COMPUTE SOAK  %.1f min, %d windows' % (elapsed / 60.0, len(samples)))
    if not samples:
        say('no samples')
        return 3
    arr = [a for a in arrivals if a >= t_start + args.window]   # skip the first window's discovery
    span = (arr[-1] - arr[0]) if len(arr) > 1 else 0.0
    hz = (len(arr) - 1) / span if span > 0 else 0.0
    gaps = [b - a for a, b in zip(arr, arr[1:])]
    big = [g for g in gaps if g > ODOM_MAX_GAP_S]
    verdict('O1', hz >= ODOM_MIN_HZ, '/lidar_slam/odom rate', '%.2f Hz over %.0f s (need >= %.1f)' % (hz, span, ODOM_MIN_HZ))
    verdict('O2', not big and bool(gaps), 'no odom gap > %.0f ms' % (1e3 * ODOM_MAX_GAP_S),
            'max %.0f ms; %d gaps over' % (1e3 * max(gaps) if gaps else float('nan'), len(big)))
    tot = [s[1] for s in samples]
    mean_tot = sum(tot) / len(tot)
    p95_tot = sorted(tot)[int(0.95 * (len(tot) - 1))]
    verdict('C1', mean_tot <= TOTAL_CPU_MAX_PCT, 'total CPU (of %d00 %%)' % NCPU,
            'mean %.0f %%, p95 window %.0f %%, max %.0f %% (need mean <= %.0f %%)'
            % (mean_tot, p95_tot, max(tot), TOTAL_CPU_MAX_PCT))
    out_pages = pswpout() - pswpout0
    verdict('C2', out_pages == 0 and max_swap <= swap0, 'nothing swapped during the soak',
            '%d page(s) swapped out; swap used %d kB at start, max %d kB' % (out_pages, swap0, max_swap))
    perc = per_label.get('perception_node')
    if perc:
        pm = sum(perc) / len(perc)
        verdict('P1', pm <= PERCEPTION_MAX_PCT, 'perception_node CPU', 'mean %.0f %% of a core, max window %.0f %% (need <= %.0f %%)'
                % (pm, max(perc), PERCEPTION_MAX_PCT))
    else:
        verdict('P1', None, 'perception_node CPU', 'perception_node not running')
    if args.perception_log:
        try:
            text = open(args.perception_log, errors='replace').read()
            verified = 'GPU cloud verified against numpy' in text
            # perception_node's own wording (perception_node.py): "cloud_backend=cuda is DISABLED".
            # Not any line with DISABLED and "cloud" in it -- fusion_node's normal startup notice
            # ("fusion_node is DISABLED ... no cloud") matched that and failed the first real soak.
            disabled = [l for l in text.splitlines()
                        if 'cloud_backend=cuda is DISABLED' in l or 'GPU cloud cannot start' in l]
            verdict('P2', verified and not disabled, 'GPU organized cloud verified, never disabled',
                    '%s; %d DISABLED line(s)' % ('verified' if verified else 'NO "verified" line', len(disabled)))
        except OSError as exc:
            verdict('P2', False, 'GPU organized cloud verified, never disabled', 'cannot read log: %s' % exc)
    else:
        verdict('P2', None, 'GPU organized cloud verified, never disabled', 'pass --perception-log to score')
    unwatched = sum(s_[1] - s_[4] for s_ in samples) / len(samples)
    say()
    say('mean CPU NOT attributed to a watched process: %.0f %% (on the desk this is mostly editors/remote '
        'sessions; in the field it should be near 0 -- subtract it before comparing to the budget)' % unwatched)
    say('mean CPU per process (% of one core):')
    for lbl, vals in sorted(per_label.items(), key=lambda kv: -sum(kv[1]) / len(kv[1])):
        say('  %-28s %6.1f %%   (max window %.0f %%)' % (lbl, sum(vals) / len(vals), max(vals)))
    gpu = sorted({r for r in [gpu_freq_mhz()]})
    say('GPU now %d MHz (jetson_clocks pins it at the max; the soak should run pinned, as in the field)' % gpu[0])
    if elapsed < MIN_SOAK_S - 5:
        say('\nSHORT RUN: %.1f min < the plan\'s 20 min -- a rehearsal, NOT the M5.3 exit' % (elapsed / 60.0))
    with open(os.path.join(args.out, 'summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    if fails:
        return 1
    return 3 if elapsed < MIN_SOAK_S - 5 else 0


if __name__ == '__main__':
    sys.exit(main())
