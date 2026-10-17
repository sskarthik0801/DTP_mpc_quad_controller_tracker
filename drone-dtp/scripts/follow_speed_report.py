#!/usr/bin/env python3
"""follow_speed_report.py -- how fast did the follower fly, and what held it back. Read-only.

Written for the 2026-09-15 20:01 deploy_follow flight ("why did it fly slower than 0.5 m/s?",
docs/fact-follow-speed-and-descent-guard.md) and kept so every number in that note can be
re-derived from the bag. For each armed+Offboard segment and each FOLLOW window inside it:

  * commanded speed (/planning/pos_cmd: horizontal, vertical, 3-D) against SUPER's max_vel,
  * achieved speed (EKF2 /fmu/out/vehicle_local_position_v1, and FAST-LIO position deltas),
  * the walker's speed and range (/dtp/follow_target), i.e. whether the aircraft fell behind,
  * how far ahead the goal was (/planning/follow_goal) -- every SUPER plan ends at rest there,
  * the goal ALTITUDE: its levels, how often it jumped >= 0.1 m, how many goals sat at the
    aircraft's own altitude (the pre-2026-09-17 descent guard's signature) and how many ticks
    /mux/state reported a latched floor (the post-change guard),
  * altitude tracking error (FAST-LIO z minus commanded z),
  * every lock taken while in Offboard (a different track id is a different person).

With --stack-log it also counts SUPER's FAILED replans per second (the fsm_node lines in a
deploy_follow stack.log; successes are not timestamped, see stack_log_bins).

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/follow_speed_report.py ~/dtp_datasets/20260915_2001_stageB_salvaged \
        --stack-log ~/dtp_datasets/20260915_1955_deploy/stack.log

Refuses a bag without metadata.yaml: reading a .db3 that `ros2 bag record` is still writing kills
the recorder (CLAUDE.md). Recover a dead recorder's bag with `ros2 bag reindex` first.
"""
import argparse
import bisect
import collections
import math
import os
import re
import sys

import numpy as np

TOPICS = ['/planning/pos_cmd', '/fmu/out/vehicle_local_position_v1',
          '/fmu/out/vehicle_control_mode', '/dtp/follow_target', '/planning/follow_goal',
          '/mux/state', '/dtp/lock_state', '/lidar_slam/odom']
FLIP_M = 0.10          # a goal-altitude change at least this big counts as a jump
SAME_ALT_M = 0.02      # a goal within this of the aircraft's altitude "sat at the aircraft"
GOAL_TAIL_S = 0.05     # see walker_and_goals


def read_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in TOPICS if t in types]
    missing = sorted(set(TOPICS) - set(want))
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    d = collections.defaultdict(list)
    while r.has_next():
        topic, data, ts = r.read_next()
        m = deserialize_message(data, cls[topic])
        t = ts * 1e-9
        if topic == '/planning/pos_cmd':
            d[topic].append((t, m.position.x, m.position.y, m.position.z,
                             m.velocity.x, m.velocity.y, m.velocity.z))
        elif topic == '/fmu/out/vehicle_local_position_v1':
            d[topic].append((t, m.vx, m.vy, m.vz))
        elif topic == '/fmu/out/vehicle_control_mode':
            d[topic].append((t, bool(m.flag_armed), bool(m.flag_control_offboard_enabled)))
        elif topic == '/dtp/follow_target':
            d[topic].append((t, m.track_id, m.position.x, m.position.y, m.position.z,
                             m.velocity.x, m.velocity.y))
        elif topic == '/planning/follow_goal':
            d[topic].append((t, m.pose.position.x, m.pose.position.y, m.pose.position.z))
        elif topic == '/lidar_slam/odom':
            p = m.pose.pose.position
            d[topic].append((t, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                             p.x, p.y, p.z))
        else:
            d[topic].append((t, m.data))
    return d, missing


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def segments(mode):
    out, start = [], None
    for t, armed, offboard in mode:
        on = armed and offboard
        if on and start is None:
            start = t
        elif not on and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, mode[-1][0]))
    return out


def follow_windows(mux, lo, hi):
    out, start = [], None
    for t, s in mux:
        if t < lo:
            if s.startswith('FOLLOW'):
                start = lo
            else:
                start = None
            continue
        if t >= hi:
            break
        if s.startswith('FOLLOW') and start is None:
            start = t
        elif not s.startswith('FOLLOW') and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, hi))
    return [w for w in out if w[1] - w[0] > 0.5]


def speed_block(d, lo, hi, t0, max_vel):
    pc = np.array([r for r in d['/planning/pos_cmd'] if lo <= r[0] < hi])
    lp = np.array([r for r in d['/fmu/out/vehicle_local_position_v1'] if lo <= r[0] < hi])
    od = np.array([r for r in d['/lidar_slam/odom'] if lo <= r[0] < hi])
    print('    [%.1f, %.1f] s  (%.1f s)' % (lo - t0, hi - t0, hi - lo))
    if len(pc):
        vh = np.hypot(pc[:, 4], pc[:, 5])
        v3 = np.linalg.norm(pc[:, 4:7], axis=1)
        vz = np.abs(pc[:, 6])
        print('      commanded |v_xy| med %.2f p95 %.2f max %.3f | |v_z| med %.2f p95 %.2f | '
              '|v_3d| med %.2f, at >= 90%% of max_vel %.0f%% of the time'
              % (np.median(vh), pct(vh, 95), vh.max(), np.median(vz), pct(vz, 95),
                 np.median(v3), 100.0 * np.mean(v3 >= 0.9 * max_vel)))
        print('      commanded path: %.2f m horizontal, %.2f m vertical, z %.2f..%.2f'
              % (np.sum(np.hypot(np.diff(pc[:, 1]), np.diff(pc[:, 2]))),
                 np.sum(np.abs(np.diff(pc[:, 3]))), pc[:, 3].min(), pc[:, 3].max()))
    if len(lp):
        ah = np.hypot(lp[:, 1], lp[:, 2])
        print('      EKF2 |v_xy| med %.2f p95 %.2f max %.2f'
              % (np.median(ah), pct(ah, 95), ah.max()))
    if len(od) > 1:
        net = math.hypot(od[-1, 2] - od[0, 2], od[-1, 3] - od[0, 3])
        path = np.sum(np.hypot(np.diff(od[:, 2]), np.diff(od[:, 3])))
        print('      FAST-LIO: %.2f m net (%.2f m/s), %.2f m path (%.2f m/s); z %.2f -> %.2f'
              % (net, net / (hi - lo), path, path / (hi - lo), od[0, 4], od[-1, 4]))
        if len(pc):
            cz = np.interp(od[:, 0], pc[:, 0], pc[:, 3])
            e = od[:, 4] - cz
            print('      altitude tracking (FAST-LIO z - commanded z): med %+.3f p95 %+.3f '
                  'max %+.3f min %+.3f' % (np.median(e), pct(e, 95), e.max(), e.min()))


def walker_and_goals(d, mux, lo, hi):
    od = d['/lidar_slam/odom']
    ot = [r[0] for r in od]

    def odom_at(t):
        return od[max(0, bisect.bisect_right(ot, t) - 1)]

    tg = [r for r in d['/dtp/follow_target'] if lo <= r[0] < hi]
    tt = [r[0] for r in tg]
    if tg:
        v = np.array([math.hypot(r[5], r[6]) for r in tg])
        rng = [math.hypot(r[2] - odom_at(r[0])[2], r[3] - odom_at(r[0])[3]) for r in tg]
        net = math.hypot(tg[-1][2] - tg[0][2], tg[-1][3] - tg[0][3])
        print('      walker: |v| med %.2f p90 %.2f, net %.2f m (%.2f m/s); range to aircraft '
              '%.2f -> %.2f m (min %.2f max %.2f); target z med %.2f'
              % (np.median(v), pct(v, 90), net, net / max(hi - lo, 1e-6), rng[0], rng[-1],
                 min(rng), max(rng), np.median([r[4] for r in tg])))
    # The mux publishes the goal before the state on the tick it leaves FOLLOW, so the first HOLD
    # goal lands just inside the window; GOAL_TAIL_S drops it.
    fg = [r for r in d['/planning/follow_goal'] if lo <= r[0] < hi - GOAL_TAIL_S]
    if not fg:
        return
    ahead = [math.hypot(r[1] - odom_at(r[0])[2], r[2] - odom_at(r[0])[3]) for r in fg]
    print('      goals: %d; goal ahead of aircraft med %.2f m (p10 %.2f, p90 %.2f)'
          % (len(fg), np.median(ahead), pct(ahead, 10), pct(ahead, 90)))
    if tg:
        to_walker = []
        for r in fg:
            k = max(0, bisect.bisect_right(tt, r[0]) - 1)
            to_walker.append(math.hypot(r[1] - tg[k][2], r[2] - tg[k][3]))
        print('      goal to walker: %.2f..%.2f m (the descent guard radius is 3.00 m)'
              % (min(to_walker), max(to_walker)))
    zs = [r[3] for r in fg]
    at_aircraft = sum(1 for r in fg if abs(r[3] - odom_at(r[0])[4]) <= SAME_ALT_M)
    flips = sum(1 for a, b in zip(zs, zs[1:]) if abs(b - a) >= FLIP_M)
    floor_ticks = sum(1 for t, s in mux if lo <= t < hi and 'alt floor' in s)
    levels = collections.Counter(round(z, 1) for z in zs)
    print('      goal altitude: %s; %d jumps >= %.2f m; %d goals at the aircraft altitude '
          '(+/-%.2f); %d /mux/state ticks with a latched floor'
          % (', '.join('%.1f x%d' % kv for kv in sorted(levels.items())), flips, FLIP_M,
             at_aircraft, SAME_ALT_M, floor_ticks))


def stack_log_bins(path, lo, hi, t0):
    """Failed replans per second, from fsm_node's timestamped WARN lines.

    Only failures are counted. SUPER prints a SUCCESSFUL replan with a bare cout
    (fsm.cpp:71): no timestamp, and block-buffered, so those lines reach the log in ~90-line
    bursts unrelated to when they happened. fsm_node replans at up to replan_rate (15 Hz in
    follow_test_ros2.yaml) while in FOLLOW_TRAJ, so 15 failures in one second means every replan
    in that second failed -- and on FAILED SUPER keeps flying the trajectory it committed earlier
    (callReplanOnce, fsm.cpp:69), which ends at rest at an older goal.
    """
    keys = [('[ReplanOnce]: GenerateExpTrajectory failed', 'exp_failed'),
            ('[ReplanOnce]: generateBackupTrajectory return', 'backup_failed'),
            ('[ReplanOnce]: Replan overtime', 'overtime'),
            ('[BackOpt] Opt failed', 'backopt_iter'),
            ('[ExpOpt] Opt failed', 'expopt_iter')]
    bins = collections.defaultdict(collections.Counter)
    pat = re.compile(r'\[fsm_node-\d+\] \[\w+\] \[(\d+\.\d+)\] \[fsm_node\]: (.*)')
    with open(path, 'rb') as f:
        for raw in f:
            line = re.sub(r'\x1b\[[0-9;]*m', '',
                          raw.replace(b'\0', b'').decode('utf-8', 'replace'))
            m = pat.search(line)
            if not m:
                continue
            t = float(m.group(1))
            if not lo <= t < hi:
                continue
            for needle, name in keys:
                if needle in m.group(2):
                    bins[int(t - t0)][name] += 1
    names = [n for _, n in keys]
    failed = sum(bins[s][n] for s in bins for n in names[:3])
    print('      SUPER failed replans: %d in %.1f s (<= %.0f attempts at 15 Hz, >= %.0f%% '
          'failed); per bag second (the *_iter columns are optimiser retries, not replans):'
          % (failed, hi - lo, 15 * (hi - lo), 100.0 * failed / max(1.0, 15 * (hi - lo))))
    print('        sec  ' + '  '.join('%13s' % n for n in names))
    for s in sorted(bins):
        print('        %3d  %s' % (s, '  '.join('%13d' % bins[s][n] for n in names)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--max-vel', type=float, default=None,
                    help='max_vel the flight used (default: read follow_test_ros2.yaml -- '
                         'which is TODAY\'s value, not necessarily the flight\'s)')
    ap.add_argument('--stack-log', default=None, help='deploy_follow stack.log, optional')
    args = ap.parse_args()
    bag = args.bag.rstrip('/')
    if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        sys.exit('%s has no metadata.yaml: still recording, or the recorder died. Refusing to '
                 'read it (ros2 bag reindex %s -s sqlite3 recovers a dead one).' % (bag, bag))
    max_vel = args.max_vel
    if max_vel is None:
        import yaml
        cfg = os.path.expanduser(
            '~/super_ws/src/SUPER/super_planner/config/follow_test_ros2.yaml')
        max_vel = float(yaml.safe_load(open(cfg))['traj_opt']['boundary']['max_vel'])
        print('max_vel %.2f from %s (pass --max-vel with the flight\'s value if it has changed)'
              % (max_vel, cfg))

    d, missing = read_bag(bag)
    if missing:
        print('topics not in the bag: %s' % ', '.join(missing))
    t0 = min(v[0][0] for v in d.values() if v)
    mux = d['/mux/state']
    locks = d['/dtp/lock_state']
    segs = segments(d['/fmu/out/vehicle_control_mode'])
    print('\n%s: bag start %.3f; %d armed+Offboard segment(s)' % (bag, t0, len(segs)))
    for lo, hi in segs:
        print('\n  OFFBOARD [%.1f, %.1f] s  (%.1f s)' % (lo - t0, hi - t0, hi - lo))
        prev = None
        for t, s in mux:
            if lo <= t < hi:
                key = '|'.join(s.split('|')[:2])
                if key != prev:
                    print('    mux  %7.1f  %s' % (t - t0, s[:110]))
                    prev = key
        tracks = collections.OrderedDict()
        prev = None
        for t, s in locks:
            if lo <= t < hi:
                key = '|'.join(s.split('|')[:2])
                if key != prev:
                    print('    lock %7.1f  %s' % (t - t0, s[:110]))
                    prev = key
                if s.startswith('LOCKED'):
                    tracks.setdefault(s.split('|')[1], t)
        if len(tracks) > 1:
            print('    NOTE: %d different tracks locked in this segment: %s'
                  % (len(tracks), ', '.join(tracks)))
        print('   whole segment')
        speed_block(d, lo, hi, t0, max_vel)
        for flo, fhi in follow_windows(mux, lo, hi):
            print('   FOLLOW window')
            speed_block(d, flo, fhi, t0, max_vel)
            walker_and_goals(d, mux, flo, fhi)
            if args.stack_log:
                stack_log_bins(args.stack_log, flo, fhi, t0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
