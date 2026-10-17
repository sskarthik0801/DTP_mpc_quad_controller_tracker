#!/usr/bin/env python3
"""nmpc_replay.py -- a recorded follow through the REAL follow_planner_node, one goal_mode per run.

    python3 drone-dtp/scripts/nmpc_replay.py BAG --arm geometric --out DIR_G1
    python3 drone-dtp/scripts/nmpc_replay.py BAG --arm geometric --out DIR_G2
    python3 drone-dtp/scripts/nmpc_replay.py BAG --arm nmpc      --out DIR_N1
    python3 drone-dtp/scripts/nmpc_replay.py BAG --arm nmpc      --out DIR_N2
    python3 drone-dtp/scripts/nmpc_replay.py --compare DIR_G1 DIR_G2 DIR_N1 DIR_N2

Plays the bag's /dtp/follow_target and /lidar_slam/odom at bag timing into follow_planner_node on
a private localhost-only domain (90, shared with nmpc_check.py -- it refuses to start if a
follow_planner_node is already there; two replays on one domain collide SILENTLY, which
loss_condition_replay.py learned as a coast count of 554 instead of 61). Records every
/dtp/follow_goal against bag time.

WHAT THIS CAN AND CANNOT SAY
  * It is OPEN LOOP: the aircraft in the bag flew the goals of the day, not these. So it scores
    the goal SIGNAL each arm produces from the same inputs -- its steps, its height jumps, its
    reversals, where it sits relative to the person -- and nothing about how an aircraft would
    have flown it. follow_speed_report.py cannot score this output (no /planning/pos_cmd, no
    /fmu/out): it is the scorer for FLIGHT bags.
  * It is not deterministic: a 15 Hz wall-clock timer samples a replayed ~8 Hz stream, so which
    target each tick reads shifts from run to run. Hence two runs per arm, and --compare reports
    the WITHIN-arm scatter first. A cross-arm difference is a result only where it exceeds it.
    (dtp-follow-behaviour.md: compare an arm against a second run of itself, never against zero.)
  * A short bag carries no percentile. 20260915_2001_stageB_salvaged follows for 13 s; say
    "consistent with run5" or not, and quote run5.
"""
import argparse
import csv
import json
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nmpc_check as nc  # noqa: E402  (the node runner, the contract constants and the metrics)


def find_db3(bag):
    if os.path.isfile(bag):
        return bag
    for root, _, files in os.walk(bag):
        for f in sorted(files):
            if f.endswith('.db3'):
                return os.path.join(root, f)
    raise SystemExit('no .db3 under %s' % bag)


def replay(a):
    import sqlite3
    os.environ['ROS_DOMAIN_ID'] = nc.DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from dtp_msgs.msg import Target

    exe = a.node_exe or nc.default_exe()
    os.makedirs(a.out, exist_ok=True)
    # Read the bag BEFORE anything is running, straight from sqlite and read-only. (Never open a
    # bag that `ros2 bag record` is still writing: the read locks the database and kills it.)
    con = sqlite3.connect('file:%s?mode=ro' % find_db3(a.bag), uri=True)
    topics = {n: i for i, n in con.execute('select id, name from topics')}
    want = {topics[t]: t for t in ('/dtp/follow_target', '/lidar_slam/odom') if t in topics}
    if len(want) != 2:
        raise SystemExit('the bag needs /dtp/follow_target and /lidar_slam/odom')
    rows = con.execute('select topic_id, timestamp, data from messages where topic_id in (%s) '
                       'order by timestamp' % ','.join(str(i) for i in want)).fetchall()
    con.close()
    t_first = rows[0][1]
    msgs = [(want[i], (ts - t_first) * 1e-9, deserialize_message(d, Target if want[i].endswith('target') else Odometry))
            for i, ts, d in rows]
    if a.start or a.duration:
        end = a.start + a.duration if a.duration else float('inf')
        msgs = [m for m in msgs if a.start <= m[1] <= end]

    rclpy.init()
    node = rclpy.create_node('nmpc_replay')
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                    depth=1, durability=DurabilityPolicy.VOLATILE)
    t_end = time.time() + 3.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if 'follow_planner_node' in node.get_node_names():
        raise SystemExit('a follow_planner_node is already on domain %s -- another replay or '
                         'nmpc_check.py is running. Two on one domain collide silently.' % nc.DOMAIN)

    pubs = {'/dtp/follow_target': node.create_publisher(Target, '/dtp/follow_target', be),
            '/lidar_slam/odom': node.create_publisher(Odometry, '/lidar_slam/odom', be)}
    state = {'bag_t': 0.0, 'person': None, 'drone': None}
    goals = []

    def on_goal(m):
        p = m.pose.position
        goals.append((state['bag_t'], (p.x, p.y, p.z), state['person'], state['drone']))
    node.create_subscription(PoseStamped, '/dtp/follow_goal', on_goal, be)

    planner = nc.PlannerNode(exe, os.path.join(a.out, 'follow_planner.log'), goal_mode=a.arm)
    try:
        t_end = time.time() + 20.0
        while time.time() < t_end and node.count_subscribers('/dtp/follow_target') < 1:
            rclpy.spin_once(node, timeout_sec=0.05)
        wall0, bag0 = time.time(), msgs[0][1]
        for topic, bt, m in msgs:
            while time.time() - wall0 < bt - bag0:
                rclpy.spin_once(node, timeout_sec=0.002)
            state['bag_t'] = bt
            if topic == '/dtp/follow_target':
                if all(math.isfinite(c) for c in (m.position.x, m.position.y, m.position.z)):
                    state['person'] = (m.position.x, m.position.y, m.position.z)
            else:
                q = m.pose.pose.position
                state['drone'] = (q.x, q.y, q.z)
            pubs[topic].publish(m)
        t_end = time.time() + 0.5
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        log = planner.stop()
        node.destroy_node()
        rclpy.shutdown()

    with open(os.path.join(a.out, 'goals.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['bag_t', 'gx', 'gy', 'gz', 'px', 'py', 'pz', 'dx', 'dy', 'dz'])
        for t, g, p, d in goals:
            w.writerow([t, *g, *(p or ('', '', '')), *(d or ('', '', ''))])

    # The goal stream comes in bursts (one per lock). Score each burst of >= 2 s on its own, then
    # pool. A burst ends on a gap in the goals OR on a jump of the PERSON (a re-lock onto another
    # position with no gap): the goal then jumps in both arms, by design, and counting that as a
    # step measures the bag and not the arm -- the first version of this did, and reported a
    # 6.6 m "largest step" for both.
    bursts, cur = [], []
    for g in goals:
        if cur and (g[0] - cur[-1][0] > 0.5 or
                    (g[2] and cur[-1][2] and math.dist(g[2], cur[-1][2]) > 1.0)):
            bursts.append(cur)
            cur = []
        cur.append(g)
    if cur:
        bursts.append(cur)
    bursts = [b for b in bursts if b[-1][0] - b[0][0] >= 2.0]
    ms = [nc.metrics([(g[0], g[1], g[2]) for g in b], settle_s=1.0) for b in bursts]
    ms = [m for m in ms if m and m['n'] > 5]
    n_tot = sum(m['n'] for m in ms)
    steps = []   # pooled over every scored burst, moving ticks only (the deadband holds the rest)
    for b in bursts:
        b = [g for g in b if g[0] - b[0][0] >= 1.0]
        steps += [d for d in (math.dist(x[1], y[1]) for x, y in zip(b, b[1:])) if d > 1e-6]
    summary = {'bag': a.bag, 'arm': a.arm, 'exe': exe, 'goals': len(goals), 'bursts': len(ms),
               'scored_goals': n_tot, 'follow_s': sum(b[-1][0] - b[0][0] for b in bursts)}
    if ms:
        summary.update({
            'step_p50': nc.pct(steps, 0.5), 'step_p95': nc.pct(steps, 0.95),
            'step_max': max(m['step_max'] for m in ms),
            'z_jumps': sum(m['z_jumps'] for m in ms), 'reversals': sum(m['reversals'] for m in ms),
            'in_band': sum(m['in_band'] * m['n'] for m in ms) / n_tot,
            'range_min': min(m['range_min'] for m in ms), 'range_max': max(m['range_max'] for m in ms)})
    solve = re.findall(r'NMPC solve p50 ([\d.]+) ms p99 ([\d.]+) ms max ([\d.]+) ms over (\d+) \| it \d+ \| '
                       r'ok (\d+) fallback (\d+) cancel (\d+) reset (\d+) '
                       r'\(new (\d+) stale (\d+) reproject (\d+) jump (\d+)\)', log)
    if solve:
        summary['solve'] = dict(zip(('p50_ms', 'p99_ms', 'max_ms', 'window', 'ok', 'fallback', 'cancel', 'reset',
                                     'reset_new', 'reset_stale', 'reset_reproject', 'reset_jump'),
                                    (float(x) for x in solve[-1])))
    summary['degraded'] = 'DEGRADED' in log
    with open(os.path.join(a.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary, indent=1))
    return 0


def load(d):
    s = json.load(open(os.path.join(d, 'summary.json')))
    g = []
    for r in csv.DictReader(open(os.path.join(d, 'goals.csv'))):
        g.append((float(r['bag_t']), (float(r['gx']), float(r['gy']), float(r['gz']))))
    return s, g


def paired_delta(a, b, tol=0.04):
    """|goal_a - goal_b| for goals within `tol` s of each other in bag time."""
    out, j = [], 0
    for t, g in a:
        while j + 1 < len(b) and abs(b[j + 1][0] - t) <= abs(b[j][0] - t):
            j += 1
        if b and abs(b[j][0] - t) <= tol:
            out.append(math.dist(g, b[j][1]))
    return out


def compare(dirs):
    runs = [load(d) for d in dirs]
    keys = ('goals', 'bursts', 'follow_s', 'step_p50', 'step_p95', 'step_max', 'z_jumps', 'reversals',
            'in_band', 'range_min', 'range_max')
    print('%-24s' % 'run' + ''.join('%14s' % k[:13] for k in keys))
    for d, (s, _) in zip(dirs, runs):
        print('%-24s' % ('%s:%s' % (s['arm'], os.path.basename(os.path.normpath(d)))[:23])
              + ''.join('%14s' % (('%.3f' % s[k]) if isinstance(s.get(k), float) else s.get(k, '-')) for k in keys))
    for s, _ in runs:
        if 'solve' in s:
            print('solve (%s): %s%s' % (s['arm'], s['solve'], '  DEGRADED' if s['degraded'] else ''))
    # Steps of the published goal, from goals.csv. A step > 1 m with no gap is INPUT-driven in
    # both arms (on run5: the trail side flipping 180 deg as a slow walker's heading flips, which
    # the solver inherits through az_des and follows). Left in, those ~5 % of steps ARE the p95,
    # so the p95 only says which side of that cliff the 5 % line fell. Count them; score the rest.
    print('\ngoal steps on the wire, moving ticks only, gaps > 0.5 s excluded:')
    print('%-24s%10s%14s%14s%14s' % ('run', 'moving', 'jumps > 1 m', 'p50 of rest', 'p95 of rest'))
    for d, (s, g) in zip(dirs, runs):
        st = [math.dist(a[1], b[1]) for a, b in zip(g, g[1:]) if b[0] - a[0] <= 0.5]
        st = [x for x in st if x > 1e-6]
        rest = [x for x in st if x <= 1.0]
        print('%-24s%10d%14d%14.3f%14.3f' % (('%s:%s' % (s['arm'], os.path.basename(os.path.normpath(d))))[:23],
                                             len(st), len(st) - len(rest), nc.pct(rest, 0.5), nc.pct(rest, 0.95)))
    by_arm = {}
    for s, g in runs:
        by_arm.setdefault(s['arm'], []).append(g)
    print('\npaired |delta goal| in metres (p50 / p95 / max, n):')
    for arm, gs in by_arm.items():
        if len(gs) >= 2:
            d = paired_delta(gs[0], gs[1])
            print('  WITHIN %-10s %.3f / %.3f / %.3f  (%d)  <- the noise floor of a replay'
                  % (arm, nc.pct(d, 0.5), nc.pct(d, 0.95), max(d) if d else float('nan'), len(d)))
    arms = list(by_arm)
    if len(arms) == 2:
        d = paired_delta(by_arm[arms[0]][0], by_arm[arms[1]][0])
        print('  ACROSS %s vs %s  %.3f / %.3f / %.3f  (%d)'
              % (arms[0], arms[1], nc.pct(d, 0.5), nc.pct(d, 0.95), max(d) if d else float('nan'), len(d)))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag', nargs='?')
    ap.add_argument('--arm', choices=('geometric', 'nmpc'))
    ap.add_argument('--out')
    ap.add_argument('--node-exe', default=None)
    ap.add_argument('--start', type=float, default=0.0, help='skip this many seconds of the bag')
    ap.add_argument('--duration', type=float, default=0.0)
    ap.add_argument('--compare', nargs='+', metavar='DIR')
    a = ap.parse_args(argv)
    if a.compare:
        return compare(a.compare)
    if not (a.bag and a.arm and a.out):
        ap.error('BAG, --arm and --out are required (or --compare DIR...)')
    return replay(a)


if __name__ == '__main__':
    sys.exit(main())
