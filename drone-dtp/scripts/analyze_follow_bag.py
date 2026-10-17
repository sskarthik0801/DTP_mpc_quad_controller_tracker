#!/usr/bin/env python3
"""Turn a recorded follow session into the numbers a paper can quote.

WHY THIS IS A SCRIPT AND NOT A NOTEBOOK
---------------------------------------
Every figure this prints has to be reproducible from a named bag six weeks from now, when the
aircraft is in pieces and nobody remembers which run produced which number. A notebook's state is
invisible; this takes a bag path and prints its provenance alongside every result.

WHAT IT REFUSES TO DO
---------------------
It does not average over conditions the reader would want separated. Detection rate at 2 m and at
9 m are different quantities, and a single "94 %" hides which one failed -- the range-bucketed
tables below exist because a scalar summary of this system was actively misleading twice:

  * "97.3 % detection" was true of one walk and 33 % of the next, because the two covered
    different ranges and one subject left frame.
  * "62 % of frames held" looked like a sparsity problem until the SAME number was bucketed by
    range and turned out to peak at 2-3 m and vanish at 1-2 m and 5-6 m -- not monotonic, so not
    sparsity. That single re-cut changed the diagnosis and the fix.

A BAG IS NOT AUTOMATICALLY VALID
--------------------------------
`bag_follow1` (2026-09-08) was recorded while FAST-LIO was diverged, so every world position in it
is meaningless while its raw sensor streams are fine. This script cannot tell the difference, so
it prints the odometry excursion and lets you judge: a stationary aircraft whose odometry wanders
kilometres invalidates every world-frame number downstream of it.

    python3 scripts/analyze_follow_bag.py /tmp/bag_follow2
"""
import argparse
import collections
import glob
import math
import os
import sqlite3
import statistics as st
import sys

# Range buckets, metres. Chosen to straddle the follow standoff band (2.5-4 m) rather than to be
# round: the interesting failures live at its edges.
BUCKETS = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 8), (8, 12)]
# A forecast this short is the held-frame fallback, not a prediction.
DEGENERATE_FORECAST_PTS = 1


def _open(bag):
    files = sorted(glob.glob(os.path.join(bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % bag)
    con = sqlite3.connect(files[0])
    topics = {r[1]: (r[0], r[2]) for r in
              con.execute('SELECT id,name,type FROM topics').fetchall()}
    return con, topics, files


def _read(con, topics, name, msgtype):
    if name not in topics:
        return []
    from rclpy.serialization import deserialize_message
    tid = topics[name][0]
    out = []
    for ts, data in con.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,)):
        out.append((ts * 1e-9, deserialize_message(bytes(data), msgtype)))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--csv', default='', help='also write per-target rows here')
    args = ap.parse_args(argv)

    from dtp_msgs.msg import TargetArray, Target
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    con, topics, files = _open(args.bag)
    print('=' * 78)
    print('FOLLOW SESSION ANALYSIS   %s' % os.path.basename(args.bag.rstrip('/')))
    print('=' * 78)

    # ---- provenance, and the validity check that must come first ------------------------------
    odom = _read(con, topics, '/lidar_slam/odom', Odometry)
    if odom:
        ex = max(math.sqrt(m.pose.pose.position.x ** 2 + m.pose.pose.position.y ** 2 +
                           m.pose.pose.position.z ** 2) for _t, m in odom)
        verdict = ('OK for world-frame results' if ex < 50.0 else
                   '*** SLAM DIVERGED -- every world position in this bag is meaningless ***')
        print('odometry excursion from origin : %.1f m   %s' % (ex, verdict))
    else:
        print('odometry excursion : no /lidar_slam/odom in this bag')

    tgts = _read(con, topics, '/dtp/targets', TargetArray)
    if not tgts:
        print('no /dtp/targets -- nothing to analyse')
        return 1
    span = tgts[-1][0] - tgts[0][0]
    print('duration %.1f s   TargetArrays %d   (%.2f Hz)' % (span, len(tgts), len(tgts) / max(span, 1e-9)))

    # ---- headline, then immediately the breakdown that qualifies it ---------------------------
    with_person = sum(1 for _t, m in tgts if m.targets)
    print('arrays carrying a person : %d / %d = %.1f %%'
          % (with_person, len(tgts), 100.0 * with_person / len(tgts)))

    # SEPARATING A REAL FAILURE FROM CORRECT BEHAVIOUR.
    #
    # The obvious proxy for "perception could not range the target this frame" is a degenerate
    # forecast (<= 1 predicted point), and it is WRONG in a way that flatters nothing and confuses
    # everything: a STATIONARY subject legitimately has no motion to extrapolate, so standing
    # still produces the same degenerate forecast as a total depth failure. Measured on a 34 s
    # fragment where the subject was ASKED to stand still: 100 % detection, one clean detection
    # per frame, and 76.9 % "held" by that proxy. Nothing was failing.
    #
    # What actually distinguishes them is in the data: perception's hold path REPUBLISHES THE
    # LAST POSITION UNCHANGED, so a held frame is bit-identical to its predecessor for that track.
    # A stationary subject that WAS successfully ranged still jitters by sensor noise and is never
    # bit-identical. So:
    #     held      -- position exactly equal to the previous sample of the same track
    #     static    -- moved less than STATIC_EPS_M but not exactly equal (ranged, just not walking)
    #     moving    -- moved more than STATIC_EPS_M
    # Only the first is a perception failure and only it belongs in a paper as a failure rate.
    STATIC_EPS_M = 0.05
    last = {}
    rows = []
    for t, m in tgts:
        for g in m.targets:
            key = int(g.track_id)
            p = (g.position.x, g.position.y, g.position.z)
            prev = last.get(key)
            if prev is None:
                kind = 'first'
            elif p == prev:
                kind = 'held'            # bit-identical: the hold path republished
            elif math.dist(p, prev) < STATIC_EPS_M:
                kind = 'static'          # ranged fine, subject simply is not walking
            else:
                kind = 'moving'
            last[key] = p
            rows.append(dict(t=t, track=g.track_id, d=g.distance_m, conf=g.confidence,
                             npts=len(g.predicted_path), kind=kind,
                             horizon=max(g.predicted_path_dt) if g.predicted_path_dt else 0.0,
                             v=math.hypot(g.velocity.x, g.velocity.y),
                             confirmed=g.track_confirmed))
    print('detections %d   distinct track ids %d' % (len(rows), len({r['track'] for r in rows})))

    print()
    print('BY RANGE -- a scalar summary of this system has been misleading twice; see the header')
    print('  range      dets    held%   conf   median|v|   forecast pts   (held = NO RANGE)')
    for lo, hi in BUCKETS:
        sub = [r for r in rows if lo <= r['d'] < hi]
        if len(sub) < 15:
            continue
        held = sum(1 for r in sub if r['kind'] == 'held')
        pts = st.median([r['npts'] for r in sub])
        print('  %2d-%-2d m %7d %8.1f %6.2f %10.2f %13.0f'
              % (lo, hi, len(sub), 100.0 * held / len(sub),
                 st.mean([r['conf'] for r in sub]),
                 st.median([r['v'] for r in sub]), pts))

    print()
    kinds = collections.Counter(r['kind'] for r in rows)
    n = max(len(rows), 1)
    print('WHY A SAMPLE LOOKS DEGENERATE -- the distinction that decides whether it is a failure')
    print('  held   (position republished unchanged -- NO RANGE, a real failure) : %5d  %5.1f %%'
          % (kinds['held'], 100.0 * kinds['held'] / n))
    print('  static (ranged, subject not walking -- CORRECT behaviour)           : %5d  %5.1f %%'
          % (kinds['static'], 100.0 * kinds['static'] / n))
    print('  moving (ranged, subject walking)                                    : %5d  %5.1f %%'
          % (kinds['moving'], 100.0 * kinds['moving'] / n))
    print('  first  (no predecessor to compare)                                  : %5d  %5.1f %%'
          % (kinds['first'], 100.0 * kinds['first'] / n))

    old = sum(1 for r in rows if r['npts'] <= DEGENERATE_FORECAST_PTS)
    print()
    print('  The old proxy (forecast <= %d point) counted %d / %d = %.1f %% as "held".'
          % (DEGENERATE_FORECAST_PTS, old, len(rows), 100.0 * old / n))
    print('  It counts a stationary subject as a perception failure. Quote the FIRST line above.')
    # How often does the old proxy fire on a sample that was ranged perfectly well?
    mis = sum(1 for r in rows
              if r['npts'] <= DEGENERATE_FORECAST_PTS and r['kind'] in ('static', 'moving'))
    if old:
        print('  Of those %d, %d (%.0f %%) were RANGED FINE -- the proxy was wrong about them.'
              % (old, mis, 100.0 * mis / old))
    hz = collections.Counter(round(r['horizon'], 1) for r in rows)
    print('forecast horizon, most common     : %s'
          % ', '.join('%.1fs x%d' % (k, v) for k, v in hz.most_common(4)))
    speeds = sorted(r['v'] for r in rows)
    if speeds:
        print('target speed  median %.2f  p90 %.2f  max %.2f m/s   (a walk is ~1.4)'
              % (speeds[len(speeds) // 2], speeds[int(len(speeds) * 0.9)], speeds[-1]))

    # ---- identity continuity: the number a reviewer asks about ---------------------------------
    # PER-TRACK LIFETIMES, not consecutive-row comparison.
    #
    # The naive version of this metric -- walk the detections in time order and count id changes --
    # reported 14.84 switches per second on a run whose 25 s samples showed ONE stable id. It was
    # wrong for a reason worth keeping: this session averages 1.69 detections per frame, so
    # consecutive ROWS are frequently two different people in the SAME frame, and their ordering
    # was being counted as identity churn. A metric that cannot tell "two people" from "one person
    # flickering" is worse than none, because both look like tracker failure.
    per_track = collections.defaultdict(list)
    for r in rows:
        per_track[r['track']].append(r['t'])
    lifetimes = sorted(((max(ts) - min(ts)), tid, len(ts)) for tid, ts in per_track.items())
    n_multi = sum(1 for _t, m in tgts if len(m.targets) > 1)
    print()
    print('IDENTITY')
    print('  distinct ids %d over %.0f s   frames with >1 detection: %d (%.1f %%)'
          % (len(per_track), span, n_multi, 100.0 * n_multi / max(len(tgts), 1)))
    long_lived = [x for x in lifetimes if x[0] >= 1.0]
    print('  tracks living >= 1 s : %d   (the rest are transient, mostly single-frame)'
          % len(long_lived))
    print('  longest-lived tracks:')
    for life, tid, n in lifetimes[-5:][::-1]:
        print('     track %-5d %6.1f s  %5d detections' % (tid, life, n))

    # ---- the decision chain, which is what makes this a FOLLOW session and not a detection run --
    print()
    print('DECISION CHAIN')
    for name, label in (('/dtp/follow_target', 'locked target published'),
                        ('/dtp/follow_goal', 'standoff goal published'),
                        ('/planning/follow_goal', 'arbitrated goal to SUPER'),
                        ('/planning/goal_status', 'SUPER answered')):
        if name in topics:
            n = con.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?',
                            (topics[name][0],)).fetchone()[0]
            print('  %-26s %6d msgs  %s' % (name, n, '' if n else '<-- BROKEN LINK'))
        else:
            print('  %-26s   not recorded' % name)

    lock = _read(con, topics, '/dtp/lock_state', String)
    if lock:
        seen = []
        for _t, m in lock:
            if not seen or seen[-1] != m.data:
                seen.append(m.data)
        print()
        print('LOCK STATE -- %d messages, %d transitions:' % (len(lock), len(seen)))
        for s in seen[:12]:
            print('   %s' % s[:110])
        if len(seen) == 1:
            print('   *** ONE state for the whole run: the state machine never advanced. ***')

    if args.csv:
        with open(args.csv, 'w') as fh:
            fh.write('t,track,distance_m,confidence,forecast_pts,horizon_s,speed_mps,confirmed\n')
            for r in sorted(rows, key=lambda r: r['t']):
                fh.write('%.3f,%d,%.3f,%.3f,%d,%.3f,%.3f,%d\n'
                         % (r['t'], r['track'], r['d'], r['conf'], r['npts'],
                            r['horizon'], r['v'], int(r['confirmed'])))
        print('\nwrote %s' % args.csv)
    print('=' * 78)
    return 0


if __name__ == '__main__':
    sys.exit(main())
