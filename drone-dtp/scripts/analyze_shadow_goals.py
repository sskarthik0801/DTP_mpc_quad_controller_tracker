#!/usr/bin/env python3
"""Stage A: were the goals we WOULD have flown correct, and did they survive platform motion?

WHAT THIS ANSWERS, AND WHY IT IS THE FIRST NON-BENCH RESULT
-----------------------------------------------------------
In Stage A the follow stack runs on a flying aircraft and commands nothing: every goal it computes
goes to /dtp/shadow/follow_goal, where nothing subscribes. That makes one question answerable that
a bench cannot answer, and it is the question the paper needs:

    with the aircraft moving -- translating, and above all YAWING -- are the standoff goals the
    stack produces geometrically correct, and does their quality DEGRADE WITH EGO-MOTION?

The second half is the real test. perception_node lifts a detection into the world through the
transform AT THE FRAME STAMP (tf_at_latest: false). The cheap alternative -- taking the newest
transform -- is wrong by yaw-rate x range (risk A8): 0.5 m at 3 rad/s and 6 m. Stationary, the two
are identical, so NO BENCH RUN CAN TELL THEM APART. A flight can: if the stamped lift is right,
target jitter and standoff error are FLAT against yaw rate. If a latest-available lift had been
used, they would climb with it. So the yaw-rate table below is evidence for a design decision that
has so far only been argued, and it is worth a figure.

WHAT IT CANNOT CONCLUDE, STATED UP FRONT
----------------------------------------
"Is the goal in free space?" is NOT answerable from this bag -- ROG-Map's occupancy is not
recorded, and reconstructing it from /cloud_registered_body would be a reimplementation of the
thing under test. What IS checked is the geometry the planner promises: the standoff band, the
hard floor, the z clamp, the trail-behind azimuth, and smoothness. Occupancy is answered by
/planning/goal_status (ACCEPTED / SNAPPED / DROPPED) when SUPER is in the loop -- which in Stage A
it deliberately is not, so that column reads "not applicable" and must not be quietly skipped.

RUNS ON A BENCH BAG TOO, ON PURPOSE
-----------------------------------
It prefers /dtp/shadow/follow_goal and falls back to /dtp/follow_goal, and treats /mux/state as
optional, so it can be exercised on 20260910_flightlegal_bench BEFORE the flight. A script whose
first run is on flight data is a script that fails on flight data.

    python3 scripts/analyze_shadow_goals.py ~/dtp_datasets/20260912_stageA
"""
import argparse
import bisect
import csv
import glob
import math
import os
import sqlite3
import sys

# follow_planner's standoff contract is THREE-TIER, and reading it as one band produces
# a false violation rate -- which this script did on its first run (22.9 % reported outside the
# band, all of it legal). From follow_planner_node.hpp:
#   * [follow_near_m, follow_far_m] = [3.0, 4.0] is the NOMINAL band, before the lead is spent.
#   * the prediction lead may then SPEND standoff toward the person (line ~800), but never
#     below min_standoff_m + kFloorReserveM. A floor the servo sits on every cycle is a
#     setpoint, not a floor, so 0.25 m is held back deliberately.
#   * min_standoff_m = 2.5 m is the ABSOLUTE floor, enforced by a radial push (line ~858).
# So a goal at 2.82 m is correct behaviour, and only below 2.5 m is a contract breach.
FOLLOW_NEAR_M = 3.0
FOLLOW_FAR_M = 4.0
MIN_STANDOFF_M = 2.5
FLOOR_RESERVE_M = 0.25
LEAD_FLOOR_M = MIN_STANDOFF_M + FLOOR_RESERVE_M      # 2.75 m
# command_mux clamps the arbitrated goal into this before SUPER sees it.
GOAL_Z_MIN_M = 0.5
GOAL_Z_MAX_M = 3.0
# Below this the subject's heading is not meaningful, so the trail azimuth is undefined and
# follow_planner deliberately holds the previous one. Those samples are excluded from the azimuth
# check rather than counted as failures.
MOVING_MPS = 0.30
# Pairing window: a goal is compared against the target sample nearest in time. Goals come at
# ~15 Hz from targets at ~10 Hz, so beyond one target period it is a different person-state.
PAIR_WINDOW_S = 0.15
# Yaw-rate buckets for the degradation table, rad/s. The top bucket is where A8 would bite.
YAW_BUCKETS = ((0.00, 0.10), (0.10, 0.30), (0.30, 0.60), (0.60, 1.00), (1.00, 9.99))
# Jitter is measured as the residual from a centred 5-sample median of the target track: robust to
# a single bad sample, and it does not mistake real walking for noise the way a difference does.
JITTER_WIN = 5
# A goal on the wrong side of the subject is only a defect if they were walking STEADILY.
# Within this long of a heading reversal it is the velocity estimate catching up.
REVERSAL_WINDOW_S = 1.0


def _rows(con, tid, topic):
    if topic not in tid:
        return []
    return con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid[topic],)).fetchall()


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _nearest(ts, t):
    """Index into sorted `ts` of the entry nearest `t`, or None if empty."""
    if not ts:
        return None
    i = bisect.bisect_left(ts, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(ts) and (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
            best = j
    return best


def _pct(v, q):
    if not v:
        return float('nan')
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def _mean(v):
    return sum(v) / len(v) if v else float('nan')


# M5.5 exit thresholds (docs/m5-m6-plan.md SS2.2), added 2026-09-13.
M55_STANDOFF_ERR_P95_M = 0.5
M55_JITTER_P95_M = 0.3
M55_SLOPE_MAX_M_PER_RADS = 0.1
M55_SLOPE_YAW_MAX_RADS = 0.8
M55_BLIND_MAX_FRAC = 0.03
# The yaw-rate regression means nothing without yaw-rate spread to regress over.
M55_MIN_YAW_SPREAD_RADS = 0.3
# Fewer FOLLOW goals than this and S1/S3 are not scored: a percentile of a handful is noise.
M55_MIN_FOLLOW_GOALS = 50
# --window airborne: odom z above this is flight. Segments shorter than AIR_MIN_SEG_S are dropped
# (a hand-carry reads as 7 s at 0.8 m) and gaps shorter than that are bridged (a bounce on
# landing is not two flights). 2026-09-13: the M5.5 scorer must score the FLIGHT -- hop 2's
# only S4 failure was a 30 s FAST-LIO stall on the ground before arming.
AIRBORNE_Z_M = 0.30
AIR_MIN_SEG_S = 10.0


def _segments(ts, flags, min_seg_s):
    """[(t0, t1), ...] where `flags` is true, gaps and runs shorter than min_seg_s removed."""
    segs = []
    cur = None
    for tt, f in zip(ts, flags):
        if f and cur is None:
            cur = [tt, tt]
        elif f:
            cur[1] = tt
        elif cur is not None:
            segs.append(cur)
            cur = None
    if cur is not None:
        segs.append(cur)
    merged = []
    for s in segs:
        if merged and s[0] - merged[-1][1] < min_seg_s:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    return [(a, b) for a, b in merged if b - a >= min_seg_s]


def _in_window(segs, tt):
    return any(a <= tt <= b for a, b in segs)


def _slope(xs, ys):
    n = len(xs)
    if n < 3:
        return float('nan')
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return float('nan') if sxx <= 1e-12 else sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx


def m55_verdict(rows, airborne, blind_frac, stage_a):
    """Score the Stage A exit. Returns 0 PASS, 1 FAIL, 3 INCOMPLETE (not a pass).

    WHAT "STANDOFF-GOAL ERROR" MEANS HERE. The distance of each goal OUTSIDE the band the planner
    is allowed to command, [LEAD_FLOOR_M, FOLLOW_FAR_M] = [2.75, 4.0] m -- zero inside it. Not
    |standoff - 3.5 m|: the prediction lead legitimately spends up to 0.75 m of standoff at walking
    pace (section 1's docstring), so a deviation from the midpoint is the design working, and a
    threshold on it would fail a correct planner on every walking subject.

    WHICH GOALS. Only those the mux emitted under FOLLOW, when /mux/state is in the bag. The
    shadow topic carries the ARBITRATED stream, and a HOLD goal is latched near the drone by design,
    nowhere near the standoff band -- scoring it failed 20260912_2034_stageA at 2.7 m on 5 HOLD
    goals out of 6 paired. Jitter (S2) is about the lift, so it uses every sample.

    WHAT THE SLOPE IS FITTED ON. The lifted target's jitter against the platform's yaw rate, over
    samples at <= 0.8 rad/s: the A8 question is whether the stamped transform lift degrades with
    ego-motion, and jitter is where a wrong lift shows. The standoff-error slope is printed too.
    """
    print()
    print('6. M5.5 VERDICT (docs/m5-m6-plan.md SS2.2)%s'
          % ('' if stage_a else ' -- NOT A STAGE A BAG: geometry only, this is not the gate'))
    out = []

    def v(code, ok, what, detail):
        out.append(ok)
        print('   %s  %-3s %-46s %s' % ({True: 'PASS', False: 'FAIL', None: 'n/a '}[ok], code, what, detail))

    have_mux = any(r['mux'] for r in rows)
    follow = [r for r in rows if r['mux'] == 'FOLLOW'] if have_mux else rows
    which = ('%d FOLLOW goals of %d' % (len(follow), len(rows))) if have_mux else \
        ('all %d goals -- no /mux/state, HOLD goals cannot be excluded' % len(rows))
    err = [max(0.0, LEAD_FLOOR_M - r['standoff'], r['standoff'] - FOLLOW_FAR_M) for r in follow]
    if len(follow) < M55_MIN_FOLLOW_GOALS:
        v('S1', None, 'standoff-goal error p95', 'only %s (need >= %d)' % (which, M55_MIN_FOLLOW_GOALS))
    else:
        e95 = _pct(err, .95)
        v('S1', e95 <= M55_STANDOFF_ERR_P95_M, 'standoff-goal error p95',
          '%.3f m outside [%.2f, %.1f] m over %s (need <= %.1f); |standoff - 3.5| p95 %.2f m for reference'
          % (e95, LEAD_FLOOR_M, FOLLOW_FAR_M, which, M55_STANDOFF_ERR_P95_M,
             _pct([abs(r['standoff'] - 0.5 * (FOLLOW_NEAR_M + FOLLOW_FAR_M)) for r in follow], .95)))
    j95 = _pct([r['jitter'] for r in rows], .95)
    v('S2', j95 <= M55_JITTER_P95_M, 'lifted-target jitter p95', '%.3f m (need <= %.1f)' % (j95, M55_JITTER_P95_M))
    sel = [r for r in follow if not math.isnan(r['yaw_rate']) and r['yaw_rate'] <= M55_SLOPE_YAW_MAX_RADS]
    yr = [r['yaw_rate'] for r in sel]
    spread = (max(yr) - min(yr)) if yr else 0.0
    if len(sel) < M55_MIN_FOLLOW_GOALS:
        v('S3', None, 'error slope vs yaw rate', 'only %d FOLLOW goals at <= %.1f rad/s (need >= %d)'
          % (len(sel), M55_SLOPE_YAW_MAX_RADS, M55_MIN_FOLLOW_GOALS))
    elif not airborne or spread < M55_MIN_YAW_SPREAD_RADS:
        v('S3', None, 'error slope vs yaw rate',
          'yaw rate spans only %.2f rad/s (need >= %.1f to fit) -- %s'
          % (spread, M55_MIN_YAW_SPREAD_RADS, 'stationary bag' if not airborne else 'the aircraft barely yawed'))
    else:
        sj = _slope(yr, [r['jitter'] for r in sel])
        se = _slope(yr, [max(0.0, LEAD_FLOOR_M - r['standoff'], r['standoff'] - FOLLOW_FAR_M) for r in sel])
        v('S3', sj <= M55_SLOPE_MAX_M_PER_RADS, 'jitter slope vs yaw rate (<= %.1f rad/s)' % M55_SLOPE_YAW_MAX_RADS,
          '%+.3f m per rad/s over n %d (need <= %.1f); standoff-error slope %+.3f'
          % (sj, len(sel), M55_SLOPE_MAX_M_PER_RADS, se))
    if blind_frac is None:
        v('S4', None, 'perception blind', '/dtp/perception_ok not in the bag')
    else:
        v('S4', blind_frac <= M55_BLIND_MAX_FRAC, 'perception blind',
          '%.1f %% of /dtp/perception_ok False (need <= %.0f %%)' % (100 * blind_frac, 100 * M55_BLIND_MAX_FRAC))
    if False in out:
        print('   M5.5: FAIL')
        return 1
    if None in out or not stage_a:
        print('   M5.5: INCOMPLETE -- not every threshold could be scored%s (NOT a pass)'
              % ('' if stage_a else ', and this is not a Stage A bag'))
        return 3
    print('   M5.5: PASS')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--out', default=None, help='CSV path (default <bag>/shadow_goals.csv)')
    ap.add_argument('--goal-topic', default=None,
                    help="score this goal topic instead of the default (the shadow topic, else "
                         "/dtp/follow_goal). /dtp/shadow/follow_goal is command_mux's ARBITRATED "
                         "output -- fence-clamped, guarded, HOLD-latched; /dtp/follow_goal is "
                         "follow_planner's own. To compare two goal_mode arms score "
                         "/dtp/follow_goal on both: raw against raw, never raw against arbitrated.")
    ap.add_argument('--window', choices=('airborne', 'armed', 'all'), default='airborne',
                    help='score only this part of the bag: airborne = odom z > %.2f m (default; '
                         'falls back to all when nothing flew), armed = PX4 arming_state ARMED, '
                         'all = the whole bag (the pre-2026-09-13 behaviour)' % AIRBORNE_Z_M)
    args = ap.parse_args(argv)

    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from dtp_msgs.msg import Target

    files = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % args.bag)
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}

    goal_topic = args.goal_topic or ('/dtp/shadow/follow_goal' if '/dtp/shadow/follow_goal' in tid
                                     else '/dtp/follow_goal')
    if goal_topic not in tid:
        raise SystemExit('bag has no %s' % (args.goal_topic or
                                            '/dtp/shadow/follow_goal nor /dtp/follow_goal'))
    # Stage A is a property of the FLIGHT (the mux's output was shadowed), not of the topic
    # scored: with --goal-topic /dtp/follow_goal a Stage A bag is still a Stage A bag.
    stage_a = '/dtp/shadow/follow_goal' in tid
    print()
    print('=' * 78)
    print('SHADOW-GOAL ANALYSIS   %s' % os.path.basename(args.bag.rstrip(os.sep)))
    print('=' * 78)
    print('goal source : %s   (%s)'
          % (goal_topic, 'STAGE A, arbitrated, commanded nothing' if goal_topic.startswith('/dtp/shadow')
             else "STAGE A bag, follow_planner's RAW goal (--goal-topic)" if stage_a
             else 'follow_planner raw -- NOT a Stage A bag, geometry only'))

    # -- target track (the locked subject) -------------------------------------------------------
    tgt_t, tgt_p, tgt_v = [], [], []
    for _ts, d in _rows(con, tid, '/dtp/follow_target'):
        m = deserialize_message(bytes(d), Target)
        tgt_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        tgt_p.append((m.position.x, m.position.y, m.position.z))
        tgt_v.append((m.velocity.x, m.velocity.y, m.velocity.z))
    if len(tgt_t) < 10:
        raise SystemExit('only %d /dtp/follow_target samples -- nothing to compare goals against'
                         % len(tgt_t))

    # -- odometry: pose, yaw, and the yaw RATE that the whole point of this rests on -------------
    od_t, od_p, od_yaw = [], [], []
    for _ts, d in _rows(con, tid, '/lidar_slam/odom'):
        m = deserialize_message(bytes(d), Odometry)
        od_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        p = m.pose.pose.position
        od_p.append((p.x, p.y, p.z))
        od_yaw.append(_yaw_of(m.pose.pose.orientation))
    od_rate = [0.0] * len(od_t)
    od_speed = [0.0] * len(od_t)
    for i in range(1, len(od_t)):
        dt = od_t[i] - od_t[i - 1]
        if dt > 1e-4:
            od_rate[i] = abs(_wrap(od_yaw[i] - od_yaw[i - 1])) / dt
            od_speed[i] = math.dist(od_p[i], od_p[i - 1]) / dt
    moved = math.dist(od_p[0], od_p[-1]) if od_p else 0.0
    span = (max(od_p, key=lambda q: q[0])[0] - min(od_p, key=lambda q: q[0])[0]) if od_p else 0.0
    airborne = max(od_speed) > 0.25 if od_speed else False
    print('odometry    : %d samples, %.1f s; net displacement %.2f m, x-span %.2f m'
          % (len(od_t), od_t[-1] - od_t[0] if od_t else 0.0, moved, span))
    print('              max speed %.2f m/s, max yaw rate %.2f rad/s  => %s'
          % (max(od_speed) if od_speed else 0.0, max(od_rate) if od_rate else 0.0,
             'PLATFORM WAS MOVING' if airborne else 'PLATFORM ESSENTIALLY STATIONARY'))
    if not airborne:
        print('              *** the yaw-rate table below cannot test A8 on a stationary bag. ***')

    # -- the scoring window: the flight, not the bag ----------------------------------------------
    # Times below are bag-relative to the first odom stamp, the same base as the per-goal rows.
    t_base = od_t[0] if od_t else 0.0
    score_win = None
    if args.window == 'armed':
        try:
            from px4_msgs.msg import VehicleStatus
            st_t, st_a = [], []
            for _ts, d in _rows(con, tid, '/fmu/out/vehicle_status_v1'):
                st_t.append(_ts * 1e-9 - t_base)
                st_a.append(deserialize_message(bytes(d), VehicleStatus).arming_state == 2)
            score_win = _segments(st_t, st_a, 2.0) if st_t else []
        except ImportError:
            print('window      : px4_msgs not importable -- falling back to airborne')
            args.window = 'airborne'
    if args.window == 'airborne':
        score_win = _segments([tt - t_base for tt in od_t], [p[2] > AIRBORNE_Z_M for p in od_p],
                              AIR_MIN_SEG_S)
    if score_win is not None and not score_win:
        print('window      : --window %s found nothing (never %s) -- scoring the WHOLE bag; this is'
              % (args.window, 'left the ground' if args.window == 'airborne' else 'armed'))
        print('              a bench result, not a flight result')
        score_win = None
    if score_win is None:
        print('window      : whole bag (%.0f s)' % ((od_t[-1] - od_t[0]) if od_t else 0.0))
    else:
        print('window      : %s -- %d segment(s), %.0f s of %.0f s: %s'
              % (args.window, len(score_win), sum(b - a for a, b in score_win),
                 (od_t[-1] - od_t[0]) if od_t else 0.0,
                 ', '.join('%.0f-%.0f s' % s for s in score_win)))

    # -- target jitter: residual from a centred median of the track ------------------------------
    jitter = [0.0] * len(tgt_p)
    h = JITTER_WIN // 2
    for i in range(len(tgt_p)):
        lo, hi = max(0, i - h), min(len(tgt_p), i + h + 1)
        win = tgt_p[lo:hi]
        med = tuple(sorted(c[k] for c in win)[len(win) // 2] for k in (0, 1, 2))
        jitter[i] = math.dist(tgt_p[i], med)

    # -- optional streams ------------------------------------------------------------------------
    mux_t, mux_s = [], []
    for _ts, d in _rows(con, tid, '/mux/state'):
        mux_t.append(_ts * 1e-9)
        mux_s.append(deserialize_message(bytes(d), String).data)
    mux_in_win = [score_win is None or _in_window(score_win, tt - t_base) for tt in mux_t]
    yaw_tg = _rows(con, tid, '/dtp/shadow/yaw_target') or _rows(con, tid, '/dtp/target_point')
    raw_goals = len(_rows(con, tid, '/dtp/follow_goal'))

    # -- the join: every goal against the person and the platform at that instant ---------------
    rows = []
    unpaired = 0
    for _ts, d in _rows(con, tid, goal_topic):
        g = deserialize_message(bytes(d), PoseStamped)
        gt = g.header.stamp.sec + g.header.stamp.nanosec * 1e-9
        gp = (g.pose.position.x, g.pose.position.y, g.pose.position.z)
        i = _nearest(tgt_t, gt)
        if i is None or abs(tgt_t[i] - gt) > PAIR_WINDOW_S:
            unpaired += 1
            continue
        j = _nearest(od_t, gt)
        k = _nearest(mux_t, gt) if mux_t else None
        tp, tv = tgt_p[i], tgt_v[i]
        dx, dy = gp[0] - tp[0], gp[1] - tp[1]
        standoff = math.hypot(dx, dy)
        speed = math.hypot(tv[0], tv[1])
        # Trail azimuth: the goal should sit OPPOSITE the subject's heading, i.e. (goal - target)
        # anti-parallel to v. 0 deg = perfectly behind them.
        trail = float('nan')
        if speed >= MOVING_MPS and standoff > 1e-6:
            trail = math.degrees(abs(_wrap(math.atan2(dy, dx) - math.atan2(-tv[1], -tv[0]))))
        # Heading reversal: did the subject's velocity direction flip by more than 90 deg
        # anywhere inside REVERSAL_WINDOW_S of this sample?
        reversing = False
        if speed >= MOVING_MPS:
            hdg = math.atan2(tv[1], tv[0])
            lo = bisect.bisect_left(tgt_t, tgt_t[i] - REVERSAL_WINDOW_S)
            hi = bisect.bisect_right(tgt_t, tgt_t[i] + REVERSAL_WINDOW_S)
            for q in range(lo, hi):
                vq = tgt_v[q]
                if math.hypot(vq[0], vq[1]) < MOVING_MPS:
                    continue
                if abs(_wrap(math.atan2(vq[1], vq[0]) - hdg)) > math.pi / 2:
                    reversing = True
                    break
        rows.append(dict(
            t=gt - (od_t[0] if od_t else gt), gx=gp[0], gy=gp[1], gz=gp[2],
            tx=tp[0], ty=tp[1], tz=tp[2], standoff=standoff, trail_deg=trail,
            subj_speed=speed, jitter=jitter[i],
            yaw_rate=od_rate[j] if j is not None else float('nan'),
            drone_speed=od_speed[j] if j is not None else float('nan'),
            mux=(mux_s[k].split('|')[0] if k is not None else ''),
            mux_reason=(mux_s[k].split('|')[1] if k is not None and '|' in mux_s[k] else ''),
            reversing=reversing,
        ))
    if not rows:
        raise SystemExit('no goal paired with a target inside %.2f s -- check the clocks'
                         % PAIR_WINDOW_S)
    n_all_rows = len(rows)
    if score_win is not None:
        rows = [r for r in rows if _in_window(score_win, r['t'])]
    print('goals       : %d paired with a target, %d unpaired (>%.2f s away)%s'
          % (n_all_rows, unpaired, PAIR_WINDOW_S,
             '' if score_win is None else '; %d inside the window' % len(rows)))
    if not rows:
        raise SystemExit('no paired goal inside the %s window -- was FOLLOW ever engaged in flight?'
                         % args.window)
    # Sections 1, 2 and 4 are about the STANDOFF goal, so they score FOLLOW goals. A HOLD goal
    # is the mux parking the drone near where it is (command_mux docstring) -- its distance to
    # the person is how close the person came to the aircraft, which is reported as exactly
    # that. 2026-09-13 hop 1: 5 'goals below the 2.5 m floor' were HOLD goals at the landed
    # aircraft while someone walked up to it.
    have_mux = any(r['mux'] for r in rows)
    follow_rows = [r for r in rows if r['mux'] == 'FOLLOW'] if have_mux else rows
    hold_rows = [r for r in rows if r['mux'] and r['mux'] != 'FOLLOW'] if have_mux else []
    if have_mux:
        print('              %d under FOLLOW (scored below), %d HOLD/other (reported, not scored)'
              % (len(follow_rows), len(hold_rows)))
    if not follow_rows:
        print('              *** no FOLLOW goal in the window: sections 1-4 have nothing to '
              'score ***')
        follow_rows = rows

    # -- 1. does the planner obey its own contract? ----------------------------------------------
    so = [r['standoff'] for r in follow_rows]
    below_floor = [r for r in follow_rows if r['standoff'] < MIN_STANDOFF_M]
    in_reserve = [r for r in follow_rows if MIN_STANDOFF_M <= r['standoff'] < LEAD_FLOOR_M]
    too_far = [r for r in follow_rows if r['standoff'] > FOLLOW_FAR_M + 0.01]
    zbad = [r for r in rows if not (GOAL_Z_MIN_M <= r['gz'] <= GOAL_Z_MAX_M)]
    rows_all = rows
    rows = follow_rows
    print()
    print('1. STANDOFF GEOMETRY -- the planner\'s own contract (%d FOLLOW goals)' % len(rows))
    print('   standoff   mean %.2f m   p05 %.2f   p50 %.2f   p95 %.2f   min %.2f   max %.2f'
          % (_mean(so), _pct(so, .05), _pct(so, .5), _pct(so, .95), min(so), max(so)))
    print('   nominal band [%.1f, %.1f] m; the lead may legally spend down to %.2f m'
          % (FOLLOW_NEAR_M, FOLLOW_FAR_M, LEAD_FLOOR_M))
    print('     beyond %.1f m (too far)     : %d of %d (%.1f %%)'
          % (FOLLOW_FAR_M, len(too_far), len(rows), 100.0 * len(too_far) / len(rows)))
    print('     inside the %.2f m reserve   : %d (%.1f %%) -- legal, but the lead is not meant'
          % (FLOOR_RESERVE_M, len(in_reserve), 100.0 * len(in_reserve) / len(rows)))
    print('                                   to reach here; no margin is left below it')
    floor_note = ('*** EACH IS A GOAL CLOSER THAN D6 PERMITS ***' if below_floor else 'OK')
    print('     BELOW the %.1f m hard floor : %d %s'
          % (MIN_STANDOFF_M, len(below_floor), floor_note))
    print('   z clamp [%.1f, %.1f] m        : %d violations %s'
          % (GOAL_Z_MIN_M, GOAL_Z_MAX_M, len(zbad),
             'OK' if not zbad else '*** CHECK command_mux ***'))
    if hold_rows:
        hd = [r['standoff'] for r in hold_rows]
        print('   HOLD goals (not standoff goals): %d; person-to-hold-point distance min %.2f m, '
              'p05 %.2f m -- how close the person came to the parked aircraft'
              % (len(hold_rows), min(hd), _pct(hd, .05)))

    tr = [r['trail_deg'] for r in rows if not math.isnan(r['trail_deg'])]
    print()
    print('2. TRAIL AZIMUTH -- is the goal BEHIND the walking subject? (0 deg = directly behind)')
    if tr:
        print('   over %d samples, subject speed >= %.2f m/s:' % (len(tr), MOVING_MPS))
        print('   mean %.0f deg   p50 %.0f   p90 %.0f   max %.0f'
              % (_mean(tr), _pct(tr, .5), _pct(tr, .9), max(tr)))
        wrong = [r for r in rows if not math.isnan(r['trail_deg']) and r['trail_deg'] > 90.0]
        rev = [r for r in wrong if r['reversing']]
        print('   goals placed in FRONT of the subject (>90 deg): %d (%.1f %%)'
              % (len(wrong), 100.0 * len(wrong) / len(tr)))
        if wrong:
            # A walker who turns round inverts their heading faster than the velocity estimate
            # follows, so the trail azimuth briefly points where they WERE. That is estimator
            # lag at a reversal; a goal on the wrong side of a STEADILY walking person is not.
            print('     of those, %d (%.0f %%) are within %.1f s of a direction REVERSAL'
                  % (len(rev), 100.0 * len(rev) / len(wrong), REVERSAL_WINDOW_S))
            if len(rev) < len(wrong):
                print('     %d are NOT at a reversal -- those are the ones to look at'
                      % (len(wrong) - len(rev)))
    else:
        print('   no samples above %.2f m/s -- azimuth undefined, nothing to check.'
              % MOVING_MPS)

    # -- 3. THE STAGE A TABLE --------------------------------------------------------------------
    print()
    print('3. *** DOES EGO-MOTION DEGRADE THE LIFT? *** (the A8 question a bench cannot answer)')
    print('   If the stamp-exact transform lookup is correct these columns are FLAT. A')
    print('   latest-available lookup would make them climb with yaw rate (yaw-rate x range).')
    print('   yaw rate (rad/s)      n   target jitter (m)      standoff sd (m)   subj speed')
    for lo, hi in YAW_BUCKETS:
        sel = [r for r in rows if not math.isnan(r['yaw_rate']) and lo <= r['yaw_rate'] < hi]
        if len(sel) < 8:
            continue
        js = [r['jitter'] for r in sel]
        ss = [r['standoff'] for r in sel]
        m = _mean(ss)
        sd = math.sqrt(_mean([(x - m) ** 2 for x in ss]))
        spd = _mean([r['subj_speed'] for r in sel])
        print('   %4.2f - %4.2f      %5d   mean %.3f p90 %.3f        %.3f        %.2f m/s'
              % (lo, hi, len(sel), _mean(js), _pct(js, .9), sd, spd))
    if not airborne:
        print('   (stationary bag: every sample is in the bottom bucket, so this proves nothing')
        print('    about A8 -- it only shows the pipeline is self-consistent at rest.)')

    # -- 4. smoothness ---------------------------------------------------------------------------
    jumps = [math.dist((a['gx'], a['gy'], a['gz']), (b['gx'], b['gy'], b['gz']))
             for a, b in zip(rows, rows[1:]) if 0 < b['t'] - a['t'] < 0.30]
    print()
    print('4. GOAL SMOOTHNESS -- a goal that teleports makes SUPER replan from scratch')
    if jumps:
        print('   consecutive goal step  mean %.3f m  p90 %.3f  p99 %.3f  MAX %.3f m'
              % (_mean(jumps), _pct(jumps, .9), _pct(jumps, .99), max(jumps)))
        pairs = [(a, b, d) for a, b, d in zip(rows, rows[1:], jumps) if d > 1.0]
        print('   steps > 1.0 m: %d of %d (%.1f %%)'
              % (len(pairs), len(jumps), 100.0 * len(pairs) / len(jumps)))
        for _a, b, d in pairs[:5]:
            # Attribute each one: a jump while the subject reverses is the standoff azimuth
            # swinging around them, which is expected. A jump with the subject walking
            # steadily is not, and is the only kind worth chasing.
            print('     t=%6.1f s  %.2f m  subject %.2f m/s  %s'
                  % (b['t'], d, b['subj_speed'],
                     'at a REVERSAL -- azimuth swinging round them' if b['reversing']
                     else 'subject walking steadily -- INVESTIGATE'))
    else:
        print('   too few consecutive goals to measure.')

    # -- 5. what the safety layer would have done ------------------------------------------------
    print()
    print('5. ARBITRATION -- what the safety layer would have done in flight')
    if mux_t:
        hist = {}
        for s, ok in zip(mux_s, mux_in_win):
            if not ok:
                continue
            st = s.split('|')[0]
            rs = s.split('|')[1] if '|' in s else ''
            hist[(st, rs)] = hist.get((st, rs), 0) + 1
        tot = sum(hist.values())
        for (st, rs), c in sorted(hist.items(), key=lambda x: -x[1])[:8]:
            print('   %-7s %-14s %6d  (%.1f %%)' % (st, rs, c, 100.0 * c / tot))
        foll = sum(c for (st, _), c in hist.items() if st == 'FOLLOW')
        print('   => FOLLOW authority for %.1f %% of the arbitration stream' % (100.0 * foll / tot))
    else:
        print('   /mux/state NOT in this bag -- arbitration cannot be reconstructed.')
    if raw_goals and stage_a:
        print('   follow_planner produced %d goals; %d were arbitrated onto the shadow topic'
              % (raw_goals, n_all_rows + unpaired))
    print('   SUPER goal_status: %s'
          % ('present -- occupancy WAS checked' if '/planning/goal_status' in tid
             else 'NOT APPLICABLE in Stage A (fsm_node never gets our goals), so whether a '
                  'goal was in free space is NOT answered by this run'))
    if yaw_tg:
        print('   yaw target published %d times (the drone would have faced them)'
              % len(yaw_tg))

    out_csv = args.out or os.path.join(args.bag, 'shadow_goals.csv')
    with open(out_csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows_all[0].keys()))
        w.writeheader()
        w.writerows(rows_all)
    print()
    print('per-goal rows -> %s' % out_csv)
    print('(for the figure: jitter and standoff sd against yaw_rate)')

    # -- 6. M5.5 verdict (docs/m5-m6-plan.md SS2.2) ----------------------------------------------
    blind = None
    pok = _rows(con, tid, '/dtp/perception_ok')
    if pok:
        from std_msgs.msg import Bool
        vals = [deserialize_message(bytes(d), Bool).data for _ts, d in pok
                if score_win is None or _in_window(score_win, _ts * 1e-9 - t_base)]
        blind = (1.0 - sum(1 for x in vals if x) / len(vals)) if vals else None
    verdicts = m55_verdict(rows_all, airborne, blind, stage_a)
    scored_on = ('the whole bag' if score_win is None
                 else '%s window, %.0f s' % (args.window, sum(b - a for a, b in score_win)))
    print('   scored on: %s' % scored_on)
    print('=' * 78)
    return verdicts


if __name__ == '__main__':
    sys.exit(main())
