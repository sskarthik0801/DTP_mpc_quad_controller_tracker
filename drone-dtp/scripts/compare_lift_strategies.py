#!/usr/bin/env python3
"""Stamp-exact vs latest-available transform lift, compared PAIRED on one recording.

THE CLAIM THIS MEASURES, AND WHY IT NEEDS A PAIRED DESIGN
--------------------------------------------------------
perception_node lifts a detection into the world through the transform valid at the CAMERA
FRAME'S OWN STAMP. The cheaper alternative -- taking the newest transform available when the
lookup happens -- is wrong by roughly range x yaw-rate x staleness (risk A8). Arguing that from
the live system is impossible: the node can only run one strategy at a time, and re-flying with
the other one to compare would change the scene as well as the strategy.

A recording solves it exactly. The bag carries the transform stream, so BOTH lifts can be
recomputed for EVERY detection from the same bytes. The static body->camera part is time
invariant and therefore cancels, which is what makes this reconstruction exact rather than
approximate:

    p_world = T_wb(t) . T_bc . p_cam                    what perception published
    p_body  = T_wb(t)^-1 . p_world                      T_bc . p_cam, recovered without knowing T_bc
    p_world' = T_wb(t_latest) . p_body                  what the other strategy would have published
    error   = |p_world' - p_world|

T_bc never has to be known, so no calibration enters the comparison.

WHAT "LATEST" MEANS, AND WHY IT IS A MODELLING CHOICE RATHER THAN A FACT
-----------------------------------------------------------------------
Delta t -- the staleness the error is proportional to -- is the gap between the frame stamp
and the stamp of the newest transform available WHEN THE LOOKUP HAPPENS. That instant depends
on where in the pipeline a latest-available implementation would do the lift, so it is an
assumption and is exposed as one:

  --lookup-at publish   (default) the lookup happens when the result was published. MEASURED
                        on the bench bag at Delta t = 194 ms mean, because perception's own
                        latency (0.245 s, dominated by the wait for odometry and 38 ms of
                        inference) has already elapsed. This is an UPPER BOUND on the error a
                        latest-available lift would incur.
  --lookup-at <ms>      the lookup happens this many ms after the frame stamp. The real
                        tf_at_latest path did NOT wait for odometry -- it lifted as soon as the
                        cloud was built -- so ~40 ms models it.

MEASURED on the bench bag, and the result is not what was assumed when this was written:

    --lookup-at publish   Delta t = +194 ms mean.  Disagreement 5.5 mm mean, 19.4 mm max.
    --lookup-at 40        Delta t =  -10 ms mean.  Disagreement 1.4 mm mean, 12.9 mm max.

The second number is NEGATIVE, and that is the interesting part: the transform stream itself
lags real time by ~50 ms, so a lift performed 40 ms after the frame stamp takes a transform
whose stamp is still slightly BEHIND that frame. A latest-available lift done promptly is
therefore nearly identical to a stamp-exact one -- it extrapolates backwards by ~10 ms rather
than forwards.

So Delta t is NOT a property of the transform stream. It is set by WHERE IN THE PIPELINE the
lift happens, and it spans -10 ms to +194 ms on the same recording depending only on that
choice. A system with our end-to-end latency (0.245 s) lifting at publish time is exposed to
the large end; one lifting immediately is not. Report both bounds; the conclusion that a flight
would resolve the question does not depend on which is used, because both exceed the measured
stationary agreement of 5.5 mm by one to two orders of magnitude once yaw is real.

WHAT THE ANSWER MEANS ON A STATIONARY RECORDING, WHICH IS THE ONLY KIND WE HAVE
------------------------------------------------------------------------------
Near zero, and that is the CONTROL, not the result: with the platform still, T_wb(t) and
T_wb(t_latest) describe the same pose, so the two strategies agree by construction. Running this
on a bench bag therefore measures only odometry noise and proves the arithmetic is implemented
correctly. It is worth doing for exactly two reasons -- it turns the paper's "algebraically
identical when stationary" into a measured bound, and it means flight day needs no new code.

    python3 scripts/compare_lift_strategies.py ~/dtp_datasets/20260910_flightlegal_bench
"""
import argparse
import bisect
import csv
import glob
import math
import os
import sqlite3
import sys

# The one time-varying link in the chain perception looks up. Everything from the body frame to
# the camera optical frame is on /tf_static and cancels in the reconstruction above.
WORLD_FRAME = 'lidar_init'
BODY_FRAME = 'lidar_imu'
# Yaw-rate buckets, rad/s. The top bucket is where eq. (1) of the paper would bite hardest.
YAW_BUCKETS = ((0.00, 0.10), (0.10, 0.30), (0.30, 0.60), (0.60, 1.00), (1.00, 9.99))
# A transform pair further apart than this is a gap, not an interpolation; those samples are
# reported separately rather than silently interpolated across a stall.
MAX_TF_GAP_S = 0.30


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('zero-length quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def slerp(q0, q1, u):
    """Shortest-arc quaternion interpolation, matching what tf2 does between two samples."""
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    dot = x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1
    if dot < 0.0:                      # take the short way round
        x1, y1, z1, w1, dot = -x1, -y1, -z1, -w1, -dot
    if dot > 0.9995:                   # nearly parallel: lerp and renormalise
        x = x0 + u * (x1 - x0)
        y = y0 + u * (y1 - y0)
        z = z0 + u * (z1 - z0)
        w = w0 + u * (w1 - w0)
    else:
        th0 = math.acos(max(-1.0, min(1.0, dot)))
        th = th0 * u
        s0 = math.sin(th0 - th) / math.sin(th0)
        s1 = math.sin(th) / math.sin(th0)
        x, y, z, w = s0 * x0 + s1 * x1, s0 * y0 + s1 * y1, s0 * z0 + s1 * z1, s0 * w0 + s1 * w1
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def apply(R, t, p):
    return (R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t[0],
            R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t[1],
            R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t[2])


def apply_inv(R, t, p):
    """R^T (p - t): the inverse of a rigid transform, without forming a 4x4."""
    d = (p[0] - t[0], p[1] - t[1], p[2] - t[2])
    return (R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
            R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
            R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2])


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pct(v, q):
    if not v:
        return float('nan')
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def mean(v):
    return sum(v) / len(v) if v else float('nan')


class TfTrack:
    """The world<-body transform samples, queryable the two ways under comparison."""

    def __init__(self, stamps, quats, trans):
        self.t = stamps
        self.q = quats
        self.p = trans

    def at_stamp(self, t):
        """Interpolated pose at exactly `t` -- what a stamp-exact lookup returns.

        Returns (q, p, gap) or None when `t` is outside the samples, which is the case tf2
        reports as an extrapolation failure rather than guessing.
        """
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i >= len(self.t):
            return None
        t0, t1 = self.t[i - 1], self.t[i]
        gap = t1 - t0
        if gap <= 0:
            return None
        u = (t - t0) / gap
        return slerp(self.q[i - 1], self.q[i], u), (
            self.p[i - 1][0] + u * (self.p[i][0] - self.p[i - 1][0]),
            self.p[i - 1][1] + u * (self.p[i][1] - self.p[i - 1][1]),
            self.p[i - 1][2] + u * (self.p[i][2] - self.p[i - 1][2]),
        ), gap

    def newest_by(self, wall):
        """Newest sample whose stamp is <= `wall` -- what a latest-available lookup returns.

        Index and stamp come back too, so the caller can report the staleness that eq. (1)
        multiplies by.
        """
        i = bisect.bisect_right(self.t, wall) - 1
        if i < 0:
            return None
        return self.q[i], self.p[i], self.t[i]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--out', default=None, help='CSV path (default <bag>/lift_comparison.csv)')
    ap.add_argument('--lookup-at', default='publish',
                    help='when a latest-available lift would look up: "publish" (upper bound, '
                         'the default) or a number of ms after the frame stamp (e.g. 40)')
    args = ap.parse_args(argv)

    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from dtp_msgs.msg import TargetArray

    files = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % args.bag)
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}
    for need in ('/tf', '/dtp/targets'):
        if need not in tid:
            raise SystemExit('bag has no %s -- cannot reconstruct either lift' % need)

    def rows(topic):
        return con.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (tid[topic],)).fetchall()

    # -- the world<-body transform stream ------------------------------------------------------
    ts, qs, ps = [], [], []
    for _recv, d in rows('/tf'):
        for tf in deserialize_message(bytes(d), TFMessage).transforms:
            if tf.header.frame_id != WORLD_FRAME or tf.child_frame_id != BODY_FRAME:
                continue
            ts.append(tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
            r = tf.transform.rotation
            tr = tf.transform.translation
            qs.append((r.x, r.y, r.z, r.w))
            ps.append((tr.x, tr.y, tr.z))
    if len(ts) < 10:
        raise SystemExit('only %d %s <- %s transforms in the bag'
                         % (len(ts), WORLD_FRAME, BODY_FRAME))
    track = TfTrack(ts, qs, ps)

    # -- platform yaw rate, from odometry ------------------------------------------------------
    od_t, od_rate = [], []
    prev = None
    for _recv, d in rows('/lidar_slam/odom'):
        m = deserialize_message(bytes(d), Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        o = m.pose.pose.orientation
        y = yaw_of((o.x, o.y, o.z, o.w))
        if prev is not None and t > prev[0]:
            od_rate.append(abs(wrap(y - prev[1])) / (t - prev[0]))
            od_t.append(t)
        prev = (t, y)

    def rate_at(t):
        if not od_t:
            return float('nan')
        i = bisect.bisect_left(od_t, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(od_t) and (best is None or abs(od_t[j] - t) < abs(od_t[best] - t)):
                best = j
        return od_rate[best]

    # -- the comparison ------------------------------------------------------------------------
    out = []
    skipped_outside = skipped_gap = no_target = 0
    for recv, d in rows('/dtp/targets'):
        msg = deserialize_message(bytes(d), TargetArray)
        if not len(msg.targets):
            no_target += 1
            continue
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if args.lookup_at == 'publish':
            wall = recv * 1e-9      # when the result was published: the upper bound
        else:
            wall = stamp + float(args.lookup_at) / 1000.0
        got = track.at_stamp(stamp)
        if got is None:
            skipped_outside += 1
            continue
        (q_s, p_s, gap) = got
        if gap > MAX_TF_GAP_S:
            skipped_gap += 1
            continue
        newest = track.newest_by(wall)
        if newest is None:
            skipped_outside += 1
            continue
        q_l, p_l, t_l = newest

        R_s = quat_to_R(*q_s)
        R_l = quat_to_R(*q_l)
        for tgt in msg.targets:
            p_world = (tgt.position.x, tgt.position.y, tgt.position.z)
            # Recover T_bc . p_cam without ever needing T_bc, then re-lift it the other way.
            p_body = apply_inv(R_s, p_s, p_world)
            p_alt = apply(R_l, p_l, p_body)
            err = math.dist(p_world, p_alt)
            rng = math.hypot(p_body[0], p_body[1], p_body[2])
            w = rate_at(stamp)
            dt = t_l - stamp        # staleness of the "latest" transform vs the frame stamp
            out.append(dict(t=stamp - ts[0], err_m=err, range_m=rng, yaw_rate=w,
                            staleness_s=dt, predicted_m=abs(rng * w * dt) if w == w else
                            float('nan')))

    if not out:
        raise SystemExit('no target paired with a transform (outside %d, gap %d, empty %d)'
                         % (skipped_outside, skipped_gap, no_target))

    print()
    print('=' * 78)
    print('LIFT-STRATEGY COMPARISON   %s' % os.path.basename(args.bag.rstrip(os.sep)))
    print('=' * 78)
    print('%s <- %s transforms : %d' % (WORLD_FRAME, BODY_FRAME, len(ts)))
    print('detections compared       : %d   (skipped: %d outside the transform span, %d across '
          'a >%.2f s gap, %d empty arrays)'
          % (len(out), skipped_outside, skipped_gap, MAX_TF_GAP_S, no_target))

    rates = [r['yaw_rate'] for r in out if r['yaw_rate'] == r['yaw_rate']]
    stale = [r['staleness_s'] for r in out]
    print('platform yaw rate         : mean %.3f, p90 %.3f, max %.3f rad/s'
          % (mean(rates), pct(rates, .9), max(rates) if rates else float('nan')))
    model = ('at publish time (UPPER BOUND)' if args.lookup_at == 'publish'
             else 'at stamp + %s ms' % args.lookup_at)
    print('lookup-time model         : %s' % model)
    print('staleness Delta t         : mean %.0f ms, p90 %.0f ms, max %.0f ms'
          % (1000 * mean(stale), 1000 * pct(stale, .9), 1000 * max(stale)))
    moving = [r for r in out if r['yaw_rate'] == r['yaw_rate'] and r['yaw_rate'] >= 0.10]
    print('samples with yaw >= 0.10 rad/s : %d of %d (%.1f %%)'
          % (len(moving), len(out), 100.0 * len(moving) / len(out)))

    errs = [r['err_m'] for r in out]
    print()
    print('DISAGREEMENT BETWEEN THE TWO STRATEGIES, |p_latest - p_stamp|')
    print('  mean %.4f m   p50 %.4f   p90 %.4f   p99 %.4f   MAX %.4f m'
          % (mean(errs), pct(errs, .5), pct(errs, .9), pct(errs, .99), max(errs)))

    print()
    print('BY PLATFORM YAW RATE -- eq. (1) predicts error ~ range x yaw-rate x staleness')
    print('  yaw rate (rad/s)       n   measured |err| (m)        predicted (m)   range')
    for lo, hi in YAW_BUCKETS:
        sel = [r for r in out if r['yaw_rate'] == r['yaw_rate'] and lo <= r['yaw_rate'] < hi]
        if len(sel) < 8:
            continue
        e = [r['err_m'] for r in sel]
        pr = [r['predicted_m'] for r in sel if r['predicted_m'] == r['predicted_m']]
        print('  %4.2f - %4.2f      %6d   mean %.4f p90 %.4f     mean %.4f     %.2f m'
              % (lo, hi, len(sel), mean(e), pct(e, .9), mean(pr),
                 mean([r['range_m'] for r in sel])))

    pred_all = [r['predicted_m'] for r in out if r['predicted_m'] == r['predicted_m']]
    if pred_all and mean(pred_all) > 0:
        ratio = mean(errs) / mean(pred_all)
        print()
        print('EQ. (1) AS A MODEL: measured mean %.4f m vs predicted mean %.4f m (ratio %.2f).'
              % (mean(errs), mean(pred_all), ratio))
        if 0.8 <= ratio <= 1.25:
            print('  The model reproduces the data to within %.0f %%. That is what licenses'
                  % (100 * abs(ratio - 1)))
            print('  extrapolating it to yaw rates this recording does not contain.')
        else:
            print('  The model is off by %.0fx here -- do NOT extrapolate it until that is'
                  % (ratio if ratio > 1 else 1 / ratio))
            print('  understood.')
    print()
    if not moving:
        print('INTERPRETATION: this recording has NO ego-motion, so this is the CONTROL, not the')
        print('result. The two strategies describe the same pose, so the numbers above are')
        print('odometry noise plus interpolation round-off -- they bound how much of any')
        print('future flight measurement is NOT attributable to the strategy. The comparison')
        print('becomes informative only on a recording with real yaw.')
    else:
        print('INTERPRETATION: this recording HAS ego-motion, so the by-bucket table is the')
        print('result.')
        print('A measured column that tracks the predicted column supports eq. (1), and means a')
        print('latest-available lift would have displaced the target by that much. A measured')
        print('column flat near the control bound would mean the effect is smaller than predicted')
        print('on this platform -- which is also a publishable answer, and should be reported.')

    out_csv = args.out or os.path.join(args.bag, 'lift_comparison.csv')
    with open(out_csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader()
        w.writerows(out)
    print()
    print('per-detection rows -> %s' % out_csv)
    print('=' * 78)
    return 0


if __name__ == '__main__':
    sys.exit(main())
