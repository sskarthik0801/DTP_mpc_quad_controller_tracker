#!/usr/bin/env python3
"""Unit checks of the camera yaw servo's pure module (docs/plan-camera-yaw-servo.md SS 4.1).

    python3 scripts/px4/yaw_servo_unit_check.py

No ROS, no pytest, no flight controller: it imports super_bringup/scripts/yaw_servo.py by
path relative to this file, drives YawServo.step() with synthetic ticks and asserts
numbers. One line per check, then "N/N checks passed"; exit status 1 on any failure.
Takes ~10-30 s (most of it the 210,000-tick soak).

What it protects against
------------------------
* **The yaw sign (the spiral trap).** The target arrives in FLU lidar_init; PX4 wants FRD.
  A target to the aircraft's LEFT (FLU +y) must give an FRD yaw < 0, equal to
  -atan2(dy_flu, dx_flu), and NOT the textbook pi/2 - psi. S1-S7, and the soak recomputes
  every servo heading independently.
* **A heading step.** Every source of heading (servo, close-range hold, fall-back to SUPER)
  goes through the 90 deg/s slew; a stalled timer (dt 0.5 s) moves it at most 9 deg.
* **A rate spike.** The feed-forward is capped (60 deg/s), slewed (90 deg/s^2), does not
  chase a one-off target jump, and is never added to SUPER's rate in the fall-back.
* **Switch off must be today's bridge, bit for bit** (Z1), and IDLE must be identity with
  no heading left over from an earlier engagement (I1, I2).

Round two (2026-09-29, after the replay of three recorded flights):
* **R: the fall-back yawspeed (F-a).** SUPER's rate goes out only when the slew clamp did
  not clip, inside the 2 deg band, and clamped to +-max_rate. |yawspeed| <= max(ff_max_rate,
  max_rate) on every tick of the soak, whatever SUPER sends.
* **X: close-range hysteresis.** In at < min_range_m, out at >= min_range_m +
  min_range_hysteresis_m; a flicker across 2.5 m is one entry, no exit.
* **Q: the stale hold.** No usable target after an active tick: hold the last aim, yawspeed
  0 (or the decaying feed-forward) for stale_hold_s of accumulated dt, then fall back.
* **V: zero-valued new parameters = the round-one module.** Digests recorded from the
  round-one yaw_servo.py BEFORE it was changed (see ref_digests()).
Where a round-one check's expectation changes with the new default stale hold (B1, B4, B5,
N1 x7), the round-one assertion still runs with stale_hold_s=0 and the new-default variant
runs beside it (B2 runs under both; its expectation is unchanged). B3 and the soak's
fall-back rule changed because of F-a, which has no switch; the soak's close-range rule
changed because of the hysteresis (its hysteresis-0 soak still covers the round-one rule).
"""
import hashlib
import math
import os
import random
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.normpath(os.path.join(HERE, '..', '..', 'super_bringup', 'scripts'))
sys.path.insert(0, MODULE_DIR)
import yaw_servo as ys  # noqa: E402  (imported by path, see above)

R = math.radians
D = math.degrees
DT = 0.02
EPS = 1e-9
NAN = float('nan')
INF = float('inf')

RESULTS = []


def check(name, ok, detail=''):
    ok = bool(ok)
    RESULTS.append((name, ok, detail))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f'  {detail}' if detail else ''))


def wrap(a):
    """The check's own wrap, deliberately not the module's."""
    return math.atan2(math.sin(a), math.cos(a))


def adiff(a, b):
    return abs(wrap(a - b))


def bits(x):
    return struct.pack('<d', x)


def in_range(a):
    return isinstance(a, float) and math.isfinite(a) and -math.pi < a <= math.pi


def servo(**kw):
    kw.setdefault('enable', True)
    return ys.YawServo(ys.YawServoParams(**kw))


def tick(s, dt=DT, state=ys.TRACK, engaged=True, psi_super=0.0, yawdot_super=0.0,
         heading=0.0, x=0.0, y=0.0, xy_valid=True, target=None):
    """One step. x, y are the vehicle's NED position; target is (x_flu, y_flu, age, frame)."""
    return s.step(dt=dt, state=state, engaged=engaged, psi_super=psi_super,
                  yawdot_super=yawdot_super, vehicle_heading=heading, vehicle_x=x,
                  vehicle_y=y, vehicle_xy_valid=xy_valid, target=target)


class Stream:
    """A yaw target published at `rate` Hz. Message m is stamped t_m = m/rate and carries
    fn(t_m) = (x_flu, y_flu). At tick time t the bridge holds the newest message with
    t_m <= t (none is published at or after t_stop); its age is t - t_m."""

    def __init__(self, fn, rate=15.0, frame='lidar_init', t_stop=None):
        self.fn, self.rate, self.frame, self.t_stop = fn, rate, frame, t_stop

    def at(self, t):
        m = math.floor(t * self.rate + 1e-9)
        if self.t_stop is not None:
            m = min(m, math.ceil(self.t_stop * self.rate - 1e-9) - 1)
        if m < 0:
            return None
        tm = m / self.rate
        x, y = self.fn(tm)
        return (x, y, t - tm, self.frame)


def run(s, T, dt=DT, dt_fn=None, stream=None, **kw):
    """Tick s for T seconds. Keyword inputs may be callables of t. -> [(t, dt, result)]."""
    out = []
    t = 0.0
    k = 0
    while t < T - 1e-9:
        d = dt_fn(k) if dt_fn else dt
        t += d
        args = {key: (v(t) if callable(v) else v) for key, v in kw.items()}
        r = tick(s, dt=d, target=stream.at(t) if stream else None, **args)
        out.append((t, d, r))
        k += 1
    return out


def orbit(theta0_deg, rate_dps, radius=5.0, jumps=()):
    """A FLU target circling the origin: FLU angle theta0 + rate*t (CCW positive), plus
    each (t_j, dtheta_deg) in `jumps` from t_j on. radius may be a callable of t."""
    def fn(t):
        th = theta0_deg + rate_dps * t + sum(j for tj, j in jumps if t >= tj - 1e-12)
        r = radius(t) if callable(radius) else radius
        return (r * math.cos(R(th)), r * math.sin(R(th)))
    return fn


def dt_sane(d):
    return d if (isinstance(d, float) and math.isfinite(d) and d > 0.0) else 0.0


def slew_worst(res, max_rate_dps=90.0):
    """(largest per-tick heading change, largest excess over max_rate*min(dt, 0.1)), rad,
    over consecutive non-passthrough ticks."""
    worst, excess = 0.0, -INF
    for (_, _, a), (_, d1, b) in zip(res, res[1:]):
        if a.passthrough or b.passthrough:
            continue
        ch = adiff(b.yaw, a.yaw)
        worst = max(worst, ch)
        excess = max(excess, ch - R(max_rate_dps) * min(dt_sane(d1), ys.DT_MAX))
    return worst, excess


def ff_slew_worst(res):
    """Largest |d ff|/dt (deg/s^2) over consecutive non-passthrough ticks, leaving out the
    close-range reset to 0 (source 2), which the plan makes immediate."""
    worst = 0.0
    for (_, _, a), (_, d1, b) in zip(res, res[1:]):
        if a.passthrough or b.passthrough or b.source == ys.SOURCE_CLOSE_HOLD:
            continue
        dc = min(dt_sane(d1), ys.DT_MAX)
        diff = abs(b.ff - a.ff)
        if dc > 0.0:
            worst = max(worst, diff / dc)
        elif diff > 0.0:
            worst = INF
    return D(worst)


def settled(res, t_from, t_to=INF):
    v = [r.yawspeed for t, _, r in res if t_from <= t < t_to]
    return min(v), max(v), sum(v) / len(v)


# ---------------------------------------------------------------------------------------

def checks_params():
    print('P   parameters: the plan\'s defaults, validated at start-up')
    p = ys.YawServoParams()
    want = dict(enable=False, ff_enable=False, target_timeout=0.5, max_rate=90.0,
                ff_max_rate=60.0, ff_max_accel=90.0, min_range_m=2.5, camera_offset_deg=0.0,
                future_tolerance=0.2, ff_window=0.2, ff_step_reset_deg=5.0,
                fallback_ff_band_deg=2.0, aim_in_hold=True, min_range_hysteresis_m=0.3,
                stale_hold_s=0.5)
    diff = {k: getattr(p, k) for k, v in want.items() if getattr(p, k) != v}
    check('P1 defaults = plan SS 2.5 (switches OFF, 0.5 s, 90/60/90, 2.5 m, 0 deg) + round two '
          '(hysteresis 0.3 m, stale hold 0.5 s)', not diff, f'mismatch {diff}' if diff else '')
    bad = [dict(max_rate=0.0), dict(max_rate=NAN), dict(target_timeout=-1.0),
           dict(camera_offset_deg=200.0), dict(ff_window=0.0), dict(enable='yes'),
           dict(enable=NAN), dict(min_range_m=INF), dict(ff_max_accel=-1.0),
           dict(max_rate=True), dict(min_range_hysteresis_m=-0.01),
           dict(min_range_hysteresis_m=NAN), dict(min_range_hysteresis_m=INF),
           dict(stale_hold_s=-0.01), dict(stale_hold_s=NAN), dict(stale_hold_s=INF),
           dict(stale_hold_s='0.5'), dict(min_range_hysteresis_m=None)]
    refused = 0
    for kw in bad:
        try:
            ys.YawServoParams(**kw)
        except (ValueError, TypeError):
            refused += 1
    check('P2 a bad parameter is refused at construction, not in flight', refused == len(bad),
          f'{refused}/{len(bad)} refused')


def checks_sign():
    print('\nS   sign: FLU lidar_init target -> FRD heading (the spiral trap)')
    s = servo()
    res = run(s, 2.0, stream=Stream(lambda t: (0.0, 5.0)))
    r0, rf = res[0][2], res[-1][2]
    exp = -math.atan2(5.0, 0.0)
    check('S1 target at FLU +y (LEFT): psi_des = -atan2(dy,dx) = -90 deg FRD',
          r0.psi_des < 0.0 and abs(r0.psi_des - exp) < 1e-12,
          f'psi_des {D(r0.psi_des):+.4f} deg, expected {D(exp):+.4f}')
    check('S2 ... and the first tick turns LEFT (FRD yaw decreases)', r0.yaw < 0.0,
          f'yaw after one tick {D(r0.yaw):+.3f} deg')
    err = D(adiff(rf.yaw, exp))
    check('S3 ... settled yaw < 0 and = -atan2 within 0.5 deg', rf.yaw < 0.0 and err < 0.5,
          f'yaw {D(rf.yaw):+.4f} deg, error {err:.2e} deg')
    wrong = math.pi / 2 - math.atan2(5.0, 0.0)
    check('S4 ... and it is NOT the pi/2 - psi variant', D(adiff(rf.yaw, wrong)) > 45.0,
          f'{D(adiff(rf.yaw, wrong)):.1f} deg from pi/2 - psi = {D(wrong):+.1f} deg')

    worst_des = worst_fin = 0.0
    side_ok = True
    for x, y in [(5, 5), (-5, 5), (-5, -5), (5, -5), (5, 0), (0, -5), (-5, 0.5), (-5, -0.5)]:
        res = run(servo(), 2.4, stream=Stream(lambda t, x=x, y=y: (float(x), float(y))))
        exp = -math.atan2(y, x)
        worst_des = max(worst_des, adiff(res[0][2].psi_des, exp))
        worst_fin = max(worst_fin, adiff(res[-1][2].yaw, exp))
        yf = res[-1][2].yaw
        if (y > 0 and not yf < 0.0) or (y < 0 and not yf > 0.0):
            side_ok = False
    check('S5 all four quadrants: psi_des = -atan2(y,x); settled within 0.5 deg; left<0, right>0',
          worst_des < 1e-12 and D(worst_fin) < 0.5 and side_ok,
          f'psi_des error {D(worst_des):.1e} deg, settled error {D(worst_fin):.1e} deg')

    rnd = random.Random(1)
    worst = 0.0
    n = 0
    while n < 2000:
        ax, ay = rnd.uniform(-50, 50), rnd.uniform(-50, 50)       # aircraft, FLU
        tx, ty = rnd.uniform(-50, 50), rnd.uniform(-50, 50)       # target, FLU
        if math.hypot(tx - ax, ty - ay) < 3.0:
            continue
        r = tick(servo(), x=ax, y=-ay, heading=rnd.uniform(-3.0, 3.0),
                 target=(tx, ty, 0.1, 'lidar_init'))
        worst = max(worst, adiff(r.psi_des, -math.atan2(ty - ay, tx - ax)))
        n += 1
    check('S6 aircraft away from the origin, 2000 random: psi_des = -atan2(dy_flu, dx_flu)',
          worst < 1e-9, f'worst error {D(worst):.1e} deg')
    r = tick(servo(), x=10.0, y=-4.0, target=(13.0, 8.0, 0.1, 'lidar_init'))
    check('S7 aircraft FLU (10,4) = NED (10,-4), target FLU (13,8): FRD -53.13 deg (left)',
          abs(r.psi_des - (-math.atan2(4.0, 3.0))) < 1e-12, f'psi_des {D(r.psi_des):+.3f} deg')


def checks_wrap():
    print('\nW   wrap across +-180 deg')
    # FLU CCW from 150 deg at 30 deg/s: the FRD bearing runs -150 -> -180 = +180 -> +90.
    res = run(servo(), 4.0, stream=Stream(orbit(150.0, 30.0)), heading=R(-150.0),
              psi_super=R(-150.0))
    w, ex = slew_worst(res)
    crossed = any(a.yaw < R(-170) and b.yaw > R(170) for (_, _, a), (_, _, b) in zip(res, res[1:]))
    allin = all(in_range(r.yaw) for _, _, r in res)
    t_end = res[-1][0]
    exp_end = -R(150.0 + 30.0 * math.floor(t_end * 15.0 + 1e-9) / 15.0)
    err_end = D(adiff(res[-1][2].yaw, exp_end))
    check('W1 bearing sweeping through +-180: continuous, every tick <= 90 deg/s * dt',
          crossed and allin and ex <= EPS,
          f'max {D(w):.3f} deg/tick (limit {90 * DT:.3f}); crossed +-180 {crossed}; '
          f'all in (-180, 180] {allin}')
    check('W2 ... and it still tracks the bearing after the wrap', err_end < 2.5,
          f'error at the end {err_end:.3f} deg')
    tgt = (5.0 * math.cos(R(170.0)), 5.0 * math.sin(R(170.0)))   # FLU +170 = FRD -170
    res = run(servo(), 0.6, stream=Stream(lambda t: tgt), heading=R(170.0), psi_super=R(170.0))
    first = wrap(res[0][2].yaw - R(170.0))
    arr = next((t for t, _, r in res if adiff(r.yaw, R(-170.0)) < 1e-9), None)
    minabs = min(abs(r.yaw) for _, _, r in res)
    check('W3 170 -> -170 deg: the slew takes the short way (+20 through 180)',
          first > 0.0 and arr is not None and arr <= 0.26 and minabs > R(160.0),
          f'first step {D(first):+.2f} deg, arrived after {arr:.2f} s, '
          f'min |yaw| {D(minabs):.1f} deg')


def checks_slew():
    print('\nL   slew limit (90 deg/s)')
    fn = lambda t: (5.0, 0.0) if t < 0.5 else (-5.0, 0.0)   # noqa: E731
    res = run(servo(), 3.5, stream=Stream(fn))
    tj = next(t for t, _, r in res if r.psi_des > R(90.0))
    arr = next((t for t, _, r in res if t >= tj and adiff(r.yaw, math.pi) < R(0.5)), None)
    w, ex = slew_worst(res)
    check('L1 180 deg target jump: every tick <= 90 deg/s * dt', ex <= EPS,
          f'max {D(w):.6f} deg/tick, limit {90 * DT:.6f}')
    check('L2 180 deg target jump: arrives within 2.2 s', arr is not None and arr - tj <= 2.2,
          f'{arr - tj:.3f} s')
    during = all(r.slew_binding for t, _, r in res if tj <= t < tj + 1.9)
    after = not any(r.slew_binding for t, _, r in res if t > tj + 2.1)
    check('L3 slew_binding set during the turn and clear after it', during and after,
          f'during {during}, clear after {after}')
    cases = [(0.02, 1.8), (0.05, 4.5), (0.1, 9.0), (0.5, 9.0), (10.0, 9.0), (0.0, 0.0),
             (-0.02, 0.0), (NAN, 0.0), (INF, 0.0), (-INF, 0.0)]
    ok = True
    det = []
    for dt, exp in cases:
        r = tick(servo(), dt=dt, target=(0.0, -5.0, 0.1, 'lidar_init'))   # FRD +90 from 0
        ch = D(adiff(r.yaw, 0.0))
        ok &= abs(ch - exp) <= 1e-9 and in_range(r.yaw)
        det.append(f'{dt:g}->{ch:.1f}')
    check('L4 dt clamped to [0, 0.1] s: a 0.5 s stall moves <= 9 deg; dt <= 0, NaN, inf move 0',
          ok, ' '.join(det) + ' (s -> deg)')


def checks_ff():
    print('\nF   feed-forward (fix 2)')
    runs = []
    res = run(servo(ff_enable=True), 6.0, stream=Stream(orbit(0.0, -30.0)))  # FRD +30 deg/s
    runs.append(res)
    lo, hi, mean = settled(res, 3.0)
    check('F1 30 deg/s orbit delivered at 15 Hz: yawspeed settles to 30 +- 5 deg/s',
          25.0 <= D(lo) and D(hi) <= 35.0,
          f'min {D(lo):.2f} max {D(hi):.2f} mean {D(mean):.2f} deg/s over 3-6 s')
    res = run(servo(ff_enable=True), 6.0, stream=Stream(orbit(0.0, 30.0)))   # FLU CCW
    runs.append(res)
    lo, hi, mean = settled(res, 3.0)
    check('F2 sign: a target circling to the LEFT (FLU CCW) -> NEGATIVE FRD yawspeed, -30 +- 5',
          -35.0 <= D(lo) and D(hi) <= -25.0,
          f'min {D(lo):.2f} max {D(hi):.2f} mean {D(mean):.2f} deg/s')

    for sgn, label in ((1, 'F3'), (-1, 'F4')):
        res = run(servo(ff_enable=True), 4.0, stream=Stream(orbit(0.0, -100.0 * sgn)))
        runs.append(res)
        mx = max(abs(r.yawspeed) for _, _, r in res)
        lo, hi, _ = settled(res, 1.5)
        t_cap = next((t for t, _, r in res if abs(abs(r.yawspeed) - R(60.0)) < 1e-9), None)
        ok = (mx <= R(60.0) + EPS and min(sgn * lo, sgn * hi) >= R(60.0) - 1e-6)
        check(f'{label} 100 deg/s sweep at 15 Hz, abrupt start ({"+" if sgn > 0 else "-"}): '
              'yawspeed capped at 60 deg/s',
              ok, f'settled {D(lo):+.4f}..{D(hi):+.4f} deg/s, max |.| {D(mx):.6f}, '
                  f'at the cap after {t_cap:.2f} s')

    res = run(servo(ff_enable=True), 3.0, stream=Stream(
        lambda t: (5.0, 0.0) if t < 1.0 else (5.0, 5.0)))     # 5 m sideways at 5 m: 45 deg
    runs.append(res)
    mx = max(abs(r.yawspeed) for _, _, r in res)
    w, ex = slew_worst(res)
    check('F5 one-off 5 m target jump from rest: no feed-forward at all, the heading slews',
          mx == 0.0 and ex <= EPS,
          f'max |yawspeed| {D(mx):.3g} deg/s; heading max {D(w):.3f} deg/tick')

    for J, label, name in ((60.0, 'with', 'F6'), (-60.0, 'against', 'F7')):
        # FLU CW orbit (FRD +30 deg/s); an FRD jump of +J deg is a FLU jump of -J. 60 deg at
        # 5 m is a 5 m chord.
        res = run(servo(ff_enable=True), 6.0, stream=Stream(orbit(0.0, -30.0,
                                                                  jumps=[(3.0, -J)])))
        runs.append(res)
        sg = 1.0 if J > 0 else -1.0
        pre = max(sg * r.ff for t, _, r in res if 2.0 <= t < 3.0)
        post = max(sg * r.ff for t, _, r in res if t >= 3.0)
        allowed = max(0.0, pre) + R(0.5)
        lo, hi, _ = settled(res, 5.0)
        check(f'{name} 5 m (60 deg) jump {label} a 30 deg/s orbit: ff does not chase it, '
              're-settles',
              post <= allowed and ff_slew_worst(res) <= 90.0 + 1e-6
              and 25.0 <= D(lo) and D(hi) <= 35.0,
              f'ff toward the jump: before max {D(pre):+.2f}, after max {D(post):+.2f} deg/s '
              f'(allowed {D(allowed):+.2f}); 2 s later {D(lo):.1f}..{D(hi):.1f} deg/s')

    flick = lambda t: (5.0, 3.5) if 1.0 <= t < 1.0 + 1.0 / 15.0 - 1e-9 else (5.0, 0.0)  # noqa
    res = run(servo(ff_enable=True), 3.0, stream=Stream(flick))
    runs.append(res)
    mx = max(abs(r.yawspeed) for _, _, r in res)
    check('F8 one-message flicker A -> B -> A (35 deg and back): no feed-forward',
          mx == 0.0, f'max |yawspeed| {D(mx):.3g} deg/s')

    worst = max(ff_slew_worst(r) for r in runs)
    check('F9 ff slew <= 90 deg/s^2 on every tick of F1-F8', worst <= 90.0 + 1e-6,
          f'max {worst:.4f} deg/s^2')

    res = run(servo(ff_enable=False), 4.0, stream=Stream(orbit(0.0, -30.0)))
    act = [r for _, _, r in res if r.active]
    ok = bool(act) and all(bits(r.yawspeed) == bits(0.0) for r in act)
    check('F10 ff_enable off: yawspeed exactly 0.0 while the servo is active', ok,
          f'{len(act)} active ticks, distinct yawspeeds {sorted({r.yawspeed for r in act})}')

    rnd = random.Random(7)
    for label, dt, dt_fn in (('100 Hz', 0.01, None), ('25 Hz', 0.04, None),
                             ('50 Hz jittered 12-28 ms', DT,
                              lambda k: rnd.uniform(0.012, 0.028))):
        res = run(servo(ff_enable=True), 6.0, dt=dt, dt_fn=dt_fn,
                  stream=Stream(orbit(0.0, -30.0)))
        lo, hi, mean = settled(res, 3.0)
        check(f'F11 30 deg/s orbit, ticks at {label}: 30 +- 5 deg/s, ff slew <= 90 deg/s^2',
              25.0 <= D(lo) and D(hi) <= 35.0 and ff_slew_worst(res) <= 90.0 + 1e-6,
              f'min {D(lo):.2f} max {D(hi):.2f} mean {D(mean):.2f} deg/s')


def checks_close():
    print('\nC   close range (< 2.5 m)')
    # 1.5 m circle at 3 m/s: a 115 deg/s bearing rate.
    res = run(servo(ff_enable=True), 2.0, stream=Stream(orbit(0.0, D(3.0 / 1.5), radius=1.5)))
    ok = all(r.source == ys.SOURCE_CLOSE_HOLD and r.yaw == 0.0 and r.yawspeed == 0.0
             and r.active and r.range_m < 2.5 for _, _, r in res)
    check('C1 target at 1.5 m moving sideways at 3 m/s (115 deg/s): heading held, yawspeed 0, '
          'source 2', ok,
          f'{len(res)} ticks, max |yaw| {max(abs(D(r.yaw)) for _, _, r in res):.3g} deg, '
          f'max |yawspeed| {max(abs(D(r.yawspeed)) for _, _, r in res):.3g} deg/s')

    rad = lambda t: 4.0 if (t < 3.0 or t >= 5.0) else 1.5   # noqa: E731
    res = run(servo(ff_enable=True), 8.0, stream=Stream(orbit(0.0, -30.0, radius=rad)))
    close = [(t, r) for t, _, r in res if r.source == ys.SOURCE_CLOSE_HOLD]
    t_in, t_out = close[0][0], close[-1][0]
    before = [r for t, _, r in res if t < t_in][-1]
    held = all(r.psi_des == before.psi_des for _, r in close)
    zero = all(r.yawspeed == 0.0 and r.ff == 0.0 for _, r in close)
    conv = adiff(close[-1][1].yaw, before.psi_des) < 1e-9
    w, ex = slew_worst(res)
    check('C2 4 m -> 1.5 m during a 30 deg/s orbit: psi_des held at its last value, ff 0 at once',
          before.source == ys.SOURCE_SERVO and before.ff > R(25.0) and held and zero and conv
          and ex <= EPS,
          f'ff {D(before.ff):.1f} deg/s -> 0 on entry; held {D(before.psi_des):+.2f} deg over '
          f'{len(close)} ticks')
    after = [(t, d, r) for t, d, r in res if t > t_out]
    first = after[0][2]
    lo, hi, _ = settled(res, 7.0)
    check('C3 back out to 4 m: ff restarts from 0, slewed, re-settles at 30 +- 5 deg/s',
          first.source == ys.SOURCE_SERVO and abs(first.ff) <= R(90.0) * DT + EPS
          and ff_slew_worst(after) <= 90.0 + 1e-6 and 25.0 <= D(lo) and D(hi) <= 35.0,
          f'first ff {D(first.ff):.2f} deg/s, settled {D(lo):.1f}..{D(hi):.1f} deg/s')


def checks_fresh_frames():
    print('\nT   freshness and frames')
    tgt = lambda age, fr='lidar_init': (0.0, -5.0, age, fr)    # noqa: E731  FRD +90
    for age, src, label in ((0.49, 1, 'fresh'), (0.50, 1, 'fresh (= the timeout)'),
                            (0.51, 0, 'stale'), (-0.19, 1, 'fresh (future, within 0.2 s)'),
                            (-0.20, 1, 'fresh (= the future limit)'),
                            (-0.21, 0, 'ignored (too far in the future)'),
                            (NAN, 0, 'ignored'), (INF, 0, 'ignored')):
        r = tick(servo(), target=tgt(age))
        check(f'T1 target age {age:+.2f} s -> {label}', r.source == src,
              f'source {r.source}, psi_des {D(r.psi_des):+.1f} deg')
    r = tick(servo(), target=None)
    check('T2 target None (never received) -> fall-back, age NaN',
          r.source == 0 and math.isnan(r.target_age_s) and math.isnan(r.range_m),
          f'source {r.source}')
    ok = True
    for fr in ('lidar_init', 'world'):
        r = tick(servo(), target=tgt(0.1, fr))
        ok &= r.source == 1 and not r.bad_frame
    check('T3 frames lidar_init and world accepted', ok)
    bad = ('map', 'base_link', '', None, 'odom', '/lidar_init', 'LIDAR_INIT', 'lidar_init ', 5)
    ok = True
    for fr in bad:
        r = tick(servo(), target=tgt(0.1, fr))
        ok &= r.source == 0 and r.bad_frame
    check('T4 any other frame (map, base_link, empty, None, ...) ignored with bad_frame set', ok,
          f'{len(bad)} frames')
    r = tick(servo(), target=tgt(0.8, 'map'))
    check('T5 a STALE target in a bad frame does not raise bad_frame', not r.bad_frame)


def clipped_of(r, dt):
    """Did the slew clamp clip on this tick (the F-a condition)? slew_binding says so when
    the tick had time; with dt_c 0 any error counts, and then psi_out stayed where it was."""
    return r.slew_binding or (dt_sane(dt) == 0.0 and r.psi_out != r.psi_des)


def checks_fallback():
    print('\nB   fall-back to SUPER without a jump (round one: stale_hold_s 0; round two: the '
          'default 0.5 s hold first)')
    tgt = (5.0 * math.cos(R(-60.0)), 5.0 * math.sin(R(-60.0)))     # FLU -60 = FRD +60
    ydot = R(3.0)
    for hold in (0.0, 0.5):
        tag = 'stale_hold_s 0 (round one)' if hold == 0.0 else 'default stale hold 0.5 s'
        res = run(servo(stale_hold_s=hold), 3.5, stream=Stream(lambda t: tgt, t_stop=1.0),
                  psi_super=0.0, yawdot_super=ydot)
        fb = [(t, r) for t, _, r in res if t > 1.0 and r.source == 0]
        t_fb = fb[0][0]
        prev = [r for t, _, r in res if t < t_fb][-1]
        last1 = [(t, r) for t, _, r in res if r.source == 1][-1]
        hold_ticks = [(t, d, r) for t, d, r in res if last1[0] < t < t_fb]
        w, ex = slew_worst(res)
        arr = next((t for t, r in fb if r.yaw == 0.0), None)
        if hold == 0.0:
            check(f'B1 [{tag}] servo aiming 60 deg off SUPER, target goes stale: no jump at the '
                  'switch',
                  prev.source == 1 and adiff(prev.yaw, R(60.0)) < 1e-9 and not hold_ticks
                  and adiff(fb[0][1].yaw, prev.yaw) <= R(90.0) * DT + EPS,
                  f'last servo yaw {D(prev.yaw):+.2f} deg, first fall-back step '
                  f'{D(adiff(fb[0][1].yaw, prev.yaw)):.3f} deg')
        else:
            held = sum(d for _, d, _ in hold_ticks)
            ok = (prev.source == 3 and hold_ticks
                  and all(r.source == 3 and r.yaw == last1[1].yaw and r.psi_des == last1[1].psi_des
                          and bits(r.yawspeed) == bits(0.0) for _, _, r in hold_ticks)
                  and abs(held - hold) <= DT + EPS
                  and adiff(fb[0][1].yaw, prev.yaw) <= R(90.0) * DT + EPS)
            check(f'B1 [{tag}] target goes stale: holds the 60 deg aim (source 3, yawspeed 0) '
                  f'for {hold} s, then no jump at the switch', ok,
                  f'{len(hold_ticks)} hold ticks = {held:.3f} s at yaw {D(prev.yaw):+.2f} deg; '
                  f'first fall-back step {D(adiff(fb[0][1].yaw, prev.yaw)):.3f} deg')
        check(f'B2 [{tag}] ... walks to psi_super at <= 90 deg/s, arriving in 60/90 s',
              ex <= EPS and arr is not None and arr - t_fb <= 60.0 / 90.0 + 0.03,
              f'max {D(w):.4f} deg/tick; arrived {arr - t_fb:.3f} s after the fall-back began')
        # Changed by F-a (round two): SUPER's rate only inside the band AND on a tick the
        # clamp did not clip. Round one also sent it on the last clipped tick (0.6 deg left).
        n_in = n_out = n_clip_in = 0
        ok = True
        for (_, _, a), (t, d, r) in zip(res, res[1:]):
            if not (t > 1.0 and r.source == 0):
                continue
            inside = abs(wrap(0.0 - r.yaw)) < R(2.0)
            clip = clipped_of(r, d)
            if inside and not clip:
                n_in += 1
                ok &= r.yawspeed == ydot
            else:
                n_out += 1
                n_clip_in += inside and clip
                ok &= bits(r.yawspeed) == bits(0.0)
        check(f'B3 [{tag}] ... yawspeed = SUPER\'s rate inside the 2 deg band on ticks the clamp '
              'did not clip, else 0 (F-a)', ok and n_in and n_out and n_clip_in == 1,
              f'{n_out} ticks 0.0 (of which {n_clip_in} inside the band but clipped: round one '
              f'sent {D(ydot):.0f} deg/s there), {n_in} = yawdot_super')

    # ff on: a 30 deg/s orbit, then the servo loses its input abruptly (lpos xy_valid drops
    # at t = 3 s) while the ff is at 30 deg/s, with psi_super right where the servo was
    # aiming, so the band rule and the decaying ff compete from the first fall-back tick.
    # (A target stream that merely stops is a weaker test: the bearing then stays constant
    # while the target is still fresh, and the ff averages down to ~0 by itself before the
    # 0.5 s timeout.)
    ydot = R(-10.0)
    psi_at_loss = R(30.0 * math.floor(3.0 * 15.0 - 1e-9) / 15.0)   # newest bearing before 3 s
    for hold in (0.0, 0.5):
        tag = 'stale_hold_s 0 (round one)' if hold == 0.0 else 'default stale hold 0.5 s'
        res = run(servo(ff_enable=True, stale_hold_s=hold), 5.0, stream=Stream(orbit(0.0, -30.0)),
                  psi_super=psi_at_loss, yawdot_super=ydot, xy_valid=lambda t: t < 3.0 - 1e-9)
        lost = [(t, d, r) for t, d, r in res if r.source in (0, 3)]
        t_loss = lost[0][0]
        ff0 = [r for t, _, r in res if t < t_loss][-1].ff
        t_zero = next(t for t, _, r in lost if r.ff == 0.0)
        ok = True
        suppressed = n_hold = n_clip = 0
        for t, d, r in lost:
            inside = abs(wrap(psi_at_loss - r.yaw)) < R(2.0)
            if r.source == 3:
                n_hold += 1
                ok &= r.yawspeed == r.ff          # the decaying ff, never SUPER's rate
                suppressed += inside
            elif r.ff != 0.0:
                ok &= r.yawspeed == r.ff
                suppressed += inside
            else:
                clip = clipped_of(r, d)
                n_clip += inside and clip
                ok &= r.yawspeed == (ydot if (inside and not clip) else 0.0)
            ok &= abs(r.yawspeed) <= max(abs(ff0), abs(ydot)) + EPS
        check(f'B4 [{tag}] ff on: after the loss the ff decays at <= 90 deg/s^2 and reaches 0 in '
              'ff/90 s' + (' (through the hold)' if hold else ''),
              abs(ff0) > R(25.0) and ff_slew_worst(res) <= 90.0 + 1e-6
              and t_zero - t_loss <= abs(D(ff0)) / 90.0 + DT + EPS and (n_hold > 0) == (hold > 0),
              f'ff {D(ff0):+.2f} deg/s at the loss, 0 after {t_zero - t_loss:.3f} s; '
              f'{n_hold} hold ticks')
        check(f'B5 [{tag}] ff on: yawspeed is the decaying ff OR SUPER\'s band rate, never their sum'
              + ('; never SUPER\'s rate in the hold' if hold else ''),
              ok and suppressed > 0 and n_clip == 0,
              f'{suppressed} ticks where SUPER\'s rate was held back while the ff decayed'
              + (f' ({n_hold} of them in the hold)' if hold else ''))


def checks_nan():
    print('\nN   NaN / inf in every input in turn')
    fields = ('dt', 'psi_super', 'yawdot_super', 'vehicle_heading', 'vehicle_x', 'vehicle_y',
              'target_x', 'target_y', 'target_age', 'vehicle_xy_valid')
    keeps_servo = ('dt', 'psi_super', 'yawdot_super')
    for field, hold in [(f, h) for h in (0.0, 0.5) for f in fields
                        if h == 0.0 or f not in keeps_servo]:
        bads = (False, NAN) if field == 'vehicle_xy_valid' else (NAN, INF, -INF)
        ok = True
        why = ''
        for bad in bads:
            s = servo(ff_enable=True, stale_hold_s=hold)
            st = Stream(orbit(0.0, -30.0))
            t = 0.0
            prev = None
            for k in range(95):
                t += DT
                a = dict(dt=DT, psi_super=R(10.0), yawdot_super=R(5.0), heading=0.0, x=0.0,
                         y=0.0, xy_valid=True)
                tg = list(st.at(t))
                inj = 75 <= k < 85
                if inj:
                    if field == 'dt':
                        a['dt'] = bad
                    elif field == 'vehicle_heading':
                        a['heading'] = bad
                    elif field == 'vehicle_x':
                        a['x'] = bad
                    elif field == 'vehicle_y':
                        a['y'] = bad
                    elif field == 'vehicle_xy_valid':
                        a['xy_valid'] = bad
                    elif field.startswith('target_'):
                        tg[{'target_x': 0, 'target_y': 1, 'target_age': 2}[field]] = bad
                    else:
                        a[field] = bad
                try:
                    r = tick(s, target=tuple(tg), **a)
                except Exception as e:  # the check: step() must never raise
                    ok, why = False, f'raised {e!r}'
                    break
                if not (in_range(r.yaw) and math.isfinite(r.yawspeed)):
                    ok, why = False, f'non-finite output {r.yaw} {r.yawspeed}'
                    break
                if prev is not None:
                    lim = R(90.0) * min(dt_sane(a['dt']), ys.DT_MAX)
                    if adiff(r.yaw, prev.yaw) > lim + EPS:
                        ok, why = False, 'slew exceeded'
                        break
                # 10 injected ticks = 0.2 s: inside the default 0.5 s stale hold.
                want = 1 if (not inj or field in keeps_servo) else (3 if hold else 0)
                if r.source != want:
                    ok, why = False, f'source {r.source} at k={k}, wanted {want}'
                    break
                if inj and field == 'dt' and r.yaw != prev.yaw:
                    ok, why = False, 'heading moved on a non-finite dt'
                    break
                if want == 3 and not (r.psi_des == prev.psi_des and r.yawspeed == r.ff
                                      and abs(r.ff) <= abs(prev.ff)):
                    ok, why = False, f'hold at k={k}: aim not held or yawspeed not the decaying ff'
                    break
                prev = r
        exp = 'servo unaffected' if field in keeps_servo else 'fall-back (source 0)'
        if field == 'dt':
            exp = 'tick frozen'
        if hold:
            exp = 'stale hold (source 3: last aim, decaying ff)'
        tag = '' if field in keeps_servo else (' [stale_hold_s 0, round one]' if hold == 0.0
                                               else ' [default stale hold 0.5 s]')
        check(f'N1 {field} = {"/".join(str(b) for b in bads)}{tag}: finite output, {exp}, '
              'recovers', ok, why)

    s = servo()
    for _ in range(5):
        tick(s)
    ok = True
    for _ in range(10):
        r = tick(s, psi_super=NAN, yawdot_super=NAN)
        ok &= r.source == 0 and in_range(r.yaw) and r.yaw == 0.0 and r.yawspeed == 0.0
    check('N2 fall-back with psi_super and yawdot_super NaN: heading held, yawspeed 0', ok)
    r = tick(servo(), heading=NAN, psi_super=R(40.0))
    check('N3 first engaged tick, heading NaN: psi_out starts from psi_super', r.yaw == R(40.0),
          f'yaw {D(r.yaw):+.2f} deg')
    p = NAN
    r = tick(servo(), heading=NAN, psi_super=p)
    check('N4 first engaged tick, heading AND psi_super NaN: passed through (documented)',
          r.passthrough and r.yaw is p)

    junk_targets = ['junk', (1.0, 2.0), (1, 2, 3, 4, 5), ('a', 'b', 'c', 'd'), 42, {'x': 1},
                    (1.0, 2.0, 0.1, 5), (None, None, None, None), (1e308, -1e308, 0.1, 'world'),
                    ('1.0', '2.0', '0.1', 'world'), (NAN, NAN, NAN, NAN)]
    junk_kw = [dict(state=None), dict(state='track'), dict(state=42), dict(state=b'TRACK'),
               dict(engaged=NAN), dict(engaged='yes'), dict(engaged=None), dict(engaged=[1]),
               dict(xy_valid=NAN), dict(xy_valid='x'), dict(xy_valid=None), dict(dt=None),
               dict(dt='0.02'), dict(psi_super=None), dict(yawdot_super='fast'),
               dict(heading=[0.0]), dict(x=1e308, y=-1e308)]
    raised = []
    bad_out = 0
    n = 0
    s = servo(ff_enable=True)
    for tg in junk_targets:
        for kw in [{}] + junk_kw:
            n += 1
            a = dict(psi_super=float(n), yawdot_super=0.5)
            a.update(kw)
            try:
                r = tick(s, target=tg, **a)
            except Exception as e:  # the check: step() must never raise
                raised.append(f'{tg!r} {kw!r}: {e!r}')
                continue
            if r.passthrough:
                bad_out += not (r.yaw is a['psi_super'] and r.yawspeed is a['yawdot_super'])
            elif not (in_range(r.yaw) and isinstance(r.yawspeed, float)
                      and math.isfinite(r.yawspeed)):
                bad_out += 1
    check('N5 malformed inputs (junk targets, states, flags, strings, 1e308): no exception, '
          'finite output', not raised and bad_out == 0,
          f'{n} combinations, {len(raised)} raised, {bad_out} bad outputs'
          + (f'; first: {raised[0]}' if raised else ''))
    check('N6 engaged = NaN is NOT engaged (bool(nan) would be True)',
          tick(servo(), engaged=NAN).passthrough)


def checks_idle():
    print('\nI   IDLE / not engaged: identity, then re-initialise')
    s = servo(ff_enable=True)
    tick(s, target=(0.0, -5.0, 0.1, 'lidar_init'))
    rnd = random.Random(5)
    ok = True
    for _ in range(500):
        if rnd.random() < 0.5:
            st, eng = ys.IDLE, rnd.random() < 0.5
        else:
            st, eng = rnd.choice([ys.TRACK, ys.HOLD, ys.IDLE]), False
        p, yd = rnd.uniform(-7, 7), rnd.uniform(-3, 3)
        r = tick(s, state=st, engaged=eng, psi_super=p, yawdot_super=yd,
                 heading=rnd.uniform(-3, 3), target=(0.0, -5.0, 0.1, 'lidar_init'))
        ok &= (r.passthrough and r.yaw is p and r.yawspeed is yd and s._psi_out is None
               and s._ff == 0.0 and not s._close and s._hold_t == 0.0
               and s._prev_source == ys.SOURCE_SUPER)
    check('I1 switch ON but IDLE or not engaged: identity (same float objects) and state reset',
          ok, '500 ticks')
    s = servo()
    res = run(s, 1.5, stream=Stream(lambda t: (0.0, -5.0)))     # FRD +90
    y1 = res[-1][2].yaw
    for _ in range(50):
        tick(s, state=ys.IDLE, engaged=False, psi_super=R(-120.0), heading=R(-120.0))
    r = tick(s, psi_super=R(-100.0), heading=R(-120.0), target=None)
    check('I2 re-engage: psi_out starts from the vehicle heading (-120), not the stale +90',
          adiff(r.yaw, R(-120.0)) <= R(90.0) * DT + EPS and adiff(r.yaw, y1) > R(150.0),
          f'before IDLE {D(y1):+.1f} deg; first tick after {D(r.yaw):+.2f} deg')


def checks_hold():
    print('\nH   aim_in_hold (plan SS 2.4 / SS 6)')
    tgt = (0.0, -5.0, 0.1, 'lidar_init')    # FRD +90
    s = servo(aim_in_hold=False)
    hold1 = [tick(s, state=ys.HOLD, target=tgt) for _ in range(25)]
    trk = tick(s, state=ys.TRACK, target=tgt)
    hold2 = [tick(s, state=ys.HOLD, target=tgt) for _ in range(10)]
    idle = tick(s, state=ys.IDLE, engaged=False, target=tgt)
    hold3 = tick(s, state=ys.HOLD, target=tgt)
    ok = (all(r.source == 0 and r.yaw == 0.0 and not r.passthrough for r in hold1)
          and trk.source == 1 and all(r.source == 1 for r in hold2) and idle.passthrough
          and hold3.source == 0)
    check('H1 aim_in_hold False: no servo in HOLD before the first TRACK, servo in HOLD after, '
          'reset by IDLE', ok,
          f'HOLD before TRACK: sources {sorted({r.source for r in hold1})}, yaw '
          f'{D(hold1[-1].yaw):+.1f}; TRACK {trk.source}; HOLD after {hold2[-1].source}; '
          f'HOLD after IDLE {hold3.source}')
    r = tick(servo(), state=ys.HOLD, target=tgt)
    check('H2 aim_in_hold True (default): the servo aims in HOLD at once',
          r.source == 1 and r.yaw > 0.0, f'source {r.source}, yaw {D(r.yaw):+.2f} deg')


def checks_offset():
    print('\nO   camera offset (+ = camera looks LEFT of the nose)')
    r = tick(servo(camera_offset_deg=10.0), target=(5.0, 0.0, 0.1, 'lidar_init'))
    check('O1 camera 10 deg LEFT, person straight ahead: body aims 10 deg RIGHT (FRD +10)',
          abs(r.psi_des - R(10.0)) < 1e-12, f'psi_des {D(r.psi_des):+.4f} deg')
    r = tick(servo(camera_offset_deg=-10.0), target=(5.0, 0.0, 0.1, 'lidar_init'))
    check('O2 camera 10 deg RIGHT: body aims 10 deg LEFT (FRD -10)',
          abs(r.psi_des - R(-10.0)) < 1e-12, f'psi_des {D(r.psi_des):+.4f} deg')
    worst = 0.0
    for delta in (10.0, -10.0, 25.0, -170.0, 180.0):
        for b in (-150.0, -60.0, 0.0, 45.0, 170.0, 179.0):
            ax, ay = 3.0, -2.0
            tx, ty = ax + 6.0 * math.cos(R(b)), ay + 6.0 * math.sin(R(b))
            r = tick(servo(camera_offset_deg=delta), x=ax, y=-ay,
                     target=(tx, ty, 0.1, 'lidar_init'))
            cam_flu = wrap(-r.psi_des + R(delta))     # camera axis, FLU
            worst = max(worst, adiff(cam_flu, R(b)))
    check('O3 camera axis (FLU) = -yaw_frd + offset lands on the person: 5 offsets x 6 bearings',
          worst < 1e-9, f'worst {D(worst):.1e} deg')
    rnd = random.Random(3)
    ok = True
    n = 0
    while n < 1000:
        vx, vy, tx, ty = (rnd.uniform(-20, 20) for _ in range(4))
        if math.hypot(tx - vx, -ty - vy) < 3.0:
            continue
        r = tick(servo(), x=vx, y=vy, target=(tx, ty, 0.1, 'lidar_init'))
        exp = math.atan2(-ty - vy, tx - vx)
        ok &= bits(r.psi_des) == bits(math.pi if exp == -math.pi else exp)
        n += 1
    check('O4 offset 0 is an exact no-op: psi_des bit-identical to the raw bearing', ok,
          f'{n} cases')


# ---- round two: F-a, hysteresis, stale hold, the round-one reference ---------------------

class FnStream:
    """A target given per tick time by fn(t) -> (x_flu, y_flu, age, frame) or anything."""

    def __init__(self, fn):
        self.fn = fn

    def at(self, t):
        return self.fn(t)


def at_range(r_m, bearing_deg=0.0, age=0.1, frame='lidar_init'):
    """A target r_m from the origin at FRD bearing `bearing_deg` (FLU angle -bearing)."""
    th = R(-bearing_deg)
    return (r_m * math.cos(th), r_m * math.sin(th), age, frame)


def ramp_fallback(rate_dps, ydot_dps, T=2.0, ff=False, **kw):
    """Fall-back from the first tick (no target at all): psi_super ramps at rate_dps from 0,
    the vehicle's heading, so psi_out starts ON it; yawdot_super = ydot_dps.
    -> [(t, dt, result, psi_super)]"""
    s = servo(ff_enable=ff, **kw)
    out = []
    for k in range(int(round(T / DT))):
        psi = wrap(R(rate_dps) * DT * k)
        out.append((DT * (k + 1), DT, tick(s, psi_super=psi, yawdot_super=R(ydot_dps)), psi))
    return out


def transitions(res):
    """(entries 1->2, exits 2->1, all source changes) over consecutive results."""
    ent = sum(1 for a, b in zip(res, res[1:]) if a.source == 1 and b.source == 2)
    ext = sum(1 for a, b in zip(res, res[1:]) if a.source == 2 and b.source == 1)
    chg = sum(1 for a, b in zip(res, res[1:]) if a.source != b.source)
    return ent, ext, chg


def checks_fallback_rate():
    print('\nR   fall-back yawspeed (F-a): SUPER\'s rate only when the clamp did not clip, in the '
          'band, <= max_rate')
    for rate in (100.0, 150.0):
        for ff in (False, True):
            res = ramp_fallback(rate, rate, ff=ff)
            bind = [(r, p) for _, _, r, p in res if r.slew_binding]
            in_band = sum(1 for r, p in bind if abs(wrap(p - r.yaw)) < R(2.0))
            nz = [r.yawspeed for r, _ in bind if bits(r.yawspeed) != bits(0.0)]
            mx = max(abs(r.yawspeed) for _, _, r, _ in res)
            ok = (len(bind) >= len(res) - 1 and not nz
                  and mx <= R(90.0) + EPS and all(r.source == 0 for _, _, r, _ in res))
            check(f'R1 SUPER\'s yaw ramping at {rate:.0f} deg/s with yaw_dot {rate:.0f} deg/s, target '
                  f'unusable (the replay\'s 200650 case, band_repro), ff {"on" if ff else "off"}: '
                  'yawspeed 0 on every binding tick, never above 90 deg/s',
                  ok, f'{len(bind)} of {len(res)} ticks binding, {len(bind) - len(nz)} of them 0.0'
                      + (f', {len(nz)} NOT (up to {D(max(abs(x) for x in nz)):.1f} deg/s)' if nz else '')
                      + f' ({in_band} inside the 2 deg band: round one sent {rate:.0f} deg/s there); '
                      f'max |yawspeed| {D(mx):.3f} deg/s (tick 0, not binding, clamped from '
                      f'{rate:.0f})')
    for sgn in (1.0, -1.0):
        res = ramp_fallback(60.0 * sgn, 150.0 * sgn)
        tail = [r for _, _, r, _ in res]
        ok = all(not r.slew_binding and r.yawspeed == sgn * R(90.0) for r in tail)
        label = (f'R2 SUPER ramping at {60 * sgn:+.0f} deg/s but sending yaw_dot {150 * sgn:+.0f} '
                 f'deg/s, heading on it, clamp not clipping: yawspeed exactly {90 * sgn:+.0f} deg/s '
                 '(clamped to max_rate)')
        check(label, ok,
              f'{len(tail)} ticks; distinct yawspeeds {sorted({round(D(r.yawspeed), 9) for r in tail})}')
    res = ramp_fallback(40.0, 40.0)
    ok = all(r.yawspeed == R(40.0) and not r.slew_binding for _, _, r, _ in res)
    check('R3 SUPER ramping at 40 deg/s with yaw_dot 40, heading on it: SUPER\'s rate passes '
          'unchanged (the clamp is a no-op below 90)', ok,
          f'{len(res)} ticks, distinct {sorted({round(D(r.yawspeed), 9) for _, _, r, _ in res})}')
    det = []
    ok = True
    for yd in (INF, -INF, NAN):
        s = servo()
        rs = [tick(s, psi_super=0.0, yawdot_super=yd) for _ in range(20)]
        ok &= all(bits(r.yawspeed) == bits(0.0) and r.source == 0 for r in rs)
        det.append(f'{yd}->{D(rs[-1].yawspeed):g}')
    check('R4 yaw_dot inf / -inf / NaN in band: 0.0, not +-90 (a non-finite rate is not clamped)',
          ok, ' '.join(det))
    s = servo()
    for _ in range(5):
        tick(s, psi_super=0.0, yawdot_super=R(20.0))
    r_same = tick(s, dt=0.0, psi_super=0.0, yawdot_super=R(20.0))
    r_move = tick(s, dt=0.0, psi_super=R(1.0), yawdot_super=R(20.0))
    r_nan = tick(s, dt=NAN, psi_super=R(1.5), yawdot_super=R(20.0))
    r_next = tick(s, psi_super=R(1.5), yawdot_super=R(20.0))
    check('R5 a dt 0 / NaN tick in fall-back: SUPER\'s rate if its heading did not move, 0 if it did '
          '(the clamp clipped: nothing may move); the next tick with time passes it again',
          r_same.yawspeed == R(20.0) and bits(r_move.yawspeed) == bits(0.0)
          and bits(r_nan.yawspeed) == bits(0.0) and r_move.yaw == 0.0
          and r_next.yawspeed == R(20.0) and r_next.yaw == R(1.5),
          f'unmoved {D(r_same.yawspeed):.1f}, moved {D(r_move.yawspeed):.1f}, NaN dt '
          f'{D(r_nan.yawspeed):.1f}, next {D(r_next.yawspeed):.1f} deg/s')
    # ff on, immediate fall-back (stale_hold_s 0): the ff decays first, then the F-a rule,
    # while SUPER ramps away at 100 deg/s from where the servo was aiming.
    s = servo(ff_enable=True, stale_hold_s=0.0)
    st = Stream(orbit(0.0, -30.0))
    t = 0.0
    for _ in range(150):
        t += DT
        tick(s, target=st.at(t))
    ff0 = s._ff
    psi0 = s._psi_out
    ok, n_dec, n_zero_bind, mx = True, 0, 0, 0.0
    for k in range(100):
        t += DT
        r = tick(s, target=st.at(t), xy_valid=False, psi_super=wrap(psi0 + R(100.0) * DT * k),
                 yawdot_super=R(100.0))
        mx = max(mx, abs(r.yawspeed))
        if r.ff != 0.0:
            n_dec += 1
            ok &= r.yawspeed == r.ff
        else:
            n_zero_bind += r.slew_binding
            ok &= (not r.slew_binding) or bits(r.yawspeed) == bits(0.0)
    check('R6 ff on, immediate fall-back while SUPER ramps away at 100 deg/s: the decaying ff, then '
          '0 while the clamp binds; never the sum, never above max(ff cap, max_rate)',
          ok and n_dec > 5 and n_zero_bind > 50 and mx <= max(R(60.0), R(90.0)) + EPS
          and abs(ff0) > R(25.0),
          f'ff {D(ff0):+.1f} deg/s at the loss, {n_dec} decaying ticks, {n_zero_bind} binding ticks '
          f'at 0.0; max |yawspeed| {D(mx):.2f} deg/s')


def checks_hysteresis():
    print('\nX   close-range hysteresis (in at < 2.5 m, out at >= 2.8 m)')
    got = {}
    for r_m in (2.51, 2.5, 2.4999999, 2.49, 1.0):
        got[r_m] = tick(servo(), target=at_range(r_m)).source
    check('X1 entry below min_range_m exactly: 2.51 and 2.5 m -> source 1; 2.4999999, 2.49, 1.0 m '
          '-> source 2',
          [got[k] for k in (2.51, 2.5, 2.4999999, 2.49, 1.0)] == [1, 1, 2, 2, 2],
          ' '.join(f'{k}:{v}' for k, v in got.items()))
    got = {}
    for r_x in (2.6, 2.79, 2.7999999, 2.8, 2.81, 3.5):
        s = servo()
        tick(s, target=at_range(2.4))
        got[r_x] = tick(s, target=at_range(r_x)).source
    s0 = servo(min_range_hysteresis_m=0.0)
    tick(s0, target=at_range(2.4))
    r0 = tick(s0, target=at_range(2.5)).source
    check('X2 exit only at >= min_range_m + hysteresis = 2.8 m: after entering at 2.4 m, 2.6 / '
          '2.79 / 2.7999999 m stay in source 2; 2.8 / 2.81 / 3.5 m leave; hysteresis 0 leaves at '
          '2.5 m (round one)',
          [got[k] for k in (2.6, 2.79, 2.7999999, 2.8, 2.81, 3.5)] == [2, 2, 2, 1, 1, 1]
          and r0 == 1 and 2.5 + 0.3 == 2.8,
          ' '.join(f'{k}:{v}' for k, v in got.items()) + f'; hysteresis 0 at 2.5: {r0}')
    # Flicker: a 15 Hz stream alternating 2.49 / 2.51 m (the 195108 bag: 8 changes in 0.42 s),
    # bearing drifting at 2 deg/s, after 1 s at 4 m.
    def flick(t, jitter=None):
        m = math.floor(t * 15.0 + 1e-9)
        r_m = 4.0 if t < 1.0 else (2.49 if m % 2 == 0 else 2.51)
        if jitter is not None and t >= 1.0:
            r_m = 2.50 + jitter[m % len(jitter)]
        return at_range(r_m, bearing_deg=2.0 * m / 15.0, age=0.25)
    rj = random.Random(4)
    jit = [rj.uniform(-0.01, 0.01) for _ in range(97)]
    for name, fn in (('alternating 2.49 / 2.51 m', flick),
                     ('2.50 +- 0.01 m random', lambda t: flick(t, jit))):
        out = {}
        for hy in (0.3, 0.0):
            res = [r for _, _, r in run(servo(ff_enable=True, min_range_hysteresis_m=hy), 4.0,
                                        stream=FnStream(fn))]
            out[hy] = transitions(res)
            if hy == 0.3:
                first2 = next(i for i, r in enumerate(res) if r.source == 2)
                stay = all(r.source == 2 and r.yawspeed == 0.0 for r in res[first2:])
        check(f'X3 flicker {name} at 15 Hz for 3 s: exactly one entry and no exit (hysteresis 0: '
              'one change per crossing, as round one)',
              out[0.3][0] == 1 and out[0.3][1] == 0 and stay and out[0.0][2] >= 10,
              f'hysteresis 0.3: {out[0.3][0]} entry, {out[0.3][1]} exits; hysteresis 0: '
              f'{out[0.0][0]} entries, {out[0.0][1]} exits, {out[0.0][2]} source changes')
    for hy, exp in ((0.3, 2.8), (0.0, 2.5)):
        res = run(servo(min_range_hysteresis_m=hy), 2.0,
                  stream=Stream(lambda t: at_range(2.4 + 0.5 * t)[:2]))    # walks out, 0.5 m/s
        i = next(i for i, (_, _, r) in enumerate(res) if r.source == 1)
        exit_r = res[i][2].range_m
        prev_r = res[i - 1][2].range_m
        check(f'X4 walking out at 0.5 m/s from 2.4 m, hysteresis {hy}: leaves on the first message at '
              f'>= {exp} m', res[0][2].source == 2 and exit_r >= exp - 1e-12 and prev_r < exp,
              f'last close-range tick at {prev_r:.4f} m, first servo tick at {exit_r:.4f} m')
    # In the latch band (2.5-2.8 m) the heading is HELD while the bearing moves.
    s = servo(ff_enable=True)
    for _ in range(50):
        tick(s, target=at_range(4.0, 10.0))
    rs = [tick(s, target=at_range(2.4, 10.0))] + [tick(s, target=at_range(2.6, 10.0 + 0.6 * k))
                                                   for k in range(50)]
    ok = all(r.source == 2 and r.psi_des == rs[0].psi_des and r.yaw == rs[0].yaw
             and r.yawspeed == 0.0 for r in rs)
    rx = [tick(s, target=at_range(2.9, 40.0)) for _ in range(40)]
    check('X5 inside the latch band (2.6 m) the bearing sweeps 10 -> 40 deg: heading held at 10 deg, '
          'yawspeed 0; at 2.9 m it aims at 40 deg, slew-limited',
          ok and rx[0].source == 1 and adiff(rx[-1].yaw, R(40.0)) < 1e-9
          and adiff(rx[0].yaw, rs[-1].yaw) <= R(90.0) * DT + EPS,
          f'held {D(rs[-1].yaw):+.2f} deg over {len(rs)} ticks while the bearing reached '
          f'{10.0 + 0.6 * 49:.1f}; then {D(rx[-1].yaw):+.2f} deg')

    def latched():
        s = servo()
        for _ in range(10):
            tick(s, target=at_range(2.4))
        return s
    s = latched()
    a = [tick(s, target=at_range(2.4, age=0.7)).source for _ in range(10)]    # 0.2 s stale
    a.append(tick(s, target=at_range(2.6)).source)
    s = latched()
    b = [tick(s, target=at_range(2.4, age=0.7)).source for _ in range(40)]    # 0.8 s stale
    b.append(tick(s, target=at_range(2.6)).source)
    s = servo(stale_hold_s=0.0)
    for _ in range(10):
        tick(s, target=at_range(2.4))
    c = [tick(s, target=at_range(2.4, age=0.7)).source, tick(s, target=at_range(2.6)).source]
    check('X6 the latch is KEPT through a stale hold (back at 2.6 m -> 2) and CLEARED by a fall-back '
          '(back at 2.6 m -> 1, judged afresh against 2.5 m); stale_hold_s 0: cleared at once',
          set(a[:10]) == {3} and a[10] == 2 and set(b[:25]) == {3} and set(b[25:40]) == {0}
          and b[40] == 1 and c == [0, 1],
          f'0.2 s stale: {a[0]}..{a[9]} then {a[10]}; 0.8 s stale: {b[0]}..{b[24]}, {b[25]}..{b[39]} '
          f'then {b[40]}; hold 0: {c}')
    res = []
    for how in ('reset()', 'IDLE', 'not engaged'):
        s = latched()
        if how == 'reset()':
            s.reset()
        elif how == 'IDLE':
            tick(s, state=ys.IDLE, target=at_range(2.4))
        else:
            tick(s, engaged=False, target=at_range(2.4))
        res.append((how, s._close, tick(s, target=at_range(2.6)).source))
    check('X7 the latch is cleared by reset(), IDLE and not-engaged: back at 2.6 m -> source 1',
          all(not c and src == 1 for _, c, src in res),
          '; '.join(f'{h}: latch {c}, then {src}' for h, c, src in res))
    rng_seq = (2.4, 2.5, 2.49, 2.5, 2.6, 2.4999, 2.79, 2.8)
    got0 = [tick(s0, target=at_range(r_m)).source for s0 in [servo(min_range_hysteresis_m=0.0)]
            for r_m in rng_seq]
    got3 = [tick(s3, target=at_range(r_m)).source for s3 in [servo()] for r_m in rng_seq]
    check('X8 hysteresis 0 = the round-one rule "close iff range < 2.5", equality included; '
          'default 0.3 on the same ranges',
          got0 == [2, 1, 2, 1, 1, 2, 1, 1] and got3 == [2, 2, 2, 2, 2, 2, 2, 1],
          f'ranges {rng_seq}: hysteresis 0 {got0}, 0.3 {got3}')
    got = [tick(s, target=at_range(r_m)).source for s in [servo(min_range_m=0.0,
                                                                min_range_hysteresis_m=5.0)]
           for r_m in (0.3, 0.01, 2.0, 0.3)]
    check('X9 min_range_m 0 disables the rule whatever the hysteresis (5 m here)', got == [1] * 4,
          f'{got}')


def hold_scenario(hold=0.5, dt_fn=None, T=3.0, reason=None, ff=False, psi_super=0.0,
                  yawdot=R(3.0), t_loss=1.0, t_back=None):
    """Aim at a static target 5 m away at FRD +60 (age 0.2 s) until t_loss, then no usable
    target (`reason`, default: stamps 0.9 s old) until t_back (None: never back).
    -> [(t, dt, result)]"""
    def fn(t):
        if t < t_loss - 1e-9 or (t_back is not None and t >= t_back - 1e-9):
            return at_range(5.0, 60.0, age=0.2)
        return at_range(5.0, 60.0, age=0.9)
    kw = dict(psi_super=psi_super, yawdot_super=yawdot)
    if reason is not None:
        fn, kw = reason(fn, t_loss, t_back, kw)
    return run(servo(stale_hold_s=hold, ff_enable=ff), T, dt_fn=dt_fn, stream=FnStream(fn), **kw)


def hold_stats(res):
    """(last usable tick index, hold ticks [(t, dt, r)], held s, first fall-back tick or None)."""
    i3 = next(i for i, (_, _, r) in enumerate(res) if r.source == 3)
    last = i3 - 1
    j = i3
    while j < len(res) and res[j][2].source == 3:
        j += 1
    hold = res[i3:j]
    held = sum(dt_sane(d) for _, d, _ in hold)
    fb = res[j] if j < len(res) and res[j][2].source == 0 else None
    return last, hold, held, fb


def checks_stale_hold():
    print('\nQ   stale hold (hold the last aim for stale_hold_s, yawspeed 0, then fall back)')
    # Q1: duration, measured in accumulated dt.
    rows = []
    rj = random.Random(8)
    jit = [rj.uniform(0.012, 0.028) for _ in range(400)]
    cases = [('50 Hz, 0.5 s', 0.5, None), ('jittered 12-28 ms, 0.5 s', 0.5, lambda k: jit[k]),
             ('50 Hz, 1.0 s', 1.0, None), ('50 Hz, 0.05 s', 0.05, None),
             ('50 Hz with a 0.3 s stall inside the hold, 0.5 s', 0.5,
              lambda k: 0.3 if k == 59 else DT)]
    ok = True
    for name, hold, dfn in cases:
        res = hold_scenario(hold=hold, dt_fn=dfn, T=3.0)
        last, hticks, held, fb = hold_stats(res)
        t_fb = fb[0] - res[last][0] if fb else math.nan
        good = (fb is not None and held <= hold + 1e-9 and held + dt_sane(fb[1]) > hold
                and t_fb <= hold + dt_sane(fb[1]) + 1e-9)
        ok &= good
        rows.append(f'{name}: {len(hticks)} ticks = {held:.3f} s held, fall-back {t_fb:.3f} s after '
                    f'the last usable tick')
    check('Q1 hold length = stale_hold_s of accumulated dt, within one tick (held <= stale_hold_s < '
          'held + the next dt)', ok, '; '.join(rows))
    res = hold_scenario(dt_fn=lambda k: 0.6 if k == 50 else DT)
    r_first = res[50][2]
    check('Q1 a 0.6 s stall as the first unusable tick: straight to the fall-back (0.6 > 0.5), moving '
          '<= 9 deg', r_first.source == 0 and adiff(r_first.yaw, res[49][2].yaw) <= R(9.0) + EPS,
          f'source {r_first.source}, moved {D(adiff(r_first.yaw, res[49][2].yaw)):.2f} deg')
    # Q2: the heading and yawspeed in the hold, ff off, SUPER 60 deg away with a 3 deg/s rate.
    res = hold_scenario()
    last, hticks, held, fb = hold_stats(res)
    aim = res[last][2]
    ok = all(r.psi_des == aim.psi_des and r.yaw == aim.yaw and bits(r.yawspeed) == bits(0.0)
             and r.active and math.isnan(r.range_m) for _, _, r in hticks)
    check('Q2 in the hold: psi_des = the last usable aim, heading unmoved, yawspeed exactly 0.0 (not '
          'SUPER\'s 3 deg/s), active, range NaN', ok and adiff(aim.yaw, R(60.0)) < 1e-9,
          f'{len(hticks)} ticks at {D(aim.yaw):+.3f} deg; SUPER at {D(0.0):+.1f} deg')
    # ... a slew in progress is finished toward the last aim, then the heading stays.
    def jump_then_lost(fn, t_loss, t_back, kw):
        def f(t):
            if t < 1.0 - 1e-9:
                return at_range(5.0, 0.0, age=0.2)
            if t < 1.1 - 1e-9:
                return at_range(5.0, 30.0, age=0.2)
            return at_range(5.0, 30.0, age=0.9)
        return f, kw
    res = hold_scenario(reason=jump_then_lost, T=2.5)
    last, hticks, held, fb = hold_stats(res)
    ys_ = [r.yaw for _, _, r in hticks]
    steps = [D(adiff(b, a)) for a, b in zip([res[last][2].yaw] + ys_, ys_)]
    reach = next(i for i, y in enumerate(ys_) if adiff(y, R(30.0)) < 1e-9)
    ok = (all(r.psi_des == res[last][2].psi_des for _, _, r in hticks)
          and all(abs(x - 1.8) < 1e-6 for x in steps[:reach])
          and all(x == 0.0 for x in steps[reach + 1:]) and adiff(res[last][2].yaw, R(30.0)) > R(20))
    check('Q2 a 30 deg turn in progress when the target is lost: the hold finishes it at 90 deg/s '
          'toward the last aim, then the heading stays', ok,
          f'{D(res[last][2].yaw):.1f} deg at the loss; +1.8 deg/tick for {reach} hold ticks, then '
          f'{len(ys_) - reach - 1} ticks unmoved at {D(ys_[-1]):.2f} deg')
    # Q3: ff on -- the ff decays in the hold, never SUPER's rate.
    def orbit_lost(t_back=None, jump_deg=0.0):
        def reason(fn, t_loss, tb, kw):
            o = orbit(0.0, -30.0)

            def f(t):
                m = math.floor(t * 15.0 + 1e-9) / 15.0
                x, y = o(m)
                if t_back is not None and t >= t_back - 1e-9 and jump_deg:
                    th = math.atan2(y, x) - R(jump_deg)
                    x, y = 5.0 * math.cos(th), 5.0 * math.sin(th)
                return (x, y, t - m + 0.2, 'lidar_init')
            kw = dict(kw, xy_valid=lambda t: t < 3.0 - 1e-9 or (t_back is not None
                                                                and t >= t_back - 1e-9),
                      psi_super=R(-120.0), yawdot_super=R(-10.0))
            return f, kw
        return reason
    res = hold_scenario(ff=True, reason=orbit_lost(), T=5.0)
    last, hticks, held, fb = hold_stats(res)
    ff0 = res[last][2].ff
    ffs = [r.ff for _, _, r in hticks]
    ok = (all(r.yawspeed == r.ff for _, _, r in hticks)
          and all(abs(b) <= abs(a) + EPS and a * b >= 0.0 for a, b in zip([ff0] + ffs, ffs))
          and all(r.yawspeed != R(-10.0) for _, _, r in hticks) and ff_slew_worst(res) <= 90.0 + 1e-6)
    t0 = next((t for t, _, r in hticks if r.ff == 0.0), math.nan) - res[last][0]
    check('Q3 ff on: in the hold yawspeed = the ff decaying monotonically at <= 90 deg/s^2 to 0; '
          'never SUPER\'s -10 deg/s, never their sum', ok and abs(ff0) > R(25.0),
          f'ff {D(ff0):+.2f} deg/s at the loss, 0 after {t0:.3f} s (|ff|/90 = {abs(D(ff0)) / 90:.3f})')
    # Q4: the target returns during the hold: back to aiming, no ff spike.
    for gap, jump in ((0.3, 0.0), (0.45, 40.0)):
        res = hold_scenario(ff=True, reason=orbit_lost(t_back=3.0 + gap, jump_deg=jump), T=7.0)
        pre = max(abs(r.ff) for t, _, r in res if 2.0 <= t < 3.0)
        srcs = [r.source for t, _, r in res if 3.0 <= t < 3.0 + gap - 1e-9]
        i_back = next(i for i, (t, _, r) in enumerate(res) if t >= 3.0 + gap - 1e-9)
        hold_des = res[i_back - 1][2].psi_des
        step = D(adiff(res[i_back][2].psi_des, hold_des))
        post = max(abs(r.ff) for _, _, r in res[i_back:])
        w, ex = slew_worst(res)
        lo, hi, _ = settled(res, 3.0 + gap + 2.5)
        check(f'Q4 target back after a {gap} s gap{f" {jump:.0f} deg away" if jump else ""} (30 deg/s '
              'orbit, ff at 30): source 3 during the gap, then 1; the first psi_des step is not a '
              'rate (no ff above its pre-gap value), ff slew and heading slew hold, re-settles',
              set(srcs) == {3} and res[i_back][2].source == 1 and post <= pre + EPS
              and ff_slew_worst(res) <= 90.0 + 1e-6 and ex <= EPS
              and 25.0 <= abs(D(lo)) and abs(D(hi)) <= 35.0,
              f'first psi_des step {step:.1f} deg in one tick (as a rate {step / DT:.0f} deg/s); '
              f'|ff| max {D(pre):.2f} before, {D(post):.2f} after; settled {D(lo):.1f}..'
              f'{D(hi):.1f} deg/s')
    # Q5: expiry -> fall-back without a jump.
    res = hold_scenario(T=3.5)
    last, hticks, held, fb = hold_stats(res)
    i_fb = res.index(fb)
    w, ex = slew_worst(res)
    arr = next(t for t, _, r in res[i_fb:] if r.yaw == 0.0) - fb[0]
    check('Q5 expiry -> fall-back: first step <= 90 deg/s x dt, psi_des = psi_super, walks 60 deg to '
          'it at 90 deg/s', fb[2].source == 0 and fb[2].psi_des == 0.0
          and adiff(fb[2].yaw, hticks[-1][2].yaw) <= R(90.0) * DT + EPS and ex <= EPS
          and arr <= 60.0 / 90.0 + DT, f'first step {D(adiff(fb[2].yaw, hticks[-1][2].yaw)):.3f} deg, '
          f'arrived {arr:.3f} s after the hold ended')
    # Q6: every reason for "no usable target" holds; round one (0 s) falls back at once.
    def rs(field, value):
        def reason(fn, t_loss, t_back, kw):
            def bad_time(t):
                return t_loss - 1e-9 <= t < t_back - 1e-9
            if field in ('age', 'frame', 'x', 'y', 'none', 'junk'):
                def f(t):
                    tg = at_range(5.0, 60.0, age=0.2)
                    if not bad_time(t):
                        return tg
                    if field == 'none':
                        return None
                    if field == 'junk':
                        return 'junk'
                    i = {'x': 0, 'y': 1, 'age': 2, 'frame': 3}[field]
                    return tg[:i] + (value,) + tg[i + 1:]
                return f, kw
            # A vehicle-side reason: the target stays fresh throughout, so it is the only one.
            key = {'xy_valid': 'xy_valid', 'vx': 'x', 'vy': 'y', 'heading': 'heading'}[field]
            kw = dict(kw, **{key: lambda t: value if bad_time(t) else
                             {'xy_valid': True, 'x': 0.0, 'y': 0.0, 'heading': 0.0}[key]})
            return (lambda t: at_range(5.0, 60.0, age=0.2)), kw
        return reason
    reasons = [('stale (age 0.7 s)', 'age', 0.7), ('future (age -0.5 s)', 'age', -0.5),
               ('age NaN', 'age', NAN), ('frame map', 'frame', 'map'),
               ('frame None', 'frame', None), ('target x NaN', 'x', NAN),
               ('target y inf', 'y', INF), ('lpos xy_valid False', 'xy_valid', False),
               ('vehicle x NaN', 'vx', NAN), ('vehicle y -inf', 'vy', -INF),
               ('heading NaN', 'heading', NAN), ('target None', 'none', None),
               ('target malformed', 'junk', None)]
    bad_new, bad_old = [], []
    for name, field, value in reasons:
        for hold in (0.5, 0.0):
            res = hold_scenario(hold=hold, reason=rs(field, value), t_loss=1.0, t_back=1.2, T=1.8)
            inj = [r for t, _, r in res if 1.0 - 1e-9 <= t < 1.2 - 1e-9]
            aim = [r for t, _, r in res if t < 1.0 - 1e-9][-1]
            after = [r for t, _, r in res if t >= 1.25]
            if hold:
                good = (len(inj) == 10 and all(r.source == 3 and r.yaw == aim.yaw
                                               and bits(r.yawspeed) == bits(0.0) for r in inj)
                        and all(r.source == 1 for r in after))
                if not good:
                    bad_new.append(name)
            elif not (all(r.source == 0 for r in inj) and all(r.source == 1 for r in after)):
                bad_old.append(name)
    check(f'Q6 every reason for no usable target ({len(reasons)}: stale, future, NaN age, bad frame x2, '
          'NaN/inf target, lpos invalid, NaN/inf vehicle x/y, NaN heading, None, malformed) -> the '
          'hold (source 3, aim held, yawspeed 0), then aims again; stale_hold_s 0: fall-back at once',
          not bad_new and not bad_old,
          f'failed with the hold: {bad_new}; with stale_hold_s 0: {bad_old}' if bad_new or bad_old
          else f'{len(reasons)} reasons x 2 settings')
    # Q7: no hold when the servo was not active before.
    a = [tick(servo()).source for _ in range(1)] + [r.source for r in [tick(s_) for s_ in [servo()]
                                                                       for _ in range(30)]]
    s = servo()
    b = [tick(s, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(10)]
    s = servo(aim_in_hold=False)
    c = ([tick(s, state=ys.HOLD, target=at_range(5.0, 60.0)).source for _ in range(20)]
         + [tick(s, state=ys.HOLD, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(10)])
    s = servo()
    for _ in range(30):
        tick(s, target=at_range(5.0, 60.0))
    tick(s, state=ys.IDLE, engaged=False)
    d = [tick(s, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(10)]
    check('Q7 no hold unless the servo owned the previous tick: first engage with no target, with a '
          'stale target, HOLD before the first TRACK (aim_in_hold False), after IDLE -> source 0',
          set(a) == {0} and set(b) == {0} and set(c) == {0} and set(d) == {0},
          f'{sorted(set(a))} {sorted(set(b))} {sorted(set(c))} {sorted(set(d))}')
    # Q8: IDLE / not engaged / reset() end a hold.
    res = []
    for how in ('IDLE', 'not engaged', 'reset()'):
        s = servo()
        for _ in range(30):
            tick(s, target=at_range(5.0, 60.0))
        pre = [tick(s, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(5)]
        if how == 'IDLE':
            tick(s, state=ys.IDLE, target=at_range(5.0, 60.0, age=0.9))
        elif how == 'not engaged':
            tick(s, engaged=False, target=at_range(5.0, 60.0, age=0.9))
        else:
            s.reset()
        post = tick(s, target=at_range(5.0, 60.0, age=0.9)).source
        res.append((how, pre, post, s._hold_t))
    check('Q8 a hold is ended by IDLE, not-engaged and reset(): the next stale tick is a fall-back',
          all(set(p) == {3} and q == 0 and h == 0.0 for _, p, q, h in res),
          '; '.join(f'{h}: {set(p)} -> {q}' for h, p, q, _ in res))
    # Q9: a fall-back never re-enters a hold; one usable tick re-arms it.
    s = servo()
    for _ in range(30):
        tick(s, target=at_range(5.0, 60.0))
    x = [tick(s, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(60)]
    y1 = tick(s, target=at_range(5.0, 60.0)).source
    z = [tick(s, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(3)]
    check('Q9 after the hold expires the fall-back stays a fall-back (no second hold); one usable '
          'tick re-arms the hold', x[:25] == [3] * 25 and set(x[25:]) == {0} and y1 == 1
          and z == [3, 3, 3], f'{x[0]}x{x.count(3)} then {x[25]}x{x.count(0)}; usable {y1}; then {z}')
    # Q10: TRACK <-> HOLD changes do not end a hold.
    s = servo()
    for _ in range(30):
        tick(s, target=at_range(5.0, 60.0))
    x = ([tick(s, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(10)]
         + [tick(s, state=ys.HOLD, target=at_range(5.0, 60.0, age=0.9)).source for _ in range(20)])
    check('Q10 the bridge going TRACK -> HOLD mid-hold does not end or restart it: 25 ticks, then 0',
          x[:25] == [3] * 25 and set(x[25:]) == {0}, f'{x.count(3)} hold ticks, then {x[25:]}')


# The round-one module's outputs on seeded inputs, recorded from super_bringup/scripts/
# yaw_servo.py as it was BEFORE round two (2026-09-29; frozen copy sha256 ecc4d055...,
# recorder: this function + soak_inputs/ref_inputs, 40,000 ticks per case). Digest prefixes:
# (full: every field of every tick; all-but: every field except yawspeed on fall-back ticks
# with the ff at 0; fb: yawspeed on those fall-back ticks where F-a cannot apply, i.e. the
# clamp did not clip and |yawdot_super| <= max_rate; n of those).
REF_N = 40000
REF_CASES = [
    ('V1 ff off, band 0', dict(enable=True, fallback_ff_band_deg=0.0), 41),
    ('V2 ff on, band 0', dict(enable=True, ff_enable=True, fallback_ff_band_deg=0.0), 42),
    ('V3 ff off', dict(enable=True), 43),
    ('V4 ff on', dict(enable=True, ff_enable=True), 44),
    ('V5 ff on, aim_in_hold off, camera +7.5', dict(enable=True, ff_enable=True, aim_in_hold=False,
                                                     camera_offset_deg=7.5), 45),
    ('V6 ff on, 45 deg/s, min_range 4, ff 30/45', dict(enable=True, ff_enable=True, max_rate=45.0,
                                                       min_range_m=4.0, ff_max_rate=30.0,
                                                       ff_max_accel=45.0), 46),
]
REF_DIGESTS = {
    ('soak_inputs', 41): ('e542b9f7db595583d0a2c9ff5c5b1da1', '560a927a7012f38db637485dffc3d98c',
                          'b756e1a29fd191617e970768abfc57d1', 557),
    ('soak_inputs', 42): ('b561ce2900e89c5ecf57d1cf3817c321', 'ed564bc7125e6164f5c74fe456f35a44',
                          '9232cc04ab27b5a912314608cd84190c', 184),
    ('soak_inputs', 43): ('956930721c27e947cc180363285ac183', 'a9851a6d2b0bb8c1b38dbb6080e979cc',
                          '72ba22fb20d983eb71bf4a946219a6e3', 629),
    ('soak_inputs', 44): ('5f3604dfdab4bbfab4be92070e8e66d6', '4eb4924ff8643af759b9d5f77eff5856',
                          'd91173850746dac84600d67049ad365f', 496),
    ('soak_inputs', 45): ('cfaad3d65113101b19ff82e9c1be9f88', '5da62c23865161e4b8fb57d9ba5cb39e',
                          'e7b21cedf114b33782ca4252b5981ad9', 1749),
    ('soak_inputs', 46): ('67f930d68d8156d36f0ba181903908a9', '843ac07138377010632653a8b6224291',
                          'f498aa1352799fa81862a8ec46f395c2', 195),
    ('ref_inputs', 41): ('1c07250f51272dbbe4a487c77ac52b02', 'b35efbaae14395604b68a61a7e2dfe93',
                         'e84b9aab7b7273454d0231017ebc6c66', 8025),
    ('ref_inputs', 42): ('39f7417ef3e22c07c392aff62f96c366', 'c36db04e2a339d0ac1142d37c6e531ad',
                         '35930a6b05f7d7c6bb1c7940feb4d64a', 8613),
    ('ref_inputs', 43): ('dcd793ad0bf6c4585e78f5924ffb7abf', '9799e8be8e2904fd7c83dbae22cfa3a1',
                         'e4a307d26a0e958803a5868b55d37e7a', 7679),
    ('ref_inputs', 44): ('5255d557df5b4dd3402c64a4d2f28b68', '6e38ea9c9d6a2e6c97aef26e460d038e',
                         'beac7434c9bc20ddc413e95ad65a5300', 8779),
    ('ref_inputs', 45): ('286a12cad167b02dd7175e6f2e11e3ec', 'c6cfd98803dfe138595a6d645be0f266',
                         '4006a66fa0df8198a7d82d37e1224d5f', 7906),
    ('ref_inputs', 46): ('a1db6bd6f1c7c4c5682852778a5d1e1e', '193aef11ed879880fdb51f4325f4bac7',
                         '3e2411c222f48249255b214ac6c46361', 4890),
}


def ref_digests(mod, kw, seed, n, gen):
    """(full, all-but, fb, n_fb) digests of `mod`'s outputs on gen(seed, n); see REF_DIGESTS."""
    s = mod.YawServo(mod.YawServoParams(**kw))
    mr = math.radians(kw.get('max_rate', 90.0))
    hf, ha, hb = hashlib.sha256(), hashlib.sha256(), hashlib.sha256()
    nb = 0
    for inp in gen(seed, n):
        r = s.step(**inp)
        head = (struct.pack('<6d', r.yaw, r.psi_des, r.psi_out, r.range_m, r.target_age_s, r.ff)
                + bytes([r.source, r.active, r.slew_binding, r.bad_frame, r.passthrough]))
        ysb = struct.pack('<d', r.yawspeed)
        hf.update(head + ysb)
        ha.update(head)
        if not (not r.passthrough and r.source == 0 and r.ff == 0.0):
            ha.update(ysb)
        else:
            yd, dt = inp['yawdot_super'], inp['dt']
            dt0 = not (isinstance(dt, float) and math.isfinite(dt) and dt > 0.0)
            clipped = r.slew_binding or (dt0 and r.psi_out != r.psi_des)
            if not clipped and math.isfinite(yd) and abs(yd) <= mr:
                hb.update(ysb)
                nb += 1
    return hf.hexdigest()[:32], ha.hexdigest()[:32], hb.hexdigest()[:32], nb


def ref_inputs(seed, n):
    """Seeded ticks that dwell where round two changed things: the fall-back with SUPER's yaw
    moving smoothly at 0-150 deg/s (yawdot consistent, or not), so the 2 deg band and the slew
    clamp meet often; targets that come and go (stale, silent, bad frame, lpos invalid) so the
    servo hands over to SUPER and back; a range wandering across 2.2-3.0 m with 15 Hz jitter, so
    min_range_m 2.5 is crossed and re-crossed. 50 Hz with jitter; TRACK/HOLD/IDLE segments.
    (Recorded with the round-one module; do not edit, or the V digests no longer apply.)"""
    rnd = random.Random(seed)
    t = 0.0
    psi_s, rate = 0.0, 0.0
    hd = 0.0
    mode, seg = ('TRACK', True), 0
    tgt_mode, tseg = 'fresh', 0
    rng, th = 2.6, 0.0
    msg = None
    for _ in range(n):
        dt = rnd.uniform(0.016, 0.024) if rnd.random() > 0.01 else rnd.choice([0.0, 0.05, 0.3])
        t += dt
        if seg <= 0:
            v = rnd.random()
            mode = ('TRACK', True) if v < 0.7 else ('HOLD', True) if v < 0.9 else ('IDLE', False)
            seg = rnd.randint(20, 300)
        seg -= 1
        if tseg <= 0:
            tgt_mode = rnd.choice(['fresh', 'fresh', 'fresh', 'silent', 'stale', 'badframe',
                                   'lposbad', 'future'])
            tseg = rnd.randint(3, 120)
        tseg -= 1
        if rnd.random() < 0.02:
            rate = math.radians(rnd.choice([0.0, 20.0, -45.0, 89.0, -95.0, 100.0, -150.0]))
        psi_s = math.atan2(math.sin(psi_s + rate * dt), math.cos(psi_s + rate * dt))
        ydot = rate if rnd.random() < 0.8 else rate * rnd.choice([0.0, 1.5, -1.0, 2.0])
        hd = math.atan2(math.sin(hd + rnd.gauss(0.0, 0.01)), math.cos(hd + rnd.gauss(0.0, 0.01)))
        rng = min(3.0, max(2.2, rng + rnd.gauss(0.0, 0.03)))
        th += rnd.gauss(0.0, 0.03)
        if tgt_mode != 'silent' and (msg is None or t - msg[4] > 1.0 / 15.0):
            r_m = rng + rnd.uniform(-0.02, 0.02)
            age0 = {'stale': 0.8, 'future': -0.5}.get(tgt_mode, rnd.uniform(0.2, 0.3))
            fr = 'camera' if tgt_mode == 'badframe' else rnd.choice(['lidar_init', 'world'])
            msg = (r_m * math.cos(th), r_m * math.sin(th), t - age0, fr, t)
        target = None if msg is None else (msg[0], msg[1], t - msg[2], msg[3])
        state, engaged = mode
        yield dict(dt=dt, state=state, engaged=engaged,
                   psi_super=psi_s if state == 'TRACK' else psi_s * 0.0 + 0.3,
                   yawdot_super=ydot if state == 'TRACK' else 0.0, vehicle_heading=hd,
                   vehicle_x=0.0, vehicle_y=0.0, vehicle_xy_valid=tgt_mode != 'lposbad',
                   target=target)


def checks_reference():
    print('\nV   min_range_hysteresis_m 0 and stale_hold_s 0 = the round-one module (recorded digests)')
    for gname, gen in (('soak_inputs', soak_inputs), ('ref_inputs', ref_inputs)):
        for label, kw, seed in REF_CASES:
            want = REF_DIGESTS[(gname, seed)]
            got = ref_digests(ys, dict(kw, min_range_hysteresis_m=0.0, stale_hold_s=0.0), seed,
                              REF_N, gen)
            band0 = kw.get('fallback_ff_band_deg', 2.0) == 0.0
            if band0:
                ok = got == want
                what = 'every field of every tick bit-identical (band 0: no F-a ticks)'
            else:
                ok = got[1:] == want[1:]
                what = ('every field bit-identical except fall-back yawspeed, which is identical '
                        'where F-a cannot apply')
            check(f'{label.split()[0]} [{gname}] {" ".join(label.split()[1:])}: {what}', ok,
                  f'{REF_N} ticks, {got[3]} fall-back rate ticks compared'
                  + ('' if ok else f'; got {got}, want {want}'))


def rand_float(rnd):
    u = rnd.random()
    if u < 0.1:
        return rnd.choice([NAN, INF, -INF, -0.0, 0.0, 5e-324, -5e-324, 1e308, -1e308, math.pi,
                           -math.pi, 7.0, -1e-300])
    if u < 0.3:
        return struct.unpack('<d', rnd.getrandbits(64).to_bytes(8, 'little'))[0]
    return rnd.uniform(-10.0, 10.0)


def checks_switch_off():
    print('\nZ   switch off = identity, bit for bit')
    rnd = random.Random(9)
    ok = True
    n = 0
    for ff in (False, True):
        s = ys.YawServo(ys.YawServoParams(enable=False, ff_enable=ff))
        for _ in range(3000):
            p, yd = rand_float(rnd), rand_float(rnd)
            tg = None if rnd.random() < 0.2 else (rand_float(rnd), rand_float(rnd),
                                                  rnd.uniform(-1, 1),
                                                  rnd.choice(['lidar_init', 'world', 'map']))
            r = tick(s, dt=rand_float(rnd), state=rnd.choice([ys.IDLE, ys.TRACK, ys.HOLD]),
                     engaged=rnd.random() < 0.8, psi_super=p, yawdot_super=yd,
                     heading=rand_float(rnd), x=rand_float(rnd), y=rand_float(rnd),
                     xy_valid=rnd.random() < 0.9, target=tg)
            ok &= (r.yaw is p and r.yawspeed is yd and bits(r.yaw) == bits(p)
                   and bits(r.yawspeed) == bits(yd) and r.passthrough and not r.active
                   and s._psi_out is None and s._ff == 0.0 and not s._win
                   and not s._close and s._hold_t == 0.0)
            n += 1
    check('Z1 enable False (ff on or off): yaw/yawspeed are psi_super/yawdot_super, same bits, '
          'state reset', ok, f'{n} random ticks incl. NaN payloads, inf, -0.0, subnormals')


# ---- the soak ----------------------------------------------------------------------------

def soak_inputs(seed, n):
    """Seeded random ticks: dt jitter, 0 and 0.5 s stalls, NaN dt; IDLE/TRACK/HOLD segments;
    a person random-walking around the aircraft (sometimes fast, sometimes inside close
    range, sometimes teleporting); 15 Hz messages with latency, dropouts, future stamps,
    wrong frames; NaNs sprinkled everywhere."""
    rnd = random.Random(seed)
    t = 0.0
    ax = ay = 0.0                      # aircraft, FLU
    hd = 0.0                           # heading, FRD
    rel_r, rel_th, w = 6.0, 0.0, 0.0   # person relative to the aircraft, FLU polar
    psi_sup = hold_psi = 0.0
    mode = (ys.IDLE, False)
    seg = 0
    msg = None
    next_msg = 0.0
    drop_until = -1.0
    bad = (NAN, INF, -INF)
    for _ in range(n):
        u = rnd.random()
        if u < 0.02:
            dt = 0.0
        elif u < 0.03:
            dt = 0.5
        elif u < 0.04:
            dt = rnd.uniform(0.0, 0.3)
        elif u < 0.044:
            dt = NAN
        elif u < 0.048:
            dt = -rnd.uniform(0.0, 0.05)
        elif u < 0.05:
            dt = INF
        else:
            dt = rnd.uniform(0.012, 0.028)
        adv = dt_sane(dt) or 0.02
        t += adv
        if seg <= 0:
            v = rnd.random()
            mode = ((ys.TRACK, True) if v < 0.50 else (ys.HOLD, True) if v < 0.75
                    else (ys.IDLE, False) if v < 0.88 else (ys.TRACK, False) if v < 0.92
                    else (ys.IDLE, True) if v < 0.96 else (ys.HOLD, False) if v < 0.98
                    else (rnd.choice([None, 'track', 'BOGUS', 7]), True))
            seg = rnd.randint(5, 400)
            hold_psi = psi_sup
        seg -= 1
        state, engaged = mode

        ax += rnd.gauss(0.0, 0.02)
        ay += rnd.gauss(0.0, 0.02)
        hd = wrap(hd + rnd.gauss(0.0, R(1.0))
                  + (R(rnd.uniform(-180, 180)) if rnd.random() < 0.002 else 0.0))
        w = max(-R(150), min(R(150), w + rnd.gauss(0.0, R(8.0))))
        if rnd.random() < 0.01:
            w *= 0.2
        rel_th += w * adv
        rel_r = max(0.5, min(15.0, rel_r + rnd.gauss(0.0, 0.05)))
        if rnd.random() < 0.001:                             # a teleport (track switch)
            rel_r, rel_th = rnd.uniform(0.5, 15.0), rnd.uniform(-math.pi, math.pi)
        px, py = ax + rel_r * math.cos(rel_th), ay + rel_r * math.sin(rel_th)

        if t >= drop_until and t >= next_msg:
            if rnd.random() < 0.003:
                drop_until = t + rnd.uniform(0.2, 3.0)
            else:
                lat = rnd.uniform(0.03, 0.15)
                if rnd.random() < 0.02:
                    lat = -rnd.uniform(0.0, 0.3)
                v = rnd.random()
                frame = ('lidar_init' if v < 0.93 else 'world' if v < 0.96
                         else rnd.choice(['map', '', 'base_link', None]))
                mx = rnd.choice(bad) if rnd.random() < 0.005 else px
                my = rnd.choice(bad) if rnd.random() < 0.005 else py
                msg = (mx, my, t - lat, frame)
                next_msg = t + rnd.uniform(0.05, 0.085)
        target = None
        if msg is not None:
            age = rnd.choice(bad) if rnd.random() < 0.002 else t - msg[2]
            target = (msg[0], msg[1], age, msg[3])

        if state == ys.TRACK and engaged:
            psi_sup = wrap(psi_sup + rnd.gauss(0.0, R(1.5))
                           + (R(rnd.uniform(-180, 180)) if rnd.random() < 0.003 else 0.0))
            psi_in = psi_sup
            ydot = rnd.gauss(0.0, R(20.0)) if rnd.random() < 0.95 else rnd.uniform(-R(200), R(200))
        elif state == ys.HOLD and engaged:
            psi_in, ydot = hold_psi, 0.0
        else:
            psi_in, ydot = hd, 0.0
        if rnd.random() < 0.002:
            psi_in = rnd.choice(bad)
        if rnd.random() < 0.002:
            ydot = rnd.choice(bad)
        heading = rnd.choice(bad) if rnd.random() < 0.002 else hd
        vx = rnd.choice(bad) if rnd.random() < 0.002 else ax
        vy = rnd.choice(bad) if rnd.random() < 0.002 else -ay
        yield dict(dt=dt, state=state, engaged=engaged, psi_super=psi_in, yawdot_super=ydot,
                   vehicle_heading=heading, vehicle_x=vx, vehicle_y=vy,
                   vehicle_xy_valid=rnd.random() > 0.005, target=target)


def indep_usable(inp, p):
    """The soak's own reading of "a usable target" (plan section 2.3 step 1), for sources 1-3."""
    tg = inp['target']
    if tg is None:
        return False
    try:
        tx, ty, age, fr = tg
        tx, ty, age = float(tx), float(ty), float(age)
    except Exception:  # malformed: not usable
        return False
    vx, vy, hd = inp['vehicle_x'], inp['vehicle_y'], inp['vehicle_heading']
    if not all(math.isfinite(v) for v in (tx, ty, age, vx, vy, hd)):
        return False
    return (-p.future_tolerance <= age <= p.target_timeout and isinstance(fr, str)
            and fr in ys.ACCEPTED_FRAMES and inp['vehicle_xy_valid'] is True
            and math.isfinite(math.hypot(tx - vx, -ty - vy)))


def soak(label, kw, seed, n):
    p = ys.YawServoParams(**kw)
    s = ys.YawServo(p)
    mr, ffm, acc = R(p.max_rate), R(p.ff_max_rate), R(p.ff_max_accel)
    band, off = R(p.fallback_ff_band_deg), R(p.camera_offset_deg)
    hold_max, r_in, r_out = p.stale_hold_s, p.min_range_m, p.min_range_m + p.min_range_hysteresis_m
    viol = []
    n_viol = 0

    def bad(k, why):
        nonlocal n_viol
        n_viol += 1
        if len(viol) < 5:
            viol.append(f'tick {k}: {why}')

    prev = None
    seen_track = False
    in_close = False          # the soak's own close-range latch
    hold_acc = 0.0            # accumulated dt of the current run of source-3 ticks
    last_aim = None           # psi_des of the last source-1/2 tick
    cnt = {0: 0, 1: 0, 2: 0, 3: 0}
    n_pass = n_bind = n_badf = n_stall = n_holds = n_expired = n_fb_bind = 0
    w_rate = w_ratio = w_ys = w_acc = w_des = w_ydot_in = w_hold = 0.0
    t0 = time.monotonic()
    for k, inp in enumerate(soak_inputs(seed, n)):
        try:
            r = s.step(**inp)
        except Exception as e:  # the check: step() must never raise
            bad(k, f'raised {e!r}')
            prev = None
            continue
        st, eng = inp['state'], inp['engaged']
        flying = p.enable and eng and isinstance(st, str) and st in (ys.TRACK, ys.HOLD)
        if flying and st == ys.TRACK:
            seen_track = True
        if not flying:
            seen_track = False
        dts = dt_sane(inp['dt'])
        dc = min(dts, ys.DT_MAX)
        n_stall += dts > ys.DT_MAX
        psi_s, ydot = inp['psi_super'], inp['yawdot_super']
        if math.isfinite(ydot):
            w_ydot_in = max(w_ydot_in, abs(ydot))

        if r.passthrough:
            n_pass += 1
            if not (r.yaw is psi_s and r.yawspeed is ydot):
                bad(k, 'pass-through is not identity')
            if flying and (math.isfinite(inp['vehicle_heading']) or math.isfinite(psi_s)):
                bad(k, 'passed through while engaged with a finite heading or psi_super')
            prev, in_close, hold_acc, last_aim = None, False, 0.0, None
            continue
        if not flying:
            bad(k, 'servo output while not engaged in TRACK/HOLD with the switch on')
        if not (in_range(r.yaw) and isinstance(r.yawspeed, float) and math.isfinite(r.yawspeed)):
            bad(k, f'output not finite/wrapped: {r.yaw!r} {r.yawspeed!r}')
            prev = None
            continue
        cnt[r.source] += 1
        n_bind += r.slew_binding
        n_badf += r.bad_frame

        if prev is not None:
            ch = adiff(r.yaw, prev.yaw)
            lim = mr * dc
            if ch > lim + EPS:
                bad(k, f'heading step {D(ch):.4f} deg > limit {D(lim):.4f}')
            if dc > 0.0:
                w_rate = max(w_rate, ch / dc)
                w_ratio = max(w_ratio, ch / lim)
        yd_f = abs(ydot) if math.isfinite(ydot) else 0.0
        if abs(r.yawspeed) > max(ffm, yd_f) + EPS:
            bad(k, f'|yawspeed| {D(r.yawspeed):.2f} > max(ff cap, |yawdot_super|)')
        # Round two (F-a): the bound is the servo's own limits, whatever SUPER sends.
        if abs(r.yawspeed) > max(ffm, mr) + EPS:
            bad(k, f'|yawspeed| {D(r.yawspeed):.2f} > max(ff_max_rate, max_rate)')
        w_ys = max(w_ys, abs(r.yawspeed))

        gap = abs(wrap(psi_s - r.yaw)) if math.isfinite(psi_s) else INF
        clip = clipped_of(r, inp['dt'])
        fb_ok = not clip and gap < band and math.isfinite(ydot)
        band_val = max(-mr, min(mr, ydot)) if fb_ok else 0.0
        near_edge = abs(gap - band) < 1e-9
        if r.source == 0 and r.ff == 0.0 and clip and gap < band:
            n_fb_bind += 1
            if bits(r.yawspeed) != bits(0.0):
                bad(k, 'fall-back sent a rate on a tick the slew clamp clipped')
        if p.ff_enable:
            if prev is not None and r.ff != 0.0:
                dff = abs(r.ff - prev.ff)
                if dff > acc * dc + EPS:
                    bad(k, f'ff step {D(dff):.4f} deg/s > {D(acc * dc):.4f}')
                if dc > 0.0:
                    w_acc = max(w_acc, dff / dc)
            if r.source == 1 and r.yawspeed != r.ff:
                bad(k, 'servo yawspeed is not the ff')
            if r.source == 2 and not (r.yawspeed == 0.0 and r.ff == 0.0):
                bad(k, 'close-range yawspeed not 0')
            if r.source == 3 and not (r.yawspeed == r.ff and prev is not None
                                      and abs(r.ff) <= abs(prev.ff) and r.ff * prev.ff >= 0.0):
                bad(k, 'stale hold: yawspeed is not the decaying ff')
            if r.source == 0:
                if r.ff != 0.0:
                    if r.yawspeed != r.ff:
                        bad(k, 'fall-back while the ff decays: yawspeed is not the ff')
                elif not near_edge and r.yawspeed != band_val:
                    bad(k, 'fall-back rule (F-a)')
            if abs(r.ff) > ffm + EPS:
                bad(k, 'ff above its cap')
        else:
            if r.ff != 0.0:
                bad(k, 'ff state non-zero with ff_enable off')
            if r.source in (1, 2, 3) and bits(r.yawspeed) != bits(0.0):
                bad(k, 'servo yawspeed not exactly 0.0 with ff off')
            if r.source == 0 and not near_edge and r.yawspeed != band_val:
                bad(k, 'fall-back rule (F-a)')

        usable = indep_usable(inp, p)
        if r.source in (1, 2):
            tg = inp['target']
            if not usable:
                bad(k, 'servo active on a stale / wrong-frame / non-finite target or invalid position')
                prev = r
                continue
            elif st == ys.HOLD and not p.aim_in_hold and not seen_track:
                bad(k, 'servo active in HOLD before the first TRACK (aim_in_hold False)')
            rng = math.hypot(tg[0] - inp['vehicle_x'], -tg[1] - inp['vehicle_y'])
            want_close = (rng < r_out) if in_close else (rng < r_in)
            if (r.source == 2) != want_close:
                bad(k, f'source {r.source} at range {rng:.3f} m (latched {in_close})')
            if (r.source == 2 and not rng < r_out) or (r.source == 1 and rng < r_in):
                bad(k, f'source {r.source} at range {rng:.3f} m outside [{r_in}, {r_out})')
            in_close = r.source == 2
            if r.source == 1:
                exp = wrap(-math.atan2(tg[1] - (-inp['vehicle_y']), tg[0] - inp['vehicle_x'])
                           + off)
                e = adiff(r.psi_des, exp)
                w_des = max(w_des, e)
                if e > 1e-9:
                    bad(k, f'psi_des {D(r.psi_des):.4f} != independent {D(exp):.4f}')
            hold_acc, last_aim = 0.0, r.psi_des
        elif r.source == 3:
            if prev is None or prev.source not in (1, 2, 3):
                bad(k, 'stale hold without an active tick before it')
            if usable and (p.aim_in_hold or seen_track or st == ys.TRACK):
                bad(k, 'stale hold with a usable target')
            if last_aim is None or bits(r.psi_des) != bits(last_aim):
                bad(k, 'stale hold: psi_des is not the last usable aim')
            if prev is not None and prev.source in (1, 2):
                n_holds += 1
            hold_acc += dts
            w_hold = max(w_hold, hold_acc)
            if hold_acc > hold_max + 1e-9:
                bad(k, f'stale hold {hold_acc:.4f} s > stale_hold_s {hold_max}')
        else:
            if prev is not None and prev.source in (1, 2, 3) and hold_max > 0.0 \
                    and hold_acc + dts <= hold_max + 1e-9:
                bad(k, f'fell back after {hold_acc + dts:.4f} s of hold < stale_hold_s {hold_max}')
            if prev is not None and prev.source == 3:
                n_expired += 1
            in_close, hold_acc, last_aim = False, 0.0, None
        prev = r
    el = time.monotonic() - t0
    print(f'    {label}: {n} ticks in {el:.1f} s; sources 0/1/2/3 = {cnt[0]}/{cnt[1]}/{cnt[2]}/'
          f'{cnt[3]}, pass-through {n_pass}, slew binding {n_bind}, bad_frame {n_badf}, '
          f'stalls > 0.1 s {n_stall}')
    print(f'      worst: heading rate {D(w_rate):.3f} deg/s (limit {p.max_rate:g}; '
          f'{w_ratio:.9f} of the per-tick limit), |yawspeed| {D(w_ys):.3f} deg/s '
          f'(bound max(ff cap, max_rate) {max(p.ff_max_rate, p.max_rate):g}; |yawdot_super| in: '
          f'{D(w_ydot_in):.1f}), ff slew {D(w_acc):.3f} deg/s^2 (limit {p.ff_max_accel:g}), '
          f'servo psi_des vs independent {D(w_des):.1e} deg')
    print(f'      stale holds {n_holds} (longest {w_hold:.3f} s of accumulated dt, limit '
          f'{hold_max:g} s; {n_expired} expired into a fall-back); fall-back ticks in the band '
          f'with the clamp clipping, sent 0.0: {n_fb_bind}')
    check(f'K {label}: finite, wrapped, slew/ff/yawspeed (<= max(ff cap, max_rate)) bounds, F-a, '
          'hysteresis, stale hold and frames hold on every tick',
          n_viol == 0, f'{n_viol} violations' + (f'; {viol}' if viol else ''))
    return n


def checks_soak():
    print('\nK   randomized soak (seeded)')
    total = 0
    for label, kw, seed, n in (
            ('ff off', dict(enable=True), 11, 50000),
            ('ff on', dict(enable=True, ff_enable=True), 12, 50000),
            ('ff on, aim_in_hold off, camera +7.5 deg',
             dict(enable=True, ff_enable=True, aim_in_hold=False, camera_offset_deg=7.5), 13,
             50000),
            ('ff on, max_rate 45, min_range 4 m, ff 30 deg/s / 45 deg/s^2',
             dict(enable=True, ff_enable=True, max_rate=45.0, min_range_m=4.0,
                  ff_max_rate=30.0, ff_max_accel=45.0), 14, 50000),
            ('ff on, hysteresis 0, stale hold 0 (round one + F-a)',
             dict(enable=True, ff_enable=True, min_range_hysteresis_m=0.0, stale_hold_s=0.0), 16,
             50000),
            ('ff off, hysteresis 1.0 m, stale hold 1.0 s',
             dict(enable=True, min_range_hysteresis_m=1.0, stale_hold_s=1.0), 17, 50000),
            ('switch off', dict(enable=False, ff_enable=True), 15, 10000)):
        total += soak(label, kw, seed, n)
    print(f'    total {total} ticks')


def checks_determinism():
    print('\nD   determinism')
    p = ys.YawServoParams(enable=True, ff_enable=True, camera_offset_deg=5.0)
    a, b = ys.YawServo(p), ys.YawServo(p)
    ok = True
    n = 0
    for inp in soak_inputs(21, 20000):
        ra, rb = a.step(**inp), b.step(**inp)
        for f in ys.YawServoOutput.__slots__:
            va, vb = getattr(ra, f), getattr(rb, f)
            if isinstance(va, float):
                ok &= isinstance(vb, float) and bits(va) == bits(vb)
            else:
                ok &= va == vb or va is vb
        n += 1
    check('D1 two instances fed the same 20,000 ticks give bit-identical outputs', ok, f'{n} ticks')


def main():
    repo = os.path.join(HERE, '..', '..')
    print(f'yaw_servo_unit_check -- {os.path.relpath(ys.__file__, repo)}\n')
    checks_params()
    checks_sign()
    checks_wrap()
    checks_slew()
    checks_ff()
    checks_close()
    checks_fresh_frames()
    checks_fallback()
    checks_nan()
    checks_idle()
    checks_hold()
    checks_offset()
    checks_switch_off()
    checks_fallback_rate()
    checks_hysteresis()
    checks_stale_hold()
    checks_reference()
    checks_determinism()
    checks_soak()

    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print(f'\n{len(RESULTS) - n_fail}/{len(RESULTS)} checks passed')
    if n_fail:
        print('\nFAILED:')
        for name, ok, detail in RESULTS:
            if not ok:
                print(f'  {name}  {detail}')
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
