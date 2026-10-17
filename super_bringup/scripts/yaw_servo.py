#!/usr/bin/env python3
"""Camera yaw servo: aim the body at /planning/yaw_target from where the aircraft IS.

This is the per-tick algorithm of docs/plan-camera-yaw-servo.md section 2.3 ("fix 1", the
heading, and "fix 2", its rate as `yawspeed` feed-forward), and nothing else. It is pure
Python on `math` alone: no rclpy, no numpy, no message types, no clock, no logging. Both
px4_offboard_bridge.py (what flies) and drone-dtp/scripts/fix1_yaw_replay.py (the replay
gate) import it, so the replay tests the code that flies. There must be one copy.

Every input is passed to `YawServo.step()` explicitly, including `dt`. The module never
reads a clock, so a replay on a fixed 0.02 s grid and the live 50 Hz timer run the same
arithmetic.

Frames and signs -- read this before touching anything
-------------------------------------------------------
SUPER and the yaw target live in FAST-LIO's `lidar_init`: FLU (x fwd, y LEFT, z UP), with an
arbitrary constant heading offset from North. EKF2's local frame is that same frame
flipped, because px4_odom_bridge declares POSE_FRAME_FRD:

    position (x, y, z) FLU  ->  (x, -y, -z) NED/FRD
    yaw      psi       FLU  ->  -psi       FRD       (z flips, so the sense of rotation flips)
    yaw rate psi_dot   FLU  ->  -psi_dot   FRD

**NOT `pi/2 - psi`.** That is the textbook ENU->NED rule. lidar_init is not ENU and PX4's
frame here is not North-aligned; pi/2 - psi would put a constant 90 deg error into every
setpoint. (px4_offboard_bridge.py docstring, docs/ImplementationDoc.md SS M3.)

Everything this module outputs is PX4 FRD: yaw in rad, wrapped to (-pi, pi], positive =
clockwise seen from above = a turn to the RIGHT; yawspeed in rad/s, same sign.

Inputs are FRD/NED too, EXCEPT the target: it is passed exactly as it arrives on
/planning/yaw_target, FLU lidar_init, and the module converts it itself, (x, y) FLU ->
(x, -y) NED, so the conversion is inside the unit-tested code.

The desired heading is the bearing from the aircraft's CURRENT position (lpos, NED) to the
target (NED):

    psi_des = atan2(-t_y_flu - p_y_ned, t_x_flu - p_x_ned)

Since p_y_ned = -p_y_flu, this is atan2(-(t_y - p_y)_flu, (t_x - p_x)_flu)
= -atan2(dy_flu, dx_flu): the FRD negation of the FLU bearing, the same psi -> -psi rule
the bridge already applies to SUPER's yaw.

Worked sign example. The aircraft at the origin, a target at FLU (0, +5), i.e. 5 m to the
aircraft's LEFT. NED target = (0, -5). psi_des = atan2(-5 - 0, 0 - 0) = atan2(-5, 0) =
-pi/2. FRD yaw -pi/2 is a LEFT turn (negative = counter-clockwise from above), and it equals
-atan2(dy_flu, dx_flu) = -atan2(5, 0) = -pi/2. The pi/2 - psi variant would give
pi/2 - pi/2 = 0, i.e. "keep looking straight ahead" -- the 90 deg trap.

A target moving counter-clockwise around the aircraft in FLU (to its left) has an
increasing FLU bearing, a decreasing FRD bearing, and so a NEGATIVE yawspeed feed-forward.

Camera offset. `camera_offset_deg` = delta is the camera optical axis's yaw relative to the
body x-axis, positive = the camera points to the LEFT of the nose (FLU positive yaw). To put
the person on the optical axis:  camera_yaw_flu = body_yaw_flu + delta = bearing_flu, so
body_yaw_flu = bearing_flu - delta, and in FRD (negate)  body_yaw_frd = bearing_frd + delta.
So  psi_des = wrap(bearing_frd + radians(delta)):  a camera looking LEFT makes the body aim
to the RIGHT of the person (positive FRD). delta = 0 is an exact no-op (the bearing is not
touched at all, bit for bit).

The algorithm, one call per bridge tick (plan section 2.3)
----------------------------------------------------------
0. Identity. With `enable` false, or the bridge not engaged (armed AND Offboard), or its
   state not TRACK/HOLD: return psi_super and yawdot_super UNTOUCHED (the very same float
   objects: no wrap, no arithmetic) and reset all internal state. So the first engaged tick
   re-initialises (step 5) and never slews from a heading left over from an earlier
   engagement.
1. The tick has a USABLE target (the servo may aim) only if all of these hold:
   - enable, engaged, TRACK or HOLD (HOLD only if `aim_in_hold`, or once a TRACK has been
     seen since the engage);
   - a target exists and is FRESH: -future_tolerance <= age <= target_timeout, with
     age = now - stamp (negative = stamped in the future);
   - its frame_id is exactly 'lidar_init' or 'world' (identity TF in this stack). A fresh
     target in any other frame is ignored and `bad_frame` is set, so the caller can warn;
   - target x/y, vehicle x/y and vehicle heading all finite, vehicle_xy_valid true, and the
     range finite.
2. Usable, and not in close range (step 3): psi_des = bearing (+ camera offset). source = 1.
3. Usable and in CLOSE RANGE, with hysteresis: the hold is ENTERED when range <
   min_range_m and LEFT only when range >= min_range_m + min_range_hysteresis_m (between the
   two it stays as it was). With hysteresis 0 this is exactly the round-one rule, "close iff
   range < min_range_m", equality included. The latch is servo state: reset() (so step 0)
   clears it, and so does every fall-back tick (source 0), so a re-acquired target is judged
   afresh against min_range_m; a stale hold (source 3) keeps it. In close range psi_des = the
   psi_des of the previous tick if the servo owned that tick too (source 1, 2 or 3), else the
   current psi_out, i.e. the heading is held where it is and nothing turns. The feed-forward
   is 0 at once (not decayed: a rate going to zero is the safe direction). source = 2;
   `active` is still True, because the servo, not SUPER, owns the heading.
3b. STALE HOLD (round two). No usable target on this tick, FOR ANY REASON (stale, stamped too
   far ahead, bad frame, non-finite target, invalid or non-finite lpos, no target), but the
   servo owned the previous tick (source 1, 2 or 3): for up to `stale_hold_s` seconds of
   accumulated dt since the last tick with a usable target (the sum of this and the earlier
   hold ticks' sane dts, a NaN / <= 0 dt counting 0, compared with a 1e-9 s tolerance), the
   servo does not fall back yet. psi_des = the psi_des of the last usable tick, so psi_out
   finishes any slew in progress toward that last aim point and then stays. source = 3,
   `active` True. The first tick past stale_hold_s falls back (step 4), and a fall-back never
   re-enters a hold: only a usable tick re-arms it. It never applies when the servo was not
   active before (a first engage with no target, HOLD before the first TRACK with
   aim_in_hold False), and step 0 / reset() clears it. stale_hold_s 0 = no hold at all (the
   round-one behaviour). Why: the target arrives already ~0.25 s old (the perception stamp),
   so a dip of ~0.25 s in the 15 Hz stream made the round-one servo swing the nose toward
   SUPER's lagging heading at up to 90 deg/s and back on the next message.
4. Neither (FALL-BACK, switch on and engaged): psi_des = psi_super (in HOLD that is the
   bridge's frozen yaw). If psi_super is not finite, psi_des = psi_out (hold). source = 0.
5. Slew limit, on EVERY tick that gets here, whatever the source:
       psi_out += clamp(wrap(psi_des - psi_out), +-max_rate * dt_c)
   dt_c = min(dt, 0.1 s), and dt that is NaN, inf, zero or negative counts as 0 (the output
   does not move), so a stalled timer cannot produce a big step. On the first tick after
   the reset of step 0, psi_out starts at the vehicle's heading, or at psi_super if the
   heading is not finite. If neither is finite the tick is passed through (identity) and
   the servo stays uninitialised; the bridge never sends a non-finite SUPER yaw, and its
   IDLE/HOLD latch needs a finite heading, so this is unreachable in practice.
   `slew_binding` = the clamp clipped this tick (only counted when dt_c > 0). NOTE: the
   yaw target arrives at ~15 Hz, so psi_des is a staircase. At 50 Hz the clamp allows
   1.8 deg per tick, and a single 15 Hz message step exceeds that for bearing rates above
   ~27 deg/s: the clamp then binds for one tick per message (the remainder follows on the
   next tick). That is harmless, but it means a binary "binding" flag fires in a brisk
   normal follow; judge a slew problem by how long it binds, not by the flag alone.
6. yawspeed:
   - source 1, 2 or 3 with ff_enable false: exactly 0.0.
   - source 2: exactly 0.0 (feed-forward cleared).
   - source 1 with ff_enable true: the feed-forward `ff`, below.
   - source 3 with ff_enable true: the feed-forward DECAYING to 0 at ff_max_accel. Never
     SUPER's rate.
   - source 0 (fall-back): if ff_enable and the feed-forward state is not yet 0, the
     DECAYING feed-forward (it slews to 0 at ff_max_accel); once it is exactly 0, SUPER's
     rate, but only if ALL of these hold (round two, "F-a"):
       * the slew clamp did NOT clip on this tick (|wrap(psi_des - psi_out_prev)| <=
         max_rate * dt_c; with dt_c 0 any non-zero error counts as clipped), i.e. the heading
         sent is SUPER's own this tick;
       * |wrap(psi_super - psi_out)| < fallback_ff_band, with psi_out the value sent THIS
         tick;
       * yawdot_super is finite;
     and then clamped to +-max_rate, so the rate sent can never exceed the heading's slew
     limit, whatever SUPER says. Otherwise 0.0. The decaying feed-forward and SUPER's rate
     are never added: one or the other is sent. So |yawspeed| <= max(ff_max_rate, max_rate)
     always, whatever the input. (Round one tested only the band on this tick's output, and
     in the 2026-09-26 20:06 bag sent SUPER's 96-126 deg/s while the clamp was binding: the
     modelled aircraft reached 124 deg/s.)

The feed-forward (fix 2)
------------------------
Per tick in source 1 with dt > 0:  step = wrap(psi_des - psi_des_prev), the change in the
desired heading since the previous source-1 tick. The feed-forward target is the TIME
average of those steps over the last `ff_window` seconds:

    avg = sum(step_i) / max(sum(dt_i), ff_window)       (a list of (dt_i, step_i))

i.e. a dt-weighted moving average of step_i/dt_i, correct at 50 Hz and still sane at other
tick rates. Dividing by at least ff_window makes a freshly (re)started average ramp up as
it fills instead of shooting to step/dt on its first sample. Then

    ff += clamp(clamp(avg, +-ff_max_rate) - ff, +-ff_max_accel * dt_c)

The staircase problem, and the step-reset rule. psi_des does not move smoothly: most ticks
carry only the aircraft's own motion (tiny steps), and the tick on which a new 15 Hz
message lands carries the whole 67 ms of the person's bearing change. A target that sweeps
at 100 deg/s therefore steps ~6.7 deg on every message. The plan's rule "a psi_des step
over 5 deg in one tick resets the average to 0" would then kill the feed-forward on every
message of a fast sweep, and the gate "100 deg/s -> capped at 60 deg/s" could never pass.
What this module does instead:

    A step |s| > ff_step_reset_deg is a JUMP unless, in the last ff_step_consistency_s
    (0.25 s: covers one or two dropped 15 Hz messages), an earlier step s_j had the SAME
    sign and 0.4*|s| <= |s_j| <= 2.5*|s|.

    A jump clears the average (it "resets the average to 0") and is not itself a sample.
    The feed-forward then slews toward the new, empty average: it decays at ff_max_accel,
    it never steps and it never chases the jump. Every step of at least
    0.4 * ff_step_reset_deg (2 deg) is remembered for this test, jumps included, whether or
    not it entered the average (a smaller one could never be "similar" to a jump).

So a continuous sweep is recognised by its own repetition: its steps look like each other.
The first message step of a sweep that starts abruptly above ~75 deg/s is treated as a jump
(one message, ~67 ms, of delay), the second is consistent with it, and from then on the
average fills and the feed-forward climbs to the cap. A one-off jump (a track switch, the
yaw hold latching its point, a re-emergence point) is not like the steps before it, and a
flicker A -> B -> A has opposite signs, so neither feeds the average. The worst a jump can
do is repeat itself in the same direction within 0.25 s (two similar jumps in a row):
the second then enters the average for 0.2 s and the feed-forward can climb by at most
ff_max_accel * ff_window = 18 deg/s before it decays again.

The average is also cleared (and the step history forgotten) when the servo (re)enters
source 1 from any other source, at close range, in a stale hold, in fall-back, and on a
timing discontinuity (dt > 0.1 s): none of those ticks has a meaningful previous psi_des. A
dt of 0 (a duplicate tick) takes no sample and does not advance psi_des_prev, so a step on
that tick is judged on the next tick with time. In particular, a target that returns after a
stale hold (source 3 -> 1) restarts the average: the first psi_des step after the gap, which
may be large (the person moved while the stream was down), is never taken as a rate, and the
feed-forward carries on from its decayed value toward the new, empty average.

Output (YawServoOutput)
-----------------------
    yaw           rad, FRD, (-pi, pi]   what to put in TrajectorySetpoint.yaw
    yawspeed      rad/s, FRD            what to put in TrajectorySetpoint.yawspeed
    active        bool   the servo owns the heading (source 1, 2 or 3)
    psi_des       rad    this tick's desired heading (NaN when passed through)
    psi_out       rad    == yaw
    range_m       m      horizontal range used this tick; NaN when no usable target
    target_age_s  s      age of the latest target (NaN when none, or age not finite)
    source        int    0 SUPER / fall-back / pass-through, 1 servo, 2 close-range hold,
                         3 stale hold (the last aim point, no usable target, <= stale_hold_s)
    slew_binding  bool   the slew clamp clipped this tick
    bad_frame     bool   a fresh target was ignored for its frame_id (evaluated only on
                         engaged ticks with the switch on)
    passthrough   bool   step 0: yaw/yawspeed are psi_super/yawdot_super untouched
    ff            rad/s  the feed-forward state after this tick (0 when ff_enable is off)

`debug_array()` gives the plan's section 2.6 layout for /bridge/yaw_servo
(std_msgs/Float32MultiArray): [active, psi_des, psi_out, yawspeed, range_m, target_age_s,
source], floats, angles in rad; see DEBUG_LAYOUT. `source` there is 0 (SUPER / fall-back),
1 (servo), 2 (close-range hold) or 3 (stale hold); `active` is 1.0 for 1, 2 and 3.

Parameters (YawServoParams; defaults are plan section 2.5)
----------------------------------------------------------
    enable                 False   fix 1. False = step 0 identity, bit for bit.
    ff_enable              False   fix 2. No effect unless enable.
    target_timeout         0.5 s   freshness, same as SUPER's kYawTargetTimeoutSec.
    future_tolerance       0.2 s   a target stamped further ahead than this is not fresh.
    max_rate               90      deg/s, the heading slew limit (step 5).
    ff_max_rate            60      deg/s, the feed-forward cap.
    ff_max_accel           90      deg/s^2, the feed-forward slew.
    min_range_m            2.5 m   below it: hold heading, feed-forward 0.
    min_range_hysteresis_m 0.3 m   the close-range hold is left only at >= min_range_m + this
                                   (round two; 0 = the round-one rule exactly). >= 0.
    stale_hold_s           0.5 s   hold the last aim point this long before falling back
                                   (round two; 0 = no hold, the round-one behaviour). >= 0.
    camera_offset_deg      0       deg, camera axis yaw vs the body x-axis, + = LEFT (above).
    ff_window              0.2 s   the averaging window.
    ff_step_reset_deg      5       deg, a psi_des step above this is judged as a jump.
    ff_step_consistency_s  0.25 s  how far back a jump looks for a similar step.
    fallback_ff_band_deg   2       deg, the fall-back passes SUPER's rate only inside it
                                   (and only when the slew clamp did not clip, step 6).
    aim_in_hold            True    False: in HOLD the servo stays inactive (fall-back to the
                                   frozen yaw) until the first TRACK after an engage (the
                                   plan's section 2.4 / 6 open question).

Parameters are validated when YawServoParams is constructed (ValueError/TypeError at
start-up, never in flight) and a YawServo snapshots them in its constructor: changing a
params object afterwards does not reach a running servo. Make a new YawServo instead.

`step()` never raises for any input value, and never returns a non-finite yaw or yawspeed
once it has been initialised (a finite heading or psi_super on the first engaged tick).
Non-finite or malformed inputs make the servo hold its last aim (source 3, if it was
active and within stale_hold_s) and then fall back (source 0); a non-finite psi_super in
fall-back holds psi_out; a non-finite yawdot_super gives yawspeed 0; a non-finite dt
freezes the tick.
"""
import math

__all__ = [
    'IDLE', 'TRACK', 'HOLD',
    'SOURCE_SUPER', 'SOURCE_SERVO', 'SOURCE_CLOSE_HOLD', 'SOURCE_STALE_HOLD',
    'ACCEPTED_FRAMES', 'DT_MAX', 'DEBUG_LAYOUT',
    'wrap_pi', 'YawServoParams', 'YawServoOutput', 'YawServo',
]

# The bridge's state names (px4_offboard_bridge.py IDLE/TRACK/HOLD), so it can pass its
# own `self.state` straight in. Anything else is treated as IDLE.
IDLE = 'IDLE'
TRACK = 'TRACK'
HOLD = 'HOLD'

SOURCE_SUPER = 0       # SUPER's heading (fall-back), or pass-through
SOURCE_SERVO = 1       # the bearing from the aircraft to the yaw target
SOURCE_CLOSE_HOLD = 2  # the servo holds its heading: target inside min_range_m (hysteresis)
SOURCE_STALE_HOLD = 3  # the servo holds its last aim point: no usable target, <= stale_hold_s

ACCEPTED_FRAMES = ('lidar_init', 'world')

# The slew and the feed-forward never integrate more than this per tick (s). A longer dt
# is a timing discontinuity for the feed-forward's average.
DT_MAX = 0.1

# /bridge/yaw_servo's layout. `source`: 0 SUPER / fall-back, 1 servo, 2 close-range hold,
# 3 stale hold. `active`: 1.0 for sources 1, 2 and 3.
DEBUG_LAYOUT = ('active', 'psi_des', 'psi_out', 'yawspeed', 'range_m', 'target_age_s',
                'source')

_PI = math.pi
_NAN = float('nan')
# The jump test (see the module docstring): an earlier step is "similar" to a step s if it
# has the same sign and a magnitude in [LO*|s|, HI*|s|].
_JUMP_RATIO_LO = 0.4
_JUMP_RATIO_HI = 2.5
# Bounds on memory whatever the tick rate.
_WINDOW_MAX_SAMPLES = 1000
_HIST_MAX = 256
# Time bookkeeping caps a single dt here, so the internal clock can never reach inf.
_DT_TIME_CAP = 1.0e3
# The stale hold's time comparison tolerance (s): 25 ticks of 0.02 s sum to 0.5 + 1e-16.
_HOLD_EPS = 1.0e-9


def wrap_pi(a):
    """Wrap a finite angle (rad) to (-pi, pi].

    A value already in (-pi, pi] is returned unchanged (the same float, bit for bit). -pi
    becomes +pi. A non-finite value is returned as it is; callers must not rely on it."""
    if -_PI < a <= _PI:
        return a
    if not math.isfinite(a):
        return a
    w = math.atan2(math.sin(a), math.cos(a))
    return _PI if w <= -_PI else w


def _to_float(v):
    """float(v), or NaN if v cannot be converted. Never raises."""
    if type(v) is float:
        return v
    try:
        return float(v)
    except Exception:  # an input sanitiser: any unconvertible value is "not a number"
        return _NAN


def _flag(v):
    """A boolean input. True/False as given; None and strings are False; a number is true
    iff finite and non-zero (so NaN is False, unlike bool(nan)); anything else by bool(),
    False if that raises."""
    if v is True or v is False:
        return v
    if v is None or isinstance(v, str):
        return False
    try:
        f = float(v)
    except Exception:  # an input sanitiser, see _to_float
        f = None
    if f is not None:
        return math.isfinite(f) and f != 0.0   # NaN and inf are False
    try:
        return bool(v)
    except Exception:  # an input sanitiser, see _to_float
        return False


def _clamp(x, lim):
    if x > lim:
        return lim
    if x < -lim:
        return -lim
    return x


def _slew(x, target, max_step):
    """Move x toward target by at most max_step (>= 0)."""
    d = target - x
    if d > max_step:
        return x + max_step
    if d < -max_step:
        return x - max_step
    return target


def _param_bool(name, v):
    if v is True or v is False:
        return v
    try:
        ok = v in (True, False)  # also 0/1 and numpy bools
    except Exception:
        ok = False
    if not ok or isinstance(v, str):
        raise TypeError(f'yaw servo parameter {name} must be a bool, got {v!r}')
    return bool(v)


def _param_num(name, v, lo, lo_open=False, hi=None):
    if isinstance(v, bool) or isinstance(v, str):
        raise TypeError(f'yaw servo parameter {name} must be a number, got {v!r}')
    try:
        f = float(v)
    except Exception:
        raise TypeError(f'yaw servo parameter {name} must be a number, got {v!r}') from None
    if not math.isfinite(f):
        raise ValueError(f'yaw servo parameter {name} must be finite, got {v!r}')
    if (f <= lo) if lo_open else (f < lo):
        raise ValueError(f'yaw servo parameter {name} must be {">" if lo_open else ">="} '
                         f'{lo}, got {f}')
    if hi is not None and f > hi:
        raise ValueError(f'yaw servo parameter {name} must be <= {hi}, got {f}')
    return f


class YawServoParams:
    """The servo's parameters, validated on construction. Units as named: seconds, metres,
    DEGREES and deg/s, deg/s^2 (converted to radians inside YawServo). Defaults are the
    plan's section 2.5; see the module docstring for each one's meaning."""

    __slots__ = ('enable', 'ff_enable', 'target_timeout', 'max_rate', 'ff_max_rate',
                 'ff_max_accel', 'min_range_m', 'camera_offset_deg', 'future_tolerance',
                 'ff_window', 'ff_step_reset_deg', 'ff_step_consistency_s',
                 'fallback_ff_band_deg', 'aim_in_hold', 'min_range_hysteresis_m',
                 'stale_hold_s')

    def __init__(self, enable=False, ff_enable=False, target_timeout=0.5, max_rate=90.0,
                 ff_max_rate=60.0, ff_max_accel=90.0, min_range_m=2.5,
                 camera_offset_deg=0.0, future_tolerance=0.2, ff_window=0.2,
                 ff_step_reset_deg=5.0, ff_step_consistency_s=0.25,
                 fallback_ff_band_deg=2.0, aim_in_hold=True, min_range_hysteresis_m=0.3,
                 stale_hold_s=0.5):
        self.enable = _param_bool('enable', enable)
        self.ff_enable = _param_bool('ff_enable', ff_enable)
        self.aim_in_hold = _param_bool('aim_in_hold', aim_in_hold)
        self.target_timeout = _param_num('target_timeout', target_timeout, 0.0, lo_open=True)
        self.max_rate = _param_num('max_rate', max_rate, 0.0, lo_open=True)
        self.ff_max_rate = _param_num('ff_max_rate', ff_max_rate, 0.0)
        self.ff_max_accel = _param_num('ff_max_accel', ff_max_accel, 0.0)
        self.min_range_m = _param_num('min_range_m', min_range_m, 0.0)
        self.min_range_hysteresis_m = _param_num('min_range_hysteresis_m',
                                                 min_range_hysteresis_m, 0.0)
        self.stale_hold_s = _param_num('stale_hold_s', stale_hold_s, 0.0)
        self.camera_offset_deg = _param_num('camera_offset_deg', camera_offset_deg,
                                            -180.0, hi=180.0)
        self.future_tolerance = _param_num('future_tolerance', future_tolerance, 0.0)
        self.ff_window = _param_num('ff_window', ff_window, 0.0, lo_open=True)
        self.ff_step_reset_deg = _param_num('ff_step_reset_deg', ff_step_reset_deg, 0.0,
                                            lo_open=True)
        self.ff_step_consistency_s = _param_num('ff_step_consistency_s',
                                                ff_step_consistency_s, 0.0, lo_open=True)
        self.fallback_ff_band_deg = _param_num('fallback_ff_band_deg',
                                               fallback_ff_band_deg, 0.0)

    def __repr__(self):
        return ('YawServoParams(' + ', '.join(f'{k}={getattr(self, k)!r}'
                                              for k in self.__slots__) + ')')


class YawServoOutput:
    """One tick's result. Field meanings, units and frames: the module docstring."""

    __slots__ = ('yaw', 'yawspeed', 'active', 'psi_des', 'psi_out', 'range_m',
                 'target_age_s', 'source', 'slew_binding', 'bad_frame', 'passthrough',
                 'ff')

    def __init__(self, yaw, yawspeed, active, psi_des, psi_out, range_m, target_age_s,
                 source, slew_binding, bad_frame, passthrough, ff):
        self.yaw = yaw
        self.yawspeed = yawspeed
        self.active = active
        self.psi_des = psi_des
        self.psi_out = psi_out
        self.range_m = range_m
        self.target_age_s = target_age_s
        self.source = source
        self.slew_binding = slew_binding
        self.bad_frame = bad_frame
        self.passthrough = passthrough
        self.ff = ff

    def debug_array(self):
        """[active, psi_des, psi_out, yawspeed, range_m, target_age_s, source] as floats
        (plan section 2.6, DEBUG_LAYOUT). Angles in rad, rates in rad/s."""
        return [1.0 if self.active else 0.0, _to_float(self.psi_des),
                _to_float(self.psi_out), _to_float(self.yawspeed),
                _to_float(self.range_m), _to_float(self.target_age_s),
                float(self.source)]

    def __repr__(self):
        return ('YawServoOutput(' + ', '.join(f'{k}={getattr(self, k)!r}'
                                              for k in self.__slots__) + ')')


def _passthrough(psi_super, yawdot_super):
    return YawServoOutput(yaw=psi_super, yawspeed=yawdot_super, active=False,
                          psi_des=_NAN, psi_out=psi_super, range_m=_NAN,
                          target_age_s=_NAN, source=SOURCE_SUPER, slew_binding=False,
                          bad_frame=False, passthrough=True, ff=0.0)


class YawServo:
    """The stateful servo. One `step()` per bridge tick; `reset()` forgets everything."""

    def __init__(self, params=None):
        if params is None:
            params = YawServoParams()
        if not isinstance(params, YawServoParams):
            raise TypeError('YawServo needs a YawServoParams')
        self.params = params
        # A snapshot, in radians: editing `params` later does not reach this servo.
        self._enable = params.enable
        self._ff_enable = params.ff_enable
        self._aim_in_hold = params.aim_in_hold
        self._timeout = params.target_timeout
        self._future_tol = params.future_tolerance
        self._max_rate = math.radians(params.max_rate)
        self._ff_max = math.radians(params.ff_max_rate)
        self._ff_acc = math.radians(params.ff_max_accel)
        self._min_range = params.min_range_m
        self._range_exit = params.min_range_m + params.min_range_hysteresis_m
        self._hold_max = params.stale_hold_s
        self._offset = math.radians(params.camera_offset_deg)
        self._win_len = params.ff_window
        self._jump = math.radians(params.ff_step_reset_deg)
        self._consistency = params.ff_step_consistency_s
        self._band = math.radians(params.fallback_ff_band_deg)
        self._dirty = True
        self.reset()

    def reset(self):
        """Back to the state of a fresh servo: uninitialised, feed-forward 0."""
        self._psi_out = None          # None: not initialised (step 5's first tick)
        self._ff = 0.0                # the slewed feed-forward, rad/s FRD
        self._win = []                # [(dt, step)] the feed-forward's average
        self._hist = []               # [(t, step)] steps >= 0.4*jump, for the jump test
        self._t = 0.0                 # internal time: the sum of sane dts, never a clock
        self._psi_des_prev = None     # psi_des of the last source-1 tick with time
        self._psi_des_servo = None    # psi_des of the previous tick if the servo owned it
                                      # (source 1/2: that tick's aim; 3: the held last aim)
        self._prev_source = SOURCE_SUPER
        self._seen_track = False      # a TRACK since the engage (aim_in_hold False)
        self._close = False           # the close-range latch (step 3's hysteresis)
        self._hold_t = 0.0            # accumulated dt since the last usable tick (step 3b)
        self._dirty = False

    # ---- the tick -----------------------------------------------------------

    def step(self, *, dt, state, engaged, psi_super, yawdot_super,
             vehicle_heading, vehicle_x, vehicle_y, vehicle_xy_valid, target):
        """One tick. Keyword-only, so that no call site can swap two arguments.

        dt               s     time since the previous tick (bag or bridge clock, never
                               read here). <= 0, NaN or inf: nothing moves this tick.
        state            the bridge state: IDLE / TRACK / HOLD (the strings above).
        engaged          bool  armed AND in Offboard.
        psi_super        rad   FRD. The yaw the bridge would send today (-msg.yaw wrapped;
                               in HOLD the frozen yaw; in IDLE the latch = heading).
        yawdot_super     rad/s FRD. Likewise (-msg.yaw_dot; 0 in HOLD and IDLE).
        vehicle_heading  rad   FRD, vehicle_local_position.heading.
        vehicle_x/_y     m     NED, vehicle_local_position.x / .y.
        vehicle_xy_valid bool  vehicle_local_position.xy_valid.
        target           None if nothing was ever received, else a 4-sequence
                         (x_flu, y_flu, age_s, frame_id): x, y in m in FLU lidar_init as
                         on the topic (NOT converted by the caller), age_s = now - stamp
                         in s (negative = stamped in the future), frame_id the header's.

        Returns a YawServoOutput. Never raises for any input value."""
        # -- step 0: identity ---------------------------------------------------
        if not self._enable:
            if self._dirty:
                self.reset()
            return _passthrough(psi_super, yawdot_super)
        is_track = isinstance(state, str) and state == TRACK
        is_hold = isinstance(state, str) and state == HOLD
        if not (is_track or is_hold) or not _flag(engaged):
            if self._dirty:
                self.reset()
            return _passthrough(psi_super, yawdot_super)

        self._dirty = True
        if is_track:
            self._seen_track = True

        dt_s = _to_float(dt)
        if not (dt_s > 0.0 and math.isfinite(dt_s)):
            dt_s = 0.0
        dt_s = min(dt_s, _DT_TIME_CAP)
        dt_c = min(dt_s, DT_MAX)

        psi_s = _to_float(psi_super)
        ydot_s = _to_float(yawdot_super)
        heading = _to_float(vehicle_heading)
        heading_ok = math.isfinite(heading)
        psi_s_ok = math.isfinite(psi_s)

        # -- step 5's initialisation: from the vehicle's heading ----------------
        if self._psi_out is None:
            if heading_ok:
                self._psi_out = wrap_pi(heading)
            elif psi_s_ok:
                self._psi_out = wrap_pi(psi_s)
            else:
                return _passthrough(psi_super, yawdot_super)
        self._t += dt_s

        # -- step 1: is there a usable target? ----------------------------------
        age = _NAN
        rng = _NAN
        bearing = _NAN
        bad_frame = False
        usable = False
        if target is not None:
            try:
                tx, ty, tage, tframe = target
            except Exception:  # not a 4-sequence: no usable target this tick
                tx = ty = tage = _NAN
                tframe = None
            tx = _to_float(tx)
            ty = _to_float(ty)
            tage = _to_float(tage)
            if math.isfinite(tage):
                age = tage
            fresh = -self._future_tol <= age <= self._timeout   # False for NaN
            frame_ok = isinstance(tframe, str) and tframe in ACCEPTED_FRAMES
            if fresh and not frame_ok:
                bad_frame = True
            if fresh and frame_ok:
                vx = _to_float(vehicle_x)
                vy = _to_float(vehicle_y)
                if (_flag(vehicle_xy_valid) and heading_ok
                        and math.isfinite(vx) and math.isfinite(vy)
                        and math.isfinite(tx) and math.isfinite(ty)):
                    # FLU -> NED: x stays, y negates. Then the bearing from the aircraft.
                    dx = tx - vx
                    dy = -ty - vy
                    r = math.hypot(dx, dy)
                    if math.isfinite(r):
                        rng = r
                        bearing = math.atan2(dy, dx)
                        usable = True

        servo = usable and (is_track or self._aim_in_hold or self._seen_track)

        # -- steps 2-4: the desired heading --------------------------------------
        prev_src = self._prev_source
        hold = False
        if not servo and prev_src != SOURCE_SUPER and self._hold_max > 0.0 \
                and self._psi_des_servo is not None:
            # Step 3b: the servo owned the previous tick. Hold its last aim while the time
            # since the last usable tick is within stale_hold_s.
            t_h = self._hold_t + dt_s
            if t_h <= self._hold_max + _HOLD_EPS:
                hold = True
                self._hold_t = t_h
        if servo:
            self._hold_t = 0.0
            # Step 3's latch: in at < min_range_m, out at >= min_range_m + hysteresis.
            if self._close:
                close = rng < self._range_exit
            else:
                close = rng < self._min_range
            self._close = close
            if close:
                source = SOURCE_CLOSE_HOLD
                if prev_src != SOURCE_SUPER and self._psi_des_servo is not None:
                    psi_des = self._psi_des_servo
                else:
                    psi_des = self._psi_out
            else:
                source = SOURCE_SERVO
                if self._offset == 0.0:
                    psi_des = wrap_pi(bearing)          # atan2 is in range: no-op but -pi
                else:
                    psi_des = wrap_pi(bearing + self._offset)
            self._psi_des_servo = psi_des
        elif hold:
            # Step 3b: the last usable tick's aim (self._psi_des_servo, left unchanged). The
            # close-range latch is kept.
            source = SOURCE_STALE_HOLD
            psi_des = self._psi_des_servo
        else:
            source = SOURCE_SUPER
            psi_des = wrap_pi(psi_s) if psi_s_ok else self._psi_out
            self._psi_des_servo = None
            self._close = False       # a re-acquired target is judged afresh
            self._hold_t = 0.0

        # -- step 5: the slew limit ------------------------------------------------
        prev_out = self._psi_out
        err = wrap_pi(psi_des - prev_out)
        lim = self._max_rate * dt_c
        binding = False
        clipped = False               # the clamp clipped, also with dt_c 0 (step 6, F-a)
        if err > lim:
            new_out = wrap_pi(prev_out + lim)
            clipped = True
            binding = dt_c > 0.0
        elif err < -lim:
            new_out = wrap_pi(prev_out - lim)
            clipped = True
            binding = dt_c > 0.0
        else:
            new_out = psi_des
        self._psi_out = new_out

        # -- step 6: yawspeed -------------------------------------------------------
        if not self._ff_enable:
            if source == SOURCE_SUPER:
                yawspeed = self._fallback_rate(psi_s, ydot_s, new_out, clipped)
            else:
                yawspeed = 0.0
        elif source == SOURCE_SERVO:
            self._sample_ff(psi_des, dt_s, prev_src)
            want = _clamp(self._win_avg(), self._ff_max)
            self._ff = _slew(self._ff, want, self._ff_acc * dt_c)
            yawspeed = self._ff
        elif source == SOURCE_CLOSE_HOLD:
            self._clear_ff_average()
            self._ff = 0.0
            yawspeed = 0.0
        elif source == SOURCE_STALE_HOLD:
            # The feed-forward decays at ff_max_accel; never SUPER's rate.
            self._clear_ff_average()
            self._ff = _slew(self._ff, 0.0, self._ff_acc * dt_c)
            yawspeed = self._ff
        else:
            # Fall-back: the feed-forward decays at ff_max_accel; SUPER's rate is used
            # only once it has reached exactly 0. Never the sum of the two.
            self._clear_ff_average()
            self._ff = _slew(self._ff, 0.0, self._ff_acc * dt_c)
            if self._ff != 0.0:
                yawspeed = self._ff
            else:
                yawspeed = self._fallback_rate(psi_s, ydot_s, new_out, clipped)

        self._prev_source = source
        if source != SOURCE_SERVO:
            self._psi_des_prev = None

        return YawServoOutput(yaw=new_out, yawspeed=yawspeed,
                              active=source != SOURCE_SUPER, psi_des=psi_des,
                              psi_out=new_out, range_m=rng, target_age_s=age,
                              source=source, slew_binding=binding, bad_frame=bad_frame,
                              passthrough=False, ff=self._ff)

    # ---- helpers ------------------------------------------------------------------

    def _fallback_rate(self, psi_s, ydot_s, psi_out, clipped):
        """Fall-back yawspeed (step 6, F-a): SUPER's rate, clamped to +-max_rate, only on a
        tick where the slew clamp did not clip and our heading is within the band of SUPER's
        heading; else 0. So it can never exceed the heading's own slew limit."""
        if (not clipped and math.isfinite(psi_s) and math.isfinite(ydot_s)
                and abs(wrap_pi(psi_s - psi_out)) < self._band):
            return _clamp(ydot_s, self._max_rate)
        return 0.0

    def _clear_ff_average(self):
        self._win = []
        self._hist = []

    def _sample_ff(self, psi_des, dt_s, prev_src):
        """Feed one source-1 tick's psi_des into the feed-forward's average."""
        if prev_src != SOURCE_SERVO or self._psi_des_prev is None or dt_s > DT_MAX:
            # (Re)entry into source 1, or a timing discontinuity: no previous psi_des to
            # difference against. Start a fresh average from here.
            self._clear_ff_average()
            self._psi_des_prev = psi_des
            return
        if dt_s <= 0.0:
            return  # no time passed: any step is judged on the next tick with time
        step = wrap_pi(psi_des - self._psi_des_prev)
        self._psi_des_prev = psi_des

        cut = self._t - self._consistency
        if self._hist and self._hist[0][0] < cut:
            self._hist = [h for h in self._hist if h[0] >= cut]
        jump = self._is_jump(step)
        if abs(step) >= _JUMP_RATIO_LO * self._jump:
            self._hist.append((self._t, step))
            if len(self._hist) > _HIST_MAX:
                del self._hist[:len(self._hist) - _HIST_MAX]

        if jump:
            self._win = []   # "resets the average to 0"; the jump is not a sample
            return
        win = self._win
        win.append((dt_s, step))
        total = 0.0
        for d, _ in win:
            total += d
        while len(win) > 1 and total - win[0][0] >= self._win_len - 1e-12:
            total -= win.pop(0)[0]
        if len(win) > _WINDOW_MAX_SAMPLES:
            del win[:len(win) - _WINDOW_MAX_SAMPLES]

    def _is_jump(self, step):
        a = abs(step)
        if not a > self._jump:
            return False
        lo = _JUMP_RATIO_LO * a
        hi = _JUMP_RATIO_HI * a
        pos = step > 0.0
        for _, s in self._hist:
            if (s > 0.0) == pos and lo <= abs(s) <= hi:
                return False
        return True

    def _win_avg(self):
        if not self._win:
            return 0.0
        tot_t = 0.0
        tot_s = 0.0
        for d, s in self._win:
            tot_t += d
            tot_s += s
        return tot_s / max(tot_t, self._win_len)
