"""The braking envelope and the speed-scaled reach (2026-09-27, "Option A"). No ROS in it.

WHY
---
The proximity veto's 1.5 m and the 3-4 m standoff were sized for about 1 m/s. max_vel is now 3.0
and max_acc 2.0, and the 2026-09-26 flights closed on the walker at up to 1.87 and 2.03 m/s in
their sprints (flight 1 reached 2.57 m from a planned ~3.4 m).

Read from those two bags before any of this was written:
  * SUPER's reference never came closer to its goal than ~1.2 x v^2/(2 max_acc): every plan ends
    at rest (goal_vel_en false), so the planner already keeps its own braking distance to the
    goal. What is NOT covered is the aircraft running past the reference (0.8 m flown with no
    acceleration feed-forward) and the aircraft carrying on along its CURRENT velocity when the
    goal jumps direction (the walker turns, the goal flips to the far side, the fence clamps).
  * The closing speed (aircraft toward a person, less the person's speed away) is ~0-0.3 m/s in
    a steady follow and peaks in the sprints. That is the signal: it grows the reach exactly when
    the aircraft is running at someone, and leaves a steady follow alone.

WHAT
----
  stop(v)      = v * react_s + v^2 / (2 brake_mps2)
                 how far the aircraft goes before it stops, from the moment a stop is due;
  track(v)     = min(track_s * v, track_max_m)
                 how far it runs past its reference;
  reach(vc,va) = d_min + stop(vc) + track(va)
                 the closest-approach floor needed at closing speed vc and ground speed va;
  extra(vc,va) = clamp(reach - base_floor_m, 0, max_extra_m)
                 what follow_planner adds to its standoff band and floor, and what the descent
                 guard adds to its radii.

d_min is the proximity veto radius (1.5), base_floor_m is follow_planner's min_standoff_m (2.5).
At the defaults (react 0.5 s, brake 2.0, track 0.4 s = no feed-forward) extra is 0 up to ~0.89 m/s
of closing (at a 0.9 m/s ground speed), so a steady follow (closing 0-0.3 m/s) is today's
behaviour; it reaches the 1.5 m cap at ~1.84 m/s.

The veto's BRAKING ENVELOPE: the aircraft's path if a stop were commanded now -- straight along
its velocity for react_s, then braking at brake_mps2, plus the tracking overshoot spread along it
-- sampled in time against each person's forecast at the same instant. A person within the veto
radius of that path at the same time is a violation. The tracking overshoot is also added along
the approach past the goal. Both act ALONG the direction of travel only; the lateral radius stays
the veto radius. At zero speed the envelope is the aircraft's own position over react_s, which is
a subset of what the segment check already tests: nothing new can veto a hovering aircraft.

HYSTERESIS (ReachFilter): the extra grows at once (the safe direction), is held for hold_s, then
shrinks at most release_mps -- slower than a walk, so a shrinking standoff never pulls the
aircraft at a person faster than walking pace, and noise cannot make the goal breathe.
"""
import math
from collections import deque


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


class SpeedParams:
    """The shared numbers. follow_planner's speed_standoff.hpp implements the same formula; the
    two unit tests assert the same table (test_brake_envelope.py / test_speed_standoff.cpp)."""

    def __init__(self, d_min_m=1.5, react_s=0.5, brake_mps2=2.0, track_s=0.4, track_max_m=2.0,
                 base_floor_m=2.5, max_extra_m=1.5, hold_s=1.0, release_mps=0.5):
        values = dict(d_min_m=d_min_m, react_s=react_s, brake_mps2=brake_mps2, track_s=track_s,
                      track_max_m=track_max_m, base_floor_m=base_floor_m,
                      max_extra_m=max_extra_m, hold_s=hold_s, release_mps=release_mps)
        for name, v in values.items():
            if not _finite(v) or v < 0.0:
                raise ValueError('speed parameter %s must be finite and >= 0, got %r' % (name, v))
        if brake_mps2 <= 0.0:
            raise ValueError('speed_brake_mps2 must be > 0, got %r' % brake_mps2)
        if release_mps <= 0.0:
            raise ValueError('speed_release_mps must be > 0 (0 would never shrink), got %r'
                             % release_mps)
        self.__dict__.update(values)


def stop_distance(v, p):
    v = max(0.0, v)
    return v * p.react_s + v * v / (2.0 * p.brake_mps2)


def track_margin(v, p):
    return min(max(0.0, v) * p.track_s, p.track_max_m)


def reach(v_close, v_ground, p):
    return p.d_min_m + stop_distance(v_close, p) + track_margin(v_ground, p)


def extra(v_close, v_ground, p):
    return min(max(reach(v_close, v_ground, p) - p.base_floor_m, 0.0), p.max_extra_m)


def closing_speed(ac_xy, ac_v, person_xy, person_vs, coasting=False):
    """Aircraft speed toward the person, less the person's speed AWAY from the aircraft, >= 0.

    person_vs is the person's recent velocity reports; the speed away is the SMALLEST of them
    along the line of sight. So a report that the person slowed, stopped or turned counts at once,
    and a glitch that says they are running away counts only if it lasts every report in the
    window. (The 2026-09-26 glitches reached 2.82 m/s, one of them a 3-report burst: a median of 5
    would have passed it.) A person walking TOWARD the aircraft adds to the closing speed. While a
    track is coasting its velocity is a forecast, not a measurement, so it is ignored: the person
    is taken as stopped, which is the conservative reading. No reports: taken as stopped.
    """
    dx, dy = person_xy[0] - ac_xy[0], person_xy[1] - ac_xy[1]
    r = math.hypot(dx, dy)
    if r < 1e-3:
        return math.hypot(ac_v[0], ac_v[1])
    ux, uy = dx / r, dy / r
    toward = ac_v[0] * ux + ac_v[1] * uy
    if coasting or not person_vs:
        away = 0.0
    else:
        away = min(v[0] * ux + v[1] * uy for v in person_vs)
    return max(0.0, toward - away)


class VelocityEstimator:
    """Horizontal velocity from successive poses. /lidar_slam/odom's twist is all zeros on this
    stack (read from both 2026-09-26 bags), so the pose is differenced: newest sample against the
    newest one at least window_s older. Measured against EKF2 on those bags: 0.08-0.10 m/s median.
    None when there is no such pair within max_age_s -- the caller then uses zero speed, which
    is today's behaviour; a stale odometry is the ODOM veto's business, not this one's."""

    def __init__(self, window_s=0.2, max_age_s=0.6):
        self.window_s = window_s
        self.max_age_s = max_age_s
        self._s = deque()

    def add(self, t, x, y):
        if not _finite(t, x, y):
            return
        if self._s and t <= self._s[-1][0]:
            if t < self._s[-1][0] - 1.0:
                self._s.clear()        # the clock went backwards: start again
            else:
                return                 # duplicate or reordered sample
        self._s.append((t, x, y))
        while self._s and t - self._s[0][0] > self.max_age_s + self.window_s:
            self._s.popleft()

    def velocity(self):
        if len(self._s) < 2:
            return None
        t1, x1, y1 = self._s[-1]
        best = None
        for t0, x0, y0 in reversed(self._s):
            if t1 - t0 >= self.window_s:
                best = (t0, x0, y0)
                break
        if best is None or t1 - best[0] > self.max_age_s:
            return None
        dt = t1 - best[0]
        return ((x1 - best[1]) / dt, (y1 - best[2]) / dt)


class PersonVelocities:
    """Each track's last n velocity reports, for closing_speed's conservative minimum."""

    def __init__(self, n=5, forget_s=1.0):
        self.n = n
        self.forget_s = forget_s
        self._tracks = {}

    def add(self, now, track_id, vx, vy):
        for k in [k for k, (_, t) in self._tracks.items() if now - t > self.forget_s]:
            del self._tracks[k]         # first: a track id seen again after a gap starts afresh
        if not _finite(vx, vy):
            return
        d, _ = self._tracks.get(track_id, (deque(maxlen=self.n), now))
        d.append((vx, vy))
        self._tracks[track_id] = (d, now)

    def get(self, track_id):
        """The reports held for this track, oldest first ([] if none)."""
        entry = self._tracks.get(track_id)
        return list(entry[0]) if entry is not None else []


class ReachFilter:
    """Grow at once, hold the peak hold_s, then shrink at most release_mps."""

    def __init__(self, p):
        self.p = p
        self.value = 0.0
        self._peak_t = None
        self._last_t = None

    def reset(self):
        self.value = 0.0
        self._peak_t = None
        self._last_t = None

    def update(self, now, want):
        want = max(0.0, want)
        if self._last_t is None or now < self._last_t:
            self._last_t = now
        dt = now - self._last_t
        self._last_t = now
        if want >= self.value:
            self.value = want
            self._peak_t = now
        elif self._peak_t is not None and now - self._peak_t < self.p.hold_s:
            pass
        else:
            self.value = max(want, self.value - self.p.release_mps * dt)
        return self.value


def braking_positions(ax, ay, vx, vy, p, n=24):
    """[(tau, x, y)] along the stop the aircraft would make if one were commanded now, from tau 0
    to the end of the stop, with the tracking overshoot spread along it. Straight along the
    current velocity: SUPER can curve, but the stop is along the way it is already going."""
    s = math.hypot(vx, vy)
    if s < 1e-6:
        return [(p.react_s * k / max(1, n - 1), ax, ay) for k in range(n)]
    ux, uy = vx / s, vy / s
    t_stop = p.react_s + s / p.brake_mps2
    e = track_margin(s, p)
    out = []
    for k in range(n):
        tau = t_stop * k / (n - 1)
        if tau <= p.react_s:
            d = s * tau
        else:
            tb = tau - p.react_s
            d = s * p.react_s + s * tb - 0.5 * p.brake_mps2 * tb * tb
        d += e * tau / t_stop
        out.append((tau, ax + ux * d, ay + uy * d))
    return out


def person_at(points, tau):
    """A person's (x, y, z) at tau from a list of (dt, x, y, z) sorted by dt (dt 0 = now).
    Linear between forecast points, held at the last one beyond the forecast."""
    if tau <= points[0][0]:
        return points[0][1:]
    for (t0, x0, y0, z0), (t1, x1, y1, z1) in zip(points, points[1:]):
        if tau <= t1:
            f = (tau - t0) / (t1 - t0) if t1 > t0 else 1.0
            return (x0 + f * (x1 - x0), y0 + f * (y1 - y0), z0 + f * (z1 - z0))
    return points[-1][1:]


def envelope_violation(path, persons, radius):
    """The worst (distance, track_id, tau) of any person inside `radius` of the braking path at
    the same instant, or None. persons: [(track_id, [(dt, x, y, z), ...])], dt sorted."""
    worst = None
    for track_id, points in persons:
        if not points:
            continue
        for tau, x, y in path:
            px, py, _ = person_at(points, tau)
            d = math.hypot(px - x, py - y)
            if d < radius and (worst is None or d < worst[0]):
                worst = (d, track_id, tau)
    return worst


def extend_segment(ax, ay, bx, by, e):
    """The goal end of the path a->b pushed e further along it (the tracking overshoot)."""
    dx, dy = bx - ax, by - ay
    n = math.hypot(dx, dy)
    if n < 1e-6 or e <= 0.0:
        return bx, by
    return bx + dx / n * e, by + dy / n * e
