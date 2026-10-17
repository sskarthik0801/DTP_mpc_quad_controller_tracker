"""The terrain offset command_mux adds to the follow height, as a state machine with no ROS in it.

WHAT IT IS
----------
terrain_estimator_node publishes the ground height under the aircraft and under the follow goal
(terrain_map.py), and the HIGHER of the two on /dtp/terrain. This turns that into one number,
the OFFSET: how far the local ground sits above (+) or below (-) the ground at the takeoff spot,
`ref_z` in lidar_init (-0.15: the LiDAR is ~0.15 m up when the stack starts on the ground).
command_mux does its height logic in a frame shifted down by the offset -- height above the
local ground, in lidar_init units -- and shifts the result back up (command_mux_node.py,
"TERRAIN-RELATIVE HEIGHT"). With the offset at 0 that is exactly the absolute behaviour.

THE HIGHER OF THE TWO SAMPLES
-----------------------------
Uphill, the ground under the goal rises first, so the aircraft climbs before it gets there.
Downhill, the ground under the aircraft is the higher one, so it descends only once BOTH the
aircraft and its goal are over the lower ground. A ditch has to be wider than the aircraft-goal
spacing plus the patch before the aircraft follows it down at all. Climb early, descend late.

WHAT BOUNDS IT
--------------
  RATE     while PX4 flies the mux's goals (armed + Offboard) the offset moves at most rate_up_mps
           toward higher ground and rate_down_mps toward lower ground, so no estimate -- right or
           wrong -- can step the goal. Climbing is allowed to be faster than descending.
  SNAP     while PX4 does NOT fly our goals (on the ground, or the pilot flying Position) it
           takes the estimate at once. Nothing is being flown, and engaging Offboard over a slope
           then starts from the right height instead of ramping to it.
  CLAMP    the offset stays within [-max_down_m, +max_up_m] of the takeoff ground. The defaults
           keep the follow goal inside ROG-Map's virtual ground (-1.0) and ceiling (3.5), which
           are absolute in lidar_init: past them SUPER drops the goal.
  DEADBAND a change smaller than deadband_m is not taken, so the goal does not dither by
           centimetres on estimate noise.
  HOLD     no estimate ever ('none'), a NaN -- the estimator is up but has not seen the ground
           there ('unknown') -- or one older than timeout_s ('stale'): the offset HOLDS where it
           is. It never decays to 0 -- on a slope, 0 is the wrong answer too, and a jump to it is
           the failure this exists to prevent.
"""
import math


class TerrainOffset:
    """See the module docstring. Times are seconds on any monotonic clock the caller owns."""

    def __init__(self, ref_z, max_up_m, max_down_m, rate_up_mps, rate_down_mps, deadband_m,
                 timeout_s):
        for name, value in (('max_up_m', max_up_m), ('max_down_m', max_down_m),
                            ('rate_up_mps', rate_up_mps), ('rate_down_mps', rate_down_mps),
                            ('deadband_m', deadband_m), ('timeout_s', timeout_s)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError('terrain %s must be finite and >= 0, got %r' % (name, value))
        if not math.isfinite(ref_z):
            raise ValueError('terrain ref_z must be finite, got %r' % ref_z)
        self.ref_z = float(ref_z)
        self.max_up_m = float(max_up_m)
        self.max_down_m = float(max_down_m)
        self.rate_up_mps = float(rate_up_mps)
        self.rate_down_mps = float(rate_down_mps)
        self.deadband_m = float(deadband_m)
        self.timeout_s = float(timeout_s)
        self.offset = 0.0
        self.target = None        # the clamped offset the last fresh estimate asked for
        self.status = 'none'      # none | unknown | stale | ok | clamped
        self._last_now = None

    def update(self, now, ground_z, arrival, engaged):
        """Advance the offset to `now`. ground_z: the latest estimate (may be NaN or None),
        arrival: when it arrived (None if never), engaged: PX4 is flying the mux's goals.
        Returns the offset."""
        dt = 0.0 if self._last_now is None else min(max(now - self._last_now, 0.0), 0.5)
        self._last_now = now
        if arrival is None or ground_z is None:
            self.status = 'none'
            return self.offset
        if (now - arrival) > self.timeout_s:
            self.status = 'stale'
            return self.offset
        if not math.isfinite(ground_z):
            self.status = 'unknown'
            return self.offset
        raw = ground_z - self.ref_z
        target = min(max(raw, -self.max_down_m), self.max_up_m)
        self.status = 'clamped' if target != raw else 'ok'
        self.target = target
        if not engaged:
            self.offset = target
            return self.offset
        err = target - self.offset
        if abs(err) <= self.deadband_m:
            return self.offset
        step = (self.rate_up_mps if err > 0.0 else self.rate_down_mps) * dt
        self.offset += max(-step, min(step, err))
        return self.offset
