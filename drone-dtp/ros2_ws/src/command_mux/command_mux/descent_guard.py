"""The descent guard (R-F9, goal-level half), as a small state machine with no ROS in it.

THE RULE
--------
Never command a goal below the drone while a person is close. "Close" is horizontal distance
under `radius_m` from the drone OR from the goal, against EVERY person on /dtp/targets, not only
the followed one.

WHY IT LATCHES (changed 2026-09-17, user decision -- "option B")
----------------------------------------------------------------
The first version raised each goal to the drone's CURRENT altitude, re-decided on every tick.
In the 2026-09-15 20:08 flight that made the follow goal's altitude flap: follow_planner puts the
goal 2.77-3.07 m from the walker, right on the 3.00 m radius, so 32 of 52 goals were raised to the
aircraft's altitude (0.72-1.20 m) and 20 stayed at 0.50 m, alternating every 0.1-0.5 s. SUPER
replanned for each flip, commanded +/-0.2 m/s of vertical speed, and spent the 3-D max_vel on
bobbing instead of following. Re-reading the current altitude also ratchets the goal DOWN with
the drone: after a dip under a branch the goal is left at the bottom of the dip.

So the guard now keeps a FLOOR:

  TRIP     goal below the drone and a person within radius_m -> floor = the drone's altitude.
  ACTIVE   every goal's z is raised to at least the floor. The floor is not re-read each tick:
           it does not follow the drone down (a dip under a branch climbs back afterwards) and it
           does not creep upward on altitude-hold error (the aircraft sat +0.07..+0.22 m above its
           command in the 2026-09-15 flight). It is raised only when the drone is more than
           raise_margin_m above it -- a pass over an obstacle -- because the rule is "not below
           the drone" and that is a real altitude change. A climb made by the mux's own HOLD is
           NOT one: see below.
  RELEASE  nobody within radius_m + release_margin_m of the drone or the goal for
           release_dwell_s -> the floor is dropped. The same shape as the proximity veto's
           release (margin 0.5 m, dwell 1.0 s), so a follow at a 3-4 m standoff stays latched.
  RESET    the node calls reset() on disarm, so a floor from one flight never carries into the
           next.

A HOLD'S CLIMB DOES NOT MOVE THE FLOOR (changed 2026-09-21, user decision)
--------------------------------------------------------------------------
A mux HOLD commands the drone's altitude + hold_climb_m (0.75 m). The guard used to treat that
climb like any other and raise the floor to it, and a follow at a 3-4 m standoff never releases
the floor, so ONE HOLD lifted the follow height for the rest of the flight. In the 2026-09-21
18:41 flight a 5 s HOLD|NO_SOURCE took the floor from 1.46 m to 2.10 m, and on a field that
slopes away the aircraft ended up 3.4-3.7 m above the ground, with the walker out of the bottom
of the camera frame.

So the node reports each HOLD with hold_started(z at HOLD entry) and hold_ended(). From a HOLD's
start until the drone is back within raise_margin_m of that altitude, the guard sees the drone
at min(its altitude, the altitude at HOLD entry), for BOTH the trip and the raise. The floor and
any new latch are where they would have been without the HOLD. Two HOLDs before the drone gets
back down keep the LOWER entry altitude. The release clears the floor but keeps this, so a latch
right after a release does not land on the climb either. reset() (disarm) clears it.

The node reports a HOLD only while PX4 flies its goals (armed + Offboard), and calls clear_hold()
whenever PX4 does not. A HOLD on the ground or under the pilot commands nothing, and registering
its altitude (near the ground) would have the guard see the drone at ground level all flight --
it would never trip again. The 2026-09-21 flight had exactly that sequence: HOLD on the ground,
takeoff in Position, Offboard engaged mid-HOLD.

The cost: after a HOLD, a goal can sit up to hold_climb_m + raise_margin_m below the drone with a
person close, and the drone comes back down while they are there. That includes a HOLD raised by
the proximity veto, where the climb was taken to get away from someone. The proximity veto still
guards the path itself.

WHAT THIS COSTS, STATED
-----------------------
Between floor and floor + raise_margin_m, a goal can sit up to raise_margin_m BELOW the drone's
current altitude with a person close; the first version allowed none. That tolerance is the
price of not ratcheting the floor upward on altitude-hold noise. It is sized above the largest
tracking error measured in flight (0.215 m) and is a parameter.

SPEED-SCALED RADII (2026-09-27, off unless the node's brake_envelope_enable)
-------------------------------------------------------------------------
The node may pass extra_radius_m to apply(): the same "extra" follow_planner adds to its standoff
band when the aircraft is closing on a person (command_mux/brake_envelope.py). It is added to the
trip radius AND the release radius alike, so the invariant this guard was built on -- the release
radius is at least the nominal standoff, so a follow stays latched -- still holds when the
standoff grows. Without it a sprint's longer standoff could release the floor while the aircraft
was running at the walker. 0.0 (the default) is exactly the behaviour above.

WHAT IT DOES NOT DO
-------------------
It guards GOALS only. SUPER's path between the drone and the goal can still dip below the floor
to pass under an obstacle, and SUPER can move a goal that lands inside an obstacle to the nearest
free cell in any direction, downward included (fsm.cpp:230). Neither passes back through this
node. The path-level protection near a person is the proximity veto.
"""
import math


def _finite(*values):
    return all(math.isfinite(v) for v in values)


class DescentGuard:
    """See the module docstring. Times are seconds on any monotonic clock the caller owns."""

    def __init__(self, radius_m, release_margin_m, release_dwell_s, raise_margin_m):
        for name, value in (('radius_m', radius_m), ('release_margin_m', release_margin_m),
                            ('release_dwell_s', release_dwell_s),
                            ('raise_margin_m', raise_margin_m)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError('descent guard %s must be finite and >= 0, got %r'
                                 % (name, value))
        self.radius_m = float(radius_m)
        self.release_margin_m = float(release_margin_m)
        self.release_dwell_s = float(release_dwell_s)
        self.raise_margin_m = float(raise_margin_m)
        self.floor_z = None
        self._clear_since = None
        self.hold_base_z = None     # the drone's altitude when a HOLD began; see the docstring
        self._holding = False

    @property
    def active(self):
        return self.floor_z is not None

    def reset(self):
        """Drop the floor and forget any HOLD climb. Returns the dropped floor, or None."""
        was = self.floor_z
        self.floor_z = None
        self._clear_since = None
        self.hold_base_z = None
        return was

    def hold_started(self, z):
        """The mux latched a HOLD point while the drone was at altitude z."""
        self._holding = True
        if math.isfinite(z):
            self.hold_base_z = z if self.hold_base_z is None else min(self.hold_base_z, z)

    def hold_ended(self):
        """The mux left HOLD. The climb stays excluded until the drone is back down."""
        self._holding = False

    def clear_hold(self):
        """PX4 is not flying the mux's goals, so nothing the drone does now is a HOLD's climb."""
        self._holding = False
        self.hold_base_z = None

    @staticmethod
    def _near(radius, gx, gy, dx, dy, people):
        """The first (track_id, x, y) within `radius` of the drone or the goal, else None.

        A non-finite person is skipped here, as it was before: NaN would silently fail every
        "< radius" test, and the proximity veto already vetoes on a person it cannot locate.
        """
        for person in people:
            _, px, py = person
            if not _finite(px, py):
                continue
            if (math.hypot(px - dx, py - dy) < radius
                    or math.hypot(px - gx, py - gy) < radius):
                return person
        return None

    def apply(self, now, goal_x, goal_y, goal_z, drone, people, extra_radius_m=0.0):
        """Return (z, event, person) for one goal.

        drone is (x, y, z) or None. people is an iterable of (track_id, x, y). event is None,
        'latched', 'raised' or 'released'; person is the (track_id, x, y) that caused a latch.
        extra_radius_m widens the trip and the release radius alike (SPEED-SCALED RADII); a
        non-finite or negative value is taken as 0.
        """
        extra = extra_radius_m if (math.isfinite(extra_radius_m) and extra_radius_m > 0.0) else 0.0
        if drone is None:
            # No position: nothing to trip or release against. Keep any floor, and restart the
            # release dwell rather than count time we could not observe as clear.
            self._clear_since = None
            if self.floor_z is not None and goal_z < self.floor_z:
                return self.floor_z, None, None
            return goal_z, None, None

        people = list(people)
        dx, dy, dz = drone
        if self.hold_base_z is not None:
            if not self._holding and dz <= self.hold_base_z + self.raise_margin_m:
                self.hold_base_z = None     # back down: the HOLD's climb is over
            else:
                dz = min(dz, self.hold_base_z)
        event = None
        cause = None
        if self.floor_z is None:
            if goal_z < dz:
                cause = self._near(self.radius_m + extra, goal_x, goal_y, dx, dy, people)
                if cause is not None:
                    self.floor_z = dz
                    self._clear_since = None
                    event = 'latched'
        else:
            if self._near(self.radius_m + self.release_margin_m + extra,
                          goal_x, goal_y, dx, dy, people) is not None:
                self._clear_since = None
            else:
                if self._clear_since is None:
                    self._clear_since = now
                if now - self._clear_since >= self.release_dwell_s:
                    self.floor_z = None
                    self._clear_since = None
                    return goal_z, 'released', None
            if dz > self.floor_z + self.raise_margin_m:
                self.floor_z = dz
                event = 'raised'

        if self.floor_z is not None and goal_z < self.floor_z:
            return self.floor_z, event, cause
        return goal_z, event, cause
