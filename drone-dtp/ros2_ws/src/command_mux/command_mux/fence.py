"""The horizontal geofence, as pure geometry with no ROS in it.

WHY IT LIVES HERE (M5.2, decided 2026-09-14)
--------------------------------------------
The M5 plan calls for a 30 m x 4 m box around every follow flight. Nothing on this aircraft
enforced one: PX4's GF_MAX_HOR_DIST is inert under the vision-only EKF2 profile, because
geofence.cpp:354 evaluates distance only when a GLOBAL home position is valid and
EKF2_GPS_CTRL=0 gives none. Of the three options in docs/m5-m6-plan.md SS4 M5.2 -- a clamp here,
GPS fusion so PX4's own fence works, venue + pilot only -- the user chose this one: it works under
the estimator profile every gate was signed off on, it needs no FC parameter change, and it is
testable on the bench like the rest of this node.

WHAT IT IS, AND WHAT IT IS NOT
------------------------------
It is a fence on the GOALS this node publishes, plus a land-in-place if the aircraft is
nevertheless carried far outside. It is a rectangle in the SLAM frame (`lidar_init`, which is
FAST-LIO's origin at the spot the stack was started -- the estimator preflight's E5 refuses an
arm more than 1 m from it), axis-aligned to the boot heading unless `yaw` rotates it.

  * SOFT boundary = the box edge. Every goal -- FOLLOW, NAV and this node's own HOLD goals -- is
    clamped into the box inset by `margin`, so when the followed person walks out of the box the
    drone follows to the edge and stops there. The clamp is applied where the goal is shaped AND
    re-checked at the single publish point, so no path through the node can emit a goal outside.
  * HARD boundary = the box edge + `breach_land_m`. If the aircraft's OWN position is further out
    than that, the node raises a FENCE veto (HOLD, with the hold goal clamped inside -- i.e.
    still commanding it back in) and, if the breach persists for the abort window with follow
    authority, LANDS IN PLACE through the same escalation as a sensor loss. That is PX4's own
    GF_ACTION=land semantics, reproduced at the goal level. A goal clamp alone is not a fence:
    SUPER routes AROUND obstacles and may leave the box doing so, and an estimator that drifts
    carries the aircraft with it while every goal reads "inside".

  It does NOT cover goals that never pass through command_mux: an RViz 2D-Goal-Pose click on
  /goal_pose or the web GCS's remote-control card publish straight into fsm_node. Those are
  operator actions with a human watching; the follower's goals are the ones nobody is clicking.

  It also does not know where the aircraft is when there is no odom -- and that case already
  vetoes (ODOM) and escalates.

WHY A BOX THAT EXCLUDES THE ORIGIN IS REFUSED
---------------------------------------------
The aircraft starts at the origin (E5). A box that does not contain it, inset, would put the
aircraft in breach on the first tick, the first goal would be clamped away from where it stands,
and -- because this node publishes nothing until one healthy tick -- a sufficiently large breach
would leave SUPER unarmed with the only symptom a repeating log line. That is a configuration
error, and the node refuses to start on it rather than fly it.
"""
import math

# A side shorter than this after the margin is applied leaves no room to place a goal: refuse.
MIN_INTERIOR_M = 0.5


class Fence:
    """Rectangle [x_min, x_max] x [y_min, y_max] in a frame rotated by `yaw` about the origin.

    'Box frame' coordinates (u, v) are the world point rotated by -yaw: u runs along the box's
    long side when yaw is the corridor heading. All public methods take and return WORLD x, y.
    """

    def __init__(self, x_min, x_max, y_min, y_max, yaw_rad=0.0, margin_m=0.5,
                 breach_land_m=3.0):
        vals = (x_min, x_max, y_min, y_max, yaw_rad, margin_m, breach_land_m)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
            raise ValueError('fence parameters must all be finite numbers, got %r' % (vals,))
        if margin_m < 0.0:
            raise ValueError('fence margin must be >= 0 (got %.3f)' % margin_m)
        if (x_max - x_min) - 2.0 * margin_m < MIN_INTERIOR_M:
            raise ValueError(
                'fence x span %.2f m minus 2 x margin %.2f m leaves < %.2f m to fly in'
                % (x_max - x_min, margin_m, MIN_INTERIOR_M))
        if (y_max - y_min) - 2.0 * margin_m < MIN_INTERIOR_M:
            raise ValueError(
                'fence y span %.2f m minus 2 x margin %.2f m leaves < %.2f m to fly in'
                % (y_max - y_min, margin_m, MIN_INTERIOR_M))
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.yaw = float(yaw_rad)
        self.margin = float(margin_m)
        # < 0 disables the hard boundary (clamp only). Not the default, and the node says so.
        self.breach_land_m = float(breach_land_m)
        self._c = math.cos(self.yaw)
        self._s = math.sin(self.yaw)

    # -- frames ---------------------------------------------------------------------------------

    def to_box(self, x, y):
        """World -> box frame."""
        return (self._c * x + self._s * y, -self._s * x + self._c * y)

    def from_box(self, u, v):
        """Box frame -> world."""
        return (self._c * u - self._s * v, self._s * u + self._c * v)

    # -- queries --------------------------------------------------------------------------------

    def outside_by(self, x, y):
        """0.0 inside the (un-inset) box, else the Euclidean distance to it. NaN in -> inf out.

        inf rather than NaN on purpose: every caller compares this against a threshold, and
        `nan > t` is False -- a non-finite position must read as the WORST breach, not as none.
        """
        if not (math.isfinite(x) and math.isfinite(y)):
            return math.inf
        u, v = self.to_box(x, y)
        du = max(self.x_min - u, 0.0, u - self.x_max)
        dv = max(self.y_min - v, 0.0, v - self.y_max)
        return math.hypot(du, dv)

    def contains(self, x, y):
        return self.outside_by(x, y) == 0.0

    def breached(self, x, y):
        """True when the aircraft is past the HARD boundary (box + breach_land_m)."""
        if self.breach_land_m < 0.0:
            return False
        return self.outside_by(x, y) > self.breach_land_m

    def clamp(self, x, y):
        """Nearest point inside the box inset by `margin`. Returns (cx, cy, moved_m).

        moved_m is 0.0 when the input was already inside the inset box. NaN in -> the box centre,
        with moved_m = inf, so a caller that logs "moved by" cannot mistake it for a small nudge.
        The centre is chosen (rather than raising) because this is the LAST gate before the
        publisher and a goal there must be inside the fence whatever came in; the caller still
        vetoes non-finite goals upstream (GOAL_INVALID) so this branch is belt, not braces.
        """
        if not (math.isfinite(x) and math.isfinite(y)):
            cx, cy = self.from_box(0.5 * (self.x_min + self.x_max),
                                   0.5 * (self.y_min + self.y_max))
            return (cx, cy, math.inf)
        u, v = self.to_box(x, y)
        cu = min(max(u, self.x_min + self.margin), self.x_max - self.margin)
        cv = min(max(v, self.y_min + self.margin), self.y_max - self.margin)
        if cu == u and cv == v:
            return (x, y, 0.0)
        cx, cy = self.from_box(cu, cv)
        return (cx, cy, math.hypot(cx - x, cy - y))

    def describe(self):
        span_x = self.x_max - self.x_min
        span_y = self.y_max - self.y_min
        hard = ('land beyond +%.1f m' % self.breach_land_m if self.breach_land_m >= 0.0
                else 'NO hard boundary (clamp only)')
        return ('%.1f x %.1f m box: x [%.1f, %.1f] y [%.1f, %.1f] in lidar_init rotated %.1f deg, '
                'goals clamped %.2f m inside the edge, %s'
                % (span_x, span_y, self.x_min, self.x_max, self.y_min, self.y_max,
                   math.degrees(self.yaw), self.margin, hard))
