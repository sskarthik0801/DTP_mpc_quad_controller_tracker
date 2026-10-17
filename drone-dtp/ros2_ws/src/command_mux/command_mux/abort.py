"""The HOLD -> LAND escalation, as a pure decision with no ROS in it.

WHY HOLD IS NOT ENOUGH
----------------------
command_mux answers every veto with a HOLD goal, and for the vetoes it was designed around --
a person too close, a follow source gone quiet -- a hold IS the safe answer: the drone stops
where it is, with SLAM still placing it and perception still watching for people. But four
silent sensor losses on the bench in one week (three camera deaths, one LiDAR link drop; every
one process-alive-and-publishing-nothing) made the other case concrete: a hold with NO
perception and possibly NO SLAM is not a hold. It is a drone hovering on whatever PX4's EKF
still believes, unable to see anyone walking under it, waiting for a pilot to notice. The
decision (user, 2026-09-11) is that this case LANDS IN PLACE.

WHAT ESCALATES AND WHAT DOES NOT
--------------------------------
Only the vetoes that mean "we can no longer see or place ourselves":
    SENSORS  sensor_watchdog says a stream is dead or perception is blind
    TARGETS  /dtp/targets stopped -- perception_node itself is gone, so the proximity veto has
             no input (the watchdog does not detect THIS one: it folds in perception_ok's
             VALUE, not its absence)
    ODOM     /lidar_slam/odom stopped -- LiDAR or FAST-LIO is gone
    FENCE    the aircraft is past the geofence's HARD boundary (box + fence_breach_land_m,
             command_mux/fence.py). Added 2026-09-14 (M5.2). Not blindness, but the same
             shape of problem: every goal is already clamped inside the box, so an aircraft
             that is nevertheless carried further out is not doing what it is told -- SUPER
             routing around something, or an estimator drifting -- and a hold outside the
             venue on that estimate is not safer than landing. This is PX4's GF_ACTION=land.
PROXIMITY never escalates: a person too close is the one case where landing is WORSE than
holding, because the landing site is next to them. GOAL_INVALID and a missing source never
escalate either -- those are the follow chain failing, not the aircraft going blind.

The condition must PERSIST for `after_s` (default 2.0 s) before firing. sensor_watchdog
detects a stall in 0.33-0.39 s and perception_node's blind path is immediate, so 2 s is
several detection periods -- long enough that a 0.4 s USB hiccup that recovers (seen 2026-09-07)
lands nobody, short enough that the drone is not left blind for long. Every tick in between
is already a HOLD.

AUTHORITY
---------
It fires only when the follow stack has authority: the mux has asserted at least one goal
(`ever_healthy`) AND PX4 reports armed + Offboard. A blind follow stack sitting on a drone that
is flying someone else's mission -- Stage A observe-only, or a NAV-only sortie -- must not land
it. On the bench (disarmed) it therefore never fires; `abort_require_offboard:=false` exists so
the exact code path can be exercised with a disarmed FC and is logged at ERROR while set.

WHAT IT DOES
------------
Sends PX4 MAV_CMD_NAV_LAND through /fmu/in/vehicle_command -- the same command QGC's Land
button sends -- and repeats it at `repeat_period_s` until PX4 acknowledges ACCEPTED (or
IN_PROGRESS), up to `max_sends`. A TEMPORARILY_REJECTED ack keeps retrying (the usual reason
is a transient mode-switch refusal); DENIED / UNSUPPORTED / FAILED stops retrying, because
resending a command PX4 has refused on principle is noise, and says so. Land mode takes PX4 out
of Offboard on its own; the bridge's setpoint stream becomes irrelevant the moment it is
accepted. This is the ONE place in the DTP stack that changes a PX4 flight mode without a human
in the loop, and it is a de-escalation, never an escalation: it can only ever take the drone
DOWN. The RC pilot's mode switch still overrides it in hardware, as it overrides everything.

It is TERMINAL. Once fired the mux stays in HOLD arbitration with an ABORT veto until the node
is relaunched: a stack that has aborted must not quietly resume following when the camera
plugs back in while the drone is half-way through landing.

This class holds the whole decision so it can be unit-tested with a fake clock. The node only
feeds it (now, veto codes, authority) and acts on what it returns.
"""

ABORT_CLASS = ('SENSORS', 'TARGETS', 'ODOM', 'FENCE')

# px4_msgs/VehicleCommandAck.msg result codes, duplicated as plain ints so this module (and its
# tests) need no px4_msgs. The node compares against the message constants at import time.
RESULT_ACCEPTED = 0
RESULT_TEMPORARILY_REJECTED = 1
RESULT_DENIED = 2
RESULT_UNSUPPORTED = 3
RESULT_FAILED = 4
RESULT_IN_PROGRESS = 5
RESULT_CANCELLED = 6

ACK_STOP_OK = frozenset((RESULT_ACCEPTED, RESULT_IN_PROGRESS))
ACK_RETRY = frozenset((RESULT_TEMPORARILY_REJECTED,))
# Everything else is final: PX4 has refused on principle, or failed after accepting.


class AbortEscalation:
    def __init__(self, after_s, repeat_period_s, max_sends):
        if after_s < 0.0 or repeat_period_s <= 0.0 or max_sends < 1:
            raise ValueError('after_s >= 0, repeat_period_s > 0, max_sends >= 1')
        self.after_s = float(after_s)
        self.repeat_period_s = float(repeat_period_s)
        self.max_sends = int(max_sends)

        self.since = None           # when the current abort-class condition began, or None
        self.fired = False
        self.fired_at = None
        self.reason = None
        self.sends = 0
        self.last_send = None
        self.ack_result = None      # last ack result code for our command, or None
        self.done = False           # no further sends: accepted, or finally rejected, or exhausted

    # -- decision -------------------------------------------------------------------------------

    def observe(self, now, veto_codes, authority, authority_note='armed+offboard'):
        """Feed one tick. Returns the reason string on the ONE tick the abort fires, else None.

        veto_codes: the codes active this tick (any iterable of str).
        authority: True only while the follow stack is actually in charge of a flying drone.
        authority_note: what granted it, for the reason string -- so a bench-flag abort is
            recorded as one and never reads as a flight event afterwards.
        """
        if self.fired:
            return None
        active = [c for c in veto_codes if c in ABORT_CLASS]
        if not authority or not active:
            # Losing authority clears the timer on purpose: a condition that began while the
            # pilot had the drone must not fire the instant Offboard is re-engaged.
            self.since = None
            return None
        if self.since is None:
            self.since = now
            return None
        held = now - self.since
        if held < self.after_s:
            return None
        self.fired = True
        self.fired_at = now
        self.reason = ('%s persisted %.1f s >= %.1f s with follow authority (%s)'
                       % ('+'.join(sorted(set(active))), held, self.after_s, authority_note))
        return self.reason

    # -- command scheduling ---------------------------------------------------------------------

    def send_due(self, now):
        """True when NAV_LAND should be (re)sent this tick."""
        if not self.fired or self.done:
            return False
        if self.sends >= self.max_sends:
            self.done = True
            return False
        if self.last_send is None:
            return True
        return (now - self.last_send) >= self.repeat_period_s

    def note_sent(self, now):
        self.sends += 1
        self.last_send = now

    def note_ack(self, result):
        """Classify PX4's answer. Returns 'accepted' | 'retry' | 'rejected'."""
        self.ack_result = int(result)
        if self.ack_result in ACK_STOP_OK:
            self.done = True
            return 'accepted'
        if self.ack_result in ACK_RETRY:
            return 'retry'
        self.done = True
        return 'rejected'

    def status(self):
        """Short token for /mux/state."""
        if not self.fired:
            return 'armed' if self.since is None else 'pending'
        ack = 'none' if self.ack_result is None else str(self.ack_result)
        return 'fired sends=%d ack=%s%s' % (self.sends, ack, ' done' if self.done else '')
