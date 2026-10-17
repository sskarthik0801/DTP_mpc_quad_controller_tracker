"""The engage gate: no goal reaches SUPER until PX4 is flying our setpoints. Pure, no ROS in it.

WHY (the 2026-09-21 20:05 flight, .claude/rules/flight-log.md)
--------------------------------------------------------------
SUPER plans from the aircraft's odometry only when it starts from rest (PlanFromRest,
super_planner.cpp). Every replan after that continues from its OWN committed reference
(generateExpTraj takes the start state off cmd_traj_info_ at replan_state_TT) and never looks at
where the aircraft is. The mux used to feed it goals from the moment FOLLOW engaged, on the
ground. So while the pilot took off and hovered in Position, SUPER "flew" the follow in its
reference and walked away from the aircraft. At the Offboard switch the bridge forwarded that
reference: pos_cmd was 3.0 m from the aircraft, which closed the gap at 1.7 m/s (max_vel 1.0)
with the walker 1.3 m from the path. Earlier engages the same day: 0.55 m and 0.89 m.

WHAT
----
Goals are withheld from SUPER until PX4 reports armed + Offboard. SUPER then sits idle (INIT or
WAIT_GOAL; pos_cmd silent, so the bridge holds its latched point at the switch), and the first
goal after the switch is planned FROM REST AT THE AIRCRAFT. The mux still arbitrates, vetoes,
latches the descent-guard floor and reports in /mux/state while the gate is shut; only the
publish is held back.

WHY IT CLOSES ONLY ON A FRESH "NOT ENGAGED" REPORT
--------------------------------------------------
Silence at the goal level is not a stop (command_mux docstring): SUPER keeps flying the last goal.
A gate that shut on a STALE report -- vehicle_control_mode arrives at ~2 Hz and the DDS agent can
drop -- would silence the proximity-veto HOLD in the air while PX4 was still in Offboard. So it
opens on a fresh armed + Offboard report and shuts only on a fresh report that says otherwise
(the pilot took over, or disarm). An unknown state leaves it as it was: shut before the first
engage, open after it.

PARKING SUPER WHEN THE GATE SHUTS (added 2026-09-30, user decision; SUPER patch 0009)
------------------------------------------------------------------------------------
Withholding goals does not stop SUPER: it keeps the last goal it was given. When its trajectory
ends more than 0.1 m from that goal (it always does once the pilot has flown away), its FSM calls
PlanFromRest on every 10 ms tick until a plan succeeds or the aircraft reaches the goal. On
2026-09-30 that ran for 45 minutes after a takeover: 76,229 failed plans, fsm_node at 144 % of a
core and 4.3 GB, the Jetson at load 10-12 and the GCS taking 20 s per request.
So when the gate shuts, the mux tells SUPER to PARK (std_msgs/Empty on /planning/park): drop the
goal, go to WAIT_GOAL, stop publishing pos_cmd. That is the state SUPER is in before the first
engage, so the next Offboard plans from rest at the aircraft exactly as the first one does. The
message is sent ParkBurst.repeats times, because the topic is BEST_EFFORT and one lost message
would leave SUPER planning. A gate that re-opens cancels what is left of the burst.

WHAT IT DOES NOT COVER
----------------------
Goals that bypass the mux (an RViz click on /goal_pose, the GCS remote-control card). Those need
the bridge to refuse a far setpoint at engage, which is not built. A re-engage within a few
seconds of a takeover is covered only with the park: without it (engage_gate_park_enable false,
or a SUPER without patch 0009) SUPER is still finishing the trajectory it had, and its reference
is wherever it got to, not the aircraft.
"""


class EngageGate:
    """Open while PX4 flies our setpoints; see the module docstring for the close rule."""

    def __init__(self, stale_s):
        self.stale_s = float(stale_s)
        self.is_open = False
        self.opened = 0          # how many times it has opened (engages seen)

    def update(self, now, report_arrival, armed, offboard):
        """One tick. Returns 'opened', 'closed' or None (no change).

        report_arrival: when the latest vehicle_control_mode arrived (None: never).
        """
        fresh = report_arrival is not None and (now - report_arrival) <= self.stale_s
        if not fresh:
            return None
        engaged = bool(armed) and bool(offboard)
        if engaged and not self.is_open:
            self.is_open = True
            self.opened += 1
            return 'opened'
        if not engaged and self.is_open:
            self.is_open = False
            return 'closed'
        return None


class ParkBurst:
    """The few park messages sent after the gate shuts; see the module docstring.

    start() on the 'closed' edge, cancel() on 'opened', due() once per tick: True when a message
    should go out now.
    """

    def __init__(self, repeats, period_s):
        self.repeats = int(repeats)
        self.period_s = float(period_s)
        self.left = 0
        self._next = 0.0
        self.sent = 0            # messages sent since the node started

    def start(self, now):
        self.left = self.repeats
        self._next = now

    def cancel(self):
        self.left = 0

    def due(self, now):
        if self.left <= 0 or now < self._next:
            return False
        self.left -= 1
        self._next = now + self.period_s
        self.sent += 1
        return True
