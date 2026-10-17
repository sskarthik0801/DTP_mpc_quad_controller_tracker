"""target_manager -- locks ONE human target, survives occlusion, and NEVER silently follows the
wrong person. NEW node -- no equivalent in the reference repo (it never needed to pick a target:
it treated every detected person as a hazard to avoid, not one to follow).

Phase 1 M1.1 (plan/dtp-imp/06), policy fully specified in plan/dtp-imp/04 SS2 (decision D13).
The four rules this implements, unchanged from the scaffold that preceded it:
  1. LOCK = the first CONFIRMED person after an explicit operator FOLLOW-engage, and outside a
     launch-area ignore radius (otherwise it locks onto the pilot at takeoff -- risk R-F13).
  2. On occlusion: COAST the CTRV prediction forward. The re-acquisition tolerance band grows
     with occlusion duration (more time hidden = less certainty where they are).
  3. Re-acquire: exactly ONE candidate inside the band -> re-lock. TWO OR MORE -> ABSTAIN, do not
     guess. If still ambiguous past a timeout, drop to LOST and signal the operator. Guessing
     wrong here means autonomously following a stranger -- the single most safety-critical piece
     of this node (risk A10/A10b, CLAUDE.md SS6 rule 5).
  4. Non-locked people are NOT discarded and are NOT republished here -- they are already on
     /dtp/targets, which command_mux's independent proximity veto subscribes to directly.

WHICH STATES PUBLISH, AND WHY SILENCE IS THE LOSS SIGNAL
-------------------------------------------------------
    LOCKED    publishes -- pass-through of the target perception saw this frame
    COASTING  publishes -- the CTRV forecast advanced to now; the person is hidden, not gone
    IDLE / ACQUIRING / ABSTAIN / LOST   publish NOTHING

follow_planner_node.hpp:48-52 states the contract from the other end: "target_manager publishes
/dtp/follow_target ONLY while a lock exists (nothing at all while IDLE/ACQUIRING/ABSTAIN/LOST),
so absence is the signal, not a hold." Its follow_target_timeout_s (0.30 s) then withholds the
goal, command_mux's source_stale_timeout_s (0.20 s) sees FOLLOW go stale, and the mux falls
through to NAV or to a climbing HOLD. That chain IS the plan 04 SS6 loss response ("ascend +
back off, NEVER hover-in-place"), which is why this node commands nothing itself on loss.

NO TIMER. THE INPUT'S RATE IS OUR RATE.
---------------------------------------
Exactly one publish per received TargetArray, in every publishing state -- coasting included.
Two reasons, both load-bearing:
  * perception_node publishes /dtp/targets on EVERY processed frame including empty ones (that
    is mandatory: command_mux's targets freshness veto cannot otherwise distinguish "nobody in
    view" from "perception is dead"). So the array arrives at the detection rate whether or not
    anyone is detected, and driving off it gives us that rate for free -- ~15 Hz, i.e. 4.5x
    margin on follow_planner's 0.30 s timeout and 7.5x on command_mux's 0.50 s.
  * Our liveness is then DERIVED from perception's liveness. A timer would let this node coast a
    ghost off a clock that is still ticking after the pipeline feeding it has died -- the
    follow chain would look healthy all the way to SUPER with no live perception behind it.

THOSE MARGINS ARE THE MEAN. THE TAIL IS THIN, AND IT IS NOT OURS TO FIX.
------------------------------------------------------------------------
4.5x and 7.5x describe the NOMINAL 66.7 ms period and nothing else. The gap the consumers
actually see is one detection period plus however long perception waited for odom to cover the
frame's stamp, and /lidar_slam/odom is measured at ~10 Hz with real jitter up to 204 ms between
messages (plan/dtp-imp/11 R-F22, std dev ~30 ms, FAST-LIO2 per-scan compute variance at >70% of
one core -- worst precisely while the drone is working hard, i.e. yawing to keep someone in
frame). That puts the worst inter-publish gap at ~0.21-0.22 s: 2.3x on command_mux's 0.50 s but
only ~1.4x on follow_planner's 0.30 s. Add the camera's own measured ~8% upstream frame loss
(plan/dtp-imp/21) and two unlucky events stack toward ~0.25 s, i.e. ~1.2x.

What that costs is a DISAGREEMENT, which is the specific thing this contract exists to prevent:
past 0.30 s follow_planner declares "target stale" and withholds the goal while this node is
still LOCKED and still coasting happily. There is no fix available HERE -- our rate is
perception's rate by construction, and adding a timer to paper over it is the ghost-coasting
failure above. The levers are upstream (perception_node's max_pair_age_s) and downstream
(follow_target_timeout_s). M1.4.1/M1.4.2 must therefore measure the DISTRIBUTION of the
/dtp/targets inter-publish gap under concurrent load, not the mean rate: a mean of 15 Hz with a
0.3 s tail is a broken follow chain that reports healthy.

WHAT `missed_frames` ON /dtp/targets ACTUALLY IS (checked, and it is not what it looks like)
-------------------------------------------------------------------------------------------
Always 0. detection.py builds output["detections"] only from tracks parsed THIS frame, and
_mark_track_seen (detection.py:586-591) zeroes track_missed_counts[track_id] before the dict is
built. A track that was missed is simply ABSENT from the array; so is one that was detected but
whose torso patch had no valid depth (detection.py:776 `continue`). So absence is the only loss
signal available to us, it conflates occlusion with an unranged sighting, and the field carries
no information. This node therefore OVERWRITES missed_frames on /dtp/follow_target with its own
coast frame count, which is the only place the depth of a coast is visible on the wire.
(dtp_msgs/Target.msg's comment describing missed_frames as the reference repo's coast counter is
now true only of that field on /dtp/targets. Correcting it needs an edit to a file this change
does not own.)

THE COAST IS BOUNDED BY THE FORECAST, NOT BY OUR TIMER
------------------------------------------------------
Coasting means evaluating the predictor's own published polyline at a later time -- never
extrapolating past its end. HumanXZPredictor emits prediction_secs = 1.5 s of forecast, so
coast_timeout_s can never usefully exceed that, and in practice the forecast runs out first
(latency compensation has already consumed its first ~0.2 s). plan 04 SS2 asks to "coast for
seconds" through multi-second trunk occlusion; under D13 (no appearance re-ID, identity held by
predicted-path matching alone) that is not reachable -- there is nothing to match against once
the forecast ends. Extending the coast means raising the predictor's horizon, not this number.

A target with NO forecast cannot be coasted at all and loses the lock on its first missed frame.
(One case that LOOKS like that and is not: perception sends an empty forecast on a HELD frame, so
the message carries a single fallback point. _rebase_retained_forecast keeps the previous path
across it -- read that docstring before touching the coast, because rebuilding naively there
turns 1.5 s of coast into 0.2 s in exactly the run-up to an occlusion.)
That happens when the predictor's prediction_depth_gate_m is exceeded, which is why
acquire_max_range_m below is pinned to the same number rather than to the detector's reach:
acquiring a person we could not coast through one tree is a lock that dies on the tree.

That gate is NOT the library default, and this file used to assume it was. HumanXZPredictor
defaults prediction_depth_gate_m and max_depth_m to 6.0 m; perception_node deliberately DECLARES
both at 10.0 rather than inheriting them, because at 6.0 the CTRV forecast switches off silently
between 6 and 10 m -- exactly where the person is farthest and the lead matters most -- and an
empty predicted_path drops the whole stack to constant-velocity extrapolation with nothing on the
wire to say so. So acquire_max_range_m is 10.0 to match. It is one number living in two packages
with no compiler between them: read perception_node's declared value before changing either.
Pinned to the library default (as it was) this node quietly refuses to acquire anyone between 6
and 10 m who is perfectly coastable, and the only symptom is a lock that never happens.

THE STAMP IS NEVER REWRITTEN, INCLUDING WHILE COASTING
------------------------------------------------------
header.stamp stays the perception stamp of the frame in which the target was last SEEN. It is
the only instant for which sensor evidence exists, and SUPER hard-gates yaw on its AGE
(kYawTargetTimeoutSec = 0.5 s, measured getSimTime() - stamp after follow_planner copies our
stamp verbatim into /planning/yaw_target). Restamping to now() would certify an arbitrarily old
extrapolation as fresh perception and would silently delete the only end-to-end perception-age
guard in the stack.

The cost, stated rather than hidden: past ~0.5 s of coasting SUPER stops pinning yaw at the
target and falls back to velocity-aligned yaw (risk R-F10) -- exactly when pointing the camera
at the predicted position would most help re-acquisition. It is accepted because the alternative
defeats a safety gate. Freshness for the two nodes that must NOT confuse the two clocks is
carried by ARRIVAL instead: follow_planner and command_mux both age our messages on arrival, and
our republishing keeps FOLLOW alive while the stamp keeps ageing honestly.

Since 2026-09-26 follow_planner covers the R-F10 gap on its own clock, openly: its coast yaw hold
(coast_yaw_hold, lost_yaw_hold_s) publishes ONE held heading, stamped now, from 0.4 s of stamp age
-- a heading, not a claim that the extrapolation is fresh. This stamp is still never rewritten.

Topics (plan/dtp-imp/01 SS2):
  Subscribes: /dtp/targets (dtp_msgs/TargetArray) -- every detected person, from perception_node
              /lidar_slam/odom (nav_msgs/Odometry) -- ONLY to latch the launch point for the
              ignore radius, and to keep distance_m honest while coasting. Not a control input.
  Publishes:  /dtp/follow_target (dtp_msgs/Target) -- the ONE locked target, is_locked_target
              true, while LOCKED or COASTING; nothing otherwise
              /dtp/lock_state (std_msgs/String, "STATE|track|detail") -- diagnostics, and the
              operator-facing half of D13's "abstain -> request operator re-designation": a log
              line is not observable to a ground station or to a bag reader. Modelled on
              /mux/state. plan/dtp-imp/01 SS2's node table does not list it yet.
"""
import copy
import math
from enum import Enum, auto

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from dtp_msgs.msg import LossEvent, Target, TargetArray

# Width of the secant used to read a velocity off the coasted forecast. _predict_future_xz emits
# prediction_steps = 20 points over prediction_secs = 1.5 s, i.e. one every 0.075 s, so 0.15 s is
# the smallest window that spans more than one native sample -- narrower and the "velocity" is
# just the slope of a single quantised segment, wider and a CTRV arc gets cut across its chord.
COAST_VELOCITY_WINDOW_S = 0.15

# M6.5 LOSS CONDITIONING (docs/m5-m6-plan.md SS5.0, "Conditioning"). Behind ONE parameter,
# loss_conditioning, default false, so both ablation arms are the same binary:
#   false  -> every loss is coasted the same way (the paper's CARRY-FORWARD arm; exactly the
#             behaviour above, LossEvents are only recorded)
#   true   -> the coast is conditioned on fsm_node's /planning/loss_event:
#     OCCLUDED     coast to min(occluded_coast_cap_s, t_max + margin) instead of the forecast's
#                  1.3 s, the polyline being extended at constant velocity to get there; inside the
#                  belief's time window the re-acquisition gate is the REGION around the dominant
#                  re-emergence point (radius max(region_min_radius_m, region_sigma_k * sigma)),
#                  outside it the ordinary band; ABSTAIN unchanged.
#     LOST/UNMAPPED  end the coast -- once the verdict has PERSISTED lost_confirm_s (below).
#     OUT_OF_VIEW  today's coast and band (follow_planner turns the camera; nothing to do here).
# Every number is a starting value the M6.3 handheld dataset sweeps.
#
# WHY NOT "END IMMEDIATELY" ON LOST, as the plan's table says: a detector miss on a visible
# person classifies as LOST by design, and on 20260909_closedloop_run5 the lock coasted 48
# times in 132 s and re-acquired in 0.19-0.29 s -- nearly all of them one-or-two-frame misses,
# not occlusions. Ending the coast on the first LOST answer would turn each into an operator
# re-designation and make the conditioned arm unflyable for a reason that has nothing to do
# with the thesis. So a LOST verdict ends the coast only after it has stood unbroken for
# lost_confirm_s (0.3 s = 2-3 frames at 8 Hz), still ~4x sooner than the carry-forward coast.
# Pre-registered here rather than tuned after the fact.
LOSS_CLS_NAMES = {LossEvent.CLS_OCCLUDED: 'OCCLUDED', LossEvent.CLS_UNMAPPED: 'UNMAPPED',
                  LossEvent.CLS_LOST: 'LOST', LossEvent.CLS_OUT_OF_VIEW: 'OUT_OF_VIEW'}
# A LossEvent whose eval_stamp disagrees with this node's clock by more than this is refused:
# the window arithmetic below is cross-node clock arithmetic (the thing GoalSource's docstring
# warns about) and a use_sim_time mismatch would otherwise place the window decades away.
LOSS_EVENT_CLOCK_SKEW_MAX_S = 2.0

# /dtp/lock_state is published on every state change plus this heartbeat, so a bag started
# mid-flight shows the current lock rather than nothing until the next transition. Same reasoning
# and same value as command_mux's STATE_HEARTBEAT_S; it is evaluated inside the TargetArray
# callback, NOT on a timer, so this node still has no clock of its own (see the module docstring).
STATE_HEARTBEAT_S = 1.0

# Two forecast times are equal for our purposes below this. predicted_path_dt is float64 seconds
# spaced 0.075 s apart, so anything this close is a duplicate or a rewound sample, not a step.
DT_EPS_S = 1e-6

# Every parameter this node caches into an attribute at construction, and the attribute it feeds.
# The table exists because a cached parameter with no set-callback is a SILENT LIE: rclpy accepts
# `ros2 param set`, reports success, `ros2 param get` reads the new value back -- and the node
# goes on using the old one forever. On a bench that reads as "tuning this had no effect", which
# is indistinguishable from having tuned the wrong knob. Every entry here is a plain threshold
# read fresh on each frame, so applying it live is safe; _on_set_parameters validates the whole
# batch before committing any of it.
_LIVE_PARAMS = {
    'launch_ignore_radius_m': ('_ignore_radius', float),
    'acquire_min_range_m': ('_acq_min_m', float),
    'acquire_max_range_m': ('_acq_max_m', float),
    'acquire_confirm_frames': ('_acq_frames', int),
    'coast_timeout_s': ('_coast_timeout', float),
    'min_coast_horizon_s': ('_min_coast_horizon', float),
    'reacquire_band_m': ('_band_m', float),
    'reacquire_band_growth_mps': ('_band_growth', float),
    'reacquire_band_max_m': ('_band_max_m', float),
    'abstain_timeout_s': ('_abstain_timeout', float),
    'height_tiebreak_enabled': ('_tiebreak_on', bool),
    'height_tiebreak_margin_m': ('_tiebreak_margin', float),
    'auto_reacquire_after_lost': ('_auto_reacquire', bool),
    'world_frame': ('_world_frame', str),
    'loss_conditioning': ('_conditioning', bool),
    'occluded_coast_cap_s': ('_occluded_cap', float),
    'reacquire_window_margin_s': ('_window_margin', float),
    'region_min_radius_m': ('_region_min_r', float),
    'region_sigma_k': ('_region_sigma_k', float),
    'lost_confirm_s': ('_lost_confirm', float),
    'loss_event_max_age_s': ('_loss_event_max_age', float),
    'stock_baseline': ('_stock_baseline', bool),
}

# Read once, at construction, and structurally impossible to honour afterwards: the publisher is
# already built against this topic name. Rejecting the set is the honest answer -- accepting it
# would leave `ros2 param get` reporting a topic nothing is published on, which is a worse lie
# than the one above because it looks like plumbing that was checked.
_CONSTRUCTION_ONLY_PARAMS = ('follow_target_topic', 'loss_event_topic')


class LockState(Enum):
    """Mirrors the state diagram in plan/dtp-imp/04 SS2."""
    IDLE = auto()        # no FOLLOW engage yet; not tracking anyone
    ACQUIRING = auto()   # engaged, waiting for a confirmed candidate outside the ignore radius
    LOCKED = auto()      # actively tracking the locked target, seen this frame
    COASTING = auto()    # locked target currently occluded; propagating the CTRV prediction
    ABSTAIN = auto()     # >=2 candidates in the re-acquisition band; refusing to guess
    LOST = auto()        # coast/ambiguity timeout exceeded; hover/NAV fallback, operator signaled


def _finite(*values):
    """True only if every value is a real, finite number.

    Deliberately duplicated from command_mux rather than shared: importing it would make a
    perception package depend on the arbitration package purely for a six-line guard, and the
    reason it exists is the same in both places -- letting a NaN reach a comparison fails OPEN.
    Every `nan <= band` test is False, so one NaN coordinate silently removes a candidate from
    the ambiguity count while leaving the count looking like it ran, which is how ABSTAIN turns
    into a confident wrong lock.
    """
    for v in values:
        if not isinstance(v, (float, int)) or isinstance(v, bool):
            return False
        if math.isnan(v) or math.isinf(v):
            return False
    return True


def _interp_polyline(ts, pts, t):
    """Linear interpolation of a (time -> xyz) polyline; None when t is outside its span.

    Returning None rather than clamping is the whole point: past ts[-1] there is no forecast, and
    a clamped endpoint would look exactly like a stationary person while actually being the end of
    what the predictor was willing to say. That distinction is what ends the coast.
    """
    if not ts or t < ts[0] - DT_EPS_S or t > ts[-1] + DT_EPS_S:
        return None
    for i in range(1, len(ts)):
        if t <= ts[i]:
            span = ts[i] - ts[i - 1]
            f = 0.0 if span <= DT_EPS_S else (t - ts[i - 1]) / span
            a, b = pts[i - 1], pts[i]
            return (a[0] + f * (b[0] - a[0]),
                    a[1] + f * (b[1] - a[1]),
                    a[2] + f * (b[2] - a[2]))
    return pts[-1]


class TargetManagerNode(Node):
    def __init__(self):
        super().__init__('target_manager')

        # --- engage ---------------------------------------------------------------------------
        # FOLLOW must be OPTED INTO and defaults off: R-F13's failure is that the "first confirmed
        # person" at takeoff is the safety pilot standing next to the aircraft. A parameter is the
        # interim operator interface -- `ros2 param set /target_manager follow_enabled true` is an
        # explicit, auditable act, and it is deliberately NOT a topic or a service because the
        # scaffold's own note forbids adding an engage RPC without updating plan/dtp-imp/01 SS2's
        # node table, which this change does not own. An RC channel or a service belongs here
        # eventually; both are a strictly larger change than the state machine below.
        self.declare_parameter('follow_enabled', False)

        # --- acquisition gates ----------------------------------------------------------------
        # Declared HERE and nowhere else -- command_mux's docstring records that it deliberately
        # does not duplicate this radius, because two numbers to keep in sync is two opinions
        # about who is being followed. dtp_params.yaml overrides it; keep this default equal to
        # that file's value so the override is a no-op match rather than a silent disagreement.
        self.declare_parameter('launch_ignore_radius_m', 3.0)
        # Do not adopt a target the follow layer would have to retreat from on its very first
        # goal: follow_planner's min_standoff_m is 2.5 m, and a lock inside it produces an
        # immediate backward goal -- or, inside command_mux's 1.5 m veto radius, a FOLLOW that
        # never gets authority at all because the mux is already vetoing.
        self.declare_parameter('acquire_min_range_m', 2.5)
        # Pinned to perception_node's DECLARED prediction_depth_gate_m (10.0), NOT to the library
        # default (6.0) and NOT to the detector's reach. Beyond that gate the predictor publishes
        # no forecast, and a target with no forecast cannot be coasted -- the lock would die on
        # the first trunk. These are one number in two packages with nothing checking them: if
        # perception_node moves the gate, move this to match, and see the module docstring for
        # what pinning it to the library default silently costs.
        self.declare_parameter('acquire_max_range_m', 10.0)
        # Acquisition motion gate -- see _eligible_to_acquire for the measurement that forced
        # it (a humanoid robot in the lab out-persisted the real subject 535 frames to 247).
        # 0.0 disables it. 0.40 m is about one deliberate step: comfortably more than the
        # position noise on a standing person, comfortably less than crossing a room.
        self.declare_parameter('acquire_motion_m', 0.40)
        # Window the displacement is measured over. Long enough that a subject who pauses mid
        # approach stays eligible, short enough that it is not remembering last minute.
        self.declare_parameter('acquire_motion_window_s', 6.0)
        # Consecutive TargetArrays the same candidate must remain the best eligible one before we
        # lock. 3 frames is 0.2 s at 15 Hz, and it STACKS on the pipeline's own min_confirm_frames
        # = 3, so ~0.4 s from first sighting to lock. That is comfortably shorter than
        # command_mux's NAV->FOLLOW handover (5 frames + 0.5 s dwell), so the mux's hysteresis --
        # not ours -- paces the handover and the two do not race each other.
        self.declare_parameter('acquire_confirm_frames', 3)

        # --- coast / re-acquisition -----------------------------------------------------------
        # The predictor's own horizon (prediction_secs = 1.5 s) is the ceiling; past it there is
        # no forecast to coast on. In practice the forecast is exhausted sooner, because its first
        # ~0.2 s was already spent on latency compensation -- whichever bound is hit first ends
        # the coast. See the module docstring for why "multi-second" is out of reach.
        #
        # SAY THIS PLAINLY SO NOBODY TUNES IT EXPECTING AN EFFECT: at the shipped numbers this
        # bound NEVER fires. The usable coast is horizon minus one latency compensation, 1.5 -
        # 0.20 = 1.30 s, and the forecast always runs out first (walked offline: 1.33 s at 15 Hz).
        # Raising coast_timeout_s alone therefore changes nothing at all -- the lever for a longer
        # coast is the PREDICTOR's prediction_secs. It is kept as the outer bound because it is
        # the one that still applies if the forecast is ever lengthened, and because it is what
        # scales the confidence decay in _build_coasted. Lowering it below ~1.3 s is the only way
        # to make it the binding constraint, which is a deliberate act, not a tuning nudge.
        self.declare_parameter('coast_timeout_s', 1.5)
        # *** THE COAST FLOOR. MEASURED 2026-09-12; SEE _remember_seen. ***
        #
        # A fresh track arrives with a ONE-POINT forecast at dt = latency (0.20 s): perception
        # will not invent a path before the predictor has warmed up, and says so. This node
        # coasts only as far as the forecast reaches, so a new lock had 0.20 s of tolerance
        # and a single dropped frame in its first second went LOST -- observed indoors at
        # 8 Hz ('coasted 0.10 s of a 0.20 s horizon') and outdoors at dusk ('0.07 of 0.20'),
        # and with auto_reacquire_after_lost false every one was a manual re-engage.
        #
        # 1.2 s: the usable coast is the horizon minus one latency, so this gives ~1.0 s of
        # constant-velocity coast to a brand-new lock -- comparable to the ~1.3 s a full
        # forecast gives an established one. Capped by coast_timeout_s regardless. Set to 0
        # to restore the old behaviour exactly.
        self.declare_parameter('min_coast_horizon_s', 1.2)
        # Band at zero occlusion: the position estimate's own spread. The torso statistic is a
        # per-channel median over a patch whose inlier count falls into the tens beyond ~6 m, and
        # a standing person is ~0.5 m wide, so 1.0 m is "the same person, allowing for the
        # estimator" rather than a confidence region.
        self.declare_parameter('reacquire_band_m', 1.0)
        # How fast that band grows while hidden. A walking human is ~1.4 m/s; what grows here is
        # the FORECAST ERROR, not the displacement, since we are comparing against a CTRV path
        # that already carries the walk. 1.0 m/s is deliberately below walking pace: a person who
        # fully reverses behind a trunk deviates at up to 2x speed and no honest band covers that.
        # THE BAND IS A BOUNDED-JUMP GATE, NOT A CONFIDENCE REGION -- what prevents a wrong lock
        # is the ABSTAIN rule, not the width of this number.
        self.declare_parameter('reacquire_band_growth_mps', 1.0)
        # Hard cap. At the coast ceiling the band is already 2.5 m, which is wider than the
        # spacing of two people walking together -- past that ABSTAIN fires on essentially every
        # re-acquisition, which is the designed behaviour and not a bug to tune away.
        self.declare_parameter('reacquire_band_max_m', 2.5)
        # Ambiguity does not improve with time: the band only GROWS, so waiting admits more
        # candidates rather than fewer. 0.5 s gives the distractor ~7 frames at 15 Hz to walk out
        # of the band; past that, continuing to coast is just widening a guess.
        self.declare_parameter('abstain_timeout_s', 0.5)

        # --- ambiguity tie-break (D13's "cheap CPU-only cue"), OFF by default ------------------
        # position.z is now a real world altitude (perception_node lifts it with the full 3-D
        # transform), so the torso-centroid height of two people is a genuine CPU-free cue. It is
        # DEFAULT OFF because its per-person repeatability has never been measured: our Z is an
        # EMA over a patch that sometimes falls back to camera height entirely, and a tie-break
        # that is wrong follows a stranger. Abstaining is the safe default until M1.4 measures the
        # Z channel's spread on one person across a walk. Turn it on with a number, not a hunch.
        self.declare_parameter('height_tiebreak_enabled', False)
        # And even then it only decides when it is DECISIVE: the best candidate's height must beat
        # the runner-up by this margin. Adult torso centroids differ by ~0.1 m, which is inside
        # our own noise, so 0.25 m means "these are visibly different people" and anything less
        # falls back to ABSTAIN.
        self.declare_parameter('height_tiebreak_margin_m', 0.25)

        # --- loss behaviour -------------------------------------------------------------------
        # After LOST the operator must re-engage (a false->true edge on follow_enabled). That is
        # D13/A10's "request operator re-designation" and CLAUDE.md SS6 rule 5: silently
        # re-acquiring the nearest person after a loss is precisely how the drone ends up
        # following a stranger, because a re-acquisition after LOST has no forecast left to match
        # against and degenerates to "nearest confirmed body". Setting this true defeats that and
        # must never be true with a person in the flight volume; it exists for bench and replay
        # sessions where re-engaging by hand through every occlusion is impractical.
        self.declare_parameter('auto_reacquire_after_lost', False)

        # --- plumbing -------------------------------------------------------------------------
        # Must equal follow_planner's own follow_target_topic parameter (its default is the same
        # string, follow_planner_node.hpp:271). Parameterised at this end only so a bench rig can
        # move both together; the /dtp/targets and /lidar_slam/odom inputs are hardcoded, matching
        # command_mux's split of "parameterise the output, hardcode the inputs".
        self.declare_parameter('follow_target_topic', '/dtp/follow_target')
        # Checked, never rewritten: we republish whatever frame_id perception stamped, because a
        # pass-through that invents a frame is worse than one that propagates a wrong one. This is
        # what perception_node's odom_frame is set to in dtp_params.yaml. Note follow_planner and
        # command_mux both default their own world frame to 'world' instead -- SUPER ignores
        # frame_id entirely, so the disagreement is silent, which is exactly why it gets a warning.
        self.declare_parameter('world_frame', 'camera_init')

        # --- M6.5 loss conditioning (see LOSS_CLS_NAMES above) ----------------------------
        # THE ablation switch. false = carry-forward arm, the behaviour documented above.
        self.declare_parameter('loss_conditioning', False)
        self.declare_parameter('loss_event_topic', '/planning/loss_event')
        # Coast cap under OCCLUDED. The predictor's 1.5 s horizon is extended at constant
        # velocity to reach it (the zero-turn-rate CTRV case, as min_coast_horizon_s does).
        self.declare_parameter('occluded_coast_cap_s', 3.0)
        # delta: the re-acquisition window is [t_min - delta, t_max + delta] about the belief.
        self.declare_parameter('reacquire_window_margin_s', 0.5)
        # Region gate about the dominant re-emergence point: radius max(min, k * sigma).
        self.declare_parameter('region_min_radius_m', 0.5)
        self.declare_parameter('region_sigma_k', 2.0)
        # A LOST / UNMAPPED verdict must stand unbroken this long before it ends the coast.
        self.declare_parameter('lost_confirm_s', 0.3)
        # An event older than this on arrival no longer drives a decision.
        self.declare_parameter('loss_event_max_age_s', 0.5)
        # M6.8 "SUPER stock" comparison arm: NO coast and NO abstain -- the lock ends on the first
        # missed frame and acquisition restarts at once on the nearest confirmed person (the
        # launch disc and motion gate still apply). BENCH AND REPLAY ONLY: it is the naive
        # follower the paper compares against, and it follows strangers by construction.
        self.declare_parameter('stock_baseline', False)

        self._ignore_radius = float(self.get_parameter('launch_ignore_radius_m').value)
        self._acq_min_m = float(self.get_parameter('acquire_min_range_m').value)
        self._acq_max_m = float(self.get_parameter('acquire_max_range_m').value)
        self._acq_motion_m = float(self.get_parameter('acquire_motion_m').value)
        self._acq_motion_window_s = float(self.get_parameter('acquire_motion_window_s').value)
        self._acq_frames = int(self.get_parameter('acquire_confirm_frames').value)
        self._coast_timeout = float(self.get_parameter('coast_timeout_s').value)
        self._min_coast_horizon = float(self.get_parameter('min_coast_horizon_s').value)
        self._band_m = float(self.get_parameter('reacquire_band_m').value)
        self._band_growth = float(self.get_parameter('reacquire_band_growth_mps').value)
        self._band_max_m = float(self.get_parameter('reacquire_band_max_m').value)
        self._abstain_timeout = float(self.get_parameter('abstain_timeout_s').value)
        self._tiebreak_on = bool(self.get_parameter('height_tiebreak_enabled').value)
        self._tiebreak_margin = float(self.get_parameter('height_tiebreak_margin_m').value)
        self._auto_reacquire = bool(self.get_parameter('auto_reacquire_after_lost').value)
        self._world_frame = str(self.get_parameter('world_frame').value)
        follow_target_topic = str(self.get_parameter('follow_target_topic').value)
        self._conditioning = bool(self.get_parameter('loss_conditioning').value)
        loss_event_topic = str(self.get_parameter('loss_event_topic').value)
        self._occluded_cap = float(self.get_parameter('occluded_coast_cap_s').value)
        self._window_margin = float(self.get_parameter('reacquire_window_margin_s').value)
        self._region_min_r = float(self.get_parameter('region_min_radius_m').value)
        self._region_sigma_k = float(self.get_parameter('region_sigma_k').value)
        self._lost_confirm = float(self.get_parameter('lost_confirm_s').value)
        self._loss_event_max_age = float(self.get_parameter('loss_event_max_age_s').value)
        self._stock_baseline = bool(self.get_parameter('stock_baseline').value)

        self._validate_parameters()

        self._state = LockState.IDLE
        self._engaged = bool(self.get_parameter('follow_enabled').value)
        if self._engaged:
            self._state = LockState.ACQUIRING

        self._locked_track_id = None
        self._locked_msg = None          # the last SEEN Target, verbatim; the coast's anchor
        self._last_seen_arrival = None   # ARRIVAL clock -- see _now()
        self._coast_frames = 0
        self._coast_ts = []              # sanitised forecast times, [0.0] + strictly increasing
        self._coast_pts = []             # matching xyz, [0] == the position that was seen
        self._latency_s = 0.0            # predicted_path_dt[0]: the compensation already applied
        self._abstain_since = None

        self._acq_candidate_id = None
        self._acq_streak = 0

        # M6.5: the newest LossEvent for the locked track, its arrival time, and the loss
        # verdict's persistence. All cleared by _remember_seen (a seen frame ends the coast).
        self._loss_event = None
        self._loss_event_arrival = None
        self._loss_class = None            # class of the newest usable event, for lock_state
        self._lost_verdict_since = None    # arrival clock when an unbroken LOST/UNMAPPED run began
        self._occluded_extended = False    # polyline already extended for this coast
        self._loss_events_seen = 0
        self._clock_skew_warned = False

        # Latched once, never updated: the launch point does not move. Latching the FIRST odom
        # rather than assuming the world origin keeps the ignore radius correct even though
        # camera_init IS by construction where FAST-LIO initialised -- if that assumption ever
        # breaks (a re-init mid-session, a different world frame), an assumed origin would put the
        # ignore disc somewhere the aircraft never was and silently stop protecting the pilot.
        self._launch_xy = None
        # track_id -> [(x, y, t)] over acquire_motion_window_s. Populated every frame, read only
        # while ACQUIRING; see _note_motion.
        self._acq_motion = {}
        self._odom_xy = None

        self._last_state_text = None
        self._last_state_pub = 0.0
        self._state_detail = ''

        self._targets_sub = self.create_subscription(
            TargetArray, '/dtp/targets', self._on_targets, qos_profile_sensor_data)
        self._odom_sub = self.create_subscription(
            Odometry, '/lidar_slam/odom', self._on_odom, qos_profile_sensor_data)
        # BEST_EFFORT to match fsm_node's publisher (fsm_ros2.hpp qos); a RELIABLE subscription
        # would never match and never error -- the /planning/pos_cmd trap again.
        self._loss_event_sub = self.create_subscription(
            LossEvent, loss_event_topic, self._on_loss_event, qos_profile_sensor_data)

        # Mirrors follow_planner's own subscriber QoS exactly (best_effort, keep_last(1),
        # volatile -- follow_planner_node.hpp:1192). Depth 1 is intended, not a typo: for a moving
        # target a queued stale message is worse than no message, and BEST_EFFORT is not optional
        # because DDS reliability is asymmetric and a mismatch delivers ZERO messages with no
        # error anywhere (measured on hardware 2026-08-14: 2176 msgs best-effort vs 0 reliable).
        self._follow_target_pub = self.create_publisher(
            Target, follow_target_topic,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       depth=1,
                       durability=QoSDurabilityPolicy.VOLATILE))
        # Diagnostics stay RELIABLE for the same reason /mux/state does: this is the record of
        # which person had the lock at every instant, and a post-flight log with holes in it
        # cannot answer "why did it stop following?" -- or, worse, "who was it following?".
        self._state_pub = self.create_publisher(String, '/dtp/lock_state', 10)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        if self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is TRUE. This node ages targets on the ARRIVAL clock, so it will '
                'not misbehave on its own -- but with no /clock publisher every age reads 0.0 and '
                'no coast, abstain or lock timeout can ever fire. It must also match '
                'perception_node, follow_planner and fsm_node or SUPER\'s yaw gate fails '
                'permanently and invisibly.')
        self._warn_unsafe_settings()

        self.get_logger().info(
            'target_manager up. engage=%s (set follow_enabled to engage) | locked target -> %s | '
            'acquire %.1f-%.1f m, outside %.1f m of launch, %d confirm frames | coast %.2f s | '
            'band %.2f m + %.2f m/s (max %.2f m) | abstain %.2f s | height tie-break %s'
            % (self._engaged, follow_target_topic, self._acq_min_m, self._acq_max_m,
               self._ignore_radius, self._acq_frames, self._coast_timeout, self._band_m,
               self._band_growth, self._band_max_m, self._abstain_timeout,
               'on' if self._tiebreak_on else 'off'))
        self.get_logger().info(
            'loss conditioning %s (M6.5 arm; LossEvents from %s) | occluded coast cap %.1f s | '
            'window margin %.2f s | region max(%.2f m, %.1f sigma) | LOST confirm %.2f s'
            % ('ON' if self._conditioning else 'OFF -- carry-forward arm', loss_event_topic,
               self._occluded_cap, self._window_margin, self._region_min_r,
               self._region_sigma_k, self._lost_confirm))
        self._publish_state('startup', force=True)

    def _warn_unsafe_settings(self):
        """Re-state the two settings that are legal, silent, and dangerous.

        Separate from _validate_parameters because these are not errors -- both have real bench
        and replay uses -- so they cannot refuse to start. What they must not do is be QUIET. They
        are re-emitted after every live parameter change as well as at start-up, so turning one on
        mid-session is as loud as launching with it, rather than a line buried in the boot log.
        """
        if self._ignore_radius <= 0.0:
            self.get_logger().error(
                'launch_ignore_radius_m <= 0: the launch-area ignore disc is DISABLED. The first '
                'confirmed person after engage is then whoever is standing next to the aircraft, '
                'which is normally the safety pilot (R-F13). Bench use only.')
        if self._auto_reacquire:
            self.get_logger().error(
                'auto_reacquire_after_lost is TRUE: after a LOST this node will re-acquire the '
                'nearest confirmed person WITHOUT operator re-designation. That defeats D13/A10. '
                'Never fly this with a person in the volume.')
        if self._stock_baseline:
            self.get_logger().error(
                'stock_baseline is TRUE (M6.8 comparison arm): no coast, no ABSTAIN, and the lock '
                'is re-taken on the nearest confirmed person after every missed frame. It follows '
                'strangers by construction. Bench and replay only.')

    def _validate_parameters(self):
        """Reject or clamp the combinations that would silently disable a gate.

        Every check here is a value that produces no error at runtime -- it just makes one of the
        safety rules above stop applying, which is the failure mode this whole file is written
        against.
        """
        if self._acq_min_m >= self._acq_max_m:
            raise ValueError(
                'acquire_min_range_m (%.2f) >= acquire_max_range_m (%.2f): no range is eligible '
                'and this node would never lock onto anyone.' % (self._acq_min_m, self._acq_max_m))
        if self._acq_frames < 1:
            raise ValueError('acquire_confirm_frames must be >= 1; %d disables the acquisition '
                             'hysteresis entirely.' % self._acq_frames)
        if self._min_coast_horizon < 0.0:
            raise ValueError('min_coast_horizon_s must be >= 0; got %.3f'
                             % self._min_coast_horizon)
        if self._coast_timeout <= 0.0:
            raise ValueError('coast_timeout_s must be > 0; %.3f means any single missed frame '
                             'drops the lock, which no forest walk survives.'
                             % self._coast_timeout)
        if self._band_max_m < self._band_m:
            raise ValueError('reacquire_band_max_m (%.2f) < reacquire_band_m (%.2f): the cap '
                             'would shrink the band below its own base.'
                             % (self._band_max_m, self._band_m))
        if self._occluded_cap < self._coast_timeout:
            raise ValueError('occluded_coast_cap_s (%.2f) < coast_timeout_s (%.2f): the '
                             'conditioned coast would be shorter than the carry-forward one it '
                             'is compared to.'
                             % (self._occluded_cap, self._coast_timeout))
        if self._window_margin < 0.0 or self._region_min_r <= 0.0 or self._region_sigma_k < 0.0:
            raise ValueError('reacquire_window_margin_s and region_sigma_k must be >= 0, '
                             'region_min_radius_m > 0')
        if self._lost_confirm < 0.0 or self._loss_event_max_age <= 0.0:
            raise ValueError('lost_confirm_s must be >= 0 and loss_event_max_age_s > 0')
        if self._abstain_timeout > self._coast_timeout:
            # Not fatal, but it is a promise that cannot be kept: ABSTAIN is only ever entered
            # from a coast, and the coast clock is the outer bound, so the larger number is dead.
            self.get_logger().warn(
                'abstain_timeout_s (%.2f) exceeds coast_timeout_s (%.2f); the coast bound always '
                'expires first, so the abstain timeout can never fire.'
                % (self._abstain_timeout, self._coast_timeout))

    def _now(self):
        """Arrival-clock seconds.

        Every age in this node is measured from ARRIVAL, never from header.stamp -- the same
        choice command_mux's GoalSource documents. Cross-node header-stamp arithmetic breaks
        silently the moment two nodes disagree about use_sim_time, and the coast timeout is the
        one number here whose failure mode is "keep following an extrapolation forever".
        """
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ---------------------------------------------------------------------------

    def _on_set_parameters(self, params):
        """The engage/disengage EDGE, plus live re-tuning of every cached threshold.

        ENGAGE. Engaging is hysteretic downstream; DISENGAGING IS INSTANT. Asymmetric on purpose,
        matching command_mux's engage hysteresis: a safety de-escalation must never wait on a
        counter or on the next TargetArray.

        It is a rising edge, not a level, and that is what makes LOST latch: after a loss the
        parameter is still true, so nothing re-engages until the operator drives it false and back
        -- which is D13/A10's "request operator re-designation" expressed in the only interface
        this node has. `ros2 param set` with an unchanged value fires no callback at all.

        THE REST OF THE BATCH IS APPLIED, NOT MERELY ACKNOWLEDGED. See _LIVE_PARAMS for why
        acknowledging without applying is the worse of the two failures. The batch is ALL-OR-
        NOTHING and validated as a unit against _validate_parameters -- the same rules the
        constructor enforces, in one place rather than two -- because the illegal states are
        COMBINATIONS (min >= max, cap below base). Setting both halves of such a pair in one
        `ros2 param set` call must succeed; setting one half alone must not, and must not leave
        the node running on half a change. A rejected batch rejects the engage in it too, which
        is right: the operator asked for one thing, and a partly-applied safety gate is not it.
        """
        engage = None
        proposed = {}
        for p in params:
            if p.name == 'follow_enabled':
                engage = bool(p.value)
            elif p.name in _CONSTRUCTION_ONLY_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason='%s is read at construction (the publisher is already built against '
                           'it); restart the node to change it' % p.name)
            elif p.name in _LIVE_PARAMS:
                attr, cast = _LIVE_PARAMS[p.name]
                try:
                    proposed[attr] = cast(p.value)
                except (TypeError, ValueError) as exc:
                    return SetParametersResult(
                        successful=False, reason='%s: %s' % (p.name, exc))

        restore = {attr: getattr(self, attr) for attr in proposed}
        for attr, value in proposed.items():
            setattr(self, attr, value)
        try:
            self._validate_parameters()
        except ValueError as exc:
            for attr, value in restore.items():
                setattr(self, attr, value)
            self.get_logger().error('rejected parameter change: %s' % exc)
            return SetParametersResult(successful=False, reason=str(exc))

        if proposed:
            self.get_logger().info(
                'parameters applied live: %s'
                % ', '.join('%s=%s' % (a.lstrip('_'), v) for a, v in sorted(proposed.items())))
            self._warn_unsafe_settings()

        if engage is not None:
            if engage and not self._engaged:
                self._enter(LockState.ACQUIRING, 'operator engaged FOLLOW')
            elif not engage and self._engaged:
                self._drop_lock()
                self._enter(LockState.IDLE, 'operator disengaged FOLLOW')
            self._engaged = engage
        return SetParametersResult(successful=True)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if not _finite(p.x, p.y):
            # Never adopt a NaN pose: it would be latched forever as the launch point and would
            # make every ignore-radius test False, i.e. fail OPEN on the pilot-lock guard.
            self.get_logger().error('non-finite odom position; ignored',
                                    throttle_duration_sec=5.0)
            return
        self._odom_xy = (p.x, p.y)
        if self._launch_xy is None:
            self._launch_xy = self._odom_xy
            self.get_logger().info(
                'launch point latched at (%.2f, %.2f); no lock will be taken within %.1f m of it. '
                'Under camera_init this should be near the origin -- a large value means FAST-LIO '
                're-initialised and the disc is not where the aircraft took off.'
                % (p.x, p.y, self._ignore_radius))

    def _on_loss_event(self, msg: LossEvent) -> None:
        """Cache the oracle's newest verdict for OUR track. Decisions happen in _coast.

        Stored whether or not conditioning is on, so /dtp/lock_state can carry the class in
        both arms' logs; only _coast reads it, and only when loss_conditioning is true.
        """
        self._loss_events_seen += 1
        if self._locked_track_id is None or int(msg.track_id) != self._locked_track_id:
            return
        now = self._now()
        eval_t = msg.eval_stamp.sec + msg.eval_stamp.nanosec * 1e-9
        if abs(eval_t - now) > LOSS_EVENT_CLOCK_SKEW_MAX_S:
            if not self._clock_skew_warned:
                self._clock_skew_warned = True
                self.get_logger().error(
                    'LossEvent eval_stamp is %.1f s from this node\'s clock; fsm_node and '
                    'target_manager disagree about time (use_sim_time?). Loss conditioning '
                    'cannot place the re-acquisition window and will ignore every event.'
                    % (eval_t - now))
            return
        self._loss_event = msg
        self._loss_event_arrival = now
        cls = int(msg.cls)
        if cls in (LossEvent.CLS_LOST, LossEvent.CLS_UNMAPPED):
            if self._lost_verdict_since is None:
                self._lost_verdict_since = now
        else:
            self._lost_verdict_since = None
        if cls != self._loss_class:
            self._loss_class = cls

    def _fresh_loss_event(self, now):
        ev = self._loss_event
        if ev is None or self._loss_event_arrival is None:
            return None
        if (now - self._loss_event_arrival) > self._loss_event_max_age:
            return None
        return ev

    def _belief_window(self, ev):
        """(t_lo, t_hi, centre_xy, radius) of the dominant re-emergence cluster, as COAST AGES.

        The oracle's reappear_dt are offsets from ITS eval_stamp; the coast age of that instant is
        eval_stamp - last_seen_arrival (same wall clock in both nodes, guarded in _on_loss_event).
        None when the event carries no belief.
        """
        if ev is None or len(ev.reappear_p) == 0 or self._last_seen_arrival is None:
            return None
        eval_age = (ev.eval_stamp.sec + ev.eval_stamp.nanosec * 1e-9) - self._last_seen_arrival
        t_lo = eval_age + float(ev.reappear_dt_min[0]) - self._window_margin
        t_hi = eval_age + float(ev.reappear_dt_max[0]) + self._window_margin
        radius = max(self._region_min_r, self._region_sigma_k * float(ev.reappear_sigma_m[0]))
        return (t_lo, t_hi, (ev.reappear_p[0].x, ev.reappear_p[0].y), radius)

    def _extend_coast_to(self, horizon):
        """Extend the coast polyline at constant velocity so _coast_reference covers `horizon`."""
        if not self._coast_ts or self._coast_ts[-1] + DT_EPS_S >= horizon:
            return
        vx, vy = self._coast_velocity(self._coast_ts[-1])
        if not _finite(vx, vy):
            vx, vy = 0.0, 0.0
        lx, ly, lz = self._coast_pts[-1]
        ext = horizon - self._coast_ts[-1]
        self._coast_ts = self._coast_ts + [horizon]
        self._coast_pts = self._coast_pts + [(lx + vx * ext, ly + vy * ext, lz)]

    def _on_targets(self, msg: TargetArray) -> None:
        now = self._now()
        # Before any state logic: every candidate's motion history has to be accumulating
        # while we are still IDLE, or the first frame after engage has nothing to judge.
        self._note_motion(msg, now)
        if msg.header.frame_id and msg.header.frame_id != self._world_frame:
            # Warn, never reject or rewrite: SUPER ignores frame_id completely, so a mismatch
            # produces a silently wrong world position rather than an error. We cannot transform
            # it (we would have to know what the sender meant) but we can refuse to be quiet.
            self.get_logger().warn(
                "/dtp/targets frame_id '%s' != world_frame '%s'; nothing downstream validates "
                'frame_id, so a mismatch here is silently wrong rather than rejected'
                % (msg.header.frame_id, self._world_frame), throttle_duration_sec=5.0)

        if self._state in (LockState.IDLE, LockState.LOST):
            if self._state is LockState.LOST and self._auto_reacquire:
                self._enter(LockState.ACQUIRING, 'auto re-acquire after loss (D13 defeated)')
            else:
                # Heartbeat only -- deliberately NO detail. Passing one here (this used to pass
                # "no lock") overwrites the sticky reason within one frame of the transition, so
                # 66 ms after "coast timeout (1.50 s) with no re-acquisition" every later sample
                # of /dtp/lock_state read "LOST|-|no lock" instead. LOST and IDLE are the two
                # states a bag reader is most likely to open ON, and the reason for the loss is
                # the entire content of the message -- see _publish_state's docstring.
                self._publish_state()
                return

        if self._state is LockState.ACQUIRING:
            self._try_acquire(msg, now)
        elif self._state in (LockState.LOCKED, LockState.COASTING, LockState.ABSTAIN):
            self._service_lock(msg, now)

        self._publish_state()

    # --- acquisition ----------------------------------------------------------------------

    def _try_acquire(self, msg: TargetArray, now) -> None:
        if self._launch_xy is None:
            # Fail closed. The ignore radius is the ONLY thing standing between engage and a lock
            # on the safety pilot, and it cannot be evaluated without a launch point. Refusing to
            # acquire costs nothing (odom is a hard requirement of the whole follow chain anyway:
            # both follow_planner and command_mux veto on its absence) and guessing costs a lock
            # on whoever is nearest.
            self.get_logger().warn(
                'engaged but no /lidar_slam/odom yet, so the launch-ignore disc cannot be '
                'evaluated; refusing to acquire (R-F13)', throttle_duration_sec=5.0)
            self._acq_candidate_id = None
            self._acq_streak = 0
            return

        best = None
        for t in msg.targets:
            if not self._eligible_to_acquire(t):
                continue
            # Nearest-confirmed, exactly as plan 01 SS4 step 5 specifies. "Nearest" is only safe
            # because of the two gates already applied: the operator engaged deliberately, and the
            # launch disc excludes the person most likely to be nearest at takeoff. Ties break on
            # the lower track_id purely so the choice is deterministic and reproducible in replay.
            key = (float(t.distance_m), int(t.track_id))
            if best is None or key < best[0]:
                best = (key, t)

        if best is None:
            self._acq_candidate_id = None
            self._acq_streak = 0
            return

        cand = best[1]
        if int(cand.track_id) != self._acq_candidate_id:
            self._acq_candidate_id = int(cand.track_id)
            self._acq_streak = 1
        else:
            self._acq_streak += 1

        if self._acq_streak < self._acq_frames:
            return

        self._locked_track_id = int(cand.track_id)
        self._remember_seen(cand, msg, now)
        self._acq_candidate_id = None
        self._acq_streak = 0
        self._enter(LockState.LOCKED,
                    'locked track %d at %.2f m' % (self._locked_track_id, cand.distance_m))
        self._publish_target(self._locked_msg)

    def _eligible_to_acquire(self, t: Target) -> bool:
        if not _finite(t.position.x, t.position.y, t.position.z, t.distance_m):
            return False
        if not t.track_confirmed:
            # min_confirm_frames = 3 upstream. An unconfirmed track is a detection, not a track:
            # it has no motion history, so nothing about it can be checked against a forecast.
            return False
        if not (self._acq_min_m <= float(t.distance_m) <= self._acq_max_m):
            return False
        d_launch = math.hypot(t.position.x - self._launch_xy[0], t.position.y - self._launch_xy[1])
        if d_launch < self._ignore_radius:
            return False

        # MOTION GATE, AT ACQUISITION ONLY.
        #
        # MEASURED 2026-09-09: over a 194 s recording with ONE human present, 66.9 % of frames
        # carried more than one detection, and the most persistent track -- 535 of 568 sampled
        # frames at confidence 0.85 -- was a HUMANOID ROBOT standing in the lab. That is not a
        # detector error in any useful sense: a bipedal humanoid IS person-shaped, and it scored
        # nearly as high as the real subject (0.85 against 0.94). Ten of thirteen persistent
        # detection clusters in that room never moved.
        #
        # CONFIDENCE CANNOT SEPARATE THESE, and neither can appearance -- that is the difficulty.
        # What separates them is that a humanoid robot, a mannequin, a statue and a poster never
        # MOVE, while a person who wants to be followed can take a step.
        #
        # Gated on ACQUISITION ONLY, deliberately. Once locked, the subject may stand still for as
        # long as they like: requiring continuous motion would drop the lock on anyone who stops
        # walking, which is both useless and unsafe. This asks one question once -- "has this
        # candidate ever moved?" -- and never asks it again.
        #
        # THE HONEST LIMITATION: it cannot distinguish a statue from a person who has stood
        # perfectly still since this node started. It converts that ambiguity into a requirement on
        # the operator -- step once to be followed -- which is a reasonable thing to ask of someone
        # requesting a drone escort, and is not something the robot in the corner will do.
        if self._acq_motion_m > 0.0:
            hist = self._acq_motion.get(int(t.track_id))
            if not hist:
                return False
            travel = max(math.hypot(x - hist[0][0], y - hist[0][1]) for x, y, _ts in hist)
            if travel < self._acq_motion_m:
                return False
        return True

    def _note_motion(self, msg: TargetArray, now) -> None:
        """Short world-position history per candidate, for the acquisition motion gate.

        Kept here rather than read off Target.velocity because velocity is an INSTANTANEOUS
        estimate and reads ~0 for a walking person the moment perception holds a frame -- it
        measured a median 0.03 m/s across an entire walk before the nearest-surface fix landed.
        Displacement over a window answers "did this thing ever move" without depending on the
        velocity estimate being healthy at the instant we happen to ask.
        """
        for t in msg.targets:
            if not _finite(t.position.x, t.position.y):
                continue
            hist = self._acq_motion.setdefault(int(t.track_id), [])
            hist.append((t.position.x, t.position.y, now))
            cutoff = now - self._acq_motion_window_s
            while hist and hist[0][2] < cutoff:
                hist.pop(0)
        # Bound the dictionary. Track ids are not reused, this node runs for hours, and a room
        # full of transient false positives would otherwise grow it without limit.
        if len(self._acq_motion) > 256:
            live = {int(t.track_id) for t in msg.targets}
            for k in [k for k in self._acq_motion if k not in live]:
                del self._acq_motion[k]

    # --- lock servicing: hold, coast, re-acquire, abstain ---------------------------------

    def _service_lock(self, msg: TargetArray, now) -> None:
        seen = None
        for t in msg.targets:
            if int(t.track_id) == self._locked_track_id:
                seen = t
                break
        # A non-finite locked target is treated exactly like a missed one. Publishing it would
        # make follow_planner drop the whole Target and command_mux VETO, and adopting it as the
        # coast anchor would poison every subsequent band test.
        if seen is not None and not _finite(seen.position.x, seen.position.y, seen.position.z):
            self.get_logger().error('locked track %d carried a non-finite position; treating as '
                                    'unseen' % self._locked_track_id, throttle_duration_sec=2.0)
            seen = None

        if seen is not None and self._state is LockState.LOCKED:
            self._remember_seen(seen, msg, now)
            self._publish_target(self._locked_msg)
            return

        if seen is not None:
            # Reappearance of the SAME id. It still has to pass the band, because plan 04 SS2's
            # rule is a bounded position jump, not an id match: an id that jumps outside the band
            # is an upstream ID switch (the tracker re-associated our number onto someone else),
            # and that is the one case where trusting the id follows a stranger with full
            # confidence. In band, though, the id wins outright over any competing candidate --
            # ultralytics never recycles an id onto a different person within a session, so an id
            # match PLUS a motion match is strictly more evidence than a motion match alone.
            ref = self._coast_reference(now)
            if ref is not None and self._within_band(seen, ref, now):
                self._resume(seen, msg, now, 'same id back in band')
                return
            if self._conditioning and ref is not None and self._within_region(seen, now):
                self._resume(seen, msg, now, 'same id back inside the re-emergence region')
                return
            self.get_logger().warn(
                'locked track %d reappeared OUTSIDE the re-acquisition band -- upstream ID '
                'switch, not our person; continuing to coast' % self._locked_track_id,
                throttle_duration_sec=2.0)

        self._coast(msg, now)

    def _coast(self, msg: TargetArray, now) -> None:
        age = now - self._last_seen_arrival
        self._coast_frames += 1
        if self._stock_baseline:
            # M6.8: the stock follower has no notion of a coast. Drop the lock and start over.
            lost_id = self._locked_track_id
            self._drop_lock()
            self._enter(LockState.ACQUIRING,
                        'stock baseline: track %s missed a frame; re-acquiring at once' % lost_id)
            self._try_acquire(msg, now)
            return
        if self._state is LockState.LOCKED:
            self._enter(LockState.COASTING, 'target occluded')

        # --- M6.5: condition the coast on the oracle's verdict ------------------------------
        coast_limit = self._coast_timeout
        region = None      # (centre_xy, radius) while the belief's window is open
        ev = self._fresh_loss_event(now) if self._conditioning else None
        if ev is not None:
            cls = int(ev.cls)
            if cls in (LossEvent.CLS_LOST, LossEvent.CLS_UNMAPPED):
                held = (0.0 if self._lost_verdict_since is None
                        else now - self._lost_verdict_since)
                if held >= self._lost_confirm:
                    self._lose('loss oracle: %s for %.2f s (blocked %.2f, unknown %.2f) after '
                               '%.2f s of coasting; not an occlusion, so the coast ends (M6.5)'
                               % (LOSS_CLS_NAMES.get(cls, cls), held, ev.blocked_fraction,
                                  ev.unknown_fraction, age))
                    return
            elif cls == LossEvent.CLS_OCCLUDED:
                win = self._belief_window(ev)
                if win is not None:
                    t_lo, t_hi, centre, radius = win
                    # Coast to min(cap, t_max + margin) -- and never SHORTER than the carry-
                    # forward coast, so an optimistic belief cannot cost a re-acquisition the
                    # baseline would have made.
                    coast_limit = max(self._coast_timeout, min(self._occluded_cap, t_hi))
                    if t_lo <= age <= t_hi:
                        region = (centre, radius)
                else:
                    coast_limit = self._occluded_cap
                if not self._occluded_extended:
                    self._occluded_extended = True
                    self._publish_state('target occluded; oracle OCCLUDED, coast to %.1f s%s'
                                        % (coast_limit, '' if win is None else
                                           ', window %.1f-%.1f s, region r %.2f m at (%.1f, %.1f)'
                                           % (t_lo, t_hi, radius, centre[0], centre[1])))
                # _build_coasted needs the polyline to reach age + latency; cover the whole cap.
                self._extend_coast_to(coast_limit + self._latency_s + DT_EPS_S)

        if age > coast_limit:
            self._lose('coast timeout (%.2f s) with no re-acquisition' % coast_limit)
            return
        if (self._state is LockState.ABSTAIN
                and self._abstain_since is not None
                and (now - self._abstain_since) > self._abstain_timeout):
            self._lose('ambiguity persisted %.2f s; refusing to guess (D13/A10)'
                       % self._abstain_timeout)
            return

        ref = self._coast_reference(now)
        if ref is None:
            # The predictor's polyline has run out. Extrapolating past it would be flying on a
            # forecast nobody made -- and for a target that never had one (unconfirmed, or beyond
            # prediction_depth_gate_m) this fires on the first missed frame, which is the intended
            # and loud behaviour rather than a bug. See acquire_max_range_m.
            self._lose('CTRV forecast exhausted after %.2f s of coasting' % age)
            return

        # NONE of _eligible_to_acquire's gates apply here, and the asymmetry is deliberate rather
        # than an oversight -- it is the difference between CHOOSING a person and RECOGNISING one.
        #   * The range window is an acquisition policy ("do not adopt a lock we would immediately
        #     have to retreat from, or could not coast"). Re-applying it would drop a lock because
        #     the person we are already following walked to 11 m, which is a reason to keep
        #     following them, not to abandon them to a stranger.
        #   * The launch-ignore disc protects the CHOICE at engage, when "nearest confirmed
        #     person" is the whole rule and the safety pilot is standing next to the aircraft.
        #     During a re-acquisition the rule is a bounded jump from a CTRV forecast, which is a
        #     far stronger identity test than distance-from-launch ever was -- and re-applying the
        #     disc would drop the lock whenever our person walked back through the launch area.
        # The residual risk, stated rather than hidden: a bystander who is inside the band AND
        # confirmed AND alone in it gets the lock. That is what the ABSTAIN rule exists for, and
        # it is why the band is a bounded-jump gate rather than something that grows to cover
        # uncertainty (see reacquire_band_growth_mps).
        if region is not None:
            # M6.5 region gate: inside the belief's window, "the same person" means "inside the
            # region the map says they can re-emerge in", not "within a bounded jump of a CTRV
            # point that may by now be inside the trunk".
            cx, cy = region[0]
            in_band = [t for t in msg.targets
                       if _finite(t.position.x, t.position.y, t.position.z)
                       and math.hypot(t.position.x - cx, t.position.y - cy) <= region[1]]
        else:
            in_band = [t for t in msg.targets
                       if _finite(t.position.x, t.position.y, t.position.z)
                       and self._within_band(t, ref, now)]

        if len(in_band) >= 2:
            # AMBIGUOUS. The tie-break is the only thing allowed to resolve it, and only when it
            # is decisive; everything else abstains. Note the deliberate asymmetry with the
            # single-candidate case below: an UNCONFIRMED track still counts toward ambiguity but
            # can never be re-locked. Evidence that we might be wrong is accepted cheaply;
            # evidence of who this is must be strong.
            picked = self._height_tiebreak(in_band)
            if picked is not None:
                self._resume(picked, msg, now,
                             'height tie-break among %d candidates' % len(in_band))
                return
            if self._state is not LockState.ABSTAIN:
                self._abstain_since = now
                self._enter(LockState.ABSTAIN,
                            '%d candidates in the %.2f m band; refusing to guess'
                            % (len(in_band), self._band_radius(now)))
        elif len(in_band) == 1 and in_band[0].track_confirmed:
            self._resume(in_band[0], msg, now, 'sole candidate in the re-emergence region (M6.5)'
                         if region is not None else 'sole candidate in band')
            return
        elif self._state is LockState.ABSTAIN:
            # At most one candidate left: either the distractors walked out, or they all did.
            # Either way there is nothing to disambiguate, so fall back to a plain coast and let
            # the coast clock -- not the shorter abstain clock -- decide. A single UNCONFIRMED
            # candidate lands here too: it is not yet lockable (min_confirm_frames = 3 upstream,
            # 0.2 s at 15 Hz) but it is no longer an ambiguity.
            self._abstain_since = None
            self._enter(LockState.COASTING, 'band no longer ambiguous; resuming plain coast')

        if self._state is LockState.COASTING:
            coasted = self._build_coasted(now, ref)
            if coasted is None:
                # This is the NORMAL end of a coast, not an anomaly, and the message says so
                # rather than reading like an internal error. The forecast still covers where the
                # person is NOW (ref exists, or we would have lost above) but no longer covers
                # now + latency, which is the point the C5 contract requires predicted_path[0] to
                # be. So the usable coast is the predictor's horizon MINUS one latency -- ~1.3 s
                # of a 1.5 s forecast, i.e. we give up ~3 frames early at 15 Hz. Publishing an
                # uncompensated head instead would silently break C5 for both consumers, which is
                # strictly worse than ending a coast that had 0.2 s left in it.
                self._lose('forecast no longer covers now + %.2f s of latency compensation '
                           '(coasted %.2f s of a %.2f s horizon)'
                           % (self._latency_s, age, self._coast_ts[-1] if self._coast_ts else 0.0))
                return
            self._publish_target(coasted)
        # ABSTAIN publishes NOTHING -- plan 04 SS6's "abstain -> HOVER -> request operator
        # re-designation". follow_planner's 0.30 s timeout withholds the goal, which is the hover.

    def _resume(self, cand: Target, msg: TargetArray, now, reason: str) -> None:
        """Re-lock onto a candidate that passed the band, and publish it in the same callback.

        The track_id genuinely changes here whenever the tracker gave up and re-issued one, and
        follow_planner treats a changed track_id as a clean slate (ResetServoState, deadband and
        suspension cleared). That is correct and wanted: the goal history from before the
        occlusion describes a situation that no longer exists.
        """
        old = self._locked_track_id
        # Read the coast age BEFORE _remember_seen resets the clock it is measured against.
        gap_s = now - self._last_seen_arrival if self._last_seen_arrival is not None else 0.0
        self._locked_track_id = int(cand.track_id)
        self._abstain_since = None
        self._remember_seen(cand, msg, now)
        self._enter(LockState.LOCKED,
                    're-acquired as track %d (was %s) after %.2f s: %s'
                    % (self._locked_track_id, old, gap_s, reason))
        self._publish_target(self._locked_msg)

    def _height_tiebreak(self, candidates):
        """D13's optional cheap CPU-only cue. Returns a candidate only when it is DECISIVE."""
        if not self._tiebreak_on or self._locked_msg is None:
            return None
        confirmed = [t for t in candidates if t.track_confirmed]
        if len(confirmed) < 2:
            # With fewer than two confirmed candidates there is no tie to break: either the
            # ambiguity is between unconfirmed detections (wait for them) or there is one real
            # candidate and the sole-candidate path above already handled it.
            return None
        z_locked = self._locked_msg.position.z
        if not _finite(z_locked):
            return None
        ranked = sorted(confirmed, key=lambda t: abs(t.position.z - z_locked))
        best = abs(ranked[0].position.z - z_locked)
        runner_up = abs(ranked[1].position.z - z_locked)
        if runner_up - best < self._tiebreak_margin:
            return None
        return ranked[0]

    def _within_region(self, t: Target, now) -> bool:
        """True if the belief's window is open and t is inside the dominant region (M6.5)."""
        ev = self._fresh_loss_event(now)
        if ev is None or int(ev.cls) != LossEvent.CLS_OCCLUDED or self._last_seen_arrival is None:
            return False
        win = self._belief_window(ev)
        if win is None:
            return False
        t_lo, t_hi, (cx, cy), radius = win
        age = now - self._last_seen_arrival
        return t_lo <= age <= t_hi and math.hypot(t.position.x - cx, t.position.y - cy) <= radius

    def _band_radius(self, now):
        age = 0.0 if self._last_seen_arrival is None else max(0.0, now - self._last_seen_arrival)
        return min(self._band_m + self._band_growth * age, self._band_max_m)

    def _within_band(self, t: Target, ref, now) -> bool:
        # Horizontal only, matching every other distance in this stack (command_mux's veto is a
        # cylinder, follow_planner's standoff is a ground-plane radius, and distance_m is
        # hypot(x, y)). Including Z would let a height estimate that fell back to camera height
        # reject a perfectly good candidate.
        return math.hypot(t.position.x - ref[0], t.position.y - ref[1]) <= self._band_radius(now)

    def _coast_reference(self, now):
        """Where the forecast says the person is RIGHT NOW; None once the forecast is exhausted.

        Candidates are DETECTIONS, i.e. statements about now, so the band is centred on the
        forecast at the coast age -- not at age + latency, which is where the drone's setpoint is
        aimed. The two differ by one closed-loop latency and mixing them up would bias the band
        forward along the person's path by ~0.3 m at walking pace.

        TWO CLOCKS MEET HERE, KNOWINGLY. The polyline's parameter is an offset from the message's
        header.stamp (that is what predicted_path_dt means), and we evaluate it at an ARRIVAL-clock
        age -- see _now() for why every age in this node is an arrival age. Arrival is later than
        stamp by the transport+decode leg, ~36 ms measured (plan/dtp-imp/21), so the reference
        trails the true forecast position by that much: ~5 cm at walking pace, against a band that
        starts at 1.0 m. It is left uncorrected on purpose. Subtracting it would mean doing
        cross-node arithmetic between a header stamp and a local clock, which is exactly the
        construct that breaks silently and totally the first time two nodes disagree about
        use_sim_time -- and it would buy 5 cm on a 100 cm gate.
        """
        if self._last_seen_arrival is None:
            return None
        return _interp_polyline(self._coast_ts, self._coast_pts, now - self._last_seen_arrival)

    # --- message construction -------------------------------------------------------------

    def _remember_seen(self, tgt: Target, arr: TargetArray, now) -> None:
        """Store the seen target and rebuild the polyline the coast will be evaluated on."""
        # Captured before anything below overwrites them: a HELD frame needs the polyline we
        # already have. See _rebase_retained_forecast.
        prev_track = None if self._locked_msg is None else int(self._locked_msg.track_id)
        prev_ts, prev_pts, prev_seen = self._coast_ts, self._coast_pts, self._last_seen_arrival

        self._locked_msg = copy.deepcopy(tgt)
        self._locked_msg.is_locked_target = True
        if not self._locked_msg.header.frame_id:
            self._locked_msg.header = copy.deepcopy(arr.header)
        elif self._locked_msg.header.stamp.sec == 0 and self._locked_msg.header.stamp.nanosec == 0:
            # A zero stamp is not "now", it is 1970: SUPER's yaw gate would compute an age of
            # decades and drop /planning/yaw_target permanently, silently, in the one direction
            # nothing logs. The array's header is the same perception stamp, so borrow it.
            self._locked_msg.header.stamp = arr.header.stamp
            self.get_logger().warn('locked Target carried a zero header.stamp; filled from the '
                                   'TargetArray header', throttle_duration_sec=10.0)
        self._last_seen_arrival = now
        self._coast_frames = 0
        # A seen frame ends the coast: whatever the oracle said about it no longer applies.
        self._loss_event = None
        self._loss_event_arrival = None
        self._loss_class = None
        self._lost_verdict_since = None
        self._occluded_extended = False

        # [0] is the position that was actually SEEN, at dt = 0, so the polyline covers the gap
        # between "now" and the first predicted sample as well as the forecast itself. Without it
        # every coast shorter than one latency would fall off the front of the table.
        ts = [0.0]
        pts = [(tgt.position.x, tgt.position.y, tgt.position.z)]
        n_p, n_dt = len(tgt.predicted_path), len(tgt.predicted_path_dt)
        if n_p != n_dt:
            # command_mux discards the whole forecast on this mismatch; matching it keeps the two
            # consumers of the same message agreeing about what the target's future is.
            self.get_logger().error(
                'track %d: predicted_path (%d) and predicted_path_dt (%d) differ in length; the '
                'forecast is unusable and this target cannot be coasted'
                % (int(tgt.track_id), n_p, n_dt), throttle_duration_sec=5.0)
            n_p = 0
        for i in range(n_p):
            dt = float(tgt.predicted_path_dt[i])
            p = tgt.predicted_path[i]
            if not _finite(dt, p.x, p.y, p.z) or dt <= ts[-1] + DT_EPS_S:
                # Stop at the first sample that is not strictly later than the previous one. A
                # STALE forecast is normal, not exceptional: process_frame only re-predicts every
                # predict_every_n frames and otherwise reuses last_predictions with its ORIGINAL
                # absolute timestamps, so early samples can already be in the past by the time
                # they reach us. Truncating is right -- reordering would invent a path.
                break
            ts.append(dt)
            pts.append((p.x, p.y, p.z))
        # predicted_path_dt[0] IS the closed-loop latency perception compensated by (Target.msg,
        # plan 14 change C5). Reading it off the message rather than declaring our own copy is
        # what keeps the coasted reference point compensated by exactly the same amount as a live
        # one, so the standoff does not jump the moment the person steps behind a tree. Read it
        # BEFORE any rebase below: perception emits the compensated head at dt = latency even on a
        # held frame, so this is the one thing a held frame still tells us correctly.
        self._latency_s = (ts[1] - ts[0]) if len(ts) > 1 else 0.0

        if len(ts) <= 2 and int(tgt.track_id) == prev_track:
            rebased = self._rebase_retained_forecast(ts, pts, prev_ts, prev_pts, prev_seen, now)
            if rebased is not None:
                ts, pts = rebased

        # *** COAST FLOOR (min_coast_horizon_s). ***  If the forecast we were given ends before
        # the floor, extend it from its last point at constant velocity out to the floor. This
        # is NOT inventing a path the predictor declined to make: it is the zero-turn-rate case
        # of the same CTRV model, applied only to keep the LOCK alive through a dropped frame,
        # and it is what a warmed-up predictor's forecast already does for the last stretch of
        # its 1.5 s. A full-length forecast is untouched. Velocity comes from the message --
        # EXACTLY zero on thin history (perception_node._velocity) -- so a brand-new track is
        # coasted as stationary, which the growing re-acquisition band (1.0 m + 1.0 m/s * age)
        # is sized to absorb for a walking person. coast_timeout_s still bounds everything.
        floor = min(self._min_coast_horizon, self._coast_timeout)
        if ts and ts[-1] + DT_EPS_S < floor:
            lx, ly, lz = pts[-1]
            ext = floor - ts[-1]
            vx, vy = float(tgt.velocity.x), float(tgt.velocity.y)
            if not (_finite(vx, vy)):
                vx, vy = 0.0, 0.0
            ts = ts + [floor]
            pts = pts + [(lx + vx * ext, ly + vy * ext, lz)]
        self._coast_ts = ts
        self._coast_pts = pts

    def _rebase_retained_forecast(self, ts, pts, prev_ts, prev_pts, prev_seen, now):
        """Carry the previous forecast across a HELD frame, or None to keep the new one.

        THE FAILURE THIS EXISTS TO PREVENT, because it is not obvious and it fires in exactly the
        situation the coast is for. perception_node deliberately sends an EMPTY forecast on a HELD
        frame -- one where the torso patch had too few depth inliers, so it republished the last
        position rather than a bad one -- because the predictor's path is anchored to the position
        it just declined to publish. _predicted_path then falls through to its constant-velocity
        fallback, which emits exactly ONE point, the compensated head. So a held frame arrives
        carrying a forecast 0.20 s long instead of 1.5 s.

        Rebuilt naively, that REPLACES a full 1.5 s polyline with a stub, and the very next missed
        frame ends the coast immediately -- "forecast no longer covers now + latency" 66 ms after
        we had a second and a half of it. And holds are not independent of occlusions: a patch
        loses its inliers when the person is going BEHIND something, so "held frame, then
        occluded" is the ordinary onset of the event the coast exists to survive, not a corner
        case. The lock would die at the tree it was supposed to survive.

        The fix is exact rather than clever. Forecast samples are absolute positions at absolute
        times; only our storage is relative. After `elapsed` seconds the same sample sits at
        (old_dt - elapsed) relative to the new sighting, so shifting the time axis and dropping
        what went into the past preserves the real remaining horizon and invents nothing.

        Three conditions, each load-bearing:
          * SAME TRACK ONLY. On a re-acquisition (_resume) the id may have changed and the
            candidate is a bounded jump away from the old anchor; splicing the previous track's
            polyline onto a new position estimate would mix two different measurements of where
            the person is. A resumed track takes whatever forecast it arrives with.
          * The retained polyline must still reach FURTHER than the stub, or there is nothing to
            keep and the stub's own compensated head is the better anchor.
          * The held position stays at dt = 0. The forecast kept moving while the estimator held,
            so the first segment carries a real discontinuity; that is information about the hold,
            not an artefact to smooth away.

        THE HOLD IS INFERRED FROM MESSAGE SHAPE, NOT DECLARED. Nothing on /dtp/targets says "this
        position was held": missed_frames is always 0 there (see the module docstring) and
        Target.msg has no such field, so a one-point forecast is the only evidence that reaches
        us. The inference is safe in the direction that matters -- if it fires on a frame that was
        not held but whose forecast had simply gone fully stale, we keep a slightly older CTRV
        path anchored to the new position, which is bounded and small, instead of dropping to a
        0.20 s horizon. Getting it wrong the other way loses the lock outright.
        """
        if prev_seen is None or len(prev_ts) <= 2:
            return None
        elapsed = now - prev_seen
        if not _finite(elapsed) or elapsed < 0.0:
            return None
        if prev_ts[-1] - elapsed <= ts[-1] + DT_EPS_S:
            return None

        out_ts, out_pts = [ts[0]], [pts[0]]
        for i in range(1, len(prev_ts)):
            shifted = prev_ts[i] - elapsed
            if shifted <= out_ts[-1] + DT_EPS_S:
                continue          # already in the past relative to this sighting
            out_ts.append(shifted)
            out_pts.append(prev_pts[i])
        if len(out_ts) <= 2:
            return None
        self.get_logger().warn(
            'track %d arrived with a %d-point forecast (perception held this frame: the torso '
            'patch was unranged); carrying the previous CTRV path forward, %.2f s of horizon '
            'left. The held position is still the anchor -- only the forecast is retained.'
            % (int(self._locked_msg.track_id), len(ts) - 1, out_ts[-1]),
            throttle_duration_sec=2.0)
        return out_ts, out_pts

    def _build_coasted(self, now, ref):
        """The CTRV forecast advanced to now. None when the forecast cannot cover the reference."""
        age = now - self._last_seen_arrival
        t_ref = age + self._latency_s
        p_ref = _interp_polyline(self._coast_ts, self._coast_pts, t_ref)
        if p_ref is None:
            return None

        out = copy.deepcopy(self._locked_msg)
        # header untouched: still the stamp of the frame this target was last SEEN in.
        out.position.x, out.position.y, out.position.z = ref
        out.is_locked_target = True

        v = self._coast_velocity(age)
        out.velocity.x, out.velocity.y = v[0], v[1]
        # velocity.z = 0.0 for the same reason perception publishes it that way: nothing in this
        # pipeline estimates a human's vertical speed, and follow_planner uses only hypot(vx, vy).
        out.velocity.z = 0.0

        # Re-anchor the forecast on the coasted reference and keep every native sample after it,
        # with its ORIGINAL dt -- the dts stay honest because header.stamp never moved.
        path = [Point(x=p_ref[0], y=p_ref[1], z=p_ref[2])]
        dts = [t_ref]
        for i in range(1, len(self._coast_ts)):
            if self._coast_ts[i] <= t_ref + DT_EPS_S:
                continue
            p = self._coast_pts[i]
            path.append(Point(x=p[0], y=p[1], z=p[2]))
            dts.append(self._coast_ts[i])
        out.predicted_path = path
        out.predicted_path_dt = dts

        # Our own coast depth, in place of the always-zero upstream value. See the module
        # docstring: this is the only place the age of an extrapolation appears on the wire.
        out.missed_frames = int(self._coast_frames)
        # `confidence` on THIS topic means "how much of this message is still evidence rather than
        # extrapolation", decaying to zero across the coast window. It equals the detector value
        # exactly on a seen frame, and the detector's own conf threshold (0.25) floors that -- so
        # a reader can tell a coasted target from a seen one by the number alone, with no new
        # field and no message change. Target.msg still calls it "detector confidence", which is
        # true only of /dtp/targets.
        decay = max(0.0, 1.0 - age / self._coast_timeout)
        out.confidence = float(self._locked_msg.confidence) * decay

        # distance_m is drone-to-person, and the drone keeps flying while the person is hidden, so
        # the stored value goes stale immediately. Recompute it from live odom when we have it;
        # perception's own value is used unchanged everywhere else so that there is only ever one
        # statistic, and this is the one case where its input has moved and ours has not.
        if self._odom_xy is not None:
            out.distance_m = float(math.hypot(ref[0] - self._odom_xy[0],
                                              ref[1] - self._odom_xy[1]))
        return out

    def _coast_velocity(self, age):
        """Tangent of the coasted polyline, so the trail heading follows a CTRV arc."""
        t0 = max(self._coast_ts[0], age - 0.5 * COAST_VELOCITY_WINDOW_S)
        t1 = min(self._coast_ts[-1], age + 0.5 * COAST_VELOCITY_WINDOW_S)
        span = t1 - t0
        if span > DT_EPS_S:
            p0 = _interp_polyline(self._coast_ts, self._coast_pts, t0)
            p1 = _interp_polyline(self._coast_ts, self._coast_pts, t1)
            if p0 is not None and p1 is not None:
                return ((p1[0] - p0[0]) / span, (p1[1] - p0[1]) / span)
        # Degenerate window (a one-point polyline). Hold the velocity that was last measured
        # rather than publishing zero: follow_planner gates the trail heading at 0.25 m/s and
        # would otherwise re-anchor the standoff azimuth to wherever the drone happens to be.
        return (self._locked_msg.velocity.x, self._locked_msg.velocity.y)

    def _publish_target(self, tgt: Target) -> None:
        if not _finite(tgt.position.x, tgt.position.y, tgt.position.z,
                       tgt.velocity.x, tgt.velocity.y, tgt.velocity.z):
            # Never put a NaN on this topic. follow_planner drops the whole Target and
            # command_mux VETOES on one, so publishing it is strictly worse than the silence that
            # already means "no lock" -- and command_mux's _person_points turns a non-finite
            # position into "cannot clear a person we cannot locate", i.e. an unrecoverable HOLD.
            self.get_logger().error('refusing to publish a non-finite follow target for track %d'
                                    % int(tgt.track_id), throttle_duration_sec=2.0)
            return
        if len(tgt.predicted_path) != len(tgt.predicted_path_dt):
            self.get_logger().error('follow target for track %d has mismatched predicted_path '
                                    'lengths; dropping the forecast' % int(tgt.track_id),
                                    throttle_duration_sec=5.0)
            tgt.predicted_path = []
            tgt.predicted_path_dt = []
        self._follow_target_pub.publish(tgt)

    # --- state bookkeeping ----------------------------------------------------------------

    def _enter(self, state: LockState, reason: str) -> None:
        if state is self._state:
            return
        prev = self._state
        self._state = state
        text = 'lock state %s -> %s: %s' % (prev.name, state.name, reason)
        if state is LockState.LOST:
            self.get_logger().error(text)
        elif state is LockState.ABSTAIN:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self._publish_state(reason, force=True)

    def _drop_lock(self) -> None:
        self._locked_track_id = None
        self._locked_msg = None
        self._last_seen_arrival = None
        self._coast_ts = []
        self._coast_pts = []
        self._latency_s = 0.0
        self._coast_frames = 0
        self._abstain_since = None
        self._acq_candidate_id = None
        self._acq_streak = 0
        self._loss_event = None
        self._loss_event_arrival = None
        self._loss_class = None
        self._lost_verdict_since = None
        self._occluded_extended = False

    def _lose(self, reason: str) -> None:
        """LOST. Silence does the rest -- and the operator must re-designate before we try again.

        plan 04 SS2's diagram returns LOST to IDLE via "hover / NAV fallback", and IDLE only
        leaves on an explicit engage. That is not ceremony: a re-acquisition attempted after the
        forecast is gone has nothing left to match identity against and degenerates into "lock the
        nearest confirmed body", which is exactly the stranger-following failure A10 exists to
        prevent.
        """
        lost_id = self._locked_track_id
        self._drop_lock()
        # The track id goes into the REASON, not just the log line, because _drop_lock has already
        # cleared it and /dtp/lock_state's track column reads '-' from here on. That topic exists
        # to answer "who was it following?" after the fact; a LOST record that has dropped the id
        # answers the "why" and loses the "who" at the same instant.
        self._enter(LockState.LOST,
                    '%s (was track %s)' % (reason, lost_id if lost_id is not None else '?'))
        if self._auto_reacquire:
            return
        # LOST is its own latch: _on_targets returns early in this state, and follow_enabled is
        # still true, so only a false->true edge on it can start acquisition again.
        self.get_logger().error(
            'FOLLOW released (track %s). Re-designate to resume:\n'
            '    ros2 param set /target_manager follow_enabled false\n'
            '    ros2 param set /target_manager follow_enabled true'
            % (lost_id if lost_id is not None else '?'))

    def _publish_state(self, detail=None, force=False) -> None:
        """"STATE|track|detail" -- the same shape as /mux/state, for the same reasons.

        `detail` is STICKY: the per-frame heartbeat passes None and keeps whatever reason last
        explained the state. Letting it blank out would mean the transition line ("2 candidates in
        the 1.13 m band") survives for one message and every later sample of the same state says
        only "ABSTAIN" -- so a bag opened at any moment other than the transition loses the one
        piece of information a reader actually needs.
        """
        now = self._now()
        if detail is not None:
            self._state_detail = detail
        text = '%s|%s|%s' % (
            self._state.name,
            self._locked_track_id if self._locked_track_id is not None else '-',
            self._state_detail)
        unchanged = (text == self._last_state_text
                     and (now - self._last_state_pub) < STATE_HEARTBEAT_S)
        if not force and unchanged:
            return
        self._last_state_text = text
        self._last_state_pub = now
        self._state_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # Constructed inside the try, matching perception_node, so that a start-up refusal --
        # _validate_parameters raising on a combination that would disable a gate -- still runs
        # the shutdown below instead of leaving the context initialised.
        node = TargetManagerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how SIGTERM arrives -- the normal stop path for systemd
        # and `ros2 launch`. Without it, every managed shutdown exits with a stack trace.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
