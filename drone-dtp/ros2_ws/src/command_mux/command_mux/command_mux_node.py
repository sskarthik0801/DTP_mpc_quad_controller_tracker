"""command_mux -- arbitrates goal SOURCES into SUPER, plus the independent human-proximity veto.

Priority: SAFETY > MANUAL RC (enforced by PX4, never here) > FOLLOW > NAV (plan/dtp-imp/02 SS4.1).
Design in plan/dtp-imp/02 SS4 and plan/dtp-imp/04 SS4; milestone plan/dtp-imp/07 M2.2.

Two things this node must get right that are easy to get wrong:
  1. The independent human-proximity veto reads /dtp/targets (EVERY detected person) directly
     -- NOT /dtp/follow_goal. A bug in follow_planner's standoff-pose math must not be able to
     defeat the veto, so the veto's input is deliberately a separate subscription from the
     goal-arbitration inputs. For the same reason there is exactly ONE place in this file that
     publishes a goal, reached from one timer: a second publish path is a path that can skip
     the veto, and that is precisely the failure this layer exists to prevent.
  2. FOLLOW requires acquisition hysteresis before it can take authority from NAV -- otherwise
     a briefly-seen bystander yanks the drone out of NAV (risk A-F13). The engage / lock /
     launch-ignore-radius logic itself lives in target_manager (launch_ignore_radius_m is
     declared THERE, deliberately not duplicated here); this node only trusts that
     /dtp/follow_target -- and hence /dtp/follow_goal -- exists at all only once a real lock
     exists, and adds the NAV->FOLLOW *handover* hysteresis, which is a mux concern.

CORRECTED TOPIC WIRING #1 -- THE NAV INPUT
------------------------------------------
Caught while scaffolding (plan/dtp-imp/01 SS2): the architecture doc originally had this node
subscribing to the NAV goal on the SAME topic name (/goal_pose) it publishes its arbitrated
output to, which is a feedback loop, not arbitration. NAV sources (RViz "2D Nav Goal",
mission_planner, explorer) must be (re)configured via their own existing launch-time topic
params to publish to /dtp/nav_goal -- no code change is needed in those nodes, only launch args.

CORRECTED TOPIC WIRING #2 -- THE OUTPUT, AND WHY IT IS NOT /goal_pose
---------------------------------------------------------------------
The arbitrated goal goes to fsm/follow_goal_topic (/planning/follow_goal), not to /goal_pose.
That is load-bearing, not cosmetic. fsm_node's /goal_pose subscriber is goalCallback ->
setGoalPosiAndYaw (fsm.cpp:184), the RViz CLICK path: it OVERWRITES goal Z with cfg_.click_height
(1.5 m, destroying the D6 matched-height trail), silently skips any goal within 0.1 m of the
robot, silently returns on a deeply-occupied goal, and publishes NO GoalStatus at all -- so
follow_planner's entire ACCEPTED/SNAPPED/DROPPED contract would be unreachable dead code.
setFollowGoal (fsm.cpp:225) is the only ingestion path that preserves Z and answers on
/planning/goal_status, and it is reachable ONLY from fsm/follow_goal_topic.

Two consequences of that choice, both handled below:
  * fsm/follow_goal_en MUST be true. Its default is false (fsm/config.hpp:73), and when it is
    false fsm_node never creates the subscriber -- so this node publishes into a topic with no
    listener and the drone simply never receives a goal, with no error anywhere. That is the
    config mistake most likely to waste a flight day, so it is logged loudly at startup as
    something to verify on the aircraft, and /mux/state reports what we publish and where.
  * setFollowGoal deliberately does NOT apply click_height, so a NAV goal from RViz -- which
    publishes z = 0 -- would be commanded at ground level. This node substitutes nav_default_z
    for a zero-Z NAV goal (see _shape_goal) and says so in its log.

WHY SILENCE IS NOT A STOP (the single most important property of this node)
--------------------------------------------------------------------------
At the SETPOINT level, going silent IS a stop -- PX4's offboard-loss failsafe catches it. At the
GOAL level it is the exact opposite: setFollowGoal stores the goal in fsm's gi_ and SUPER keeps
flying to it indefinitely, so publishing nothing leaves the LAST goal active and the drone still
moving toward it. Therefore every condition that must stop the drone -- any veto, a stale
source, no source at all -- publishes a HOLD goal near the drone's current position. It never
merely stops publishing. plan/dtp-imp/02 SS4.1's "FOLLOW going stale (target lost) -> NAV/hover,
never a frozen last setpoint" is only achievable this way.

The one deliberate silence is THE ENGAGE GATE (below), and only while PX4 is NOT flying our
setpoints: there SUPER's goal moves nothing, and feeding it is what made the aircraft lurch at the
Offboard switch.

A corollary that decides the hold geometry: a hold goal placed EXACTLY at the current position
is worse than useless. closeToGoal(0.1) (fsm.cpp:133) sends the FSM straight back to WAIT_GOAL
*without planning anything*, and callReplanOnce returns early outside FOLLOW_TRAJ -- so the
drone finishes the trajectory it is already on, which is the one heading toward whatever
triggered the veto. The hold goal is therefore always displaced by at least
HOLD_MIN_DISPLACEMENT_M so SUPER actually replans and decelerates.

TWO-MUX CONTRACT, AND A HAZARD IT INHERITS (plan/dtp-imp/07 Step 2.2.3)
----------------------------------------------------------------------
fsm_ros2.hpp:427 publishes /planning/pos_cmd ONLY while the FSM is in FOLLOW_TRAJ or EMER_STOP.
Once the drone reaches the HOLD goal, closeToGoal(0.1) returns the FSM to WAIT_GOAL and
/planning/pos_cmd STOPS ENTIRELY -- a goal-level hover produces setpoint SILENCE, not a hover
setpoint stream. offboard_tracker must therefore synthesize the hold itself (Step 2.2.3: "a
goal-level hover MUST force the setpoint path to a defined hover/zero-velocity setpoint
initialized at the current state"); nothing this node can publish achieves that from here.
/mux/state's first '|'-separated field is a stable machine-readable state token precisely so
offboard_tracker -- and a bag reader asking "why did it stop?" -- can key off it instead of
parsing prose.

WHY EVERY VETO INPUT HAS ITS OWN TIMEOUT
----------------------------------------
A veto that fails OPEN is worse than no veto, because it is trusted. Every veto input therefore
vetoes on ABSENCE as well as on a bad value:
  * /dtp/sensors_ok never received, older than sensors_ok_timeout_s, or False -> veto. This
    closes a real measured gap: sensor_watchdog detects the 2026-09-07 silent Orbbec stall in
    0.33 s and until now NOTHING consumed its verdict. Note the boot trap
    (sensor_watchdog.py:154-160): inside its 20 s startup grace a topic that has NEVER published
    is only WARN, so sensors_ok reads True with no sensor alive at all. Value alone is not
    health -- the /dtp/targets freshness gate below is what actually covers that window.
  * /dtp/targets never received or older than targets_timeout_s -> veto. This REQUIRES
    perception_node to publish an EMPTY TargetArray every frame when nobody is detected;
    otherwise "nobody in view" and "perception is dead" are the same signal on the wire and no
    freshness gate can separate them. If perception ever stops doing that, this node vetoes
    permanently and names the reason in /mux/state -- the safe direction, and a loud one.
  * /lidar_slam/odom never received or older than odom_timeout_s -> veto. Odom is not decoration
    here: the proximity veto is a check on the PATH from the drone to the goal (plan 04 SS4 says
    "predicted closest approach", not instantaneous distance), so with no drone position there
    is no path to check and the veto cannot run at all.
  * A non-finite number in any goal, odom pose or target position -> veto. NaN silently defeats
    every "< radius" comparison in Python (nan < 1.5 is False), which is a fail-OPEN bug sitting
    inside a safety check.

Operationally surprising consequence, stated here so nobody discovers it in the field: a veto
blocks NAV as well as FOLLOW. An operator cannot click the drone away from a person it has
vetoed on -- the mux keeps holding. That is deliberate (plan 04 SS4 hard-vetoes ANY setpoint, and
a veto a NAV click can overrule is not a safety layer), and it is why the independent last line
of defence is the RC pilot: flipping the mode switch drops PX4 out of Offboard in hardware,
which nothing on the companion can block or delay.

NAV IS LATCHED, FOLLOW IS STREAMED -- AND THEY ARE JUDGED DIFFERENTLY
--------------------------------------------------------------------
source_stale_timeout_s (0.2 s) is written for a CONTINUOUS source. FOLLOW is one: follow_planner
publishes at goal_publish_rate_hz and its silence is the designed loss signal, so a stale FOLLOW
loses authority immediately. NAV is not: RViz's "2D Nav Goal" publishes ONE message and means
"fly there". Expiring it 0.2 s later would make NAV unusable and would also delete the very
fallback plan 02 SS4.1 asks for ("FOLLOW stale -> NAV/hover"). So the newest NAV goal stays
valid until superseded. Consequence worth knowing before it surprises someone in flight: when
FOLLOW drops, the drone resumes the last NAV goal even if it reached that point earlier. With no
NAV goal ever received, the fallback is HOLD.

ABORT: WHEN HOLD IS NOT ENOUGH (added 2026-09-11)
-------------------------------------------------
A HOLD is the right answer to a person too close or a follow source gone quiet. It is NOT the
right answer to the aircraft going blind: four silent sensor losses on the bench in one week
(three camera deaths, one LiDAR link drop, every one process-alive-and-publishing-nothing) mean
"hold with no perception and possibly no SLAM" is a real flight state, and it is a drone hovering
on whatever EKF still believes, unable to see anyone under it. Decision (user, 2026-09-11): that
case LANDS IN PLACE.

So the SENSORS / TARGETS / ODOM vetoes ESCALATE. If any of them persists for abort_after_s
(2.0 s) while this node has follow authority (it has asserted at least one goal AND PX4 reports
armed + Offboard), the node sends PX4 MAV_CMD_NAV_LAND on /fmu/in/vehicle_command -- the same
command QGC's Land button sends -- and repeats it until PX4 acknowledges. PROXIMITY never
escalates (landing next to the person is worse than holding above them); a missing or invalid
goal never escalates (that is the follow chain failing, not the aircraft going blind). The whole
decision, its timing and its retry policy live in command_mux/abort.py, which has no ROS in it
and is unit-tested with a fake clock; this file only feeds it and acts on what it returns.

This is the ONE place in the DTP stack that changes a PX4 flight mode without a human in the
loop. Kevin's px4_offboard_bridge deliberately never does (its docstring: "a node that can spin
motors on its own is the wrong shape"), and that split is respected here in the only way that
matters: this node can take the drone DOWN and cannot take it UP, ARM it, or put it in Offboard.
The RC pilot's mode switch overrides a Land as it overrides everything.

It is TERMINAL. After firing, the mux stays in HOLD arbitration with an ABORT veto until the
node is relaunched, so a camera that plugs back in while the drone is half-way down does not
quietly resume following. /mux/state reads ABORT|LAND|<why>, and /dtp/abort (latched) carries
the reason for the log.

abort_require_offboard:=false exists for ONE purpose: exercising this exact path on the bench
against a disarmed FC, where the ack can be observed. It is logged at ERROR every 5 s while set.

THE GEOFENCE (M5.2, added 2026-09-14, user decision: option 1 of docs/m5-m6-plan.md SS4 M5.2)
--------------------------------------------------------------------------------------------
Nothing else on this aircraft fences a follow flight: PX4's GF_MAX_HOR_DIST is inert without a
global home position (geofence.cpp:354) and EKF2_GPS_CTRL=0 gives none. So the box lives here,
at the goal level, as command_mux/fence.py (pure geometry, unit-tested):
  * Every goal this node publishes -- FOLLOW, NAV, and its own HOLD goals -- is clamped into the
    box inset by fence_margin_m. A person who walks out of the box is followed to the edge and
    no further. The clamp runs where the goal is shaped (so the proximity veto sees the clamped
    segment too, under the same both-goals rule as the descent guard) and is re-checked at the
    single publish point, where a bite is an ERROR because it means a path skipped the clamp.
  * The aircraft's OWN position is checked every tick. Outside the box at all: WARN + reported in
    /mux/state (the clamped goals are already commanding it back in). Further out than
    fence_breach_land_m: a FENCE veto (HOLD, hold goal clamped inside) that ESCALATES to
    NAV_LAND through the same path as a sensor loss -- PX4's own GF_ACTION=land semantics. A
    goal clamp alone is not a fence: SUPER routes AROUND obstacles and can leave the box doing
    so, and a drifting estimator carries the aircraft out while every goal reads "inside".
  * The box is a rectangle in lidar_init -- FAST-LIO's origin at the spot the stack was started,
    axes on the boot heading unless fence_yaw_deg turns it. The node REFUSES TO START on a box
    that does not contain the origin, since the aircraft starts there (estimator preflight E5).
  * It fences only what passes through this node. An RViz click on /goal_pose and the web GCS's
    remote-control card publish straight into fsm_node: operator actions with a human watching.
  * fence_enable:=false exists for a bench rig that deliberately drives goals far away. It is
    logged at ERROR every 5 s while set, and stage_a_preflight.py fails on it.

THE ENGAGE GATE (added 2026-09-22, user decision; command_mux/engage_gate.py)
---------------------------------------------------------------------------
No goal is published to SUPER until PX4 reports armed + Offboard. SUPER replans from its own
committed reference, never from the aircraft, so goals fed while the pilot flew in Position let
that reference walk away: at the 2026-09-21 20:05 Offboard switch pos_cmd was 3.0 m from the
aircraft, which closed the gap at 1.7 m/s. With the gate SUPER idles until the switch, and its
first plan starts from rest at the aircraft. Everything else -- arbitration, vetoes, the
descent-guard latch, /mux/state -- runs as before; only the publish is held, and /mux/state says
"to SUPER: held until Offboard". A HOLD latched while the gate was shut is dropped when it opens,
so the first goal is anchored on the aircraft as it is then.
  * It opens on a FRESH armed + Offboard report and shuts only on a fresh report that says
    otherwise. A stale report never shuts it: that would silence the proximity-veto HOLD in the
    air while PX4 was still in Offboard.
  * It reads PX4's state from the abort's subscription, so it is ON only with abort_enable true
    (every live follow). Observe-only sets abort_enable false and its goals go to the shadow
    topic, so there it is off. engage_gate_enable:=false turns it off for a bench rig; the node
    logs an ERROR at startup and /mux/state says ENGAGE GATE OFF.
  * When it shuts, SUPER is told to PARK (engage_gate.py; needs SUPER patch 0009): drop the goal
    and idle until the next one. Without that it re-plans to the stale goal on every tick for as
    long as the stack runs (2026-09-30: 45 minutes, fsm_node at 144 % of a core).
    engage_gate_park_enable:=false turns the park off.
  * Not covered: goals that bypass this node (RViz /goal_pose, the GCS RC card).

TERRAIN-RELATIVE HEIGHT (added 2026-09-21, user decision; OFF unless terrain_follow_enable)
-----------------------------------------------------------------------------------------
Every height in this node is otherwise absolute in lidar_init: z = 0 is the LiDAR's height where
the stack started. On the 2026-09-21 18:41 flight the field fell ~1.2 m along the corridor, the
goal held its lidar_init height, and the aircraft ended 3.4-3.7 m above the ground with the
walker out of the bottom of the camera frame. With terrain_follow_enable true:
  * terrain_estimator_node publishes the ground to follow on /dtp/terrain (terrain_map.py), and
    command_mux/terrain.py turns it into an OFFSET: the local ground minus the takeoff ground,
    rate-limited while PX4 flies our goals, clamped, held when stale.
  * The FOLLOW goal's z is REPLACED by terrain_follow_z_m (1.2, the same height as follow_planner's
    goal_z_min_m) plus the offset. follow_planner's own z is target z + 0.5 floored at 1.2, and
    the target z it reads is the walker's shins, off by ~1 m on a slope; it has been the 1.2
    floor on every flight so far. Adding the offset to it would count the slope twice whenever
    the target z did track it.
  * The goal clamps, the descent guard and the HOLD goal all work in the frame shifted down by
    the offset -- height above the local ground, in lidar_init units -- and the result is
    shifted back. So the descent guard's floor is a height above the ground: following the
    ground down with a person near is not "descending toward them", and without this the latched
    floor would hold the aircraft at the takeoff altitude all the way down the slope. The result
    is then clamped to [terrain_abs_z_min_m, goal_z_max_m] in lidar_init, which is where ROG-Map's
    virtual ground (-1.0, inflated to about -0.4) and ceiling (3.5) sit -- past them SUPER drops
    the goal. Those two are absolute, so terrain following is bounded to about -1.2 / +1.5 m of
    relief from the takeoff spot (terrain_max_down_m / terrain_max_up_m).
  * The proximity veto and the geofence are untouched: they compare absolute positions.
  * NAV goals stay absolute (an RViz click means a lidar_init point).
  * With the offset at 0 -- the switch off, or no estimate ever received -- every line of this is
    the absolute behaviour, exactly. ON with no /dtp/terrain logs an ERROR every 5 s.
  * "In Offboard" (the rate limit, and the HOLD exclusion in descent_guard.py) is read from
    vehicle_control_mode, which this node subscribes to only with abort_enable true -- every live
    follow. With it false (observe-only) the offset snaps, which is harmless: nothing is flown.

THE BRAKING ENVELOPE AND SPEED-SCALED RADII (added 2026-09-27; OFF unless brake_envelope_enable)
-----------------------------------------------------------------------------------------------
The 1.5 m veto radius and the 3.0/3.5 m descent-guard radii were sized for ~1 m/s. With
brake_envelope_enable true (command_mux/brake_envelope.py has the formula and the reasoning):
  * The aircraft's horizontal velocity is the /lidar_slam/odom pose differenced over 0.2 s (its
    twist is all zeros on this stack).
  * The veto ALSO tests the aircraft's braking envelope: the path it would fly if a stop were
    commanded now (react, then brake to rest, plus the tracking overshoot), against every
    person's forecast AT THE SAME INSTANT, with the same radius, cylinder and latch. And the
    goal end of the commanded path is pushed out by the tracking overshoot. Both act along the
    direction of travel; the lateral radius is unchanged, and at zero speed nothing new can veto.
  * The descent guard's trip and release radii grow by the speed-scaled "extra" (closing speed
    -> extra, grown at once, held, shrunk slowly). follow_planner's speed-scaled standoff, which
    adds the same extra to its band, is NOT on this branch (2026-10-01 port onto the rollback,
    user): the guard's release radius is then wider than the standoff in a sprint, which only
    keeps the floor latched longer.
  * OFF (the node default): every line of this is skipped, and the node is what it was before
    2026-09-27.
  * The pass-by radius and the flyable HOLD that were built on top of this (2026-09-30) are not
    on this branch.

WHAT THIS NODE DELIBERATELY DOES NOT DO
---------------------------------------
  * No range-scaled closing-speed limit. Speed is not commanded here -- this node publishes
    positions at a few Hz. A closing-speed limit has to live where velocity is actually produced:
    traj_opt's max_vel (one shared cap that also bounds the emergency BACKUP trajectory, see
    CLAUDE.md) and offboard_tracker's setpoint stream. Faking one by throttling goal motion would
    look like protection without being any.
  * No max-descent-RATE limit, for the same reason. What IS available at goal level, and is
    implemented, is the geometric half of R-F9: never command a goal below the drone while a
    person is within descent_guard_radius_m. Since 2026-09-17 it LATCHES an altitude floor with
    a release margin and dwell (command_mux/descent_guard.py), because re-deciding it every tick
    at the follow standoff made the goal's altitude flap.
  * No launch_ignore_radius_m, no lock/re-ID logic: target_manager owns those (its docstring
    states the ownership). Duplicating the radius here would create two numbers to keep in sync
    and a second, subtly different opinion about who is being followed.
  * No RC arbitration. A pilot flipping the mode switch drops PX4 out of Offboard in hardware;
    nothing on the companion participates in that, and nothing here should pretend to. The
    ABORT above is not arbitration with the pilot: it is a Land the pilot can override.

Topics:
  Subscribes: /dtp/follow_goal (geometry_msgs/PoseStamped, FOLLOW source, from follow_planner),
              /dtp/nav_goal (geometry_msgs/PoseStamped, NAV source -- RViz/mission_planner/
              explorer, reconfigured to publish HERE, not to /goal_pose directly),
              /dtp/targets (dtp_msgs/TargetArray, independent proximity veto -- ALL people),
              /dtp/sensors_ok (std_msgs/Bool, sensor_watchdog's verdict -> sensor veto),
              /lidar_slam/odom (nav_msgs/Odometry, drone position: the start of the commanded
              path the proximity veto is computed against, and the anchor for the HOLD goal)
              /fmu/out/vehicle_control_mode (px4_msgs/VehicleControlMode, armed + offboard:
              the authority gate for ABORT, and THE ENGAGE GATE),
              /fmu/out/vehicle_command_ack (px4_msgs/VehicleCommandAck, PX4's answer to LAND)
  Publishes:  /planning/follow_goal (geometry_msgs/PoseStamped, the single arbitrated goal --
              fsm/follow_goal_topic, NOT /goal_pose; see CORRECTED TOPIC WIRING #2),
              /mux/state (std_msgs/String, "STATE|reason|detail", diagnostics + the Step 2.2.3
              hand-off token for offboard_tracker),
              /fmu/in/vehicle_command (px4_msgs/VehicleCommand, NAV_LAND, ABORT only),
              /dtp/abort (std_msgs/String, latched, the abort reason -- for the flight log)
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String

from dtp_msgs.msg import TargetArray

from command_mux import abort as abort_policy
from command_mux import brake_envelope as be
from command_mux.descent_guard import DescentGuard
from command_mux.engage_gate import EngageGate, ParkBurst
from command_mux.fence import Fence
from command_mux.terrain import TerrainOffset

# px4_msgs lives in Kevin's workspace, not ours. Import failure is recorded, not raised: with
# abort_enable false (bench without the FC overlay) the node must still run, and with it true
# the constructor refuses to start and names the missing overlay, which beats a NameError at
# the moment of the first abort.
try:
    from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleControlMode
    _PX4_IMPORT_ERROR = None
except ImportError as _exc:  # pragma: no cover - depends on the overlay
    VehicleCommand = VehicleCommandAck = VehicleControlMode = None
    _PX4_IMPORT_ERROR = _exc

# One decision per tick, one publish path. 50 Hz bounds the latency this node ADDS to the follow
# chain at 20 ms, small against the 150-250 ms closed-loop sense->setpoint budget (plan 14 C5),
# while the per-tick work (a few hundred distance computations) is trivial next to perception's.
CONTROL_RATE_HZ = 50.0

# Republish the currently-arbitrated goal at least this often even when it has not moved. The
# output QoS is BEST_EFFORT because fsm_node's own goal subscriber is best-effort keep_last(1),
# so one dropped datagram must not be able to lose a HOLD command. 2 Hz is slow enough that
# repeating an unchanged goal cannot thrash SUPER's replanner -- setFollowGoal sets
# gi_.new_goal on EVERY message received (fsm.cpp:239), which is risk A13.
GOAL_REPUBLISH_HZ = 2.0

# Suppresses byte-identical republishes and nothing more. Goal SMOOTHING belongs to
# follow_planner (its goal_deadband_m, ~0.15 m); a second, larger deadband here would only add
# latency to a stream that has already been deadbanded once.
GOAL_DEADBAND_M = 0.05

# /mux/state is published on every change plus this heartbeat, so a bag started mid-flight still
# shows the current arbitration rather than nothing until the next transition.
STATE_HEARTBEAT_S = 1.0

# A HOLD goal closer than this to the drone trips closeToGoal(0.1) (fsm.cpp:133), which returns
# the FSM to WAIT_GOAL without planning -- the drone then finishes the trajectory it is already
# flying instead of stopping. Any hold must therefore be displaced by more than 0.1 m; 0.15 m
# gives margin against odom noise without commanding a noticeable lurch.
HOLD_MIN_DISPLACEMENT_M = 0.15

# The latch below is released once the drone is further than this from its own hold point.
#
# WHY THIS EXISTS: the latch's original rationale -- a hold point recomputed from live odom runs
# away from a decelerating drone -- only holds while the drone is CONVERGING on the latch. It says
# nothing about a drone that is 20 m away. Without a release the latch is permanent, and since a
# veto that never clears (e.g. /dtp/targets never publishing, which is today's tree) keeps the mux
# in HOLD forever, the goal stays pinned at the drone's position AT BOOT. A pilot who then flies
# manually to 20 m and hands over to Offboard gets an immediate 20 m transit back at max_vel.
# 3 m is far beyond any deceleration overshoot at follow speeds and far below a manual reposition.
HOLD_REANCHOR_M = 3.0

# Hard floor on the predicted closest approach to ANY person (plan 04 SS4 item 2, "~1.5 m").
DEFAULT_PROXIMITY_VETO_RADIUS_M = 1.5

# The keep-out is a CYLINDER, not a sphere. A person is a ~1.8 m vertical object reported as one
# point: a sphere under-protects the case R-F9 is actually about (drone directly overhead,
# where 3-D distance reads "safe" while the props are at head height) and over-protects a
# harmless high fly-past. At the D6 matched-height trail the vertical term is inactive and the
# veto reduces to exactly the 1.5 m horizontal floor the plan specifies.
#
# THE BAND IS +/- THIS AROUND THE REPORTED POINT, AND IN FLIGHT THAT POINT IS NOT THE CENTROID.
# perception_node lifts the near-surface median of its torso patch, which sits 65 % of the way
# down the box: the lower body, near the ground. 2026-09-29 12:32 close pass: tracked z
# 0.26-0.72, aircraft z 2.27-2.73 (lidar_init), so at 2.0 the cylinder's top was where the
# aircraft was. (This comment used to say "2.0 m above/below the centroid covers head height".)
# 2.0 was flown up to and including 2026-09-29 and stays the default, per the switch convention;
# dtp_params.yaml carries 3.0 since 2026-09-30 -- ~3 m above the ground, head (~1.8 m) plus
# ~1.2 m of prop clearance.
DEFAULT_PROXIMITY_HALF_HEIGHT_M = 2.0

# Release only outside radius + margin, and only after the path has stayed clear for the dwell.
# Without both, a person walking parallel at exactly the floor makes the mux chatter between
# FOLLOW and HOLD at the tick rate, which is more dangerous than either state held steadily.
DEFAULT_PROXIMITY_RELEASE_MARGIN_M = 0.5
DEFAULT_VETO_RELEASE_DWELL_S = 1.0

# How far into a target's CTRV forecast the veto looks. Not the whole predicted_path: a 5 s
# forecast of a walking person spans ~7 m, so checking all of it would veto almost any maneuver
# on the grounds that someone MIGHT cross the corridor long after the drone has passed. 1.5 s
# covers the closed-loop latency (150-250 ms) plus several replan periods at 15 Hz, which is the
# window in which this goal is actually being flown.
DEFAULT_PREDICTION_HORIZON_S = 1.5

# /dtp/targets has NO specified publish rate anywhere in this repo -- it is whatever the
# YOLO/fusion pipeline achieves, and the concurrent-load test in plan 21 SS3.4 measured power and
# thermals but never achieved detection rate. 0.5 s is deliberately looser than the 0.2 s goal
# staleness: this must not veto on a normal detector hiccup, only on the pipeline being gone.
# RETUNE once the real rate is measured; until then treat it as provisional.
DEFAULT_TARGETS_TIMEOUT_S = 0.5

# /dtp/sensors_ok is a 5 Hz Bool (sensor_watchdog.py:132); 1.0 s rides out 4 consecutive losses.
DEFAULT_SENSORS_OK_TIMEOUT_S = 1.0

# /lidar_slam/odom is 10 Hz nominal. fsm.cpp:100 refuses to step the FSM on odom older than
# 0.1 s, so nothing here can react before SUPER does and this timeout must not pretend to --
# SUPER printing "No odom" is the first symptom by design. 0.3 s rides out one missed odom,
# because a veto we cannot execute (no position -> no hold point) helps nobody.
DEFAULT_ODOM_TIMEOUT_S = 0.3

# NAV -> FOLLOW handover hysteresis (plan 02 SS4.1, "K confirmed frames + dwell"). At
# follow_planner's 15 Hz goal rate 5 frames is 0.33 s, so the 0.5 s dwell dominates: ~7 goal
# periods of an unbroken healthy follow stream before FOLLOW takes authority. Comfortably longer
# than any single-frame detector glitch, comfortably shorter than target_manager's coast window.
# ASYMMETRIC ON PURPOSE: engaging FOLLOW is hysteretic, LOSING it is instant. A safety
# de-escalation must never wait on a counter.
DEFAULT_ENGAGE_HYSTERESIS_FRAMES = 5
DEFAULT_ENGAGE_DWELL_S = 0.5

# HOLD commands the current position raised by this much. Climbing is the response plan 04 SS4
# item 2 names ("hover/climb"); it is also the only direction that increases separation from a
# person below, and it guarantees the displacement HOLD_MIN_DISPLACEMENT_M requires.
DEFAULT_HOLD_CLIMB_M = 0.75

# ROG-Map virtual_ground_height = -1.0 and virtual_ceil_height = 3.5 (follow_test_ros2.yaml):
# a goal outside that OPEN interval sits in virtual occupancy and comes straight back DROPPED.
# Clamped with margin. follow_planner owns the primary clamp on the follow goal; this is the last
# line before SUPER and it is the ONLY clamp NAV goals ever get. A clamp that BITES is logged,
# because a silently relocated goal is the exact class of bug /planning/goal_status was added for.
DEFAULT_GOAL_Z_MIN_M = 0.5
DEFAULT_GOAL_Z_MAX_M = 3.0

# RViz's "2D Nav Goal" publishes z = 0. Since the arbitrated output goes to setFollowGoal, which
# deliberately does NOT apply cfg_.click_height, passing that through would command the ground.
# 1.5 m is exactly the click_height this substitution stands in for.
DEFAULT_NAV_DEFAULT_Z_M = 1.5
NAV_ZERO_Z_EPS_M = 1e-3

# The half of R-F9 that IS enforceable at goal level: never command a goal below the drone's
# current altitude while a person is within this horizontal radius. Wider than the veto radius
# on purpose -- descending toward someone should stop well before the hard floor is reached.
DEFAULT_DESCENT_GUARD_RADIUS_M = 3.0
# The guard latches an altitude floor (command_mux/descent_guard.py). It releases only after
# nobody has been within radius + this margin for the dwell -- the proximity veto's own numbers,
# so a follow at a 3-4 m standoff stays latched instead of flapping on the 3.0 m line.
DEFAULT_DESCENT_GUARD_RELEASE_MARGIN_M = 0.5
DEFAULT_DESCENT_GUARD_RELEASE_DWELL_S = 1.0
# The floor is raised to the drone's altitude only when the drone is this far above it. Above the
# largest altitude-hold error measured in flight (+0.215 m, 2026-09-15), so the floor does not
# creep upward on tracking noise; a pass over an obstacle does raise it. A HOLD's climb does NOT
# (2026-09-21): _report_hold_to_guard reports each HOLD flown in Offboard, and the guard sees the
# drone at its altitude at HOLD entry until it is back down (descent_guard.py, "A HOLD'S CLIMB
# DOES NOT MOVE THE FLOOR").
DEFAULT_DESCENT_GUARD_RAISE_MARGIN_M = 0.3

# ---- TERRAIN-RELATIVE HEIGHT (module docstring section, and command_mux/terrain.py) ----
# The takeoff ground in lidar_init: the LiDAR is ~0.15 m up when the stack starts on the ground
# (the ToF read 0.15-0.16 on the ground on 2026-09-21, and the LiDAR map put the ground at -0.15).
DEFAULT_TERRAIN_REF_Z_M = -0.15
DEFAULT_TERRAIN_FOLLOW_Z_M = 1.2
# Clamps on the offset. With the follow height at 1.2 these keep the goal in [0.0, 2.7]: above
# ROG-Map's inflated virtual ground (-0.4) and below the 3.0 goal ceiling.
DEFAULT_TERRAIN_MAX_UP_M = 1.5
DEFAULT_TERRAIN_MAX_DOWN_M = 1.2
# Climb faster than descend. At the follow speed (0.6 m/s on 2026-09-21) 0.2 m/s of descent keeps
# up with a 33 % downhill; the field that day was ~4 %.
DEFAULT_TERRAIN_RATE_UP_MPS = 0.3
DEFAULT_TERRAIN_RATE_DOWN_MPS = 0.2
DEFAULT_TERRAIN_DEADBAND_M = 0.05
# The estimator publishes at ~5 Hz; five missed messages is stale, and stale HOLDS the offset.
DEFAULT_TERRAIN_TIMEOUT_S = 1.0
# The lowest absolute goal in terrain mode: ROG-Map virtual_ground_height -1.0 + inflation 0.4 +
# robot_r 0.2 = -0.4, plus 0.1 of margin.
DEFAULT_TERRAIN_ABS_Z_MIN_M = -0.3

# ---- THE BRAKING ENVELOPE (module docstring, and command_mux/brake_envelope.py) ----
# Every default below is the one brake_envelope.SpeedParams documents. OFF by default.
DEFAULT_SPEED_REACT_S = 0.5         # perception age ~0.21 s + SUPER replan_forward_dt 0.1 + ticks
DEFAULT_SPEED_BRAKE_MPS2 = 2.0      # SUPER's max_acc: it never plans a harder stop than this
DEFAULT_SPEED_TRACK_S = 0.4         # overshoot per m/s with NO acceleration feed-forward (0.8 m
DEFAULT_SPEED_TRACK_MAX_M = 2.0     # flown at 2.25 m/s). Once the FF has flown: 0.25 / 0.25.
DEFAULT_SPEED_BASE_FLOOR_M = 2.5    # follow_planner's min_standoff_m
DEFAULT_SPEED_MAX_EXTRA_M = 1.5     # the cap
DEFAULT_SPEED_HOLD_S = 1.0
DEFAULT_SPEED_RELEASE_MPS = 0.5
# People further than this do not widen the descent guard: their closing speed says nothing
# about a 3-4.5 m radius. The acquisition band's far edge.
SPEED_PERSON_RANGE_M = 10.0

# ---- GEOFENCE (module docstring section THE GEOFENCE, and command_mux/fence.py) ----
# The plan's 30 m x 4 m box (docs/m5-m6-plan.md SS4 M5.2), laid out so the aircraft starts 2 m
# from the near end facing down the corridor: the follower flies BEHIND the subject, so the
# subject walks away from the start point. Re-cut per venue in dtp_params.yaml; the node refuses a
# box that does not contain the origin.
DEFAULT_FENCE_X_MIN_M = -2.0
DEFAULT_FENCE_X_MAX_M = 28.0
DEFAULT_FENCE_Y_MIN_M = -2.0
DEFAULT_FENCE_Y_MAX_M = 2.0
DEFAULT_FENCE_YAW_DEG = 0.0
# Goals are clamped this far INSIDE the edge. SUPER plans a path AROUND the goal, not to a point,
# and the S1 config's robot_r will be the measured prop radius + 0.05 (~0.3 m); 0.5 m keeps the
# planned envelope off the fence line for a goal sitting on the clamp.
DEFAULT_FENCE_MARGIN_M = 0.5
# The HARD boundary: the aircraft this far outside the box raises the FENCE veto, which lands the
# aircraft if it persists abort_after_s with follow authority. At S1's 0.5 m/s the aircraft covers
# 1 m in the 2 s window, so in practice crossing this line IS the land decision; 3 m is far
# enough that SUPER routing around an obstacle at the edge (robot_r + inflation < 1 m) never
# reaches it, and close enough that a drifting estimator is caught within the venue's margin.
# < 0 disables the hard boundary (clamp only) -- logged at startup as a choice.
DEFAULT_FENCE_BREACH_LAND_M = 3.0

# A quaternion of zero length is not a rotation; get_yaw_from_quaternion would turn it into an
# arbitrary heading rather than reject it (fsm_ros2.hpp only screens for NaN). Sloppy NAV
# publishers do emit (0,0,0,0), so anything shorter than this is replaced with identity.
QUAT_MIN_NORM = 1e-6

# ---- ABORT (see the module docstring section and command_mux/abort.py) ----
# 2.0 s: several watchdog detection periods (0.33-0.39 s measured), so a USB hiccup that recovers
# lands nobody, while a drone is never left blind for long. Every tick in between is a HOLD.
DEFAULT_ABORT_AFTER_S = 2.0
# NAV_LAND is resent at this period until PX4 acks ACCEPTED, up to ABORT_MAX_SENDS. 2 Hz x 10 =
# 5 s of trying; if PX4 has not taken a Land in 5 s the pilot is the answer, not more packets.
ABORT_REPEAT_PERIOD_S = 0.5
ABORT_MAX_SENDS = 10
# /fmu/out/vehicle_control_mode was measured at ~2 Hz on this aircraft (2026-09-11). A report
# older than this means the DDS agent is gone, and armed/offboard are unknown, not false.
PX4_MODE_STALE_S = 3.0
# PARK (engage_gate.py): the park message goes out this many times after the gate shuts, this far
# apart. The topic is BEST_EFFORT, and a lost park leaves SUPER planning for as long as it runs.
PARK_REPEATS = 3
PARK_PERIOD_S = 0.3
# PX4 uXRCE-DDS: BEST_EFFORT + TRANSIENT_LOCAL both ways (px4_offboard_bridge.py,
# offboard_engage.py -- the two conventions verified against this FC).
PX4_QOS_DEPTH_PUB = 1
PX4_QOS_DEPTH_SUB = 5

STATE_FOLLOW = 'FOLLOW'
STATE_NAV = 'NAV'
STATE_HOLD = 'HOLD'
STATE_ABORT = 'ABORT'

VETO_SENSORS = 'SENSORS'
VETO_TARGETS = 'TARGETS'
VETO_ODOM = 'ODOM'
VETO_PROXIMITY = 'PROXIMITY'
VETO_GOAL_INVALID = 'GOAL_INVALID'
VETO_FENCE = 'FENCE'
VETO_ABORT = 'ABORT'


def _finite(*values):
    """True only if every value is a real, finite number.

    Exists because the alternative -- letting a NaN reach a comparison -- fails OPEN. Every
    `nan < radius` test is False, so one NaN coordinate anywhere upstream silently disables the
    proximity veto while leaving it looking like it ran.
    """
    for v in values:
        if not isinstance(v, float) and not isinstance(v, int):
            return False
        if math.isnan(v) or math.isinf(v):
            return False
    return True


def _horiz_dist_point_to_segment(px, py, ax, ay, bx, by):
    """Exact 2-D distance from (px,py) to the segment (ax,ay)-(bx,by).

    Horizontal-only and exact, because the keep-out is a vertical cylinder: a cylinder can only
    be entered if the path's horizontal projection enters the disc, so this test alone decides
    whether the vertical extent is even worth checking.
    """
    dx = bx - ax
    dy = by - ay
    den = dx * dx + dy * dy
    if den < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / den
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class GoalSource:
    """One arbitrated goal input: its latest message, when it ARRIVED, and its stream health.

    Ages are measured from ARRIVAL, never from header.stamp. Cross-node header-stamp arithmetic
    breaks silently the moment two nodes disagree about use_sim_time -- the same trap that
    permanently and invisibly disables SUPER's kYawTargetTimeoutSec yaw gate (super_planner.h:102)
    with no error message anywhere. Arrival time cannot fail that way.
    """

    def __init__(self, name):
        self.name = name
        self.msg = None
        self.last_arrival = None
        self.stream_start = None    # when the current unbroken stream began
        self.stream_frames = 0      # messages in the current unbroken stream

    def note(self, msg, now, stale_timeout_s):
        if self.last_arrival is None or (now - self.last_arrival) > stale_timeout_s:
            # A gap longer than the staleness window starts a NEW stream rather than continuing
            # the old one, so the engage hysteresis restarts from zero instead of resuming a
            # count earned before the dropout.
            self.stream_start = now
            self.stream_frames = 0
        self.msg = msg
        self.last_arrival = now
        self.stream_frames += 1

    def age(self, now):
        return None if self.last_arrival is None else now - self.last_arrival

    def fresh(self, now, stale_timeout_s):
        age = self.age(now)
        return age is not None and age <= stale_timeout_s

    def break_stream(self):
        self.stream_frames = 0
        self.stream_start = None

    def age_str(self, now):
        age = self.age(now)
        return 'never' if age is None else '%.2fs' % age


class CommandMuxNode(Node):
    def __init__(self):
        super().__init__('command_mux')

        # Per-source staleness watchdog (plan/dtp-imp/02 SS4): drop a source older than this and
        # fall through to the next priority rather than act on a frozen goal. Applies to FOLLOW
        # only -- see "NAV IS LATCHED, FOLLOW IS STREAMED" in the module docstring.
        self.declare_parameter('source_stale_timeout_s', 0.2)
        # Independent human-proximity veto hard floor (plan/dtp-imp/04 SS4).
        self.declare_parameter('proximity_veto_radius_m', DEFAULT_PROXIMITY_VETO_RADIUS_M)
        self.declare_parameter('proximity_veto_half_height_m', DEFAULT_PROXIMITY_HALF_HEIGHT_M)
        self.declare_parameter('proximity_release_margin_m', DEFAULT_PROXIMITY_RELEASE_MARGIN_M)
        self.declare_parameter('veto_release_dwell_s', DEFAULT_VETO_RELEASE_DWELL_S)
        self.declare_parameter('prediction_horizon_s', DEFAULT_PREDICTION_HORIZON_S)
        self.declare_parameter('targets_timeout_s', DEFAULT_TARGETS_TIMEOUT_S)
        self.declare_parameter('sensors_ok_timeout_s', DEFAULT_SENSORS_OK_TIMEOUT_S)
        self.declare_parameter('odom_timeout_s', DEFAULT_ODOM_TIMEOUT_S)
        self.declare_parameter('engage_hysteresis_frames', DEFAULT_ENGAGE_HYSTERESIS_FRAMES)
        self.declare_parameter('engage_dwell_s', DEFAULT_ENGAGE_DWELL_S)
        self.declare_parameter('hold_climb_m', DEFAULT_HOLD_CLIMB_M)
        self.declare_parameter('goal_z_min_m', DEFAULT_GOAL_Z_MIN_M)
        self.declare_parameter('goal_z_max_m', DEFAULT_GOAL_Z_MAX_M)
        self.declare_parameter('nav_default_z_m', DEFAULT_NAV_DEFAULT_Z_M)
        self.declare_parameter('descent_guard_radius_m', DEFAULT_DESCENT_GUARD_RADIUS_M)
        self.declare_parameter('descent_guard_release_margin_m',
                               DEFAULT_DESCENT_GUARD_RELEASE_MARGIN_M)
        self.declare_parameter('descent_guard_release_dwell_s',
                               DEFAULT_DESCENT_GUARD_RELEASE_DWELL_S)
        self.declare_parameter('descent_guard_raise_margin_m',
                               DEFAULT_DESCENT_GUARD_RAISE_MARGIN_M)
        # THE BRAKING ENVELOPE (module docstring). Off by default: bench-gated, never flown.
        self.declare_parameter('brake_envelope_enable', False)
        self.declare_parameter('speed_react_s', DEFAULT_SPEED_REACT_S)
        self.declare_parameter('speed_brake_mps2', DEFAULT_SPEED_BRAKE_MPS2)
        self.declare_parameter('speed_track_s', DEFAULT_SPEED_TRACK_S)
        self.declare_parameter('speed_track_max_m', DEFAULT_SPEED_TRACK_MAX_M)
        self.declare_parameter('speed_base_floor_m', DEFAULT_SPEED_BASE_FLOOR_M)
        self.declare_parameter('speed_max_extra_m', DEFAULT_SPEED_MAX_EXTRA_M)
        self.declare_parameter('speed_hold_s', DEFAULT_SPEED_HOLD_S)
        self.declare_parameter('speed_release_mps', DEFAULT_SPEED_RELEASE_MPS)
        # SUPER's max_acc, passed by follower.launch.py from the SUPER config it already reads.
        # 0 = not known (a bare `ros2 run`); then the braking deceleration cannot be checked.
        self.declare_parameter('super_max_acc_mps2', 0.0)
        # TERRAIN-RELATIVE HEIGHT (module docstring). Off by default: bench-gated, never flown.
        self.declare_parameter('terrain_follow_enable', False)
        self.declare_parameter('terrain_topic', '/dtp/terrain')
        self.declare_parameter('terrain_ref_z_m', DEFAULT_TERRAIN_REF_Z_M)
        self.declare_parameter('terrain_follow_z_m', DEFAULT_TERRAIN_FOLLOW_Z_M)
        self.declare_parameter('terrain_max_up_m', DEFAULT_TERRAIN_MAX_UP_M)
        self.declare_parameter('terrain_max_down_m', DEFAULT_TERRAIN_MAX_DOWN_M)
        self.declare_parameter('terrain_rate_up_mps', DEFAULT_TERRAIN_RATE_UP_MPS)
        self.declare_parameter('terrain_rate_down_mps', DEFAULT_TERRAIN_RATE_DOWN_MPS)
        self.declare_parameter('terrain_deadband_m', DEFAULT_TERRAIN_DEADBAND_M)
        self.declare_parameter('terrain_timeout_s', DEFAULT_TERRAIN_TIMEOUT_S)
        self.declare_parameter('terrain_abs_z_min_m', DEFAULT_TERRAIN_ABS_Z_MIN_M)
        # GEOFENCE (module docstring). A rectangle in lidar_init; see fence.py for the geometry.
        self.declare_parameter('fence_enable', True)
        self.declare_parameter('fence_x_min_m', DEFAULT_FENCE_X_MIN_M)
        self.declare_parameter('fence_x_max_m', DEFAULT_FENCE_X_MAX_M)
        self.declare_parameter('fence_y_min_m', DEFAULT_FENCE_Y_MIN_M)
        self.declare_parameter('fence_y_max_m', DEFAULT_FENCE_Y_MAX_M)
        self.declare_parameter('fence_yaw_deg', DEFAULT_FENCE_YAW_DEG)
        self.declare_parameter('fence_margin_m', DEFAULT_FENCE_MARGIN_M)
        self.declare_parameter('fence_breach_land_m', DEFAULT_FENCE_BREACH_LAND_M)
        # SUPER never validates frame_id -- ROG-Map ignores it entirely and treats the odom pose
        # as world-frame (m1_super_lidar.launch.py), so a wrong frame is accepted in silence.
        # 'world' is what SUPER's configs, RViz's fixed frame and the static TF already use.
        self.declare_parameter('world_frame', 'world')
        # Parameterised so a bench rig can point the arbitrated output at /goal_pose, but the
        # DEFAULT is the follow path on purpose -- see CORRECTED TOPIC WIRING #2.
        self.declare_parameter('arbitrated_goal_topic', '/planning/follow_goal')
        # ABORT (module docstring). abort_enable false ONLY for observe-only sorties where this
        # node has no authority over the aircraft (Stage A) -- there its output is shadowed and
        # landing someone else's flight would be wrong.
        self.declare_parameter('abort_enable', True)
        self.declare_parameter('abort_after_s', DEFAULT_ABORT_AFTER_S)
        # BENCH TEST ONLY. False lets the abort fire without armed+offboard so the exact path
        # can be exercised against a disarmed FC. Logged at ERROR every 5 s while false.
        self.declare_parameter('abort_require_offboard', True)
        # THE ENGAGE GATE (module docstring). False ONLY for a bench rig that wants goals with no
        # Offboard; logged at ERROR. Needs abort_enable (it reads PX4's state from that path).
        self.declare_parameter('engage_gate_enable', True)
        # PARK SUPER when the gate shuts (engage_gate.py). std_msgs/Empty on park_topic, which a
        # SUPER with patch 0009 subscribes; an older SUPER does not, and nothing changes for it.
        self.declare_parameter('engage_gate_park_enable', True)
        self.declare_parameter('park_topic', '/planning/park')
        self.declare_parameter('vehicle_command_topic', '/fmu/in/vehicle_command')
        self.declare_parameter('vehicle_command_ack_topic', '/fmu/out/vehicle_command_ack')
        self.declare_parameter('vehicle_control_mode_topic', '/fmu/out/vehicle_control_mode')

        self._stale_timeout = float(self.get_parameter('source_stale_timeout_s').value)
        self._veto_radius = float(self.get_parameter('proximity_veto_radius_m').value)
        self._veto_half_height = float(self.get_parameter('proximity_veto_half_height_m').value)
        self._release_margin = float(self.get_parameter('proximity_release_margin_m').value)
        self._release_dwell = float(self.get_parameter('veto_release_dwell_s').value)
        self._pred_horizon = float(self.get_parameter('prediction_horizon_s').value)
        self._targets_timeout = float(self.get_parameter('targets_timeout_s').value)
        self._sensors_timeout = float(self.get_parameter('sensors_ok_timeout_s').value)
        self._odom_timeout = float(self.get_parameter('odom_timeout_s').value)
        self._engage_frames = int(self.get_parameter('engage_hysteresis_frames').value)
        self._engage_dwell = float(self.get_parameter('engage_dwell_s').value)
        self._hold_climb = float(self.get_parameter('hold_climb_m').value)
        self._z_min = float(self.get_parameter('goal_z_min_m').value)
        self._z_max = float(self.get_parameter('goal_z_max_m').value)
        self._nav_default_z = float(self.get_parameter('nav_default_z_m').value)
        self._descent_guard = DescentGuard(
            float(self.get_parameter('descent_guard_radius_m').value),
            float(self.get_parameter('descent_guard_release_margin_m').value),
            float(self.get_parameter('descent_guard_release_dwell_s').value),
            float(self.get_parameter('descent_guard_raise_margin_m').value))
        self._brake_enable = bool(self.get_parameter('brake_envelope_enable').value)
        # Built even when disabled, so a bad value is refused at startup regardless of the flag.
        self._speed_p = be.SpeedParams(
            d_min_m=self._veto_radius,
            react_s=float(self.get_parameter('speed_react_s').value),
            brake_mps2=float(self.get_parameter('speed_brake_mps2').value),
            track_s=float(self.get_parameter('speed_track_s').value),
            track_max_m=float(self.get_parameter('speed_track_max_m').value),
            base_floor_m=float(self.get_parameter('speed_base_floor_m').value),
            max_extra_m=float(self.get_parameter('speed_max_extra_m').value),
            hold_s=float(self.get_parameter('speed_hold_s').value),
            release_mps=float(self.get_parameter('speed_release_mps').value))
        super_acc = float(self.get_parameter('super_max_acc_mps2').value)
        if self._brake_enable and super_acc > 0.0 and self._speed_p.brake_mps2 > super_acc + 1e-9:
            raise ValueError(
                'speed_brake_mps2 %.2f > SUPER max_acc %.2f: the braking envelope would assume a '
                'harder stop than SUPER ever plans. Lower speed_brake_mps2.'
                % (self._speed_p.brake_mps2, super_acc))
        self._super_acc = super_acc
        self._ac_vel_est = be.VelocityEstimator()
        self._person_vel = be.PersonVelocities()
        self._reach_filter = be.ReachFilter(self._speed_p)
        self._ac_vel = (0.0, 0.0)       # this tick's aircraft velocity; zero with the switch off
        self._v_close = 0.0             # the largest closing speed on any person this tick
        self._reach_extra = 0.0         # the filtered extra; 0.0 with the switch off
        self._terrain_enable = bool(self.get_parameter('terrain_follow_enable').value)
        self._terrain = TerrainOffset(
            float(self.get_parameter('terrain_ref_z_m').value),
            float(self.get_parameter('terrain_max_up_m').value),
            float(self.get_parameter('terrain_max_down_m').value),
            float(self.get_parameter('terrain_rate_up_mps').value),
            float(self.get_parameter('terrain_rate_down_mps').value),
            float(self.get_parameter('terrain_deadband_m').value),
            float(self.get_parameter('terrain_timeout_s').value))
        self._terrain_follow_z = float(self.get_parameter('terrain_follow_z_m').value)
        self._terrain_abs_z_min = float(self.get_parameter('terrain_abs_z_min_m').value)
        self._terrain_z = None          # the latest /dtp/terrain point.z (may be NaN)
        self._terrain_arrival = None
        self._dz = 0.0                  # this tick's terrain offset; 0.0 with the switch off
        self._terrain_logged = 0.0
        self._fence_enable = bool(self.get_parameter('fence_enable').value)
        # Built even when disabled, so a bad box is refused at startup regardless of the flag and
        # the flag can never hide a config error until the day someone turns it on.
        self._fence = Fence(
            float(self.get_parameter('fence_x_min_m').value),
            float(self.get_parameter('fence_x_max_m').value),
            float(self.get_parameter('fence_y_min_m').value),
            float(self.get_parameter('fence_y_max_m').value),
            math.radians(float(self.get_parameter('fence_yaw_deg').value)),
            float(self.get_parameter('fence_margin_m').value),
            float(self.get_parameter('fence_breach_land_m').value))
        if self._fence.clamp(0.0, 0.0)[2] != 0.0:
            raise ValueError(
                'geofence does not contain the origin (with its %.2f m margin): %s. The aircraft '
                'starts AT the origin (estimator preflight E5), so this box would put it in '
                'breach on the first tick. Fix fence_x/y_* in dtp_params.yaml.'
                % (self._fence.margin, self._fence.describe()))
        self._fence_outside_m = 0.0     # the aircraft's own breach distance, refreshed per tick
        self._fence_disabled_logged = 0.0
        self._world_frame = str(self.get_parameter('world_frame').value)
        goal_topic = str(self.get_parameter('arbitrated_goal_topic').value)
        self._abort_enable = bool(self.get_parameter('abort_enable').value)
        # Re-read every tick (see _service_abort) so the bench test can flip it with
        # `ros2 param set` on the live node; the ERROR log while it is false is the guard.
        self._abort_require_offboard = bool(self.get_parameter('abort_require_offboard').value)
        self._abort = abort_policy.AbortEscalation(
            float(self.get_parameter('abort_after_s').value),
            ABORT_REPEAT_PERIOD_S, ABORT_MAX_SENDS)
        self._px4_armed = False
        self._px4_offboard = False
        self._px4_mode_arrival = None
        self._px4_mode_warned = False
        # THE ENGAGE GATE. Only with abort_enable: that is where PX4's state comes from, and
        # observe-only (abort_enable false) publishes to the shadow topic anyway.
        self._engage_gate_enable = (bool(self.get_parameter('engage_gate_enable').value)
                                    and self._abort_enable)
        self._engage_gate = EngageGate(PX4_MODE_STALE_S)
        self._park_enable = (bool(self.get_parameter('engage_gate_park_enable').value)
                             and self._engage_gate_enable)
        self._park_topic = str(self.get_parameter('park_topic').value)
        self._park = ParkBurst(PARK_REPEATS, PARK_PERIOD_S)

        self._follow = GoalSource('follow')
        self._nav = GoalSource('nav')

        self._targets = []
        self._targets_arrival = None
        self._sensors_ok = None
        self._sensors_arrival = None
        self._odom_p = None
        self._odom_arrival = None

        self._prox_active = False
        self._prox_clear_since = None
        self._hold_point = None       # latched on entry to HOLD; see _hold_goal()
        self._hold_anchor = None      # the odom the hold was latched on (re-anchor reference)
        self._hold_reported = False   # this HOLD was reported to the descent guard
        # Until this node has seen ONE fully-healthy tick it must not assert any goal at all --
        # not even a HOLD. Publishing to /planning/follow_goal is what sets SUPER's started_ flag
        # (fsm.cpp:236-238), so a mux that boots straight into HOLD would, as its very first act,
        # command a goal 0.75 m above a landed drone and arm the FSM to fly it.
        self._ever_healthy = False
        self._state = None
        self._last_state_text = None
        self._last_state_pub = 0.0
        self._last_goal = None
        self._last_goal_pub = 0.0

        self._follow_goal_sub = self.create_subscription(
            PoseStamped, '/dtp/follow_goal', self._on_follow_goal, qos_profile_sensor_data)
        self._nav_goal_sub = self.create_subscription(
            PoseStamped, '/dtp/nav_goal', self._on_nav_goal, qos_profile_sensor_data)
        self._targets_sub = self.create_subscription(
            TargetArray, '/dtp/targets', self._on_targets, qos_profile_sensor_data)
        self._odom_sub = self.create_subscription(
            Odometry, '/lidar_slam/odom', self._on_odom, qos_profile_sensor_data)
        # sensor_watchdog publishes this RELIABLE, but subscribing BEST_EFFORT is the choice that
        # survives someone later making the publisher best-effort: DDS reliability is asymmetric,
        # a RELIABLE publisher satisfies a BEST_EFFORT subscriber, and only the reverse fails --
        # silently, which is how a safety input disappears without anyone noticing
        # (CLAUDE.md M0.7: BEST_EFFORT sub 2176 msgs vs RELIABLE sub 0). Losing an occasional
        # Bool is harmless here because the freshness gate, not the last value, is the guard.
        if self._terrain_enable:
            self.create_subscription(
                PointStamped, str(self.get_parameter('terrain_topic').value), self._on_terrain,
                10)
        self._sensors_sub = self.create_subscription(
            Bool, '/dtp/sensors_ok', self._on_sensors_ok,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       depth=5,
                       durability=QoSDurabilityPolicy.VOLATILE))

        # Mirrors fsm_node's own goal QoS (best_effort, keep_last(1), volatile): only the newest
        # goal matters, and queueing stale goals behind a transport hiccup would be worse than
        # dropping them. GOAL_REPUBLISH_HZ is what covers the drops.
        self._goal_pub = self.create_publisher(
            PoseStamped, goal_topic,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       depth=1,
                       durability=QoSDurabilityPolicy.VOLATILE))
        # Same QoS as the goal: fsm_node subscribes its park input best_effort, keep_last(1).
        self._park_pub = self.create_publisher(
            Empty, self._park_topic,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       depth=1,
                       durability=QoSDurabilityPolicy.VOLATILE))
        # Diagnostics stay RELIABLE: this is the record of which source had authority at every
        # instant (plan 02 SS4.1's "logged priority state machine"), and a post-flight log with
        # holes in it cannot answer "why did it stop?".
        self._state_pub = self.create_publisher(String, '/mux/state', 10)

        self._cmd_pub = None
        if self._abort_enable:
            if _PX4_IMPORT_ERROR is not None:
                raise RuntimeError(
                    'abort_enable is true but px4_msgs is not importable (%s). Source the '
                    'workspace that holds px4_msgs (~/super_ws) before launching, or set '
                    'abort_enable:=false for a bench rig with no flight controller.'
                    % _PX4_IMPORT_ERROR)
            px4_pub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                     history=QoSHistoryPolicy.KEEP_LAST,
                                     depth=PX4_QOS_DEPTH_PUB)
            px4_sub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                     history=QoSHistoryPolicy.KEEP_LAST,
                                     depth=PX4_QOS_DEPTH_SUB)
            self._cmd_pub = self.create_publisher(
                VehicleCommand, str(self.get_parameter('vehicle_command_topic').value),
                px4_pub_qos)
            self.create_subscription(
                VehicleCommandAck, str(self.get_parameter('vehicle_command_ack_topic').value),
                self._on_vehicle_command_ack, px4_sub_qos)
            self.create_subscription(
                VehicleControlMode, str(self.get_parameter('vehicle_control_mode_topic').value),
                self._on_vehicle_control_mode, px4_sub_qos)
        # Latched so a bag or a late subscriber still gets the reason.
        self._abort_pub = self.create_publisher(
            String, '/dtp/abort',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._tick)

        self.get_logger().info(
            'command_mux up. Arbitrated goal -> %s (NOT /goal_pose: only setFollowGoal preserves '
            'goal Z and answers on /planning/goal_status). VERIFY ON THE AIRCRAFT that '
            'fsm/follow_goal_en is true -- its default is false, and when false fsm_node never '
            'creates that subscriber and the drone silently never receives a goal. NAV sources '
            'must publish to /dtp/nav_goal, not /goal_pose.' % goal_topic)
        self.get_logger().info(
            'veto: radius %.2f m x +/-%.2f m, targets_timeout %.2f s, sensors_ok_timeout %.2f s, '
            'odom_timeout %.2f s; engage %d frames + %.2f s dwell'
            % (self._veto_radius, self._veto_half_height, self._targets_timeout,
               self._sensors_timeout, self._odom_timeout, self._engage_frames,
               self._engage_dwell))
        g = self._descent_guard
        self.get_logger().info(
            'descent guard: a goal below the drone with a person within %.2f m latches an '
            'altitude floor; released after %.1f s with nobody within %.2f m; raised when the '
            'drone is > %.2f m above it, except by a HOLD\'s own climb; reset on disarm'
            % (g.radius_m, g.release_dwell_s, g.radius_m + g.release_margin_m, g.raise_margin_m))
        if self._brake_enable:
            sp = self._speed_p
            self.get_logger().warn(
                'BRAKING ENVELOPE ON (bench-gated, never flown): the veto also tests the path '
                'of a stop commanded now (react %.2f s, brake %.2f m/s2, overshoot %.2f s/m '
                'capped %.2f m) against each forecast; the descent-guard radii grow by up to '
                '%.2f m with the closing speed (base floor %.2f, hold %.1f s, shrink %.2f m/s). '
                'Reach at 1/2/3 m/s closing: %.2f / %.2f / %.2f m. SUPER max_acc %s.'
                % (sp.react_s, sp.brake_mps2, sp.track_s, sp.track_max_m, sp.max_extra_m,
                   sp.base_floor_m, sp.hold_s, sp.release_mps,
                   be.reach(1.0, 1.0, sp), be.reach(2.0, 2.0, sp), be.reach(3.0, 3.0, sp),
                   '%.2f' % self._super_acc if self._super_acc > 0.0
                   else 'NOT KNOWN (not launched by follower.launch.py): brake not checked'))
        else:
            self.get_logger().info('braking envelope OFF: the veto tests the path to the goal '
                                   'only, descent-guard radii fixed (brake_envelope_enable false)')
        if self._terrain_enable:
            t = self._terrain
            self.get_logger().warn(
                'TERRAIN-RELATIVE HEIGHT ON (bench-gated, never flown): FOLLOW goals at %.2f m '
                'above the local ground (takeoff ground %.2f), from %s; offset within -%.1f/+%.1f '
                'm, %.2f m/s down / %.2f m/s up while in Offboard, held when stale (%.1f s); '
                'descent guard and HOLD in height above ground; absolute goal clamp [%.2f, %.2f]'
                % (self._terrain_follow_z, t.ref_z, self.get_parameter('terrain_topic').value,
                   t.max_down_m, t.max_up_m, t.rate_down_mps, t.rate_up_mps, t.timeout_s,
                   self._terrain_abs_z_min, self._z_max))
        else:
            self.get_logger().info('terrain-relative height OFF: every height absolute in '
                                   'lidar_init (terrain_follow_enable false)')
        if self._fence_enable:
            self.get_logger().info(
                'GEOFENCE ON: %s. FOLLOW, NAV and HOLD goals are clamped inside it; the aircraft '
                'past the hard boundary raises FENCE, which %s. It fences ONLY goals that pass '
                'through this node -- an RViz click on /goal_pose or the GCS remote-control card '
                'is not covered.'
                % (self._fence.describe(),
                   'escalates to NAV_LAND like a sensor loss' if self._abort_enable
                   else 'HOLDs (abort_enable is false, so it cannot land)'))
            if self._fence.breach_land_m < 0.0:
                self.get_logger().warn(
                    'fence_breach_land_m < 0: the geofence clamps goals but will NEVER land the '
                    'aircraft for leaving the box. A deliberate choice, and a weaker fence.')
        else:
            self.get_logger().error(
                'GEOFENCE DISABLED (fence_enable:=false): nothing bounds where the follower can '
                'take the aircraft. Bench only. stage_a_preflight.py fails on this.')
        if self._abort_enable:
            self.get_logger().info(
                'ABORT armed: %s persisting %.1f s while armed+offboard with follow authority '
                '-> NAV_LAND on %s (land in place), resent every %.1f s up to %d times until '
                'acked. Terminal until relaunch.'
                % ('/'.join(abort_policy.ABORT_CLASS), self._abort.after_s,
                   self.get_parameter('vehicle_command_topic').value,
                   ABORT_REPEAT_PERIOD_S, ABORT_MAX_SENDS))
            if not self._abort_require_offboard:
                self.get_logger().error(
                    'abort_require_offboard is FALSE: the abort will fire on a DISARMED aircraft. '
                    'Bench test only. Never fly with this set.')
        else:
            self.get_logger().warn(
                'ABORT DISABLED (abort_enable:=false): a sensor loss in flight will HOLD, not '
                'land. Correct only for an observe-only sortie where this node has no authority.')
        if self._engage_gate_enable:
            self.get_logger().info(
                'ENGAGE GATE ON: no goal reaches SUPER until PX4 reports armed + Offboard, so its '
                'first plan starts from the aircraft. /mux/state says "to SUPER: held until '
                'Offboard" while it is shut.')
            if self._park_enable:
                self.get_logger().info(
                    'PARK ON: when the engage gate shuts, SUPER is told on %s to drop its goal '
                    'and idle (needs SUPER patch 0009), so it does not re-plan to a stale goal '
                    'after a takeover.' % self._park_topic)
            else:
                self.get_logger().warn(
                    'PARK OFF (engage_gate_park_enable:=false): after a takeover SUPER keeps its '
                    'last goal and re-plans to it for as long as the stack runs.')
        elif self._abort_enable:
            self.get_logger().error(
                'ENGAGE GATE OFF (engage_gate_enable:=false): SUPER gets goals before Offboard '
                'and flies them in its own reference, so the aircraft can lurch at the switch. '
                'Bench only. Never fly with this.')
        else:
            self.get_logger().info(
                'engage gate off: abort_enable is false, so this node has no PX4 state (correct '
                'for observe-only, where goals go to the shadow topic).')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs -----------------------------------------------------------------------------

    def _on_follow_goal(self, msg: PoseStamped) -> None:
        self._check_frame(msg.header.frame_id, 'follow')
        self._follow.note(msg, self._now(), self._stale_timeout)

    def _on_nav_goal(self, msg: PoseStamped) -> None:
        self._check_frame(msg.header.frame_id, 'nav')
        # NAV is latched, so the staleness argument here only affects stream bookkeeping we do
        # not use for NAV -- the newest message simply wins.
        self._nav.note(msg, self._now(), self._stale_timeout)

    def _on_targets(self, msg: TargetArray) -> None:
        # The veto's input, deliberately independent of the goal sources: this is the whole
        # reason the node subscribes /dtp/targets rather than reading follow_planner's output.
        self._targets = list(msg.targets)
        self._targets_arrival = self._now()
        if self._brake_enable:
            for t in self._targets:
                self._person_vel.add(self._targets_arrival, t.track_id, t.velocity.x,
                                     t.velocity.y)

    def _on_terrain(self, msg: PointStamped) -> None:
        self._terrain_z = msg.point.z
        self._terrain_arrival = self._now()

    def _update_terrain(self, now):
        """This tick's terrain offset (terrain.py). 0.0, and nothing else runs, when off."""
        if not self._terrain_enable:
            return 0.0
        engaged = self._px4_engaged(now)
        dz = self._terrain.update(now, self._terrain_z, self._terrain_arrival, engaged)
        status = self._terrain.status
        if status in ('none', 'stale', 'unknown') and (now - self._terrain_logged) >= 5.0:
            self._terrain_logged = now
            why = {'none': '/dtp/terrain never received (is terrain_estimator running?)',
                   'stale': '/dtp/terrain has stopped (terrain_estimator died?)',
                   'unknown': 'the ground under the aircraft and the goal is not mapped yet '
                              '(expected on the ground before a lock: the LiDAR sees no ground '
                              'within ~1.2 m of itself)'}[status]
            text = 'terrain-relative height: %s -- HOLDING the offset at %+.2f m' % (why, dz)
            # ERROR only while PX4 flies our goals; on the ground this is bring-up, not a fault.
            # TWO call sites on purpose: rclpy caches a logging call site's severity and RAISES
            # ("Logger severity cannot be changed between calls") if one line logs at both --
            # the first version did, and terrain_check.py T7 watched it kill this node.
            if engaged:
                self.get_logger().error(text)
            else:
                self.get_logger().warn(text)
        return dz

    def _terrain_clamp(self, z):
        """The absolute clamp in terrain mode (ROG-Map's virtual ground and ceiling)."""
        if not self._terrain_enable:
            return z
        return min(max(z, self._terrain_abs_z_min), self._z_max)

    def _on_sensors_ok(self, msg: Bool) -> None:
        self._sensors_ok = bool(msg.data)
        self._sensors_arrival = self._now()

    def _on_vehicle_control_mode(self, msg) -> None:
        was = (self._px4_armed, self._px4_offboard)
        self._px4_armed = bool(msg.flag_armed)
        self._px4_offboard = bool(msg.flag_control_offboard_enabled)
        self._px4_mode_arrival = self._now()
        if (self._px4_armed, self._px4_offboard) != was:
            self.get_logger().info('PX4: armed=%s offboard=%s'
                                   % (self._px4_armed, self._px4_offboard))
        if was[0] and not self._px4_armed:
            dropped = self._descent_guard.reset()
            if dropped is not None:
                self.get_logger().info('descent guard: floor %.2f m dropped on disarm' % dropped)

    def _on_vehicle_command_ack(self, msg) -> None:
        if not self._abort.fired or int(msg.command) != int(VehicleCommand.VEHICLE_CMD_NAV_LAND):
            return
        verdict = self._abort.note_ack(int(msg.result))
        if verdict == 'accepted':
            self.get_logger().error(
                'ABORT: PX4 ACCEPTED NAV_LAND (result %d) after %d send(s) -- landing in place. '
                'This node stays in ABORT until relaunched.' % (msg.result, self._abort.sends))
        elif verdict == 'retry':
            self.get_logger().warn(
                'ABORT: PX4 answered NAV_LAND TEMPORARILY_REJECTED (send %d/%d) -- resending'
                % (self._abort.sends, ABORT_MAX_SENDS))
        else:
            self.get_logger().error(
                'ABORT: PX4 REFUSED NAV_LAND (result %d, result_param1 %d) -- not resending. The '
                'aircraft is NOT landing on our command: PILOT, TAKE IT.'
                % (msg.result, msg.result_param1))

    def _update_engage_gate(self, now):
        """THE ENGAGE GATE, once per tick before any goal is shaped. 'opened', 'closed' or None."""
        if not self._engage_gate_enable:
            return None
        edge = self._engage_gate.update(now, self._px4_mode_arrival, self._px4_armed,
                                        self._px4_offboard)
        if edge == 'opened':
            # A HOLD latched while the gate was shut was never flown. Re-anchor it on the
            # aircraft as it is now, or the first goal SUPER gets would be a stale point.
            self._hold_point = None
            self.get_logger().info(
                'engage gate OPEN: PX4 armed + Offboard, goals to SUPER resume; its first plan '
                'starts from the aircraft')
        elif edge == 'closed':
            self.get_logger().info(
                'engage gate SHUT: PX4 is no longer flying our setpoints (armed=%s offboard=%s); '
                'goals to SUPER held until the next Offboard'
                % (self._px4_armed, self._px4_offboard))
        return edge

    def _service_park(self, now, gate_edge):
        """PARK SUPER after the gate shuts (engage_gate.py). Once per tick, after the gate."""
        if not self._park_enable:
            return
        if gate_edge == 'opened':
            self._park.cancel()
        elif gate_edge == 'closed':
            self._park.start(now)
            if self._park_pub.get_subscription_count() == 0:
                self.get_logger().error(
                    'park: nothing subscribes %s, so SUPER is NOT parked and will keep '
                    're-planning to its last goal. Is SUPER patch 0009 built in? Stop the stack '
                    'after landing.' % self._park_topic)
            else:
                self.get_logger().info(
                    'park: SUPER told to drop its goal and idle until the next Offboard')
        if self._park.due(now):
            self._park_pub.publish(Empty())

    def _px4_engaged(self, now):
        """armed + Offboard per a FRESH vehicle_control_mode. Unknown reads as not engaged."""
        if self._px4_mode_arrival is None or (now - self._px4_mode_arrival) > PX4_MODE_STALE_S:
            return False
        return self._px4_armed and self._px4_offboard

    def _service_abort(self, now, vetoes):
        """Escalate persisting sensor-class vetoes to a Land; keep an abort terminal.

        Called once per tick, after the health vetoes are known and before the state is chosen,
        so the ABORT veto it appends forces HOLD arbitration on the same tick.
        """
        if not self._abort_enable:
            return
        if self._abort.fired:
            vetoes.append((VETO_ABORT, 'ABORTED: %s' % self._abort.reason))
            if self._cmd_pub is not None and self._abort.send_due(now):
                self._send_nav_land(now)
            return
        self._abort_require_offboard = bool(self.get_parameter('abort_require_offboard').value)
        if not self._abort_require_offboard:
            self.get_logger().error(
                'abort_require_offboard is FALSE -- bench test only, never fly with this',
                throttle_duration_sec=5.0)
        authority = self._ever_healthy and (self._px4_engaged(now)
                                            or not self._abort_require_offboard)
        if (self._abort_require_offboard and self._px4_mode_arrival is None
                and not self._px4_mode_warned and self._ever_healthy):
            self._px4_mode_warned = True
            self.get_logger().warn(
                'no /fmu/out/vehicle_control_mode yet: armed/offboard unknown, so the ABORT '
                'cannot fire. Is the uXRCE-DDS agent up? (harmless on a bench without the FC)')
        note = ('armed+offboard' if self._abort_require_offboard
                else 'BENCH FLAG abort_require_offboard=false, aircraft NOT checked')
        reason = self._abort.observe(now, [code for code, _ in vetoes], authority, note)
        if reason is None:
            return
        self.get_logger().error(
            '*** ABORT: LANDING IN PLACE *** %s. Sending NAV_LAND to PX4; the pilot can override '
            'with the mode switch at any time.' % reason)
        self._abort_pub.publish(String(data=reason))
        vetoes.append((VETO_ABORT, 'ABORTED: %s' % reason))
        if self._cmd_pub is not None and self._abort.send_due(now):
            self._send_nav_land(now)

    def _send_nav_land(self, now):
        msg = VehicleCommand()
        msg.timestamp = int(now * 1e6)
        msg.command = int(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        # |Unused|Unused|Unused|Desired yaw|Lat|Lon|Alt| -- NaN yaw keeps the current heading;
        # zero lat/lon/alt with no global position means "here".
        msg.param1 = msg.param2 = msg.param3 = 0.0
        msg.param4 = float('nan')
        msg.param5 = msg.param6 = 0.0
        msg.param7 = 0.0
        # Same addressing as offboard_engage.py, the one convention verified against this FC.
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = min(self._abort.sends, 255)
        msg.from_external = True
        self._cmd_pub.publish(msg)
        self._abort.note_sent(now)
        self.get_logger().error('ABORT: NAV_LAND sent (%d/%d)'
                                % (self._abort.sends, ABORT_MAX_SENDS))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if not _finite(p.x, p.y, p.z):
            # Leave the previous pose in place and let it age out into an ODOM veto. Adopting a
            # NaN position would poison every distance test downstream instead of vetoing.
            self.get_logger().error('non-finite odom position; ignoring (will veto on staleness)',
                                    throttle_duration_sec=2.0)
            return
        self._odom_p = (p.x, p.y, p.z)
        self._odom_arrival = self._now()
        if self._brake_enable:
            # The pose's own stamp, not its arrival: FAST-LIO's scans are 0.1 s apart, arrivals
            # jitter. A zero stamp (a bare test publisher) falls back to the arrival.
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._ac_vel_est.add(st if st > 0.0 else self._odom_arrival, p.x, p.y)

    def _check_frame(self, frame_id, source):
        if frame_id and frame_id != self._world_frame:
            # Worth a warning rather than a rejection: SUPER ignores frame_id completely, so a
            # mismatch produces a silently wrong goal, never an error. We cannot transform it
            # (we would need to know what the sender meant), but we can refuse to be quiet.
            self.get_logger().warn(
                "%s goal frame_id '%s' != world_frame '%s'; SUPER ignores frame_id entirely, so "
                'a mismatch here is silently wrong, not rejected' % (source, frame_id,
                                                                     self._world_frame),
                throttle_duration_sec=5.0)

    # --- veto -------------------------------------------------------------------------------

    def _input_health_vetoes(self, now):
        """Vetoes that depend only on input health, not on the goal. Fail closed on absence."""
        out = []

        if self._sensors_arrival is None:
            out.append((VETO_SENSORS, '/dtp/sensors_ok never received (sensor_watchdog down?)'))
        elif (now - self._sensors_arrival) > self._sensors_timeout:
            out.append((VETO_SENSORS, '/dtp/sensors_ok stale %.2fs > %.2fs'
                        % (now - self._sensors_arrival, self._sensors_timeout)))
        elif not self._sensors_ok:
            out.append((VETO_SENSORS, 'sensor_watchdog reports NOT ok (see /dtp/sensor_health)'))

        if self._targets_arrival is None:
            out.append((VETO_TARGETS,
                        '/dtp/targets never received -- the proximity veto has no input, and '
                        'perception_node must publish an EMPTY TargetArray when nobody is seen'))
        elif (now - self._targets_arrival) > self._targets_timeout:
            out.append((VETO_TARGETS, '/dtp/targets stale %.2fs > %.2fs'
                        % (now - self._targets_arrival, self._targets_timeout)))

        if self._odom_arrival is None:
            out.append((VETO_ODOM, '/lidar_slam/odom never received'))
        elif (now - self._odom_arrival) > self._odom_timeout:
            out.append((VETO_ODOM, '/lidar_slam/odom stale %.2fs > %.2fs'
                        % (now - self._odom_arrival, self._odom_timeout)))

        return out

    def _person_points(self, target):
        """Current position plus the forecast points inside the prediction horizon.

        predicted_path is already latency-compensated by the publisher (Target.msg), so it is
        used as published and never re-compensated here.
        """
        pts = []
        p = target.position
        if _finite(p.x, p.y, p.z):
            pts.append((p.x, p.y, p.z, 'now'))
        else:
            # A person we cannot locate is not a person we can clear. Reported as a violation by
            # the caller rather than skipped, because skipping is the fail-open branch.
            return None

        path = list(target.predicted_path)
        dts = list(target.predicted_path_dt)
        if len(dts) != len(path):
            # Target.msg promises equal lengths; if a publisher breaks that we cannot tell WHEN
            # any forecast point applies, so the forecast is dropped and only "now" is used.
            self.get_logger().warn(
                'target %d: predicted_path/%d and predicted_path_dt/%d differ in length; '
                'ignoring the forecast for this target' % (target.track_id, len(path), len(dts)),
                throttle_duration_sec=5.0)
            return pts

        for pt, dt in zip(path, dts):
            if not _finite(dt) or dt > self._pred_horizon or dt < 0.0:
                continue
            if _finite(pt.x, pt.y, pt.z):
                pts.append((pt.x, pt.y, pt.z, '+%.2fs' % dt))
        return pts

    def _person_forecast(self, target):
        """[(dt, x, y, z)] now plus the whole forecast, dt-sorted, for the braking envelope; None
        for a person we cannot locate (a violation, as in _person_points)."""
        p = target.position
        if not _finite(p.x, p.y, p.z):
            return None
        pts = [(0.0, p.x, p.y, p.z)]
        path = list(target.predicted_path)
        dts = list(target.predicted_path_dt)
        if len(dts) == len(path):
            pts += sorted((dt, q.x, q.y, q.z) for q, dt in zip(path, dts)
                          if _finite(dt, q.x, q.y, q.z) and dt > 0.0)
        return pts

    def _update_speed(self, now):
        """THE BRAKING ENVELOPE's inputs, once per tick: the aircraft's velocity, the largest
        closing speed on anyone within SPEED_PERSON_RANGE_M, and the filtered extra radius."""
        if not self._brake_enable:
            return
        v = self._ac_vel_est.velocity()
        self._ac_vel = v if v is not None else (0.0, 0.0)
        fresh = (self._targets_arrival is not None
                 and (now - self._targets_arrival) <= self._targets_timeout)
        if self._odom_p is None or not fresh:
            # Nothing to decide from: hold the extra where it is (the TARGETS / ODOM vetoes own
            # this case), and let the filter release it on its own clock once inputs return.
            self._v_close = 0.0
            return
        ax, ay = self._odom_p[0], self._odom_p[1]
        vc_max = 0.0
        for t in self._targets:
            q = t.position
            if not _finite(q.x, q.y) or math.hypot(q.x - ax, q.y - ay) > SPEED_PERSON_RANGE_M:
                continue
            vc = be.closing_speed((ax, ay), self._ac_vel, (q.x, q.y),
                                  self._person_vel.get(t.track_id),
                                  coasting=t.missed_frames > 0)
            vc_max = max(vc_max, vc)
        self._v_close = vc_max
        want = be.extra(vc_max, math.hypot(*self._ac_vel), self._speed_p)
        self._reach_extra = self._reach_filter.update(now, want)

    def _brake_violation(self, ax, ay, az, radius):
        """The worst person inside `radius` of the braking envelope at the same instant, as a
        reason string, or None. Same cylinder test as the path check, at the aircraft's height."""
        vx, vy = self._ac_vel
        path = be.braking_positions(ax, ay, vx, vy, self._speed_p)
        persons = []
        for target in self._targets:
            pts = self._person_forecast(target)
            if pts is None:
                return ('track %d has a non-finite position -- cannot clear a person we cannot '
                        'locate' % target.track_id)
            pz = pts[0][3]
            if az < pz - self._veto_half_height or az > pz + self._veto_half_height:
                continue
            persons.append((target.track_id, pts))
        worst = be.envelope_violation(path, persons, radius)
        if worst is None:
            return None
        s = math.hypot(vx, vy)
        return ('track %d at %.2f m from the braking path at +%.2fs (%.2f m/s: stop %.2f m + '
                'overshoot %.2f m; floor %.2f m)'
                % (worst[1], worst[0], worst[2], s, be.stop_distance(s, self._speed_p),
                   be.track_margin(s, self._speed_p), self._veto_radius))

    def _proximity_violation(self, ax, ay, az, bx, by, bz):
        """Worst cylinder incursion of any person against the path (a,b), or None.

        Horizontal distance is exact against the segment; the vertical test uses the segment's
        FULL z-range, which over-vetoes slightly when the near end of the path sits at a very
        different altitude from the far end. That is the conservative direction, and at the D6
        matched-height trail the two ends are at nearly the same altitude anyway.
        """
        radius = self._veto_radius + (self._release_margin if self._prox_active else 0.0)
        if self._brake_enable:
            # The aircraft runs past its reference by the tracking overshoot, along the approach.
            bx, by = be.extend_segment(ax, ay, bx, by,
                                       be.track_margin(math.hypot(*self._ac_vel), self._speed_p))
        seg_z_lo = min(az, bz)
        seg_z_hi = max(az, bz)
        worst = None

        for target in self._targets:
            pts = self._person_points(target)
            if pts is None:
                return ('track %d has a non-finite position -- cannot clear a person we cannot '
                        'locate' % target.track_id)
            for (px, py, pz, when) in pts:
                if seg_z_hi < (pz - self._veto_half_height):
                    continue
                if seg_z_lo > (pz + self._veto_half_height):
                    continue
                h = _horiz_dist_point_to_segment(px, py, ax, ay, bx, by)
                if h < radius and (worst is None or h < worst[0]):
                    worst = (h, target.track_id, when)

        if worst is None:
            if self._brake_enable:
                return self._brake_violation(ax, ay, az, radius)
            return None
        return ('track %d at %.2f m horizontally of the commanded path (floor %.2f m, %s)'
                % (worst[1], worst[0], self._veto_radius, worst[2]))

    def _proximity_veto(self, now, goal):
        """Latched proximity veto with release margin + dwell. Returns a reason, or None.

        Evaluated against the goal we WOULD publish, never against the hold point already being
        commanded: releasing on the basis of a path the drone is not about to fly would let the
        veto clear and then re-fire the instant the real goal returned -- a chattering veto.
        """
        if self._odom_p is None:
            return None                      # covered by VETO_ODOM; nothing to compute against
        ax, ay, az = self._odom_p
        if goal is None:
            bx, by, bz = ax, ay, az          # no candidate: still guard the point we sit at
        else:
            bx, by, bz = goal[0], goal[1], goal[2]

        reason = self._proximity_violation(ax, ay, az, bx, by, bz)
        if reason is not None:
            self._prox_active = True
            self._prox_clear_since = None
            return reason

        if not self._prox_active:
            return None

        if self._prox_clear_since is None:
            self._prox_clear_since = now
        held = now - self._prox_clear_since
        if held >= self._release_dwell:
            self._prox_active = False
            self._prox_clear_since = None
            return None
        return 'clear, releasing in %.2fs' % (self._release_dwell - held)

    # --- arbitration ------------------------------------------------------------------------

    def _select_source(self, now):
        """Highest-priority source with a usable goal: FOLLOW, then NAV, else nothing."""
        if self._follow.fresh(now, self._stale_timeout):
            frames_ok = self._follow.stream_frames >= self._engage_frames
            dwell_ok = (self._follow.stream_start is not None
                        and (now - self._follow.stream_start) >= self._engage_dwell)
            if frames_ok and dwell_ok:
                return STATE_FOLLOW, self._follow.msg
        else:
            # Instant de-escalation: re-engaging must pay the full hysteresis again.
            self._follow.break_stream()

        if self._nav.msg is not None:
            return STATE_NAV, self._nav.msg
        return None, None

    def _shape_goal(self, pose_msg, state):
        """Turn a source PoseStamped into (x, y, z, qx, qy, qz, qw), or None if unusable."""
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        if not _finite(p.x, p.y, p.z):
            return None

        x, y, z = p.x, p.y, p.z

        if state == STATE_NAV and abs(z) < NAV_ZERO_Z_EPS_M:
            self.get_logger().info(
                'NAV goal has z=0 (RViz "2D Nav Goal" always does); substituting %.2f m. '
                'setFollowGoal does not apply click_height, so passing it through would command '
                'the ground.' % self._nav_default_z, throttle_duration_sec=10.0)
            z = self._nav_default_z

        if self._terrain_enable and state == STATE_FOLLOW:
            # Height above the local ground (module docstring, TERRAIN-RELATIVE HEIGHT): the fixed
            # follow height replaces follow_planner's z, in the shifted frame, then shifted back.
            z = self._terrain_clamp(
                min(max(self._terrain_follow_z, self._z_min), self._z_max) + self._dz)
        else:
            clamped = min(max(z, self._z_min), self._z_max)
            if abs(clamped - z) > 1e-6:
                self.get_logger().warn(
                    '%s goal z %.2f clamped to %.2f -- outside [%.2f, %.2f]; ROG-Map virtual '
                    'ground / ceiling would have made SUPER drop it'
                    % (state, z, clamped, self._z_min, self._z_max),
                    throttle_duration_sec=5.0)
                z = clamped

        if not _finite(q.x, q.y, q.z, q.w) or math.sqrt(
                q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) < QUAT_MIN_NORM:
            self.get_logger().warn(
                '%s goal has an unusable orientation; substituting identity (SUPER would have '
                'derived an arbitrary yaw from it)' % state, throttle_duration_sec=5.0)
            return (x, y, z, 0.0, 0.0, 0.0, 1.0)
        return (x, y, z, q.x, q.y, q.z, q.w)

    # --- geofence ---------------------------------------------------------------------------

    def _fence_xy(self, x, y, what, level='warn'):
        """Clamp a horizontal position into the inset box, logging when the clamp bites.

        Returns (x, y) unchanged when the fence is disabled or the point is already inside.
        `what` names the goal for the log; `level` is 'warn' for the expected cases (a source
        asked for something outside) and 'error' for the publish-point re-check, where a bite
        means a code path skipped this function.
        """
        if not self._fence_enable:
            return x, y
        cx, cy, moved = self._fence.clamp(x, y)
        if moved == 0.0:
            return x, y
        text = ('geofence: %s goal (%.2f, %.2f) is %.2f m outside the box (or its %.2f m margin); '
                'clamped to (%.2f, %.2f)'
                % (what, x, y, self._fence.outside_by(x, y), self._fence.margin, cx, cy))
        if level == 'error':
            self.get_logger().error(text + ' AT THE PUBLISHER -- a code path skipped the clamp; '
                                    'the goal is still fenced, but this is a bug')
        else:
            self.get_logger().warn(text, throttle_duration_sec=2.0)
        return cx, cy

    def _fence_veto(self):
        """Refresh the aircraft's own breach distance; return a (code, text) veto or None.

        The aircraft OUTSIDE the box at all is reported (and every goal is already clamped back
        inside, so SUPER is commanding a return); past the hard boundary it is a FENCE veto,
        which abort.py escalates. With no odom there is nothing to check, and that case is
        already the ODOM veto.
        """
        self._fence_outside_m = 0.0
        if not self._fence_enable or self._odom_p is None:
            return None
        x, y, _ = self._odom_p
        out = self._fence.outside_by(x, y)
        self._fence_outside_m = out
        if out <= 0.0:
            return None
        if self._fence.breached(x, y):
            self.get_logger().error(
                'geofence BREACH: aircraft at (%.2f, %.2f) is %.2f m outside the box, past the '
                '%.2f m hard boundary -- FENCE veto (lands if it persists with authority)'
                % (x, y, out, self._fence.breach_land_m), throttle_duration_sec=1.0)
            return (VETO_FENCE,
                    'aircraft %.2f m outside the geofence (hard boundary %.2f m)'
                    % (out, self._fence.breach_land_m))
        self.get_logger().warn(
            'geofence: aircraft at (%.2f, %.2f) is %.2f m outside the box (hard boundary at '
            '%.2f m); goals are clamped inside, so it is being commanded back'
            % (x, y, out, self._fence.breach_land_m), throttle_duration_sec=2.0)
        return None

    def _apply_fence(self, goal, state):
        """The goal-level half of the fence: the source's goal clamped into the inset box."""
        if goal is None:
            return None
        x, y = self._fence_xy(goal[0], goal[1], state)
        if (x, y) == (goal[0], goal[1]):
            return goal
        return (x, y) + goal[2:]

    def _apply_descent_guard(self, goal, now):
        """Never command a goal below the drone while a person is close (R-F9, goal-level half).

        The decision and its latch live in command_mux/descent_guard.py; this feeds it and logs
        what it did. Logged per EVENT, not per tick: while latched, raising a goal to the floor is
        the normal state of a follow and would otherwise log continuously.
        """
        if goal is None:
            return goal
        people = [(t.track_id, t.position.x, t.position.y) for t in self._targets
                  if _finite(t.position.x, t.position.y, t.position.z)]
        # In the frame shifted down by the terrain offset, so the floor is a height above the
        # ground (TERRAIN-RELATIVE HEIGHT). _dz is 0.0 with the switch off.
        dz = self._dz
        drone = None if self._odom_p is None else (
            self._odom_p[0], self._odom_p[1], self._odom_p[2] - dz)
        if self._brake_enable:
            z, event, cause = self._descent_guard.apply(
                now, goal[0], goal[1], goal[2] - dz, drone, people,
                extra_radius_m=self._reach_extra)
        else:
            z, event, cause = self._descent_guard.apply(
                now, goal[0], goal[1], goal[2] - dz, drone, people)
        z = self._terrain_clamp(z + dz)
        if event == 'latched':
            self.get_logger().warn(
                'descent guard: floor latched at %.2f m (goal z %.2f), person (track %d) within '
                '%.2f m%s' % (z, goal[2], cause[0],
                              self._descent_guard.radius_m + self._reach_extra,
                              ' (terrain offset %+.2f)' % dz if self._terrain_enable else ''))
        elif event == 'raised':
            self.get_logger().info('descent guard: floor raised to %.2f m (drone climbed)'
                                   % (self._descent_guard.floor_z + dz))
        elif event == 'released':
            self.get_logger().info(
                'descent guard: floor released, nobody within %.2f m for %.1f s'
                % (self._descent_guard.radius_m + self._descent_guard.release_margin_m
                   + self._reach_extra,
                   self._descent_guard.release_dwell_s))
        if z == goal[2]:
            return goal
        return (goal[0], goal[1], z) + goal[3:]

    def _hold_goal(self, vetoes):
        """The latched HOLD goal, or None if we have no position to anchor it to.

        LATCHED, not recomputed each tick: a hold point recomputed from live odom while the drone
        is still decelerating runs away from the drone at exactly the speed the drone is closing
        on it, so the drone chases its own stop point and never stops. Latch once on entry, and
        release the latch only when HOLD is left.
        """
        if self._hold_point is not None:
            # Re-anchor if the drone is no longer anywhere near where the hold was LATCHED. See
            # HOLD_REANCHOR_M: a permanent veto would otherwise pin the goal to the boot position
            # for the whole flight, and re-assert it at GOAL_REPUBLISH_HZ. Measured against the
            # anchor (the odom at latch time), not the hold goal: the geofence can clamp the hold
            # goal several metres from an aircraft outside the box, and comparing against the
            # goal would then re-anchor every tick and log at 50 Hz while it flew back in.
            if (self._odom_p is not None
                    and math.dist(self._odom_p, self._hold_anchor) > HOLD_REANCHOR_M):
                self.get_logger().warn(
                    'hold point re-anchored: drone is %.1f m from where the hold was latched '
                    '(>%.1f m), so the latch is stale rather than protective'
                    % (math.dist(self._odom_p, self._hold_anchor), HOLD_REANCHOR_M))
                self._hold_point = None
            else:
                return self._hold_point
        if self._odom_p is None:
            self.get_logger().error(
                'HOLD required (%s) but no odom -- cannot anchor a hold goal, so the PREVIOUS '
                'goal stays active inside SUPER. SUPER is also refusing to step its own FSM '
                'without odom (fsm.cpp:100), so recovery is upstream, not here.'
                % ','.join(c for c, _ in vetoes), throttle_duration_sec=1.0)
            return None

        x, y, z = self._odom_p
        self._hold_anchor = (x, y, z)
        # Every hold goal goes through the fence too: a hold anchored on an aircraft that is
        # outside the box must pull it back in, not park it out there.
        fx, fy = self._fence_xy(x, y, 'HOLD')
        # The z bounds in the terrain-shifted frame (_dz is 0.0 with the switch off).
        target_z = self._terrain_clamp(
            min(max(z + self._hold_climb, self._z_min + self._dz), self._z_max + self._dz))
        if ((target_z - z) >= HOLD_MIN_DISPLACEMENT_M
                or math.hypot(fx - x, fy - y) >= HOLD_MIN_DISPLACEMENT_M):
            self._hold_point = (fx, fy, target_z, 0.0, 0.0, 0.0, 1.0)
            return self._hold_point

        # No climb available (already at the ceiling clamp). Back off horizontally from the
        # nearest person instead -- plan 04 SS6's "back off along the approach vector". A hold
        # goal that is not displaced at all would leave the drone flying its current trajectory.
        away = self._retreat_direction(x, y)
        if away is None:
            self._hold_point = (x, y, target_z, 0.0, 0.0, 0.0, 1.0)
            self.get_logger().warn(
                'HOLD at the ceiling clamp with no person to back away from: the hold goal is '
                'within closeToGoal(0.1), so SUPER will finish its current trajectory before '
                'stopping rather than braking now.', throttle_duration_sec=2.0)
            return self._hold_point
        rx, ry = self._fence_xy(x + away[0] * self._hold_climb, y + away[1] * self._hold_climb,
                                'HOLD-retreat')
        self._hold_point = (rx, ry, target_z, 0.0, 0.0, 0.0, 1.0)
        return self._hold_point

    def _retreat_direction(self, x, y):
        """Unit vector away from the nearest located person, or None if there is none."""
        best = None
        for target in self._targets:
            p = target.position
            if not _finite(p.x, p.y, p.z):
                continue
            d = math.hypot(x - p.x, y - p.y)
            if d < 1e-3:
                continue                     # directly overhead: no meaningful lateral direction
            if best is None or d < best[0]:
                best = (d, (x - p.x) / d, (y - p.y) / d)
        return None if best is None else (best[1], best[2])

    # --- the single decision + publish path -------------------------------------------------

    def _tick(self):
        now = self._now()
        self._dz = self._update_terrain(now)
        self._update_speed(now)
        gate_edge = self._update_engage_gate(now)
        self._service_park(now, gate_edge)
        vetoes = self._input_health_vetoes(now)

        state, pose_msg = self._select_source(now)
        goal = None
        goal_requested = None      # the source's goal BEFORE this node rewrites it; see below
        if pose_msg is not None:
            goal = self._shape_goal(pose_msg, state)
            if goal is None:
                vetoes.append((VETO_GOAL_INVALID,
                               '%s published a non-finite position' % state))
            else:
                goal_requested = goal
                goal = self._apply_descent_guard(self._apply_fence(goal, state), now)

        fence_veto = self._fence_veto()
        if fence_veto is not None:
            vetoes.append(fence_veto)
        if not self._fence_enable and (now - self._fence_disabled_logged) >= 5.0:
            self._fence_disabled_logged = now
            self.get_logger().error(
                'GEOFENCE DISABLED (fence_enable:=false). Bench only. Never fly with this.')

        # Skipped only when targets are stale, and that case already has its own veto -- deciding
        # from a stale people list would be exactly the "sit on the last good value" failure.
        targets_fresh = (self._targets_arrival is not None
                         and (now - self._targets_arrival) <= self._targets_timeout)
        if targets_fresh:
            # Check BOTH the goal the source asked for and the one after our own rewrites, and
            # veto if EITHER is unsafe.
            #
            # WHY: _apply_descent_guard raises the goal's z to its altitude floor, which
            # flattens the drone->goal segment. _proximity_violation gates on that segment's
            # z-range, so a flattened segment can sit entirely above the person's cylinder and
            # every person gets skipped -- the descent guard silently switching OFF the proximity
            # veto, on exactly the trigger the two share (a person close by). No transformation
            # this node applies to a goal may be allowed to reduce what the veto sees.
            reason = self._proximity_veto(now, goal)
            if reason is None and goal_requested is not None and goal_requested != goal:
                reason = self._proximity_veto(now, goal_requested)
            if reason is not None:
                vetoes.append((VETO_PROXIMITY, reason))

        self._service_abort(now, vetoes)

        if vetoes or goal is None:
            forced = self._state != STATE_HOLD and self._state != STATE_ABORT
            state = STATE_ABORT if self._abort.fired else STATE_HOLD
            goal = self._hold_goal(vetoes)
        else:
            forced = self._state != state
            # Released only on the way OUT of HOLD, so the next HOLD re-anchors at wherever the
            # drone actually is rather than reusing a stale point from an earlier event.
            self._hold_point = None
            self._ever_healthy = True
        self._report_hold_to_guard(now, state)

        if forced and self._state is not None:
            self.get_logger().info('%s -> %s' % (self._state, state))
        self._state = state

        self._publish_goal(goal, now, forced or gate_edge == 'opened')
        self._publish_state(now, state, vetoes, goal)

    def _report_hold_to_guard(self, now, state):
        """Tell the descent guard which climbs are this node's own HOLD's (descent_guard.py).

        Only while PX4 is flying our goals (armed + Offboard, per a fresh report). A HOLD latched
        on the ground, or while the pilot flies in Position, commands nothing. Registering its
        altitude would make the guard see the drone at ground level for the rest of the flight,
        which is the guard switched off. When Offboard is engaged mid-HOLD, as the 2026-09-21
        18:41 flight was, the altitude at engagement is registered, since the climb starts there.
        """
        holding = state in (STATE_HOLD, STATE_ABORT)
        if not self._px4_engaged(now):
            self._descent_guard.clear_hold()
            self._hold_reported = False
        elif not holding:
            self._descent_guard.hold_ended()
            self._hold_reported = False
        elif not self._hold_reported and self._odom_p is not None:
            self._descent_guard.hold_started(self._odom_p[2] - self._dz)
            self._hold_reported = True

    def _publish_goal(self, goal, now, forced):
        if goal is None:
            return
        if not self._ever_healthy:
            # Nothing has been healthy yet, so the only goal we could publish is a boot HOLD --
            # and publishing ANYTHING here is what latches SUPER's started_ flag (fsm.cpp:236-238)
            # and hands it a goal to fly. Staying silent until one clean tick means a mux that
            # comes up into a fully-vetoed world never arms SUPER at all, which is the safe
            # failure. Once healthy, HOLD publishes normally -- silence is NOT a stop mid-flight,
            # because SUPER keeps flying to whatever goal it already holds.
            self.get_logger().warn(
                'withholding goal: no healthy tick yet, so SUPER is deliberately left unarmed',
                throttle_duration_sec=5.0)
            return
        if self._engage_gate_enable and not self._engage_gate.is_open:
            # THE ENGAGE GATE: PX4 is not flying our setpoints, so SUPER gets nothing to fly in
            # its own reference. _last_goal is left alone; the tick the gate opens forces a
            # publish.
            return
        moved = (self._last_goal is None
                 or math.dist(goal[:3], self._last_goal[:3]) > GOAL_DEADBAND_M)
        due = (now - self._last_goal_pub) >= (1.0 / GOAL_REPUBLISH_HZ)
        if not (forced or moved or due):
            return

        # The fence's last gate. Every path above already clamps, so this never bites; if it
        # does, the goal is still fenced and the ERROR names the bug.
        fx, fy = self._fence_xy(goal[0], goal[1], 'published', level='error')
        if (fx, fy) != (goal[0], goal[1]):
            goal = (fx, fy) + goal[2:]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal[0], goal[1], goal[2]
        msg.pose.orientation.x = goal[3]
        msg.pose.orientation.y = goal[4]
        msg.pose.orientation.z = goal[5]
        msg.pose.orientation.w = goal[6]
        self._goal_pub.publish(msg)
        self._last_goal = goal
        self._last_goal_pub = now

    def _publish_state(self, now, state, vetoes, goal):
        """'STATE|reason|detail' -- the first field is a stable token, the rest is for humans."""
        if state == STATE_ABORT:
            reason = 'LAND'
            others = '; '.join(text for _, text in vetoes if not text.startswith('ABORTED'))
            detail = '%s [%s] | %s' % (self._abort.reason, self._abort.status(), others)
        elif vetoes:
            reason = '+'.join(code for code, _ in vetoes)
            detail = '; '.join(text for _, text in vetoes)
        elif state == STATE_HOLD:
            reason = 'NO_SOURCE'
            detail = ('no usable goal source (follow %s, nav %s)'
                      % (self._follow.age_str(now), self._nav.age_str(now)))
        else:
            reason = 'OK'
            detail = 'follow %s, nav %s' % (self._follow.age_str(now), self._nav.age_str(now))

        if goal is None and state in (STATE_HOLD, STATE_ABORT):
            detail += ' [NO HOLD GOAL PUBLISHED -- no odom]'
        elif goal is not None:
            detail += ' | goal (%.2f, %.2f, %.2f)' % (goal[0], goal[1], goal[2])
        if self._engage_gate_enable and not self._engage_gate.is_open:
            detail += ' | to SUPER: held until Offboard'
        elif self._abort_enable and not self._engage_gate_enable:
            detail += ' | ENGAGE GATE OFF'
        if not self._fence_enable:
            detail += ' | FENCE OFF'
        elif self._fence_outside_m > 0.0:
            detail += ' | fence: aircraft %.2f m outside' % self._fence_outside_m
        if self._descent_guard.active:
            # The floor as an absolute height; in terrain mode it moves with the ground.
            detail += ' | alt floor %.2f' % (self._descent_guard.floor_z + self._dz)
        if self._terrain_enable:
            detail += ' | terrain %+.2f %s' % (self._dz, self._terrain.status)
        if self._brake_enable:
            s = math.hypot(*self._ac_vel)
            detail += (' | brake v %.2f closing %.2f reach %.2f extra %.2f'
                       % (s, self._v_close,
                          be.reach(self._v_close, s, self._speed_p), self._reach_extra))

        text = '%s|%s|%s' % (state, reason, detail)
        if text != self._last_state_text or (now - self._last_state_pub) >= STATE_HEARTBEAT_S:
            self._state_pub.publish(String(data=text))
            self._last_state_text = text
            self._last_state_pub = now


def main(args=None):
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how SIGTERM arrives -- the normal stop path for systemd
        # and `ros2 launch`. Without it, every managed shutdown exits with a stack trace.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
