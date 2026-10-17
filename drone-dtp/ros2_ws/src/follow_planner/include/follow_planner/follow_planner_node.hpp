#ifndef FOLLOW_PLANNER_NODE_HPP
#define FOLLOW_PLANNER_NODE_HPP

// Phase 2 M2.1 (plan/dtp-imp/07 M2.1). Was a Phase-0 M0.1 scaffold until this change --
// modeled on mission_planner/include/waypoint_mission/ros2_waypoint_planner.hpp, per
// plan/dtp-imp/01 SS2's node/topic contract table and plan/dtp-imp/03 SS8's package layout.
//
// Job: turn the locked target's position + predicted path (dtp_msgs/Target, from
// target_manager) into a STANDOFF VIEWPOINT and publish it as a moving goal. Explicitly NOT
// this node's job to publish directly to SUPER's /goal_pose -- it publishes to
// /dtp/follow_goal, which command_mux arbitrates against the NAV goal source
// before republishing a single arbitrated goal (plan/dtp-imp/01 SS2). Two upstream design
// corrections this node must respect once real logic lands (both already in plan/dtp-imp/14,
// see CLAUDE.md SS4/SS8):
//   - do NOT reuse SUPER's click-goal path/semantics -- setGoalPosiAndYaw silently
//     Z-overrides, snaps within 3m, or drops an occupied goal with no feedback. This is
//     exactly why /planning/goal_status (dtp_msgs/GoalStatus) exists: subscribe it and
//     hold/re-project on DROPPED rather than assuming a published goal was honored.
//   - rate-limit / deadband the published goal and keep it AHEAD of the drone (a lead point
//     along the predicted path) so SUPER's point-to-point FSM doesn't thrash into a
//     rest-to-rest replan every time distance-to-goal dips near zero (risk A13).
//
// ---------------------------------------------------------------------------------------
// THREE THINGS THAT CHANGED WHEN THE REAL LOGIC LANDED, AND WHY. Read before editing.
//
// 1. THE YAW-TARGET TOPIC IS /planning/yaw_target, NOT /dtp/target_point.
//    The scaffold published PointStamped on /dtp/target_point because plan/dtp-imp/01 SS2's
//    fsm_node row says so. fsm_node does not listen there. It subscribes cfg_.yaw_target_topic,
//    whose default is "/planning/yaw_target" (fsm/config.hpp:76) and which
//    super_planner/config/follow_test_ros2.yaml sets to exactly that. The failure was silent
//    and total: with nothing ever arriving, the yaw target never becomes valid and
//    generateExpTraj falls through to free_end=true -- velocity-aligned yaw
//    (super_planner.cpp:794-811). The drone flies a perfect standoff and never looks at the
//    person, which is precisely the R-F10 failure YAW_TO_TARGET exists to prevent, with no
//    warning anywhere. The SUPER side has three consistent references to the /planning name
//    (config default, YAML, and the M0.7 0.7.4 hardware test that passed with it) against one
//    on the DTP side, so the DTP side moved. plan/dtp-imp/01 SS2's fsm_node row still needs
//    the same correction.
//
// 2. THE WORK LIVES IN A TIMER, NOT IN FollowTargetCallback.
//    A callback-driven design cannot detect its own silence, and every loss condition this
//    node owns (stale target, dead odom, person behind the drone) is a NON-EVENT -- defined by
//    something that stopped arriving. It also could not honour goal_publish_rate_hz, because
//    /dtp/follow_target has no declared rate anywhere in the repo: it is whatever the YOLO
//    pipeline achieves that second. So the target callback only caches, and a fixed-rate timer
//    decides and publishes.
//
// 3. SILENCE IS THE LOSS SIGNAL -- BOTH DIRECTIONS.
//    Inbound: target_manager publishes /dtp/follow_target ONLY while a lock exists (nothing at
//    all while IDLE/ACQUIRING/ABSTAIN/LOST), so absence is the signal, not a hold.
//    Outbound: when this node cannot produce a trustworthy goal it STOPS PUBLISHING rather
//    than republishing a frozen one. That is what makes command_mux's source_stale_timeout_s
//    fire and fall through to NAV/hover -- the designed escalation path (plan/dtp-imp/04 SS6,
//    A15: "NEVER hover-in-place... Never a frozen setpoint"). Publishing a goal we know is
//    being dropped keeps FOLLOW looking healthy to the mux, which is the failure mode to avoid.
// ---------------------------------------------------------------------------------------

#include "rclcpp/rclcpp.hpp"
#include "dtp_msgs/msg/target.hpp"
#include "dtp_msgs/msg/goal_status.hpp"
#include "dtp_msgs/msg/loss_event.hpp"
#include "dtp_msgs/msg/visibility_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "Eigen/Core"

#include "follow_planner/nmpc_follow.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace follow_planner {

    // ---- Named constants. Each is a number the plan does NOT commit to and that we judged not
    // ---- worth a runtime knob; each carries the reason it is what it is.

    // Below this speed the target's velocity direction is estimator noise, not a heading. The
    // upstream CTRV predictor already noise-gates its yaw-rate term (plan/dtp-imp/14 SS3.3), so a
    // heading taken from below that gate would spin the standoff azimuth around a person who is
    // simply standing still -- exactly the goal jitter the deadband exists to suppress. Under it
    // we HOLD the last heading that was fast enough to trust.
    constexpr double kHeadingSpeedFloorMps = 0.25;

    // A horizontal line-of-sight shorter than this makes atan2 meaningless (person effectively
    // overhead). SUPER applies the same 1e-3 guard to its own yaw target and leaves yaw FREE
    // rather than snapping to an arbitrary angle (super_planner.cpp:800-810); mirroring the
    // number keeps both ends agreeing about when a bearing exists at all.
    constexpr double kLosHorizontalFloorM = 1e-3;

    // The yaw hold (lost_yaw_hold_s) publishes its heading as a point this far out
    // along it. SUPER aims from the trajectory's END at the point (super_planner.cpp:800-810), so
    // the bearing it flies drifts by atan(offset / distance): 100 m keeps a 3 m HOLD climb or a
    // 10 m NAV leg under 6 deg, which is what makes the point a heading rather than a place.
    constexpr double kLostYawHoldDistanceM = 100.0;

    // Nearer than this, the bearing to the last point the camera was aimed at is not a heading
    // worth holding (the aircraft is next to or over it). The hold then keeps the aircraft's own
    // heading.
    constexpr double kLostYawHoldMinRangeM = 1.0;

    // A coasting target keeps the perception stamp of the frame it was last SEEN in (target_manager
    // never rewrites it), and SUPER drops a yaw target older than 0.5 s (kYawTargetTimeoutSec,
    // super_planner.h:102). The coast yaw hold takes over at this age: one goal tick (67 ms) and
    // some transport short of SUPER's gate, so the yaw target never lapses.
    constexpr double kCoastYawHoldAgeS = 0.4;

    // Tolerance for "this ACCEPTED status is answering MY request". setFollowGoal reports SNAPPED
    // whenever ||snapped - requested|| > 1e-6 (fsm.cpp:233), and both sides are double precision
    // (SUPER's Vec3f is Vecf<3> over decimal_t = double, eigen_alias.hpp:41,48), so an ACCEPTED
    // goal_p is bit-for-bit our point and this is pure slack. It is unambiguous because
    // goal_deadband_m guarantees two CONSECUTIVE published goals differ by >= 0.15 m -- more than
    // two orders of magnitude above this tolerance.
    constexpr double kAcceptedMatchTolM = 1e-3;

    // Re-projection escalation after a large SNAP or a DROP. We cannot see which side of the
    // person is open -- ROG-Map belongs to SUPER, not to us -- so the only honest strategy is to
    // probe symmetrically and let GoalStatus answer within one tick. ~20 deg is small enough that
    // a successful probe is still recognisably a trail-behind viewpoint, large enough to clear a
    // 0.4 m inflation shell (inflation_resolution 0.2 m x inflation_step 2) at a 3 m radius.
    constexpr double kReprojectAzimuthStepRad = 0.35;
    constexpr double kReprojectRadiusStepM = 0.25;

    // Never swing the commanded viewpoint more than 90 deg off directly-behind. Past that it
    // stops being a trail and becomes a side/orbit viewpoint, which plan/dtp-imp/04 SS1
    // explicitly defers ("v1 validates behind-standoff only") -- an obstacle escalation must not
    // silently promote the drone into an unvalidated flight mode.
    constexpr double kMaxAzimuthOffsetRad = M_PI / 2.0;

    // On an accepted goal the re-projection bias decays instead of snapping back to zero. An
    // instant return to the nominal trail re-grazes the same inflated cell that caused the snap
    // and re-triggers it, giving a two-tick oscillation; decaying needs several consecutive
    // accepted goals to unwind, which is the commit hysteresis A1/A13 ask for.
    constexpr double kReprojectDecay = 0.8;

    // SUPER's closeToGoal(0.1) (fsm.cpp:133,176-181) is the parking trip-wire, but the danger is
    // NOT inside it -- inside 0.1 m the FSM sits cleanly in WAIT_GOAL and holds. The danger is
    // the annulus just OUTSIDE, where GENERATE_TRAJ calls PlanFromRest for a hop of a few
    // centimetres on every tick: that is the stop-and-go stutter of R-F7. So we refuse to hand
    // SUPER any goal that close to the drone and republish the previous one instead.
    //
    // 0.25 m is bracketed from three sides: above 2x closeToGoal's 0.1 m so we are never in the
    // stuttering annulus; above goal_deadband_m 0.15 m so the two guards do not both fire on the
    // same tiny motion; and BELOW the steady-state following error, or it would fire during
    // ordinary following and turn a smooth servo into 0.25 m steps.
    // *** THE THIRD BOUND IS AN ESTIMATE, NOT A MEASUREMENT. *** It assumes the drone trails its
    // moving goal by roughly v * tau ~ 1.4 m/s * 0.3 s ~ 0.4 m, and tau is not measured anywhere
    // in this repo. plan/dtp-imp/07 Step 2.3.1's walk-pause-walk gate is where that gets checked:
    // if the log shows the park guard engaging while the person is walking steadily, this number
    // is too large, not the servo.
    constexpr double kParkGuardM = 0.25;

    // The lead is never allowed to spend the standoff all the way down to min_standoff_m; this
    // much is held back. A hard floor the servo sits on every cycle is not a floor, it is a
    // setpoint, and the first estimator error goes straight through it. 0.25 m is 10 % of the
    // floor and comfortably exceeds the sub-decimetre detection jitter plan/dtp-imp/07 Step 2.1.1
    // cites, so the reserve is not itself consumed by noise.
    constexpr double kFloorReserveM = 0.25;

    // Period of the periodic state line. Loud enough to reconstruct a flight from the log, quiet
    // enough not to become the thing that slows the node down.
    constexpr int kStateLogPeriodMs = 2000;


    class FollowPlanner {
    private:
        rclcpp::Node::SharedPtr nh_;

        // Standoff band (decision D17, plan/dtp-imp/12): 3-4m nominal, 2.5m hard floor.
        // Parameterised, not hardcoded, so it's tunable per-mission without a rebuild.
        double follow_near_m_{3.0};
        double follow_far_m_{4.0};
        double min_standoff_m_{2.5};
        double goal_publish_rate_hz_{15.0};

        // How far the goal is carried forward along the person's predicted path. NO VALUE FOR
        // THIS EXISTS ANYWHERE IN THE REPO -- the lead point is named as a requirement in four
        // places (the block comment above, plan/dtp-imp/01 SS4 step 6, 04 SS1, 07 Step 2.1.1) and
        // plan/dtp-imp/14:89 says to scale it down as LIO covariance grows, but no file commits
        // to a number, so this default is a choice made here and the reasoning belongs with it.
        // 1.0 m: the only hard requirement is that ||robot - goal|| never reaches 0.1 m
        // (closeToGoal, fsm.cpp:133). The goal already sits min_standoff_m = 2.5 m behind the
        // person, so the lead's only job is to absorb the case where the person STOPS and the
        // drone keeps closing. 1.0 m is ~0.7 s of walking at 1.4 m/s -- comfortably more than one
        // replan period (1/15 s) and the 150-250 ms closed-loop latency (plan 14 C5) -- while
        // staying far inside planning_horizon (7.0 m, follow_test_ros2.yaml) so the goal stays
        // plannable. It is consumed as ARC LENGTH along the predicted path, so it shrinks to zero
        // by itself when the person slows: see LeadAnchor().
        // THIS IS A CEILING, NOT AN AMOUNT. The lead is budgeted against the margin between the
        // commanded standoff and min_standoff_m, because the two act on the same axis and the
        // floor wins -- so at the default 3.0 m standoff only 0.5 m of it is spendable. See the
        // lead-budget block in ComputeStandoffGoal for why that is the right trade.
        double lead_distance_m_{1.0};

        // Hold the published goal VALUE still until it has moved this far. 0.15 m sits above the
        // sub-decimetre detection jitter plan 07 Step 2.1.1 names, and below ROG-Map's 0.2 m
        // inflation_resolution, so a deadbanded goal can never hide a real cell-occupancy change.
        double goal_deadband_m_{0.15};

        // A SNAPPED goal displaced less than this is accepted silently. 0.5 m is well inside the
        // 3-4 m band, so adopting it cannot violate min_standoff_m. SUPER's own snap radius is a
        // hardcoded 3.0 m literal (fsm.cpp:229) -- larger than the entire band -- so an
        // unexamined snap really can teleport the goal.
        double snap_accept_m_{0.5};

        // Consecutive large snaps / outright drops before we give up on this viewpoint. 5 at
        // 15 Hz is a third of a second of probing: long enough to have tried both azimuth signs
        // at two magnitudes, short enough that command_mux hears about it promptly.
        int snap_persist_n_{5};
        int drop_persist_n_{5};

        // Fail-closed window for a missing GoalStatus. 0.2 s is three goal-publish periods.
        double goal_status_timeout_s_{0.2};

        // How long without /dtp/follow_target before we stop publishing a goal. 0.3 s is
        // deliberately LOOSER than command_mux's 0.2 s staleness on our output, so the two
        // watchdogs do not race: follow_planner going silent should be what trips the mux, not
        // the mux timing out on a goal follow_planner still believes in.
        double follow_target_timeout_s_{0.3};

        // /lidar_slam/odom is 10 Hz nominal, so 0.3 s is three missed messages. This deliberately
        // is not the system's first line of defence: fsm.cpp:100 refuses to step the FSM on odom
        // older than 0.1 s, tighter than anything else in the stack, so SUPER printing "No odom"
        // IS the first symptom by design. This gate only stops US computing a world-frame
        // standoff from a stale drone position.
        double odom_timeout_s_{0.3};

        // Goal Z is clamped into this interval. ROG-Map's virtual_ground_height = -1.0 and
        // virtual_ceil_height = 3.5 (follow_test_ros2.yaml) make anything outside (-1.0, 3.5)
        // virtual occupancy, which setFollowGoal returns as DROPPED -- so an unclamped Z is a
        // silent, permanent follow failure. [0.5, 3.0] keeps margin on both sides.
        double goal_z_min_m_{0.5};
        double goal_z_max_m_{3.0};

        // Goal Z = target Z + this. Decision D6 is a MATCHED-HEIGHT horizontal trail, not a
        // trail-from-above (plan/dtp-imp/04 SS1). The fused target Z is a cluster centroid, i.e.
        // roughly mid-torso (~1.0-1.2 m), so +0.5 m puts the drone at about head height instead
        // of chest height -- still "similar height" per D6, but with the person's head below the
        // rotor plane at the standoff distances we command.
        double goal_height_offset_m_{0.5};

        // How far the viewpoint may rotate off directly-behind, toward the side the drone is
        // already on. This is the A1 near-side bias (plan/dtp-imp/14 SS3.5, "bias the goal to the
        // near/open side"): SUPER plans collision-optimally to an endpoint with ZERO visibility
        // awareness, so a goal on the FAR side of the person invites a transit that swings the
        // camera off them mid-flight. 45 deg keeps the viewpoint recognisably behind.
        double near_side_bias_max_deg_{45.0};

        // Usable half-angle of the body-fixed camera. The Gemini 336L is 94 x 68 deg
        // (plan/dtp-imp/02 SS6), so the true half-angle is 47 deg; 35 leaves 12 deg of margin for
        // the attitude excursions that swing the frame, since D14 defers the gimbal and nothing
        // else stabilises it.
        double camera_fov_half_deg_{35.0};

        // How long the person may sit outside that cone before it counts as a loss. Not instant:
        // the drone yaws constantly while tracking and a momentary excursion is normal. Past
        // this, anything still arriving on /dtp/follow_target is target_manager coasting on CTRV
        // with no fresh visual evidence, and CTRV past ~1.5 s is not something to fly a standoff
        // off (plan/dtp-imp/14 SS3.3).
        double target_behind_timeout_s_{1.0};

        // Minimum clearance the straight line from the drone to the commanded goal must keep from
        // the person -- see TransitCrossesKeepOut. It CANNOT be min_standoff_m: the goal itself
        // sits at (or just above) that distance, so the guard would trip on its own endpoint on
        // every cycle. It is bracketed instead: strictly below min_standoff_m (2.5) so a legal
        // goal never trips it, and strictly above command_mux's proximity_veto_radius_m (1.5) so
        // follow_planner stops commanding a transit BEFORE the independent veto has to fire.
        // Layers nesting instead of fighting, which is plan/dtp-imp/01 SS5's rule. 2.0 m leaves
        // 0.5 m on each side.
        double transit_clearance_m_{2.0};

        // Once suspended by repeated drops, how far the person must move before we try again. A
        // timer-based retry just re-drops: the drops are geometric (person standing against a
        // trunk), so the only condition under which a retry can succeed is the geometry actually
        // changing. 1.0 m is roughly one pace.
        double suspend_reset_move_m_{1.0};

        // "world", not "odom". The plan's prose calls the world frame odom, but nothing publishes
        // that string: FAST-LIO's origin frame is camera_init, m1_super_lidar.launch.py anchors a
        // static identity world -> camera_init, and SUPER/ROG-Map visualise in "world"
        // (follow_test_ros2.yaml:161). ROG-Map ignores frame_id and TF entirely and treats the
        // odom pose as world-frame, so a wrong frame_id here is accepted in silence -- which is
        // exactly why the string has to be chosen deliberately rather than left to a default.
        std::string world_frame_id_{"world"};

        // M6.5 LOSS CONDITIONING (docs/m5-m6-plan.md SS5.0). ONE switch, default false, shared
        // with target_manager (follower.launch.py's loss_conditioning:= sets both), so the two
        // ablation arms are one binary. false = the carry-forward arm: everything in this file
        // behaves exactly as documented above and /planning/loss_event is ignored. true, while
        // the follow target is COASTING (missed_frames > 0) and the oracle's newest verdict for
        // this track is fresh:
        //   OCCLUDED     goal = the oracle's viewpoint (a standoff candidate with a free line of
        //                sight to the predicted re-emergence point, >= min_standoff from the
        //                hidden person -- "pre-position for the edge"), yaw target = that point.
        //   OUT_OF_VIEW  goal as today; yaw target = the CTRV point, so the camera turns to it.
        //   LOST/UNMAPPED  nothing here: target_manager ends the coast and the target goes stale.
        // THE YAW STAMP: PublishYawTarget deliberately republishes the PERCEPTION stamp so that
        // SUPER's 0.5 s yaw gate measures real staleness. Under conditioning, and ONLY while the
        // oracle has a fresh verdict for a coasting target, the yaw target is stamped `now`
        // instead -- pointing the camera at a predicted point is the whole point of the arm, and
        // the exposure is bounded by target_manager's coast cap (3.0 s) and loss_event_timeout_s.
        bool loss_conditioning_{false};
        std::string loss_event_topic_{"/planning/loss_event"};
        double loss_event_timeout_s_{1.0};

        // THE YAW HOLD (2026-09-26, user). Without it, whenever SUPER has no fresh yaw target it
        // falls back to its free-end yaw: the heading follows the trajectory's displacement. That
        // happened twice per loss: from 0.5 s into a coast (the coasting target's perception
        // stamp ages past SUPER's gate), and for good once the target went stale (this node fell
        // silent). On the 2026-09-26 20:12 flight the HOLD after the loss was a ~1.5 m
        // dip-and-climb, past patch 0006's 0.3 m threshold, and the nose turned 83 deg away from
        // where the walker had been within ~2 s, so auto re-acquire had nothing to look at.
        // Now the node holds ONE heading, latched once per episode when EITHER
        //   * a coasting target's perception stamp reaches kCoastYawHoldAgeS (coast_yaw_hold_), or
        //   * the target goes stale after having had one (the loss),
        // toward the LAST POINT THE CAMERA WAS AIMED AT (last_yaw_point_: the person, the CTRV
        // point, or, under M6.5 conditioning, the oracle's re-emergence point), or the aircraft's
        // own heading if that point is within kLostYawHoldMinRangeM. It is published as a point
        // kLostYawHoldDistanceM out along it, stamped now. A seen target ends it at once, and so
        // does a fresh oracle verdict (conditioning aims the camera itself); a hold latched in a
        // coast carries through the loss unchanged.
        //   * The coast half changes the M6.5 carry-forward arm: its coasting yaw targets used to
        //     keep the perception stamp and lapse at 0.5 s (target_manager.py, "THE STAMP IS
        //     NEVER REWRITTEN"). coast_yaw_hold false restores that arm and keeps the loss half.
        //   * Why bounded: this is a claim about where the person was, not where they are. 15 s
        //     from the latch covers the airborne re-lock (5.8 s) and the pilot's takeover (6.5 s)
        //     of 2026-09-26, and bounds the one side effect: an Offboard re-engage inside the
        //     window turns the nose back to the held heading. A NAV goal inside it keeps it too.
        // 0 turns the whole hold off: the behaviour flown up to 2026-09-26.
        // Carried onto the 2026-09-30 rollback on its own, without the speed-scaled standoff it
        // was committed with (user).
        double lost_yaw_hold_s_{15.0};
        bool coast_yaw_hold_{true};

        // M6.8 "SUPER stock" baseline: a goal directly behind the person's CURRENT position on a
        // constant-velocity heading -- no latency compensation, no lead, no near-side bias. Bench
        // and replay only; it is a comparison arm, not a flight mode.
        bool stock_baseline_{false};
        // Which implementation answers ComputeStandoffGoal. "geometric" is the closed-form
        // standoff that has flown. "nmpc" (nmpc_follow.hpp, docs/fact-nmpc-follow-goal.md)
        // optimises the goal over the forecast. Anything else refuses to start, so a typo fails
        // loud instead of silently flying geometric under another name. Read at construction:
        // changing the arm is a relaunch.
        std::string goal_mode_{"geometric"};

        // goal_mode nmpc. The horizon is target_manager's min_coast_horizon_s: the shortest
        // forecast upstream guarantees, and inside the 1.5 s this node trusts CTRV for. Its
        // stages are one goal tick apart, which makes the warm start an exact index shift.
        double nmpc_horizon_s_{1.2};
        int nmpc_stages_{18};           // derived: horizon x goal_publish_rate_hz
        double nmpc_dt_s_{1.0 / 15.0};  // derived: one goal tick
        int nmpc_thin_stages_{6};       // horizon when the forecast has < 2 usable points
        bool nmpc_coast_shrink_{true};
        int nmpc_coast_full_n_{8};      // coast depth at which the horizon reaches its floor
        // Stage 1 is the standoff contract. A later stage is a goal AHEAD of the standoff point
        // (the "carrot" that could make SUPER's goal_vel_en bite): it breaks that contract and
        // every scorer built on it, so it is a bench experiment behind its own switch.
        int nmpc_publish_stage_{1};
        bool nmpc_carrot_experiment_{false};
        int nmpc_max_iters_{150};
        double nmpc_time_budget_ms_{8.0};   // the solve is cancelled here and its iterate kept
        double nmpc_time_panic_ms_{12.0};   // logged at ERROR
        int nmpc_fallback_persist_n_{30};   // ~2 s of refused solves latches the geometric goal
        // One cancelled solve is harmless: its iterate descends from a warm start, so it is at
        // least as good as the plan already being flown. A RUN of them is a starved machine, and
        // a plan that gets one iteration a tick never converges (69 % of goals in the band with
        // a zero budget). After this many in a row the solve is refused like any other failure.
        int nmpc_cancel_persist_n_{5};
        int nmpc_cancel_run_{0};
        // Furthest the solver may sit from the geometric goal. It is a garbage filter, not a
        // leash: at 1.5 m it fired in an ordinary 90 deg turn, where the solver cuts the corner
        // the geometric goal swings round, and the fallback then published a 1.5 m jump -- the
        // very thing this mode exists to remove. Safety is the range check beside it and
        // ApplyHardGuards, neither of which depends on this number.
        double nmpc_sanity_jump_m_{3.0};
        // The solver's standoff dead zone sits this far INSIDE the commanded band. Without the
        // inset it rides the band's far edge while trailing, and any lag (a person setting off
        // after a pause) spills outside it: 14 % of goals, by up to 0.43 m, on the bench.
        double nmpc_band_margin_m_{0.25};
        double nmpc_reseed_jump_m_{1.0};    // a geometric goal that moves this far is a step
        double nmpc_warm_max_age_s_{0.25};
        nmpc::Weights nmpc_w_;
        // Stage 2 runs only where nmpc_w_vis or nmpc_w_clear is > 0 (both default 0 = Stage 1),
        // and only on a field that is for this track, knows at least one height of the person,
        // and is fresh. Past the timeout the weight RAMPS to zero: a step would move the goal
        // by the whole visibility offset in one tick, which is the flapping this mode removes.
        // The field's age counts the person's displacement since it was built, at 1 m/s.
        std::string visibility_topic_{"/planning/visibility"};
        double nmpc_vis_timeout_s_{0.5};
        double nmpc_vis_fade_s_{0.3};
        nmpc::Limits nmpc_lim_;
        // Mirror of command_mux's fence, OFF by default so the mux stays the one source of
        // truth. On, stage_a_preflight must find this box equal to command_mux's.
        nmpc::Fence nmpc_fence_;

        std::string follow_target_topic_{"/dtp/follow_target"};
        std::string odom_topic_{"/lidar_slam/odom"};
        std::string goal_status_topic_{"/planning/goal_status"};
        std::string follow_goal_topic_{"/dtp/follow_goal"};
        std::string yaw_target_topic_{"/planning/yaw_target"};

        // Every field below is written by one callback and read by another. MutuallyExclusive
        // groups serialise callbacks WITHIN a group; they do not serialise ACROSS groups, and
        // main() runs a MultiThreadedExecutor, so odom / target / status / timer genuinely run in
        // parallel on different threads. One mutex over the whole state block is the cheap
        // correct answer: the critical sections are tens of microseconds of arithmetic, so there
        // is nothing to win from finer locking and plenty to lose from getting it wrong.
        std::mutex state_mtx_;

        Eigen::Vector3d cur_position_{Eigen::Vector3d::Zero()};
        double cur_yaw_{0.0};
        bool had_odom_{false};
        double odom_rcv_time_{0.0};

        Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};      // raw "now" estimate
        Eigen::Vector3d target_ref_position_{Eigen::Vector3d::Zero()};  // latency-compensated
        Eigen::Vector3d target_lead_position_{Eigen::Vector3d::Zero()}; // + lead along the path
        rclcpp::Time target_stamp_;
        bool had_target_{false};
        double target_rcv_time_{0.0};
        uint32_t target_track_id_{0};

        // The forecast itself, for goal_mode nmpc: times relative to target_stamp_. The
        // geometric path only ever needed its first point and an arc length along it.
        std::vector<double> target_path_t_;
        std::vector<Eigen::Vector3d> target_path_p_;
        Eigen::Vector3d target_velocity_{Eigen::Vector3d::Zero()};

        double heading_az_rad_{0.0};
        bool has_heading_{false};
        double last_in_fov_time_{0.0};
        uint32_t target_missed_frames_{0};

        // M6.5: the oracle's newest verdict for the current track, and when it arrived.
        dtp_msgs::msg::LossEvent loss_event_;
        bool has_loss_event_{false};
        double loss_event_rcv_time_{0.0};
        std::string conditioned_mode_;

        // The yaw hold: latched once per episode, cleared by a seen target or a fresh verdict.
        bool yaw_hold_latched_{false};
        bool yaw_hold_expired_{false};
        double yaw_hold_start_{0.0};
        Eigen::Vector3d yaw_hold_point_{Eigen::Vector3d::Zero()};
        // Where the camera was last told to look, by anything other than the hold itself.
        Eigen::Vector3d last_yaw_point_{Eigen::Vector3d::Zero()};
        bool has_last_yaw_point_{false};

        // goal_mode nmpc. The solver continues the signal IT produced (nmpc_g0_ and the point
        // before it), not the deadbanded value on the wire: the deadband holds the published
        // goal still for up to 0.15 m, and a velocity differenced from that is a sawtooth.
        nmpc::Solver nmpc_;
        Eigen::Matrix3Xd nmpc_prev_;
        Eigen::Vector3d nmpc_g0_{Eigen::Vector3d::Zero()};
        Eigen::Vector3d nmpc_v0_{Eigen::Vector3d::Zero()};
        double nmpc_prev_time_{0.0};
        bool nmpc_prev_valid_{false};
        double geo_az_{0.0};                 // the trail azimuth the geometric goal just used
        double nmpc_last_reproject_az_{0.0};
        double nmpc_last_reproject_radius_{0.0};
        Eigen::Vector3d nmpc_last_geo_{Eigen::Vector3d::Zero()};
        int nmpc_fallback_run_{0};
        bool nmpc_degraded_{false};
        // Stage 2: fsm_node's newest dtp_msgs/VisibilityField, already reduced to the two series
        // the solver reads. nmpc_vis_scale_ is how much of nmpc_w_vis / nmpc_w_clear this tick
        // used, for the log line: 0 is Stage 1.
        nmpc::Visibility nmpc_vis_;
        std::vector<double> nmpc_vis_occ_, nmpc_vis_clr_;   // the averaged bins behind nmpc_vis_
        bool nmpc_vis_valid_{false};
        double nmpc_vis_rcv_time_{0.0};
        uint32_t nmpc_vis_track_{0};
        Eigen::Vector3d nmpc_vis_centre_{Eigen::Vector3d::Zero()};
        double nmpc_vis_scale_{0.0};
        int nmpc_vis_side_{0};   // which way round a shadow the trail bearing was last moved
        uint64_t nmpc_ok_{0}, nmpc_fallbacks_{0}, nmpc_cancels_{0}, nmpc_resets_{0};
        uint64_t nmpc_reset_why_[4]{0, 0, 0, 0};  // nothing to continue, stale, re-projection, jump
        int nmpc_last_iters_{0};
        std::vector<double> nmpc_ms_;        // ring of recent solve times, for the log line
        size_t nmpc_ms_next_{0};

        // Goal servo / deadband state.
        Eigen::Vector3d published_goal_{Eigen::Vector3d::Zero()};
        double published_yaw_{0.0};
        bool has_published_goal_{false};

        // Correlation state for GoalStatus, which carries no sequence number.
        Eigen::Vector3d pending_goal_{Eigen::Vector3d::Zero()};
        double pending_publish_time_{0.0};
        bool has_pending_{false};
        bool status_link_ok_{false};

        // GoalStatus reaction state.
        //
        // has_ever_been_adopted_ exists for ONE reason and it is a real upstream defect, not
        // defensive padding: fsm.h:66-70 declares `struct GoalInfo { bool new_goal; Vec3f goal_p;
        // double goal_yaw; } gi_;` with no member initialisers, and Eigen's default constructor
        // does not zero. If the very first follow goal is deeply occupied, setFollowGoal returns
        // GOAL_DROPPED with out_goal_p = gi_.goal_p (fsm.cpp:230-232) and fsm_ros2.hpp:309-311
        // publishes that uninitialised memory to us as GoalStatus.goal_p. The FSM itself is
        // protected (started_{false} gates INIT), so this is a wire-protocol defect rather than a
        // flight defect -- but we would act on it. The proper fix is member initialisers in
        // fsm.h; this flag is the mitigation on our side and must stay until that lands. It is
        // deliberately NOT reset on target loss: it describes SUPER's lifetime, not ours.
        bool has_ever_been_adopted_{false};
        Eigen::Vector3d last_adopted_goal_{Eigen::Vector3d::Zero()};
        int consecutive_snaps_{0};
        int consecutive_drops_{0};
        double reproject_az_rad_{0.0};
        double reproject_radius_m_{0.0};
        bool suspended_{false};
        Eigen::Vector3d suspend_target_position_{Eigen::Vector3d::Zero()};

        std::string hold_reason_{"startup"};

        rclcpp::Subscription<dtp_msgs::msg::Target>::SharedPtr follow_target_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<dtp_msgs::msg::GoalStatus>::SharedPtr goal_status_sub_;
        rclcpp::Subscription<dtp_msgs::msg::LossEvent>::SharedPtr loss_event_sub_;
        rclcpp::Subscription<dtp_msgs::msg::VisibilityField>::SharedPtr visibility_sub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_goal_pub_;
        // Was target_point_pub_ on /dtp/target_point in the scaffold; see point 1 of the block
        // comment at the top of this file for why the name and the topic both had to move.
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yaw_target_pub_;

        rclcpp::TimerBase::SharedPtr goal_timer_;

        // One MutuallyExclusive callback group per subscription -- matches SUPER's own ROS2
        // convention (see WaypointPlanner) and is deliberate ahead of the Phase 2 M2.4 ROS2
        // hardening workstream: isolating callbacks avoids one slow callback blocking another
        // under a MultiThreadedExecutor.
        rclcpp::CallbackGroup::SharedPtr target_cb_group_, odom_cb_group_, goal_status_cb_group_,
                loss_event_cb_group_, visibility_cb_group_;
        rclcpp::CallbackGroup::SharedPtr goal_timer_cb_group_;

        // ---------------------------------------------------------------- small helpers

        static double WrapPi(double a) {
            while (a > M_PI) { a -= 2.0 * M_PI; }
            while (a < -M_PI) { a += 2.0 * M_PI; }
            return a;
        }

        static bool IsFinite(const Eigen::Vector3d &v) {
            return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
        }

        double Now() const { return nh_->get_clock()->now().seconds(); }

        // ---------------------------------------------------------------- callbacks

        void FollowTargetCallback(const dtp_msgs::msg::Target::SharedPtr msg) {
            // Compute the trail-behind standoff pose from msg->position + msg->predicted_path
            // (latency-compensated by the publisher already -- see dtp_msgs/msg/Target.msg),
            // clamp to [follow_near_m_, follow_far_m_], never below min_standoff_m_, bias toward
            // the near/open side (risk A1 mitigation, plan/dtp-imp/14), and publish
            // follow_goal_pub_ + yaw_target_pub_ with rate-limit/deadband applied. All of that
            // now happens in GoalTimerCallback -- point 2 of the block comment at the top of this
            // file says why it cannot happen here. This callback caches and validates only.
            const Eigen::Vector3d p_now(msg->position.x, msg->position.y, msg->position.z);
            const Eigen::Vector3d v(msg->velocity.x, msg->velocity.y, msg->velocity.z);

            // Reject non-finite input rather than let it reach the wire. A NaN in a goal is not a
            // degraded goal, it is an undefined one: setFollowGoal would run getNearestInfCellNot
            // on it, and closeToGoal() compares with `<`, which is false against NaN for every
            // threshold -- i.e. the drone would fly toward nowhere and never consider itself
            // arrived, in a state no watchdog in this system looks for.
            if (!IsFinite(p_now) || !IsFinite(v)) {
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                     "follow_target track %u carried a non-finite position/velocity; dropped.",
                                     msg->track_id);
                return;
            }

            std::lock_guard<std::mutex> lock(state_mtx_);
            const double now = Now();

            // Z IS UP here. Target.msg's header says "We keep Y (up, ENU)", which is wrong as
            // written -- Y-up is the PIPELINE frame's convention (plan/dtp-imp/01 SS3), not the
            // odom/world frame the message body declares twice; in ENU, Z is up. The standoff
            // math below therefore treats position.z as altitude and x/y as the ground plane.
            target_position_ = p_now;
            target_stamp_ = rclcpp::Time(msg->header.stamp);
            target_rcv_time_ = now;
            target_missed_frames_ = msg->missed_frames;

            // The publisher has ALREADY compensated one closed-loop sense->setpoint latency:
            // predicted_path[0] is where the person is estimated to be when our setpoint lands,
            // not "now" (Target.msg; plan/dtp-imp/14 change C5). Re-compensating here would
            // double-count it, so we take that point as the geometric reference and never add
            // latency of our own anywhere in this node.
            target_ref_position_ = p_now;
            if (!msg->predicted_path.empty()) {
                const Eigen::Vector3d p0(msg->predicted_path.front().x,
                                         msg->predicted_path.front().y,
                                         msg->predicted_path.front().z);
                if (IsFinite(p0)) { target_ref_position_ = p0; }
            }

            const double speed = std::hypot(v.x(), v.y());
            if (speed > kHeadingSpeedFloorMps) {
                heading_az_rad_ = std::atan2(v.y(), v.x());
                has_heading_ = true;
            } else if (!has_heading_) {
                // First sight of a person who is standing still: there is no heading to trail
                // from. Anchor the trail so "behind the person" is wherever the drone already is,
                // i.e. command no lateral motion at all. Guessing a heading here would fly the
                // drone in an arc around a stationary stranger for no reason at all.
                const double az_drone = std::atan2(cur_position_.y() - p_now.y(),
                                                   cur_position_.x() - p_now.x());
                heading_az_rad_ = WrapPi(az_drone + M_PI);
                has_heading_ = true;
            }

            target_lead_position_ = LeadAnchor(msg);

            target_velocity_ = v;
            target_path_t_.clear();
            target_path_p_.clear();
            const size_t n_path = std::min(msg->predicted_path.size(), msg->predicted_path_dt.size());
            for (size_t i = 0; i < n_path; ++i) {
                target_path_t_.push_back(msg->predicted_path_dt[i]);
                target_path_p_.emplace_back(msg->predicted_path[i].x, msg->predicted_path[i].y,
                                            msg->predicted_path[i].z);
            }

            if (!had_target_ || msg->track_id != target_track_id_) {
                // A new lock is a clean slate: the deadband must not compare against a goal from
                // before the gap, and any suspension was about the PREVIOUS person's surroundings.
                // has_ever_been_adopted_ is the one thing that survives -- see above.
                ResetServoState();
                suspended_ = false;
                last_in_fov_time_ = now;
                target_track_id_ = msg->track_id;
                has_loss_event_ = false;
            }
            if (msg->missed_frames == 0) {
                // A seen frame ends the coast; the oracle's verdict on it no longer applies.
                has_loss_event_ = false;
            }
            had_target_ = true;

            const double moved = (target_position_ - suspend_target_position_).norm();
            if (suspended_ && moved > suspend_reset_move_m_) {
                RCLCPP_INFO(nh_->get_logger(),
                            "target moved %.2f m since goals began being rejected; re-arming %s.",
                            moved, follow_goal_topic_.c_str());
                suspended_ = false;
                consecutive_drops_ = 0;
                consecutive_snaps_ = 0;
            }
        }

        void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            const Eigen::Vector3d p(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
            if (!IsFinite(p)) { return; }

            const auto &q = msg->pose.pose.orientation;
            const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));

            std::lock_guard<std::mutex> lock(state_mtx_);
            had_odom_ = true;
            odom_rcv_time_ = Now();
            cur_position_ = p;
            // POSE only, never twist. plan/dtp-imp/01 SS3 records that /lidar_slam/odom's pose is
            // world-frame while its twist is BODY-frame, and plan 01 SS2 still lists the
            // body-frame twist as an outstanding fast_lio item -- so pose is the half that is
            // unambiguous today, and the scaffold's choice to read only position was right.
            if (std::isfinite(yaw)) { cur_yaw_ = yaw; }
        }

        // Stage 2: reduce the field to its two azimuth series once per message (~250 multiply-adds
        // at 8 Hz), not once per cost evaluation. What "occluded" means is decided HERE:
        //   * a bin with no clear radius at all is as bad as a blocked one (occluded 1), and its
        //     clear radius reads as the near edge;
        //   * UNKNOWN is not occluded. 1 - vis - unk is what the map KNOWS is blocked, and a
        //     sensor that cannot see under 7 deg must not steer the goal away from a clear bearing;
        //   * a field that knows no height of the person says nothing, and is dropped whole.
        void VisibilityCallback(const dtp_msgs::msg::VisibilityField::SharedPtr msg) {
            const size_t n = msg->bins;
            if (n < 8 || msg->vis.size() != n || msg->unk.size() != n || msg->clear_r.size() != n ||
                !std::isfinite(msg->az0) || !(msg->daz > 0.0f) || msg->heights_used == 0) {
                std::lock_guard<std::mutex> lock(state_mtx_);
                nmpc_vis_valid_ = false;
                return;
            }
            std::vector<double> occ(n), clr(n);
            for (size_t i = 0; i < n; ++i) {
                const bool no_viewpoint = !std::isfinite(msg->clear_r[i]);
                clr[i] = no_viewpoint ? static_cast<double>(msg->r_near) : static_cast<double>(msg->clear_r[i]);
                const double known = (std::isfinite(msg->vis[i]) && std::isfinite(msg->unk[i]))
                    ? 1.0 - static_cast<double>(msg->vis[i]) - static_cast<double>(msg->unk[i]) : 0.0;
                occ[i] = no_viewpoint ? 1.0 : std::clamp(known, 0.0, 1.0);
            }
            // Average successive fields bin by bin before fitting: ROG-Map's cells at a trunk's
            // tangent flicker at frame rate (docs/m6-loss-oracle.md, M6.4b), and one toggling bin
            // should move the cost a little, slowly, not open and close a notch under the solver.
            // Same track and same binning only; anything else starts again from this field.
            {
                std::lock_guard<std::mutex> lock(state_mtx_);
                if (nmpc_vis_valid_ && nmpc_vis_track_ == msg->track_id && nmpc_vis_occ_.size() == n) {
                    for (size_t i = 0; i < n; ++i) {
                        occ[i] = 0.5 * occ[i] + 0.5 * nmpc_vis_occ_[i];
                        clr[i] = 0.5 * clr[i] + 0.5 * nmpc_vis_clr_[i];
                    }
                }
                nmpc_vis_occ_ = occ;
                nmpc_vis_clr_ = clr;
            }
            const nmpc::Visibility v = nmpc::Visibility::FromBins(occ, clr, msg->az0, msg->daz);
            const Eigen::Vector3d centre(msg->centre_p.x, msg->centre_p.y, msg->centre_p.z);
            if (!IsFinite(centre)) { return; }
            std::lock_guard<std::mutex> lock(state_mtx_);
            nmpc_vis_ = v;
            nmpc_vis_valid_ = true;
            nmpc_vis_rcv_time_ = Now();
            nmpc_vis_track_ = msg->track_id;
            nmpc_vis_centre_ = centre;
        }

        // M6.5: cache the oracle's newest verdict for OUR track; the timer decides.
        void LossEventCallback(const dtp_msgs::msg::LossEvent::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mtx_);
            if (!had_target_ || msg->track_id != target_track_id_) { return; }
            loss_event_ = *msg;
            has_loss_event_ = true;
            loss_event_rcv_time_ = Now();
        }

        // The verdict to condition on this tick, or nullptr (conditioning off, target seen, no
        // event, event stale, or event for another track).
        const dtp_msgs::msg::LossEvent *ConditioningEvent(double now) const {
            if (!loss_conditioning_ || !has_loss_event_ || target_missed_frames_ == 0) { return nullptr; }
            if ((now - loss_event_rcv_time_) > loss_event_timeout_s_) { return nullptr; }
            if (loss_event_.track_id != target_track_id_) { return nullptr; }
            return &loss_event_;
        }

        void GoalStatusCallback(const dtp_msgs::msg::GoalStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mtx_);

            const double stamp = rclcpp::Time(msg->header.stamp).seconds();
            const Eigen::Vector3d adopted(msg->goal_p.x, msg->goal_p.y, msg->goal_p.z);

            // On STATUS_DROPPED, hold the last good goal / re-project rather than assuming the
            // published goal was honored (risk R-F4 / A6). Getting there first needs correlation.
            //
            // GoalStatus originally carried no correlation key at all: no sequence number, no echo
            // of the REQUESTED point (only the ADOPTED one), and header.stamp is fsm_node's INGEST
            // time rather than our request's stamp. That is not merely imprecise, it is wrong in a
            // specific way -- /planning/follow_goal has SEVERAL publishers. command_mux republishes
            // its own HOLD goals there at 2 Hz whenever a veto stands, and fsm answers every one of
            // them. Gating only ACCEPTED on geometry let a status answering a mux HOLD fall into
            // the SNAPPED branch, where the ~3 m gap to our standoff goal read as a large snap:
            // five of those and we suspended ourselves mid-follow, exactly when the mux was about
            // to hand authority back. It ran the other way too -- a mux hold that came back snapped
            // within tolerance silently cleared an escalation we had legitimately earned from real
            // rejections of our own goals.
            //
            // Fixed at the source on 2026-09-08 by adding requested_p to dtp_msgs/GoalStatus and
            // echoing it in fsm_ros2.hpp. Correlate on THAT, for every status: it is authoritative
            // where goal_p is not, because on DROPPED goal_p is the previous goal and bears no
            // geometric relationship to the request.
            if (!has_pending_) { return; }                    // answers somebody else's goal (an RViz click)
            if (stamp <= pending_publish_time_) { return; }   // ingested before we published: older request
            const Eigen::Vector3d requested(msg->requested_p.x, msg->requested_p.y,
                                            msg->requested_p.z);
            if ((requested - pending_goal_).norm() > kAcceptedMatchTolM) {
                return;  // answers a goal we did not send (command_mux's HOLD or NAV, or an RViz click)
            }

            has_pending_ = false;
            status_link_ok_ = true;

            switch (msg->status) {
                case dtp_msgs::msg::GoalStatus::STATUS_ACCEPTED: {
                    has_ever_been_adopted_ = true;
                    last_adopted_goal_ = adopted;
                    consecutive_snaps_ = 0;
                    consecutive_drops_ = 0;
                    RelaxReprojection();
                    // ACCEPTED means INGESTED, not REACHABLE and not PLANNED. SUPER can still fail
                    // to build a trajectory afterwards ("Goal is invalid, skip this goal",
                    // fsm.cpp:139-142, straight back to WAIT_GOAL) and that failure is reported on
                    // NO topic. Do not read this as confirmation that the drone is moving.
                    break;
                }
                case dtp_msgs::msg::GoalStatus::STATUS_SNAPPED: {
                    has_ever_been_adopted_ = true;
                    last_adopted_goal_ = adopted;
                    const double delta = (adopted - pending_goal_).norm();
                    if (delta <= snap_accept_m_) {
                        // The COMMON case, not an edge case: the 2026-08-14 closed-loop test on
                        // the NX reported SNAPPED rather than ACCEPTED (CLAUDE.md SS13 / M0.7
                        // 0.7.4). Adopt it and do not fight it -- republishing the identical
                        // requested point makes SUPER snap it identically and we oscillate.
                        consecutive_snaps_ = 0;
                        consecutive_drops_ = 0;
                        RelaxReprojection();
                        break;
                    }
                    ++consecutive_snaps_;
                    EscalateReprojection();
                    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                         "goal snapped %.2f m (> snap_accept_m %.2f), %d in a row; "
                                         "re-projecting standoff az%+.0f deg r+%.2f m.",
                                         delta, snap_accept_m_, consecutive_snaps_,
                                         reproject_az_rad_ * 180.0 / M_PI, reproject_radius_m_);
                    if (consecutive_snaps_ >= snap_persist_n_) {
                        // Sustained snapping means the person is walking beside an obstacle and
                        // the trail-behind viewpoint is simply not achievable here. Same
                        // escalation as an outright drop.
                        Suspend("standoff repeatedly snapped away");
                    }
                    break;
                }
                case dtp_msgs::msg::GoalStatus::STATUS_DROPPED: {
                    // THE PREVIOUS GOAL IS STILL ACTIVE AND SUPER IS STILL FLYING TO IT. That is
                    // what makes this the dangerous status: the drone is heading to a stale point
                    // while we would otherwise believe we had commanded a new one. So
                    // last_adopted_goal_ becomes the goal SUPER is still using -- never our request.
                    if (has_ever_been_adopted_) {
                        last_adopted_goal_ = adopted;
                    } else {
                        RCLCPP_ERROR(nh_->get_logger(),
                                     "first-ever follow goal was DROPPED; IGNORING its goal_p "
                                     "(%.2f %.2f %.2f) -- SUPER's GoalInfo has no member initialisers "
                                     "(fsm.h:66-70), so that is uninitialised memory, not a position.",
                                     adopted.x(), adopted.y(), adopted.z());
                    }
                    ++consecutive_drops_;
                    EscalateReprojection();
                    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                         "goal DROPPED (deeply occupied), %d in a row; the PREVIOUS goal "
                                         "is still active; re-projecting standoff az%+.0f deg r+%.2f m.",
                                         consecutive_drops_,
                                         reproject_az_rad_ * 180.0 / M_PI, reproject_radius_m_);
                    if (consecutive_drops_ >= drop_persist_n_) {
                        Suspend("goals repeatedly dropped");
                    }
                    break;
                }
                default: {
                    // The enum and the .msg agree numerically today (fsm.h:101-105 vs
                    // GoalStatus.msg) and fsm_ros2.hpp:308 casts between them with a bare
                    // static_cast<uint8_t>, so if either is ever reordered without the other this
                    // branch is the only thing that will notice. Fail closed.
                    RCLCPP_WARN(nh_->get_logger(),
                                "GoalStatus carried unknown status %u -- fsm's GoalIngestStatus and "
                                "dtp_msgs/GoalStatus have diverged. Treating as DROPPED.",
                                static_cast<unsigned>(msg->status));
                    ++consecutive_drops_;
                    if (consecutive_drops_ >= drop_persist_n_) {
                        Suspend("unknown goal status");
                    }
                    break;
                }
            }
        }

        // ---------------------------------------------------------------- the goal servo

        void GoalTimerCallback() {
            std::lock_guard<std::mutex> lock(state_mtx_);
            const double now = Now();

            ExpirePendingStatus(now);

            const bool target_fresh = had_target_ && (now - target_rcv_time_) <= follow_target_timeout_s_;
            const bool odom_fresh = had_odom_ && (now - odom_rcv_time_) <= odom_timeout_s_;

            if (!target_fresh) {
                // Absence IS the signal, which is why this covers two very different situations
                // with one test: target_manager released the lock (IDLE/ABSTAIN/LOST all publish
                // nothing), or perception died. Neither is a reason to keep flying at a ghost.
                // A lost lock also clears any suspension: whatever geometry was rejecting our
                // goals belonged to a situation that no longer exists.
                suspended_ = false;
                Hold(had_target_ ? "target stale" : "no target yet");
                if (had_target_) { YawHold(now, "lost"); }
                return;
            }
            if (!odom_fresh) {
                Hold(had_odom_ ? "odom stale" : "no odom yet");
                return;
            }

            // Yaw is published even when the goal is being withheld, and the asymmetry is
            // deliberate: a yaw target moves no mass toward the person, it only decides where the
            // camera points along whatever trajectory SUPER is already flying -- including one
            // built from a NAV goal after the mux has fallen through. So during a handover the
            // camera keeps tracking the person even though FOLLOW is commanding nothing, which is
            // what lets target_manager keep its lock alive instead of coasting blind. The goal is
            // the committal half, so the goal is the half that gets withheld.
            const dtp_msgs::msg::LossEvent *ev = ConditioningEvent(now);
            const bool ev_occluded = ev != nullptr && ev->cls == dtp_msgs::msg::LossEvent::CLS_OCCLUDED;
            const bool ev_out_of_view = ev != nullptr && ev->cls == dtp_msgs::msg::LossEvent::CLS_OUT_OF_VIEW;
            if (ev_occluded) {
                ReleaseYawHold(now, "the oracle's OCCLUDED verdict aims the camera");
                AimYawAt(Eigen::Vector3d(ev->yaw_target_p.x, ev->yaw_target_p.y, ev->yaw_target_p.z), now);
                NoteConditioned("OCCLUDED: yaw to the predicted re-emergence point");
            } else if (ev_out_of_view) {
                ReleaseYawHold(now, "the oracle's OUT_OF_VIEW verdict aims the camera");
                AimYawAt(target_ref_position_, now);
                NoteConditioned("OUT_OF_VIEW: yaw to the CTRV point");
            } else {
                // Past kCoastYawHoldAgeS SUPER would drop the perception-stamped target: hold
                // instead. If the hold cannot publish (off, window run out), the old behaviour.
                const bool coast_hold_due = CoastYawHoldDue(now);
                if (!(coast_hold_due && YawHold(now, "coasting past SUPER's 0.5 s yaw gate"))) {
                    if (!coast_hold_due) { ReleaseYawHold(now, "the target is seen"); }
                    PublishYawTarget();
                }
                if (!conditioned_mode_.empty()) { conditioned_mode_.clear(); }
            }

            if (TargetBehind(now)) {
                Hold("target behind the drone");
                return;
            }

            if (suspended_) {
                // Deliberate silence: point 3 of the block comment at the top of this file.
                Hold("suspended after repeated goal rejections");
                return;
            }

            Eigen::Vector3d goal;
            double goal_yaw = published_yaw_;
            if (ev_occluded && ev->viewpoint_valid) {
                // Pre-position for the edge: the oracle already scored the annulus against the
                // map. The floor is still enforced against the person's present position, and Z
                // is still clamped, because a safety number checked elsewhere is not a safety
                // number checked here.
                goal = Eigen::Vector3d(ev->viewpoint_p.x, ev->viewpoint_p.y, ev->viewpoint_p.z);
                const double az = std::atan2(goal.y() - target_position_.y(), goal.x() - target_position_.x());
                if (!IsFinite(goal) || !EnforceStandoffFloor(goal, target_position_, az)) {
                    Hold("conditioned viewpoint degenerate");
                    return;
                }
                goal.z() = std::clamp(goal.z(), goal_z_min_m_, goal_z_max_m_);
                const double dx = ev->yaw_target_p.x - goal.x();
                const double dy = ev->yaw_target_p.y - goal.y();
                if (std::hypot(dx, dy) >= kLosHorizontalFloorM) { goal_yaw = std::atan2(dy, dx); }
            } else if (!ComputeStandoffGoal(goal, goal_yaw)) {
                Hold("standoff geometry degenerate");
                return;
            }

            if (TransitCrossesKeepOut(goal)) {
                Hold("transit to the standoff would pass inside the standoff floor");
                return;
            }

            // Deadband and park guard. Note that we keep PUBLISHING on every tick -- command_mux
            // needs the liveness, and silence is how we tell it FOLLOW is unavailable -- but we
            // hold the goal VALUE still, so SUPER receives an identical point rather than a
            // jittering one. This matters because setFollowGoal sets gi_.new_goal = true on EVERY
            // message it receives (fsm.cpp:239): an undeadbanded goal drives WAIT_GOAL ->
            // GENERATE_TRAJ continuously, while an identical one is harmless, because
            // GENERATE_TRAJ then finds closeToGoal(0.1) true and returns without replanning.
            if (has_published_goal_ &&
                ((goal - published_goal_).norm() <= goal_deadband_m_ ||
                 (goal - cur_position_).norm() < kParkGuardM)) {
                goal = published_goal_;
                goal_yaw = published_yaw_;
            }

            PublishGoal(goal, goal_yaw, now);
            hold_reason_.clear();
            LogNmpcStats();

            // Both the REQUESTED goal and the one SUPER actually adopted are logged, because they
            // are routinely different -- SNAPPED is the common case on hardware, not the
            // exception -- and the gap between the two lines is the fastest way to tell a
            // standoff that is being fought by the map from one that is simply wrong.
            RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), kStateLogPeriodMs,
                                 "FOLLOW track %u | person %.1f %.1f %.1f | goal %.1f %.1f %.1f | "
                                 "adopted %.1f %.1f %.1f%s | standoff %.2f m | az%+.0f deg r+%.2f m | "
                                 "snaps %d drops %d | status link %s",
                                 target_track_id_,
                                 target_position_.x(), target_position_.y(), target_position_.z(),
                                 goal.x(), goal.y(), goal.z(),
                                 last_adopted_goal_.x(), last_adopted_goal_.y(), last_adopted_goal_.z(),
                                 has_ever_been_adopted_ ? "" : " (none yet)",
                                 std::hypot(goal.x() - target_position_.x(),
                                            goal.y() - target_position_.y()),
                                 reproject_az_rad_ * 180.0 / M_PI, reproject_radius_m_,
                                 consecutive_snaps_, consecutive_drops_,
                                 status_link_ok_ ? "up" : "SILENT");
        }

        // Consume lead_distance_m_ of ARC LENGTH along the CTRV predicted path. Arc length rather
        // than a fixed time offset is the whole trick: it makes the lead shrink to zero by itself
        // as the person slows, so a stationary person gets no lead at all (their predicted path
        // collapses toward a point) and the standoff is then measured from where they actually
        // are. A time-based lead would keep pushing the goal forward along a heading that has
        // stopped being real, which is the walk-pause-walk failure plan 07 M2.3.1 gates on.
        Eigen::Vector3d LeadAnchor(const dtp_msgs::msg::Target::SharedPtr &msg) const {
            Eigen::Vector3d prev = target_ref_position_;
            double remaining = lead_distance_m_;

            for (const auto &pt : msg->predicted_path) {
                const Eigen::Vector3d cur(pt.x, pt.y, pt.z);
                if (!IsFinite(cur)) { break; }
                const double seg = (cur - prev).norm();
                if (seg < 1e-9) { continue; }
                if (seg >= remaining) {
                    return prev + (cur - prev) * (remaining / seg);
                }
                remaining -= seg;
                prev = cur;
            }

            // No path at all (the predictor has not converged, or the publisher omitted it).
            // Extrapolate along the held heading instead, but only while the person is genuinely
            // moving -- otherwise fall through with no lead, which is the correct answer for
            // someone standing still. A path that merely ran SHORT is left alone: its end is the
            // furthest point the predictor is willing to stand behind, and inventing more than it
            // offered would be flying on a forecast nobody made.
            if (msg->predicted_path.empty() && has_heading_ && remaining > 0.0 &&
                std::hypot(msg->velocity.x, msg->velocity.y) > kHeadingSpeedFloorMps) {
                prev.x() += remaining * std::cos(heading_az_rad_);
                prev.y() += remaining * std::sin(heading_az_rad_);
            }
            return prev;
        }

        // THE SEAM for goal_mode. GoalTimerCallback calls this and nothing else, so every guard
        // around the call (staleness, TargetBehind, the transit keep-out, the deadband and park
        // guard, the GoalStatus escalation) wraps whichever implementation answers. The
        // geometric goal is computed on EVERY tick in every mode: it is the shipping behaviour,
        // and it is what goal_mode nmpc cold-starts from and falls back to. Same contract as
        // before the split: false means "no goal this tick" and the caller Holds.
        bool ComputeStandoffGoal(Eigen::Vector3d &goal, double &goal_yaw) {
            Eigen::Vector3d geo = goal;
            double geo_yaw = goal_yaw;
            if (!ComputeStandoffGoalGeometric(geo, geo_yaw)) { return false; }
            if (goal_mode_ != "nmpc" || stock_baseline_ || nmpc_degraded_) {
                goal = geo;
                goal_yaw = geo_yaw;
                return true;
            }
            if (!ComputeStandoffGoalNmpc(geo, goal, goal_yaw)) {
                // The solver only ever offers a candidate. Refusing it is not a reason to stop
                // following: the geometric goal is a live decision made this tick, so the node
                // keeps publishing (it never Holds or Suspends on its own arithmetic) and the
                // next solve starts cold.
                goal = geo;
                goal_yaw = geo_yaw;
                nmpc_prev_valid_ = false;
                ++nmpc_fallbacks_;
                if (++nmpc_fallback_run_ >= nmpc_fallback_persist_n_) {
                    nmpc_degraded_ = true;
                    RCLCPP_ERROR(nh_->get_logger(),
                                 "goal_mode nmpc: %d consecutive solves refused -- DEGRADED to the "
                                 "geometric goal until this node is relaunched.", nmpc_fallback_run_);
                }
            }
            return true;
        }

        // The nmpc arm of ComputeStandoffGoal. `geo` is this tick's geometric goal: the cold
        // start, the sanity reference, and what the caller publishes if this returns false.
        bool ComputeStandoffGoalNmpc(const Eigen::Vector3d &geo, Eigen::Vector3d &goal, double &goal_yaw) {
            const double now = Now();
            const double dt = nmpc_dt_s_;

            // How much horizon the forecast can carry. A thin forecast or a deep coast gets a
            // short one, held harder to its own history: the same distrust of CTRV past ~1.5 s
            // that the rest of this node has, made continuous.
            nmpc::Problem prob;
            const double coast = static_cast<double>(target_missed_frames_);
            const double shrink = nmpc_coast_shrink_
                ? std::clamp(1.0 - coast / static_cast<double>(nmpc_coast_full_n_), 0.33, 1.0) : 1.0;
            int n = std::max(4, static_cast<int>(std::lround(nmpc_stages_ * shrink)));

            // Stage k is the person at (first forecast point) + (age of the message) + k ticks.
            // The first forecast point already carries the closed-loop latency and nothing is
            // added to it; the age only keeps the 15 Hz horizon moving between 8 Hz detections.
            const double t_first = target_path_t_.empty() ? 0.0 : target_path_t_.front();
            const double t0 = t_first + std::max(0.0, now - target_rcv_time_);
            int usable = 0;
            prob.target = nmpc::SampleTargetPath(target_path_t_, target_path_p_, target_ref_position_,
                                                 target_velocity_, t0, dt, n, usable);
            if (usable < 2 && n > nmpc_thin_stages_) {
                n = nmpc_thin_stages_;
                prob.target = nmpc::SampleTargetPath(target_path_t_, target_path_p_,
                                                     target_ref_position_, target_velocity_, t0, dt, n,
                                                     usable);
            }
            prob.n = n;
            prob.dt = dt;
            prob.p_now = target_position_;
            prob.az_des = geo_az_;
            prob.band_hi = follow_far_m_ - nmpc_band_margin_m_;
            prob.band_lo = std::clamp(follow_near_m_ + nmpc_band_margin_m_ + reproject_radius_m_,
                                      follow_near_m_, prob.band_hi);
            prob.floor_m = min_standoff_m_ + kFloorReserveM;
            prob.z_offset = goal_height_offset_m_;
            prob.z_min = goal_z_min_m_;
            prob.z_max = goal_z_max_m_;
            prob.publish_stage = std::min(nmpc_publish_stage_, n);
            prob.w = nmpc_w_;
            prob.w.prev *= 1.0 + 0.2 * coast;
            nmpc_vis_scale_ = 0.0;
            if ((nmpc_w_.vis > 0.0 || nmpc_w_.clear > 0.0) && nmpc_vis_valid_ &&
                nmpc_vis_track_ == target_track_id_) {
                const double moved = std::hypot(target_position_.x() - nmpc_vis_centre_.x(),
                                                target_position_.y() - nmpc_vis_centre_.y());
                const double age = (now - nmpc_vis_rcv_time_) + moved / 1.0;
                nmpc_vis_scale_ = std::clamp(1.0 - (age - nmpc_vis_timeout_s_) / nmpc_vis_fade_s_, 0.0, 1.0);
                prob.visibility = nmpc_vis_;
            }
            prob.w.vis *= nmpc_vis_scale_;
            prob.w.clear *= nmpc_vis_scale_;
            // A trail bearing that lies in a known shadow is moved to the nearest clear one, and
            // the ordinary bearing term takes the goal there (nmpc::ClearBearing says why the
            // visibility cost cannot be left to do it). Only while the field has weight; when it
            // fades the asked-for bearing returns and the goal walks back at its own speed limit.
            if (nmpc_vis_scale_ > 0.0 && nmpc_w_.vis > 0.0) {
                prob.az_des = nmpc::ClearBearing(prob.visibility, geo_az_, nmpc_vis_side_);
            } else {
                nmpc_vis_side_ = 0;
            }
            prob.lim = nmpc_lim_;
            prob.fence = nmpc_fence_;
            prob.max_iters = nmpc_max_iters_;
            prob.time_budget_ms = nmpc_time_budget_ms_;

            // A COMMANDED step must not be smoothed away. A new track, a re-projection (which
            // exists to move the goal), a jump in the geometric answer or a stale horizon all
            // restart the signal from the geometric goal instead of easing towards it.
            const double elapsed = now - nmpc_prev_time_;
            // Why, in the order tested: no horizon to continue (a new track, a Hold, a refused
            // solve), a stale one, a re-projection, the geometric goal jumped.
            // The trail AZIMUTH is deliberately not tested. It was (a step > 0.2 rad), to catch
            // the re-projection's alternating azimuth offset, and on run5 it fired 44 times in
            // 1587 solves on nothing but heading noise -- the heading is atan2 of a differenced
            // velocity -- each one a jump to the geometric goal. Smoothing that noise is what
            // this mode is for; the re-projection is caught by its own two variables.
            const int why_reset = !nmpc_prev_valid_ ? 0
                : (elapsed <= 0.0 || elapsed > nmpc_warm_max_age_s_) ? 1
                : ReprojectionEscalated() ? 2
                : (geo - nmpc_last_geo_).norm() > nmpc_reseed_jump_m_ ? 3 : -1;
            const bool reset = why_reset >= 0;
            if (reset) {
                ++nmpc_resets_;
                ++nmpc_reset_why_[why_reset];
                prob.g0 = geo;
                prob.v0.setZero();
                prob.has_prev = false;
                prob.init.resize(3, n);
                for (int k = 0; k < n; ++k) { prob.init.col(k) = geo + (prob.target.col(k) - prob.target.col(0)); }
            } else {
                prob.g0 = nmpc_g0_;
                prob.v0 = nmpc_v0_;
                prob.prev = nmpc::ShiftHorizon(nmpc_prev_, nmpc_g0_, elapsed, dt, n);
                prob.has_prev = true;
                prob.init = prob.prev;
            }
            nmpc_last_reproject_az_ = reproject_az_rad_;
            nmpc_last_reproject_radius_ = reproject_radius_m_;
            nmpc_last_geo_ = geo;

            const nmpc::Result r = nmpc_.Solve(prob);
            if (nmpc_ms_.size() < 512) { nmpc_ms_.push_back(r.solve_ms); }
            else { nmpc_ms_[nmpc_ms_next_++ % 512] = r.solve_ms; }
            nmpc_last_iters_ = r.iterations;
            if (r.budget_cancel) { ++nmpc_cancels_; }
            nmpc_cancel_run_ = r.budget_cancel ? nmpc_cancel_run_ + 1 : 0;
            if (r.solve_ms > nmpc_time_panic_ms_) {
                RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                      "goal_mode nmpc: solve took %.1f ms (budget %.1f, panic %.1f)",
                                      r.solve_ms, nmpc_time_budget_ms_, nmpc_time_panic_ms_);
            }

            // The sanity bracket. The solver is licensed to shade the geometric answer, not to
            // disagree with it wholesale, which is what makes "never publish garbage" checkable.
            const char *why = r.why;
            bool ok = r.ok;
            if (ok && nmpc_cancel_run_ > nmpc_cancel_persist_n_) { ok = false; why = "starved of solve time"; }
            if (ok && (r.goal - geo).norm() > nmpc_sanity_jump_m_) { ok = false; why = "too far from the geometric goal"; }
            if (ok) {
                const double range = std::hypot(r.goal.x() - target_position_.x(),
                                                r.goal.y() - target_position_.y());
                if (range < min_standoff_m_ - 0.5 || range > follow_far_m_ + 1.5) {
                    ok = false;
                    why = "range to the person out of bounds";
                }
            }
            if (!ok) {
                RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                                     "goal_mode nmpc: solve refused (%s, lbfgs %d); publishing the "
                                     "geometric goal this tick.", why, r.lbfgs_code);
                return false;
            }

            goal = r.goal;
            if (!ApplyHardGuards(goal, target_ref_position_, target_position_, geo_az_)) { return false; }
            const double dx = target_ref_position_.x() - goal.x();
            const double dy = target_ref_position_.y() - goal.y();
            if (std::hypot(dx, dy) >= kLosHorizontalFloorM) { goal_yaw = std::atan2(dy, dx); }
            if (!IsFinite(goal) || !std::isfinite(goal_yaw)) { return false; }

            // Continue from what was actually output (after the hard guards), at its own rate.
            Eigen::Vector3d v = reset ? Eigen::Vector3d::Zero() : Eigen::Vector3d((goal - nmpc_g0_) / elapsed);
            const double vmax = nmpc_lim_.goal_vel_max;
            if (v.norm() > vmax) { v *= vmax / v.norm(); }
            nmpc_v0_ = v;
            nmpc_g0_ = goal;
            nmpc_prev_ = r.horizon;
            nmpc_prev_time_ = now;
            nmpc_prev_valid_ = true;
            nmpc_fallback_run_ = 0;
            ++nmpc_ok_;
            return true;
        }

        // An ESCALATION is a step that exists to move the goal (the azimuth offset grows and
        // flips sign, the radius steps out), so it restarts the signal. A RELAXATION is a decay
        // on every ACCEPTED status, which the solver should simply follow; restarting on it
        // would reset at status rate for seconds after every escalation.
        bool ReprojectionEscalated() const {
            return std::fabs(reproject_az_rad_) > std::fabs(nmpc_last_reproject_az_) + 1e-9 ||
                   reproject_az_rad_ * nmpc_last_reproject_az_ < 0.0 ||
                   reproject_radius_m_ > nmpc_last_reproject_radius_ + 1e-9;
        }

        void LogNmpcStats() {
            if (goal_mode_ != "nmpc" || nmpc_ms_.empty()) { return; }
            std::vector<double> ms = nmpc_ms_;
            std::sort(ms.begin(), ms.end());
            RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), kStateLogPeriodMs,
                                 "NMPC solve p50 %.3f ms p99 %.3f ms max %.3f ms over %zu | it %d | "
                                 "ok %lu fallback %lu cancel %lu reset %lu (new %lu stale %lu "
                                 "reproject %lu jump %lu) | vis %.2f%s",
                                 ms[ms.size() / 2], ms[ms.size() * 99 / 100], ms.back(), ms.size(),
                                 nmpc_last_iters_, static_cast<unsigned long>(nmpc_ok_),
                                 static_cast<unsigned long>(nmpc_fallbacks_),
                                 static_cast<unsigned long>(nmpc_cancels_),
                                 static_cast<unsigned long>(nmpc_resets_),
                                 static_cast<unsigned long>(nmpc_reset_why_[0]),
                                 static_cast<unsigned long>(nmpc_reset_why_[1]),
                                 static_cast<unsigned long>(nmpc_reset_why_[2]),
                                 static_cast<unsigned long>(nmpc_reset_why_[3]), nmpc_vis_scale_,
                                 nmpc_degraded_ ? " | DEGRADED to geometric" : "");
        }

        bool ComputeStandoffGoalGeometric(Eigen::Vector3d &goal, double &goal_yaw) {
            const Eigen::Vector3d &p_now = target_position_;
            const Eigen::Vector3d &p_ref = target_ref_position_;
            const Eigen::Vector3d &anchor = target_lead_position_;

            // Azimuth of the viewpoint, measured from the person outward.
            const double az_behind = WrapPi(heading_az_rad_ + M_PI);
            const double az_drone = std::atan2(cur_position_.y() - p_ref.y(),
                                               cur_position_.x() - p_ref.x());

            const double bias_max = near_side_bias_max_deg_ * M_PI / 180.0;
            const double near_side = std::clamp(WrapPi(az_drone - az_behind), -bias_max, bias_max);
            const double offset = std::clamp(near_side + reproject_az_rad_,
                                             -kMaxAzimuthOffsetRad, kMaxAzimuthOffsetRad);
            const double az = WrapPi(az_behind + offset);
            geo_az_ = az;

            if (stock_baseline_) {
                // M6.8 comparison arm: directly behind the person's PRESENT position on the
                // constant-velocity heading, mid-band standoff, nothing else. The goal contract
                // (snap/drop escalation) still applies -- that is SUPER's, not the follower's.
                const double standoff_s = std::clamp(0.5 * (follow_near_m_ + follow_far_m_) + reproject_radius_m_,
                                                     follow_near_m_, follow_far_m_);
                goal.x() = p_now.x() + standoff_s * std::cos(az_behind);
                goal.y() = p_now.y() + standoff_s * std::sin(az_behind);
                goal.z() = std::clamp(p_now.z() + goal_height_offset_m_, goal_z_min_m_, goal_z_max_m_);
                if (!EnforceStandoffFloor(goal, p_now, az_behind)) { return false; }
                const double sdx = p_now.x() - goal.x(), sdy = p_now.y() - goal.y();
                if (std::hypot(sdx, sdy) >= kLosHorizontalFloorM) { goal_yaw = std::atan2(sdy, sdx); }
                return IsFinite(goal) && std::isfinite(goal_yaw);
            }

            // Nominal commanded standoff is the MIDPOINT of the band, not its near edge, and the
            // reason is arithmetic rather than taste. The near edge leaves only
            // (follow_near_m - min_standoff_m) = 0.5 m of margin over the floor, and the latency
            // compensation already baked into predicted_path[0] spends ~0.3 m of that at walking
            // pace -- so a near-edge nominal leaves nothing for the lead and parks the commanded
            // goal exactly on the floor, which is the one place it must not sit. The midpoint also
            // gives the re-projection somewhere to step OUTWARD to, which is the only direction it
            // is permitted to move.
            const double nominal = 0.5 * (follow_near_m_ + follow_far_m_);
            const double standoff = std::clamp(nominal + reproject_radius_m_,
                                               follow_near_m_, follow_far_m_);

            // THE LEAD AND THE STANDOFF PULL ON THE SAME AXIS, so one of them has to give, and it
            // is not going to be the floor. Both are measured along the person's direction of
            // travel: pushing the goal `lead` further down the path moves it `lead` CLOSER to the
            // person, so a naive 1.0 m lead on a 3.0 m standoff commands 2.0 m -- under the 2.5 m
            // floor, on every cycle. (The obvious repair, adding the lead back into the radius,
            // is worse than it looks: goal = anchor - (standoff + lead)*u reduces algebraically to
            // person - standoff*u, so it cancels the lead exactly and buys nothing.)
            //
            // So the lead gets a BUDGET: the margin between the commanded standoff and the hard
            // floor, less the floor reserve and less the prediction gap below. At the defaults
            // that works out to 3.5 - 2.5 - 0.25 - 0.28 = 0.47 m of lead at a 1.4 m/s walk, and it
            // grows toward the full lead_distance_m_ as a re-projection pushes the standoff out.
            // The commanded goal then sits 2.75 m from the person -- the reserve above the floor,
            // by construction rather than by luck.
            //
            // WHAT THIS BUYS AND WHAT IT DOES NOT, because the plan overstates it: plan/dtp-imp/04
            // SS1 says the lead "prevents SUPER from parking". It cannot. A paused person gives a
            // static goal, a static goal is reachable, and the drone arrives -- no offset fixes
            // that. What actually happens on arrival is benign: GENERATE_TRAJ finds closeToGoal
            // true and returns to WAIT_GOAL without calling PlanFromRest, so the drone holds
            // rather than stuttering. The lead's real job is the WALKING case, keeping the goal
            // ahead of a drone that would otherwise close on it, and softening the resume when a
            // paused person starts again. The band [follow_near_m, follow_far_m] describes the
            // ACHIEVED standoff -- commanded plus the drone's tracking lag -- and confirming the
            // achieved value lands inside it is an M2.3.1 measurement (plan 07 Step 2.3.1), not
            // something this node can assert about itself. For calibration: a walk-pause-walk
            // desk simulation of this exact arithmetic against a first-order drone with tau =
            // 0.3 s settled at a 3.13 m achieved standoff, never commanded closer than 2.75 m, and
            // never engaged the park guard. Those are desk numbers off an assumed tau, not flight
            // numbers -- they say the geometry is self-consistent, nothing more.
            //
            // Note this uses the anchor LeadAnchor() actually produced, not lead_distance_m_: the
            // anchor collapses onto the person when they slow, so the lead fades out on its own.
            double lead_x = anchor.x() - p_ref.x();
            double lead_y = anchor.y() - p_ref.y();
            const double lead_raw = std::hypot(lead_x, lead_y);
            // pred_gap is subtracted because the standoff above is measured from p_ref (where the
            // person will be when the setpoint lands) while the floor is enforced against p_now
            // (where they are). At 1.4 m/s and a 0.2 s closed loop that gap is ~0.3 m, and if the
            // budget ignores it the lead quietly spends the floor reserve instead of its own.
            const double pred_gap = std::hypot(p_ref.x() - p_now.x(), p_ref.y() - p_now.y());
            const double lead_budget = std::max(
                    0.0, standoff - min_standoff_m_ - kFloorReserveM - pred_gap);
            if (lead_raw > lead_budget) {
                const double k = (lead_raw > 1e-9) ? (lead_budget / lead_raw) : 0.0;
                lead_x *= k;
                lead_y *= k;
            }

            goal.x() = p_ref.x() + standoff * std::cos(az) + lead_x;
            goal.y() = p_ref.y() + standoff * std::sin(az) + lead_y;
            goal.z() = p_ref.z() + goal_height_offset_m_;

            // THE HARD FLOOR, enforced against the person's PRESENT position and not only against
            // the lead anchor. The anchor is a prediction; min_standoff_m is a safety number, and
            // a safety number checked against a prediction is only ever as good as the prediction.
            // Both points are checked because the lead makes them differ by up to
            // lead_distance_m_, and the order is not arbitrary: the predicted point is enforced
            // FIRST and the present one LAST, so when the two disagree the present position is
            // the one that ends up satisfied.
            // Enforced HORIZONTALLY, which is stricter than a 3-D norm: with D6's matched-height
            // trail the vertical component is small, so clearing 2.5 m in the ground plane
            // guarantees at least 2.5 m in space.
            //
            // Z last, because the floor only moves the goal in the ground plane. Outside
            // ROG-Map's (virtual_ground_height, virtual_ceil_height) = (-1.0, 3.5) the goal is in
            // virtual occupancy and comes straight back DROPPED, so this clamp is the difference
            // between a follow that works and one that silently never starts.
            if (!ApplyHardGuards(goal, p_ref, p_now, az)) { return false; }

            // Goal yaw points from the viewpoint at the person. Under yaw_mode 3 (YAW_TO_TARGET)
            // SUPER derives heading from /planning/yaw_target instead and this is mostly unused,
            // but it still has to be right: it is what yaw_mode 2 (YAW_TO_GOAL) would fly, and
            // followGoalCallback turns any NaN quaternion component into a NaN yaw
            // (fsm_ros2.hpp:297-301).
            const double dx = p_ref.x() - goal.x();
            const double dy = p_ref.y() - goal.y();
            if (std::hypot(dx, dy) >= kLosHorizontalFloorM) {
                goal_yaw = std::atan2(dy, dx);
            }  // else: person effectively overhead -- keep the previous yaw rather than snap.

            return IsFinite(goal) && std::isfinite(goal_yaw);
        }

        // Push `goal` radially away from `p` until it is at least min_standoff_m_ clear in the
        // ground plane. Returns false only when the result is not finite, i.e. the geometry was
        // beyond rescue and no goal should be published at all.
        bool EnforceStandoffFloor(Eigen::Vector3d &goal, const Eigen::Vector3d &p, double az) const {
            double vx = goal.x() - p.x();
            double vy = goal.y() - p.y();
            double r = std::hypot(vx, vy);
            if (r < kLosHorizontalFloorM) {
                // The goal landed on top of the person, so there is no direction to normalise.
                // Fall back to the commanded azimuth, which is always a valid direction.
                vx = std::cos(az);
                vy = std::sin(az);
                r = 1.0;
            }
            if (r < min_standoff_m_) {
                const double k = min_standoff_m_ / r;
                goal.x() = p.x() + vx * k;
                goal.y() = p.y() + vy * k;
            }
            return std::isfinite(goal.x()) && std::isfinite(goal.y());
        }

        // The safety projection every goal_mode ends with, in ONE place so the two arms cannot
        // drift: the floor against the predicted point, then against the present one (present
        // wins), then the Z clamp last. The reasons are at the call in
        // ComputeStandoffGoalGeometric. A solver's penalty terms shape its answer so that this
        // rarely has to move it; they never replace it.
        bool ApplyHardGuards(Eigen::Vector3d &goal, const Eigen::Vector3d &p_ref,
                             const Eigen::Vector3d &p_now, double az) const {
            if (!EnforceStandoffFloor(goal, p_ref, az)) { return false; }
            if (!EnforceStandoffFloor(goal, p_now, az)) { return false; }
            goal.z() = std::clamp(goal.z(), goal_z_min_m_, goal_z_max_m_);
            return true;
        }

        // A goal can satisfy min_standoff_m at BOTH ends and still be reached only by flying
        // through the person. That is not hypothetical: with the person walking head-on at the
        // drone, "behind the person" is on the far side, the 45 deg near-side bias reduces the
        // pass-by without removing it, and the commanded transit clears the person by about 1.6 m
        // -- inside our own 2.5 m floor and barely outside command_mux's 1.5 m veto. SUPER will
        // not save us here: ROG-Map is a static-world grid and a walking person is either filtered
        // as a dynamic outlier or leaves occupancy trails, which is exactly why plan/dtp-imp/04
        // SS4 item 1 says NOT to inject them and to keep an independent layer instead.
        //
        // The response is to WITHHOLD, not to route around. Rotating past the 45 deg bias to fly
        // an arc around a person is a maneuver plan/dtp-imp/04 SS1 explicitly leaves out of v1
        // ("v1 validates behind-standoff only"), and doing it here would put the drone into an
        // unvalidated flight mode at the exact moment a human is closest. Holding hands the moment
        // to command_mux, which owns the proximity veto and the closing-speed limit (SS4 items
        // 2 and 3). The condition self-clears: a person walking at the drone walks past it, after
        // which the trail geometry is ordinary again.
        // MEASURED ON HARDWARE 2026-09-08, and it blocked the first closed loop: this guard
        // withheld every goal with "transit to the standoff would pass inside the standoff floor"
        // while follow_planner was otherwise computing correct standoff goals. Two separate
        // defects, and the second is the interesting one.
        //
        // 1. THE START POINT LATCHES IT. t was clamped to [0,1], so t=0 is the drone's own
        //    position. Whenever the drone is ALREADY within transit_clearance_m_ of the person --
        //    which is the normal state of a follower, and guaranteed at engage -- the closest
        //    approach is the drone's current distance and the guard returns true forever. The
        //    person being near us NOW is not something a new goal causes, and withholding the
        //    goal cannot undo it.
        //
        // 2. IT TESTS A STRAIGHT LINE THE DRONE WILL NEVER FLY. SUPER is a planner: it routes
        //    around obstacles, and ROG-Map keeps the person occupied (they are never purged --
        //    see plan/dtp-imp/21 section 4), so the person is avoided from raw returns whether or
        //    not we gate on them here. Meanwhile command_mux's proximity veto already tests the
        //    ACTUAL commanded path against every detected person, at 50 Hz, with better
        //    information than our straight-line approximation. This guard was therefore
        //    duplicating a real veto with a worse model -- and, because a standoff point sits
        //    BEHIND the person, the straight line to it passes near them almost by construction.
        //    Gating on that makes following geometrically impossible from in front.
        //
        // What remains here is the part we genuinely own and command_mux cannot see: whether the
        // GOAL ITSELF, and the final approach into it, sits inside the keep-out. The lead-in is
        // left to SUPER and to the runtime veto.
        bool TransitCrossesKeepOut(const Eigen::Vector3d &goal) const {
            const double ax = cur_position_.x(), ay = cur_position_.y();
            const double bx = goal.x(), by = goal.y();
            const double px = target_position_.x(), py = target_position_.y();

            const double dx = bx - ax, dy = by - ay;
            const double len2 = dx * dx + dy * dy;
            if (len2 <= 1e-12) {
                return false;   // goal is where we already are; nothing is being commanded
            }
            const double len = std::sqrt(len2);

            // Skip the lead-in: the first transit_clearance_m_ of travel is the region our
            // current position dominates, and is SUPER's to route. Never skip the whole segment.
            const double t_lo = std::min(transit_clearance_m_ / len, 0.9);
            double t = std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, t_lo, 1.0);

            const double cx = ax + t * dx, cy = ay + t * dy;
            return std::hypot(px - cx, py - cy) < transit_clearance_m_;
        }

        // The person is outside the body-fixed camera's usable cone. This is a loss condition and
        // not merely a nuisance because D14 defers the gimbal: with nothing stabilising the
        // camera, a person behind the drone produces no new detections at all, so whatever is
        // still arriving on /dtp/follow_target is target_manager coasting on a forecast.
        //
        // THE OBVIOUS OBJECTION, and why it does not apply: withholding the goal removes the
        // trajectory, and SUPER's yaw comes from the trajectory, so it looks like this could latch
        // -- no goal, therefore no yaw, therefore the person never re-enters frame. It cannot latch
        // at engage, because target_manager only locks on a CONFIRMED detection, so the person is
        // in frame by construction when last_in_fov_time_ is first set. And it cannot latch
        // mid-flight either: the person is walking, so either they re-enter the cone and we resume,
        // or target_manager's own coast timeout expires, /dtp/follow_target stops, and we escalate
        // to "target stale" -- which is the terminating path. Do not "fix" this by publishing a
        // goal anyway; that would assert FOLLOW authority on a position no camera has confirmed
        // for a second, which is the R-F12 low-confidence case (plan/dtp-imp/04 SS6).
        bool TargetBehind(double now) {
            const double bearing = std::atan2(target_ref_position_.y() - cur_position_.y(),
                                              target_ref_position_.x() - cur_position_.x());
            const double err = std::fabs(WrapPi(bearing - cur_yaw_));
            if (err <= camera_fov_half_deg_ * M_PI / 180.0) {
                last_in_fov_time_ = now;
                return false;
            }
            return (now - last_in_fov_time_) > target_behind_timeout_s_;
        }

        void ExpirePendingStatus(double now) {
            if (!has_pending_ || (now - pending_publish_time_) <= goal_status_timeout_s_) { return; }
            has_pending_ = false;
            status_link_ok_ = false;

            // A status that never arrives is AMBIGUOUS in a way an explicit DROPPED is not, and
            // the difference decides whether we are allowed to stop publishing. Three causes
            // produce silence: command_mux is arbitrating another source so our goal never leaves
            // the mux; fsm/follow_goal_en is false (and false is its DEFAULT, fsm/config.hpp:73)
            // so followGoalCallback and the status publisher were never created at all; or the
            // QoS link is incompatible. Only the first is recoverable by continuing to publish --
            // and if silence suspended us the way a DROPPED does, FOLLOW could never win the mux
            // back once it had lost it, which is a deadlock we would have built ourselves. So we
            // keep publishing and leave the counters untouched; what we refuse to do is treat
            // last_adopted_goal_ as knowledge. Being wrong here is bounded: command_mux's
            // human-proximity veto reads /dtp/targets directly and does not depend on us at all.
            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
                                 "no GoalStatus within %.2f s of publishing a follow goal. Either command_mux "
                                 "is arbitrating another source, or fsm/follow_goal_en is false (its default), "
                                 "or the QoS link is broken. Still publishing, but the adopted goal is unknown.",
                                 goal_status_timeout_s_);
        }

        void EscalateReprojection() {
            // Alternate the sign so consecutive probes try opposite sides, growing the magnitude
            // each time, and step the radius outward in parallel. Outward ONLY: moving the
            // viewpoint closer to a person in order to dodge an obstacle would trade an obstacle
            // risk for a human one, which is the wrong direction for every rule in
            // plan/dtp-imp/04 SS4.
            const double mag = std::min(std::fabs(reproject_az_rad_) + kReprojectAzimuthStepRad,
                                        kMaxAzimuthOffsetRad);
            reproject_az_rad_ = (reproject_az_rad_ > 0.0) ? -mag : mag;
            reproject_radius_m_ = std::min(reproject_radius_m_ + kReprojectRadiusStepM,
                                           std::max(0.0, follow_far_m_ - follow_near_m_));
        }

        void RelaxReprojection() {
            reproject_az_rad_ *= kReprojectDecay;
            if (std::fabs(reproject_az_rad_) < 1e-3) { reproject_az_rad_ = 0.0; }
            reproject_radius_m_ = std::max(0.0, reproject_radius_m_ - kReprojectRadiusStepM);
        }

        void Suspend(const char *why) {
            if (suspended_) { return; }
            suspended_ = true;
            suspend_target_position_ = target_position_;
            RCLCPP_ERROR(nh_->get_logger(),
                         "%s after %d snaps / %d drops -- going SILENT on %s so command_mux falls "
                         "through to NAV/hover. Will re-arm once the target moves %.2f m.",
                         why, consecutive_snaps_, consecutive_drops_,
                         follow_goal_topic_.c_str(), suspend_reset_move_m_);
        }

        // Deliberately does NOT clear suspended_. Hold() calls this on every tick, including the
        // ticks that are silent *because* we are suspended, so clearing the flag here would
        // un-suspend us one tick after suspending and the drop-escalation would never actually
        // take effect. Suspension is cleared only where "start over" genuinely applies: a new
        // lock, a target that has moved suspend_reset_move_m_, or an outright target loss.
        void ResetServoState() {
            has_published_goal_ = false;
            has_pending_ = false;
            consecutive_snaps_ = 0;
            consecutive_drops_ = 0;
            reproject_az_rad_ = 0.0;
            reproject_radius_m_ = 0.0;
            nmpc_prev_valid_ = false;
        }

        // Stop publishing a goal, and say so ONCE per reason rather than 15 times a second. This
        // is a hold of our OUTPUT, not a hover command: this node has no authority to command a
        // hover, and plan/dtp-imp/04 SS6 (A15) is explicit that a frozen setpoint is the wrong
        // answer to target loss -- the drone may be directly over the person it just lost.
        // command_mux owns what happens next, and our silence is how it finds out.
        void Hold(const char *reason) {
            if (hold_reason_ != reason) {
                RCLCPP_WARN(nh_->get_logger(),
                            "withholding %s: %s. command_mux should fall through to NAV/hover.",
                            follow_goal_topic_.c_str(), reason);
                hold_reason_ = reason;
            }
            ResetServoState();
        }

        void PublishGoal(const Eigen::Vector3d &goal, double yaw, double now) {
            geometry_msgs::msg::PoseStamped msg;
            // OUR clock, not the target's -- the opposite of PublishYawTarget, deliberately. This
            // stamp is what command_mux's staleness watchdog reads, and what it must measure is
            // whether follow_planner is alive and deciding. The age of the PERCEPTION is already
            // gated upstream by follow_target_timeout_s, so a goal published here is never built
            // on data older than that.
            msg.header.stamp = nh_->get_clock()->now();
            msg.header.frame_id = world_frame_id_;
            msg.pose.position.x = goal.x();
            msg.pose.position.y = goal.y();
            msg.pose.position.z = goal.z();
            msg.pose.orientation.w = std::cos(yaw * 0.5);
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = std::sin(yaw * 0.5);
            follow_goal_pub_->publish(msg);

            published_goal_ = goal;
            published_yaw_ = yaw;
            has_published_goal_ = true;

            // One goal in flight per tick, recorded with its publish time: that pair is the only
            // correlation handle GoalStatus leaves us.
            pending_goal_ = goal;
            pending_publish_time_ = now;
            has_pending_ = true;
        }

        void PublishYawTarget() {
            geometry_msgs::msg::PointStamped msg;
            // The TARGET's stamp, not ours. SUPER hard-gates this topic at kYawTargetTimeoutSec =
            // 0.5 s (super_planner.h:102) and silently falls back to velocity-aligned yaw beyond
            // it. That gate is measuring how stale the PERCEPTION is, which is the right thing to
            // measure, so restamping our republish with "now" would defeat it completely -- we
            // would be certifying arbitrarily old data as fresh. The comparison SUPER makes is
            // (getSimTime() - stamp), so follow_planner and fsm_node MUST agree on use_sim_time or
            // the gate fails permanently and invisibly; both nodes log their setting at start-up
            // so a mismatch is a two-line grep instead of an afternoon.
            msg.header.stamp = target_stamp_;
            msg.header.frame_id = world_frame_id_;
            // The PERSON's position -- not the standoff goal, and not the lead anchor. SUPER
            // computes yaw = atan2(target - trajectory_END_position) (super_planner.cpp:800-810),
            // so handing it the lead point would aim the camera lead_distance_m_ ahead of the
            // person, which is exactly the framing error YAW_TO_TARGET exists to remove.
            msg.point.x = target_ref_position_.x();
            msg.point.y = target_ref_position_.y();
            msg.point.z = target_ref_position_.z();
            yaw_target_pub_->publish(msg);
            last_yaw_point_ = target_ref_position_;
            has_last_yaw_point_ = true;
        }

        // M6.5 and the yaw hold only: a PREDICTED or HELD point, stamped now. See the
        // loss_conditioning_ and lost_yaw_hold_s_ comments for why each may bypass the
        // perception-stamp rule, and what bounds it.
        void PublishYawTargetAt(const Eigen::Vector3d &p, double now) {
            (void) now;
            if (!IsFinite(p)) { PublishYawTarget(); return; }
            geometry_msgs::msg::PointStamped msg;
            msg.header.stamp = nh_->get_clock()->now();
            msg.header.frame_id = world_frame_id_;
            msg.point.x = p.x();
            msg.point.y = p.y();
            msg.point.z = p.z();
            yaw_target_pub_->publish(msg);
        }

        // See lost_yaw_hold_s_. Latches on the first call of an episode, then republishes the same
        // point until the window runs out. Returns false when it published nothing (off, no
        // odometry, or the window has run out).
        bool YawHold(double now, const char *why) {
            if (lost_yaw_hold_s_ <= 0.0 || !had_odom_ || yaw_hold_expired_) { return false; }
            if (!yaw_hold_latched_) {
                const Eigen::Vector3d aim = has_last_yaw_point_ ? last_yaw_point_ : target_ref_position_;
                const double dx = aim.x() - cur_position_.x();
                const double dy = aim.y() - cur_position_.y();
                const bool toward = IsFinite(aim) && std::hypot(dx, dy) >= kLostYawHoldMinRangeM;
                const double yaw = toward ? std::atan2(dy, dx) : cur_yaw_;
                yaw_hold_point_ = cur_position_ + kLostYawHoldDistanceM * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
                yaw_hold_start_ = now;
                yaw_hold_latched_ = true;
                RCLCPP_INFO(nh_->get_logger(),
                            "target %u %s: holding yaw at %.1f deg (%s; aircraft heading %.1f deg) for up to %.1f s",
                            target_track_id_, why, yaw * 180.0 / M_PI,
                            toward ? "toward where the camera was last aimed" : "the aircraft's own heading",
                            cur_yaw_ * 180.0 / M_PI, lost_yaw_hold_s_);
            }
            if (now - yaw_hold_start_ > lost_yaw_hold_s_) {
                RCLCPP_INFO(nh_->get_logger(), "yaw hold ended: %.1f s since it latched; SUPER's own yaw from here",
                            lost_yaw_hold_s_);
                yaw_hold_expired_ = true;
                return false;
            }
            PublishYawTargetAt(yaw_hold_point_, now);   // deliberately does not move last_yaw_point_
            return true;
        }

        void ReleaseYawHold(double now, const char *why) {
            if (yaw_hold_latched_) {
                RCLCPP_INFO(nh_->get_logger(), "yaw hold released after %.1f s: %s (track %u)",
                            now - yaw_hold_start_, why, target_track_id_);
            }
            yaw_hold_latched_ = false;
            yaw_hold_expired_ = false;
        }

        bool CoastYawHoldDue(double now) const {
            return coast_yaw_hold_ && lost_yaw_hold_s_ > 0.0 && target_missed_frames_ > 0 &&
                   (now - target_stamp_.seconds()) >= kCoastYawHoldAgeS;
        }

        // A conditioned (M6.5) yaw target: published fresh, and remembered as where the camera
        // was last aimed so that a hold after it keeps looking there.
        void AimYawAt(const Eigen::Vector3d &p, double now) {
            if (IsFinite(p)) {
                last_yaw_point_ = p;
                has_last_yaw_point_ = true;
            }
            PublishYawTargetAt(p, now);
        }

        void NoteConditioned(const char *what) {
            if (conditioned_mode_ != what) {
                conditioned_mode_ = what;
                RCLCPP_INFO(nh_->get_logger(), "loss conditioning: %s (track %u, coast depth %u)",
                            what, target_track_id_, target_missed_frames_);
            }
        }

        // Refuse to run on a configuration that cannot produce a safe goal, rather than flying a
        // quietly wrong one. Note what is NOT here: plan/dtp-imp/07 Step 2.1.1 asked for a
        // launch-time assertion that SUPER's click_height <= -5 (risk R-F17). That requirement is
        // OBSOLETE -- it was written before setFollowGoal existed, when the plan was to reuse the
        // click path, and setFollowGoal deliberately omits the click_height override
        // (fsm.cpp:226-227). click_height 1.5 is now correct for the /goal_pose click path that
        // stays enabled for bench use. What replaced R-F17 as the config error that actually
        // breaks following -- fsm/follow_goal_en defaulting to false -- lives in SUPER's process,
        // not ours, so it is detected at runtime instead: see ExpirePendingStatus.
        // The solver's weights, by parameter name. One table so declare and get cannot drift.
        std::vector<std::pair<std::string, double *>> NmpcWeightParams() {
            return {{"nmpc_w_standoff", &nmpc_w_.standoff}, {"nmpc_w_bearing", &nmpc_w_.bearing},
                    {"nmpc_w_alt", &nmpc_w_.alt},
                    {"nmpc_w_floor", &nmpc_w_.floor},       {"nmpc_w_zlim", &nmpc_w_.zlim},
                    {"nmpc_w_fence", &nmpc_w_.fence},       {"nmpc_w_prev", &nmpc_w_.prev},
                    {"nmpc_w_prev_decay", &nmpc_w_.prev_decay}, {"nmpc_w_pub", &nmpc_w_.pub},
                    {"nmpc_w_vlim", &nmpc_w_.vlim},         {"nmpc_w_alim", &nmpc_w_.alim},
                    {"nmpc_w_acc", &nmpc_w_.acc},           {"nmpc_w_jerk", &nmpc_w_.jerk},
                    {"nmpc_w_vis", &nmpc_w_.vis},           {"nmpc_w_clear", &nmpc_w_.clear},
                    {"nmpc_w_vrel", &nmpc_w_.vrel}};
        }

        void ValidateParameters() {
            if (goal_mode_ != "geometric" && goal_mode_ != "nmpc") {
                throw std::runtime_error("goal_mode must be 'geometric' or 'nmpc', not '" +
                                         goal_mode_ + "'");
            }
            if (goal_publish_rate_hz_ > 0.0) {
                nmpc_dt_s_ = 1.0 / goal_publish_rate_hz_;
                nmpc_stages_ = static_cast<int>(std::lround(nmpc_horizon_s_ * goal_publish_rate_hz_));
            }
            if (!(nmpc_horizon_s_ > 0.0) || nmpc_stages_ < 4 || nmpc_thin_stages_ < 4 ||
                nmpc_thin_stages_ > nmpc_stages_) {
                throw std::runtime_error("nmpc_horizon_s must give >= 4 stages at "
                                         "goal_publish_rate_hz, and 4 <= nmpc_thin_stages <= that");
            }
            if (nmpc_publish_stage_ < 1 || nmpc_publish_stage_ > nmpc_thin_stages_ ||
                (nmpc_publish_stage_ > 1 && !nmpc_carrot_experiment_)) {
                throw std::runtime_error("nmpc_publish_stage must be 1 (the standoff contract); a "
                                         "later stage needs nmpc_carrot_experiment and is a bench arm");
            }
            if (!(nmpc_time_budget_ms_ >= 0.0) || nmpc_time_budget_ms_ * goal_publish_rate_hz_ >= 1000.0) {
                throw std::runtime_error("nmpc_time_budget_ms must be under one goal period");
            }
            if (nmpc_max_iters_ < 1 || nmpc_coast_full_n_ < 1 || nmpc_fallback_persist_n_ < 1 ||
                !(nmpc_lim_.goal_vel_max > 0.0) || !(nmpc_lim_.goal_acc_max > 0.0) ||
                !(nmpc_lim_.smooth_mu > 0.0) || !(nmpc_sanity_jump_m_ >= 0.0) ||
                !(nmpc_band_margin_m_ >= 0.0) || 2.0 * nmpc_band_margin_m_ > follow_far_m_ - follow_near_m_ ||
                !(nmpc_vis_timeout_s_ >= 0.0) || !(nmpc_vis_fade_s_ > 0.0)) {
                throw std::runtime_error("an nmpc_* limit or count is not positive");
            }
            for (const auto &w : NmpcWeightParams()) {
                if (!(*w.second >= 0.0)) { throw std::runtime_error(w.first + " must be >= 0"); }
            }
            if (goal_mode_ == "nmpc" && stock_baseline_) {
                RCLCPP_WARN(nh_->get_logger(), "goal_mode nmpc with stock_baseline: the stock "
                            "baseline wins, the goal is geometric. stage_a_preflight FAILs this.");
            }
            if (follow_far_m_ < follow_near_m_) {
                throw std::runtime_error("follow_far_m must be >= follow_near_m");
            }
            if (min_standoff_m_ > follow_near_m_) {
                throw std::runtime_error("min_standoff_m must be <= follow_near_m: the hard floor "
                                         "cannot sit outside the band the band logic may command");
            }
            // command_mux's proximity_veto_radius_m is 1.5 m (command_mux_node.py:50). A standoff
            // floor at or under it would mean follow_planner routinely commanding goals that the
            // independent safety veto exists to reject -- the two layers fighting instead of
            // nesting, which plan/dtp-imp/01 SS5 explicitly warns against.
            if (min_standoff_m_ < 1.5) {
                throw std::runtime_error("min_standoff_m below command_mux's 1.5 m proximity veto "
                                         "radius: follow_planner would be commanding goals the "
                                         "safety veto exists to reject");
            }
            // At or above min_standoff_m the transit guard trips on the commanded goal itself --
            // the goal sits at exactly that distance -- so follow_planner would hold forever,
            // publish nothing, and look from the outside exactly like a node that had died.
            if (transit_clearance_m_ >= min_standoff_m_) {
                throw std::runtime_error("transit_clearance_m must be < min_standoff_m, or the "
                                         "transit guard trips on its own goal on every cycle");
            }
            if (goal_publish_rate_hz_ <= 0.0) {
                throw std::runtime_error("goal_publish_rate_hz must be > 0");
            }
            // NaN would compare false against the window and hold the heading forever.
            if (!(lost_yaw_hold_s_ >= 0.0) || !std::isfinite(lost_yaw_hold_s_)) {
                throw std::runtime_error("lost_yaw_hold_s must be a finite number >= 0 (0 = off)");
            }
            if (goal_z_max_m_ <= goal_z_min_m_) {
                throw std::runtime_error("goal_z_max_m must be > goal_z_min_m");
            }
            // ROG-Map's virtual ground/ceiling, verified in follow_test_ros2.yaml. Outside it
            // every goal returns DROPPED and following never starts, with nothing in any log
            // saying why.
            if (goal_z_min_m_ <= -1.0 || goal_z_max_m_ >= 3.5) {
                RCLCPP_WARN(nh_->get_logger(),
                            "goal Z clamp [%.2f, %.2f] reaches ROG-Map's virtual ground/ceiling "
                            "(-1.0, 3.5); goals at the edge will come back DROPPED.",
                            goal_z_min_m_, goal_z_max_m_);
            }
            if (goal_deadband_m_ <= 0.0) {
                RCLCPP_WARN(nh_->get_logger(),
                            "goal_deadband_m %.3f disables the deadband; SUPER will see a new goal on "
                            "every tick because gi_.new_goal is set unconditionally (fsm.cpp:239).",
                            goal_deadband_m_);
            }
            if (goal_status_timeout_s_ < 2.0 / goal_publish_rate_hz_) {
                RCLCPP_WARN(nh_->get_logger(),
                            "goal_status_timeout_s %.3f is under two publish periods; expect spurious "
                            "'no GoalStatus' warnings under normal transport jitter.",
                            goal_status_timeout_s_);
            }
        }

    public:
        FollowPlanner() = default;

        explicit FollowPlanner(const rclcpp::Node::SharedPtr &nh) {
            nh_ = nh;
            target_stamp_ = nh_->get_clock()->now();

            nh_->declare_parameter<double>("follow_near_m", follow_near_m_);
            nh_->declare_parameter<double>("follow_far_m", follow_far_m_);
            nh_->declare_parameter<double>("min_standoff_m", min_standoff_m_);
            nh_->declare_parameter<double>("goal_publish_rate_hz", goal_publish_rate_hz_);
            nh_->declare_parameter<double>("lead_distance_m", lead_distance_m_);
            nh_->declare_parameter<double>("goal_deadband_m", goal_deadband_m_);
            nh_->declare_parameter<double>("snap_accept_m", snap_accept_m_);
            nh_->declare_parameter<int>("snap_persist_n", snap_persist_n_);
            nh_->declare_parameter<int>("drop_persist_n", drop_persist_n_);
            nh_->declare_parameter<double>("goal_status_timeout_s", goal_status_timeout_s_);
            nh_->declare_parameter<double>("follow_target_timeout_s", follow_target_timeout_s_);
            nh_->declare_parameter<double>("odom_timeout_s", odom_timeout_s_);
            nh_->declare_parameter<double>("goal_z_min_m", goal_z_min_m_);
            nh_->declare_parameter<double>("goal_z_max_m", goal_z_max_m_);
            nh_->declare_parameter<double>("goal_height_offset_m", goal_height_offset_m_);
            nh_->declare_parameter<double>("near_side_bias_max_deg", near_side_bias_max_deg_);
            nh_->declare_parameter<double>("camera_fov_half_deg", camera_fov_half_deg_);
            nh_->declare_parameter<double>("target_behind_timeout_s", target_behind_timeout_s_);
            nh_->declare_parameter<double>("transit_clearance_m", transit_clearance_m_);
            nh_->declare_parameter<double>("suspend_reset_move_m", suspend_reset_move_m_);
            nh_->declare_parameter<std::string>("world_frame_id", world_frame_id_);
            nh_->declare_parameter<std::string>("follow_target_topic", follow_target_topic_);
            nh_->declare_parameter<std::string>("odom_topic", odom_topic_);
            nh_->declare_parameter<std::string>("goal_status_topic", goal_status_topic_);
            nh_->declare_parameter<std::string>("follow_goal_topic", follow_goal_topic_);
            nh_->declare_parameter<std::string>("yaw_target_topic", yaw_target_topic_);
            nh_->declare_parameter<bool>("loss_conditioning", loss_conditioning_);
            nh_->declare_parameter<std::string>("loss_event_topic", loss_event_topic_);
            nh_->declare_parameter<double>("loss_event_timeout_s", loss_event_timeout_s_);
            nh_->declare_parameter<bool>("stock_baseline", stock_baseline_);
            nh_->declare_parameter<double>("lost_yaw_hold_s", lost_yaw_hold_s_);
            nh_->declare_parameter<bool>("coast_yaw_hold", coast_yaw_hold_);
            nh_->declare_parameter<std::string>("goal_mode", goal_mode_);
            nh_->declare_parameter<double>("nmpc_horizon_s", nmpc_horizon_s_);
            nh_->declare_parameter<int>("nmpc_thin_stages", nmpc_thin_stages_);
            nh_->declare_parameter<bool>("nmpc_coast_shrink", nmpc_coast_shrink_);
            nh_->declare_parameter<int>("nmpc_coast_full_n", nmpc_coast_full_n_);
            nh_->declare_parameter<int>("nmpc_publish_stage", nmpc_publish_stage_);
            nh_->declare_parameter<bool>("nmpc_carrot_experiment", nmpc_carrot_experiment_);
            nh_->declare_parameter<int>("nmpc_max_iters", nmpc_max_iters_);
            nh_->declare_parameter<double>("nmpc_time_budget_ms", nmpc_time_budget_ms_);
            nh_->declare_parameter<double>("nmpc_time_panic_ms", nmpc_time_panic_ms_);
            nh_->declare_parameter<int>("nmpc_fallback_persist_n", nmpc_fallback_persist_n_);
            nh_->declare_parameter<int>("nmpc_cancel_persist_n", nmpc_cancel_persist_n_);
            nh_->declare_parameter<double>("nmpc_sanity_jump_m", nmpc_sanity_jump_m_);
            nh_->declare_parameter<double>("nmpc_band_margin_m", nmpc_band_margin_m_);
            nh_->declare_parameter<std::string>("visibility_topic", visibility_topic_);
            nh_->declare_parameter<double>("nmpc_vis_timeout_s", nmpc_vis_timeout_s_);
            nh_->declare_parameter<double>("nmpc_vis_fade_s", nmpc_vis_fade_s_);
            nh_->declare_parameter<double>("nmpc_reseed_jump_m", nmpc_reseed_jump_m_);
            nh_->declare_parameter<double>("nmpc_warm_max_age_s", nmpc_warm_max_age_s_);
            nh_->declare_parameter<double>("nmpc_goal_vel_max_mps", nmpc_lim_.goal_vel_max);
            nh_->declare_parameter<double>("nmpc_goal_acc_max_mps2", nmpc_lim_.goal_acc_max);
            nh_->declare_parameter<double>("nmpc_smooth_mu", nmpc_lim_.smooth_mu);
            for (const auto &w : NmpcWeightParams()) { nh_->declare_parameter<double>(w.first, *w.second); }
            nh_->declare_parameter<bool>("nmpc_fence_enable", nmpc_fence_.enable);
            nh_->declare_parameter<double>("nmpc_fence_x_min_m", nmpc_fence_.x_min);
            nh_->declare_parameter<double>("nmpc_fence_x_max_m", nmpc_fence_.x_max);
            nh_->declare_parameter<double>("nmpc_fence_y_min_m", nmpc_fence_.y_min);
            nh_->declare_parameter<double>("nmpc_fence_y_max_m", nmpc_fence_.y_max);
            nh_->declare_parameter<double>("nmpc_fence_yaw_deg", 0.0);
            nh_->declare_parameter<double>("nmpc_fence_margin_m", nmpc_fence_.margin);

            nh_->get_parameter("follow_near_m", follow_near_m_);
            nh_->get_parameter("follow_far_m", follow_far_m_);
            nh_->get_parameter("min_standoff_m", min_standoff_m_);
            nh_->get_parameter("goal_publish_rate_hz", goal_publish_rate_hz_);
            nh_->get_parameter("lead_distance_m", lead_distance_m_);
            nh_->get_parameter("goal_deadband_m", goal_deadband_m_);
            nh_->get_parameter("snap_accept_m", snap_accept_m_);
            nh_->get_parameter("snap_persist_n", snap_persist_n_);
            nh_->get_parameter("drop_persist_n", drop_persist_n_);
            nh_->get_parameter("goal_status_timeout_s", goal_status_timeout_s_);
            nh_->get_parameter("follow_target_timeout_s", follow_target_timeout_s_);
            nh_->get_parameter("odom_timeout_s", odom_timeout_s_);
            nh_->get_parameter("goal_z_min_m", goal_z_min_m_);
            nh_->get_parameter("goal_z_max_m", goal_z_max_m_);
            nh_->get_parameter("goal_height_offset_m", goal_height_offset_m_);
            nh_->get_parameter("near_side_bias_max_deg", near_side_bias_max_deg_);
            nh_->get_parameter("camera_fov_half_deg", camera_fov_half_deg_);
            nh_->get_parameter("target_behind_timeout_s", target_behind_timeout_s_);
            nh_->get_parameter("transit_clearance_m", transit_clearance_m_);
            nh_->get_parameter("suspend_reset_move_m", suspend_reset_move_m_);
            nh_->get_parameter("world_frame_id", world_frame_id_);
            nh_->get_parameter("follow_target_topic", follow_target_topic_);
            nh_->get_parameter("odom_topic", odom_topic_);
            nh_->get_parameter("goal_status_topic", goal_status_topic_);
            nh_->get_parameter("follow_goal_topic", follow_goal_topic_);
            nh_->get_parameter("yaw_target_topic", yaw_target_topic_);
            nh_->get_parameter("loss_conditioning", loss_conditioning_);
            nh_->get_parameter("loss_event_topic", loss_event_topic_);
            nh_->get_parameter("loss_event_timeout_s", loss_event_timeout_s_);
            nh_->get_parameter("stock_baseline", stock_baseline_);
            nh_->get_parameter("lost_yaw_hold_s", lost_yaw_hold_s_);
            nh_->get_parameter("coast_yaw_hold", coast_yaw_hold_);
            nh_->get_parameter("goal_mode", goal_mode_);
            nh_->get_parameter("nmpc_horizon_s", nmpc_horizon_s_);
            nh_->get_parameter("nmpc_thin_stages", nmpc_thin_stages_);
            nh_->get_parameter("nmpc_coast_shrink", nmpc_coast_shrink_);
            nh_->get_parameter("nmpc_coast_full_n", nmpc_coast_full_n_);
            nh_->get_parameter("nmpc_publish_stage", nmpc_publish_stage_);
            nh_->get_parameter("nmpc_carrot_experiment", nmpc_carrot_experiment_);
            nh_->get_parameter("nmpc_max_iters", nmpc_max_iters_);
            nh_->get_parameter("nmpc_time_budget_ms", nmpc_time_budget_ms_);
            nh_->get_parameter("nmpc_time_panic_ms", nmpc_time_panic_ms_);
            nh_->get_parameter("nmpc_fallback_persist_n", nmpc_fallback_persist_n_);
            nh_->get_parameter("nmpc_cancel_persist_n", nmpc_cancel_persist_n_);
            nh_->get_parameter("nmpc_sanity_jump_m", nmpc_sanity_jump_m_);
            nh_->get_parameter("nmpc_band_margin_m", nmpc_band_margin_m_);
            nh_->get_parameter("visibility_topic", visibility_topic_);
            nh_->get_parameter("nmpc_vis_timeout_s", nmpc_vis_timeout_s_);
            nh_->get_parameter("nmpc_vis_fade_s", nmpc_vis_fade_s_);
            nh_->get_parameter("nmpc_reseed_jump_m", nmpc_reseed_jump_m_);
            nh_->get_parameter("nmpc_warm_max_age_s", nmpc_warm_max_age_s_);
            nh_->get_parameter("nmpc_goal_vel_max_mps", nmpc_lim_.goal_vel_max);
            nh_->get_parameter("nmpc_goal_acc_max_mps2", nmpc_lim_.goal_acc_max);
            nh_->get_parameter("nmpc_smooth_mu", nmpc_lim_.smooth_mu);
            for (const auto &w : NmpcWeightParams()) { nh_->get_parameter(w.first, *w.second); }
            nh_->get_parameter("nmpc_fence_enable", nmpc_fence_.enable);
            nh_->get_parameter("nmpc_fence_x_min_m", nmpc_fence_.x_min);
            nh_->get_parameter("nmpc_fence_x_max_m", nmpc_fence_.x_max);
            nh_->get_parameter("nmpc_fence_y_min_m", nmpc_fence_.y_min);
            nh_->get_parameter("nmpc_fence_y_max_m", nmpc_fence_.y_max);
            nmpc_fence_.yaw = nh_->get_parameter("nmpc_fence_yaw_deg").as_double() * M_PI / 180.0;
            nh_->get_parameter("nmpc_fence_margin_m", nmpc_fence_.margin);

            ValidateParameters();

            // QoS(10) then .keep_last(1): the depth argument is IMMEDIATELY overridden, so the
            // effective depth is 1 -- and that is intended, because for a moving standoff goal a
            // queued stale message is worse than no message. The expression is kept verbatim
            // because it is the profile SUPER's own fsm_ros2.hpp uses and the M0.7 0.7.4 hardware
            // test passed with it. BEST_EFFORT is not optional on any of these: DDS reliability is
            // ASYMMETRIC, and fsm_node publishes /planning/goal_status BEST_EFFORT, so a
            // subscriber left on rclcpp's RELIABLE default receives exactly zero messages with no
            // error anywhere. Measured on hardware 2026-08-14: 2176 msgs to a BEST_EFFORT sub, 0
            // to a RELIABLE one in the same run (CLAUDE.md).
            const rclcpp::QoS qos(rclcpp::QoS(10).best_effort().keep_last(1).durability_volatile());
            rclcpp::SubscriptionOptions so;

            target_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = target_cb_group_;
            follow_target_sub_ = nh_->create_subscription<dtp_msgs::msg::Target>(
                    follow_target_topic_, qos,
                    std::bind(&FollowPlanner::FollowTargetCallback, this, std::placeholders::_1), so);

            odom_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = odom_cb_group_;
            odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
                    odom_topic_, qos,
                    std::bind(&FollowPlanner::OdomCallback, this, std::placeholders::_1), so);

            goal_status_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = goal_status_cb_group_;
            goal_status_sub_ = nh_->create_subscription<dtp_msgs::msg::GoalStatus>(
                    goal_status_topic_, qos,
                    std::bind(&FollowPlanner::GoalStatusCallback, this, std::placeholders::_1), so);

            loss_event_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            so.callback_group = loss_event_cb_group_;
            loss_event_sub_ = nh_->create_subscription<dtp_msgs::msg::LossEvent>(
                    loss_event_topic_, qos,
                    std::bind(&FollowPlanner::LossEventCallback, this, std::placeholders::_1), so);

            // Only a node that will USE the field subscribes to it: fsm_node builds it whether or
            // not anyone listens, but a subscription nobody needs is one more thing on the graph.
            if (goal_mode_ == "nmpc" && (nmpc_w_.vis > 0.0 || nmpc_w_.clear > 0.0)) {
                visibility_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                so.callback_group = visibility_cb_group_;
                visibility_sub_ = nh_->create_subscription<dtp_msgs::msg::VisibilityField>(
                        visibility_topic_, qos,
                        std::bind(&FollowPlanner::VisibilityCallback, this, std::placeholders::_1), so);
            }

            follow_goal_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(follow_goal_topic_, qos);
            yaw_target_pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>(yaw_target_topic_, qos);

            goal_timer_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            goal_timer_ = nh_->create_wall_timer(
                    std::chrono::duration<double>(1.0 / goal_publish_rate_hz_),
                    std::bind(&FollowPlanner::GoalTimerCallback, this), goal_timer_cb_group_);

            RCLCPP_INFO(nh_->get_logger(),
                        "follow_planner up. band %.2f-%.2f m (floor %.2f), lead %.2f m, deadband %.2f m, "
                        "%.1f Hz | goal -> %s | yaw target -> %s | frame '%s' | use_sim_time=%s",
                        follow_near_m_, follow_far_m_, min_standoff_m_, lead_distance_m_,
                        goal_deadband_m_, goal_publish_rate_hz_,
                        follow_goal_topic_.c_str(), yaw_target_topic_.c_str(), world_frame_id_.c_str(),
                        nh_->get_parameter("use_sim_time").as_bool() ? "true" : "false");
            RCLCPP_INFO(nh_->get_logger(),
                        "loss conditioning %s (M6.5 arm; LossEvents from %s, fresh for %.1f s)%s",
                        loss_conditioning_ ? "ON" : "OFF -- carry-forward arm", loss_event_topic_.c_str(),
                        loss_event_timeout_s_, stock_baseline_ ? " | STOCK BASELINE (M6.8): goal behind the "
                        "present position on a CV heading -- comparison arm, not a flight mode" : "");
            if (lost_yaw_hold_s_ > 0.0) {
                RCLCPP_INFO(nh_->get_logger(), "yaw hold ON: %.1f s from the latch, %s, toward where the camera was "
                            "last aimed", lost_yaw_hold_s_,
                            coast_yaw_hold_ ? "from 0.40 s into a coast and at a loss" : "at a loss only (coast_yaw_hold off)");
            } else {
                RCLCPP_INFO(nh_->get_logger(), "yaw hold OFF: SUPER's own yaw once a yaw target lapses");
            }
            if (goal_mode_ == "nmpc") {
                RCLCPP_INFO(nh_->get_logger(),
                            "goal_mode nmpc | horizon %.2f s = %d stages of %.4f s | goal limits %.1f m/s "
                            "%.1f m/s2 | budget %.1f ms, %d iterations | fence mirror %s | visibility %s",
                            nmpc_horizon_s_, nmpc_stages_, nmpc_dt_s_, nmpc_lim_.goal_vel_max,
                            nmpc_lim_.goal_acc_max, nmpc_time_budget_ms_, nmpc_max_iters_,
                            nmpc_fence_.enable ? "ON" : "off",
                            (nmpc_w_.vis > 0.0 || nmpc_w_.clear > 0.0) ? visibility_topic_.c_str()
                                                                      : "off (Stage 1)");
            } else {
                RCLCPP_INFO(nh_->get_logger(), "goal_mode %s", goal_mode_.c_str());
            }
        }
    };

}  // namespace follow_planner

#endif  // FOLLOW_PLANNER_NODE_HPP
