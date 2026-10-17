# Published by SUPER's fsm_node on /planning/goal_status -- a new topic that does not exist in
# upstream SUPER (plan/dtp-imp/07 M2.1 Step 2.1.3; risk R-F4, plan/dtp-imp/11).
#
# Upstream SUPER's setGoalPosiAndYaw() silently does one of three things to an incoming goal and
# never reports which: uses it as-is, SNAPS it to the nearest free inflated cell within a fixed
# radius (getNearestInfCellNot), or DROPS it outright (keeps the previous goal) if the point is
# "deeply occupied" -- with no topic, service, or log line a follower can observe. For a
# continuously-moving standoff goal that is dangerous: follow_planner must know when its
# requested goal didn't take, or it will believe the drone is heading somewhere it isn't.

std_msgs/Header header

uint8 STATUS_ACCEPTED = 0    # goal used exactly as requested
uint8 STATUS_SNAPPED  = 1    # goal moved to the nearest free cell within the snap radius
uint8 STATUS_DROPPED  = 2    # goal rejected outright (deeply occupied); the PREVIOUS goal is still active

uint8 status

# The goal position SUPER actually used: == the requested goal if ACCEPTED, the snapped point
# if SNAPPED, or the still-active previous goal_p if DROPPED.
geometry_msgs/Point goal_p

# The goal position that was REQUESTED, echoed back verbatim.
#
# WHY THIS EXISTS: /planning/follow_goal has more than one publisher -- follow_planner's standoff
# goals, and command_mux's own HOLD and NAV goals -- and fsm answers EVERY message on it with
# exactly one status. Without an echo of the request there is no way for a subscriber to tell
# which of its own goals, if any, a given status is answering. goal_p cannot serve as that key:
# on DROPPED it is the PREVIOUS goal, geometrically unrelated to the request. Correlating on
# goal_p alone let follow_planner consume statuses answering mux goals, count them as rejections
# of its own, and suspend itself mid-follow.
geometry_msgs/Point requested_p
