# M6.1 (docs/m5-m6-plan.md SS5.0) -- the loss oracle's answer to ONE coasting
# /dtp/follow_target. Published by SUPER's fsm_node on /planning/loss_event, from inside the
# planner process so that it queries the SAME ROG-Map the planner avoids obstacles with ("one
# map, two consumers"). Nothing else in the stack builds a map.
#
# It answers every /dtp/follow_target whose missed_frames > 0 (target_manager COASTING). Seen
# frames (missed_frames == 0) get no answer: there is nothing to classify.
#
# The question it answers, Eq. 1 of the paper: WHY did the camera stop seeing the person?
#   OCCLUDED     -- the line of sight from the camera to where CTRV says the person now is
#                   crosses OCCUPIED cells before it gets there (a trunk, a tube, a wall).
#   UNMAPPED     -- it crosses UNKNOWN cells first. Reported separately, CONDITIONED AS LOST.
#   LOST         -- clear line of sight. Includes detector false negatives, by design.
#   OUT_OF_VIEW  -- the point is outside the camera's cone; decided BEFORE any ray is cast.
# The decision is a fraction of rays over the target's bounding cylinder, not one ray to one
# point, thresholded at rho_star (loss_oracle/rho_star in the SUPER config).

std_msgs/Header header             # stamp = the answered target's stamp (its last SEEN frame);
                                   # frame_id = the map frame (lidar_init)
builtin_interfaces/Time eval_stamp # the oracle's clock when it evaluated. reappear_dt* below
                                   # are offsets from THIS instant, not from header.stamp, which
                                   # is a coast old by the time the event exists.
uint32 track_id
uint32 missed_frames               # the coast depth of the target answered, echoed
bool onset                         # first answer of this coast (a seen frame preceded it)

uint8 CLS_OCCLUDED = 0
uint8 CLS_UNMAPPED = 1
uint8 CLS_LOST = 2
uint8 CLS_OUT_OF_VIEW = 3
uint8 cls

# Eq. 1 evidence. rays_cast = 0 for OUT_OF_VIEW.
float32 blocked_fraction           # rays whose first non-free cell was OCCUPIED
float32 unknown_fraction           # rays whose first non-free cell was UNKNOWN / off the map
uint16 rays_cast
geometry_msgs/Point query_p        # the CTRV point tested -- the target's coasted position,
                                   # echoed verbatim (the GoalStatus.requested_p lesson)
geometry_msgs/Point camera_p       # the ray origin: body position + the camera offset
float32 bearing_deg                # bearing of query_p from the camera axis, for the cone test

# The occluder. Valid only for CLS_OCCLUDED.
geometry_msgs/Point occluder_p     # first OCCUPIED cell on the blocked ray nearest the centre
geometry_msgs/Point edge_left      # where the occluder ends, left as seen from the camera
geometry_msgs/Point edge_right
bool edge_left_valid               # false: no free column within loss_oracle/edge_max_m
bool edge_right_valid

# Reappearance belief (M6.4). K CTRV rollouts from query_p; each rollout's first point that is
# outside the occluder's angular interval AND has a free 3-D ray from the camera is its
# re-emergence. Clustered by side. Empty unless CLS_OCCLUDED.
geometry_msgs/Point[] reappear_p   # cluster mean re-emergence point
float32[] reappear_dt              # median time to re-emergence, seconds after eval_stamp
float32[] reappear_dt_min          # earliest in the cluster
float32[] reappear_dt_max          # latest in the cluster
float32[] reappear_sigma_m         # RMS horizontal spread of the cluster about its mean
float32[] reappear_w               # cluster mass, fraction of rollouts
int8[] reappear_side               # +1 = left of the occluder as seen from the camera, -1 = right
float32 still_hidden_w             # mass of rollouts that never re-emerged within the horizon
uint16 rollouts

# Reduction to what SUPER and follow_planner consume (paper Sec. V-E).
# yaw_target_p is one of three things, and which one is recoverable from any bag because they
# are exact copies of the same doubles: == reappear_p[0] -> the dominant cluster's mean;
# == query_p -> no cluster (and no look point); anything else -> the M6.4b look point
# (loss_oracle/hidden_look): the mean position at hidden_look_s of the rollouts that never
# re-emerged, chosen when still_hidden_w >= hidden_look_min_w or there is no cluster. Then
# viewpoint_p is searched about the look point instead -- a clear ray to it is the sidestep
# that looks behind the occluder. reappear_* are never changed by that branch.
geometry_msgs/Point yaw_target_p   # dominant cluster mean | query_p | look point (see above)
bool viewpoint_valid
geometry_msgs/Point viewpoint_p    # standoff candidate on follow_planner's annulus about
                                   # yaw_target_p: free line of sight to it, >= min_standoff
                                   # from the hidden person's position, nearest the drone

# Cost, for Table III and the SS2.4 budgets.
uint32 classify_us                 # cone test + Eq. 1 rays
uint32 eval_us                     # everything, including the belief and the viewpoint
