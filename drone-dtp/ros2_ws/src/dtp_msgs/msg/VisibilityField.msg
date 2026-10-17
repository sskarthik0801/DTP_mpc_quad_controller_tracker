# Stage 2 of follow_planner's goal_mode nmpc (docs/fact-nmpc-follow-goal.md) -- WHERE AROUND THE
# PERSON CAN THE CAMERA SEE THEM FROM? An azimuth-binned answer, computed inside SUPER's fsm_node
# by the loss oracle against the SAME ROG-Map the planner avoids obstacles with ("one map, two
# consumers"), and published on /planning/visibility.
#
# Why it exists: ROG-Map cannot be queried from another process, and subscribing /rog_map/occ
# would cost fsm_node ~8e6 cell reads a second on its publish thread. This is ~300 bytes.
#
# Why it is NOT more fields on LossEvent: LossEvent answers only a COASTING target ("why did we
# lose them?"). A visibility cost is needed while the person is SEEN, to avoid the occlusion
# rather than recover from it. It is published for EVERY locked /dtp/follow_target, seen or
# coasting, and it has its own switch (loss_oracle/visibility_enable) so that it and the M6.5
# loss_conditioning arm stay two independent variables.

std_msgs/Header header             # stamp = the answered target's stamp; frame_id = the map frame
builtin_interfaces/Time eval_stamp # fsm_node's clock when it evaluated
uint32 track_id
uint32 missed_frames               # 0 = the person is SEEN; > 0 = this is about the coast point

geometry_msgs/Point centre_p       # the point the annulus is built about, echoed verbatim
geometry_msgs/Point camera_p       # where the camera was (the height floor below depends on it)
float32 view_z                     # the height the candidate viewpoints were placed at
float32 r_near                     # the annulus: loss_oracle/standoff_near .. standoff_far
float32 r_far

# Bin b is the viewpoint at azimuth az0 + b*daz, measured FROM centre_p OUTWARD (so it is the
# same angle follow_planner calls the trail azimuth), world frame, radians.
float32 az0
float32 daz
uint16 bins

float32[] clear_r                  # how far out from r_near the viewpoint stays out of the
                                   #   INFLATED occupancy, capped at r_far. NaN: not even r_near.
float32[] vis                      # fraction of the target's sample heights with a FREE ray
                                   #   from the viewpoint (at min(clear_r, mid-band)). Eq. 1's
                                   #   rho, from a CANDIDATE viewpoint instead of the camera.
float32[] unk                      # fraction whose first non-free cell was UNKNOWN. The known
                                   #   occluded fraction is 1 - vis - unk. "Cannot know" is not
                                   #   "blocked", and a consumer must not steer away from it.

uint16 heights_used                # sample heights the map can know at this range (the MID360
uint16 heights_dropped             #   looks no lower than 7 deg). 0 used => the field says
                                   #   nothing: treat it as ABSENT, never as "invisible".
uint32 eval_us
