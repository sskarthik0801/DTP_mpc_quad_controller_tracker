# One detected/tracked human, in the FAST-LIO2 world (odom) frame.
#
# Position is full 3-D (a drone needs altitude). The ground-robot reference this is adapted
# from (human_dtp) discards height and works in an (x,z) plane only -- see plan/dtp-imp/14 SS3
# "DROP: XZ-only height discard". We keep the vertical axis here.
#
# *** Z IS UP. x/y are the horizontal plane (REP-103 / ENU). ***
# This line previously said "we keep Y (up, ENU)" and was WRONG. Every consumer does its
# horizontal maths on (x, y) and treats z as altitude -- command_mux's proximity veto and
# descent guard, and follow_planner's standoff geometry. The world frame is FAST-LIO's
# camera_init, which is Z-up. perception_node swizzles the camera->world transform (rows 1
# and 2) precisely because detection_pipeline assumes a Y-up world and this frame is not.
#
# Two fields are new versus the reference repo's per-person output dict (confidence, velocity)
# -- see plan/dtp-imp/03 SS2 "Gaps to fix in the port": the reference has neither on the wire,
# because velocity is re-derived internally and confidence is a YOLO gate then discarded.

std_msgs/Header header

# Stable tracker ID. Used for target lock and re-identification.
#
# NOT ByteTrack, despite plan/dtp-imp/14:71: detection.py:620 calls model.track() with no
# tracker= argument, so ultralytics' default botsort.yaml is in force -- BoT-SORT with
# sparseOptFlow GMC, the configuration risk R-F11 and plan 03:142 warn against. Its CPU cost
# is unmeasured; M1.4.1 measures it before any workaround is chosen.
uint32 track_id

# World-frame (odom) position and velocity.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# CTRV-predicted future path, world frame. The publisher (perception_node / target_manager)
# is responsible for latency-compensating this -- i.e. the first point should represent the
# target's estimated position one closed-loop sense->setpoint latency into the future, not the
# raw "now" position -- see plan/dtp-imp/14 SS2 change C5. predicted_path_dt[i] is the time
# offset in seconds from header.stamp for predicted_path[i]; both arrays are the same length.
geometry_msgs/Point[] predicted_path
float64[] predicted_path_dt

# Track lifecycle, mirroring the reference repo's confirm/coast logic (plan/dtp-imp/03 SS2):
# a track is "confirmed" after min_confirm_frames sightings, and is coasted (position still
# published, from the predictor) for a configurable window after going unseen before being
# dropped from TargetArray entirely.
#
# *** missed_frames IS ALWAYS 0 ON /dtp/targets. *** detection.py builds its detection list
# only from tracks parsed THIS frame and _mark_track_seen zeroes the counter first, so ABSENCE
# from the array is the only loss signal that topic carries. Do not gate re-identification on
# this field there. It IS meaningful on /dtp/follow_target, where target_manager overwrites it
# with the coast depth.
bool track_confirmed
uint32 missed_frames

# Detector confidence [0,1] and range to the drone [m] -- both NEW vs. the reference repo.
float32 confidence
float32 distance_m

# True for AT MOST ONE Target in a TargetArray: the currently locked/followed target
# (decision D13, plan/dtp-imp/12). All other Targets are tracked/logged/keep-out only --
# command_mux's human-proximity veto (plan/dtp-imp/04 SS4) reads ALL of them, not just this one.
bool is_locked_target
