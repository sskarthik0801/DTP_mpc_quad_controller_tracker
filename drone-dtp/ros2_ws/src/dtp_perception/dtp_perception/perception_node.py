"""perception_node -- the detect/track/range/predict brain, wrapped in ROS2.

A thin ROS2 wrapper around `detection_pipeline.HumanXZPredictor.process_frame`, which is ported
UNCHANGED and stays that way (CLAUDE.md SS7). Everything ROS-shaped lives here; the
detect/track/range/predict logic never becomes ROS-aware. Once per processed frame this node:

  1. builds the organized H x W x 3 cloud (in THIS process -- see below),
  2. looks up world <- camera AT THAT FRAME'S OWN STAMP (never "latest" -- risk A8),
  3. calls process_frame once,
  4. does the four jobs process_frame does not do -- height, velocity, confidence, latency
     compensation -- and gates its range against each track's own recent motion,
  5. publishes one dtp_msgs/Target per person on /dtp/targets. EVERY person, EVERY processed
     frame, including frames with nobody in them.

THE CLOUD IS BUILT HERE, NOT RECEIVED OVER DDS  (change to plan/dtp-imp/01 SS2's topic table)
---------------------------------------------------------------------------------------------
1280 x 720 x 3 x float32 is 11.06 MB per message; at the 15 Hz detection rate that is 166 MB/s
into an rclpy subscriber. This repo has already measured what that does: a 2.76 MB colour message
at 30 Hz (83 MB/s) lost ~8% of frames in Python and the fix was to stop sending it (see
image_io.py). The organized cloud is 4x larger per message at twice the failing throughput, and
under BEST_EFFORT an 11 MB sample fragments into thousands of datagrams where losing one loses
the sample. plan/dtp-imp/14 C3, risk A4, plan/dtp-imp/06 M1.4.1 and plan/dtp-imp/07:151 all
already prescribe the same answer: one process.

So `depth_source` is instantiated HERE and `range_image()` is turned into the H x W x 3 array
in the colour callback, at zero serialization cost. `fusion_node` keeps the /dtp/points_fused
contract for RViz, bags and the record-once/replay-many harness (plan/dtp-imp/03 SS5); this node
can consume that topic instead (`cloud_source:=topic`) for REPLAY, which is the only rate at
which sending it is defensible. It is not the flight path and the node says so at start-up.

The deprojection itself lives in `dtp_perception.organized_cloud`, NOT here. It used to be a
private `_unit_rays` in this file and an identical private `_unit_rays` in fusion_node, and the
contract's instruction that the two "must stay identical" had nothing enforcing it: the two copies
build the array RViz sees and the array the drone flies on, so a divergence would show as a
plausible-looking cloud in one and a wrong one in the other. One module, imported by both, is that
instruction made structural.

The consequence is that the message_filters ApproximateTimeSynchronizer is gone from the default
path, replaced by an explicit skew guard -- which is strictly better: `range_image()` returns the
newest depth frame regardless of the colour stamp, and colour (29.98 Hz) and depth (29.96 Hz) are
NOT hardware-locked (`time_sync_en: false`, risk A8), so the synchronizer was never synchronising
the thing that matters. See MAX_DEPTH_COLOUR_SKEW_S.

WHY THE FRAME WAITS FOR ODOM INSTEAD OF TAKING THE LATEST TRANSFORM
-------------------------------------------------------------------
`/lidar_slam/odom` runs at 10.10 Hz (measured, plan/dtp-imp/21) and colour at 29.98 Hz, so the
newest odom sample is on average ~50 ms OLDER than the frame we want to lift, and tf2 correctly
refuses to extrapolate forward. Taking "latest" instead would be wrong by yaw-rate x range --
0.5 m at 3 rad/s and 6 m (risk A8 / R-F5). So frames sit in a short queue and are processed the
moment TF covers their stamp; a frame TF never covers is DROPPED, not guessed. That buys one
odom period of latency, which is leg 3 of the budget behind `latency_compensation_s`.

Extrapolation into the future is therefore the NORMAL case here and not an error condition, which
is why `_lookup` classifies ExtrapolationException as RETRYABLE and LookupException /
ConnectivityException as not: the first means odom is late, the second means the chain is absent
and waiting only burns queue on a frame that will never lift.

*** AND AS OF 2026-09-08 THE CHAIN IS ABSENT. *** FAST-LIO publishes camera_init -> <body>; the
Orbbec driver publishes camera_link -> camera_color_optical_frame; NOTHING publishes
<body> -> camera, so the lookup this node depends on fails on the aircraft right now and every
frame publishes an empty array. That is a missing static transform owned outside this file (see
_drain's one-time warning for what it has to be and where the numbers live), not a defect here --
but nothing in this node can be tested on the aircraft until it exists.

HEIGHT IS SAMPLED HERE, AND IS THEREFORE DUPLICATED CODE  *** READ THIS BEFORE EDITING ***
-------------------------------------------------------------------------------------------
`_extract_xz_from_pointcloud` returns only x and z -- the ground-robot reference discarded height
("XZ-only height-discard", which CLAUDE.md SS7 and plan/dtp-imp/14 say to leave behind). A drone
needs altitude, so `_sample_patch` below re-implements that function's patch geometry EXACTLY and
takes the y median over the identical pixels.

That means the patch geometry now exists in two places with no shared code path and no compiler
that will notice them diverging: `detection_pipeline/detection.py:432-478` (FROZEN) and here. The
failure mode is silent and asymmetric -- change `half_patch_scale_h` or the 0.65 centre fraction
upstream and the POSITION sample moves while the HEIGHT sample stays put, and the only symptom is
"the drone sits a bit high", which reads as a tuning issue and gets tuned instead of fixed.
Mitigations actually in force here: every constant that IS an attribute is read off the predictor
instance rather than restated (`half_patch_scale_w/h`, `max_depth_m`), and the two that are
method-local literals upstream are named constants below citing their line numbers. If
detection_pipeline/ is ever unfrozen, the two-line fix is to have
`_extract_xz_from_pointcloud` return all three medians and let the caller ignore y.

Topics (plan/dtp-imp/01 SS2, whose table still lists /dtp/points_fused as an INPUT here -- that
entry is STALE and superseded by plan/dtp-imp/14 C3; correct it there):
  Subscribes: /camera/color/image_raw/compressed (sensor_msgs/CompressedImage) by default -- see
              dtp_perception/image_io for the 8%-frame-loss measurement that decided that;
              whatever the selected depth source needs (depth_source=camera ->
              /camera/depth/image_raw + both camera_infos + /tf_static; depth_source=lidar ->
              /cloud_registered_body + /lidar_slam/odom); TF (world -> camera, via tf2_ros).
              NOT /dtp/points_fused -- the cloud is built in-process, see above. That topic is
              subscribed ONLY under cloud_source:=topic, which is the replay path.
  Publishes:  /dtp/targets (dtp_msgs/TargetArray) at the detection rate, 15 Hz -- ONE message per
              ACCEPTED colour frame, UNCONDITIONALLY. That includes frames with nobody in them and
              frames that could not be evaluated at all (no camera_info, no range image, skew
              rejected, TF never covered, process_frame raised), which publish an EMPTY array with
              the frame's own stamp. An empty array is a POSITIVE statement that the pipeline is
              alive; a skipped publish is indistinguishable from a dead node. Read by BOTH
              target_manager (to pick/coast the locked target) and command_mux (for the
              independent human-proximity veto -- it must see every person, not just the locked
              one, so a bug in the follow logic cannot defeat it).

              This is the rate command_mux's targets_timeout_s comment says is "not specified
              anywhere in this repo": 15 Hz, and 0.5 s is a fine timeout against it.

              /dtp/detections_2d (std_msgs/String, JSON) -- the same frame's boxes in colour-image
              pixels, for dtp_image_overlay (the laptop RViz picture) only, and only while
              something subscribes. See DETECTIONS_2D_TOPIC.
"""
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException, LookupException,
                     TransformListener)

from dtp_msgs.msg import Target, TargetArray

from dtp_perception import calib_io, cuda_cloud, depth_source, image_io, organized_cloud

# Ported UNCHANGED from the reference repo and imported, never edited (CLAUDE.md SS7). It is not
# a ROS package: it lives at the repository root and reaches this process on PYTHONPATH, which is
# also why an ImportError here is a deployment fault, not a code fault.
from detection_pipeline.detection import HumanXZPredictor

# ---------------------------------------------------------------------------------------------
# Rate
# ---------------------------------------------------------------------------------------------
# Detection rate. NOT the camera's 30 Hz: yolo11s TensorRT measured ~66% GPU when driven at 30 Hz
# alongside the SUPER stack (2026-09-08), and the lever plan/dtp-imp/21 names is the rate, not the
# model -- 15 Hz is roughly a third of the load. 15 rather than 10 because
# follow_planner.goal_publish_rate_hz is 15.0, so at 15 Hz there is exactly one fresh target per
# goal period; at 10 Hz one goal in three is computed from a target the planner already used.
#
# THIS IS A BUDGET, NOT AN ACHIEVEMENT. The concurrent-load run measured power and thermals and
# never measured detection rate (command_mux's DEFAULT_TARGETS_TIMEOUT_S and dtp_params.yaml's
# source_stale_timeout_s both annotate their own timeouts PROVISIONAL for this reason).
# plan/dtp-imp/06 M1.4.1 is the gate that measures it. 5 Hz is the hard floor: below that
# follow_planner's 0.30 s follow_target_timeout_s has less than 1.5x margin and one dropped frame
# breaks it.
#
# THE NOMINAL MARGINS ARE COMFORTABLE AND THE TAIL IS NOT, AND THE TAIL IS WHAT THE CONSUMERS SEE.
# 15 Hz is 66.7 ms, i.e. 7.5x on command_mux's 0.5 s targets_timeout_s and 4.5x on follow_planner's
# 0.30 s follow_target_timeout_s -- but those are ratios against the MEAN. The worst realistic
# INTER-PUBLISH gap is one detection period plus an odom stall (see MAX_PAIR_AGE_S), ~0.21-0.22 s,
# which is 2.3x and ~1.4x. Add the camera's own measured ~8% upstream frame loss (plan/dtp-imp/21:
# p95 inter-arrival 33.6 ms, max 65.5 ms on the compressed arm) and two unlucky events stack toward
# ~0.25 s, i.e. ~1.2x on follow_planner. That is not a break, but it is thin, and it is the number
# that decides whether this design holds. M1.4.1/M1.4.2 must therefore measure the DISTRIBUTION of
# the /dtp/targets inter-publish gap under concurrent load, not just the mean rate: a mean of 15 Hz
# with a 0.3 s tail is a broken follow chain that reports healthy.
DEFAULT_DETECT_RATE_HZ = 15.0
# Decimation slack as a fraction of the target period. A 29.98 Hz stream against a 15 Hz target
# gives inter-frame gaps of 33.4 ms against a 66.7 ms period; without slack the 2nd frame is
# rejected, the 3rd accepted, and the achieved rate drifts to 10 Hz. A quarter-period of slack
# accepts every 2nd frame exactly.
DECIMATE_TOL_FRAC = 0.25
# The TF-wait queue is drained this many times per detection period. Purely a quantisation knob:
# a frame becomes processable the instant an odom sample covers its stamp, and draining only at
# the detection rate would add up to a full period of avoidable latency on top of the odom wait.
# The drain does nothing when the queue is empty.
DRAIN_RATE_MULTIPLIER = 4.0

# ---------------------------------------------------------------------------------------------
# Timing guards
# ---------------------------------------------------------------------------------------------
# Reject a frame whose depth and colour stamps disagree by more than this. 3/4 of a 30 Hz period.
# At 1.4 m/s a person moves 3.5 cm in 25 ms, which is under the depth noise; a 100 ms skew during
# a 3 rad/s yaw is 0.3 rad of geometric disagreement between the YOLO box and the cloud it indexes.
MAX_DEPTH_COLOUR_SKEW_S = 0.025
# How long a frame may sit waiting for odom to cover its stamp before it is dropped blind.
#
# THIS WAS 0.15 s, JUSTIFIED AGAINST THE WRONG NUMBER, AND THE RIGHT ONE IS ALREADY IN THIS REPO.
# The old reasoning was "one odom period is 99 ms worst case (10.10 Hz measured)" -- but 10.10 Hz
# is the MEAN rate, and plan/dtp-imp/11 R-F22 records a direct measurement on this aircraft of
# /lidar_slam/odom inter-message gaps reaching 204 ms (std dev ~30 ms), attributed to FAST-LIO2's
# per-scan compute variance at >70% of a single core. 204 ms > 150 ms, so during an odom stall --
# which happens preferentially when the drone is working hard, i.e. YAWING TO TRACK SOMEONE --
# roughly three consecutive frames were dropped and published blind. The follow chain survives that
# (target_manager coasts, so /dtp/follow_target keeps flowing), but command_mux's proximity veto
# goes EMPTY for that window, which is the fail-open direction, at the moment ego-motion is highest.
#
# 0.25 s costs nothing in the mean: this cap governs only the TAIL. The mean wait is set by the
# odom period (~50 ms, half a period) and does not move, so leg 3 of the latency budget behind
# DEFAULT_LATENCY_COMPENSATION_S is unchanged. It also slightly REDUCES the worst inter-publish gap
# -- a stalled frame now publishes for real when odom arrives at +204 ms instead of blind at
# +217 ms. The cost is queue depth: the pending deque goes from 3 to 5 entries, ~55 MB of pinned
# clouds instead of ~33 MB, against 6.5 GB in use on a box with headroom.
#
# Do NOT raise it further. Past this point the QUEUE DEPTH, not the odom gap, becomes the thing
# bounding the worst inter-publish gap, and the cap stops being a guard and starts being the cause.
MAX_PAIR_AGE_S = 0.25
# *** THE TIMEOUT MUST BE ZERO INSIDE A CALLBACK. ***
# tf2's lookup timeout LOOKS like it waits for the transform. It blocks the calling thread on a
# condition variable and spins NOTHING, so if that thread is the executor thread then nothing is
# delivering /tf while it waits, the transform can never arrive, and the call burns the whole
# timeout before raising "frame does not exist". This repo has hit it twice: see
# depth_source.wait_until_ready's docstring (it silently disabled the person metric in
# scripts/outdoor_depth_test.py, i.e. the entire result of the test) and fusion_node.__init__.
TF_LOOKUP_TIMEOUT = Duration(seconds=0.0)
# Spun before the executor takes the node, so /tf_static (TRANSIENT_LOCAL -- delivered on
# subscription match, which only happens while spinning) is in the buffer before the first frame.
# Without it the first frames fail the lookup and are published as empty arrays for no reason.
TF_WARMUP_S = 1.0

# ---------------------------------------------------------------------------------------------
# The torso patch -- MUST MATCH detection_pipeline/detection.py:432-478 EXACTLY
# ---------------------------------------------------------------------------------------------
# The 0.50/0.65 box fractions are method-local literals upstream (detection.py:443-444) with no
# attribute to read them off, so they are the two constants that genuinely have to be restated
# here. 0.65 of the box height is the torso, deliberately below the head-and-shoulders line.
# Everything else the patch depends on IS an attribute and is read off the predictor instance.
PATCH_CENTRE_W_FRAC = 0.50
PATCH_CENTRE_H_FRAC = 0.65

# Nearest-surface selection inside the torso patch. See _sample_patch for the measurement that
# forced this; briefly, a plain median over the patch flips between the subject and the wall
# behind them whenever the patch straddles the silhouette, which rejected 62 % of frames.
#
# 20th percentile, not the minimum: the minimum is a single sample and therefore exactly the
# speckle the median was there to reject. A percentile keeps the outlier rejection while still
# choosing the NEAR surface rather than the middle of whatever mix of surfaces is in view.
NEAR_SURFACE_PCT = 20.0
# Keep samples within this depth of the near surface. A standing adult is ~0.30 m front-to-back;
# 0.45 m covers a turned torso and an outstretched arm without reaching a wall that is metres
# away, which is the separation this has to resolve.
NEAR_SURFACE_BAND_M = 0.45
# The live tripwire on that duplication. On the FIRST frame of a track _smooth_xz returns the raw
# value (empty history, detection.py:426-428) and _limit_motion_step is skipped (confirmed is
# still False), so on that frame process_frame's position_xz_m is EXACTLY the swizzled lift of our
# own medians. Any drift between the two copies of the patch geometry breaks that identity on the
# first frame of every track, which is the only moment it is checkable at runtime. 1 mm is far
# below any real disagreement and far above float64 round-off through a 4x4.
# Geometry-drift tolerance for the patch tripwire. Was 1e-3 when _sample_patch was required to
# reproduce detection.py's median bit-for-bit. Since 2026-09-09 it selects the nearest surface
# instead, so an edge-straddling patch legitimately disagrees by the foreground-to-background
# separation -- metres, in a room. 1.5 m is above anything surface selection can explain and
# below the several-metre error that moving the patch onto different pixels would produce.
PATCH_GEOMETRY_DRIFT_TOL_M = 1.5

# ---------------------------------------------------------------------------------------------
# Height (world Z) -- the channel nothing upstream filters
# ---------------------------------------------------------------------------------------------
# Per-frame vertical step clamp, the vertical analogue of detection.py's _limit_motion_step
# (max_step_m = 0.35 in the ground plane). A walking human's torso centroid does not rise or fall
# 0.20 m between frames; a patch that half-lands on the floor or a railing does exactly that.
MAX_STEP_Z_M = 0.20

# ---------------------------------------------------------------------------------------------
# Latency compensation (plan/dtp-imp/14 change C5)
# ---------------------------------------------------------------------------------------------
# predicted_path[0] is defined as where the person will be one closed-loop sense->setpoint latency
# after header.stamp, and BOTH consumers are written to that and neither re-compensates
# (follow_planner_node.hpp:404-414, command_mux._person_points). The whole stack's compensation
# happens here and nowhere else.
#
# THIS IS A DERIVED PARAMETER, NOT A MEASUREMENT, and every leg below is marked MEASURED or
# ESTIMATED because the difference is the whole point. (An earlier version of this comment said
# ~199 ms mean / ~382 ms worst. The mean is about right; the WORST CASE WAS WRONG, and the reason
# is a number already in this repo -- it folded the cloud build into leg 1 and used one nominal
# odom period, 99 ms, rather than the 204 ms inter-message gap plan/dtp-imp/11 R-F22 MEASURED on
# this aircraft. See MAX_PAIR_AGE_S.)
#
#   1. stamp -> colour delivered + decoded      ~36 / ~72 ms   MEASURED (derived). plan 21's
#      corrected A/B gives 50.1 ms mean stamp->inference-done on the compressed arm with 14.2 ms
#      of inference, so ~36 ms is transport + camera pipeline; JPEG decode is 5.9 ms of it. The
#      86.6 ms worst total was recorded on the raw arm.
#   2. organized-cloud build                    ~15 / ~20 ms   ESTIMATED. fusion_node's own timing
#      note, taken on a development x86 host, NOT the NX -- treat it as a floor.
#      MEASURED on the NX 2026-09-13 (scripts/bench_cloud_backend.py, 200 real frames, camera
#      depth): numpy 38 / 45 ms (mean / p95), and it was a floor -- 2.5x the estimate. With
#      cloud_backend=cuda it is 16-31 / 21-34 ms depending on how far the GPU governor has
#      clocked down. The 0.20 s below was NOT re-derived from either; the mean total moves by
#      less than one odom period.
#   3. wait for odom to cover the stamp         ~50 / 204 ms   MEASURED both ends. /lidar_slam/odom
#      is 10.10 Hz (plan 21:333) so the mean wait is half a period; 204 ms is R-F22's worst gap.
#   4. YOLO inference inside process_frame      14.2 / ~25 ms  MEASURED standalone (13.6-14.2 ms).
#      NOT measured with the SUPER stack co-resident.
#   5. patch medians, height, velocity, path, message build   ~5 / ~10 ms   ESTIMATED, never
#      instrumented -- which is why profile=True is on below and timings_ms["post"] is logged.
#   6. publish -> target_manager -> follow_planner            ~3 / ~5 ms    ESTIMATED (two DDS hops).
#   7. follow_planner's goal timer, 15.0 Hz     33 / 67 ms     rate VERIFIED, wait arithmetic.
#   8. command_mux arbitration, 50.0 Hz         10 / 20 ms     rate VERIFIED, wait arithmetic.
#   9. SUPER's next replan, replan_rate 15.0    33 / 67 ms     rate VERIFIED, wait arithmetic.
#  10. SUPER trajectory -> offboard setpoint    ~15 / ~25 ms   ESTIMATED, never measured. The
#      least-grounded leg in the chain.
#   TOTAL ~214 ms mean, ~520 ms worst.
#
# NOTHING IN THAT BUDGET IS AN END-TO-END MEASUREMENT. Every leg is either a component measurement
# taken without the SUPER stack running, or an estimate. The one number that would settle it --
# stamp on /dtp/targets to setpoint on the wire, under concurrent load -- does not exist.
#
# 0.20 is shipped rather than the mean or the worst, for three reasons and the third decides it:
# (a) it sits essentially at the mean (214 ms), inside the 150-250 ms plan/dtp-imp/14 C5 specifies;
# (b) follow_planner_node.hpp:746-751 justifies commanding the standoff band MIDPOINT by saying
# "the latency compensation already baked into predicted_path[0] spends ~0.3 m of that at walking
# pace" -- 0.3 m at 1.4 m/s IS 0.214 s, so the other half of the stack sized its margins for this
# number and shipping a different one silently invalidates that node's arithmetic;
# (c) OVER-COMPENSATION IS THE DANGEROUS DIRECTION. It pushes the reference point AHEAD of the
# person and SHORTENS the real standoff, where under-compensation only leaves the drone trailing.
# Shipping the 520 ms worst case would spend 0.73 m of a margin only 0.5 m wide (follow_near_m 3.0
# minus min_standoff 2.5) and park the commanded goal PAST the hard floor.
#
# plan/dtp-imp/06 M1.4.2 is the gate that MEASURES this end to end. It has not been run. When it
# is, the measurement replaces this derivation and this comment gets corrected in place.
DEFAULT_LATENCY_COMPENSATION_S = 0.20

# ---------------------------------------------------------------------------------------------
# Per-track range gate (the "trunk hijacking" filter fusion_node's docstring defers to this node)
# ---------------------------------------------------------------------------------------------
# A 2-state (range, range-rate) constant-velocity Kalman filter per track, on the RAW patch range
# -- not on the published position. The bearing to a person comes from YOLO and is reliable; the
# range is the channel that jumps when a torso patch half-lands on a doorway edge, a reflective
# floor or the wall behind. Rejecting on range therefore targets the actual failure.
#
# Measurement sigma grows with range because every candidate sensor's error does: stereo
# disparity error is quadratic in depth and the LiDAR's return count per torso falls off. A flat
# sigma would gate a 3 m target far too loosely and an 8 m one far too tightly.
RANGE_SIGMA_BASE_M = 0.05
RANGE_SIGMA_FRAC = 0.02
# Process noise as an along-ray acceleration. 2 m/s^2 lets the filter follow a person starting,
# stopping or turning within a frame or two without the gate fighting them.
RANGE_ACCEL_SIGMA_MPS2 = 2.0
# Initial range-rate uncertainty: a person may already be walking at any speed when first seen, so
# the filter must not be confident about zero on frame two.
RANGE_INIT_RATE_SIGMA_MPS = 2.0
# Reject a measurement outside max(RANGE_GATE_SIGMA * innovation sigma, RANGE_GATE_FLOOR_M).
# The absolute floor is what stops a briefly over-confident filter from rejecting a real
# acceleration; 0.35 m is detection.py's own max_step_m, so the two clamps agree on what counts as
# a step a human can take between frames rather than disagreeing silently.
RANGE_GATE_SIGMA = 4.0
RANGE_GATE_FLOOR_M = 0.35
# After this many CONSECUTIVE rejections the measurement is accepted and the filter reset.
# Sustained disagreement is the world, not a glitch -- a person who stepped behind a doorway and
# reappeared two metres away, or a tracker id re-associated onto someone else. At 15 Hz this is
# 0.2 s of holding, which is inside every consumer's freshness gate.
RANGE_MAX_HOLD_FRAMES = 3
# A gap longer than this makes the constant-velocity prediction meaningless (the person could be
# anywhere), so the filter restarts instead of gating against a stale state.
RANGE_KF_MAX_DT_S = 0.5

# ---------------------------------------------------------------------------------------------
# Velocity -- computed here because process_frame re-derives it internally and never publishes it
# ---------------------------------------------------------------------------------------------
# The same window and smoothing detection.py's own speed estimate uses (detection.py:279-312:
# data[-25:], _ema_1d alpha 0.30, trimmed mean of the last <=6 instantaneous values at
# trim_ratio 0.10). Matching it is the point: follow_planner derives the standoff viewpoint from
# atan2(velocity.y, velocity.x), and if that heading disagrees with the heading the CTRV path
# implies, the drone's viewpoint and the path it is following point in different directions.
VELOCITY_HISTORY_N = 25
VELOCITY_EMA_ALPHA = 0.30
VELOCITY_TRIM_RATIO = 0.10
VELOCITY_SAMPLES = 6
# Fewer differences than this and no velocity is reported at all -- the same floor
# _predict_future_xz applies before it will estimate a speed (detection.py:299).
MIN_VELOCITY_DIFFS = 3
# Same clip _predict_future_xz applies to its speed estimate (detection.py:366). A human does not
# sustain 3 m/s in this scenario, and a larger number here is a depth glitch differentiated.
MAX_HUMAN_SPEED_MPS = 3.0

# ---------------------------------------------------------------------------------------------
# Misc
# ---------------------------------------------------------------------------------------------
# Two history samples closer together than this are the same instant; dividing by that dt turns
# depth noise into a velocity. Same threshold detection.py:288 uses.
DT_EPS_S = 1e-4
LOG_THROTTLE_S = 5.0

# ---------------------------------------------------------------------------------------------
# /dtp/detections_2d -- the boxes, for a human to look at (dtp_image_overlay)
# ---------------------------------------------------------------------------------------------
# /dtp/targets carries world positions only, so nothing outside this process could draw the box a
# range was sampled from. This topic is a VIEWER side channel and nothing in the flight path reads
# it. JSON in a std_msgs/String on purpose: a new dtp_msgs type means rebuilding the package
# SUPER's fsm_node links against, for a message only a browser consumes. The header stamp travels
# inside the JSON and is the colour frame's own stamp, which is what lets the viewer pair a box
# with the exact JPEG it was drawn on. Built only while something subscribes, and only AFTER
# /dtp/targets and /dtp/perception_ok have gone out, so a fault here can never cost the veto a
# message.
DETECTIONS_2D_TOPIC = '/dtp/detections_2d'


def _jnum(v, nd=3):
    """A float for JSON: rounded, and null rather than NaN/inf (json.dumps raises on those)."""
    if v is None:
        return None
    v = float(v)
    return round(v, nd) if math.isfinite(v) else None


# ---------------------------------------------------------------------------------------------
# Organized cloud on the GPU (cloud_backend: cuda) -- see dtp_perception.cuda_cloud
# ---------------------------------------------------------------------------------------------
# Measured 2026-09-13 over 200 real frames: numpy 38 ms wall / 112 ms CPU per frame (~90 % of a
# core at 8 Hz, the largest single cost in this node); the GPU 16-31 ms wall / 10-14 ms CPU,
# bit-identical on every frame. Replaying the same bag through this node twice per backend: 1.9
# cores (numpy) -> 0.7 cores (cuda). Per-frame WALL time did not improve -- the GPU is shared with
# YOLO -- so this is CPU headroom for the stack, not latency. The first frames are built BOTH
# ways and compared, and the numpy
# cloud is the one flown while that happens: a GPU path that disagrees with the reference is
# switched off before it ever reaches process_frame, rather than trusted because a bench agreed.
CUDA_SELFCHECK_FRAMES = 5
# A GPU error costs one frame nothing (it is rebuilt in numpy); this many in a row means the GPU
# is not coming back, so it is dropped for the life of the node and said so at ERROR.
CUDA_MAX_CONSECUTIVE_FAILURES = 3
CLOUD_BACKENDS = ('numpy', 'cuda')
# The organized cloud's wire layout (point_step 12, x/y/z FLOAT32 at 0/4/8, is_dense False) is NOT
# restated here. It lives once in dtp_perception.organized_cloud, next to the writer that produces
# it, because a reader and a writer that agree only by coincidence is precisely how a sheared cloud
# reaches process_frame -- and a sheared cloud does not raise, it projects plausibly.

# world (Z-up ENU, e.g. camera_init) -> the "pipeline" frame process_frame assumes (Y-up), by
# swapping rows 1 and 2.
#
# WHY THIS IS NEEDED AND WHY IT GOES ON THE WORLD SIDE. detection.py:479's
# _camera_xz_to_odom_xz computes x_odom from T[0,0]/T[0,2]/T[0,3] and "z_odom" from row 2 -- i.e.
# it assumes the GROUND PLANE is the world's X-Z plane, which is what the ground-robot reference
# had. `camera_init` is Z-up, so handing it a raw T_world_camera makes its "z_odom" the person's
# ALTITUDE, and distance, prediction, region classification and the max_depth gate all become
# nonsense with no error anywhere. After this swizzle T_pipe rows 0 and 2 are the two HORIZONTAL
# world axes, so process_frame's (x, z) are world (X, Y), cam_x_odom/cam_z_odom are the drone's
# horizontal position, and cam_yaw_odom = atan2(T[0,2], T[2,2]) is a correct ground-plane heading.
#
# plan/dtp-imp/01 SS3 asks for this remap on the CAMERA side (a camera->pipeline axis swap). That
# is the wrong side and needs correcting there: process_frame reads only cloud channels 0 and 2,
# which are identical in camera-optical and pipeline conventions, and never reads channel 1 -- so
# no remap belongs on the cloud. The need is real, the location was not.
#
# S is a reflection (det = -1), which is harmless here: process_frame only ever uses rows 0 and 2
# as a 2-D affine map and handedness never enters. The UNSWIZZLED matrix is kept as well, because
# the height lift needs its real row 2.
PIPELINE_SWIZZLE = np.array([[1.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])


class _PendingFrame:
    """One decimated frame waiting for TF to cover its stamp. Holds an 11 MB cloud -- keep few."""

    __slots__ = ('stamp_msg', 'stamp_s', 'bgr', 'cloud', 'camera_frame', 'cloud_ms')

    def __init__(self, stamp_msg, stamp_s, bgr, cloud, camera_frame, cloud_ms=float('nan')):
        self.stamp_msg = stamp_msg
        self.stamp_s = stamp_s
        self.bgr = bgr
        self.cloud = cloud
        self.camera_frame = camera_frame
        self.cloud_ms = cloud_ms


class _TrackState:
    """What perception_node remembers about one track -- NOT what the predictor remembers.

    The predictor owns the smoothed x/z history, the EMA and the motion clamp, and it is frozen,
    so nothing here can reach into it. This is the state for the two channels this node owns
    outright: world Z, and the per-track range gate.
    """

    __slots__ = ('world_z', 'has_height', 'last_xy', 'last_velocity',
                 'kf_r', 'kf_v', 'kf_p00', 'kf_p01', 'kf_p11', 'kf_t', 'reject_run',
                 'height_fallback_logged', 'holds')

    def __init__(self):
        self.world_z = float('nan')
        self.has_height = False
        self.last_xy = None            # last PUBLISHED horizontal position, for the hold path
        self.last_velocity = (0.0, 0.0)
        # Lifetime count of frames on which this track's range was rejected. Carried in the hold
        # log line because that line is THROTTLED: at 15 Hz an unthrottled one is 15 lines a second
        # on a box whose CPU is already the constraint, but the contract requires every hold to be
        # visible, and a running total makes a throttled line report the ones it swallowed.
        self.holds = 0
        self.kf_t = None
        self.kf_r = float('nan')
        self.kf_v = 0.0
        self.kf_p00 = 0.0
        self.kf_p01 = 0.0
        self.kf_p11 = 0.0
        self.reject_run = 0
        self.height_fallback_logged = False

    # -- the range gate ------------------------------------------------------------------------

    def _kf_reset(self, r_meas, t_s, r_var):
        self.kf_r = r_meas
        self.kf_v = 0.0
        self.kf_p00 = r_var
        self.kf_p01 = 0.0
        self.kf_p11 = RANGE_INIT_RATE_SIGMA_MPS ** 2
        self.kf_t = t_s
        self.reject_run = 0

    def accept_range(self, r_meas, t_s):
        """Is `r_meas` consistent with this track's OWN recent range motion? Updates the filter.

        Returns (accepted, innovation, gate). The filter's estimate is deliberately NOT published:
        process_frame owns the position and applies its own EMA and motion clamp over the same
        patch, and publishing a second, differently-filtered range would put the drone at one
        distance and the predictor's history at another. What this provides is a GATE -- one bad
        depth patch (a doorway edge, a reflective floor, the wall behind a person who paused) is
        held out instead of teleporting the target, and because the covariance keeps growing while
        held, a person who really did move is re-accepted within RANGE_MAX_HOLD_FRAMES.
        """
        r_var = (RANGE_SIGMA_BASE_M + RANGE_SIGMA_FRAC * max(r_meas, 0.0)) ** 2

        if self.kf_t is None or not math.isfinite(self.kf_r):
            self._kf_reset(r_meas, t_s, r_var)
            return True, 0.0, float('inf')

        dt = t_s - self.kf_t
        if dt <= 0.0 or dt > RANGE_KF_MAX_DT_S:
            # Out-of-order or a long gap: a constant-velocity prediction across it means nothing,
            # so restart rather than gate against a state that no longer describes this person.
            self._kf_reset(r_meas, t_s, r_var)
            return True, 0.0, float('inf')

        # Predict. Written out rather than via numpy: this is 2x2, runs per track per frame, and
        # the closed form is easier to check against than a matrix expression.
        r_pred = self.kf_r + self.kf_v * dt
        sa2 = RANGE_ACCEL_SIGMA_MPS2 ** 2
        p00 = self.kf_p00 + 2.0 * dt * self.kf_p01 + dt * dt * self.kf_p11 + sa2 * dt ** 4 / 4.0
        p01 = self.kf_p01 + dt * self.kf_p11 + sa2 * dt ** 3 / 2.0
        p11 = self.kf_p11 + sa2 * dt * dt

        innov = r_meas - r_pred
        s = p00 + r_var
        gate = max(RANGE_GATE_SIGMA * math.sqrt(s), RANGE_GATE_FLOOR_M)

        if abs(innov) > gate:
            self.reject_run += 1
            if self.reject_run <= RANGE_MAX_HOLD_FRAMES:
                # Coast on the prediction and KEEP the inflated covariance -- that is what widens
                # the gate on the next frame and makes the hold self-limiting.
                self.kf_r, self.kf_p00, self.kf_p01, self.kf_p11 = r_pred, p00, p01, p11
                self.kf_t = t_s
                return False, innov, gate
            self._kf_reset(r_meas, t_s, r_var)
            return True, innov, gate

        k0 = p00 / s
        k1 = p01 / s
        self.kf_r = r_pred + k0 * innov
        self.kf_v = self.kf_v + k1 * innov
        self.kf_p00 = (1.0 - k0) * p00
        self.kf_p01 = (1.0 - k0) * p01
        self.kf_p11 = p11 - k1 * p01
        self.kf_t = t_s
        self.reject_run = 0
        return True, innov, gate


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # model: the TensorRT engine. The default is an ABSOLUTE on-device path, not a relative
        # one -- 'models/yolo11s.engine' resolved against the process working directory, so it
        # only worked if you happened to launch from the right place (the same defect fixed for
        # calib_file on 2026-09-07).
        # Engines deliberately do NOT ship in the package share, and this is not just about the
        # 8-22 MB: a TensorRT engine is specific to the GPU, driver and TensorRT version it was
        # BUILT on, so a shipped one would be silently wrong (or refuse to load) elsewhere.
        # Build on-target -- plan/dtp-imp/05 M0.2.
        self.declare_parameter('model', os.path.expanduser('~/models/yolo11s.engine'))
        self.declare_parameter('conf', 0.25)
        # imgsz must match the shape the engine was BUILT for. A TensorRT engine has a fixed input
        # shape; a mismatch either costs a resize on every frame or refuses to load. 640 is what
        # every measurement in scripts/ (bench_yolo, compare_transport_detect) was taken at, so it
        # is the only number whose cost on this hardware is known.
        self.declare_parameter('imgsz', 640)

        # odom_frame: the WORLD frame. 'camera_init' is FAST-LIO's origin frame and the
        # header.frame_id of /lidar_slam/odom -- verified, and what config/dtp_params.yaml already
        # sets. The previous default 'odom' is the plan's informal prose name for the world frame
        # and NOTHING publishes that string, so it could never resolve; a default that only works
        # when a particular params file happens to be loaded is a start-up failure waiting to
        # happen. (dtp_params.yaml's comment describing this node's default as 'odom' is now stale
        # -- the override it sets is correct and still wins.)
        # MEASURED ON THE AIRCRAFT 2026-09-08 from a live /lidar_slam/odom:
    #   header.frame_id  = lidar_init   (world)
    #   child_frame_id   = lidar_imu    (body)
    # NOT camera_init. That name was inferred from this file and recorded as "verified";
    # it was never read off the topic. Verify with:
    #   ros2 topic echo --field header.frame_id /lidar_slam/odom
        self.declare_parameter('odom_frame', 'lidar_init')
        # camera_frame: EMPTY MEANS "ask the cloud", and that is the honest default.
        # dtp_params.yaml deliberately refuses to set this, on the grounds that a
        # wrong-but-plausible frame id fails the lookup exactly like a wrong one while LOOKING
        # checked. That reasoning is right and this default respects it by not guessing at all:
        # on the internal path the cloud is rasterised into depth_source.COLOUR_OPTICAL_FRAME by
        # construction, and on the replay path the cloud message carries its own frame_id. Either
        # way the string is read from the thing being transformed rather than asserted. The
        # resolved name is printed at start-up, so it is visible rather than implied.
        self.declare_parameter('camera_frame', '')

        # Compressed by default: rclpy drops ~8% of raw 2.76 MB frames at 30 Hz under BEST_EFFORT
        # because it cannot deserialise them fast enough, while a C++ subscriber loses none. JPEG
        # is 0.17 MB and keeps up. Measured 2026-09-07 -- see image_io.
        self.declare_parameter('use_compressed', True)

        # cloud_source: 'internal' builds the organized cloud in THIS process from depth_source;
        # 'topic' subscribes /dtp/points_fused. See the module docstring -- 'topic' is the REPLAY
        # path, because 11.06 MB per message at the detection rate is 166 MB/s into rclpy and this
        # repo already measured ~8% loss at a third of that.
        self.declare_parameter('cloud_source', 'internal')
        # cloud_backend: where the internal cloud is built. 'numpy' is the reference and the node's
        # default; 'cuda' is the same chain on the GPU (dtp_perception.cuda_cloud), ~100 ms of CPU
        # cheaper per frame and verified against numpy on the first frames of every run. If CUDA
        # cannot be brought up, the node says so at ERROR and runs numpy -- it does not refuse to
        # see. Ignored under cloud_source:=topic, where no cloud is built here.
        self.declare_parameter('cloud_backend', 'numpy')
        self.declare_parameter('depth_source', 'camera')
        self.declare_parameter('calib_file', calib_io.DEFAULT_CALIB)
        self.declare_parameter('depth_ready_timeout_s', 5.0)
        # Deproject EVERY depth pixel (stride 1), not depth_source's default of 2.
        # DEPTH_STRIDE = 2 is justified for range_in_roi, where 102k points is far more than the
        # tens of samples a torso statistic needs. That reasoning does NOT carry to the organized
        # cloud: stride 2 projects at most 424x240 = 101,760 depth points into 921,600 colour
        # pixels, so under 10% of the cloud is ever non-NaN (which is exactly the "89899 pts in
        # view" plan/dtp-imp/21 recorded). At 8 m a torso patch is ~133 px, giving ~13 valid
        # samples at stride 2 against ~52 at stride 1 -- and _extract_xz_from_pointcloud has NO
        # minimum-inlier gate, so a median over 13 samples with a few on the wall behind is
        # reported with full confidence. The cost is 407k un-projections instead of 102k, a few ms
        # of numpy. The LiDAR source ignores this entirely, which is why it is only passed to the
        # camera source and why the number is only defensible there.
        self.declare_parameter('depth_stride', 1)
        # Sensor believability cap, matching depth_source.MAX_RANGE_M and fusion_node.max_depth.
        # Deliberately NOT the same knob as max_depth_m below: this one says how far we believe
        # the hardware, that one says how far we are willing to ACQUIRE a person. Collapsing them
        # would mean a policy change silently editing what the cloud contains.
        self.declare_parameter('max_range_m', 15.0)

        # max_depth_m: acquisition policy cap, and it MUST be declared rather than inherited.
        # HumanXZPredictor defaults it to 6.0, so acquisition would silently stop at 6 m -- while
        # fusion_node.max_depth and depth_source.MAX_RANGE_M are both 15.0. Three caps, two
        # values, and the binding one was a library default nobody chose. 10.0 sits well beyond
        # the 3-4 m standoff band and inside the ~8-10 m the stereo is trusted to.
        self.declare_parameter('max_depth_m', 10.0)
        # The predictor's own forecast gate also defaults to 6.0. Left there it would switch the
        # CTRV forecast off between 6 and 10 m -- exactly where the person is farthest, the lead
        # matters most, and predicted_path silently becoming empty would drop us to
        # constant-velocity extrapolation with nothing on the wire to say so.
        self.declare_parameter('prediction_depth_gate_m', 10.0)
        self.declare_parameter('detect_rate_hz', DEFAULT_DETECT_RATE_HZ)
        self.declare_parameter('latency_compensation_s', DEFAULT_LATENCY_COMPENSATION_S)
        self.declare_parameter('max_pair_age_s', MAX_PAIR_AGE_S)
        # See _lookup(). BENCH ONLY -- it trades risk-A8 correctness for availability when
        # the camera and SLAM clocks disagree. Must be false for any flight result.
        self.declare_parameter('tf_at_latest', False)
        # DECLARED, not hardcoded, because this gate rejects a frame outright and the quantity it
        # gates has never been measured on the aircraft. It is DELIVERY skew -- the colour stamp
        # against depth_source.last_stamp_s, the newest depth message handed to this process -- so
        # it carries transport and scheduling delay, not just sensor phase. If the depth leg
        # happens to run, say, 30 ms behind the compressed colour leg, EVERY frame exceeds a
        # 25 ms threshold and this node publishes an empty TargetArray at a healthy 15 Hz
        # forever: indistinguishable on the wire from "nobody is here", and unfixable without a
        # rebuild if the number is baked in. Measure the real phase before widening it; a wider
        # window trades pairing error for availability and both failures are quiet.
        self.declare_parameter('max_depth_colour_skew_s', MAX_DEPTH_COLOUR_SKEW_S)
        # Minimum surviving samples in the torso patch before a range is believed. Reuses
        # depth_source.MIN_ROI_SAMPLES and its reasoning verbatim: fewer retained samples than
        # this is noise, not a measurement. plan/dtp-imp/01 SS4 step 4 and plan/dtp-imp/06 M1.4.3
        # both require this gate; _extract_xz_from_pointcloud has none (`if not np.any(valid_mask)`
        # means ONE surviving sample produces a confident median), and detection_pipeline/ is
        # frozen, so it lives here -- with the honest caveat recorded in _build_target.
        self.declare_parameter('min_patch_samples', depth_source.MIN_ROI_SAMPLES)
        # max_step_m: the predictor's per-frame motion clamp on a confirmed track's WORLD position
        # (detection.py _limit_motion_step), applied before its EMA (smooth_alpha 0.25). The two
        # together cap how fast the PUBLISHED position can move: smooth_alpha * max_step_m per
        # processed frame. At the library default 0.35 that is 0.0875 m/frame = 0.9 m/s at the
        # ~10.5 Hz actually flown -- slower than a brisk walk. Flight 2026-10-02 20:03: the walker
        # jogged at ~2 m/s, the detector boxed them and the depth read 4 -> 9.3 m, but the
        # published position could only advance at 0.87 m/s, which was the aircraft's own speed,
        # so the range
        # read a constant 3.7 m for 3 s until the track was dropped and rebuilt at 9.4 m. Passed
        # through to the predictor so the frozen module stays frozen; the node default is the
        # library's, dtp_params.yaml chooses the flown value.
        # Gate: scripts/max_step_replay_check.py.
        self.declare_parameter('max_step_m', 0.35)
        # smooth_alpha: the predictor's EMA weight on the same position (detection.py _smooth_xz),
        # applied after the clamp. The lag it leaves behind a steadily moving person is about
        # (1 - alpha) / alpha frames of their motion: 3 frames at the library's 0.25 (0.7 m on a
        # 2.5 m/s walker at 10 Hz), 1 frame at 0.5 (0.25 m). The price is noise: a standing
        # person's jitter passes at sqrt(alpha / (2 - alpha)) of the raw, 0.38x at 0.25 and
        # 0.58x at 0.5. _build_target reads self._pred.smooth_alpha for the height EMA too, so
        # z follows the same weight. Node default is the library's; dtp_params.yaml chooses.
        self.declare_parameter('smooth_alpha', 0.25)
        # draw copies the frame and runs ~8 cv2 calls per detection per frame. Off on the aircraft.
        self.declare_parameter('draw', False)
        # See DETECTIONS_2D_TOPIC. On by default because it costs one subscriber-count query per
        # frame while nobody is watching; false removes the publisher entirely.
        self.declare_parameter('publish_detections_2d', True)

        self._world_frame = str(self.get_parameter('odom_frame').value)
        self._camera_frame_param = str(self.get_parameter('camera_frame').value).strip()
        self._latency_s = float(self.get_parameter('latency_compensation_s').value)
        self._max_pair_age = float(self.get_parameter('max_pair_age_s').value)
        self._max_skew_s = float(self.get_parameter('max_depth_colour_skew_s').value)
        self._tf_at_latest = bool(self.get_parameter('tf_at_latest').value)
        self._min_patch_samples = int(self.get_parameter('min_patch_samples').value)
        self._conf_floor = float(self.get_parameter('conf').value)
        detect_rate = max(1e-3, float(self.get_parameter('detect_rate_hz').value))
        self._detect_period_s = 1.0 / detect_rate

        # use_sim_time must AGREE across perception_node, follow_planner and fsm_node: SUPER's yaw
        # gate compares getSimTime() - header.stamp, and a mismatch fails it permanently and
        # invisibly (follow_planner_node.hpp:1043-1047), leaving the drone flying a perfect
        # standoff while never looking at the person (risk R-F10). dtp_bringup.preflight has
        # assert_wall_clock for exactly this and follower.launch.py calls it -- it is not called
        # from here because dtp_perception does not depend on dtp_bringup and adding the
        # dependency to get one assertion would couple the two packages' build order.
        if bool(self.get_parameter('use_sim_time').value):
            self.get_logger().warn(
                'use_sim_time is TRUE. Every node in the follow chain must agree, including '
                "SUPER's fsm_node -- a mismatch fails SUPER's yaw-target age gate silently and "
                'the drone follows without ever looking at the person (R-F10).')

        self._warned = set()
        self._tracks = {}
        self._last_accept_s = None
        self._blind_frames = 0
        # Bounded by the wait window, not by taste: at most one frame per detection period can be
        # waiting for odom, and each entry pins an 11 MB cloud. Built here, before any subscription
        # exists, so no callback can ever meet half-built state.
        self._pending = deque(maxlen=max(2, int(math.ceil(self._max_pair_age * detect_rate)) + 1))

        # The predictor. Every kwarg below is set DELIBERATELY; the ones not listed keep the
        # library default on purpose.
        model_path = os.path.expanduser(str(self.get_parameter('model').value))
        self._pred = HumanXZPredictor(
            model_path=model_path,
            conf=self._conf_floor,
            imgsz=int(self.get_parameter('imgsz').value),
            # ONE TIME PER PUBLISHED TARGET. With detect_every_n=2 the predictor reuses the
            # PREVIOUS frame's boxes against the CURRENT cloud on alternate frames while
            # _mark_track_seen records a fresh sighting -- at 3 m and 1.4 m/s lateral that is ~9 px
            # of box offset into a patch whose half-width is 8 px, i.e. the sample walks off the
            # torso, with nothing on the wire to say so. The rate is halved by decimating at the
            # ROS callback instead, which also halves the CPU rather than only the GPU.
            detect_every_n=1,
            draw=bool(self.get_parameter('draw').value),
            # The only free instrumentation of the post-processing leg, which is otherwise
            # unmeasured and is leg 3 of the latency budget above.
            profile=True,
            # PINNED, not inherited: True returns ONE detection and would silently blind
            # command_mux's proximity veto to everyone but the biggest box. The veto reads
            # /dtp/targets precisely so a follow-logic bug cannot defeat it; handing it one person
            # defeats it from the other side.
            select_largest_only=False,
            max_depth_m=float(self.get_parameter('max_depth_m').value),
            prediction_depth_gate_m=float(self.get_parameter('prediction_depth_gate_m').value),
            # The per-frame clamp on a confirmed track's world position; see the parameter's
            # comment. Library default 0.35 = 0.9 m/s of trackable speed at 10 Hz, which froze
            # the range on a jogging walker on 2026-10-02.
            max_step_m=float(self.get_parameter('max_step_m').value),
            # The EMA weight behind that clamp; see the parameter's comment. Library default 0.25.
            smooth_alpha=float(self.get_parameter('smooth_alpha').value),
            # Time-based ghost-track window instead of the frame-count default. max_missed_frames=5
            # scales with rate (0.17 s at 30 Hz, 0.33 s at 15 Hz, and the CTRV model keeps
            # predicting a vanished person for that long); detection.py's own comment at line 84
            # asks for the time-based form so the window is rate-independent.
            max_missed_secs=0.5,
            # No checkpoint ships for the eigen trajectory model, and CPU is already tight
            # (risk R-F11).
            eigen_enabled=False,
        )
        _rate = float(self.get_parameter('detect_rate_hz').value)
        self.get_logger().info(
            'position filter: max_step_m %.2f, smooth_alpha %.2f -> the published position of a '
            'confirmed track moves at most %.3f m per processed frame (%.1f m/s at detect_rate_hz '
            '%.1f)' % (self._pred.max_step_m, self._pred.smooth_alpha,
                       self._pred.smooth_alpha * self._pred.max_step_m,
                       self._pred.smooth_alpha * self._pred.max_step_m * _rate, _rate))
        # detect_every_n IS AN ACTIVE FOOTGUN AND NOTHING ELSE IN THE CODE PREVENTS IT. The kwarg
        # above pins it to 1, but the library default is 2 and the obvious "cheap" way to raise the
        # rate is to set it -- so this refuses to start if it is ever anything else, whether from a
        # future parameter, a merge, or an upstream default change. With detect_every_n > 1,
        # _should_run_inference skips the model and process_frame reuses self.last_parsed_detections
        # -- the PREVIOUS frame's boxes -- against the CURRENT cloud, while _mark_track_seen still
        # records a fresh sighting. At 3 m and 1.4 m/s lateral that is ~9 px of box offset into a
        # patch whose half-width is 8 px: the sample point walks off the torso onto whatever is
        # behind, with NOTHING on the wire saying so. It also breaks _confidence_by_track, which
        # would then report the previous inference frame's confidences against this frame's ids.
        # The rate lever is the ROS-callback decimator in _accept_now, which halves the CPU (decode,
        # cloud build, patch) as well as the GPU rather than only the GPU.
        if self._pred.detect_every_n != 1:
            raise ValueError(
                'HumanXZPredictor.detect_every_n is %d, must be 1. Frame decimation happens at the '
                'ROS colour callback (see _accept_now / detect_rate_hz), never in the predictor: '
                'with detect_every_n > 1 the PREVIOUS frame\'s boxes are indexed into the CURRENT '
                'cloud while the track is marked freshly seen, so the torso sample silently walks '
                'onto the background.' % self._pred.detect_every_n)

        # Bound once, at construction, so a rename inside the frozen module fails loudly HERE at
        # start-up rather than as an AttributeError inside a 15 Hz callback. Reused rather than
        # reimplemented so the velocity we publish is the same statistic the CTRV forecast is
        # built from -- see VELOCITY_HISTORY_N. Both are @staticmethod upstream, which is why they
        # bind off the CLASS and take no self.
        self._ema_1d = HumanXZPredictor._ema_1d
        self._trimmed_mean = HumanXZPredictor._robust_trimmed_mean

        # TF buffer/listener: how a ROS2 node asks "what was the transform between two frames at a
        # given time" -- the Buffer caches recent transforms, the Listener keeps it fed. Created
        # BEFORE the warm-up spin below, because /tf_static is TRANSIENT_LOCAL and is delivered on
        # subscription match, which only happens while the node is being spun.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cloud_source = str(self.get_parameter('cloud_source').value).strip().lower()
        if cloud_source not in ('internal', 'topic'):
            raise ValueError(
                "cloud_source %r -- expected 'internal' or 'topic'. Refusing to guess: silently "
                'defaulting would run one topology while the log claims the other.' % cloud_source)
        self._internal_cloud = (cloud_source == 'internal')

        self._depth = None
        if self._internal_cloud:
            kind = str(self.get_parameter('depth_source').value)
            # Loaded even when depth_source is 'camera' and the extrinsic goes unused, for the
            # reason fusion_node states: load_for_node is the start-up IDENTITY gate. It refuses a
            # calibration belonging to a different camera body (two Orbbec bodies are attached to
            # this NX and the driver binds whichever it enumerates first) and a LiDAR stream from
            # another robot on the same DDS domain. When fusion_node is not running, this node is
            # the only place that gate can fire.
            T_camera_lidar, _intr = calib_io.load_for_node(
                self, self.get_parameter('calib_file').value)
            kw = {'max_range_m': float(self.get_parameter('max_range_m').value)}
            if kind.strip().lower() == 'camera':
                # Only the camera source takes a stride; passing it to the LiDAR source is a
                # TypeError, not a no-op.
                kw['stride'] = int(self.get_parameter('depth_stride').value)
            self._depth = depth_source.create(self, kind, T_camera_lidar=T_camera_lidar, **kw)

        # After the predictor on purpose: YOLO has already created this process's CUDA context, so
        # the GPU cloud adds no context of its own. See CUDA_SELFCHECK_FRAMES for why the first
        # frames are built twice.
        backend = str(self.get_parameter('cloud_backend').value).strip().lower()
        if backend not in CLOUD_BACKENDS:
            raise ValueError(
                "cloud_backend %r -- expected one of %s. Refusing to guess: silently defaulting "
                'would run one backend while the log claims the other.'
                % (backend, ', '.join(CLOUD_BACKENDS)))
        self._cuda = None
        self._cuda_checks_left = 0
        self._cuda_failures = 0
        self._cuda_worst = (0.0, 0.0)
        self._cloud_backend = 'numpy'
        if self._internal_cloud and backend == 'cuda':
            try:
                self._cuda = cuda_cloud.CudaCloud()
                self._cuda.warm_up()
                self._cuda_checks_left = CUDA_SELFCHECK_FRAMES
                self._cloud_backend = 'cuda'
            except cuda_cloud.CudaCloudError as exc:
                self._cuda = None
                self._cloud_backend = 'numpy (cuda unavailable)'
                self.get_logger().error(
                    'cloud_backend=cuda but the GPU cloud cannot start (%s). Running the numpy '
                    'cloud instead: identical output, but about 1.2 cores more CPU (bag replay, '
                    '2026-09-13) taken from FAST-LIO and the rest of the stack. Check the DTP '
                    'environment (dtp_env_jetson.sh puts libcudss on LD_LIBRARY_PATH).' % exc)

        use_compressed = bool(self.get_parameter('use_compressed').value)
        color_topic, color_type = image_io.image_topic_and_type(
            '/camera/color/image_raw', use_compressed)

        # MUST happen before main() hands this node to an executor: wait_until_ready spins the node
        # itself, which is only legal outside the executor's own spin, and it is the only thing
        # that makes the camera source's /tf_static lookup work at all -- tf2's own lookup timeout
        # does NOT spin, so the transform never arrives and the lookup fails with "frame does not
        # exist" (the bug diagnosed in scripts/outdoor_depth_test.py). Our own /tf_static needs the
        # same treatment, which is what the else branch buys.
        ready_timeout = float(self.get_parameter('depth_ready_timeout_s').value)
        warmup_start = time.time()
        if self._depth is not None:
            self._depth.wait_until_ready(ready_timeout)
        # TOPPED UP, not skipped on the depth path. wait_until_ready returns the INSTANT its own
        # inputs are up, which on a warm system is a few tens of milliseconds -- long before our
        # /tf_static has arrived. That matters more than it looks: a missing static transform
        # raises LookupException, which _lookup classifies as NOT retryable, so those first frames
        # would be dropped and published blind rather than queued. The warm-up is what turns "the
        # chain is not published yet" into "the chain is not published at all".
        self._spin_for(max(0.0, TF_WARMUP_S - (time.time() - warmup_start)))

        # PERCEPTION HEALTH, separate from liveness. An empty TargetArray means "alive and
        # evaluated this frame, nobody here"; it is NOT distinguishable on the wire from "alive
        # and BLIND", and the blind state silently empties command_mux's human-proximity veto
        # while every freshness gate stays satisfied. sensor_watchdog folds this Bool into
        # /dtp/sensors_ok, which command_mux already treats as a hard veto -- so a blind
        # pipeline stops the drone instead of reading as a calm one.
        self._ok_pub = self.create_publisher(Bool, '/dtp/perception_ok', 10)
        self._targets_pub = self.create_publisher(
            TargetArray, '/dtp/targets', qos_profile_sensor_data)
        self._det2d_pub = None
        if bool(self.get_parameter('publish_detections_2d').value):
            self._det2d_pub = self.create_publisher(
                String, DETECTIONS_2D_TOPIC, qos_profile_sensor_data)

        # Subscriptions come up LAST, after the publisher every callback path needs, so no callback
        # can meet half-built state.
        if self._internal_cloud:
            self._color_sub = self.create_subscription(
                color_type, color_topic, self._on_color, qos_profile_sensor_data)
            self._sync = None
        else:
            # message_filters syncs two independently-arriving topics by timestamp so a callback
            # only fires once BOTH a colour frame and a cloud close in time are available. Kept for
            # the replay path only: the two streams on a bag are independent publishers again.
            self._color_sub = Subscriber(
                self, color_type, color_topic, qos_profile=qos_profile_sensor_data)
            self._cloud_sub = Subscriber(
                self, PointCloud2, '/dtp/points_fused', qos_profile=qos_profile_sensor_data)
            self._sync = ApproximateTimeSynchronizer(
                [self._color_sub, self._cloud_sub], queue_size=5, slop=self._max_skew_s)
            self._sync.registerCallback(self._on_synced_frame)

        self._drain_timer = self.create_timer(
            self._detect_period_s / DRAIN_RATE_MULTIPLIER, self._drain)

        self.get_logger().info(
            'perception_node up: %s, detect %.1f Hz, latency_compensation %.3f s, world=%r '
            'camera=%r' % (image_io.describe(use_compressed), detect_rate, self._latency_s,
                           self._world_frame, self._camera_frame_param or '<from the cloud>'))
        if self._internal_cloud:
            self.get_logger().info('  cloud built IN-PROCESS from depth_source[%s], backend %s: %s'
                                   % (self._depth.name, self._cloud_backend,
                                      self._depth.describe()))
            if self._cuda is not None:
                self.get_logger().info(
                    '  GPU cloud on %s; the first %d frames are also built in numpy, compared, '
                    'and the numpy cloud is flown until they agree'
                    % (self._cuda.device_name, CUDA_SELFCHECK_FRAMES))
        else:
            self.get_logger().warn(
                '  cloud_source=topic: subscribing /dtp/points_fused. This is the REPLAY path. '
                'An organized 1280x720 cloud is 11.06 MB per message and rclpy already loses ~8% '
                'of frames at a third of that throughput -- do not fly it (plan/dtp-imp/14 C3).')

    # -- small helpers ---------------------------------------------------------------------------

    def _spin_for(self, seconds):
        """Spin this node before the executor owns it. NEVER call from a callback."""
        deadline = time.time() + float(seconds)
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _warn_once(self, key, message):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(message)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _track(self, track_id):
        state = self._tracks.get(track_id)
        if state is None:
            state = _TrackState()
            self._tracks[track_id] = state
        return state

    def _forget_missing_tracks(self, live_ids):
        """Drop per-track state for tracks the predictor has already retired.

        Without this the dicts grow for the life of the flight, and a recycled ultralytics id
        would inherit a stranger's height and range filter -- which is worse than growth.
        """
        for track_id in [t for t in self._tracks if t not in live_ids]:
            if track_id not in self._pred.track_missed_counts:
                del self._tracks[track_id]

    # -- frame intake ------------------------------------------------------------------------

    def _accept_now(self, stamp_s):
        """Rate decimation, on the STAMP rather than on arrival, so jitter cannot double-count."""
        if self._last_accept_s is None or stamp_s < self._last_accept_s:
            self._last_accept_s = stamp_s
            return True
        if (stamp_s - self._last_accept_s) < self._detect_period_s * (1.0 - DECIMATE_TOL_FRAC):
            return False
        self._last_accept_s = stamp_s
        return True

    def _enqueue(self, pend):
        """Queue a frame for the world lift, then try to drain immediately.

        The eviction is explicit rather than left to deque(maxlen=...): a silent drop would be a
        frame that produced no /dtp/targets message at all, which is the one thing this node must
        never do (see _publish_blind).
        """
        if len(self._pending) >= self._pending.maxlen:
            evicted = self._pending.popleft()
            self._publish_blind(evicted.stamp_msg,
                                'TF is more than %.3f s behind and the frame queue is full'
                                % self._max_pair_age)
        self._pending.append(pend)
        self._drain()

    def _on_color(self, msg) -> None:
        """Internal-cloud path: colour arrives, depth is pulled from depth_source at that instant.

        msg is Image or CompressedImage per use_compressed; image_io.decode() gives BGR.
        """
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self._accept_now(stamp_s):
            return

        try:
            bgr = image_io.decode(msg)
        except ValueError as exc:
            # _publish_blind, not a bare return: _accept_now has ALREADY consumed this frame's
            # decimation slot, so the next colour frame 33 ms from now is rejected as too soon and
            # the gap to the next /dtp/targets becomes 133 ms. Staying silent through it is
            # indistinguishable from the node being dead, which is the one thing this node must
            # never look like -- see _publish_blind.
            self._publish_blind(msg.header.stamp, 'colour decode failed: %s' % exc)
            return

        K, size = self._depth.colour_intrinsics()
        if K is None:
            self._publish_blind(msg.header.stamp,
                                'no /camera/color/camera_info yet -- cannot build the cloud')
            return
        width, height = size
        # The cloud is rasterised at the resolution camera_info ADVERTISES; YOLO boxes land in the
        # DECODED image's pixel space and index the cloud with those pixels. If the two ever differ
        # every box indexes the wrong part of the cloud and every range is wrong while the overlays
        # still look plausible. depth_source._roi_size flags the same hazard for range_in_roi.
        if bgr.shape[0] != height or bgr.shape[1] != width:
            self._publish_blind(
                msg.header.stamp,
                'decoded colour is %dx%d but /camera/color/camera_info says %dx%d -- refusing to '
                'index the cloud with boxes measured in a different pixel space'
                % (bgr.shape[1], bgr.shape[0], width, height))
            return

        t_cloud = time.perf_counter()
        try:
            cloud = self._cloud_from_range(K, width, height)
        except (ValueError, depth_source.DepthSourceError) as exc:
            # DepthSourceError subclasses RuntimeError, NOT ValueError, so catching ValueError
            # alone let it escape this subscription callback and kill the node -- producing
            # exactly the permanent command_mux HOLD this file is architected to prevent. It is
            # raised for a depth encoding outside 16UC1/mono16/32FC1 (depth_source.py:661), which
            # is a configuration fault to report and survive, not to die on.
            self._publish_blind(msg.header.stamp, str(exc))
            return
        if cloud is None:
            missing = ', '.join(self._depth.missing()) or 'unknown'
            self._publish_blind(msg.header.stamp,
                                'depth_source[%s] has no range image yet (%s)'
                                % (self._depth.name, missing))
            return
        cloud_ms = (time.perf_counter() - t_cloud) * 1e3

        depth_stamp = self._depth_stamp_s()
        skew = abs(depth_stamp - stamp_s)
        if not math.isfinite(skew) or skew > self._max_skew_s:
            # Colour and depth are NOT hardware-locked on this camera (time_sync_en: false, risk
            # A8), so this is a real condition and not a theoretical one.
            self._publish_blind(msg.header.stamp,
                                'depth/colour skew %.3f s exceeds %.3f s -- the box and the cloud '
                                'describe different instants'
                                % (skew, self._max_skew_s))
            return

        camera_frame = self._camera_frame_param or depth_source.COLOUR_OPTICAL_FRAME
        self._enqueue(_PendingFrame(msg.header.stamp, stamp_s, bgr, cloud, camera_frame,
                                    cloud_ms))

    def _on_synced_frame(self, color_msg, cloud_msg: PointCloud2) -> None:
        """Replay path: colour and an organized /dtp/points_fused, synced on their stamps.

        color_msg is Image or CompressedImage depending on use_compressed; decode it with
        image_io.decode(), which normalises BOTH to BGR.
        """
        stamp_s = color_msg.header.stamp.sec + color_msg.header.stamp.nanosec * 1e-9
        if not self._accept_now(stamp_s):
            return

        try:
            bgr = image_io.decode(color_msg)
        except ValueError as exc:
            # Same reasoning as the internal path: the decimation slot is already spent, so a bare
            # return costs a whole publish. See _on_color.
            self._publish_blind(color_msg.header.stamp, 'colour decode failed: %s' % exc)
            return

        try:
            cloud = self._cloud_from_msg(cloud_msg)
        except ValueError as exc:
            self._publish_blind(color_msg.header.stamp, str(exc))
            return

        if bgr.shape[:2] != cloud.shape[:2]:
            self._publish_blind(
                color_msg.header.stamp,
                'decoded colour is %dx%d but the cloud is %dx%d -- refusing to index the cloud '
                'with boxes measured in a different pixel space'
                % (bgr.shape[1], bgr.shape[0], cloud.shape[1], cloud.shape[0]))
            return

        cloud_stamp = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9
        skew = abs(cloud_stamp - stamp_s)
        if not math.isfinite(skew) or skew > self._max_skew_s:
            self._publish_blind(
                color_msg.header.stamp,
                'cloud/colour skew %.3f s exceeds %.3f s' % (skew, self._max_skew_s))
            return

        camera_frame = self._camera_frame_param or cloud_msg.header.frame_id
        if not camera_frame:
            self._publish_blind(color_msg.header.stamp,
                                '/dtp/points_fused carries an empty header.frame_id and '
                                'camera_frame is unset -- nothing says what frame to lift from')
            return

        self._enqueue(_PendingFrame(color_msg.header.stamp, stamp_s, bgr, cloud, camera_frame))

    # -- cloud construction --------------------------------------------------------------------

    def _cloud_from_range(self, K, width, height):
        """(H, W, 3) float32 in the COLOUR OPTICAL frame, or None if depth is not up yet.

        The deprojection is `organized_cloud.build`, shared with fusion_node rather than duplicated
        here -- see the module docstring. What that module guarantees, and what everything below
        depends on:

          * cloud[v, u] = range[v, u] * UNIT_ray[v, u], with the ray table built from the COLOUR
            intrinsics. UNIT rays, not z * [(u-cx)/fx, (v-cy)/fy, 1]: range_image() stores
            EUCLIDEAN RANGE, not optical Z, and at this camera's ~90 deg HFOV a non-unit ray makes
            Z under-read by 1/cos(45) = 1.41x at the frame edge -- an edge-of-frame person reported
            40% closer than they are, which is exactly the error depth_source exists to remove.
          * NO RETURN IS NaN IN ALL THREE CHANNELS, never 0.0 and never a sentinel.
            _extract_xz_from_pointcloud's mask is isfinite(x) & isfinite(z) & (z > 0), so a 0.0
            fill is REJECTED by z > 0 but ACCEPTED as a finite x, and any positive sentinel is
            accepted outright and poisons the median -- "nothing here" would read as a person on
            top of the aircraft. Because the ray table has no NaN, a range NaN propagates to all
            three channels, which is what makes _sample_patch's height median unable to fail on a
            frame where the position median succeeded.
          * no axis remap. process_frame reads channels 0 and 2 only, which are identical in the
            camera-optical and "pipeline" conventions, and never reads channel 1. The remap the
            plan asks for belongs on the WORLD side -- see PIPELINE_SWIZZLE.

        The shape is VERIFIED rather than trusted: YOLO boxes are measured in the decoded image's
        pixel space and index this array directly, so a mismatch does not raise anywhere -- it
        indexes the wrong part of the cloud and reports a plausible, wrong range for every person.

        Raises ValueError with a printable reason for anything the geometry cannot vouch for;
        the caller turns that into a blind publish rather than a dropped frame.

        With cloud_backend=cuda the same cloud comes from dtp_perception.cuda_cloud, which is held
        to that module's contract: every guarantee above holds for it too, and the first frames
        prove it against this numpy path before its output is used (see _cuda_self_check).
        """
        if self._cuda is not None:
            cloud = self._gpu_cloud(K, width, height)
        else:
            cloud = self._numpy_cloud(K)
        if cloud is not None and cloud.shape != (height, width, 3):
            raise ValueError(
                'the organized cloud is %s but /camera/color/camera_info says %dx%d -- '
                'refusing to index it with boxes measured in a different pixel space'
                % (cloud.shape, width, height))
        return cloud

    def _numpy_cloud(self, K):
        """The reference path: range_image() then organized_cloud.build()."""
        rng = self._depth.range_image()
        if rng is None:
            return None
        # intrinsics_problem() rather than catching build()'s exception, which is what that helper
        # exists for: an all-zero K from a driver that has not loaded its calibration yet is NORMAL
        # at start-up, not exceptional, and it must reach the log as a sentence about the camera
        # rather than as a traceback. (Left to divide by fx = 0, the cloud would be entirely
        # non-finite -- and an all-NaN cloud reads downstream as "nobody is here", a statement about
        # the WORLD, which is the failure that looks least like one.)
        problem = organized_cloud.intrinsics_problem(K, rng.shape)
        if problem:
            raise ValueError('cannot build the organized cloud: %s' % problem)
        return organized_cloud.build(rng, K)

    def _gpu_cloud(self, K, width, height):
        """The GPU path, with its two ways back to numpy: a CUDA error, and a failed self-check."""
        problem = organized_cloud.intrinsics_problem(K, (height, width))
        if problem:
            raise ValueError('cannot build the organized cloud: %s' % problem)
        try:
            cloud = self._depth.organized_cloud_cuda(self._cuda)
        except cuda_cloud.CudaCloudError as exc:
            # This frame still gets a cloud. Going blind because the GPU hiccupped would trip
            # command_mux's sensor veto over something numpy can answer in 40 ms.
            self._on_cuda_failure(exc)
            return self._numpy_cloud(K)
        self._cuda_failures = 0
        if cloud is None or self._cuda_checks_left <= 0:
            return cloud
        return self._cuda_self_check(cloud, K)

    def _cuda_self_check(self, gpu_cloud, K):
        """Build this frame in numpy too, compare, and fly the NUMPY cloud while checking.

        Same inputs, same executor turn -- nothing can change the depth frame between the two
        builds -- so any disagreement is the GPU path's, and it is switched off at ERROR. The bench
        (scripts/bench_cloud_backend.py) found 200 of 200 real frames bit-identical; this is what
        keeps that true on the aircraft, with its own driver, CUDA and extrinsic, rather than
        assumed from a bag.
        """
        ref = self._numpy_cloud(K)
        if ref is None:
            return gpu_cloud
        mask_frac, max_diff = cuda_cloud.compare(ref, gpu_cloud)
        self._cuda_worst = (max(self._cuda_worst[0], mask_frac),
                            max(self._cuda_worst[1], max_diff))
        if not cuda_cloud.within_camera_tolerance(mask_frac, max_diff):
            self.get_logger().error(
                'GPU cloud DISAGREES with the numpy reference: finite-mask mismatch %.2e, max '
                '|diff| %.2e m (tolerance %.0e / %.0e m). cloud_backend=cuda is DISABLED for the '
                'life of this node; flying the numpy cloud. Reproduce offline with '
                'scripts/bench_cloud_backend.py on a bag from this aircraft.'
                % (mask_frac, max_diff, cuda_cloud.CAMERA_MASK_TOL_FRAC,
                   cuda_cloud.CAMERA_VALUE_TOL_M))
            self._cuda = None
            self._cuda_checks_left = 0
            self._cloud_backend = 'numpy (cuda failed its self-check)'
            return ref
        self._cuda_checks_left -= 1
        if self._cuda_checks_left == 0:
            self.get_logger().info(
                'GPU cloud verified against numpy on %d frames (worst finite-mask mismatch %.2e, '
                'worst |diff| %.2e m) -- flying the GPU cloud from the next frame'
                % (CUDA_SELFCHECK_FRAMES, self._cuda_worst[0], self._cuda_worst[1]))
        return ref

    def _on_cuda_failure(self, exc):
        self._cuda_failures += 1
        if self._cuda_failures < CUDA_MAX_CONSECUTIVE_FAILURES:
            self.get_logger().error('GPU cloud failed (%s); this frame was rebuilt in numpy'
                                    % exc, throttle_duration_sec=LOG_THROTTLE_S)
            return
        self.get_logger().error(
            'GPU cloud failed %d frames in a row (last: %s). cloud_backend=cuda is DISABLED for '
            'the life of this node; numpy from here on, about 1.2 cores more CPU.'
            % (self._cuda_failures, exc))
        self._cuda = None
        self._cloud_backend = 'numpy (cuda disabled after %d failures)' % self._cuda_failures

    def _depth_stamp_s(self):
        """Stamp of the frame range_image() last rasterised.

        The contract calls for a public `last_stamp_s` on DepthSource next to colour_intrinsics()
        (RangeSample.stamp_s already exposes the same value through the other query, so there is
        precedent). It is not there yet and depth_source.py is not this node's to edit, so this
        reads the public property when it exists and falls back to the private buffer stamp. The
        fallback is a READ of a documented internal, and it is exactly why the property should
        land: with it, the skew guard is enforceable; without it, it is a convention.
        """
        stamp = getattr(self._depth, 'last_stamp_s', None)
        if stamp is None:
            self._warn_once(
                'no-last-stamp',
                'DepthSource has no public last_stamp_s; using _buf_stamp for the depth/colour '
                'skew guard. Add the property (3 lines, beside colour_intrinsics) so the guard '
                'does not depend on another module keeping a private name.')
            stamp = getattr(self._depth, '_buf_stamp', float('nan'))
        return float(stamp)

    def _cloud_from_msg(self, msg: PointCloud2):
        """Zero-copy (H, W, 3) float32 view of /dtp/points_fused. Raises ValueError otherwise.

        The layout verification is organized_cloud.from_pointcloud2 -- the exact inverse of the
        to_pointcloud2 fusion_node publishes with, and deliberately not a second reader written
        here. A reader and a writer that agree only by coincidence is how the replay harness ends
        up feeding process_frame a sheared image, which does not raise: it shears the cloud
        diagonally and then projects perfectly plausibly, the same bug class depth_source records
        for its own `msg.step // 2`.

        `is_dense` is the one thing that reader deliberately does not police, because it changes no
        byte of the payload and rejecting on it would refuse a usable bag. It still matters to any
        PCL consumer of the same topic -- True is a promise they act on by SKIPPING the finiteness
        check, and this cloud is full of NaN by design -- so the warning is raised here, where
        there is a logger.
        """
        cloud = organized_cloud.from_pointcloud2(msg)
        if msg.is_dense:
            self._warn_once('is-dense',
                            '/dtp/points_fused sets is_dense=True. NaN is present by design in an '
                            'organized cloud and True is a promise PCL consumers act on by '
                            'skipping the finiteness check.')
        return cloud

    # -- TF ------------------------------------------------------------------------------------

    def _lookup(self, stamp_msg, camera_frame):
        """(T_world_camera 4x4, retryable, reason). T is None on failure.

        `retryable` distinguishes "odom has not caught up yet" -- the NORMAL case, since
        /lidar_slam/odom runs at 10.10 Hz against colour's 29.98 Hz so the newest sample is ~50 ms
        older than the frame -- from "this chain does not exist", where waiting only burns queue.
        """
        try:
            # tf_at_latest is an ESCAPE HATCH for a clock-source mismatch. OFF by default,
            # because it is wrong in flight.
            #
            # MEASURED 2026-09-08, first end-to-end bench run: the Orbbec driver stamps colour
            # from DEVICE time while FAST-LIO stamps from ROS time, so a colour frame sits
            # persistently ~53 ms AHEAD of the newest transform. That is an OFFSET, not a lag --
            # waiting does not close it. Frames aged out at 0.485 s and still failed with
            # "extrapolation into the future", so every single frame published blind.
            #
            # With the airframe STATIONARY the risk-A8 error this introduces is yaw-rate x range
            # = 0, so bench numbers are unaffected. IN FLIGHT IT IS NOT ZERO: at 1 rad/s and 4 m
            # a 53 ms offset displaces the target by ~0.2 m. The real fix is aligning the clock
            # sources, or buffering /lidar_slam/odom and interpolating to the frame stamp. This
            # flag exists so bench work is not blocked meanwhile, and it announces itself at
            # start-up so nobody mistakes it for correct.
            stamp = (rclpy.time.Time() if self._tf_at_latest
                     else rclpy.time.Time.from_msg(stamp_msg))
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, camera_frame, stamp,
                timeout=TF_LOOKUP_TIMEOUT)   # <-- ZERO. See TF_LOOKUP_TIMEOUT.
        except ExtrapolationException as exc:
            return None, True, str(exc)
        except (LookupException, ConnectivityException) as exc:
            return None, False, str(exc)
        except Exception as exc:                                        # noqa: BLE001
            # tf2 raises a family of exceptions and a new one must not take the node down: a dead
            # perception_node is a permanent command_mux HOLD, an empty array is a recoverable one.
            return None, False, '%s: %s' % (type(exc).__name__, exc)

        T = np.eye(4)
        # Reused rather than re-derived: a second quaternion-to-matrix expression with a divergent
        # sign convention is exactly how a mirrored world enters, and quat_to_R's docstring records
        # that it is the same expression as scripts/depth_compare.py.
        T[:3, :3] = depth_source.quat_to_R(tf.transform.rotation)
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y
        T[2, 3] = tf.transform.translation.z
        if not np.isfinite(T).all():
            return None, False, 'transform contains non-finite values'
        return T, False, ''

    def _drain(self):
        """Process every queued frame TF can now cover, oldest first.

        Strictly in stamp order and never skipping ahead: TF coverage is monotone in time, so if
        the head is not covered nothing behind it is either, and publishing a later frame first
        would put stamps out of order on /dtp/targets.
        """
        while self._pending:
            pend = self._pending[0]
            T, retryable, reason = self._lookup(pend.stamp_msg, pend.camera_frame)
            if T is not None:
                self._pending.popleft()
                self._process(pend, T)
                continue
            # Re-read the clock every pass: _process runs a whole inference, so a `now` hoisted out
            # of the loop would under-report the age of everything behind it and hold expired
            # frames past their window.
            age = self._now_s() - pend.stamp_s
            if retryable and age <= self._max_pair_age:
                break
            self._pending.popleft()
            if not retryable:
                # A non-retryable failure is almost always THE MISSING LINK, not a typo, so say
                # which link and where the numbers live rather than leaving an operator to read a
                # tf2 exception. As of 2026-09-08 nothing on this aircraft publishes it: FAST-LIO
                # publishes camera_init -> <body> and the Orbbec driver publishes camera_link ->
                # camera_color_optical_frame, and the two halves never meet. The only record of the
                # relationship is T_camera_lidar in calib/drone_cam_livox.yaml, and it is usable as
                # <body> -> camera ONLY because config/mid360.yaml sets the LiDAR->IMU extrinsic to
                # identity, which makes FAST-LIO's body frame the Livox frame. follower.launch.py
                # states the same thing; see the risks in plan/dtp-imp/21.
                self._warn_once(
                    'tf-chain',
                    '%s <- %s does not resolve, and this is NOT the "odom has not caught up yet" '
                    'case -- the chain is missing, not late. Nothing publishes <body> -> %s: '
                    'FAST-LIO gives %s -> <body> and the camera driver gives camera_link -> %s. '
                    'A static publisher owns that gap and must load T_camera_lidar through '
                    'calib_io (which carries the camera-serial and LiDAR-source gates) and publish '
                    'its INVERSE -- never hardcoded numbers. Until it exists every frame here '
                    'publishes an EMPTY TargetArray. Underlying: %s'
                    % (self._world_frame, pend.camera_frame, depth_source.COLOUR_OPTICAL_FRAME,
                       self._world_frame, depth_source.COLOUR_OPTICAL_FRAME, reason))
            # FAIL CLOSED. Never call process_frame with T_odom_camera=None: with T=None,
            # _camera_xz_to_odom_xz returns the CAMERA-frame (x, z) unchanged, and command_mux and
            # follow_planner would consume those as world coordinates with no error anywhere on any
            # topic. That is the single most dangerous silent failure in this half of the system.
            self._publish_blind(
                pend.stamp_msg,
                '%s <- %s at the frame stamp: %s (age %.3f s)'
                % (self._world_frame, pend.camera_frame, reason, age))

    # -- publishing ----------------------------------------------------------------------------

    def _publish_blind(self, stamp_msg, reason):
        """Publish an EMPTY TargetArray for a frame we could not evaluate, and say so loudly.

        Publishing is not optional. An empty array is a positive statement that the pipeline is
        alive; SKIPPING the publish is indistinguishable from the pipeline being dead and trips
        command_mux's targets_timeout_s veto into HOLD.
        """
        self._blind_frames += 1
        self._ok_pub.publish(Bool(data=False))
        self.get_logger().error(
            'no world lift for this frame -- publishing an EMPTY TargetArray (%d so far): %s'
            % (self._blind_frames, reason), throttle_duration_sec=LOG_THROTTLE_S)
        # SECOND-ORDER EFFECT, and it needs an owner outside this node: an empty array keeps
        # command_mux's freshness gate satisfied while emptying the proximity veto, so a "blind"
        # state is one where the veto is CONTENT rather than alarmed. The clean place to raise it
        # is /dtp/sensors_ok, which command_mux already treats as a hard veto -- but
        # sensor_watchdog owns that topic, so this is a cross-node requirement this node cannot
        # satisfy alone.
        msg = TargetArray()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self._world_frame
        self._targets_pub.publish(msg)
        if self._want_2d():
            # A blind frame is the one a viewer most needs explained: without the reason the page
            # shows a camera with nobody boxed, which reads as "nobody is there".
            self._publish_2d(stamp_msg, {'ok': False, 'reason': str(reason)[:400], 'dets': []})

    def _want_2d(self):
        return self._det2d_pub is not None and self._det2d_pub.get_subscription_count() > 0

    def _publish_2d(self, stamp_msg, payload):
        """Viewer side channel. Swallows every failure: see DETECTIONS_2D_TOPIC."""
        try:
            payload['stamp'] = [int(stamp_msg.sec), int(stamp_msg.nanosec)]
            self._det2d_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        except Exception as exc:                                        # noqa: BLE001
            self.get_logger().warn('%s not published: %s: %s'
                                   % (DETECTIONS_2D_TOPIC, type(exc).__name__, exc),
                                   throttle_duration_sec=LOG_THROTTLE_S)

    @staticmethod
    def _box_2d(det, target, held, conf_by_id):
        """One row of /dtp/detections_2d."""
        track_id = int(det['track_id'])
        x1, y1, x2, y2 = (float(v) for v in det['bbox_xyxy'])
        row = {
            'id': track_id,
            'box': [_jnum(x1, 1), _jnum(y1, 1), _jnum(x2, 1), _jnum(y2, 1)],
            # Where the range sample is centred -- the same fractions _sample_patch uses. Drawn so
            # a patch sitting on the silhouette edge (the 2026-09-09 range-jump cause) is visible.
            'patch': [_jnum(x1 + PATCH_CENTRE_W_FRAC * (x2 - x1), 1),
                      _jnum(y1 + PATCH_CENTRE_H_FRAC * (y2 - y1), 1)],
            'conf': _jnum(conf_by_id.get(track_id)),
            'confirmed': bool(det.get('track_confirmed')),
            # True when _build_target refused this frame's range and republished the last one.
            'held': bool(held),
        }
        if target is not None:
            row['xyz'] = [_jnum(target.position.x), _jnum(target.position.y),
                          _jnum(target.position.z)]
            row['vel'] = [_jnum(target.velocity.x), _jnum(target.velocity.y)]
            row['dist'] = _jnum(target.distance_m)
        return row

    def _process(self, pend, T_world_cam):
        # process_frame assumes a Y-up world; camera_init is Z-up. See PIPELINE_SWIZZLE. The
        # UNSWIZZLED matrix is kept for the height lift, which needs its real row 2.
        T_pipe = PIPELINE_SWIZZLE @ T_world_cam

        try:
            _frame_out, out = self._pred.process_frame(
                pend.bgr, pend.cloud,
                # ALWAYS passed. Omitted, process_frame defaults to time.time(), which breaks under
                # use_sim_time and desynchronises the CTRV history from every stamp downstream.
                timestamp=pend.stamp_s,
                T_odom_camera=T_pipe)
        except Exception as exc:                                        # noqa: BLE001
            # A detector hiccup must not take the node down -- see _lookup.
            self.get_logger().error('process_frame raised %s: %s' % (type(exc).__name__, exc),
                                    throttle_duration_sec=LOG_THROTTLE_S)
            self._publish_blind(pend.stamp_msg, 'process_frame raised')
            return

        if not out or not out.get('success'):
            self._publish_blind(pend.stamp_msg,
                                'process_frame declined: %s' % (out or {}).get('reason', '?'))
            return

        # output["agv_movement"] is IGNORED, deliberately. It is the ground-robot AVOID policy
        # (stop / slow_down / normal_operation); CLAUDE.md SS7 and plan/dtp-imp/14 both say
        # "invert, don't port". Reading it -- even logging it as a command -- would create a
        # second, contradictory safety opinion alongside command_mux's veto.
        detections = out.get('detections') or []
        conf_by_id = self._confidence_by_track()

        msg = TargetArray()
        msg.header.stamp = pend.stamp_msg
        # The WORLD frame, not the camera frame. follow_planner stamps its own output 'world' and
        # ROG-Map ignores frame_id entirely, so this is documentation rather than plumbing -- which
        # is exactly why it has to be right rather than defaulted.
        msg.header.frame_id = self._world_frame

        live_ids = set()
        want_2d = self._want_2d()
        seen_2d = []
        for det in detections:
            track_id = int(det['track_id'])
            live_ids.add(track_id)
            holds_before = self._track(track_id).holds
            target = self._build_target(det, pend, T_world_cam, T_pipe, conf_by_id)
            if target is not None:
                msg.targets.append(target)
            if want_2d:
                seen_2d.append((det, target, self._track(track_id).holds != holds_before))
        self._forget_missing_tracks(live_ids)

        # MANDATORY even when empty: an empty array says "the pipeline is alive and nobody is in
        # view", and skipping it is indistinguishable from the pipeline being dead -- which trips
        # command_mux into HOLD in exactly the situation (nobody around) where holding is least
        # warranted. command_mux's own start-up comment notes that a never-publishing /dtp/targets
        # pins the mux in HOLD forever.
        self._targets_pub.publish(msg)
        # This frame was actually EVALUATED -- world lift succeeded and the detector ran.
        # That is what makes an empty array above trustworthy rather than merely quiet.
        self._ok_pub.publish(Bool(data=True))

        timings = out.get('timings_ms')
        if timings:
            # The age is the number that degrades first under load and nobody has instrumented it:
            # SUPER hard-gates /planning/yaw_target at kYawTargetTimeoutSec = 0.5 s against
            # getSimTime() - header.stamp, on a stamp that has travelled perception ->
            # target_manager -> follow_planner. Its failure mode is silent (SUPER falls back to
            # velocity-aligned yaw and the drone never looks at the person -- R-F10).
            self.get_logger().info(
                'frame: %d target(s), total %.1f ms (infer %.1f, post %.1f), cloud %.1f ms [%s], '
                'stamp age at publish %.3f s'
                % (len(msg.targets), timings.get('total', float('nan')),
                   timings.get('inference', float('nan')), timings.get('post', float('nan')),
                   pend.cloud_ms, self._cloud_backend, self._now_s() - pend.stamp_s),
                throttle_duration_sec=LOG_THROTTLE_S)

        if want_2d:
            try:
                timings = timings or {}
                self._publish_2d(pend.stamp_msg, {
                    'ok': True,
                    'w': int(pend.bgr.shape[1]), 'h': int(pend.bgr.shape[0]),
                    'dets': [self._box_2d(d, t, held, conf_by_id) for d, t, held in seen_2d],
                    'infer_ms': _jnum(timings.get('inference'), 1),
                    'cloud_ms': _jnum(pend.cloud_ms, 1),
                    'age_s': _jnum(self._now_s() - pend.stamp_s),
                })
            except Exception as exc:                                    # noqa: BLE001
                self.get_logger().warn('%s not built: %s: %s'
                                       % (DETECTIONS_2D_TOPIC, type(exc).__name__, exc),
                                       throttle_duration_sec=LOG_THROTTLE_S)

    # -- per-target construction ---------------------------------------------------------------

    def _sample_patch(self, cloud, box):
        """Nearest-surface (x, y, z) in the CAMERA OPTICAL frame plus the inlier count, or None.

        *** THE PATCH GEOMETRY STAYS BIT-IDENTICAL TO detection.py:432-478. THE STATISTIC OVER IT
        DELIBERATELY DOES NOT, since 2026-09-09. *** The full reasoning and the measurement that
        forced it are at the return statement below.

        What that split means in practice. The GEOMETRY -- which pixels are looked at -- must stay
        identical, because detection.py samples the same patch for horizontal position and this
        function samples it for height, and a geometry divergence would give a person against a
        wall the person's distance and the wall's height. The STATISTIC over those pixels is now
        ours: a plain median cannot choose between two surfaces, and choosing the near one is the
        whole job here.

        The two therefore no longer produce the same number, and that is intended, not drift. It
        is also survivable in a way a geometry divergence would not be, because detection.py's
        published position is EMA-smoothed and motion-limited (detection.py:792-794) while this
        one is raw -- they were never directly comparable after a track's first frame anyway.

        Five things replicated literally, each a silent divergence if changed:
          * int() TRUNCATES toward zero -- round() shifts the centre by up to a pixel on odd boxes.
          * max(1, ...) on half_w/half_h -- without it a small box gets a zero-width patch.
          * the centre is clamped BEFORE px1/px2 are formed, not after.
          * min(w, ...)/min(h, ...) with (h, w) = cloud.shape[:2], in that order.
          * the mask is computed from x and z ONLY. Do NOT add isfinite(y): it would change the
            support set and desynchronise the medians. It is a no-op today, because the cloud is
            range * unit-ray and every pixel's three channels are jointly finite or jointly NaN --
            and it is a silent divergence the day the cloud is built differently.

        The inlier count is the one thing detection.py throws away and the reason this returns it:
        _extract_xz_from_pointcloud's `if not np.any(valid_mask)` means ONE surviving sample
        produces a confident median.
        """
        x1, y1, x2, y2 = map(int, box)
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)

        cx = int(x1 + PATCH_CENTRE_W_FRAC * bw)
        cy = int(y1 + PATCH_CENTRE_H_FRAC * bh)

        # Read off the predictor INSTANCE, not restated: three fewer ways for the two copies of
        # this geometry to drift apart, at zero cost.
        half_w = max(1, int(self._pred.half_patch_scale_w * bw))
        half_h = max(1, int(self._pred.half_patch_scale_h * bh))

        h, w = cloud.shape[:2]
        cx = max(0, min(cx, w - 1))
        cy = max(0, min(cy, h - 1))

        px1 = max(0, cx - half_w)
        px2 = min(w, cx + half_w + 1)
        py1 = max(0, cy - half_h)
        py2 = min(h, cy + half_h + 1)

        patch = cloud[py1:py2, px1:px2, :3]
        if patch.size == 0:
            return None
        patch = patch.reshape(-1, 3)
        xv = patch[:, 0]
        yv = patch[:, 1]
        zv = patch[:, 2]

        mask = np.isfinite(xv) & np.isfinite(zv) & (zv > 0.0)
        if self._pred.max_depth_m is not None:
            mask &= (zv <= self._pred.max_depth_m)

        n_valid = int(np.count_nonzero(mask))
        if n_valid == 0:
            return None
        # NEAREST-SURFACE SELECTION, then medians WITHIN that surface.
        #
        # WHY THIS IS NO LONGER detection.py's PLAIN MEDIAN. Measured on the aircraft 2026-09-09
        # over a 194 s walk (bag_follow2): the range to the subject jumped 1-2 m between
        # consecutive frames and the per-track range gate rejected 62 % of frames -- 117
        # range-jump holds against 21 inlier holds. Each hold republishes the last position, so
        # velocity read a median 0.03 m/s while the subject walked at ~1.4 m/s, the forecast
        # collapsed to a single 0.20 s point, the coast died in 0.11 s, and target_manager sat in
        # LOST for the entire run. One statistic, five symptoms.
        #
        # The mechanism: a MEDIAN rejects outliers WITHIN one surface but cannot choose BETWEEN
        # two. The patch sits at 0.65 of the box height, and at close range a large box makes it
        # straddle the subject's silhouette edge, so the median flips between the person (~2.5 m)
        # and the wall behind them (~4+ m). Held-frame rate was 79.8 % at 2-3 m against 3-5 % at
        # 1-2 m and 5-6 m -- NOT monotonic in range, which is what rules out sparsity and points
        # at edge-straddling.
        #
        # In a box drawn around a person, the person is the NEAREST thing in it, and the
        # background is exactly what the median keeps drifting onto. So: take a low percentile to
        # locate the near surface, keep only samples within a person's own depth extent of it, and
        # take medians there. Robust to background bleed AND to speckle, because within the
        # selected surface it is still a median.
        #
        # THIS DELIBERATELY DIVERGES FROM detection.py:478, which the previous docstring required
        # it to match bit-for-bit. That requirement existed so height and position would describe
        # the same surface. They still do -- x, y and z all come from one selected support -- but
        # the support is now ours rather than the ported module's. detection_pipeline/ stays
        # unchanged (CLAUDE.md section 7); we simply stop using its range extraction, which was
        # written for a ground robot and a different sensor geometry. _build_target must therefore
        # publish OUR position, not det['position_xz_m'], or the two disagree at every edge.
        zsel = zv[mask]
        z_near = float(np.percentile(zsel, NEAR_SURFACE_PCT))
        near = mask.copy()
        near[mask] = zsel <= (z_near + NEAR_SURFACE_BAND_M)
        n_near = int(np.count_nonzero(near))
        if n_near == 0:                      # a percentile is a real sample, so this cannot fire;
            near, n_near = mask, n_valid     # the cost of being wrong is a crash in a callback
        return (float(np.median(xv[near])), float(np.median(yv[near])),
                float(np.median(zv[near])), n_near)

    def _build_target(self, det, pend, T, T_pipe, conf_by_id):
        track_id = int(det['track_id'])
        state = self._track(track_id)

        px = float(det['position_xz_m']['x'])       # world X (swizzled)
        py = float(det['position_xz_m']['z'])       # world Y -- process_frame's "z" is our Y
        distance_m = float(det['distance_to_agv_m'])

        sample = self._sample_patch(pend.cloud, det['bbox_xyxy'])
        ranged = sample is not None
        reason = 'no valid samples in the torso patch'

        if ranged:
            x_cam, y_cam, z_cam, n_valid = sample
            if n_valid < self._min_patch_samples:
                ranged = False
                reason = '%d inliers < %d' % (n_valid, self._min_patch_samples)
            # The same _reject_far_points gate detection.py:413 applies -- to the MEDIAN, not per
            # sample. Redundant today (every masked sample already satisfies it, so the median
            # must too) and kept because it documents the coupling: if the mask above ever changes,
            # this is what stops a rejected median reaching the height lift.
            elif not (math.isfinite(x_cam) and math.isfinite(z_cam)) or z_cam <= 0.0 or (
                    self._pred.max_depth_m is not None and z_cam > self._pred.max_depth_m):
                ranged = False
                reason = 'median z=%.2f outside the depth gate' % z_cam
            else:
                accepted, innov, gate = state.accept_range(
                    math.sqrt(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam), pend.stamp_s)
                if not accepted:
                    ranged = False
                    reason = ('range jumped %.2f m against a %.2f m gate (%d consecutive)'
                              % (innov, gate, state.reject_run))

        if ranged:
            self._assert_patch_identity(track_id, det, T_pipe, x_cam, z_cam, px, py)
            # FULL 3-D lift with the UNSWIZZLED matrix. T[2,1] is the whole point: it is the one
            # element process_frame throws away -- the camera-y column of the world-Z row -- and
            # discarding it is literally the "XZ-only height-discard" CLAUDE.md SS7 and
            # plan/dtp-imp/14:165 both say to leave behind.
            #
            # Camera-frame y is DOWN (depth_source un-projects [(u-cx)z/fx, (v-cy)z/fy, z] with v
            # increasing down the image), so a torso sample BELOW the optical axis has y_cam > 0.
            # THERE IS NO SIGN FLIP IN THIS FORMULA. The sign is carried entirely by T[2,1], which
            # for a level forward-looking camera is about -1. If you find yourself inserting a
            # manual minus sign, the rotation is wrong -- go fix the rotation.
            world_z = (T[2, 0] * x_cam + T[2, 1] * y_cam + T[2, 2] * z_cam + T[2, 3])
            if state.has_height:
                # Clamp, then EMA -- the same order detection.py applies to x/z, and the same
                # alpha read off the predictor so height does not jitter relative to horizontal
                # position. The three components still have slightly different lag (x/y come from
                # process_frame's own smoothed history, z is smoothed here), which the freeze
                # leaves no way to avoid; matching the alpha is the closest available approximation
                # and is why it must be that number and not a fresh one.
                step = min(MAX_STEP_Z_M, max(-MAX_STEP_Z_M, world_z - state.world_z))
                limited = state.world_z + step
                alpha = self._pred.smooth_alpha
                state.world_z = alpha * limited + (1.0 - alpha) * state.world_z
            else:
                state.world_z = world_z
                state.has_height = True
            state.last_xy = (px, py)
        else:
            state.holds += 1
            self.get_logger().warn(
                'track %d unranged this frame (%s) -- holding the last published position '
                '(%d holds on this track so far). NOTE process_frame has ALREADY folded this '
                "frame's sample into tracks_xz, the EMA and the motion clamp; the gate cannot run "
                'before it because detection_pipeline/ is frozen. The predictor history stays '
                'contaminated and its CTRV forecast is built on it -- this gate holds what WE '
                'publish, it does not clean what the predictor remembers.'
                % (track_id, reason, state.holds),
                throttle_duration_sec=LOG_THROTTLE_S)
            if state.last_xy is not None:
                px, py = state.last_xy
                # Recomputed so distance and position agree: det['distance_to_agv_m'] belongs to
                # the position we are declining to publish.
                distance_m = float(math.hypot(px - T[0, 3], py - T[1, 3]))
            # else: nothing held yet, so publish what process_frame produced. An omitted person is
            # INVISIBLE to command_mux's proximity veto, which is the fail-open direction; a
            # thinly-sampled one is merely mis-placed. Fail toward "someone is there".

            if not state.has_height:
                if not state.height_fallback_logged:
                    state.height_fallback_logged = True
                    self.get_logger().info(
                        'track %d has never had a height; assuming the torso is at camera '
                        'altitude (%.2f m). Bounded and finite, and wrong by at most the '
                        'vertical offset -- which decision D6 (matched-height horizontal '
                        'trail) keeps small by design.' % (track_id, float(T[2, 3])))
                # `has_height` is deliberately NOT set. This is a value to PUBLISH, not a filter
                # state to smooth from: latching it would make the first real height arrive through
                # the MAX_STEP_Z_M clamp and the EMA, starting from camera altitude -- at 0.20 m of
                # step and alpha 0.25 that is a second of ramp toward a number we could have taken
                # raw. The contract's rule is "on the first sighting take the raw value", and the
                # first sighting is the first RANGED frame, not the first frame.
                state.world_z = float(T[2, 3])

        if ranged:
            vx, vy = self._velocity(det.get('history_xz') or [])
            state.last_velocity = (vx, vy)
        else:
            # history_xz already contains the sample we just rejected -- process_frame appended it
            # before we could object. Differencing it would turn the very glitch we are holding out
            # into a velocity spike, and that spike would then be extrapolated into
            # predicted_path[0]. Hold the last velocity computed on an accepted frame instead.
            vx, vy = state.last_velocity

        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(state.world_z)):
            # Never publish a non-finite position: command_mux._person_points returns None on one,
            # which _proximity_violation converts into "cannot clear a person we cannot locate" ->
            # a permanent, unrecoverable HOLD. Our construction cannot produce one (the fallbacks
            # are all finite), so this firing means an assumption above is broken.
            self.get_logger().error(
                'track %d produced a non-finite position (%.3f, %.3f, %.3f) -- dropping it from '
                'this TargetArray rather than pinning command_mux in HOLD forever'
                % (track_id, px, py, state.world_z), throttle_duration_sec=LOG_THROTTLE_S)
            return None

        tgt = Target()
        tgt.header.stamp = pend.stamp_msg
        tgt.header.frame_id = self._world_frame
        tgt.track_id = max(0, track_id)
        tgt.position.x = px
        tgt.position.y = py
        tgt.position.z = float(state.world_z)
        tgt.velocity.x = vx
        tgt.velocity.y = vy
        # DELIBERATE: nothing in this pipeline estimates a human's vertical speed, follow_planner
        # uses only hypot(vx, vy), and a Z derived by differencing our own EMA'd height would be
        # noise dressed as a measurement.
        tgt.velocity.z = 0.0
        tgt.track_confirmed = bool(det['track_confirmed'])
        tgt.missed_frames = max(0, int(det['missed_frames']))
        tgt.confidence = float(conf_by_id.get(track_id, self._conf_floor))
        # HORIZONTAL ground-plane distance, which is what process_frame computes and what
        # command_mux's veto -- an explicit CYLINDER: horizontal radius plus a separate
        # proximity_veto_half_height_m test -- composes with. Target.msg calls this "range to the
        # drone", which reads as 3-D and is wrong as written; correct it there.
        tgt.distance_m = float(distance_m)
        # target_manager owns the lock and holds the single _locked_track_id; this node does not
        # know it. Feeding the lock back so /dtp/targets carried the flag would create a cycle
        # between the two nodes and give command_mux's INDEPENDENT veto a data dependency on the
        # follow logic it exists to be independent of. command_mux never reads the flag anyway.
        tgt.is_locked_target = False

        # On a held frame the predictor's forecast is anchored to the position we just declined to
        # publish, and follow_planner takes predicted_path.front() as its reference point -- so
        # forwarding it would fly the drone to the rejected position anyway and defeat the hold
        # entirely. Passing an empty forecast falls through to constant velocity from the HELD
        # position, which is the only re-anchor available without re-running the CTRV ourselves.
        forecast = (det.get('predictions_xz') or []) if ranged else []
        points, dts = self._predicted_path(
            forecast, pend.stamp_s, px, py, float(state.world_z), vx, vy)
        tgt.predicted_path = points
        tgt.predicted_path_dt = dts
        return tgt

    def _assert_patch_identity(self, track_id, det, T_pipe, x_cam, z_cam, px, py):
        """Runtime tripwire on the duplicated patch GEOMETRY. See PATCH_GEOMETRY_DRIFT_TOL_M.

        Only checkable on a track's FIRST frame, which is the only frame where process_frame
        applies neither its EMA nor its motion clamp -- after that the two quantities legitimately
        differ and no assertion is possible. `history_xz` has exactly one entry on that frame.

        This is the only thing that will notice `half_patch_scale_h` or the 0.65 centre fraction
        changing upstream. Without it the divergence shows up as "the drone sits a bit high", which
        reads as a tuning problem and gets tuned instead of fixed.
        """
        if len(det.get('history_xz') or []) != 1:
            return
        ex = T_pipe[0, 0] * x_cam + T_pipe[0, 2] * z_cam + T_pipe[0, 3]
        ey = T_pipe[2, 0] * x_cam + T_pipe[2, 2] * z_cam + T_pipe[2, 3]
        err = max(abs(ex - px), abs(ey - py))
        # SINCE 2026-09-09 A SMALL DISAGREEMENT IS EXPECTED, not a fault: _sample_patch selects
        # the NEAREST surface in the patch while detection.py takes a plain median over all of it,
        # so wherever the patch straddles an edge the two legitimately differ by roughly the
        # foreground-to-background separation. That divergence is the fix, not the bug.
        #
        # What this tripwire still catches, and is still worth keeping for, is GEOMETRY drift --
        # half_patch_scale_h or the 0.65 centre fraction changing upstream. That moves the patch
        # onto different pixels entirely and produces a disagreement far larger than any surface
        # selection can explain. Hence the wider tolerance and the softened wording: the old
        # threshold would now fire on every edge-straddling frame and be tuned away, taking the
        # geometry check with it.
        if err > PATCH_GEOMETRY_DRIFT_TOL_M:
            self.get_logger().warn(
                'track %d: our torso patch and _extract_xz_from_pointcloud disagree by %.2f m on '
                'the first frame -- larger than nearest-surface selection can account for. Check '
                'whether the patch GEOMETRY in detection.py:432-478 and _sample_patch has drifted '
                '(half_patch_scale_*, the 0.65 centre fraction, the clamp order); a statistic '
                'difference alone should not reach this.' % (track_id, err),
                throttle_duration_sec=LOG_THROTTLE_S)

    def _velocity(self, history):
        """(vx, vy) in the world ground plane. EXACTLY zero -- never NaN -- on thin history.

        follow_planner computes heading_az = atan2(v.y, v.x) and gates on
        hypot(vx, vy) > kHeadingSpeedFloorMps, so a noisy velocity spins the standoff viewpoint
        around a person who is standing still. A raw two-point difference does exactly that: it is
        dominated by depth noise. This is the recipe detection.py:300-312 uses internally for its
        speed estimate, applied per component, so the heading follow_planner derives from velocity
        agrees with the heading the CTRV path implies.
        """
        if len(history) < 2:
            return 0.0, 0.0
        # FINITE FILTER FIRST, THEN THE 25-SAMPLE WINDOW -- that order, because it is
        # _predict_future_xz's order (detection.py:274-280). Slicing first would let a single
        # non-finite entry shorten the window instead of being skipped over, and the two estimates
        # would then be built from different spans of the same history. It is a no-op today
        # (_reject_far_points drops non-finite samples before tracks_xz ever sees them) and it is a
        # silent divergence the day that changes.
        data = np.asarray([(h['x'], h['z'], h['t']) for h in history], dtype=np.float64)
        data = data[np.isfinite(data).all(axis=1)]
        data = data[-VELOCITY_HISTORY_N:]
        if len(data) < 2:
            return 0.0, 0.0

        t = data[:, 2]
        keep = np.ones(len(t), dtype=bool)
        keep[1:] = np.diff(t) > DT_EPS_S
        data = data[keep]
        if len(data) < 2:
            return 0.0, 0.0

        dt = np.diff(data[:, 2])
        if len(dt) < MIN_VELOCITY_DIFFS:
            # Same floor _predict_future_xz applies before it will estimate a speed at all
            # (detection.py:299). Below it, reporting a velocity would hand follow_planner a
            # standoff heading that the CTRV path it is following does not agree with.
            return 0.0, 0.0

        xs = self._ema_1d(data[:, 0], alpha=VELOCITY_EMA_ALPHA)
        ys = self._ema_1d(data[:, 1], alpha=VELOCITY_EMA_ALPHA)
        vx_inst = np.diff(xs)[-VELOCITY_SAMPLES:] / dt[-VELOCITY_SAMPLES:]
        vy_inst = np.diff(ys)[-VELOCITY_SAMPLES:] / dt[-VELOCITY_SAMPLES:]

        vx = self._trimmed_mean(vx_inst, trim_ratio=VELOCITY_TRIM_RATIO)
        vy = self._trimmed_mean(vy_inst, trim_ratio=VELOCITY_TRIM_RATIO)
        if vx is None or vy is None or not (math.isfinite(vx) and math.isfinite(vy)):
            return 0.0, 0.0

        speed = math.hypot(vx, vy)
        if speed > MAX_HUMAN_SPEED_MPS:
            scale = MAX_HUMAN_SPEED_MPS / speed
            vx, vy = vx * scale, vy * scale
        return float(vx), float(vy)

    def _predicted_path(self, preds, stamp_s, px, py, world_z, vx, vy):
        """(points, dts) meeting the change-C5 contract. dts[0] IS latency_compensation_s.

        Three things that will bite, in order:

        a. The predictor's grid does not land on 0.20 s. _predict_future_xz emits 20 points spaced
           prediction_secs / prediction_steps = 0.075 s, so the polyline is interpolated at exactly
           the compensation and the native samples beyond it are emitted after, each with its own
           true dt -- which preserves both the contract and the predictor's own resolution (which
           follow_planner's LeadAnchor consumes as arc length).
        b. The forecast can be STALE and its timestamps are ABSOLUTE. do_predict requires
           confirmed AND depth <= prediction_depth_gate_m AND frame_idx % predict_every_n == 0;
           otherwise process_frame reuses last_predictions[track_id] carrying its ORIGINAL absolute
           timestamps. So dt is always computed as t - header stamp, never assumed to be
           0.075 * (i + 1), and stale points (dt <= 0) are dropped here rather than relying on
           command_mux to clean our output.
        c. z is the target's CURRENT world Z, held constant. The forecast is a 2-D ground-plane
           CTRV and there is no vertical model; a fabricated one would feed follow_planner's
           goal_height_offset_m an invented altitude. Constant Z is also the conservative input to
           command_mux's +/- proximity_veto_half_height_m cylinder test.
        """
        lat = self._latency_s
        rel = []
        for p in preds:
            dt = float(p['t']) - stamp_s
            x, y = float(p['x']), float(p['z'])
            if math.isfinite(dt) and dt > 0.0 and math.isfinite(x) and math.isfinite(y):
                rel.append((dt, x, y))
        rel.sort(key=lambda r: r[0])

        head = None
        tail = []
        for i, (dt, x, y) in enumerate(rel):
            if dt < lat - DT_EPS_S:
                continue
            if dt <= lat + DT_EPS_S:
                head = (x, y)
                tail = rel[i + 1:]
            else:
                prev_dt, prev_x, prev_y = (rel[i - 1] if i > 0 else (0.0, px, py))
                span = dt - prev_dt
                f = 0.0 if span <= DT_EPS_S else (lat - prev_dt) / span
                head = (prev_x + f * (x - prev_x), prev_y + f * (y - prev_y))
                tail = rel[i:]
            break

        if head is None:
            # Fully stale forecast, or none at all. Constant velocity for the one point the
            # contract requires, and NOTHING after it: inventing more than the predictor offered is
            # flying on a forecast nobody made, which is the argument LeadAnchor's own comment
            # makes. Both consumers handle a short path correctly.
            head = (px + vx * lat, py + vy * lat)
            tail = []

        points = []
        dts = []
        for dt, x, y in [(lat, head[0], head[1])] + tail:
            # Strictly increasing, all finite. command_mux WARNS and discards the WHOLE forecast on
            # a length mismatch, which silently downgrades the veto to "now" only; follow_planner
            # drops the whole Target on a non-finite point.
            if dts and dt <= dts[-1] + DT_EPS_S:
                continue
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = world_z
            points.append(pt)
            dts.append(float(dt))
        return points, dts

    def _confidence_by_track(self):
        """{track_id: detector confidence} for the frame just inferred, or {} if unavailable.

        _parse_yolo_results (detection.py:646-665) keeps only track_id and box -- it DROPS
        boxes_obj.conf entirely -- and detection_pipeline/ is frozen, so the only route to a real
        number is reading the results object the predictor already retains on
        self.last_infer_results (detection.py:686). That is a READ of a public attribute of the
        frozen module, not an edit.

        FRAGILE COUPLING, deliberately taken: it depends on ultralytics' Results shape and on
        detection.py continuing to cache last_infer_results. Hence the broad except and the fall
        back to the conf threshold, an honest lower bound carrying zero information. Acceptable
        because nothing downstream currently READS confidence (command_mux and follow_planner both
        checked) -- it exists for logging, target_manager's cheap tie-break (plan/dtp-imp/04 SS2
        rule 3) and future gating. On a non-inference frame the value would be the previous
        inference frame's; detect_every_n=1 above means every processed frame is an inference
        frame, so today it is per-frame.
        """
        try:
            results = self._pred.last_infer_results
            if not results:
                return {}
            boxes = results[0].boxes
            if boxes is None or boxes.id is None:
                return {}
            ids = boxes.id.int().cpu().tolist()
            confs = boxes.conf.float().cpu().tolist()
            return {int(i): float(c) for i, c in zip(ids, confs)}
        except Exception as exc:                                        # noqa: BLE001
            self._warn_once('conf-scrape',
                            'cannot read confidence off predictor.last_infer_results (%s: %s); '
                            'falling back to the conf threshold %.2f for every target'
                            % (type(exc).__name__, exc, self._conf_floor))
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # Constructed inside the try so that a start-up refusal -- calib_io's camera-identity gate,
        # a missing engine, an unknown cloud_source -- still runs the shutdown below instead of
        # leaving the context initialised.
        node = PerceptionNode()
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
