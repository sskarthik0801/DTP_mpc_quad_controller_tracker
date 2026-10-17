"""fusion_node -- an OFF-BY-DEFAULT debug view of the organized cloud, for RViz, bags and replay.

*** THIS NODE IS NOT IN THE PERCEPTION PATH AND MUST NOT BE PUT BACK INTO IT. ***

It used to be the thing that built the organized H x W x 3 cloud for the whole stack. It is now a
DEBUG PUBLISHER and nothing else: perception_node builds the identical array in its own process
(see the arithmetic below), and both sides call the same dtp_perception.organized_cloud so a bag
recorded here deprojects exactly the way the flight path does. The projection, deskew, accumulation
and z-buffering that this docstring used to list as future work live in dtp_perception/depth_source
and are done.

Still future work (plan/dtp-imp/06 M1.2/M1.4):
  - fuse the two depth sources rather than choosing one, if the outdoor test says neither alone is
    enough: camera stereo stays usable to ~8-10 m and must NOT be NaN-ed beyond 4 m (risk A9), and
    LiDAR is reliably useful to ~15 m on a person, not 40 m.

THREE GATES, AND WHY ONE IS NOT ENOUGH
--------------------------------------
  1. `enable` (default **False**) -- when false this node constructs NOTHING: no calibration load,
     no DepthSource, no colour subscription, no publisher. It logs why and idles.
  2. `publish_rate_hz` (default DEBUG_PUBLISH_HZ = 2.0) -- 0.0 disables publishing while keeping
     the DepthSource open, which is useful for a bring-up check.
  3. `get_subscription_count() == 0` -- nothing is built while nobody is looking.

Gate 1 exists because gates 2 and 3 CANNOT DO ITS JOB. Both of them stop the node PUBLISHING;
neither stops it SUBSCRIBING. A constructed CameraDepthSource pulls /camera/depth/image_raw at
about 24 MB/s and the colour stream on top of that, forever, in a Python process, for a topic
nobody reads -- and on this box a Python subscriber on a multi-MB/s topic drops frames
(plan/dtp-imp/21), so an idle-but-constructed fusion_node is not free, it is a competitor to
perception_node for the same frames. Gate 1 is also the one that is answerable from a launch file:
"is this in the flight graph" should not depend on who happened to subscribe.

WHY THE PERCEPTION PATH DOES NOT CONSUME /dtp/points_fused
-----------------------------------------------------------
One organized cloud at colour resolution is 1280 x 720 x 3 x 4 B = **11.06 MB per message**. At the
camera's 29.98 Hz that is 332 MB/s through DDS; even at a 15 Hz detection rate it is 166 MB/s. This
box has already been measured failing an order of magnitude below that: a 2.76 MB raw colour
message at 30 Hz (~83 MB/s) lost ~8 % of frames in an rclpy subscriber (plan/dtp-imp/21,
2026-09-07), and the fix was to stop sending it. There is no compressed transport for a
PointCloud2 (image_io's docstring says so), so the same ceiling applies here with 4x the message
and no escape hatch -- and under BEST_EFFORT an 11 MB sample fragments into thousands of UDP
datagrams, where losing any one of them loses the whole sample.

So `perception_node` builds the identical (H, W, 3) array IN ITS OWN PROCESS from its own
DepthSource and hands the numpy array straight to `process_frame`, with no serialization at all.
That is what plan/dtp-imp/14 C3 (RED), risk A4 (RED), plan/dtp-imp/06 M1.4.1 and
plan/dtp-imp/07:151 all already prescribe ("consolidate perception into a single process"); it is
a change to plan/dtp-imp/01 SS2's node/topic table and needs recording there, the same way the
LiDAR subscription moving into depth_source did.

What survives in this node is the RViz / `ros2 bag` / lidar_overlay_node view and the
record-once/replay-many harness (plan/dtp-imp/03 SS5), for which a few frames a second is plenty --
hence DEBUG_PUBLISH_HZ and the three gates above. The PointCloud2 layout is still a CONTRACT even
though nobody consumes it in flight, because the harness replays these bags into `process_frame`.

THE ARRAY ON THE WIRE
---------------------
Defined once, in dtp_perception/organized_cloud.py, which owns both the deprojection and the
PointCloud2 layout -- axes, the NaN convention, the 12-byte stride, is_dense=False, and the
`array.array('B', ...)` fast path. Read that file, not this one, before writing a consumer;
`organized_cloud.from_pointcloud2` is the verified reader and refuses a layout it cannot vouch for.
Nothing about the wire format is decided in this file any more, which is the point: a bag recorded
here must deproject exactly the way perception_node's in-process path does, and the only way to
guarantee that is for there to be one implementation.

WHAT CHANGED, AND WHY THE LiDAR SUBSCRIPTION LEFT THIS FILE
-----------------------------------------------------------
Depth ingestion now belongs to `dtp_perception.depth_source`, selected by the `depth_source`
parameter ('camera' or 'lidar'). This node no longer subscribes `/livox/lidar` itself.

That is a deliberate change to plan/dtp-imp/01 SS2's node/topic table and it needs to be reflected
there. Two reasons it is the right one:

  * The choice between the camera's own stereo depth and the LiDAR projected through our extrinsic
    is NOT settled. Measured indoors 2026-09-08 the camera returns 88.4 % of pixels at 30 Hz
    against the LiDAR's 13.5 % coverage in-FOV, and it does so through the FACTORY depth<->colour
    extrinsic, which survives any remount. But the 336L is ACTIVE stereo and direct sunlight
    defeats its IR projector, and the outdoor test (scripts/outdoor_depth_test.py) has not been
    run. The abstraction exists so that result selects a config value instead of forcing a
    redesign of this node.
  * When the LiDAR source IS selected it consumes `/cloud_registered_body`, not `/livox/lidar` --
    FAST-LIO's DESKEWED body-frame cloud. This node's previous docstring already required
    "FAST-LIO2-motion-compensated (deskewed)" points; the raw driver sweep is not deskewed, and
    FAST-LIO computes exactly what was being asked for anyway.

Deliberately NOT this node's job (lands in perception_node instead, Phase 1 M1.1/M1.4):
  per-track "trunk hijacking" robustness -- rejecting a range sample that's inconsistent with
  a SPECIFIC track's recent motion needs a track to exist first, so it's a per-track Kalman
  gate downstream of detection, not something this node (which runs before detection) can do.
  See CLAUDE.md SS5 and plan/dtp-imp/14 SS3.4 for the split rationale.

Topics (plan/dtp-imp/01 SS2, node/topic contract table):
  Subscribes: /camera/color/image_raw/compressed (CompressedImage) by default,
              plus whatever the selected depth source needs, on this same node:
                depth_source=camera -> /camera/depth/image_raw, /camera/depth/camera_info,
                                       /camera/color/camera_info, /tf_static
                depth_source=lidar  -> /cloud_registered_body, /lidar_slam/odom,
                                       /camera/color/camera_info
  Publishes:  /dtp/points_fused (sensor_msgs/PointCloud2, organized H x W x 3, colour optical
              frame -- same layout the reference repo's detection_pipeline.process_frame expects,
              so a replayed bag drives it with no change). Throttled: see DEBUG_PUBLISH_HZ.
  Nothing at all when `enable` is false, which is the default.
"""
import math

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from dtp_perception import calib_io, depth_source, image_io, organized_cloud

# ---------------------------------------------------------------------------------------------
# Publish rate. This is a DEBUG topic and the arithmetic in the module docstring is the reason:
# 11.06 MB per message against a transport this box was measured losing frames on at 83 MB/s.
# ---------------------------------------------------------------------------------------------
# 2 Hz is 22 MB/s -- enough to watch a cloud in RViz or to record a replay bag, and low enough that
# it cannot starve FAST-LIO2, which is the dominant crash mode this stack has (risk R-F11).
#
# The CPU side of one frame, timed on a development x86 host (NOT the NX, so treat it as a floor):
# ~15-17 ms in organized_cloud.build for the 11 MB array, ~3 ms tobytes, ~3 ms into array.array --
# ~25 ms, i.e. ~5 % of a core at 2 Hz here and some larger fraction on the aircraft. That is
# affordable for a topic nobody subscribes to in flight and would not be at 30 Hz.
#
# Raise it only with a measurement in hand. Set publish_rate_hz to 0.0 to disable publishing
# outright; the node then still holds the DepthSource open, which is useful for a bring-up check.
# To make it cost nothing at all, leave `enable` false -- see the module docstring's three gates.
DEBUG_PUBLISH_HZ = 2.0

# ---------------------------------------------------------------------------------------------
# The array and the wire layout live in dtp_perception/organized_cloud.py.
# ---------------------------------------------------------------------------------------------
# Deliberately NOT restated here. This node and perception_node must produce byte-identical clouds
# -- the replay harness records bags from THIS topic and drives `process_frame` with them, so a
# divergence between the two deprojections would make a bag test pass while the flight path was
# wrong. Two copies of the deprojection with no compiler to notice them drifting is precisely the
# duplication hazard the perception contract calls out for the torso patch; there is no reason to
# create a second instance of it here when one function serves both.
#
# organized_cloud owns: unit rays (range is EUCLIDEAN, not Z), NaN in all three channels for a
# no-return pixel, the 12-byte point_step, is_dense=False, and the array.array('B') fast path.

# Refuse a frame whose depth and colour stamps disagree by more than this.
#
# The two streams are NOT hardware-locked (`time_sync_en: false`, risk A8): colour runs at 29.98
# Hz, depth at 29.96 Hz, and the phase between them has never been measured. range_image() returns
# the newest depth frame it holds regardless of which colour frame we are answering, so the cloud
# we stamp with a colour stamp was in general built from a slightly different instant.
#
# 0.025 s is 3/4 of a 30 Hz period. At 1.4 m/s a person moves 3.5 cm in it, which is under the
# depth noise; a 100 ms skew during a 3 rad/s yaw is 0.3 rad of geometric disagreement between a
# YOLO box and the cloud it indexes. perception_node's in-process path must use the same number, so
# it is named here in the same terms rather than left as a literal in two files.
MAX_DEPTH_COLOUR_SKEW_S = 0.025

# ---------------------------------------------------------------------------------------------
# Instrumentation
# ---------------------------------------------------------------------------------------------
# How often to report cloud FILL -- the fraction of the organized cloud that is not NaN.
#
# This is NOT the 88.4 % depth-pixel validity measured on 2026-09-08 and the difference decides
# whether a torso patch has any samples at all. Depth is 848x480 (407k pixels), colour is 1280x720
# (922k), so at stride 1 and perfect validity under 45 % of the organized cloud can be non-NaN, and
# at stride 2 under 10 % (the "89899 pts in view" in plan/dtp-imp/21). At 8 m the torso patch is
# ~133 px, so 10 % fill is ~13 samples -- and detection.py's _extract_xz_from_pointcloud has NO
# minimum-inlier gate, so a handful of those landing on the wall behind moves the median outright.
# Logging it puts the number on the record before plan/dtp-imp/06 M1.2's hit-count gate is run.
FILL_LOG_PERIOD_S = 10.0


def _stamp_seconds(header) -> float:
    """std_msgs/Header -> float seconds. Duplicated from depth_source rather than imported: that
    one is module-private there, and reaching across for it would make a private name part of this
    file's contract."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # enable: the master switch, DEFAULT FALSE. See the module docstring's three gates.
        #
        # False constructs NOTHING below this point -- and that is the difference between this and
        # publish_rate_hz:=0.0 or having no subscribers, both of which only stop the node
        # PUBLISHING. Constructing the DepthSource opens /camera/depth/image_raw (~24 MB/s) and the
        # colour stream, and on this box a Python subscriber on a multi-MB/s topic drops frames, so
        # a constructed-but-idle fusion_node competes with perception_node for exactly the frames
        # perception_node needs. It also skips calib_io.load_for_node, which is right: that call
        # spends up to 5 s on DDS discovery and can RAISE on a camera/LiDAR identity mismatch, and
        # a node that is doing nothing must not be able to fail launch.
        self.declare_parameter('enable', False)
        self._enabled = bool(self.get_parameter('enable').value)
        if not self._enabled:
            # Say it at INFO, once, with the switch spelled out. The alternative symptom is an
            # empty /dtp/points_fused an hour into a debugging session with nothing explaining it.
            self.get_logger().info(
                'fusion_node is DISABLED (enable:=false, the default) -- no depth subscription, '
                'no cloud, no /dtp/points_fused. This is the normal flight-graph state: '
                'perception_node builds the identical cloud in-process (11.06 MB per message is '
                'not sendable, see the module docstring). Start with enable:=true only to look at '
                'the cloud in RViz or to record a replay bag.')
            return

        # calib_file: the T_camera_lidar extrinsic + intrinsics, produced by the
        # direct_visual_lidar_calibration workflow (plan/dtp-imp/14 SS3.4) and gated by
        # dtp_perception.calibration.score_alignment.
        # A BARE FILENAME is the default on purpose -- calib_io resolves it against the package
        # SHARE directory. This used to default to the relative 'calib/drone_cam_livox.yaml',
        # which resolves against the process working directory and so never resolved from the
        # install space on the aircraft. Pass an absolute path to override.
        # max_depth: reject depth returns beyond this range (m), whichever sensor produced them.
        self.declare_parameter('calib_file', calib_io.DEFAULT_CALIB)
        self.declare_parameter('max_depth', 15.0)  # LiDAR reliable ~15m on a person, not 40m (A9)

        # depth_source: WHERE range comes from. 'camera' is the default on the 2026-09-08 indoor
        # measurement -- 88.4 % pixel validity at 30 Hz through the factory depth<->colour
        # extrinsic, so no calibration of ours is in the loop and a remount cannot invalidate it,
        # against 13.5 % in-FOV coverage from the LiDAR with nothing in the lower third.
        # The default is provisional BY DESIGN. The 336L is active stereo and sunlight defeats its
        # IR projector; if scripts/outdoor_depth_test.py shows depth collapsing outdoors, flip this
        # to 'lidar' and accept that every remount then costs a recalibration. That switch is the
        # entire reason this is a parameter and not an import.
        self.declare_parameter('depth_source', 'camera')
        # How long to block at start-up waiting for the source's inputs. Not fatal on timeout:
        # sensor drivers legitimately start after us, and refusing to launch on start-up ordering
        # would make bring-up brittle for no safety gain (same rule calib_io.check_camera_serial
        # applies). The source simply reports no range until its inputs arrive.
        self.declare_parameter('depth_ready_timeout_s', 5.0)

        # depth_stride: deproject every Nth DEPTH pixel when building the cloud.
        #
        # depth_source defaults this to 2, and its reasoning is sound FOR ITS OWN QUERY: 102k
        # points is far more than the tens of samples range_in_roi's torso statistic needs. That
        # reasoning does NOT carry to the organized cloud. Stride 2 projects at most 424x240 =
        # 101,760 points into 921,600 colour pixels, so under 10 % of the cloud can ever be non-NaN
        # however good the depth is -- which at 8 m leaves a torso patch of ~13 samples for a
        # median that has no minimum-inlier gate behind it. Stride 1 costs a 407k-point
        # deprojection and z-buffer argsort per PUBLISHED frame instead of 102k -- and
        # range_image() is lazy, so at DEBUG_PUBLISH_HZ that is twice a second, not thirty times.
        # Not measured on the NX.
        #
        # Ignored by the LiDAR source, which has no pixel grid to stride over -- which is exactly
        # why the number is declared here rather than changed in depth_source: it is only
        # defensible for the camera path.
        self.declare_parameter('depth_stride', 1)
        # publish_rate_hz: see DEBUG_PUBLISH_HZ. 0.0 disables the publish entirely.
        self.declare_parameter('publish_rate_hz', DEBUG_PUBLISH_HZ)

        self._max_depth = float(self.get_parameter('max_depth').value)
        # Normalised the same way depth_source.create normalises it, so the branch below dispatches
        # on the same string create() will. create() still raises on an unknown name -- it must,
        # because a silent fallback would run one sensor while the log claims the other.
        kind = str(self.get_parameter('depth_source').value).strip().lower()

        # Loaded unconditionally, even when depth_source is 'camera' and the extrinsic goes unused.
        # load_for_node is not just a file read -- it is the start-up identity gate: it refuses to
        # run a calibration that belongs to a DIFFERENT camera body (two Orbbec bodies are attached
        # to this NX and the driver binds whichever it enumerates first), and it refuses a LiDAR
        # stream coming from another robot on the same DDS domain. The camera half of that is
        # load-bearing in BOTH modes, because the intrinsics belong to a specific body. Skipping
        # the load to make camera mode start faster would drop that gate silently.
        self._T_camera_lidar, self._intrinsics = calib_io.load_for_node(
            self, self.get_parameter('calib_file').value)

        # Sensor-data QoS (best-effort, small keep-last depth) is the correct match for
        # high-rate LiDAR/camera streams -- see the QoS discussion in plan/dtp-imp/05 M0.7
        # Step 0.7.2. Using the default RELIABLE QoS here would silently mismatch some
        # publishers and drop the connection with no error. The depth source applies the same
        # profile to every stream it opens.
        #
        # `stride` is passed ONLY for the camera source. LidarDepthSource does not take it and
        # forwards its surplus kwargs to DepthSource.__init__, so an unconditional stride= is a
        # TypeError at construction the moment anyone runs depth_source:=lidar -- a start-up crash
        # in a code path nobody exercises indoors.
        src_kw = {}
        if kind == 'camera':
            src_kw['stride'] = int(self.get_parameter('depth_stride').value)
        self._depth = depth_source.create(
            self, kind, T_camera_lidar=self._T_camera_lidar, max_range_m=self._max_depth, **src_kw)

        # Compressed by default -- see image_io: rclpy loses ~8% of raw 2.76 MB frames at
        # 30 Hz under BEST_EFFORT, JPEG at 0.17 MB keeps up.
        self.declare_parameter('use_compressed', True)
        _use_compressed = bool(self.get_parameter('use_compressed').value)
        _color_topic, _color_type = image_io.image_topic_and_type(
            '/camera/color/image_raw', _use_compressed)
        self._image_sub = self.create_subscription(
            _color_type, _color_topic, self._on_image, qos_profile_sensor_data)

        self._fused_pub = self.create_publisher(
            PointCloud2, '/dtp/points_fused', qos_profile_sensor_data)

        _rate = float(self.get_parameter('publish_rate_hz').value)
        self._period_s = (1.0 / _rate) if _rate > 0.0 else 0.0
        self._last_publish_s = float('-inf')
        self._last_fill_log_s = float('-inf')

        # No unit-ray cache lives here any more. organized_cloud.build factors the deprojection so
        # that the only (H, W) intermediate is the per-pixel scale, which makes the 11 MB ray table
        # unnecessary rather than merely cached -- and removes the question of who invalidates it
        # when a CameraInfo arrives carrying a new resolution mid-run.
        self._warned = set()
        self._depth_ok = None        # tri-state: None = never reported yet. See _note_depth_state.
        self._has_subscriber = None

        # MUST happen here, before main() hands this node to an executor: wait_until_ready spins
        # the node itself, which is only legal outside the executor's own spin. It is also the only
        # thing that makes the camera source's /tf_static lookup work at all -- tf2's own
        # lookup_transform timeout does NOT spin, so the transform never arrives and the lookup
        # fails with "frame does not exist" (the bug diagnosed in scripts/outdoor_depth_test.py).
        self._depth.wait_until_ready(float(self.get_parameter('depth_ready_timeout_s').value))

        self.get_logger().info(
            'fusion_node ENABLED, publishing the organized H x W x 3 cloud on /dtp/points_fused '
            'at %s. This is a DEBUG topic (11.06 MB/message) -- nothing is built while it has no '
            'subscriber, and perception_node builds its own copy in-process. If this line appears '
            'in a FLIGHT log, something passed enable:=true and this node is now competing with '
            'perception_node for the depth stream. max_depth=%s transport=%s'
            % (('%.1f Hz' % (1.0 / self._period_s)) if self._period_s > 0.0
               else 'NO rate (publish_rate_hz=0, publishing disabled)',
               self._max_depth, image_io.describe(_use_compressed)))
        self.get_logger().info('  depth_source=%s: %s'
                               % (self._depth.name, self._depth.describe()))

    # msg is Image or CompressedImage per use_compressed; image_io.decode() gives BGR.
    def _on_image(self, msg) -> None:
        """Build the organized cloud for this colour frame and publish it, or publish nothing.

        Every early return here is a refusal to publish, never a degraded publish. depth_source's
        module docstring states the rule this follows: it never invents a range, because a follower
        that has lost range must know it has lost range. A cloud built from geometry we cannot
        vouch for is the same lie in array form, and it would be believed -- process_frame reads
        whatever it is handed and reports a confident median over it.
        """
        # The rate gate comes first because it is the only check that costs nothing. This callback
        # fires at 29.98 Hz; everything below it -- JPEG decode, deprojection, 11 MB of
        # serialization -- happens at DEBUG_PUBLISH_HZ.
        if self._period_s <= 0.0:
            return
        now = self._now()
        if (now - self._last_publish_s) < self._period_s:
            return

        # A debug publisher must cost nothing when nobody is looking, and in the flight graph
        # nobody is: perception_node builds its own cloud in-process (see the module docstring), so
        # this topic normally has zero subscribers and the node reduces to one float comparison per
        # colour frame. RViz, `ros2 bag record` and lidar_overlay_node all subscribe normally and
        # turn it back on. Logged on the transition rather than every frame -- sensor_watchdog's
        # rule: a message that repeats at sensor rate gets muted, and a muted message is worse than
        # none.
        if self._fused_pub.get_subscription_count() == 0:
            self._note_subscribers(False)
            return
        self._note_subscribers(True)

        # Stamped before the work, not after: a persistent failure below must not turn this into a
        # 30 Hz decode-and-deproject loop that starves FAST-LIO while publishing nothing.
        self._last_publish_s = now

        try:
            bgr = image_io.decode(msg)
        except Exception as exc:
            self.get_logger().error('colour decode failed, no cloud this frame: %s' % exc,
                                    throttle_duration_sec=5.0)
            return

        try:
            rng = self._depth.range_image()
        except Exception as exc:
            # A raise here would unwind through the executor and kill the process, possibly
            # mid-flight -- the same reasoning depth_source gives for disabling itself instead of
            # raising when FAST-LIO's body frame is wrong. A bad depth encoding is permanent, so
            # this is warn-once rather than throttled: a repeat adds nothing.
            self._warn_once('range_image_raised',
                            'depth source raised while building the range image (%s). No cloud '
                            'will be published until this is fixed.' % exc)
            return

        K, size = self._depth.colour_intrinsics()
        if rng is None or K is None or size is None:
            self._note_depth_state(False)
            return
        self._note_depth_state(True)
        w, h = int(size[0]), int(size[1])

        # THE RESOLUTION GATE, and the reason this node decodes a frame it does not otherwise need.
        #
        # The cloud is rasterised with the COLOUR intrinsics into the resolution
        # /camera/color/camera_info advertises. Downstream, YOLO boxes are measured in the pixel
        # space of the DECODED image and used to index this cloud directly. Those two are the same
        # today (1280x720 both) and nothing enforces it: a JPEG that decodes to a different size,
        # or a CameraInfo republished at another resolution, re-indexes every patch with no error
        # anywhere -- the sampled point simply stops being on the person. depth_source flags this
        # same hazard for range_in_roi via `_roi_size`; the organized cloud has no such escape
        # hatch, so it is checked here and the frame is dropped.
        #
        # Throttled rather than warn-once: this condition leaves the topic permanently silent, and
        # a single line at start-up is not enough to explain a dead topic an hour later.
        if bgr.shape[:2] != (h, w) or rng.shape != (h, w):
            self.get_logger().error(
                'REFUSING to publish: decoded colour is %dx%d, camera_info says %dx%d, range '
                'image is %s (all height x width). A cloud indexed by one resolution and sampled '
                'with boxes measured in another silently moves every range sample off the target.'
                % (bgr.shape[0], bgr.shape[1], h, w, 'x'.join(str(n) for n in rng.shape)),
                throttle_duration_sec=30.0)
            return

        # Colour<->depth skew. The cloud carries the COLOUR stamp (organized_cloud.to_pointcloud2
        # requires it), but its geometry came from whatever depth frame the source held; publishing
        # a badly skewed pair under one stamp certifies a disagreement as simultaneity, and the TF
        # lift downstream is done at that stamp.
        colour_stamp = _stamp_seconds(msg.header)
        depth_stamp = getattr(self._depth, 'last_stamp_s', None)
        if depth_stamp is None:
            # DepthSource does not expose its buffer stamp yet. Reaching into _buf_stamp from here
            # would make the guarantee unenforceable from inside depth_source, so the guard
            # degrades instead: adding a `last_stamp_s` property beside colour_intrinsics() turns
            # it on, and RangeSample.stamp_s already publishes the same value through the other
            # query.
            self._warn_once(
                'no_skew_guard',
                'depth source exposes no last_stamp_s, so the colour<->depth skew guard is '
                'DISABLED: published clouds carry the colour stamp with depth of unknown age. '
                'Add a last_stamp_s property to DepthSource to turn it on.')
        elif math.isfinite(depth_stamp):
            skew = abs(colour_stamp - float(depth_stamp))
            if skew > MAX_DEPTH_COLOUR_SKEW_S:
                # Expect this occasionally: colour and depth are free-running at ~30 Hz with an
                # unmeasured phase offset, so a fraction of frames legitimately miss the window and
                # the topic runs slightly under its nominal rate. If it fires CONSTANTLY the phase
                # is bad or use_sim_time disagrees between this node and the driver -- measure the
                # offset before widening the threshold, because widening it is the option that
                # silently ships the error.
                self.get_logger().warn(
                    'skipping frame: depth is %.0f ms from the colour stamp (limit %.0f ms). '
                    'Constant repeats mean a stream phase problem or a use_sim_time mismatch, not '
                    'a threshold that is too tight.' % (skew * 1e3, MAX_DEPTH_COLOUR_SKEW_S * 1e3),
                    throttle_duration_sec=5.0)
                return

        # --- deprojection and packing ------------------------------------------------------------
        # Both are organized_cloud's, not this file's: the replay harness records bags from this
        # topic and drives `process_frame` with them, so a second deprojection here would let a bag
        # test pass while the in-process flight path was wrong.
        #
        # The gate runs BEFORE build() rather than catching its ValueError, for one reason: an
        # all-zero K is what the driver publishes before its calibration is loaded, i.e. a NORMAL
        # start-up state, and it deserves a throttled error naming the cause rather than a stack
        # trace. Everything else build() can raise is a bug, and the except below keeps it from
        # unwinding through the executor and killing the node mid-flight.
        problem = organized_cloud.intrinsics_problem(K, rng.shape)
        if problem:
            self.get_logger().error(
                'no cloud this frame: %s. An unusable CameraInfo would deproject to an all-NaN '
                'cloud, which reads downstream as "the scene is empty" rather than as a '
                'configuration fault.' % problem, throttle_duration_sec=30.0)
            return

        # The colour message's OWN frame_id, read rather than hardcoded: the cloud is in whatever
        # frame the driver says its colour images are in, and asserting a name we did not read is
        # how a TF chain ends up resolving against a frame that is not the one the data is in.
        # to_pointcloud2 REFUSES an empty one, so the fallback (and its warning) happens here where
        # there is a logger to say which stream produced it.
        header = msg.header
        if not (header.frame_id or '').strip():
            # THE COLOUR IMAGE'S STAMP is carried across unchanged; only the name is substituted.
            # perception_node looks up T_world_camera AT that stamp (risk A8: "latest" costs
            # yaw_rate x range, ~0.5 m at 3 rad/s and 6 m) and the replay harness re-times
            # everything from it, so it is never re-stamped here.
            header = Header(stamp=msg.header.stamp, frame_id=self._fallback_frame())

        try:
            # range_image() hands back a buffer it REUSES between frames; build() only reads it and
            # always allocates its result, so nothing here retains the source buffer.
            cloud = organized_cloud.build(rng, K)
            out = organized_cloud.to_pointcloud2(cloud, header)
        except Exception as exc:
            self._warn_once('build_raised',
                            'organized_cloud refused this frame (%s). No cloud will be published '
                            'until this is fixed -- it is a geometry or layout fault, not a '
                            'transient.' % exc)
            return

        # Fill is measured on the CLOUD's z channel rather than on the range image: that is the
        # array a consumer actually samples, and it is the one place the NaN convention has already
        # been applied.
        self._maybe_log_fill(now, cloud[:, :, 2], w, h)
        self._fused_pub.publish(out)

    # -- helpers ---------------------------------------------------------------------------------

    def _fallback_frame(self):
        self._warn_once('empty_frame_id',
                        'the colour image carries an EMPTY frame_id, so the published cloud is '
                        'labelled %s instead. That name is a guess from depth_source, not '
                        'something read off this stream -- fix the driver rather than trusting it.'
                        % depth_source.COLOUR_OPTICAL_FRAME)
        return depth_source.COLOUR_OPTICAL_FRAME

    def _maybe_log_fill(self, now, z_channel, w, h):
        """Report what fraction of the cloud is a real measurement. See FILL_LOG_PERIOD_S.

        `z_channel` is cloud[:, :, 2]; by organized_cloud's joint-NaN guarantee its finiteness is
        the finiteness of all three channels, so one channel is the whole answer.
        """
        if (now - self._last_fill_log_s) < FILL_LOG_PERIOD_S:
            return
        self._last_fill_log_s = now
        filled = int(np.count_nonzero(np.isfinite(z_channel)))
        self.get_logger().info(
            'cloud fill %.1f%% (%d of %d px) at %dx%d. This is the number the torso patch is '
            'drawn from, NOT depth pixel validity.'
            % (100.0 * filled / float(w * h), filled, w * h, w, h))

    def _note_depth_state(self, ok):
        """Log the depth source going blind and coming back, once per transition each way."""
        was = self._depth_ok
        if ok == was:
            return
        self._depth_ok = ok
        if not ok:
            self.get_logger().warn(
                'no range image yet -- publishing NOTHING rather than an empty or invented cloud. '
                'Still waiting on: %s' % (', '.join(self._depth.missing()) or 'unknown'))
        elif was is False:
            self.get_logger().info('depth source producing range images again; publishing resumed')
        # was is None and ok: the first good frame. Nothing to announce -- the start-up log already
        # said a cloud was coming and it is now on the topic.

    def _note_subscribers(self, any_now):
        if any_now == self._has_subscriber:
            return
        self._has_subscriber = any_now
        if any_now:
            self.get_logger().info('/dtp/points_fused has a subscriber -- building the cloud at '
                                   '%.1f Hz (~11 MB/message at 1280x720; see the module docstring)'
                                   % (1.0 / self._period_s))
        else:
            self.get_logger().info('/dtp/points_fused has no subscribers -- not building the '
                                   'cloud. This is the normal flight-graph state.')

    def _warn_once(self, key, message) -> None:
        """First occurrence only. Same rule as depth_source._warn_once: a warning that repeats at
        sensor rate gets muted, and a muted warning is worse than none."""
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(message)

    def _now(self) -> float:
        # Node clock, so the throttle follows /clock under use_sim_time and a bag replayed at half
        # speed still decimates by the same factor it would live.
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
