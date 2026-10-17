"""depth_source -- one range-to-target interface, two sensors behind it, picked by a parameter.

WHY THIS EXISTS
---------------
The follower needs one thing from depth: **how far away is the person in this box**. There are two
ways to get it on this aircraft, and as of 2026-09-08 we cannot yet prove which one to fly:

  * **camera** -- the Gemini 336L's own stereo depth. Measured indoors 2026-09-08: **88.4 % of
    pixels valid at 30 Hz**. It is registered to the colour sensor by the **factory** depth<->colour
    extrinsic, published on `/tf_static` by the driver. Both sensors live in one housing, so that
    transform survives any remount, re-tape or drop. Nothing of ours has to be calibrated.

  * **lidar** -- the Mid-360, projected through **our** solved `T_camera_lidar`. Measured over the
    same scene: **13.5 % coverage inside the camera FOV, and nothing at all in the lower third**
    (the Mid-360's vertical floor is about -7 deg, so the ground immediately in front of the
    aircraft is simply not sampled). It also depends on an extrinsic that **every remount
    invalidates** -- see calib_io's docstring for what a stale extrinsic costs.

On those numbers the camera wins outright and the LiDAR would be left to FAST-LIO and ROG-Map.
**But every one of those numbers came from indoors.** The 336L is an ACTIVE stereo camera: an IR
dot projector paints texture and two IR cameras triangulate it. Direct sunlight carries far more
IR power than that projector, so outdoors the dots wash out and the camera falls back on natural
texture only -- a much weaker regime. `scripts/outdoor_depth_test.py` is the test that settles it
and it has not been run yet.

So this module exists to make that pending result a **config change instead of a redesign**. The
rest of the pipeline asks for a range in a colour-image ROI and never learns which sensor answered.

WHAT "THE SAME THING" MEANS HERE, PRECISELY
-------------------------------------------
Both implementations reduce to exactly one job: **produce points in the COLOUR OPTICAL frame**.
Everything after that -- projection through the colour intrinsics, z-buffering, the torso patch,
the robust range statistic, the coverage number -- lives once, in `DepthSource`, and is therefore
bit-for-bit identical between the two. If the reduction lived in each subclass the two sources
would drift apart under maintenance and the abstraction would be a lie: an A/B between them would
be measuring our two implementations, not the two sensors.

`RangeSample.source` is for LOGGING ONLY. If any caller branches on it, the abstraction has failed
and the outdoor test stops being a config change.

WHY EUCLIDEAN RANGE AND NOT DEPTH-Z
------------------------------------
`range_m` is the Euclidean distance in the colour optical frame, NOT the optical Z component. The
336L's colour FOV is 90 deg horizontal, so at the frame edge Z under-reads the true distance by
1/cos(45 deg) = 1.41x. Reporting Z as "range" would put a person at the edge of frame 40 % closer
than they are, which drives a standoff follower straight at them. Z-ordering is unaffected: for a
FIXED pixel the ray direction is fixed, so range = Z / cos(theta) with theta constant, and sorting
by range is identical to sorting by Z. The z-buffer is therefore still exact.

WHAT THIS MODULE REFUSES TO DO
-------------------------------
It never invents a range. No return in the ROI produces `RangeSample.valid == False` and a NaN,
never a last-known value and never an extrapolation. A follower that has lost range must know it
has lost range -- coasting on a stale number is how the drone keeps closing on a person who is no
longer where it thinks. Holding/coasting is `target_manager`'s decision (plan/dtp-imp/04 SS2), made
with the track state this module does not have.

TWO CONFIGURATION FACTS THAT WILL BITE, BOTH VERIFIED IN config/mid360.yaml
----------------------------------------------------------------------------
1. `scan_bodyframe_pub_en: false` (config/mid360.yaml:56). **FAST-LIO does not publish
   `/cloud_registered_body` today**, so `depth_source:=lidar` will sit with no data and no error
   from the LiDAR side -- the subscription is valid, nothing arrives. Flip that key to `true` before
   selecting the LiDAR source. `LidarDepthSource` says exactly this in its start-up log rather than
   leaving you to discover it as an unexplained silence; that file is owned elsewhere, so this
   module warns instead of fixing.
2. `extrinsic_T: [0,0,0]` and `extrinsic_R: identity` (config/mid360.yaml:45-46) are the LiDAR->IMU
   extrinsic. FAST-LIO publishes `/cloud_registered_body` in the **IMU body** frame, and only
   because that extrinsic is identity is the body frame the same thing as the LiDAR frame -- which
   is what `T_camera_lidar` is defined against. If anyone ever measures a real LiDAR->IMU offset
   and sets it there, every LiDAR range from this module shifts by that offset with no error
   anywhere. `LidarDepthSource` reads the parameter back off the running fast_lio node and refuses
   to start on a definite mismatch (see `_check_fastlio_body_is_lidar`).

WHY /cloud_registered_body AND NOT /livox/lidar
------------------------------------------------
`/livox/lidar` is the raw driver sweep: not deskewed, so on a moving aircraft the points inside one
sweep are smeared along the flight path. `fusion_node`'s own docstring already required
"FAST-LIO2-motion-compensated (deskewed)" points (risk A9, plan/dtp-imp/14 C6/C7). FAST-LIO's
body-frame output is exactly that, already computed, for free. `/cloud_registered` (world frame)
would work too but would drag map drift into a purely geometric measurement.

USAGE
-----
    src = depth_source.create(node, 'camera')       # or 'lidar', + T_camera_lidar=...
    src.wait_until_ready(5.0)                       # BEFORE handing `node` to an executor
    ...
    s = src.range_in_roi((x1, y1, x2, y2))
    if s.valid and s.age_s < 0.2:
        use(s.range_m)
"""
import math
import time
from collections import deque
from typing import NamedTuple, Optional, Tuple

import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

SOURCES = ('camera', 'lidar')

# ---------------------------------------------------------------------------------------------
# Ranging band. Shared by both sources ON PURPOSE: a range statistic computed over different
# windows is not comparable, and comparability is the entire point of this module.
# ---------------------------------------------------------------------------------------------
# Below this we do not believe either sensor. The 336L is specified to 0.17 m and the Mid-360's
# driver blanks its first 0.5 m (`blind: 0.5`, config/mid360.yaml:35), so the LiDAR physically
# cannot answer inside 0.5 m whatever this says. That asymmetry is real and is a reason the LiDAR
# source can never be the sole input to the 1.5 m proximity veto: at knife-fight range it is blind.
MIN_RANGE_M = 0.3
# Above this we do not believe either sensor either. 15 m matches fusion_node's `max_depth` default
# and CLAUDE.md SS5: "LiDAR is reliably useful to ~15 m on a person, not 40 m". The 336L's optimal
# window is 0.25-6 m and it degrades well before 15 m, but we cap rather than truncate -- CLAUDE.md
# SS5 is explicit that NaN-ing camera depth early "throws away the densest near-range data".
MAX_RANGE_M = 15.0

# ---------------------------------------------------------------------------------------------
# ROI reduction. A YOLO person box is roughly half background at the shoulders and between the
# legs, so a plain median over the box lands on the wall behind whenever the person occupies less
# than half of it. These constants carve out the chest.
# ---------------------------------------------------------------------------------------------
# Central half of the box width, and the 15 %-55 % band of its height: below the head (a head-box
# margin is nearly all background) and above the legs (which straddle background between them).
TORSO_W_FRAC = 0.50
TORSO_TOP_FRAC = 0.15
TORSO_H_FRAC = 0.40
# A patch smaller than this samples nothing and reports "no range", which reads in a log like a
# sensor failure rather than like the spurious 6-pixel detection it actually was. Clamp up, but
# never past the caller's own box.
MIN_PATCH_PX = 6
# Fewer retained samples than this is noise, not a measurement. The LiDAR is the binding case: at
# 13.5 % frame coverage a person at 4 m yields a few tens of points in the torso patch, so this has
# to stay low enough that the LiDAR source is usable at all and high enough that one stray return
# cannot become a range.
MIN_ROI_SAMPLES = 6
# Seed the "which of these samples is the person" search at a LOW percentile, not the median: in a
# trail-behind follow the person is the nearest thing in their own torso patch.
#
# 10 % rather than 0 %, and the exact number is the knob that decides what counts as clutter: an
# intruding foreground object narrower than 10 % of the torso patch does NOT capture the estimate
# (a twig, a railing edge, one speckle return), while anything wider does. Verified against the
# synthetic patch: a bar covering 9.2 % of the patch leaves the range on the person; widening it
# past 10 % moves the range onto the bar. Lower this only with that trade in mind -- 0 % lets a
# single bad return define range-to-human.
CLUSTER_SEED_PCTL = 10.0
# Keep samples within this of the seed. A human torso is ~0.3 m deep; 0.6 m holds the whole person
# plus sensor noise while excluding a wall a metre behind them.
#
# The bias this creates is deliberate and it is the safe direction. If something foreground (a
# railing, a branch) intrudes into the torso patch, the seed lands on the intruder and we report
# the person as NEARER than they are -- and for a standoff follower an error toward "closer"
# increases separation, while an error toward "farther" drives the aircraft in. Fail near.
CLUSTER_BAND_M = 0.6

# ---------------------------------------------------------------------------------------------
# Quality reporting
# ---------------------------------------------------------------------------------------------
# `coverage` is the fraction of coarse cells in the torso patch that got at least one return.
#
# It is defined this way because the obvious per-sensor definitions are NOT comparable: camera
# coverage is naturally "valid depth pixels / expected depth pixels" (what
# scripts/outdoor_depth_test.py's eval_box computes), while the Mid-360 emits a fixed point budget
# spread over 360 deg and has no pixel grid at all. Reporting both under one field name would be a
# silent unit mismatch across a parameter switch -- exactly the class of bug this module exists to
# avoid. Cell occupancy answers the question a follower actually cares about ("how much of the
# person did I measure") identically for both.
COVERAGE_CELL_PX = 16

# ---------------------------------------------------------------------------------------------
# Camera source
# ---------------------------------------------------------------------------------------------
# 16UC1 depth is millimetres by ROS convention and that is what this camera publishes
# (CLAUDE.md: /camera/depth/image_raw 848x480 16UC1 at 29.9 Hz).
DEPTH_SCALE_MM = 0.001
# Deproject every Nth depth pixel. 848x480 is 407k points and this runs in Python at camera rate;
# stride 2 is 102k, which is far more than the tens of samples the torso statistic needs. Raise it
# to 1 only if something downstream needs a dense organized cloud and the CPU is there.
DEPTH_STRIDE = 2
# The Orbbec driver's colour optical frame. The depth frame is taken from the depth message's own
# header rather than assumed, because that is the string the TF is actually published against.
COLOUR_OPTICAL_FRAME = 'camera_color_optical_frame'
DEPTH_OPTICAL_FRAME_FALLBACK = 'camera_depth_optical_frame'

# ---------------------------------------------------------------------------------------------
# LiDAR source
# ---------------------------------------------------------------------------------------------
# Stack this many seconds of sweeps before projecting. One Mid-360 sweep is far too sparse on a
# person at 3-10 m (risk A9; plan/dtp-imp/14 C6/C7), and fusion_node's docstring already specified
# a 0.1-0.2 s window. At 10 Hz this is 2 sweeps.
LIDAR_ACCUM_S = 0.2
# Hard cap on the buffer so a stalled consumer cannot grow it without bound.
LIDAR_MAX_SWEEPS = 8
# A sweep whose nearest odom sample is further away in time than this cannot be re-registered onto
# the newest sweep, and is DROPPED rather than stacked un-registered. fast_lio publishes the cloud
# and the odometry from the same iteration with the same stamp, so in healthy operation the match
# is essentially exact and this never fires; when it does fire, odom is broken and stacking would
# smear the person across however far the aircraft moved.
ODOM_MATCH_TOL_S = 0.05
LIDAR_ODOM_HISTORY = 64

# Node to ask for FAST-LIO's LiDAR->IMU extrinsic when DDS will not name the publisher of the
# cloud. A GUESS: no file in this repo records the runtime node name -- only the executable
# (`fastlio_mapping`) and the source file (`laserMapping.cpp`). The endpoint-resolved name is the
# real path; this exists so an unnamed endpoint degrades to "probably right" instead of "no check".
FASTLIO_NODE_FALLBACK = '/laserMapping'
# Give up asking after this many clouds and say so once. Bounded because a permanently
# unanswerable query polled at 10 Hz forever is just a leak.
BODY_FRAME_CHECK_TRIES = 20
# Anything above this is a deliberately configured offset, not float noise in a YAML round-trip.
BODY_FRAME_TOL_M = 1e-6

# PointCloud2 datatype id -> numpy scalar type. Used to build a structured dtype and view the
# message buffer directly. The per-point `read_points`/`struct.unpack` path is not an option here:
# CLAUDE.md SS"what the 336L gives us" is explicit that anything consuming a multi-MB/s cloud from
# Python must use numpy.frombuffer, and this runs at sensor rate on the aircraft.
_PC2_NUMPY = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
              5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


class DepthSourceError(RuntimeError):
    """Raised when a depth source cannot be built, or is built against the wrong geometry."""


class RangeSample(NamedTuple):
    """One answer to "how far is the thing in this box". Identical shape from both sources.

    `range_m` is NaN whenever there was no usable measurement -- check `valid` first, always.
    `source` is for logs only; branching on it defeats the point of this module.
    """
    range_m: float          # Euclidean range in the colour optical frame, or NaN
    n_points: int           # samples retained by the cluster gate
    coverage: float         # 0..1 filled fraction of the torso patch's coarse cells
    spread_m: float         # 16-84 % spread of the retained cluster, or NaN
    stamp_s: float          # header stamp of the data behind this answer, or NaN
    age_s: float            # node clock now - stamp_s at query time, or NaN
    source: str             # 'camera' | 'lidar' -- LOGGING ONLY

    @property
    def valid(self) -> bool:
        return bool(np.isfinite(self.range_m))


_EMPTY_FIELDS = dict(range_m=float('nan'), n_points=0, coverage=0.0,
                     spread_m=float('nan'), stamp_s=float('nan'), age_s=float('nan'))


def quat_to_R(q) -> np.ndarray:
    """geometry_msgs quaternion -> 3x3 rotation. Same expression as scripts/depth_compare.py."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def cloud_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N, 3) float32 of finite xyz, via a single buffer view.

    Builds a numpy structured dtype from the message's own field offsets and `point_step`, so
    padding and extra fields (intensity, ring, timestamp) cost nothing and no assumption is made
    about layout. A truncated buffer is trimmed rather than reshaped into an exception.
    """
    if msg.is_bigendian:
        # Nothing in this stack is big-endian; refusing beats byte-swapping a path that would then
        # never be exercised again.
        raise DepthSourceError('big-endian PointCloud2 is not supported')
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.name not in ('x', 'y', 'z') or f.count != 1:
            continue
        np_type = _PC2_NUMPY.get(f.datatype)
        if np_type is None:
            raise DepthSourceError('PointCloud2 field %r has datatype %d' % (f.name, f.datatype))
        names.append(f.name)
        formats.append(np_type)
        offsets.append(f.offset)
    if sorted(names) != ['x', 'y', 'z']:
        raise DepthSourceError('PointCloud2 has no x/y/z float fields (got %s)'
                               % ', '.join(f.name for f in msg.fields))

    dt = np.dtype({'names': names, 'formats': formats, 'offsets': offsets,
                   'itemsize': msg.point_step})
    buf = memoryview(msg.data)
    n = len(buf) // msg.point_step
    arr = np.frombuffer(buf[:n * msg.point_step], dtype=dt)
    pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32, copy=False)
    return pts[np.isfinite(pts).all(axis=1)]


class DepthSource:
    """Base: owns the colour geometry, the z-buffer, and the whole ROI statistic.

    A subclass implements exactly one method, `_points_in_colour_frame`, returning
    `(points Nx3 in the COLOUR OPTICAL frame, header-stamp seconds)`. Everything a caller can
    observe is computed here, so the two sources cannot drift apart.
    """

    name = 'base'

    def __init__(self, node, colour_info_topic='/camera/color/camera_info',
                 min_range_m=MIN_RANGE_M, max_range_m=MAX_RANGE_M, roi_size=None):
        self._node = node
        self._log = node.get_logger()
        self._min_range = float(min_range_m)
        self._max_range = float(max_range_m)

        # ROIs arrive in whatever pixel space the detector ran in. Today perception_node runs YOLO
        # on the decoded 1280x720 colour image and /camera/color/camera_info is also 1280x720, so
        # they coincide -- but the moment anything downsamples for inference, every ROI would be
        # silently reinterpreted against the wrong intrinsics and every range would be wrong with
        # no error. Declaring the size makes that a scale factor instead of a bug.
        self._roi_size = tuple(roi_size) if roi_size else None

        self._K = None            # colour intrinsics, 3x3
        self._size = None         # (width, height) the colour intrinsics belong to
        self._colour_info_sub = node.create_subscription(
            CameraInfo, colour_info_topic, self._on_colour_info, qos_profile_sensor_data)

        # The range image is recomputed at most once per incoming frame, and only if somebody asks.
        # A node that subscribes but never queries therefore pays nothing beyond the message.
        self._buf = None
        self._buf_key = None
        self._buf_stamp = float('nan')
        self._warned = set()

    # -- subclass contract -------------------------------------------------------------------

    def _points_in_colour_frame(self) -> Optional[Tuple[np.ndarray, float, object]]:
        """Return (points Nx3 float32 in the colour optical frame, stamp_s, cache_key) or None.

        `cache_key` identifies the underlying frame so the base can skip re-projecting data it has
        already rasterised; any hashable that changes exactly when the data does will do.
        """
        raise NotImplementedError

    def _missing(self):
        """Subclass part of the readiness report, as a list of short human-readable strings."""
        return []

    # -- readiness ---------------------------------------------------------------------------

    def missing(self):
        out = []
        if self._K is None:
            out.append('colour camera_info')
        out.extend(self._missing())
        return out

    @property
    def ready(self) -> bool:
        return not self.missing()

    def wait_until_ready(self, timeout_s=5.0) -> bool:
        """Spin the node until every input this source needs has arrived. Returns readiness.

        *** CALL THIS BEFORE HANDING THE NODE TO AN EXECUTOR, AND NEVER FROM A CALLBACK. ***
        It calls rclpy.spin_once itself, which is legal only outside the executor's own spin.

        Why spinning is mandatory rather than merely convenient: `tf2_ros.Buffer.lookup_transform`
        takes a `timeout` argument that LOOKS like it waits for the transform, and does not spin
        the node -- so `/tf_static` is never delivered, never enters the buffer, and the call fails
        with "frame does not exist" after burning the full timeout. That is a real bug that was hit
        in scripts/depth_compare.py and diagnosed in scripts/outdoor_depth_test.py:170-173, where it
        silently disabled the person metric, i.e. the entire result of the test. The camera source
        depends on that same static transform, so the same trap applies here verbatim.
        """
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self.ready:
                self._log.info('depth_source[%s] ready' % self.name)
                return True
        self._log.warn('depth_source[%s] NOT ready after %.1fs -- still waiting on: %s'
                       % (self.name, timeout_s, ', '.join(self.missing()) or 'nothing?'))
        return False

    # -- geometry ----------------------------------------------------------------------------

    def _on_colour_info(self, msg: CameraInfo) -> None:
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._size = (int(msg.width), int(msg.height))

    def colour_intrinsics(self):
        """(K 3x3, (width, height)) of the frame `range_image()` is expressed in, or (None, None).

        Exposed so a caller that wants XYZ rather than range can rebuild it without duplicating
        the projection: for pixel (u, v), dir = normalize([(u-cx)/fx, (v-cy)/fy, 1]) and
        p = range * dir. That is exactly the organized H x W x 3 cloud fusion_node publishes.
        """
        return self._K, self._size

    @property
    def last_stamp_s(self) -> float:
        """Stamp of the newest DELIVERED depth message, seconds; NaN before the first one.

        Public because perception_node gates every frame on the colour<->depth skew and was
        otherwise forced to reach for the private `_buf_stamp`. That coupling failed in the worst
        possible direction: a rename here would make its getattr fall back to NaN, the skew test
        `not math.isfinite(skew)` would be True on EVERY frame, and the node would publish an
        empty TargetArray at a healthy 15 Hz forever -- silent, total blindness that looks
        identical on the wire to "nobody is here".

        This is DELIVERY time, not capture time: it is the stamp of the newest depth Image handed
        to this process, so a skew computed against it includes transport and scheduling delay,
        not just sensor phase. Callers gating on it should size their threshold accordingly.
        """
        return self._buf_stamp

    # -- the range image ---------------------------------------------------------------------

    def range_image(self) -> Optional[np.ndarray]:
        """(H, W) float32 nearest-return Euclidean range in the colour frame; NaN where nothing.

        The returned array is REUSED between frames. Copy it if you intend to hold it.
        """
        if self._K is None:
            return None
        got = self._points_in_colour_frame()
        if got is None:
            return None
        pts, stamp_s, key = got
        w, h = self._size
        # The shape test is part of the cache hit, not just of the allocation: a CameraInfo carrying
        # a new resolution mid-run changes self._K and self._size while the frame key is unchanged,
        # and serving the old buffer would then hand back ranges indexed by the OLD intrinsics.
        if (key is not None and key == self._buf_key
                and self._buf is not None and self._buf.shape == (h, w)):
            return self._buf

        if self._buf is None or self._buf.shape != (h, w):
            self._buf = np.empty((h, w), np.float32)
        self._buf.fill(np.nan)
        self._buf_key = key
        self._buf_stamp = stamp_s

        if pts.size:
            z = pts[:, 2]
            rng = np.linalg.norm(pts, axis=1)
            keep = (z > 0.0) & (rng >= self._min_range) & (rng <= self._max_range)
            p = pts[keep]
            rng = rng[keep]
            if p.size:
                fx, fy = self._K[0, 0], self._K[1, 1]
                cx, cy = self._K[0, 2], self._K[1, 2]
                u = fx * p[:, 0] / p[:, 2] + cx
                v = fy * p[:, 1] / p[:, 2] + cy
                inb = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                u = u[inb].astype(np.intp)
                v = v[inb].astype(np.intp)
                rng = rng[inb]
                # Nearest return wins the pixel. Writing in far-to-near order lets the near sample
                # overwrite the far one -- ordinary z-buffering, and exact here because for a fixed
                # pixel range and depth-Z differ only by the constant 1/cos(theta) of that ray.
                order = np.argsort(-rng)
                self._buf[v[order], u[order]] = rng[order]
        return self._buf

    # -- the organized cloud on the GPU --------------------------------------------------------

    def organized_cloud_cuda(self, cuda):
        """`organized_cloud.build(self.range_image(), K)`, computed on the GPU, or None if not up.

        `cuda` is a cuda_cloud.CudaCloud. Returns a FRESHLY allocated (H, W, 3) float32 array, like
        build(). This base version keeps the subclass's `_points_in_colour_frame` in numpy and
        moves everything after it to the GPU -- bit-identical, see cuda_cloud. CameraDepthSource
        overrides it to move its deprojection too, which is where the camera's CPU cost is.
        Raises cuda_cloud.CudaCloudError on a GPU failure; choosing a fallback is the caller's job.
        """
        if self._K is None:
            return None
        got = self._points_in_colour_frame()
        if got is None:
            return None
        pts, stamp_s, _key = got
        cloud = cuda.from_points(pts, self._K, self._size, self._min_range, self._max_range)
        self._note_rasterised(stamp_s)
        return cloud

    def _note_rasterised(self, stamp_s) -> None:
        """Bookkeeping the GPU path owes the rest of this class.

        `last_stamp_s` MUST advance: perception_node's depth/colour skew guard reads it on every
        frame, and a stamp stuck at NaN or at an old frame publishes every frame blind -- silent,
        total blindness that looks like "nobody is here" (see last_stamp_s). And the numpy cache
        key is invalidated, because `_buf` still holds whatever range_image() last rasterised:
        leaving the key matching would let a later range_image()/range_in_roi() call on the same
        frame serve that stale buffer as if it were this frame.
        """
        self._buf_stamp = stamp_s
        self._buf_key = None

    # -- the query -----------------------------------------------------------------------------

    def range_in_roi(self, roi, roi_size=None) -> RangeSample:
        """Range to whatever occupies `roi = (x1, y1, x2, y2)` in colour-image pixels.

        `roi_size` is the (width, height) of the image those coordinates were measured in; omit it
        when the ROI is already in `/camera/color/camera_info` pixels (see __init__).
        """
        rimg = self.range_image()
        now = self._node.get_clock().now().nanoseconds * 1e-9
        stamp = self._buf_stamp
        age = (now - stamp) if math.isfinite(stamp) else float('nan')

        # A stamp in the future is the signature of a use_sim_time mismatch between this node and
        # the driver: the age gate every caller applies then either never fires or always fires,
        # and in both cases silently. Say it once, loudly, rather than returning a plausible number.
        if math.isfinite(age) and age < -0.1:
            self._warn_once(
                'future_stamp',
                'depth data is stamped %.2f s in the FUTURE. This node and the sensor driver '
                'disagree about use_sim_time; every staleness check downstream is meaningless '
                'until that is fixed.' % (-age))

        if rimg is None:
            return RangeSample(source=self.name, **_EMPTY_FIELDS)

        h, w = rimg.shape
        x1, y1, x2, y2 = self._roi_to_camera_pixels(roi, roi_size, w, h)
        px1, py1, px2, py2 = self._torso_patch(x1, y1, x2, y2, w, h)
        if px2 <= px1 or py2 <= py1:
            return RangeSample(source=self.name, **dict(_EMPTY_FIELDS, stamp_s=stamp, age_s=age))

        patch = rimg[py1:py2, px1:px2]
        finite = np.isfinite(patch)
        vals = patch[finite]
        if vals.size < MIN_ROI_SAMPLES:
            return RangeSample(range_m=float('nan'), n_points=int(vals.size), coverage=0.0,
                               spread_m=float('nan'), stamp_s=stamp, age_s=age, source=self.name)

        # Nearest cluster, seeded low -- see CLUSTER_SEED_PCTL / CLUSTER_BAND_M for why near.
        seed = float(np.percentile(vals, CLUSTER_SEED_PCTL))
        near = (vals >= seed - CLUSTER_BAND_M) & (vals <= seed + CLUSTER_BAND_M)
        cluster = vals[near]
        if cluster.size < MIN_ROI_SAMPLES:
            return RangeSample(range_m=float('nan'), n_points=int(cluster.size), coverage=0.0,
                               spread_m=float('nan'), stamp_s=stamp, age_s=age, source=self.name)

        rng = float(np.median(cluster))
        p16, p84 = np.percentile(cluster, (16.0, 84.0))
        # The band cap is applied a second time here, after aggregation, because a cluster whose
        # median lands outside the trusted window is not a measurement we should hand a follower
        # even though each of its samples passed individually.
        if not (self._min_range <= rng <= self._max_range):
            rng = float('nan')

        coverage = self._patch_coverage(finite)
        return RangeSample(range_m=rng, n_points=int(cluster.size), coverage=coverage,
                           spread_m=float(p84 - p16), stamp_s=stamp, age_s=age, source=self.name)

    def _roi_to_camera_pixels(self, roi, roi_size, w, h):
        x1, y1, x2, y2 = (float(v) for v in roi)
        ref = roi_size or self._roi_size
        if ref and (int(ref[0]), int(ref[1])) != (w, h):
            sx, sy = w / float(ref[0]), h / float(ref[1])
            x1, x2, y1, y2 = x1 * sx, x2 * sx, y1 * sy, y2 * sy
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        return (int(max(0, math.floor(x1))), int(max(0, math.floor(y1))),
                int(min(w, math.ceil(x2))), int(min(h, math.ceil(y2))))

    @staticmethod
    def _torso_patch(x1, y1, x2, y2, w, h):
        """Chest-centred sub-box of a person ROI. See TORSO_* for why not the whole box."""
        bw, bh = x2 - x1, y2 - y1
        if bw <= 0 or bh <= 0:
            return 0, 0, 0, 0
        cx = 0.5 * (x1 + x2)
        pw = max(min(bw, MIN_PATCH_PX), bw * TORSO_W_FRAC)
        ph = max(min(bh, MIN_PATCH_PX), bh * TORSO_H_FRAC)
        top = y1 + bh * TORSO_TOP_FRAC
        px1 = int(max(0, math.floor(cx - pw / 2.0)))
        px2 = int(min(w, math.ceil(cx + pw / 2.0)))
        py1 = int(max(0, math.floor(top)))
        py2 = int(min(h, math.ceil(top + ph)))
        return px1, py1, px2, py2

    @staticmethod
    def _patch_coverage(finite_mask) -> float:
        """Filled fraction of COVERAGE_CELL_PX cells -- the one quality number both sources share."""
        ph, pw = finite_mask.shape
        if ph == 0 or pw == 0:
            return 0.0
        ny = max(1, -(-ph // COVERAGE_CELL_PX))
        nx = max(1, -(-pw // COVERAGE_CELL_PX))
        ys = np.minimum((np.arange(ph) // COVERAGE_CELL_PX), ny - 1)
        xs = np.minimum((np.arange(pw) // COVERAGE_CELL_PX), nx - 1)
        cell = ys[:, None] * nx + xs[None, :]
        filled = np.unique(cell[finite_mask])
        return float(filled.size) / float(nx * ny)

    # -- misc ---------------------------------------------------------------------------------

    def _warn_once(self, key, message) -> None:
        """Log a warning the first time only. A warning that repeats at sensor rate gets muted,
        and a muted warning is worse than none (same reasoning as sensor_watchdog's transition
        logging)."""
        if key not in self._warned:
            self._warned.add(key)
            self._log.warn('depth_source[%s] %s' % (self.name, message))

    def describe(self) -> str:
        """One line for a node's start-up log, so the active depth source is never a mystery."""
        raise NotImplementedError


class CameraDepthSource(DepthSource):
    """Depth from the 336L's own stereo, registered to colour by the camera's FACTORY extrinsic.

    Needs NO calibration of ours. The depth->colour transform comes off `/tf_static`, published by
    the driver, and describes two sensors inside one housing -- so unlike the LiDAR path it is
    unaffected by how the camera is mounted, re-taped or knocked. That is the property that makes
    this the default (scripts/depth_compare.py's own docstring makes the same point).

    The pending question this cannot answer indoors: the 336L is ACTIVE stereo, and direct sunlight
    swamps its IR projector. scripts/outdoor_depth_test.py decides whether this stays the default.
    """

    name = 'camera'

    def __init__(self, node, depth_topic='/camera/depth/image_raw',
                 depth_info_topic='/camera/depth/camera_info',
                 colour_frame=COLOUR_OPTICAL_FRAME, stride=DEPTH_STRIDE, **kw):
        super().__init__(node, **kw)
        self._stride = max(1, int(stride))
        self._colour_frame = colour_frame

        self._depth_msg = None
        self._depth_seq = 0        # cheap frame identity: message objects are not hashable
        self._Kd = None
        self._depth_size = None
        self._R_cd = None
        self._t_cd = None

        import tf2_ros
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        node.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, depth_info_topic, self._on_depth_info,
                                 qos_profile_sensor_data)

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._Kd = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._depth_size = (int(msg.width), int(msg.height))

    def _on_depth(self, msg: Image) -> None:
        self._depth_msg = msg
        self._depth_seq += 1
        # Resolved here, in a callback, rather than in __init__ on purpose. /tf_static is
        # TRANSIENT_LOCAL so a late subscriber still receives it -- but only once the node is being
        # spun, which during construction it is not (see wait_until_ready for the full trap). By
        # the time depth frames are arriving the executor is running and the buffer is fed.
        if self._R_cd is None:
            self._try_static_tf(msg.header.frame_id or DEPTH_OPTICAL_FRAME_FALLBACK)

    def _try_static_tf(self, depth_frame) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(self._colour_frame, depth_frame,
                                                  rclpy.time.Time())
        except Exception as exc:
            self._warn_once('no_tf',
                            'no %s <- %s transform yet (%s). Depth is being received but cannot be '
                            'registered to colour, so no range can be produced.'
                            % (self._colour_frame, depth_frame, exc))
            return
        self._R_cd = quat_to_R(tf.transform.rotation)
        t = tf.transform.translation
        self._t_cd = np.array([t.x, t.y, t.z], dtype=np.float64)
        self._log.info('depth_source[camera] depth->colour transform acquired (%s <- %s, '
                       'baseline %.4f m, factory extrinsic -- survives any remount)'
                       % (self._colour_frame, depth_frame, float(np.linalg.norm(self._t_cd))))

    def _missing(self):
        out = []
        if self._depth_msg is None:
            out.append('depth image')
        if self._Kd is None:
            out.append('depth camera_info')
        if self._R_cd is None:
            out.append('depth->colour TF')
        return out

    @staticmethod
    def _depth_metres(msg) -> np.ndarray:
        """(h, w) float32 metres from a depth Image. Shared by the numpy and the CUDA path, so the
        two cannot disagree about units, row padding or which encodings are accepted."""
        dw, dh = int(msg.width), int(msg.height)
        enc = (msg.encoding or '').lower()
        if enc in ('16uc1', 'mono16'):
            # Reshape through `step`, not width: a driver is free to pad rows, and reshaping to
            # (h, w) on a padded buffer either raises or shears the image diagonally -- the second
            # of which projects perfectly plausibly and is wrong everywhere.
            row = msg.step // 2 if msg.step else dw
            return (np.frombuffer(msg.data, np.uint16)
                    .reshape(dh, row)[:, :dw].astype(np.float32) * DEPTH_SCALE_MM)
        if enc == '32fc1':
            row = msg.step // 4 if msg.step else dw
            return np.frombuffer(msg.data, np.float32).reshape(dh, row)[:, :dw].astype(
                np.float32, copy=True)
        raise DepthSourceError(
            'depth encoding %r is not 16UC1/mono16 (mm) or 32FC1 (m). Guessing the unit here '
            'would scale every range by 1000.' % msg.encoding)

    def organized_cloud_cuda(self, cuda):
        """Camera override: the WHOLE chain on the GPU, starting from the metric depth image.

        This is where the saving is -- the numpy deprojection and its float64 matmul are the
        single most expensive CPU step in perception_node (see cuda_cloud's docstring).
        """
        if self._K is None:
            return None
        msg = self._depth_msg
        if msg is None or self._Kd is None or self._R_cd is None:
            return None
        depth_m = self._depth_metres(msg)           # DepthSourceError before the GPU is touched
        cloud = cuda.from_depth(depth_m, self._Kd, self._R_cd, self._t_cd, self._stride,
                                self._K, self._size, self._min_range, self._max_range)
        self._note_rasterised(_stamp_seconds(msg.header))
        return cloud

    def _points_in_colour_frame(self):
        msg = self._depth_msg
        if msg is None or self._Kd is None or self._R_cd is None:
            return None
        key = ('camera', self._depth_seq)

        dw, dh = int(msg.width), int(msg.height)
        depth_m = self._depth_metres(msg)

        s = self._stride
        vv, uu = np.mgrid[0:dh:s, 0:dw:s]
        zz = depth_m[0:dh:s, 0:dw:s]
        ok = np.isfinite(zz) & (zz > 0)
        if not ok.any():
            return np.zeros((0, 3), np.float32), _stamp_seconds(msg.header), key
        u = uu[ok].astype(np.float32)
        v = vv[ok].astype(np.float32)
        z = zz[ok]
        pts_d = np.stack([(u - self._Kd[0, 2]) * z / self._Kd[0, 0],
                          (v - self._Kd[1, 2]) * z / self._Kd[1, 1],
                          z], axis=1)
        pts_c = pts_d @ self._R_cd.T + self._t_cd
        return pts_c.astype(np.float32, copy=False), _stamp_seconds(msg.header), key

    def describe(self) -> str:
        return ('camera stereo depth (%s), registered to colour by the FACTORY depth<->colour TF. '
                'No calibration of ours is involved, so a remount cannot invalidate it. Measured '
                'indoors 2026-09-08: 88.4%% of pixels valid at 30 Hz. NOT yet validated in direct '
                'sunlight -- the 336L is active stereo; see scripts/outdoor_depth_test.py.'
                % ('stride %d' % self._stride))


class LidarDepthSource(DepthSource):
    """Depth from the Mid-360, projected into the colour frame through OUR solved T_camera_lidar.

    Sparse but sunlight-proof, which is the whole reason it is kept alive as an option: measured
    indoors 2026-09-08 it gave 13.5 % coverage in the camera FOV and NOTHING in the lower third
    (the sensor's vertical floor is about -7 deg), against the camera's 88.4 % -- but a LiDAR does
    not care how bright the sun is.

    THE COST OF CHOOSING THIS, STATED PLAINLY: it depends on `calib/drone_cam_livox.yaml`, which
    describes one physical mounting of two separate devices. Every remount, re-tape, or hard landing
    invalidates it, and a stale extrinsic does not fail loudly -- it projects in-view and reports
    confident wrong ranges (calib_io's module docstring is the long version).
    """

    name = 'lidar'

    def __init__(self, node, T_camera_lidar, cloud_topic='/cloud_registered_body',
                 odom_topic='/lidar_slam/odom', accum_s=LIDAR_ACCUM_S,
                 fastlio_node=FASTLIO_NODE_FALLBACK, check_body_frame=True, **kw):
        super().__init__(node, **kw)
        T = np.asarray(T_camera_lidar, dtype=np.float64)
        if T.shape != (4, 4):
            raise DepthSourceError('T_camera_lidar must be 4x4, got %s' % (T.shape,))
        self._R_cl = T[:3, :3]
        self._t_cl = T[:3, 3]
        self._accum_s = float(accum_s)
        self._cloud_topic = cloud_topic

        self._sweeps = deque(maxlen=LIDAR_MAX_SWEEPS)   # (stamp_s, points Nx3 body)
        self._odom = deque(maxlen=LIDAR_ODOM_HISTORY)   # (stamp_s, T_world_body 4x4)
        self._sweep_seq = 0

        # Body-frame check state -- see _maybe_check_body_frame.
        self._check_state = 'pending' if check_body_frame else 'skipped'
        self._check_tries = 0
        self._check_client = None
        self._check_srv_type = None
        self._check_target = None
        self._fastlio_fallback = fastlio_node

        node.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        node.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        # config/mid360.yaml:56 ships `scan_bodyframe_pub_en: false`, so this topic does not exist
        # yet. Say so at start-up: the symptom otherwise is a valid subscription that never fires,
        # which reads as a hardware fault rather than as a one-line config change. That file is
        # owned outside this package, so this warns rather than fixing.
        self._log.warn(
            'depth_source[lidar] subscribing %s. NOTE config/mid360.yaml:56 ships '
            '`scan_bodyframe_pub_en: false`, which means FAST-LIO does NOT publish this topic. '
            'If no cloud arrives, that key is why -- set it to true and relaunch fast_lio.'
            % cloud_topic)

    # -- "is FAST-LIO's body frame actually the LiDAR frame?" ---------------------------------
    #
    # WHY THIS CHECK EXISTS: `/cloud_registered_body` is expressed in FAST-LIO's IMU body frame,
    # while `T_camera_lidar` is defined against the LIDAR frame. The two are the same frame ONLY
    # because config/mid360.yaml:45-46 sets the LiDAR->IMU extrinsic to identity. If anyone ever
    # measures the real offset and sets it -- a correct and likely thing to do -- every range out
    # of this source shifts by that offset, silently, everywhere, with the cloud still projecting
    # plausibly into the image. Same failure shape as a mirrored extrinsic (calib_io) and the
    # wrong camera body (check_camera_serial): valid inputs, confident wrong ranges.
    #
    # WHY IT RUNS ON THE FIRST CLOUD RATHER THAN IN __init__:
    #   * at construction fast_lio may legitimately not be up, and DDS discovery has not run, so
    #     the check would mostly report "cannot determine" and teach people to ignore it;
    #   * the question is meaningless until LiDAR data actually flows;
    #   * it lets us resolve WHICH node to ask from the cloud topic's own publisher endpoint.
    #     calib_io.check_lidar_source documents why a hardcoded node name is wrong in both
    #     directions -- ROS 2 does not enforce unique node names, so the name you query may not be
    #     the node that published your data. The endpoint is the only identity that is actually
    #     tied to the cloud we consume. FASTLIO_NODE_FALLBACK is used only when DDS reports no
    #     name, and it is a GUESS: no file in this repo records fast_lio's runtime node name.
    #
    # WHY A MISMATCH DISABLES THE SOURCE INSTEAD OF RAISING: this runs inside a subscription
    # callback, where raising would unwind through the executor and kill the process with a
    # traceback -- possibly mid-flight. Producing NO range is the fail-closed equivalent and
    # composes with the loss-of-target handling that already exists (target_manager coasts, then
    # LOST; command_mux falls through on a stale source). A follower with no range is a handled
    # state; a follower with wrong range is not.
    #
    # Everything here is non-blocking. A service call from inside a callback must NOT spin the
    # node (re-entrant spin is illegal), so this polls service_is_ready() and uses call_async with
    # a done-callback; the executor completes it on a later iteration.

    def _maybe_check_body_frame(self) -> None:
        if self._check_state != 'pending':
            return
        self._check_tries += 1
        if self._check_tries > BODY_FRAME_CHECK_TRIES:
            self._check_state = 'unverified'
            self._warn_once(
                'body_frame_unverified',
                'could not read mapping.extrinsic_T from fast_lio after %d clouds -- UNVERIFIED '
                'that its body frame is the LiDAR frame. If that extrinsic is ever non-identity, '
                'every range from this source is offset by it with no error anywhere.'
                % BODY_FRAME_CHECK_TRIES)
            return

        if self._check_client is None:
            target = self._resolve_cloud_publisher()
            if target is None:
                return
            try:
                from rcl_interfaces.srv import GetParameters
            except ImportError:
                self._check_state = 'unverified'
                return
            self._check_target = target
            self._check_srv_type = GetParameters
            self._check_client = self._node.create_client(
                GetParameters, target.rstrip('/') + '/get_parameters')

        if not self._check_client.service_is_ready():
            return
        req = self._check_srv_type.Request()
        req.names = ['mapping.extrinsic_T']
        self._check_state = 'in_flight'
        self._check_client.call_async(req).add_done_callback(self._on_body_frame_result)

    def _resolve_cloud_publisher(self):
        try:
            infos = self._node.get_publishers_info_by_topic(self._cloud_topic)
        except Exception:
            infos = []
        for i in infos:
            name = (i.node_name or '').strip()
            if name and name != '_NODE_NAME_UNKNOWN_':
                return '%s/%s' % ((i.node_namespace or '').rstrip('/'), name)
        if infos:
            self._warn_once(
                'cloud_pub_unnamed',
                'DDS reported no node name for the publisher on %s; falling back to %s, which is '
                'a GUESS -- no file in this repo records fast_lio\'s runtime node name.'
                % (self._cloud_topic, self._fastlio_fallback))
        return self._fastlio_fallback or None

    def _on_body_frame_result(self, fut) -> None:
        try:
            res = fut.result()
            vals = list(res.values[0].double_array_value) if res and res.values else []
        except Exception:
            vals = []
        if not vals:
            # Retry on the next cloud: the node may have answered before its parameters were
            # declared, which is a transient the caller should not have to care about.
            self._check_state = 'pending'
            return
        offset = float(np.linalg.norm(np.asarray(vals, dtype=float)))
        if offset > BODY_FRAME_TOL_M:
            self._check_state = 'failed'
            self._log.error(
                'depth_source[lidar] DISABLED -- FAST-LIO\'s body frame is NOT the LiDAR frame.\n'
                '  %s mapping.extrinsic_T : %s   (|T| = %.4f m)\n'
                '  /cloud_registered_body is published in the IMU body frame; T_camera_lidar is\n'
                '  defined against the LIDAR frame. They coincide only while that extrinsic is\n'
                '  identity, which config/mid360.yaml:45 currently makes it.\n'
                '  Every range would be wrong by %.3f m and nothing downstream would notice, so\n'
                '  this source will now report NO range rather than a confident wrong one.\n'
                '  Fix: compose the LiDAR->IMU extrinsic into T_camera_lidar, or run\n'
                '  depth_source:=camera.'
                % (self._check_target, [round(v, 6) for v in vals], offset, offset))
            return
        self._check_state = 'ok'
        self._log.info('depth_source[lidar] body frame verified == LiDAR frame '
                       '(%s mapping.extrinsic_T is identity)' % self._check_target)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._sweeps.append((_stamp_seconds(msg.header), cloud_xyz(msg)))
        self._sweep_seq += 1
        self._maybe_check_body_frame()

    def _on_odom(self, msg: Odometry) -> None:
        T = np.eye(4)
        T[:3, :3] = quat_to_R(msg.pose.pose.orientation)
        p = msg.pose.pose.position
        T[:3, 3] = (p.x, p.y, p.z)
        self._odom.append((_stamp_seconds(msg.header), T))

    def _missing(self):
        out = []
        if self._check_state == 'failed':
            # Not "waiting for" anything -- deliberately dead. Reported through the same channel so
            # a caller polling `ready` cannot mistake a disabled source for a healthy one.
            return ['DISABLED: fast_lio body frame is not the LiDAR frame (see the error above)']
        if not self._sweeps:
            out.append('%s (see scan_bodyframe_pub_en in config/mid360.yaml)' % self._cloud_topic)
        if not self._odom:
            out.append('/lidar_slam/odom')
        return out

    def _pose_at(self, stamp_s):
        """Odom pose nearest `stamp_s`, or None if none is close enough.

        Nearest-in-time is sufficient rather than lazy: fast_lio publishes the registered cloud and
        the odometry from the same iteration with the same header stamp, so the match is essentially
        exact. ODOM_MATCH_TOL_S exists to catch the case where that stops being true.
        """
        best, best_dt = None, ODOM_MATCH_TOL_S
        for t, T in self._odom:
            dt = abs(t - stamp_s)
            if dt <= best_dt:
                best, best_dt = T, dt
        return best

    def _points_in_colour_frame(self):
        if self._check_state == 'failed' or not self._sweeps:
            return None
        newest_t, newest_pts = self._sweeps[-1]
        key = ('lidar', self._sweep_seq)

        T_wn = self._pose_at(newest_t)
        if T_wn is None:
            # Correct-but-sparse beats plausible-but-wrong: without a pose for the newest sweep we
            # cannot register the older ones onto it, so we use the newest alone. One sweep in its
            # own frame needs no registration and is exactly right, just thin.
            self._warn_once('no_odom_match',
                            'no /lidar_slam/odom sample within %.0f ms of the newest sweep; '
                            'falling back to a SINGLE sweep (no accumulation). Ranges stay correct '
                            'but coverage on a person drops hard.' % (ODOM_MATCH_TOL_S * 1e3))
            return self._to_colour(newest_pts), newest_t, key

        T_nw = _inverse_rigid(T_wn)
        chunks = [newest_pts]
        for t, pts in list(self._sweeps)[:-1]:
            if newest_t - t > self._accum_s or pts.size == 0:
                continue
            T_wo = self._pose_at(t)
            if T_wo is None:
                # Dropped, not stacked un-registered. Stacking a sweep we cannot place would smear
                # the person across however far the aircraft travelled since -- 0.6 m at 3 m/s over
                # two sweeps, which is a quarter of the whole standoff band.
                continue
            T_no = T_nw @ T_wo
            chunks.append(pts @ T_no[:3, :3].T.astype(np.float32)
                          + T_no[:3, 3].astype(np.float32))
        return self._to_colour(np.vstack(chunks)), newest_t, key

    def _to_colour(self, pts_lidar):
        if pts_lidar.size == 0:
            return np.zeros((0, 3), np.float32)
        return (pts_lidar @ self._R_cl.T.astype(np.float32)
                + self._t_cl.astype(np.float32)).astype(np.float32, copy=False)

    def describe(self) -> str:
        return ('Mid-360 via OUR T_camera_lidar, %.2f s of FAST-LIO-deskewed body-frame sweeps '
                're-registered through /lidar_slam/odom. Sunlight-proof, but measured indoors '
                '2026-09-08 at 13.5%% FOV coverage with nothing below about -7 deg, and EVERY '
                'REMOUNT INVALIDATES THE EXTRINSIC (recalibrate, see plan/dtp-imp/19).'
                % self._accum_s)


def _inverse_rigid(T):
    """Inverse of a rigid 4x4. Transpose-and-negate, not np.linalg.inv: it is exact for a rotation
    and cannot introduce the small non-orthonormality that a general inverse can."""
    out = np.eye(4)
    R = T[:3, :3]
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def create(node, kind, T_camera_lidar=None, **kw) -> DepthSource:
    """Build the depth source named by `kind` ('camera' or 'lidar').

    An unrecognised name RAISES. It is never quietly coerced to a default: `depth_source:=lidr`
    silently falling back to the camera would mean an outdoor test that appears to run the LiDAR
    path and actually measures the camera -- and the result would be believed.
    """
    kind = (kind or '').strip().lower()
    if kind == 'camera':
        return CameraDepthSource(node, **kw)
    if kind == 'lidar':
        if T_camera_lidar is None:
            raise DepthSourceError(
                "depth_source 'lidar' needs T_camera_lidar. Load it with "
                "calib_io.load_for_node(node, calib_file) and pass it in -- there is no identity "
                "fallback, because an identity extrinsic treats the camera and LiDAR as co-located "
                "and makes every range wrong while overlays still look plausible.")
        return LidarDepthSource(node, T_camera_lidar, **kw)
    raise DepthSourceError(
        'unknown depth_source %r -- expected one of %s. Refusing to guess: silently defaulting '
        'would run one sensor while the log claims the other.' % (kind, ', '.join(SOURCES)))
