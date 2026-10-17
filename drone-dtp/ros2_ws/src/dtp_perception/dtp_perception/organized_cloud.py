"""organized_cloud -- the (H, W, 3) camera-frame cloud, and the PointCloud2 wire format for it.

Pure functions. No node, no subscriptions, no module-level mutable state, so the same code runs
inside `perception_node`'s hot path, inside `fusion_node`'s debug publisher, and inside a unit
test with nothing else running.

WHY THIS IS A LIBRARY RATHER THAN A TOPIC
-----------------------------------------
One organized cloud at colour resolution is 1280 x 720 x 3 x 4 B = **11.06 MB per message**. At
the 15 Hz detection rate that is 166 MB/s through DDS. This box has already been measured failing
an order of magnitude below that: a 2.76 MB raw colour message at 30 Hz (~83 MB/s) lost ~8 % of
frames in an rclpy subscriber (plan/dtp-imp/21, 2026-09-07), and the fix was to stop sending it.
There is no compressed transport for a PointCloud2, and under BEST_EFFORT an 11 MB sample
fragments into thousands of UDP datagrams where losing any one of them loses the whole sample.

So the cloud is never published into the perception path. `perception_node` calls `build()` in its
own process and hands the numpy array straight to `process_frame` with no serialization at all --
which is what plan/dtp-imp/14 C3 (RED), risk A4 (RED), plan/dtp-imp/06 M1.4.1 and
plan/dtp-imp/07:151 all already prescribe ("consolidate perception into a single process"). It is
a change to plan/dtp-imp/01 SS2's node/topic table, which still lists /dtp/points_fused as a
perception_node INPUT, and needs correcting there.

`to_pointcloud2` exists anyway, because the RViz / `ros2 bag` / replay-harness view is real and the
layout is a contract the harness replays bags against. It is a DEBUG path (see fusion_node), not
the perception path.

WHAT THE ARRAY IS
-----------------
`build()` returns (H, W, 3) float32, C-contiguous, in the CAMERA COLOUR OPTICAL frame --
x right, **y DOWN**, z forward (REP-103 optical), frame_id `camera_color_optical_frame`.

*** NO AXIS REMAP IS APPLIED HERE AND NONE BELONGS HERE. *** `process_frame` reads channels 0 and
2 only, which are identical in the optical and the "pipeline" conventions plan/dtp-imp/01 SS3
describes; only channel 1's SIGN differs, and `process_frame` never reads it. The remap SS3 asks
for is real but belongs on the WORLD side, because `_camera_xz_to_odom_xz` assumes a Y-up world
while `camera_init` is Z-up. Correct SS3 in place rather than remapping the cloud: remapping here
would silently negate the one channel the height lift depends on.

RANGE IS EUCLIDEAN, SO THE RAYS MUST BE UNIT RAYS
-------------------------------------------------
`DepthSource.range_image()` stores `np.linalg.norm(p)`, NOT optical Z (depth_source.py:430). So
the cloud is NOT `z * [(u-cx)/fx, (v-cy)/fy, 1]`; it is `range * normalize([(u-cx)/fx, (v-cy)/fy,
1])`. The colour FOV is 90 deg horizontal, so treating a Euclidean range as a Z re-introduces a
1/cos(45 deg) = 1.41x error at the horizontal frame edge -- an edge-of-frame person would come out
40 % nearer than they are, which is exactly the error depth_source exists to remove. (At the image
CORNER of this 1280x720 / fx 611.2 / fy 611.3 calibration it is worse still: 1.56x.) The unit-ray
form is the exact inverse of the rasterisation that produced the range image, so the round trip is
lossless bar the sub-pixel truncation in range_image's `u.astype(np.intp)`: at most one pixel,
i.e. range/fx ~= 1.7 mm per metre, nothing against a torso centroid.

`build()` computes that WITHOUT materialising the (H, W, 3) ray table, and the algebra is worth
stating because the factored form does not look like the formula above:

    unit_ray = [xn, yn, 1] / n     with  xn = (u-cx)/fx,  yn = (v-cy)/fy,  n = hypot(xn, yn, 1)
    cloud    = range * unit_ray    =  [s*xn, s*yn, s]     with  s = range / n

`xn` varies only with the column and `yn` only with the row, so the only (H, W) intermediate is
`s`. That removes an 11 MB per-intrinsics cache -- and with it the question of who owns it, which
is the whole reason this file has no module state. It is not a speed/memory trade either: measured
at 1280x720 on a development x86 host (NOT the NX, so treat these as a floor), `build()` end to
end takes 14.9-17.0 ms, against 14.6 ms for the cached form's broadcast multiply ALONE, before the
table is built or held.

The two forms differ only by float32 rounding: measured max |build - range*unit_ray| = 9.5e-07 m
over 1.35 M samples spanning 0.5-12 m, i.e. micrometres, which is far below perception_node's 1 mm
PATCH_IDENTITY_TOL_M tripwire and far below any real disagreement. `unit_rays()` below builds the
explicit table for tests and for anyone re-checking that claim.

NO RETURN IS NaN IN ALL THREE CHANNELS -- NEVER 0.0, NEVER A SENTINEL
---------------------------------------------------------------------
The consumer's validity test is `isfinite(x) & isfinite(z) & (z > 0)` (detection.py:468). A 0.0
fill is rejected by `z > 0` but is ACCEPTED as a finite x, and any positive sentinel is accepted
outright and poisons the median -- i.e. a "nothing here" pixel would read downstream as a person
standing on top of the aircraft, which is the one reading that makes a follower brake or veto.

Because the cloud is `range * unit_ray` and the ray table contains no NaN, every pixel's three
channels are JOINTLY finite or JOINTLY NaN. That is load-bearing, not incidental: it is what makes
perception_node's height sample unable to fail on a frame where the position sample succeeded.
Anything that ever writes one channel independently of the other two breaks it.

FAILURES RAISE
--------------
`build()` and `from_pointcloud2()` raise ValueError on geometry they cannot vouch for -- a
degenerate CameraInfo, a mismatched shape, a wrong stride. A library that returned an empty or
zero-filled cloud instead would be handing `process_frame` a confident median over invented
geometry, which is the failure this whole file is arranged to prevent.

*** BOTH CALLERS RUN INSIDE ROS CALLBACKS AND MUST CATCH IT. *** An uncaught raise unwinds through
the executor and kills the process -- and for perception_node that is worse than a dropped frame,
because a silent /dtp/targets pins command_mux in HOLD forever. Use `intrinsics_problem()` to gate
cheaply before calling when you want to log a throttled reason instead.
"""
import array

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField

# ---------------------------------------------------------------------------------------------
# The wire layout. Every one of these is load-bearing, not a default.
# ---------------------------------------------------------------------------------------------
# 12 bytes per point: x, y, z as float32 with NO padding. PCL's PointXYZ pads to 16 for SSE
# alignment and we must not, because a C-contiguous (H, W, 3) float32 array IS the 12-byte-stride
# layout byte for byte -- so `from_pointcloud2` is a ZERO-COPY view. A 16-byte stride would instead
# force a strided copy of 11 MB per frame in Python, which is the cost this pipeline cannot pay.
POINT_STEP_BYTES = 12
FIELD_NAMES = ('x', 'y', 'z')
# datatype id 7 == PointField.FLOAT32. Named here so `from_pointcloud2` can state the expected
# triple without importing the constant into every comparison.
_FLOAT32 = PointField.FLOAT32

# A no-return pixel. See the module docstring -- NaN, never 0.0, never a sentinel.
NO_RETURN = np.float32('nan')


def intrinsics_problem(k_colour, shape=None):
    """Return a human-readable reason `build()` would refuse these intrinsics, or None.

    Exists so a ROS callback can gate and log with a throttle instead of catching an exception on
    a condition that is NORMAL at start-up: a driver publishes an all-zero K before its calibration
    is loaded, and dividing by fx = 0 would fill the whole cloud with inf/NaN and produce a cloud
    that is merely EMPTY rather than obviously broken -- "the scene is empty" reads downstream as a
    fact about the world, not as a configuration fault.
    """
    K = np.asarray(k_colour, dtype=np.float64)
    if K.shape != (3, 3):
        return 'colour K is %s, expected 3x3' % (K.shape,)
    if not np.all(np.isfinite(K[:2, :3])):
        return 'colour K contains a non-finite focal length or principal point: %s' % (K[:2, :3],)
    fx, fy = float(K[0, 0]), float(K[1, 1])
    if not (fx > 0.0 and fy > 0.0):
        return ('colour camera_info has a non-positive focal length (fx=%r fy=%r) -- almost '
                'always a driver that has not loaded its calibration yet' % (fx, fy))
    if shape is not None:
        h, w = int(shape[0]), int(shape[1])
        if h <= 1 or w <= 0:
            return ('range image is %dx%d (height x width); height must be > 1 or the cloud is '
                    'UNORGANIZED and process_frame cannot slice a patch out of it' % (h, w))
    return None


def build(range_image, k_colour):
    """Deproject a colour-frame range image into (H, W, 3) float32, camera colour optical frame.

    `range_image` is `DepthSource.range_image()`: (H, W) float32 nearest-return EUCLIDEAN range,
    already registered into the colour frame by depth_source (the camera source applies the FACTORY
    depth->colour TF, the LiDAR source applies T_camera_lidar, and both rasterise with the colour
    K). Nothing further is needed here and nothing further is done here.

    `k_colour` is the 3x3 from `DepthSource.colour_intrinsics()`. Skew (K[0,1]) is ignored, exactly
    as depth_source's own forward projection ignores it (`u = fx * x / z + cx`) -- that is what
    makes this the exact INVERSE of the rasterisation rather than merely a similar model.

    A non-positive or NaN range becomes NaN in all three channels. depth_source's band gate already
    excludes anything below MIN_RANGE_M so a 0.0 cannot reach here today, but a 0.0 that ever did
    would multiply out to a point at the camera origin -- "person on top of us", the reading that
    makes command_mux veto -- so it is mapped explicitly rather than trusted not to occur.

    Raises ValueError on geometry that cannot be vouched for; see the module docstring.

    The returned array is always FRESHLY ALLOCATED, never a reused buffer, and that is a contract
    rather than an implementation detail: perception_node pins up to MAX_PAIR_AGE_S worth of clouds
    in its TF-wait deque, so a caller that got a recycled buffer would watch queued frames mutate
    under it. (`range_image()` itself DOES hand back a reused buffer -- its docstring says so --
    which is fine, because this function only reads it.)
    """
    rng = np.asarray(range_image, dtype=np.float32)
    if rng.ndim != 2:
        raise ValueError('range image must be (H, W), got %s' % (rng.shape,))
    h, w = int(rng.shape[0]), int(rng.shape[1])

    problem = intrinsics_problem(k_colour, rng.shape)
    if problem:
        raise ValueError(problem)
    K = np.asarray(k_colour, dtype=np.float64)

    # The per-axis normalised image coordinates, in float64 and then cast: these are W and H
    # elements, so the accuracy is free, and it keeps the result independent of NumPy's
    # value-based-vs-NEP50 scalar promotion rules, which differ between 1.x and 2.x and would
    # otherwise decide silently whether this ran in float32 or float64.
    xn = ((np.arange(w, dtype=np.float64) - K[0, 2]) / K[0, 0]).astype(np.float32)   # (W,)
    yn = ((np.arange(h, dtype=np.float64) - K[1, 2]) / K[1, 1]).astype(np.float32)   # (H,)

    # n = |[xn, yn, 1]|, the only (H, W) intermediate. See the algebra in the module docstring.
    n = np.sqrt((xn * xn)[None, :] + (yn * yn)[:, None] + np.float32(1.0), dtype=np.float32)
    scale = np.divide(rng, n, dtype=np.float32)

    # `rng > 0.0` is False for NaN as well as for 0.0 and negatives, which covers both "no return"
    # and "impossible return"; `isfinite` additionally catches +inf, which `> 0.0` would let
    # through. That last one is not hypothetical bookkeeping -- an inf range would give z = inf and
    # x = inf * 0 = NaN on the principal-point column, i.e. one pixel whose channels are NEITHER
    # jointly finite NOR jointly NaN, and that joint property is what the height sample relies on.
    # NaN then propagates into all three channels for free below, because NaN * xn is NaN even
    # where xn is exactly 0.0.
    scale = np.where(np.isfinite(rng) & (rng > 0.0), scale, NO_RETURN)

    cloud = np.empty((h, w, 3), dtype=np.float32)
    np.multiply(scale, xn[None, :], out=cloud[:, :, 0])
    np.multiply(scale, yn[:, None], out=cloud[:, :, 1])
    cloud[:, :, 2] = scale
    return cloud


def unit_rays(k_colour, width, height):
    """(H, W, 3) float32 table of UNIT ray directions, one per colour pixel.

    NOT used by `build()` -- it is 11 MB and `build()` is arranged to avoid materialising it. This
    exists so a test (or a person) can check `build()` against the literal
    `range * normalize([(u-cx)/fx, (v-cy)/fy, 1])` the contract is written in, and so the two forms
    can be diffed if anyone ever doubts the factoring. Allocate it in a loop at your peril.
    """
    problem = intrinsics_problem(k_colour, (height, width))
    if problem:
        raise ValueError(problem)
    K = np.asarray(k_colour, dtype=np.float64)
    w, h = int(width), int(height)
    xn = ((np.arange(w, dtype=np.float64) - K[0, 2]) / K[0, 0]).astype(np.float32)
    yn = ((np.arange(h, dtype=np.float64) - K[1, 2]) / K[1, 1]).astype(np.float32)
    x = np.broadcast_to(xn[None, :], (h, w))
    y = np.broadcast_to(yn[:, None], (h, w))
    n = np.sqrt(x * x + y * y + np.float32(1.0), dtype=np.float32)
    return np.ascontiguousarray(np.stack([x / n, y / n, np.float32(1.0) / n], axis=2),
                                dtype=np.float32)


def to_pointcloud2(cloud, header):
    """(H, W, 3) float32 -> an ORGANIZED sensor_msgs/PointCloud2. DEBUG PATH ONLY.

    `header` supplies the stamp and the frame_id, and both are contracts:

      * the stamp must be the COLOUR image's stamp, never `now()` and never the depth stamp.
        perception_node looks up T_world_camera AT this stamp (risk A8 / R-F5: "latest" costs
        yaw_rate x range, ~0.5 m at 3 rad/s and 6 m), and a replay harness re-times everything from
        it. A wall-clock stamp here would certify arbitrarily old geometry as current, everywhere
        downstream, with no error on any topic.
      * the frame_id must be the colour message's OWN, read rather than asserted. An empty frame_id
        RAISES rather than defaulting: a cloud labelled with a name nobody read is how a TF chain
        ends up resolving against a frame the data is not in, and the caller is the only one that
        can log which stream produced it.
    """
    arr = np.asarray(cloud)
    if arr.dtype != np.float32 or arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError('cloud must be (H, W, 3) float32, got %s %s' % (arr.shape, arr.dtype))
    h, w = int(arr.shape[0]), int(arr.shape[1])
    if h <= 1:
        raise ValueError(
            'cloud height is %d; an organized cloud needs height > 1, which is the property '
            'process_frame depends on when it does `h, w = point_cloud_xyz.shape[:2]`' % h)
    frame_id = (header.frame_id or '').strip()
    if not frame_id:
        raise ValueError(
            'refusing to publish a cloud with an empty frame_id -- see to_pointcloud2')

    out = PointCloud2()
    out.header.stamp = header.stamp
    out.header.frame_id = frame_id
    # height > 1 is what makes this cloud ORGANIZED.
    out.height = h
    out.width = w
    # Fresh PointField objects per message rather than a module-level list: these are mutable ROS
    # message objects, and a shared list would let one caller's edit reach every other publisher in
    # the process. Three tiny constructions against 11 MB of payload is not a cost worth having an
    # opinion about.
    out.fields = [PointField(name=n, offset=i * 4, datatype=_FLOAT32, count=1)
                  for i, n in enumerate(FIELD_NAMES)]
    out.is_bigendian = False
    out.point_step = POINT_STEP_BYTES
    out.row_step = POINT_STEP_BYTES * w
    # MANDATORY False. NaN is present by design here (a no-return pixel), and is_dense=True is a
    # promise PCL consumers act on by SKIPPING the finiteness check -- they would then treat NaN as
    # a coordinate.
    out.is_dense = False

    # array.array('B', ...), NOT the bytes object directly. rosidl's generated setter for a uint8
    # sequence takes a fast path only for an array.array of typecode 'B'; anything else falls into
    # an `all(isinstance(v, int) ...)` assertion under __debug__, which for 11 MB is eleven million
    # Python-level checks per message. array.array's bytes constructor is a memcpy. This is the
    # difference between a 2 Hz debug topic and a hung node.
    out.data = array.array('B', np.ascontiguousarray(arr).tobytes())
    return out


def from_pointcloud2(msg):
    """Zero-copy (H, W, 3) float32 view of an organized cloud. Raises ValueError otherwise.

    The exact inverse of `to_pointcloud2`, and it lives beside it deliberately: a reader and a
    writer that agree only by coincidence is how the replay harness ends up feeding `process_frame`
    a sheared image.

    EVERY FIELD IS VERIFIED RATHER THAN ASSUMED, because a wrong stride does not raise -- it shears
    the image diagonally and then projects perfectly plausibly. Same bug class depth_source records
    for its own `msg.step // 2`.

    `is_dense` is deliberately NOT checked here. It changes no byte of the payload and this reader
    always tests finiteness itself, so refusing on it would reject a usable bag; a caller with a
    logger should still warn, because is_dense=True is a false promise to any PCL consumer of the
    same topic.
    """
    if msg.is_bigendian:
        raise ValueError('cloud is big-endian; nothing in this stack is')
    if msg.height <= 1:
        raise ValueError('cloud is UNORGANIZED (height=%d). process_frame indexes it by pixel: '
                         'point_cloud_xyz[py1:py2, px1:px2, :3]' % msg.height)
    if msg.point_step != POINT_STEP_BYTES or msg.row_step != POINT_STEP_BYTES * msg.width:
        raise ValueError(
            'cloud has point_step=%d row_step=%d; this consumer needs %d and %d (no padding to 16 '
            '-- a 16-byte stride forces a strided copy of 11 MB per frame)'
            % (msg.point_step, msg.row_step, POINT_STEP_BYTES, POINT_STEP_BYTES * msg.width))
    got = {f.name: (f.offset, f.datatype, f.count) for f in msg.fields}
    for i, name in enumerate(FIELD_NAMES):
        if got.get(name) != (i * 4, _FLOAT32, 1):
            raise ValueError('cloud field %r is %r; needs (offset %d, FLOAT32, 1)'
                             % (name, got.get(name), i * 4))
    need = int(msg.height) * int(msg.row_step)
    if len(msg.data) < need:
        raise ValueError('cloud data is %d bytes, needs %d' % (len(msg.data), need))
    flat = np.frombuffer(msg.data, dtype=np.float32, count=need // 4)
    return flat.reshape(int(msg.height), int(msg.width), 3)
