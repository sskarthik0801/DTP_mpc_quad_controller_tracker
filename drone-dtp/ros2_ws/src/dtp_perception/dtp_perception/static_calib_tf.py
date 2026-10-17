"""static_calib_tf -- publish the ONE static transform without which nothing in DTP can resolve.

*** THE TF CHAIN DOES NOT EXIST ON THIS AIRCRAFT WITHOUT THIS NODE. ***

State of the frame graph as recorded in this repository on 2026-09-08, before this file existed:

    camera_init  --> <body>                      FAST-LIO. On /lidar_slam/odom for certain;
                                                 whether ALSO on /tf is UNVERIFIED here (see the
                                                 last section).
    camera_link  --> camera_color_optical_frame  Orbbec driver, /tf_static. Verified.
    <body>       --> the camera                  NOTHING PUBLISHES THIS.

So `lookup_transform('camera_init', 'camera_color_optical_frame', t)` FAILS, which means
perception_node cannot lift a single detection into the world. Today's
dtp_bringup/launch/follower.launch.py already states this and deliberately declines to invent the
transform; this node is where it is invented, from the one record of that relationship that exists.

WHERE THE NUMBERS COME FROM, AND WHY THEY ARE NOT TYPED IN HERE
----------------------------------------------------------------
`calib/drone_cam_livox.yaml` holds `T_camera_lidar`: the 4x4 mapping the Livox Mid-360 frame into
the camera colour optical frame, solved by the direct_visual_lidar_calibration workflow
(plan/dtp-imp/19) and gated by dtp_perception.calibration.score_alignment. It is loaded through
`calib_io.load_for_node`, never read directly and never transcribed, because that call is not a
file read -- it is the start-up identity gate:

  * it refuses a calibration belonging to a DIFFERENT camera body (two Orbbec bodies were attached
    to this NX on 2026-09-07 and the driver bound whichever it enumerated first), and
  * it refuses a LiDAR stream coming from another robot on the same DDS domain (which happened on
    2026-09-07 and looked entirely plausible).

Both matter more here than anywhere else: this node turns a calibration into a TF that EVERY world
position in the stack is computed through. A wrong extrinsic here does not fail -- it puts every
person in the wrong place, confidently, forever.

THE BODY FRAME IS THE IMU, NOT THE LIDAR, AND THAT OFFSET IS COMPOSED IN HERE
-----------------------------------------------------------------------------
The calibration is LiDAR -> camera. What we need is BODY -> camera, and FAST-LIO's body frame is
its IMU frame: `/lidar_slam/odom` reports the pose of the IMU, and `mapping.extrinsic_T` /
`extrinsic_R` are the LiDAR pose IN that IMU frame (p_imu = R p_lidar + T -- laserMapping.cpp's
`Lidar_T_wrt_IMU`, and pointBodyToWorld applies exactly that before the world rotation). The two
frames coincide only while that extrinsic is identity. It is NOT identity on this aircraft:
the deployed FAST-LIO config carries the Mid-360's datasheet offset [-0.011, -0.02329, 0.04412]
(|T| = 0.051 m), and this node's own runtime check reported the published transform "WRONG BY
0.051 m" on every launch from 2026-09-08 to 2026-09-11. This repo's config/mid360.yaml says
identity; that copy is not the one fast_lio runs from and it misled two docstrings.

So the published transform is COMPOSED:

    T_body__optical = T_body__lidar @ inverse(T_camera__lidar)

where T_body__lidar comes from the `lidar_to_body_t` / `lidar_to_body_r` parameters, which must
hold the SAME numbers as FAST-LIO's `mapping.extrinsic_T` / `extrinsic_R`. That duplication is the
price of publishing a static transform before FAST-LIO is necessarily up; the guard against the
two drifting apart is _check_fastlio_extrinsic, which reads FAST-LIO's live parameters and logs at
ERROR, repeatedly, if they differ from what was composed. The defaults are identity, so a launch
that forgets the parameters fails LOUDLY (the 0.051 m message) rather than quietly.

depth_source.LidarDepthSource still checks for identity on the LiDAR DEPTH path (not the default)
and refuses a non-identity extrinsic there; that path projects /cloud_registered_body, which is in
the IMU frame, through T_camera_lidar and has no composition yet.

TWO PARENTS IS THE TRAP, AND IT IS WHY THIS FILE IS LONGER THAN ONE static_transform_publisher
-----------------------------------------------------------------------------------------------
The obvious implementation -- `static_transform_publisher <body> camera_color_optical_frame` -- is
WRONG on this aircraft, and wrong in the worst way: intermittently.

The Orbbec driver already publishes a parent for `camera_color_optical_frame` (camera_link, via its
own optical-frame chain). tf2 is a TREE: each frame has exactly one parent, and tf2's static cache
is keyed on the CHILD, so a second static transform naming the same child does not error -- it
OVERWRITES. Both publishers are TRANSIENT_LOCAL, so each subscriber receives both on subscription
match in whatever order discovery hands them over. Win the race and the lookup works; lose it and
the driver's `camera_link` parent is restored, `camera_init` is unreachable, and perception_node
publishes empty TargetArrays with nothing saying why. Per-process, per-run, nondeterministic.

So this node finds the ROOT of whatever chain already claims the camera frame and bridges to THAT
root instead, composing the driver's own published chain into the transform:

    T_body__root = T_body__optical @ inverse(T_root__optical)

The composite `body -> ... -> camera_color_optical_frame` is identical either way; the difference
is that the tree stays a tree. When nothing claims the camera frame yet (driver not up, or a bench
rig with no camera at all) it publishes `body -> camera_frame` directly, which is the contract's
literal form, and says in the log that a driver starting LATER will create the two-parent
condition. `camera_tf_wait_s` is how long it waits for the driver before deciding.

THE BODY FRAME NAME IS A PARAMETER WITH NO DEFAULT, ON PURPOSE
--------------------------------------------------------------
It is `/lidar_slam/odom`'s `child_frame_id`, and NOTHING IN THIS REPOSITORY RECORDS IT. Upstream
FAST_LIO forks conventionally use the literal "body", but that is a convention, not a record. A
wrong-but-plausible frame id fails the lookup in exactly the same way as an obviously wrong one
while LOOKING like it had been checked, so this node refuses to start rather than guess. Read it
off the aircraft:

    ros2 topic echo --field child_frame_id /lidar_slam/odom
    ros2 run tf2_tools view_frames

Then it VERIFIES the answer against `/lidar_slam/odom` at runtime (see _on_odom), because a
parameter that was set once and drifted is no better than a guess.

WHAT THIS NODE CANNOT FIX
-------------------------
Whether FAST-LIO broadcasts `camera_init -> <body>` on /tf AT ALL is unverified in this repo:
fast_lio is an external package (Ericsii/FAST_LIO@ros2, workspace.repos) and is not in this tree.
If it publishes only `/lidar_slam/odom` and no TF, this transform is necessary but NOT sufficient
and something must bridge odometry to TF. _check_chain answers that question at runtime instead of
leaving it for someone to rediscover downstream as "perception publishes empty arrays".

Topics:
  Publishes:  /tf_static (one TransformStamped, latched)
  Subscribes: /lidar_slam/odom (frame-name verification only), /tf, /tf_static
"""
import math

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from dtp_perception import calib_io, depth_source

# ---------------------------------------------------------------------------------------------
# Start-up timing
# ---------------------------------------------------------------------------------------------
# /tf_static is TRANSIENT_LOCAL: it is delivered on SUBSCRIPTION MATCH, which only happens while
# the node is spinning. So the driver's camera chain cannot be discovered without spinning first,
# and tf2's own lookup timeout will not do it for us -- that timeout blocks the calling thread on a
# condition variable and SPINS NOTHING, so the transform can never arrive and the call burns the
# whole timeout before raising "frame does not exist". This repo has hit that twice (see
# depth_source.wait_until_ready's docstring, and fusion_node.__init__). Everything below therefore
# spins explicitly and every lookup uses a ZERO timeout.
DEFAULT_CAMERA_TF_WAIT_S = 5.0
SPIN_SLICE_S = 0.05
# Zero, always, and for the reason above. Named rather than inlined so it cannot be "fixed" to a
# non-zero value by someone who reads the tf2 docs and not this comment.
TF_LOOKUP_TIMEOUT = Duration(seconds=0.0)

# ---------------------------------------------------------------------------------------------
# Health checks that run forever
# ---------------------------------------------------------------------------------------------
# How often to ask whether world -> camera actually resolves. Logged on TRANSITION only --
# sensor_watchdog's rule: a message that repeats at tick rate gets muted, and a muted message is
# worse than none. 2 s is slow enough to be free and fast enough that an operator watching a
# bring-up sees the answer while they are still looking at the terminal.
CHAIN_CHECK_PERIOD_S = 2.0
# Give the rest of the graph this long to appear before calling a broken chain an ERROR. FAST-LIO
# needs to initialise and the camera driver needs to enumerate USB; neither is a fault.
CHAIN_GRACE_S = 20.0

# Guard against a malformed /tf_static that contains a cycle: walking parents would not terminate.
# No real tree is anywhere near this deep -- the Orbbec's own chain is three links.
MAX_FRAME_DEPTH = 32

# ---------------------------------------------------------------------------------------------
# Numerical gates
# ---------------------------------------------------------------------------------------------
# The inverse and the quaternion are both checked by ROUND TRIP rather than trusted. Both are
# short, both are the kind of code that is wrong in a way that still LOOKS like a rotation, and
# both sit on the path every world position in the stack is computed through.
#
# THE TOLERANCE IS SET BY THE FILE, NOT BY THE ARITHMETIC, and getting that backwards is a bug
# this file already had. Measured on the shipped calib/drone_cam_livox.yaml:
#   * transpose-and-negate round trip |inv(T) @ T - I|      = 8.3e-10
#   * quaternion round trip |quat_to_R(R_to_quat(R)) - R|   = 1.3e-9
# while the same two operations over 20,000 EXACTLY orthonormal random rotations (plus the four
# 180 deg cases that exercise every branch of the Shepperd form) are worst 3.8e-15. So those
# residuals are the YAML's own 9-decimal quantisation -- calib_io's ORTHONORMAL_TOL comment says
# the file round-trips at 9 decimals -- and NOT float64 error. A 1e-9 gate would refuse to publish
# a perfectly good calibration.
#
# 1e-6 sits three orders above the file's quantisation and nine below an O(1) sign error, which is
# the failure being hunted. In physical terms 1e-6 of rotation is 10 um at 10 m; nothing that
# matters can hide beneath it.
ROUNDTRIP_TOL = 1e-6
# A hand-typed 3x3 in a YAML: 1e-6 is far looser than float64 and far tighter than any transposed
# or mis-signed block, which is what is being caught.
ROTATION_TOL = 1e-6
# How closely the composed LiDAR->body extrinsic must match what FAST-LIO is actually running.
# 0.1 mm / 1e-4 in rotation: the numbers are copied verbatim from the same YAML, so any real
# disagreement is a stale copy (tens of millimetres), not rounding.
EXTRINSIC_MATCH_TOL = 1e-4


class CalibTfError(RuntimeError):
    """Refusal to publish. Raised in the constructor; main() reports it and exits non-zero."""


def _inverse_rigid(T):
    """Inverse of a rigid 4x4. Transpose-and-negate, not np.linalg.inv.

    Exact for a rotation, and it cannot introduce the small non-orthonormality a general inverse
    can. Deliberately a copy of depth_source._inverse_rigid rather than an import: that name is
    module-private there, and reaching across for it would make a private name part of this file's
    contract -- the same rule fusion_node applies to `_stamp_seconds`. The duplication is safe
    because the result is round-tripped against the input below; a divergence cannot go unnoticed.
    """
    out = np.eye(4)
    R = T[:3, :3]
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def rigid_from_rt(t, r):
    """4x4 from a 3-vector and a row-major 9-vector, in FAST-LIO's own parameter layout.

    Validated as a rotation (orthonormal, det +1) rather than trusted: these numbers are typed
    into a YAML by hand, and a transposed or mistyped block is still nine plausible floats.
    """
    t = np.asarray(t, dtype=np.float64).reshape(-1)
    r = np.asarray(r, dtype=np.float64).reshape(-1)
    if t.shape != (3,) or r.shape != (9,):
        raise CalibTfError('lidar_to_body_t must have 3 values and lidar_to_body_r 9 (row-major); '
                           'got %d and %d' % (t.size, r.size))
    R = r.reshape(3, 3)
    orth = float(np.abs(R.T @ R - np.eye(3)).max())
    det = float(np.linalg.det(R))
    if orth > ROTATION_TOL or abs(det - 1.0) > ROTATION_TOL:
        raise CalibTfError('lidar_to_body_r is not a rotation (|R^T R - I| = %.2e, det = %.6f). '
                           'A transposed or mistyped block is still nine plausible numbers; '
                           'refusing to compose it into the only transform in the graph.'
                           % (orth, det))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def compose_body_optical(T_camera_lidar, lidar_to_body_t, lidar_to_body_r):
    """The transform this node publishes: body (IMU) -> camera optical frame.

    T_camera_lidar maps LiDAR-frame points into the camera optical frame (the calibration file's
    convention, checked by calib_io). (lidar_to_body_t, lidar_to_body_r) map LiDAR-frame points
    into the body frame (FAST-LIO's mapping.extrinsic_T / _R convention). A TransformStamped
    with parent=body, child=optical encodes optical -> body, so:

        T_body_optical = T_body_lidar @ T_lidar_optical = T_body_lidar @ inverse(T_camera_lidar)

    Pure so it can be unit-tested against a hand-built case without a ROS graph.
    """
    T_camera_lidar = np.asarray(T_camera_lidar, dtype=np.float64)
    T_lidar_optical = _inverse_rigid(T_camera_lidar)
    err = float(np.abs(T_lidar_optical @ T_camera_lidar - np.eye(4)).max())
    if err > ROUNDTRIP_TOL:
        raise CalibTfError(
            'inverse(T_camera_lidar) round trip is off by %.3e (limit %.1e) -- the inverse is '
            'wrong, or the file is not a rigid transform. Refusing to publish it.'
            % (err, ROUNDTRIP_TOL))
    T_body_lidar = rigid_from_rt(lidar_to_body_t, lidar_to_body_r)
    return T_body_lidar @ T_lidar_optical


def _R_to_quat(R):
    """3x3 rotation -> (x, y, z, w), the geometry_msgs order.

    Shepperd's branch: pick the largest of the four components to divide by, so the divisor is
    never near zero. The naive trace-only form loses all precision at 180 deg, and this extrinsic
    IS near a 180 deg pairing of axes -- T_camera_lidar's rotation block is essentially a signed
    axis permutation -- so the branch is load-bearing here, not defensive boilerplate.

    Verified by round trip against depth_source.quat_to_R at construction: a sign error in any one
    branch produces a perfectly valid unit quaternion describing a different rotation, which is
    exactly the class of mistake that would place every person in the wrong part of the world and
    still look like a rotation.
    """
    m00, m01, m02 = float(R[0, 0]), float(R[0, 1]), float(R[0, 2])
    m10, m11, m12 = float(R[1, 0]), float(R[1, 1]), float(R[1, 2])
    m20, m21, m22 = float(R[2, 0]), float(R[2, 1]), float(R[2, 2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


class _Q:
    """Minimal x/y/z/w holder so depth_source.quat_to_R can check our conversion without a
    geometry_msgs round trip. It takes anything with those four attributes."""

    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


class StaticCalibTf(Node):
    def __init__(self):
        super().__init__('static_calib_tf')

        # body_frame: NO DEFAULT, and the node refuses to start without it. See the module
        # docstring -- a plausible guess fails the lookup exactly like an implausible one while
        # looking checked, and this transform is what every world position is computed through.
        self.declare_parameter('body_frame', '')
        # camera_frame: the CHILD the calibration describes. This one is safe to default, because
        # unlike the body frame it IS recorded in this repository -- depth_source.COLOUR_OPTICAL_
        # FRAME is the string the Orbbec driver publishes and the string range_image() rasterises
        # into. Note this is the frame the calibration TARGETS, not necessarily the frame this node
        # ends up naming as the child; see _resolve_root.
        self.declare_parameter('camera_frame', depth_source.COLOUR_OPTICAL_FRAME)
        # world_frame: used ONLY to check that the chain resolves end to end, never published.
        # 'camera_init' is FAST-LIO's origin frame and the header.frame_id of /lidar_slam/odom;
        # config/dtp_params.yaml already sets odom_frame to it. Nothing named 'odom' exists here.
        # MEASURED ON THE AIRCRAFT 2026-09-08 from a live /lidar_slam/odom:
    #   header.frame_id  = lidar_init   (world)
    #   child_frame_id   = lidar_imu    (body)
    # NOT camera_init. That name was inferred from this file and recorded as "verified";
    # it was never read off the topic. Verify with:
    #   ros2 topic echo --field header.frame_id /lidar_slam/odom
        self.declare_parameter('world_frame', 'lidar_init')
        # LiDAR -> body (IMU) extrinsic, in FAST-LIO's own layout and convention (p_body = R p_lidar
        # + T): copy mapping.extrinsic_T / extrinsic_R from the mid360.yaml FAST-LIO RUNS FROM, not
        # from this repo's copy. Identity by default so a launch that forgets them fails loudly at
        # _on_extrinsic instead of publishing a plausible transform 51 mm off.
        self.declare_parameter('lidar_to_body_t', [0.0, 0.0, 0.0])
        self.declare_parameter('lidar_to_body_r', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('calib_file', calib_io.DEFAULT_CALIB)
        self.declare_parameter('camera_tf_wait_s', DEFAULT_CAMERA_TF_WAIT_S)
        # verify_odom: the runtime cross-check of body_frame and world_frame against
        # /lidar_slam/odom. Off only for a bench rig with no FAST-LIO, where the check would warn
        # forever about a node that is legitimately absent.
        self.declare_parameter('verify_odom', True)

        self._body_frame = str(self.get_parameter('body_frame').value or '').strip()
        self._camera_frame = str(self.get_parameter('camera_frame').value or '').strip()
        self._world_frame = str(self.get_parameter('world_frame').value or '').strip()
        if not self._body_frame:
            raise CalibTfError(
                'body_frame is UNSET and this node will not guess it.\n'
                '  It is the child_frame_id of /lidar_slam/odom, and nothing in this repository\n'
                '  records it. Upstream FAST_LIO forks conventionally use "body", but a\n'
                '  wrong-but-plausible frame id fails every lookup in exactly the way an\n'
                '  obviously wrong one does, while LOOKING checked -- and what fails is every\n'
                '  world position in the follow stack.\n'
                '  Read it off the aircraft and pass it:\n'
                '    ros2 topic echo --field child_frame_id /lidar_slam/odom\n'
                '    ros2 run tf2_tools view_frames\n'
                '    ros2 run dtp_perception static_calib_tf --ros-args -p body_frame:=<name>')
        if not self._camera_frame:
            raise CalibTfError('camera_frame is empty; it is the child frame the calibration '
                               'targets and there is nothing sensible to substitute')

        # The identity gates. See the module docstring: this is not a file read.
        T_camera_lidar, _intr = calib_io.load_for_node(
            self, self.get_parameter('calib_file').value)

        # The direction, spelled out because it is the single easiest thing to get backwards here
        # and a mirrored extrinsic still projects in-view: the YAML's T_camera_lidar maps a point
        # in the LIDAR frame into the CAMERA OPTICAL frame. A TransformStamped with
        # header.frame_id = P and child_frame_id = C encodes the map C -> P, so the transform whose
        # parent is <body> and whose child is the camera optical frame is the INVERSE of what the
        # file holds -- composed with LiDAR -> body, because <body> is the IMU, not the LiDAR
        # (see the module docstring). Both the inverse and the rotation are checked inside.
        self._lidar_to_body_t = [float(v) for v in self.get_parameter('lidar_to_body_t').value]
        self._lidar_to_body_r = [float(v) for v in self.get_parameter('lidar_to_body_r').value]
        T_body_optical = compose_body_optical(
            T_camera_lidar, self._lidar_to_body_t, self._lidar_to_body_r)
        self._lidar_offset_m = float(np.linalg.norm(self._lidar_to_body_t))

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._static_bcast = StaticTransformBroadcaster(self)

        # Spin BEFORE deciding anything: /tf_static arrives on subscription match and only while
        # spinning. This is also legal only here, outside the executor's own spin -- main() has not
        # handed the node over yet.
        wait_s = float(self.get_parameter('camera_tf_wait_s').value)
        self._spin_for(wait_s, until=lambda: self._resolve_root()[0] != self._camera_frame)

        root, chain = self._resolve_root()
        if root == self._body_frame:
            raise CalibTfError(
                'the camera frame %r already resolves to %r through %s -- something else is '
                'already publishing this bridge. Publishing a second one would give %r two '
                'parents and make every lookup depend on discovery order. Remove the other '
                'publisher, or this node.'
                % (self._camera_frame, self._body_frame, ' -> '.join(chain), root))

        if root == self._camera_frame:
            # Nothing claims the camera frame. Publish the contract's literal form.
            T_body_child = T_body_optical
            note = ('nothing else publishes a parent for %r, so it is used as the child directly'
                    % self._camera_frame)
            self._direct = True
        else:
            # Something (the Orbbec driver) already owns the camera frame. Bridge to the ROOT of
            # its chain and compose, so the tree stays a tree -- see the module docstring.
            T_root_optical = self._lookup_matrix(root, self._camera_frame)
            if T_root_optical is None:
                raise CalibTfError(
                    'the camera frame %r has a parent chain (%s) but %s -> %s could not be looked '
                    'up. Refusing to publish: bridging to the wrong link would give a frame two '
                    'parents.'
                    % (self._camera_frame, ' -> '.join(chain), root, self._camera_frame))
            T_body_child = T_body_optical @ _inverse_rigid(T_root_optical)
            note = ('%r is already published by another authority through %s, so this bridges to '
                    'the ROOT of that chain and composes its transform in. The composite '
                    '%s -> %s is identical; the difference is that no frame gets two parents.'
                    % (self._camera_frame, ' -> '.join(chain), self._body_frame,
                       self._camera_frame))
            self._direct = False

        self._child_frame = root
        self._publish_static(T_body_child, note)

        if bool(self.get_parameter('verify_odom').value):
            # Small topic, and the only runtime evidence of what the body frame is actually called.
            self._odom_checked = False
            self.create_subscription(Odometry, '/lidar_slam/odom', self._on_odom,
                                     qos_profile_sensor_data)

        self._t0 = self._now()
        self._chain_ok = None          # tri-state: None = never reported. Logged on transition.
        self.create_timer(CHAIN_CHECK_PERIOD_S, self._check_chain)

        # See _check_fastlio_extrinsic: the assumption body == LiDAR frame, checked rather than
        # merely documented.
        self._extrinsic_state = 'pending'
        self._extrinsic_client = None
        self._extrinsic_target = None
        self.create_timer(CHAIN_CHECK_PERIOD_S, self._check_fastlio_extrinsic)

    # -- publication -----------------------------------------------------------------------------

    def _publish_static(self, T, note) -> None:
        """Publish the bridge, and say loudly what was bridged to what."""
        q = _R_to_quat(T[:3, :3])
        # NORMALISED before publishing. The rotation block came out of a YAML written to 9
        # decimals, so it is orthonormal only to ~1e-9 and the quaternion it produces has norm
        # 0.999999999 rather than 1 (measured on the shipped calibration). tf2 validates incoming
        # quaternions and DROPS a transform whose norm is off, logging "invalid quaternion" -- and
        # a dropped static transform here means the whole chain silently does not exist, which is
        # the exact failure this node was written to end.
        norm = math.sqrt(sum(v * v for v in q))
        if not (norm > 0.0) or not math.isfinite(norm):
            raise CalibTfError('the rotation produced a degenerate quaternion (norm %r)' % norm)
        q = tuple(v / norm for v in q)

        # Round trip through the package's own quaternion->R, which is the expression every other
        # consumer of a pose in this repo uses. A sign error in one branch of _R_to_quat yields a
        # perfectly valid unit quaternion for a DIFFERENT rotation; nothing downstream could tell.
        back = depth_source.quat_to_R(_Q(*q))
        qerr = float(np.abs(back - T[:3, :3]).max())
        if qerr > ROUNDTRIP_TOL:
            raise CalibTfError(
                'rotation -> quaternion round trip is off by %.3e (limit %.1e). The quaternion '
                'describes a DIFFERENT rotation than the calibration does, and nothing downstream '
                'would notice. Refusing to publish.' % (qerr, ROUNDTRIP_TOL))

        msg = TransformStamped()
        # A static transform's stamp is not used for interpolation -- tf2 answers any query time
        # from the static cache -- so now() is correct here and carries no geometry claim, unlike
        # the cloud stamps elsewhere in this package.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._body_frame
        msg.child_frame_id = self._child_frame
        msg.transform.translation.x = float(T[0, 3])
        msg.transform.translation.y = float(T[1, 3])
        msg.transform.translation.z = float(T[2, 3])
        msg.transform.rotation.x, msg.transform.rotation.y = float(q[0]), float(q[1])
        msg.transform.rotation.z, msg.transform.rotation.w = float(q[2]), float(q[3])
        self._static_bcast.sendTransform(msg)

        self.get_logger().info(
            'PUBLISHED THE MISSING BRIDGE  %s -> %s\n'
            '    translation : [%+.6f, %+.6f, %+.6f] m   (baseline %.3f m)\n'
            '    rotation    : [%+.6f, %+.6f, %+.6f, %+.6f] xyzw\n'
            '    derived from: lidar_to_body @ inverse(T_camera_lidar); the calibration is NOT\n'
            '                  hand-entered, and the LiDAR->body (IMU) offset composed in is\n'
            '                  %s (|T| = %.3f m)\n'
            '    %s\n'
            '    That offset must equal FAST-LIO\'s mapping.extrinsic_T/_R; checked below.'
            % (self._body_frame, self._child_frame,
               T[0, 3], T[1, 3], T[2, 3], float(np.linalg.norm(T[:3, 3])),
               q[0], q[1], q[2], q[3],
               [round(v, 5) for v in self._lidar_to_body_t], self._lidar_offset_m, note))
        if self._direct and self._child_frame == self._camera_frame:
            self.get_logger().warn(
                'no other authority publishes a parent for %r right now. If the camera driver '
                'starts LATER it will publish one too, and that frame will then have TWO PARENTS '
                '-- tf2 keeps whichever transform a given subscriber received last, so the lookup '
                'starts working or failing per process and per run. Start the driver before this '
                'node, or raise camera_tf_wait_s (currently %.1f s).'
                % (self._camera_frame, float(self.get_parameter('camera_tf_wait_s').value)))

    # -- TF graph --------------------------------------------------------------------------------

    def _resolve_root(self):
        """(root frame of whatever chain already contains camera_frame, the chain as a list).

        Returns (camera_frame, [camera_frame]) when nothing claims it. Reads tf2's own view of the
        graph rather than guessing at the driver's frame names, because the Orbbec's optical chain
        (camera_link -> camera_color_frame -> camera_color_optical_frame) is a driver
        implementation detail we must not encode.
        """
        try:
            import yaml
            frames = yaml.safe_load(self._buffer.all_frames_as_yaml()) or {}
        except Exception as exc:
            # Never fatal: an unparsable frame dump means we fall back to the direct form, which is
            # the contract's literal behaviour, and _publish_static warns about the consequence.
            self.get_logger().warn('could not read the tf2 frame graph (%s); assuming nothing '
                                   'else publishes %r' % (exc, self._camera_frame))
            return self._camera_frame, [self._camera_frame]

        chain = [self._camera_frame]
        node = self._camera_frame
        seen = {node}
        for _ in range(MAX_FRAME_DEPTH):
            entry = frames.get(node) or {}
            parent = str(entry.get('parent') or '').strip()
            if not parent or parent in seen:
                break
            seen.add(parent)
            chain.append(parent)
            node = parent
        return node, list(reversed(chain))

    def _lookup_matrix(self, target, source):
        """T_target__source as a 4x4, or None. Zero timeout -- see TF_LOOKUP_TIMEOUT."""
        try:
            tf = self._buffer.lookup_transform(target, source, rclpy.time.Time(),
                                               timeout=TF_LOOKUP_TIMEOUT)
        except Exception as exc:
            self.get_logger().warn('lookup %s -> %s failed: %s' % (target, source, exc))
            return None
        T = np.eye(4)
        T[:3, :3] = depth_source.quat_to_R(tf.transform.rotation)
        T[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y,
                    tf.transform.translation.z]
        return T

    def _check_chain(self) -> None:
        """Does world -> camera actually resolve now? Logged on transition only.

        This is the question dtp_bringup/launch/follower.launch.py leaves open and the one thing
        that decides whether perception_node can work at all. Answering it here turns "perception
        publishes empty arrays and nobody knows why" into one line at bring-up. It also catches
        FAST-LIO dying later, which looks identical downstream.
        """
        try:
            ok = bool(self._buffer.can_transform(self._world_frame, self._camera_frame,
                                                 rclpy.time.Time(), timeout=TF_LOOKUP_TIMEOUT))
        except Exception:
            ok = False
        if ok and ok == self._chain_ok:
            return
        if ok:
            self._chain_ok = True
            self.get_logger().info(
                'TF chain COMPLETE: %s -> %s resolves. perception_node can lift detections into '
                'the world.' % (self._world_frame, self._camera_frame))
            return
        # Not resolving. During the grace period this is start-up ordering, not a fault.
        booting = (self._now() - self._t0) < CHAIN_GRACE_S
        if booting and self._chain_ok is None:
            return
        if self._chain_ok is False:
            # Already reported in full. Repeat a ONE-LINE reminder rather than going silent: unlike
            # every other transition-logged condition in this package, this one is permanent and
            # total -- the full explanation below scrolls away in seconds during bring-up, and the
            # downstream symptom (perception publishing empty arrays) names no cause at all.
            self.get_logger().error(
                'TF chain still BROKEN: %s -> %s does not resolve; no detection can be placed in '
                'the world.' % (self._world_frame, self._camera_frame),
                throttle_duration_sec=60.0)
            return
        self._chain_ok = False
        self.get_logger().error(
            'TF chain BROKEN: %s -> %s does not resolve even though this node published\n'
            '  %s -> %s. The missing link is upstream of us, and there are three candidates:\n'
            '    1. FAST-LIO is not running, or has stopped.\n'
            '    2. FAST-LIO publishes /lidar_slam/odom but NO TF -- whether the ros2 fork\n'
            '       broadcasts %s -> %s is unverified in this repo (fast_lio is external, see\n'
            '       workspace.repos). If so, something must bridge odometry to TF and this\n'
            '       transform alone is necessary but not sufficient.\n'
            '    3. body_frame is set to %r but FAST-LIO names its body frame something else.\n'
            '  Check with: ros2 run tf2_tools view_frames'
            % (self._world_frame, self._camera_frame, self._body_frame, self._child_frame,
               self._world_frame, self._body_frame, self._body_frame))

    # -- runtime verification of the two assumptions ---------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        """Cross-check body_frame and world_frame against the node that defines them.

        Once, on the first message: these are static properties of FAST-LIO's configuration, and a
        check that re-fires at 10 Hz would be muted within a minute.
        """
        if self._odom_checked:
            return
        self._odom_checked = True
        child = (msg.child_frame_id or '').strip()
        parent = (msg.header.frame_id or '').strip()
        if child and child != self._body_frame:
            self.get_logger().error(
                'BODY FRAME MISMATCH -- the bridge is published under the wrong name.\n'
                '  body_frame parameter      : %r\n'
                '  /lidar_slam/odom says     : %r\n'
                '  This node published %s -> %s, so nothing connects %r to the camera and every\n'
                '  world position in the follow stack will fail to resolve. Relaunch with\n'
                '  body_frame:=%s. (A static transform cannot be withdrawn once latched, so this\n'
                '  is a message, not a fix.)'
                % (self._body_frame, child, self._body_frame, self._child_frame, child, child))
        else:
            self.get_logger().info('body frame verified: /lidar_slam/odom child_frame_id is %r'
                                   % child)
        if parent and parent != self._world_frame:
            self.get_logger().warn(
                'world_frame is %r but /lidar_slam/odom is stamped in %r. Nothing this node '
                'publishes depends on it -- only the chain check above does -- but '
                'perception_node lifts into the world frame and must agree with the odometry.'
                % (self._world_frame, parent))

    def _check_fastlio_extrinsic(self) -> None:
        """Does the LiDAR->body extrinsic composed into the published transform match the one
        FAST-LIO is actually running?

        The two are copies of the same YAML numbers (see the module docstring for why a copy is
        unavoidable), and copies drift: someone re-tunes mid360.yaml, or launches with this
        node's identity defaults. Either way every world position shifts by the difference with
        the overlays still looking plausible, so the live values are read back from FAST-LIO's
        parameter service and compared.

        Everything here is non-blocking. A service call from inside a timer callback must not spin
        the node (re-entrant spin is illegal), so this polls service_is_ready() and uses call_async
        with a done-callback; the executor completes it on a later iteration. It is also entirely
        wrapped: an unanswerable parameter query must never be able to take down the node that owns
        the only transform in the graph.
        """
        if self._extrinsic_state != 'pending':
            return
        try:
            from rcl_interfaces.srv import GetParameters
            if self._extrinsic_client is None:
                # Resolve WHICH node to ask from the odometry topic's own publisher endpoint rather
                # than from a hardcoded name -- calib_io.check_lidar_source documents why a
                # hardcoded node name is wrong in both directions (ROS 2 does not enforce unique
                # node names, so the node you query may not be the one that published your data).
                target = None
                for info in self._node_publishers('/lidar_slam/odom'):
                    name = (info.node_name or '').strip()
                    if name and name != '_NODE_NAME_UNKNOWN_':
                        target = '%s/%s' % ((info.node_namespace or '').rstrip('/'), name)
                        break
                if target is None:
                    return          # FAST-LIO not up yet, or DDS has not named it. Try again.
                self._extrinsic_target = target
                self._extrinsic_client = self.create_client(
                    GetParameters, target.rstrip('/') + '/get_parameters')
            if not self._extrinsic_client.service_is_ready():
                return
            req = GetParameters.Request()
            req.names = ['mapping.extrinsic_T', 'mapping.extrinsic_R']
            self._extrinsic_state = 'in_flight'
            self._extrinsic_client.call_async(req).add_done_callback(self._on_extrinsic)
        except Exception as exc:
            self._extrinsic_state = 'unverified'
            self.get_logger().warn(
                'could not check FAST-LIO\'s mapping.extrinsic_T (%s) -- UNVERIFIED that its body '
                'frame is the LiDAR frame. If that extrinsic is ever non-identity, every world '
                'position is offset by it with no error anywhere.' % exc)

    def _on_extrinsic(self, fut) -> None:
        try:
            res = fut.result()
            live_t = list(res.values[0].double_array_value) if res and res.values else []
            live_r = list(res.values[1].double_array_value) if res and len(res.values) > 1 else []
        except Exception:
            live_t, live_r = [], []
        if len(live_t) != 3 or len(live_r) != 9:
            # The node may have answered before its parameters were declared. Retry on the next
            # tick rather than treating a transient as a verdict.
            self._extrinsic_state = 'pending'
            return
        dt = float(np.linalg.norm(np.asarray(live_t) - np.asarray(self._lidar_to_body_t)))
        dr = float(np.abs(np.asarray(live_r) - np.asarray(self._lidar_to_body_r)).max())
        if dt > EXTRINSIC_MATCH_TOL or dr > EXTRINSIC_MATCH_TOL:
            self._extrinsic_state = 'failed'
            self.get_logger().error(
                'THE PUBLISHED TRANSFORM IS WRONG BY %.3f m -- the LiDAR->body extrinsic composed '
                'into %s -> %s is not the one FAST-LIO is running.\n'
                '  composed (lidar_to_body_t / _r) : %s / %s\n'
                '  %s mapping.extrinsic_T / _R     : %s / %s\n'
                '  (translation differs by %.4f m, rotation by %.2e)\n'
                '  Every world position, every height and every proximity veto is offset by the\n'
                '  difference, and nothing else in the stack would notice.\n'
                '  Fix: set static_calib_tf lidar_to_body_t / lidar_to_body_r in dtp_params.yaml\n'
                '  to the mapping.extrinsic_T / _R in the mid360.yaml FAST-LIO runs from, and\n'
                '  relaunch this node.'
                % (dt, self._body_frame, self._child_frame,
                   [round(v, 5) for v in self._lidar_to_body_t],
                   [round(v, 3) for v in self._lidar_to_body_r],
                   self._extrinsic_target,
                   [round(v, 5) for v in live_t], [round(v, 3) for v in live_r],
                   dt, dr))
            return
        self._extrinsic_state = 'ok'
        self.get_logger().info(
            'LiDAR->body extrinsic verified against %s (mapping.extrinsic_T/_R match what was '
            'composed, |T| = %.3f m), so %s -> %s is a valid body -> camera transform'
            % (self._extrinsic_target, self._lidar_offset_m, self._body_frame, self._child_frame))

    # -- small helpers ---------------------------------------------------------------------------

    def _node_publishers(self, topic):
        try:
            return self.get_publishers_info_by_topic(topic)
        except Exception:
            return []

    def _spin_for(self, seconds, until=None) -> None:
        """Spin this node for up to `seconds`, stopping early when `until()` becomes true.

        Legal ONLY before main() hands the node to an executor. See the start-up timing block.
        """
        import time as _time
        deadline = _time.time() + max(0.0, float(seconds))
        while _time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=SPIN_SLICE_S)
            if until is not None and until():
                return

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StaticCalibTf()
    except (CalibTfError, calib_io.CalibError) as exc:
        # A refusal, not a crash. Reported as one line of ERROR with the reason and no traceback,
        # then a non-zero exit so `ros2 launch` and systemd both treat it as the failure it is.
        # There is no degraded mode to fall back to: without this transform the follow stack cannot
        # place a person anywhere, and publishing a guessed one would place them WRONGLY, which is
        # worse than not starting.
        get_logger('static_calib_tf').error(
            'REFUSING TO PUBLISH THE CAMERA EXTRINSIC.\n%s' % exc)
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(2)

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
