"""cuda_cloud against the REAL numpy path it replaces -- not against a copy of it.

The reference in every test is depth_source's own `_points_in_colour_frame` + `range_image` and
organized_cloud's own `build`, driven through a DepthSource built without a ROS node (the two
methods only read attributes). A copied reference would pass while the real one drifted.

What is asserted, and why it is two different things (cuda_cloud's docstring has the long form):
  * the POINTS path (LiDAR source) is bit-identical -- np.array_equal, NaN positions included;
  * the CAMERA path is within cuda_cloud's tolerance, because its depth->colour matmul is not
    GUARANTEED to round identically in numpy and on the GPU across library versions. (Measured
    2026-09-13 it was exact on every scene here and on 200 real frames.)
The GPU tests skip on a machine without CUDA; the two that need no GPU always run.
"""
import math

import numpy as np
import pytest
from sensor_msgs.msg import Image

from dtp_perception import cuda_cloud, organized_cloud
from dtp_perception.depth_source import CameraDepthSource, DepthSource, DepthSourceError

COLOUR_SIZE = (1280, 720)
DEPTH_SIZE = (424, 240)
K_COLOUR = np.array([[640.5, 0.0, 641.2], [0.0, 640.1, 362.7], [0.0, 0.0, 1.0]])
K_DEPTH = np.array([[212.3, 0.0, 211.6], [0.0, 212.0, 119.4], [0.0, 0.0, 1.0]])


def _rot(yaw_deg, pitch_deg):
    a, b = math.radians(yaw_deg), math.radians(pitch_deg)
    Ry = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                   [-math.sin(a), 0.0, math.cos(a)]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(b), -math.sin(b)],
                   [0.0, math.sin(b), math.cos(b)]])
    return Rx @ Ry


# A factory-like depth->colour extrinsic: a small rotation and a ~15 mm baseline.
R_CD = _rot(0.31, -0.12)
T_CD = np.array([-0.0149, 0.00021, 0.00034])


def _scene_mm(seed=0):
    """A 424x240 depth scene with the features that stress a z-buffer: a far wall, a floor ramp,
    a near 'person' box, speckle noise and invalid (0) pixels."""
    rng = np.random.default_rng(seed)
    w, h = DEPTH_SIZE
    depth = np.full((h, w), 6000.0)
    depth[h // 2:, :] = np.linspace(1500.0, 5000.0, h - h // 2)[::-1, None]
    depth[60:200, 170:250] = 2600.0 + 40.0 * rng.standard_normal((140, 80))
    depth += 8.0 * rng.standard_normal((h, w))
    depth[rng.random((h, w)) < 0.07] = 0.0
    return np.clip(depth, 0.0, 65535.0).astype(np.uint16)


def _image_msg(depth, encoding='16UC1', stamp=(100, 5)):
    msg = Image()
    msg.height, msg.width = depth.shape
    msg.encoding = encoding
    msg.step = depth.dtype.itemsize * depth.shape[1]
    msg.data = depth.tobytes()
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
    return msg


def _camera_source(msg, stride=1, min_r=0.3, max_r=15.0):
    """A CameraDepthSource with its inputs already 'received' and no ROS node behind it."""
    src = object.__new__(CameraDepthSource)
    src._K, src._size = K_COLOUR.copy(), COLOUR_SIZE
    src._min_range, src._max_range = float(min_r), float(max_r)
    src._buf, src._buf_key, src._buf_stamp = None, None, float('nan')
    src._warned = set()
    src._stride = int(stride)
    src._depth_msg, src._depth_seq = msg, 1
    src._Kd, src._depth_size = K_DEPTH.copy(), DEPTH_SIZE
    src._R_cd, src._t_cd = R_CD.copy(), T_CD.copy()
    return src


class _PointsSource(DepthSource):
    """The base class with a fixed point set -- the LiDAR source's shape, minus the LiDAR."""

    name = 'points'

    def __init__(self, pts):
        self._K, self._size = K_COLOUR.copy(), COLOUR_SIZE
        self._min_range, self._max_range = 0.3, 15.0
        self._buf, self._buf_key, self._buf_stamp = None, None, float('nan')
        self._warned = set()
        self._pts = pts

    def _points_in_colour_frame(self):
        return self._pts, 42.0, ('points', 1)


def _numpy_cloud(src):
    return organized_cloud.build(src.range_image(), K_COLOUR)


@pytest.fixture(scope='module')
def cuda():
    try:
        return cuda_cloud.CudaCloud()
    except cuda_cloud.CudaCloudError as exc:
        pytest.skip('no usable CUDA here: %s' % exc)


# -- need a GPU --------------------------------------------------------------------------------

@pytest.mark.parametrize('stride', [1, 2])
@pytest.mark.parametrize('seed', [0, 1, 2])
def test_camera_path_matches_numpy_16uc1(cuda, stride, seed):
    msg = _image_msg(_scene_mm(seed))
    ref = _numpy_cloud(_camera_source(msg, stride))
    got = _camera_source(msg, stride).organized_cloud_cuda(cuda)
    mask_frac, max_diff = cuda_cloud.compare(ref, got)
    assert got.shape == (COLOUR_SIZE[1], COLOUR_SIZE[0], 3) and got.dtype == np.float32
    # ~92k pixels at stride 1 and ~23k at stride 2 (a quarter of the samples).
    assert np.isfinite(ref[:, :, 2]).sum() > 80000 // (stride * stride), 'scene too sparse'
    assert cuda_cloud.within_camera_tolerance(mask_frac, max_diff), (
        'mask mismatch %.2e (tol %.0e), max |diff| %.2e m (tol %.0e)'
        % (mask_frac, cuda_cloud.CAMERA_MASK_TOL_FRAC, max_diff, cuda_cloud.CAMERA_VALUE_TOL_M))


def test_camera_path_matches_numpy_32fc1_with_nan(cuda):
    depth = _scene_mm(3).astype(np.float32) * 0.001
    depth[::17, ::13] = np.nan
    msg = _image_msg(depth, encoding='32FC1')
    ref = _numpy_cloud(_camera_source(msg))
    got = _camera_source(msg).organized_cloud_cuda(cuda)
    assert cuda_cloud.within_camera_tolerance(*cuda_cloud.compare(ref, got))


def test_points_path_is_bit_identical(cuda):
    rng = np.random.default_rng(7)
    n = 60000
    # Includes points behind the camera, closer than min range, beyond max range, off-image, and
    # many deliberate collisions (repeated rays at different ranges) for the z-buffer.
    dirs = rng.uniform([-1.2, -0.8, -0.2], [1.2, 0.8, 1.0], size=(n, 3))
    ranges = rng.uniform(0.1, 20.0, size=(n, 1))
    pts = (dirs / np.linalg.norm(dirs, axis=1, keepdims=True) * ranges).astype(np.float32)
    pts = np.vstack([pts, pts[:5000] * np.float32(1.37)])
    ref = _numpy_cloud(_PointsSource(pts))
    got = _PointsSource(pts).organized_cloud_cuda(cuda)
    assert np.array_equal(ref, got, equal_nan=True)


def test_every_call_returns_a_fresh_array(cuda):
    msg = _image_msg(_scene_mm(0))
    src = _camera_source(msg)
    a = src.organized_cloud_cuda(cuda)
    b = src.organized_cloud_cuda(cuda)
    assert not np.shares_memory(a, b)
    a[:] = 0.0
    assert np.isfinite(b[:, :, 2]).any()


def test_empty_depth_is_all_nan(cuda):
    msg = _image_msg(np.zeros((DEPTH_SIZE[1], DEPTH_SIZE[0]), np.uint16))
    got = _camera_source(msg).organized_cloud_cuda(cuda)
    assert np.isnan(got).all()
    assert np.isnan(_PointsSource(np.zeros((0, 3), np.float32)).organized_cloud_cuda(cuda)).all()


def test_stamp_advances_and_numpy_cache_is_invalidated(cuda):
    src = _camera_source(_image_msg(_scene_mm(0), stamp=(100, 0)))
    src.range_image()                                   # numpy caches frame A under its key
    assert src.last_stamp_s == pytest.approx(100.0)
    msg_b = _image_msg(_scene_mm(1), stamp=(101, 500000000))
    src._depth_msg, src._depth_seq = msg_b, 2
    src.organized_cloud_cuda(cuda)
    assert src.last_stamp_s == pytest.approx(101.5)    # the skew guard sees frame B
    assert src._buf_key is None                          # no stale frame-A buffer can be served
    fresh = _camera_source(msg_b).range_image()
    assert np.array_equal(src.range_image(), fresh, equal_nan=True)


def test_bad_intrinsics_raise_value_error(cuda):
    src = _camera_source(_image_msg(_scene_mm(0)))
    src._K = np.zeros((3, 3))
    with pytest.raises(ValueError):
        src.organized_cloud_cuda(cuda)


# -- need no GPU ---------------------------------------------------------------------------------

class _MustNotBeCalled:
    def __getattr__(self, name):
        raise AssertionError('the GPU was touched before the encoding was checked')


def test_bad_encoding_raises_before_the_gpu_is_touched():
    msg = _image_msg(_scene_mm(0).astype(np.uint8), encoding='8UC1')
    with pytest.raises(DepthSourceError):
        _camera_source(msg).organized_cloud_cuda(_MustNotBeCalled())


def test_compare_counts_mask_and_value_disagreement():
    a = np.zeros((2, 5, 3), np.float32)
    b = a.copy()
    b[0, 0, :] = np.nan
    b[1, 1, 0] = 3e-6
    mask_frac, max_diff = cuda_cloud.compare(a, b)
    assert mask_frac == pytest.approx(0.1)
    assert max_diff == pytest.approx(3e-6)
