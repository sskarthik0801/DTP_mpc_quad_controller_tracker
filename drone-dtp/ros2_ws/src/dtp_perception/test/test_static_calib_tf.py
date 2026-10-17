"""compose_body_optical: the one multiply every world position in the stack goes through.

Direction is the whole test. Two conventions meet here -- the calibration file's T_camera_lidar
(LiDAR points INTO the camera optical frame) and FAST-LIO's mapping.extrinsic_T/_R (LiDAR points
INTO the IMU/body frame) -- and a TransformStamped(parent=body, child=optical) must map optical
points INTO body. Getting any one of the three backwards still produces a rigid transform of the
right magnitude, so the check is done on a POINT, by hand, not on matrix identities.
"""
import math

import numpy as np
import pytest

from dtp_perception.static_calib_tf import CalibTfError, compose_body_optical, rigid_from_rt

# The Mid-360 datasheet LiDAR->IMU offset as shipped in FAST-LIO's mid360.yaml and as deployed on
# the aircraft (verified live 2026-09-11 via /laser_mapping mapping.extrinsic_T).
MID360_T = [-0.011, -0.02329, 0.04412]
IDENTITY_R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def _optical_from_lidar_example():
    """A T_camera_lidar of the shape the real calibration has: a signed axis permutation
    (optical z = LiDAR x forward, optical x = -LiDAR y, optical y = -LiDAR z) plus a small
    mount offset. Not the real numbers -- the test must not depend on a data file."""
    R = np.array([[0.0, -1.0, 0.0],
                  [0.0, 0.0, -1.0],
                  [1.0, 0.0, 0.0]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.02, -0.05, -0.10]
    return T


def _apply(T, p):
    return (T @ np.array([p[0], p[1], p[2], 1.0]))[:3]


def test_identity_extrinsic_is_the_plain_inverse():
    T_cl = _optical_from_lidar_example()
    T_bo = compose_body_optical(T_cl, [0.0, 0.0, 0.0], IDENTITY_R)
    assert np.allclose(T_bo @ T_cl, np.eye(4), atol=1e-12)


def test_point_round_trip_through_lidar_frame_by_hand():
    """A point expressed in the optical frame, carried into the body frame two ways."""
    T_cl = _optical_from_lidar_example()
    T_bo = compose_body_optical(T_cl, MID360_T, IDENTITY_R)

    p_opt = np.array([0.3, -0.2, 2.5])                       # something 2.5 m ahead of the camera
    # By hand: optical -> lidar is inverse(T_cl); lidar -> body is R p + T.
    p_lidar = _apply(np.linalg.inv(T_cl), p_opt)
    p_body_hand = np.asarray(IDENTITY_R).reshape(3, 3) @ p_lidar + np.asarray(MID360_T)
    p_body = _apply(T_bo, p_opt)
    assert np.allclose(p_body, p_body_hand, atol=1e-12)
    # And the composed translation differs from the plain inverse by exactly the extrinsic,
    # because R is identity: nothing else may have moved.
    T_bo_id = compose_body_optical(T_cl, [0.0, 0.0, 0.0], IDENTITY_R)
    assert np.allclose(T_bo[:3, 3] - T_bo_id[:3, 3], MID360_T, atol=1e-12)
    assert math.isclose(float(np.linalg.norm(MID360_T)), 0.0511, abs_tol=5e-4)


def test_nonidentity_rotation_is_applied_on_the_lidar_side():
    """With a LiDAR->body yaw of 90 deg, a LiDAR-frame +x point must land on body +y before the
    offset is added -- i.e. the rotation acts on LiDAR-frame coordinates, not optical ones."""
    T_cl = _optical_from_lidar_example()
    c, s = math.cos(math.pi / 2), math.sin(math.pi / 2)
    R_yaw90 = [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
    T_bo = compose_body_optical(T_cl, MID360_T, R_yaw90)
    # Build an optical point that IS LiDAR (1, 0, 0): push (1,0,0) through T_cl.
    p_opt = _apply(T_cl, [1.0, 0.0, 0.0])
    p_body = _apply(T_bo, p_opt)
    expected = np.array([0.0, 1.0, 0.0]) + np.asarray(MID360_T)
    assert np.allclose(p_body, expected, atol=1e-12)


def test_output_is_rigid():
    T_bo = compose_body_optical(_optical_from_lidar_example(), MID360_T, IDENTITY_R)
    R = T_bo[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-12)
    assert np.allclose(T_bo[3], [0.0, 0.0, 0.0, 1.0])


def test_refuses_a_rotation_block_that_is_not_a_rotation():
    with pytest.raises(CalibTfError):
        rigid_from_rt(MID360_T, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0])   # scaled z
    with pytest.raises(CalibTfError):
        rigid_from_rt(MID360_T, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0])  # reflection
    with pytest.raises(CalibTfError):
        rigid_from_rt([0.0, 0.0], IDENTITY_R)                                     # wrong arity
