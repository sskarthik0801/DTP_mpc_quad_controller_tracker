"""Coordinate-frame geometry for the Spot port.

The detection pipeline assumes a camera convention -- channel 0 = X (right/lateral),
channel 2 = Z (forward), Y ignored -- and a body that never pitches or rolls. Spot
breaks both: its frames are X-forward / Y-left / Z-up, and the body bobs every gait
cycle (and the front cameras are mounted rolled ~90 deg).

cloud_to_pipeline_frame() fixes all of it at once:
  1. transforms the organized depth cloud from the camera SENSOR frame into Spot's
     gravity-aligned body frame ('flat_body') -- this cancels gait pitch/roll AND
     absorbs the sensor's mount rotation, because the sensor frame already encodes it;
  2. remaps the axes (X_fwd,Y_left,Z_up) -> (X_right,Y_up,Z_fwd) so channel 0 is true
     lateral and channel 2 is true forward, exactly what process_frame expects.

Frame bridge: the sensor frame lives in the IMAGE snapshot, 'flat_body' lives in the
robot-STATE snapshot. Both contain 'vision', so we compose the transform through it.
"""
import numpy as np
from bosdyn.client.frame_helpers import (get_a_tform_b, GRAV_ALIGNED_BODY_FRAME_NAME,
                                         VISION_FRAME_NAME)


def flatbody_tform_sensor(image_snapshot, sensor_frame, state_snapshot):
    vision_T_sensor = get_a_tform_b(image_snapshot, VISION_FRAME_NAME, sensor_frame)
    vision_T_flatbody = get_a_tform_b(state_snapshot, VISION_FRAME_NAME,
                                      GRAV_ALIGNED_BODY_FRAME_NAME)
    if vision_T_sensor is None or vision_T_flatbody is None:
        return None
    return vision_T_flatbody.inverse() * vision_T_sensor


def _spot_to_pipeline_axes(pts):
    x_fwd, y_left, z_up = pts[..., 0], pts[..., 1], pts[..., 2]
    return np.stack((-y_left, z_up, x_fwd), axis=-1)


_SPOT_TO_PIPE = np.array([[0.0, -1.0, 0.0, 0.0],
                          [0.0,  0.0, 1.0, 0.0],
                          [1.0,  0.0, 0.0, 0.0],
                          [0.0,  0.0, 0.0, 1.0]])


def flatbody_pose_in_pipeline(state_snapshot):
    vision_T_flatbody = get_a_tform_b(state_snapshot, VISION_FRAME_NAME,
                                      GRAV_ALIGNED_BODY_FRAME_NAME)
    if vision_T_flatbody is None:
        return None
    return _SPOT_TO_PIPE @ vision_T_flatbody.to_matrix() @ _SPOT_TO_PIPE.T


def cloud_to_pipeline_frame(cloud_sensor, image_snapshot, sensor_frame, state_snapshot):
    T = flatbody_tform_sensor(image_snapshot, sensor_frame, state_snapshot)
    if T is None:
        return cloud_sensor
    M = T.to_matrix()
    R = M[:3, :3].astype(np.float32)
    t = M[:3, 3].astype(np.float32)
    flat = cloud_sensor @ R.T + t
    return _spot_to_pipeline_axes(flat).astype(np.float32)
