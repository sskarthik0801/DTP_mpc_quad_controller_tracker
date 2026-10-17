#!/usr/bin/env python3
"""Is the deployed camera<->LiDAR extrinsic right in PITCH (dv)? A wide score_alignment sweep on
flight-bag frames.

    python3 dv_sweep.py <bag_name> <t_bag> [<t_bag> ...]

For each bag time: the colour frame nearest it, and 1.0 s of /cloud_registered_body registered
into the body frame at the frame stamp (odom), converted to the MID360 frame (p_lidar = p_body -
extrinsic_T), then dtp_perception.calibration.score_alignment's own per-pair sweep with the
DEPLOYED T_camera_lidar, widened from +-6 px to +-60 px (the stock sweep stops at +-6 and the
09-13 du check needed a wider one to find its peak). Pick frames with the aircraft still (on the
ground or hovering) so 1 s of scans does not smear.
"""
import sys

import cv2
import numpy as np
import rosbag2_py
import scipy.ndimage as ndi
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from _step1_lib import BAG_DIR, Bag
from overlay_check import CALIB, T_LI, body_pose

sys.path.insert(0, '/home/ideas-ad/SUPER-Human-DTP/drone-dtp/ros2_ws/src/dtp_perception')
from dtp_perception.calibration import score_alignment as sa  # noqa: E402

sa.SHIFTS = list(range(-60, 61, 2))
OUTNPZ = ('/tmp/claude-1000/-home-ideas-ad-SUPER-Human-DTP/d04c92ae-44af-44ea-8c75-b858015fae68/'
          'scratchpad/step1_measure/overlay/_pair.npz')


def main():
    name, times = sys.argv[1], [float(x) for x in sys.argv[2:]]
    b = Bag(name)
    T = np.array(yaml.safe_load(open(CALIB))['T_camera_lidar'], float)
    fx, cx, fy, cy = b.K[2], b.K[4], b.K[6], b.K[7]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG_DIR + name, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw/compressed']))
    for tq in times:
        r.seek(int((b.start + tq) * 1e9))
        _, d, _ = r.read_next()
        m = deserialize_message(d, get_message(types['/camera/color/image_raw/compressed']))
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)[:, :, ::-1]
        R, p = body_pose(b, st)
        P, _ = b.window_points(st - 1.0, st)
        pl = (P - p) @ R - T_LI
        a0, a1 = b.pos(st - 1.0), b.pos(st)
        np.savez(OUTNPZ, image=img, cloud=pl)
        res = sa._sweep_one_pair(OUTNPZ, T, (fx, fy, cx, cy), ndi)
        if res is None:
            print('t %.1f: too few points' % tq)
            continue
        du, dv, nv, nh = res
        line = 't %6.1f  moved %.2f m in the 1 s window  AGL %.2f  pts %d  ' % (
            tq, np.linalg.norm(a1 - a0), a1[2] - b.ground(a1[0], a1[1]), len(pl))
        for lab, sc, n in (('du', du, nv), ('dv', dv, nh)):
            if sc is None:
                line += '%s: too few edges   ' % lab
                continue
            s = np.array(sorted(sc)); v = np.array([sc[k] for k in s])
            line += '%s peak %+3d px (contrast %.3f, %d edges)   ' % (lab, s[v.argmax()], (v.max() - v.min()) / (v.max() + 1e-9), n)
        print(line)


if __name__ == '__main__':
    main()
