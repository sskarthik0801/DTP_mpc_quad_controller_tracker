#!/usr/bin/env python3
"""Visual check of the composed camera<-body transform and of the LiDAR walker truth.

    python3 overlay_check.py <bag_name> <t_bag> [<t_bag> ...]

Decodes the colour frame nearest each bag time, projects 0.3 s of /cloud_registered_body (the
scans up to the frame stamp, odom-registered into lidar_init, then into the body at the frame
stamp) through T_camera_body = T_camera_lidar * T_lidar_body (the composed transform, with
FAST-LIO's LiDAR->IMU extrinsic), and draws: every point coloured by range (near red -> far blue),
DYNAMIC points in the body band as white rings, the camera track position (lidar_init) as a
green cross at its z and at z+-0.8. Writes PNGs to <scratch>/step1_measure/overlay/.
"""
import os
import sys

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from _step1_lib import BAG_DIR, BODY_HI, BODY_LO, Bag, build_static, is_static

OUT = ('/tmp/claude-1000/-home-ideas-ad-SUPER-Human-DTP/d04c92ae-44af-44ea-8c75-b858015fae68/'
       'scratchpad/step1_measure/overlay')
CALIB = '/home/ideas-ad/SUPER-Human-DTP/drone-dtp/ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml'
T_LI = np.array([-0.011, -0.02329, 0.04412])     # mid360.yaml extrinsic_T (p_body = p_lidar + T)


def T_camera_body():
    T = np.array(yaml.safe_load(open(CALIB))['T_camera_lidar'], float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3]
    out[:3, 3] = T[:3, 3] - T[:3, :3] @ T_LI
    return out


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def body_pose(b, t):
    od = b.c['od']
    i = int(np.clip(np.searchsorted(b.od_t, t), 1, len(b.od_t) - 1))
    j = i if abs(b.od_t[i] - t) < abs(b.od_t[i - 1] - t) else i - 1
    # position interpolated, rotation nearest (10 Hz; fine for a check)
    return quat_R(*od[j, 4:8]), b.pos(t)


def main():
    name, times = sys.argv[1], [float(x) for x in sys.argv[2:]]
    b = Bag(name)
    build_static(b)
    Tcb = T_camera_body()
    K = b.K
    fx, cx, fy, cy = K[2], K[4], K[6], K[7]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG_DIR + name, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw/compressed']))
    cf = b.camera_frames()
    os.makedirs(OUT, exist_ok=True)
    for tq in times:
        r.seek(int((b.start + tq - 0.2) * 1e9))
        best = None
        while r.has_next():
            topic, d, ts = r.read_next()
            m = deserialize_message(d, get_message(types[topic]))
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if best is None or abs(st - b.start - tq) < abs(best[0] - b.start - tq):
                best = (st, m)
            if st - b.start > tq + 0.2:
                break
        st, m = best
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        R, p = body_pose(b, st)
        P, _ = b.window_points(st - 0.30, st)
        pb = (P - p) @ R                                   # world -> body at the frame stamp
        pc = pb @ Tcb[:3, :3].T + Tcb[:3, 3]
        ok = pc[:, 2] > 0.5
        u = fx * pc[:, 0] / pc[:, 2] + cx
        v = fy * pc[:, 1] / pc[:, 2] + cy
        rng = np.linalg.norm(pc, axis=1)
        h = P[:, 2] - b.ground(P[:, 0], P[:, 1])
        body = ok & (h > BODY_LO) & (h < BODY_HI)
        dyn = np.zeros(len(P), bool)
        dyn[body] = ~is_static(b, P[body], st)
        for i in np.nonzero(ok & (u >= 0) & (u < img.shape[1]) & (v >= 0) & (v < img.shape[0]))[0]:
            c = np.clip((rng[i] - 2) / 16, 0, 1)
            cv2.circle(img, (int(u[i]), int(v[i])), 2, (int(255 * c), 60, int(255 * (1 - c))), -1)
            if dyn[i]:
                cv2.circle(img, (int(u[i]), int(v[i])), 6, (255, 255, 255), 1)
        k = np.argmin(np.abs(cf['t'] - st))
        if abs(cf['t'][k] - st) < 0.15:
            for dz in (-0.8, 0.0, 0.8):
                w = np.array([cf['x'][k], cf['y'][k], cf['z'][k] + dz])
                q = ((w - p) @ R) @ Tcb[:3, :3].T + Tcb[:3, 3]
                if q[2] > 0.5:
                    uu, vv = int(fx * q[0] / q[2] + cx), int(fy * q[1] / q[2] + cy)
                    cv2.drawMarker(img, (uu, vv), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
            label = 'cam track %d r %.2f m' % (cf['tid'][k], cf['r'][k])
        else:
            label = 'no camera target'
        cv2.putText(img, '%s t %.2f  %s  (white rings = dynamic LiDAR pts)' % (name[8:], st - b.start, label),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        f = os.path.join(OUT, '%s_%.2f.jpg' % (name[8:], st - b.start))
        cv2.imwrite(f, img)
        print(f)


if __name__ == '__main__':
    main()
