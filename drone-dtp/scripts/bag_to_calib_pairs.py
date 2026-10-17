#!/usr/bin/env python3
"""M5.4b: turn a recorded bag -- a Stage A FLIGHT bag above all -- into score_alignment pairs.

WHY THIS EXISTS
---------------
Every camera<->LiDAR solve on this aircraft is MOTORS OFF, and the plan's M5.4 exit is a motor-on
`score_alignment` within +/-2 px. The obvious way to get motor-on data, a props-on ground run
through `recalib.sh capture`, does not fit this aircraft:
  * PX4 disarms ~10 s after arming when no takeoff is detected (COM_DISARM_PRFLT = 10), and
    capture_calib_data dwells 15 s per viewpoint;
  * capture needs the LiDAR on PointCloud2 (xfer_format 0), which starves FAST-LIO -- the exact
    configuration that sent it 3.74 km off on 2026-09-12;
  * idle-throttle vibration on the ground is not hover vibration anyway.
A Stage A flight already records what score_alignment needs, WITH the motors at hover and the
estimator healthy: colour frames, /camera/color/camera_info, /cloud_registered_body and
/lidar_slam/odom (scripts/stage_a_record.sh). This writes those into the capture format --
pair_NNN.npz (image, cloud in the LiDAR frame), camera_info.yaml, T_camera_lidar.txt -- so the
unchanged scorer can run on flight data:

    python3 drone-dtp/scripts/bag_to_calib_pairs.py ~/dtp_datasets/<date>_stageA --out ~/calib_runs/<date>_flight
    ros2 run dtp_perception score_alignment --run_dir ~/calib_runs/<date>_flight

HOW ONE PAIR IS BUILT
---------------------
For a colour frame at t_img, every /cloud_registered_body scan within +/- --window s is moved into
the body frame AT t_img through FAST-LIO's own poses (scan and odometry share stamps exactly), then
into the MID360 frame with the inverse of FAST-LIO's extrinsic (lidar_to_body in dtp_params.yaml,
which static_calib_tf verifies against the running FAST-LIO). On a moving aircraft this removes the
ego-motion between scan and frame, so what remains is the camera<->LiDAR extrinsic -- the quantity
under test -- plus residual timing. Frames taken while the aircraft yaws faster than --max-yaw-rate
are skipped: at 0.5 rad/s one colour frame period smears ~10 px at the image edge, and a smear is
not an extrinsic.

VALIDATE IT BEFORE TRUSTING A FLIGHT NUMBER: run it on a bench bag and compare with a real capture
of the same mount -- the converter must reproduce the motors-off result, or a flight "shift" is the
converter's.

--raw-lidar (STATIONARY RIGS ONLY): take /livox/lidar CustomMsg points instead, which are already in
the MID360 frame, so neither FAST-LIO's pose nor its LiDAR->IMU extrinsic enters the pair. That is
the control for the one confound the default path cannot remove: the deployed FAST-LIO runs
extrinsic_est_en true, so /cloud_registered_body is registered with the ONLINE ESTIMATE of that
extrinsic while this script inverts the CONFIG value; any drift in the estimate reads as a pixel
shift. Raw points are not deskewed or motion-compensated, hence stationary only (the script refuses
a frame where the rig moves or yaws). Needs ~/ws_livox sourced for livox_ros_driver2 messages.
"""
import argparse
import bisect
import glob
import math
import os
import sqlite3
import sys

import numpy as np

DEFAULT_CALIB = os.path.expanduser(
    '~/super_ws/src/drone-dtp/ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml')
DEFAULT_PARAMS = os.path.expanduser(
    '~/super_ws/src/drone-dtp/ros2_ws/src/dtp_bringup/config/dtp_params.yaml')


def _rows(con, tid, topic):
    if topic not in tid:
        return []
    return con.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid[topic],)).fetchall()


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _quat_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _xyz(cloud):
    n = cloud.width * cloud.height
    off = {f.name: f.offset for f in cloud.fields}
    buf = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(n, cloud.point_step)
    out = np.empty((n, 3), dtype=np.float64)
    for i, k in enumerate('xyz'):
        out[:, i] = buf[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0]
    return out[np.isfinite(out).all(axis=1)]


class Poses:
    """FAST-LIO odometry, interpolated (position linear, rotation normalised quaternion lerp)."""

    def __init__(self, samples):
        samples.sort(key=lambda s: s[0])
        self.t = [s[0] for s in samples]
        self.p = [s[1] for s in samples]
        self.q = [s[2] for s in samples]

    def at(self, t, max_gap=0.15):
        """(position, R_world_body) at t, or None outside the odometry or across a gap."""
        i = bisect.bisect_left(self.t, t)
        if i < len(self.t) and abs(self.t[i] - t) < 1e-4:      # scans share odom's stamps exactly
            return self.p[i], _quat_to_R(*self.q[i])
        if i <= 0 or i >= len(self.t):
            return None
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > max_gap:
            return None
        f = (t - t0) / (t1 - t0)
        q0, q1 = np.array(self.q[i - 1]), np.array(self.q[i])
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = (1 - f) * q0 + f * q1
        q /= np.linalg.norm(q)
        return (1 - f) * self.p[i - 1] + f * self.p[i], _quat_to_R(*q)

    def yaw_rate(self, t, h=0.1):
        a, b = self.at(t - h), self.at(t + h)
        if a is None or b is None:
            return float('nan')
        dR = a[1].T @ b[1]
        return abs(math.atan2(dR[1, 0], dR[0, 0])) / (2 * h)

    def speed(self, t, h=0.1):
        a, b = self.at(t - h), self.at(t + h)
        return float('nan') if a is None or b is None else float(np.linalg.norm(b[0] - a[0]) / (2 * h))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--out', required=True, help='run_dir to create (score_alignment --run_dir)')
    ap.add_argument('--calib', default=DEFAULT_CALIB, help='the drone_cam_livox.yaml under test')
    ap.add_argument('--params', default=DEFAULT_PARAMS, help='dtp_params.yaml (lidar_to_body_t/_r)')
    ap.add_argument('--every', type=float, default=2.0, help='seconds between pairs')
    ap.add_argument('--window', type=float, default=None,
                    help='+/- seconds of scans accumulated per pair. Default 1.75 s for the FAST-LIO path, '
                         '0.5 s for --raw-lidar: score_alignment needs ~200k points per pair, and '
                         '/cloud_registered_body is thinned by point_filter_num 3 (~5.7k per scan) while a raw '
                         'MID360 scan is ~17k. At 0.25 s (28k points) run5 scored INCONCLUSIVE on both axes.')
    ap.add_argument('--max-yaw-rate', type=float, default=0.5, help='skip frames yawing faster, rad/s')
    ap.add_argument('--max-pairs', type=int, default=40)
    ap.add_argument('--t-start', type=float, default=0.0, help='seconds into the bag to begin')
    ap.add_argument('--t-end', type=float, default=1e9, help='seconds into the bag to end')
    ap.add_argument('--raw-lidar', action='store_true',
                    help='use /livox/lidar (MID360 frame, no FAST-LIO extrinsic); stationary rigs only')
    args = ap.parse_args(argv)
    if args.window is None:
        args.window = 0.5 if args.raw_lidar else 1.75

    import yaml
    import cv2
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, CompressedImage, CameraInfo

    files = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % args.bag)
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}
    need = ('/camera/color/image_raw/compressed', '/camera/color/camera_info',
            '/livox/lidar' if args.raw_lidar else '/cloud_registered_body', '/lidar_slam/odom')
    missing = [t for t in need if t not in tid]
    if missing:
        raise SystemExit('bag lacks %s' % ', '.join(missing))

    calib = yaml.safe_load(open(args.calib))
    T_cam_lidar = np.array(calib['T_camera_lidar'], dtype=float)
    params = yaml.safe_load(open(args.params))
    stc = None
    for node_cfg in params.values():
        rp = (node_cfg or {}).get('ros__parameters') or {}
        if 'lidar_to_body_t' in rp:
            stc = rp
            break
    if stc is None:
        raise SystemExit('no lidar_to_body_t in %s' % args.params)
    t_lb = np.array(stc['lidar_to_body_t'], dtype=float)
    R_lb = np.array(stc['lidar_to_body_r'], dtype=float).reshape(3, 3)

    poses = []
    for _ts, d in _rows(con, tid, '/lidar_slam/odom'):
        m = deserialize_message(bytes(d), Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        poses.append((_stamp(m.header), np.array([p.x, p.y, p.z]), (q.w, q.x, q.y, q.z)))
    P = Poses(poses)
    scans = []
    if args.raw_lidar:
        from livox_ros_driver2.msg import CustomMsg
        for _ts, d in _rows(con, tid, '/livox/lidar'):
            m = deserialize_message(bytes(d), CustomMsg)
            pts = np.array([(q.x, q.y, q.z) for q in m.points], dtype=np.float64)
            pts = pts[np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 0.1)]
            scans.append((_stamp(m.header), pts))
    else:
        for _ts, d in _rows(con, tid, '/cloud_registered_body'):
            m = deserialize_message(bytes(d), PointCloud2)
            scans.append((_stamp(m.header), d))
    scan_t = [s[0] for s in scans]
    info = deserialize_message(bytes(_rows(con, tid, '/camera/color/camera_info')[0][1]), CameraInfo)
    t0_bag = min(poses[0][0], scans[0][0])

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'camera_info.yaml'), 'w') as f:
        f.write('# Intrinsics from /camera/color/camera_info in %s\n' % os.path.basename(args.bag.rstrip('/')))
        f.write('resolution: [%d, %d]\n' % (info.width, info.height))
        f.write('fx: %.6f\nfy: %.6f\ncx: %.6f\ncy: %.6f\n' % (info.k[0], info.k[4], info.k[2], info.k[5]))
        f.write('distortion_model: %s\n' % info.distortion_model)
        f.write('d: [%s]\n' % ', '.join('%.8f' % v for v in info.d))
    np.savetxt(os.path.join(args.out, 'T_camera_lidar.txt'), T_cam_lidar)
    with open(os.path.join(args.out, 'SOURCE.txt'), 'w') as f:
        f.write('bag: %s\ncalib: %s\nparams: %s\nwindow: %.2f s  every: %.2f s  max_yaw_rate: %.2f rad/s\n'
                % (os.path.abspath(args.bag), args.calib, args.params, args.window, args.every, args.max_yaw_rate))

    n_pairs = skipped_yaw = skipped_pose = 0
    next_t = None
    n_frames = con.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?',
                           (tid['/camera/color/image_raw/compressed'],)).fetchone()[0]
    print('bag %s: %d colour frames, %d scans, %d poses'
          % (os.path.basename(args.bag.rstrip('/')), n_frames, len(scans), len(poses)))
    print('pair   t(s)   scans  points   yaw_rate  speed')
    for _ts, d in _rows(con, tid, '/camera/color/image_raw/compressed'):
        img_msg = deserialize_message(bytes(d), CompressedImage)
        t_img = _stamp(img_msg.header)
        rel = t_img - t0_bag
        if rel < args.t_start or rel > args.t_end:
            continue
        if next_t is not None and t_img < next_t:
            continue
        yr = P.yaw_rate(t_img)
        if args.raw_lidar and math.isfinite(yr) and (yr > 0.02 or P.speed(t_img) > 0.05):
            skipped_yaw += 1       # raw points are not motion-compensated: stationary only
            next_t = t_img + 0.2
            continue
        if not math.isfinite(yr) or yr > args.max_yaw_rate:
            skipped_yaw += 1
            next_t = t_img + 0.2
            continue
        pose_img = P.at(t_img)
        if pose_img is None:
            skipped_pose += 1
            continue
        p_i, R_i = pose_img
        lo = bisect.bisect_left(scan_t, t_img - args.window)
        hi = bisect.bisect_right(scan_t, t_img + args.window)
        chunks = []
        for k in range(lo, hi):
            ts, blob = scans[k]
            if args.raw_lidar:
                chunks.append(blob)                     # already the MID360 frame
                continue
            ps = P.at(ts)
            if ps is None:
                continue
            p_s, R_s = ps
            body = _xyz(deserialize_message(bytes(blob), PointCloud2))
            world = body @ R_s.T + p_s
            body_at_img = (world - p_i) @ R_i           # R_i^T (world - p_i), row-vector form
            chunks.append((body_at_img - t_lb) @ R_lb)  # R_lb^T (body - t_lb): MID360 frame
        if not chunks:
            skipped_pose += 1
            continue
        cloud = np.vstack(chunks).astype(np.float32)
        img = cv2.imdecode(np.frombuffer(bytes(img_msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        np.savez_compressed(os.path.join(args.out, 'pair_%03d.npz' % n_pairs), image=img[:, :, ::-1],
                            encoding='rgb8', cloud=cloud, sweeps=len(chunks), stamp=t_img)
        print('%4d %7.1f  %5d  %6d   %6.3f  %5.2f' % (n_pairs, rel, len(chunks), len(cloud), yr, P.speed(t_img)))
        n_pairs += 1
        next_t = t_img + args.every
        if n_pairs >= args.max_pairs:
            break
    print('wrote %d pairs to %s (skipped: %d moving/yawing or without a yaw-rate estimate, %d without poses)'
          % (n_pairs, args.out, skipped_yaw, skipped_pose))
    print('next: ros2 run dtp_perception score_alignment --run_dir %s' % args.out)
    return 0 if n_pairs else 1


if __name__ == '__main__':
    sys.exit(main())
