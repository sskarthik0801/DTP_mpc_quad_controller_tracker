#!/usr/bin/env python3
"""Step 1 pre-build measurement (5): CPU cost, in pure numpy on this Jetson, of the LiDAR part of
the hybrid range source for one colour frame.

    python3 cpu_cost.py [bag_name] [t_bag]      # default 09-29 bag, t 476.1 (in flight)

Input: the real /cloud_registered_body scans of a 0.5 s window (5 scans), in the body frame, with
their odom poses. Timed (median of 200 runs, single thread, float32 math as depth_source uses):
  A  per-scan ingest: one scan body -> lidar_init (done once per scan arrival, 10 Hz)
  B  per-frame: the accumulated lidar_init points -> body at the frame stamp -> colour camera
     (composed T_camera_body) -> pinhole projection -> points inside one person box (+ range)
  C  per-frame, worst case: B plus re-transforming all 5 scans from body each frame
  D  for scale: depth_source.range_image's full z-buffered 1280x720 raster of the same points
Reported as ms per call and as % of one core at 15 Hz (the plan's rate; the node runs 8 Hz).
"""
import os
import sys
import time

import numpy as np
import yaml

from _step1_lib import Bag

CALIB = '/home/ideas-ad/SUPER-Human-DTP/drone-dtp/ros2_ws/src/dtp_perception/calib/drone_cam_livox.yaml'
T_LI = np.array([-0.011, -0.02329, 0.04412])


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def timeit(f, n=200):
    f()
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        f()
        ts.append(time.perf_counter() - t0)
    return 1e3 * np.median(ts), 1e3 * np.percentile(ts, 95)


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'stage_b_2026-09-29_122511'
    tq = float(sys.argv[2]) if len(sys.argv) > 2 else 476.1
    b = Bag(name)
    t = b.start + tq
    idx = b.scans_between(t - 0.5, t)
    od = b.c['od']
    scans, poses = [], []
    for i in idx:
        j = int(np.argmin(np.abs(b.od_t - b.sc_t[i])))
        R, p = quat_R(*od[j, 4:8]), od[j, 1:4]
        w = b.pts[b.sc_off[i]:b.sc_off[i + 1]].astype(np.float64)
        scans.append(((w - p) @ R).astype(np.float32))          # back to the body frame
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, p
        poses.append(T)
    npts = sum(len(s) for s in scans)
    Tcl = np.array(yaml.safe_load(open(CALIB))['T_camera_lidar'], float)
    Tcb = np.eye(4)
    Tcb[:3, :3] = Tcl[:3, :3]
    Tcb[:3, 3] = Tcl[:3, 3] - Tcl[:3, :3] @ T_LI
    fx, cx, fy, cy = b.K[2], b.K[4], b.K[6], b.K[7]
    W, H = 1280, 720
    box = (600, 180, 660, 360)
    Twb_now = poses[-1]
    world = [s @ P[:3, :3].T.astype(np.float32) + P[:3, 3].astype(np.float32) for s, P in zip(scans, poses)]
    acc = np.vstack(world)

    def ingest():
        s, P = scans[-1], poses[-1]
        return s @ P[:3, :3].T.astype(np.float32) + P[:3, 3].astype(np.float32)

    def frame(pts=None):
        pts = acc if pts is None else pts
        Tcw = Tcb @ np.linalg.inv(Twb_now)          # camera <- world at the frame stamp
        pc = pts @ Tcw[:3, :3].T.astype(np.float32) + Tcw[:3, 3].astype(np.float32)
        z = pc[:, 2]
        ok = z > 0.3
        pc = pc[ok]
        u = fx * pc[:, 0] / pc[:, 2] + cx
        v = fy * pc[:, 1] / pc[:, 2] + cy
        m = (u >= box[0]) & (u < box[2]) & (v >= box[1]) & (v < box[3])
        r = np.linalg.norm(pc[m], axis=1)
        return r

    def frame_worst():
        pts = np.vstack([s @ P[:3, :3].T.astype(np.float32) + P[:3, 3].astype(np.float32)
                         for s, P in zip(scans, poses)])
        return frame(pts)

    buf = np.empty((H, W), np.float32)

    def raster():
        Tcw = Tcb @ np.linalg.inv(Twb_now)
        pc = acc @ Tcw[:3, :3].T.astype(np.float32) + Tcw[:3, 3].astype(np.float32)
        buf.fill(np.nan)
        z = pc[:, 2]
        rng = np.linalg.norm(pc, axis=1)
        keep = (z > 0.0) & (rng >= 0.3) & (rng <= 15.0)
        p, rng = pc[keep], rng[keep]
        u = fx * p[:, 0] / p[:, 2] + cx
        v = fy * p[:, 1] / p[:, 2] + cy
        inb = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        u, v, rng = u[inb].astype(np.intp), v[inb].astype(np.intp), rng[inb]
        o = np.argsort(-rng)
        buf[v[o], u[o]] = rng[o]

    print('bag %s t %.1f: %d scans in 0.5 s, %d points (%.0f per scan); load avg %s' % (
        name, tq, len(scans), npts, npts / len(scans), os.getloadavg()))
    for lab, f, hz in (('A per-scan ingest body->lidar_init (10 Hz)', ingest, 10),
                       ('B per-frame world->camera, project, box mask', frame, 15),
                       ('C per-frame worst case (re-transform 5 scans + B)', frame_worst, 15),
                       ('D full z-buffered 1280x720 raster (range_image)', raster, 15)):
        med, p95 = timeit(f)
        print('  %-52s %6.2f ms (p95 %6.2f)  -> %4.1f %% of a core at %d Hz' % (lab, med, p95, med * hz / 10, hz))
    print('  in-box points for this frame: %d' % len(frame()))


if __name__ == '__main__':
    main()
