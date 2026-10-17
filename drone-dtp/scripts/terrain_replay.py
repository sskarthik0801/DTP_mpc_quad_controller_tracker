#!/usr/bin/env python3
"""terrain_replay.py -- command_mux's terrain-relative height over a recorded flight, open loop.

It feeds the bag's /cloud_registered_body + /lidar_slam/odom through the same TerrainMap the
terrain_estimator_node uses, samples the ground under the aircraft and under the goal the mux
published (/planning/follow_goal), and runs the same TerrainOffset command_mux would have run,
engaged whenever PX4 was armed + Offboard. Read-only; run on the NX or anywhere with the bag.

The check that matters is independent of the LiDAR: PX4's downward rangefinder
(vehicle_local_position.dist_bottom). "AGL if terrain-relative" = the ToF reading + (the goal
height the terrain mode would have commanded - the altitude actually flown). Over flat ground
it equals the follow height (~1.35 m); on the 2026-09-21 18:41 flight's slope the absolute
height grew to 3.4-3.7 m.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/terrain_replay.py bags/stage_b_2026-09-21_183532 [--every 5]
"""
import argparse
import collections
import math
import os
import time

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

from command_mux.terrain import TerrainOffset
from command_mux.terrain_map import TerrainMap

FOLLOW_Z = 1.2      # dtp_params.yaml goal_z_min_m: the flat follow height in lidar_init
REF_Z = -0.15       # the takeoff ground in lidar_init


def quat_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--every', type=float, default=5.0, help='print a row every N s (default 5)')
    ap.add_argument('--stride', type=int, default=2, help='use every N-th scan (node default 2)')
    ap.add_argument('--start', type=float, default=0.0,
                    help='ignore scans before this many seconds into the bag, as if the '
                         'estimator had been started then (default 0: from the beginning)')
    ap.add_argument('--stop', type=float, default=1e9, help='stop this many seconds into the bag')
    ap.add_argument('--probe', default='', help='x,y: print the cells around it at the end')
    args = ap.parse_args()
    bag = args.bag.rstrip('/')
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    r = rosbag2_py.SequentialReader()
    storage = meta.get('storage_identifier', 'sqlite3')
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = ['/cloud_registered_body', '/lidar_slam/odom', '/planning/follow_goal',
              '/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_control_mode',
              '/dtp/targets']
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    cls = {t: get_message(types[t]) for t in topics if t in types}

    tmap = TerrainMap()
    off = TerrainOffset(REF_Z, max_up_m=1.5, max_down_m=1.2, rate_up_mps=0.3, rate_down_mps=0.2,
                        deadband_m=0.05, timeout_s=1.0)
    odoms = collections.deque(maxlen=50)
    goal_xy = None
    people = ((), -1e9)
    tof = (math.nan, False)
    engaged = False
    t0 = None
    n_scan = 0
    cpu = []
    last_print = -1e9
    rows = []
    print('   t     x      lio_z  ToF   g_drone g_goal  offset  goal_z  AGL(abs)  AGL(terrain)  '
          'mode')
    while r.has_next():
        topic, data, t = r.read_next()
        t *= 1e-9
        t0 = t if t0 is None else t0
        if t - t0 > args.stop:
            break
        m = deserialize_message(data, cls[topic])
        if topic == '/lidar_slam/odom':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odoms.append((stamp(m.header), (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)))
        elif topic == '/planning/follow_goal':
            goal_xy = (m.pose.position.x, m.pose.position.y, t)
        elif topic == '/fmu/out/vehicle_local_position_v1':
            tof = (m.dist_bottom, bool(m.dist_bottom_valid))
        elif topic == '/dtp/targets':
            people = (tuple((x.position.x, x.position.y) for x in m.targets), t)
        elif topic == '/fmu/out/vehicle_control_mode':
            engaged = bool(m.flag_armed and m.flag_control_offboard_enabled)
        elif topic == '/cloud_registered_body':
            n_scan += 1
            if n_scan % args.stride or not odoms or t - t0 < args.start:
                continue
            cs = stamp(m.header)
            os_, p, q = min(odoms, key=lambda o: abs(o[0] - cs))
            if abs(os_ - cs) > 0.05:
                continue
            c0 = time.process_time()
            pts = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
            world = pts.astype(np.float64) @ quat_to_R(*q).T + np.array(p)
            tmap.add_scan(world, p, exclude=people[0] if t - people[1] <= 0.5 else ())
            gd, _ = tmap.ground_at(p[0], p[1])
            gg = None
            if goal_xy is not None and t - goal_xy[2] <= 1.0:
                gg, _ = tmap.ground_at(goal_xy[0], goal_xy[1])
            known = [g for g in (gd, gg) if g is not None]
            ground = max(known) if known else math.nan
            offset = off.update(t, ground, t, engaged)
            cpu.append(time.process_time() - c0)
            rel = t - t0
            goal_z = FOLLOW_Z + offset
            agl_abs = tof[0] if tof[1] else math.nan
            agl_ter = agl_abs + (goal_z - p[2]) if tof[1] else math.nan
            rows.append((rel, p[0], p[2], agl_abs, gd, gg, offset, goal_z, agl_ter, engaged))
            if rel - last_print >= args.every:
                last_print = rel

                def f(v):
                    return ' --- ' if v is None or not math.isfinite(v) else '%5.2f' % v
                print('%6.1f %6.2f %6.2f %s  %s  %s  %6.2f  %6.2f   %s       %s       %s'
                      % (rel, p[0], p[2], f(agl_abs), f(gd), f(gg), offset, goal_z, f(agl_abs),
                         f(agl_ter), 'OFFBOARD' if engaged else '-'))
    if args.probe:
        px, py = (float(v) for v in args.probe.split(','))
        print('\ncells around (%.1f, %.1f): ground z (points)' % (px, py))
        span = (1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5)
        for dy in span:
            line = []
            for dx in span[::-1]:
                hist = tmap._cells.get((tmap._index(px + dx) + (1 << 20)) * (1 << 21)
                                       + tmap._index(py + dy) + (1 << 20))
                g = tmap._ground(hist)
                line.append('%6s(%2d)' % ('--' if g is None else '%.2f' % g, len(hist or ())))
            print('  y%+.1f  %s' % (py + dy, ' '.join(line)))
    a = np.array([(x[3], x[8]) for x in rows if x[9] and math.isfinite(x[3])])
    print('\n%d scans used, map %d cells, %.1f ms CPU per scan (p50), %.1f ms p95'
          % (len(rows), len(tmap), 1e3 * np.median(cpu), 1e3 * np.percentile(cpu, 95)))
    if len(a):
        print('In Offboard, height above ground from the ToF: flown (absolute) p50 %.2f max %.2f '
              '| terrain-relative p50 %.2f, range %.2f..%.2f'
              % (np.median(a[:, 0]), a[:, 0].max(), np.median(a[:, 1]), a[:, 1].min(),
                 a[:, 1].max()))


if __name__ == '__main__':
    main()
