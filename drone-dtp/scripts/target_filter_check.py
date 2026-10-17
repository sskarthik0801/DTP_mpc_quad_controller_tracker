#!/usr/bin/env python3
"""M5.3a bench gate for the target-point filter, against the REAL fsm_node. No hardware.

WHAT IT PROVES
--------------
fsm_node's ROG-Map is fed a synthetic LiDAR scene (a person, a tube, a wall) and synthetic
/dtp/follow_target messages, on a private localhost-only DDS domain, once with
rog_map/target_exclusion/enable false and once true. The map is then read back off /rog_map/occ.
The scan pattern is random like the MID360's non-repetitive one but seeded per scan, and each scan
is published only after the previous one has been inserted. Cell COUNTS still move a little between
runs (on two full runs F1's OFF trail read 195 and 189, F5's 195 and 247) because /rog_map/occ is sampled at 2 Hz on
its own timer; every threshold below has margin over that, and the ON-arm zeros never moved.

  F1  TRAIL      a person walks 4 m across the sensor's view. OFF: they leave occupied cells behind
                 them. ON: none.
  F2  NO EROSION a person stands 0.3 m (surface to surface) beside a mapped tube for 10 s. ON: every
                 tube cell whose centre is inside the cylinder is unchanged -- no hits, no misses.
  F3  LINE OF    a phantom obstacle is mapped between sensor and person, then removed; the person
      SIGHT      stands behind where it was. ON clears it EXACTLY as much as OFF (same rays, and the
                 phantom is outside the cylinder), and completely inside the band the rays to the
                 person cross. Cells above the wall's top edge or below the MID360's -7 deg floor
                 have no ray through them in either arm and stay -- that is ROG-Map, not the filter.
  F4  COASTED    the same walk as F1 but every target is a COAST (missed_frames > 0): never filtered,
                 so the trail is back. A coasted position is where an occluder is.
  F5  HELD       the same walk with one-point forecasts (perception's HELD frames): never filtered.
  F6  STALE      targets stop mid-walk: filtering stops within max_age and the marker is deleted.

The scan density is the MID360's at point_filter_num 3 (~0.3 returns per square degree, ~1 ray
per 0.1 m cell at 3 m). With a dense synthetic scan every stale cell is re-cleared within one frame
and F1's OFF arm could not show a trail at all -- which is exactly the trap a denser simulator
would hide.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/target_filter_check.py          # ~4 min, exits non-zero on any FAIL
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

DOMAIN = '88'
SUPER_CFG_DIR = os.path.expanduser('~/super_ws/src/SUPER/super_planner/config')
BASE_CFG = os.path.join(SUPER_CFG_DIR, 'follow_test_ros2.yaml')

SCAN_DT = 0.1            # stamp time between scans (the MID360's 10 Hz)
PACE_S = 0.35            # wall time between publishes: every scan is inserted before the next
TARGET_LAG_S = 0.29      # target stamp behind scan stamp, p50 measured on run5
RAYS_PER_DEG2 = 0.315    # ~200k pts/s / point_filter_num 3 / 10 Hz over 360 x 59 deg
AZ = (-60.0, 60.0)
EL = (-7.0, 52.0)        # the MID360's vertical field of view
RES = 0.1

# Scene (lidar_init == world, sensor at the origin). The effective virtual ground in the follow
# config is -0.5 m, so everything stands on -0.45.
FLOOR = -0.45
PERSON_R, PERSON_TOP = 0.20, 1.20
PERSON_Z = PERSON_TOP - 0.65 * (PERSON_TOP - FLOOR)   # perception's waist sample
# The wall must sit inside follow_test_ros2.yaml's visualization range (15 x 15 m around the
# sensor), or /rog_map/occ never contains it.
WALL_X, WALL_Y, WALL_TOP = 7.0, 5.0, 2.5
TUBE_R, TUBE_TOP = 0.15, 1.50


def cylinder_t(d, cx, cy, r, zb, zt):
    a = d[:, 0] ** 2 + d[:, 1] ** 2
    b = -2.0 * (d[:, 0] * cx + d[:, 1] * cy)
    c = cx * cx + cy * cy - r * r
    disc = b * b - 4 * a * c
    t = np.full(len(d), np.inf)
    ok = (disc >= 0) & (a > 1e-12)
    t_hit = (-b[ok] - np.sqrt(disc[ok])) / (2 * a[ok])
    z = t_hit * d[ok, 2]
    good = (t_hit > 0) & (z >= zb) & (z <= zt)
    idx = np.nonzero(ok)[0][good]
    t[idx] = t_hit[good]
    return t


def scan(k, objects):
    """One MID360-like scan: seeded random directions, nearest hit per ray."""
    rng = np.random.default_rng(1000 + k)
    n = int(RAYS_PER_DEG2 * (AZ[1] - AZ[0]) * (EL[1] - EL[0]))
    az = np.radians(rng.uniform(*AZ, n))
    # uniform on the sphere band, so density per square degree is flat in elevation
    s = rng.uniform(math.sin(math.radians(EL[0])), math.sin(math.radians(EL[1])), n)
    el = np.arcsin(s)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
    t = np.full(n, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        tw = np.where(d[:, 0] > 1e-6, WALL_X / d[:, 0], np.inf)
    yw, zw = tw * d[:, 1], tw * d[:, 2]
    t = np.where((np.abs(yw) <= WALL_Y) & (zw >= FLOOR) & (zw <= WALL_TOP), tw, t)
    for (cx, cy, r, zb, zt) in objects:
        t = np.minimum(t, cylinder_t(d, cx, cy, r, zb, zt))
    hit = np.isfinite(t)
    return (d[hit] * t[hit, None]).astype(np.float32)


class Harness:
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        from visualization_msgs.msg import Marker
        from dtp_msgs.msg import Target
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('target_filter_check')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                         depth=1, durability=DurabilityPolicy.VOLATILE)
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/cloud_registered', qos)
        self.odom_pub = self.node.create_publisher(Odometry, '/lidar_slam/odom', qos)
        self.target_pub = self.node.create_publisher(Target, '/dtp/follow_target', qos)
        self.occ = None
        self.occ_t = 0.0
        self.markers = []
        self.node.create_subscription(PointCloud2, '/rog_map/occ', self._on_occ, qos)
        self.node.create_subscription(Marker, '/rog_map/target_exclusion', self._on_marker, qos)
        self.Target, self.PointCloud2, self.Odometry = Target, PointCloud2, Odometry
        self.next_odom = 0.0

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()

    def _on_occ(self, msg):
        from sensor_msgs_py import point_cloud2
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        self.occ = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        self.occ_t = time.time()

    def _on_marker(self, msg):
        self.markers.append((time.time(), msg.action, msg.pose.position.x, msg.pose.position.y))

    def spin(self, secs):
        end = time.time() + secs
        while True:
            now = time.time()
            if now >= self.next_odom:
                m = self.Odometry()
                m.header.stamp = self.node.get_clock().now().to_msg()
                m.header.frame_id = 'lidar_init'
                m.child_frame_id = 'lidar_imu'
                m.pose.pose.orientation.w = 1.0
                self.odom_pub.publish(m)
                self.next_odom = now + 0.05
            left = end - time.time()
            if left <= 0:
                return
            self.rclpy.spin_once(self.node, timeout_sec=min(0.02, left))

    def stamp(self, t):
        from builtin_interfaces.msg import Time
        return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    def publish_scan(self, pts, t):
        from sensor_msgs_py import point_cloud2
        from sensor_msgs.msg import PointField
        from std_msgs.msg import Header
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        data = np.hstack([pts, np.full((len(pts), 1), 100.0, dtype=np.float32)])
        msg = point_cloud2.create_cloud(Header(stamp=self.stamp(t), frame_id='world'), fields, data)
        self.cloud_pub.publish(msg)

    def publish_target(self, p, v, t, missed=0, forecast_pts=10):
        from geometry_msgs.msg import Point
        m = self.Target()
        m.header.stamp = self.stamp(t)
        m.header.frame_id = 'lidar_init'
        m.track_id = 7
        m.position.x, m.position.y, m.position.z = p
        m.velocity.x, m.velocity.y, m.velocity.z = v[0], v[1], 0.0
        m.predicted_path = [Point(x=p[0] + v[0] * (0.2 + 0.075 * i), y=p[1] + v[1] * (0.2 + 0.075 * i), z=p[2])
                            for i in range(forecast_pts)]
        m.predicted_path_dt = [0.2 + 0.075 * i for i in range(forecast_pts)]
        m.track_confirmed = True
        m.missed_frames = missed
        m.confidence = 0.9
        m.is_locked_target = True
        self.target_pub.publish(m)

    def snapshot(self, after):
        """The first /rog_map/occ published after `after` (wall time)."""
        end = time.time() + 5.0
        while time.time() < end:
            self.spin(0.1)
            if self.occ is not None and self.occ_t > after + 0.05:
                return self.occ.copy()
        raise RuntimeError('no /rog_map/occ from fsm_node')


def start_fsm(cfg_path, log_path):
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')
    rel = os.path.relpath(cfg_path, SUPER_CFG_DIR)   # config_name is joined onto ROOT_DIR/config/
    log = open(log_path, 'w')
    proc = subprocess.Popen(['ros2', 'run', 'super_planner', 'fsm_node', '--ros-args',
                             '-p', 'config_name:=%s' % rel],
                            env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True)
    return proc, log


def stop_fsm(proc, log):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=15)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
    log.close()


def in_box(pts, x, y, z):
    return pts[(pts[:, 0] >= x[0]) & (pts[:, 0] <= x[1]) & (pts[:, 1] >= y[0]) & (pts[:, 1] <= y[1])
               & (pts[:, 2] >= z[0]) & (pts[:, 2] <= z[1])]


def cells(pts):
    return {tuple(np.round(p / RES).astype(int)) for p in pts}


def run_scenario(h, name, enable, body, tmpdir, cfg_on, cfg_off):
    cfg = cfg_on if enable else cfg_off
    log_path = os.path.join(tmpdir, '%s_%s.log' % (name, 'on' if enable else 'off'))
    proc, log = start_fsm(cfg, log_path)
    try:
        h.occ, h.occ_t, h.markers = None, 0.0, []
        # fsm_node preallocates ~2 GB for A*; wait until its map is publishing
        t0 = time.time()
        k = 0
        while time.time() - t0 < 40.0:
            h.publish_scan(scan(k, []), time.time())
            k += 1
            h.spin(PACE_S)
            if h.occ is not None:
                break
        if h.occ is None:
            raise RuntimeError('fsm_node never published /rog_map/occ (log: %s)' % log_path)
        result = body(h, k)
        with open(log_path) as f:
            result['log'] = f.read()
        return result
    finally:
        stop_fsm(proc, log)


def walk(h, k0, mode):
    """Person walks y -2 -> +2 at x 3 (1 m/s), then stands 1 s. mode: seen | coast | held | stop."""
    t0 = time.time()
    for i in range(20):                        # map the wall first
        h.publish_scan(scan(k0 + i, []), t0 + (i) * SCAN_DT)
        h.spin(PACE_S)
    k0 += 20
    t_walk = t0 + 20 * SCAN_DT
    n = 40
    for i in range(n + 10):
        ts = t_walk + i * SCAN_DT
        y = -2.0 + min(i, n) * 0.1
        vy = 1.0 if i < n else 0.0
        tt = ts - TARGET_LAG_S
        y_obs = -2.0 + max(0.0, min(i * SCAN_DT - TARGET_LAG_S, n * SCAN_DT)) * 1.0
        if not (mode == 'stop' and i >= 20):
            if mode == 'seen' or mode == 'stop':
                h.publish_target((3.0, y_obs, PERSON_Z), (0.0, vy), tt)
            elif mode == 'coast':
                h.publish_target((3.0, y_obs, PERSON_Z), (0.0, vy), tt, missed=3)
            elif mode == 'held':
                h.publish_target((3.0, y_obs, PERSON_Z), (0.0, vy), tt, forecast_pts=1)
        h.spin(0.02)
        h.publish_scan(scan(k0 + i, [(3.0, y, PERSON_R, FLOOR, PERSON_TOP)]), ts)
        h.spin(PACE_S)
    occ = h.snapshot(time.time())
    trail = in_box(occ, (2.5, 3.5), (-2.3, 1.0), (FLOOR, PERSON_TOP + 0.1))
    wall = in_box(occ, (WALL_X - 0.3, WALL_X + 0.3), (-5, 5), (FLOOR, WALL_TOP))
    return {'trail': len(trail), 'wall': len(wall), 'markers': list(h.markers)}


def tube(h, k0):
    tx, ty = 4.0, 0.40
    px, py = 4.0, ty - (TUBE_R + 0.30 + PERSON_R)
    t0 = time.time()
    objs_tube = [(tx, ty, TUBE_R, FLOOR, TUBE_TOP)]
    for i in range(40):
        h.publish_scan(scan(k0 + i, objs_tube), t0 + i * SCAN_DT)
        h.spin(PACE_S)
    before = h.snapshot(time.time())
    k0 += 40
    t1 = t0 + 40 * SCAN_DT
    for i in range(100):
        ts = t1 + i * SCAN_DT
        h.publish_target((px, py, PERSON_Z), (0.0, 0.0), ts - TARGET_LAG_S)
        h.spin(0.02)
        h.publish_scan(scan(k0 + i, objs_tube + [(px, py, PERSON_R, FLOOR, PERSON_TOP)]), ts)
        h.spin(PACE_S)
    after = h.snapshot(time.time())
    box = ((tx - 0.35, tx + 0.35), (ty - 0.35, ty + 0.35), (FLOOR, TUBE_TOP + 0.1))
    tb, ta = cells(in_box(before, *box)), cells(in_box(after, *box))

    def in_cyl(c):
        x, y, z = (np.array(c) * RES)
        return math.hypot(x - px, y - py) <= 0.6 and PERSON_Z - 1.0 <= z <= PERSON_Z + 1.3
    inside_before = {c for c in tb if in_cyl(c)}
    return {'tube_before': len(tb), 'tube_after': len(ta), 'inside_before': len(inside_before),
            'inside_lost': len(inside_before - ta), 'lost_total': len(tb - ta)}


def los(h, k0):
    gx, gy = 2.0, 0.0
    px, py = 3.5, 0.0
    t0 = time.time()
    for i in range(15):
        h.publish_scan(scan(k0 + i, [(gx, gy, 0.15, FLOOR, 1.0)]), t0 + i * SCAN_DT)
        h.spin(PACE_S)
    ghost_before = len(in_box(h.snapshot(time.time()), (gx - 0.3, gx + 0.3), (-0.3, 0.3), (FLOOR, 1.1)))
    k0 += 15
    t1 = t0 + 15 * SCAN_DT
    for i in range(80):
        ts = t1 + i * SCAN_DT
        h.publish_target((px, py, PERSON_Z), (0.0, 0.0), ts - TARGET_LAG_S)
        h.spin(0.02)
        h.publish_scan(scan(k0 + i, [(px, py, PERSON_R, FLOOR, PERSON_TOP)]), ts)
        h.spin(PACE_S)
    occ = h.snapshot(time.time())
    # The band every ray ending on the person crosses at the phantom's range: from the -7 deg floor
    # of the field of view to the person's top, scaled back to x = gx, and within the person's width.
    z_lo = -gx * math.tan(math.radians(7.0)) + RES
    z_hi = PERSON_TOP * gx / px - RES
    ghost = in_box(occ, (gx - 0.3, gx + 0.3), (-0.3, 0.3), (FLOOR, 1.1))
    return {'ghost_before': ghost_before, 'ghost_after': len(ghost), 'ghost_cells': cells(ghost),
            'ghost_in_los': len(in_box(ghost, (gx - 0.3, gx + 0.3), (-PERSON_R * gx / px, PERSON_R * gx / px),
                                       (z_lo, z_hi))),
            'person_cells': len(in_box(occ, (px - 0.4, px + 0.4), (-0.4, 0.4), (FLOOR, PERSON_TOP + 0.1)))}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--logs', default=None, help='keep fsm_node logs here (default: a temp dir)')
    ap.add_argument('--only', default=None, help='comma list of F1..F6')
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    tmpdir = args.logs or tempfile.mkdtemp(prefix='target_filter_check_')
    os.makedirs(tmpdir, exist_ok=True)
    base = open(BASE_CFG).read()
    if not re.search(r'^  target_exclusion:\n    enable: (true|false)', base, re.M):
        raise SystemExit('%s has no rog_map/target_exclusion block -- is the M5.3a patch applied?' % BASE_CFG)
    cfg_off = os.path.join(tmpdir, 'filter_off.yaml')
    cfg_on = os.path.join(tmpdir, 'filter_on.yaml')
    open(cfg_off, 'w').write(re.sub(r'^(  target_exclusion:\n    enable:) (true|false)', r'\1 false', base, flags=re.M))
    open(cfg_on, 'w').write(re.sub(r'^(  target_exclusion:\n    enable:) (true|false)', r'\1 true', base, flags=re.M))

    only = set(args.only.split(',')) if args.only else None
    results = []

    def check(name, ok, detail):
        results.append((name, ok))
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))

    h = Harness()
    try:
        print('\ntarget_filter_check -- real fsm_node on ROS_DOMAIN_ID %s (localhost only); logs in %s\n'
              % (DOMAIN, tmpdir))
        if only is None or 'F1' in only:
            off = run_scenario(h, 'F1', False, lambda hh, k: walk(hh, k, 'seen'), tmpdir, cfg_on, cfg_off)
            on = run_scenario(h, 'F1', True, lambda hh, k: walk(hh, k, 'seen'), tmpdir, cfg_on, cfg_off)
            print('F1  trail: occupied cells in the walked strip, OFF %d / ON %d (wall %d / %d)'
                  % (off['trail'], on['trail'], off['wall'], on['wall']))
            check('F1 OFF arm shows a trail (the scenario can fail)', off['trail'] >= 10, 'cells=%d' % off['trail'])
            check('F1 ON leaves no trail', on['trail'] == 0, 'cells=%d' % on['trail'])
            check('F1 ON still maps the wall (within 10 % of OFF)',
                  off['wall'] > 0 and abs(on['wall'] - off['wall']) <= 0.10 * off['wall'],
                  '%d vs %d' % (on['wall'], off['wall']))
            filt = re.findall(r'target exclusion: filtered (\d+) scans \((\d+) hits dropped, (\d+) misses masked', on['log'])
            check('F1 ON logs dropped hits and masked misses',
                  any(int(a) > 0 and int(b) > 0 and int(c) > 0 for a, b, c in filt), str(filt[:2]))
            check('F1 OFF never filters', 'TARGET EXCLUSION' not in off['log'] and not off['markers'],
                  'markers=%d' % len(off['markers']))
            adds = [m for m in on['markers'] if m[1] == 0]
            check('F1 ON publishes the cylinder marker per filtered scan', len(adds) >= 30, 'ADD=%d' % len(adds))
        if only is None or 'F2' in only:
            on = run_scenario(h, 'F2', True, tube, tmpdir, cfg_on, cfg_off)
            print('F2  tube cells %d before, %d after; %d inside the cylinder, %d of those lost; %d lost in total'
                  % (on['tube_before'], on['tube_after'], on['inside_before'], on['inside_lost'], on['lost_total']))
            check('F2 setup: part of the tube lies inside the cylinder', on['inside_before'] >= 5,
                  'cells=%d' % on['inside_before'])
            check('F2 ON: no tube cell inside the cylinder is eroded', on['inside_lost'] == 0,
                  'lost=%d' % on['inside_lost'])
        if only is None or 'F3' in only:
            off = run_scenario(h, 'F3', False, los, tmpdir, cfg_on, cfg_off)
            on = run_scenario(h, 'F3', True, los, tmpdir, cfg_on, cfg_off)
            print('F3  phantom cells OFF %d -> %d, ON %d -> %d; inside the line of sight ON %d; person cells OFF %d / ON %d'
                  % (off['ghost_before'], off['ghost_after'], on['ghost_before'], on['ghost_after'],
                     on['ghost_in_los'], off['person_cells'], on['person_cells']))
            check('F3 setup: the phantom was mapped', on['ghost_before'] >= 5, 'cells=%d' % on['ghost_before'])
            check('F3 ON clears the phantom exactly as OFF does', on['ghost_cells'] == off['ghost_cells'],
                  'ON %d vs OFF %d cells' % (on['ghost_after'], off['ghost_after']))
            check('F3 ON: nothing left where the rays to the person pass', on['ghost_in_los'] == 0,
                  'cells=%d' % on['ghost_in_los'])
            check('F3 ON: the standing person is not in the map (OFF: they are)',
                  on['person_cells'] == 0 and off['person_cells'] > 0,
                  'ON %d / OFF %d' % (on['person_cells'], off['person_cells']))
        for name, mode, label in (('F4', 'coast', 'COASTED'), ('F5', 'held', 'HELD')):
            if only is None or name in only:
                on = run_scenario(h, name, True, lambda hh, k, m=mode: walk(hh, k, m), tmpdir, cfg_on, cfg_off)
                print('%s  %s targets: trail cells %d, marker ADDs %d'
                      % (name, label, on['trail'], sum(1 for m in on['markers'] if m[1] == 0)))
                check('%s ON: %s targets are never filtered' % (name, label),
                      on['trail'] >= 10 and not any(m[1] == 0 for m in on['markers']),
                      'trail=%d markers=%d' % (on['trail'], len(on['markers'])))
        if only is None or 'F6' in only:
            on = run_scenario(h, 'F6', True, lambda hh, k: walk(hh, k, 'stop'), tmpdir, cfg_on, cfg_off)
            adds = [m for m in on['markers'] if m[1] == 0]
            dels = [m for m in on['markers'] if m[1] == 2]
            print('F6  targets stop after 20 scans: marker ADDs %d, DELETEs %d, trail cells %d'
                  % (len(adds), len(dels), on['trail']))
            # 20 targets were sent; 0.5 s max_age at 10 Hz stamps covers <= 5 + the lag's worth more
            check('F6 ON: filtering stops within max_age of the last target', 15 <= len(adds) <= 28,
                  'ADD=%d' % len(adds))
            check('F6 ON: the marker is deleted once filtering stops', len(dels) >= 1 and dels[-1][0] > adds[-1][0],
                  'DELETE=%d' % len(dels))
            check('F6 ON: the unfiltered half leaves a trail again', on['trail'] >= 5, 'cells=%d' % on['trail'])
    finally:
        h.close()

    bad = [n for n, ok in results if not ok]
    print('\n%d/%d checks passed%s' % (len(results) - len(bad), len(results),
                                      '' if not bad else ' -- FAILED: ' + ', '.join(bad)))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
