#!/usr/bin/env python3
"""M6.2 replay harness, stage 1: a recorded bag -> the REAL fsm_node's loss oracle -> LossEvents.

Plays a bag's FAST-LIO output and the follower's /dtp/follow_target into a private, localhost-only
fsm_node (SUPER + ROG-Map + the M6.1 loss oracle) at bag timing, and records every
/planning/loss_event it produces together with the bag-time context of the target it answered.

WHAT IS REPLAYED, AND HOW
  /lidar_slam/odom          as recorded.
  /cloud_registered_body    transformed into lidar_init with the odom of the SAME stamp (FAST-LIO
                            stamps both from lidar_end_time) and published as /cloud_registered,
                            which is what fsm_node's ROG-Map ingests on the aircraft. So the map
                            the oracle reads is rebuilt from the bag exactly as it was built live.
  /dtp/follow_target        as recorded (target_manager's LOCKED pass-throughs and COASTING
                            forecasts; the oracle answers the coasting ones).
  /dtp/lock_state           NOT replayed -- read for the per-episode summary, so each coast onset
                            is paired with what the recorded target_manager did next.

OUTPUT (--out DIR)
  events.jsonl     one line per LossEvent, every field, plus bag_t (the bag time of the
                   follow_target answered) and episode (a running index per coast onset).
  episodes.csv     one row per coast onset: first / majority class, blocked and unknown
                   fractions, the belief at onset (side, dt, sigma), whether a viewpoint was
                   offered, timing, and the recorded outcome of that coast (re-acquired after X s
                   or LOST, with the reason).
  fsm_node.log     the planner's stdout, including the oracle's periodic timing line.

DETERMINISM. The oracle seeds its rollouts per event, so two replays of the same bag give the
same belief for the same event -- but the MAP is rebuilt from scans whose arrival order and
timing this script controls, and ROG-Map's raycasting is order-sensitive at cell level. Expect
identical classes and near-identical fractions between runs, not bit-identical maps. The
perception replay (replay_perception.sh) is the non-deterministic stage; this one is not.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/loss_replay.py ~/dtp_datasets/20260909_closedloop_run5 \\
        --out ~/dtp_datasets/20260909_closedloop_run5/loss_replay
"""
import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np

DOMAIN = '87'
SUPER_CFG_DIR = os.path.expanduser('~/super_ws/src/SUPER/super_planner/config')
BASE_CFG = os.path.join(SUPER_CFG_DIR, 'follow_test_ros2.yaml')
CLS = ['OCCLUDED', 'UNMAPPED', 'LOST', 'OUT_OF_VIEW']


def quat_to_R(w, x, y, z):
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def stamp_ns(st):
    return int(st.sec) * 1_000_000_000 + int(st.nanosec)


def make_config(out_dir, filter_on, virtual_ground=None):
    """A variant of follow_test_ros2.yaml for the replayed fsm_node.

    virtual_ground: override rog_map/virtual_ground_height (metres, lidar_init). NEEDED FOR BENCH
    BAGS: the flown value (-1.0, effective -0.5 after ROG-Map's inflation inset) assumes FAST-LIO
    started near the floor. A rig on a table or carried at 1.5 m puts the person's waist a metre
    below that plane, every sample height below it is dropped, and the oracle classifies on one
    ray at the plane -- which is head height, where a cabinet-sized occluder no longer blocks.
    run5 read 32 of 38 labelled occlusions as LOST for exactly that reason (2026-09-14).
    """
    base = open(BASE_CFG).read()
    if not re.search(r'^loss_oracle:\n  enable: true', base, re.M):
        raise SystemExit('%s has no loss_oracle block with enable: true (patch 0004?)' % BASE_CFG)
    cfg = re.sub(r'^(  target_exclusion:\n    enable:) (true|false)',
                 r'\1 %s' % ('true' if filter_on else 'false'), base, flags=re.M)
    if virtual_ground is not None:
        cfg, n = re.subn(r'^(  virtual_ground_height:) *[-0-9.]+', r'\1 %.2f' % virtual_ground, cfg, flags=re.M)
        if n != 1:
            raise SystemExit('could not override rog_map/virtual_ground_height in %s (%d matches)' % (BASE_CFG, n))
    # The config must live under the SOURCE config dir: fsm_node joins config_name onto ROOT_DIR.
    path = os.path.join(SUPER_CFG_DIR, '.loss_replay_%s.yaml' % ('on' if filter_on else 'off'))
    open(path, 'w').write(cfg)
    open(os.path.join(out_dir, 'fsm_config.yaml'), 'w').write(cfg)
    return path


def start_fsm(cfg_path, log_path, domain):
    env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
    rel = os.path.relpath(cfg_path, SUPER_CFG_DIR)
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


def event_to_dict(m):
    p = lambda q: [q.x, q.y, q.z]  # noqa: E731
    return {
        'stamp': stamp_ns(m.header.stamp) * 1e-9, 'eval_stamp': stamp_ns(m.eval_stamp) * 1e-9,
        'track_id': int(m.track_id), 'missed_frames': int(m.missed_frames), 'onset': bool(m.onset),
        'cls': int(m.cls), 'cls_name': CLS[m.cls] if m.cls < 4 else '?',
        'blocked_fraction': float(m.blocked_fraction), 'unknown_fraction': float(m.unknown_fraction),
        'rays_cast': int(m.rays_cast), 'query_p': p(m.query_p), 'camera_p': p(m.camera_p),
        'bearing_deg': float(m.bearing_deg), 'occluder_p': p(m.occluder_p),
        'edge_left': p(m.edge_left), 'edge_right': p(m.edge_right),
        'edge_left_valid': bool(m.edge_left_valid), 'edge_right_valid': bool(m.edge_right_valid),
        'reappear_p': [p(q) for q in m.reappear_p], 'reappear_dt': [float(v) for v in m.reappear_dt],
        'reappear_dt_min': [float(v) for v in m.reappear_dt_min],
        'reappear_dt_max': [float(v) for v in m.reappear_dt_max],
        'reappear_sigma_m': [float(v) for v in m.reappear_sigma_m],
        'reappear_w': [float(v) for v in m.reappear_w], 'reappear_side': [int(v) for v in m.reappear_side],
        'still_hidden_w': float(m.still_hidden_w), 'rollouts': int(m.rollouts),
        'yaw_target_p': p(m.yaw_target_p), 'viewpoint_valid': bool(m.viewpoint_valid),
        'viewpoint_p': p(m.viewpoint_p), 'classify_us': int(m.classify_us), 'eval_us': int(m.eval_us),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--out', required=True)
    ap.add_argument('--rate', type=float, default=1.0, help='playback speed (default 1.0)')
    ap.add_argument('--filter', choices=['off', 'on'], default='off',
                    help='rog_map/target_exclusion in the replayed fsm_node (default: off, as flown)')
    ap.add_argument('--start', type=float, default=0.0, help='skip this many seconds of bag')
    ap.add_argument('--duration', type=float, default=0.0, help='stop after this many seconds (0 = all)')
    ap.add_argument('--domain', default=DOMAIN)
    ap.add_argument('--settle', type=float, default=25.0, help='seconds to let fsm_node preallocate')
    ap.add_argument('--virtual-ground', type=float, default=None,
                    help='override rog_map/virtual_ground_height for a bag whose lidar_init origin sits high '
                         'above the floor (bench rig, handheld); see make_config')
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = args.domain
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.makedirs(args.out, exist_ok=True)

    import rclpy
    import rosbag2_py
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from nav_msgs.msg import Odometry
    from dtp_msgs.msg import Target, LossEvent

    cfg_path = make_config(args.out, args.filter == 'on', args.virtual_ground)
    proc, log = start_fsm(cfg_path, os.path.join(args.out, 'fsm_node.log'), args.domain)

    rclpy.init()
    node = rclpy.create_node('loss_replay')
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                    depth=1, durability=DurabilityPolicy.VOLATILE)
    ev_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                        depth=100, durability=DurabilityPolicy.VOLATILE)
    cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', be)
    odom_pub = node.create_publisher(Odometry, '/lidar_slam/odom', be)
    target_pub = node.create_publisher(Target, '/dtp/follow_target', be)
    events = []
    node.create_subscription(LossEvent, '/planning/loss_event', lambda m: events.append(m), ev_qos)
    occ_seen = {'t': None}
    node.create_subscription(PointCloud2, '/rog_map/occ', lambda m: occ_seen.__setitem__('t', time.time()), be)

    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]

    def spin(secs):
        end = time.time() + secs
        while True:
            left = end - time.time()
            if left <= 0:
                return
            rclpy.spin_once(node, timeout_sec=min(0.005, left))

    try:
        print('waiting %.0f s for fsm_node (A* preallocation) ...' % args.settle)
        spin(args.settle)
        if proc.poll() is not None:
            raise RuntimeError('fsm_node exited early; see %s' % os.path.join(args.out, 'fsm_node.log'))

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        topics = ['/cloud_registered_body', '/lidar_slam/odom', '/dtp/follow_target', '/dtp/lock_state']
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

        odom_by_stamp = {}
        latest_odom = None
        bag_t0 = None
        wall_t0 = None
        n_pub = {t: 0 for t in topics}
        lock_states = []            # (bag_t, text)
        answered = {}               # (track, stamp_ns, missed) -> (bag_t, seq)
        target_seq = 0
        last_progress = 0.0
        while reader.has_next():
            topic, data, ts = reader.read_next()
            bag_t = ts * 1e-9
            if bag_t0 is None:
                bag_t0 = bag_t
                wall_t0 = time.time()
            rel = bag_t - bag_t0
            if rel < args.start:
                continue
            if args.duration > 0 and rel > args.start + args.duration:
                break
            # pace
            due = wall_t0 + (rel - args.start) / max(args.rate, 1e-3)
            while time.time() < due:
                spin(min(0.005, due - time.time()))
            if topic == '/lidar_slam/odom':
                m = deserialize_message(data, Odometry)
                odom_by_stamp[stamp_ns(m.header.stamp)] = m
                if len(odom_by_stamp) > 50:
                    for k in sorted(odom_by_stamp)[:-30]:
                        del odom_by_stamp[k]
                latest_odom = m
                odom_pub.publish(m)
            elif topic == '/cloud_registered_body':
                m = deserialize_message(data, PointCloud2)
                od = odom_by_stamp.get(stamp_ns(m.header.stamp), latest_odom)
                if od is None:
                    continue
                pts = np.asarray(point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z')),
                                 dtype=np.float64).reshape(-1, 3)
                q = od.pose.pose.orientation
                R = quat_to_R(q.w, q.x, q.y, q.z)
                t = od.pose.pose.position
                pw = pts @ R.T + np.array([t.x, t.y, t.z])
                data_out = np.hstack([pw.astype(np.float32), np.full((len(pw), 1), 100.0, np.float32)])
                cloud_pub.publish(point_cloud2.create_cloud(
                    Header(stamp=m.header.stamp, frame_id='world'), fields, data_out))
            elif topic == '/dtp/follow_target':
                m = deserialize_message(data, Target)
                target_seq += 1
                answered[(int(m.track_id), stamp_ns(m.header.stamp), int(m.missed_frames))] = (bag_t, target_seq)
                target_pub.publish(m)
            else:
                m = deserialize_message(data, String)
                lock_states.append((bag_t, m.data))
            n_pub[topic] += 1
            spin(0.0)
            if rel - last_progress >= 10.0:
                last_progress = rel
                print('  bag %6.1f s | clouds %d odom %d targets %d | events %d'
                      % (rel, n_pub['/cloud_registered_body'], n_pub['/lidar_slam/odom'],
                         n_pub['/dtp/follow_target'], len(events)), flush=True)
        spin(1.5)
    finally:
        stop_fsm(proc, log)
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.remove(cfg_path)
        except OSError:
            pass

    # ---- write events.jsonl with bag context and episode indices -------------------------------
    rows = []
    episode = 0
    prev = None
    for m in events:
        d = event_to_dict(m)
        key = (d['track_id'], stamp_ns(m.header.stamp), d['missed_frames'])
        bt = answered.get(key)
        d['bag_t'] = None if bt is None else bt[0] - bag_t0
        if (d['onset'] or prev is None or prev['track_id'] != d['track_id']
                or d['missed_frames'] <= prev['missed_frames']):
            episode += 1
        d['episode'] = episode
        rows.append(d)
        prev = d
    with open(os.path.join(args.out, 'events.jsonl'), 'w') as f:
        for d in rows:
            f.write(json.dumps(d) + '\n')

    # ---- per-episode summary ------------------------------------------------------------------
    def pct(v, q):
        v = sorted(v)
        return v[min(len(v) - 1, int(q * len(v)))] if v else 0

    eps = {}
    for d in rows:
        eps.setdefault(d['episode'], []).append(d)
    out_rows = []
    for ep, ds in sorted(eps.items()):
        first = ds[0]
        counts = {c: sum(1 for d in ds if d['cls_name'] == c) for c in CLS}
        majority = max(counts, key=counts.get)
        onset_t = first['bag_t']
        outcome, outcome_dt, outcome_text = '?', None, ''
        if onset_t is not None:
            for bt, s in lock_states:
                if bt - bag_t0 <= onset_t:
                    continue
                if s.startswith('LOCKED|'):
                    outcome, outcome_dt, outcome_text = 'RE-ACQUIRED', bt - bag_t0 - onset_t, s
                    break
                if s.startswith('LOST|'):
                    outcome, outcome_dt, outcome_text = 'LOST', bt - bag_t0 - onset_t, s
                    break
        belief = first['reappear_p']
        out_rows.append({
            'episode': ep, 'track_id': first['track_id'], 'onset_bag_t': None if onset_t is None else round(onset_t, 3),
            'n_events': len(ds), 'first_cls': first['cls_name'], 'majority_cls': majority,
            'n_occluded': counts['OCCLUDED'], 'n_unmapped': counts['UNMAPPED'], 'n_lost': counts['LOST'],
            'n_out_of_view': counts['OUT_OF_VIEW'],
            'onset_blocked': round(first['blocked_fraction'], 3), 'onset_unknown': round(first['unknown_fraction'], 3),
            'onset_bearing_deg': round(first['bearing_deg'], 1),
            'query_x': round(first['query_p'][0], 2), 'query_y': round(first['query_p'][1], 2),
            'onset_side': first['reappear_side'][0] if belief else '',
            'onset_dt': round(first['reappear_dt'][0], 2) if belief else '',
            'onset_sigma': round(first['reappear_sigma_m'][0], 2) if belief else '',
            'onset_w': round(first['reappear_w'][0], 2) if belief else '',
            'viewpoint_frac': round(sum(1 for d in ds if d['viewpoint_valid']) / len(ds), 2),
            'classify_us_p99': pct([d['classify_us'] for d in ds], 0.99),
            'eval_us_p99': pct([d['eval_us'] for d in ds], 0.99),
            'recorded_outcome': outcome, 'recorded_outcome_dt': None if outcome_dt is None else round(outcome_dt, 2),
            'recorded_outcome_text': outcome_text,
        })
    with open(os.path.join(args.out, 'episodes.csv'), 'w', newline='') as f:
        if out_rows:
            w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
            w.writeheader()
            w.writerows(out_rows)

    total = {c: sum(1 for d in rows if d['cls_name'] == c) for c in CLS}
    print('\n%s: %d LossEvents in %d coast episodes | classes %s | classify p99 %d us, eval p99 %d us'
          % (os.path.basename(args.bag.rstrip('/')), len(rows), len(eps), total,
             pct([d['classify_us'] for d in rows], 0.99), pct([d['eval_us'] for d in rows], 0.99)))
    if out_rows:
        first_counts = {c: sum(1 for r in out_rows if r['first_cls'] == c) for c in CLS}
        maj_counts = {c: sum(1 for r in out_rows if r['majority_cls'] == c) for c in CLS}
        print('episodes by first verdict %s | by majority %s' % (first_counts, maj_counts))
        oc = [r for r in out_rows if r['recorded_outcome'] != '?']
        print('recorded outcomes: %d re-acquired, %d lost (of %d with a known outcome)'
              % (sum(1 for r in oc if r['recorded_outcome'] == 'RE-ACQUIRED'),
                 sum(1 for r in oc if r['recorded_outcome'] == 'LOST'), len(oc)))
    print('wrote %s and %s' % (os.path.join(args.out, 'events.jsonl'), os.path.join(args.out, 'episodes.csv')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
