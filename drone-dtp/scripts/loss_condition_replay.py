#!/usr/bin/env python3
"""M6.2 replay harness, stage 2: the REAL target_manager on a recorded bag, one ablation arm.

Plays a bag's /dtp/targets and /lidar_slam/odom into a private, localhost-only target_manager at
bag timing, and replays the oracle's LossEvents (events.jsonl from loss_replay.py) at the bag time
of the coasting frame each one answered, re-stamped to the replay clock. Then it scores what the
lock did: every coast, how it ended, how fast, and how far the re-acquired candidate was from the
last seen position.

Run it once per arm on the same bag and events:
    --arm off     loss_conditioning false  (the carry-forward arm)
    --arm on      loss_conditioning true   (M6.5: region gate, belief-bounded coast, LOST ends it)
    --arm stock   stock_baseline true      (M6.8: no coast, no abstain)

THE APPROXIMATION, stated: the LossEvents were computed against the RECORDED coasts (the bag's
own target_manager, carry-forward arm). The replayed node sees the same /dtp/targets, so its
coasts start at the same frames and its CTRV points are the same forecast, but under the ON arm
they run longer than the recorded ones -- and for those extra frames there is no recorded event.
The last event of the episode stays "fresh" for loss_event_max_age_s (0.5 s) and then the plain
coast rule applies, so the ON arm here is a LOWER bound on what a live oracle gives it. A
closed-loop replay (fsm_node + target_manager together) removes that; it is the same script with
loss_replay.py's fsm_node in the loop and is not written yet.

auto_reacquire_after_lost is forced TRUE: nobody re-designates in a replay, and the metric is per
coast, not per session. It is the bench-only setting and this is the bench.

OUTPUT (--out DIR): coasts.csv (one row per coast), lock_state.txt (every transition with the bag
time), summary.json, and the summary printed. Compare two arms with:
    loss_condition_replay.py --compare DIR_OFF DIR_ON
"""
import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

DOMAIN = '87'


def start_target_manager(arm, logdir, domain):
    env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
    log = open(os.path.join(logdir, 'target_manager_%s.log' % arm), 'w')
    args = ['ros2', 'run', 'dtp_perception', 'target_manager', '--ros-args',
            '-p', 'follow_enabled:=true', '-p', 'auto_reacquire_after_lost:=true',
            '-p', 'world_frame:=lidar_init',
            '-p', 'loss_conditioning:=%s' % ('true' if arm == 'on' else 'false'),
            '-p', 'stock_baseline:=%s' % ('true' if arm == 'stock' else 'false')]
    proc = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True)
    return proc, log


def stop(proc, log):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
    log.close()


def load_events(path):
    evs = []
    with open(path) as f:
        for line in f:
            d = json.loads(line)
            if d.get('bag_t') is not None:
                evs.append(d)
    evs.sort(key=lambda d: d['bag_t'])
    return evs


def dict_to_event(d, LossEvent, Point, now_msg):
    ev = LossEvent()
    ev.eval_stamp = now_msg
    ev.track_id = d['track_id']
    ev.missed_frames = d['missed_frames']
    ev.onset = d['onset']
    ev.cls = d['cls']
    ev.blocked_fraction = d['blocked_fraction']
    ev.unknown_fraction = d['unknown_fraction']
    ev.rays_cast = d['rays_cast']
    for name in ('query_p', 'camera_p', 'occluder_p', 'edge_left', 'edge_right', 'yaw_target_p', 'viewpoint_p'):
        v = d[name]
        setattr(ev, name, Point(x=v[0], y=v[1], z=v[2]))
    ev.bearing_deg = d['bearing_deg']
    ev.edge_left_valid = d['edge_left_valid']
    ev.edge_right_valid = d['edge_right_valid']
    ev.reappear_p = [Point(x=v[0], y=v[1], z=v[2]) for v in d['reappear_p']]
    ev.reappear_dt = d['reappear_dt']
    ev.reappear_dt_min = d['reappear_dt_min']
    ev.reappear_dt_max = d['reappear_dt_max']
    ev.reappear_sigma_m = d['reappear_sigma_m']
    ev.reappear_w = d['reappear_w']
    ev.reappear_side = d['reappear_side']
    ev.still_hidden_w = d['still_hidden_w']
    ev.rollouts = d['rollouts']
    ev.viewpoint_valid = d['viewpoint_valid']
    ev.classify_us = d['classify_us']
    ev.eval_us = d['eval_us']
    return ev


def replay(args):
    os.environ['ROS_DOMAIN_ID'] = args.domain
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.makedirs(args.out, exist_ok=True)
    import rclpy
    import rosbag2_py
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Point
    from dtp_msgs.msg import Target, TargetArray, LossEvent

    events = load_events(args.events)
    proc, log = start_target_manager(args.arm, args.out, args.domain)
    rclpy.init()
    node = rclpy.create_node('loss_condition_replay')
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                    depth=1, durability=DurabilityPolicy.VOLATILE)
    odom_pub = node.create_publisher(Odometry, '/lidar_slam/odom', be)
    targets_pub = node.create_publisher(TargetArray, '/dtp/targets', be)
    event_pub = node.create_publisher(LossEvent, '/planning/loss_event', be)
    states = []           # (bag_rel_t, text)
    fts = []              # (bag_rel_t, Target)
    clock = {'rel': 0.0}
    node.create_subscription(String, '/dtp/lock_state', lambda m: states.append((clock['rel'], m.data)), 10)
    node.create_subscription(Target, '/dtp/follow_target', lambda m: fts.append((clock['rel'], m)),
                             qos_profile_sensor_data)

    def spin(secs):
        end = time.time() + secs
        while True:
            left = end - time.time()
            if left <= 0:
                return
            rclpy.spin_once(node, timeout_sec=min(0.005, left))

    last_seen_pos = {}     # track -> (x, y) of the last seen frame (from /dtp/targets)
    try:
        spin(4.0)
        if proc.poll() is not None:
            raise RuntimeError('target_manager exited; see %s' % args.out)
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/dtp/targets', '/lidar_slam/odom', '/cloud_registered_body',
                                                           '/dtp/follow_target']))
        bag_t0, wall_t0 = None, None
        ei = 0
        while reader.has_next():
            topic, data, ts = reader.read_next()
            bag_t = ts * 1e-9
            if bag_t0 is None:
                bag_t0, wall_t0 = bag_t, time.time()
            rel = bag_t - bag_t0
            if args.duration > 0 and rel > args.duration:
                break
            due = wall_t0 + rel / max(args.rate, 1e-3)
            while time.time() < due:
                spin(min(0.005, due - time.time()))
            clock['rel'] = rel
            # events due before this message
            while ei < len(events) and events[ei]['bag_t'] <= rel:
                event_pub.publish(dict_to_event(events[ei], LossEvent, Point, node.get_clock().now().to_msg()))
                ei += 1
            if topic == '/lidar_slam/odom':
                odom_pub.publish(deserialize_message(data, Odometry))
            elif topic == '/dtp/targets':
                m = deserialize_message(data, TargetArray)
                for t in m.targets:
                    last_seen_pos[int(t.track_id)] = (t.position.x, t.position.y, rel)
                targets_pub.publish(m)
            spin(0.0)
        spin(2.0)
    finally:
        stop(proc, log)
        node.destroy_node()
        rclpy.shutdown()

    score_lock(states, last_seen_pos, args.out, extra={
        'bag': os.path.basename(args.bag.rstrip('/')), 'arm': args.arm, 'mode': 'open-loop',
        'events_replayed': len(events), 'follow_targets': len(fts)})
    return 0


def score_lock(states, last_seen_pos, out_dir, extra=None):
    """Per-coast outcomes from a /dtp/lock_state transcript -> coasts.csv, summary.json (printed).

    states: [(bag_rel_t, text)]; last_seen_pos: track -> (x, y, t) from /dtp/targets. Shared by
    the open-loop replay here and the closed-loop one in loss_closed_loop_replay.py.
    """
    with open(os.path.join(out_dir, 'lock_state.txt'), 'w') as f:
        for t, s in states:
            f.write('%8.3f  %s\n' % (t, s))
    coasts = []
    cur = None
    prev_state = None
    for t, s in states:
        st = s.split('|')[0]
        if st == 'COASTING' and prev_state != 'COASTING' and prev_state != 'ABSTAIN':
            cur = {'onset_t': round(t, 3), 'track': s.split('|')[1], 'outcome': 'OPEN', 'end_t': None,
                   'reason': '', 'abstained': 0, 'relock_track': '', 'jump_m': ''}
            coasts.append(cur)
        if cur is not None and cur['outcome'] == 'OPEN':
            if st == 'ABSTAIN' and prev_state != 'ABSTAIN':
                cur['abstained'] += 1
            if st == 'LOCKED':
                cur['outcome'] = 'RE-ACQUIRED'
                cur['end_t'] = round(t, 3)
                cur['reason'] = s.split('|', 2)[2] if s.count('|') >= 2 else s
                cur['relock_track'] = s.split('|')[1]
                m = re.search(r'\(was (\d+)\)', cur['reason'])
                old = int(m.group(1)) if m else None
                new = int(cur['relock_track']) if cur['relock_track'].isdigit() else None
                if old in last_seen_pos and new in last_seen_pos:
                    ox, oy, _ = last_seen_pos[old]
                    nx, ny, _ = last_seen_pos[new]
                    cur['jump_m'] = round(math.hypot(nx - ox, ny - oy), 2)
            elif st == 'LOST':
                cur['outcome'] = 'LOST'
                cur['end_t'] = round(t, 3)
                cur['reason'] = s.split('|', 2)[2] if s.count('|') >= 2 else s
        prev_state = st
    for c in coasts:
        c['duration'] = '' if c['end_t'] is None else round(c['end_t'] - c['onset_t'], 3)
        c['via'] = ('region' if 're-emergence region' in c['reason'] else
                    'oracle' if 'loss oracle' in c['reason'] else
                    'same_id' if 'same id' in c['reason'] else
                    'band' if 'sole candidate in band' in c['reason'] else
                    'tiebreak' if 'tie-break' in c['reason'] else
                    'timeout' if 'coast timeout' in c['reason'] else
                    'forecast' if 'forecast' in c['reason'] else '')
    with open(os.path.join(out_dir, 'coasts.csv'), 'w', newline='') as f:
        cols = ['onset_t', 'track', 'outcome', 'end_t', 'duration', 'via', 'abstained', 'relock_track', 'jump_m',
                'reason']
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(coasts)
    n = len(coasts)
    re_acq = [c for c in coasts if c['outcome'] == 'RE-ACQUIRED']
    lost = [c for c in coasts if c['outcome'] == 'LOST']
    durs = sorted(c['duration'] for c in re_acq if c['duration'] != '')
    summary = dict(extra or {})
    summary.update({
        'coasts': n,
        're_acquired': len(re_acq), 'lost': len(lost), 'open': n - len(re_acq) - len(lost),
        're_acquire_rate': round(len(re_acq) / n, 3) if n else None,
        're_acquire_median_s': durs[len(durs) // 2] if durs else None,
        're_acquire_via': {k: sum(1 for c in re_acq if c['via'] == k)
                           for k in ('same_id', 'band', 'region', 'tiebreak')},
        'lost_via': {k: sum(1 for c in lost if c['via'] == k) for k in ('oracle', 'timeout', 'forecast')},
        'abstain_entries': sum(c['abstained'] for c in coasts),
        'relocks_with_jump_over_2m': sum(1 for c in re_acq if c['jump_m'] != '' and c['jump_m'] > 2.0),
    })
    json.dump(summary, open(os.path.join(out_dir, 'summary.json'), 'w'), indent=1)
    print(json.dumps(summary, indent=1))
    return summary


def compare(args):
    a = json.load(open(os.path.join(args.compare[0], 'summary.json')))
    b = json.load(open(os.path.join(args.compare[1], 'summary.json')))
    keys = ['coasts', 're_acquired', 'lost', 're_acquire_rate', 're_acquire_median_s', 'abstain_entries',
            'relocks_with_jump_over_2m']
    print('%-28s %14s %14s' % ('', a.get('mode', '') + ' ' + a['arm'], b.get('mode', '') + ' ' + b['arm']))
    for k in keys:
        print('%-28s %14s %14s' % (k, a[k], b[k]))
    for k in ('re_acquire_via', 'lost_via'):
        for kk in a[k]:
            print('%-28s %14s %14s' % (k + '.' + kk, a[k][kk], b[k][kk]))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag', nargs='?')
    ap.add_argument('events', nargs='?')
    ap.add_argument('--arm', choices=['off', 'on', 'stock'], default='off')
    ap.add_argument('--out')
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--duration', type=float, default=0.0)
    ap.add_argument('--domain', default=DOMAIN)
    ap.add_argument('--compare', nargs=2, metavar='DIR')
    args = ap.parse_args(argv)
    if args.compare:
        return compare(args)
    if not (args.bag and args.events and args.out):
        ap.error('bag, events and --out are required (or --compare DIR DIR)')
    return replay(args)


if __name__ == '__main__':
    sys.exit(main())
