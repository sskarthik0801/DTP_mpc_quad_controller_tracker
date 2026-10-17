#!/usr/bin/env python3
"""M6.2 replay harness, stage 3: the CLOSED LOOP on a recorded bag -- oracle, lock and follower live.

loss_condition_replay.py replays the oracle's RECORDED events, which answer the RECORDED coasts;
under the conditioned arm the coasts run longer than the recording did and the extra frames have
no event, so that comparison is a lower bound. This script closes the loop: the real fsm_node
(SUPER + ROG-Map + the M6.1 oracle), the real target_manager and the real follow_planner run
together on a private localhost-only domain, and the bag supplies only what the SENSORS gave --
/lidar_slam/odom, /cloud_registered_body (transformed into lidar_init, as the map was built live)
and perception's /dtp/targets. target_manager coasts, fsm_node answers those coasts as they
happen, target_manager and follow_planner condition on the answers (or ignore them, per arm).

    --arm off | on | stock           the same three arms as loss_condition_replay.py

fsm_node receives no goal (follow_planner publishes /dtp/follow_goal and nothing arbitrates it
onto /planning/follow_goal here), so SUPER never plans: the map, the oracle and the lock are what
is being exercised, and the drone's pose is the recorded one. The follower's goals and yaw
targets are counted, not scored.

auto_reacquire_after_lost is forced TRUE, as in the open-loop replay. Each arm needs its own run
(~bag length + 30 s); never run two on the same domain at once -- they would see each other's
targets (found the hard way on 2026-09-14).

OUTPUT (--out DIR): coasts.csv, lock_state.txt, summary.json (mode: closed-loop), events.jsonl
(the live LossEvents), fsm_node.log, target_manager_<arm>.log, follow_planner_<arm>.log, and the
CPU seconds each process used over the run (summary.json: cpu_s). Compare arms with
    loss_condition_replay.py --compare DIR_OFF DIR_ON
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loss_replay as lr  # noqa: E402
import loss_condition_replay as lcr  # noqa: E402


def start_follow_planner(arm, logdir, domain):
    env = dict(os.environ, ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
    log = open(os.path.join(logdir, 'follow_planner_%s.log' % arm), 'w')
    args = ['ros2', 'run', 'follow_planner', 'follow_planner_node', '--ros-args',
            '-p', 'loss_conditioning:=%s' % ('true' if arm == 'on' else 'false'),
            '-p', 'stock_baseline:=%s' % ('true' if arm == 'stock' else 'false'),
            '-p', 'goal_status_timeout_s:=100.0']
    proc = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True)
    return proc, log


def cpu_seconds(pid):
    """CPU seconds of every process in pid's session (start_new_session=True), not just pid.

    `ros2 run` is a Python wrapper whose child is the real node; measuring the wrapper alone reads
    0.0 (it did, on the first run). Summing the session gets the node and the wrapper together.
    """
    total = 0.0
    found = False
    tck = os.sysconf('SC_CLK_TCK')
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open('/proc/%s/stat' % d) as f:
                parts = f.read().rsplit(')', 1)[1].split()
            if int(parts[3]) != pid:          # session id
                continue
            total += (int(parts[11]) + int(parts[12])) / tck
            found = True
        except (OSError, IndexError, ValueError):
            continue
    return total if found else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('bag')
    ap.add_argument('--out', required=True)
    ap.add_argument('--arm', choices=['off', 'on', 'stock'], default='off')
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--duration', type=float, default=0.0)
    ap.add_argument('--filter', choices=['off', 'on'], default='off',
                    help='rog_map/target_exclusion in the replayed fsm_node (default: off, as flown)')
    ap.add_argument('--domain', default=lr.DOMAIN)
    ap.add_argument('--settle', type=float, default=25.0)
    ap.add_argument('--virtual-ground', type=float, default=None,
                    help='override rog_map/virtual_ground_height (see loss_replay.make_config)')
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = args.domain
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.makedirs(args.out, exist_ok=True)

    import rclpy
    import rosbag2_py
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, PointStamped
    from dtp_msgs.msg import Target, TargetArray, LossEvent

    cfg_path = lr.make_config(args.out, args.filter == 'on', args.virtual_ground)
    fsm_proc, fsm_log = lr.start_fsm(cfg_path, os.path.join(args.out, 'fsm_node.log'), args.domain)
    tm_proc, tm_log = lcr.start_target_manager(args.arm, args.out, args.domain)
    fp_proc, fp_log = start_follow_planner(args.arm, args.out, args.domain)

    rclpy.init()
    node = rclpy.create_node('loss_closed_loop_replay')
    be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                    depth=1, durability=DurabilityPolicy.VOLATILE)
    ev_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                        depth=100, durability=DurabilityPolicy.VOLATILE)
    cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', be)
    odom_pub = node.create_publisher(Odometry, '/lidar_slam/odom', be)
    targets_pub = node.create_publisher(TargetArray, '/dtp/targets', be)
    states, fts, events, goals, yaws = [], [], [], [], []
    clock = {'rel': 0.0}
    node.create_subscription(String, '/dtp/lock_state', lambda m: states.append((clock['rel'], m.data)), 10)
    node.create_subscription(Target, '/dtp/follow_target', lambda m: fts.append((clock['rel'], m)),
                             qos_profile_sensor_data)
    node.create_subscription(LossEvent, '/planning/loss_event', lambda m: events.append((clock['rel'], m)), ev_qos)
    node.create_subscription(PoseStamped, '/dtp/follow_goal', lambda m: goals.append(clock['rel']), be)
    node.create_subscription(PointStamped, '/planning/yaw_target', lambda m: yaws.append(clock['rel']), be)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]

    def spin(secs):
        end = time.time() + secs
        while True:
            left = end - time.time()
            if left <= 0:
                return
            rclpy.spin_once(node, timeout_sec=min(0.005, left))

    last_seen_pos = {}
    cpu0 = {}
    n_pub = {'cloud': 0, 'odom': 0, 'targets': 0}
    try:
        print('waiting %.0f s for fsm_node (A* preallocation) ...' % args.settle)
        spin(args.settle)
        for name, p in (('fsm_node', fsm_proc), ('target_manager', tm_proc), ('follow_planner', fp_proc)):
            if p.poll() is not None:
                raise RuntimeError('%s exited early; see %s' % (name, args.out))
            cpu0[name] = cpu_seconds(p.pid)
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(
            topics=['/cloud_registered_body', '/lidar_slam/odom', '/dtp/targets']))
        odom_by_stamp, latest_odom = {}, None
        bag_t0, wall_t0, last_progress = None, None, 0.0
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
            if topic == '/lidar_slam/odom':
                m = deserialize_message(data, Odometry)
                odom_by_stamp[lr.stamp_ns(m.header.stamp)] = m
                if len(odom_by_stamp) > 50:
                    for k in sorted(odom_by_stamp)[:-30]:
                        del odom_by_stamp[k]
                latest_odom = m
                odom_pub.publish(m)
                n_pub['odom'] += 1
            elif topic == '/cloud_registered_body':
                m = deserialize_message(data, PointCloud2)
                od = odom_by_stamp.get(lr.stamp_ns(m.header.stamp), latest_odom)
                if od is None:
                    continue
                pts = np.asarray(point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z')),
                                 dtype=np.float64).reshape(-1, 3)
                q = od.pose.pose.orientation
                t = od.pose.pose.position
                pw = pts @ lr.quat_to_R(q.w, q.x, q.y, q.z).T + np.array([t.x, t.y, t.z])
                data_out = np.hstack([pw.astype(np.float32), np.full((len(pw), 1), 100.0, np.float32)])
                cloud_pub.publish(point_cloud2.create_cloud(
                    Header(stamp=m.header.stamp, frame_id='world'), fields, data_out))
                n_pub['cloud'] += 1
            else:
                m = deserialize_message(data, TargetArray)
                for tg in m.targets:
                    last_seen_pos[int(tg.track_id)] = (tg.position.x, tg.position.y, rel)
                targets_pub.publish(m)
                n_pub['targets'] += 1
            spin(0.0)
            if rel - last_progress >= 10.0:
                last_progress = rel
                print('  bag %6.1f s | clouds %d odom %d targets %d | follow_targets %d events %d | lock %s'
                      % (rel, n_pub['cloud'], n_pub['odom'], n_pub['targets'], len(fts), len(events),
                         states[-1][1].split('|')[0] if states else '-'), flush=True)
        spin(2.0)
        cpu = {name: (None if cpu0.get(name) is None or cpu_seconds(p.pid) is None
                      else round(cpu_seconds(p.pid) - cpu0[name], 1))
               for name, p in (('fsm_node', fsm_proc), ('target_manager', tm_proc), ('follow_planner', fp_proc))}
        bag_len = clock['rel']
    finally:
        for p, lg in ((fp_proc, fp_log), (tm_proc, tm_log)):
            lcr.stop(p, lg)
        lr.stop_fsm(fsm_proc, fsm_log)
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.remove(cfg_path)
        except OSError:
            pass

    with open(os.path.join(args.out, 'events.jsonl'), 'w') as f:
        for rel, m in events:
            d = lr.event_to_dict(m)
            d['bag_t'] = round(rel, 3)
            f.write(json.dumps(d) + '\n')
    cls_counts = {c: sum(1 for _, m in events if m.cls == i) for i, c in enumerate(lr.CLS)}
    lcr.score_lock(states, last_seen_pos, args.out, extra={
        'bag': os.path.basename(args.bag.rstrip('/')), 'arm': args.arm, 'mode': 'closed-loop',
        'bag_seconds': round(bag_len, 1), 'events': len(events), 'event_classes': cls_counts,
        'follow_targets': len(fts), 'follow_goals': len(goals), 'yaw_targets': len(yaws),
        'cpu_s': cpu,
        'cpu_pct_of_core': {k: (None if v is None or bag_len <= 0
                                else round(100.0 * v / (bag_len / max(args.rate, 1e-3)), 1))
                            for k, v in cpu.items()}})
    return 0


if __name__ == '__main__':
    sys.exit(main())
