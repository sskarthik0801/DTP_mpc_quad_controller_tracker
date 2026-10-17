#!/usr/bin/env python3
"""Bag-driven bench of the REAL, INSTALLED px4_offboard_bridge with the camera yaw servo.

    source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash \
        && source ~/super_ws/install/setup.bash
    python3 scripts/px4/yaw_servo_bag_bench.py --out /tmp/ysb            # the three bags, 3 bridges each
    python3 scripts/px4/yaw_servo_bag_bench.py --out /tmp/ysb --solo     # + one bridge at a time
    python3 scripts/px4/yaw_servo_bag_bench.py --analyze /tmp/ysb/<run>  # re-analyse a finished run

What it does
------------
It feeds the node the RECORDED inputs of a flight, in real time, and records what the node
publishes. Only four topics are taken from the bag and republished, under a sandbox namespace:
/planning/pos_cmd, /planning/yaw_target, /fmu/out/vehicle_local_position_v1 and
/fmu/out/vehicle_control_mode. Nothing else of the bag is published: in particular not the
recorded /fmu/in/trajectory_setpoint or /fmu/in/offboard_control_mode (the old bridge's outputs).

* Isolation. Everything it starts runs with ROS_DOMAIN_ID 96 (--domain) and ROS_LOCALHOST_ONLY=1,
  and every topic of every bridge is remapped under /soak/bench/ through the bridge's own topic
  parameters (cmd_topic, local_position_topic, vehicle_control_mode_topic, yaw_target_topic,
  setpoint_topic, control_mode_topic, yaw_servo_debug_topic), so a domain mistake still cannot
  reach /fmu/in/*. Before it starts it refuses to run if any readable process already has that
  ROS_DOMAIN_ID, or a gate harness (*_check.py, *_replay.py, the soak) is running. Bridges and the
  recorder are started in their own sessions, and stopped by the process group it owns (SIGINT,
  then SIGTERM, then SIGKILL), never by a name pattern. The bag is opened read-only
  (rosbag2_py SequentialReader); nothing is written into bags/.
* The node under test is ~/super_ws/install/super_bringup/lib/super_bringup/px4_offboard_bridge.py
  (env BRIDGE to override), run with python3 exactly as `ros2 run` runs it; its parameters are
  the defaults except the topics and the switches.
* Variants (default all three, one bridge each, in PARALLEL on the same inputs and separate
  outputs): off (both switches off, the baseline), fix1 (yaw_servo_enable), fix12 (+
  yaw_servo_ff_enable). --solo also plays each bag once per variant with that bridge alone, as
  the control for the tick rate.
* TIME (approach A of the task). The player republishes each message at its original relative
  bag (receive) time: wall = t_bag + offset, offset = wall at start - t_bag at start. The yaw
  target's header stamp is re-stamped to (original stamp + offset), so its age at arrival on the
  node's system clock is what it was in flight (the node's freshness test is node clock - stamp).
  pos_cmd, lpos and vehicle_control_mode are republished as the recorded CDR bytes (the node
  uses none of their stamps). The node stays on the system clock, as in flight (no sim time).
* QoS. PX4 topics are published BEST_EFFORT + TRANSIENT_LOCAL (uXRCE-DDS), pos_cmd and the yaw
  target BEST_EFFORT / VOLATILE / KEEP_LAST 1 (SUPER, follow_planner). Before the play starts
  it waits until every input publisher has matched every bridge's subscription
  (Publisher.get_subscription_count), checks QoS compatibility on the graph, checks each
  bridge's own topic list, checks that no /fmu, /planning or /bridge topic exists on the domain,
  and waits until the recorder receives every bridge's heartbeat.
* Window: from --pre (30 s) before the first Offboard report to --post (35 s) after the Offboard-
  off report (clipped to the bag).
* Recorder: a subprocess (this script, --role recorder) records every setpoint (its own timestamp
  and all fields), every offboard_control_mode and every /bridge/yaw_servo debug message with its
  receive time, in npz chunks every 5 s (a crash loses at most 5 s).

The analysis (checks 1-10 of the task, printed as PASS/FAIL lines with the measured number)
imports drone-dtp/scripts/fix1_yaw_replay.py for the offline replay of the same module on the
same bag and for its fitted PX4 yaw model (never a copy of its code).
"""
import argparse
import glob
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.realpath(__file__))
_CAND = os.path.normpath(os.path.join(HERE, '..', '..'))
REPO = os.environ.get('YSB_REPO') or (
    _CAND if os.path.isdir(os.path.join(_CAND, 'drone-dtp', 'scripts'))
    else os.path.expanduser('~/SUPER-Human-DTP'))
BRIDGE = os.environ.get('BRIDGE', os.path.expanduser(
    '~/super_ws/install/super_bringup/lib/super_bringup/px4_offboard_bridge.py'))
BAGS = ('stage_b_2026-09-26_200650', 'stage_b_2026-09-26_195108',
        'stage_b_2026-09-21_195658_salvaged')
NS = '/soak/bench'
VARIANTS = {
    'off': [],
    'fix1': ['yaw_servo_enable:=true'],
    'fix12': ['yaw_servo_enable:=true', 'yaw_servo_ff_enable:=true'],
}
T_CMD, T_YT, T_LPOS, T_VCM = ('/planning/pos_cmd', '/planning/yaw_target',
                              '/fmu/out/vehicle_local_position_v1',
                              '/fmu/out/vehicle_control_mode')
IN_TOPICS = {T_CMD: 'pos_cmd', T_YT: 'yaw_target', T_LPOS: 'vehicle_local_position',
             T_VCM: 'vehicle_control_mode'}
KIND = {T_CMD: 0, T_YT: 1, T_LPOS: 2, T_VCM: 3}
HARNESS_NAMES = ('yaw_servo_soak.py', 'yaw_servo_bag_bench.py', 'fix1_yaw_replay.py',
                 'yaw_chain_report.py', 'nmpc_replay.py')
MAX_RATE = math.radians(90.0)
FF_ACC = math.radians(90.0)
SLEW_TOL = math.radians(0.001)       # per pair, on top of limit * dt (the task's tolerance)
FF_TOL = math.radians(0.001)         # deg/s, on top of ff_max_accel * dt (float32 + us stamps)
F32_PI = 3.1415927410125732          # float32(pi): the largest |yaw| a float32 setpoint can hold


# ---------------------------------------------------------------------------------------------
# environment, isolation, processes
# ---------------------------------------------------------------------------------------------

def env_setup(domain, logdir):
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    os.environ['PYTHONUNBUFFERED'] = '1'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '0'
    os.makedirs(logdir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = logdir


def _ancestors():
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])
        except Exception:
            break
    return out


def _argv(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except Exception:
        return []


def domain_users(domain, exclude=()):
    """Readable processes whose environment has ROS_DOMAIN_ID=<domain> -> [(pid, argv)]."""
    key = f'ROS_DOMAIN_ID={domain}'.encode()
    hits = []
    for p in os.listdir('/proc'):
        if not p.isdigit() or int(p) in exclude:
            continue
        try:
            with open(f'/proc/{p}/environ', 'rb') as f:
                env = f.read().split(b'\0')
        except Exception:
            continue
        if key in env:
            hits.append((int(p), _argv(int(p))))
    return hits


def harnesses(exclude=()):
    """Running gate harnesses, by the basename of an argv token (never a flattened line)."""
    hits = []
    for p in os.listdir('/proc'):
        if not p.isdigit() or int(p) in exclude:
            continue
        argv = _argv(int(p))
        base = [os.path.basename(a) for a in argv[:4]]
        if any(b.endswith('_check.py') or b in HARNESS_NAMES for b in base):
            hits.append((int(p), argv))
    return hits


def isolation_check(domain):
    ex = _ancestors()
    bad = [f'pid {p} has ROS_DOMAIN_ID={domain}: {" ".join(a)[:160]}' for p, a in domain_users(domain, ex)]
    bad += [f'pid {p} is a gate/replay harness: {" ".join(a)[:160]}' for p, a in harnesses(ex)]
    return bad


def start_proc(argv, log):
    f = open(log, 'wb')
    p = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    f.close()
    return p


def stop_proc(p, grace=6.0):
    """SIGINT to the process group we created (start_new_session: pgid == pid), then SIGTERM,
    then SIGKILL. Waits on the PID. Returns the exit code."""
    if p.poll() is not None:
        return p.returncode
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 3.0), (signal.SIGKILL, 3.0)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            break
        try:
            return p.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue
    return p.poll()


def bridge_argv(name, ns, extra):
    argv = [sys.executable, BRIDGE, '--ros-args', '-r', f'__node:=ysb_bridge_{name}',
            '-p', f'cmd_topic:={ns}/in/pos_cmd',
            '-p', f'local_position_topic:={ns}/in/vehicle_local_position',
            '-p', f'vehicle_control_mode_topic:={ns}/in/vehicle_control_mode',
            '-p', f'yaw_target_topic:={ns}/in/yaw_target',
            '-p', f'setpoint_topic:={ns}/{name}/trajectory_setpoint',
            '-p', f'control_mode_topic:={ns}/{name}/offboard_control_mode',
            '-p', f'yaw_servo_debug_topic:={ns}/{name}/yaw_servo']
    for x in extra:
        argv += ['-p', x]
    return argv


# ---------------------------------------------------------------------------------------------
# the recorder (subprocess role)
# ---------------------------------------------------------------------------------------------

def role_recorder(cfg_path):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_check_compatible, QoSCompatibility)
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
    from std_msgs.msg import Float32MultiArray

    cfg = json.load(open(cfg_path))
    out = cfg['out']
    rclpy.init()
    node = Node(cfg.get('node', 'ysb_recorder'))
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=500)
    bufs, counts = {}, {}

    def add(key, row):
        bufs[key].append(row)
        counts[key] += 1

    for name, t in cfg['bridges'].items():
        for kind in ('sp', 'ocm', 'dbg'):
            if t.get(kind):
                bufs[f'{kind}_{name}'] = []
                counts[f'{kind}_{name}'] = 0
        node.create_subscription(
            TrajectorySetpoint, t['sp'],
            lambda m, k=f'sp_{name}': add(k, (time.time_ns(), m.timestamp, *m.position.tolist(),
                                              *m.velocity.tolist(), *m.acceleration.tolist(),
                                              *m.jerk.tolist(), m.yaw, m.yawspeed)), qos)
        node.create_subscription(
            OffboardControlMode, t['ocm'],
            lambda m, k=f'ocm_{name}': add(k, (time.time_ns(), m.timestamp)), qos)
        if t.get('dbg'):
            node.create_subscription(
                Float32MultiArray, t['dbg'],
                lambda m, k=f'dbg_{name}': add(k, (time.time_ns(), *list(m.data)[:7])), qos)

    chunk = [0]

    def flush():
        arrs = {}
        for k, rows in bufs.items():
            if not rows:
                continue
            arrs[k + '__recv'] = np.array([r[0] for r in rows], np.int64)
            if k.startswith('dbg_'):
                arrs[k + '__data'] = np.array([r[1:] for r in rows], np.float64)
            else:
                arrs[k + '__ts'] = np.array([r[1] for r in rows], np.int64)
                if k.startswith('sp_'):
                    arrs[k + '__f'] = np.array([r[2:] for r in rows], np.float64)
            rows.clear()
        if arrs:
            tmp = os.path.join(out, f'.rec_{chunk[0]:05d}.npz')
            np.savez(tmp, **arrs)
            os.replace(tmp, os.path.join(out, f'rec_{chunk[0]:05d}.npz'))
            chunk[0] += 1

    def graph():
        g = {}
        for name, t in cfg['bridges'].items():
            for kind in ('sp', 'ocm', 'dbg'):
                top = t.get(kind)
                if not top:
                    continue
                infos = node.get_publishers_info_by_topic(top)
                mine = [i.qos_profile for i in node.get_subscriptions_info_by_topic(top)
                        if i.node_name == node.get_name()]
                # Graph QoS against graph QoS (a local profile's "default" deadline against the
                # graph's "Infinite" reads as a false ERROR). WARNING = both default liveliness.
                g[f'{kind}_{name}'] = [bool(mine) and qos_check_compatible(i.qos_profile, mine[0])[0]
                                      != QoSCompatibility.ERROR for i in infos]
        return g

    def status():
        tmp = os.path.join(out, '.rec_status.json')
        try:
            g = graph()
        except Exception as e:  # after SIGINT rclpy has shut the context down: no graph
            g = {'error': repr(e)[:200]}
        with open(tmp, 'w') as f:
            json.dump({'t': time.time(), 'counts': counts, 'graph': g, 'chunks': chunk[0]}, f)
        os.replace(tmp, os.path.join(out, 'rec_status.json'))

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)
    t_flush = t_stat = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - t_stat > 0.5:
                status()
                t_stat = now
            if now - t_flush > cfg.get('flush_s', 5.0):
                flush()
                t_flush = now
    except KeyboardInterrupt:
        pass
    finally:
        flush()
        status()
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def load_rec(rundir):
    import numpy as np
    parts = {}
    for f in sorted(glob.glob(os.path.join(rundir, 'rec_*.npz'))):
        z = np.load(f)
        for k in z.files:
            parts.setdefault(k, []).append(z[k])
    return {k: np.concatenate(v) for k, v in parts.items()}


# ---------------------------------------------------------------------------------------------
# the bag and the player
# ---------------------------------------------------------------------------------------------

def read_inputs(bag):
    """The four input topics of a bag: {topic: [(t_ns, raw bytes, msg)]}, receive-time order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    missing = [t for t in IN_TOPICS if t not in types]
    if missing:
        sys.exit(f'{bag}: missing input topics {missing}')
    r.set_filter(rosbag2_py.StorageFilter(topics=list(IN_TOPICS)))
    cls = {t: get_message(types[t]) for t in IN_TOPICS}
    d = {t: [] for t in IN_TOPICS}
    while r.has_next():
        topic, data, t = r.read_next()
        d[topic].append((t, bytes(data), deserialize_message(data, cls[topic])))
    for v in d.values():
        v.sort(key=lambda x: x[0])
    return d, types


def bag_start_ns(bag):
    import yaml
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        m = yaml.safe_load(f)['rosbag2_bagfile_information']
    return int(m['starting_time']['nanoseconds_since_epoch']), int(m['duration']['nanoseconds'])


def play_window(d, pre_s, post_s):
    vcm = d[T_VCM]
    on = next((t for t, _, m in vcm if m.flag_control_offboard_enabled), None)
    if on is None:
        sys.exit('no Offboard report in the bag')
    off = next((t for t, _, m in vcm if t > on and not m.flag_control_offboard_enabled), vcm[-1][0])
    last = max(v[-1][0] for v in d.values() if v)
    first = min(v[0][0] for v in d.values() if v)
    return max(first, on - int(pre_s * 1e9)), min(last, off + int(post_s * 1e9)), on, off


def run_play(bag, variants, out, pre_s, post_s, domain):
    """One play of `bag` into len(variants) bridges started in parallel. Returns the run dir."""
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_check_compatible, QoSCompatibility)
    from geometry_msgs.msg import PointStamped
    from mars_quadrotor_msgs.msg import PositionCommand
    from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition

    name = os.path.basename(os.path.normpath(bag))
    rundir = os.path.join(out, f'{name}__{"+".join(variants)}')
    os.makedirs(rundir, exist_ok=True)
    for f in glob.glob(os.path.join(rundir, 'rec_*.npz')):
        os.remove(f)
    print(f'\n=== {name}: bridges {variants} -> {rundir}', flush=True)
    d, types = read_inputs(bag)
    t0b, _ = bag_start_ns(bag)
    ws, we, on, off = play_window(d, pre_s, post_s)
    events = sorted((t, KIND[top], i) for top, rows in d.items()
                    for i, (t, _, _) in enumerate(rows) if ws <= t <= we)
    print(f'  window bag t {(ws - t0b) / 1e9:.2f}-{(we - t0b) / 1e9:.2f} s (Offboard report at '
          f'{(on - t0b) / 1e9:.2f}, off at {(off - t0b) / 1e9:.2f}); {len(events)} messages: ' +
          ', '.join(f'{IN_TOPICS[top]} {sum(1 for t, _, _ in rows if ws <= t <= we)}'
                    for top, rows in d.items()), flush=True)

    bad = isolation_check(domain)
    if bad:
        sys.exit('isolation check failed:\n  ' + '\n  '.join(bad))

    procs, meta = {}, {'bag': bag, 'name': name, 'variants': variants, 'bridge': BRIDGE,
                       'bridge_realpath': os.path.realpath(BRIDGE), 'domain': domain, 'ns': NS,
                       'window': [ws, we], 'offboard_on': on, 'offboard_off': off,
                       'bag_start_ns': t0b, 'pre_s': pre_s, 'post_s': post_s}
    rcfg = {'out': rundir, 'node': 'ysb_recorder', 'bridges': {}}
    for v in variants:
        rcfg['bridges'][v] = {'sp': f'{NS}/{v}/trajectory_setpoint',
                              'ocm': f'{NS}/{v}/offboard_control_mode',
                              'dbg': f'{NS}/{v}/yaw_servo' if VARIANTS[v] else None}
    with open(os.path.join(rundir, 'recorder.json'), 'w') as f:
        json.dump(rcfg, f)

    rclpy.init()
    node = None
    try:
        for v in variants:
            procs[v] = start_proc(bridge_argv(v, NS, VARIANTS[v]), os.path.join(rundir, f'bridge_{v}.log'))
        procs['_rec'] = start_proc([sys.executable, os.path.realpath(__file__), '--role', 'recorder',
                                    '--config', os.path.join(rundir, 'recorder.json')],
                                   os.path.join(rundir, 'recorder.log'))
        meta['pids'] = {k: p.pid for k, p in procs.items()}
        node = Node('ysb_player')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        super_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        pubs = {T_CMD: node.create_publisher(PositionCommand, f'{NS}/in/pos_cmd', super_qos),
                T_YT: node.create_publisher(PointStamped, f'{NS}/in/yaw_target', super_qos),
                T_LPOS: node.create_publisher(VehicleLocalPosition, f'{NS}/in/vehicle_local_position',
                                              px4_qos),
                T_VCM: node.create_publisher(VehicleControlMode, f'{NS}/in/vehicle_control_mode',
                                             px4_qos)}
        n_servo = sum(1 for v in variants if VARIANTS[v])
        want = {T_CMD: len(variants), T_LPOS: len(variants), T_VCM: len(variants), T_YT: n_servo}
        qos_of = {T_CMD: super_qos, T_YT: super_qos, T_LPOS: px4_qos, T_VCM: px4_qos}

        def ready():
            st = {}
            try:
                with open(os.path.join(rundir, 'rec_status.json')) as f:
                    st = json.load(f)
            except Exception:
                pass
            c = st.get('counts', {})
            g = st.get('graph', {})
            m_in = {IN_TOPICS[t]: pubs[t].get_subscription_count() for t in pubs}
            ok_in = all(pubs[t].get_subscription_count() == want[t] for t in pubs)
            ok_out = all(c.get(f'ocm_{v}', 0) >= 25 for v in variants) and all(
                g.get(f'dbg_{v}') == [True] for v in variants if VARIANTS[v]) and all(
                g.get(f'sp_{v}') == [True] for v in variants)
            dead = [k for k, p in procs.items() if p.poll() is not None]
            return ok_in and ok_out and not dead, m_in, c, g, dead

        t_end = time.time() + 40.0
        while True:
            ok, m_in, c, g, dead = ready()
            if ok or dead or time.time() > t_end:
                break
            time.sleep(0.2)
        meta['discovery'] = {'inputs_matched': m_in, 'want': {IN_TOPICS[t]: n for t, n in want.items()},
                             'recorder_counts': c, 'recorder_graph': g, 'dead': dead}
        if not ok:
            raise RuntimeError(f'discovery failed: inputs {m_in} want {want}, recorder {c} {g}, dead {dead}')
        # QoS compatibility of every input subscription, and each bridge's own topics.
        qchk = {}
        for t, pub in pubs.items():
            top = f'{NS}/in/{IN_TOPICS[t]}'
            mine = [i.qos_profile for i in node.get_publishers_info_by_topic(top)
                    if i.node_name == node.get_name()]
            qchk[IN_TOPICS[t]] = [
                (i.node_name, bool(mine) and qos_check_compatible(mine[0], i.qos_profile)[0]
                 != QoSCompatibility.ERROR, qos_check_compatible(mine[0], i.qos_profile)[1] if mine else 'no pub')
                for i in node.get_subscriptions_info_by_topic(top)]
        meta['input_qos_ok'] = qchk
        gr = {}
        for v in variants:
            nn = f'ysb_bridge_{v}'
            gr[v] = {'subs': sorted(t for t, _ in node.get_subscriber_names_and_types_by_node(nn, '/')),
                     'pubs': sorted(t for t, _ in node.get_publisher_names_and_types_by_node(nn, '/'))}
        meta['bridge_graph'] = gr
        alltopics = sorted(t for t, _ in node.get_topic_names_and_types())
        meta['domain_topics'] = alltopics
        leak = [t for t in alltopics if t.startswith(('/fmu', '/planning', '/bridge'))]
        foreign = [t for v in gr.values() for t in v['subs'] + v['pubs']
                   if not t.startswith(NS + '/') and t not in ('/rosout', '/parameter_events')]
        if leak or foreign or not all(x[1] for v in qchk.values() for x in v):
            raise RuntimeError(f'sandbox check failed: leak {leak} foreign {foreign} qos {qchk}')
        print(f'  discovery OK: inputs matched {m_in}; outputs {c}; bridge topics sandboxed; '
              f'no /fmu /planning /bridge topic on domain {domain}', flush=True)

        # ---- play ------------------------------------------------------------------------
        n = len(events)
        ev_tsched = np.zeros(n, np.int64)
        ev_tpub = np.zeros(n, np.int64)
        ev_kind = np.array([k for _, k, _ in events], np.int8)
        ev_tbag = np.array([t for t, _, _ in events], np.int64)
        rows_of = {KIND[t]: d[t] for t in d}
        pub_of = {KIND[t]: pubs[t] for t in pubs}
        offset = time.time_ns() + 500_000_000 - ws
        meta['offset_ns'] = offset
        restamped = {}
        for j, (tb, k, i) in enumerate(events):
            ts = tb + offset
            ev_tsched[j] = ts
            dt = ts - time.time_ns()
            if dt > 0:
                time.sleep(dt / 1e9)
            if k == 1:
                m = rows_of[1][i][2]
                s0 = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
                s1 = s0 + offset
                m.header.stamp.sec, m.header.stamp.nanosec = divmod(s1, 1_000_000_000)
                restamped[i] = (s0, s1)
                ev_tpub[j] = time.time_ns()
                pub_of[1].publish(m)
            else:
                ev_tpub[j] = time.time_ns()
                pub_of[k].publish(rows_of[k][i][1])
            if j % 2000 == 0:
                print(f'  play {j}/{n} bag t {(tb - t0b) / 1e9:.1f} s, late '
                      f'{(ev_tpub[j] - ts) / 1e6:.2f} ms', flush=True)
        time.sleep(2.0)
        meta['play_done_ns'] = time.time_ns()
        late = (ev_tpub - ev_tsched) / 1e6
        meta['player_late_ms'] = {'p50': float(np.median(late)), 'p99': float(np.percentile(late, 99)),
                                  'max': float(late.max())}
        print(f'  play done: player lateness ms p50 {np.median(late):.3f} p99 '
              f'{np.percentile(late, 99):.3f} max {late.max():.3f}', flush=True)

        # ---- the inputs as published, for the analysis --------------------------------
        sel = lambda top: [(i, r) for i, r in enumerate(d[top]) if ws <= r[0] <= we]  # noqa: E731
        tpub_of = {}
        for j, (tb, k, i) in enumerate(events):
            tpub_of[(k, i)] = (tb, ev_tpub[j])
        arr = {'ev_kind': ev_kind, 'ev_tbag': ev_tbag, 'ev_tsched': ev_tsched, 'ev_tpub': ev_tpub}
        c_ = sel(T_CMD)
        arr['cmd_tbag'] = np.array([tpub_of[(0, i)][0] for i, _ in c_], np.int64)
        arr['cmd_tpub'] = np.array([tpub_of[(0, i)][1] for i, _ in c_], np.int64)
        arr['cmd_f'] = np.array([[m.position.x, m.position.y, m.position.z, m.velocity.x, m.velocity.y,
                                  m.velocity.z, m.acceleration.x, m.acceleration.y, m.acceleration.z,
                                  m.jerk.x, m.jerk.y, m.jerk.z, m.yaw, m.yaw_dot]
                                 for _, (_, _, m) in c_], np.float64).reshape(-1, 14)
        l_ = sel(T_LPOS)
        arr['lpos_tbag'] = np.array([tpub_of[(2, i)][0] for i, _ in l_], np.int64)
        arr['lpos_tpub'] = np.array([tpub_of[(2, i)][1] for i, _ in l_], np.int64)
        arr['lpos_f'] = np.array([[m.x, m.y, m.z, m.heading, float(m.xy_valid), float(m.z_valid)]
                                  for _, (_, _, m) in l_], np.float64).reshape(-1, 6)
        v_ = sel(T_VCM)
        arr['vcm_tbag'] = np.array([tpub_of[(3, i)][0] for i, _ in v_], np.int64)
        arr['vcm_tpub'] = np.array([tpub_of[(3, i)][1] for i, _ in v_], np.int64)
        arr['vcm_f'] = np.array([[float(m.flag_armed), float(m.flag_control_offboard_enabled)]
                                 for _, (_, _, m) in v_], np.float64).reshape(-1, 2)
        y_ = sel(T_YT)
        arr['yt_tbag'] = np.array([tpub_of[(1, i)][0] for i, _ in y_], np.int64)
        arr['yt_tpub'] = np.array([tpub_of[(1, i)][1] for i, _ in y_], np.int64)
        arr['yt_stamp0'] = np.array([restamped[i][0] for i, _ in y_], np.int64)
        arr['yt_stamp1'] = np.array([restamped[i][1] for i, _ in y_], np.int64)
        arr['yt_f'] = np.array([[m.point.x, m.point.y] for _, (_, _, m) in y_], np.float64).reshape(-1, 2)
        arr['yt_frame'] = np.array([m.header.frame_id for _, (_, _, m) in y_])
        np.savez(os.path.join(rundir, 'player.npz'), **arr)
    finally:
        rc = {}
        if '_rec' in procs:
            rc['_rec'] = stop_proc(procs['_rec'])
        for k, p in procs.items():
            if k != '_rec':
                rc[k] = stop_proc(p)
        meta['exit_codes'] = rc
        meta['alive_after_stop'] = [k for k, p in procs.items() if p.poll() is None]
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        with open(os.path.join(rundir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1, default=str)
    print(f'  stopped: exit codes {rc}', flush=True)
    return rundir


# ---------------------------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------------------------

def _f32(x):
    import numpy as np
    return np.float32(x)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _u32(a):
    import numpy as np
    return np.ascontiguousarray(np.asarray(a, np.float32)).view(np.uint32)


def pct(x, q):
    import numpy as np
    x = np.asarray(x, float)
    return float(np.percentile(x, q)) if len(x) else math.nan


def parse_log(path, offset_ns, t0_ns):
    """ROS console lines -> [(severity, t_rel_s or None, text)] plus tracebacks / stray stderr."""
    rx = re.compile(r'^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[(\d+)\.(\d+)\] \[([^\]]+)\]: (.*)$')
    out = []
    try:
        lines = open(path, errors='replace').read().splitlines()
    except FileNotFoundError:
        return out
    for ln in lines:
        m = rx.match(ln)
        if m:
            t = int(m.group(2)) * 1_000_000_000 + int(m.group(3).ljust(9, '0')[:9])
            out.append((m.group(1), (t - offset_ns - t0_ns) / 1e9 if offset_ns is not None else None,
                        m.group(5)))
        elif ln.strip():
            out.append(('STDERR', None, ln))
    return out


def norm(s):
    return re.sub(r'[-+]?\d+(\.\d+)?', '#', s)


class Sp:
    """One bridge's setpoints, arrays sorted by tick timestamp."""

    def __init__(self, rec, v):
        import numpy as np
        k = f'sp_{v}'
        if k + '__recv' not in rec:
            self.n = 0
            self.ts = np.zeros(0, np.int64)
            return
        o = np.argsort(rec[k + '__ts'], kind='stable')
        self.recv = rec[k + '__recv'][o]
        self.ts = rec[k + '__ts'][o] * 1000          # ns, the bridge clock at the tick
        f = rec[k + '__f'][o]
        self.pos, self.vel, self.acc, self.jerk = f[:, 0:3], f[:, 3:6], f[:, 6:9], f[:, 9:12]
        self.yaw, self.ys = f[:, 12], f[:, 13]
        self.n = len(self.ts)


def analyse(rundir, replay_cache=None, quiet=False):
    """Checks 1-10 for every bridge of one run. Returns (lines, summary dict)."""
    import numpy as np
    sys.path.insert(0, os.path.join(REPO, 'drone-dtp', 'scripts'))
    import fix1_yaw_replay as fr

    meta = json.load(open(os.path.join(rundir, 'meta.json')))
    P = dict(np.load(os.path.join(rundir, 'player.npz')))
    rec = load_rec(rundir)
    off_ns = meta['offset_ns']
    variants = meta['variants']
    bag = meta['bag']
    L = []
    S = {'run': os.path.basename(rundir), 'bridges': {}}

    def say(s=''):
        L.append(s)
        if not quiet:
            print(s, flush=True)

    def check(tag, ok, txt):
        say(f'  [{"PASS" if ok else "FAIL"}] {tag} {txt}')
        return ok

    # ---- the replay of the same bag (the module, offline) --------------------------------
    key = meta['bag']
    if replay_cache is not None and key in replay_cache:
        c, oB, oC = replay_cache[key]
    else:
        c = fr.prepare(bag)
        oB = fr.run(fr.P(enable=True), c.X)
        oC = fr.run(fr.P(enable=True, ff_enable=True), c.X)
        if replay_cache is not None:
            replay_cache[key] = (c, oB, oC)
    t0 = c.t0                                    # the replay's t0 (first /lidar_slam/odom), bag s
    to_bag = lambda w: (np.asarray(w, np.int64) - off_ns) / 1e9     # noqa: E731  wall ns -> bag s
    rel = lambda w: to_bag(w) - t0               # noqa: E731

    # ---- inputs as published -----------------------------------------------------------------
    cmd_tpub, cmd = P['cmd_tpub'], P['cmd_f']
    lpos_tpub, lp = P['lpos_tpub'], P['lpos_f']
    vcm_tpub, vcm = P['vcm_tpub'], P['vcm_f']
    yt_tpub = P['yt_tpub']
    # the bridge's conversion, with the bridge's own arithmetic (math, float32 on the wire)
    n_c = len(cmd)
    c_ok = np.array([all(math.isfinite(v) for v in (r[0], r[1], r[2], r[12], r[13])) for r in cmd], bool)
    c_pos = np.array([[r[0], -r[1], -r[2]] for r in cmd], np.float32).reshape(-1, 3)
    c_vel = np.array([[r[3], -r[4], -r[5]] if all(map(math.isfinite, r[3:6])) else [np.nan] * 3
                      for r in cmd], np.float32).reshape(-1, 3)
    c_acc = np.array([[r[6], -r[7], -r[8]] if all(map(math.isfinite, r[6:9])) else [np.nan] * 3
                      for r in cmd], np.float32).reshape(-1, 3)
    c_jerk = np.array([[r[9], -r[10], -r[11]] if all(map(math.isfinite, r[9:12])) else [np.nan] * 3
                       for r in cmd], np.float32).reshape(-1, 3)
    c_yaw = np.array([_wrap(-r[12]) for r in cmd], np.float32)
    c_ys = np.array([-r[13] for r in cmd], np.float32)
    l_ok = np.array([bool(r[4]) and bool(r[5]) and all(map(math.isfinite, r[0:4])) for r in lp], bool)
    l_pos = np.array(lp[:, 0:3], np.float32)
    l_yaw = np.array([_wrap(r[3]) if math.isfinite(r[3]) else math.nan for r in lp], np.float32)
    NAN3 = _u32([np.nan] * 3)

    # mode timeline as published: (t_pub, armed, offboard) at changes
    edges = []
    prev = None
    for t, (a, o) in zip(vcm_tpub, vcm):
        s = (bool(a), bool(o))
        if s != prev:
            edges.append((int(t), s[0], s[1]))
            prev = s
    on_t = [t for t, a, o in edges if o]
    t_on = next(t for t, a, o in edges if o)
    t_off = next((t for t, a, o in edges if t > t_on and not o), None)
    eng_on = next((t for t, a, o in edges if a and o), None)
    say(f'\n##### {S["run"]}   (times: bag s since the replay\'s t0 = first /lidar_slam/odom)')
    say(f'mode reports as published: ' + '; '.join(f'{rel(t):.3f} armed={a} offboard={o}' for t, a, o in edges))
    say(f'player: lateness ms p50 {meta["player_late_ms"]["p50"]:.3f} p99 {meta["player_late_ms"]["p99"]:.3f} '
        f'max {meta["player_late_ms"]["max"]:.3f}; discovery {meta["discovery"]["inputs_matched"]}; '
        f'input QoS compatible {all(x[1] for v in meta["input_qos_ok"].values() for x in v)}')

    def offboard_at(t_ns):
        """(armed, offboard) of the last report published at or before t_ns."""
        k = np.searchsorted(vcm_tpub, t_ns, 'right') - 1
        return (False, False) if k < 0 else (bool(vcm[k][0]), bool(vcm[k][1]))

    sp = {v: Sp(rec, v) for v in variants}
    ref_classes = {}

    def classify(s, with_yaw):
        """Per setpoint: (kind, src index, k back) with kind TRACK / HOLD / LATCH / NONE, matched on
        the exact float32 bytes of position+velocity+acceleration+jerk (+ yaw, yawspeed)."""
        kinds, srcs, backs = [], [], []
        pos_u, vel_u, acc_u, jerk_u = _u32(s.pos), _u32(s.vel), _u32(s.acc), _u32(s.jerk)
        yaw_u, ys_u = _u32(s.yaw), _u32(s.ys)
        cp, cv, ca, cj = _u32(c_pos), _u32(c_vel), _u32(c_acc), _u32(c_jerk)
        cy, cs = _u32(c_yaw), _u32(c_ys)
        lpu, lyu = _u32(l_pos), _u32(l_yaw)
        zero = _u32([0.0])[0]
        for i in range(s.n):
            t = s.ts[i]
            kind, src, back = 'NONE', -1, -1
            kc = int(np.searchsorted(cmd_tpub, t, 'right')) - 1
            for b in range(0, 8):
                j = kc - b
                if j < 0:
                    break
                if not c_ok[j] or not np.array_equal(pos_u[i], cp[j]):
                    continue
                if (np.array_equal(vel_u[i], cv[j]) and np.array_equal(acc_u[i], ca[j])
                        and np.array_equal(jerk_u[i], cj[j])
                        and (not with_yaw or (yaw_u[i] == cy[j] and ys_u[i] == cs[j]))):
                    kind, src, back = 'TRACK', j, b
                    break
                if (np.array_equal(vel_u[i], NAN3) and np.array_equal(acc_u[i], NAN3)
                        and np.array_equal(jerk_u[i], NAN3)
                        and (not with_yaw or (yaw_u[i] == cy[j] and ys_u[i] == zero))):
                    kind, src, back = 'HOLD', j, b
                    break
            if kind == 'NONE':
                kl = int(np.searchsorted(lpos_tpub, t, 'right')) - 1
                for b in range(0, 8):
                    j = kl - b
                    if j < 0:
                        break
                    if (l_ok[j] and np.array_equal(pos_u[i], lpu[j]) and np.array_equal(vel_u[i], NAN3)
                            and np.array_equal(acc_u[i], NAN3) and np.array_equal(jerk_u[i], NAN3)
                            and (not with_yaw or (yaw_u[i] == lyu[j] and ys_u[i] == zero))):
                        kind, src, back = 'LATCH', j, b
                        break
            kinds.append(kind)
            srcs.append(src)
            backs.append(back)
        return np.array(kinds), np.array(srcs), np.array(backs)

    # the replay's per-tick outputs, on its 50 Hz grid in bag seconds
    ticks = c.ticks
    ia, ib = c.ia, c.ib
    rep = {'fix1': oB, 'fix12': oC}
    rep_yaw = {k: np.array([x.yaw for x in o]) for k, o in rep.items()}
    rep_ys = {k: np.array([x.yawspeed for x in o]) for k, o in rep.items()}
    rep_src = {k: np.array([x.source for x in o]) for k, o in rep.items()}
    rep_sim = {k: fr.model_of(c, rep_yaw[k], rep_ys[k]) for k in rep}
    rep_sim['off'] = fr.model_of(c, c.psi, c.psid)

    def score(sim):
        e = [math.degrees(fr.wrap(s_['bp'] + sim[s_['i']])) for s_ in c.samples]
        return fr.p3(e), e

    S['replay'] = {k: score(rep_sim[k])[0] for k in rep_sim}
    S['samples'] = len(c.samples)

    for v in variants:
        s = sp[v]
        servo = bool(VARIANTS[v])
        B = S['bridges'][v] = {}
        say(f'\n--- bridge {v} ({"switch off" if not servo else "servo" + (" + ff" if "ff" in " ".join(VARIANTS[v]) else "")})'
            f': {s.n} setpoints')
        if s.n == 0:
            check('0', False, 'no setpoints recorded')
            continue
        # ---------- debug, paired to its tick's setpoint (same tick: yaw/yawspeed bits equal)
        dk = f'dbg_{v}'
        dbg_ts = None
        if servo and dk + '__recv' in rec:
            drecv, dd = rec[dk + '__recv'], rec[dk + '__data']
            o = np.argsort(drecv, kind='stable')
            drecv, dd = drecv[o], dd[o]
            orr = np.argsort(s.recv, kind='stable')
            srecv = s.recv[orr]
            dbg_ts = np.zeros(len(dd), np.int64)
            sp_idx = np.full(len(dd), -1)
            yb, sb = _u32(s.yaw), _u32(s.ys)
            db2, db3 = _u32(dd[:, 2]), _u32(dd[:, 3])
            unpaired = 0
            for j in range(len(dd)):
                q = int(np.searchsorted(srecv, drecv[j]))
                best = None
                for r_ in range(max(0, q - 4), min(len(srecv), q + 5)):
                    i = orr[r_]
                    if yb[i] == db2[j] and sb[i] == db3[j] and abs(int(s.recv[i]) - int(drecv[j])) < 15_000_000:
                        if best is None or abs(int(s.recv[i]) - int(drecv[j])) < abs(int(s.recv[best]) - int(drecv[j])):
                            best = i
                if best is None:
                    unpaired += 1
                else:
                    sp_idx[j] = best
                    dbg_ts[j] = s.ts[best]
            dup = len(sp_idx[sp_idx >= 0]) - len(set(sp_idx[sp_idx >= 0].tolist()))
            say(f'  debug: {len(dd)} messages, paired to their tick\'s setpoint by (yaw, yawspeed) bits and '
                f'receive time: {len(dd) - unpaired} paired, {unpaired} unpaired, {dup} setpoints paired twice')
            B['debug_n'], B['debug_unpaired'] = len(dd), unpaired
            o = np.argsort(dbg_ts, kind='stable')
            dd, dbg_ts, drecv, sp_idx = dd[o], dbg_ts[o], drecv[o], sp_idx[o]
            src_of_sp = np.full(s.n, -1.0)
            src_of_sp[sp_idx[sp_idx >= 0]] = dd[sp_idx >= 0, 6]
        # engaged ticks: servo -> has a debug; off -> by the published mode reports
        eng_mask = np.array([all(offboard_at(t)) for t in s.ts])
        # ---------- check 1 / 2: classification against the inputs --------------------------
        kinds, srcs, backs = classify(s, with_yaw=not servo)
        ref_classes[v] = (kinds, srcs)
        nk = {k: int(np.sum(kinds == k)) for k in ('TRACK', 'HOLD', 'LATCH', 'NONE')}
        bk = np.bincount(backs[backs >= 0], minlength=3)
        in_off = np.array([offboard_at(t)[1] for t in s.ts])
        # outside Offboard: a setpoint whose tick is after an Offboard-off report was published
        # (allowance: the report's delivery, the bridge acts on the next tick) or before any
        # Offboard-on report
        lat_after_off = []
        n_out = 0
        for i in np.flatnonzero(~in_off):
            t = s.ts[i]
            k = np.searchsorted(vcm_tpub, t, 'right') - 1
            lat_after_off.append((t - vcm_tpub[k]) / 1e6 if k >= 0 else math.inf)
            n_out += 1
        allowance_ms = 20.0
        n_viol_out = sum(1 for x in lat_after_off if x > allowance_ms)
        first_after_on = [(s.ts[np.searchsorted(s.ts, t)] - t) / 1e6 for t in on_t
                          if np.searchsorted(s.ts, t) < s.n]
        if not servo:
            B['c1'] = check('1', nk['NONE'] == 0,
                            f'switch off: every setpoint\'s yaw and yawspeed (and all other fields) are exactly a '
                            f'SUPER command\'s (-yaw wrapped, -yaw_dot) or the HOLD/latch value: TRACK {nk["TRACK"]}, '
                            f'HOLD {nk["HOLD"]}, LATCH {nk["LATCH"]}, unmatched {nk["NONE"]} (source = the latest '
                            f'input published before the tick on {bk[0] if len(bk) else 0}, one before on '
                            f'{bk[1] if len(bk) > 1 else 0}, older {int(np.sum(backs >= 2))})')
        else:
            B['c2a'] = check('2a', nk['NONE'] == 0,
                             f'switch on: position, velocity, acceleration, jerk are exactly the conversion of the '
                             f'input they came from (TRACK: the pos_cmd; HOLD: its position, the rest NaN; LATCH: '
                             f'lpos): TRACK {nk["TRACK"]}, HOLD {nk["HOLD"]}, LATCH {nk["LATCH"]}, unmatched '
                             f'{nk["NONE"]}')
        B['classes'] = nk
        B['c1b'] = check('1b' if not servo else '2c', n_viol_out == 0,
                         f'setpoints only in Offboard: {int(np.sum(in_off))} with the last published report saying '
                         f'Offboard, {n_out} not ({n_viol_out} more than {allowance_ms:.0f} ms after an Offboard-off '
                         f'report or before any Offboard report; the {n_out} are '
                         f'{", ".join(f"{x:.2f}" for x in lat_after_off[:6])} ms after the report); first setpoint after '
                         f'each Offboard-on report: {", ".join(f"{x:.1f} ms" for x in first_after_on)}')
        B['n_out_offboard'] = n_out
        if servo and 'off' in ref_classes:
            k0, s0 = ref_classes['off']
            so = sp['off']
            # same input, same kind -> the same bytes of position/velocity/acceleration/jerk
            ref = {}
            for i in range(so.n):
                if k0[i] != 'NONE':
                    ref.setdefault((k0[i], int(s0[i])), (_u32(so.pos[i]).tobytes() + _u32(so.vel[i]).tobytes()
                                                          + _u32(so.acc[i]).tobytes() + _u32(so.jerk[i]).tobytes()))
            n_cmp = n_diff = 0
            for i in range(s.n):
                kk = (kinds[i], int(srcs[i]))
                if kk in ref:
                    n_cmp += 1
                    b = (_u32(s.pos[i]).tobytes() + _u32(s.vel[i]).tobytes() + _u32(s.acc[i]).tobytes()
                         + _u32(s.jerk[i]).tobytes())
                    n_diff += b != ref[kk]
            yaw_diff = 0
            # how many of this bridge's setpoints carry a yaw other than the off bridge's for the same input
            yref = {}
            for i in range(so.n):
                if k0[i] != 'NONE':
                    yref[(k0[i], int(s0[i]))] = (_u32(so.yaw[i])[()], _u32(so.ys[i])[()])
            for i in range(s.n):
                kk = (kinds[i], int(srcs[i]))
                if kk in yref and (_u32(s.yaw[i])[()], _u32(s.ys[i])[()]) != yref[kk]:
                    yaw_diff += 1
            B['c2b'] = check('2b', n_diff == 0 and n_cmp > 0,
                             f'same input as the switch-off bridge ({n_cmp} setpoints whose (kind, source input) the '
                             f'off bridge also sent): position/velocity/acceleration/jerk bytes differ on {n_diff}; '
                             f'yaw or yawspeed differ on {yaw_diff} ({100 * yaw_diff / max(1, n_cmp):.1f} %)')
        # ---------- check 3: slew per pair, over the engaged time -----------------------------
        e_idx = np.flatnonzero(eng_mask) if not servo else np.array(sorted(set(sp_idx[sp_idx >= 0].tolist())))
        # consecutive = no bridge tick (heartbeat) between the two setpoints: across a takeover and
        # re-engage the setpoint array is contiguous but the servo was reset in between
        ots_all = np.sort(rec[f'ocm_{v}__ts']) * 1000
        k_of = np.searchsorted(ots_all, s.ts, 'left')
        adj = np.zeros(s.n, bool)
        if s.n > 1:
            adj[:-1] = (k_of[1:] - k_of[:-1]) <= 1
        worst, worst_at, nviol, npair = 0.0, None, 0, 0
        worst_excess = -math.inf
        for a, b in zip(e_idx, e_idx[1:]):
            if b != a + 1 or not adj[a]:
                continue
            dt = (s.ts[b] - s.ts[a]) / 1e9
            dy = abs(_wrap(float(s.yaw[b]) - float(s.yaw[a])))
            npair += 1
            if dt <= 0:
                nviol += 1
                continue
            if dy / dt > worst:
                worst, worst_at = dy / dt, b
            ex = dy - (MAX_RATE * dt + SLEW_TOL)
            worst_excess = max(worst_excess, ex)
            nviol += ex > 0
        txt = (f'|dyaw| <= 90 deg/s x pair dt + 0.001 deg over {npair} consecutive engaged pairs: '
               f'{nviol} violations; worst {math.degrees(worst):.4f} deg/s at t {rel(s.ts[worst_at]):.2f} s '
               f'(margin {math.degrees(-worst_excess):.5f} deg)' if worst_at is not None else 'no pairs')
        if servo:
            B['c3'] = check('3', nviol == 0 and npair > 0, txt)
        else:
            say(f'  [info] 3 (switch off, SUPER\'s own heading, not limited): {txt}')
        B['slew_worst_dps'] = math.degrees(worst)
        # ---------- checks 4, 5 ------------------------------------------------------------------
        fin = np.isfinite(s.yaw) & np.isfinite(s.ys)
        yaw_in = (s.yaw >= -F32_PI) & (s.yaw <= F32_PI)
        n_negpi = int(np.sum(s.yaw == -F32_PI))
        B['c5'] = check('5', bool(fin.all() and yaw_in.all()),
                        f'yaw/yawspeed finite on {int(fin.sum())}/{s.n}; yaw within [-f32(pi), f32(pi)] on '
                        f'{int(yaw_in.sum())}/{s.n} (min {math.degrees(np.nanmin(s.yaw)):.3f}, max '
                        f'{math.degrees(np.nanmax(s.yaw)):.3f} deg; exactly -f32(pi) {n_negpi})')
        if servo:
            ff_on = 'yaw_servo_ff_enable:=true' in VARIANTS[v]
            ys_max = float(np.max(np.abs(s.ys[e_idx]))) if len(e_idx) else 0.0
            cond = ys_max <= MAX_RATE + 1e-6
            # per debug tick
            src = dd[:, 6]
            act = np.isin(src, (1.0, 2.0, 3.0))
            nz_act = int(np.sum(act & (dd[:, 3] != 0.0)))
            txt = f'|yawspeed| max {math.degrees(ys_max):.3f} deg/s over the engaged ticks'
            if not ff_on:
                cond = cond and nz_act == 0
                txt += f'; ff off and servo active (source 1/2/3, {int(act.sum())} ticks): non-zero on {nz_act}'
            else:
                # ff slew while source 1 on both ticks of a pair
                worst_a, nva, na = 0.0, 0, 0
                s2 = nz2 = 0
                for j in range(1, len(dd)):
                    if sp_idx[j] < 0 or sp_idx[j - 1] < 0 or sp_idx[j] != sp_idx[j - 1] + 1 or not adj[sp_idx[j - 1]]:
                        continue
                    dt = (dbg_ts[j] - dbg_ts[j - 1]) / 1e9
                    if src[j] == 1.0 and src[j - 1] == 1.0:
                        na += 1
                        da = abs(float(dd[j, 3]) - float(dd[j - 1, 3]))
                        worst_a = max(worst_a, da / dt if dt > 0 else math.inf)
                        nva += da > FF_ACC * dt + FF_TOL
                    if src[j] == 2.0:
                        s2 += 1
                        nz2 += dd[j, 3] != 0.0
                cond = cond and nva == 0 and nz2 == 0
                txt += (f'; ff slew over {na} source-1 pairs: max {math.degrees(worst_a):.3f} deg/s^2, '
                        f'{nva} over 90 deg/s^2 x dt + 0.001 deg/s; source 2 ticks {s2}, non-zero yawspeed on {nz2}')
            B['c4'] = check('4', cond, txt)
        # ---------- check 6: the sign -------------------------------------------------------------
        if servo:
            n_ok = n_all = n_step = 0
            exc = []
            for s_ in c.samples:
                t_b = c.ticks[s_['i']]      # the sample's tick on the replay grid; the sample itself:
                t_s = s_['t'] + t0          # bag s
                w = t_s * 1e9 + off_ns
                i = int(np.searchsorted(s.ts, w, 'right')) - 1
                if i < 0 or not eng_mask[i]:
                    continue
                j = int(np.searchsorted(dbg_ts, s.ts[i], 'left'))
                if j >= len(dbg_ts) or dbg_ts[j] != s.ts[i] or dd[j, 6] != 1.0 or abs(s_['M']) <= 5.0:
                    continue
                kl = int(np.searchsorted(lpos_tpub, s.ts[i], 'right')) - 1
                hd = lp[kl, 3]
                delta = _wrap(float(s.yaw[i]) - hd)
                n_all += 1
                good = math.copysign(1, delta) == -math.copysign(1, s_['M']) and delta != 0.0
                n_ok += good
                if not good:
                    ky = int(np.searchsorted(yt_tpub, s.ts[i], 'right')) - 1
                    tx, ty = P['yt_f'][ky]
                    # the aim point's side (FLU, from lpos converted to FLU): bearing - heading
                    bt = math.atan2(ty - (-lp[kl, 1]), tx - lp[kl, 0])
                    aim_side = math.degrees(_wrap(bt - (-hd)))
                    step = _wrap(dd[j, 1] - dd[j, 2])
                    toward = math.copysign(1, step) == -math.copysign(1, s_['M'])
                    n_step += toward
                    exc.append(f'servo step psi_des-psi_out {math.degrees(step):+.1f} deg ('
                               f'{"toward" if toward else "AWAY FROM"} the person); '
                               f't {s_["t"]:.2f} s: person {s_["M"]:+.1f} deg (FLU, + = left), heading FRD '
                               f'{math.degrees(hd):+.1f}, yaw {math.degrees(float(s.yaw[i])):+.1f} (yaw-heading '
                               f'{math.degrees(delta):+.1f}), psi_des {math.degrees(dd[j, 1]):+.1f}, aim point '
                               f'{aim_side:+.1f} deg off the nose (FLU), |psi_des-psi_out| '
                               f'{abs(math.degrees(_wrap(dd[j, 1] - dd[j, 2]))):.1f}, range {dd[j, 4]:.2f} m, target '
                               f'age {dd[j, 5]:.3f} s, lock [{s_["lock"][:22]}]')
            B['c6'] = check('6', n_all > 0 and n_ok == n_all,
                            f'sign: person > 5 deg off the nose and source 1: commanded yaw on the person\'s side of '
                            f'the heading on {n_ok}/{n_all}' + ('' if n_ok == n_all else f' ({n_all - n_ok} exceptions, below; in {n_step} of them the '
                                   f'servo\'s own step psi_des - psi_out points toward the person: the heading compared '
                                   f'against is the one flown, not the one this servo commanded (open loop))'))
            B['sign'] = (n_ok, n_all)
            for e in exc:
                say(f'      exception: {e}')
        # ---------- check 7: against the replay -------------------------------------------------
        tb_node = to_bag(s.ts)
        if servo:
            ry, rys, rsrc, ro = rep_yaw[v], rep_ys[v], rep_src[v], rep[v]
        else:
            ry, rys, rsrc, ro = np.asarray(c.psi, float), np.asarray(c.psid, float), None, None
        W = np.arange(ia, ib)
        # only replay ticks inside the node's own engaged stream
        eng_ts = s.ts[e_idx] if len(e_idx) else s.ts[:0]
        lo, hi = (to_bag(eng_ts[0]), to_bag(eng_ts[-1])) if len(eng_ts) else (0, 0)
        W = W[(ticks[W] >= lo) & (ticks[W] <= hi)]
        uy = np.unwrap(s.yaw.astype(float))
        yi = np.interp(ticks[W], tb_node, uy)
        dyi = np.degrees(np.abs([fr.wrap(a - b) for a, b in zip(yi, ry[W])]))
        kz = np.searchsorted(tb_node, ticks[W], 'right') - 1
        dyz = np.degrees(np.abs([fr.wrap(float(s.yaw[k]) - b) for k, b in zip(kz, ry[W])]))
        dsz = np.degrees(np.abs(s.ys[kz] - rys[W]))
        txt = (f'agreement with the replay on its {len(W)} engaged 50 Hz ticks: |yaw node(interp) - replay| deg p50 '
               f'{pct(dyi, 50):.3f} p95 {pct(dyi, 95):.3f} max {np.max(dyi):.3f}; zero-order hold p50 {pct(dyz, 50):.3f} '
               f'p95 {pct(dyz, 95):.3f} max {np.max(dyz):.3f}; |yawspeed| deg/s p95 {pct(dsz, 95):.3f} max {np.max(dsz):.3f}')
        B['agree'] = dict(n=len(W), yaw_p50=pct(dyi, 50), yaw_p95=pct(dyi, 95), yaw_max=float(np.max(dyi)),
                          zoh_p95=pct(dyz, 95), zoh_max=float(np.max(dyz)), ys_p95=pct(dsz, 95),
                          ys_max=float(np.max(dsz)))
        if servo:
            kd = np.searchsorted(dbg_ts, (ticks[W] * 1e9 + off_ns).astype(np.int64), 'right') - 1
            nsrc = dd[kd, 6]
            agree = float(np.mean(nsrc == rsrc[W]))
            B['agree']['src'] = agree
            txt += f'; source equal on {100 * agree:.2f} % of the ticks'
        say(f'  [info] 7 {txt}')
        # the largest differences, with context
        dsi = np.degrees(np.abs(np.interp(ticks[W], tb_node, s.ys.astype(float)) - rys[W]))
        B['agree']['ys_interp_p95'], B['agree']['ys_interp_max'] = pct(dsi, 95), float(np.max(dsi))
        say(f'      |yawspeed node(interp) - replay| deg/s p50 {pct(dsi, 50):.3f} p95 {pct(dsi, 95):.3f} max '
            f'{np.max(dsi):.3f}; the largest yaw and yawspeed differences:')
        for q in list(np.argsort(-dyi)[:3]) + list(np.argsort(-dsz)[:2]):
            i = W[q]
            k = kz[q]
            ctx = ''
            if servo:
                ctx = (f' replay src {rsrc[i]} slew {"BIND" if ro[i].slew_binding else "-"} age {ro[i].target_age_s:.3f}'
                       f' psi_des {math.degrees(ro[i].psi_des):+.2f}; node src {int(nsrc[q])} age {dd[kd[q], 5]:.3f} '
                       f'psi_des {math.degrees(dd[kd[q], 1]):+.2f}')
            ctx += (f'; yawspeed replay {math.degrees(rys[i]):+.2f} node {math.degrees(float(s.ys[k])):+.2f} deg/s')
            say(f'      t {ticks[i] - t0:.3f}: replay {math.degrees(ry[i]):+.3f} node interp '
                f'{math.degrees(fr.wrap(yi[q])):+.3f} (last node tick {1e3 * (ticks[i] - tb_node[k]):.1f} ms before, '
                f'{math.degrees(float(s.yaw[k])):+.3f}; next {math.degrees(float(s.yaw[min(k + 1, s.n - 1)])):+.3f})' + ctx)
        # the node's commands through the replay's fitted PX4 model
        cmdz = np.array(ry, float)
        ysz = np.array(rys, float)
        Wall = np.arange(ia, ib)
        kk = np.searchsorted(tb_node, ticks[Wall], 'right') - 1
        good = (kk >= 0) & np.array([eng_mask[k] if k >= 0 else False for k in kk])
        cmdz[Wall[good]] = s.yaw[kk[good]]
        ysz[Wall[good]] = s.ys[kk[good]]
        sim = fr.model_of(c, cmdz, ysz)
        pn, en = score(sim)
        pr = S['replay'][v]
        B['model'] = dict(node=pn, replay=pr, filled=int(np.sum(~good)))
        dp95 = pn[1] - pr[1]
        B['c7'] = check('7', abs(dp95) <= 0.5,
                        f'person off-centre through the replay\'s fitted PX4 yaw model ({len(c.samples)} samples): '
                        f'node p50/p95/max {pn[0]:.2f} / {pn[1]:.2f} / {pn[2]:.2f} deg vs the replay\'s module '
                        f'{pr[0]:.2f} / {pr[1]:.2f} / {pr[2]:.2f} (p95 difference {dp95:+.2f}, limit 0.5; '
                        f'{int(np.sum(~good))} replay ticks before the node\'s first engaged setpoint used the replay\'s value)')
        # ---------- check 8: tick ---------------------------------------------------------------
        de = np.diff(s.ts[e_idx]) / 1e6 if len(e_idx) > 1 else np.zeros(0)
        de = de[np.diff(e_idx) == 1] if len(e_idx) > 1 else de
        ok_ = f'ocm_{v}'
        ots = np.sort(rec[ok_ + '__ts']) * 1000
        orc = np.sort(rec[ok_ + '__recv'])
        g_ts = np.diff(ots) / 1e6
        g_rc = np.diff(orc) / 1e6
        rate = (len(ots) - 1) / ((ots[-1] - ots[0]) / 1e9)
        B['tick'] = dict(p50=pct(de, 50), p99=pct(de, 99), max=float(de.max()) if len(de) else math.nan,
                         ocm_rate=rate, ocm_gap_ts=float(g_ts.max()), ocm_gap_recv=float(g_rc.max()),
                         ocm_n=len(ots))
        B['c8'] = check('8', g_ts.max() < 1000.0 and abs(rate - 50.0) < 1.0,
                        f'tick: engaged setpoint intervals ms p50 {pct(de, 50):.3f} p99 {pct(de, 99):.3f} max '
                        f'{de.max() if len(de) else math.nan:.3f}; offboard_control_mode {len(ots)} over the run, '
                        f'{rate:.3f} Hz, longest gap {g_ts.max():.3f} ms by its stamps / {g_rc.max():.3f} ms by '
                        f'receive time (PX4 drops Offboard after 1000 ms)')
        # ---------- check 9: source timeline and the log ---------------------------------------
        logl = parse_log(os.path.join(rundir, f'bridge_{v}.log'), off_ns, int(t0 * 1e9))
        if servo:
            src = dd[:, 6]
            dts = np.diff(dbg_ts) / 1e9
            secs = {k: float(np.sum(np.where(src[1:] == k, np.clip(dts, 0, 0.1), 0.0)))
                    + (0.02 if src[0] == k else 0.0) for k in (0.0, 1.0, 2.0, 3.0)}
            B['secs'] = secs
            say(f'  [info] 9 seconds per source (by tick time): fall-back/SUPER {secs[0.0]:.2f}, servo '
                f'{secs[1.0]:.2f}, close-range hold {secs[2.0]:.2f}, stale hold {secs[3.0]:.2f}; replay '
                + ', '.join(f'{int(k)}: {0.02 * np.sum(rsrc[ia:ib] == k):.2f}' for k in (0, 1, 2, 3)))
            ev = [ln for ln in logl if 'yaw servo' in ln[2] and ('ACTIVE' in ln[2] or 'INACTIVE' in ln[2])]
            ntr = 0
            for j in range(1, len(src)):
                if src[j] != src[j - 1]:
                    ntr += 1
                    tr = rel(dbg_ts[j])
                    why = ''
                    if src[j] == 0.0:
                        why = (f'age {dd[j, 5]:.3f} s' if dd[j, 5] > 0.5 else f'range {dd[j, 4]:.2f} age {dd[j, 5]:.3f}')
                    elif src[j] == 3.0:
                        why = f'no usable target: age {dd[j, 5]:.3f} s'
                    elif src[j] == 2.0:
                        why = f'range {dd[j, 4]:.2f} m'
                    elif src[j] == 1.0:
                        why = f'range {dd[j, 4]:.2f} m, age {dd[j, 5]:.3f} s'
                    lg = [x for x in ev if x[1] is not None and abs(x[1] - tr) < 0.03]
                    say(f'      t {tr:8.3f}  source {int(src[j - 1])} -> {int(src[j])}  ({why})'
                        + (f'  log: "{lg[0][2][:120]}"' if lg else ''))
            B['transitions'] = ntr
        sev = {}
        for sv, t, txt in logl:
            if sv in ('WARN', 'ERROR', 'FATAL', 'STDERR') or 'Traceback' in txt:
                sev.setdefault((sv, norm(txt)), []).append((t, txt))
        B['log_anomalies'] = {f'{a} {b}': len(x) for (a, b), x in sev.items()}
        say(f'  [info] 9 log: {len(logl)} lines; WARN/ERROR/FATAL/stderr, distinct: {len(sev)}')
        for (sv, nt), x in sev.items():
            tt = [f'{y[0]:.2f}' for y in x[:6] if y[0] is not None]
            say(f'      {len(x)} x {sv}: "{x[0][1][:170]}"' + (f' (at t {", ".join(tt)})' if tt else ''))
        # ---------- check 10: the takeover --------------------------------------------------------
        if t_off is not None:
            last_sp = s.ts[s.ts <= t_off + 1_000_000_000]
            n_after = int(np.sum(s.ts > t_off))
            later_on = [t for t in on_t if t > t_off]
            lim = later_on[0] if later_on else math.inf
            n_after = int(np.sum((s.ts > t_off + int(allowance_ms * 1e6)) & (s.ts < lim)))
            lst = s.ts[s.ts <= t_off + int(allowance_ms * 1e6)][-1]
            txt = (f'takeover (Offboard-off report at t {rel(t_off):.3f}): last setpoint {(lst - t_off) / 1e6:+.2f} ms '
                   f'from the report, {n_after} setpoints later')
            ok10 = n_after == 0
            if servo:
                dl = drecv[drecv <= t_off + 1_000_000_000]
                n_dafter = int(np.sum((drecv > t_off + int(allowance_ms * 1e6)) & (drecv < lim)))
                lastd = dl[-1] if len(dl) else None
                txt += (f'; last debug received {(lastd - t_off) / 1e6:+.2f} ms from it, {n_dafter} later'
                        if lastd is not None else '; no debug')
                ok10 = ok10 and n_dafter == 0
            B['c10'] = check('10', ok10, txt)
            B['takeover_last_ms'] = (lst - t_off) / 1e6
        # ---------- target age at arrival, as the node saw it (servo) ---------------------------
        if servo:
            age = dd[:, 5]
            st_est = dbg_ts - np.round(age * 1e9).astype(np.int64)
            s1 = P['yt_stamp1']
            tpub = P['yt_tpub']
            # unique stamps (a coast republishes the same stamp) -> first publication of each
            uniq = {}
            for q in range(len(s1)):
                uniq.setdefault(int(s1[q]), q)
            ukeys = np.array(sorted(uniq), np.int64)
            first_tick = {}
            for j in range(len(dd)):
                if not math.isfinite(age[j]):
                    continue
                q = int(np.searchsorted(ukeys, st_est[j]))
                for r_ in (q - 1, q):
                    if 0 <= r_ < len(ukeys) and abs(int(ukeys[r_]) - int(st_est[j])) < 20_000:
                        first_tick.setdefault(int(ukeys[r_]), j)
                        break
            ks = sorted(first_tick)
            fa = np.array([age[first_tick[k]] * 1e3 for k in ks])
            q0 = np.array([uniq[k] for k in ks], int)
            pa = (tpub[q0] - s1[q0]) / 1e6                       # age at its first publication
            # the wait: that tick's time - the LATEST publication of that stamp before it (a coast
            # republishes a stamp; its first copy may have been overwritten in the depth-1 queue)
            pubs_of = {}
            for q in range(len(s1)):
                pubs_of.setdefault(int(s1[q]), []).append(int(tpub[q]))
            diff = []
            # a stamp already current at the engage is first SEEN on the first engaged tick (the debug
            # starts there), not on the tick after its arrival: leave those out of the wait
            ks = [k for k in ks if max(x for x in pubs_of[k] if x <= dbg_ts[first_tick[k]]) > dbg_ts[0]]
            fa = np.array([age[first_tick[k]] * 1e3 for k in ks])
            for k in ks:
                tt = dbg_ts[first_tick[k]]
                before = [x for x in pubs_of[k] if x <= tt]
                diff.append((tt - max(before)) / 1e6 if before else math.nan)
            diff = np.array(diff)
            q_last = np.array([int(np.flatnonzero((s1 == k) & (tpub <= dbg_ts[first_tick[k]]))[-1]) for k in ks], int)
            rec_same = (P['yt_tbag'][q_last] - P['yt_stamp0'][q_last]) / 1e6
            n_late = int(np.sum(diff > 25.0))
            rec_age = (P['yt_tbag'][q0] - P['yt_stamp0'][q0]) / 1e6
            inwin = (P['yt_tbag'] >= c.ta * 1e9) & (P['yt_tbag'] < c.tb * 1e9)
            rec_win = (P['yt_tbag'][inwin] - P['yt_stamp0'][inwin]) / 1e6
            ra = np.array([x.target_age_s for x in (rep[v][ia:ib])], float) * 1e3
            na = age[np.isfinite(age)] * 1e3
            B['age'] = dict(n=len(ks), node_first_p50=pct(fa, 50), rec_p50=pct(rec_win, 50),
                            wait_min=float(diff.min()), wait_max=float(diff.max()), wait_p50=pct(diff, 50),
                            node_tick_p50=pct(na, 50), replay_tick_p50=pct(ra[np.isfinite(ra)], 50),
                            pub_minus_rec_max=float(np.max(np.abs(pa - rec_age))))
            say(f'  [info] age: recorded recv - stamp of the yaw targets received in the replay\'s Offboard window '
                f'({int(inwin.sum())}): ms p50 {pct(rec_win, 50):.1f} p95 {pct(rec_win, 95):.1f} max {rec_win.max():.1f}. '
                f'Of the {len(ukeys)} distinct stamps played, the node saw {len(ks)} while engaged: age at their first '
                f'publication (player) - recorded recv - stamp: max |d| {np.max(np.abs(pa - rec_age)):.3f} ms; the '
                f'recorded recv - stamp of those same messages p50 {pct(rec_same, 50):.1f} p95 {pct(rec_same, 95):.1f}; the '
                f'node\'s target_age_s on the first tick that had each ({len(ks)}, the one current at the engage left out): '
                f'p50 {pct(fa, 50):.1f} p95 {pct(fa, 95):.1f} max '
                f'{fa.max():.1f} ms, i.e. that tick\'s time - the publication: min {diff.min():.2f} p50 {pct(diff, 50):.2f} '
                f'max {diff.max():.2f} ms, over 25 ms on {n_late} (expected 0-20 + delivery: the wait for the next '
                f'50 Hz tick; measured from the latest copy of that stamp published before the tick). Per engaged tick: node '
                f'p50 {pct(na, 50):.1f} p95 {pct(na, 95):.1f} vs the replay\'s p50 {pct(ra[np.isfinite(ra)], 50):.1f} '
                f'p95 {pct(ra[np.isfinite(ra)], 95):.1f} ms')
        # ---------- inputs the bridge counted (its own 5 s line) -------------------------------
        cin = re.findall(r'^\[INFO\] \[(\d+)\.(\d+)\] \[[^\]]+\]: \w+ cmd_in=(\d+)',
                         open(os.path.join(rundir, f'bridge_{v}.log'), errors='replace').read(), re.M)
        if cin:
            tl = int(cin[-1][0]) * 1_000_000_000 + int(cin[-1][1].ljust(9, '0')[:9])
            n_pub = int(np.sum(cmd_tpub <= tl))
            B['cmd_in'] = (int(cin[-1][2]), n_pub)
            say(f'  [info] the bridge\'s last 5 s line counted {B["cmd_in"][0]} pos_cmd received; {n_pub} had been '
                f'published by that line\'s time ({n_pub - B["cmd_in"][0]} not received)')
    return L, S


# ---------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--role', default='main', help=argparse.SUPPRESS)
    ap.add_argument('--config', help=argparse.SUPPRESS)
    ap.add_argument('--bags', nargs='*', help='bag directories (default: the three in <repo>/bags)')
    ap.add_argument('--variants', default='off,fix1,fix12')
    ap.add_argument('--solo', action='store_true', help='also one play per variant, one bridge alone')
    ap.add_argument('--solo-only', action='store_true', help='only the one-bridge plays')
    ap.add_argument('--out', default=None, help='output directory')
    ap.add_argument('--pre', type=float, default=30.0, help='s of IDLE before the first Offboard report (>= 10)')
    ap.add_argument('--post', type=float, default=35.0, help='s after the Offboard-off report')
    ap.add_argument('--domain', type=int, default=96)
    ap.add_argument('--analyze', nargs='*', help='re-analyse finished run directories')
    a = ap.parse_args()
    if a.role == 'recorder':
        role_recorder(a.config)
        return 0
    out = a.out or os.path.join(os.getcwd(), 'yaw_servo_bag_bench_out')
    os.makedirs(out, exist_ok=True)
    env_setup(a.domain, os.path.join(out, 'roslog'))
    if a.analyze is not None:
        cache = {}
        allS = []
        for r in a.analyze:
            L, S = analyse(r, cache)
            with open(os.path.join(r, 'report.txt'), 'w') as f:
                f.write('\n'.join(L) + '\n')
            allS.append(S)
        with open(os.path.join(out, 'summary.json'), 'w') as f:
            json.dump(allS, f, indent=1, default=str)
        return 0
    if a.pre < 10.0:
        sys.exit('--pre must be >= 10 s (at least 10 s of IDLE before the engage)')
    if not os.path.isfile(BRIDGE):
        sys.exit(f'bridge not found: {BRIDGE}')
    bags = a.bags or [os.path.join(REPO, 'bags', b) for b in BAGS]
    variants = [v for v in a.variants.split(',') if v]
    for v in variants:
        if v not in VARIANTS:
            sys.exit(f'unknown variant {v}')
    bad = isolation_check(a.domain)
    if bad:
        sys.exit('isolation check failed:\n  ' + '\n  '.join(bad))
    print(f'bridge under test: {BRIDGE} -> {os.path.realpath(BRIDGE)}; domain {a.domain}, '
          f'ROS_LOCALHOST_ONLY=1; sandbox {NS}')
    runs = []
    for bag in bags:
        if not a.solo_only:
            runs.append(run_play(bag, variants, out, a.pre, a.post, a.domain))
        if a.solo or a.solo_only:
            for v in variants:
                runs.append(run_play(bag, [v], out, a.pre, a.post, a.domain))
    cache = {}
    allS = []
    for r in runs:
        L, S = analyse(r, cache)
        with open(os.path.join(r, 'report.txt'), 'w') as f:
            f.write('\n'.join(L) + '\n')
        allS.append(S)
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(allS, f, indent=1, default=str)
    left = domain_users(a.domain, _ancestors())
    print(f'\nprocesses left on domain {a.domain}: {left if left else "none"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
