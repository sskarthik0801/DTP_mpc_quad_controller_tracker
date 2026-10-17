"""Extract the two 2026-09-26 Offboard bags into a cache pickle (read-only).

Run with ROS sourced (px4_msgs, mars_quadrotor_msgs):
    source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
    python3 extract_bags.py

Never run it on a bag that `ros2 bag record` is still writing (CLAUDE.md: the read locks the
.db3 and kills the recorder). This script refuses if any recorder is running.
"""
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import BAGS, CACHE, cache_file  # noqa: E402


def recorder_running():
    """Match argv tokens exactly (never a flattened command line; CLAUDE.md trap)."""
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        for i, a in enumerate(argv):
            if os.path.basename(a) == 'ros2' and argv[i + 1:i + 3] == ['bag', 'record']:
                return pid
    return None


def extract(bag_dir, out):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    topics = {
        '/fmu/in/trajectory_setpoint': 'sp',
        '/fmu/out/vehicle_local_position_v1': 'lp',
        '/fmu/out/vehicle_control_mode': 'vcm',
        '/fmu/out/vehicle_status_v1': 'vs',
        '/planning/pos_cmd': 'pc',
        '/lidar_slam/odom': 'od',
    }
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    d = {k: [] for k in topics.values()}
    while r.has_next():
        topic, data, ts = r.read_next()
        m = deserialize_message(data, cls[topic])
        t = ts * 1e-9
        k = topics[topic]
        if k == 'sp':      # bag time, bridge stamp, pos xyz, vel xyz, acc xyz, yaw
            d[k].append((t, m.timestamp * 1e-6, *m.position, *m.velocity, *m.acceleration, m.yaw))
        elif k == 'lp':    # bag time, stamp, stamp_sample, xyz, vxyz, axyz, heading, flags, vxy_max
            d[k].append((t, m.timestamp * 1e-6, m.timestamp_sample * 1e-6, m.x, m.y, m.z, m.vx,
                         m.vy, m.vz, m.ax, m.ay, m.az, m.heading, float(m.xy_valid),
                         float(m.v_xy_valid), m.vxy_max, m.dist_bottom))
        elif k == 'vcm':
            d[k].append((t, float(m.flag_armed), float(m.flag_control_offboard_enabled)))
        elif k == 'vs':
            d[k].append((t, m.timestamp * 1e-6, m.nav_state, m.arming_state))
        elif k == 'pc':    # SUPER's command, lidar_init FLU
            d[k].append((t, m.position.x, m.position.y, m.position.z, m.velocity.x, m.velocity.y,
                         m.velocity.z, m.acceleration.x, m.acceleration.y, m.acceleration.z))
        elif k == 'od':    # FAST-LIO, lidar_init FLU
            p = m.pose.pose.position
            d[k].append((t, p.x, p.y, p.z))
    res = {k: np.array(v) for k, v in d.items()}
    res['t0'] = min(v[0, 0] for v in res.values() if len(v))
    with open(out, 'wb') as f:
        pickle.dump(res, f)
    print(out, {k: v.shape for k, v in res.items() if hasattr(v, 'shape')})


if __name__ == '__main__':
    pid = recorder_running()
    if pid:
        sys.exit(f'refusing: ros2 bag record is running (pid {pid})')
    os.makedirs(CACHE, exist_ok=True)
    for name, path in BAGS.items():
        extract(path, cache_file(name))
