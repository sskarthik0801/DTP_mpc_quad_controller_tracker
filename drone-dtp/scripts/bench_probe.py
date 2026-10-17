#!/usr/bin/env python3
"""Small probes for a bench bring-up of the full graph. Each subcommand prints one line per item.

  bench_probe.py hz  SECS TOPIC:TYPE [TOPIC:TYPE ...]   rates on BEST_EFFORT subscriptions
                                                          (ros2 topic hz cannot see FAST-LIO's odom)
  bench_probe.py lag SECS                                 /lidar_slam/odom stamp lag p50 / max
  bench_probe.py cpu SECS NAME [NAME ...]                 % of one core per process basename (all pids)
  bench_probe.py synth SECS X Y Z                         publish a COASTING /dtp/follow_target at 8 Hz
                                                          at (X, Y, Z) in lidar_init and count the
                                                          /planning/loss_event answers (the live oracle)

TYPE is e.g. nav_msgs/msg/Odometry. Exit code is always 0; read the numbers.
"""
import os
import sys
import time


def _import_type(spec):
    pkg, _, name = spec.split('/')
    mod = __import__('%s.msg' % pkg, fromlist=[name])
    return getattr(mod, name)


def cmd_hz(secs, specs):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('bench_probe_hz')
    counts = {}
    for spec in specs:
        topic, tspec = spec.split(':')
        counts[topic] = 0

        def cb(m, t=topic):
            counts[t] += 1
        node.create_subscription(_import_type(tspec), topic, cb, qos_profile_sensor_data)
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for topic, n in counts.items():
        print('%-40s %6.2f Hz  (%d in %.0f s)' % (topic, n / secs, n, secs))
    node.destroy_node()
    rclpy.shutdown()


def cmd_lag(secs):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('bench_probe_lag')
    lags = []

    def cb(m):
        lags.append(node.get_clock().now().nanoseconds * 1e-9 - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    node.create_subscription(Odometry, '/lidar_slam/odom', cb, qos_profile_sensor_data)
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if lags:
        s = sorted(lags)
        print('odom stamp lag: p50 %.3f s  max %.3f s  (%d samples)' % (s[len(s) // 2], s[-1], len(s)))
    else:
        print('odom stamp lag: NO ODOM')
    node.destroy_node()
    rclpy.shutdown()


def _pids_named(names):
    out = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open('/proc/%s/comm' % d) as f:
                comm = f.read().strip()
            with open('/proc/%s/cmdline' % d, 'rb') as f:
                argv = f.read().split(b'\0')
        except OSError:
            continue
        base = os.path.basename(argv[0].decode(errors='replace')) if argv and argv[0] else comm
        # python nodes: the script name is argv[1]
        if base.startswith('python') and len(argv) > 1:
            base = os.path.basename(argv[1].decode(errors='replace'))
        for n in names:
            if base == n or comm == n or base.startswith(n):
                out.setdefault(n, []).append(int(d))
    return out


def _cpu(pid):
    try:
        with open('/proc/%d/stat' % pid) as f:
            parts = f.read().rsplit(')', 1)[1].split()
        return (int(parts[11]) + int(parts[12])) / os.sysconf('SC_CLK_TCK')
    except (OSError, IndexError, ValueError):
        return None


def cmd_cpu(secs, names):
    pids = _pids_named(names)
    t0 = time.time()
    c0 = {n: sum(_cpu(p) or 0.0 for p in ps) for n, ps in pids.items()}
    time.sleep(secs)
    dt = time.time() - t0
    for n in names:
        if n not in pids:
            print('%-24s not running' % n)
            continue
        c1 = sum(_cpu(p) or 0.0 for p in pids[n])
        rss = 0
        for p in pids[n]:
            try:
                with open('/proc/%d/statm' % p) as f:
                    rss += int(f.read().split()[1]) * os.sysconf('SC_PAGE_SIZE') // (1024 * 1024)
            except OSError:
                pass
        print('%-24s %6.1f %% of a core   RSS %5d MB   pids %s' % (n, 100.0 * (c1 - c0[n]) / dt, rss, pids[n]))


def cmd_synth(secs, x, y, z):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Point
    from dtp_msgs.msg import Target, LossEvent
    rclpy.init()
    node = rclpy.create_node('bench_probe_synth')
    pub = node.create_publisher(Target, '/dtp/follow_target', qos_profile_sensor_data)
    events = []
    node.create_subscription(LossEvent, '/planning/loss_event', lambda m: events.append(m), qos_profile_sensor_data)
    n = 0
    end = time.time() + secs
    nxt = time.time()
    while time.time() < end:
        if time.time() >= nxt:
            n += 1
            m = Target()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'lidar_init'
            m.track_id = 9999
            m.position.x, m.position.y, m.position.z = x, y, z
            m.velocity.x = 0.5
            m.predicted_path = [Point(x=x + 0.5 * (0.2 + 0.075 * i), y=y, z=z) for i in range(10)]
            m.predicted_path_dt = [0.2 + 0.075 * i for i in range(10)]
            m.track_confirmed = True
            m.missed_frames = n
            m.confidence = 0.5
            m.is_locked_target = True
            pub.publish(m)
            nxt += 1.0 / 8.0
        rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.3)
    rclpy.spin_once(node, timeout_sec=0.2)
    cls = ['OCCLUDED', 'UNMAPPED', 'LOST', 'OUT_OF_VIEW']
    counts = {c: sum(1 for e in events if e.cls == i) for i, c in enumerate(cls)}
    ev = sorted(e.eval_us for e in events)
    print('synthetic coasting targets: %d published, %d LossEvents answered (%s)' % (n, len(events), counts))
    if ev:
        print('  eval_us p50 %d  p99 %d  max %d; last: blocked %.2f unknown %.2f rays %d'
              % (ev[len(ev) // 2], ev[min(len(ev) - 1, int(0.99 * len(ev)))], ev[-1],
                 events[-1].blocked_fraction, events[-1].unknown_fraction, events[-1].rays_cast))
    node.destroy_node()
    rclpy.shutdown()


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return 0
    if a[0] == 'hz':
        cmd_hz(float(a[1]), a[2:])
    elif a[0] == 'lag':
        cmd_lag(float(a[1]))
    elif a[0] == 'cpu':
        cmd_cpu(float(a[1]), a[2:])
    elif a[0] == 'synth':
        cmd_synth(float(a[1]), float(a[2]), float(a[3]), float(a[4]))
    else:
        print(__doc__)
    return 0


if __name__ == '__main__':
    sys.exit(main())
