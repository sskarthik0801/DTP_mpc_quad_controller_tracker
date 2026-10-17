#!/usr/bin/env python3
"""Characterise /lidar_slam/odom and /tf timing against the colour stamp, under real load.

WHY: perception_node lifts a detection into the world by looking up world <- camera AT THE FRAME
STAMP, and waits (up to max_pair_age_s) for odom to cover that stamp. On 2026-09-11 ~24 % of
frames aged out and published blind against Kevin's full navigation stack, while 0 % did against
a bare FAST-LIO. Three different causes would produce that, and they need different fixes:

  (a) a STEADY stamp-base offset (camera stamps ahead of odom)    -> align the clocks / interpolate
  (b) odom STALLS longer than max_pair_age_s                       -> nothing in our node can fix it
  (c) /tf published later than the odom topic carrying the same pose -> read odom, not /tf

This measures all three on the live graph and prints the distributions. Read-only.
"""
import argparse
import statistics
import sys
import time


def pct(v, q):
    if not v:
        return float('nan')
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    n = Node('odom_lag_probe')

    odom_arrival = []      # wall time each odom msg arrived
    odom_stamp = []        # its header stamp
    tf_arrival = []
    tf_stamp = []
    cam = []               # (arrival, stamp) for colour camera_info

    def now():
        return n.get_clock().now().nanoseconds * 1e-9

    def on_odom(m):
        odom_arrival.append(now())
        odom_stamp.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def on_tf(m):
        for t in m.transforms:
            if t.child_frame_id == 'lidar_imu':
                tf_arrival.append(now())
                tf_stamp.append(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)

    def on_cam(m):
        cam.append((now(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))

    n.create_subscription(Odometry, '/lidar_slam/odom', on_odom, qos_profile_sensor_data)
    n.create_subscription(TFMessage, '/tf', on_tf, 10)
    n.create_subscription(CameraInfo, '/camera/color/camera_info', on_cam, qos_profile_sensor_data)

    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

    print()
    print('WINDOW %.0f s   odom msgs %d   tf(lidar_imu) msgs %d   colour msgs %d'
          % (args.seconds, len(odom_arrival), len(tf_arrival), len(cam)))

    if len(odom_arrival) < 5:
        print('not enough odom to characterise -- is FAST-LIO running?')
        rclpy.shutdown()
        return 1

    gaps = [b - a for a, b in zip(odom_arrival, odom_arrival[1:])]
    print()
    print('(b) ODOM INTER-ARRIVAL GAPS -- a gap > max_pair_age_s ages frames out no matter what')
    print('    mean %.0f ms   p50 %.0f   p90 %.0f   p99 %.0f   MAX %.0f ms'
          % (1000 * statistics.fmean(gaps), 1000 * pct(gaps, .5), 1000 * pct(gaps, .9),
             1000 * pct(gaps, .99), 1000 * max(gaps)))
    over = [g for g in gaps if g > 0.45]
    print('    gaps > 450 ms (the deployed max_pair_age_s): %d of %d  (%.1f %%)'
          % (len(over), len(gaps), 100.0 * len(over) / len(gaps)))

    lag = [a - s for a, s in zip(odom_arrival, odom_stamp)]
    print()
    print('(a) ODOM STAMP -> ARRIVAL (how far behind real time the pose is when we get it)')
    print('    mean %.0f ms   p50 %.0f   p90 %.0f   MAX %.0f ms'
          % (1000 * statistics.fmean(lag), 1000 * pct(lag, .5), 1000 * pct(lag, .9),
             1000 * max(lag)))

    if cam:
        camlag = [a - s for a, s in cam]
        print('    COLOUR stamp -> arrival: mean %.0f ms  p90 %.0f ms'
              % (1000 * statistics.fmean(camlag), 1000 * pct(camlag, .9)))
        print('    => a colour frame is stamped %.0f ms AHEAD of the newest odom STAMP on average'
              % (1000 * (statistics.fmean(lag) - statistics.fmean(camlag))))

    if tf_arrival and len(tf_arrival) > 3:
        tlag = [a - s for a, s in zip(tf_arrival, tf_stamp)]
        print()
        print('(c) /tf(lidar_imu) STAMP -> ARRIVAL, vs odom carrying the same pose')
        print('    tf   mean %.0f ms   p90 %.0f ms   (%d msgs)'
              % (1000 * statistics.fmean(tlag), 1000 * pct(tlag, .9), len(tf_arrival)))
        print('    odom mean %.0f ms   p90 %.0f ms'
              % (1000 * statistics.fmean(lag), 1000 * pct(lag, .9)))
        delta = statistics.fmean(tlag) - statistics.fmean(lag)
        print('    => /tf arrives %+.0f ms relative to the odom topic. Positive means reading the'
              % (1000 * delta))
        print('       odom TOPIC instead of /tf would gain that much, and nothing more.')

    # The decisive question, answered directly: for each colour frame, how long AFTER its stamp
    # did the first odom sample at-or-after that stamp arrive? That is exactly the wait
    # perception_node performs, measured without perception_node.
    waits = []
    never = 0
    for _arr, s in cam:
        hit = None
        for oa, os_ in zip(odom_arrival, odom_stamp):
            if os_ >= s:
                hit = oa - s
                break
        if hit is None:
            never += 1
        else:
            waits.append(hit)
    if waits:
        print()
        print('THE ACTUAL WAIT perception_node performs (stamp -> first odom covering it):')
        print('    mean %.0f ms   p50 %.0f   p90 %.0f   p99 %.0f   MAX %.0f ms'
              % (1000 * statistics.fmean(waits), 1000 * pct(waits, .5), 1000 * pct(waits, .9),
                 1000 * pct(waits, .99), 1000 * max(waits)))
        for cap in (0.25, 0.45):
            bad = [w for w in waits if w > cap]
            print('    would EXCEED max_pair_age_s=%.2f on %d of %d frames (%.1f %%)'
                  % (cap, len(bad), len(waits), 100.0 * len(bad) / len(waits)))
        print('    (%d frames had no covering odom inside the window and are excluded)' % never)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
