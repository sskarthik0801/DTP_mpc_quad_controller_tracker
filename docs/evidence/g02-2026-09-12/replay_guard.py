"""Replay every recorded /lidar_slam/odom through px4_odom_bridge's DivergenceGuard."""
import sqlite3, sys, glob, os, importlib.util, math
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
spec = importlib.util.spec_from_file_location('b', os.path.expanduser('~/SUPER-Human-DTP/super_bringup/scripts/px4_odom_bridge.py'))
b = importlib.util.module_from_spec(spec); spec.loader.exec_module(b)
for db in sorted(glob.glob(os.path.expanduser('~/dtp_datasets/*/*.db3'))):
    con = sqlite3.connect(db)
    tid = con.execute("select id from topics where name='/lidar_slam/odom'").fetchone()
    if not tid: continue
    rows = []
    for ts, data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp', tid):
        m = deserialize_message(data, Odometry); p = m.pose.pose.position; s = m.header.stamp
        rows.append((ts/1e9, s.sec + s.nanosec*1e-9, (p.x, p.y, p.z)))
    if not rows: continue
    t0 = rows[0][0]
    g = b.DivergenceGuard(0.5, 5.0, 1.0, 1.0)
    pubs = {'fastlio-1'}; events = []; last_state = g.state; fwd = 0
    maxv = 0; prev = None
    for arr, st, p in rows:
        if prev and st > prev[0]:
            maxv = max(maxv, math.dist(p, prev[1]) / (st - prev[0]))
        # a >15 s silence followed by a pose near the origin is a FAST-LIO restart in these bags
        if prev and arr - prev[2] > 15 and math.hypot(*p) < 1.0:
            pubs = {'fastlio-%d' % (len(events) + 2)}
        if g.admit(st, p, arr, pubs): fwd += 1
        if g.state != last_state:
            events.append('t=%.1f %s %s' % (arr - t0, g.state, g.reason[:90])); last_state = g.state
        prev = (st, p, arr)
    print('%-28s n=%5d forwarded=%5d max_speed=%5.2f m/s  final=%s' % (os.path.basename(os.path.dirname(db)), len(rows), fwd, maxv, g.state))
    for e in events: print('     ', e)
