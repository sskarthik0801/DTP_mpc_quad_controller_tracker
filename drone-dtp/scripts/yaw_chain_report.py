#!/usr/bin/env python3
"""Where does the camera's aiming error come from? Read-only on FINISHED bags (never one still recording).

At every /dtp/follow_target sample while PX4 is in Offboard and a target is locked:
  e_cam   = person bearing - aircraft heading        (what the camera sees; + = person to the left)
  split exactly into
  e_aim   = person bearing - yaw_target bearing      (the aim point vs the person)
  e_super = yaw_target bearing - pos_cmd yaw         (SUPER's heading command vs the aim point, from HERE)
  e_px4   = pos_cmd yaw - aircraft heading           (PX4 tracking SUPER's heading)
All in lidar_init, z up; bearings from the aircraft's current position; heading from /lidar_slam/odom.
"""
import bisect, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/dtp/follow_target', '/planning/yaw_target', '/planning/pos_cmd', '/lidar_slam/odom',
          '/fmu/out/vehicle_status_v1', '/dtp/lock_state', '/camera/color/camera_info']


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    out = {t: [] for t in TOPICS}
    cam = None
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/camera/color/camera_info':
            if cam is None:
                cam = deserialize_message(data, get_message(types[topic]))
            continue
        out[topic].append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out, cam


class Series:
    def __init__(self, rows):
        self.t = [r[0] for r in rows]
        self.v = [r[1] for r in rows]

    def at(self, t, max_age):
        i = bisect.bisect_right(self.t, t) - 1
        if i < 0 or t - self.t[i] > max_age:
            return None
        return self.v[i]


def pct(x, ps=(50, 90, 95, 99, 100)):
    x = np.abs(np.asarray(x))
    return ' '.join(f'p{p}={np.percentile(x, p):5.1f}' for p in ps) if len(x) else 'n/a'


def main(bag):
    d, cam = read(bag)
    t0 = d['/lidar_slam/odom'][0][0]
    if cam is not None:
        hfov = math.degrees(2 * math.atan(cam.width / (2 * cam.k[0])))
        print(f'camera {cam.width}x{cam.height} fx={cam.k[0]:.0f}: HFOV {hfov:.1f} deg '
              f'(half {hfov / 2:.1f})')
    odom = Series([(t, m) for t, m in d['/lidar_slam/odom']])
    yt = Series([(t, m) for t, m in d['/planning/yaw_target']])
    cmd = Series([(t, m) for t, m in d['/planning/pos_cmd']])
    st = Series([(t, m.nav_state) for t, m in d['/fmu/out/vehicle_status_v1']])
    lk = Series([(t, m.data) for t, m in d['/dtp/lock_state']])
    print('lock states seen:', sorted({m.data.split()[0] if m.data else '' for _, m in d['/dtp/lock_state']}))
    rows = []
    for t, m in d['/dtp/follow_target']:
        if st.at(t, 2.0) != 14:                  # OFFBOARD
            continue
        o, y, c = odom.at(t, 0.2), yt.at(t, 0.5), cmd.at(t, 0.3)
        if o is None:
            continue
        p = o.pose.pose.position
        q = o.pose.pose.orientation
        hd = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        dx, dy = m.position.x - p.x, m.position.y - p.y
        rng = math.hypot(dx, dy)
        bp = math.atan2(dy, dx)
        e_cam = wrap(bp - hd)
        row = dict(t=t - t0, rng=rng, bp=bp, hd=hd, e_cam=e_cam, lock=(lk.at(t, 5.0) or ''),
                   vperp=None, e_aim=None, e_super=None, e_px4=None, yawdot=None)
        # the person's speed across the line of sight, from the aircraft
        vx, vy = m.velocity.x, m.velocity.y
        row['vperp'] = (-math.sin(bp) * vx + math.cos(bp) * vy)
        row['vabs'] = math.hypot(vx, vy)
        if y is not None:
            ba = math.atan2(y.point.y - p.y, y.point.x - p.x)
            row['e_aim'] = wrap(bp - ba)
            if c is not None:
                row['e_super'] = wrap(ba - c.yaw)
                row['e_px4'] = wrap(c.yaw - hd)
                row['yawdot'] = c.yaw_dot
        rows.append(row)
    if not rows:
        print('no Offboard + target samples')
        return
    print(f'{len(rows)} target samples in Offboard, t {rows[0]["t"]:.1f}-{rows[-1]["t"]:.1f} s')
    D = math.degrees
    full = [r for r in rows if r['e_super'] is not None]
    print('|angle| deg          ', 'p50 / p90 / p95 / p99 / max')
    print(' e_cam  (camera off-centre)  ', pct([D(r['e_cam']) for r in rows]))
    print(' e_aim  (aim pt vs person)   ', pct([D(r['e_aim']) for r in full]))
    print(' e_super(SUPER cmd vs aim)   ', pct([D(r['e_super']) for r in full]))
    print(' e_px4  (heading vs SUPER)   ', pct([D(r['e_px4']) for r in full]))
    # bearing rate the camera had to follow
    br = []
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        if 0.02 < dt < 0.5:
            br.append(D(wrap(b['bp'] - a['bp'])) / dt)
    hr = []
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        if 0.02 < dt < 0.5:
            hr.append(D(wrap(b['hd'] - a['hd'])) / dt)
    print(' person bearing rate deg/s   ', pct(br))
    print(' aircraft yaw rate deg/s     ', pct(hr))
    print(' SUPER cmd yaw_dot deg/s     ', pct([D(r['yawdot']) for r in full]))
    print(' range m                     ', pct([r['rng'] for r in rows]))
    print(' person speed m/s            ', pct([r['vabs'] for r in rows]))
    print(' person speed across LOS m/s ', pct([r['vperp'] for r in rows]))
    # which link dominates when the camera is far off-centre
    big = [r for r in full if abs(D(r['e_cam'])) > 20]
    if big:
        print(f'\nsamples with |e_cam| > 20 deg: {len(big)} of {len(full)}; mean share of the error:')
        for k in ('e_aim', 'e_super', 'e_px4'):
            # signed contribution along e_cam's sign
            s = np.mean([r[k] * np.sign(r['e_cam']) for r in big]) / np.mean([abs(r['e_cam']) for r in big])
            print(f'   {k:8s} {100 * s:5.0f} %   (mean |{k}| {D(np.mean([abs(r[k]) for r in big])):5.1f} deg)')
    # is the error a lag? correlate e_cam with the bearing rate
    pairs = [(D(wrap(b['bp'] - a['bp'])) / (b['t'] - a['t']), D(b['e_cam']))
             for a, b in zip(rows, rows[1:]) if 0.02 < b['t'] - a['t'] < 0.5]
    if len(pairs) > 10:
        x, y = np.array(pairs).T
        k = np.polyfit(x, y, 1)
        print(f'\ne_cam vs bearing rate: slope {k[0]:.2f} s (= an effective lag), '
              f'corr {np.corrcoef(x, y)[0, 1]:.2f}')
    # the last samples before each lock loss
    print('\nlock transitions in Offboard (last target samples before a non-LOCKED state):')
    prev = None
    for i, r in enumerate(rows):
        s = r['lock'].split()[0] if r['lock'] else ''
        if prev is not None and prev.startswith('LOCKED') and not s.startswith('LOCKED'):
            tail = rows[max(0, i - 8):i + 1]
            print(f'  t={r["t"]:.1f}s -> {r["lock"][:60]!r}')
            for q in tail:
                print(f'     t={q["t"]:6.1f} rng {q["rng"]:4.1f} e_cam {D(q["e_cam"]):+6.1f} '
                      f'vperp {q["vperp"]:+5.2f} ' +
                      (f'aim {D(q["e_aim"]):+6.1f} super {D(q["e_super"]):+6.1f} px4 {D(q["e_px4"]):+6.1f}'
                       if q['e_super'] is not None else ''))
        prev = s


if __name__ == '__main__':
    for b in sys.argv[1:]:
        print('=' * 90, '\n', b)
        main(b)
