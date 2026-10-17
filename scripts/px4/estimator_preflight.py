#!/usr/bin/env python3
"""G0.2 estimator preflight: refuse to arm on an estimate that cannot be trusted. GO / NO-GO.

On 2026-09-12 the aircraft was armed twice on a FAST-LIO pose 3.74 km from the truth, and
every health flag said it was fine: EKF2 `xy_valid`, `z_valid`, `eph 0.06 m`, odometry at
10 Hz. A diverged-then-settled LiDAR estimate is precise and self-consistent, so no *health*
flag can catch it -- only *where it is* can. docs/fact-fastlio-divergence-2026-09-12.md has
the evidence and the bench reproduction. This script asks the questions that would have
refused that session, and is safe to run at any time: it only subscribes and reads /proc.

    source /opt/ros/humble/setup.bash
    source ~/ws_livox/install/setup.bash
    source ~/super_ws/install/setup.bash
    python3 scripts/px4/estimator_preflight.py              # 60 s, bridge required
    python3 scripts/px4/estimator_preflight.py --window 20  # shorter, but see E7
    python3 scripts/px4/estimator_preflight.py --no-bridge  # bench, no PX4 bridge running

Run it with the aircraft ON THE GROUND WHERE IT WILL TAKE OFF, stack and bridges up, before
arming. Exit 0 = GO, 2 = NO-GO. Thresholds are docs/m5-m6-plan.md SS2.2 G0.2 unless noted:

  E1  one LiDAR source   exactly one publisher on /livox/lidar (CustomMsg), /livox/imu and
                         /lidar_slam/odom; exactly one livox_ros_driver2_node and one
                         fastlio_mapping process. [root cause, 09-12: a second driver took
                         the MID360's stream and starved FAST-LIO of scans for 225 s]
  E2  FAST-LIO is newer  fastlio_mapping started AFTER the running Livox driver. A driver
      than its driver    started later means FAST-LIO lived through a LiDAR outage, i.e.
                         integrated IMU alone. Restart FAST-LIO. [root cause]
  E3  odometry rate      >= 9.5 Hz in every 10 s of the window; consecutive stamps never
                         more than 1.0 s apart (the bridge guard's max_gap_s)
  E4  odometry age       p95 of (arrival - stamp) <= 150 ms. EKF2_DELAY_MAX is 200 ms on
                         this FC; in the 09-12 bag EV samples 200-540 ms old coincided with
                         17 EKF2 position resets in 135 s. [added 2026-09-12, beyond SS2.2]
  E5  near the origin    |odom| <= 1.0 m for the whole window. FAST-LIO starts at (0,0,0);
                         the aircraft should be where FAST-LIO was started. THIS is the check
                         that refuses the 09-12 session.
  E6  EKF2 follows EV    |EKF2 local position - FRD(odom)| <= 0.20 m, with xy_valid, z_valid
                         and not dead-reckoning (M2: 7 cm lever arm + 10.4 cm tracking)
  E7  no EKF2 resets     xy/z/heading reset counters unchanged across the window. The plan
                         asks for "none in the last 60 s", so --window below 60 WARNs
  E8  bridge guard       /px4_odom_bridge/status is TRACKING (not WAITING or DIVERGED)
"""
import argparse
import math
import os
import sys
import time

ODOM = '/lidar_slam/odom'
LPOS = '/fmu/out/vehicle_local_position_v1'
STATUS = '/px4_odom_bridge/status'
VO = '/fmu/in/vehicle_visual_odometry'

MIN_RATE_HZ = 9.5
MAX_STAMP_GAP_S = 1.0
MAX_AGE_P95_S = 0.150
MAX_ORIGIN_M = 1.0
MAX_EKF_VS_EV_M = 0.20
RESET_WINDOW_S = 60.0


def _procs(basename):
    """(pid, start_ticks) of processes whose EXECUTABLE (argv[0]) has this exact basename.

    argv[0] only -- not any argv token, and never a flattened command line. A shell that merely
    MENTIONS the name must not count (CLAUDE.md, the pkill -f trap), and neither may the
    `ros2 run fast_lio fastlio_mapping` wrapper, whose argv[0] is python3: counting it made a
    single FAST-LIO read as two on the first hardware run of this check (2026-09-12).
    """
    out = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                argv = [a.decode(errors='replace') for a in fh.read().split(b'\0') if a]
            with open(f'/proc/{pid}/stat') as fh:
                stat = fh.read()
        except OSError:
            continue
        # comm may contain spaces and parens; fields after the LAST ')' are fixed.
        fields = stat[stat.rfind(')') + 2:].split()
        if fields[0] == 'Z':
            continue
        if argv and os.path.basename(argv[0]) == basename:
            out.append((int(pid), int(fields[19])))   # field 22 overall = starttime
    return out


def _p95(vals):
    s = sorted(vals)
    return s[int(0.95 * (len(s) - 1))] if s else float('nan')


def run_checks(node, spin, window_s=RESET_WINDOW_S, require_bridge=True, log=print):
    """Collect for `window_s` on an existing rclpy node and return [(state, name, detail)].

    `spin(seconds)` must spin `node`. Kept separate from main() so drone-dtp's
    stage_a_preflight.py can run exactly these checks inside its own node.
    """
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleLocalPosition
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from std_msgs.msg import String

    rows = []

    def ok(n, d=''):
        rows.append(('PASS', n, d))

    def warn(n, d=''):
        rows.append(('WARN', n, d))

    def fail(n, d=''):
        rows.append(('FAIL', n, d))

    odom, lpos, status = [], [], {}

    def on_odom(m):
        s = m.header.stamp
        p = m.pose.pose.position
        odom.append((time.time(), s.sec + s.nanosec * 1e-9, (p.x, p.y, p.z)))

    def on_lpos(m):
        lpos.append((time.time(), m))

    px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
    subs = [node.create_subscription(Odometry, ODOM, on_odom, qos_profile_sensor_data),
            node.create_subscription(VehicleLocalPosition, LPOS, on_lpos, px4_qos),
            node.create_subscription(String, STATUS,
                                     lambda m: status.__setitem__('text', m.data), latched)]

    log(f'estimator preflight: listening {window_s:.0f} s -- keep the aircraft still, '
        'on the ground, where it will take off')
    t0 = time.time()
    spin(window_s)
    t1 = time.time()
    for s in subs:
        node.destroy_subscription(s)

    # -- E1 one source ------------------------------------------------------------------
    def pubs(topic):
        return node.get_publishers_info_by_topic(topic)

    lidar = pubs('/livox/lidar')
    types = sorted({p.topic_type for p in lidar})
    for topic, got in (('/livox/lidar', lidar), ('/livox/imu', pubs('/livox/imu')),
                       (ODOM, pubs(ODOM))):
        names = ', '.join(sorted(p.node_name for p in got)) or 'none'
        if len(got) == 1:
            ok(f'E1 single publisher {topic}', names)
        else:
            fail(f'E1 single publisher {topic}',
                 f'{len(got)} ({names}) -- two drivers or two FAST-LIOs share one sensor')
    if lidar and types != ['livox_ros_driver2/msg/CustomMsg']:
        fail('E1 /livox/lidar type', f'{types} -- a PointCloud2 driver on this topic is '
             'exactly what starved FAST-LIO on 09-12')
    drivers, fastlios = _procs('livox_ros_driver2_node'), _procs('fastlio_mapping')
    for what, got in (('livox_ros_driver2_node', drivers), ('fastlio_mapping', fastlios)):
        if len(got) == 1:
            ok(f'E1 one {what} process', f'pid {got[0][0]}')
        else:
            fail(f'E1 one {what} process',
                 f'{len(got)} running (pids {[p for p, _ in got]}) -- kill the duplicate, '
                 'then restart FAST-LIO: the one that survives may already have diverged')

    # -- E2 FAST-LIO newer than its driver ----------------------------------------------
    if len(drivers) == 1 and len(fastlios) == 1:
        hz = os.sysconf('SC_CLK_TCK')
        lead = (fastlios[0][1] - drivers[0][1]) / hz
        if lead > 0:
            ok('E2 FAST-LIO started after the Livox driver', f'{lead:.1f} s after')
        else:
            fail('E2 FAST-LIO started after the Livox driver',
                 f'the driver is {-lead:.1f} s NEWER than FAST-LIO: FAST-LIO lived through a '
                 'LiDAR outage and integrated IMU alone. Restart FAST-LIO.')
    else:
        fail('E2 FAST-LIO started after the Livox driver', 'needs exactly one of each (E1)')

    # -- E3 rate and stamp gaps ------------------------------------------------------------
    dur = t1 - t0
    if len(odom) < 2:
        fail('E3 odometry rate', f'{len(odom)} messages on {ODOM} in {dur:.0f} s')
    else:
        win = min(10.0, dur)
        worst = min(sum(1 for a, _, _ in odom if s <= a < s + win) / win
                    for s in [t0 + k * 0.5 for k in range(int((dur - win) / 0.5) + 1)])
        gap = max(b[1] - a[1] for a, b in zip(odom, odom[1:]))
        if worst >= MIN_RATE_HZ:
            ok('E3 odometry rate', f'worst {win:.0f} s window {worst:.1f} Hz')
        else:
            fail('E3 odometry rate', f'worst {win:.0f} s window {worst:.1f} Hz < {MIN_RATE_HZ}')
        if gap <= MAX_STAMP_GAP_S:
            ok('E3 odometry stamp gaps', f'max {gap:.2f} s')
        else:
            fail('E3 odometry stamp gaps', f'max {gap:.2f} s > {MAX_STAMP_GAP_S} s: FAST-LIO '
                 'went without scans -- restart it')

    # -- E4 age ---------------------------------------------------------------------------
    if odom:
        age = _p95([a - s for a, s, _ in odom])
        if age <= MAX_AGE_P95_S:
            ok('E4 odometry age', f'p95 {age * 1e3:.0f} ms')
        else:
            fail('E4 odometry age', f'p95 {age * 1e3:.0f} ms > {MAX_AGE_P95_S * 1e3:.0f} ms '
                 '(EKF2_DELAY_MAX 200 ms). FAST-LIO is starved of CPU -- check load, '
                 'extrinsic_est_en, and a laptop RViz on /cloud_registered')

    # -- E5 near the origin ------------------------------------------------------------------
    if odom:
        far = max(math.hypot(*p) for _, _, p in odom)
        last = odom[-1][2]
        if far <= MAX_ORIGIN_M:
            ok('E5 odometry near lidar_init origin', f'max |p| {far:.2f} m')
        else:
            fail('E5 odometry near lidar_init origin',
                 f'max |p| {far:.1f} m, now ({last[0]:.1f}, {last[1]:.1f}, {last[2]:.1f}) > '
                 f'{MAX_ORIGIN_M} m. Either the aircraft was carried after FAST-LIO started, '
                 'or FAST-LIO has diverged. Either way: restart FAST-LIO HERE.')

    # -- E6 EKF2 follows EV, E7 resets -----------------------------------------------------
    if not lpos:
        fail('E6 EKF2 local position', f'nothing on {LPOS} -- is the uXRCE-DDS link up?')
    else:
        invalid = sum(1 for _, m in lpos if not (m.xy_valid and m.z_valid))
        dr = sum(1 for _, m in lpos if m.dead_reckoning)
        if invalid or dr:
            fail('E6 EKF2 position valid',
                 f'{invalid} invalid / {dr} dead-reckoning of {len(lpos)} samples')
        else:
            ok('E6 EKF2 position valid', f'{len(lpos)} samples, all valid')
        if odom:
            worst, j = 0.0, 0
            for arr, m in lpos:
                while j + 1 < len(odom) and odom[j + 1][0] <= arr:
                    j += 1
                ox, oy, oz = odom[j][2]
                worst = max(worst, math.dist((m.x, m.y, m.z), (ox, -oy, -oz)))
            if worst <= MAX_EKF_VS_EV_M:
                ok('E6 |EKF2 - FRD(odom)|', f'max {worst:.3f} m')
            else:
                fail('E6 |EKF2 - FRD(odom)|', f'max {worst:.2f} m > {MAX_EKF_VS_EV_M} m -- EKF2 '
                     'is not following the LiDAR estimate (bridge down, or fusing something '
                     'else)')
        first, last_m = lpos[0][1], lpos[-1][1]
        resets = {k: (getattr(last_m, k) - getattr(first, k)) % 256
                  for k in ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter')}
        if any(resets.values()):
            fail('E7 no EKF2 resets', f'{resets} in {dur:.0f} s')
        elif dur < RESET_WINDOW_S - 1:
            warn('E7 no EKF2 resets', f'none, but only {dur:.0f} s observed (plan asks 60 s)')
        else:
            ok('E7 no EKF2 resets', f'none in {dur:.0f} s')

    # -- E8 bridge guard -------------------------------------------------------------------
    if require_bridge:
        text = status.get('text')
        vo = pubs(VO)
        if len(vo) != 1:
            fail('E8 single publisher ' + VO, f'{len(vo)} -- the odom bridge is '
                 + ('not running' if not vo else 'running twice'))
        if text is None:
            fail('E8 odom bridge guard', f'nothing on {STATUS} -- bridge down, or an old '
                 'bridge without the divergence guard')
        elif text.startswith('TRACKING'):
            ok('E8 odom bridge guard', text)
        else:
            fail('E8 odom bridge guard', text)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--window', type=float, default=RESET_WINDOW_S,
                    help='seconds to observe (default 60; E7 WARNs below 60)')
    ap.add_argument('--no-bridge', action='store_true',
                    help='bench only: do not require px4_odom_bridge (E8)')
    args = ap.parse_args(argv)
    if args.window < 10:
        ap.error('--window must be >= 10 s (E3 needs a 10 s window)')

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('estimator_preflight')

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        spin(2.0)   # discovery settle
        rows = run_checks(node, spin, args.window, not args.no_bridge)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    w = max(len(r[1]) for r in rows)
    print('\nESTIMATOR PREFLIGHT (G0.2)')
    for state, name, detail in rows:
        mark = {'PASS': ' ok ', 'WARN': 'WARN', 'FAIL': 'FAIL'}[state]
        print(f'  [{mark}] {name:<{w}}  {detail}')
    fails = sum(1 for r in rows if r[0] == 'FAIL')
    warns = sum(1 for r in rows if r[0] == 'WARN')
    print()
    if fails:
        print(f'NO-GO: {fails} failing check(s). Do not arm.')
        return 2
    print(f'GO with {warns} warning(s) -- read them.' if warns else 'GO.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
