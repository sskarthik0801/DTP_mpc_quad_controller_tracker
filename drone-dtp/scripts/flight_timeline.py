#!/usr/bin/env python3
"""Stage A flight timeline from a stage_a_record bag: arm epochs, airborne time, lock states,
FAST-LIO stamp lag, detection fraction against range and light. Read-only. Run on the NX.

    python3 flight_timeline.py <bag_dir> [--every N]   (N-th JPEG decoded for brightness, default 20)
"""
import argparse
import bisect
import collections
import os

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/dtp/lock_state', '/mux/state', '/lidar_slam/odom', '/dtp/targets', '/dtp/perception_ok',
          '/fmu/out/vehicle_status_v1', '/camera/color/image_raw/compressed']
AIRBORNE_Z = 0.30
LAG_BAD_S = 0.30
RANGE_BINS = [(0, 3), (3, 4), (4, 6), (6, 8), (8, 10), (10, 99)]


def pct(v, p):
    if not v:
        return float('nan')
    s = sorted(v)
    return s[min(len(s) - 1, int(p * len(s)))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--every', type=int, default=20)
    args = ap.parse_args()
    bag = args.bag.rstrip('/')
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    storage = meta.get('storage_identifier', 'sqlite3')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in TOPICS if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}

    t0 = None
    lock = []          # (t, state, full)
    mux = []           # (t, state)
    odom = []          # (t, z, lag)
    tg = []            # (t, has, dist)
    pok = []           # (t, bool)
    arm = []           # (t, arming_state)
    bright = []        # (t, mean)
    n_img = 0
    try:
        import cv2
    except ImportError:
        cv2 = None

    while reader.has_next():
        topic, data, tns = reader.read_next()
        if t0 is None:
            t0 = tns
        t = (tns - t0) * 1e-9
        m = deserialize_message(data, cls[topic])
        if topic == '/dtp/lock_state':
            lock.append((t, m.data.split('|')[0], m.data))
        elif topic == '/mux/state':
            mux.append((t, '|'.join(m.data.split('|')[:2])))
        elif topic == '/lidar_slam/odom':
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            odom.append((t, m.pose.pose.position.z, tns * 1e-9 - st))
        elif topic == '/dtp/targets':
            if m.targets:
                tg.append((t, True, float(m.targets[0].distance_m)))
            else:
                tg.append((t, False, float('nan')))
        elif topic == '/dtp/perception_ok':
            pok.append((t, bool(m.data)))
        elif topic == '/fmu/out/vehicle_status_v1':
            arm.append((t, int(m.arming_state)))
        elif topic == '/camera/color/image_raw/compressed':
            n_img += 1
            if cv2 is not None and n_img % args.every == 0:
                img = cv2.imdecode(np.frombuffer(m.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    bright.append((t, float(img.mean())))

    dur = max(x[0] for x in odom) if odom else 0.0
    print('=' * 78)
    print('FLIGHT TIMELINE   %s   (%.0f s, start %s)' % (os.path.basename(bag), dur,
          meta.get('starting_time', {}).get('nanoseconds_since_epoch', 0) // 10 ** 9))
    print('=' * 78)

    # ---- arm epochs ---------------------------------------------------------------------
    epochs = []
    prev = None
    a_start = None
    for t, a in arm:
        if prev is not None and a != prev:
            if a == 2:
                a_start = t
            elif prev == 2 and a_start is not None:
                epochs.append((a_start, t))
                a_start = None
        prev = a
    if a_start is not None:
        epochs.append((a_start, dur))
    print('\n1. ARMING (PX4 vehicle_status)')
    if not arm:
        print('   no vehicle_status in the bag')
    for i, (a, b) in enumerate(epochs):
        print('   armed epoch %d: t=%6.1f -> %6.1f s  (%.0f s)' % (i + 1, a, b, b - a))
    if not epochs:
        print('   never armed')

    # ---- airborne from odom z ---------------------------------------------------------------
    ot = [x[0] for x in odom]
    oz = [x[1] for x in odom]
    air = [(t, z > AIRBORNE_Z) for t, z in zip(ot, oz)]
    air_t = sum(0.1 for _, a in air if a)
    zmax = max(oz) if oz else float('nan')
    print('\n2. AIRBORNE (odom z > %.2f m): %.0f s of %.0f s, max z %.2f m' % (AIRBORNE_Z, air_t, dur, zmax))
    segs = []
    cur = None
    for t, a in air:
        if a and cur is None:
            cur = t
        elif not a and cur is not None:
            if t - cur > 2.0:
                segs.append((cur, t))
            cur = None
    if cur is not None:
        segs.append((cur, dur))
    for a, b in segs:
        zz = [z for t, z in zip(ot, oz) if a <= t <= b]
        print('   airborne t=%6.1f -> %6.1f s (%.0f s)  z p50 %.2f max %.2f m' % (a, b, b - a, pct(zz, .5), max(zz)))

    def airborne_at(t):
        i = bisect.bisect_left(ot, t)
        i = min(max(i, 0), len(air) - 1)
        return air[i][1] if air else False

    # ---- lock timeline ----------------------------------------------------------------------
    print('\n3. LOCK STATE (/dtp/lock_state)')
    in_state = collections.Counter()
    in_state_air = collections.Counter()
    trans = []
    for i, (t, s, full) in enumerate(lock):
        t_next = lock[i + 1][0] if i + 1 < len(lock) else dur
        in_state[s] += t_next - t
        if airborne_at(t):
            in_state_air[s] += t_next - t
        if i == 0 or s != lock[i - 1][1]:
            trans.append((t, s, full))
    tot_air = sum(in_state_air.values()) or 1.0
    print('   %-10s %8s %8s   %s' % ('state', 'total s', 'air s', 'air %'))
    for s in ['LOCKED', 'COASTING', 'ACQUIRING', 'LOST', 'IDLE', 'ABSTAIN']:
        if in_state[s] or in_state_air[s]:
            print('   %-10s %8.1f %8.1f   %5.1f' % (s, in_state[s], in_state_air[s], 100.0 * in_state_air[s] / tot_air))
    print('   transitions: %d' % len(trans))
    lost_reasons = collections.Counter()
    reacq = []
    last_lost = None
    for t, s, full in trans:
        if s == 'LOST':
            r = full.split('|', 2)[2] if full.count('|') >= 2 else ''
            lost_reasons[r.split('(')[0].strip()[:60]] += 1
            last_lost = t
        if s == 'LOCKED' and last_lost is not None:
            reacq.append(t - last_lost)
            last_lost = None
    for r, c in lost_reasons.most_common():
        print('   LOST x%d: %s' % (c, r))
    if reacq:
        print('   LOST -> LOCKED again: %d times, %.1f s median, %.1f s max' % (len(reacq), pct(reacq, .5), max(reacq)))
    print('   LOCKED runs while airborne:')
    runs = []
    for i, (t, s, full) in enumerate(trans):
        if s == 'LOCKED':
            t_end = trans[i + 1][0] if i + 1 < len(trans) else dur
            if airborne_at(t) or airborne_at(t_end):
                rng = full.split(' at ')[-1].split(' m')[0] if ' at ' in full else '?'
                runs.append((t, t_end - t, rng))
    for t, d, rng in runs:
        print('     t=%6.1f s  %5.1f s  (locked at %s m)' % (t, d, rng))
    if runs:
        print('     longest %.1f s, median %.1f s, n=%d' % (max(r[1] for r in runs), pct([r[1] for r in runs], .5), len(runs)))

    # ---- odom stamp lag ------------------------------------------------------------------------
    lags = [x[2] for x in odom]
    print('\n4. FAST-LIO STAMP LAG (bag receive time - header.stamp)')
    print('   overall p50 %.3f  p95 %.3f  max %.3f s   (> %.1f s on %.1f %% of messages)' % (
        pct(lags, .5), pct(lags, .95), max(lags), LAG_BAD_S, 100.0 * sum(1 for x in lags if x > LAG_BAD_S) / max(1, len(lags))))
    bad = []
    cur = None
    for t, z, l in odom:
        if l > LAG_BAD_S and cur is None:
            cur = [t, t, l]
        elif l > LAG_BAD_S and cur is not None:
            cur[1] = t
            cur[2] = max(cur[2], l)
        elif l <= LAG_BAD_S and cur is not None:
            if cur[1] - cur[0] > 1.0:
                bad.append(tuple(cur))
            cur = None
    if cur is not None:
        bad.append(tuple(cur))
    for a, b, mx in bad:
        print('   stalled t=%6.1f -> %6.1f s (%.0f s), max lag %.1f s, %s' % (a, b, b - a, mx, 'AIRBORNE' if airborne_at(a) or airborne_at(b) else 'on the ground'))
    if not bad:
        print('   no stall longer than 1 s')
    air_lags = [l for t, z, l in odom if airborne_at(t)]
    if air_lags:
        print('   airborne only: p50 %.3f  p95 %.3f  max %.3f s' % (pct(air_lags, .5), pct(air_lags, .95), max(air_lags)))

    # ---- detection vs range -----------------------------------------------------------------------
    print('\n5. DETECTION (/dtp/targets: a frame with >= 1 person) vs RANGE')
    n_all = len(tg)
    n_has = sum(1 for x in tg if x[1])
    print('   all frames: %d, with a person %d (%.0f %%)' % (n_all, n_has, 100.0 * n_has / max(1, n_all)))
    air_tg = [x for x in tg if airborne_at(x[0])]
    if air_tg:
        print('   airborne : %d, with a person %d (%.0f %%)' % (len(air_tg), sum(1 for x in air_tg if x[1]), 100.0 * sum(1 for x in air_tg if x[1]) / len(air_tg)))
    # attribute an empty frame to the last known range within 2 s
    binc = collections.Counter()
    binh = collections.Counter()
    last_d = None
    last_t = None
    for t, has, d in tg:
        if has:
            last_d, last_t = d, t
            dd = d
        elif last_t is not None and t - last_t <= 2.0:
            dd = last_d
        else:
            continue
        for lo, hi in RANGE_BINS:
            if lo <= dd < hi:
                binc[(lo, hi)] += 1
                binh[(lo, hi)] += 1 if has else 0
                break
    print('   %-10s %7s %7s   %s' % ('range m', 'frames', 'seen', 'seen %  (empty frames credited to the last range seen <= 2 s before)'))
    for b in RANGE_BINS:
        if binc[b]:
            print('   %-10s %7d %7d   %5.1f' % ('%d-%d' % b if b[1] < 99 else '>%d' % b[0], binc[b], binh[b], 100.0 * binh[b] / binc[b]))
    dists = [d for t, has, d in tg if has]
    if dists:
        print('   range when seen: p05 %.1f  p50 %.1f  p95 %.1f  max %.1f m' % (pct(dists, .05), pct(dists, .5), pct(dists, .95), max(dists)))

    # ---- light --------------------------------------------------------------------------------------
    print('\n6. LIGHT (mean grey of every %d-th colour JPEG, /255) and detection per 60 s' % args.every)
    if not bright:
        print('   no images decoded (cv2 missing or no compressed images)')
    else:
        print('   %-12s %6s %8s %8s' % ('window', 'light', 'frames', 'seen %'))
        w = 60.0
        k = 0
        while k * w < dur:
            bw = [b for t, b in bright if k * w <= t < (k + 1) * w]
            fw = [x for x in tg if k * w <= x[0] < (k + 1) * w]
            if bw and fw:
                print('   %4.0f-%4.0f s   %5.0f %8d %7.0f%s' % (k * w, (k + 1) * w, np.mean(bw), len(fw), 100.0 * sum(1 for x in fw if x[1]) / len(fw), '  air' if airborne_at(k * w + w / 2) else ''))
            k += 1

    # ---- perception_ok --------------------------------------------------------------------------------
    if pok:
        blind = sum(1 for t, b in pok if not b)
        blind_air = sum(1 for t, b in pok if not b and airborne_at(t))
        n_air = sum(1 for t, b in pok if airborne_at(t))
        print('\n7. PERCEPTION_OK False: %.1f %% overall, %.1f %% airborne (%d of %d)' % (
            100.0 * blind / len(pok), 100.0 * blind_air / max(1, n_air), blind_air, n_air))

    # ---- mux --------------------------------------------------------------------------------------------
    if mux:
        mc = collections.Counter()
        for i, (t, s) in enumerate(mux):
            if airborne_at(t):
                mc[s] += (mux[i + 1][0] if i + 1 < len(mux) else dur) - t
        tot = sum(mc.values()) or 1.0
        print('\n8. MUX while airborne (time-weighted)')
        for s, d in mc.most_common(6):
            print('   %-22s %5.1f %%' % (s, 100.0 * d / tot))
    print()


if __name__ == '__main__':
    main()
