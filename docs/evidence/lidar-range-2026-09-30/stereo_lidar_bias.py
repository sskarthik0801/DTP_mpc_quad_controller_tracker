#!/usr/bin/env python3
"""Step 1 pre-build measurement (1): the camera-vs-LiDAR range bias in the overlap, 3-10 m.

    python3 stereo_lidar_bias.py [bag_name ...]        # default: the five flight bags

Camera: every /dtp/targets entry that is confirmed, has a detector confidence > 0.3, and is not
a HELD frame. Camera range = HORIZONTAL distance from the aircraft (odom at the frame stamp) to
the published position. That position is process_frame's EMA (alpha 0.25) of the raw patch lift,
so on a moving walker it LAGS (~3 frames = 0.375 s at 8 Hz, plus depth<->colour delivery skew).

LiDAR, all from /cloud_registered_body odom-transformed into lidar_init, points 0.3-2.1 m above
the local ground, accumulated over 0.5 s CENTRED on the frame stamp (the walker's own motion
averages out; each FAST-LIO scan is timed at its mid-acquisition, stamp - 0.05 s):
  r_disc  lidar_on_person.py's gate: within 0.6 m horizontally of the camera position, then
          depth_source's nearest-surface cluster (10th-pct seed, +-0.6 m) median. >= 5 points.
          It TRUNCATES: a camera error > ~0.8 m leaves the walker outside the disc.
  r_dyn   the MOVING-walker truth: only points classified dynamic (_step1_lib.is_static), within
          0.8 m laterally of the camera BEARING and +-4 m of the camera range, densest-cluster
          median. >= 5 points. Blind to a person who stands still.
  r_wall  what a naive whole-wedge LiDAR range would say: all points (static too) in the same
          wedge, nearest-surface rule. Shows the fence/background risk, not a truth.
Truth used for the bias: r_dyn for a walking track, r_disc for a standing person (perception
speed < 0.3 m/s and the disc's points static).
bias = camera - truth. On a moving walker bias = b - tau * rdot (rdot = range rate, receding > 0);
b and tau are fitted per bin (robust least squares). rdot comes from the r_dyn series.
Also recorded per frame: aircraft pitch (nose-up +) and whether the walker's body (0.3-1.7 m)
lies above the MID360's -7 deg floor at that attitude.

Per-frame rows: <scratch>/step1_measure/bias_rows.csv.
"""
import csv
import os
import sys

import numpy as np

from _step1_lib import (BAGS, BODY_HI, BODY_LO, Bag, build_static, densest_cluster, is_static,
                        lidar_floor_deg, nearest_cluster, pitch_deg)

OUT = ('/tmp/claude-1000/-home-ideas-ad-SUPER-Human-DTP/d04c92ae-44af-44ea-8c75-b858015fae68/'
       'scratchpad/step1_measure')
HALF_WIN = 0.25
MIN_PTS = 5


def frame_ranges(b, t, cx, cy, ax, ay, az, brg, rcam):
    a = np.array([ax, ay])
    reach = rcam + 4.5
    P, _ = b.window_points(t - HALF_WIN, t + HALF_WIN, (ax, ay), reach)
    out = dict(r_disc=np.nan, n_disc=0, disc_static=np.nan, r_dyn=np.nan, n_dyn=0,
               r_wall=np.nan, n_wall=0)
    if len(P) == 0:
        return out
    h = P[:, 2] - b.ground(P[:, 0], P[:, 1])
    P = P[(h > BODY_LO) & (h < BODY_HI)]
    dx, dy = P[:, 0] - a[0], P[:, 1] - a[1]
    rr = np.hypot(dx, dy)
    lat = rr * np.sin(np.arctan2(dy, dx) - brg)
    front = np.cos(np.arctan2(dy, dx) - brg) > 0
    disc = np.hypot(P[:, 0] - cx, P[:, 1] - cy) < 0.6
    r, n, _ = nearest_cluster(rr[disc])
    if n >= MIN_PTS:
        out['r_disc'], out['n_disc'] = r, n
        out['disc_static'] = float(is_static(b, P[disc], t).mean())
    wedge = front & (np.abs(lat) < 0.8) & (np.abs(rr - rcam) < 4.0)
    if wedge.any():
        r, n, _ = nearest_cluster(rr[wedge & (np.abs(lat) < 0.6)])
        if n >= MIN_PTS:
            out['r_wall'], out['n_wall'] = r, n
        dyn = ~is_static(b, P[wedge], t)
        r, n = densest_cluster(rr[wedge][dyn])
        if n >= MIN_PTS:
            out['r_dyn'], out['n_dyn'] = r, n
    return out


def analyse(name):
    b = Bag(name)
    build_static(b)
    cf = b.camera_frames()
    walker_ids = set(b.ft[:, 2].astype(int))
    sel = np.nonzero((cf['r'] >= 3.0) & (cf['r'] < 10.0))[0]
    rows = []
    for k in sel:
        t = cf['t'][k]
        g = b.ground(cf['x'][k], cf['y'][k])
        floor = lidar_floor_deg(b, t, cf['brg'][k])
        el_top = np.degrees(np.arctan2(g + 1.7 - cf['az'][k], cf['r'][k]))
        el_bot = np.degrees(np.arctan2(g + 0.3 - cf['az'][k], cf['r'][k]))
        row = dict(bag=name, t_bag=t - b.start, tid=cf['tid'][k],
                   locked_id=int(cf['tid'][k] in walker_ids), r_cam=cf['r'][k], brg=cf['brg'][k],
                   x=cf['x'][k], y=cf['y'][k], speed=float(np.hypot(cf['vx'][k], cf['vy'][k])),
                   agl=cf['az'][k] - b.ground(cf['ax'][k], cf['ay'][k]),
                   pitch=pitch_deg(b, t), floor=floor,
                   body_vis=float(np.clip((el_top - floor) / max(el_top - el_bot, 1e-6), 0, 1)))
        row.update(frame_ranges(b, t, cf['x'][k], cf['y'][k], cf['ax'][k], cf['ay'][k],
                                cf['az'][k], cf['brg'][k], cf['r'][k]))
        rows.append(row)
    for tid in set(r['tid'] for r in rows):
        idx = [i for i, r in enumerate(rows) if r['tid'] == tid]
        tt = np.array([rows[i]['t_bag'] for i in idx])
        rl = np.array([rows[i]['r_dyn'] for i in idx])
        for j, i in enumerate(idx):
            near = (np.abs(tt - tt[j]) <= 0.4) & np.isfinite(rl)
            ok = near.sum() >= 4 and np.ptp(tt[near]) > 0.3
            rows[i]['rdot'] = float(np.polyfit(tt[near], rl[near], 1)[0]) if ok else np.nan
    for r in rows:
        standing = r['speed'] < 0.3 and np.isfinite(r['r_disc']) and r['disc_static'] > 0.5
        r['kind'] = 'standing' if standing else 'walking'
        r['r_truth'] = r['r_disc'] if standing else r['r_dyn']
    return rows


def stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return None
    return dict(n=len(x), med=np.median(x), mean=x.mean(), sd=x.std(),
                p16=np.percentile(x, 16), p84=np.percentile(x, 84),
                p05=np.percentile(x, 5), p95=np.percentile(x, 95),
                mad=1.4826 * np.median(np.abs(x - np.median(x))))


def fit_b_tau(bias, rdot):
    m = np.isfinite(bias) & np.isfinite(rdot)
    if m.sum() < 10 or np.std(rdot[m]) < 0.2:
        return np.nan, np.nan
    A = np.vstack([np.ones(m.sum()), -rdot[m]]).T
    y = bias[m]
    coef = np.linalg.lstsq(A, y, rcond=None)[0]
    for _ in range(2):
        res = y - A @ coef
        ok = np.abs(res) < 2.5 * 1.4826 * np.median(np.abs(res)) + 1e-9
        coef = np.linalg.lstsq(A[ok], y[ok], rcond=None)[0]
    return coef[0], coef[1]


BINS = [(3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9), (9, 10)]


def table(A, mask_all, title, truth='r_truth'):
    print('\n' + title)
    print('  bin    | frames | truth ok | bias med [p16, p84]  (p05, p95)   MAD  | |bias|>0.5 | fit b   tau  | tracks | LiDAR-visible body (med) | pitch med')
    for lo, hi in BINS:
        m = mask_all & (A['r_cam'] >= lo) & (A['r_cam'] < hi)
        if m.sum() == 0:
            continue
        bias = A['r_cam'][m] - A[truth][m]
        s = stats(bias)
        bfit, tau = fit_b_tau(bias, A['rdot'][m])
        ntr = len(set(zip(A['bag'][m], A['tid'][m])))
        if s:
            print('  %2d-%2d m | %5d  |  %4.0f%%   | %+5.2f [%+5.2f,%+5.2f] (%+5.2f,%+5.2f) %4.2f |   %4.0f%%    | %+5.2f  %4.2f | %4d   |   %4.0f%%     | %+5.1f' % (
                lo, hi, m.sum(), 100 * s['n'] / m.sum(), s['med'], s['p16'], s['p84'], s['p05'], s['p95'],
                s['mad'], 100 * np.mean(np.abs(bias[np.isfinite(bias)]) > 0.5), bfit, tau, ntr,
                100 * np.median(A['body_vis'][m]), np.median(A['pitch'][m])))
        else:
            print('  %2d-%2d m | %5d  |     0%%   |   no truth' % (lo, hi, m.sum()))


def main():
    names = sys.argv[1:] or BAGS
    os.makedirs(OUT, exist_ok=True)
    allrows = []
    for name in names:
        allrows += analyse(name)
    keys = list(allrows[0].keys())
    with open(os.path.join(OUT, 'bias_rows.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(allrows)
    A = {k: np.array([r[k] for r in allrows]) for k in keys}
    walk = A['kind'] == 'walking'
    stand = A['kind'] == 'standing'
    for name in names:
        table(A, A['bag'] == name, '== %s  ALL persons (walking truth = dynamic cluster, standing = disc)' % name)
    table(A, np.ones(len(A['bag']), bool), '== ALL FIVE BAGS POOLED, ALL persons')
    for name in names:
        bm = A['bag'] == name
        table(A, bm & walk, '== %s  WALKING (truth = dynamic LiDAR cluster)  camera - LiDAR, m' % name)
        table(A, bm & stand, '== %s  STANDING (truth = disc cluster)' % name)
    table(A, walk, '== ALL FIVE BAGS POOLED, WALKING')
    table(A, stand, '== ALL FIVE BAGS POOLED, STANDING')
    table(A, walk & (A['locked_id'] == 1), '== POOLED, WALKING, locked-walker tracks only')
    print('\n== POOLED WALKING 5-10 m, bias split by range rate (receding vs approaching)')
    for lo, hi in [(5, 7), (7, 10)]:
        for lab, mm in [('receding rdot>+0.4', A['rdot'] > 0.4),
                        ('approach rdot<-0.4', A['rdot'] < -0.4),
                        ('|rdot|<0.3', np.abs(A['rdot']) < 0.3)]:
            m = walk & mm & (A['r_cam'] >= lo) & (A['r_cam'] < hi)
            s = stats(A['r_cam'][m] - A['r_truth'][m])
            if s:
                print('  %d-%2d m %-20s n %4d  med %+5.2f  [%+5.2f,%+5.2f]' % (
                    lo, hi, lab, s['n'], s['med'], s['p16'], s['p84']))
    print('\n== truth availability on WALKING frames 5-10 m vs LiDAR visibility of the body')
    for lab, mm in [('body fully above floor', A['body_vis'] >= 0.99),
                    ('partly', (A['body_vis'] > 0.01) & (A['body_vis'] < 0.99)),
                    ('below the floor (nose-up)', A['body_vis'] <= 0.01)]:
        m = walk & mm & (A['r_cam'] >= 5) & (A['r_cam'] < 10)
        if m.sum():
            print('  %-28s frames %4d  dyn truth found %4.0f%%  disc found %4.0f%%  pitch med %+5.1f' % (
                lab, m.sum(), 100 * np.isfinite(A['r_dyn'][m]).mean(),
                100 * np.isfinite(A['r_disc'][m]).mean(), np.median(A['pitch'][m])))
    m = walk & np.isfinite(A['r_disc']) & np.isfinite(A['r_dyn'])
    print('\ndisc vs dyn where both valid (walking): median |diff| %.3f m, p90 %.3f (n %d)' % (
        np.median(np.abs(A['r_disc'][m] - A['r_dyn'][m])),
        np.percentile(np.abs(A['r_disc'][m] - A['r_dyn'][m]), 90), m.sum()))
    m = walk & np.isfinite(A['r_wall']) & np.isfinite(A['r_dyn']) & (A['r_cam'] >= 5)
    d = A['r_wall'][m] - A['r_dyn'][m]
    big = np.abs(d) > 0.5
    print('naive whole-wedge nearest-surface vs dyn truth, walking 5-10 m: |diff|>0.5 m in %.1f%% (n %d)%s' % (
        100 * np.mean(big), m.sum(), (', of which the wedge NEARER %.0f%%' % (100 * np.mean(d[big] < 0))) if big.any() else ''))
    for lo, hi in [(5, 7), (7, 10)]:
        m = np.isfinite(A['r_truth']) & (A['r_cam'] >= lo) & (A['r_cam'] < hi)
        e = A['r_cam'][m] - A['r_truth'][m]
        print('camera gross errors %d-%d m (all persons): |cam - LiDAR| > 1 m in %.1f%%, > 2 m in %.1f%% (n %d); camera SHORT in %.0f%% of the > 1 m cases' % (
            lo, hi, 100 * np.mean(np.abs(e) > 1), 100 * np.mean(np.abs(e) > 2), m.sum(), 100 * np.mean(e[np.abs(e) > 1] < 0) if np.any(np.abs(e) > 1) else 0))
    print('\n== frame-to-frame range step, same track, consecutive frames <= 0.2 s apart (pooled), m')
    print('  bin    | camera |step| p50 / p90 / max | LiDAR truth |step| p50 / p90')
    for lo, hi in BINS:
        sc, sl = [], []
        for name in names:
            for tid in set(A['tid'][A['bag'] == name]):
                m = np.nonzero((A['bag'] == name) & (A['tid'] == tid))[0]
                for i0, i1 in zip(m[:-1], m[1:]):
                    if A['t_bag'][i1] - A['t_bag'][i0] <= 0.2 and lo <= A['r_cam'][i1] < hi:
                        sc.append(abs(A['r_cam'][i1] - A['r_cam'][i0]))
                        if np.isfinite(A['r_truth'][i1]) and np.isfinite(A['r_truth'][i0]):
                            sl.append(abs(A['r_truth'][i1] - A['r_truth'][i0]))
        if sc:
            print('  %2d-%2d m | %5.2f / %5.2f / %5.2f          | %5.2f / %5.2f   (n %d / %d)' % (
                lo, hi, np.median(sc), np.percentile(sc, 90), np.max(sc),
                np.median(sl) if sl else np.nan, np.percentile(sl, 90) if sl else np.nan,
                len(sc), len(sl)))


if __name__ == '__main__':
    main()
