#!/usr/bin/env python3
"""Step 1 pre-build measurements (2) and (3): LiDAR points on the walker at 5-25 m with 0.4 / 0.6 s
windows, and what a whole-body image box would catch around them (background, margin, the
nearest-surface rule's outcome).

    python3 lidar_box_points.py [bag_name ...]      # needs stereo_lidar_bias.py's bias_rows.csv

The walker: _walker_track.py (the LiDAR's dynamic points, seeded where camera and LiDAR agree,
followed past the camera's 10 m cap). One sample per 0.1 s scan, including coasting states inside
a tracked span (so a scan with no points still counts as a window).

(2) POINTS ON THE WALKER. For trailing windows of 0.1 (one scan), 0.4 and 0.6 s: the dynamic
    points 0.3-2.1 m above the local ground within 0.5 m horizontally of the walker's tracked
    position AT EACH SCAN'S TIME (i.e. perfectly motion-compensated). Also: how far the walker
    moves in the window (track velocity x window), total and along the line of sight, and whether
    the body lies above the MID360's -7 deg floor at the aircraft's attitude.
(3) THE BOX. A YOLO-like box around the walker, 0.6 m wide x 1.8 m tall (feet to head), shrunk
    10 % at each edge (0.48 m x 0.18-1.62 m), seen from the colour camera (odom pose + the
    composed T_body_camera). A LiDAR point is IN the box when its direction falls inside that
    angular footprint, optionally with the vertical misprojection dv found in this study (the
    LiDAR projects ~3 deg HIGH: a point at true elevation e lands at e + dv), and its horizontal
    range is within +-3 m of the walker's. Older scans are shifted by the walker's velocity x age
    (the design's motion compensation) before the test. Points are 'walker' (within 0.6 m of the
    walker and > 0.1 m above the ground, static-labelled or not) or 'background' (everything
    else, the ground included).
    The design rule on the in-box ranges: depth_source's nearest-surface cluster (10th-pct seed,
    +-0.6 m band) median, needing MIN points (2 or 3). Outcome against the walker's tracked range:
    OK (|err| <= 0.4 m), NEAR (too short), FAR (too long: the dangerous side), NONE.
    'ground-rejected' drops points < 0.25 m above the local ground (the stack's terrain map could
    provide that) before the rule.

Outputs: tables on stdout; per-window rows in <scratch>/step1_measure/box_rows.csv.
"""
import csv
import os
import sys

import numpy as np

from _step1_lib import (BAGS, BODY_HI, BODY_LO, Bag, build_static, is_static, lidar_floor_deg,
                        nearest_cluster, pitch_deg)
from _walker_track import walker_states

OUT = ('/tmp/claude-1000/-home-ideas-ad-SUPER-Human-DTP/d04c92ae-44af-44ea-8c75-b858015fae68/'
       'scratchpad/step1_measure')
T_BODY_CAM = np.array([0.08329, -0.01614, 0.06705])   # camera optical centre in lidar_imu
BOX_HALF_W = 0.24
BOX_LO, BOX_HI = 0.18, 1.62
GATE_M = 3.0
WALKER_R = 0.5
BOX_WALKER_R = 0.6
DV_DEG = 3.0
GROUND_REJECT_M = 0.25
WINDOWS = (0.1, 0.4, 0.6)
BINS = [(5, 8), (8, 10), (10, 12), (12, 15), (15, 17), (17, 25)]


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def cam_pos(b, t):
    i = min(np.searchsorted(b.od_t, t), len(b.od_t) - 1)
    return b.pos(t) + quat_R(*b.c['od'][i, 4:8]) @ T_BODY_CAM


def analyse(name):
    b = Bag(name)
    build_static(b)
    S = walker_states(b, keep_coast=True)
    if len(S) == 0:
        return []
    St = S[:, 0]

    def walker_at(t):
        k = np.searchsorted(St, t)
        if k <= 0 or k >= len(St) or St[k] - St[k - 1] > 0.35:
            j = int(np.argmin(np.abs(St - t)))
            return S[j, 1:3] + S[j, 3:5] * (t - St[j]) if abs(St[j] - t) < 0.35 else None
        f = (t - St[k - 1]) / (St[k] - St[k - 1])
        return S[k - 1, 1:3] * (1 - f) + S[k, 1:3] * f

    rows = []
    for k in range(len(S)):
        t, wx, wy, vx, vy, n_scan, coast = S[k]
        c = cam_pos(b, t)
        a = b.pos(t)
        r_w = float(np.hypot(wx - a[0], wy - a[1]))
        if r_w < 5 or r_w > 25:
            continue
        az_w = np.arctan2(wy - c[1], wx - c[0])
        rc_w = np.hypot(wx - c[0], wy - c[1])
        g_w = float(b.ground(wx, wy))
        u = np.array([np.cos(np.arctan2(wy - a[1], wx - a[0])), np.sin(np.arctan2(wy - a[1], wx - a[0]))])
        floor = lidar_floor_deg(b, t, np.arctan2(wy - a[1], wx - a[0]))
        e_top = np.degrees(np.arctan2(g_w + 1.7 - a[2], r_w))
        e_bot = np.degrees(np.arctan2(g_w + 0.3 - a[2], r_w))
        row = dict(bag=name, t_bag=t - b.start, r=r_w, coast=coast, speed=float(np.hypot(vx, vy)),
                   v_rad=float(abs(vx * u[0] + vy * u[1])), pitch=pitch_deg(b, t),
                   agl=float(a[2] - b.ground(a[0], a[1])),
                   body_vis=float(np.clip((e_top - floor) / (e_top - e_bot), 0, 1)))
        # gather the 0.6 s window around the walker (all points), label, shift
        P, T = b.window_points(t - 0.6 + 1e-3, t + 1e-3, (wx, wy), GATE_M + 1.5)
        if len(P):
            h = P[:, 2] - b.ground(P[:, 0], P[:, 1])
            dyn = ~is_static(b, P, t) if len(P) < 4000 else np.ones(len(P), bool)
            wpos = np.array([walker_at(tj) if walker_at(tj) is not None else [np.nan, np.nan] for tj in T])
            near_w = np.hypot(P[:, 0] - wpos[:, 0], P[:, 1] - wpos[:, 1]) < WALKER_R
            is_w = dyn & near_w & (h > BODY_LO) & (h < BODY_HI)
            # for the box: anything within BOX_WALKER_R of the walker and off the ground is the
            # walker's (feet, arms, a static-labelled point on a slow walker), not background
            w_rel = (np.hypot(P[:, 0] - wpos[:, 0], P[:, 1] - wpos[:, 1]) < BOX_WALKER_R) & (h > 0.1)
            age = t - T
        else:
            h = dyn = is_w = w_rel = age = np.zeros(0)
        for W in WINDOWS:
            m = age < W
            row['n_%.1f' % W] = int(np.count_nonzero(is_w[m])) if len(P) else 0
        if len(P) and is_w.any():
            row['h_p10'], row['h_p90'] = np.percentile(h[is_w & (age < 0.6)], [10, 90]) if (is_w & (age < 0.6)).any() else (np.nan, np.nan)
        else:
            row['h_p10'] = row['h_p90'] = np.nan
        # the box
        for W in (0.4, 0.6):
            m = age < W if len(P) else np.zeros(0, bool)
            Q = P[m].copy() if len(P) else np.zeros((0, 3))
            if len(Q):
                Q[:, 0] += vx * age[m]
                Q[:, 1] += vy * age[m]
            iw, hh = (w_rel[m].astype(bool), h[m]) if len(P) else (np.zeros(0, bool), np.zeros(0))
            dx, dy = Q[:, 0] - c[0], Q[:, 1] - c[1]
            rc = np.hypot(dx, dy)
            az = np.angle(np.exp(1j * (np.arctan2(dy, dx) - az_w)))
            el = np.arctan2(Q[:, 2] - c[2], rc)
            rh = np.hypot(Q[:, 0] - a[0], Q[:, 1] - a[1])
            inaz = np.abs(az) < np.arctan2(BOX_HALF_W, rc_w)
            ingate = np.abs(rh - r_w) < GATE_M
            for dvd in (0.0, DV_DEG):
                dv = np.radians(dvd)
                lo = np.arctan2(g_w + BOX_LO - c[2], rc_w) - dv
                hi = np.arctan2(g_w + BOX_HI - c[2], rc_w) - dv
                inbox = inaz & (el >= lo) & (el <= hi) & ingate
                key = 'W%.1f_dv%d' % (W, dvd)
                bg = inbox & ~iw
                row[key + '_nw'] = int(np.count_nonzero(inbox & iw))
                row[key + '_nbg'] = int(np.count_nonzero(bg))
                row[key + '_nbg_front'] = int(np.count_nonzero(bg & (rh < r_w - 0.3)))
                row[key + '_nbg_behind'] = int(np.count_nonzero(bg & (rh > r_w + 0.3)))
                row[key + '_nbg_ground'] = int(np.count_nonzero(bg & (hh < GROUND_REJECT_M)))
                behind = rh[bg & (rh > r_w)]
                row[key + '_gap_behind'] = float(behind.min() - r_w) if len(behind) else np.nan
                for gr in (0, 1):
                    sel = inbox & ((hh >= GROUND_REJECT_M) if gr else True)
                    for mn in (2, 3):
                        rr, n, _ = nearest_cluster(rh[sel])
                        if n < mn:
                            out = 'NONE'
                        elif abs(rr - r_w) <= 0.4:
                            out = 'OK'
                        else:
                            out = 'NEAR' if rr < r_w else 'FAR'
                        row['%s_g%d_min%d' % (key, gr, mn)] = out
        rows.append(row)
    return rows


def pct(x, q):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return np.percentile(x, q) if len(x) else np.nan


def main():
    names = sys.argv[1:] or BAGS
    rows = []
    for name in names:
        rows += analyse(name)
    keys = list(rows[0].keys())
    with open(os.path.join(OUT, 'box_rows.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    A = {k: np.array([r[k] for r in rows]) for k in keys}
    print('Windows per bag and range bin (one per 0.1 s scan):')
    for name in names:
        print('  %-36s %s' % (name, '  '.join('%d-%d m: %d' % (lo, hi, np.sum((A['bag'] == name) & (A['r'] >= lo) & (A['r'] < hi))) for lo, hi in BINS)))

    print('\n(2) POINTS ON THE WALKER (dynamic, 0.3-2.1 m above ground, within 0.5 m of the tracked walker), pooled')
    print('  bin     | windows | body above floor | 0.1 s: p50 p90  >=1   | 0.4 s: p10 p50 p90  >=2   >=3  | 0.6 s: p10 p50 p90  >=2   >=3  | heights hit p10-p90')
    for lo, hi in BINS:
        m = (A['r'] >= lo) & (A['r'] < hi)
        if not m.any():
            continue
        n1, n4, n6 = A['n_0.1'][m], A['n_0.4'][m], A['n_0.6'][m]
        print('  %2d-%2d m | %5d   |     %4.0f%%       | %4.0f %4.0f  %4.0f%%  | %4.0f %4.0f %4.0f  %4.0f%% %4.0f%% | %4.0f %4.0f %4.0f  %4.0f%% %4.0f%% | %.2f-%.2f m' % (
            lo, hi, m.sum(), 100 * np.mean(A['body_vis'][m] >= 0.99),
            np.median(n1), pct(n1, 90), 100 * np.mean(n1 >= 1),
            pct(n4, 10), np.median(n4), pct(n4, 90), 100 * np.mean(n4 >= 2), 100 * np.mean(n4 >= 3),
            pct(n6, 10), np.median(n6), pct(n6, 90), 100 * np.mean(n6 >= 2), 100 * np.mean(n6 >= 3),
            pct(A['h_p10'][m], 50), pct(A['h_p90'][m], 50)))
    print('\n  same, only windows with the whole body above the LiDAR floor (attitude not blinding it)')
    for lo, hi in BINS:
        m = (A['r'] >= lo) & (A['r'] < hi) & (A['body_vis'] >= 0.99)
        if m.sum() < 5:
            continue
        n4, n6 = A['n_0.4'][m], A['n_0.6'][m]
        print('  %2d-%2d m | %5d   | 0.4 s p50 %3.0f  >=2 %4.0f%%  >=3 %4.0f%% | 0.6 s p50 %3.0f  >=2 %4.0f%%  >=3 %4.0f%%' % (
            lo, hi, m.sum(), np.median(n4), 100 * np.mean(n4 >= 2), 100 * np.mean(n4 >= 3),
            np.median(n6), 100 * np.mean(n6 >= 2), 100 * np.mean(n6 >= 3)))
    print('\n  per bag, 8-17 m: windows | 0.4 s >=2 / >=3 | 0.6 s >=2 / >=3')
    for name in names:
        m = (A['bag'] == name) & (A['r'] >= 8) & (A['r'] < 17)
        if m.any():
            print('  %-36s %5d | %4.0f%% / %4.0f%% | %4.0f%% / %4.0f%%' % (
                name, m.sum(), 100 * np.mean(A['n_0.4'][m] >= 2), 100 * np.mean(A['n_0.4'][m] >= 3),
                100 * np.mean(A['n_0.6'][m] >= 2), 100 * np.mean(A['n_0.6'][m] >= 3)))
    print('\n  walker motion inside the window (track speed x window), 8-17 m: total | along the line of sight')
    m = (A['r'] >= 8) & (A['r'] < 17)
    for W in (0.4, 0.6):
        print('   %.1f s: p50 %.2f  p90 %.2f  max %.2f m | radial p50 %.2f  p90 %.2f  max %.2f m   (speed p50 %.2f p90 %.2f m/s)' % (
            W, W * np.median(A['speed'][m]), W * pct(A['speed'][m], 90), W * np.max(A['speed'][m]),
            W * np.median(A['v_rad'][m]), W * pct(A['v_rad'][m], 90), W * np.max(A['v_rad'][m]),
            np.median(A['speed'][m]), pct(A['speed'][m], 90)))

    print('\n(3) THE BOX: background inside the whole-body box within +-3 m of the walker\'s range (motion-shifted)')
    for W in (0.4, 0.6):
        for dvd in (0, int(DV_DEG)):
            key = 'W%.1f_dv%d' % (W, dvd)
            print('\n  window %.1f s, vertical misprojection %d deg %s' % (W, dvd, '(as deployed, per this study)' if dvd else '(calibration perfect)'))
            print('  bin     | walker pts in box p50 (>=2) | bg pts p50 p90  any | bg front / behind / of it ground (mean) | gap to 1st bg behind p10 p50, <0.6 m | rule min2: OK NEAR FAR NONE | min3: OK NEAR FAR NONE | ground-rejected min2: OK NEAR FAR NONE')
            for lo, hi in BINS:
                m = (A['r'] >= lo) & (A['r'] < hi)
                if not m.any():
                    continue
                nw, nb = A[key + '_nw'][m], A[key + '_nbg'][m]
                gap = A[key + '_gap_behind'][m]

                def outc(col):
                    v = A[col][m]
                    return ' '.join('%3.0f' % (100 * np.mean(v == o)) for o in ('OK', 'NEAR', 'FAR', 'NONE'))
                print('  %2d-%2d m | %4.0f (%4.0f%%)               | %4.0f %4.0f %4.0f%% | %5.1f / %5.1f / %5.1f              | %5.2f %5.2f  %4.0f%%         | %s    | %s | %s' % (
                    lo, hi, np.median(nw), 100 * np.mean(nw >= 2), np.median(nb), pct(nb, 90), 100 * np.mean(nb > 0),
                    np.mean(A[key + '_nbg_front'][m]), np.mean(A[key + '_nbg_behind'][m]), np.mean(A[key + '_nbg_ground'][m]),
                    pct(gap, 10), pct(gap, 50), 100 * np.mean(np.nan_to_num(gap, nan=9) < 0.6),
                    outc(key + '_g0_min2'), outc(key + '_g0_min3'), outc(key + '_g1_min2')))


if __name__ == '__main__':
    main()
