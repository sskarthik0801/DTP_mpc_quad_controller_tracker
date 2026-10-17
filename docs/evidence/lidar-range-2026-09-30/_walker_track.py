"""LiDAR-only walker tracker for the Step 1 scripts: follows the walker's DYNAMIC points out past
the camera's 10 m cap. Seeded from frames where the camera and the moving-walker LiDAR truth agree
(stereo_lidar_bias.py's bias_rows.csv: r_dyn finite, walking), tracked forward from the last and
backward from the first frame of every such run, one scan (0.1 s) per step.

A step takes the dynamic points 0.3-2.1 m above the local ground within GATE_M (growing while
coasting) of the constant-velocity prediction, from the scan at the step and its two neighbours
(each shifted by the velocity to the step time); the measured position is their median xy; the
velocity is a 1.0 s line fit of the measured positions (clipped to 4 m/s). It stops after
MAX_COAST_S without points, or when the walker leaves the camera's FOV or 25 m. Every state keeps
`n` = points on the walker in the step's own scan (0.1 s).
"""
import csv

import numpy as np

from _step1_lib import BODY_HI, BODY_LO, is_static

ROWS = ('/tmp/claude-1000/-home-ideas-ad-SUPER-Human-DTP/d04c92ae-44af-44ea-8c75-b858015fae68/'
        'scratchpad/step1_measure/bias_rows.csv')
GATE_M = 0.7
MAX_COAST_S = 1.5
MAX_SPEED = 4.0          # the 09-29 walker ran back at ~3.3 m/s (t 450-453)
MAX_TRACK_S = 40.0
FOV_HALF_DEG = 50.0      # keep the walker inside the camera's 90 deg horizontal FOV (+margin)
MAX_RANGE_M = 25.0


def seeds(b):
    rows = [r for r in csv.DictReader(open(ROWS)) if r['bag'] == b.name]
    good = [r for r in rows if r['kind'] == 'walking' and r['r_dyn'] not in ('', 'nan')
            and np.isfinite(float(r['r_dyn']))]
    out, run = [], []
    for r in good + [None]:
        if r is not None and run and float(r['t_bag']) - float(run[-1]['t_bag']) < 0.4 \
                and r['tid'] == run[-1]['tid']:
            run.append(r)
            continue
        if len(run) >= 3:
            pts = []
            for q in run:
                t = float(q['t_bag']) + b.start
                a = b.pos(t)
                br, rr = float(q['brg']), float(q['r_dyn'])
                pts.append((t, a[0] + rr * np.cos(br), a[1] + rr * np.sin(br)))
            pts = np.array(pts)
            for end, dirn in ((pts[-3:], +1), (pts[:3], -1)):
                v = np.array([np.polyfit(end[:, 0], end[:, 1], 1)[0],
                              np.polyfit(end[:, 0], end[:, 2], 1)[0]]) if np.ptp(end[:, 0]) > 0.05 else np.zeros(2)
                k = -1 if dirn > 0 else 0
                out.append((end[k, 0], end[k, 1:], v, dirn))
        run = [r] if r is not None else []
    return out


def track(b, t0, p0, v0, dirn):
    p, v = np.array(p0, float), np.array(v0, float)
    sp = np.linalg.norm(v)
    if sp > MAX_SPEED:
        v *= MAX_SPEED / sp
    hist = [(t0, p.copy())]
    states, coast = [], 0.0
    i = int(np.argmin(np.abs(b.sc_t - 0.05 - t0)))
    while True:
        i += dirn
        if i < 0 or i >= len(b.sc_t):
            break
        t = b.sc_t[i] - 0.05
        if abs(t - t0) > MAX_TRACK_S:
            break
        dt = t - hist[-1][0] if coast == 0 else t - states[-1][0]
        pred = p + v * dt
        gate = GATE_M + 1.0 * coast
        q_all, n = [], 0
        for j in (i - 1, i, i + 1):
            if j < 0 or j >= len(b.sc_t):
                continue
            tj = b.sc_t[j] - 0.05
            a = b.pts[b.sc_off[j]:b.sc_off[j + 1]]
            pj = pred + v * (tj - t)
            m = (np.abs(a[:, 0] - pj[0]) < gate) & (np.abs(a[:, 1] - pj[1]) < gate)
            q = a[m].astype(float)
            if len(q):
                h = q[:, 2] - b.ground(q[:, 0], q[:, 1])
                q = q[(h > BODY_LO) & (h < BODY_HI) & (np.hypot(q[:, 0] - pj[0], q[:, 1] - pj[1]) < gate)]
            if len(q):
                q = q[~is_static(b, q, tj)]
            if len(q):
                q[:, :2] += v * (t - tj)
                q_all.append(q)
                if j == i:
                    n = len(q)
        q = np.vstack(q_all) if q_all else np.zeros((0, 3))
        if len(q):
            p = np.median(q[:, :2], axis=0)
            hist.append((t, p.copy()))
            hh = [x for x in hist if abs(x[0] - t) <= 1.0]
            if len(hh) >= 3 and abs(hh[-1][0] - hh[0][0]) > 0.3:
                tt = np.array([x[0] for x in hh])
                pp = np.array([x[1] for x in hh])
                v = np.array([np.polyfit(tt, pp[:, 0], 1)[0], np.polyfit(tt, pp[:, 1], 1)[0]])
                sp = np.linalg.norm(v)
                if sp > MAX_SPEED:
                    v *= MAX_SPEED / sp
            coast = 0.0
        else:
            p = pred
            coast += abs(dt)
            if coast > MAX_COAST_S:
                break
        a0 = b.pos(t)
        rng = np.hypot(p[0] - a0[0], p[1] - a0[1])
        off = np.degrees(np.angle(np.exp(1j * (np.arctan2(p[1] - a0[1], p[0] - a0[0]) - b.yaw(t)))))
        if rng > MAX_RANGE_M or abs(off) > FOV_HALF_DEG:
            break
        states.append((t, p[0], p[1], v[0], v[1], n, coast))
    return states


def walker_states(b, keep_coast=False):
    """All tracked states for a bag, one per scan time: where several tracks cover a scan, the one
    with the least coasting (then the most points) wins. Returns an (N, 7) array
    [t, x, y, vx, vy, n_scan, coast_s], sorted by t; coasting states (the constant-velocity
    prediction inside a gap of <= MAX_COAST_S) dropped unless keep_coast."""
    best = {}
    for t0, p0, v0, dirn in seeds(b):
        for s in track(b, t0, p0, v0, dirn):
            k = round(s[0], 2)
            if k not in best or (s[6], -s[5]) < (best[k][6], -best[k][5]):
                best[k] = s
    if not best:
        return np.zeros((0, 7))
    S = np.array(sorted(best.values()))
    return S if keep_coast else S[S[:, 6] == 0]
