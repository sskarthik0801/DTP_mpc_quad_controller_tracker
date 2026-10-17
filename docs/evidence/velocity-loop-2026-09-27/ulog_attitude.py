"""Attitude loop from the 2026-09-21 ULogs: the lag between a tilt setpoint and the tilt.

Run with pyulog: ~/PX4-Autopilot/.venv/bin/python ulog_attitude.py
1. First order + delay fit of tilt(t) to tilt_sp(t), per axis, 6 s blocks (gain and offset free).
2. The guide's p.110 angle check on sharp wobbles: estimated/setpoint peak ratio and peak lag.
"""
import os
import sys

import numpy as np
from pyulog import ULog
from scipy.signal import find_peaks

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ULOG_DIR, ULOGS, fopdt  # noqa: E402

DT = 0.005
WINDOWS = {'F1': [(6, 90)], 'F2': [(19.5, 48), (49.5, 102)], 'F3': [(19, 73)], 'F4': [(18, 128)]}


def euler(q0, q1, q2, q3):
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1))
    return roll, pitch


def fit(u, y):
    best = None
    for delay in np.arange(0.0, 0.101, 0.005):
        for tau in np.arange(0.03, 0.40, 0.01):
            ys = fopdt(u, tau, delay, DT)
            A = np.c_[ys, np.ones_like(ys)]
            c, *_ = np.linalg.lstsq(A, y, rcond=None)
            r = np.mean((y - A @ c) ** 2)
            if best is None or r < best[0]:
                best = (r, tau, delay, c[0])
    r, tau, delay, gain = best
    return tau, delay, gain, 1 - r / np.var(y)


for fl, wins in WINDOWS.items():
    u = ULog(os.path.join(ULOG_DIR, ULOGS[fl]), ['vehicle_attitude', 'vehicle_attitude_setpoint'])
    t0 = u.start_timestamp
    a = u.get_dataset('vehicle_attitude').data
    s = u.get_dataset('vehicle_attitude_setpoint').data
    ta, tsp = (a['timestamp'] - t0) / 1e6, (s['timestamp'] - t0) / 1e6
    ra, pa = euler(*[a[f'q[{i}]'] for i in range(4)])
    rs, ps = euler(*[s[f'q_d[{i}]'] for i in range(4)])
    for w0, w1 in wins:
        t = np.arange(w0, w1, DT)
        for name, sp, act in (('roll', rs, ra), ('pitch', ps, pa)):
            idx = np.clip(np.searchsorted(tsp, t, side='right') - 1, 0, len(tsp) - 1)
            U, Y = np.degrees(sp[idx]), np.degrees(np.interp(t, ta, act))
            res = []
            for c0 in np.arange(w0, w1 - 6, 6):
                m = (t >= c0) & (t < c0 + 6)
                if np.std(U[m]) >= 1.0:
                    res.append(fit(U[m], Y[m]))
            if not res:
                continue
            res = np.array(res)
            q = lambda k: np.percentile(res[:, k], [25, 50, 75])  # noqa: E731
            print(f'{fl} {name:5s} [{w0}-{w1}] n={len(res)}  tau {q(0)[1]:.3f} s (IQR {q(0)[0]:.2f}-{q(0)[2]:.2f})'
                  f'  delay {q(1)[1]*1000:.0f} ms (IQR {q(1)[0]*1000:.0f}-{q(1)[2]*1000:.0f})'
                  f'  gain {q(2)[1]:.2f}  R2 {q(3)[1]:.3f}')
            if fl in ('F3', 'F4'):   # clean wobbles: the guide's peak comparison
                ratios, lags = [], []
                for sgn in (1, -1):
                    pk, _ = find_peaks(sgn * U, prominence=4, distance=60)
                    for p in pk:
                        if abs(U[p]) < 5:
                            continue
                        j = p + np.argmax(sgn * Y[p:p + 80])
                        ratios.append(Y[j] / U[p])
                        lags.append((j - p) * DT * 1000)
                if ratios:
                    print(f'      p.110 check: {len(ratios)} setpoint peaks >= 5 deg, estimated/setpoint '
                          f'{np.median(ratios):.2f} (IQR {np.percentile(ratios,25):.2f}-{np.percentile(ratios,75):.2f}),'
                          f' peak lag {np.median(lags):.0f} ms')
