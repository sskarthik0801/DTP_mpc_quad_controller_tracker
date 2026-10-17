"""INDICATIVE ONLY: a rotation between EKF2's acceleration and FAST-LIO's, 2026-09-26 flights.

python3 yaw_rotation.py   (numpy only)
EKF2's ax/ay is the FC's IMU rotated by the heading EKF2 takes from FAST-LIO (EV yaw). FAST-LIO's
trajectory does not depend on how the MID360 is bolted to the FC. If the MID360 is yawed by psi
against the FC, EKF2's acceleration is rotated by about psi against FAST-LIO's. EKF2 can absorb part
of a real offset through its position corrections, so this is a lower bound, not a measurement of
the mount (the open flight blocker in .claude/rules/px4-estimator.md).
Fit: a_lio = c * a_ekf over the whole armed flight, complex c = scale * exp(i*angle), best lag.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import BAGS, lpf, raw  # noqa: E402

DT, FC = 0.02, 1.5


def fit(zl, ze):
    zl, ze = zl - zl.mean(), ze - ze.mean()
    c = np.vdot(ze, zl) / np.vdot(ze, ze)
    return c, np.mean(np.abs(zl - c * ze) ** 2) / np.mean(np.abs(zl) ** 2)


for b in BAGS:
    d = raw(b)
    t0 = d['t0']
    vs = d['vs']
    armed = vs[vs[:, 3] == 2, 0] - t0
    a0, a1 = armed[0] + 8, armed[-1] - 3
    t = np.arange(a0, a1, DT)
    lp, od = d['lp'], d['od']
    tl = lp[:, 2] + np.median(lp[:, 0] - lp[:, 2]) - t0
    f2 = lambda x: lpf(lpf(x, FC, DT), FC, DT)  # noqa: E731
    ex, ey = f2(np.interp(t, tl, lp[:, 9])), f2(np.interp(t, tl, lp[:, 10]))
    to = od[:, 0] - t0
    dd = lambda x: np.gradient(np.gradient(f2(x), DT), DT)  # noqa: E731
    Ax, Ay = dd(np.interp(t, to, od[:, 1])), dd(np.interp(t, to, -od[:, 2]))   # FLU -> NED (x, -y)
    best = None
    for lag in range(-15, 16):
        s1 = slice(50 + max(lag, 0), len(t) - 50 + min(lag, 0))
        s2 = slice(50 + max(-lag, 0), len(t) - 50 + min(-lag, 0))
        c, r = fit(Ax[s1] + 1j * Ay[s1], ex[s2] + 1j * ey[s2])
        if best is None or r < best[0]:
            best = (r, lag, c)
    r, lag, c = best
    print(f'{b}: armed {a0:.0f}-{a1:.0f} s, |a_lio| rms {np.sqrt(np.mean(Ax**2+Ay**2)):.2f} m/s2, lag {lag*DT*1000:+.0f} ms:'
          f' a_lio = {abs(c):.3f} * R({np.degrees(np.angle(c)):+.2f} deg) * a_ekf, residual {r*100:.0f} % of power')
    n = len(t) // 2
    for name, s in (('first half', slice(50, n)), ('second half', slice(n, len(t) - 50))):
        c, _ = fit(Ax[s] + 1j * Ay[s], ex[s] + 1j * ey[s])
        print(f'   {name}: {np.degrees(np.angle(c)):+.2f} deg, scale {abs(c):.3f}')
