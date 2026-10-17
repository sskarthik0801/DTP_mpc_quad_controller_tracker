"""MEASURED (fit): commanded vs achieved horizontal acceleration in the 2026-09-26 Offboard windows.

python3 offboard_accfit.py   (numpy only)
PX4's command is rebuilt from the logged setpoint and state:
    acc_cmd = VP*(vsp - v) + I - VD*LPF5(dv/dt),  I = VI * integral(vsp - v)  (+ unknown start value)
and fitted to EKF2's ax/ay:  a_meas = gain * FOPDT(acc_cmd; tau, delay) - cd*v + c  (per-axis c).
Both sides low-passed at 4 Hz. The fit is a ridge along tau + delay; the R2 grid shows it.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import BAGS, VD, VI, VP, fopdt, load, lpf  # noqa: E402

FC = 4.0


def design(R, tau, delay):
    dt, ev = R['dt'], R['vsp'] - R['v']
    acc = VP * ev + np.cumsum(ev, axis=0) * dt * VI - VD * R['vdot_f']
    X, Y = [], []
    for ax in (0, 1):
        m = lpf(fopdt(acc[:, ax], tau, delay, dt), FC, dt)
        n = len(m)
        X.append(np.c_[m, -lpf(R['v'][:, ax], FC, dt), np.full(n, float(ax == 0)), np.full(n, float(ax == 1))])
        Y.append(lpf(R['am'][:, ax], FC, dt))
    return np.vstack(X), np.concatenate(Y)


def solve(R, tau, delay):
    X, Y = design(R, tau, delay)
    c, *_ = np.linalg.lstsq(X, Y, rcond=None)
    r = np.mean((Y - X @ c) ** 2)
    return r, c, 1 - r / np.var(Y)


if __name__ == '__main__':
    for b in BAGS:
        R = load(b)
        best = None
        for delay in np.arange(0.0, 0.161, 0.01):
            for tau in np.arange(0.05, 0.41, 0.01):
                r, c, r2 = solve(R, tau, delay)
                if best is None or r < best[0]:
                    best = (r, tau, delay, c, r2)
        r, tau, delay, c, r2 = best
        print(f'{b}: best tau {tau:.2f} s, delay {delay*1000:.0f} ms, gain {c[0]:.2f}, drag {c[1]:.2f} 1/s,'
              f' offsets {c[2]:+.2f},{c[3]:+.2f} m/s2, R2 {r2:.3f}')
        delays = [0.02, 0.05, 0.07, 0.09, 0.11]
        print('   R2 grid, rows tau, columns delay ms: ' + ' '.join(f'{int(x*1000):5d}' for x in delays))
        for tau in (0.05, 0.10, 0.15, 0.17, 0.20, 0.25):
            print(f'   tau {tau:.2f}                          ' + ' '.join(f'{solve(R, tau, x)[2]:.3f}' for x in delays))
