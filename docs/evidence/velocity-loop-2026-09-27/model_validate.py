"""MODELLED: does the model fly the 2026-09-26 Offboard windows the way the aircraft did?

python3 model_validate.py   (numpy only; a few minutes)
The model is driven by the recorded setpoint stream (no feedforward, as flown), starting from the
measured state, and compared with the measured velocity (output error) and the two events.
"""
import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import BAGS, EVENTS, load  # noqa: E402
from offboard_score import event_metrics  # noqa: E402
from sim_model import STOCK, run  # noqa: E402

DT = 0.001


def reference(b, ff=False):
    """The setpoint PX4 received, at 1 ms (zero-order hold), plus SUPER's acceleration if ff."""
    R = load(b)
    d = R['raw']
    sp = d['sp']
    tsp = sp[:, 1] + R['off'] - R['t0'] + 0.005
    w0, w1 = R['win']
    t = np.arange(w0, w1, DT)
    i = np.clip(np.searchsorted(tsp, t, side='right') - 1, 0, len(sp) - 1)
    P = sp[i, 2:4]
    V = np.where(np.isfinite(sp[i, 5:7]), sp[i, 5:7], 0.0)
    A = None
    if ff:   # SUPER pos_cmd acceleration, lidar_init FLU -> NED (x, -y); corr with d(vel_ff)/dt 0.95
        pc = d['pc']
        tpc = pc[:, 0] - R['t0'] + 0.005
        A = np.c_[np.interp(t, tpc, pc[:, 7]), np.interp(t, tpc, -pc[:, 8])]
    pm = np.c_[np.interp(t, R['t'], R['p'][:, 0]), np.interp(t, R['t'], R['p'][:, 1])]
    vm = np.c_[np.interp(t, R['t'], R['v'][:, 0]), np.interp(t, R['t'], R['v'][:, 1])]
    return t, P, V, A, pm, vm, R


def score(data, tau, delay, kd, verbose=False, label=''):
    tot, out = 0.0, []
    for b, (t, P, V, A, pm, vm, R) in data.items():
        p, v, *_ = run(t, P, V, None, STOCK, tau=tau, delay=delay, kdrag=kd, p0=pm[0], v0=vm[0])
        tot += np.mean(np.sum((v - vm) ** 2, 1))
        a0, a1, name = EVENTS[b]
        out.append((name, event_metrics(t, P, V, p, v, R['win'][0] + a0, R['win'][0] + a1)))
    rms = np.sqrt(tot / len(data))
    if verbose:
        print(f'  {label:10s} tau {tau:.2f} delay {delay*1000:3.0f} ms drag {kd:.2f}: velocity rms error {rms:.3f} m/s | ' +
              ' | '.join(f'{n}: behind {m["behind"]:.2f} ahead {m["ahead"]:.2f} peak {m["vpk"]:.2f}' for n, m in out))
    return rms


if __name__ == '__main__':
    data = {b: reference(b) for b in BAGS}
    print('MEASURED:')
    for b, (t, P, V, A, pm, vm, R) in data.items():
        a0, a1, name = EVENTS[b]
        m = event_metrics(t, P, V, pm, vm, R['win'][0] + a0, R['win'][0] + a1)
        print(f'  {b} {name}: behind {m["behind"]:.2f} ahead {m["ahead"]:.2f} peak {m["vpk"]:.2f} (reference {m["vref"]:.2f})')
    best = min((score(data, *x), x) for x in itertools.product(
        [0.10, 0.14, 0.17, 0.20, 0.24], [0.03, 0.05, 0.07, 0.09], [0.15, 0.30, 0.45]))
    print(f'OUTPUT-ERROR BEST: tau {best[1][0]}, delay {best[1][1]}, drag {best[1][2]}, velocity rms {best[0]:.3f} m/s')
    print('MODEL:')
    score(data, *best[1], True, 'best')
    score(data, 0.17, 0.05, 0.30, True, 'adopted')
    score(data, 0.12, 0.03, 0.30, True, 'fast')
    score(data, 0.22, 0.08, 0.30, True, 'slow')
    score(data, 0.15, 0.04, 0.00, True, 'no drag')
