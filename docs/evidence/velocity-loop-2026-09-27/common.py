"""Shared paths, filters and the Offboard-window reconstruction.

Read-only analysis. Nothing here talks to ROS, DDS or the flight controller.
"""
import os
import pickle

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
BAGS = {
    '195108': os.path.join(REPO, 'bags', 'stage_b_2026-09-26_195108'),
    '200650': os.path.join(REPO, 'bags', 'stage_b_2026-09-26_200650'),
}
ULOG_DIR = os.path.expanduser('~/px4_logs')
ULOGS = {  # the 2026-09-21 retune flights (docs/px4-tuning-notes.md section 9)
    'F1': '2026-09-21_18_48_30.ulg',   # Stabilized hover
    'F2': '2026-09-21_19_18_48.ulg',   # Stabilized/Position/Altitude, offboard bridge UP (confounded)
    'F3': '2026-09-21_20_12_25.ulg',   # wobbles, no LIO, no bridge
    'F4': '2026-09-21_20_13_44.ulg',   # up and down, no LIO, no bridge
}
CACHE = os.environ.get('VELTUNE_CACHE', '/tmp/velocity-loop-2026-09-27')

# PX4 gains flown on 2026-09-26 (firmware/px4-params-2026-09-21-post-batscale-baroautocal.params)
KP, VP, VI, VD = 0.95, 1.8, 0.4, 0.2

# Events scored in each Offboard window, seconds after the Offboard engage
EVENTS = {'195108': (0.0, 6.0, 'engage'), '200650': (9.0, 16.5, 'sprint')}


def lpf(x, fc, dt):
    """First-order low-pass, the shape PX4's AlphaFilter uses."""
    a = dt / (dt + 1.0 / (2 * np.pi * fc))
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def fopdt(u, tau, delay, dt):
    """First order plus dead time."""
    k = int(round(delay / dt))
    ud = np.r_[np.full(k, u[0]), u[:len(u) - k]] if k > 0 else u.copy()
    y = np.empty_like(ud)
    y[0] = ud[0]
    a = dt / (tau + dt)
    for i in range(1, len(ud)):
        y[i] = y[i - 1] + a * (ud[i] - y[i - 1])
    return y


def cache_file(bag):
    return os.path.join(CACHE, f'bag_{bag}.pkl')


def raw(bag):
    with open(cache_file(bag), 'rb') as f:
        return pickle.load(f)


def offboard_window(d):
    """(start, end) in bag-relative seconds of the first armed+Offboard stretch."""
    vcm = d['vcm']
    on = (vcm[:, 1] > 0) & (vcm[:, 2] > 0)
    e = np.where(np.diff(on.astype(int)) != 0)[0]
    return vcm[e[0] + 1, 0] - d['t0'], vcm[e[1] + 1, 0] - d['t0']


def load(bag, sp_lat=0.005):
    """PX4's horizontal setpoints reconstructed on the local-position samples of the Offboard window.

    vsp = vel_ff + MPC_XY_P * (pos_sp - pos)  (PositionControl.cpp:127-135). NED.
    Local position is placed on PX4's timestamp_sample, mapped to bag time by the median transport
    offset; the setpoint is the last one PX4 had received (bridge stamp + sp_lat).
    """
    d = raw(bag)
    t0 = d['t0']
    sp, lp = d['sp'], d['lp']
    off = np.median(lp[:, 0] - lp[:, 2])
    w0, w1 = offboard_window(d)
    tl = lp[:, 2] + off - t0
    m = (tl >= w0) & (tl <= w1)
    L, tl = lp[m], tl[m]
    tsp = sp[:, 1] + off - t0 + sp_lat
    idx = np.clip(np.searchsorted(tsp, tl, side='right') - 1, 0, len(sp) - 1)
    S = sp[idx]
    psp = S[:, 2:4]
    vff = np.where(np.isfinite(S[:, 5:7]), S[:, 5:7], 0.0)
    p, v, am = L[:, 3:5], L[:, 6:8], L[:, 9:11]
    dt = float(np.median(np.diff(tl)))
    vdot = np.gradient(v, tl, axis=0)
    vdot_f = np.c_[lpf(vdot[:, 0], 5, dt), lpf(vdot[:, 1], 5, dt)]   # MPC_VELD_LP 5 Hz
    return dict(t=tl, psp=psp, vff=vff, p=p, v=v, am=am, vsp=vff + KP * (psp - p),
                vdot_f=vdot_f, dt=dt, t0=t0, off=off, win=(w0, w1), raw=d)
