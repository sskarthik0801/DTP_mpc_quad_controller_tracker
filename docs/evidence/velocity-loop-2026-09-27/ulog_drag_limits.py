"""From the 2026-09-21 ULogs: rotor drag, and the limits that matter as speed rises.

Run with pyulog: ~/PX4-Autopilot/.venv/bin/python ulog_drag_limits.py
1. Drag: body-frame IMU specific force xy against body-frame velocity. Thrust lies along body z, so
   what the accelerometer sees in x/y is aerodynamic force: f_xy = -k * v_xy + offset.
2. Modes flown, and what the Position-mode segments contain (hover only).
3. Motor outputs, tilt, collective thrust: headroom.
4. Battery: voltage, current against speed, consumption.
"""
import os
import sys

import numpy as np
from pyulog import ULog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ULOG_DIR, ULOGS  # noqa: E402

AIR = {'F1': (6, 91), 'F2': (5, 103), 'F3': (8, 73), 'F4': (12, 128)}
NAV = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 14: 'OFFBOARD', 15: 'STAB'}


def to_body(q, v):
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return R.T @ v


for fl, (a0, a1) in AIR.items():
    u = ULog(os.path.join(ULOG_DIR, ULOGS[fl]),
             ['vehicle_acceleration', 'vehicle_attitude', 'vehicle_local_position', 'vehicle_status',
              'manual_control_setpoint', 'actuator_motors', 'vehicle_thrust_setpoint', 'battery_status'])
    t0 = u.start_timestamp
    ds = {d.name: d.data for d in u.data_list}
    L, A, Q = ds['vehicle_local_position'], ds['vehicle_acceleration'], ds['vehicle_attitude']
    tl = (L['timestamp'] - t0) / 1e6
    m = (tl > a0) & (tl < a1)
    tt = tl[m]
    fb = np.c_[[np.interp(tt, (A['timestamp'] - t0) / 1e6, A[f'xyz[{i}]']) for i in range(3)]].T
    qq = np.c_[[np.interp(tt, (Q['timestamp'] - t0) / 1e6, Q[f'q[{i}]']) for i in range(4)]].T
    vn = np.c_[L['vx'][m], L['vy'][m], L['vz'][m]]
    vb = np.array([to_body(qq[i], vn[i]) for i in range(len(tt))])
    ks = []
    for ax in (0, 1):
        X = np.c_[vb[:, ax], np.ones(len(tt))]
        c, *_ = np.linalg.lstsq(X, fb[:, ax], rcond=None)
        ks.append((-c[0], c[1], 1 - np.var(fb[:, ax] - X @ c) / np.var(fb[:, ax])))
    sp = np.hypot(vn[:, 0], vn[:, 1])
    print(f'== {fl} {ULOGS[fl]}')
    print(f'  drag: speed p95 {np.percentile(sp,95):.2f} max {sp.max():.2f} m/s | k_x {ks[0][0]:.2f} (R2 {ks[0][2]:.2f})'
          f' k_y {ks[1][0]:.2f} (R2 {ks[1][2]:.2f}) 1/s | hover offsets {ks[0][1]:+.2f}, {ks[1][1]:+.2f} m/s2')
    vs = ds['vehicle_status']
    ns = vs['nav_state']
    ch = [0] + list(np.where(np.diff(ns) != 0)[0] + 1)
    print('  modes:', ', '.join(f'{max(0.0, (int(vs["timestamp"][i]) - int(t0)) / 1e6):.1f}s '
                                f'{NAV.get(int(ns[i]), int(ns[i]))}' for i in ch))
    mc = ds['manual_control_setpoint']
    tm = (mc['timestamp'] - t0) / 1e6
    for i, j in zip(ch, ch[1:] + [len(ns) - 1]):
        if int(ns[i]) != 2:
            continue
        s0, s1 = (vs['timestamp'][i] - t0) / 1e6, (vs['timestamp'][j] - t0) / 1e6
        mm = (tm >= s0) & (tm <= s1)
        ml = (tl >= s0) & (tl <= s1)
        if mm.any() and ml.any():
            print(f'  POSCTL {s0:.1f}-{s1:.1f}s: |roll stick| max {np.abs(mc["roll"][mm]).max():.2f}, '
                  f'|pitch stick| max {np.abs(mc["pitch"][mm]).max():.2f}, speed max {np.hypot(L["vx"][ml], L["vy"][ml]).max():.2f} m/s')
    if 'actuator_motors' in ds:
        M = np.c_[[ds['actuator_motors'][f'control[{i}]'] for i in range(4)]].T
        fl_ = M.mean(1) > 0.25
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (Q['q[1]'] ** 2 + Q['q[2]'] ** 2), -1, 1)))
        thr = -ds['vehicle_thrust_setpoint']['xyz[2]']
        print(f'  headroom: motor out p50 {np.median(M[fl_]):.2f} max {M[fl_].max():.2f}; tilt max {tilt.max():.0f} deg;'
              f' collective thrust sp max {thr.max():.2f}')
    b = ds['battery_status']
    tb = (b['timestamp'] - t0) / 1e6
    fly = b['current_a'] > 10
    si = np.interp(tb, tl, np.hypot(L['vx'], L['vy']))
    bands = []
    for lo, hi in ((0, 0.5), (0.5, 1.5), (1.5, 3.5)):
        k = fly & (si >= lo) & (si < hi)
        if k.sum() > 5:
            bands.append(f'{lo}-{hi} m/s {np.median(b["current_a"][k]):.0f} A')
    print(f'  battery: {b["voltage_v"][fly][0]:.1f} -> {b["voltage_v"][fly][-1]:.1f} V (min {b["voltage_v"][fly].min():.1f}),'
          f' current max {b["current_a"].max():.0f} A, discharged since boot {b["discharged_mah"][-1]:.0f} mAh,'
          f' scale {b["scale"][fly].min():.2f}-{b["scale"][fly].max():.2f}; ' + '; '.join(bands))
