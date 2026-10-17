"""MEASURED: the horizontal loop in the two 2026-09-26 Offboard windows, stock gains, no feedforward.

python3 offboard_score.py   (numpy only; run extract_bags.py first)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import BAGS, EVENTS, load, lpf  # noqa: E402


def event_metrics(t, P, V, p, v, t0, t1):
    """Along-track error against the reference's own direction: max behind up to the reference's
    speed peak, max ahead after it, and the peak speed."""
    m = (t >= t0) & (t <= t1)
    s = np.hypot(*V[m].T)
    u = V[m] / np.maximum(s[:, None], 1e-6)
    al = np.sum((p[m] - P[m]) * u, axis=1)
    al[s <= 0.05] = np.nan
    k = int(np.argmax(s))
    return dict(behind=-np.nanmin(al[:k + 1]), ahead=np.nanmax(al[k:]),
                vpk=np.hypot(*v[m].T).max(), vref=s.max())


if __name__ == '__main__':
    for b in BAGS:
        R = load(b)
        t, dt, vff, vsp, v = R['t'], R['dt'], R['vff'], R['vsp'], R['v']
        e = R['p'] - R['psp']
        w0, w1 = R['win']
        d = R['raw']
        print(f'== {b}: Offboard {w0:.1f}-{w1:.1f} s after bag start ({w1-w0:.1f} s), {len(t)} samples, dt {dt*1000:.1f} ms')
        off = d['lp'][:, 0] - d['lp'][:, 1]
        print(f'   transport: local position arrives {np.median(off)*1000:.0f} ms after its PX4 stamp (median)')
        a0, a1, name = EVENTS[b]
        mm = event_metrics(t, R['psp'], vff, R['p'], v, w0 + a0, w0 + a1)
        print(f'   {name}: {mm["behind"]:.2f} m behind, {mm["ahead"]:.2f} m ahead, peak {mm["vpk"]:.2f} m/s (reference {mm["vref"]:.2f})')

        def lag(x, y):
            best = (0, -1)
            for L in range(40):
                c = np.mean([np.corrcoef(x[:len(x) - L, a], y[L:, a])[0, 1] for a in (0, 1)])
                if c > best[1]:
                    best = (L * dt, c)
            return best
        l1, l2 = lag(vsp, v), lag(vff, v)
        print(f'   lag vsp->v {l1[0]*1000:.0f} ms (r {l1[1]:.3f}); vel_ff->v {l2[0]*1000:.0f} ms (r {l2[1]:.3f})')
        print(f'   |vsp-v| rms {np.sqrt(np.mean(np.sum((vsp-v)**2,1))):.2f} m/s; |p-psp| rms {np.sqrt(np.mean(np.sum(e**2,1))):.2f}'
              f' p95 {np.percentile(np.hypot(*e.T),95):.2f} max {np.hypot(*e.T).max():.2f} m')
        print(f'   peaks: vsp {np.hypot(*vsp.T).max():.2f}, v {np.hypot(*v.T).max():.2f}, vel_ff {np.hypot(*vff.T).max():.2f} m/s')
        # steady stretches (reference speed > 0.6 m/s, |d vel_ff/dt| < 0.35 m/s^2 after a 1 Hz filter, >= 1.5 s)
        vs1 = np.c_[lpf(vff[:, 0], 1.0, dt), lpf(vff[:, 1], 1.0, dt)]
        acc = np.hypot(*np.gradient(vs1, t, axis=0).T)
        spd = np.hypot(*vff.T)
        idx = np.where((spd > 0.6) & (acc < 0.35))[0]
        for g in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
            if len(g) * dt < 1.5:
                continue
            u = vff[g] / spd[g, None]
            al = np.sum(e[g] * u, 1)
            ct = e[g, 0] * u[:, 1] - e[g, 1] * u[:, 0]
            dv = np.sum((v[g] - vff[g]) * u, 1)
            print(f'   steady {t[g[0]]-w0:.1f}-{t[g[-1]]-w0:.1f} s at {spd[g].mean():.2f} m/s: along {al.mean():+.3f} m,'
                  f' cross {np.abs(ct).mean():.3f} m, speed error {dv.mean():+.3f} m/s')
        # 0.5 s table of the steady cruise in flight 1, which the strict criterion above misses
        if b == '195108':
            for tt in np.arange(22.5, 27.01, 0.5):
                i = np.searchsorted(t, w0 + tt)
                s = np.hypot(*vff[i])
                u = vff[i] / s
                print(f'     t+{tt:4.1f}: vel_ff {s:.2f}, along {e[i] @ u:+.2f} m, |vsp-v| {np.hypot(*(vsp[i]-v[i])):.2f} m/s')
        # the reference itself
        sp = d['sp']
        tsp = sp[:, 1] + R['off'] - R['t0']
        m = (tsp > w0) & (tsp < w1)
        s, ts = sp[m], tsp[m]
        dp = np.hypot(*np.diff(s[:, 2:4], axis=0).T)
        vv = np.where(np.isfinite(s[:-1, 5:7]), s[:-1, 5:7], 0)
        jump = dp - np.hypot(*vv.T) * np.diff(ts)
        k = int(np.argmax(jump))
        print(f'   reference: largest pos_sp jump beyond vel*dt {jump[k]:.2f} m at t+{ts[k]-w0:.1f} s;'
              f' vz {np.nanmin(s[:,7]):+.2f} (up) / {np.nanmax(s[:,7]):+.2f} (down) m/s')
        pc = d['pc']
        tpc = pc[:, 0] - R['t0']
        mp = (tpc > w0) & (tpc < w1)
        acc = pc[mp, 7:9]
        jerk = np.hypot(*np.gradient(acc, tpc[mp], axis=0).T)
        print(f'   SUPER pos_cmd: |acc| max {np.hypot(*acc.T).max():.2f} m/s2, jerk p50 {np.median(jerk):.1f} p95 {np.percentile(jerk,95):.1f} m/s3')
        vx = d['lp'][:, 15]
        fin = np.isfinite(vx)
        print(f'   EKF2 vxy_max finite in {fin.sum()} samples' + (f', min {vx[fin].min():.2f} m/s at'
              f' {d["lp"][fin,0][0]-R["t0"]:.1f} s (hagl {d["lp"][fin,16][0]:.2f} m)' if fin.any() else ''))
