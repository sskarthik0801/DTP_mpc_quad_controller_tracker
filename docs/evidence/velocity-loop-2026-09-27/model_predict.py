"""MODELLED: predictions for candidate gain sets, feedforward on and off.

python3 model_predict.py [sections]   (numpy only; all sections take several minutes)
Sections: steps margins moves disturbance replay tail dragff sensitivity ladder jumps
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import BAGS, EVENTS  # noqa: E402
from sim_model import DELAY, KDRAG, TAU, Gains, jerk_limited_move, run  # noqa: E402

DT = 0.001
SETS = [
    Gains(0.95, 1.8, 0.4, 0.20, 'stock 0.95/1.8/0.4/0.20'),
    Gains(0.95, 1.8, 0.4, 0.10, 'D down 0.95/1.8/0.4/0.10'),
    Gains(0.95, 2.5, 0.5, 0.10, '0.95/2.5/0.5/0.10'),
    Gains(0.95, 3.0, 0.6, 0.10, '0.95/3.0/0.6/0.10'),
    Gains(0.95, 3.0, 1.0, 0.10, '0.95/3.0/1.0/0.10'),
    Gains(0.95, 4.0, 0.4, 0.05, 'guide 0.95/4.0/0.4/0.05'),
    Gains(1.20, 3.0, 0.6, 0.10, '1.2/3.0/0.6/0.10'),
    Gains(1.50, 3.0, 0.6, 0.10, '1.5/3.0/0.6/0.10'),
]
TUNED = SETS[3]
PLANTS = [('fast tau.12 d.03', 0.12, 0.03), ('adopted tau.17 d.05', 0.17, 0.05), ('slow tau.22 d.08', 0.22, 0.08)]


def vel_step(g, amp=2.0, tau=TAU, delay=DELAY, kd=KDRAG):
    """Velocity setpoint step 0 -> amp (position not controlled: Direct velocity mode)."""
    t = np.arange(0, 6, DT)
    V = np.zeros((len(t), 2))
    V[t >= 0.5, 0] = amp
    p, v, a, vsp, acc = run(t, np.zeros_like(V), V, None, g, tau=tau, delay=delay, kdrag=kd, pos_ctrl=False)
    m = t >= 0.5
    tt, x = t[m] - 0.5, v[m, 0] / amp
    rise = tt[np.argmax(x >= 0.9)] - tt[np.argmax(x >= 0.1)]
    out = np.where(np.abs(x - 1) > 0.05)[0]
    return rise, max(0.0, x.max() - 1), (tt[out[-1]] if len(out) else 0.0), acc[:, 0].max()


def margins(g, tau=TAU, delay=DELAY, kd=KDRAG, ctrl_hz=50.0, veld=5.0):
    """Phase margin (deg), gain margin (dB) of the inner velocity loop and of the position loop."""
    w = np.logspace(-2, 2.5, 20000)
    s = 1j * w
    plant = 0.95 * np.exp(-s * delay) / (tau * s + 1) / (s + kd) * np.exp(-s / ctrl_hz / 2)
    Lv = (g.vp + g.vi / s + g.vd * s / (s / (2 * np.pi * veld) + 1)) * plant
    Lp = g.kp * ((g.vp + g.vi / s) * plant / (1 + Lv)) / s

    def pm_gm(L):
        mag, ph = np.abs(L), np.unwrap(np.angle(L))
        i = np.where(np.diff(np.sign(mag - 1)) != 0)[0]
        j = np.where(np.diff(np.sign(np.degrees(ph) + 180)) != 0)[0]
        return ((180 + np.degrees(ph[i[-1]])) % 360 if len(i) else np.nan,
                -20 * np.log10(mag[j[0]]) if len(j) else np.inf)
    return pm_gm(Lv) + pm_gm(Lp)


def move(g, vmax, ff=True, amax=2.0, jmax=8.0, kff=0.0, tau=TAU, delay=DELAY, kd=KDRAG, tail=4.0):
    """Rest-to-rest move: max behind, cruise lag (at the start of braking), max past the goal,
    peak speed, error 1 s and 3 s after the reference stops, peak acceleration command."""
    t, P, V, A, vc, tstop = jerk_limited_move(max(12.0, vmax * 4 + vmax ** 2 / amax), vmax, amax, jmax, DT, tail)
    p, v, a, vsp, acc = run(t, P, V, (A + kff * V) if ff else None, g, tau=tau, delay=delay, kdrag=kd)
    e = p[:, 0] - P[:, 0]
    idec = np.argmax(A[:, 0] < -1e-6)
    after = lambda s: e[min(len(e) - 1, np.argmax(t >= tstop + s))]  # noqa: E731
    return dict(behind=-e[t <= tstop].min(), cruise=-e[idec], past=(p[t > tstop, 0] - P[-1, 0]).max(),
                vpk=v[:, 0].max(), vc=vc, e1=after(1.0), e3=after(3.0), acmd=np.abs(acc[:, 0]).max())


def steady_disturbance(g, amp=1.0):
    t = np.arange(0, 12, DT)
    Z = np.zeros((len(t), 2))
    p, *_ = run(t, Z, Z, None, g, dist=[amp, 0])
    x = np.abs(p[:, 0])
    return x.max(), x[np.argmax(t >= 2)], x[np.argmax(t >= 5)]


def main(which):
    print(f'Model: tau {TAU} s, delay {DELAY*1000:.0f} ms, drag {KDRAG} 1/s, 50 Hz, tilt limit 45 deg\n')
    if 'steps' in which:
        print('STEPS: velocity setpoint step 0 -> 2 m/s (Direct velocity), adopted plant')
        print(f'  {"gains":26s} rise10-90 overshoot settle5% peak-acc-cmd | inner PM/GM | position PM/GM')
        for g in SETS:
            r, o, ts, am = vel_step(g)
            pv, gv, pp, gp = margins(g)
            print(f'  {g.name:26s} {r:5.2f} s  {o*100:5.1f} %  {ts:5.2f} s  {am:5.2f} m/s2 | {pv:4.0f} deg {gv:4.1f} dB | {pp:4.0f} deg {gp:4.1f} dB')
    if 'margins' in which:
        print('\nMARGINS across the plant bracket: inner PM / position PM / step overshoot')
        print('  ' + f'{"gains":26s}' + ''.join(f' | {n:20s}' for n, _, _ in PLANTS))
        for g in SETS:
            row = []
            for n, tau, dl in PLANTS:
                pv, gv, pp, gp = margins(g, tau=tau, delay=dl)
                row.append(f'{pv:3.0f} / {pp:3.0f} / {vel_step(g, tau=tau, delay=dl)[1]*100:4.1f}%')
            print(f'  {g.name:26s}' + ''.join(f' | {x:20s}' for x in row))
    if 'moves' in which:
        print('\nMOVES: rest-to-rest, max_acc 2.0, jerk 8. behind / cruise lag / past goal / peak speed')
        for vmax in (2.0, 3.0, 4.0):
            print(f'  {vmax} m/s')
            r = move(SETS[0], vmax, ff=False)
            print(f'    {"stock, feedforward OFF":34s} {r["behind"]:.2f} / {r["cruise"]:+.2f} / {r["past"]:.2f} / {r["vpk"]:.2f}')
            for g in SETS:
                r = move(g, vmax)
                print(f'    {g.name + ", feedforward ON":34s} {r["behind"]:.2f} / {r["cruise"]:+.2f} / {r["past"]:.2f} / {r["vpk"]:.2f}  acc cmd {r["acmd"]:.2f}')
    if 'disturbance' in which:
        print('\nDISTURBANCE: a constant 1 m/s2 push from hover. peak / after 2 s / after 5 s [m]')
        for g in SETS:
            print(f'  {g.name:26s} ' + ' / '.join(f'{x:.2f}' for x in steady_disturbance(g)))
    if 'replay' in which:
        from model_validate import reference
        from offboard_score import event_metrics
        print('\nREPLAY: the flown SUPER references, feedforward from pos_cmd acceleration')
        for b in BAGS:
            t, P, V, A, pm, vm, R = reference(b, ff=True)
            a0, a1, name = EVENTS[b]
            w = R['win'][0]
            m = event_metrics(t, P, V, pm, vm, w + a0, w + a1)
            print(f'  {b} {name}: MEASURED (no feedforward) behind {m["behind"]:.2f} ahead {m["ahead"]:.2f} peak {m["vpk"]:.2f}')
            for ff, g in [(False, SETS[0])] + [(True, g) for g in SETS[:7]]:
                p, v, *_ = run(t, P, V, A if ff else None, g, p0=pm[0], v0=vm[0])
                m = event_metrics(t, P, V, p, v, w + a0, w + a1)
                e = np.hypot(*(p - P).T)
                print(f'    {"FF ON " if ff else "FF OFF"} {g.name:26s} behind {m["behind"]:.2f} ahead {m["ahead"]:+.2f} peak {m["vpk"]:.2f}'
                      f' | window error rms {np.sqrt(np.mean(e**2)):.2f} p95 {np.percentile(e,95):.2f} max {e.max():.2f}')
    if 'tail' in which:
        print('\nTAIL: 3 m/s move, error 1 s and 3 s after the reference stops (drag carried by the integrator)')
        for g in SETS[:7]:
            r = move(g, 3.0, tail=8.0)
            print(f'  {g.name:26s} behind {r["behind"]:.2f} cruise {r["cruise"]:+.2f} past {r["past"]:.2f} | +1 s {r["e1"]:+.2f} +3 s {r["e3"]:+.2f}')
    if 'dragff' in which:
        print('\nDRAG FEEDFORWARD (not built): acc_ff = a_ref + 0.3*v_ref, true drag 0.2 / 0.3 / 0.4')
        for vmax in (3.0, 4.0):
            for kd in (0.2, 0.3, 0.4):
                a = move(SETS[0], vmax, kd=kd, tail=8.0)
                b = move(SETS[0], vmax, kff=0.3, kd=kd, tail=8.0)
                c = move(TUNED, vmax, kff=0.3, kd=kd, tail=8.0)
                print(f'  {vmax} m/s drag {kd}: FF stock {a["behind"]:.2f}/{a["cruise"]:+.2f}/{a["past"]:.2f} | FF+drag stock'
                      f' {b["behind"]:.2f}/{b["cruise"]:+.2f}/{b["past"]:.2f} | FF+drag tuned {c["behind"]:.2f}/{c["past"]:.2f}  (behind/cruise/past)')
    if 'sensitivity' in which:
        print('\nSENSITIVITY: 3 m/s move, feedforward on, behind / past, stock vs 0.95/3.0/0.6/0.10')
        for n, tau, dl in PLANTS:
            for kd in (0.2, 0.4):
                s = move(SETS[0], 3.0, tau=tau, delay=dl, kd=kd)
                u = move(TUNED, 3.0, tau=tau, delay=dl, kd=kd)
                print(f'  {n:20s} drag {kd}: stock {s["behind"]:.2f}/{s["past"]:.2f}  tuned {u["behind"]:.2f}/{u["past"]:.2f}')
    if 'ladder' in which:
        print('\nLADDER: what each tuning flight should show on a 2 m/s Direct-velocity step')
        ladder = [('V0 setup (I 0.2)', Gains(0.95, 1.8, 0.2, 0.20)), ('V1 D 0.1', Gains(0.95, 1.8, 0.2, 0.10)),
                  ('V2 P 2.2', Gains(0.95, 2.2, 0.2, 0.10)), ('V3 P 2.6', Gains(0.95, 2.6, 0.2, 0.10)),
                  ('V4 P 3.1', Gains(0.95, 3.1, 0.2, 0.10)), ('V5 P 3.7', Gains(0.95, 3.7, 0.2, 0.10)),
                  ('V6 I 0.6 at P 3.1', Gains(0.95, 3.1, 0.6, 0.10)), ('as flown 1.8/0.4/0.2', SETS[0])]
        print(f'  {"step":22s} rise  overshoot settle5% | slow plant: rise overshoot | inner PM adopted/slow | acc cmd -> tilt')
        for n, g in ladder:
            r, o, ts, am = vel_step(g)
            r2, o2, _, _ = vel_step(g, tau=0.22, delay=0.08)
            print(f'  {n:22s} {r:4.2f} s {o*100:4.1f} % {ts:5.2f} s | {r2:4.2f} s {o2*100:4.1f} % |'
                  f' {margins(g)[0]:3.0f}/{margins(g, tau=0.22, delay=0.08)[0]:3.0f} deg | {am:.1f} m/s2 -> {np.degrees(np.arctan(am/9.81)):.0f} deg')
    if 'jumps' in which:
        print('\nJUMPS: first-sample acceleration command for a sudden position-setpoint jump (tilt limit 45 deg = 9.8 m/s2)')
        for g in (SETS[0], TUNED):
            print(f'  {g.name:26s} ' + ', '.join(f'{j} m -> {g.vp*g.kp*j:.1f} m/s2 ({np.degrees(np.arctan(min(g.vp*g.kp*j, 9.81)/9.81)):.0f} deg)' for j in (0.5, 1.0, 3.0)))


if __name__ == '__main__':
    main(sys.argv[1:] or ['steps', 'margins', 'moves', 'disturbance', 'replay', 'tail', 'dragff', 'sensitivity', 'ladder', 'jumps'])
