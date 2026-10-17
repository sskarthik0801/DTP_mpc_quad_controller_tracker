"""PX4 v1.17 horizontal PositionControl on a fitted plant (scratch model, 2026-09-27).

Controller, from src/modules/mc_pos_control/PositionControl/PositionControl.cpp (d6f12ad):
    vsp  = vel_ff + KP*(p_sp - p)                     (MPC_XY_VEL_MAX 12 is never reached here)
    acc  = acc_ff + VP*(vsp - v) + I - VD*LPF_5Hz(dv/dt)
    tilt limit by normalising (-acc_x, -acc_y, g) to MPC_TILTMAX_AIR (45 deg)
    anti-windup when saturated: ev -= (2/VP)*(acc - acc_produced); I += VI*ev*dt
    run at 50 Hz with a zero-order hold.
Plant, per horizontal axis:
    a_thrust = gain * FOPDT(acc_produced; tau, delay)    attitude loop (fit_attitude.py, fit_offboard_acc.py)
    a        = a_thrust - kdrag*v + dist                  rotor drag (fit_drag.py)
Adopted: tau 0.17 s, delay 0.05 s, gain 0.95, kdrag 0.30 1/s. No wind, perfect velocity estimate.
"""
import numpy as np

G = 9.80665


class Gains:
    def __init__(self, kp=0.95, vp=1.8, vi=0.4, vd=0.2, name=None):
        self.kp, self.vp, self.vi, self.vd = kp, vp, vi, vd
        self.name = name or f'{kp}/{vp}/{vi}/{vd}'


STOCK = Gains(0.95, 1.8, 0.4, 0.2, 'stock 0.95/1.8/0.4/0.20')
TAU, DELAY, GAIN, KDRAG = 0.17, 0.05, 0.95, 0.30


def run(t, P, V, A, g=STOCK, tau=TAU, delay=DELAY, gain=GAIN, kdrag=KDRAG, tilt_deg=45.0,
        ctrl_hz=50.0, dt=0.001, pos_ctrl=True, p0=None, v0=None, veld_lp=5.0, dist=None):
    """t, P, V, A: reference sampled every dt (N x 2). A None = no acceleration feedforward.
    Returns p, v, a, vsp, acc_produced (N x 2 each)."""
    n = len(t)
    p = (P[0] if p0 is None else p0).astype(float).copy()
    v = np.zeros(2) if v0 is None else np.asarray(v0, float).copy()
    ath = np.zeros(2)
    I = np.zeros(2)
    dist = np.zeros(2) if dist is None else np.asarray(dist, float)
    buf = [np.zeros(2) for _ in range(max(0, int(round(delay / dt))) + 1)]
    every = max(1, int(round(1.0 / (ctrl_hz * dt))))
    dtc = every * dt
    alpha_d = dtc / (dtc + 1.0 / (2 * np.pi * veld_lp))
    vprev = v.copy()
    vdot_f = np.zeros(2)
    acc_out = np.zeros(2)
    vsp = np.zeros(2)
    amax = G * np.tan(np.radians(tilt_deg))
    out = [np.zeros((n, 2)) for _ in range(5)]
    for k in range(n):
        if k % every == 0:
            vdot = (v - vprev) / dtc
            vprev = v.copy()
            vdot_f = vdot_f + alpha_d * (vdot - vdot_f)
            vsp = V[k] + (g.kp * (P[k] - p) if pos_ctrl else 0.0)
            ev = vsp - v
            acc = (A[k] if A is not None else 0.0) + g.vp * ev + I - g.vd * vdot_f
            nrm = np.linalg.norm(acc)
            prod = acc * (amax / nrm) if nrm > amax else acc.copy()
            if nrm > amax:
                ev = ev - (2.0 / g.vp) * (acc - prod)
            I = I + g.vi * ev * dtc
            acc_out = prod
        buf.append(acc_out.copy())
        cmd = buf.pop(0)
        ath = ath + (cmd - ath) * dt / tau
        a = gain * ath - kdrag * v + dist
        v = v + a * dt
        p = p + v * dt
        for o, x in zip(out, (p, v, a, vsp, acc_out)):
            o[k] = x
    return tuple(out)


def jerk_limited_move(dist, vmax, amax, jmax=8.0, dt=0.001, tail=4.0):
    """1-D rest-to-rest S-curve along x (what a SUPER plan to a goal looks like).
    Returns t, P, V, A (N x 2), the cruise speed reached, and the time the reference stops."""
    def accel(v_target):
        tj = amax / jmax
        if v_target < amax * tj:
            tj = np.sqrt(v_target / jmax)
            hold = 0.0
        else:
            hold = (v_target - amax * tj) / amax
        return [(tj, jmax), (hold, 0.0), (tj, -jmax)]

    def integrate(seg):
        a = v = x = 0.0
        for d, j in seg:
            x += v * d + 0.5 * a * d * d + j * d ** 3 / 6
            v += a * d + 0.5 * j * d * d
            a += j * d
        return x

    lo, hi = 0.0, vmax
    for _ in range(60):
        vc = 0.5 * (lo + hi)
        if 2 * integrate(accel(vc)) > dist:
            hi = vc
        else:
            lo = vc
    vc = lo
    seg = accel(vc)
    cruise = max(0.0, (dist - 2 * integrate(seg)) / vc) if vc > 0 else 0.0
    full = seg + [(cruise, 0.0)] + [(d, -j) for d, j in seg]
    t_stop = sum(d for d, _ in full)
    t = np.arange(0, t_stop + tail, dt)
    jt = np.zeros_like(t)
    e0 = 0.0
    for d, j in full:
        jt[(t >= e0) & (t < e0 + d)] = j
        e0 += d
    a = np.cumsum(jt) * dt
    v = np.cumsum(a) * dt
    x = np.cumsum(v) * dt
    z = np.zeros_like(t)
    return t, np.c_[x, z], np.c_[v, z], np.c_[a, z], vc, t_stop
