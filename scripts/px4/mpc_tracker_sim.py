#!/usr/bin/env python3
"""Offline bench gate for the MPC tracker. No ROS, no flight controller, numpy only.

Flies `super_bringup/scripts/mpc_tracker_core.py` -- the exact code the drone runs --
against a simulated quadrotor that is deliberately NOT the MPC's model:

  * the true acceleration lag is 0.16 s (the MPC assumes 0.12 s),
  * there is a further 20 ms pure delay (DDS + uORB + attitude loop),
  * wind / thrust-model error: a constant (0.6, -0.4, +0.5) m/s^2 push,
  * the position/velocity the MPC sees are noisy.

For comparison it also flies a copy of PX4's own cascade (position P 0.95 -> velocity
PID 1.8/0.4/0.2, with SUPER's velocity as feedforward) -- roughly what the stock
px4_offboard_bridge gets today.

    python3 scripts/px4/mpc_tracker_sim.py            # the gate, prints PASS/FAIL
    python3 scripts/px4/mpc_tracker_sim.py --plot     # also writes mpc_tracker_sim.png

Run it after ANY change to mpc_tracker_core.py or to its parameters.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'super_bringup', 'scripts'))
from mpc_tracker_core import Tracker, TrackerParams  # noqa: E402

CTRL_DT = 0.02          # 50 Hz, the node's publish_rate
SIM_DT = 0.001
TAU_TRUE = 0.16
DELAY = 0.020
WIND = np.array([0.6, -0.4, 0.5])     # NED; +z is a thrust shortfall (pushes down)
NOISE_P, NOISE_V = 0.01, 0.03


class Plant:
    """3-axis point mass with a lagged, delayed acceleration response."""

    def __init__(self, p0):
        self.p = np.array(p0, float)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.queue = [np.zeros(3)] * int(round(DELAY / SIM_DT))

    def run(self, u, duration):
        for _ in range(int(round(duration / SIM_DT))):
            self.queue.append(u)
            u_eff = self.queue.pop(0)
            self.a += (SIM_DT / TAU_TRUE) * (u_eff - self.a)
            acc = self.a + WIND
            self.p += SIM_DT * self.v + 0.5 * SIM_DT ** 2 * acc
            self.v += SIM_DT * acc


class Px4Cascade:
    """PX4 mc_pos_control, roughly: P on position, PID on velocity, outputs accel."""

    def __init__(self):
        self.i = np.zeros(3)
        self.v_err_prev = np.zeros(3)

    def step(self, p, v, rp, rv, ra, dt):
        v_sp = 0.95 * (rp - p) + rv
        v_sp[2] = 1.0 * (rp[2] - p[2]) + rv[2]
        e = v_sp - v
        self.i += e * dt
        self.i = np.clip(self.i, -2, 2)
        de = (e - self.v_err_prev) / dt
        self.v_err_prev = e
        u = 1.8 * e + 0.4 * self.i + 0.2 * de
        u[2] = 4.0 * e[2] + 2.0 * self.i[2]
        n = math.hypot(u[0], u[1])
        if n > 2.5:
            u[:2] *= 2.5 / n
        u[2] = min(max(u[2], -2.0), 1.5)
        return u


# ---- references (what SUPER's pos_cmd would carry), NED ------------------------

def figure_eight(t):
    """Lemniscate, 1.1 m/s peak (SUPER max_vel is 1.0) at 1.5 m altitude, with a 0.5 m height wave."""
    w = 2 * math.pi / 20.0
    A = 2.5
    p = np.array([A * math.sin(w * t), A * math.sin(2 * w * t) / 2,
                  -1.5 - 0.25 * math.sin(w * t)])
    v = np.array([A * w * math.cos(w * t), A * w * math.cos(2 * w * t),
                  -0.25 * w * math.cos(w * t)])
    a = np.array([-A * w * w * math.sin(w * t), -2 * A * w * w * math.sin(2 * w * t),
                  0.25 * w * w * math.sin(w * t)])
    return p, v, a


def hold(point):
    point = np.array(point, float)
    return lambda t: (point, np.zeros(3), np.zeros(3))


def fly(controller, ref, p0, T, seed=0):
    rng = np.random.default_rng(seed)
    plant = Plant(p0)
    log = {'t': [], 'p': [], 'v': [], 'u': [], 'ref': [], 'solve_ms': []}
    t = 0.0
    while t < T:
        pm = plant.p + rng.normal(0, NOISE_P, 3)
        vm = plant.v + rng.normal(0, NOISE_V, 3)
        rp, rv, ra = ref(t)
        t0 = time.perf_counter()
        if isinstance(controller, Tracker):
            u = controller.step(pm, vm, rp, rv, ra, CTRL_DT)
            controller.after_send(vm, u, CTRL_DT)
        else:
            u = controller.step(pm, vm, rp, rv, ra, CTRL_DT)
        log['solve_ms'].append(1e3 * (time.perf_counter() - t0))
        for k, val in (('t', t), ('p', plant.p.copy()), ('v', plant.v.copy()),
                       ('u', u.copy()), ('ref', rp.copy())):
            log[k].append(val)
        plant.run(u, CTRL_DT)
        t += CTRL_DT
    return {k: np.array(v) for k, v in log.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args()

    prm = TrackerParams()
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f'  [{"PASS" if ok else "FAIL"}] {name}: {detail}')

    print('S1 hover in wind (ref fixed at 1.5 m up, plant starts there)')
    trk = Tracker(prm)
    L = fly(trk, hold([0, 0, -1.5]), [0, 0, -1.5], 15.0)
    err = np.linalg.norm(L['p'] - L['ref'], axis=1)
    late = err[L['t'] > 8.0]
    check('steady error after 8 s', late.max() < 0.05, f'max {late.max() * 100:.1f} cm (< 5 cm)')
    check('wind estimate', np.allclose(trk.obs.d_hat, WIND, atol=0.15),
          f'd_hat {np.round(trk.obs.d_hat, 2)} vs true {WIND}')

    print('S2 figure-eight, 1.1 m/s peak, 40 s')
    L2 = fly(Tracker(prm), figure_eight, figure_eight(0)[0], 40.0)
    e2 = np.linalg.norm(L2['p'] - L2['ref'], axis=1)[L2['t'] > 5.0]
    rms2 = math.sqrt(np.mean(e2 ** 2))
    check('tracking RMS', rms2 < 0.10, f'{rms2 * 100:.1f} cm (< 10 cm), max {e2.max() * 100:.1f} cm')
    Lc = fly(Px4Cascade(), figure_eight, figure_eight(0)[0], 40.0)
    ec = np.linalg.norm(Lc['p'] - Lc['ref'], axis=1)[Lc['t'] > 5.0]
    print(f'         (for comparison, PX4-style cascade: RMS {math.sqrt(np.mean(ec ** 2)) * 100:.1f} cm,'
          f' max {ec.max() * 100:.1f} cm)')

    print('S3 far setpoint: reference 3 m away at engage (the 2026-09-21 case)')
    L3 = fly(Tracker(prm), hold([3, 0, -1.5]), [0, 0, -1.5], 10.0)
    spd = np.linalg.norm(L3['v'], axis=1)
    over = L3['p'][:, 0].max() - 3.0
    check('approach speed bounded', spd.max() < 1.6, f'peak {spd.max():.2f} m/s (< 1.6)')
    check('no big overshoot', over < 0.15, f'{over * 100:.1f} cm past the goal (< 15 cm)')

    print('S4 limits and cost')
    U = np.vstack([L['u'], L2['u'], L3['u']])
    xy = np.hypot(U[:, 0], U[:, 1]).max()
    check('tilt disc', xy <= prm.a_xy_max + 1e-6, f'max |a_xy| {xy:.2f} <= {prm.a_xy_max}')
    check('vertical box', U[:, 2].min() >= -prm.a_up_max - 1e-6 and U[:, 2].max() <= prm.a_down_max + 1e-6,
          f'a_z in [{U[:, 2].min():.2f}, {U[:, 2].max():.2f}]')
    ms = np.concatenate([L['solve_ms'], L2['solve_ms'], L3['solve_ms']])
    p99 = np.percentile(ms, 99)
    check('solve time p99', p99 < 5.0, f'{p99:.2f} ms (< 5 ms; this PC, the Orin is ~3x slower)')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
        ax[0].plot(L2['ref'][:, 0], -L2['ref'][:, 1], 'k--', label='SUPER reference')
        ax[0].plot(L2['p'][:, 0], -L2['p'][:, 1], label='MPC')
        ax[0].plot(Lc['p'][:, 0], -Lc['p'][:, 1], alpha=0.6, label='PX4 cascade')
        ax[0].set_title('S2 figure-eight (top view, FLU)'); ax[0].axis('equal'); ax[0].legend()
        ax[1].plot(L2['t'], np.linalg.norm(L2['p'] - L2['ref'], axis=1), label='MPC')
        ax[1].plot(Lc['t'], np.linalg.norm(Lc['p'] - Lc['ref'], axis=1), label='PX4 cascade')
        ax[1].set_title('S2 tracking error [m]'); ax[1].set_xlabel('t [s]'); ax[1].legend()
        ax[2].plot(L3['t'], L3['p'][:, 0], label='x [m]')
        ax[2].plot(L3['t'], spd, label='speed [m/s]')
        ax[2].set_title('S3 far setpoint (goal x = 3 m)'); ax[2].set_xlabel('t [s]'); ax[2].legend()
        out = os.path.join(os.getcwd(), 'mpc_tracker_sim.png')
        fig.tight_layout(); fig.savefig(out, dpi=110)
        print(f'plot: {out}')

    ok = all(results)
    print(f'\n{"PASS" if ok else "FAIL"} ({sum(results)}/{len(results)})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
