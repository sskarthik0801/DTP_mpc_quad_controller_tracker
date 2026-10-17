#!/usr/bin/env python3
"""Linear MPC that tracks SUPER's trajectory by commanding ACCELERATION. No ROS.

This is the maths behind `px4_mpc_tracker.py`. It lives in its own file, with only
numpy, so the offline simulator (`scripts/px4/mpc_tracker_sim.py`) can test the exact
same code the drone runs.

Where it sits
-------------
    SUPER  --/planning/pos_cmd-->  [ THIS MPC ]  --acceleration setpoint-->  PX4
                                       ^                                       |
                                       +----- /fmu/out/vehicle_local_position -+

The stock bridge sends PX4 a POSITION setpoint and lets PX4's PID cascade
(position P -> velocity PID -> acceleration) do the tracking. This replaces the outer
two PX4 loops: we send an ACCELERATION setpoint, and PX4 only turns it into tilt and
thrust (attitude, rate and motor control stay PX4's).

The model -- the same 1D drone as the mpc_learn lessons, once per axis
--------------------------------------------------------------------
Each axis (x, y, z in PX4 NED) is

    p[k+1] = p[k] + h*v[k] + h^2/2 * (a[k] + d)
    v[k+1] = v[k] + h*(a[k] + d)
    a[k+1] = alpha*a[k] + (1 - alpha)*u[k]          alpha = exp(-h / tau)

  u   the acceleration we command (the thing the MPC chooses)
  a   the acceleration the airframe actually has. It lags u by `tau` because the
      aircraft has to TILT before it accelerates sideways.
  d   a constant unknown acceleration: wind, a wrong hover thrust, a tilted mount.
      Estimated online (`DisturbanceObserver`), which is what removes steady offsets.

Every stage of the horizon costs

    qp*(p - p_ref)^2 + qv*(v - v_ref)^2 + r*(u - (a_ref - d))^2 + rdu*(u - u_prev)^2

and the last stage's position/velocity terms are multiplied by `terminal`.

The constraints are on u only: a disc of radius `a_xy_max` horizontally (that is a
tilt limit) and [-a_up_max, +a_down_max] vertically (NED: up is negative).

Solving it
----------
Because the model is linear and the cost quadratic, the whole horizon collapses into
one quadratic program in the N inputs per axis:

    minimise 0.5 * U'HU + G'U     subject to  U in the disc/box set

H does not change between ticks, so it is built once. G depends on the state and the
reference and is rebuilt every tick (a few small matrix products). We first try the
unconstrained answer -H^-1 G; if it already respects the limits it IS the solution.
Otherwise we run accelerated projected gradient (FISTA), warm-started from last tick's
plan shifted by one stage. No solver library needed -- numpy only, which matters
because nothing beyond numpy is installed on the Orin.
"""
import math

import numpy as np


class TrackerParams:
    """Every tuning number in one place. Defaults are for the S2 envelope (1 m/s, 1 m/s^2)."""

    def __init__(self, **kw):
        self.h = 0.05             # s, MPC model step
        self.N = 20               # stages -> 1.0 s horizon
        self.tau = 0.12           # s, accel lag (tilt response). Measure it from a log.
        self.qp = 10.0            # position weight
        self.qv = 3.0             # velocity weight
        self.r = 0.5              # weight on (u - feedforward)
        self.rdu = 2.0            # weight on change of u between stages (smoothness)
        self.terminal = 5.0       # extra weight on the last stage
        self.a_xy_max = 2.5       # m/s^2 horizontal  (~14 deg of tilt)
        self.a_up_max = 2.0       # m/s^2 upward
        self.a_down_max = 1.5     # m/s^2 downward
        self.max_pos_err = 1.0    # m, a reference further than this is pulled in
        self.v_ref_max = 1.5      # m/s, reference velocity cap
        self.acc_extrap_s = 0.3   # s, how long the reference's accel is extrapolated
        self.d_time_const = 0.6   # s, disturbance observer time constant
        self.d_max = 1.5          # m/s^2, disturbance estimate clamp
        self.max_iters = 60       # FISTA iterations when the limits are active
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(f'unknown MPC parameter {k!r}')
            setattr(self, k, v)


class AccelMPC:
    """Condensed 3-axis linear MPC. Axes share H; x/y are coupled only by the tilt disc."""

    def __init__(self, prm: TrackerParams):
        self.prm = prm
        h, N = prm.h, prm.N
        alpha = math.exp(-h / prm.tau)
        A = np.array([[1.0, h, 0.5 * h * h],
                      [0.0, 1.0, h],
                      [0.0, 0.0, alpha]])
        B = np.array([0.0, 0.0, 1.0 - alpha])
        E = np.array([0.5 * h * h, h, 0.0])

        # Stacked prediction X = Phi x0 + Gam U + The d, stages 1..N, rows [p, v, a] per stage.
        Phi = np.zeros((3 * N, 3))
        Gam = np.zeros((3 * N, N))
        The = np.zeros((3 * N, 1))
        Ak = np.eye(3)
        for k in range(N):
            Ak = A @ Ak                                   # A^(k+1)
            Phi[3 * k:3 * k + 3] = Ak
        for k in range(N):                                # stage k+1
            Apow = np.eye(3)
            for j in range(k, -1, -1):                    # input j acts through A^(k-j)
                Gam[3 * k:3 * k + 3, j] = Apow @ B
                The[3 * k:3 * k + 3, 0] += Apow @ E
                Apow = A @ Apow

        q = np.tile([prm.qp, prm.qv, 0.0], N)
        q[-3:-1] *= prm.terminal
        Q = np.diag(q)

        # First-difference operator for (u_k - u_{k-1}); row 0 is u_0 - u_prev.
        D = np.eye(N) - np.eye(N, k=-1)

        self.Phi, self.Gam, self.The = Phi, Gam, The
        self.GtQ = Gam.T @ Q
        self.D = D
        self.H = Gam.T @ Q @ Gam + prm.r * np.eye(N) + prm.rdu * D.T @ D
        self.Hinv = np.linalg.inv(self.H)
        self.L = float(np.linalg.eigvalsh(self.H).max())

        self.U_prev = None                                # last plan, (N, 3)
        self.last_iters = 0

    # -- constraints --------------------------------------------------------

    def project(self, U):
        prm = self.prm
        U = U.copy()
        n = np.hypot(U[:, 0], U[:, 1])
        over = n > prm.a_xy_max
        if np.any(over):
            U[over, :2] *= (prm.a_xy_max / n[over])[:, None]
        # NED: up is negative z.
        np.clip(U[:, 2], -prm.a_up_max, prm.a_down_max, out=U[:, 2])
        return U

    def _feasible(self, U):
        prm = self.prm
        return (np.all(np.hypot(U[:, 0], U[:, 1]) <= prm.a_xy_max + 1e-9)
                and np.all(U[:, 2] >= -prm.a_up_max - 1e-9)
                and np.all(U[:, 2] <= prm.a_down_max + 1e-9))

    # -- solve --------------------------------------------------------------

    def solve(self, x0, d, Xref, Uref, u_prev):
        """One MPC solve.

        x0     (3, 3)  columns = axes x, y, z; rows = [p, v, a]
        d      (3,)    disturbance estimate per axis
        Xref   (3N, 3) stacked [p_ref, v_ref, a_ref] for stages 1..N, per axis
        Uref   (N, 3)  input reference per stage (feedforward accel minus d)
        u_prev (3,)    the command actually sent last tick
        Returns the plan U (N, 3); U[0] is what to send now.
        """
        prm = self.prm
        free = self.Phi @ x0 + self.The @ d[None, :] - Xref          # (3N, 3)
        G = self.GtQ @ free - prm.r * Uref                            # (N, 3)
        G[0] -= prm.rdu * u_prev                                      # D' e u_prev, e = [1,0..]

        U = -self.Hinv @ G
        if self._feasible(U):
            self.last_iters = 0
            self.U_prev = U
            return U

        # Warm start: last plan shifted one stage, else the clipped unconstrained answer.
        if self.U_prev is not None:
            y = np.vstack([self.U_prev[1:], self.U_prev[-1:]])
        else:
            y = U
        y = self.project(y)
        U_old = y
        t = 1.0
        for i in range(prm.max_iters):
            U_new = self.project(y - (self.H @ y + G) / self.L)
            t_new = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * t * t))
            y = U_new + ((t - 1.0) / t_new) * (U_new - U_old)
            if np.max(np.abs(U_new - U_old)) < 1e-5:
                U_old = U_new
                break
            U_old, t = U_new, t_new
        self.last_iters = i + 1
        self.U_prev = U_old
        return U_old

    def reset(self):
        self.U_prev = None


class DisturbanceObserver:
    """Estimates the acceleration `a` the airframe has (it lags u) and the unknown `d`.

    a is propagated open-loop from the commands we sent. d integrates the velocity
    prediction error: if the aircraft keeps ending up slower than the model said,
    something is pushing it, and d grows until the model agrees with reality.
    """

    def __init__(self, prm: TrackerParams):
        self.prm = prm
        self.reset()

    def reset(self):
        self.a_hat = np.zeros(3)
        self.d_hat = np.zeros(3)
        self.v_pred = None

    def update(self, v_meas, dt, frozen=False):
        """Fold in a new velocity measurement. Call BEFORE solving this tick."""
        if self.v_pred is not None and not frozen and dt > 0.0:
            innov = v_meas - self.v_pred
            self.d_hat += innov / self.prm.d_time_const
            np.clip(self.d_hat, -self.prm.d_max, self.prm.d_max, out=self.d_hat)

    def predict(self, v_meas, u_sent, dt):
        """Propagate to the next tick with the command we just sent. Call AFTER sending."""
        self.v_pred = v_meas + dt * (self.a_hat + self.d_hat)
        beta = 1.0 - math.exp(-dt / self.prm.tau)
        self.a_hat = self.a_hat + beta * (u_sent - self.a_hat)


def build_reference(prm: TrackerParams, p_now, ref_p, ref_v, ref_a, age=0.0):
    """Turn one SUPER command (a point on its trajectory) into a horizon reference.

    SUPER's pos_cmd is a single sample: position, velocity, acceleration at "now".
    We extrapolate it forward: constant acceleration for `acc_extrap_s`, then constant
    velocity. `age` is how old the command already is.

    Safety: if SUPER's point for "now" is further than `max_pos_err` from where the
    aircraft IS, the WHOLE horizon is shifted towards the aircraft until it is that far.
    That turns a far setpoint (e.g. the 3 m pos_cmd at the 2026-09-21 engage) into a
    bounded approach instead of a lunge. Only the present offset is clamped: clamping
    every stage would also bend a normal trajectory, whose 1 s-ahead point is ~1 m away.

    Returns Xref (3N, 3) and the per-stage reference accel (N, 3), both NED.
    """
    N, h = prm.N, prm.h
    Xref = np.zeros((3 * N, 3))
    Aref = np.zeros((N, 3))
    vmax = prm.v_ref_max
    e0 = ref_p + ref_v * age - p_now
    n0 = np.linalg.norm(e0)
    shift = np.zeros(3)
    if n0 > prm.max_pos_err:
        shift = e0 * (prm.max_pos_err / n0 - 1.0)
    for k in range(N):
        t = age + (k + 1) * h
        ta = min(t, prm.acc_extrap_s)
        p = ref_p + ref_v * t + ref_a * (ta * t - 0.5 * ta * ta)
        v = ref_v + ref_a * ta
        a = ref_a if t <= prm.acc_extrap_s else np.zeros(3)
        nv = np.linalg.norm(v)
        if nv > vmax:
            v = v * (vmax / nv)
        Xref[3 * k] = p + shift
        Xref[3 * k + 1] = v
        Xref[3 * k + 2] = a
        tu = age + k * h                                   # input k acts from stage k
        Aref[k] = ref_a if tu <= prm.acc_extrap_s else 0.0
    return Xref, Aref


class Tracker:
    """Observer + reference + MPC, glued together. The ROS node calls only this."""

    def __init__(self, prm: TrackerParams = None):
        self.prm = prm or TrackerParams()
        self.mpc = AccelMPC(self.prm)
        self.obs = DisturbanceObserver(self.prm)
        self.u_prev = np.zeros(3)

    def reset(self):
        self.mpc.reset()
        self.obs.reset()
        self.u_prev = np.zeros(3)

    def step(self, p, v, ref_p, ref_v, ref_a, dt, ref_age=0.0, freeze_obs=False):
        """p, v: measured NED state. ref_*: SUPER's command in NED. Returns u (3,) NED."""
        p = np.asarray(p, float)
        v = np.asarray(v, float)
        self.obs.update(v, dt, frozen=freeze_obs)
        Xref, Aref = build_reference(self.prm, p, np.asarray(ref_p, float),
                                     np.asarray(ref_v, float), np.asarray(ref_a, float),
                                     ref_age)
        x0 = np.vstack([p, v, self.obs.a_hat])                       # (3, 3)
        d = self.obs.d_hat
        U = self.mpc.solve(x0, d, Xref, Aref - d[None, :], self.u_prev)
        u = U[0].copy()
        if not np.all(np.isfinite(u)):
            raise FloatingPointError('MPC produced a non-finite command')
        self.u_prev = u
        return u

    def after_send(self, v, u, dt_next):
        self.obs.predict(np.asarray(v, float), u, dt_next)
