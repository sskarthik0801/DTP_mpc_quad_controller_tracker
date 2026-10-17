#!/usr/bin/env python3
"""The camera yaw servo's replay gate (docs/plan-camera-yaw-servo.md SS 4.3). Read-only on FINISHED bags
(never one still recording: reading a .db3 under `ros2 bag record` kills the recorder).

    python3 drone-dtp/scripts/fix1_yaw_replay.py <bag> [<bag> ...]    # the full report per bag
    python3 drone-dtp/scripts/fix1_yaw_replay.py --gate               # SS 4.3 on the three bags in <repo>/bags
    python3 drone-dtp/scripts/fix1_yaw_replay.py --study              # min_range_m x stale_hold_s, and the ranges flown

Round two (2026-09-29): the module's defaults now include the 0.5 s stale hold and the 0.3 m close-range
hysteresis, and its fall-back sends SUPER's rate only when the slew clamp did not clip (clamped to 90 deg/s).
B / C below are the module at its defaults. --gate keeps the plan's SS 4.3 criteria verbatim and counts only
those; it then prints restated readings of the two rate criteria (the commanded heading over 0.2 s and the
modelled vehicle rate while the target is fresh, each beside the person's own bearing rate, and the modelled
vehicle rate over the whole window <= 90 deg/s), marked as not the plan's and not counted.

1. It steps the REAL servo, super_bringup/scripts/yaw_servo.py (imported by path; the file is printed), tick
   by tick at 50 Hz. There is no second copy of the algorithm here: every heading below is the module's output.
2. The servo's inputs are what px4_offboard_bridge.py would have held at each tick, rebuilt CAUSALLY from rows
   RECEIVED at or before the tick (bag receive time: the host clock, which is ROS time on this stack):
   - engaged = armed and Offboard, from /fmu/out/vehicle_control_mode (else vehicle_status_v1: nav_state 14,
     arming_state 2). TRACK while the last /planning/pos_cmd is < 0.3 s old (cmd_timeout), else HOLD; IDLE
     when not engaged. PRE_S of IDLE ticks are stepped before the engage, as in the node.
   - psi_super / yawdot_super = the bridge's own setpoint: -wrap(pos_cmd.yaw), -yaw_dot of the last command
     in TRACK; the frozen yaw and 0 in HOLD; the latch (the heading) at an engage straight into HOLD. Checked
     against the bridge's recorded /fmu/in/trajectory_setpoint at its own tick stamps.
   - the vehicle: /fmu/out/vehicle_local_position_v1 (NED x, y, heading, xy_valid), as the bridge; else
     FAST-LIO's odometry converted FLU -> FRD (x, -y, -yaw).
   - the target: the last /planning/yaw_target received, (x, y) FLU as on the topic, age = tick - header
     stamp, frame_id as recorded. The stamp is the PERCEPTION stamp, so a message arrives already ~0.25 s old.
3. PX4's yaw response is fitted on this flight in FRD: dpsi/dt = K*wrap(cmd(t-tau) - psi) + yawspeed(t-tau),
   capped at 200 deg/s (AttitudeControl: P on the heading error + yawspeed feed-forward), on the as-flown
   command against FAST-LIO's heading; then it is driven by each variant's (yaw, yawspeed):
     A  as flown: the emulated bridge yaw (== the module with enable False, checked)
     B  fix 1: YawServoParams(enable=True)           C  fix 1 + 2: YawServoParams(enable=True, ff_enable=True)
   plus B / C with aim_in_hold False, and diagnostic rows (not gated) that change one input or limit at a time.
4. Scored as before: at every /dtp/follow_target sample in Offboard, person bearing - heading, FLU, from
   FAST-LIO's position (the model's FRD heading is negated at that boundary). + = the person left of the nose.
Also printed: the yaw target's age at arrival and how much of Offboard it is fresh; heading rates, commanded
(the module's output per tick, and over 0.2 s) and modelled; the slew-limit report; the source timeline; what
happened after each LOST; three sanity checks (switch off = identity; the sign; causality by truncation).
The aircraft's position is taken as flown: yaw does not move a quadrotor's position.
"""
import argparse, importlib.util, math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))   # drone-dtp/scripts, also when run via ~/super_ws/src/drone-dtp
REPO = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE)
from yaw_chain_report import wrap, Series  # noqa: E402


def _load_servo():
    """The one copy of the algorithm: <repo>/super_bringup/scripts/yaw_servo.py, else the workspace's copy."""
    for d in (os.path.join(REPO, 'super_bringup', 'scripts'), os.path.expanduser('~/super_ws/src/super_bringup/scripts')):
        f = os.path.join(d, 'yaw_servo.py')
        if os.path.isfile(f):
            spec = importlib.util.spec_from_file_location('yaw_servo', f)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod, os.path.realpath(f)
    sys.exit(f'yaw_servo.py not found in {REPO}/super_bringup/scripts nor in ~/super_ws/src/super_bringup/scripts')


ys, YS_FILE = _load_servo()
P = ys.YawServoParams

DT = 0.02                      # the bridge's 50 Hz tick
CMD_TIMEOUT = 0.3              # px4_offboard_bridge cmd_timeout: TRACK while the last pos_cmd is younger than this
PRE_S = 5.0                    # IDLE ticks stepped before the engage
RATE_CAP = math.radians(200)   # MC_YAWRATE_MAX
FRESH_MAX, RATE_MAX = 15.0, 90.0   # SS 4.3: deg/s with the target fresh / over the whole Offboard window
D = np.degrees
BAGS = ('stage_b_2026-09-26_200650', 'stage_b_2026-09-26_195108', 'stage_b_2026-09-21_195658_salvaged')
BASELINE = {  # the 2026-09-29 replay (its own copy, no slew, centred ff), p50/p95/max: meas, model, fix 1, fix 1 + 2
    '200650': ((2.1, 16.8, 20.6), (2.0, 15.7, 19.5), (0.3, 4.2, 5.0), (0.4, 1.8, 5.0)),
    '195108': ((7.4, 16.9, 20.0), (7.3, 17.2, 19.4), (1.6, 8.5, 11.0), (1.1, 5.7, 7.1)),
    '2026-09-21': ((4.6, 16.7, 20.6), (5.0, 16.2, 21.0), (1.2, 7.0, 9.8), (0.5, 2.1, 3.8)),
}
ROUND1 = {  # the servo's round-one build (2026-09-29, before round two), replayed here: fix 1, fix 1 + 2, p50/p95/max
    '200650': ((0.7, 4.2, 5.4), (0.5, 1.8, 5.4)),
    '195108': ((1.8, 9.7, 30.5), (1.3, 8.8, 30.5)),
    '2026-09-21': ((1.1, 6.1, 8.7), (0.8, 2.2, 5.3)),
}
GATE = [  # SS 4.3 verbatim: (bag key, variant, statistic, limit; 'flown' = the as-flown p95)
    ('200650', 'B', 1, 5.0), ('200650', 'B', 2, 6.0), ('200650', 'C', 1, 2.5),
    ('195108', 'B', 1, 9.5), ('195108', 'C', 1, 6.5), ('2026-09-21', 'B', 1, 'flown')]

YT, CMD, VCM, STATUS, LPOS, ODOM, FT, LOCK, SP, CAM = (
    '/planning/yaw_target', '/planning/pos_cmd', '/fmu/out/vehicle_control_mode', '/fmu/out/vehicle_status_v1',
    '/fmu/out/vehicle_local_position_v1', '/lidar_slam/odom', '/dtp/follow_target', '/dtp/lock_state',
    '/fmu/in/trajectory_setpoint', '/camera/color/camera_info')


def _st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _fin(*v):
    return all(math.isfinite(x) for x in v)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap_b(a):   # px4_offboard_bridge._wrap_pi
    return math.atan2(math.sin(a), math.cos(a))


ROWS = {  # what is kept of each message; row[0] is always the bag receive time (s)
    YT: lambda m: (_st(m.header), m.header.frame_id, m.point.x, m.point.y),
    CMD: lambda m: (_st(m.header), m.yaw, m.yaw_dot, _fin(m.position.x, m.position.y, m.position.z, m.yaw, m.yaw_dot)),
    VCM: lambda m: (m.flag_armed, m.flag_control_offboard_enabled),
    STATUS: lambda m: (m.nav_state, m.arming_state),
    LPOS: lambda m: (m.x, m.y, m.z, m.heading, m.xy_valid, m.z_valid, m.timestamp_sample * 1e-6, m.timestamp * 1e-6),
    ODOM: lambda m: (_st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)),
    FT: lambda m: (m.position.x, m.position.y),
    LOCK: lambda m: (m.data,),
    SP: lambda m: (m.timestamp * 1e-6, m.yaw, m.yawspeed),
    CAM: lambda m: (_st(m.header),),
}


def read(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in ROWS if t in types]))
    cls = {t: get_message(types[t]) for t in ROWS if t in types}
    d = {t: [] for t in ROWS}
    while r.has_next():
        topic, data, t = r.read_next()
        d[topic].append((t * 1e-9,) + ROWS[topic](deserialize_message(data, cls[topic])))
    for rows in d.values():
        rows.sort(key=lambda row: row[0])
    return d


class Cursor:
    """The last row RECEIVED at or before t. Queries come in time order; the cursor only moves forward and never
    reads a row past t, so nothing received after a tick can reach that tick (the causality argument)."""

    def __init__(self, rows):
        self.rows, self.k = rows, 0

    def at(self, t):
        rows, k = self.rows, self.k
        while k < len(rows) and rows[k][0] <= t:
            k += 1
        self.k = k
        return rows[k - 1] if k else None


def bridge_inputs(d, ticks, vehicle='lpos', age_from='stamp', until=math.inf):
    """Per tick: px4_offboard_bridge's state and what it would hand YawServo.step(), from rows received at or
    before the tick (and at or before `until`, for the truncation test). on_cmd / on_tick logic as in the node."""
    keep = lambda rows: [r for r in rows if r[0] <= until]  # noqa: E731
    cmd = Cursor([r for r in keep(d[CMD]) if r[4]])                        # on_cmd drops a non-finite command
    if d[VCM]:
        mode = Cursor([(r[0], bool(r[1] and r[2])) for r in keep(d[VCM])])
    else:
        mode = Cursor([(r[0], r[1] == 14 and r[2] == 2) for r in keep(d[STATUS])])
    if vehicle == 'lpos':   # (t, heading, x, y, xy_valid, latch ok) in NED/FRD
        veh = Cursor([(r[0], r[4], r[1], r[2], bool(r[5]), bool(r[5] and r[6]) and _fin(r[1], r[2], r[3], r[4]))
                      for r in keep(d[LPOS])])
    else:                   # FAST-LIO, FLU -> FRD
        veh = Cursor([(r[0], -r[4], r[2], -r[3], True, _fin(r[2], r[3], r[4])) for r in keep(d[ODOM])])
    yt = Cursor(keep(d[YT]))
    X = {k: [] for k in ('state', 'engaged', 'psi', 'psid', 'hd', 'px', 'py', 'valid', 'target')}
    sp, sp_t, state = None, -math.inf, ys.IDLE
    for t in ticks:
        c, m, v, y = cmd.at(t), mode.at(t), veh.at(t), yt.at(t)
        if c is not None and c[0] > sp_t:          # a command received since the setpoint was last written
            sp, sp_t = (_wrap_b(-c[2]), -c[3]), c[0]
        engaged = bool(m and m[1])
        if not engaged:                            # IDLE: latch "hold here", yaw = heading
            state = ys.IDLE
            if v is not None and v[5]:
                sp, sp_t = (_wrap_b(v[1]), 0.0), t
        elif c is not None and t - c[0] < CMD_TIMEOUT:
            state = ys.TRACK
        else:                                      # HOLD: frozen yaw, yawspeed 0 (re-latch on the IDLE -> HOLD edge)
            if state == ys.IDLE and v is not None and v[5]:
                sp, sp_t = (_wrap_b(v[1]), 0.0), t
            state = ys.HOLD
            if sp is not None:
                sp = (sp[0], 0.0)
        X['state'].append(state)
        X['engaged'].append(engaged)
        X['psi'].append(sp[0] if sp else math.nan)
        X['psid'].append(sp[1] if sp else 0.0)
        X['hd'].append(v[1] if v else math.nan)
        X['px'].append(v[2] if v else math.nan)
        X['py'].append(v[3] if v else math.nan)
        X['valid'].append(bool(v and v[4]))
        X['target'].append(None if y is None else (y[3], y[4], t - (y[1] if age_from == 'stamp' else y[0]), y[2]))
    return X


def run(params, X):
    s = ys.YawServo(params)
    return [s.step(dt=DT, state=X['state'][i], engaged=X['engaged'][i], psi_super=X['psi'][i],
                   yawdot_super=X['psid'][i], vehicle_heading=X['hd'][i], vehicle_x=X['px'][i],
                   vehicle_y=X['py'][i], vehicle_xy_valid=X['valid'][i], target=X['target'][i])
            for i in range(len(X['psi']))]


def simulate(cmd, ff, psi0, K, tau):
    """PX4's heading on the DT grid (FRD): the command reaches the attitude controller tau later."""
    n_tau = int(round(tau / DT))
    psi = np.empty(len(cmd))
    psi[0] = psi0
    for i in range(1, len(cmd)):
        j = max(0, i - 1 - n_tau)
        rate = K * wrap(cmd[j] - psi[i - 1]) + ff[j]
        psi[i] = wrap(psi[i - 1] + max(-RATE_CAP, min(RATE_CAP, rate)) * DT)
    return psi


def p3(x):
    x = np.abs(np.asarray(x, float))
    return (np.percentile(x, 50), np.percentile(x, 95), x.max()) if len(x) else (math.nan,) * 3


def f3(p):
    return f'p50 {p[0]:5.1f}  p95 {p[1]:5.1f}  max {p[2]:5.1f}'


def pmx(x):
    return f'p95 {np.percentile(x, 95):5.1f} max {np.max(x):6.1f}' if len(x) else '   n/a            '


def _same(a, b):
    return a == b or (a != a and b != b)


class Ctx:
    """Everything replay() and study() share for one bag: the engaged window on the 50 Hz grid, the bridge's
    inputs, FAST-LIO on the grid, the PX4 yaw model fitted on this flight, and the person samples to score."""


def prepare(bag):
    c = Ctx()
    c.name = name = os.path.basename(os.path.normpath(bag))
    c.d = d = read(bag)
    c.key = next((k for k in BASELINE if k in name), None)
    c.t0 = d[ODOM][0][0]
    # ---- the engaged window, as the bridge sees it
    if d[VCM]:
        c.eng_src, eng = VCM, [(r[0], bool(r[1] and r[2])) for r in d[VCM]]
    else:
        c.eng_src, eng = STATUS + ' (no vehicle_control_mode in the bag)', [(r[0], r[1] == 14 and r[2] == 2) for r in d[STATUS]]
    ons = [i for i, r in enumerate(eng) if r[1]]
    if not ons:
        print(f'{name}: never armed in Offboard')
        return None
    c.ta = ta = eng[ons[0]][0]
    c.tb = tb = next((r[0] for r in eng[ons[-1]:] if not r[1]), eng[-1][0])
    c.n_win = sum(1 for a, b in zip([(0, False)] + eng, eng) if b[1] and not a[1])
    c.n_pre, c.n_eng = n_pre, n_eng = int(round(PRE_S / DT)), int(math.ceil((tb - ta) / DT))
    c.ticks = ticks = ta + (np.arange(n_pre + n_eng) - n_pre) * DT
    c.ia, c.ib = ia, ib = n_pre, n_pre + n_eng
    c.W = W = slice(ia, ib)
    c.veh_src = 'lpos' if d[LPOS] else 'odom'
    c.X = X = bridge_inputs(d, ticks, vehicle=c.veh_src)
    c.state = np.array(X['state'])
    # ---- FAST-LIO on the grid, for the fit and the score (as the 2026-09-29 replay)
    c.odom = odom = Series([(r[0], r) for r in d[ODOM]])
    c.st = st = Series([(r[0], r[1]) for r in d[STATUS]])
    c.lk = lk = Series([(r[0], r[1]) for r in d[LOCK]])
    c.pos = pos = np.full((len(ticks), 2), np.nan)
    c.hd = hd = np.full(len(ticks), np.nan)
    for i, t in enumerate(ticks):
        o = odom.at(t, 0.3)
        if o is not None:
            pos[i] = o[2], o[3]
            hd[i] = o[4]
    c.meas = meas = -hd                            # FRD
    # ---- the yaw target's age on every tick, and "fresh" by the servo's own rule
    p0 = P()
    c.age = age = np.array([tg[2] if tg is not None else np.nan for tg in X['target']])
    c.fresh = (age >= -p0.future_tolerance) & (age <= p0.target_timeout)
    # ---- PX4's yaw model, fitted on the as-flown command (FRD)
    c.psi, c.psid = psi, psid = np.array(X['psi']), np.array(X['psid'])
    ok = ~np.isnan(meas[W])
    best = None
    for K in (1.5, 2.0, 2.4, 2.8, 3.2, 4.0, 5.0):
        for tau in (0.0, 0.04, 0.08, 0.12, 0.16, 0.2):
            s = simulate(psi[W], psid[W], meas[ia], K, tau)
            rms = math.sqrt(np.mean(wrap(s[ok] - meas[W][ok]) ** 2))
            if best is None or rms < best[0]:
                best = (rms, K, tau)
    c.rms, c.K, c.tau = best
    # ---- the person samples to score (the 2026-09-29 rule: vehicle_status nav_state 14, FAST-LIO position)
    off = [r[0] for r in d[STATUS] if r[1] == 14]
    sa, sb = (off[0], off[-1]) if off else (ta, tb)
    c.samples, c.dropped = [], 0
    for r in d[FT]:
        t = r[0]
        if not (sa <= t < sb) or (off and st.at(t, 2.0) != 14):
            continue
        if not (ta <= t < tb):
            c.dropped += 1
            continue
        i = ia + int((t - ta) / DT)
        if np.isnan(pos[i, 0]) or np.isnan(hd[i]):
            continue
        bp = math.atan2(r[2] - pos[i, 1], r[1] - pos[i, 0])
        c.samples.append(dict(t=t - c.t0, i=i, bp=bp, rng=math.hypot(r[1] - pos[i, 0], r[2] - pos[i, 1]),
                              M=D(wrap(bp - hd[i])), lock=lk.at(t, 5.0) or ''))
    # ---- the person's bearing from the aircraft between /dtp/follow_target samples (as yaw_chain_report)
    c.fb = []
    prev = None
    for r in d[FT]:
        if ta - 1 <= r[0] < tb + 1:
            o = odom.at(r[0], 0.2)
            if o is None:
                continue
            b = math.atan2(r[2] - o[3], r[1] - o[2])
            rng = math.hypot(r[2] - o[3], r[1] - o[2])
            if prev and 0.02 < r[0] - prev[0] < 0.5:
                c.fb.append((r[0], D(wrap(b - prev[1])) / (r[0] - prev[0]), rng, prev[0]))
            prev = (r[0], b)
    return c


def model_of(c, cmd, ff):
    """PX4's heading (FRD) on the grid under (cmd, ff), from the model fitted on this flight; NaN outside Offboard."""
    s = np.full(len(c.ticks), np.nan)
    s[c.W] = simulate(np.asarray(cmd, float)[c.W], np.asarray(ff, float)[c.W], c.meas[c.ia], c.K, c.tau)
    return s


def fresh_at(c, t):
    """Was the yaw target fresh (the servo's rule) at bag time t, within Offboard?"""
    j = int(round((t - c.ticks[0]) / DT))
    return 0 <= j < len(c.ticks) and c.ia <= j < c.ib and bool(c.fresh[j])


def person_rates(c):
    """The person's own bearing rate from the aircraft (deg/s, |.|) while the target was fresh: between
    consecutive /dtp/follow_target samples, and over ~0.2 s (the sample 0.15-0.3 s earlier)."""
    pair = [abs(x[1]) for x in c.fb if fresh_at(c, x[0]) and fresh_at(c, x[3])]
    bear, tt = {}, []
    for r in c.d[FT]:
        if c.ta - 1 <= r[0] < c.tb + 1:
            o = c.odom.at(r[0], 0.2)
            if o is not None:
                bear[r[0]] = math.atan2(r[2] - o[3], r[1] - o[2])
                tt.append(r[0])
    tt = np.array(tt)
    w02 = []
    for k, t in enumerate(tt):
        j = np.searchsorted(tt, t - 0.2)
        cand = [q for q in (j - 1, j) if 0 <= q < k and 0.15 <= t - tt[q] <= 0.3]
        if not cand or not fresh_at(c, t):
            continue
        q = min(cand, key=lambda q: abs(t - tt[q] - 0.2))
        if fresh_at(c, tt[q]):
            w02.append(abs(D(wrap(bear[t] - bear[tt[q]]))) / (t - tt[q]))
    return np.array(pair), np.array(w02)


def replay(bag, brief=False):
    out = (lambda *a, **k: None) if brief else print
    c = prepare(bag)
    if c is None:
        return None
    d, name, key, t0, ta, tb, n_win = c.d, c.name, c.key, c.t0, c.ta, c.tb, c.n_win
    n_eng, ticks, ia, ib, W = c.n_eng, c.ticks, c.ia, c.ib, c.W
    veh_src, X, state = c.veh_src, c.X, c.state
    odom, st, lk, pos, hd, meas = c.odom, c.st, c.lk, c.pos, c.hd, c.meas
    eng_src = c.eng_src
    print(f'{name}: engaged (armed + Offboard, {eng_src}) t {ta - t0:.2f}-{tb - t0:.2f} s ({tb - ta:.1f} s'
          f'{", %d windows: the span is replayed" % n_win if n_win > 1 else ""}); '
          f'TRACK {DT * np.sum(state[W] == ys.TRACK):.1f} s, HOLD {DT * np.sum(state[W] == ys.HOLD):.1f} s')
    # ---- clocks, and the vehicle source against FAST-LIO
    a = np.array([r[0] - r[1] for r in d[CMD]])
    clk = f'clocks: pos_cmd recv - stamp p50 {1e3 * np.median(a):.1f} ms (SUPER stamps ROS now)' if len(a) else 'clocks:'
    if d[CAM]:
        cam = np.array([r[0] - r[1] for r in d[CAM]])
        th = [1e3 * np.median(x) for x in np.array_split(cam, 3)]
        clk += (f'; camera_info recv - stamp p50 {1e3 * np.median(cam):.1f} ms, by thirds of the bag '
                f'{th[0]:.1f}/{th[1]:.1f}/{th[2]:.1f} ms (the camera, whose stamp the yaw target carries, is on the host clock)')
    out(clk)
    if veh_src == 'lpos':
        lp = d[LPOS]
        out(f'vehicle for the servo: {LPOS} (NED; lpos recv - PX4 timestamp p50 '
            f'{1e3 * np.median([r[0] - r[8] for r in lp]):.1f} ms, i.e. on the host clock)')
        ts = np.array([r[7] for r in lp])
        dd, dh = [], []
        for r in d[ODOM]:
            if ta <= r[0] < tb:
                j = np.searchsorted(ts, r[1])
                if 0 < j < len(ts):
                    f = (r[1] - ts[j - 1]) / (ts[j] - ts[j - 1])
                    x = lp[j - 1][1] + f * (lp[j][1] - lp[j - 1][1])
                    y = lp[j - 1][2] + f * (lp[j][2] - lp[j - 1][2])
                    dd.append(math.hypot(x - r[2], y + r[3]))
                    dh.append(abs(wrap(lp[j - 1][4] + f * wrap(lp[j][4] - lp[j - 1][4]) + r[4])))
        vx = np.array(X['px'][W]) - pos[W, 0]
        vy = np.array(X['py'][W]) + pos[W, 1]
        g = np.hypot(vx, vy)
        g = g[~np.isnan(g)]
        out(f'  lpos vs FRD(odom) in Offboard, time-aligned at the odom stamp: {len(dd)} pairs, horizontal m p50 '
            f'{np.median(dd):.3f} p95 {np.percentile(dd, 95):.3f} max {max(dd):.3f}; heading deg p50 '
            f'{D(np.median(dh)):.2f} p95 {D(np.percentile(dh, 95)):.2f} max {D(max(dh)):.2f}')
        out(f'  as each is held at a tick (the bridge\'s latest lpos vs the scorer\'s latest odom): horizontal m '
            f'p50 {np.median(g):.3f} p95 {np.percentile(g, 95):.3f} max {g.max():.3f}')
    else:
        out(f'vehicle for the servo: FRD(FAST-LIO odometry) -- no {LPOS} in the bag')
    # ---- the yaw target's age, and how long it counts as fresh
    p0 = P()
    frames = sorted({r[2] for r in d[YT]})
    arr = np.array([r[0] - r[1] for r in d[YT] if ta <= r[0] < tb])
    age, fresh = c.age, c.fresh
    Xr = bridge_inputs(d, ticks, vehicle=veh_src, age_from='recv')
    ager = np.array([tg[2] if tg is not None else np.nan for tg in Xr['target']])
    fresh_old = (ager >= -p0.future_tolerance) & (ager <= p0.target_timeout)
    if len(arr):
        out(f'yaw target {YT} (frame {frames}): {len(arr)} in Offboard; age at arrival (recv - stamp) ms p50 '
            f'{1e3 * np.median(arr):.0f} p95 {1e3 * np.percentile(arr, 95):.0f} max {1e3 * arr.max():.0f}; '
            f'{np.sum(arr > p0.target_timeout)} arrived already older than {p0.target_timeout} s')
    out(f'  fresh (age <= {p0.target_timeout} s, the servo\'s rule) {DT * fresh[W].sum():.1f} s of {DT * n_eng:.1f} s '
        f'({100 * fresh[W].mean():.1f} %); the old replay\'s rule (a message received within {p0.target_timeout} s) '
        f'{DT * fresh_old[W].sum():.1f} s ({100 * fresh_old[W].mean():.1f} %)')
    # ---- the bridge emulation against the bridge's own recorded setpoints, at its own tick stamps
    sp = sorted((r for r in d[SP] if ta <= r[1] < tb), key=lambda r: r[1])
    if sp:   # the setpoint is float32; a tick's timestamp is the bridge's clock at that tick
        Xs = bridge_inputs(d, [r[1] for r in sp], vehicle=veh_src)
        ry, rd = np.array([r[2] for r in sp]), np.array([r[3] for r in sp])
        ey = np.array(Xs['psi'], np.float32).astype(float) == ry
        ed = np.array(Xs['psid'], np.float32).astype(float) == rd
        dy = D(np.abs(wrap(np.array(Xs['psi']) - ry)))
        cm = [r for r in d[CMD] if r[4]]
        ct = np.array([r[0] for r in cm])
        cy = np.array([np.float32(_wrap_b(-r[2])) for r in cm], float)
        behind = ahead = 0
        for i in np.flatnonzero(~ey):
            k = np.searchsorted(ct, sp[i][1], side='right') - 1
            behind += 0 < k and cy[k - 1] == ry[i]
            ahead += k + 1 < len(cy) and cy[k + 1] == ry[i]
        out(f'bridge emulation vs its recorded {SP} ({len(sp)} in Offboard, at its own tick stamps): yaw identical '
            f'(float32) on {100 * ey.mean():.1f} %, yawspeed on {100 * ed.mean():.1f} %; of the {np.sum(~ey)} others the '
            f'bridge had the command before the latest one on {behind}, the next one on {ahead} (DDS/executor order within '
            f'a few ms of the tick); |d yaw| p99 {np.percentile(dy, 99):.3f}, max {dy.max():.2f} deg')
    # ---- PX4's yaw model, fitted on the as-flown command (FRD), in prepare()
    psi, psid, K, tau, rms = c.psi, c.psid, c.K, c.tau, c.rms
    print(f'  PX4 yaw model fitted on this flight: K {K} /s, delay {tau * 1000:.0f} ms, heading RMS error '
          f'{D(rms):.2f} deg over {tb - ta:.0f} s of Offboard')

    def model(cmd, ff):
        return model_of(c, cmd, ff)

    # ---- the variants: the real module, at its defaults (round two: stale hold 0.5 s, hysteresis 0.3 m)
    V = {'A': dict(label='as flown (model replay)', yaw=psi, yws=psid, outs=None)}
    runs = [('B', 'fix 1', P(enable=True), X), ('C', 'fix 1 + 2', P(enable=True, ff_enable=True), X),
            ('Bh', 'fix 1, aim_in_hold False', P(enable=True, aim_in_hold=False), X),
            ('Ch', 'fix 1 + 2, aim_in_hold False', P(enable=True, ff_enable=True, aim_in_hold=False), X)]
    Xo = bridge_inputs(d, ticks, vehicle='odom') if veh_src == 'lpos' else None
    for tag, change, Xd in (('r', 'age from receive time (old rule)', Xr), ('s', 'no slew limit (max_rate 1e6)', X),
                            ('n', 'no close-range rule (min_range_m 0)', X), ('o', 'vehicle = FRD(odom), not lpos', Xo),
                            ('b', 'no fall-back yawspeed (fallback_ff_band_deg 0)', X),
                            ('z', 'no stale hold (stale_hold_s 0)', X),
                            ('y', 'no close-range hysteresis (min_range_hysteresis_m 0)', X),
                            ('q', "round one's parameters (hysteresis 0, stale hold 0)", X)):
        if Xd is None:
            continue
        kw = {'s': {'max_rate': 1e6}, 'n': {'min_range_m': 0.0}, 'b': {'fallback_ff_band_deg': 0.0},
              'z': {'stale_hold_s': 0.0}, 'y': {'min_range_hysteresis_m': 0.0},
              'q': {'stale_hold_s': 0.0, 'min_range_hysteresis_m': 0.0}}.get(tag, {})
        runs += [('B' + tag, f'fix 1     | {change}', P(enable=True, **kw), Xd),
                 ('C' + tag, f'fix 1 + 2 | {change}', P(enable=True, ff_enable=True, **kw), Xd)]
    for k, label, prm, Xd in runs:
        o = run(prm, Xd)
        V[k] = dict(label=label, outs=o, yaw=np.array([x.yaw for x in o]), yws=np.array([x.yawspeed for x in o]))
    for v in V.values():
        v['sim'] = model(v['yaw'], v['yws'])
    # ---- score at the person samples (prepare(): nav_state 14, FAST-LIO position)
    rows, dropped = [], c.dropped
    for s_ in c.samples:
        e = {'M': s_['M']}
        for k, v in V.items():
            e[k] = D(wrap(s_['bp'] + v['sim'][s_['i']]))       # FLU heading = -FRD
        rows.append(dict(t=s_['t'], i=s_['i'], bp=s_['bp'], rng=s_['rng'], e=e, lock=s_['lock']))
    if not rows:
        print('no person samples in Offboard')
        return None
    S = {k: p3([r['e'][k] for r in rows]) for k in ['M'] + list(V)}
    bl = BASELINE.get(key)
    r1 = ROUND1.get(key)
    blt = lambda j: f'   {bl[j][0]:4.1f} / {bl[j][1]:4.1f} / {bl[j][2]:4.1f}' if bl else ''  # noqa: E731
    r1t = lambda j: f'   {r1[j][0]:4.1f} / {r1[j][1]:4.1f} / {r1[j][2]:4.1f}' if r1 else ''  # noqa: E731
    print(f'  {len(rows)} person samples; camera off-centre |deg| (frame edge 46)'
          f'{"            2026-09-29 replay" if bl else ""}{"     round one (09-29 build)" if r1 else ""}'
          f'{"  (%d outside the engaged window dropped)" % dropped if dropped else ""}')
    print(f'    as flown (measured)        {f3(S["M"])}{blt(0)}')
    print(f'    as flown (model replay)    {f3(S["A"])}{blt(1)}   <- model check' +
          (f' (d p50/p95/max {S["A"][0] - bl[1][0]:+.1f}/{S["A"][1] - bl[1][1]:+.1f}/{S["A"][2] - bl[1][2]:+.1f})' if bl else ''))
    print(f'    fix 1                      {f3(S["B"])}{blt(2)}{r1t(0)}')
    print(f'    fix 1 + 2                  {f3(S["C"])}{blt(3)}{r1t(1)}')
    out('    secondary (not gated):')
    for k in ('Bh', 'Ch'):
        out(f'    {V[k]["label"]:30s}{f3(S[k])}')
    out('    diagnostic, one change at a time (not gated; to attribute the differences from the old replay):')
    for k in [k for k in V if len(k) == 2 and k[1] in 'rsnobzyq']:
        out(f'    {V[k]["label"]:61s}{f3(S[k])}')
    res = dict(name=name, key=key, S=S, K=K, tau=tau, rms=rms, rates={})
    # ---- heading rates
    k1 = np.arange(ia + 1, ib)
    k10 = np.arange(ia + 10, ib)
    fr1 = fresh[k1]
    fr10 = np.array([fresh[j - 10:j + 1].all() for j in k10])
    out(f'  heading rate, deg/s, over the Offboard window | while the target was fresh ({DT * fresh[W].sum():.1f} s):')
    out(f'    {"":10s} {"commanded, per tick":22s} {"commanded, over 0.2 s":22s} {"modelled vehicle":22s}|'
        f' {"commanded, per tick":22s} {"cmd, 0.2 s":22s} {"modelled vehicle":18s}')
    for k, nm in (('A', 'as flown'), ('B', 'fix 1'), ('C', 'fix 1 + 2'), ('Bh', 'fix 1 ~h'), ('Ch', 'fix 1+2 ~h'),
                  ('Bn', 'fix 1 ~n'), ('Cn', 'fix 1+2 ~n'), ('Br', 'fix 1 ~r'), ('Cr', 'fix 1+2 ~r'),
                  ('Bb', 'fix 1 ~b'), ('Cb', 'fix 1+2 ~b'), ('Bz', 'fix 1 ~z'), ('Cz', 'fix 1+2 ~z'),
                  ('By', 'fix 1 ~y'), ('Cy', 'fix 1+2 ~y'), ('Bq', 'fix 1 ~q'), ('Cq', 'fix 1+2 ~q')):
        v = V[k]
        rc = D(np.abs(wrap(v['yaw'][k1] - v['yaw'][k1 - 1]))) / DT
        rw = D(np.abs(wrap(v['yaw'][k10] - v['yaw'][k10 - 10]))) / (10 * DT)
        rm = D(np.abs(wrap(v['sim'][k1] - v['sim'][k1 - 1]))) / DT
        yw = D(np.abs(v['yws'][W]))
        acc = D(np.abs(np.diff(v['yws'][ia - 1:ib]))) / DT
        res['rates'][k] = dict(cmd=rc.max(), cmd_f=rc[fr1].max() if fr1.any() else 0.0, mod=rm.max(),
                               mod_f=rm[fr1].max() if fr1.any() else 0.0, rw_f=rw[fr10].max() if fr10.any() else 0.0,
                               rc=rc, rw=rw, rm=rm, rw_fa=rw[fr10], rm_fa=rm[fr1])
        out(f'    {nm:10s} {pmx(rc):22s} {pmx(rw):22s} {pmx(rm):22s}| {pmx(rc[fr1]):22s} {pmx(rw[fr10]):22s} {pmx(rm[fr1])}'
            f'   yawspeed max {yw.max():5.1f}, max change {acc.max():6.0f} deg/s^2')
    lpw = [r for r in d[LPOS] if ta <= r[0] < tb]
    lrate = [(b[0], abs(D(wrap(b[4] - a[4]))) / (b[7] - a[7])) for a, b in zip(lpw, lpw[1:]) if b[7] > a[7]]
    if lrate:
        out(f'    measured vehicle (lpos.heading): {pmx([x[1] for x in lrate])}   (~h: aim_in_hold False; diagnostics: ~n '
            'min_range_m 0, ~r age from receive time, ~b fallback_ff_band_deg 0 = no fall-back yawspeed, ~z stale_hold_s 0, '
            '~y min_range_hysteresis_m 0, ~q both 0 = round one\'s parameters)')
    # ---- the person's own bearing rate while the target was fresh (the restated rows of --gate), and the aim
    # point's (psi_des of fix 1 over 0.2 s, while aiming): what the servo is asked to follow
    pr_pair, pr_w02 = person_rates(c)
    oB = V['B']['outs']
    aim_w02 = np.array([abs(D(wrap(oB[q].psi_des - oB[q - 10].psi_des))) / (10 * DT) for q in k10
                        if all(oB[j].source == 1 for j in range(q - 10, q + 1)) and fresh[q - 10:q + 1].all()])
    res['person'] = dict(pair=pr_pair, w02=pr_w02, aim_w02=aim_w02)
    out(f'  while the target was fresh: the person\'s bearing rate from the aircraft, per /dtp/follow_target pair '
        f'{pmx(pr_pair)}, over ~0.2 s {pmx(pr_w02)}; the aim point\'s (yaw target) bearing over 0.2 s while aiming '
        f'{pmx(aim_w02)} deg/s')
    # ---- the fastest commanded turns while the target was fresh: what the person was doing
    fb = c.fb   # (t, the person's bearing rate from the aircraft since the previous sample, range, t_prev)
    fbt = np.array([x[0] for x in fb])
    fbin = [abs(x[1]) for x in fb if ta <= x[0] < tb]
    if fbin:
        out(f'  the person\'s bearing rate from the aircraft (/dtp/follow_target pairs in Offboard, FAST-LIO position): '
            f'p50 {np.median(fbin):.1f} p95 {np.percentile(fbin, 95):.1f} max {max(fbin):.1f} deg/s')
    ytt = np.array([r[0] for r in d[YT]])

    def person_at(t):
        if not len(fbt):
            return 'n/a'
        j = int(np.argmin(np.abs(fbt - t)))
        return (f'person bearing rate {fb[j][1]:+6.1f} deg/s at range {fb[j][2]:.1f} m (sample {fb[j][0] - t:+.2f} s)'
                if abs(fbt[j] - t) < 0.25 else 'no person sample within 0.25 s')

    for k in ('B', 'C'):
        v, R = V[k], res['rates'][k]
        idx = [j for j in np.argsort(-R['rc']) if fr1[j]]
        picked = []
        for j in idx:
            if all(abs(k1[j] - q) > 50 for q in picked):
                picked.append(k1[j])
            if len(picked) == 3:
                break
        n15 = np.sum(R['rc'][fr1] > FRESH_MAX)
        cat = {'slew binding': 0, 'a new yaw target this tick': 0, 'close-range edge': 0, 'other': 0}
        for j in np.flatnonzero(fr1 & (R['rc'] > FRESH_MAX)):
            i = k1[j]
            if v['outs'][i].slew_binding:
                cat['slew binding'] += 1
            elif v['outs'][i].source != v['outs'][i - 1].source:
                cat['close-range edge'] += 1
            elif np.any((ytt > ticks[i] - DT) & (ytt <= ticks[i])):
                cat['a new yaw target this tick'] += 1
            else:
                cat['other'] += 1
        clean = np.array([fr10[q] and all(v['outs'][j].source == 1 and not v['outs'][j].slew_binding
                                          for j in range(k10[q] - 10, k10[q] + 1)) for q in range(len(k10))])
        out(f'  fastest commanded turns while fresh, {v["label"]}: {n15} of {fr1.sum()} fresh ticks > {FRESH_MAX:.0f} deg/s '
            f'per tick ({", ".join(f"{b} {a}" for a, b in cat.items())}); over 0.2 s: {np.sum(R["rw"][fr10] > FRESH_MAX)} of '
            f'{fr10.sum()} windows; over 0.2 s in windows all aiming (source 1) with no slew binding: {pmx(R["rw"][clean])}')
        for i in picked:
            o = v['outs'][i]
            j0 = max(ia, i - 10)
            pd = D(wrap(o.psi_des - v['outs'][j0].psi_des)) / ((i - j0) * DT) if _fin(o.psi_des, v['outs'][j0].psi_des) else math.nan
            newmsg = any(ticks[i] - DT < r[0] <= ticks[i] for r in d[YT])
            out(f'    t={ticks[i] - t0:6.2f}s cmd {D(abs(wrap(v["yaw"][i] - v["yaw"][i - 1]))) / DT:5.1f} per tick '
                f'(step {D(abs(wrap(v["yaw"][i] - v["yaw"][i - 1]))):.2f} deg{", a new yaw target this tick" if newmsg else ""}), '
                f'{D(abs(wrap(v["yaw"][i] - v["yaw"][i - 10]))) / 0.2:5.1f} over 0.2 s; psi_des {pd:+6.1f} deg/s over 0.2 s; '
                f'modelled {D(abs(wrap(v["sim"][i] - v["sim"][i - 1]))) / DT:5.1f}; src {o.source} range {o.range_m:.1f} m '
                f'slew {"BIND" if o.slew_binding else "-"}; {person_at(ticks[i])}; [{(lk.at(ticks[i], 5.0) or "")[:24]}]')
    # ---- the slew limit
    src_same = all(a.source == b.source for a, b in zip(V['B']['outs'], V['C']['outs']))
    for k in ('B', 'C'):
        o = V[k]['outs']
        bnd = np.array([x.slew_binding for x in o[W]])
        resid = np.array([abs(D(wrap(x.psi_des - x.psi_out))) if _fin(x.psi_des, x.psi_out) else 0.0 for x in o[W]])
        rl, j = [], 0
        while j < len(bnd):
            if bnd[j]:
                j2 = j
                while j2 < len(bnd) and bnd[j2]:
                    j2 += 1
                rl.append((j, j2 - j))
                j = j2
            else:
                j += 1
        single_src1 = sum(1 for s, n in rl if n == 1 and o[ia + s].source == 1)
        out(f'  slew limit, {V[k]["label"]}: bound on {bnd.sum()} ticks in {len(rl)} runs ({single_src1} single-tick runs '
            f'while aiming), longest {max([n for _, n in rl], default=0)} ticks; largest |wrap(psi_des - psi_out)| '
            f'{resid.max():.1f} deg')
        for s, n in rl:
            if n > 3:
                i = ia + s
                seg = o[i:i + n]
                out(f'    run of {n} ticks ({n * DT:.2f} s) from t={ticks[i] - t0:.2f}s: residual up to '
                    f'{max(resid[s:s + n]):.1f} deg; sources {sorted({x.source for x in seg})}; target '
                    f'{"fresh" if fresh[i] else "STALE"} (age {age[i]:.2f} s); bridge {state[i]}; '
                    f'[{(lk.at(ticks[i], 5.0) or "")[:24]}]; psi_des {D(seg[0].psi_des):+.1f} -> {D(seg[-1].psi_des):+.1f}')
    # ---- the source timeline
    o, pB = V['B']['outs'], P(enable=True)
    rngs = np.array([x.range_m for x in o[W]], float)

    def why(i):
        x = o[i]
        if x.source == 1:
            return 'servo'
        if x.source == 2:
            return f'close range ({x.range_m:.2f} m)'
        what = 'stale hold' if x.source == 3 else 'fall-back'
        tg = X['target'][i]
        if tg is None:
            return f'{what}: no target yet'
        if not math.isfinite(tg[2]):
            return f'{what}: bad stamp'
        if tg[2] > pB.target_timeout:
            return f'{what}: stale (age {tg[2]:.2f} s)'
        if tg[2] < -pB.future_tolerance:
            return f'{what}: future stamp'
        if x.bad_frame:
            return f'{what}: bad frame {tg[3]!r}'
        if not X['valid'][i] or not _fin(X['px'][i], X['py'][i], X['hd'][i]):
            return f'{what}: lpos invalid'
        return f'{what}: {state[i]} (other)'

    srcs = np.array([x.source for x in o[W]])
    out(f'  source timeline, fix 1 (fix 1 + 2 {"identical" if src_same else "DIFFERENT"}) in Offboard: SUPER/fall-back '
        f'{DT * np.sum(srcs == 0):.2f} s, servo {DT * np.sum(srcs == 1):.2f} s, close-range hold {DT * np.sum(srcs == 2):.2f} s, '
        f'stale hold {DT * np.sum(srcs == 3):.2f} s; range < {pB.min_range_m} m on {DT * np.sum(rngs < pB.min_range_m):.2f} s, '
        f'min range {np.nanmin(rngs):.2f} m')
    trans, last = [], None
    for i in range(ia, ib):
        cur = (o[i].source, state[i])
        if cur != last:
            trans.append(i)
        last = cur
    for i in trans[:40]:
        out(f'    t={ticks[i] - t0:6.2f}s  {state[i]:5s} src {o[i].source}  {why(i):34s}  [{(lk.at(ticks[i], 5.0) or "")[:26]}]')
    if len(trans) > 40:
        out(f'    ... {len(trans) - 40} more transitions')
    # ---- worst moments and the losses
    out('  the 6 samples where the person was furthest off-centre as flown:')
    for r in sorted(rows, key=lambda r: -abs(r['e']['M']))[:6]:
        out(f'    t={r["t"]:6.1f}s  flown {r["e"]["M"]:+6.1f}  fix1 {r["e"]["B"]:+6.1f}  fix1+2 {r["e"]["C"]:+6.1f}  '
            f'range {r["rng"]:4.1f}  [{r["lock"][:28]}]')
    for k in ('B', 'C'):
        out(f'  the 3 samples where the person was furthest off-centre under {V[k]["label"]}:')
        for r in sorted(rows, key=lambda r: -abs(r['e'][k]))[:3]:
            x = V[k]['outs'][r['i']]
            out(f'    t={r["t"]:6.1f}s  {r["e"][k]:+6.1f} (flown {r["e"]["M"]:+6.1f})  src {x.source}  slew '
                f'{"BIND" if x.slew_binding else "-"}  target age {x.target_age_s:.2f} s  range {r["rng"]:4.1f}  '
                f'{(r["t"] + t0 - ta):5.2f} s after the engage  [{r["lock"][:28]}]')
    out('  before each LOST in Offboard:')
    prevs, lost = '', []
    for r in d[LOCK]:
        if r[1].startswith('LOST') and not prevs.startswith('LOST') and st.at(r[0], 2.0) == 14:
            lost.append(r[0])
        prevs = r[1]
    for tl in lost:
        before = [r for r in rows if r['t'] <= tl - t0][-3:]
        out(f'    LOST at t={tl - t0:.1f}s; last person samples: ' +
            ', '.join(f't={r["t"]:.1f} flown {r["e"]["M"]:+.1f} fix1 {r["e"]["B"]:+.1f} fix1+2 {r["e"]["C"]:+.1f}' for r in before))
    if not lost:
        out('    none')
    for tl in lost:
        il = ia + max(0, int(math.ceil((tl - ta) / DT)))
        if il >= ib - 1:
            continue
        y = Cursor(d[YT]).at(tl)
        s0 = il
        while s0 > ia and not fresh[s0 - 1]:
            s0 -= 1
        stale = s0 if not fresh[il] else next((i for i in range(il, ib) if not fresh[i]), None)
        back = next((i for i in range(il, ib) if fresh[i]), None) if not fresh[il] else None
        out(f'  after the LOST at t={tl - t0:.1f}s ({(ib - il) * DT:.1f} s to the end of Offboard): the yaw target is stale from '
            + (f't={ticks[stale] - t0:.2f}s [{(lk.at(ticks[stale], 5.0) or "")[:24]}]' if stale is not None else 'never')
            + (f', fresh again at t={ticks[back] - t0:.2f}s' if back is not None else ', to the end of Offboard' if stale is not None else '')
            + f'; its last message in Offboard arrived at t={Cursor(d[YT]).at(tb)[0] - t0:.2f}s'
            + (f'; the last aim point before the loss: FLU ({y[3]:.1f}, {y[4]:.1f})' if y else ''))
        seg = range(il, ib)
        for k, nm in (('M', 'as flown, measured'), ('A', 'as flown, model'), ('B', 'fix 1'), ('C', 'fix 1 + 2'),
                      ('Bb', 'fix 1 ~b (diag)'), ('Bz', 'fix 1 ~z (diag)'), ('Bq', 'fix 1 ~q (diag)')):
            h = hd if k == 'M' else -V[k]['sim']       # FLU
            sw = max(abs(D(wrap(h[i] - h[il]))) for i in seg)
            away = max(abs(D(wrap(math.atan2(y[4] - pos[i, 1], y[3] - pos[i, 0]) - h[i]))) for i in seg) if y else math.nan
            if k == 'M':
                rr, how = [x[1] for x in lrate if tl <= x[0] < tb], 'lpos.heading'
            else:
                rr, how = [abs(D(wrap(h[i] - h[i - 1]))) / DT for i in seg if i > il], 'modelled'
            cm = '' if k == 'M' else (f'; commanded max {max(abs(D(wrap(V[k]["yaw"][i] - V[k]["yaw"][i - 1]))) / DT for i in seg if i > il):.1f} deg/s, '
                                      f'swing {max(abs(D(wrap(V[k]["yaw"][i] - V[k]["yaw"][il]))) for i in seg):.1f}, '
                                      f'yawspeed max {max(abs(D(V[k]["yws"][i])) for i in seg):.1f}')
            if k != 'M' and V[k]['outs']:
                ss = [V[k]['outs'][i].source for i in seg]
                cm += f'; sources {sorted(set(ss))}, bound {sum(V[k]["outs"][i].slew_binding for i in seg)} ticks'
            out(f'    {nm:19s} heading swing {sw:5.1f} deg, max {away:5.1f} deg off the last aim point, rate max '
                f'{max(rr):6.1f} deg/s ({how}){cm}')
    # ---- sanity checks
    o0 = run(P(), X)
    ident = sum(1 for i, x in enumerate(o0) if x.passthrough and x.yaw is X['psi'][i] and x.yawspeed is X['psid'][i])
    s0 = model([x.yaw for x in o0], [x.yawspeed for x in o0])
    same = np.array_equal(s0, V['A']['sim'], equal_nan=True)
    out(f'  sanity 1, switch off = identity: {ident}/{len(o0)} ticks return psi_super/yawdot_super untouched (the same '
        f'objects); model replay from the module\'s output {"IDENTICAL to" if same else "DIFFERS from"} the as-flown row')
    oB = V['B']['outs']
    left = [r for r in rows if r['e']['M'] > 3 and oB[r['i']].source == 1]
    agree = [r for r in rows if abs(r['e']['M']) > 5 and oB[r['i']].source == 1]
    n_ok = sum(1 for r in agree if np.sign(wrap(oB[r['i']].yaw - X['hd'][r['i']])) == -np.sign(r['e']['M']))
    if left:
        r = max(left, key=lambda r: r['e']['M'])
        x = oB[r['i']]
        out(f'  sanity 2, the sign: t={r["t"]:.1f}s person {r["e"]["M"]:+.1f} deg LEFT of the nose (FLU bearing '
            f'{D(r["bp"]):+.1f}, FLU heading {D(hd[r["i"]]):+.1f}); lpos FRD heading {D(X["hd"][r["i"]]):+.1f}; servo psi_des '
            f'{D(x.psi_des):+.1f}, commanded yaw {D(x.yaw):+.1f} -> yaw - heading {D(wrap(x.yaw - X["hd"][r["i"]])):+.1f} deg '
            f'({"a LEFT turn, correct" if wrap(x.yaw - X["hd"][r["i"]]) < 0 else "WRONG SIGN"}); over all samples |e| > 5 deg '
            f'while aiming: {n_ok}/{len(agree)} turn toward the person')
    else:
        out(f'  sanity 2, the sign: no sample > 3 deg left while aiming; |e| > 5: {n_ok}/{len(agree)} turn toward the person')
    cz = []
    for f in (0.25, 0.5, 0.8):
        T = ta + f * (tb - ta) + 0.0071
        XT = bridge_inputs(d, ticks, vehicle=veh_src, until=T)
        nb = bad = aft = 0
        for k in ('B', 'C'):
            o2 = run(P(enable=True) if k == 'B' else P(enable=True, ff_enable=True), XT)
            s2 = model([x.yaw for x in o2], [x.yawspeed for x in o2])
            for i, t in enumerate(ticks):
                a1, a2 = V[k]['outs'][i], o2[i]
                eq = (_same(a1.yaw, a2.yaw) and _same(a1.yawspeed, a2.yawspeed) and _same(a1.psi_des, a2.psi_des)
                      and a1.source == a2.source and _same(V[k]['sim'][i], s2[i]))
                if t <= T:
                    nb += 1
                    bad += not eq
                else:
                    aft += not eq
        cz.append(f'T={T - t0:.2f}s: {bad} of {nb} changed at or before T ({aft} after)')
    out('  sanity 3, causality (inputs truncated at T, fix 1 and fix 1 + 2, outputs and model): ' + '; '.join(cz))
    return res


def gate(results):
    lines = []
    for key, var, j, lim in GATE:
        r = next((r for r in results if r and r['key'] == key), None)
        if r is None:
            lines.append((False, f'{key}: bag not replayed'))
            continue
        val = r['S'][var][j]
        if lim == 'flown':
            lim = r['S']['M'][1]
            txt = f'{key} {"servo" if var == "B" else "servo + ff"} p95 {val:.2f} <= as flown p95 {lim:.2f} deg ' \
                  f'(model replay of as flown {r["S"]["A"][1]:.2f})'
        else:
            txt = f'{key} {"servo" if var == "B" else "servo + ff"} {("p50", "p95", "max")[j]} {val:.2f} <= {lim} deg'
        lines.append((val <= lim, txt))
    for r in results:
        if not r:
            continue
        for var in ('B', 'C'):
            R, nm = r['rates'][var], ('servo' if var == 'B' else 'servo + ff')
            lines.append((R['cmd_f'] <= FRESH_MAX, f'{r["key"] or r["name"]} {nm}: commanded heading rate while the target is '
                          f'fresh, max {R["cmd_f"]:.1f} <= {FRESH_MAX:.0f} deg/s per 20 ms tick (the same over 0.2 s: '
                          f'{R["rw_f"]:.1f}; modelled vehicle {R["mod_f"]:.1f})'))
            lines.append((R['cmd'] <= RATE_MAX + 1e-6, f'{r["key"] or r["name"]} {nm}: commanded heading rate over the whole '
                          f'Offboard window, max {R["cmd"]:.6f} <= {RATE_MAX:.0f} deg/s (float tolerance 1e-6; modelled '
                          f'vehicle max {R["mod"]:.1f})'))
    for ok, txt in lines:
        print(f'  [{"PASS" if ok else "FAIL"}] {txt}')
    n = sum(ok for ok, _ in lines)
    print(f'{n}/{len(lines)}')
    # Restated readings of the two rate criteria. NOT the plan's, NOT counted in the N/N above: they are
    # there so a reader can see whether a FAIL above is the servo following a fast bearing or doing
    # something wrong. The plan's "with the target fresh, the heading rate stays <= 15 deg/s" is read above
    # per 20 ms tick of the commanded heading, where the 15 Hz yaw target makes a staircase.
    print(' restated, NOT the plan\'s criteria and NOT counted in the N/N above:')
    rl = []
    for r in results:
        if not r:
            continue
        pr = r['person']
        for var in ('B', 'C'):
            R, nm = r['rates'][var], ('servo' if var == 'B' else 'servo + ff')
            tag = r['key'] or r['name']
            rl.append((R['rw_f'] <= FRESH_MAX,
                       f'{tag} {nm}: commanded heading over a 0.2 s window while the target is fresh, max {R["rw_f"]:.1f} '
                       f'(p95 {np.percentile(R["rw_fa"], 95) if len(R["rw_fa"]) else math.nan:.1f}) <= {FRESH_MAX:.0f} deg/s | '
                       f'the person\'s own bearing rate from the aircraft over ~0.2 s, target fresh: {pmx(pr["w02"]).strip()}; '
                       f'the aim point\'s over 0.2 s while aiming: {pmx(pr["aim_w02"]).strip()}'))
            rl.append((R['mod_f'] <= FRESH_MAX,
                       f'{tag} {nm}: modelled vehicle yaw rate while the target is fresh, max {R["mod_f"]:.1f} '
                       f'(p95 {np.percentile(R["rm_fa"], 95) if len(R["rm_fa"]) else math.nan:.1f}) <= {FRESH_MAX:.0f} deg/s | '
                       f'the person\'s own bearing rate per /dtp/follow_target pair, target fresh: {pmx(pr["pair"]).strip()}'))
            rl.append((R['mod'] <= RATE_MAX,
                       f'{tag} {nm}: modelled vehicle yaw rate over the whole Offboard window, max {R["mod"]:.1f} <= '
                       f'{RATE_MAX:.0f} deg/s'))
    for ok, txt in rl:
        print(f'  [{"pass" if ok else "fail"}] (restated) {txt}')
    return n == len(lines)


STUDY_MIN_RANGE = (2.5, 2.0, 1.5, 0.0)
STUDY_HOLD = (0.0, 0.5, 1.0)


def _runs_of(flags):
    best = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    return best


def metrics(c, prm):
    """One configuration on one bag: the module's outputs, the modelled heading, and the study's numbers."""
    o = run(prm, c.X)
    yaw, yws = np.array([x.yaw for x in o]), np.array([x.yawspeed for x in o])
    sim = model_of(c, yaw, yws)
    err = np.array([D(wrap(s['bp'] + sim[s['i']])) for s in c.samples])
    k1 = np.arange(c.ia + 1, c.ib)
    rm = D(np.abs(wrap(sim[k1] - sim[k1 - 1]))) / DT
    rc = D(np.abs(wrap(yaw[k1] - yaw[k1 - 1]))) / DT
    ow = o[c.W]
    srcs = [x.source for x in ow]
    return dict(o=o, yaw=yaw, sim=sim, err=err, p=p3(err), mod=rm.max(), rm=rm, rc=rc,
                bind=_runs_of([x.slew_binding for x in ow]), nbind=sum(x.slew_binding for x in ow),
                secs={s: DT * srcs.count(s) for s in (0, 1, 2, 3)},
                changes=sum(1 for a, b in zip(srcs, srcs[1:]) if a != b))


def study(bag):
    """--study: min_range_m x stale_hold_s (hysteresis 0.3) and round one's parameters, fix 1 and fix 1 + 2; the
    ranges flown; and the evidence on the close-range rule's premise."""
    c = prepare(bag)
    if c is None or not c.samples:
        print(f'{bag}: nothing to study')
        return
    print(f'{c.name}: Offboard {c.tb - c.ta:.1f} s, {len(c.samples)} person samples; PX4 yaw model K {c.K} /s, '
          f'delay {1e3 * c.tau:.0f} ms; as flown (measured) off-centre {f3(p3([s["M"] for s in c.samples]))}')
    cfgs = [(f'{mr:3.1f}   {h:3.1f}', dict(min_range_m=mr, stale_hold_s=h)) for mr in STUDY_MIN_RANGE for h in STUDY_HOLD]
    cfgs.append(('r1 params*', dict(min_range_m=2.5, stale_hold_s=0.0, min_range_hysteresis_m=0.0)))
    M = {}
    for lbl, kw in cfgs:
        M[lbl] = (metrics(c, P(enable=True, **kw)), metrics(c, P(enable=True, ff_enable=True, **kw)))
    print('  min_r hold (hysteresis 0.3 m) | off-centre p50/p95/max: fix 1 | fix 1+2 | max modelled vehicle rate, deg/s: '
          'fix 1 / 1+2 | longest slew-binding run: fix 1 / 1+2 | s in source 0/1/2/3 (fix 1) | source changes: fix 1 / 1+2')
    for lbl, (a, b) in M.items():
        s = a['secs']
        print(f'  {lbl:10s} | {a["p"][0]:4.1f} / {a["p"][1]:4.1f} / {a["p"][2]:4.1f} | {b["p"][0]:4.1f} / {b["p"][1]:4.1f} / '
              f'{b["p"][2]:4.1f} | {a["mod"]:5.1f} / {b["mod"]:5.1f} | {a["bind"]:2d} ticks {DT * a["bind"]:.2f} s / {b["bind"]:2d} '
              f'ticks {DT * b["bind"]:.2f} s | {s[0]:5.2f} / {s[1]:5.2f} / {s[2]:4.2f} / {s[3]:4.2f} | {a["changes"]} / {b["changes"]}')
    print('  * r1 params = round one\'s parameters: min_range_m 2.5, hysteresis 0, stale hold 0. NOT the round-one module: '
          'its fall-back yawspeed rule is gone (F-a applies here).')
    # ---- the ranges flown
    base = M['2.5   0.5'][0]['o']
    rs = np.array([x.range_m for x in base[c.W]], float)
    ok = ~np.isnan(rs)
    pr = np.array([s['rng'] for s in c.samples])
    print(f'  range flown: servo range (lpos -> yaw target, on {DT * ok.sum():.1f} s with a usable target) min {np.nanmin(rs):.2f} m, '
          f'time under 2.5 / 2.0 / 1.5 m: {DT * np.sum(rs < 2.5):.2f} / {DT * np.sum(rs < 2.0):.2f} / '
          f'{DT * np.sum(rs < 1.5):.2f} s; person range (FAST-LIO -> /dtp/follow_target, {len(pr)} samples) min '
          f'{pr.min():.2f} m, samples under 2.5 / 2.0 / 1.5 m: {np.sum(pr < 2.5)} / {np.sum(pr < 2.0)} / {np.sum(pr < 1.5)}')
    # ---- the close-range rule's premise: "at close range a person stepping sideways demands huge rates"
    fbw = [x for x in c.fb if c.ta <= x[0] < c.tb]
    print('  the person\'s own bearing rate from the aircraft by the person\'s range (/dtp/follow_target pairs in Offboard):')
    for lo, hi in ((0.0, 2.5), (2.5, 3.5), (3.5, 99.0)):
        v = np.array([abs(x[1]) for x in fbw if lo <= x[2] < hi])
        print(f'    {lo:3.1f}-{hi if hi < 99 else math.inf:3.1f} m: n {len(v):3d}' +
              (f', p50 {np.median(v):5.1f} p95 {np.percentile(v, 95):5.1f} max {v.max():5.1f} deg/s' if len(v) else ''))
    n0 = M['0.0   0.5'][0]
    o0 = n0['o']
    k10 = np.arange(c.ia + 10, c.ib)
    print('  the aim point\'s (yaw target) bearing rate over 0.2 s, by the servo\'s range, with min_range_m 0 (source 1 throughout):')
    for lo, hi in ((0.0, 2.5), (2.5, 3.5), (3.5, 99.0)):
        v = np.array([abs(D(wrap(o0[q].psi_des - o0[q - 10].psi_des))) / (10 * DT) for q in k10
                      if all(o0[j].source == 1 for j in range(q - 10, q + 1)) and lo <= o0[q].range_m < hi])
        print(f'    {lo:3.1f}-{hi if hi < 99 else math.inf:3.1f} m: n {len(v):4d}' +
              (f', p50 {np.median(v):5.1f} p95 {np.percentile(v, 95):5.1f} max {v.max():5.1f} deg/s' if len(v) else ''))
    k1 = np.arange(c.ia + 1, c.ib)
    for lbl in ('2.5   0.5', '0.0   0.5'):
        for j, var in enumerate(('fix 1', 'fix 1 + 2')):
            m = M[lbl][j]
            o = m['o']
            close = np.array([_fin(o[i].range_m) and o[i].range_m < 2.5 for i in k1])
            cw = [q for q in k10 if all(_fin(o[i].range_m) and o[i].range_m < 2.5 for i in range(q - 10, q + 1))]
            r02 = np.array([abs(D(wrap(m['yaw'][q] - m['yaw'][q - 10]))) / (10 * DT) for q in cw])
            e_c = [e for e, s in zip(m['err'], c.samples) if s['rng'] < 2.5]
            e_m = [e for e, s in zip(m['err'], c.samples) if 2.5 <= s['rng'] < 3.5]
            nb = sum(o[i].slew_binding for i in k1[close])
            print(f'  while the servo range < 2.5 m ({DT * close.sum():.2f} s), min_range_m {lbl.split()[0]}, {var}: commanded '
                  f'max {m["rc"][close].max() if close.any() else math.nan:5.1f} per tick, over 0.2 s {pmx(r02).strip()}; modelled '
                  f'vehicle max {m["rm"][close].max() if close.any() else math.nan:5.1f} deg/s; slew bound on {nb} ticks; '
                  f'off-centre at person samples < 2.5 m {f3(p3(e_c)) if e_c else "n/a"}, 2.5-3.5 m {f3(p3(e_m)) if e_m else "n/a"}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='*')
    ap.add_argument('--gate', action='store_true', help='evaluate SS 4.3 (default bags: the three in <repo>/bags)')
    ap.add_argument('--study', action='store_true', help='the min_range_m x stale_hold_s parameter study and the '
                    'close-range evidence (default bags: the three in <repo>/bags)')
    a = ap.parse_args()
    bags = a.bags or ([os.path.join(REPO, 'bags', b) for b in BAGS] if (a.gate or a.study) else [])
    if not bags:
        ap.error('give a bag, or --gate, or --study')
    print(f'servo module: {YS_FILE}')
    if a.study:
        print(f'defaults: {P(enable=True)!r}')
        for b in bags:
            print('=' * 80, '\n', b)
            study(b)
        return
    results = []
    for b in bags:
        print('=' * 80, '\n', b)
        results.append(replay(b, brief=a.gate))
    if a.gate:
        print('=' * 80, '\n gate, docs/plan-camera-yaw-servo.md SS 4.3')
        sys.exit(0 if gate(results) else 1)


if __name__ == '__main__':
    main()
