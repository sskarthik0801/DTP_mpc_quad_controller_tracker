#!/usr/bin/env python3
"""Bench gate for ROG-Map's visible-but-unhit decay (patch 0007), against the REAL fsm_node. No hardware.

WHAT IT PROVES
--------------
fsm_node's ROG-Map is fed a synthetic MID360-like scene on a private localhost-only DDS domain,
once with rog_map/unhit_decay/enable false and once true, and the map is read back off
/rog_map/occ. The sensor sits at the origin 0.15 m above a ground plane -- the grounded-aircraft
geometry of the 2026-09-14 flight day, where a walker's cells stayed occupied for 7 minutes
because every ray through them went into the sky (docs/fact-rogmap-unhit-decay.md).

  U1  SKY        a person stands 3.1 m away with nothing behind them, then leaves. OFF: their cells
                 stay for the rest of the run (no ray ever crosses them). ON: gone, and the time it
                 took is reported.
  U2  OCCLUDED   a second person stands 3.1 m away on the other side and leaves at the same moment,
                 but a tube appears between the sensor and where they stood. ON must leave those
                 cells alone (no line of sight, no evidence): at least as many as OFF within noise.
  U3  OUT OF     an object above the MID360's +52 deg is mapped by a few wide scans at the start and
      VIEW       then never scanned again. Neither arm may touch it.
  U4  FAR        a person beyond range_max (8 m) with sky behind, who also leaves. Both arms keep
                 them (the sweep stops at range_max), so a distant remnant is still a remnant.
  U5  REAL       the wall, the ground plane and the tube keep returning hits: ON keeps at least 97 %
                 of OFF's cells in each.
  U6  COST       the decay pass's own timing from fsm_node's log: max under 15 ms.

Cell counts move a little between runs (/rog_map/occ is sampled on its own 2 Hz timer); every
threshold has margin over that.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/unhit_decay_check.py          # ~4 min, exits non-zero on any FAIL
"""
import argparse
import math
import os
import re
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import target_filter_check as tfc  # noqa: E402  (Harness, start_fsm, stop_fsm, cylinder_t)

DOMAIN = '89'
RES = 0.1
PACE_S = 0.2             # wall time between scans; each is inserted before the next (~10 ms)
SCAN_DT = 0.1
RAYS_PER_DEG2 = 0.315    # the MID360's ~200k pts/s / point_filter_num 3 / 10 Hz over 360 x 59 deg
AZ = (-75.0, 75.0)
EL = (-7.0, 52.0)
GROUND = -0.15           # the sensor is 0.15 m above the ground: the aircraft on its legs
WALL_X, WALL_Y, WALL_TOP = 7.0, 5.0, 2.5

# The scene. A and B are 3.1 m away at +-45 deg; nothing lies behind either (the wall spans
# |y| <= 5 at x = 7, a ray through A reaches x = 7 at y = 7). The tube T appears in front of B's spot
# when B leaves. L is a "lamp" above the field of view. F is 8.5 m away, beyond range_max.
A = (2.2, 2.2, 0.20, GROUND, 1.60)
B = (2.2, -2.2, 0.20, GROUND, 1.60)
T = (1.1, -1.1, 0.25, GROUND, 1.00)
L = (0.9, 0.0, 0.15, 1.30, 1.50)          # elevation 55..59 deg from the sensor
F = (5.5, 6.5, 0.30, GROUND, 1.60)        # range 8.2..8.8 m

T_STAND = 3.0            # scan time when A, B, F appear
T_LEAVE = 11.0           # ...and leave; T appears
T_END = 45.0


def scan(k, objects, el=EL):
    rng = np.random.default_rng(2000 + k)
    n = int(RAYS_PER_DEG2 * (AZ[1] - AZ[0]) * (el[1] - el[0]))
    az = np.radians(rng.uniform(*AZ, n))
    s = rng.uniform(math.sin(math.radians(el[0])), math.sin(math.radians(el[1])), n)
    e = np.arcsin(s)
    d = np.stack([np.cos(e) * np.cos(az), np.cos(e) * np.sin(az), np.sin(e)], axis=1)
    t = np.full(n, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        tw = np.where(d[:, 0] > 1e-6, WALL_X / d[:, 0], np.inf)
        tg = np.where(d[:, 2] < -1e-6, GROUND / d[:, 2], np.inf)
    yw, zw = tw * d[:, 1], tw * d[:, 2]
    t = np.where((np.abs(yw) <= WALL_Y) & (zw >= GROUND) & (zw <= WALL_TOP), tw, t)
    t = np.minimum(t, np.where(tg * np.hypot(d[:, 0], d[:, 1]) <= 12.0, tg, np.inf))
    for (cx, cy, r, zb, zt) in objects:
        t = np.minimum(t, tfc.cylinder_t(d, cx, cy, r, zb, zt))
    hit = np.isfinite(t)
    return (d[hit] * t[hit, None]).astype(np.float32)


def column(occ, obj, margin=0.15, z=(0.0, 1.7)):
    cx, cy, r = obj[0], obj[1], obj[2]
    sel = (np.hypot(occ[:, 0] - cx, occ[:, 1] - cy) <= r + margin) & (occ[:, 2] >= z[0]) & (occ[:, 2] <= z[1])
    return int(sel.sum())


def wall_cells(occ):
    return int(((occ[:, 0] >= 6.8) & (occ[:, 0] <= 7.3) & (np.abs(occ[:, 1]) <= WALL_Y)
                & (occ[:, 2] >= 0.0) & (occ[:, 2] <= WALL_TOP)).sum())


def ground_cells_in(occ, r0, r1):
    h = np.hypot(occ[:, 0], occ[:, 1])
    return int(((h >= r0) & (h < r1) & (occ[:, 2] >= -0.35) & (occ[:, 2] <= -0.05)).sum())


def ground_cells(occ):
    return int(((np.hypot(occ[:, 0], occ[:, 1]) <= 7.5) & (occ[:, 2] >= -0.35) & (occ[:, 2] <= -0.05)).sum())


def make_cfg(enable, tmpdir):
    base = open(tfc.BASE_CFG).read()
    if not re.search(r'^  unhit_decay:\n    enable: (true|false)', base, re.M):
        raise SystemExit('%s has no rog_map/unhit_decay block (patch 0007?)' % tfc.BASE_CFG)
    cfg = re.sub(r'^(  unhit_decay:\n    enable:) (true|false)', r'\1 %s' % ('true' if enable else 'false'),
                 base, flags=re.M)
    # the oracle and the target filter are not under test; keep the filter off as flown in Stage A
    cfg = re.sub(r'^(  target_exclusion:\n    enable:) (true|false)', r'\1 false', cfg, flags=re.M)
    path = os.path.join(tfc.SUPER_CFG_DIR, '.unhit_decay_%s.yaml' % ('on' if enable else 'off'))
    open(path, 'w').write(cfg)
    open(os.path.join(tmpdir, os.path.basename(path)), 'w').write(cfg)
    return path


def run_arm(h, enable, tmpdir, cfg):
    log_path = os.path.join(tmpdir, 'fsm_%s.log' % ('on' if enable else 'off'))
    proc, log = tfc.start_fsm(cfg, log_path)
    snaps = []           # (scan_t, occ array)
    try:
        h.occ, h.occ_t = None, 0.0
        t0 = time.time()
        k = 0
        while time.time() - t0 < 40.0:        # fsm_node preallocates ~2 GB; wait for the map
            h.publish_scan(scan(k, [L], el=(EL[0], 65.0)), time.time())
            k += 1
            h.spin(PACE_S)
            if h.occ is not None:
                break
        if h.occ is None:
            raise RuntimeError('fsm_node never published /rog_map/occ (log: %s)' % log_path)
        k0 = k
        base_t = time.time()
        next_snap = 1.0
        while True:
            st = (k - k0) * SCAN_DT
            if st > T_END:
                break
            objs = []
            el = EL
            if st < T_STAND:
                objs = [L]
                el = (EL[0], 65.0)            # map the lamp, then never scan that high again
            elif st < T_LEAVE:
                objs = [A, B, F]
            else:
                objs = [T]
            h.publish_scan(scan(k, objs, el=el), base_t + st)
            k += 1
            h.spin(PACE_S)
            if st >= next_snap and h.occ is not None:
                snaps.append((st, h.occ.copy()))
                next_snap += 1.0
        h.spin(1.0)
        snaps.append((T_END, h.snapshot(time.time() - 0.6)))
    finally:
        tfc.stop_fsm(proc, log)
    return snaps, open(log_path).read()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--keep', action='store_true', help='keep the logs and configs under /tmp')
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    tfc.DOMAIN = DOMAIN
    tmpdir = tempfile.mkdtemp(prefix='unhit_decay_check_')
    cfg_off, cfg_on = make_cfg(False, tmpdir), make_cfg(True, tmpdir)
    h = tfc.Harness()
    results = {}
    try:
        for enable in (False, True):
            print('== arm %s' % ('ON' if enable else 'OFF'), flush=True)
            snaps, log = run_arm(h, enable, tmpdir, cfg_on if enable else cfg_off)
            results[enable] = (snaps, log)
            for st, occ in snaps:
                if int(st) % 4 == 0 or st == T_END:
                    print('   t %4.0f s  A %3d  B %3d  T %3d  L %2d  F %3d  wall %4d  ground %4d'
                          % (st, column(occ, A), column(occ, B), column(occ, T), column(occ, L, 0.1, (1.2, 1.6)),
                             column(occ, F, 0.2), wall_cells(occ), ground_cells(occ)), flush=True)
    finally:
        h.close()
        for p in (cfg_off, cfg_on):
            try:
                os.remove(p)
            except OSError:
                pass

    off, on = results[False][0], results[True][0]
    off_end, on_end = off[-1][1], on[-1][1]
    checks = []

    def check(name, ok, detail):
        checks.append((name, ok, detail))
        print('  %-4s %s  %s' % ('PASS' if ok else 'FAIL', name, detail), flush=True)

    print('== verdicts')
    # U1 sky remnant
    standing = [column(o, A) for st, o in off if st <= T_LEAVE]
    a_off_leave = standing[-1] if standing else 0
    a_off, a_on = column(off_end, A), column(on_end, A)
    check('U1a OFF keeps the sky remnant', a_off >= 15 and a_off >= 0.5 * a_off_leave,
          'A column: %d cells while standing, %d at the end (OFF)' % (a_off_leave, a_off))
    check('U1b ON clears it', a_on <= 1, 'A column at the end: %d cells (ON)' % a_on)
    t_clear = next((st for st, o in on if st > T_LEAVE and column(o, A) <= 1), None)
    check('U1c ...within 15 s of the person leaving', t_clear is not None and t_clear - T_LEAVE <= 15.0,
          'cleared %s s after leaving' % ('never' if t_clear is None else '%.0f' % (t_clear - T_LEAVE)))
    # U2 occluded
    b_off, b_on = column(off_end, B), column(on_end, B)
    check('U2 ON leaves the occluded remnant alone', b_off >= 15 and b_on >= 0.85 * b_off,
          'B column at the end: OFF %d, ON %d' % (b_off, b_on))
    # U3 out of view
    l_off, l_on = column(off_end, L, 0.1, (1.2, 1.6)), column(on_end, L, 0.1, (1.2, 1.6))
    check('U3 out-of-view object untouched', l_off >= 1 and l_on >= l_off, 'lamp cells: OFF %d, ON %d' % (l_off, l_on))
    # U4 beyond range_max
    f_off, f_on = column(off_end, F, 0.2), column(on_end, F, 0.2)
    check('U4 remnant beyond range_max untouched', f_off >= 8 and f_on >= 0.85 * f_off,
          'F column at the end: OFF %d, ON %d' % (f_off, f_on))
    # U5 real surfaces
    for name, fn in (('wall', wall_cells), ('ground', ground_cells), ('tube', lambda o: column(o, T, 0.15, (0.0, 1.0)))):
        v_off, v_on = fn(off_end), fn(on_end)
        check('U5 real %s keeps its cells' % name, v_off >= 10 and v_on >= 0.98 * v_off,
              '%s cells: OFF %d, ON %d' % (name, v_off, v_on))
    # the ground at grazing incidence is where a decay that double-counts real rays shows first
    bins = ((0.0, 3.0), (3.0, 5.0), (5.0, 7.5))
    print('     ground cells by horizontal range (OFF / ON): ' + '  '.join(
        '%g-%g m %d/%d' % (a, b, ground_cells_in(off_end, a, b), ground_cells_in(on_end, a, b)) for a, b in bins))
    # U6 cost
    ms = [float(mx) for _, mx in re.findall(r'pass ([0-9.]+) ms \(max ([0-9.]+)\)', results[True][1])]
    check('U6 decay pass cost', bool(ms) and max(ms) < 15.0,
          'max pass %.2f ms over %d log lines' % (max(ms) if ms else float('nan'), len(ms)))
    n_ok = sum(1 for _, ok, _ in checks if ok)
    print('== %d/%d' % (n_ok, len(checks)))
    if args.keep or n_ok != len(checks):
        print('logs: %s' % tmpdir)
    return 0 if n_ok == len(checks) else 1


if __name__ == '__main__':
    sys.exit(main())
