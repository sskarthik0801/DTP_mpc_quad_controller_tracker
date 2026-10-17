#!/usr/bin/env python3
"""M6.1 / M6.4 bench gate for the loss oracle, against the REAL fsm_node. No hardware.

WHAT IT PROVES
--------------
fsm_node's ROG-Map is fed a synthetic LiDAR scene and synthetic /dtp/follow_target messages on a
private localhost-only DDS domain (the target_filter_check harness), with loss_oracle/enable true.
Every COASTING target (missed_frames > 0) must come back as one dtp_msgs/LossEvent on
/planning/loss_event, and the answer must be the one the geometry dictates:

  O1  OCCLUDED     a person seen beside a 0.4 m tube steps behind it and the CTRV point sits behind
                   the tube: OCCLUDED, the occluder on the tube's near face, both edges found at the
                   tube's width, the reappearance belief exits on the side the person was walking
                   to, and a viewpoint is offered that keeps >= 2.5 m from the hidden person.
  O5  OUT_OF_VIEW  the same map, a CTRV point at 60 deg off the nose: OUT_OF_VIEW, no rays cast.
  O4  LOST         a person seen in the open vanishes (a detector miss): clear line of sight, LOST,
                   even though their own returns are still occupied cells (r_excl keeps them out).
  O2  UNMAPPED     a scan that never covered the bearing the CTRV point is at: the rays run through
                   unknown space, UNMAPPED.
  O3  TRAIL        the paper's trail case. A person walks straight away from the sensor, then
                   vanishes; the coast point is beyond their trail, which lies on the line of sight.
                   With the target filter OFF the trail is OCCUPIED cells decaying toward UNKNOWN, and
                   the oracle reports OCCLUDED or UNMAPPED -- the documented false verdict; the gate
                   only asserts it is NOT LOST, and prints the mix. With the filter ON the person's
                   cells were never written (nor observed: the filter masks misses too), and the
                   oracle's shadow rule reads that unknown corridor as the person's own, so it says
                   LOST. Found on this gate 2026-09-14: before the shadow rule BOTH arms read
                   UNMAPPED (unknown fraction 1.00 / 0.93).
  O6  BUDGET       over every event above: classify_us p99 <= 2 ms, eval_us p99 <= 10 ms (SS2.4).
  O7  SILENCE      seen targets (missed_frames == 0) are never answered.

M6.4b (docs/m6-belief-around-trees-plan.md), the two tree cases, each run with the B/B2 switches
(loss_oracle/rollout_collision, loss_oracle/hidden_look) OFF and ON in the same map:
  O8  AROUND       the O1 tube plus a second trunk (r 0.3) in the rollouts' path. OFF: the rollouts
                   walk through it and the exit test's 0.4 m stop-short records a re-emergence
                   INSIDE the trunk (the defect the plan documents). ON: rollouts deflect round it,
                   no cluster mean is inside the trunk, the dominant side is unchanged. A second
                   coast starts INSIDE the trunk: ON still carries a belief and logs the back-off.
  O9  BEHIND       a 1 m trunk, the person straight behind it walking away at 0.7 m/s: most
                   rollouts (0.62-0.84) stay in the widening shadow. OFF: yaw target = the minority
                   cluster's mean or query_p, never a look point. ON: yaw target = the hidden
                   rollouts' look point at 1.5 s, a viewpoint with a clear ray to it, >= 2.5 m
                   from the person, off the camera-trunk line; reappear_* identical to OFF.
  PAIRING          --dump FILE writes every event; --compare FILE checks the stream of a later run
                   field by field (minus *_us and stamps) against it -- with the switches OFF the
                   new binary must reproduce the old one bit for bit (the draw order is untouched).
                   Scene stamps start at FIXED_T0, not the wall clock, so per-event seeds repeat.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/loss_oracle_check.py          # ~20 min (8 fsm_node starts), exits non-zero on any FAIL
    python3 drone-dtp/scripts/loss_oracle_check.py --only E,F --dump base.json   # the M6.4b scenes
"""
import argparse
import json
import math
import os
import re
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import target_filter_check as tfc  # noqa: E402

TUBE = (3.0, 0.0, 0.20, tfc.FLOOR, 1.8)      # a 0.4 m "trunk" 3 m off the nose
# O8: a second trunk beside the tube's shadow, its centre 0.2 m BEHIND the person's line (x 4.2)
# and 0.35 m right of the line of sight. A rollout walking -y down x = 4 therefore crosses the
# trunk's LIT face (the arc the sensor sees, x < 4.2) and exits the tube's shadow (|y| > ~0.28)
# inside the footprint, 0.2 m from the centre -- and the trunk's near face is within r_excl
# (0.4) of that exit, so the exit ray stops short of it and reads free (the plan's SS1 defect).
# Eq. 1's rays and the extent sweep also stop 0.4 m short at this range, so the trunk never
# counts as the occluder of the coast point: the tube is. ROG-Map holds only the lit face; the
# back half (x > 4.2) and the interior are UNKNOWN, so B can keep the belief out of the mapped
# half only -- the assertions below say exactly that.
TRUNK2 = (4.2, -0.35, 0.30, tfc.FLOOR, 1.8)
O8B_COAST = (3.95, -0.25)
BODY_R = 0.25                                # loss_oracle/rollout_body_radius in the follow config
# O9: a wide trunk (r 0.5, a mature tree or a pillar) with the person straight behind it,
# walking away at 0.7 m/s. Measured on the pre-0005 binary, 2026-09-14 (12 events each):
# still_hidden_w behind an r 0.3 trunk at 1.0 m/s is 0.31-0.59 (median 0.39) -- NOT "most
# hidden", and the cluster viewpoint already looks behind the trunk there; r 0.5 at 1.0 m/s
# 0.47-0.72; r 0.5 at 0.7 m/s 0.62-0.84 (median 0.72); r 0.3 at 0.6 m/s 0.44-0.78. O1 is
# <= 0.09. hidden_look_min_w 0.5 (the hidden mass is the majority) therefore fires here with
# 0.12 of margin and never in O1/O8.
WIDE = (3.0, 0.0, 0.50, tfc.FLOOR, 1.8)
# Scene stamps (scans and targets) count from here, not from the wall clock, so the oracle's
# per-event seed (track id, missed frames, stamp) is the same on every run and two binaries
# can be compared event by event. Nothing in fsm_node compares a message stamp to its clock
# (odom freshness is arrival time; the target filter's age is scan stamp - target stamp).
FIXED_T0 = 1.9e9
P_R, P_TOP, P_Z = tfc.PERSON_R, tfc.PERSON_TOP, tfc.PERSON_Z


class OracleHarness(tfc.Harness):
    def __init__(self):
        super().__init__()
        from dtp_msgs.msg import LossEvent
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                         depth=50, durability=DurabilityPolicy.VOLATILE)
        self.events = []
        self.node.create_subscription(LossEvent, '/planning/loss_event', self._on_event, qos)

    def _on_event(self, m):
        self.events.append(m)

    def take_events(self):
        ev, self.events = self.events, []
        return ev


def map_scene(h, k0, objects, n=30):
    t0 = FIXED_T0
    for i in range(n):
        h.publish_scan(tfc.scan(k0 + i, objects), t0 + i * tfc.SCAN_DT)
        h.spin(tfc.PACE_S)
    return k0 + n, t0 + n * tfc.SCAN_DT


def seen_phase(h, k0, t0, person, objects, n=15, v=(0.0, 0.0)):
    """Scans with the person in the scene, targets missed_frames = 0."""
    px, py = person
    for i in range(n):
        ts = t0 + i * tfc.SCAN_DT
        h.publish_target((px, py, P_Z), v, ts - tfc.TARGET_LAG_S, missed=0)
        h.spin(0.02)
        h.publish_scan(tfc.scan(k0 + i, objects + [(px, py, P_R, tfc.FLOOR, P_TOP)]), ts)
        h.spin(tfc.PACE_S)
    return k0 + n, t0 + n * tfc.SCAN_DT


def coast_phase(h, k0, t0, query, v, objects, n=12):
    """Scans WITHOUT the person, targets coasting at `query` with missed_frames 1..n."""
    qx, qy = query
    h.take_events()
    for i in range(n):
        ts = t0 + i * tfc.SCAN_DT
        h.publish_target((qx, qy, P_Z), v, ts - tfc.TARGET_LAG_S, missed=i + 1)
        h.spin(0.02)
        h.publish_scan(tfc.scan(k0 + i, objects), ts)
        h.spin(tfc.PACE_S)
    h.spin(0.5)
    return k0 + n, t0 + n * tfc.SCAN_DT, h.take_events()


def cls_name(c):
    return ['OCCLUDED', 'UNMAPPED', 'LOST', 'OUT_OF_VIEW'][c] if 0 <= c <= 3 else '?'


def summarise(events):
    if not events:
        return 'no events'
    counts = {}
    for e in events:
        counts[cls_name(e.cls)] = counts.get(cls_name(e.cls), 0) + 1
    e = events[-1]
    return ('%s | last: blocked %.2f unknown %.2f rays %d bearing %.0f occ (%.2f, %.2f) '
            'edges L%s (%.2f,%.2f) R%s (%.2f,%.2f) '
            'clusters %s hidden %.2f yaw (%.2f, %.2f) vp %s (%.2f, %.2f, %.2f) classify %d us eval %d us'
            % (counts, e.blocked_fraction, e.unknown_fraction, e.rays_cast, e.bearing_deg,
               e.occluder_p.x, e.occluder_p.y, 'v' if e.edge_left_valid else '-', e.edge_left.x, e.edge_left.y,
               'v' if e.edge_right_valid else '-', e.edge_right.x, e.edge_right.y,
               [(int(s), round(float(w), 2), round(float(dt), 2), round(p.x, 2), round(p.y, 2))
                for s, w, dt, p in zip(e.reappear_side, e.reappear_w, e.reappear_dt, e.reappear_p)],
               e.still_hidden_w, e.yaw_target_p.x, e.yaw_target_p.y,
               'v' if e.viewpoint_valid else '-', e.viewpoint_p.x, e.viewpoint_p.y, e.viewpoint_p.z,
               e.classify_us, e.eval_us))


def event_dict(e):
    """Every field of a LossEvent as plain Python, for --dump / --compare."""
    P = lambda p: [p.x, p.y, p.z]
    return {'track_id': e.track_id, 'missed_frames': e.missed_frames, 'onset': e.onset, 'cls': e.cls,
            'blocked_fraction': e.blocked_fraction, 'unknown_fraction': e.unknown_fraction,
            'rays_cast': e.rays_cast, 'query_p': P(e.query_p), 'camera_p': P(e.camera_p),
            'bearing_deg': e.bearing_deg, 'occluder_p': P(e.occluder_p),
            'edge_left': P(e.edge_left), 'edge_right': P(e.edge_right),
            'edge_left_valid': e.edge_left_valid, 'edge_right_valid': e.edge_right_valid,
            'reappear_p': [P(p) for p in e.reappear_p], 'reappear_dt': list(e.reappear_dt),
            'reappear_dt_min': list(e.reappear_dt_min), 'reappear_dt_max': list(e.reappear_dt_max),
            'reappear_sigma_m': list(e.reappear_sigma_m), 'reappear_w': list(e.reappear_w),
            'reappear_side': [int(v) for v in e.reappear_side], 'still_hidden_w': e.still_hidden_w,
            'rollouts': e.rollouts, 'yaw_target_p': P(e.yaw_target_p), 'viewpoint_valid': e.viewpoint_valid,
            'viewpoint_p': P(e.viewpoint_p), 'classify_us': e.classify_us, 'eval_us': e.eval_us}


PAIR_IGNORE = ('classify_us', 'eval_us')     # stamps are not in event_dict at all
# Only the explicit switches-OFF arms pair with a pre-0005 dump: O1-O7 run on the shipped follow
# config, where the switches are ON since 0005, and the ON arms are meant to differ.
PAIR_KEYS = ('O8_off', 'O8b_off', 'O9_off')


def pair_compare(ref, cur):
    """Match events by missed_frames within each scene key; every shared field must be equal.
    Returns (matched, mismatched, missing, first_diff_text)."""
    matched = mismatched = missing = 0
    first = ''
    for key, ref_ev in ref.items():
        cur_ev = {e['missed_frames']: e for e in cur.get(key, [])}
        for re_ in ref_ev:
            ce = cur_ev.get(re_['missed_frames'])
            if ce is None:
                missing += 1
                continue
            bad = [f for f in re_ if f not in PAIR_IGNORE and re_[f] != ce.get(f)]
            if bad:
                mismatched += 1
                if not first:
                    first = '%s missed %d: %s ref=%r cur=%r' % (key, re_['missed_frames'], bad[0],
                                                               re_[bad[0]], ce.get(bad[0]))
            else:
                matched += 1
    return matched, mismatched, missing, first


def seg_clearance(a, b, c):
    """Horizontal distance from point c to segment a-b (all (x, y))."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((cx - ax) * vx + (cy - ay) * vy) / l2))
    return math.hypot(cx - (ax + t * vx), cy - (ay + t * vy))


def scenario_a(h, k):
    """O1 occluded behind the tube, O5 out of view, O4 lost in the open."""
    out = {}
    k, t = map_scene(h, k, [TUBE])
    # O1: the person is seen at y = +1.2 beside the tube, walking -y (to the RIGHT as seen from
    # the camera looking +x), then steps behind it.
    k, t = seen_phase(h, k, t, (4.6, 1.2), [TUBE], v=(0.0, -0.8))
    seen_leak = len(h.take_events())
    k, t, ev = coast_phase(h, k, t, (4.6, 0.0), (0.0, -0.8), [TUBE])
    out['O1'] = ev
    out['O7_seen_leak'] = seen_leak
    # O5: a coast point 60 deg off the nose.
    k, t, ev = coast_phase(h, k, t, (2.0, 3.46), (0.0, 0.0), [TUBE], n=6)
    out['O5'] = ev
    # O4: seen in the open at bearing -20 deg, then vanishes without moving.
    k, t = seen_phase(h, k, t, (4.0, -1.5), [TUBE])
    k, t, ev = coast_phase(h, k, t, (4.0, -1.5), (0.0, 0.0), [TUBE])
    out['O4'] = ev
    return out


def scenario_b(h, k):
    """O2: the scan only ever covered +-15 deg; the coast point is at 28 deg."""
    az_saved = tfc.AZ
    tfc.AZ = (-15.0, 15.0)
    try:
        k, t = map_scene(h, k, [])
        k, t, ev = coast_phase(h, k, t, (4.0 * math.cos(math.radians(28)), 4.0 * math.sin(math.radians(28))),
                               (0.0, 0.0), [])
    finally:
        tfc.AZ = az_saved
    return {'O2': ev}


def scenario_c(h, k):
    """O3: walk straight away from the sensor, vanish, coast beyond the trail."""
    k, t = map_scene(h, k, [])
    for i in range(25):
        ts = t + i * tfc.SCAN_DT
        x = 3.0 + 0.1 * i
        h.publish_target((x, 0.0, P_Z), (1.0, 0.0), ts - tfc.TARGET_LAG_S, missed=0)
        h.spin(0.02)
        h.publish_scan(tfc.scan(k + i, [(x, 0.0, P_R, tfc.FLOOR, P_TOP)]), ts)
        h.spin(tfc.PACE_S)
    k += 25
    t += 25 * tfc.SCAN_DT
    k, t, ev = coast_phase(h, k, t, (6.0, 0.0), (1.0, 0.0), [], n=8)
    occ = h.snapshot(time.time())
    trail = tfc.in_box(occ, (2.8, 5.6), (-0.4, 0.4), (tfc.FLOOR, P_TOP + 0.1))
    return {'O3': ev, 'trail_cells': len(trail)}


def scenario_e(h, k):
    """O8: the O1 tube plus TRUNK2 in the rollouts' path; a second coast starting inside TRUNK2."""
    objs = [TUBE, TRUNK2]
    k, t = map_scene(h, k, objs, n=40)
    k, t = seen_phase(h, k, t, (4.0, 1.2), objs, v=(0.0, -0.8))
    k, t, ev = coast_phase(h, k, t, (4.0, 0.2), (0.0, -0.8), objs)
    out = {'O8': ev, 'occ': h.snapshot(time.time())}
    # Start inside the trunk's MAPPED face: the CTRV forecast walked the person into it (case B
    # itself). ROG-Map's cells are floor-indexed (centres at (k + 1/2) res), and at 4.2 m the
    # trunk's lit face is the single column x in [3.9, 4.0) for y in [-0.6, -0.2] (read back
    # 2026-09-14), so the point sits at a cell centre -- (4.0, -0.2) was a cell BOUNDARY and read
    # free. Its rays still cross the tube (x = 3, |y| <= 0.15), so it classifies OCCLUDED.
    k, t, ev = coast_phase(h, k, t, O8B_COAST, (0.0, -0.8), objs)
    out['O8b'] = ev
    return out


O9_V = (0.7, 0.0)          # the walker's velocity (module-level so a sweep can vary it)
O9_SEEN = (3.5, 0.9)       # beside the trunk (0.2 m gap), visible from the camera
O9_COAST = (4.2, 0.0)


def scenario_f(h, k):
    """O9: a wide trunk, the person seen beside it walking +x, then straight behind it."""
    objs = [WIDE]
    k, t = map_scene(h, k, objs, n=40)
    k, t = seen_phase(h, k, t, O9_SEEN, objs, v=O9_V)
    k, t, ev = coast_phase(h, k, t, O9_COAST, O9_V, objs)
    return {'O9': ev, 'occ': h.snapshot(time.time())}


def inside_trunk(p, trunk, margin=0.1):
    return math.hypot(p.x - trunk[0], p.y - trunk[1]) < trunk[2] + margin


def in_mapped_half(p, trunk, margin=0.05):
    """Inside the footprint AND on the side the sensor (at the origin, looking +x) can see."""
    return inside_trunk(p, trunk, margin) and p.x < trunk[0] - margin


def occ_clearance_2d(occ, p, z_lo, z_hi):
    """Horizontal distance from point p (x, y) to the nearest occupied cell with z in [z_lo, z_hi]."""
    band = occ[(occ[:, 2] >= z_lo) & (occ[:, 2] <= z_hi)]
    if len(band) == 0:
        return float('inf')
    return float(np.min(np.hypot(band[:, 0] - p[0], band[:, 1] - p[1])))


def ray_occ_min_dist(occ, a, b, skip_start, stop_short):
    """Min 3-D distance from any /rog_map/occ cell centre to the segment a->b minus its first
    skip_start and last stop_short metres. A centre within half a cell of the segment is a cell the
    ray certainly enters (the cube's inscribed sphere); a graze of a cube's corner beyond that is
    within the map's resolution and ROG-Map's cell-stepping caster may legitimately not visit it."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    if L < 1e-9 or L - stop_short <= skip_start:
        return float('inf')
    u = d / L
    t = np.clip((occ - a) @ u, skip_start, L - stop_short)
    return float(np.min(np.linalg.norm(occ - (a + t[:, None] * u), axis=1)))


def ray_hits_occ(occ, a, b, skip_start, stop_short, half=0.5 * tfc.RES):
    """Does the 3-D segment a->b, minus its first skip_start and last stop_short metres, enter any
    occupied cell cube (half-width `half` about each /rog_map/occ point)? Exact, so it also counts
    corner grazes the map's caster may skip -- informational next to ray_occ_min_dist."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    if L < 1e-9 or L - stop_short <= skip_start:
        return False
    u = d / L
    t0, t1 = skip_start, L - stop_short
    # slab test per cell, vectorised
    lo, hi = occ - half, occ + half
    with np.errstate(divide='ignore', invalid='ignore'):
        tl = (lo - a) / u
        th = (hi - a) / u
    tmin = np.where(np.isfinite(tl), np.minimum(tl, th), -np.inf)
    tmax = np.where(np.isfinite(tl), np.maximum(tl, th), np.inf)
    # an axis with u == 0: inside the slab iff a is between lo and hi
    zero = np.abs(u) < 1e-12
    for k in range(3):
        if zero[k]:
            ok = (a[k] >= lo[:, k]) & (a[k] <= hi[:, k])
            tmin[:, k] = np.where(ok, -np.inf, np.inf)
            tmax[:, k] = np.where(ok, np.inf, -np.inf)
    enter = np.maximum(tmin.max(axis=1), t0)
    leave = np.minimum(tmax.min(axis=1), t1)
    return bool(np.any(enter <= leave))


def pct(vals, q):
    if not vals:
        return 0
    v = sorted(vals)
    return v[min(len(v) - 1, int(q * len(v)))]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--logs', default=None, help='keep fsm_node logs here (default: a temp dir)')
    ap.add_argument('--only', default=None, help='comma list of A,B,C,D,E,F (scenarios; E/F = M6.4b O8/O9)')
    ap.add_argument('--dump', default=None, help='write every event of every scene here (JSON)')
    ap.add_argument('--compare', default=None, help='a --dump file from an earlier run: the pairing check')
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = tfc.DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    tmpdir = args.logs or tempfile.mkdtemp(prefix='loss_oracle_check_')
    os.makedirs(tmpdir, exist_ok=True)
    base = open(tfc.BASE_CFG).read()
    if not re.search(r'^loss_oracle:\n  enable: true', base, re.M):
        raise SystemExit('%s has no loss_oracle block with enable: true -- is patch 0004 applied?' % tfc.BASE_CFG)
    cfg_off = os.path.join(tmpdir, 'filter_off.yaml')
    cfg_on = os.path.join(tmpdir, 'filter_on.yaml')
    off_txt = re.sub(r'^(  target_exclusion:\n    enable:) (true|false)', r'\1 false', base, flags=re.M)
    open(cfg_off, 'w').write(off_txt)
    open(cfg_on, 'w').write(re.sub(r'^(  target_exclusion:\n    enable:) (true|false)', r'\1 true', base, flags=re.M))
    # M6.4b: the B / B2 switches, filter off. A config without the keys (a pre-0005 tree) runs
    # as-is: that is the baseline arm, and the ON arm is then the same binary.
    b_keys = re.findall(r'^  (rollout_collision|hidden_look): (true|false)', base, re.M)
    cfg_b_off = os.path.join(tmpdir, 'belief_off.yaml')
    cfg_b_on = os.path.join(tmpdir, 'belief_on.yaml')
    open(cfg_b_off, 'w').write(re.sub(r'^(  (?:rollout_collision|hidden_look):) (true|false)', r'\1 false', off_txt, flags=re.M))
    open(cfg_b_on, 'w').write(re.sub(r'^(  (?:rollout_collision|hidden_look):) (true|false)', r'\1 true', off_txt, flags=re.M))
    if len(b_keys) < 2:
        print('NOTE: %s has no rollout_collision / hidden_look keys (patch 0005 not applied): the B/B2 ON arm '
              'is the same binary as OFF' % tfc.BASE_CFG)
    only = set(args.only.split(',')) if args.only else None
    dump = {}

    results = []
    all_events = []

    def check(name, ok, detail):
        results.append((name, ok))
        print('  %s  %-62s %s' % ('PASS' if ok else 'FAIL', name, detail))

    h = OracleHarness()
    try:
        print('\nloss_oracle_check -- real fsm_node on ROS_DOMAIN_ID %s (localhost only); logs in %s\n'
              % (tfc.DOMAIN, tmpdir))
        if only is None or 'A' in only:
            a = tfc.run_scenario(h, 'A', False, scenario_a, tmpdir, cfg_on, cfg_off)
            ev = a['O1']
            all_events += ev + a['O5'] + a['O4']
            dump.update({'O1': ev, 'O5': a['O5'], 'O4': a['O4']})
            print('O1  %s' % summarise(ev))
            check('O1 every coasting target is answered (>= 10 of 12)', len(ev) >= 10, 'events=%d' % len(ev))
            occl = [e for e in ev if e.cls == 0]
            check('O1 class OCCLUDED on >= 90 %% of answers', len(ev) > 0 and len(occl) >= 0.9 * len(ev),
                  '%d/%d' % (len(occl), len(ev)))
            check('O1 blocked fraction >= rho* (0.5)', all(e.blocked_fraction >= 0.5 for e in occl), '')
            check('O1 occluder on the tube\'s near face (x 2.5-3.0, |y| <= 0.35)',
                  all(2.5 <= e.occluder_p.x <= 3.0 and abs(e.occluder_p.y) <= 0.35 for e in occl), '')
            check('O1 both edges found', all(e.edge_left_valid and e.edge_right_valid for e in occl), '')
            check('O1 edges at the tube\'s width (left y > right y, 0.1 <= |y| <= 0.5)',
                  all(e.edge_left.y > e.edge_right.y and 0.1 <= e.edge_left.y <= 0.5
                      and -0.5 <= e.edge_right.y <= -0.1 for e in occl), '')
            dom_ok = [e for e in occl if len(e.reappear_side) > 0 and int(e.reappear_side[0]) == -1
                      and float(e.reappear_w[0]) >= 0.6]
            check('O1 dominant re-emergence on the RIGHT (the walking side), mass >= 0.6',
                  len(occl) > 0 and len(dom_ok) >= 0.9 * len(occl), '%d/%d' % (len(dom_ok), len(occl)))
            check('O1 dominant re-emergence point right of the tube (y <= -0.15) and 0.1-2.0 s away',
                  all(e.reappear_p[0].y <= -0.15 and 0.1 <= float(e.reappear_dt[0]) <= 2.0 for e in dom_ok), '')
            check('O1 still-hidden mass <= 0.3', all(e.still_hidden_w <= 0.3 for e in occl), '')
            vp = [e for e in occl if e.viewpoint_valid]
            check('O1 a viewpoint is offered', len(vp) >= 0.9 * len(occl), '%d/%d' % (len(vp), len(occl)))
            check('O1 viewpoint >= 2.5 m from the hidden person, z in [0.5, 3.0]',
                  all(math.hypot(e.viewpoint_p.x - 4.6, e.viewpoint_p.y) >= 2.5 and 0.5 <= e.viewpoint_p.z <= 3.0
                      for e in vp), '')
            check('O1 yaw target = the dominant cluster mean',
                  all(abs(e.yaw_target_p.x - e.reappear_p[0].x) < 1e-6 for e in dom_ok), '')
            onsets = [e for e in ev if e.onset]
            check('O1 exactly one onset, on the first answer', len(onsets) == 1 and ev[0].onset,
                  'onsets=%d' % len(onsets))
            check('O1 query_p echoes the coast point', all(abs(e.query_p.x - 4.6) < 1e-6 for e in ev), '')
            check('O7 seen targets are never answered', a['O7_seen_leak'] == 0, 'leaked=%d' % a['O7_seen_leak'])
            ev = a['O5']
            print('O5  %s' % summarise(ev))
            check('O5 OUT_OF_VIEW with no rays', len(ev) >= 4 and all(e.cls == 3 and e.rays_cast == 0 for e in ev),
                  'events=%d' % len(ev))
            check('O5 bearing ~60 deg', all(55.0 <= e.bearing_deg <= 65.0 for e in ev), '')
            ev = a['O4']
            print('O4  %s' % summarise(ev))
            lost = [e for e in ev if e.cls == 2]
            check('O4 LOST with a clear line of sight (>= 90 %%)', len(ev) >= 10 and len(lost) >= 0.9 * len(ev),
                  '%d/%d' % (len(lost), len(ev)))
            check('O4 blocked and unknown fractions both < 0.5',
                  all(e.blocked_fraction < 0.5 and e.unknown_fraction < 0.5 for e in ev), '')
        if only is None or 'B' in only:
            b = tfc.run_scenario(h, 'B', False, scenario_b, tmpdir, cfg_on, cfg_off)
            ev = b['O2']
            all_events += ev
            dump['O2'] = ev
            print('O2  %s' % summarise(ev))
            unm = [e for e in ev if e.cls == 1]
            check('O2 UNMAPPED where the scan never looked (>= 90 %%)', len(ev) >= 10 and len(unm) >= 0.9 * len(ev),
                  '%d/%d' % (len(unm), len(ev)))
            check('O2 unknown fraction >= rho*', all(e.unknown_fraction >= 0.5 for e in unm), '')
        if only is None or 'C' in only:
            c = tfc.run_scenario(h, 'C', False, scenario_c, tmpdir, cfg_on, cfg_off)
            ev = c['O3']
            all_events += ev
            dump['O3_off'] = ev
            print('O3 OFF  trail cells %d | %s' % (c['trail_cells'], summarise(ev)))
            check('O3 setup: the walk left a trail (filter off)', c['trail_cells'] >= 10, 'cells=%d' % c['trail_cells'])
            wrong = [e for e in ev if e.cls in (0, 1)]
            occl = [e for e in ev if e.cls == 0]
            check('O3 filter OFF: the trail reads OCCLUDED/UNMAPPED, never LOST (the documented false verdict)',
                  len(ev) >= 6 and len(wrong) == len(ev), '%d/%d (OCCLUDED %d)' % (len(wrong), len(ev), len(occl)))
            check('O3 filter OFF: any "occluder" reported is the trail (x 2.8-5.6)',
                  all(2.8 <= e.occluder_p.x <= 5.6 for e in occl), '')
        if only is None or 'D' in only:
            d = tfc.run_scenario(h, 'D', True, scenario_c, tmpdir, cfg_on, cfg_off)
            ev = d['O3']
            all_events += ev
            dump['O3_on'] = ev
            print('O3 ON   trail cells %d | %s' % (d['trail_cells'], summarise(ev)))
            check('O3 filter ON: no trail', d['trail_cells'] == 0, 'cells=%d' % d['trail_cells'])
            lost = [e for e in ev if e.cls == 2]
            check('O3 filter ON: the same loss reads LOST', len(ev) >= 6 and len(lost) >= 0.9 * len(ev),
                  '%d/%d' % (len(lost), len(ev)))
        if only is None or 'E' in only:
            # O8 AROUND, both arms of the B/B2 switches (run_scenario's `enable` picks the config).
            e_off = tfc.run_scenario(h, 'E', False, scenario_e, tmpdir, cfg_b_on, cfg_b_off)
            e_on = tfc.run_scenario(h, 'E', True, scenario_e, tmpdir, cfg_b_on, cfg_b_off)
            dump.update({'O8_off': e_off['O8'], 'O8b_off': e_off['O8b'], 'O8_on': e_on['O8'], 'O8b_on': e_on['O8b']})
            all_events += e_off['O8'] + e_off['O8b'] + e_on['O8'] + e_on['O8b']
            for arm, res in (('OFF', e_off), ('ON', e_on)):
                print('O8  %-3s %s' % (arm, summarise(res['O8'])))
                print('O8b %-3s %s' % (arm, summarise(res['O8b'])))
            occ_off = [e for e in e_off['O8'] if e.cls == 0]
            occ_on = [e for e in e_on['O8'] if e.cls == 0]
            check('O8 setup: OCCLUDED by the tube in both arms (>= 10 of 12 each)',
                  len(occ_off) >= 10 and len(occ_on) >= 10 and
                  all(2.5 <= e.occluder_p.x <= 3.0 and abs(e.occluder_p.y) <= 0.35 for e in occ_off + occ_on),
                  'off %d on %d' % (len(occ_off), len(occ_on)))
            right_off = [e for e in occ_off if len(e.reappear_side) and int(e.reappear_side[0]) == -1]
            check('O8 OFF: dominant re-emergence on the right', len(right_off) >= 0.9 * len(occ_off),
                  '%d/%d' % (len(right_off), len(occ_off)))
            z_lo, z_hi = P_Z - 0.5 - 0.06, P_Z + 0.4 + 0.06     # the sample heights' cells
            occ_map = e_off['occ']
            in_off = [e for e in right_off if in_mapped_half(e.reappear_p[0], TRUNK2, margin=0.0)]
            check('O8 OFF: the dominant cluster mean lies INSIDE the second trunk\'s mapped half (the documented defect)',
                  len(right_off) > 0 and len(in_off) >= 0.9 * len(right_off), '%d/%d' % (len(in_off), len(right_off)))
            d_occ_off = [occ_clearance_2d(occ_map, (e.reappear_p[0].x, e.reappear_p[0].y), z_lo, z_hi) for e in right_off]
            check('O8 OFF: the dominant cluster mean sits ON a mapped cell of the trunk (<= 0.1 m from one)',
                  len(d_occ_off) > 0 and sum(1 for d in d_occ_off if d <= 0.1) >= 0.9 * len(d_occ_off),
                  'min %.2f max %.2f' % (min(d_occ_off), max(d_occ_off)) if d_occ_off else '')
            right_on = [e for e in occ_on if len(e.reappear_side) and int(e.reappear_side[0]) == -1]
            check('O8 ON: dominant re-emergence still on the right', len(right_on) >= 0.9 * len(occ_on),
                  '%d/%d' % (len(right_on), len(occ_on)))
            d_on = [math.hypot(p.x - TRUNK2[0], p.y - TRUNK2[1]) for e in occ_on for p in e.reappear_p]
            in_on = [e for e in occ_on if any(in_mapped_half(p, TRUNK2) for p in e.reappear_p)]
            check('O8 ON: no cluster mean inside the trunk\'s mapped half (its lit face, x < 4.15)', len(in_on) == 0,
                  '%d/%d; centre distances min %.2f' % (len(in_on), len(occ_on), min(d_on) if d_on else -1))
            # B's actual promise: the belief never sits inside the MAPPED body of an obstacle. The
            # trunk's unlit back half and its interior are UNKNOWN to the map and a rollout may pass
            # there. The body probes are samples every half cell out to the body radius, and a
            # sample just outside an occupied cube can be a cell's half-diagonal (0.07) from its
            # centre, so the centre of a person the map vouches for is >= body radius - one cell
            # (0.15 m) from every occupied cell centre. Measured 2026-09-14: ON min 0.19, OFF 0.03.
            occ_map_on = e_on['occ']
            d_occ_on = [occ_clearance_2d(occ_map_on, (p.x, p.y), z_lo, z_hi) for e in occ_on for p in e.reappear_p]
            d_yaw_on = [occ_clearance_2d(occ_map_on, (e.yaw_target_p.x, e.yaw_target_p.y), z_lo, z_hi) for e in occ_on]
            check('O8 ON: every cluster mean and the yaw target keep >= body radius - one cell (%.2f m) from every mapped cell' % (BODY_R - tfc.RES),
                  len(d_occ_on) > 0 and all(d >= BODY_R - tfc.RES for d in d_occ_on + d_yaw_on),
                  'min %.2f (OFF %.2f)' % (min(d_occ_on + d_yaw_on), min(d_occ_off) if d_occ_off else -1))
            yaw_in_on = [e for e in occ_on if in_mapped_half(e.yaw_target_p, TRUNK2)]
            check('O8 ON: the yaw target is never inside the trunk\'s mapped half', len(yaw_in_on) == 0,
                  '%d/%d' % (len(yaw_in_on), len(occ_on)))
            dt_off = [float(e.reappear_dt[0]) for e in right_off]
            dt_on = [float(e.reappear_dt[0]) for e in right_on]
            print('O8  dominant dt: OFF med %.2f s  ON med %.2f s; ON still-hidden max %.2f'
                  % (sorted(dt_off)[len(dt_off) // 2] if dt_off else -1, sorted(dt_on)[len(dt_on) // 2] if dt_on else -1,
                     max(e.still_hidden_w for e in occ_on) if occ_on else -1))
            check('O8 ON: B2 does not fire (still-hidden < 0.5; the yaw target is a cluster mean)',
                  all(e.still_hidden_w < 0.5 and len(e.reappear_p) > 0 and
                      abs(e.yaw_target_p.x - e.reappear_p[0].x) < 1e-9 for e in occ_on), '')
            check('O8 ON: a viewpoint is still offered, >= 2.5 m from the hidden person',
                  len(occ_on) > 0 and all(e.viewpoint_valid and math.hypot(e.viewpoint_p.x - 4.0, e.viewpoint_p.y - 0.2) >= 2.5
                                          for e in occ_on), '')
            occb_off = [e for e in e_off['O8b'] if e.cls == 0]
            occb_on = [e for e in e_on['O8b'] if e.cls == 0]
            check('O8b setup: a coast INSIDE the trunk still classifies OCCLUDED (tube), both arms',
                  len(occb_off) >= 10 and len(occb_on) >= 10, 'off %d on %d' % (len(occb_off), len(occb_on)))
            check('O8b ON: every event still carries a belief (a cluster or a look point)',
                  len(occb_on) > 0 and all(len(e.reappear_p) > 0 or
                                           abs(e.yaw_target_p.x - e.query_p.x) + abs(e.yaw_target_p.y - e.query_p.y) > 1e-6
                                           for e in occb_on), '')
            check('O8b ON: the yaw target is never inside the trunk\'s mapped half',
                  all(not in_mapped_half(e.yaw_target_p, TRUNK2) for e in occb_on), '')
            d_yaw_b = [occ_clearance_2d(occ_map_on, (e.yaw_target_p.x, e.yaw_target_p.y), z_lo, z_hi) for e in occb_on]
            check('O8b ON: the yaw target keeps >= %.2f m from every mapped cell (OFF sits on one)' % (BODY_R - tfc.RES),
                  all(d >= BODY_R - tfc.RES for d in d_yaw_b),
                  'min %.2f, OFF %.2f' % (min(d_yaw_b) if d_yaw_b else -1,
                                         min(occ_clearance_2d(occ_map, (e.yaw_target_p.x, e.yaw_target_p.y), z_lo, z_hi) for e in occb_off) if occb_off else -1))
            backoff = len(re.findall(r'\[LossOracle\] rollout start .*backed off', e_on['log']))
            check('O8b ON: fsm_node logged the start back-off', backoff >= 1, 'lines=%d' % backoff)
            check('O8 OFF: fsm_node never logged a back-off (collision off)',
                  'backed off' not in e_off['log'], '')
        if only is None or 'F' in only:
            f_off = tfc.run_scenario(h, 'F', False, scenario_f, tmpdir, cfg_b_on, cfg_b_off)
            f_on = tfc.run_scenario(h, 'F', True, scenario_f, tmpdir, cfg_b_on, cfg_b_off)
            dump.update({'O9_off': f_off['O9'], 'O9_on': f_on['O9']})
            all_events += f_off['O9'] + f_on['O9']
            for arm, res in (('OFF', f_off), ('ON', f_on)):
                print('O9  %-3s %s' % (arm, summarise(res['O9'])))
            occ_off = [e for e in f_off['O9'] if e.cls == 0]
            occ_on = [e for e in f_on['O9'] if e.cls == 0]
            check('O9 setup: OCCLUDED by the wide trunk in both arms (>= 10 of 12 each)',
                  len(occ_off) >= 10 and len(occ_on) >= 10 and
                  all(2.3 <= e.occluder_p.x <= 3.0 and abs(e.occluder_p.y) <= 0.6 for e in occ_off + occ_on),
                  'off %d on %d' % (len(occ_off), len(occ_on)))
            hid_off = [e.still_hidden_w for e in occ_off]
            check('O9 OFF: most rollouts stay hidden (still_hidden_w >= 0.5 on >= 90 %%)',
                  len(hid_off) > 0 and sum(1 for w in hid_off if w >= 0.5) >= 0.9 * len(hid_off),
                  'min %.2f med %.2f' % (min(hid_off or [0]), sorted(hid_off or [0])[len(hid_off) // 2]))

            def yaw_is(e, p):
                return abs(e.yaw_target_p.x - p.x) < 1e-9 and abs(e.yaw_target_p.y - p.y) < 1e-9
            plain_off = [e for e in occ_off if (len(e.reappear_p) == 0 and yaw_is(e, e.query_p)) or
                         (len(e.reappear_p) > 0 and yaw_is(e, e.reappear_p[0]))]
            check('O9 OFF: yaw target = query_p (no cluster) or the dominant cluster mean -- never a look point',
                  len(plain_off) == len(occ_off), '%d/%d' % (len(plain_off), len(occ_off)))
            look_on = [e for e in occ_on if not yaw_is(e, e.query_p) and
                       not (len(e.reappear_p) > 0 and yaw_is(e, e.reappear_p[0]))]
            check('O9 ON: yaw target is the look point (neither query_p nor reappear_p[0])',
                  len(occ_on) > 0 and len(look_on) >= 0.9 * len(occ_on), '%d/%d' % (len(look_on), len(occ_on)))
            check('O9 ON: the look point is behind the trunk, ahead of the person, short of the wall (x in [4.5, 6.8], |y| <= 0.7)',
                  all(4.5 <= e.yaw_target_p.x <= 6.8 and abs(e.yaw_target_p.y) <= 0.7 for e in look_on), '')
            vp_on = [e for e in look_on if e.viewpoint_valid]
            check('O9 ON: a viewpoint is offered', len(look_on) > 0 and len(vp_on) >= 0.9 * len(look_on),
                  '%d/%d' % (len(vp_on), len(look_on)))
            check('O9 ON: viewpoint >= 2.5 m from the hidden person, z in [0.5, 3.0]',
                  all(math.hypot(e.viewpoint_p.x - 4.2, e.viewpoint_p.y) >= 2.5 and 0.5 <= e.viewpoint_p.z <= 3.0
                      for e in vp_on), '')
            # The oracle promises a camera ray from the viewpoint to the look point with no OCCUPIED
            # cell, cast at the top sample height (P_Z + 0.4). Check that against the map the
            # harness read back, in that height band, and that the ray is not through the trunk's
            # middle (the mapped shell is sparse near its tangent points, so a ray may graze it).
            z_ray = P_Z + 0.4
            segs = [((e.viewpoint_p.x, e.viewpoint_p.y, e.viewpoint_p.z), (e.yaw_target_p.x, e.yaw_target_p.y, z_ray)) for e in vp_on]
            dmin = [ray_occ_min_dist(f_on['occ'], a, b, 0.3, 0.4) for a, b in segs]
            grazes = sum(ray_hits_occ(f_on['occ'], a, b, 0.3, 0.4) for a, b in segs)
            geo = [seg_clearance((e.viewpoint_p.x, e.viewpoint_p.y), (e.yaw_target_p.x, e.yaw_target_p.y), (WIDE[0], WIDE[1]))
                   for e in vp_on]
            # The oracle's castRay saw no OCCUPIED cell on each of these rays when it answered; the
            # harness reads the map back once, up to 5 s later, and the rays pass the trunk's -y
            # TANGENT, whose cells are marginal (grazing hits against misses from rays that reach
            # the wall behind) and flicker. Measured 2026-09-14: 6 of 12 rays graze a cube corner
            # in the read-back map, the nearest centre 0.037 m off one ray. So: the geometric floor
            # on every event, the cell-centre test on the majority, and the numbers printed. The
            # viewpoint search's clear-ray criterion on a sparse shell is an open item in
            # docs/m6-loss-oracle.md (an inflated-map ray, or a margin, would settle it).
            clear = sum(1 for d in dmin if d >= 0.5 * tfc.RES)
            check('O9 ON: the viewpoint\'s ray to the look point (first 0.3 m and last 0.4 m skipped, as the oracle casts it) passes >= 0.35 m from the trunk centre on every event and no occupied cell centre within half a cell on most',
                  all(g >= 0.35 for g in geo) and clear >= 0.5 * len(dmin) and len(dmin) > 0,
                  'clear of every cell centre %d/%d, nearest %.3f m; exact-cube corner grazes %d/%d; centre clearance min %.2f (r %.1f: the shell is sparse near its tangents)'
                  % (clear, len(dmin), min(dmin) if dmin else -1, grazes, len(segs), min(geo) if geo else -1, WIDE[2]))
            check('O9 ON: the viewpoint is off the camera-trunk line (|y| >= 0.3) -- the sidestep',
                  all(abs(e.viewpoint_p.y) >= 0.3 for e in vp_on), '')
            check('O9 ON: reappear_* untouched by B2 -- same cluster count and sides as OFF, event by event',
                  len(occ_on) == len(occ_off) and
                  all(list(a.reappear_side) == list(b.reappear_side) for a, b in zip(occ_off, occ_on)),
                  '' if len(occ_on) == len(occ_off) else 'counts differ (timing): off %d on %d' % (len(occ_off), len(occ_on)))
        if all_events:
            c99 = pct([e.classify_us for e in all_events], 0.99)
            e99 = pct([e.eval_us for e in all_events], 0.99)
            print('O6  %d events: classify p50 %d / p99 %d us, eval p50 %d / p99 %d us'
                  % (len(all_events), pct([e.classify_us for e in all_events], 0.5), c99,
                     pct([e.eval_us for e in all_events], 0.5), e99))
            check('O6 classification p99 <= 2 ms', c99 <= 2000, '%d us' % c99)
            check('O6 full evaluation p99 <= 10 ms', e99 <= 10000, '%d us' % e99)
        if args.dump:
            with open(args.dump, 'w') as f:
                json.dump({k: [event_dict(e) for e in v] for k, v in dump.items()}, f)
            print('dumped %d scene streams (%d events) to %s'
                  % (len(dump), sum(len(v) for v in dump.values()), args.dump))
        if args.compare:
            ref = json.load(open(args.compare))
            cur = {k: [event_dict(e) for e in v] for k, v in dump.items()}
            shared = [k for k in PAIR_KEYS if k in ref and k in cur]
            matched, mismatched, missing, first = pair_compare({k: ref[k] for k in shared}, cur)
            print('PAIRING vs %s: scenes %s: %d matched, %d mismatched, %d missing%s'
                  % (args.compare, ','.join(shared), matched, mismatched, missing, ('; first: ' + first) if first else ''))
            check('PAIRING every matched event equals the reference field by field (minus *_us)',
                  matched > 0 and mismatched == 0, '%d matched, %d mismatched' % (matched, mismatched))
            check('PAIRING >= 90 %% of the reference events were matched by missed_frames',
                  matched >= 0.9 * (matched + missing), '%d/%d' % (matched, matched + missing))
    finally:
        h.close()

    bad = [n for n, ok in results if not ok]
    print('\n%d/%d checks passed%s'
          % (len(results) - len(bad), len(results), '' if not bad else ' -- FAILED: ' + ', '.join(bad)))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
