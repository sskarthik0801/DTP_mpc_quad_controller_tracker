#!/usr/bin/env python3
"""visibility_field_check.py -- gate for SUPER patch 0008: dtp_msgs/VisibilityField from the REAL fsm_node.

The field says, per azimuth around the locked person, how far out a viewpoint stays clear of the
inflated map and what fraction of the person it would see. follow_planner's goal_mode nmpc turns
it into a cost (docs/fact-nmpc-follow-goal.md, Stage 2). This script checks the PRODUCER, against
the same synthetic LiDAR scenes loss_oracle_check.py uses (it imports them), on that script's
domain (88): never run the two, or target_filter_check.py, at once.

  V1  seen => a field   every SEEN locked frame is answered with a field and with NO LossEvent --
                        the mirror of loss_oracle_check's O7. The shape is right (24 bins of
                        15 deg from -pi, the annulus 3-4 m), centre_p echoes the target.
  V2  open field        nothing is known to be occluded from anywhere: 1 - vis - unk <= 0.1 in
                        every bin that has a viewpoint at all. UNKNOWN is not occlusion.
  V3  the tube          a 0.4 m tube 2 m from the person blocks a NARROW run of azimuths centred
                        on the tube's bearing from the person, and nowhere more than 45 deg off
                        it. "Blocked" is what the consumer makes of a bin: known-occluded, or no
                        viewpoint at all (the near edge already inside the inflated map).
  V4  the wide trunk    a 1.0 m trunk at the same place blocks more of the bearings round it.
                        It shows up mostly as "no viewpoint": ROG-Map holds a trunk as a shell of
                        its sensor-lit face, so a ray from elsewhere enters through an unlit side
                        and reads UNKNOWN, not OCCUPIED -- the field knows occlusion only near
                        where the aircraft has looked from.
  V5  coasting too      a coasting target gets a field (missed_frames > 0) AND its LossEvent.
  V6  the floor         heights_used + heights_dropped is the configured sample count, and at
                        least one height is knowable in these scenes.
  V7  budget            eval_us over every field received: p99 <= 1.5 ms.
  V8  the switch        loss_oracle/visibility_enable false: no field at all, LossEvents as before.
  V9  end to end        the REAL fsm_node feeding the REAL follow_planner_node (goal_mode nmpc), and
                        the answer checked against the scene's GEOMETRY, not against either node:
                        a person standing still, with a tube exactly between them and the aircraft,
                        so the trail bearing runs straight through it. Stage 1
                        (nmpc_w_vis 0) parks the goal in the tube's shadow -- the segment from the
                        goal to the person passes through the tube. Stage 2 must end with that
                        segment CLEAR of it, still in the standoff band. Needs --planner-exe (or a
                        sourced follow_planner built with Stage 2).

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/visibility_field_check.py            # ~6 min (4 fsm_node starts)
    # a scratch build: source its overlay too, and tell the harness where ITS config dir is --
    # fsm_node resolves config_name against the ROOT_DIR it was compiled with.
    python3 drone-dtp/scripts/visibility_field_check.py --super-cfg-dir /path/to/super_planner/config
"""
import argparse
import math
import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import target_filter_check as tfc   # noqa: E402
import loss_oracle_check as loc     # noqa: E402

PERSON = (4.6, 1.2)                 # beside the tube, in plain view of the sensor at the origin
OPEN_PERSON = (4.0, -1.5)


class VisHarness(loc.OracleHarness):
    def __init__(self):
        super().__init__()
        from dtp_msgs.msg import VisibilityField
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                         depth=50, durability=DurabilityPolicy.VOLATILE)
        self.fields = []
        # A method, not self.fields.append: take_fields() REPLACES the list, and a callback bound to
        # the old list's append would go on filling a list nobody reads.
        self.node.create_subscription(VisibilityField, '/planning/visibility', self._on_field, qos)

    def _on_field(self, m):
        self.fields.append(m)

    def take_fields(self):
        f, self.fields = self.fields, []
        return f


def scene(objects, person, coast=False):
    def body(h, k):
        k, t = loc.map_scene(h, k, objects)
        h.take_fields()
        h.take_events()
        k, t = loc.seen_phase(h, k, t, person, objects, n=15)
        out = {'seen_fields': h.take_fields(), 'seen_events': h.take_events()}
        if coast:
            k, t, ev = loc.coast_phase(h, k, t, person, (0.0, 0.0), objects, n=8)
            out['coast_fields'] = h.take_fields()
            out['coast_events'] = ev
        return out
    return body


def seg_dist(a, b, c):
    """Distance from point c to segment a-b, all (x, y)."""
    ax, ay, bx, by, cx, cy = a[0], a[1], b[0], b[1], c[0], c[1]
    L2 = (bx - ax) ** 2 + (by - ay) ** 2
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((cx - ax) * (bx - ax) + (cy - ay) * (by - ay)) / L2))
    return math.hypot(ax + t * (bx - ax) - cx, ay + t * (by - ay) - cy)


def e2e_object():
    """A tube 2 m from the person, exactly on the line from the person to the aircraft (the
    origin). A person standing still has no heading, so follow_planner trails them from wherever
    the aircraft already is: the trail bearing IS that line, and it runs through this tube. This
    is the case the feature is for -- something has come between the aircraft and the person --
    and it is the geometry the map knows best, because the tube's lit face is the one facing the
    aircraft. (An earlier version steered the bearing with a made-up velocity on a person who
    never moved; the solver's friction term rightly followed the phantom motion.)"""
    b = math.atan2(-PERSON[1], -PERSON[0])
    return (PERSON[0] + 2.0 * math.cos(b), PERSON[1] + 2.0 * math.sin(b), 0.20, tfc.FLOOR, 1.8)


def e2e_scene(planner_exe, logpath, w_vis, obj):
    import signal
    import subprocess
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    v = (0.0, 0.0)

    def body(h, k):
        goals = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                         depth=5, durability=DurabilityPolicy.VOLATILE)
        sub = h.node.create_subscription(PoseStamped, '/dtp/follow_goal',
                                         lambda m: goals.append((m.pose.position.x, m.pose.position.y)), qos)
        log = open(logpath, 'w')
        # The scene harness publishes a frame every ~0.37 s of wall time (it waits for the map to
        # take each scan), so the planner's 0.3 s staleness gates are widened for this scene only.
        proc = subprocess.Popen([planner_exe, '--ros-args', '-p', 'goal_mode:=nmpc', '-p', 'nmpc_w_vis:=%s' % w_vis,
                                 '-p', 'nmpc_w_clear:=%s' % (4.0 if w_vis > 0 else 0.0),
                                 '-p', 'follow_target_timeout_s:=1.5', '-p', 'odom_timeout_s:=1.5',
                                 '-p', 'nmpc_warm_max_age_s:=1.5', '-p', 'nmpc_vis_timeout_s:=1.5',
                                 '-p', 'goal_status_timeout_s:=100.0', '-p', 'goal_z_min_m:=1.2'],
                                env=dict(os.environ, ROS_DOMAIN_ID=tfc.DOMAIN, ROS_LOCALHOST_ONLY='1'),
                                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        try:
            k, t = loc.map_scene(h, k, [obj])
            h.take_fields()
            k, t = loc.seen_phase(h, k, t, PERSON, [obj], n=45, v=v)
        finally:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=10)
            except Exception:
                os.killpg(proc.pid, signal.SIGKILL)
            log.close()
            h.node.destroy_subscription(sub)
        return {'goals': goals, 'planner_log': open(logpath).read(), 'fields': h.take_fields()}
    return body


def occluded(f):
    """Per bin: the KNOWN occluded fraction, or None where there is no viewpoint / no answer."""
    out = []
    for c, v, u in zip(f.clear_r, f.vis, f.unk):
        out.append(None if (math.isnan(c) or math.isnan(v) or math.isnan(u)) else 1.0 - v - u)
    return out


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def blocked(f):
    """Per bin, what follow_planner's VisibilityCallback makes of it: a bin with NO viewpoint (the
    near edge of the annulus is already inside the inflated map -- a trunk close to the person
    does that) is as bad as a fully occluded one; otherwise the KNOWN occluded fraction."""
    return [1.0 if math.isnan(c) else (0.0 if (math.isnan(v) or math.isnan(u)) else max(0.0, 1.0 - v - u))
            for c, v, u in zip(f.clear_r, f.vis, f.unk)]


def occluded_run(f, bearing, thresh=0.5):
    """(blocked bins within 45 deg of `bearing`; how far the furthest KNOWN-OCCLUDED bin is from it,
    in degrees; the blocked mass within 45 deg).

    "Only on the object's bearing" is asserted on the known-occluded bins alone. A bin with no
    viewpoint can be anywhere the scene has structure -- these scenes have a back wall, and a
    viewpoint cannot stand in it -- and that is correct, not a stray occlusion."""
    b = blocked(f)
    offs = [wrap(f.az0 + i * f.daz - bearing) for i in range(f.bins)]
    near = sum(1 for o, x in zip(offs, b) if x >= thresh and abs(o) <= math.pi / 4)
    known = [o for o, x in zip(offs, occluded(f)) if x is not None and x >= thresh]
    mass = sum(x for o, x in zip(offs, b) if abs(o) <= math.pi / 4)
    return near, (max(abs(k) for k in known) * 180.0 / math.pi if known else 0.0), mass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--logs', default=None)
    ap.add_argument('--only-e2e', action='store_true', help='run V9 alone (~4 min), while working on the consumer')
    ap.add_argument('--planner-exe', default=None, help='follow_planner_node for V9 (default: the sourced install)')
    ap.add_argument('--super-cfg-dir', default=None,
                    help="the config dir of the super_planner SOURCE tree fsm_node was built from")
    args = ap.parse_args(argv)
    if args.super_cfg_dir:
        tfc.SUPER_CFG_DIR = os.path.abspath(args.super_cfg_dir)
        tfc.BASE_CFG = os.path.join(tfc.SUPER_CFG_DIR, 'follow_test_ros2.yaml')
    os.environ['ROS_DOMAIN_ID'] = tfc.DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    tmpdir = args.logs or tempfile.mkdtemp(prefix='visibility_field_check_')
    os.makedirs(tmpdir, exist_ok=True)

    base = open(tfc.BASE_CFG).read()
    if not re.search(r'^  visibility_enable: (true|false)', base, re.M):
        raise SystemExit('%s has no loss_oracle/visibility_enable -- is patch 0008 applied?' % tfc.BASE_CFG)
    base = re.sub(r'^(  target_exclusion:\n    enable:) (true|false)', r'\1 false', base, flags=re.M)
    cfg_on = os.path.join(tmpdir, 'vis_on.yaml')
    cfg_off = os.path.join(tmpdir, 'vis_off.yaml')
    open(cfg_on, 'w').write(re.sub(r'^(  visibility_enable:) (true|false)', r'\1 true', base, flags=re.M))
    open(cfg_off, 'w').write(re.sub(r'^(  visibility_enable:) (true|false)', r'\1 false', base, flags=re.M))

    results = []

    def check(name, ok, detail=''):
        results.append(bool(ok))
        print('  %s  %-70s %s' % ('PASS' if ok else 'FAIL', name, detail))

    h = VisHarness()
    try:
        print('\nvisibility_field_check -- real fsm_node on ROS_DOMAIN_ID %s (localhost only); logs in %s\n'
              % (tfc.DOMAIN, tmpdir))

        if not args.only_e2e:
            # ---- open field --------------------------------------------------------------------------
            r = tfc.run_scenario(h, 'open', True, scene([], OPEN_PERSON), tmpdir, cfg_on, cfg_off)
            fs = r['seen_fields']
            check('V1 every seen frame gets a field (>= 13 of 15), and no LossEvent',
                  len(fs) >= 13 and len(r['seen_events']) == 0, 'fields=%d events=%d' % (len(fs), len(r['seen_events'])))
            shape_ok = all(f.bins == 24 and len(f.vis) == 24 and len(f.unk) == 24 and len(f.clear_r) == 24
                           and abs(f.az0 + math.pi) < 1e-5 and abs(f.daz - math.pi / 12) < 1e-5
                           and abs(f.r_near - 3.0) < 1e-5 and abs(f.r_far - 4.0) < 1e-5 for f in fs)
            check('V1 shape: 24 bins of 15 deg from -pi, annulus 3-4 m', bool(fs) and shape_ok)
            check('V1 centre_p echoes the target, missed_frames 0',
                  all(abs(f.centre_p.x - OPEN_PERSON[0]) < 1e-6 and abs(f.centre_p.y - OPEN_PERSON[1]) < 1e-6
                      and f.missed_frames == 0 for f in fs))
            last = fs[-1] if fs else None
            if last is not None:
                occ = [o for o in occluded(last) if o is not None]
                print('open   viewpoints %d/24, max known-occluded %.2f, mean vis %.2f, mean unk %.2f'
                      % (len(occ), max(occ) if occ else float('nan'),
                         sum(v for v in last.vis if not math.isnan(v)) / max(1, len(occ)),
                         sum(u for u in last.unk if not math.isnan(u)) / max(1, len(occ))))
                check('V2 open field: nothing is KNOWN to be occluded (<= 0.1 in every bin)',
                      len(occ) >= 12 and max(occ) <= 0.1, 'bins with a viewpoint: %d' % len(occ))
                check('V6 sample heights are accounted for, and at least one is knowable',
                      last.heights_used + last.heights_dropped == 3 and last.heights_used >= 1,
                      'used %d dropped %d' % (last.heights_used, last.heights_dropped))
            all_us = [f.eval_us for f in fs]

            # ---- the tube and the wide trunk -----------------------------------------------------------
            runs = {}
            for name, obj in (('tube', loc.TUBE), ('wide', loc.WIDE)):
                r = tfc.run_scenario(h, name, True, scene([obj], PERSON, coast=(name == 'tube')), tmpdir, cfg_on, cfg_off)
                all_us += [x.eval_us for x in r['seen_fields'] + r.get('coast_fields', [])]
                f = r['seen_fields'][-1] if r['seen_fields'] else None
                bearing = math.atan2(obj[1] - PERSON[1], obj[0] - PERSON[0])   # the object, seen FROM the person
                if f is None:
                    check('V3/V4 %s: a field arrived' % name, False)
                    continue
                n, off, mass = occluded_run(f, bearing)
                runs[name] = mass
                for i in range(f.bins):   # the bins round the object's bearing, as the map has them
                    d = wrap(f.az0 + i * f.daz - bearing) * 180.0 / math.pi
                    if abs(d) <= 52.0:
                        print('         %+4.0f deg off: clear_r %.1f  vis %.2f  unk %.2f  known-occluded %.2f'
                              % (d, f.clear_r[i], f.vis[i], f.unk[i], 1.0 - f.vis[i] - f.unk[i]))
                print('%-6s object bears %.0f deg from the person; %d bins blocked within 45 deg of it (mass %.2f); '
                      'the furthest known-occluded bin is %.0f deg off' % (name, bearing * 180.0 / math.pi, n, mass, off))
                check('%s: occluded from a run of azimuths on the object\'s bearing, and only there'
                      % ('V3 tube' if name == 'tube' else 'V4 wide trunk'), n >= 1 and off <= 45.0,
                      '%d blocked bins on the bearing, furthest known-occluded %.0f deg off' % (n, off))
                if name == 'tube':
                    cf, ce = r['coast_fields'], r['coast_events']
                    check('V5 a coasting target gets a field AND its LossEvent',
                          len(cf) >= 6 and len(ce) >= 6 and all(x.missed_frames > 0 for x in cf),
                          'fields=%d events=%d' % (len(cf), len(ce)))
            if 'tube' in runs and 'wide' in runs:
                check('V4 the wide trunk blocks more of the bearings round it than the tube',
                      runs['wide'] > runs['tube'], 'blocked mass: wide %.2f vs tube %.2f' % (runs['wide'], runs['tube']))

            # From the messages, not from fsm_node's own line: that prints once per 200 fields (25 s of
            # flight), and these scenes are ~23 frames each.
            us = sorted(all_us)
            check('V7 budget: visibility eval p99 <= 1500 us', bool(us) and us[min(len(us) - 1, int(0.99 * len(us)))] <= 1500,
                  'p50 %d us, p99 %d us, max %d us over %d fields'
                  % (us[len(us) // 2], us[min(len(us) - 1, int(0.99 * len(us)))], us[-1], len(us)) if us else 'no fields')

            # ---- the switch ------------------------------------------------------------------------------
            r = tfc.run_scenario(h, 'switch', False, scene([loc.TUBE], PERSON, coast=True), tmpdir, cfg_on, cfg_off)
            check('V8 visibility_enable false: no field at all, LossEvents as before',
                  len(r['seen_fields']) == 0 and len(r['coast_fields']) == 0 and len(r['coast_events']) >= 6,
                  'fields=%d events=%d' % (len(r['seen_fields']) + len(r['coast_fields']), len(r['coast_events'])))
        # ---- V9: end to end, against the scene's geometry ---------------------------------------
        import nmpc_check as nc
        planner = args.planner_exe or nc.default_exe()
        ends = {}
        for w_vis in (0.0, 10.0):
            tag = 'e2e_stage%d' % (2 if w_vis else 1)
            obj = e2e_object()
            r = tfc.run_scenario(h, tag, True, e2e_scene(planner, os.path.join(tmpdir, tag + '_planner.log'),
                                                          w_vis, obj), tmpdir, cfg_on, cfg_off)
            if r['fields'] and w_vis > 0:
                f = r['fields'][-1]
                bearing = math.atan2(obj[1] - PERSON[1], obj[0] - PERSON[0])
                print('e2e    the field fsm_node published (last of %d), heights used %d dropped %d:'
                      % (len(r['fields']), f.heights_used, f.heights_dropped))
                for i in range(f.bins):
                    d = wrap(f.az0 + i * f.daz - bearing) * 180.0 / math.pi
                    if abs(d) <= 38.0:
                        print('         %+4.0f deg off the trail bearing: clear_r %.1f  vis %.2f  unk %.2f  blocked %.2f'
                              % (d, f.clear_r[i], f.vis[i], f.unk[i], blocked(f)[i]))
            g = r['goals'][-10:]
            if not g:
                check('V9 end to end (%s): the planner published goals' % tag, False, 'see %s' % tmpdir)
                continue
            gx, gy = sum(x for x, _ in g) / len(g), sum(y for _, y in g) / len(g)
            ends[w_vis] = (seg_dist((gx, gy), PERSON, obj[:2]), math.hypot(gx - PERSON[0], gy - PERSON[1]))
            used = [float(x) for x in re.findall(r'\| vis ([\d.]+)', r['planner_log'])]
            print('e2e    nmpc_w_vis %.0f: goal (%.2f, %.2f), %.2f m from the person; its line of sight passes %.2f m '
                  'from the tube axis (radius %.2f); vis scale used, max %.2f'
                  % (w_vis, gx, gy, ends[w_vis][1], ends[w_vis][0], loc.TUBE[2], max(used) if used else float('nan')))
        if 0.0 in ends and 10.0 in ends:
            check('V9 Stage 1 parks the goal in the tube\'s shadow (that is what the scene is built to do)',
                  ends[0.0][0] <= loc.TUBE[2], 'line of sight %.2f m from the axis' % ends[0.0][0])
            check('V9 Stage 2 ends with a CLEAR line of sight past the tube, still in the band',
                  ends[10.0][0] >= loc.TUBE[2] + 0.10 and 2.75 <= ends[10.0][1] <= 4.0,
                  'line of sight %.2f m from the axis, %.2f m from the person' % ends[10.0])
    finally:
        h.close()

    print('\n%d/%d checks passed' % (sum(results), len(results)))
    return 0 if results and all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
