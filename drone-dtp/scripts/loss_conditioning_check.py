#!/usr/bin/env python3
"""M6.5 bench gate for loss conditioning, against the REAL target_manager and follow_planner.

No hardware, no fsm_node: this script plays the oracle. It runs both nodes on a private
localhost-only DDS domain, feeds them odom (drone at the origin, nose along +x) and an 8 Hz
/dtp/targets stream, and answers every COASTING /dtp/follow_target with a synthetic
dtp_msgs/LossEvent of a chosen class -- exactly what fsm_node's oracle would publish -- then
reads what the two nodes do about it. Each scene runs in BOTH arms (loss_conditioning false, then
true) from a fresh pair of nodes, so the gate asserts the DIFFERENCE the switch makes, not just
the ON behaviour:

  S1  LOST     the person vanishes in clear view; every answer is LOST.
               OFF: coasts the forecast out (~1.3 s), then LOST on "forecast".
               ON:  LOST within lost_confirm_s + a few frames, reason names the oracle.
  S2  OCCLUDED the person vanishes; every answer is OCCLUDED with a belief 1.5 m to the right and
               0.8-1.4 s away, and a viewpoint.
               OFF: follow_target stops at ~1.3 s; the goal stays the standoff; from 0.4 s of
                    stamp age the yaw target is follow_planner's coast yaw hold: one heading toward
                    the CTRV point, stamped now, carried through the loss (since 2026-09-26; with
                    --no-coast-yaw-hold it keeps the perception stamp and SUPER's 0.5 s gate drops
                    it, the documented carry-forward arm).
               ON:  follow_target still published past the forecast, until the belief's
                    t_max + margin (1.9 s); the goal IS the oracle's viewpoint; the yaw target IS
                    the re-emergence point, stamped fresh; after the loss the yaw hold keeps that
                    bearing (the camera does not swing back to the CTRV point).
  S3  REGION   as S2, and 1.0 s into the coast a stranger appears 1.0 m from the CTRV point --
               inside the ordinary band, outside the belief's region.
               OFF: re-locks onto the stranger (the wrong lock the paper counts).
               ON:  refuses; then a candidate inside the region is re-locked at once.
  S4  OUT_OF_VIEW  every answer is OUT_OF_VIEW.
               OFF: the coast yaw hold (--no-coast-yaw-hold: the perception stamp).
               ON:  yaw target = the CTRV point, fresh.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/loss_conditioning_check.py      # ~2 min, exits non-zero on any FAIL
    python3 drone-dtp/scripts/loss_conditioning_check.py --no-coast-yaw-hold   # the pre-2026-09-26 arm
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '89'
DETECT_HZ = 8.0
PERSON_Z = 0.9


class Nodes:
    goal_mode = None   # --goal-mode: run follow_planner in this arm (default: the node's own)
    coast_yaw_hold = True   # --no-coast-yaw-hold: follow_planner's coast_yaw_hold false

    def __init__(self, conditioning, logdir):
        env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')
        arm = 'on' if conditioning else 'off'
        self.logs = [open(os.path.join(logdir, 'target_manager_%s.log' % arm), 'w'),
                     open(os.path.join(logdir, 'follow_planner_%s.log' % arm), 'w')]
        tm_args = ['ros2', 'run', 'dtp_perception', 'target_manager', '--ros-args',
                   '-p', 'follow_enabled:=true', '-p', 'launch_ignore_radius_m:=0.0',
                   '-p', 'acquire_motion_m:=0.0', '-p', 'world_frame:=lidar_init',
                   '-p', 'loss_conditioning:=%s' % ('true' if conditioning else 'false')]
        fp_args = ['ros2', 'run', 'follow_planner', 'follow_planner_node', '--ros-args',
                   '-p', 'loss_conditioning:=%s' % ('true' if conditioning else 'false'),
                   '-p', 'goal_status_timeout_s:=100.0']
        if Nodes.goal_mode:
            fp_args += ['-p', 'goal_mode:=%s' % Nodes.goal_mode]
        if not Nodes.coast_yaw_hold:
            fp_args += ['-p', 'coast_yaw_hold:=false']
        self.procs = [subprocess.Popen(a, env=env, stdout=lg, stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL, start_new_session=True)
                      for a, lg in zip((tm_args, fp_args), self.logs)]

    def stop(self):
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in self.procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for lg in self.logs:
            lg.close()


class Harness:
    def __init__(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped, PointStamped
        from dtp_msgs.msg import Target, TargetArray, LossEvent
        from rcl_interfaces.srv import SetParameters
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('loss_conditioning_check')
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                        depth=1, durability=DurabilityPolicy.VOLATILE)
        self.odom_pub = self.node.create_publisher(Odometry, '/lidar_slam/odom', be)
        self.targets_pub = self.node.create_publisher(TargetArray, '/dtp/targets', be)
        self.event_pub = self.node.create_publisher(LossEvent, '/planning/loss_event', be)
        self.follow_targets = []
        self.states = []
        self.goals = []
        self.yaws = []
        self.node.create_subscription(Target, '/dtp/follow_target', self._on_ft, qos_profile_sensor_data)
        self.node.create_subscription(String, '/dtp/lock_state', self._on_state, 10)
        self.node.create_subscription(PoseStamped, '/dtp/follow_goal', self._on_goal, be)
        self.node.create_subscription(PointStamped, '/planning/yaw_target', self._on_yaw, be)
        self.param_cli = self.node.create_client(SetParameters, '/target_manager/set_parameters')
        self.Target, self.TargetArray, self.LossEvent = Target, TargetArray, LossEvent
        self.answer = None          # callable(follow_target) -> LossEvent or None
        self.next_odom = 0.0
        self.seq = 0

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_ft(self, m):
        self.follow_targets.append((self.now(), m))
        if m.missed_frames > 0 and self.answer is not None:
            ev = self.answer(m)
            if ev is not None:
                self.event_pub.publish(ev)

    def _on_state(self, m):
        self.states.append((self.now(), m.data))

    def _on_goal(self, m):
        self.goals.append((self.now(), m))

    def _on_yaw(self, m):
        self.yaws.append((self.now(), m))

    def spin(self, secs):
        end = time.time() + secs
        while True:
            t = time.time()
            if t >= self.next_odom:
                from nav_msgs.msg import Odometry
                o = Odometry()
                o.header.stamp = self.node.get_clock().now().to_msg()
                o.header.frame_id = 'lidar_init'
                o.child_frame_id = 'lidar_imu'
                o.pose.pose.orientation.w = 1.0
                self.odom_pub.publish(o)
                self.next_odom = t + 0.1
            left = end - time.time()
            if left <= 0:
                return
            self.rclpy.spin_once(self.node, timeout_sec=min(0.01, left))

    def clear(self):
        self.follow_targets, self.states, self.goals, self.yaws = [], [], [], []

    def target(self, track, x, y, vx=0.0, vy=0.0, stamp=None):
        from geometry_msgs.msg import Point
        t = self.Target()
        t.header.stamp = stamp if stamp is not None else self.node.get_clock().now().to_msg()
        t.header.frame_id = 'lidar_init'
        t.track_id = track
        t.position.x, t.position.y, t.position.z = x, y, PERSON_Z
        t.velocity.x, t.velocity.y = vx, vy
        t.track_confirmed = True
        t.confidence = 0.9
        t.distance_m = math.hypot(x, y)
        for i in range(18):
            dt = 0.2 + 0.075 * i
            t.predicted_path.append(Point(x=x + vx * dt, y=y + vy * dt, z=PERSON_Z))
            t.predicted_path_dt.append(dt)
        return t

    def publish_targets(self, *targets):
        arr = self.TargetArray()
        arr.header.stamp = self.node.get_clock().now().to_msg()
        arr.header.frame_id = 'lidar_init'
        arr.targets.extend(targets)
        self.targets_pub.publish(arr)

    def frames(self, n, make_targets):
        """n detection frames at DETECT_HZ; make_targets(i) -> list of Target."""
        for i in range(n):
            self.publish_targets(*make_targets(i))
            self.spin(1.0 / DETECT_HZ)

    def event(self, ft, cls, reappear=None, dt=(0.8, 1.4, 1.1), sigma=0.15, viewpoint=None):
        from geometry_msgs.msg import Point
        ev = self.LossEvent()
        ev.header = ft.header
        ev.eval_stamp = self.node.get_clock().now().to_msg()
        ev.track_id = ft.track_id
        ev.missed_frames = ft.missed_frames
        ev.cls = cls
        ev.blocked_fraction = 1.0 if cls == self.LossEvent.CLS_OCCLUDED else 0.0
        ev.query_p = ft.position
        ev.yaw_target_p = ft.position
        if reappear is not None:
            ev.reappear_p.append(Point(x=reappear[0], y=reappear[1], z=PERSON_Z))
            ev.reappear_dt_min.append(dt[0])
            ev.reappear_dt_max.append(dt[1])
            ev.reappear_dt.append(dt[2])
            ev.reappear_sigma_m.append(sigma)
            ev.reappear_w.append(0.9)
            ev.reappear_side.append(-1)
            ev.yaw_target_p = Point(x=reappear[0], y=reappear[1], z=PERSON_Z)
        if viewpoint is not None:
            ev.viewpoint_valid = True
            ev.viewpoint_p = Point(x=viewpoint[0], y=viewpoint[1], z=viewpoint[2])
        return ev

    def set_follow(self, enabled):
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from rcl_interfaces.srv import SetParameters
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'follow_enabled'
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled)
        req.parameters = [p]
        fut = self.param_cli.call_async(req)
        end = time.time() + 5.0
        while not fut.done() and time.time() < end:
            self.spin(0.05)
        return fut.done() and fut.result() is not None and fut.result().results[0].successful

    def reengage(self):
        self.set_follow(False)
        self.spin(0.3)
        self.set_follow(True)
        self.spin(0.3)

    def wait_state(self, prefix, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.spin(0.05)
            for t, s in self.states:
                if s.startswith(prefix):
                    return t, s
        return None

    def wait_ready(self):
        end = time.time() + 30.0
        while time.time() < end:
            # target_manager has NO timer: /dtp/lock_state goes out at startup and then only from
            # its /dtp/targets callback. Waiting in silence therefore sees a state only if this
            # subscription happened to match before the startup message -- a race that read as
            # "nodes did not come up" about one run in three (0 lock_state messages, parameter
            # service ready, /dtp/targets matched). An empty array is what a live perception
            # node sends when it sees nobody.
            self.publish_targets()
            self.spin(0.2)
            up = any(s.startswith('ACQUIRING') or s.startswith('IDLE') for _, s in self.states)
            if self.param_cli.service_is_ready() and up:
                return True
        return False


def walk_then_vanish(h, seen_frames=16):
    """The person walks +x at 1 m/s from (4, 0) for seen_frames, then vanishes. Returns onset time."""
    h.frames(seen_frames, lambda i: [h.target(7, 4.0 + i / DETECT_HZ, 0.0, vx=1.0)])
    return h.now()


def last_seen_stamp(h):
    for t, m in reversed(h.follow_targets):
        if m.missed_frames == 0:
            return m.header.stamp
    return None


def run_arm(conditioning, logdir, check):
    arm = 'ON' if conditioning else 'OFF'
    h = Harness()
    nodes = Nodes(conditioning, logdir)
    out = {}
    try:
        if not h.wait_ready():
            # Say what was and was not seen: "did not come up" has so far always been the HARNESS
            # not seeing nodes that had logged themselves up (2026-09-20, five times in a day).
            raise RuntimeError('nodes did not come up (arm %s); see %s. lock_state messages seen: %d (last %r); '
                               'target_manager parameter service ready: %s; subscribers on /dtp/targets: %d'
                               % (arm, logdir, len(h.states), h.states[-1][1] if h.states else None,
                                  h.param_cli.service_is_ready(), h.node.count_subscribers('/dtp/targets')))
        h.spin(1.0)

        # ---- S1 LOST ----------------------------------------------------------------------
        h.clear()
        h.answer = lambda ft: h.event(ft, h.LossEvent.CLS_LOST)
        onset = walk_then_vanish(h)
        h.frames(24, lambda i: [])
        lost = [(t, s) for t, s in h.states if s.startswith('LOST')]
        out['S1'] = (onset, lost, list(h.follow_targets))
        h.answer = None
        h.reengage()

        # ---- S2 OCCLUDED ------------------------------------------------------------------
        h.clear()
        vp = (3.0, -1.0, 1.4)     # 3.2 m from where the person vanished: floor-safe
        rp = (7.5, -3.0)          # > the 2.5 m band cap from every CTRV point: only the region reaches it

        def occl(ft):
            # A physically consistent oracle: the re-emergence is at coast age 0.8-1.4 s, so the
            # offsets it reports shrink as the coast ages (the real oracle re-rolls from the
            # coasted point each frame and behaves the same way).
            age = ft.missed_frames / DETECT_HZ
            return h.event(ft, h.LossEvent.CLS_OCCLUDED, reappear=rp, viewpoint=vp,
                           dt=(0.8 - age, 1.4 - age, 1.1 - age))  # noqa: E127
        h.answer = occl
        onset = walk_then_vanish(h)
        h.frames(24, lambda i: [])       # 2.5 s of coasting frames, then 0.5 s after the loss
        out['S2'] = (onset, list(h.follow_targets), list(h.goals), list(h.yaws), list(h.states))
        h.answer = None
        h.reengage()

        # ---- S3 REGION --------------------------------------------------------------------
        h.clear()
        h.answer = occl
        onset = walk_then_vanish(h)
        stranger_from = int(1.0 * DETECT_HZ)      # window [0.3, 1.9] s is open from 1.0 s on
        ft_ref = None

        def s3(i):
            nonlocal ft_ref
            if i < stranger_from:
                return []
            if ft_ref is None:
                ft_ref = h.follow_targets[-1][1].position
            # inside the band (1.0 m from the CTRV point), outside the region (r 0.5 at rp)
            return [h.target(21, ft_ref.x, ft_ref.y + 1.0)]
        h.frames(stranger_from + 4, s3)             # stranger visible 1.0-1.5 s
        relocked_stranger = any(s.startswith('LOCKED|21') for _, s in h.states)
        # now a candidate inside the region, still inside the window (1.5-2.0 s), and 3 m from
        # the CTRV point -- outside the band, so only the region gate can accept it
        h.frames(4, lambda i: [h.target(22, rp[0] + 0.2, rp[1] - 0.1)])
        region_states = [s for _, s in h.states if s.startswith('LOCKED|22')]
        relocked_region = bool(region_states)
        via_region = any('re-emergence region' in s for s in region_states)
        out['S3'] = (onset, relocked_stranger, relocked_region, via_region, list(h.states))
        h.answer = None
        h.reengage()

        # ---- S4 OUT_OF_VIEW ---------------------------------------------------------------
        h.clear()
        h.answer = lambda ft: h.event(ft, h.LossEvent.CLS_OUT_OF_VIEW)
        onset = walk_then_vanish(h)
        h.frames(8, lambda i: [])
        out['S4'] = (onset, list(h.follow_targets), list(h.yaws))
        h.answer = None
    finally:
        nodes.stop()
        h.close()
    return out


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


def far(m):
    """follow_planner's yaw hold publishes its heading as a point 100 m out."""
    return math.hypot(m.point.x, m.point.y) > 50.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--logs', default=None)
    ap.add_argument('--no-coast-yaw-hold', action='store_true',
                    help="follow_planner coast_yaw_hold false: the documented carry-forward arm's coast yaw")
    ap.add_argument('--goal-mode', default=None, choices=('geometric', 'nmpc'),
                    help="run follow_planner in this goal_mode (default: the node's own, geometric)")
    args = ap.parse_args(argv)
    Nodes.goal_mode = args.goal_mode
    Nodes.coast_yaw_hold = not args.no_coast_yaw_hold
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = args.logs or tempfile.mkdtemp(prefix='loss_conditioning_check_')
    os.makedirs(logdir, exist_ok=True)
    results = []

    def check(name, ok, detail=''):
        results.append((name, ok))
        print('  %s  %-70s %s' % ('PASS' if ok else 'FAIL', name, detail))

    print('\nloss_conditioning_check -- real target_manager + follow_planner on ROS_DOMAIN_ID %s; logs in %s\n'
          % (DOMAIN, logdir))
    off = run_arm(False, logdir, check)
    on = run_arm(True, logdir, check)

    # S1
    for arm, res in (('OFF', off), ('ON', on)):
        onset, lost, fts = res['S1']
        t_lost = lost[0][0] - onset if lost else None
        coasted = [t for t, m in fts if m.missed_frames > 0]
        print('S1 %s  LOST after %s s, %d coasting targets published, reason: %s'
              % (arm, '%.2f' % t_lost if t_lost is not None else 'never', len(coasted),
                 lost[0][1] if lost else '-'))
        if arm == 'OFF':
            check('S1 OFF: LOST verdicts are ignored; the coast runs the forecast out (>= 1.0 s)',
                  t_lost is not None and t_lost >= 1.0 and 'loss oracle' not in lost[0][1],
                  '%.2f s' % t_lost if t_lost else 'never')
        else:
            check('S1 ON: LOST after lost_confirm_s (0.3 s) + a few frames, <= 0.8 s',
                  t_lost is not None and 0.25 <= t_lost <= 0.8, '%.2f s' % t_lost if t_lost else 'never')
            check('S1 ON: the reason names the oracle', bool(lost) and 'loss oracle' in lost[0][1])

    # S2
    for arm, res in (('OFF', off), ('ON', on)):
        onset, fts, goals, yaws, states = res['S2']
        coast_ages = [t - onset for t, m in fts if m.missed_frames > 0]
        last_age = max(coast_ages) if coast_ages else 0.0
        late_goals = [m for t, m in goals if t - onset >= 0.5 and t - onset <= last_age]
        late_yaws = [(t, m) for t, m in yaws if t - onset >= 0.5 and t - onset <= last_age]
        vp = (3.0, -1.0, 1.4)
        rp = (7.5, -3.0)
        at_vp = [g for g in late_goals if math.hypot(g.pose.position.x - vp[0], g.pose.position.y - vp[1]) < 0.05
                 and abs(g.pose.position.z - vp[2]) < 0.05]
        at_rp = [(t, m) for t, m in late_yaws if math.hypot(m.point.x - rp[0], m.point.y - rp[1]) < 0.05]
        fresh = [(t, m) for t, m in late_yaws if abs(t - stamp_s(m.header.stamp)) < 0.2]
        print('S2 %s  coasted to %.2f s (%d targets); late goals %d (at viewpoint %d); late yaw targets %d '
              '(at re-emergence point %d, fresh-stamped %d)'
              % (arm, last_age, len(coast_ages), len(late_goals), len(at_vp), len(late_yaws), len(at_rp), len(fresh)))
        post = [(t, m) for t, m in yaws if t - onset > last_age + 0.45]
        if arm == 'OFF':
            check('S2 OFF: the coast ends with the forecast (< 1.6 s)', 0.8 <= last_age < 1.6, '%.2f s' % last_age)
            check('S2 OFF: goals are the standoff, never the viewpoint', len(late_goals) > 0 and not at_vp)
            if Nodes.coast_yaw_hold:
                pts = {(round(m.point.x, 3), round(m.point.y, 3)) for t, m in late_yaws + post}
                b = [math.degrees(math.atan2(m.point.y, m.point.x)) for t, m in late_yaws]
                check('S2 OFF: the coast yaw hold -- one heading toward the CTRV point (0 deg), stamped now',
                      len(late_yaws) > 0 and len(fresh) == len(late_yaws) and all(far(m) for t, m in late_yaws)
                      and max(abs(x) for x in b) < 1.0, '%d/%d fresh, %.2f deg' % (len(fresh), len(late_yaws), b[0] if b else 0))
                check('S2 OFF: the coast\'s held heading carries through the loss unchanged',
                      len(post) > 0 and len(pts) == 1, '%d distinct points, %d after the loss' % (len(pts), len(post)))
            else:
                check('S2 OFF: yaw targets keep the perception stamp', len(late_yaws) > 0 and not fresh)
        else:
            # The belief says out by 1.4 s, so the coast runs to t_max + margin = 1.9 s: past
            # the 1.3 s forecast, and short of the 3.0 s cap -- bounded by the belief, as designed.
            check('S2 ON: the coast continues past the forecast, to the belief\'s t_max + margin (1.6-2.1 s)',
                  1.6 <= last_age <= 2.1, '%.2f s' % last_age)
            check('S2 ON: every late goal is the oracle\'s viewpoint',
                  len(late_goals) > 0 and len(at_vp) == len(late_goals), '%d/%d' % (len(at_vp), len(late_goals)))
            check('S2 ON: every late yaw target is the re-emergence point, fresh-stamped',
                  len(late_yaws) > 0 and len(at_rp) == len(late_yaws) and len(fresh) == len(late_yaws),
                  '%d/%d/%d' % (len(at_rp), len(fresh), len(late_yaws)))
            check('S2 ON: lock_state names the oracle and the region',
                  any('oracle OCCLUDED' in s and 'region' in s for _, s in states))
            want = math.degrees(math.atan2(rp[1], rp[0]))
            b = [math.degrees(math.atan2(m.point.y, m.point.x)) for t, m in post]
            check('S2 ON: after the loss the yaw hold keeps the re-emergence bearing',
                  len(post) > 0 and all(far(m) for t, m in post) and max(abs(x - want) for x in b) < 1.0,
                  '%.2f deg (want %.2f), %d messages' % (b[0] if b else float('nan'), want, len(post)))

    # S3
    for arm, res in (('OFF', off), ('ON', on)):
        onset, stranger, region, via_region, states = res['S3']
        print('S3 %s  re-locked onto the in-band stranger: %s; onto the in-region (out-of-band) candidate: %s%s'
              % (arm, stranger, region, ' via the region gate' if via_region else ''))
        if arm == 'OFF':
            check('S3 OFF: the band re-locks onto the stranger (the wrong lock)', stranger)
            check('S3 OFF: the out-of-band candidate is never re-locked', not region)
        else:
            check('S3 ON: the region refuses the in-band stranger', not stranger)
            check('S3 ON: the candidate inside the region is re-locked, by the region gate', region and via_region)

    # S4
    for arm, res in (('OFF', off), ('ON', on)):
        onset, fts, yaws = res['S4']
        coasting = [(t, m) for t, m in fts if m.missed_frames > 0]
        late = [(t, m) for t, m in yaws if t - onset >= 0.3 and coasting and t <= coasting[-1][0]]
        fresh = [1 for t, m in late if abs(t - stamp_s(m.header.stamp)) < 0.2]
        print('S4 %s  %d late yaw targets, %d fresh-stamped' % (arm, len(late), len(fresh)))
        if arm == 'OFF' and Nodes.coast_yaw_hold:
            # the hold takes over at 0.4 s of STAMP age, ~0.28 s after onset plus a goal tick
            held = [(t, m) for t, m in late if t - onset >= 0.5]
            check('S4 OFF: the coast yaw hold -- a held heading, stamped now',
                  len(held) > 0 and all(far(m) and abs(t - stamp_s(m.header.stamp)) < 0.2 for t, m in held),
                  '%d held' % len(held))
        elif arm == 'OFF':
            check('S4 OFF: yaw targets keep the perception stamp', len(late) > 0 and not fresh)
        else:
            check('S4 ON: yaw targets to the CTRV point are fresh-stamped', len(late) > 0 and len(fresh) == len(late),
                  '%d/%d' % (len(fresh), len(late)))

    bad = [n for n, ok in results if not ok]
    print('\n%d/%d checks passed%s'
          % (len(results) - len(bad), len(results), '' if not bad else ' -- FAILED: ' + '; '.join(bad)))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
