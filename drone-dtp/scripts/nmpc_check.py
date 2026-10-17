#!/usr/bin/env python3
"""nmpc_check.py -- the goal_mode gate: the REAL follow_planner_node, sandboxed, in both arms.

Re-run after ANY change to follow_planner. ~6 min, needs no aircraft, no LiDAR, no FC, and cannot
touch any of them: every participant is on a private localhost-only DDS domain (90), and
follow_planner has no authority anyway -- its goal goes to command_mux, which is not started here.
The script plays target_manager (it publishes /dtp/follow_target itself, so nothing about WHAT is
locked is under test), the odometry (a simple aircraft that flies towards the last goal at
1 m/s), and where a scene needs it SUPER's GoalStatus and the loss oracle.

Every scene runs against `goal_mode:=geometric` and `goal_mode:=nmpc`, each on a fresh node, so
what is asserted is the DIFFERENCE between the arms as well as each arm's contract:

  N1  identity     goal_mode geometric reproduces the pre-nmpc binary's goals to 1e-9. The
                   reference is nmpc_check_geometric_ref.json, recorded with --record-ref from
                   the binary installed before the solver existed. Gates everything below: if the
                   geometric arm moved, no comparison further down means anything.
  N2  fail loud    an unknown goal_mode, a negative weight and nmpc_publish_stage 2 without the
                   carrot switch all refuse to start. A mode that falls back silently on a typo
                   is a failure this repo has already had three times.
  N3  band         steady walk: goals sit in the standoff band the planner promises.
  N4  floor        a person backing TOWARDS the goal: no goal closer than min_standoff_m to the
                   person the node had last been told about (the deadband may hold a goal, so the
                   bound is min_standoff_m - goal_deadband_m, and nmpc may be no worse than
                   geometric). A person walking AT the aircraft is withheld by the transit guard
                   in both arms alike.
  N5  z            a target z at the ground and near the ceiling: every goal z in
                   [goal_z_min_m, goal_z_max_m]; a NaN target publishes nothing.
  N6  smoothness   walk, pause, 90 deg turn: nmpc is NO WORSE than geometric on the per-tick goal
                   step (p95), on goal-height jumps and on direction reversals. The pause is the
                   new failure mode: a cost with two near-equal minima can chatter, and the
                   closed-form goal structurally cannot.
  N7  budget       the node's own "NMPC solve" line: p99 <= 5 ms, max <= 8 ms, and the goal
                   stream stays at >= 14 Hz. An idle-box number: necessary, not sufficient.
  N8  fallback     every solve refused (nmpc_sanity_jump_m 0): the stream continues, stays on
                   contract, and the node latches DEGRADED. Every solve starved
                   (nmpc_time_budget_ms 0): a run of cancelled solves is refused the same way.
  N9  silence      /dtp/follow_target stops: both arms go silent within the same timeout. The
                   solver must not extend the node's willingness to publish.
  N10 escalation   five DROPPED statuses: the goal is re-projected and the node suspends, in
                   both arms.
  N11 conditioning loss_conditioning + an OCCLUDED event with a viewpoint: the goal IS the
                   viewpoint in both arms (the oracle's branch bypasses ComputeStandoffGoal).
  N12 visibility   Stage 2, the script playing fsm_node's dtp_msgs/VisibilityField. With the trail
                   bearing occluded and nmpc_w_vis on, the goal walks out of the shadow and stays
                   in the band; when the field stops, the weight RAMPS off and the goal walks back
                   with no step larger than before; a field that knows no height of the person,
                   and no field at all, are both exactly Stage 1. Needs a dtp_msgs with
                   VisibilityField (SUPER patch 0008's message): without it N12 FAILS and says so.

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/nmpc_check.py
    python3 drone-dtp/scripts/nmpc_check.py --node-exe PATH     # test a scratch build
    python3 drone-dtp/scripts/nmpc_check.py --record-ref        # re-record N1's reference
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '90'
DETECT_HZ = 8.0
ODOM_HZ = 10.0
REF = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nmpc_check_geometric_ref.json')

# follow_planner's contract, as deployed (dtp_params.yaml) and as analyze_shadow_goals.py reads it.
FOLLOW_NEAR_M = 3.0
FOLLOW_FAR_M = 4.0
LEAD_FLOOR_M = 2.75      # min_standoff_m + the floor reserve: the lead may spend down to here
MIN_STANDOFF_M = 2.5
DEADBAND_M = 0.15
Z_MIN, Z_MAX = 1.2, 3.0


def _env():
    return dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')


def default_exe():
    out = subprocess.run(['ros2', 'pkg', 'prefix', 'follow_planner'], capture_output=True, text=True)
    return os.path.join(out.stdout.strip(), 'lib', 'follow_planner', 'follow_planner_node')


class PlannerNode:
    """The real node, run by path so a scratch build or an old binary can be put under test."""

    def __init__(self, exe, logpath, **params):
        params.setdefault('goal_z_min_m', Z_MIN)          # the deployed follow height
        params.setdefault('goal_status_timeout_s', 100.0)  # no SUPER here to answer
        args = [exe, '--ros-args']
        for k, v in params.items():
            args += ['-p', '%s:=%s' % (k, str(v).lower() if isinstance(v, bool) else v)]
        self.logpath = logpath
        self.log = open(logpath, 'w')
        self.proc = subprocess.Popen(args, env=_env(), stdout=self.log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)

    def stop(self):
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.log.close()
        return open(self.logpath).read()


class Harness:
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped, PointStamped
        from dtp_msgs.msg import Target, GoalStatus, LossEvent
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('nmpc_check')
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                        depth=1, durability=DurabilityPolicy.VOLATILE)
        self.Target, self.GoalStatus, self.LossEvent, self.Odometry = Target, GoalStatus, LossEvent, Odometry
        self.odom_pub = self.node.create_publisher(Odometry, '/lidar_slam/odom', be)
        self.target_pub = self.node.create_publisher(Target, '/dtp/follow_target', be)
        self.status_pub = self.node.create_publisher(GoalStatus, '/planning/goal_status', be)
        self.event_pub = self.node.create_publisher(LossEvent, '/planning/loss_event', be)
        self.node.create_subscription(PoseStamped, '/dtp/follow_goal', self._on_goal, be)
        self.node.create_subscription(PointStamped, '/planning/yaw_target', lambda m: None, be)
        self.goals = []          # (t, (x, y, z), person the node had last been told about)
        self.on_goal = None      # callable(goal xyz), for scenes that answer as SUPER
        self.last_person = None
        self.drone = [0.0, 0.0, Z_MIN]
        self.drone_follows = True
        try:   # Stage 2's message: only in a dtp_msgs built with msg/VisibilityField.msg
            from dtp_msgs.msg import VisibilityField
            self.VisibilityField = VisibilityField
            self.vis_pub = self.node.create_publisher(VisibilityField, '/planning/visibility', be)
        except ImportError:
            self.VisibilityField = self.vis_pub = None

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()

    def now(self):
        return time.time()

    def _on_goal(self, m):
        g = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        self.goals.append((self.now(), g, self.last_person))
        if self.on_goal is not None:
            self.on_goal(g)

    def wait_for_node(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.node.count_subscribers('/dtp/follow_target') >= 1:
                return True
        return False

    def target(self, track, p, v, missed=0):
        from geometry_msgs.msg import Point
        t = self.Target()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'lidar_init'
        t.track_id = track
        t.position.x, t.position.y, t.position.z = p
        t.velocity.x, t.velocity.y = v[0], v[1]
        t.track_confirmed = True
        t.is_locked_target = True
        t.missed_frames = missed
        t.confidence = 0.9
        t.distance_m = math.hypot(p[0] - self.drone[0], p[1] - self.drone[1])
        for i in range(20):  # perception's forecast: 20 points, 0.075 s apart, 1.5 s, ground plane
            dt = 0.2 + 0.075 * i
            t.predicted_path.append(Point(x=p[0] + v[0] * dt, y=p[1] + v[1] * dt, z=p[2]))
            t.predicted_path_dt.append(dt)
        return t

    def _publish_odom(self, look_at):
        o = self.Odometry()
        o.header.stamp = self.node.get_clock().now().to_msg()
        o.header.frame_id = 'lidar_init'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.drone
        yaw = math.atan2(look_at[1] - self.drone[1], look_at[0] - self.drone[0])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.odom_pub.publish(o)

    def run(self, secs, track, person, publish_target=True, missed=0):
        """person(t) -> ((x, y, z), (vx, vy)). The aircraft flies at the newest goal at <= 1 m/s."""
        t_start = self.now()
        next_target = next_odom = t_start
        last = t_start
        look = person(0.0)[0]
        while True:
            now = self.now()
            if now - t_start >= secs:
                break
            if now >= next_odom:
                step = now - last
                last = now
                if self.drone_follows and self.goals:
                    # Speed proportional to the distance left, capped at 1 m/s: the aircraft
                    # trails a moving goal by ~v/1 m, as SUPER's rest-to-rest plans do (v^2/2a =
                    # 0.64 m at 0.8 m/s). An aircraft that sat ON the goal would hold the node
                    # inside its 0.25 m park guard for the whole scene, which no flight does.
                    g = self.goals[-1][1]
                    d = [g[i] - self.drone[i] for i in range(3)]
                    n = math.sqrt(sum(c * c for c in d))
                    if n > 1e-6:
                        k = min(1.0, n) * step / n
                        self.drone = [self.drone[i] + k * d[i] for i in range(3)]
                self._publish_odom(look)
                next_odom += 1.0 / ODOM_HZ
            if publish_target and now >= next_target:
                p, v = person(now - t_start)
                look = p
                if all(math.isfinite(c) for c in p):
                    self.last_person = p
                self.target_pub.publish(self.target(track, p, v, missed))
                next_target += 1.0 / DETECT_HZ
            self.rclpy.spin_once(self.node, timeout_sec=0.005)

    def reset(self, drone=(0.0, 0.0, Z_MIN)):
        self.goals = []
        self.drone = list(drone)
        self.last_person = None
        self.on_goal = None
        self.drone_follows = True


# ---- scenes: person(t) ------------------------------------------------------------------------
def walk(speed=0.8, z=0.1):
    return lambda t: ((4.0 + speed * t, 0.0, z), (speed, 0.0))


def walk_pause_walk(speed=0.8):
    def f(t):
        if t < 6.0:
            return (4.0 + speed * t, 0.0, 0.1), (speed, 0.0)
        if t < 10.0:
            return (4.0 + speed * 6.0, 0.0, 0.1), (0.0, 0.0)
        return (4.0 + speed * (t - 4.0), 0.0, 0.1), (speed, 0.0)
    return f


def turn(speed=0.8):
    def f(t):
        if t < 5.0:
            return (4.0 + speed * t, 0.0, 0.1), (speed, 0.0)
        a = min(t - 5.0, 2.0) / 2.0 * (math.pi / 2.0)          # 90 deg over 2 s
        r = speed * 2.0 / (math.pi / 2.0)
        x0 = 4.0 + speed * 5.0
        if t < 7.0:
            return (x0 + r * math.sin(a), r * (1.0 - math.cos(a)), 0.1), (speed * math.cos(a), speed * math.sin(a))
        return (x0 + r, r + speed * (t - 7.0), 0.1), (0.0, speed)
    return f


def at_drone(speed=1.0):
    return lambda t: ((7.0 - speed * t, 0.3, 0.1), (-speed, 0.0))


def back_up():
    """Walks away, then backs TOWARDS the trailing goal at 0.24 m/s -- under the 0.25 m/s heading
    floor, so the trail azimuth holds and the goal is not flipped to the far side (which the
    transit guard would then withhold). This is the scene where the standoff floor has work."""
    def f(t):
        if t < 4.0:
            return (4.0 + 0.8 * t, 0.0, 0.1), (0.8, 0.0)
        return (7.2 - 0.24 * (t - 4.0), 0.0, 0.1), (-0.24, 0.0)
    return f


# ---- metrics ------------------------------------------------------------------------------------
def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float('nan')


def metrics(goals, settle_s=3.0):
    if not goals:
        return None
    t0 = goals[0][0]
    g = [x for x in goals if x[0] - t0 >= settle_s]
    steps, zj, rev, prev_d = [], 0, 0, None
    for a, b in zip(g, g[1:]):
        d = [b[1][i] - a[1][i] for i in range(3)]
        n = math.sqrt(sum(c * c for c in d))
        steps.append(n)
        if abs(d[2]) >= 0.10:
            zj += 1
        if n > 1e-6:
            if prev_d is not None:
                c = sum(x * y for x, y in zip(d, prev_d[0])) / (n * prev_d[1])
                if c < -0.5:        # turned back on itself by more than 120 deg
                    rev += 1
            prev_d = (d, n)
    ranges = [math.hypot(x[1][0] - x[2][0], x[1][1] - x[2][1]) for x in g if x[2] is not None]
    span = g[-1][0] - g[0][0] if len(g) > 1 else 0.0
    return {'n': len(g), 'rate': (len(g) - 1) / span if span > 0 else 0.0,
            'step_p95': pct(steps, 0.95), 'step_max': max(steps) if steps else 0.0,
            'z_jumps': zj, 'reversals': rev,
            'in_band': sum(LEAD_FLOOR_M - 1e-6 <= r <= FOLLOW_FAR_M + 1e-6 for r in ranges) / max(1, len(ranges)),
            'range_min': min(ranges) if ranges else float('nan'),
            'range_max': max(ranges) if ranges else float('nan'),
            'z_min': min(x[1][2] for x in g), 'z_max': max(x[1][2] for x in g)}


# ---- N1: the identity poses ----------------------------------------------------------------------
IDENTITY_POSES = [  # (person xyz, velocity xy, aircraft xyz)
    ((4.0, 0.0, 0.1), (0.8, 0.0), (0.0, 0.0, 1.2)),
    ((4.0, 0.0, 0.1), (0.0, 0.0), (0.0, 0.0, 1.2)),
    ((5.0, 2.0, 0.9), (0.0, 1.2), (1.0, -1.0, 1.5)),
    ((3.0, -1.0, 0.1), (-1.0, 0.3), (0.0, 0.0, 1.2)),
    ((6.0, 0.5, 2.9), (0.5, 0.5), (2.0, 2.0, 2.0)),
    ((2.0, 0.0, 0.1), (-1.4, 0.0), (0.0, 0.0, 1.2)),      # inside the floor: both floors act
    ((8.0, -3.0, 0.03), (0.3, -0.9), (5.0, 1.0, 1.2)),
    ((4.5, 4.5, 1.0), (1.0, -1.0), (0.0, 0.0, 0.8)),
]


def identity_goals(h, exe, logdir, tag):
    node = PlannerNode(exe, os.path.join(logdir, 'identity_%s.log' % tag), goal_mode='geometric')
    out = []
    try:
        if not h.wait_for_node():
            raise RuntimeError('node did not come up; see %s' % logdir)
        for i, (p, v, d) in enumerate(IDENTITY_POSES):
            h.reset(d)
            h.drone_follows = False
            h.run(1.0, 100 + i, lambda t, p=p, v=v: (p, v))   # a NEW track: no deadband history
            # The aircraft and the message are both held still, so the goal is a pure function of
            # them -- once the node has this pose's odometry. Take the last one.
            out.append(list(h.goals[-1][1]) if h.goals else None)
            h.run(0.6, 100 + i, lambda t, p=p, v=v: (p, v), publish_target=False)  # go silent
    finally:
        node.stop()
    return out


# ---- the scenes every arm runs --------------------------------------------------------------------
def run_arm(h, exe, mode, logdir):
    node = PlannerNode(exe, os.path.join(logdir, 'arm_%s.log' % mode), goal_mode=mode)
    res = {}
    try:
        if not h.wait_for_node():
            raise RuntimeError('node did not come up (%s); see %s' % (mode, logdir))
        track = 1
        for name, scene, secs in (('walk', walk(), 14.0), ('pause', walk_pause_walk(), 16.0),
                                  ('turn', turn(), 14.0)):
            h.reset()
            h.run(secs, track, scene)
            res[name] = metrics(h.goals)
            h.run(0.6, track, scene, publish_target=False)
            track += 1

        h.reset()
        h.run(10.0, track, back_up())
        res['back_up'] = metrics(h.goals, settle_s=4.0)
        h.run(0.6, track, back_up(), publish_target=False)
        track += 1

        h.reset()
        h.drone_follows = False
        h.run(3.5, track, at_drone())          # 7 m -> 3.5 m away, closing at 1 m/s
        res['at_drone_goals'] = len(h.goals)
        h.run(0.6, track, at_drone(), publish_target=False)
        track += 1

        h.reset()
        h.run(3.0, track, walk(z=0.03))
        h.run(0.6, track, walk(), publish_target=False)
        h.run(3.0, track + 1, walk(z=2.9))
        res['z'] = metrics(h.goals, settle_s=0.0)
        h.run(0.6, track, walk(), publish_target=False)
        track += 2

        h.reset()
        h.run(2.0, track, lambda t: ((float('nan'), 0.0, 0.1), (0.8, 0.0)))
        res['nan_goals'] = len(h.goals)
        track += 1

        h.reset()
        h.run(4.0, track, walk())
        t_stop = h.now()
        h.run(1.5, track, walk(), publish_target=False)
        res['silence_after_s'] = (h.goals[-1][0] - t_stop) if h.goals else float('nan')
        res['goals_before_silence'] = len(h.goals)
    finally:
        res['log'] = node.stop()
    return res


def run_escalation(h, exe, mode, logdir):
    """SUPER drops every goal it is sent. The node must re-project, then suspend (go silent)."""
    node = PlannerNode(exe, os.path.join(logdir, 'escalation_%s.log' % mode), goal_mode=mode,
                       goal_status_timeout_s=0.2)
    try:
        if not h.wait_for_node():
            raise RuntimeError('node did not come up')
        h.reset()

        def drop(g):
            from geometry_msgs.msg import Point
            s = h.GoalStatus()
            s.header.stamp = h.node.get_clock().now().to_msg()
            s.status = h.GoalStatus.STATUS_DROPPED
            s.requested_p = Point(x=g[0], y=g[1], z=g[2])
            s.goal_p = Point(x=g[0], y=g[1], z=g[2])
            h.status_pub.publish(s)
        h.on_goal = drop
        h.drone_follows = False
        h.run(5.0, 50, lambda t: ((4.0, 0.0, 0.1), (0.0, 0.0)))
        distinct = len({tuple(round(c, 3) for c in g[1]) for g in h.goals})
        last = (h.now() - h.goals[-1][0]) if h.goals else float('nan')
    finally:
        log = node.stop()
    return distinct, last, log


def run_conditioning(h, exe, mode, logdir):
    node = PlannerNode(exe, os.path.join(logdir, 'conditioning_%s.log' % mode), goal_mode=mode,
                       loss_conditioning=True)
    view = (1.0, 2.5, 1.5)
    try:
        if not h.wait_for_node():
            raise RuntimeError('node did not come up')
        h.reset()
        h.drone_follows = False
        h.run(2.0, 60, walk())
        h.goals = []
        t_start = h.now()
        from geometry_msgs.msg import Point
        while h.now() - t_start < 2.0:
            p = (5.6, 0.0, 0.1)
            ft = h.target(60, p, (0.8, 0.0), missed=3)
            h.last_person = p
            h.target_pub.publish(ft)
            ev = h.LossEvent()
            ev.header = ft.header
            ev.eval_stamp = ft.header.stamp
            ev.track_id, ev.missed_frames, ev.cls = 60, 3, h.LossEvent.CLS_OCCLUDED
            ev.query_p = ft.position
            ev.yaw_target_p = Point(x=6.5, y=1.0, z=0.1)
            ev.viewpoint_valid = True
            ev.viewpoint_p = Point(x=view[0], y=view[1], z=view[2])
            h.event_pub.publish(ev)
            h._publish_odom(p)
            end = h.now() + 1.0 / DETECT_HZ
            while h.now() < end:
                h.rclpy.spin_once(h.node, timeout_sec=0.005)
        late = [g[1] for g in h.goals if g[0] - t_start > 0.5]
        worst = max((math.dist(g, view) for g in late), default=float('nan'))
    finally:
        log = node.stop()
    return len(late), worst, log


def vis_field(h, track, centre, occluded_at, half_width, heights_used=2):
    """A VisibilityField as fsm_node would publish it: known-occluded within half_width of
    `occluded_at` (an azimuth FROM the person), seen clearly everywhere else, annulus clear."""
    from geometry_msgs.msg import Point
    f = h.VisibilityField()
    f.header.stamp = h.node.get_clock().now().to_msg()
    f.header.frame_id = 'lidar_init'
    f.eval_stamp = f.header.stamp
    f.track_id = track
    f.centre_p = Point(x=centre[0], y=centre[1], z=centre[2])
    f.view_z, f.r_near, f.r_far = 1.2, 3.0, 4.0
    f.bins, f.az0, f.daz = 24, -math.pi, 2.0 * math.pi / 24
    for i in range(24):
        off = abs(math.atan2(math.sin(f.az0 + i * f.daz - occluded_at), math.cos(f.az0 + i * f.daz - occluded_at)))
        f.clear_r.append(4.0)
        f.vis.append(0.0 if off < half_width else 1.0)
        f.unk.append(0.0)
    f.heights_used, f.heights_dropped = heights_used, 3 - heights_used
    return f


def run_visibility(h, exe, logdir, tag, publish, secs=9.0, **params):
    """A person who walked +x and stopped at (5.6, 0); the aircraft parked at the origin, so the
    trail bearing is exactly pi. `publish(t)` says whether, and what, fsm_node publishes at t.
    Returns [(t, goal azimuth off the trail bearing in degrees, range, step)] and the node's log."""
    node = PlannerNode(exe, os.path.join(logdir, 'visibility_%s.log' % tag), goal_mode='nmpc', **params)
    rows = []
    try:
        if not h.wait_for_node():
            raise RuntimeError('node did not come up')
        h.reset()
        h.drone_follows = False
        h.run(2.0, 80, lambda t: ((4.0 + 0.8 * t, 0.0, 0.1), (0.8, 0.0)))   # establishes the heading
        person = (5.6, 0.0, 0.1)
        h.goals = []
        t0 = h.now()
        next_f = t0
        while h.now() - t0 < secs:
            t = h.now() - t0
            if h.now() >= next_f:
                f = publish(t)
                if f is not None:
                    h.vis_pub.publish(f(h, 80, person))
                next_f += 1.0 / DETECT_HZ
            h.run(1.0 / DETECT_HZ, 80, lambda _t: (person, (0.0, 0.0)))
        prev = None
        for tg, g, _p in h.goals:
            az = math.atan2(g[1] - person[1], g[0] - person[0])
            off = abs(math.atan2(math.sin(az - math.pi), math.cos(az - math.pi))) * 180.0 / math.pi
            rows.append((tg - t0, off, math.hypot(g[0] - person[0], g[1] - person[1]),
                         0.0 if prev is None else math.dist(g, prev)))
            prev = g
    finally:
        log = node.stop()
    return rows, log


def refuses_to_start(exe, logdir, tag, **params):
    node = PlannerNode(exe, os.path.join(logdir, 'refuse_%s.log' % tag), **params)
    try:
        code = node.proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        code = None
    log = node.stop()
    return code not in (None, 0), log


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--node-exe', default=None, help='follow_planner_node under test (default: the sourced install)')
    ap.add_argument('--record-ref', action='store_true', help="record N1's reference goals from --node-exe and exit")
    ap.add_argument('--logs', default=None)
    ap.add_argument('--only-n12', action='store_true', help='run the Stage 2 visibility scenes alone (~1.5 min)')
    a = ap.parse_args(argv)
    exe = a.node_exe or default_exe()
    if not os.path.isfile(exe):
        print('no follow_planner_node at %r -- source the workspace or pass --node-exe' % exe)
        return 2
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = a.logs or tempfile.mkdtemp(prefix='nmpc_check_')
    os.makedirs(logdir, exist_ok=True)
    print('\nnmpc_check -- real follow_planner_node (%s) on ROS_DOMAIN_ID %s (localhost only); logs in %s\n'
          % (exe, DOMAIN, logdir))

    results = []

    def check(name, ok, detail=''):
        results.append(bool(ok))
        print('  %s  %-66s %s' % ('PASS' if ok else 'FAIL', name, detail))

    h = Harness()
    try:
        if a.record_ref:
            goals = identity_goals(h, exe, logdir, 'ref')
            with open(REF, 'w') as f:
                json.dump({'exe': exe, 'recorded': time.strftime('%F %T'), 'goals': goals}, f, indent=1)
            print('recorded %d reference goals to %s' % (len(goals), REF))
            return 0

        if not a.only_n12:
            # ---- N1 ---------------------------------------------------------------------------------
            if not os.path.isfile(REF):
                check('N1 identity: a recorded geometric reference exists', False, 'run --record-ref on the OLD binary')
                ref = None
            else:
                ref = json.load(open(REF))['goals']
            got = identity_goals(h, exe, logdir, 'new')
            if ref is not None:
                # None is a pose the node WITHHOLDS a goal for (the transit keep-out, with the person
                # walking at the aircraft). Withholding is behaviour too: it must match.
                worst = max((0.0 if g is None and r is None else
                             float('inf') if g is None or r is None else
                             max(abs(x - y) for x, y in zip(g, r))) for g, r in zip(got, ref))
                check('N1 identity: goal_mode geometric reproduces the pre-nmpc binary', worst <= 1e-9,
                      'worst |delta| %.2e m over %d poses (%d withheld in both)'
                      % (worst, len(ref), sum(r is None for r in ref)))

            # ---- N2 ---------------------------------------------------------------------------------
            for tag, params, needle in (
                    ('banana', {'goal_mode': 'banana'}, "goal_mode must be"),
                    ('weight', {'goal_mode': 'nmpc', 'nmpc_w_standoff': -1.0}, 'nmpc_w_standoff'),
                    ('carrot', {'goal_mode': 'nmpc', 'nmpc_publish_stage': 2}, 'nmpc_publish_stage')):
                refused, log = refuses_to_start(exe, logdir, tag, **params)
                check('N2 fail loud: %s refuses to start and says why' % tag, refused and needle in log)

            # ---- the two arms -----------------------------------------------------------------------
            arms = {m: run_arm(h, exe, m, logdir) for m in ('geometric', 'nmpc')}
            geo, nm = arms['geometric'], arms['nmpc']
            for m, r in arms.items():
                print('%-9s walk  %s' % (m, {k: round(v, 3) for k, v in r['walk'].items()}))
                print('%-9s pause %s' % (m, {k: round(v, 3) for k, v in r['pause'].items()}))
                print('%-9s turn  %s' % (m, {k: round(v, 3) for k, v in r['turn'].items()}))

            for m, r in arms.items():
                check('N3 band (%s): >= 99 %% of steady-walk goals in [%.2f, %.2f] m' % (m, LEAD_FLOOR_M, FOLLOW_FAR_M),
                      r['walk']['in_band'] >= 0.99, '%.1f %%, range %.2f-%.2f m'
                      % (100 * r['walk']['in_band'], r['walk']['range_min'], r['walk']['range_max']))
            for m, r in arms.items():
                check('N4 floor (%s): person backing towards the goal, no goal under %.2f m'
                      % (m, MIN_STANDOFF_M - DEADBAND_M), r['back_up']['range_min'] >= MIN_STANDOFF_M - DEADBAND_M - 1e-6,
                      'min %.3f m over %d goals' % (r['back_up']['range_min'], r['back_up']['n']))
            # NOT "no closer than geometric": the solver's standoff term is a dead zone, so it rightly
            # lets a person close the range to the band's near edge before it moves, where the
            # closed-form goal re-centres on every frame. The contract is the commanded band.
            check('N4 floor: nmpc stays in the commanded band while the person backs towards it',
                  nm['back_up']['range_min'] >= FOLLOW_NEAR_M - 0.05,
                  'nmpc %.3f m (geometric %.3f m), band from %.1f m'
                  % (nm['back_up']['range_min'], geo['back_up']['range_min'], FOLLOW_NEAR_M))
            check('N4 transit: a person walking AT the aircraft is withheld in both arms alike',
                  abs(nm['at_drone_goals'] - geo['at_drone_goals']) <= 3,
                  'goals published: nmpc %d, geometric %d' % (nm['at_drone_goals'], geo['at_drone_goals']))
            for m, r in arms.items():
                check('N5 z (%s): every goal z in [%.1f, %.1f], a NaN target publishes nothing' % (m, Z_MIN, Z_MAX),
                      r['z']['z_min'] >= Z_MIN - 1e-9 and r['z']['z_max'] <= Z_MAX + 1e-9 and r['nan_goals'] == 0,
                      'z %.3f-%.3f, %d goals on NaN' % (r['z']['z_min'], r['z']['z_max'], r['nan_goals']))
            for scene in ('walk', 'pause', 'turn'):
                g, n = geo[scene], nm[scene]
                check('N6 smoothness (%s): nmpc step p95 no worse than geometric' % scene,
                      n['step_p95'] <= g['step_p95'] + 0.01, 'nmpc %.3f vs %.3f m' % (n['step_p95'], g['step_p95']))
                check('N6 smoothness (%s): nmpc largest step no worse than geometric' % scene,
                      n['step_max'] <= g['step_max'] + 0.01, 'nmpc %.3f vs %.3f m' % (n['step_max'], g['step_max']))
                check('N6 smoothness (%s): no more height jumps or reversals than geometric' % scene,
                      n['z_jumps'] <= g['z_jumps'] and n['reversals'] <= g['reversals'],
                      'nmpc %d/%d vs %d/%d' % (n['z_jumps'], n['reversals'], g['z_jumps'], g['reversals']))
                check('N6 band (%s): nmpc keeps >= 95 %% of goals in the band' % scene, n['in_band'] >= 0.95,
                      '%.1f %% (geometric %.1f %%)' % (100 * n['in_band'], 100 * g['in_band']))

            lines = re.findall(r'NMPC solve p50 ([\d.]+) ms p99 ([\d.]+) ms max ([\d.]+) ms over (\d+)', nm['log'])
            if lines:
                p50, p99, mx, cnt = (float(x) for x in lines[-1])
                check('N7 budget: solve p99 <= 5 ms and max <= 8 ms', p99 <= 5.0 and mx <= 8.0,
                      'p50 %.3f p99 %.3f max %.3f ms over the last %d solves' % (p50, p99, mx, cnt))
            else:
                check('N7 budget: the node logged its solve times', False)
            check('N7 budget: the goal stream holds >= 14 Hz in both arms',
                  nm['walk']['rate'] >= 14.0 and geo['walk']['rate'] >= 14.0,
                  'nmpc %.2f Hz, geometric %.2f Hz' % (nm['walk']['rate'], geo['walk']['rate']))
            check('N7: the solver answered (no fallbacks on clean scenes)',
                  'DEGRADED' not in nm['log'] and not re.search(r'fallback [1-9]', nm['log'].split('NMPC solve')[-1]),
                  (re.findall(r'ok \d+ fallback \d+ cancel \d+ reset \d+', nm['log']) or ['?'])[-1])

            for m, r in arms.items():
                check('N9 silence (%s): goals stop within 0.45 s of the target stream' % m,
                      r['goals_before_silence'] > 20 and r['silence_after_s'] <= 0.45,
                      'last goal %.2f s after the last target' % r['silence_after_s'])

            # ---- N8 ---------------------------------------------------------------------------------
            for tag, params, needle in (('refused', {'nmpc_sanity_jump_m': 0.0, 'nmpc_fallback_persist_n': 15}, 'DEGRADED'),
                                        ('starved', {'nmpc_time_budget_ms': 0.0}, 'DEGRADED')):
                node = PlannerNode(exe, os.path.join(logdir, 'fallback_%s.log' % tag), goal_mode='nmpc', **params)
                try:
                    ok_up = h.wait_for_node()
                    h.reset()
                    h.run(8.0, 70, walk())
                    m = metrics(h.goals)
                finally:
                    log = node.stop()
                cancels = re.findall(r'cancel (\d+)', log)
                said = (needle in log) if needle else (bool(cancels) and int(cancels[-1]) > 0)
                check('N8 fallback (%s): the stream continues on contract, and the node says so' % tag,
                      ok_up and m is not None and m['rate'] >= 14.0 and m['in_band'] >= 0.99 and said,
                      '%.2f Hz, %.1f %% in band' % (m['rate'], 100 * m['in_band']) if m else 'no goals')

            # ---- N10, N11 ---------------------------------------------------------------------------
            for m in ('geometric', 'nmpc'):
                distinct, last, log = run_escalation(h, exe, m, logdir)
                check('N10 escalation (%s): DROPPED re-projects the goal, then the node suspends' % m,
                      distinct >= 2 and last >= 1.0 and 'uspend' in log,
                      '%d distinct goals, silent for the last %.1f s' % (distinct, last))
            for m in ('geometric', 'nmpc'):
                n, worst, log = run_conditioning(h, exe, m, logdir)
                check("N11 conditioning (%s): the goal is the oracle's viewpoint" % m, n >= 10 and worst <= 1e-6,
                      '%d goals, worst %.2e m from the viewpoint' % (n, worst))
        # ---- N12 --------------------------------------------------------------------------------
        if h.VisibilityField is None:
            check('N12 visibility: dtp_msgs has VisibilityField', False,
                  'rebuild dtp_msgs (msg/VisibilityField.msg, SUPER patch 0008) and source it')
        else:
            shadow = lambda hh, tr, c: vis_field(hh, tr, c, math.pi, math.pi / 6.0)   # noqa: E731
            blind = lambda hh, tr, c: vis_field(hh, tr, c, math.pi, math.pi / 6.0, heights_used=0)  # noqa: E731
            on = {'nmpc_w_vis': 10.0, 'nmpc_w_clear': 4.0}
            base, _ = run_visibility(h, exe, logdir, 'stage1', lambda t: None)
            absent, log_absent = run_visibility(h, exe, logdir, 'absent', lambda t: None, **on)
            unknow, log_blind = run_visibility(h, exe, logdir, 'blind', lambda t: blind, **on)
            lit, log = run_visibility(h, exe, logdir, 'field', lambda t: shadow if t < 5.0 else None, secs=14.0, **on)

            def settle(rows, a, b):
                v = [r for r in rows if a <= r[0] <= b]
                return (sum(r[1] for r in v) / len(v), sum(r[2] for r in v) / len(v)) if v else (float('nan'),) * 2
            b_off, b_rng = settle(base, 3.0, 9.0)
            check('N12 visibility: Stage 1 parks the goal on the trail bearing', b_off <= 3.0, '%.1f deg off' % b_off)
            # "Is Stage 1" is asserted on what the node SAYS it used (the weight scale in its own
            # log line) and on the bearing. Not on the range to 5 cm: a parked goal rests anywhere
            # in the solver's dead zone, and where depends on which tick the person stopped on
            # (two Stage 1 runs differ by ~0.07 m).
            for name, rows, lg in (('no field at all', absent, log_absent),
                                   ('a field that knows no height', unknow, log_blind)):
                o, r = settle(rows, 3.0, 9.0)
                used = [float(x) for x in re.findall(r'\| vis ([\d.]+)', lg)]
                check('N12 visibility: %s is Stage 1 (weight 0 throughout, same bearing)' % name,
                      bool(used) and max(used) == 0.0 and abs(o - b_off) <= 1.0 and abs(r - b_rng) <= 0.25,
                      'vis scale max %.2f, %.1f deg off, %.2f m (Stage 1: %.1f, %.2f)'
                      % (max(used) if used else float('nan'), o, r, b_off, b_rng))
            o_on, r_on = settle(lit, 3.5, 5.0)
            o_after, _ = settle(lit, 12.5, 14.0)
            check('N12 visibility: the goal walks out of a 30-deg-wide shadow, and stays in the band',
                  o_on >= 30.0 and FOLLOW_NEAR_M - 0.05 <= r_on <= FOLLOW_FAR_M + 0.05,
                  '%.1f deg off the trail bearing at %.2f m' % (o_on, r_on))
            # It walks back slowly (~5 deg/s: the friction term, and nothing is urgent about
            # returning to a bearing), so this looks 8 s on and asks for direction, not speed.
            check('N12 visibility: the field stops -> the weight ramps off and the goal walks back',
                  o_after <= o_on - 15.0 and re.search(r'vis 0\.00', log.split('NMPC solve')[-1]) is not None,
                  '%.1f deg off 8 s after the last field (was %.1f)' % (o_after, o_on))
            step_fade = max((r[3] for r in lit if 5.0 <= r[0] <= 7.0), default=0.0)
            step_before = max((r[3] for r in lit if 1.0 <= r[0] <= 5.0), default=0.0)
            check('N12 visibility: losing the field causes no step larger than having it did',
                  step_fade <= max(step_before, DEADBAND_M) + 0.05,
                  'largest step %.3f m while fading, %.3f m before' % (step_fade, step_before))
    finally:
        h.close()

    print('\n%d/%d checks passed' % (sum(results), len(results)))
    return 0 if results and all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
