#!/usr/bin/env python3
"""Bench gate for patch 0006 (yaw hold on a short trajectory), against the REAL fsm_node. No hardware.

WHAT IT PROVES
--------------
On 2026-09-14 the first Offboard hold flew a pirouette: with the follower LOST, command_mux re-sent
its hold goal at ~2 Hz, SUPER re-planned from wherever the wobbling aircraft was, and YawTrajOpt's
free-end yaw aligned the heading to the direction of that few-centimetre correction -- a different
direction every replan. The commanded yaw rate ran 52 deg/s median, 129 deg/s peak, for the whole
44 s (bag 20260914_2020_stageB). Patch 0006 makes a free-end yaw HOLD the current heading when the
whole trajectory moves less than 0.3 m.

This gate reproduces the loop against the real planner on a private localhost-only DDS domain
(the target_filter_check harness): fsm_node on follow_test_ros2.yaml (yaw_mode 3, no yaw target,
so the free-end fallback is in force), a synthetic MID360 scene so the map is known, and an
odometry that FOLLOWS the planner's own /planning/pos_cmd (position and yaw) plus a slowly
rotating hover offset of WOBBLE_M -- the aircraft never quite sits on the goal, as in flight.

  H1  HOLD    a hold goal at the aircraft's own position, re-sent at 2 Hz for HOLD_S seconds.
              PASS: the commanded yaw never leaves its starting value by more than YAW_HOLD_DEG,
              and the commanded yaw rate stays under RATE_HOLD_DPS. (The pre-0006 binary spins.)
  H2  MOVE    a goal 3 m away at +90 deg. PASS: the commanded yaw turns toward +90 deg (within
              YAW_MOVE_DEG at the end) -- the fix must not stop the nose following a real move.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/yaw_hold_check.py                   # ~2 min, exits non-zero on any FAIL
    python3 drone-dtp/scripts/yaw_hold_check.py --binary ~/backups/backup-pre-yawhold-*/fsm_node --expect-spin
                                                                  # the OLD binary: H1 must FAIL (proves the gate sees the bug)
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import target_filter_check as tfc  # noqa: E402

WOBBLE_M = 0.20        # hover offset radius: below 0006's 0.3 m floor, above SUPER's 0.1 m goal tolerance
WOBBLE_PERIOD_S = 20.0 # the offset rotates once per 20 s, like a slow hover circle
HOLD_S = 25.0
MOVE_S = 20.0
YAW_HOLD_DEG = 15.0
RATE_HOLD_DPS = 15.0
YAW_MOVE_DEG = 25.0
HOVER_Z = 0.8          # hover height above the map origin, as flown (0.75 m); the floor is at -0.45 and
                       # ROG-Map inflates it 0.4 m, so a hover at 0 leaves the corridor 5 cm thick


BOX_HALF_M = 8.0       # walls of the synthetic room, +-8 m about the start point
FLOOR_Z = tfc.FLOOR    # -0.45: just above the effective virtual ground (-0.5)
CEIL_Z = 2.8


def scan360(k, origin):
    """One MID360-like scan over the FULL azimuth (the real sensor sees 360 deg; the target_filter
    scene's +-60 deg cone leaves the cells beside and behind the sensor UNKNOWN, and SUPER's
    corridor then fails on the aircraft's own position). Rays hit a floor plane, a ceiling and
    the four walls of a box, so everything around the start point is mapped FREE."""
    rng = np.random.default_rng(5000 + k)
    n = int(tfc.RAYS_PER_DEG2 * 360.0 * (tfc.EL[1] - tfc.EL[0]))
    az = np.radians(rng.uniform(-180.0, 180.0, n))
    s = rng.uniform(math.sin(math.radians(tfc.EL[0])), math.sin(math.radians(tfc.EL[1])), n)
    el = np.arcsin(s)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
    o = np.asarray(origin, dtype=np.float64)
    t = np.full(n, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for axis, lim in ((2, FLOOR_Z), (2, CEIL_Z), (0, o[0] + BOX_HALF_M), (0, o[0] - BOX_HALF_M),
                          (1, o[1] + BOX_HALF_M), (1, o[1] - BOX_HALF_M)):
            tt = (lim - o[axis]) / d[:, axis]
            tt = np.where(tt > 0.05, tt, np.inf)
            t = np.minimum(t, tt)
    hit = np.isfinite(t)
    return (o + d[hit] * t[hit, None]).astype(np.float32)


class YawHarness(tfc.Harness):
    def __init__(self):
        super().__init__()
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from geometry_msgs.msg import PoseStamped
        from mars_quadrotor_msgs.msg import PositionCommand
        self.PoseStamped = PoseStamped
        self.goal_pub = self.node.create_publisher(PoseStamped, '/planning/follow_goal', 10)
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        self.node.create_subscription(PositionCommand, '/planning/pos_cmd', self._on_cmd, be)
        self.cmds = []                       # (wall t, x, y, z, yaw, yaw_dot)
        self.base_p = np.array([0.0, 0.0, HOVER_Z])
        self.odom_p = self.base_p.copy()
        self.odom_yaw = 0.0
        self.cmd_p = None
        self.cmd_yaw = None
        self.t0 = time.time()

    def _on_cmd(self, m):
        self.cmds.append((time.time(), m.position.x, m.position.y, m.position.z, m.yaw, m.yaw_dot))
        self.cmd_p = np.array([m.position.x, m.position.y, m.position.z])
        self.cmd_yaw = m.yaw

    def wobble(self):
        a = 2.0 * math.pi * (time.time() - self.t0) / WOBBLE_PERIOD_S
        return np.array([WOBBLE_M * math.cos(a), WOBBLE_M * math.sin(a), 0.0])

    def spin(self, secs):
        """Publish odometry at 20 Hz that tracks the planner's last command plus the hover wobble."""
        end = time.time() + secs
        while True:
            now = time.time()
            if now >= self.next_odom:
                if self.cmd_p is not None:
                    self.odom_p = self.cmd_p + self.wobble()
                    self.odom_yaw = self.cmd_yaw
                else:
                    self.odom_p = self.base_p + self.wobble()
                m = self.Odometry()
                m.header.stamp = self.node.get_clock().now().to_msg()
                m.header.frame_id = 'lidar_init'
                m.child_frame_id = 'lidar_imu'
                m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, self.odom_p)
                m.pose.pose.orientation.z = math.sin(self.odom_yaw / 2.0)
                m.pose.pose.orientation.w = math.cos(self.odom_yaw / 2.0)
                self.odom_pub.publish(m)
                self.next_odom = now + 0.05
            left = end - time.time()
            if left <= 0:
                return
            self.rclpy.spin_once(self.node, timeout_sec=min(0.02, left))

    def send_goal(self, p):
        g = self.PoseStamped()
        g.header.stamp = self.node.get_clock().now().to_msg()
        g.header.frame_id = 'lidar_init'
        g.pose.position.x, g.pose.position.y, g.pose.position.z = map(float, p)
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def run_phase(self, goal, secs, k0, goal_hz=2.0, scan_hz=1.0 / tfc.PACE_S):
        """Re-send `goal` at goal_hz and a scene scan at scan_hz for `secs`; return the commands seen."""
        n0 = len(self.cmds)
        next_goal = 0.0
        next_scan = 0.0
        k = k0
        end = time.time() + secs
        while time.time() < end:
            now = time.time()
            if now >= next_goal:
                self.send_goal(goal)
                next_goal = now + 1.0 / goal_hz
            if now >= next_scan:
                self.publish_scan(scan360(k, self.odom_p), time.time())
                k += 1
                next_scan = now + 1.0 / scan_hz
            self.spin(0.05)
        return self.cmds[n0:], k


def start_fsm(cfg_path, log_path, binary=None):
    env = dict(os.environ, ROS_DOMAIN_ID=tfc.DOMAIN, ROS_LOCALHOST_ONLY='1')
    rel = os.path.relpath(cfg_path, tfc.SUPER_CFG_DIR)
    log = open(log_path, 'w')
    if binary:
        argv = ['stdbuf', '-oL', binary, '--ros-args', '-p', 'config_name:=%s' % rel]
    else:
        argv = ['stdbuf', '-oL', 'ros2', 'run', 'super_planner', 'fsm_node', '--ros-args', '-p', 'config_name:=%s' % rel]
    proc = subprocess.Popen(argv, env=env, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)
    return proc, log


def unwrap(yaws):
    out = [yaws[0]]
    for y in yaws[1:]:
        d = y - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--binary', default=None, help='run this fsm_node binary instead of the installed one')
    ap.add_argument('--expect-spin', action='store_true',
                    help='invert H1: PASS only if the heading DOES spin (for the pre-0006 binary)')
    ap.add_argument('--logs', default=None)
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = tfc.DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    logdir = args.logs or tempfile.mkdtemp(prefix='yaw_hold_check_')
    os.makedirs(logdir, exist_ok=True)
    fails = 0
    results = []

    def check(name, ok, detail):
        nonlocal fails
        results.append((name, ok, detail))
        print('  [%s] %-44s %s' % ('ok  ' if ok else 'FAIL', name, detail), flush=True)
        if not ok:
            fails += 1

    print('\nyaw_hold_check -- real fsm_node%s on ROS_DOMAIN_ID %s (localhost only); logs in %s\n'
          % (' (%s)' % args.binary if args.binary else '', tfc.DOMAIN, logdir), flush=True)
    log_path = os.path.join(logdir, 'fsm_node.log')
    proc, log = start_fsm(tfc.BASE_CFG, log_path, args.binary)
    h = None
    try:
        h = YawHarness()
        h.spin(6.0)                                   # let fsm_node come up and see odom
        # a known map before the first goal: 20 scans at the harness pace
        k = 0
        for _ in range(20):
            h.publish_scan(scan360(k, h.odom_p), time.time())
            k += 1
            h.spin(tfc.PACE_S)
        check('fsm_node alive after scene', proc.poll() is None, 'pid %d' % proc.pid)

        # ---- H1: hold goal at the aircraft's own position, re-sent at 2 Hz ----------------------
        hold_goal = np.array([0.0, 0.0, HOVER_Z])
        h.send_goal(hold_goal)
        h.spin(2.0)
        cmds, k = h.run_phase(hold_goal, HOLD_S, k)
        check('H1 planner commanded during the hold', len(cmds) > 50, '%d pos_cmd in %.0f s' % (len(cmds), HOLD_S))
        if cmds:
            t_start = cmds[0][0] + 3.0                # ignore the first 3 s: the initial turn onto the trajectory
            body = [c for c in cmds if c[0] >= t_start]
            yaws = unwrap([c[4] for c in body]) if body else []
            excursion = (max(yaws) - min(yaws)) if yaws else float('nan')
            rates = sorted(abs(math.degrees(c[5])) for c in body)
            rate_p99 = rates[int(0.99 * (len(rates) - 1))] if rates else float('nan')
            total_turn = abs(yaws[-1] - yaws[0]) if yaws else float('nan')
            detail = ('yaw excursion %.1f deg, net turn %.1f deg, |yaw_dot| p99 %.1f deg/s over %d cmds'
                      % (math.degrees(excursion), math.degrees(total_turn), rate_p99, len(body)))
            held = (math.degrees(excursion) <= YAW_HOLD_DEG) and (rate_p99 <= RATE_HOLD_DPS)
            if args.expect_spin:
                check('H1 (OLD binary) heading SPINS on a hold', not held, detail)
            else:
                check('H1 heading holds on a re-planned hold goal', held, detail)
            disp = [math.hypot(c[1] - hold_goal[0], c[2] - hold_goal[1]) for c in body]
            check('H1 the planner was really re-planning near the goal',
                  bool(disp) and max(disp) < 0.5 and max(disp) > 0.02,
                  'cmd position from goal: min %.2f max %.2f m' % (min(disp), max(disp)) if disp else 'no cmds')

        # ---- H2: a real move, 3 m at +90 deg ----------------------------------------------------
        yaw_before = h.cmd_yaw if h.cmd_yaw is not None else 0.0
        move_goal = np.array([0.0, 3.0, HOVER_Z])
        cmds, k = h.run_phase(move_goal, MOVE_S, k)
        check('H2 planner commanded during the move', len(cmds) > 50, '%d pos_cmd in %.0f s' % (len(cmds), MOVE_S))
        if cmds:
            tail = cmds[-max(5, len(cmds) // 10):]
            yaw_end = math.degrees(np.mean(unwrap([c[4] for c in tail])))
            err = abs((yaw_end - 90.0 + 180.0) % 360.0 - 180.0)
            moved = math.hypot(cmds[-1][1] - cmds[0][1], cmds[-1][2] - cmds[0][2])
            detail = ('yaw %.0f -> %.0f deg (target +90, err %.0f), commanded travel %.2f m'
                      % (math.degrees(yaw_before), yaw_end, err, moved))
            if args.expect_spin:
                # The old binary carries its spin into the move, so its end yaw is wherever the
                # pirouette left it (135 deg on 2026-09-15's run). Informational only.
                print('  [info] H2 (OLD binary) nose on a real 3 m move        ' + detail, flush=True)
            else:
                check('H2 nose turns toward a real 3 m move', err <= YAW_MOVE_DEG and moved > 1.0, detail)
        check('fsm_node alive at the end', proc.poll() is None, '')
    finally:
        if h is not None:
            h.close()
        tfc.stop_fsm(proc, log)

    print()
    if fails:
        print('yaw_hold_check: %d FAIL' % fails)
        return 1
    print('yaw_hold_check: %d/%d PASS' % (len(results), len(results)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
