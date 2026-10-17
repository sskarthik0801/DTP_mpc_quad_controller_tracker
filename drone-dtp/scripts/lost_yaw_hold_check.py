#!/usr/bin/env python3
"""lost_yaw_hold_check.py -- the gate for follow_planner's yaw hold (lost_yaw_hold_s, coast_yaw_hold).

Why it exists: on the 2026-09-26 20:12 flight SUPER turned the camera 83 deg away from the walker
within ~2 s of the loss, because follow_planner fell silent on /planning/yaw_target and SUPER went
back to its free-end yaw (flight-log.md). SUPER also drops a coasting target's yaw target 0.5 s
into the coast (its perception stamp is never rewritten). Since 2026-09-26 follow_planner holds
ONE heading, toward where the camera was last aimed, from 0.4 s into a coast and after a loss.

The REAL follow_planner_node, sandboxed on a private localhost-only DDS domain (93). The script
plays target_manager (/dtp/follow_target) and the odometry. follow_planner has no authority:
nothing here reaches command_mux, SUPER or the aircraft. ~1 min.

  L1  hold        drone at the origin heading +30 deg, person 4 m out at 0 deg, then the target
                  stops. The yaw targets while it is fresh are the person, perception-stamped
                  (unchanged). After it goes stale: a held point within 0.5 s, on the bearing to
                  the person's last estimate (0 deg, not the aircraft's 30), ~100 m out, stamped
                  now, and the SAME point after the aircraft turns to -60 deg (latched). It stops
                  at lost_yaw_hold_s (2 s here) and stays silent after.
  L2  release     a fresh target mid-hold ends the hold at once (person again, perception stamp);
                  a second loss latches a NEW hold toward the new last estimate.
  L3  off         lost_yaw_hold_s 0: silence after the loss, the behaviour flown up to 2026-09-26.
  L4  no target   odometry but never a target: nothing on the yaw topic.
  L5  degenerate  the last estimate within 1 m of the aircraft: the hold keeps the aircraft's own
                  heading instead of a bearing that means nothing.
  L6  refuse      lost_yaw_hold_s -1 refuses to start.
  L7  coast       seen, then a 1.2 s coast (perception stamp frozen, as target_manager does), then
                  the loss. The person, perception-stamped, until the stamp is 0.4 s old; the held
                  point from before 0.5 s (SUPER's gate), one point through the coast AND the loss,
                  no flip back; the window counted from the latch.
  L8  flicker     a one-frame miss: never a held point.
  L9  coast off   coast_yaw_hold false: the coast keeps the perception stamp (the documented M6.5
                  carry-forward arm); the loss half still holds.
  L10 re-emerge   loss_conditioning, an OCCLUDED verdict aiming at (3, 3): after the loss the hold
                  keeps THAT bearing (45 deg), not the CTRV point's (0 deg).

Usage:
    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/lost_yaw_hold_check.py
    python3 drone-dtp/scripts/lost_yaw_hold_check.py --node-exe PATH     # test a scratch build
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

DOMAIN = '93'
DETECT_HZ = 8.0
ODOM_HZ = 10.0
HOLD_S = 2.0            # lost_yaw_hold_s under test: short, so the scene is short
STALE_S = 0.3           # follow_planner's follow_target_timeout_s default
TICK_S = 1.0 / 15.0     # follow_planner's goal timer
DRONE = (0.0, 0.0, 1.2)


def _env():
    return dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')


def default_exe():
    out = subprocess.run(['ros2', 'pkg', 'prefix', 'follow_planner'], capture_output=True, text=True)
    return os.path.join(out.stdout.strip(), 'lib', 'follow_planner', 'follow_planner_node')


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class PlannerNode:
    def __init__(self, exe, logpath, **params):
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
        from geometry_msgs.msg import PointStamped
        from dtp_msgs.msg import Target, LossEvent
        self.LossEvent = LossEvent
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('lost_yaw_hold_check')
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                        depth=10, durability=DurabilityPolicy.VOLATILE)
        self.Target, self.Odometry = Target, Odometry
        self.odom_pub = self.node.create_publisher(Odometry, '/lidar_slam/odom', be)
        self.target_pub = self.node.create_publisher(Target, '/dtp/follow_target', be)
        self.event_pub = self.node.create_publisher(LossEvent, '/planning/loss_event', be)
        self.node.create_subscription(PointStamped, '/planning/yaw_target', self._on_yaw, be)
        self.yaws = []           # (receive time, x, y, stamp seconds)
        self.sent_stamps = set()
        self.heading_deg = 30.0

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_yaw(self, m):
        self.yaws.append((self.now(), m.point.x, m.point.y, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))

    def wait_for_node(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.node.count_subscribers('/dtp/follow_target') >= 1 and \
                    self.node.count_publishers('/planning/yaw_target') >= 1:
                return True
        return False

    def _target(self, p, stamp=None, missed=0):
        from geometry_msgs.msg import Point
        t = self.Target()
        t.header.stamp = stamp if stamp is not None else self.node.get_clock().now().to_msg()
        t.missed_frames = missed
        self.sent_stamps.add(round(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, 6))
        t.header.frame_id = 'lidar_init'
        t.track_id = 7
        t.position.x, t.position.y, t.position.z = p
        t.track_confirmed = True
        t.is_locked_target = True
        t.confidence = 0.9
        t.distance_m = math.hypot(p[0] - DRONE[0], p[1] - DRONE[1])
        for i in range(20):   # a standing person: every forecast point is the position
            t.predicted_path.append(Point(x=p[0], y=p[1], z=p[2]))
            t.predicted_path_dt.append(0.2 + 0.075 * i)
        return t

    def _odom(self):
        o = self.Odometry()
        o.header.stamp = self.node.get_clock().now().to_msg()
        o.header.frame_id = 'lidar_init'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = DRONE
        yaw = math.radians(self.heading_deg)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.odom_pub.publish(o)

    def run(self, secs, person=None, turn_to=None, turn_at=None):
        """Odometry throughout; the target at DETECT_HZ while person is not None. Returns the time
        of the last target sent (or None)."""
        t0 = self.now()
        next_t = next_o = t0
        last = None
        while self.now() - t0 < secs:
            now = self.now()
            if turn_to is not None and now - t0 >= turn_at:
                self.heading_deg = turn_to
            if now >= next_o:
                self._odom()
                next_o += 1.0 / ODOM_HZ
            if person is not None and now >= next_t:
                self.target_pub.publish(self._target(person))
                last = now
                next_t += 1.0 / DETECT_HZ
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
        return last

    def coast(self, secs, person, stamp, event_at=None):
        """target_manager coasting: the target keeps coming at DETECT_HZ with missed_frames
        counting up and the stamp of the frame it was last SEEN in. event_at: answer every
        coasting target with an OCCLUDED LossEvent whose yaw target is that point."""
        from geometry_msgs.msg import Point
        t0 = self.now()
        next_t = next_o = t0
        missed = 0
        last = None
        while self.now() - t0 < secs:
            now = self.now()
            if now >= next_o:
                self._odom()
                next_o += 1.0 / ODOM_HZ
            if now >= next_t:
                missed += 1
                tgt = self._target(person, stamp=stamp, missed=missed)
                self.target_pub.publish(tgt)
                if event_at is not None:
                    ev = self.LossEvent()
                    ev.header = tgt.header
                    ev.eval_stamp = self.node.get_clock().now().to_msg()
                    ev.track_id = tgt.track_id
                    ev.missed_frames = missed
                    ev.cls = self.LossEvent.CLS_OCCLUDED
                    ev.query_p = tgt.position
                    ev.yaw_target_p = Point(x=event_at[0], y=event_at[1], z=0.9)
                    self.event_pub.publish(ev)
                last = now
                next_t += 1.0 / DETECT_HZ
            self.rclpy.spin_once(self.node, timeout_sec=0.005)
        return last

    def last_seen_stamp(self):
        from builtin_interfaces.msg import Time
        s = max(self.sent_stamps)
        return Time(sec=int(s), nanosec=int(round((s - int(s)) * 1e9)))

    def reset(self, heading_deg=30.0):
        self.yaws = []
        self.sent_stamps = set()
        self.heading_deg = heading_deg


def bearing_deg(y):
    return math.degrees(math.atan2(y[2] - DRONE[1], y[1] - DRONE[0]))


def is_person(y, p, tol=0.05):
    return math.hypot(y[1] - p[0], y[2] - p[1]) <= tol


def is_far(y):
    return math.hypot(y[1] - DRONE[0], y[2] - DRONE[1]) > 50.0


def fresh(y):
    return abs(y[0] - y[3]) < 0.2


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--node-exe', default=None, help='follow_planner_node under test (default: the sourced install)')
    ap.add_argument('--logs', default=None)
    a = ap.parse_args(argv)
    exe = a.node_exe or default_exe()
    if not os.path.isfile(exe):
        print('no follow_planner_node at %r -- source the workspace or pass --node-exe' % exe)
        return 2
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logdir = a.logs or tempfile.mkdtemp(prefix='lost_yaw_hold_check_')
    os.makedirs(logdir, exist_ok=True)
    print('\nlost_yaw_hold_check -- real follow_planner_node (%s) on ROS_DOMAIN_ID %s (localhost only); logs in %s\n'
          % (exe, DOMAIN, logdir))

    results = []

    def check(name, ok, detail=''):
        results.append(bool(ok))
        print('  %s  %-66s %s' % ('PASS' if ok else 'FAIL', name, detail))

    def scene(tag, **params):
        node = PlannerNode(exe, os.path.join(logdir, '%s.log' % tag), **params)
        if not h.wait_for_node():
            node.stop()
            check('%s: node came up' % tag, False, 'see %s.log' % tag)
            return None
        h.reset()
        return node

    h = Harness()
    try:
        # ---- L1 hold ------------------------------------------------------------------------
        person = (4.0, 0.0, 0.9)
        node = scene('L1', lost_yaw_hold_s=HOLD_S)
        if node:
            last = h.run(3.0, person=person)
            h.run(HOLD_S + 2.5, turn_to=-60.0, turn_at=0.5)
            log = node.stop()
            before = [y for y in h.yaws if y[0] <= last + 0.05]
            after = [y for y in h.yaws if y[0] > last + 0.05]
            held = [y for y in after if is_far(y)]
            check('L1 hold: while fresh, the yaw target is the person, perception-stamped',
                  len(before) > 10 and all(is_person(y, person) and round(y[3], 6) in h.sent_stamps for y in before),
                  '%d yaw targets' % len(before))
            first = held[0][0] - last if held else float('nan')
            check('L1 hold: a held point within %.1f s of the last target' % (STALE_S + 0.2),
                  bool(held) and first <= STALE_S + 0.2, '%.2f s' % first)
            b = [bearing_deg(y) for y in held]
            check('L1 hold: on the bearing to the last estimate (0 deg), not the heading (30)',
                  bool(b) and max(abs(wrap_deg(x)) for x in b) < 0.5,
                  'bearing %.2f..%.2f deg' % (min(b), max(b)) if b else 'none')
            pts = {(round(y[1], 3), round(y[2], 3)) for y in held}
            check('L1 hold: latched -- one point, unmoved by the aircraft turning to -60 deg',
                  len(pts) == 1, '%d distinct points in %d messages' % (len(pts), len(held)))
            check('L1 hold: every held point stamped now (SUPER\'s 0.5 s gate passes it)',
                  bool(held) and all(fresh(y) for y in held))
            end = held[-1][0] - last if held else float('nan')
            check('L1 hold: ends at lost_yaw_hold_s (%.1f s) after going stale' % HOLD_S,
                  bool(held) and HOLD_S <= end <= STALE_S + HOLD_S + 0.2, 'last held point %.2f s after the target' % end)
            # Until follow_target_timeout_s runs out the target is still fresh to the node, so the
            # person keeps coming (the unchanged behaviour); after that only the held point.
            other = [y for y in after if not is_far(y)]
            check('L1 hold: the person until stale, then only the held point, then silence',
                  all(is_person(y, person) and y[0] - last <= STALE_S + 0.15 for y in other)
                  and not [y for y in after if y[0] - last > STALE_S + HOLD_S + 0.3],
                  '%d person messages in the last %.2f s before stale' % (len(other), STALE_S))
            check('L1 hold: the node logs the hold and its end',
                  'lost: holding yaw at' in log and 'yaw hold ended' in log)

        # ---- L2 release and re-arm ------------------------------------------------------------
        p1, p2 = (4.0, 0.0, 0.9), (6.0, 2.0, 0.9)
        node = scene('L2', lost_yaw_hold_s=10.0)
        if node:
            h.run(2.0, person=p1)
            h.run(1.0)                                   # stale for 1 s: holding toward p1
            t_back = h.now()
            last2 = h.run(2.0, person=p2)                # a fresh target mid-hold
            h.run(1.0)                                   # lost again
            log = node.stop()
            mid = [y for y in h.yaws if y[0] < t_back]
            back = [y for y in h.yaws if t_back + 0.3 <= y[0] <= last2 + 0.05]
            again = [y for y in h.yaws if y[0] > last2 + 0.05]
            check('L2 release: holding toward the first estimate before the target returns',
                  any(is_far(y) and abs(wrap_deg(bearing_deg(y))) < 0.5 for y in mid))
            check('L2 release: a fresh target ends the hold -- the person, perception-stamped',
                  len(back) > 10 and all(is_person(y, p2) and round(y[3], 6) in h.sent_stamps for y in back),
                  '%d yaw targets' % len(back))
            want = math.degrees(math.atan2(p2[1], p2[0]))
            b2 = [bearing_deg(y) for y in again if is_far(y)]
            check('L2 re-arm: a second loss latches a NEW hold toward the new estimate',
                  bool(b2) and max(abs(wrap_deg(x - want)) for x in b2) < 0.5,
                  '%.2f deg (want %.2f)' % (b2[0], want) if b2 else 'none')
            check('L2: the node logs the release', 'yaw hold released' in log)

        # ---- L3 off ---------------------------------------------------------------------------
        node = scene('L3', lost_yaw_hold_s=0.0)
        if node:
            last = h.run(2.0, person=person)
            h.run(2.0)
            log = node.stop()
            after = [y for y in h.yaws if y[0] > last + STALE_S + 0.15]
            check('L3 off: lost_yaw_hold_s 0 -> nothing once stale (the flown behaviour)',
                  not after and 'yaw hold OFF' in log, '%d messages' % len(after))

        # ---- L4 no target yet -----------------------------------------------------------------
        node = scene('L4', lost_yaw_hold_s=HOLD_S)
        if node:
            h.run(2.0)
            node.stop()
            check('L4 no target yet: odometry alone publishes no yaw target', not h.yaws, '%d messages' % len(h.yaws))

        # ---- L5 degenerate --------------------------------------------------------------------
        near = (0.5, 0.0, 0.9)
        node = scene('L5', lost_yaw_hold_s=HOLD_S)
        if node:
            last = h.run(2.0, person=near)
            h.run(1.0)
            log = node.stop()
            b = [bearing_deg(y) for y in h.yaws if y[0] > last + 0.05 and is_far(y)]
            check('L5 degenerate: estimate within 1 m -> the aircraft\'s own heading (30 deg)',
                  bool(b) and max(abs(wrap_deg(x - 30.0)) for x in b) < 0.5 and "the aircraft's own heading" in log,
                  '%.2f deg' % b[0] if b else 'none')

        # ---- L6 refuse ------------------------------------------------------------------------
        node = PlannerNode(exe, os.path.join(logdir, 'L6.log'), lost_yaw_hold_s=-1.0)
        try:
            node.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pass
        exited = node.proc.poll() is not None
        log = node.stop()
        check('L6 refuse: lost_yaw_hold_s -1 refuses to start',
              exited and 'lost_yaw_hold_s must be' in log)
        # ---- L7 coast ---------------------------------------------------------------------
        node = scene('L7', lost_yaw_hold_s=HOLD_S)
        if node:
            h.run(2.0, person=person)
            seen = h.last_seen_stamp()
            t_seen = seen.sec + seen.nanosec * 1e-9
            last = h.coast(1.2, person, seen)
            h.run(HOLD_S + 1.0)
            log = node.stop()
            coastal = [y for y in h.yaws if y[0] > t_seen + 0.05]
            held = [y for y in coastal if is_far(y)]
            first = held[0][0] - t_seen if held else float('nan')
            before = [y for y in coastal if not held or y[0] < held[0][0]]
            after = [y for y in coastal if held and y[0] >= held[0][0]]
            check('L7 coast: the person, perception-stamped, until the hold takes over',
                  len(before) > 2 and all(is_person(y, person) and abs(y[3] - t_seen) < 1e-6 for y in before),
                  '%d yaw targets' % len(before))
            check('L7 coast: the held point starts at a stamp age in [0.40, 0.50) s -- before SUPER drops it',
                  bool(held) and 0.40 <= first < 0.50, 'stamp age %.3f s' % first)
            pts = {(round(y[1], 3), round(y[2], 3)) for y in after}
            check('L7 coast: one held point through the coast and the loss, no flip back',
                  bool(held) and len(pts) == 1 and len(after) == len(held) and held[-1][0] > last + STALE_S,
                  '%d distinct points; last %.2f s after the last coasted target' % (len(pts), held[-1][0] - last if held else 0))
            b = [bearing_deg(y) for y in held]
            check('L7 coast: toward where the camera was aimed (0 deg), stamped now',
                  bool(b) and max(abs(wrap_deg(x)) for x in b) < 0.5 and all(fresh(y) for y in held))
            span = held[-1][0] - held[0][0] if held else float('nan')
            check('L7 coast: the window runs from the latch (%.1f s)' % HOLD_S,
                  bool(held) and HOLD_S - 0.2 <= span <= HOLD_S + 0.2 and 'coasting past' in log,
                  'held for %.2f s' % span)

        # ---- L8 flicker -------------------------------------------------------------------
        node = scene('L8', lost_yaw_hold_s=HOLD_S)
        if node:
            h.run(2.0, person=person)
            h.coast(0.1, person, h.last_seen_stamp())     # one missed frame
            h.run(1.0, person=person)
            log = node.stop()
            check('L8 flicker: a one-frame miss never holds',
                  not [y for y in h.yaws if is_far(y)] and 'holding yaw' not in log,
                  '%d held points' % len([y for y in h.yaws if is_far(y)]))

        # ---- L9 coast off -----------------------------------------------------------------
        node = scene('L9', lost_yaw_hold_s=HOLD_S, coast_yaw_hold=False)
        if node:
            h.run(2.0, person=person)
            seen = h.last_seen_stamp()
            t_seen = seen.sec + seen.nanosec * 1e-9
            last = h.coast(1.2, person, seen)
            h.run(1.0)
            log = node.stop()
            during = [y for y in h.yaws if t_seen + 0.05 < y[0] <= last + 0.05]
            after = [y for y in h.yaws if y[0] > last + STALE_S + 0.15]
            check('L9 coast off: the coast keeps the perception stamp (the M6.5 carry-forward arm)',
                  len(during) > 5 and all(is_person(y, person) and abs(y[3] - t_seen) < 1e-6 for y in during),
                  '%d yaw targets' % len(during))
            check('L9 coast off: the loss half still holds, toward 0 deg',
                  bool(after) and all(is_far(y) and abs(wrap_deg(bearing_deg(y))) < 0.5 for y in after)
                  and 'at a loss only' in log, '%d held points' % len(after))

        # ---- L10 re-emergence (conditioning) --------------------------------------------------
        rp = (3.0, 3.0)
        node = scene('L10', lost_yaw_hold_s=HOLD_S, loss_conditioning=True)
        if node:
            h.run(2.0, person=person)
            seen = h.last_seen_stamp()
            t_seen = seen.sec + seen.nanosec * 1e-9
            last = h.coast(1.0, person, seen, event_at=rp)
            h.run(1.0)
            log = node.stop()
            cond = [y for y in h.yaws if t_seen + 0.4 < y[0] <= last + 0.05]
            after = [y for y in h.yaws if y[0] > last + STALE_S + 0.15]
            check('L10 re-emerge: while coasting, the camera aims at the oracle\'s point, fresh',
                  len(cond) > 3 and all(is_person(y, rp) and fresh(y) for y in cond), '%d yaw targets' % len(cond))
            b = [bearing_deg(y) for y in after]
            check('L10 re-emerge: after the loss the hold keeps that bearing (45 deg), not the CTRV one (0)',
                  bool(b) and all(is_far(y) for y in after) and max(abs(wrap_deg(x - 45.0)) for x in b) < 0.5,
                  '%.2f deg' % b[0] if b else 'none')
    finally:
        h.close()

    print('\n%d/%d checks passed' % (sum(results), len(results)))
    return 0 if results and all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
