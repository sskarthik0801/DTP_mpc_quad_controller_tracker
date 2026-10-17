#!/usr/bin/env python3
"""Bench gate for patch 0009 (park SUPER on a takeover), against the REAL fsm_node. No hardware.

WHAT IT PROVES
--------------
SUPER's FSM has no "stop planning". A trajectory that ends more than 0.1 m from the goal sends it
to GENERATE_TRAJ, which calls PlanFromRest on every 10 ms tick until a plan succeeds or the
aircraft reaches the goal. After a pilot takeover neither happens. On 2026-09-30 it re-planned to
a stale goal for 45 minutes after a flight: 76,229 failed plans, fsm_node at 144 % of a core, the
Jetson at load 10-12 and the GCS taking 20 s per request.
Patch 0009 adds a park input: std_msgs/Empty on /planning/park drops the goal, puts the FSM in
WAIT_GOAL and silences pos_cmd until the next goal. command_mux sends it when its engage gate
shuts.

Same harness as yaw_hold_check (real fsm_node on follow_test_ros2.yaml, a private localhost-only
DDS domain, a synthetic MID360 room, odometry that follows /planning/pos_cmd). A "takeover" here
is the odometry no longer following the command: the aircraft goes where the pilot takes it.

  P0  PARK BEFORE ANY GOAL   a park with no goal ever sent must not start SUPER or break the first
                             goal: no pos_cmd after it, and the first goal is then flown.
  P1  THE LOOP (no park)     goal 4 m away, takeover part-way, aircraft held 2.5 m aside. PASS:
                             SUPER is still planning and commanding LOOP_S later. This is the
                             fault, reproduced, so P2 means something.
  P2  PARK                   one park message. PASS: pos_cmd stops within PARK_STOP_S and stays
                             silent for QUIET_S, no PlanFromRest in that time, "PARKED" logged.
  P3  PARK IS IDEMPOTENT     three more parks: still silent, logged once only.
  P4  THE NEXT GOAL          a new goal after the park. PASS: pos_cmd resumes, and its first
                             command is at the aircraft (planned from rest there, not from where
                             the old reference was).
  P5  PARK WHILE FOLLOWING   a park mid-trajectory with the odometry still following: pos_cmd
                             stops at once, and the goal is not resumed.

    source ~/super_ws/install/setup.bash
    python3 drone-dtp/scripts/park_check.py        # ~2 min, exits non-zero on any FAIL
"""
import argparse
import math
import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import target_filter_check as tfc  # noqa: E402
import yaw_hold_check as yhc       # noqa: E402

HOVER_Z = yhc.HOVER_Z
LOOP_S = 12.0          # P1: how long SUPER is watched re-planning to the stale goal
PARK_STOP_S = 0.5      # P2: pos_cmd must be silent this soon after the park
QUIET_S = 10.0         # P2: and stay silent this long
START_TOL_M = 0.35     # P4: the first command after a park is this close to the aircraft
                       #     (the harness wobbles the odometry by 0.20 m)
PARK_TOPIC = '/planning/park'


class ParkHarness(yhc.YawHarness):
    def __init__(self):
        super().__init__()
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from std_msgs.msg import Empty
        self.Empty = Empty
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.park_pub = self.node.create_publisher(Empty, PARK_TOPIC, be)
        self.follow_cmd = True      # False = takeover: the odometry stays at self.pilot_p
        self.pilot_p = None
        self.cmd_yaw = 0.0

    def _on_cmd(self, m):
        self.cmds.append((time.time(), m.position.x, m.position.y, m.position.z, m.yaw, m.yaw_dot))
        if self.follow_cmd:
            self.cmd_p = np.array([m.position.x, m.position.y, m.position.z])
            self.cmd_yaw = m.yaw

    def takeover(self, p):
        """The pilot holds the aircraft at p, whatever SUPER commands from here on."""
        self.follow_cmd = False
        self.pilot_p = [float(v) for v in p]
        self.base_p = np.array(self.pilot_p)
        self.cmd_p = None

    def handback(self):
        """Offboard again: the odometry follows the next command, and stays put until one comes."""
        self.follow_cmd = True
        self.cmd_p = None

    def park(self):
        self.park_pub.publish(self.Empty())

    def run(self, secs, k, goal=None, goal_hz=2.0):
        """Spin for `secs` with scans; re-send `goal` at goal_hz when given. Returns (cmds, k)."""
        n0 = len(self.cmds)
        next_goal = 0.0
        next_scan = 0.0
        end = time.time() + secs
        while time.time() < end:
            now = time.time()
            if goal is not None and now >= next_goal:
                self.send_goal(goal)
                next_goal = now + 1.0 / goal_hz
            if now >= next_scan:
                self.publish_scan(yhc.scan360(k, self.odom_p), time.time())
                k += 1
                next_scan = now + tfc.PACE_S
            self.spin(0.05)
        return self.cmds[n0:], k


def log_count(path, needle):
    try:
        with open(path, errors='ignore') as f:
            return sum(needle in line for line in f)
    except OSError:
        return -1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--logs', default=None)
    args = ap.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = tfc.DOMAIN
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    logdir = args.logs or tempfile.mkdtemp(prefix='park_check_')
    os.makedirs(logdir, exist_ok=True)
    fails = 0
    results = []

    def check(name, ok, detail):
        nonlocal fails
        results.append((name, ok, detail))
        print('  [%s] %-50s %s' % ('ok  ' if ok else 'FAIL', name, detail), flush=True)
        if not ok:
            fails += 1

    print('\npark_check -- real fsm_node on ROS_DOMAIN_ID %s (localhost only); logs in %s\n'
          % (tfc.DOMAIN, logdir), flush=True)
    log_path = os.path.join(logdir, 'fsm_node.log')
    proc, log = yhc.start_fsm(tfc.BASE_CFG, log_path)
    h = None
    try:
        h = ParkHarness()
        h.spin(6.0)
        k = 0
        for _ in range(20):
            h.publish_scan(yhc.scan360(k, h.odom_p), time.time())
            k += 1
            h.spin(tfc.PACE_S)
        check('fsm_node alive after scene', proc.poll() is None, 'pid %d' % proc.pid)
        subs = h.park_pub.get_subscription_count()
        check('fsm_node subscribes %s' % PARK_TOPIC, subs == 1, '%d subscriber(s)' % subs)

        # ---- P0: a park before any goal ---------------------------------------------------------
        for _ in range(3):
            h.park()
            h.spin(0.2)
        cmds, k = h.run(3.0, k)
        parked0 = log_count(log_path, 'PARKED')     # the parks above are logged once, as one park
        check('P0 a park before any goal commands nothing', len(cmds) == 0 and parked0 == 1,
              '%d pos_cmd in 3 s, %d "PARKED" line(s)' % (len(cmds), parked0))

        # ---- P1: the loop. Goal 4 m ahead, takeover part-way, aircraft held aside ---------------
        goal = np.array([4.0, 0.0, HOVER_Z])
        cmds, k = h.run(3.0, k, goal=goal)
        check('P0 the first goal after it is flown', len(cmds) > 50, '%d pos_cmd in 3 s' % len(cmds))
        # The pilot takes over and holds the aircraft 2.5 m to the side of where it was. The mux's
        # gate would shut here and send no more goals; so does this harness.
        h.takeover([h.odom_p[0], h.odom_p[1] + 2.5, HOVER_Z])
        plans0 = log_count(log_path, '[PlanFromRest]')
        cmds, k = h.run(LOOP_S, k)
        plans1 = log_count(log_path, '[PlanFromRest]')
        late = [c for c in cmds if c[0] >= cmds[0][0] + LOOP_S - 3.0] if cmds else []
        check('P1 without a park SUPER keeps planning to the stale goal',
              (plans1 - plans0) >= 3 and len(late) > 0,
              '%d PlanFromRest lines and %d pos_cmd in %.0f s, %d pos_cmd in the last 3 s'
              % (plans1 - plans0, len(cmds), LOOP_S, len(late)))

        # ---- P2: park ---------------------------------------------------------------------------
        t_park = time.time()
        h.park()
        cmds, k = h.run(QUIET_S, k)
        after = [c for c in cmds if c[0] > t_park + PARK_STOP_S]
        last = (max(c[0] for c in cmds) - t_park) if cmds else 0.0
        check('P2 pos_cmd stops after one park', len(after) == 0,
              '%d pos_cmd later than %.1f s after the park (last one at +%.2f s), watched %.0f s'
              % (len(after), PARK_STOP_S, last, QUIET_S))
        h.spin(0.5)
        plans2 = log_count(log_path, '[PlanFromRest]')
        check('P2 no planning while parked', (plans2 - plans1) <= 2,
              '%d PlanFromRest lines since the park' % (plans2 - plans1))
        parked = log_count(log_path, 'PARKED') - parked0
        check('P2 the park is logged', parked == 1, '%d new "PARKED" line(s)' % parked)

        # ---- P3: idempotent ---------------------------------------------------------------------
        for _ in range(3):
            h.park()
            h.spin(0.3)
        cmds, k = h.run(2.0, k)
        parked = log_count(log_path, 'PARKED') - parked0
        check('P3 repeated parks change nothing', len(cmds) == 0 and parked == 1,
              '%d pos_cmd, %d new "PARKED" line(s)' % (len(cmds), parked))

        # ---- P4: the next goal is planned from rest at the aircraft ------------------------------
        here = np.array(h.odom_p, dtype=float)
        goal2 = np.array([here[0] - 3.0, here[1], HOVER_Z])
        h.handback()
        cmds, k = h.run(4.0, k, goal=goal2)
        check('P4 a new goal un-parks', len(cmds) > 50, '%d pos_cmd in 4 s' % len(cmds))
        if cmds:
            first = np.array(cmds[0][1:4])
            d = float(np.linalg.norm(first[:2] - np.array(h.pilot_p[:2])))
            d_old = float(np.linalg.norm(first[:2] - goal[:2]))
            check('P4 its first command is at the aircraft', d <= START_TOL_M,
                  'first pos_cmd %.2f m from the aircraft, %.2f m from the old goal' % (d, d_old))
            moved = math.hypot(cmds[-1][1] - cmds[0][1], cmds[-1][2] - cmds[0][2])
            check('P4 and it flies toward the new goal', moved > 0.5, 'commanded travel %.2f m' % moved)

        # ---- P5: park mid-trajectory, odometry still following ----------------------------------
        t_park = time.time()
        before = [c for c in h.cmds if c[0] > t_park - 0.5]
        h.park()
        cmds, k = h.run(5.0, k)
        after = [c for c in cmds if c[0] > t_park + PARK_STOP_S]
        parked = log_count(log_path, 'PARKED') - parked0
        check('P5 a park mid-trajectory stops the commands',
              len(before) > 20 and len(after) == 0 and parked == 2,
              '%d pos_cmd in the 0.5 s before the park, %d later than %.1f s after it, '
              '%d new "PARKED" line(s)' % (len(before), len(after), PARK_STOP_S, parked))
        check('fsm_node alive at the end', proc.poll() is None, '')
    finally:
        if h is not None:
            h.close()
        tfc.stop_fsm(proc, log)

    print()
    if fails:
        print('park_check: %d FAIL' % fails)
        return 1
    print('park_check: %d/%d PASS' % (len(results), len(results)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
