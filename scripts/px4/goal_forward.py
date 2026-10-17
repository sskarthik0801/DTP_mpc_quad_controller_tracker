#!/usr/bin/env python3
"""Send SUPER one short goal: `--distance` metres straight ahead, at `--height`.

    source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash
    python3 scripts/px4/goal_forward.py --dry-run      # print the goal, send nothing
    python3 scripts/px4/goal_forward.py                # 1.0 m forward at 0.5 m

This is flight-test P6 made *relative*. `/goal_pose` is absolute in FAST-LIO's
`lidar_init` frame, so "1 m forward" is not `x: 1.0` unless the aircraft happens to be
sitting on the origin facing +x. This reads the live `/lidar_slam/odom`, adds the offset
along the **current heading**, and sends the result.

Four things about SUPER's goal handling that are not obvious from the topic
--------------------------------------------------------------------------
1. **`fsm/click_height` OVERRIDES the z you send.** `Fsm::setGoalPosiAndYaw`
   (`super_core/fsm.cpp:187`) does `if (click_height > -5) click_point.z() = click_height;`
   before anything else. The config ships **1.5**, so today every goal — including the
   `z: 1.2` in `docs/flight-test-plan.md` §P6 — is silently flown at 1.5 m. Height is a
   **config** setting, not a message field. This script refuses to send unless the
   installed config already matches `--height`, and prints the edit if it does not.
2. **The goal gets snapped.** `getNearestInfCellNot(OCCUPIED, ..., 3.0)` moves the goal to
   the nearest non-inflated-occupied cell **up to 3 m away**, so what SUPER flies to is
   not necessarily what you sent. Watch the `-- [Fsm] Get goal at` line in the stack log
   and compare it against the goal printed here.
3. **Goals closer than 0.1 m to the current position are silently dropped** (same
   function). Nothing is logged; the FSM just stays in WAIT_GOAL.
4. **The heading follows the velocity**, so the aircraft yaws into the direction of travel.
   Straight ahead, that is a no-op — which is why this script keeps the goal yaw equal to
   the current yaw rather than inventing one. The click config runs `yaw_mode: 3` since
   2026-09-12, which ignores the goal yaw entirely; with nothing on `/planning/yaw_target`
   it is still heading-to-velocity, so this script is unaffected
   (`docs/fact-super-yaw-mode.md`).

Why it refuses to send while disarmed or out of Offboard
--------------------------------------------------------
`px4_offboard_bridge` only forwards `/planning/pos_cmd` when armed **and** in Offboard;
otherwise it latches the live estimate (IDLE). Send a goal on the ground and SUPER starts
its trajectory clock anyway — then the moment you engage Offboard and arm, the bridge
picks the trajectory up **wherever it has already got to**, not at the start. That is a
lurch, not a takeoff. `--allow-disarmed` exists for bench work with props off.

**Ctrl-C here does NOT stop the aircraft.** This script is a publisher, not a controller;
SUPER owns the trajectory once the goal is accepted. The abort is the RC ladder:
channel-11 Offboard switch OFF (you get whatever channel 5 is on — leave it on Position).
See docs/flight-test-plan.md §5.
"""
import argparse
import math
import os
import re
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mars_quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

ODOM_TOPIC = '/lidar_slam/odom'
GOAL_TOPIC = '/goal_pose'
CMD_TOPIC = '/planning/pos_cmd'

SETTLE = 5.0        # DDS discovery on this box routinely needs >2 s for every /fmu/ topic
MIN_ODOM_HZ = 5.0   # FAST-LIO runs at 10 Hz; half of that is already broken
ARRIVE_R = 0.20     # "there" for the progress monitor, in metres

CONFIG_CANDIDATES = [
    '~/super_ws/install/super_planner/share/super_planner/config/click_smooth_ros2.yaml',
    '~/super_ws/src/SUPER/super_planner/config/click_smooth_ros2.yaml',
]


def read_click_height(path):
    """`fsm/click_height` out of the planner config, or None if unreadable."""
    try:
        with open(path) as f:
            m = re.search(r'^\s*click_height:\s*([-+0-9.eE]+)', f.read(), re.M)
        return float(m.group(1)) if m else None
    except OSError:
        return None


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalSender(Node):

    def __init__(self):
        super().__init__('goal_forward')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        # SUPER subscribes best_effort/keep_last(1)/volatile (FsmRos2::init). A RELIABLE
        # publisher is compatible with that; a BEST_EFFORT one would also match, but
        # RELIABLE means the single goal message cannot be dropped on the wire.
        self.goal_pub = self.create_publisher(
            PoseStamped, GOAL_TOPIC,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

        self.odom = None
        self.n_odom = 0
        self.lpos = None
        self.mode = None
        self.n_cmd = 0

        # FAST-LIO publishes /Odometry with default QoS (RELIABLE); BEST_EFFORT here is
        # compatible with that and with anything else that might republish it.
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            durability=DurabilityPolicy.VOLATILE,
                                            history=HistoryPolicy.KEEP_LAST, depth=10))
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 lambda m: setattr(self, 'lpos', m), px4_qos)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode',
                                 lambda m: setattr(self, 'mode', m), px4_qos)
        # Must match SUPER's publisher exactly or an rclpy default (RELIABLE) silently
        # never connects -- the trap documented in docs/fact-super-cmd-stream.md.
        self.create_subscription(PositionCommand, CMD_TOPIC, self.on_cmd,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            durability=DurabilityPolicy.VOLATILE,
                                            history=HistoryPolicy.KEEP_LAST, depth=1))

    def on_odom(self, m):
        self.odom = m
        self.n_odom += 1

    def on_cmd(self, _):
        self.n_cmd += 1

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self):
        """(x, y, z, yaw) in lidar_init, or None."""
        if self.odom is None:
            return None
        p = self.odom.pose.pose
        return p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)


def preflight(n, args):
    """Returns a list of (label, ok, detail). Empty ok=False means clear to send."""
    out = []

    n.n_odom = 0
    t0 = time.time()
    n.spin(2.0)
    hz = n.n_odom / (time.time() - t0)
    out.append((f'LiDAR odometry live ({ODOM_TOPIC})', hz >= MIN_ODOM_HZ,
                f'{hz:.1f} Hz from {n.count_publishers(ODOM_TOPIC)} publisher(s)'))

    subs = n.goal_pub.get_subscription_count()
    out.append(('SUPER fsm_node subscribed to ' + GOAL_TOPIC, subs > 0,
                f'{subs} subscriber(s); {n.count_publishers(CMD_TOPIC)} publisher(s) '
                f'on {CMD_TOPIC}'))

    sp_pubs = n.count_publishers('/fmu/in/trajectory_setpoint')
    out.append(('px4_offboard_bridge running', sp_pubs > 0,
                f'{sp_pubs} publisher(s) on /fmu/in/trajectory_setpoint'))

    p = n.lpos
    ok = p is not None and p.xy_valid and p.z_valid and not p.dead_reckoning
    out.append(('EKF2 position valid, not dead-reckoning', ok,
                'no /fmu/out/vehicle_local_position_v1' if p is None else
                f'xy_valid={p.xy_valid} z_valid={p.z_valid} '
                f'dead_reckoning={p.dead_reckoning}'))

    m = n.mode
    engaged = m is not None and m.flag_armed and m.flag_control_offboard_enabled
    out.append(('armed and in Offboard', engaged or args.allow_disarmed,
                'no /fmu/out/vehicle_control_mode' if m is None else
                f'armed={m.flag_armed} offboard={m.flag_control_offboard_enabled}'
                + ('  (overridden by --allow-disarmed)'
                   if args.allow_disarmed and not engaged else '')))
    return out


def check_click_height(want):
    """SUPER flies at fsm/click_height, not at the goal's z. Verify they agree."""
    seen = []
    for c in CONFIG_CANDIDATES:
        path = os.path.expanduser(c)
        h = read_click_height(path)
        if h is not None:
            seen.append((path, h))
    if not seen:
        return False, ('could not read fsm/click_height from any planner config -- '
                       'checked ' + ', '.join(CONFIG_CANDIDATES)), seen
    vals = {h for _, h in seen}
    if len(vals) > 1:
        return False, ('src/ and install/ disagree on click_height '
                       f'({sorted(vals)}) -- a rebuild reverted one of them'), seen
    have = seen[0][1]
    if have <= -5.0:
        return True, (f'click_height={have} (<= -5), so the goal message z is honoured '
                      'directly -- note an RViz "2D Goal Pose" click sends z=0'), seen
    if abs(have - want) > 1e-6:
        return False, (f'config flies every goal at {have} m, not the {want} m asked '
                       'for -- height is a config setting, not a message field'), seen
    return True, f'click_height={have} m, matching --height', seen


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--distance', type=float, default=1.0,
                    help='metres along the current heading (default 1.0)')
    ap.add_argument('--height', type=float, default=0.5,
                    help='altitude in lidar_init, i.e. above the FAST-LIO init pose, '
                         'NOT above ground (default 0.5)')
    ap.add_argument('--dry-run', action='store_true',
                    help='run every check and print the goal; publish nothing')
    ap.add_argument('--yes', action='store_true', help='skip the typed confirmation')
    ap.add_argument('--allow-disarmed', action='store_true',
                    help='bench only, PROPS OFF: send while not armed+Offboard')
    ap.add_argument('--monitor', type=float, default=40.0,
                    help='seconds to report progress after sending (0 to exit at once)')
    args = ap.parse_args()

    rclpy.init()
    n = GoalSender()
    try:
        print(f'settling for {SETTLE:.0f} s (DDS discovery)...')
        n.spin(SETTLE)

        checks = preflight(n, args)
        hok, hmsg, seen = check_click_height(args.height)
        checks.append((f'planner flies at --height ({args.height} m)', hok, hmsg))

        print()
        for label, ok, detail in checks:
            print(f'  [{"OK  " if ok else "FAIL"}] {label:<46} {detail}')

        pose = n.pose()
        if pose is None:
            print('\nREFUSING: no pose on ' + ODOM_TOPIC + '. Is the M1 stack up?')
            return 1
        x, y, z, yaw = pose

        gx = x + args.distance * math.cos(yaw)
        gy = y + args.distance * math.sin(yaw)
        gz = args.height

        print(f'\n  now   lidar_init [{x:+.2f} {y:+.2f} {z:+.2f}] '
              f'yaw {math.degrees(yaw):+.1f} deg')
        print(f'  goal  lidar_init [{gx:+.2f} {gy:+.2f} {gz:+.2f}] '
              f'yaw {math.degrees(yaw):+.1f} deg (unchanged)')
        print(f'  move  {args.distance:+.2f} m forward, {gz - z:+.2f} m vertical')

        if args.distance < 0.1:
            print('\nREFUSING: SUPER silently drops goals within 0.1 m of the current '
                  'position (fsm.cpp:200). Nothing would happen.')
            return 1
        if abs(gz - z) > 0.3:
            print(f'  NOTE: this also changes altitude by {gz - z:+.2f} m. SUPER will '
                  'fly one trajectory doing both.')

        failed = [label for label, ok, _ in checks if not ok]
        if failed:
            print('\nREFUSING, ' + str(len(failed)) + ' check(s) failed:')
            for label in failed:
                print('  - ' + label)
            if not hok and seen and seen[0][1] > -5.0:
                print('\nTo fly at ' + f'{args.height}' + ' m, set it in BOTH copies '
                      '(a rebuild reverts install/ from src/, and only install/ is '
                      'read at run time):')
                for path, _ in seen:
                    print(f"  sed -i 's/^\\(\\s*click_height:\\).*/\\1 {args.height}/' "
                          f'{path}')
                print('  # then restart the stack -- the config is read once, at init')
            return 1

        if args.dry_run:
            print('\n--dry-run: nothing published.')
            return 0

        if not args.yes:
            print('\n*** This commands the aircraft to MOVE. Abort is the channel-11 '
                  'Offboard switch, not Ctrl-C. ***')
            try:
                if input('Type GO to send: ').strip() != 'GO':
                    print('aborted')
                    return 130
            except EOFError:
                print('\nno tty for the confirmation -- pass --yes deliberately')
                return 1

        msg = PoseStamped()
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = gx, gy, gz
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        n.n_cmd = 0
        n.goal_pub.publish(msg)
        print(f'sent -> {GOAL_TOPIC}')

        if args.monitor <= 0:
            return 0

        print('watching (Ctrl-C stops the REPORT, not the aircraft)')
        t0 = time.time()
        best = float('inf')
        while time.time() - t0 < args.monitor:
            n.spin(0.5)
            p = n.pose()
            if p is None:
                continue
            d = math.dist((p[0], p[1], p[2]), (gx, gy, gz))
            best = min(best, d)
            print(f'  t={time.time() - t0:5.1f}s  '
                  f'[{p[0]:+.2f} {p[1]:+.2f} {p[2]:+.2f}]  '
                  f'{d:5.2f} m to go   {CMD_TOPIC} {n.n_cmd} msgs')
            if d < ARRIVE_R:
                print(f'\narrived: {d:.2f} m from the goal after '
                      f'{time.time() - t0:.1f} s.')
                return 0
        if n.n_cmd == 0:
            print(f'\nSUPER never published on {CMD_TOPIC}. Either the goal was '
                  'snapped/rejected (check the stack log for "Get goal at" or "deeply '
                  'occupied") or the FSM never left WAIT_GOAL.')
            return 1
        print(f'\nstill {best:.2f} m short after {args.monitor:.0f} s '
              f'({n.n_cmd} commands seen). Not necessarily a fault -- at max_vel 0.5 '
              'a long leg takes a while -- but check the stack log.')
        return 1
    except KeyboardInterrupt:
        print('\ninterrupted -- THIS DID NOT STOP THE AIRCRAFT. Use the RC.')
        return 130
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
