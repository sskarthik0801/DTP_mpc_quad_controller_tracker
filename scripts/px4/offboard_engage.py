#!/usr/bin/env python3
"""M3 operator tool: switch PX4 into Offboard, and (only if asked) arm or disarm.

The bridge (`px4_offboard_bridge.py`) deliberately never changes mode and never arms
-- a node that can spin motors on its own is the wrong shape. This script is the
explicit, human-initiated half of that split.

    source /opt/ros/humble/setup.bash && source ~/super_ws/install/setup.bash

    python3 scripts/px4/offboard_engage.py --status     # read-only: what is PX4 doing?
    python3 scripts/px4/offboard_engage.py              # engage Offboard (no arming)
    python3 scripts/px4/offboard_engage.py --arm        # engage, then ARM  (PROPS OFF)
    python3 scripts/px4/offboard_engage.py --disarm     # disarm
    python3 scripts/px4/offboard_engage.py --hold       # leave Offboard -> Hold

Preconditions it checks before commanding anything (refuses otherwise):
  * `/fmu/in/offboard_control_mode` is being published at >2 Hz -- PX4 rejects the
    Offboard switch outright without that heartbeat already streaming.
  * EKF2 has a valid local position (`xy_valid`/`z_valid`) and is not dead-reckoning.

`/fmu/in/trajectory_setpoint` is NOT a precondition (corrected 2026-09-28). PX4 v1.17 needs
only the heartbeat and a valid position to allow Offboard (offboardCheck.cpp:44-61), and
since 2026-09-28 the bridge sends setpoints only once PX4 reports Offboard. So the
setpoint stream is checked AFTER the switch instead: it must then be running.

It does NOT check for RC. Note `COM_RCL_EXCEPT` bit 2 (value 4) must be set for PX4 to
allow Offboard with no RC link at all; with it clear and no receiver bound, the mode
switch is refused and you will see VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED. That is a
deliberate safety interlock -- clearing it also removes RC as the manual-takeover path,
so it is a decision to make on purpose, not a workaround to reach for.

Unlike the rest of scripts/px4/, this speaks ROS 2 (uXRCE-DDS), not MAVLink -- so it
needs the workspace sourced rather than pymavlink.
"""
import argparse
import sys
import time

import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleCommandAck,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_OFFBOARD / _AUTO+LOITER.
CUSTOM = 1.0
MAIN_OFFBOARD = 6.0
MAIN_AUTO, SUB_LOITER = 4.0, 3.0

# DDS discovery on this box routinely takes >2 s to hook up every /fmu/ topic, and
# vehicle_status only publishes at 2 Hz -- too short a settle reports "nothing there"
# for a perfectly healthy link.
SETTLE = 5.0

NAV = {v: k[len('NAVIGATION_STATE_'):]
       for k, v in vars(VehicleStatus).items() if k.startswith('NAVIGATION_STATE_')}
ACK = {v: k[len('VEHICLE_CMD_RESULT_'):]
       for k, v in vars(VehicleCommandAck).items()
       if k.startswith('VEHICLE_CMD_RESULT_')}


class Engager(Node):

    def __init__(self):
        super().__init__('offboard_engage')
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command',
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.status = None
        self.lpos = None
        self.acks = []
        self.n_ocm = 0
        self.n_sp = 0
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._set('status'), px4_qos)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self._set('lpos'), px4_qos)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 lambda m: self.acks.append(m), px4_qos)
        # The bridge's own output, watched locally to confirm it is actually streaming.
        self.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode',
                                 self._bump('n_ocm'), px4_qos)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                 self._bump('n_sp'), px4_qos)

    def _set(self, attr):
        return lambda m: setattr(self, attr, m)

    def _bump(self, attr):
        return lambda m: setattr(self, attr, getattr(self, attr) + 1)

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, command, p1=0.0, p2=0.0, p3=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.acks.clear()
        self.cmd_pub.publish(m)

    def await_ack(self, command, secs=3.0):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            for a in self.acks:
                if a.command == command:
                    return a
        return None


def show(node, label):
    s, p = node.status, node.lpos
    if s is None:
        print(f'{label}: no /fmu/out/vehicle_status_v1 -- is the XRCE agent up?')
        return
    armed = 'ARMED' if s.arming_state == VehicleStatus.ARMING_STATE_ARMED else 'disarmed'
    pos = 'no position'
    if p is not None:
        pos = (f'xy_valid={p.xy_valid} z_valid={p.z_valid} '
               f'dead_reckoning={p.dead_reckoning} '
               f'NED [{p.x:+.2f} {p.y:+.2f} {p.z:+.2f}]')
    print(f'{label}: mode={NAV.get(s.nav_state, s.nav_state)} {armed}  |  {pos}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--status', action='store_true', help='read-only; command nothing')
    g.add_argument('--arm', action='store_true',
                   help='engage Offboard then ARM. PROPS OFF unless you mean it.')
    g.add_argument('--disarm', action='store_true', help='disarm immediately')
    g.add_argument('--hold', action='store_true', help='leave Offboard -> Hold')
    args = ap.parse_args()

    rclpy.init()
    node = Engager()
    rc = 0
    try:
        node.spin(SETTLE)
        show(node, 'before')
        # Measure the bridge's rates only AFTER discovery has settled -- counting from
        # node start would average in the seconds before the subscriptions matched and
        # under-report a healthy 50 Hz stream by half.
        node.n_ocm = node.n_sp = 0
        t0 = time.time()
        node.spin(2.0)
        window = time.time() - t0
        hb, sp = node.n_ocm / window, node.n_sp / window

        if args.status:
            print(f'bridge heartbeat: {hb:.0f} Hz on '
                  f'/fmu/in/offboard_control_mode, {sp:.0f} Hz on '
                  '/fmu/in/trajectory_setpoint')
            return 0

        if args.disarm:
            node.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                      VehicleCommand.ARMING_ACTION_DISARM)
            a = node.await_ack(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
            print(f'disarm -> {ACK.get(a.result, a.result) if a else "NO ACK"}')
            node.spin(1.0)
            show(node, 'after')
            return 0 if a and a.result == 0 else 1

        if args.hold:
            # param3 carries the sub-mode: AUTO + LOITER == PX4's "Hold".
            node.send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                      CUSTOM, MAIN_AUTO, SUB_LOITER)
            a = node.await_ack(VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
            print(f'hold -> {ACK.get(a.result, a.result) if a else "NO ACK"}')
            node.spin(1.0)
            show(node, 'after')
            return 0

        # --- engage Offboard (with or without a following arm) ---
        print(f'bridge heartbeat: {hb:.0f} Hz on /fmu/in/offboard_control_mode, '
              f'{sp:.0f} Hz on /fmu/in/trajectory_setpoint '
              '(0 is normal outside Offboard since 2026-09-28)')
        if hb <= 2.0:
            print('REFUSING: PX4 rejects the Offboard switch unless the heartbeat is '
                  'already running at >2 Hz. Start px4_offboard_bridge.py first.')
            return 1
        p = node.lpos
        if p is None or not (p.xy_valid and p.z_valid) or p.dead_reckoning:
            print('REFUSING: EKF2 has no valid local position (or is dead-reckoning). '
                  'Offboard position control needs one -- is px4_odom_bridge running?')
            return 1

        node.send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, CUSTOM, MAIN_OFFBOARD)
        a = node.await_ack(VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
        print(f'offboard -> {ACK.get(a.result, a.result) if a else "NO ACK"}')
        node.spin(1.0)
        show(node, 'after mode')
        if node.status is None or node.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print('Offboard did not engage. With no RC receiver bound this is expected '
                  'until COM_RCL_EXCEPT bit 2 (value 4) is set -- read the note at the '
                  'top of this file before setting it.')
            return 1
        # In Offboard now, so the bridge must be sending setpoints. Without them PX4 keeps
        # flying whatever setpoint it last had -- so do not go on to arm.
        node.n_sp = 0
        t0 = time.time()
        node.spin(1.0)
        sp = node.n_sp / (time.time() - t0)
        print(f'setpoints in Offboard: {sp:.0f} Hz on /fmu/in/trajectory_setpoint')
        if sp <= 2.0:
            print('NOT ARMING: PX4 is in Offboard but the bridge is not sending setpoints. '
                  'Is px4_offboard_bridge.py receiving /fmu/out/vehicle_control_mode? '
                  'Leave Offboard (--hold) and check it.')
            return 1

        if args.arm:
            print('\n*** ARMING in 3 s -- PROPS OFF. Ctrl-C to abort. ***')
            node.spin(3.0)
            node.send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                      VehicleCommand.ARMING_ACTION_ARM)
            a = node.await_ack(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
            print(f'arm -> {ACK.get(a.result, a.result) if a else "NO ACK"}')
            node.spin(1.0)
            show(node, 'after arm')
            rc = 0 if a and a.result == 0 else 1
    except KeyboardInterrupt:
        print('\naborted')
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
