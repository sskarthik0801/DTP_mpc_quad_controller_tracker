#!/usr/bin/env python3
"""Reboot the flight controller (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN).

    python3 scripts/px4/fc_reboot.py           # dry run -- says what it would do
    python3 scripts/px4/fc_reboot.py --yes     # actually reboot

Over MAVLink UDP 14569 like the rest of scripts/px4/, never a serial port and
never the FC's USB tty -- a second reader corrupts the stream (CLAUDE.md).

It refuses while the vehicle is ARMED, and while the FC reports it is not
landed.  PX4's commander refuses a reboot when armed as well, but a tool whose
whole job is to reset the flight controller must not rely on the thing it is
resetting to say no.

`--yes` instead of an interactive prompt on purpose: this is also run by the
web GCS, whose children get stdin=DEVNULL, so a prompt would hang until the
timeout with no clue why.  Same reasoning as set_params.py's --apply.

Three things to expect afterwards, none of which is a fault:

  * every /fmu/* topic disappears and comes back.  The uXRCE-DDS client
    restarts with the FC and ARK's dds-agent.service re-establishes the
    session; ROS 2 publishers rematch on their own.  If a bridge was running,
    check it is still publishing rather than assuming it is.
  * EKF2 restarts with no absolute position source until px4_odom_bridge is
    publishing again, so `local position: 0` for a few seconds is expected.
  * this reboot is exactly the close-and-reopen of the client's UART that sits
    behind the NuttX serial use-after-free which has hardfaulted THIS FC seven
    times (firmware/faults/README.md).  A fresh dump on the card afterwards is
    that known upstream bug, not something you did -- archive it with
    scripts/px4/clear_fault_logs.py --clear.
"""
import sys
import time

from pymavlink import mavutil

CONN = 'udpin:127.0.0.1:14569'
REBOOT = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
ARMED_BIT = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
RESULT = {v: k[len('MAV_RESULT_'):] for k, v in vars(mavutil.mavlink).items()
          if k.startswith('MAV_RESULT_') and isinstance(v, int)}
LANDED = {v: k[len('MAV_LANDED_STATE_'):] for k, v in vars(mavutil.mavlink).items()
          if k.startswith('MAV_LANDED_STATE_') and isinstance(v, int)}

GONE_AFTER = 3.0        # no heartbeat for this long -> the FC really went down
BACK_TIMEOUT = 40.0     # ARK FPV boots in ~5 s; this is slack, not an estimate


def observe(m, seconds):
    """Collect heartbeat / landed-state / boot-time for a moment.

    ARK OS components (comp 191/192/236) heartbeat on this same shared UDP bus,
    so the autopilot has to be picked out explicitly -- take the first match and
    you get sysid 0 and a tool that quietly talks to the wrong thing (CLAUDE.md).
    """
    seen = {'hb': None, 'landed': None, 'boot_ms': None}
    end = time.time() + seconds
    while time.time() < end:
        msg = m.recv_match(type=['HEARTBEAT', 'EXTENDED_SYS_STATE', 'SYSTEM_TIME'],
                           blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_srcComponent() != 1:
            continue
        kind = msg.get_type()
        if kind == 'HEARTBEAT':
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                seen['hb'] = msg
        elif kind == 'EXTENDED_SYS_STATE':
            seen['landed'] = msg.landed_state
        elif kind == 'SYSTEM_TIME':
            seen['boot_ms'] = msg.time_boot_ms
    return seen


def main():
    go = '--yes' in sys.argv[1:]

    m = mavutil.mavlink_connection(CONN)
    print(f'connecting on {CONN} ...')
    # Long enough to catch EXTENDED_SYS_STATE and SYSTEM_TIME, which are slower
    # than the 1 Hz heartbeat; without them we simply have less to report.
    before = observe(m, 4.0)
    hb = before['hb']
    if hb is None:
        sys.exit('no PX4 heartbeat on the bus -- is mavlink-router up? '
                 '(docs/mavlink-setup.md)')
    m.target_system, m.target_component = hb.get_srcSystem(), 1
    armed = bool(hb.base_mode & ARMED_BIT)
    landed = before['landed']
    print(f'autopilot sys={m.target_system} comp=1  '
          f'{"ARMED" if armed else "disarmed"}  '
          f'landed_state={LANDED.get(landed, "unknown")}  '
          f'time_boot_ms={before["boot_ms"]}')

    if armed:
        sys.exit('REFUSED: the vehicle is ARMED. Disarm before rebooting the FC.')
    if landed == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
        sys.exit('REFUSED: the FC reports the vehicle is IN AIR.')

    if not go:
        print('\nDRY RUN -- nothing sent.')
        print('This would send MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=1 '
              '(reboot autopilot).')
        print('Re-run with --yes to actually reboot.')
        return 0

    print('\nsending MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (param1=1, reboot) ...')
    m.mav.command_long_send(m.target_system, m.target_component,
                            REBOOT, 0, 1, 0, 0, 0, 0, 0, 0)

    # The ACK is best-effort: PX4 answers and then reboots, so a lost ACK is
    # normal and is NOT evidence the command failed. The heartbeat going away
    # is the evidence.
    ack = None
    end = time.time() + 3.0
    while time.time() < end:
        msg = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
        if msg is not None and msg.command == REBOOT:
            ack = msg
            break
    if ack is None:
        print('no COMMAND_ACK (normal -- the FC may already be rebooting)')
    else:
        name = RESULT.get(ack.result, str(ack.result))
        print(f'COMMAND_ACK: {name}')
        if ack.result not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                              mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
            sys.exit(f'REFUSED by PX4: {name}')

    # Down, then up. Both halves matter: "it stopped answering" alone could be
    # a dead link, and "it is answering" alone could be that it never rebooted.
    print(f'waiting for the link to drop (no heartbeat for {GONE_AFTER:.0f}s) ...')
    last, dropped = time.time(), None
    end = time.time() + 15.0
    while time.time() < end:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg is not None and msg.get_srcComponent() == 1 \
                and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            last = time.time()
        if time.time() - last > GONE_AFTER:
            dropped = last
            break
    if dropped is None:
        print('WARNING: the autopilot never stopped heartbeating. It may not '
              'have rebooted -- check `uptime` on the PX4 shell.')
        return 1
    print('link dropped; waiting for the FC to come back ...')

    end = time.time() + BACK_TIMEOUT
    while time.time() < end:
        after = observe(m, 1.0)
        if after['hb'] is not None:
            took = time.time() - dropped
            boot = after['boot_ms']
            print(f'FC is back after {took:.1f}s  time_boot_ms={boot}')
            if boot is not None and before['boot_ms'] is not None \
                    and boot < before['boot_ms']:
                print('boot time went backwards -- confirmed reboot.')
            print('\nReboot complete. Expect /fmu/* to reappear as the XRCE '
                  'session re-forms; then check for a new hardfault dump:')
            print('    python3 scripts/px4/clear_fault_logs.py')
            return 0
    print(f'ERROR: no heartbeat {BACK_TIMEOUT:.0f}s after the reboot. '
          'Check power and the USB link.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
