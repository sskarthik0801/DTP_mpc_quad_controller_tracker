#!/usr/bin/env python3
"""G0.1a: force uXRCE-DDS session losses to reproduce the `uxrce_dds_client` hardfault.

WHY
---
The FC has hard-faulted 22 times in `uxrce_dds_client` (pc:0x08022f60, r3:0x08020e59), the
newest 2026-09-13, across two firmware builds and long after the Tegra-DMA fix it was once
blamed on. The addresses decode to a use-after-free in NuttX's serial driver: `uart_close`
tears down RX DMA (`up_dma_shutdown`) while `uart_datareceived` is still on the buffer. It
fires when the client **closes and reopens its UART** -- i.e. on XRCE session loss, the event
most likely to happen in flight when something else goes wrong. Our board build enables RX DMA
on that UART (`CONFIG_UART7_RXDMA=y`); the candidate fix is to turn it off (G0.1b). A fix that
was never reproduced is not a fix, so this script makes the trigger happen on demand and
measures a per-cycle fault rate.

WHAT IT DOES
------------
Over the MAVLink shell (never a serial port; UDP 14569, the same path as px4_shell.py), each
cycle runs on the FC:
    uxrce_dds_client stop      # destructor -> uxr_close_serial_transport -> up_dma_shutdown
    uxrce_dds_client start -t serial -d <dev> -b p:SER_TEL1_BAUD

`stop` closes the UART and `start` reopens it -- exactly the close/reopen the decode points at.
A hard fault RESETS the FC, which the script detects as `time_boot_ms` going backwards, and
counts as a fault. After a reset the client is restarted automatically by the FC's rcS, so the
run continues on its own. Nothing here can arm anything, and it refuses to start if the vehicle
is armed.

    source /opt/ros/humble/setup.bash    # any shell with pymavlink works
    python3 scripts/px4/xrce_fault_repro.py                 # 300 cycles or 5 faults
    python3 scripts/px4/xrce_fault_repro.py --cycles 500 --max-faults 5
    python3 scripts/px4/xrce_fault_repro.py --csv /tmp/g01a.csv

Run it with props OFF. When it stops, scan the card:
    python3 scripts/px4/clear_fault_logs.py            # confirms the fault count and signature

The rate p̂ = faults / cycles sets the fix's pass bar: after CONFIG_UART7_RXDMA=n, 0 faults in
max(500, ceil(3/p̂)) cycles (plan §2.2 G0.1). Read docs/xrce-agent-nx.md before touching the link.
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

CONN = 'udpin:127.0.0.1:14569'
DEV_SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
F_RESPOND = mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
F_EXCLUSIVE = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
F_MULTI = mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI
ARMED_BIT = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def connect(timeout=15):
    """Attach to the PX4 autopilot (comp 1), not one of ARK OS's other components.

    ARK's components heartbeat on the same UDP bus (comp 191/192/236, autopilot=8); taking the
    first heartbeat gives sysid 0 and a component that answers nothing (CLAUDE.md).
    """
    m = mavutil.mavlink_connection(CONN)
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 \
                and hb.get_srcComponent() == 1:
            m.target_system, m.target_component = hb.get_srcSystem(), 1
            return m, hb
    return None, None


def shell(m, cmd, settle):
    payload = (cmd + '\n').encode()
    while payload:
        chunk, payload = payload[:70], payload[70:]
        m.mav.serial_control_send(DEV_SHELL, F_RESPOND | F_EXCLUSIVE | F_MULTI,
                                  0, 0, len(chunk), list(chunk) + [0] * (70 - len(chunk)))
    out, end = [], time.time() + settle
    while time.time() < end:
        msg = m.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.4)
        if msg is None:
            m.mav.serial_control_send(DEV_SHELL, F_RESPOND | F_MULTI, 0, 0, 0, [0] * 70)
            continue
        if msg.count:
            out.append(bytes(msg.data[:msg.count]).decode(errors='replace'))
    return ''.join(out)


def boot_ms(m, timeout=3.0):
    """FC uptime in ms; resets to a small value on a reboot. None if the FC is not talking."""
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type='SYSTEM_TIME', blocking=True, timeout=0.5)
        if msg and msg.get_srcComponent() == 1:
            return msg.time_boot_ms
    return None


def wait_alive(m, timeout=60.0):
    """After a reset, wait for the FC's heartbeat to come back. Returns True if it did."""
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 \
                and hb.get_srcComponent() == 1:
            return not bool(hb.base_mode & ARMED_BIT)
    return False


def wilson_upper(faults, n, z=1.96):
    """Upper 95% bound on the per-cycle fault probability (0 faults -> a real bound, not 0)."""
    if n == 0:
        return 1.0
    p = faults / n
    denom = 1 + z * z / n
    centre = p + z * z / (2 * n)
    margin = z * math.sqrt((p * (1 - p) + z * z / (4 * n)) / n)
    return (centre + margin) / denom


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--cycles', type=int, default=300, help='max session-loss cycles (def 300)')
    ap.add_argument('--max-faults', type=int, default=5,
                    help='stop after this many FC resets (default 5); enough to estimate a rate')
    ap.add_argument('--device', default='/dev/ttyS6',
                    help='FC serial device for TELEM1 (ark_fpv: /dev/ttyS6 = UART7)')
    ap.add_argument('--baud-param', default='SER_TEL1_BAUD')
    ap.add_argument('--down-s', type=float, default=0.6,
                    help='seconds the client stays stopped before restart')
    ap.add_argument('--settle-s', type=float, default=2.5,
                    help='seconds to let the client reconnect before the reboot check')
    ap.add_argument('--csv', default=None, help='append per-cycle rows here')
    args = ap.parse_args(argv)

    m, hb = connect()
    if m is None:
        sys.exit(f'no PX4 heartbeat on {CONN} -- is mavlink-router up? (docs/mavlink-setup.md)')
    if hb.base_mode & ARMED_BIT:
        sys.exit('REFUSED: the vehicle is ARMED. This resets the FC repeatedly; disarm first.')

    start_cmd = (f'uxrce_dds_client start -t serial -d {args.device} '
                 f'-b p:{args.baud_param}')
    csv = open(args.csv, 'a') if args.csv else None
    if csv:
        csv.write('cycle,unix,boot_before_ms,boot_after_ms,fault,note\n')

    print(f'FC sys={m.target_system} comp=1, disarmed. Forcing session losses on {args.device}.')
    print(f'up to {args.cycles} cycles, stop at {args.max_faults} faults. '
          'Props off. Ctrl-C to stop.\n')
    shell(m, 'uxrce_dds_client status', 1.0)   # flush

    cycles = faults = 0
    fault_cycles = []
    t0 = time.time()
    try:
        for i in range(1, args.cycles + 1):
            before = boot_ms(m)
            shell(m, 'uxrce_dds_client stop', 0.8)
            time.sleep(args.down_s)
            shell(m, start_cmd, 1.2)
            time.sleep(args.settle_s)
            after = boot_ms(m)
            cycles += 1

            fault = note = ''
            if after is None:
                # FC went quiet: almost certainly a reset mid-cycle. Wait for it to return.
                alive = wait_alive(m, 60.0)
                after2 = boot_ms(m)
                fault = 'FAULT'
                note = 'FC silent then %s (boot_ms=%s)' % (
                    'recovered' if alive else 'DID NOT RECOVER', after2)
                faults += 1
                fault_cycles.append(i)
            elif before is not None and after < before:
                fault = 'FAULT'
                note = 'boot_ms %d -> %d (reset)' % (before, after)
                faults += 1
                fault_cycles.append(i)

            elapsed = time.time() - t0
            rate = elapsed / cycles
            if fault or i % 20 == 0:
                print('cycle %4d/%d faults=%d %-6s %s [%.0fs, %.1fs/cyc]'
                      % (i, args.cycles, faults, fault, note, elapsed, rate))
            if csv:
                csv.write('%d,%.1f,%s,%s,%d,%s\n'
                          % (i, time.time(), before, after, 1 if fault else 0, note))
                csv.flush()
            if faults >= args.max_faults:
                print(f'\nreached {args.max_faults} faults; stopping early.')
                break
    except KeyboardInterrupt:
        print('\ninterrupted.')
    finally:
        if csv:
            csv.close()

    print('\n=== G0.1a result ===')
    print(f'cycles run:     {cycles}')
    print(f'faults (resets): {faults}   at cycles {fault_cycles}')
    if cycles:
        p = faults / cycles
        hi = wilson_upper(faults, cycles)
        print(f'per-cycle rate p̂: {p:.4f}   (95% upper bound {hi:.4f})')
        if faults:
            need = max(500, math.ceil(3 / p))
            print(f'the fix must survive >= {need} clean cycles (plan §2.2 G0.1)')
        else:
            print(f'0 faults in {cycles} cycles: the trigger is not this, or the rate is below '
                  f'~1/{cycles}. Consider more --cycles, or the 14 h soak decides.')
    print('\nNow scan the card to confirm the count and signature:')
    print('  python3 scripts/px4/clear_fault_logs.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
