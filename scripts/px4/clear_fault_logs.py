#!/usr/bin/env python3
"""Archive PX4 crash dumps off the SD card, then clear them.

A hardfault log is the only evidence that the flight controller **reset itself**. On the
bench that is an annoyance; in flight it is a fall. So this tool does not just delete:
it pulls each dump into `firmware/faults/` (tracked in git), verifies the local copy,
and only then removes it from the SD card.

    python3 scripts/px4/clear_fault_logs.py            # list what's there (read-only)
    python3 scripts/px4/clear_fault_logs.py --clear    # archive to firmware/faults/, then delete
    python3 scripts/px4/clear_fault_logs.py --clear --no-archive   # just delete (asks first)

Two different things get called "the crash dump", and they clear differently:

1. **The uncommitted fault in BBSRAM.** The STM32H7 keeps the fault state in
   battery-backed SRAM across the reset. PX4's `rcS` runs `hardfault_log check` at every
   boot and, if one is pending, `hardfault_log commit` writes it to the SD card as
   `/fs/microsd/fault_<date>.log` and rearms. `hardfault_log reset` clears the *reboot
   counter* — the thing that makes PX4 keep complaining at boot after a crash.
   This script runs `check`, and commits a pending fault before archiving so it is not
   lost.

2. **The committed files on the SD card** — `fault_*.log`. Ordinary files; they are what
   fills the card and what shows up in QGroundControl. `rm` clears them, which is what
   `--clear` does after archiving.

Transport: MAVLink over UDP **14569** like every other script here — the NuttX shell
(`SERIAL_CONTROL`) for `ls`/`rm`/`hardfault_log`, and MAVLink **FTP** for the downloads.
**Never open the FC serial devices directly.**
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil, mavftp

CONN = os.environ.get('PX4_CONN', 'udpin:127.0.0.1:14569')
SD = '/fs/microsd'
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ARCHIVE = os.path.join(REPO, 'firmware', 'faults')
HEADER = b'[hardfault_log]'

DEV_SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
F_RESPOND = mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
F_EXCLUSIVE = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
F_MULTI = mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI


def connect():
    """Attach to the FC, not to one of ARK OS's other components on the same bus.

    ARK's components heartbeat here too (comp 191/192/236, autopilot=8). Taking the
    first heartbeat gives sysid 0 and a component that answers nothing.
    """
    m = mavutil.mavlink_connection(CONN)
    end = time.time() + 15
    while time.time() < end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 \
                and hb.get_srcComponent() == 1:
            m.target_system, m.target_component = hb.get_srcSystem(), 1
            return m
    sys.exit(f'no PX4 heartbeat on {CONN} — is mavlink-router up?')


def shell(m, cmd, settle=4.0):
    """Run one NuttX shell command and return its output."""
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


def list_faults(m):
    """Names of fault_*.log / fault_*.txt on the SD card, oldest first."""
    text = shell(m, f'ls {SD}')
    names = []
    for line in text.splitlines():
        n = line.strip()
        if n.startswith('fault_') and (n.endswith('.log') or n.endswith('.txt')):
            names.append(n)
    return sorted(set(names))


def ftp_get(remote, local):
    """Download one file. A fresh session per file — reusing one wedges after the first."""
    m = connect()
    ftp = mavftp.MAVFTP(m, target_system=m.target_system, target_component=1)
    ftp.cmd_get([remote, local])
    r = ftp.process_ftp_reply('OpenFileRO', timeout=120)
    m.close()
    return r.error_code == 0


def archive(name):
    """Pull one dump into firmware/faults/. Returns (ok, message).

    Refuses to overwrite an existing archived copy whose content differs — that would
    destroy evidence, which is the one thing this script exists to prevent.
    """
    dest = os.path.join(ARCHIVE, name)
    tmp = dest + '.part'
    if not ftp_get(f'{SD}/{name}', tmp):
        return False, 'FTP download failed'
    try:
        with open(tmp, 'rb') as f:
            data = f.read()
        if not data.startswith(HEADER):
            return False, f'downloaded file does not start with {HEADER!r} — not a fault log'
        if os.path.exists(dest):
            with open(dest, 'rb') as f:
                if f.read() == data:
                    os.remove(tmp)
                    return True, 'already archived, identical'
            return False, 'a DIFFERENT file of this name is already archived — refusing'
        os.replace(tmp, dest)
        return True, f'archived {len(data)} bytes'
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)


def summarize(path):
    """One line of what a dump actually says — task and pc are what identify a crash."""
    try:
        with open(path, 'r', errors='replace') as f:
            head = [next(f) for _ in range(12)]
    except Exception:
        return ''
    task = pc = cfsr = '?'
    for line in head:
        if 'running task:' in line:
            task = line.split('running task:')[1].strip()
        if ' pc:' in line:
            pc = line.split(' pc:')[1].split()[0]
        if 'cfsr:' in line:
            cfsr = line.split('cfsr:')[1].split()[0]
    return f'task={task} pc={pc} cfsr={cfsr}'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--clear', action='store_true',
                    help='actually delete from the SD card (default is list-only)')
    ap.add_argument('--no-archive', action='store_true',
                    help='delete without saving to firmware/faults/ — asks for confirmation')
    ap.add_argument('--reset-counter', action='store_true',
                    help='also run `hardfault_log reset` (clears the boot-time reboot counter)')
    args = ap.parse_args()

    m = connect()
    print(f'FC sys={m.target_system} comp=1 via {CONN}\n')

    # A fault still sitting in BBSRAM has not reached the SD card yet. Commit it first,
    # or clearing the card now would silently drop the most recent crash.
    check = shell(m, 'hardfault_log check')
    if 'Fault Logged on' in check:
        print('An UNCOMMITTED hardfault is pending in BBSRAM — committing it to the SD '
              'card first so it is not lost.')
        print(shell(m, 'hardfault_log commit', settle=6.0).strip())
        print()

    names = list_faults(m)
    if not names:
        print(f'No fault_* files on {SD}. Nothing to clear.')
        if args.reset_counter:
            print(shell(m, 'hardfault_log reset').strip())
        return 0

    print(f'{len(names)} crash dump(s) on {SD}:')
    for n in names:
        local = os.path.join(ARCHIVE, n)
        mark = 'archived' if os.path.exists(local) else 'NOT ARCHIVED'
        extra = f'  {summarize(local)}' if os.path.exists(local) else ''
        print(f'  {n}   [{mark}]{extra}')

    if not args.clear:
        print('\nRead-only. Re-run with --clear to archive them into '
              f'{os.path.relpath(ARCHIVE, REPO)}/ and remove them from the card.')
        return 0

    if args.no_archive:
        print('\n--no-archive: these dumps will be DELETED WITHOUT A COPY.')
        print('A hardfault log is the only record that the FC reset itself.')
        if input('Type DELETE to confirm: ').strip() != 'DELETE':
            print('aborted')
            return 1

    print()
    failed = []
    for n in names:
        if not args.no_archive:
            ok, msg = archive(n)
            print(f'  {n}: {msg}')
            if not ok:
                failed.append(n)
                continue
        shell(m, f'rm {SD}/{n}', settle=2.0)

    left = list_faults(m)
    still = [n for n in names if n in left]
    print(f'\n{len(names) - len(still)} removed from the card, {len(still)} remaining.')
    if failed:
        print('Kept on the card because archiving failed: ' + ', '.join(failed))
    if args.reset_counter:
        print(shell(m, 'hardfault_log reset').strip())
    if not args.no_archive and not failed:
        print(f'\nArchived into {os.path.relpath(ARCHIVE, REPO)}/ — commit them:')
        print(f'  git add {os.path.relpath(ARCHIVE, REPO)}/ && git commit')
    return 1 if (failed or still) else 0


if __name__ == '__main__':
    sys.exit(main())
