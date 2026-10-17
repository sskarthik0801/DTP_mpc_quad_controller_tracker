#!/usr/bin/env python3
"""G0.1: provoke XRCE session loss on purpose and count FC hardfaults per cycle.

The `uxrce_dds_client` hardfault (22 of the 24 dumps in firmware/faults/) is a use-after-free:
`uart_close()` tears down RX DMA while `uart_datareceived()` is still running on it. The client
closes its UART exactly once per session loss -- 3 missed pings, then init() -> deinit() ->
close() (uxrce_dds_client.cpp:116,180) -- and every dump that can be checked against the Jetson's
journal lands 3-4 s after a Jetson SHUTDOWN, or on a power cycle. That is: agent gone, 3 pings
missed, close, while the Jetson's UART line is doing something as it powers down.

One cycle here reproduces that without rebooting the Jetson:
    1. dds-agent running, client connected (checked on the FC)
    2. stop dds-agent, then drive RX activity onto /dev/ttyTHS1 (--rx)
    3. wait until the FC client reports disconnected -- it has closed and reopened the UART
       under that RX activity -- plus a margin
    4. stop the RX activity, start dds-agent, wait for connected
A hardfault resets the FC; that is detected over MAVLink (SYSTEM_TIME going backwards, or the
heartbeat vanishing), the card is re-listed, and the new dump is recorded.

    python3 scripts/px4/xrce_cycle_test.py --cycles 300 --max-faults 5          # dry run
    python3 scripts/px4/xrce_cycle_test.py --cycles 300 --max-faults 5 --yes    # do it

    --rx random   (default) random bytes at 3 Mbaud: RX DMA continuously busy during the close
    --rx break    repeated UART breaks: the line held low, like a powered-down transmitter
    --rx none     control: plain agent stop/start, the line idle

BENCH ONLY. Refuses to start, and stops, if the FC reports ARMED or IN AIR. Each fault is an FC
reset. Stops dds-agent for most of the run, so /fmu/* is absent while it runs, and ALWAYS starts
it again on exit. Needs passwordless sudo for systemctl, pymavlink, and nothing else on UDP 14569.
Writes one CSV row per event (default firmware/g01/cycles-<time>.csv).
"""
import argparse
import csv
import datetime
import os
import random
import subprocess
import sys
import termios
import threading
import time

from pymavlink import mavutil

CONN = 'udpin:127.0.0.1:14569'
TTY = '/dev/ttyTHS1'
BAUD = 3000000
AGENT = 'dds-agent.service'
SD = '/fs/microsd'
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DEV_SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
F_RESPOND = mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
F_EXCLUSIVE = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
F_MULTI = mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI
ARMED_BIT = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR

HB_GONE_S = 3.0          # no FC heartbeat this long = the FC is down
FC_BACK_TIMEOUT_S = 90.0
CONNECT_TIMEOUT_S = 30.0
DISCONNECT_TIMEOUT_S = 15.0
AFTER_DISCONNECT_S = 1.5  # keep the RX activity going past the close/reopen


class FC:
    """One MAVLink connection; every recv goes through pump(), so resets are never missed."""

    def __init__(self):
        self.m = mavutil.mavlink_connection(CONN)
        self.last_hb = 0.0
        self.boot_ms = None
        self.armed = False
        self.landed = None
        self.resets = []          # wall times at which a real reset (boot_ms backwards) was seen
        self.hb_gaps = 0          # transient heartbeat gaps that recovered -- NOT resets
        self._serial = []
        self._gone = False

    def pump(self, seconds):
        """Drain MAVLink for `seconds`, tracking heartbeat/boot/armed and detecting resets.

        A reset shows up two ways and we take either: SYSTEM_TIME.time_boot_ms jumps backwards,
        or the heartbeat goes absent for > HB_GONE_S and then returns. Both are recorded once per
        event in self.resets.
        """
        end = time.time() + seconds
        while True:
            left = end - time.time()
            if left <= 0:
                return
            msg = self.m.recv_match(type=['HEARTBEAT', 'SYSTEM_TIME', 'EXTENDED_SYS_STATE'],
                                    blocking=True, timeout=min(left, 0.4))
            now = time.time()
            if msg is None:
                if self.last_hb and now - self.last_hb > HB_GONE_S and not self._gone:
                    self._gone = True
                continue
            if msg.get_srcComponent() != 1:
                continue
            kind = msg.get_type()
            if kind == 'HEARTBEAT' and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                # A heartbeat gap that RECOVERS is NOT counted as a reset. This tool does
                # blocking shell I/O (uxrce status) between pumps, so its own heartbeat
                # tracking routinely goes >3 s stale and then catches up -- that is not an
                # FC event. A real reset is unambiguous: boot_ms goes backwards (below) or a
                # crash dump appears on the card (checked at the end). Only those count.
                if self._gone:
                    self.hb_gaps += 1
                    self._gone = False
                self.last_hb = now
                self.armed = bool(msg.base_mode & ARMED_BIT)
            elif kind == 'SYSTEM_TIME':
                b = msg.time_boot_ms
                if self.boot_ms is not None and b + 2000 < self.boot_ms:
                    self.resets.append(now)   # uptime went backwards -> a real reset
                self.boot_ms = b
            elif kind == 'EXTENDED_SYS_STATE':
                self.landed = msg.landed_state

    def acquire(self, timeout=CONNECT_TIMEOUT_S):
        """Attach to the PX4 autopilot (comp 1), not one of ARK OS's other components."""
        end = time.time() + timeout
        while time.time() < end:
            hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 \
                    and hb.get_srcComponent() == 1:
                self.m.target_system, self.m.target_component = hb.get_srcSystem(), 1
                self.last_hb = time.time()
                self.armed = bool(hb.base_mode & ARMED_BIT)
                return True
        return False

    def shell(self, cmd, settle=3.0):
        """Run one NuttX shell command over MAVLink (independent of the XRCE/agent link)."""
        payload = (cmd + '\n').encode()
        while payload:
            chunk, payload = payload[:70], payload[70:]
            self.m.mav.serial_control_send(
                DEV_SHELL, F_RESPOND | F_EXCLUSIVE | F_MULTI,
                0, 0, len(chunk), list(chunk) + [0] * (70 - len(chunk)))
        out, end = [], time.time() + settle
        while time.time() < end:
            msg = self.m.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.4)
            if msg is None:
                self.m.mav.serial_control_send(DEV_SHELL, F_RESPOND | F_MULTI, 0, 0, 0, [0] * 70)
                continue
            if msg.count:
                out.append(bytes(msg.data[:msg.count]).decode(errors='replace'))
        return ''.join(out)

    def xrce_connected(self):
        """True/False from the client's own status, or None if the shell did not answer."""
        text = self.shell('uxrce_dds_client status', 2.0)
        if 'connected' not in text:
            return None
        # "Running, connected" vs "Running, disconnected"
        return 'disconnected' not in text

    def sd_faults(self):
        text = self.shell(f'ls {SD}', 2.0)
        return sorted({n.strip() for n in text.splitlines()
                       if n.strip().startswith('fault_')
                       and (n.strip().endswith('.log') or n.strip().endswith('.txt'))})


# --------------------------------------------------------------------------- RX activity


class RxDriver:
    """Drives activity onto the FC's RX line while its UART is closed and reopened.

    The use-after-free needs `uart_datareceived` to be live on the DMA buffer at the moment
    `up_dma_shutdown` frees it, so the point is to keep bytes (or line transitions) arriving
    across the close. Runs in its own thread; stop() joins it and closes the port.
    """

    def __init__(self, mode):
        self.mode = mode
        self.fd = None
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self.mode == 'none':
            return
        # dds-agent has been stopped, so the port is free. stty sets the raw 3 Mbaud line;
        # termios lacks a B3000000 constant on this build, so configure it with stty.
        subprocess.run(['stty', '-F', TTY, str(BAUD), 'raw', '-echo', '-echoe', '-echok',
                        '-crtscts'], check=True)
        self.fd = os.open(TTY, os.O_RDWR | os.O_NOCTTY)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        blob = bytes(random.getrandbits(8) for _ in range(4096))
        while not self._stop.is_set():
            try:
                if self.mode == 'random':
                    os.write(self.fd, blob)
                    time.sleep(0.001)
                elif self.mode == 'break':
                    termios.tcsendbreak(self.fd, 0)
                    time.sleep(0.02)
            except OSError:
                return

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.fd is not None:
            try:
                termios.tcflush(self.fd, termios.TCIOFLUSH)
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None


# --------------------------------------------------------------------------- systemctl


def sctl(action):
    """systemctl <action> dds-agent via passwordless sudo -n. Returns True on success."""
    r = subprocess.run(['sudo', '-n', 'systemctl', action, AGENT],
                       capture_output=True, text=True)
    return r.returncode == 0


def agent_active():
    r = subprocess.run(['systemctl', 'is-active', '--quiet', AGENT])
    return r.returncode == 0


# --------------------------------------------------------------------------- main


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--cycles', type=int, default=300, help='max session-loss cycles (def 300)')
    ap.add_argument('--max-faults', type=int, default=5,
                    help='stop after this many resets (def 5)')
    ap.add_argument('--rx', choices=('random', 'break', 'none'), default='random',
                    help='what to drive onto the RX line during the close (see the docstring)')
    ap.add_argument('--yes', action='store_true',
                    help='actually run; without it this is a dry run')
    ap.add_argument('--csv', default=None,
                    help='CSV path (default firmware/g01/cycles-<time>.csv)')
    args = ap.parse_args(argv)

    if not args.csv:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        args.csv = os.path.join(REPO, 'firmware', 'g01', f'cycles-{stamp}.csv')

    if not sctl('is-enabled') and not agent_active():
        pass  # is-enabled may print disabled; we only need start/stop to work

    print(f'G0.1 XRCE cycle test: rx={args.rx}, up to {args.cycles} cycles, '
          f'stop at {args.max_faults} faults.')
    print(f'  agent: {AGENT}   tty: {TTY} @ {BAUD}   csv: {args.csv}')
    if not args.yes:
        print('\nDRY RUN -- nothing done. This WOULD, per cycle:')
        print(f'  1. sudo systemctl stop {AGENT}   (removes /fmu/* while the run proceeds)')
        print(f'  2. drive "{args.rx}" onto {TTY} until the FC client disconnects (+%.1fs)'
              % AFTER_DISCONNECT_S)
        print(f'  3. stop the activity, sudo systemctl start {AGENT}, wait for reconnect')
        print('  Each fault RESETS the FC. Bench only, props off. Re-run with --yes.')
        return 0

    # passwordless sudo is a hard requirement; fail loudly rather than prompt into a void
    if subprocess.run(['sudo', '-n', 'true']).returncode != 0:
        sys.exit('need passwordless sudo for systemctl (sudo -n failed). See CLAUDE.md.')

    fc = FC()
    if not fc.acquire():
        sys.exit(f'no PX4 heartbeat on {CONN} -- is mavlink-router up? (docs/mavlink-setup.md)')
    if fc.armed or fc.landed == IN_AIR:
        sys.exit('REFUSED: the FC reports ARMED or IN AIR. This resets the FC; bench only.')

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    faults = 0
    dumps0 = fc.sd_faults()
    t0 = time.time()
    fault_cycles = []
    with open(args.csv, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['cycle', 'unix', 'boot_before_ms', 'boot_after_ms',
                    'disconnected', 'reset', 'note'])
        rx = None
        try:
            if not agent_active() and not sctl('start'):
                sys.exit(f'could not start {AGENT} to establish the baseline')
            fc.pump(2.0)
            for i in range(1, args.cycles + 1):
                if fc.armed or fc.landed == IN_AIR:
                    print('vehicle became ARMED/IN AIR -- stopping.')
                    break
                boot_before = fc.boot_ms
                n_reset0 = len(fc.resets)

                sctl('stop')
                rx = RxDriver(args.rx)
                rx.start()

                # wait for the client to notice the agent is gone and drop the session
                disconnected = False
                end = time.time() + DISCONNECT_TIMEOUT_S
                while time.time() < end:
                    fc.pump(0.5)
                    state = fc.xrce_connected()
                    if state is False:
                        disconnected = True
                        break
                    if len(fc.resets) > n_reset0:      # crashed before it even reported
                        break
                fc.pump(AFTER_DISCONNECT_S)             # stay present across the close/reopen

                rx.stop()
                rx = None
                sctl('start')

                # let it reconnect (or come back from a reset)
                end = time.time() + CONNECT_TIMEOUT_S
                while time.time() < end:
                    fc.pump(0.5)
                    if fc.xrce_connected() is True:
                        break

                reset = len(fc.resets) > n_reset0
                if reset and fc._gone:
                    fc.pump(FC_BACK_TIMEOUT_S)          # wait out the reboot if still down
                boot_after = fc.boot_ms
                note = ''
                if reset:
                    faults += 1
                    fault_cycles.append(i)
                    note = 'FC RESET (boot %s -> %s)' % (boot_before, boot_after)
                elif not disconnected:
                    note = 'client never reported disconnected'

                elapsed = time.time() - t0
                if reset or i % 10 == 0:
                    print('cycle %4d/%d  faults=%d  disc=%s  %s  [%.0fs]'
                          % (i, args.cycles, faults, disconnected, note, elapsed))
                w.writerow([i, '%.1f' % time.time(), boot_before, boot_after,
                            int(disconnected), int(reset), note])
                fh.flush()
                if faults >= args.max_faults:
                    print(f'\nreached {args.max_faults} faults; stopping early.')
                    break
        except KeyboardInterrupt:
            print('\ninterrupted.')
        finally:
            if rx is not None:
                rx.stop()
            # ALWAYS leave the agent running, whatever happened.
            if not agent_active():
                sctl('start')
            print('dds-agent restarted:', agent_active())

    cycles = i
    dumps1 = fc.sd_faults()
    new_dumps = [d for d in dumps1 if d not in dumps0]
    print('\n=== G0.1 cross-confirm result (rx=%s) ===' % args.rx)
    print(f'cycles run:      {cycles}')
    print(f'faults (resets): {faults}   at cycles {fault_cycles}')
    print(f'new SD dumps:    {len(new_dumps)}  {new_dumps}')
    print(f'hb gaps (not faults): {fc.hb_gaps}   (transient; tool does blocking shell I/O)')
    if cycles:
        p = faults / cycles
        print(f'per-cycle rate:  {p:.4f}')
    print(f'CSV: {args.csv}')
    print('\nScan/scrape the card: python3 scripts/px4/clear_fault_logs.py [--clear]')
    return 0


if __name__ == '__main__':
    sys.exit(main())
