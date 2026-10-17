#!/bin/bash
# Test a physical UART wire to the ARK FPV FC before starting MAVROS.
# Configures the port, listens for a fixed window, and counts MAVLink
# HEARTBEAT messages (msgid 0, sent by ArduPilot at 1Hz on every link).
#
# Usage: 01_test_uart.sh [port] [baud] [seconds]
#   default port    /dev/ttyTHS1
#   default baud    921600
#   default seconds 10

set -euo pipefail

PORT="${1:-/dev/ttyTHS1}"
BAUD="${2:-57600}"   # FC is wired via the VTX connector (SERIAL2, pins 3/4), configured for 57600 8N1 in QGC
DURATION="${3:-10}"

if [ ! -e "$PORT" ]; then
    echo "ERROR: $PORT does not exist."
    echo "  -> On Jetson, run: sudo /opt/nvidia/jetson-io/jetson-io.py to enable the UART pinmux."
    exit 1
fi

if ! groups | grep -qw dialout; then
    echo "WARNING: current user is not in the 'dialout' group."
    echo "  -> sudo usermod -aG dialout \$USER   then log out/in, or this will fail with permission denied."
fi

# fail fast instead of hanging if something else already has the port open
if command -v fuser >/dev/null 2>&1 && fuser "$PORT" >/dev/null 2>&1; then
    echo "WARNING: $PORT is already open by another process:"
    fuser -v "$PORT" || true
    echo "  -> kill that process (e.g. an old screen/minicom/mavros session) first."
fi

echo "=== UART test: $PORT @ ${BAUD} baud for ${DURATION}s ==="

python3 - "$PORT" "$BAUD" "$DURATION" <<'PYEOF'
import os
import select
import sys
import termios
import time

port, baud_str, duration_str = sys.argv[1], sys.argv[2], sys.argv[3]
baud = int(baud_str)
duration = float(duration_str)

baud_const_name = f"B{baud}"
if not hasattr(termios, baud_const_name):
    sys.exit(f"ERROR: termios has no constant {baud_const_name} (unsupported baud rate)")
baud_const = getattr(termios, baud_const_name)

fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
try:
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

    # raw mode: no echo, no line editing, no signal chars, 8N1, ignore modem lines
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = (cflag | termios.CLOCAL | termios.CREAD) & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    cflag |= termios.CS8
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, baud_const, baud_const, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)

    total_bytes = 0
    nonzero_bytes = 0
    heartbeats = 0
    v1_frames = 0
    v2_frames = 0
    buf = bytearray()

    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        r, _, _ = select.select([fd], [], [], max(0, min(0.5, remaining)))
        if fd not in r:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        total_bytes += len(chunk)
        nonzero_bytes += sum(1 for b in chunk if b != 0)
        buf.extend(chunk)

        # scan for MAVLink v1 (0xFE) / v2 (0xFD) frames; msgid 0 == HEARTBEAT
        i = 0
        while i < len(buf):
            b0 = buf[i]
            if b0 == 0xFE and i + 6 <= len(buf):          # MAVLink v1: STX LEN SEQ SYSID COMPID MSGID
                length = buf[i + 1]
                frame_len = 6 + length + 2
                if i + frame_len <= len(buf):
                    msgid = buf[i + 5]
                    v1_frames += 1
                    if msgid == 0:
                        heartbeats += 1
                    i += frame_len
                    continue
                else:
                    break
            elif b0 == 0xFD and i + 10 <= len(buf):       # MAVLink v2: STX LEN INCOMPAT COMPAT SEQ SYSID COMPID MSGID(3)
                length = buf[i + 1]
                incompat_flags = buf[i + 2]
                signed = 13 if (incompat_flags & 0x01) else 0
                frame_len = 10 + length + 2 + signed
                if i + frame_len <= len(buf):
                    msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                    v2_frames += 1
                    if msgid == 0:
                        heartbeats += 1
                    i += frame_len
                    continue
                else:
                    break
            i += 1
        # drop everything already scanned/consumed, keep only the unparsed tail
        buf = buf[i:]

    elapsed = duration
    print(f"bytes received     : {total_bytes} ({total_bytes/elapsed:.1f} B/s)")
    print(f"non-zero bytes     : {nonzero_bytes}")
    print(f"MAVLink v1 frames  : {v1_frames}")
    print(f"MAVLink v2 frames  : {v2_frames}")
    print(f"HEARTBEAT messages : {heartbeats}  (ArduPilot sends these at ~1Hz on every link)")
    print()

    if total_bytes == 0:
        print("RESULT: NO SIGNAL — zero bytes received.")
        print("Troubleshooting (in priority order, per ARK Electronics UART guide):")
        print("  1. Swap TX/RX wires — the #1 UART debugging step, fixes most no-signal cases.")
        print("  2. Confirm baud rate matches on both ends (FC telemetry port config vs this script's arg).")
        print("  3. Check common GND between FC and Jetson.")
        print("  4. Confirm 3.3V logic levels — do not wire a 5V UART directly.")
        print("  5. At 921600 baud, long/untwisted cable runs lose data without HW flow control (CTS/RTS) — keep it short.")
        print("  6. Confirm nothing else has the port open (screen/minicom/an old mavros instance).")
        print("  7. If wiring looks right, check with a multimeter/scope for toggling voltage on the FC TX pin.")
        sys.exit(1)
    elif heartbeats == 0:
        print("RESULT: bytes arriving but no valid HEARTBEAT parsed — check baud rate match and TX/RX polarity;")
        print("garbled/mismatched-baud data still produces bytes but no valid MAVLink frames.")
        sys.exit(1)
    else:
        print("RESULT: OK — FC is alive and sending MAVLink HEARTBEATs over this UART.")
        sys.exit(0)
finally:
    os.close(fd)
PYEOF
