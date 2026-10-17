#!/usr/bin/env python3
"""Run read-only status commands on the PX4 NuttX shell via MAVLink
SERIAL_CONTROL (the same mechanism as QGC's MAVLink Console)."""
import sys, time
from pymavlink import mavutil

CMDS = sys.argv[1:] or [
    "uxrce_dds_client status",
    "mavlink status",
]

m = mavutil.mavlink_connection("udpin:127.0.0.1:14569")

hb = None
end = time.time() + 15
while time.time() < end:
    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    if msg and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 \
            and msg.get_srcComponent() == 1:
        hb = msg
        break
if hb is None:
    raise SystemExit("no PX4 heartbeat")
m.target_system, m.target_component = hb.get_srcSystem(), 1
print(f"shell on sys={m.target_system} comp=1\n")

DEV_SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
F_RESPOND = mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
F_EXCLUSIVE = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
F_MULTI = mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI


def send(text):
    payload = text.encode()
    while payload:
        chunk, payload = payload[:70], payload[70:]
        buf = list(chunk) + [0] * (70 - len(chunk))
        m.mav.serial_control_send(
            DEV_SHELL, F_RESPOND | F_EXCLUSIVE | F_MULTI,
            0, 0, len(chunk), buf)


def drain(seconds):
    out, end = [], time.time() + seconds
    while time.time() < end:
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.4)
        if msg is None:
            # nudge for more output
            m.mav.serial_control_send(DEV_SHELL, F_RESPOND | F_MULTI, 0, 0, 0, [0] * 70)
            continue
        if msg.count:
            out.append(bytes(msg.data[:msg.count]).decode(errors="replace"))
    return "".join(out)


drain(1.0)                      # clear anything pending
for cmd in CMDS:
    print(f"=========== {cmd} ===========")
    send(cmd + "\n")
    text = drain(4.0)
    print(text.strip() or "(no output)")
    print()
