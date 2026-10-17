#!/usr/bin/env python3
"""Why is PX4 stuck in ARMING_STATE_INIT? Capture health/battery/preflight info.
Read-only apart from requesting SYS_STATUS to be streamed."""
import time
from pymavlink import mavutil

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
print(f"autopilot sys={m.target_system} comp=1\n")

# ask for SYS_STATUS + BATTERY_STATUS explicitly
for msgid in (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
              mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msgid, 500000, 0, 0, 0, 0, 0)

got = {}
texts = []
end = time.time() + 15
while time.time() < end:
    msg = m.recv_match(
        type=["SYS_STATUS", "BATTERY_STATUS", "STATUSTEXT", "EXTENDED_SYS_STATE",
              "ESTIMATOR_STATUS", "GPS_RAW_INT"],
        blocking=True, timeout=1.0)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "STATUSTEXT":
        s = msg.text
        if isinstance(s, bytes):
            s = s.decode(errors="ignore")
        s = s.rstrip("\x00")
        if s not in texts:
            texts.append(s)
    else:
        got[t] = msg

ss = got.get("SYS_STATUS")
if ss:
    present, health = ss.onboard_control_sensors_present, ss.onboard_control_sensors_health
    print(f"battery      : {ss.voltage_battery} mV, {ss.battery_remaining}% remaining")
    print(f"sensors      : present=0x{present:08x} health=0x{health:08x}")
    bad = present & ~health
    if bad:
        print("\nUNHEALTHY / failing sensors:")
        for name, bit in sorted(vars(mavutil.mavlink).items()):
            if name.startswith("MAV_SYS_STATUS_SENSOR_") and isinstance(bit, int) and bad & bit:
                print(f"  - {name.replace('MAV_SYS_STATUS_SENSOR_', '')}")
    else:
        print("all present sensors healthy")
else:
    print("no SYS_STATUS received")

bs = got.get("BATTERY_STATUS")
if bs:
    v = [x for x in bs.voltages if x not in (0, 65535)]
    print(f"\nBATTERY_STATUS: cells={v} mV  remaining={bs.battery_remaining}%")

gps = got.get("GPS_RAW_INT")
if gps:
    print(f"\nGPS: fix_type={gps.fix_type} sats={gps.satellites_visible}")

es = got.get("ESTIMATOR_STATUS")
if es:
    print(f"\nESTIMATOR flags=0x{es.flags:08x}")

if texts:
    print("\nSTATUSTEXT:")
    for s in texts:
        print(f"  {s}")
else:
    print("\n(no STATUSTEXT captured)")
