#!/usr/bin/env python3
"""Phase 1: enable uXRCE-DDS transport on TELEM1. Transport only -- deliberately
does NOT touch EKF2, which must stay on its current aiding sources until
px4_odom_bridge is actually publishing vision odometry.

Refuses to write if the vehicle is armed. Verifies every write by read-back.
"""
import struct, sys, time
from pymavlink import mavutil

CONN = "udpin:127.0.0.1:14569"
DRY_RUN = "--apply" not in sys.argv

INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32

# (name, value, type, rationale)
CHANGES = [
    ("MAV_0_CONFIG",  0,       INT32, "free TELEM1: MAVLink already runs over USB via mavlink-router"),
    ("UXRCE_DDS_CFG", 101,     INT32, "start the XRCE client on TELEM1 <- THE blocker"),
    ("SER_TEL1_BAUD", 3000000, INT32, "match ARK dds-agent (MicroXRCEAgent -b 3000000)"),
]

INT_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32, INT32,
}


def decode(pv):
    if pv.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", pv.param_value))[0]
    return pv.param_value


def as_float_bits(value):
    """PX4 carries INT32 params bit-packed in the float param_value field."""
    return struct.unpack("<f", struct.pack("<i", int(value)))[0]


def read_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(
        m.target_system, m.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if pv is None:
            continue
        pid = pv.param_id
        if isinstance(pid, bytes):
            pid = pid.decode(errors="ignore")
        if pid.rstrip("\x00") == name:
            return decode(pv)
    return None


m = mavutil.mavlink_connection(CONN)
print(f"connecting on {CONN} ...")

# The UDP bus is shared with mavlink-routerd and other ARK OS components, which
# also emit HEARTBEATs. wait_heartbeat() grabs whichever arrives first, so hunt
# explicitly for the real PX4 autopilot before writing anything.
AUTOPILOT_COMP = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
hb = None
end = time.time() + 20
while time.time() < end:
    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    if msg is None:
        continue
    if (msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
            and msg.get_srcComponent() == AUTOPILOT_COMP):
        hb = msg
        break
    print(f"  (ignoring heartbeat from sys={msg.get_srcSystem()} "
          f"comp={msg.get_srcComponent()} autopilot={msg.autopilot})")

if hb is None:
    sys.exit("ERROR: no PX4 autopilot heartbeat seen in 20s")

m.target_system = hb.get_srcSystem()
m.target_component = hb.get_srcComponent()

armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
state = mavutil.mavlink.enums["MAV_STATE"][hb.system_status].name
print(f"FC sysid={m.target_system} comp={m.target_component}  "
      f"state={state}  armed={armed}")
if armed:
    sys.exit("ABORT: vehicle is ARMED -- refusing to change serial/transport params.")
if state == "MAV_STATE_UNINIT":
    # Diagnosed benign: no battery/power module (battery reads 65535 mV) and no
    # horizontal aiding source, so EKF sits in CONST_POS_MODE and commander holds
    # ARMING_STATE_INIT. Gyro/accel/mag/baro all report healthy. Params live in
    # flash and are writable regardless of arming state.
    print("  note: UNINIT is expected here (USB power, no battery, no position "
          "aiding) -- proceeding.")

print("\n--- current values ---")
before = {}
for name, want, _t, _why in CHANGES:
    before[name] = read_param(m, name)
    print(f"  {name:<15} = {before[name]}   ->  {want}")

if DRY_RUN:
    print("\n[dry run] re-run with --apply to write.")
    sys.exit(0)

print("\n--- writing ---")
ok = True
for name, want, ptype, why in CHANGES:
    if before[name] == want:
        print(f"  {name:<15} already {want}, skipping")
        continue
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), as_float_bits(want), ptype)
    time.sleep(0.35)
    got = read_param(m, name)
    mark = "OK " if got == want else "FAIL"
    if got != want:
        ok = False
    print(f"  [{mark}] {name:<15} = {got}   ({why})")

print("\n--- verify (fresh read) ---")
for name, want, _t, _why in CHANGES:
    got = read_param(m, name)
    print(f"  {name:<15} = {got}  {'OK' if got == want else 'MISMATCH'}")
    if got != want:
        ok = False

print("\nRESULT:", "all params set" if ok else "SOME WRITES FAILED")
print("NOTE: serial/transport params need an FC reboot to take effect.")
print("NOTE: EKF2 params intentionally NOT touched (phase 2, after the odom bridge exists).")
