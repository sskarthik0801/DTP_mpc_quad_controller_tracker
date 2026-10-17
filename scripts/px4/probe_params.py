#!/usr/bin/env python3
"""Read PX4 params over ARK OS's mavlink-router companion_aux UDP endpoint.
Decodes INT32 params correctly: PX4 ships them bit-packed in the float
`param_value` field, so a raw float read shows denormal garbage."""
import struct, sys, time
from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else "udpin:127.0.0.1:14569"

GROUPS = {
    "uXRCE-DDS (M2/M3 transport)": [
        ("UXRCE_DDS_CFG", "0=disabled; must point at the UART carrying XRCE"),
        ("XRCE_DDS_0_CFG", "alt name on some builds"),
    ],
    "Serial / MAVLink": [
        ("MAV_0_CONFIG", "101=TELEM1 102=TELEM2"),
        ("MAV_1_CONFIG", ""),
        ("MAV_2_CONFIG", ""),
        ("SER_TEL1_BAUD", "agent runs at 3000000"),
        ("SER_TEL2_BAUD", ""),
    ],
    "EKF2 external vision (M2)": [
        ("EKF2_EV_CTRL", "bitmask: 1=horiz pos 2=vert pos 4=vel 8=yaw"),
        ("EKF2_HGT_REF", "0=Baro 1=GPS 2=Range 3=Vision"),
        ("EKF2_GPS_CTRL", "0=GPS aiding off (needed indoors/canopy)"),
        ("EKF2_EV_DELAY", "ms"),
    ],
    "Offboard safety (M3)": [
        ("COM_RCL_EXCEPT", "bit2(4)=allow offboard without RC"),
        ("COM_OBL_RC_ACT", "action when offboard link lost"),
        ("COM_OF_LOSS_T", "offboard loss timeout (s)"),
    ],
    "Airframe": [("SYS_AUTOSTART", ""), ("MAV_TYPE", "")],
}

TYPE_NAMES = {v: k for k, v in vars(mavutil.mavlink).items()
              if k.startswith("MAV_PARAM_TYPE_")}


def decode(pv):
    """Return python value honouring param_type (PX4 bit-packs ints in a float)."""
    t = pv.param_type
    raw = pv.param_value
    int_types = {
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32, mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    }
    if t in int_types:
        return struct.unpack("<i", struct.pack("<f", raw))[0]
    return round(raw, 6)


m = mavutil.mavlink_connection(CONN)
print(f"connecting on {CONN} ...")
if m.wait_heartbeat(timeout=15) is None:
    sys.exit("ERROR: no heartbeat")
print(f"FC sysid={m.target_system} compid={m.target_component}\n")

found, ptypes = {}, {}
names = [n for g in GROUPS.values() for n, _ in g]
for name in names:
    for attempt in range(2):
        m.mav.param_request_read_send(
            m.target_system, m.target_component, name.encode(), -1)
        end = time.time() + 1.0
        got = False
        while time.time() < end:
            pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if pv is None:
                continue
            pid = pv.param_id
            if isinstance(pid, bytes):
                pid = pid.decode(errors="ignore")
            pid = pid.rstrip("\x00")
            found[pid] = decode(pv)
            ptypes[pid] = pv.param_type
            if pid == name:
                got = True
                break
        if got:
            break

for group, entries in GROUPS.items():
    print(f"--- {group} ---")
    for name, note in entries:
        if name in found:
            tn = TYPE_NAMES.get(ptypes[name], "?").replace("MAV_PARAM_TYPE_", "")
            note = f"   # {note}" if note else ""
            print(f"  {name:<15} = {str(found[name]):<10} [{tn}]{note}")
        else:
            print(f"  {name:<15} = <not present>")
    print()
