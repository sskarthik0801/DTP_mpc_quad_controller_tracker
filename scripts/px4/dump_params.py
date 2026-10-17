#!/usr/bin/env python3
"""Dump every PX4 parameter to a QGC-compatible .params file.

    python3 scripts/px4/dump_params.py firmware/px4-params-YYYY-MM-DD.params

Read-only: it never writes to the FC. Use it before ANY parameter change or
reflash -- firmware/README.md calls for a full dump as the restore point, and
probe_params.py (a fixed shortlist) is not enough.

Output format matches QGC's "Save to file", so the result can be loaded straight
back via QGC -> Parameters -> Tools -> Load from file:

    MAV_ID<TAB>COMPONENT_ID<TAB>PARAM_NAME<TAB>VALUE<TAB>TYPE

Goes over ARK's mavlink-router companion_aux endpoint (UDP 14569), never a
serial port -- a second opener corrupts the stream.
"""
import os
import struct
import sys
import time

from pymavlink import mavutil

CONN = os.environ.get("PX4_CONN", "udpin:127.0.0.1:14569")
INT_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32, mavutil.mavlink.MAV_PARAM_TYPE_INT32,
}


def decode(pv):
    """PX4 bit-packs INT32 params into MAVLink's float param_value field."""
    if pv.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", pv.param_value))[0]
    return pv.param_value


def pname(pv):
    pid = pv.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="ignore")
    return pid.rstrip("\x00")


def connect():
    m = mavutil.mavlink_connection(CONN)
    print(f"connecting on {CONN} ...")
    # ARK OS components share this bus and also emit HEARTBEATs; wait_heartbeat()
    # grabs one of those and yields sysid 0, silently breaking every read.
    end = time.time() + 20
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if hb is None:
            continue
        if (hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                and hb.get_srcComponent() == 1):
            m.target_system, m.target_component = hb.get_srcSystem(), 1
            return m
    sys.exit("ERROR: no PX4 autopilot heartbeat in 20s")


def main():
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} <output.params>")
    out = sys.argv[1]

    m = connect()
    print(f"FC sysid={m.target_system} comp=1\n")

    params, total = {}, None
    m.mav.param_request_list_send(m.target_system, m.target_component)
    last = time.time()
    while time.time() - last < 12:
        pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if pv is None:
            continue
        last = time.time()
        total = pv.param_count
        params[pname(pv)] = pv
        if total and len(params) >= total:
            break
        if len(params) % 200 == 0:
            print(f"  {len(params)}/{total or '?'} ...")

    # The broadcast dump drops messages; chase stragglers by index.
    if total and len(params) < total:
        have = {pv.param_index for pv in params.values()}
        missing = [i for i in range(total) if i not in have]
        print(f"  {len(missing)} params missing from the broadcast, re-requesting")
        for idx in missing:
            for _ in range(3):
                m.mav.param_request_read_send(
                    m.target_system, m.target_component, b"", idx)
                pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
                if pv is not None:
                    params[pname(pv)] = pv
                    break

    if not params:
        sys.exit("ERROR: no params received")
    if total and len(params) != total:
        print(f"  WARNING: got {len(params)} of {total} params")

    stamp = time.strftime("%Y-%m-%d %H:%M:%S %Z")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        f.write("# PX4 parameter backup taken over MAVLink\n")
        f.write(f"# {stamp}  sysid={m.target_system}\n")
        f.write("# MAV_ID COMPONENT_ID PARAM_NAME VALUE TYPE\n")
        for name in sorted(params):
            pv = params[name]
            val = decode(pv)
            f.write(f"{m.target_system}\t1\t{name}\t{val}\t{pv.param_type}\n")

    print(f"\nwrote {len(params)} params -> {out}")


if __name__ == "__main__":
    main()
