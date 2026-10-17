#!/usr/bin/env python3
"""Download a ULog from the FC's SD card over MAVLink (LOG_REQUEST_LIST / LOG_REQUEST_DATA).

    python3 scripts/px4/fetch_ulog.py                 # list the card, fetch the newest log
    python3 scripts/px4/fetch_ulog.py --list          # list only
    python3 scripts/px4/fetch_ulog.py --id 188        # a given entry
    python3 scripts/px4/fetch_ulog.py --size 43721174 # match by byte size (from `ls -l` in px4_shell.py)

Saves to ~/px4_logs/<date>_<time>.ulg by default (-o to override). About 80 s for 44 MB.

Refuses while armed: the FC is still writing the newest log, and a copy taken mid-write is
truncated -- that is how ARK's logloader looped for a day (docs/fact-ark-logloader-loop.md).
Read the result with pyulog from ~/PX4-Autopilot/.venv.

Goes over ARK's mavlink-router companion_aux endpoint (UDP 14569), never a serial port.
"""
import argparse
import datetime
import os
import sys
import time

from pymavlink import mavutil

CONN = os.environ.get("PX4_CONN", "udpin:127.0.0.1:14569")
CHUNK = 90          # LOG_DATA payload bytes
BLOCK = 20000       # chunks per LOG_REQUEST_DATA


def connect():
    m = mavutil.mavlink_connection(CONN)
    end = time.time() + 15
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        # ARK OS components also heartbeat on this bus; only comp 1 is PX4.
        if hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 and hb.get_srcComponent() == 1:
            return m, hb
    sys.exit("no PX4 heartbeat (after an FC power cycle: sudo systemctl restart mavlink-router)")


def list_logs(m, sysid):
    m.mav.log_request_list_send(sysid, 1, 0, 0xFFFF)
    entries, t0 = {}, time.time()
    while time.time() - t0 < 10:
        e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)
        if e is None:
            continue
        entries[e.id] = e
        if e.num_logs and len(entries) >= e.num_logs:
            break
    return entries


def download(m, sysid, entry):
    buf = bytearray(entry.size)
    have = bytearray((entry.size + CHUNK - 1) // CHUNK)
    for attempt in range(30):
        missing, i = [], 0
        while i < len(have):
            if have[i]:
                i += 1
                continue
            j = i
            while j < len(have) and not have[j] and j - i < BLOCK:
                j += 1
            missing.append((i, j))
            i = j
        if not missing:
            break
        print(f"  pass {attempt}: {sum(b - a for a, b in missing)} chunks to fetch", flush=True)
        for a, b in missing:
            m.mav.log_request_data_send(sysid, 1, entry.id, a * CHUNK, (b - a) * CHUNK)
            last = time.time()
            while time.time() - last < 1.0:
                d = m.recv_match(type="LOG_DATA", blocking=True, timeout=0.5)
                if d is None or d.id != entry.id:
                    continue
                last = time.time()
                k = d.ofs // CHUNK
                if d.count:
                    buf[d.ofs:d.ofs + d.count] = bytes(d.data[:d.count])
                    have[k] = 1
                if k >= b - 1:
                    break
    m.mav.log_request_end_send(sysid, 1)
    return buf, len(have) - sum(have)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--id", type=int)
    ap.add_argument("--size", type=int)
    ap.add_argument("-o", "--out")
    args = ap.parse_args()

    m, hb = connect()
    sysid = hb.get_srcSystem()
    entries = list_logs(m, sysid)
    if not entries:
        sys.exit("the FC listed no logs")
    newest = sorted(entries.values(), key=lambda e: e.id)[-5:]
    for e in newest:
        ts = datetime.datetime.utcfromtimestamp(e.time_utc).strftime("%Y-%m-%d %H:%M:%S UTC") if e.time_utc else "no RTC"
        print(f"  id {e.id:4d}  {e.size:>11,d} B  {ts}")
    print(f"  ({len(entries)} logs on the card; newest 5 shown)")
    if args.list:
        return

    if args.id is not None:
        entry = entries.get(args.id) or sys.exit(f"no log with id {args.id}")
    elif args.size is not None:
        match = [e for e in entries.values() if e.size == args.size]
        entry = match[0] if match else sys.exit(f"no log of {args.size} B")
    else:
        entry = newest[-1]

    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED and entry.id == newest[-1].id:
        sys.exit("REFUSING: armed, and the newest log is still being written. Disarm first.")

    out = args.out
    if not out:
        stamp = datetime.datetime.utcfromtimestamp(entry.time_utc).strftime("%Y-%m-%d_%H_%M_%S") \
            if entry.time_utc else f"id{entry.id}"
        out = os.path.expanduser(f"~/px4_logs/{stamp}.ulg")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    print(f"fetching id {entry.id} ({entry.size:,d} B) -> {out}")
    t0 = time.time()
    buf, missing = download(m, sysid, entry)
    if missing:
        sys.exit(f"INCOMPLETE: {missing} chunks never arrived; nothing written")
    with open(out, "wb") as f:
        f.write(buf)
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
