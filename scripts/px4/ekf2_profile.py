#!/usr/bin/env python3
"""Switch EKF2's aiding configuration between named profiles.

    python3 scripts/px4/ekf2_profile.py --show
    python3 scripts/px4/ekf2_profile.py vision-only --apply
    python3 scripts/px4/ekf2_profile.py vision-gps  --apply
    python3 scripts/px4/ekf2_profile.py stock       --apply

Dry-run unless --apply. Refuses to write while armed. Verifies by read-back.

Why a profile switch: which sources EKF2 should fuse changes with the mission
(under canopy vs open sky), and you want to move between configurations -- and
back to stock -- without hand-editing parameters and misremembering which value
meant what.

A GPS *is* fitted on this airframe (u-blox, DGPS fix); "vision-only" refers to
GPS aiding being switched off by configuration, NOT to missing hardware, and
NOT to vision being the sole aiding source -- optical flow and a downward ToF
are fitted and fused too. See the vision-only notes.

EKF2_EV_CTRL is a bitmask: 1=horiz pos, 2=vert pos, 4=velocity, 8=yaw.
EKF2_HGT_REF: 0=Baro, 1=GPS, 2=Range, 3=Vision.
EKF2_GPS_CTRL is a bitmask; 0 disables GPS aiding entirely, 7 is the default.
EKF2_OF_CTRL: 0=optical flow fusion off, 1=on.
EKF2_RNG_CTRL: 0=range aiding off, 1=conditional, 2=enabled.

Only 'vision-only' manages EKF2_OF_CTRL/EKF2_RNG_CTRL. Switching to 'vision-gps'
or 'stock' leaves flow and range at whatever they were -- check with --show.

Note we never set the velocity bit (4): px4_odom_bridge deliberately does not
publish velocity, because nav_msgs/Odometry's twist frame is ambiguous across
FAST-LIO forks and wrong-frame velocity is worse for EKF2 than none. If you
later enable `publish_velocity:=true` and verify the frame on hardware, change
11 -> 15 here.
"""
import argparse
import struct
import sys
import time

from pymavlink import mavutil

CONN = "udpin:127.0.0.1:14569"
INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32

PROFILES = {
    "vision-only": {
        "desc": "LiDAR vision for position/height, fused with optical flow + range. GPS off.",
        "validated": True,
        "params": {
            "EKF2_EV_CTRL": 11,   # horiz pos + vert pos + yaw
            "EKF2_HGT_REF": 3,    # Vision
            "EKF2_GPS_CTRL": 0,   # GPS aiding off
            "EKF2_OF_CTRL": 1,    # optical flow fusion ON (deliberate, 2026-09-02)
            "EKF2_RNG_CTRL": 2,   # range aiding enabled
        },
        "notes": [
            "The NAME IS HISTORICAL. It means 'GPS aiding off', not 'vision is the",
            "only source'. Measured on the FC 2026-09-02: EKF2_OF_CTRL=1 and",
            "EKF2_RNG_CTRL=2 with live UAVCAN hardware behind both (sensor_optical_flow",
            "quality 159, distance_sensor 0.01-25 m, downward). Flow and range are now",
            "set here explicitly so the profile is deterministic instead of inheriting",
            "whatever was left on the FC.",
            "Keeping flow is a deliberate choice: it adds velocity aiding that LiDAR",
            "odometry does not provide (the bridge publishes no velocity on purpose).",
            "px4_odom_bridge MUST be running for a valid position estimate -- flow",
            "aids velocity, not absolute position, so it is not a position fallback.",
            "Treat the bridge as flight-critical.",
            "OPEN: EKF2_EV_POS_X/Y/Z are all 0, i.e. EKF2 thinks the LiDAR sits at the",
            "vehicle centre. The flow lever arm IS set (EKF2_OF_POS_Z=0.14), so this",
            "looks like an omission, not a decision. Measure FC -> MID360 IMU in FRD.",
            "OPEN: EKF2_HGT_REF=3 makes vision the primary height reference while",
            "EKF2_RNG_CTRL=2 also aids height. Verify they agree over non-flat ground",
            "(tables, kerbs) before trusting height in flight.",
        ],
    },
    "vision-gps": {
        "desc": "GPS primary for position/height; vision contributes yaw only.",
        "validated": False,
        "params": {
            "EKF2_EV_CTRL": 8,    # yaw only
            "EKF2_HGT_REF": 1,    # GPS
            "EKF2_GPS_CTRL": 7,   # GPS aiding on (PX4 default)
        },
        "notes": [
            "UNVALIDATED starting point -- re-check when the GPS is actually fitted.",
            "Deliberately conservative: it does NOT fuse vision and GPS horizontal",
            "position together. Those two can fight, because FAST-LIO's frame is FRD",
            "with an arbitrary heading offset and its origin is wherever the drone",
            "booted, while GPS is North-aligned and global. Fusing both positions",
            "needs EKF2 to reconcile that offset and should be validated in flight",
            "logs before you trust it.",
            "For flying under canopy you likely still want 'vision-only'; this",
            "profile is for open-sky transit and RTL safety.",
        ],
    },
    "stock": {
        "desc": "PX4 defaults. External vision off, GPS aiding on.",
        "validated": True,
        "params": {
            "EKF2_EV_CTRL": 0,
            "EKF2_HGT_REF": 1,
            "EKF2_GPS_CTRL": 7,
        },
        "notes": ["Safe fallback / revert target."],
    },
}

INT_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32, INT32,
}
TRACKED = ["EKF2_EV_CTRL", "EKF2_HGT_REF", "EKF2_GPS_CTRL",
           "EKF2_OF_CTRL", "EKF2_RNG_CTRL"]


def decode(pv):
    if pv.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", pv.param_value))[0]
    return round(pv.param_value, 4)


def encode_int(v):
    return struct.unpack("<f", struct.pack("<i", int(v)))[0]


def connect():
    m = mavutil.mavlink_connection(CONN)
    print(f"connecting on {CONN} ...")
    # ARK OS components share this bus and also emit HEARTBEATs; wait_heartbeat()
    # would grab one of those and yield sysid 0 (broadcast), silently breaking
    # every subsequent param read.
    end = time.time() + 20
    while time.time() < end:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if (msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                and msg.get_srcComponent() == 1):
            m.target_system, m.target_component = msg.get_srcSystem(), 1
            return m, msg
    sys.exit("ERROR: no PX4 autopilot heartbeat in 20s")


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


def identify(current):
    for pname, prof in PROFILES.items():
        if all(current.get(k) == v for k, v in prof["params"].items()):
            return pname
    return None


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="profiles:\n" + "\n".join(
            f"  {n:<12} {p['desc']}" + ("" if p["validated"] else "  [UNVALIDATED]")
            for n, p in PROFILES.items()))
    ap.add_argument("profile", nargs="?", choices=sorted(PROFILES),
                    help="profile to apply")
    ap.add_argument("--apply", action="store_true", help="actually write")
    ap.add_argument("--show", action="store_true",
                    help="print current EKF2 aiding config and exit")
    args = ap.parse_args()

    if not args.profile and not args.show:
        ap.error("give a profile, or --show")

    m, hb = connect()
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    state = mavutil.mavlink.enums["MAV_STATE"][hb.system_status].name
    print(f"FC sysid={m.target_system} comp=1  state={state}  armed={armed}\n")

    current = {k: read_param(m, k) for k in TRACKED}
    active = identify(current)
    print("current EKF2 aiding config:")
    for k in TRACKED:
        print(f"  {k:<15} = {current[k]}")
    print(f"  -> matches profile: {active or '(none / custom)'}\n")

    if args.show:
        return

    if armed:
        sys.exit("ABORT: vehicle is ARMED -- refusing to change EKF2 aiding sources.")

    prof = PROFILES[args.profile]
    print(f"--- profile '{args.profile}' ---")
    print(f"  {prof['desc']}")
    if not prof["validated"]:
        print("  ** UNVALIDATED -- read the notes before trusting this in flight **")
    for line in prof["notes"]:
        print(f"  {line}")
    print()
    for k, want in prof["params"].items():
        mark = "" if current[k] == want else "   <-- change"
        print(f"  {k:<15} {str(current[k]):>4}  ->  {want}{mark}")

    if not args.apply:
        print("\n[dry run] re-run with --apply to write.")
        return

    print("\n--- writing ---")
    ok = True
    for k, want in prof["params"].items():
        if current[k] == want:
            print(f"  {k:<15} already {want}, skipping")
            continue
        m.mav.param_set_send(
            m.target_system, m.target_component, k.encode(),
            encode_int(want), INT32)
        time.sleep(0.35)
        got = read_param(m, k)
        if got != want:
            ok = False
        print(f"  [{'OK ' if got == want else 'FAIL'}] {k:<15} = {got}")

    print("\n--- verify (fresh read) ---")
    final = {k: read_param(m, k) for k in prof["params"]}
    for k in prof["params"]:
        want = prof["params"][k]
        if final[k] != want:
            ok = False
        print(f"  {k:<15} = {final[k]}  {'OK' if final[k] == want else 'MISMATCH'}")

    print("\nRESULT:", f"profile '{args.profile}' applied" if ok
          else "SOME WRITES FAILED")
    print("NOTE: EKF2 reads these at init -- restart the estimator")
    print("      ('ekf2 stop' then 'ekf2 start' on the PX4 shell) or reboot the FC.")


if __name__ == "__main__":
    main()
