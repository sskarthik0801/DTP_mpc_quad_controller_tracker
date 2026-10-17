#!/usr/bin/env python3
"""Set EKF2's external-vision lever arm: where the LiDAR sits on the airframe.

    python3 scripts/px4/ev_lever_arm.py            # dry run
    python3 scripts/px4/ev_lever_arm.py --apply

EKF2 treats an incoming vision measurement as describing the body origin -- the
FC IMU (EKF2_IMU_POS_* are all 0 on this airframe). The bridge actually reports
the pose of the *MID360's internal IMU*, which is not at that origin. Left at 0,
EKF2 charges the rotation-induced motion of that offset point to translation:
error on the order of the offset itself, worst during the yaw a human-following
drone does constantly.

DERIVATION (measured 2026-09-02: LiDAR directly above the FC, no horizontal
offset; 10.5 cm from the FC to where the MID360's glass dome starts)

FAST-LIO's `body` frame is the MID360 IMU, NOT the LiDAR:
  laserMapping.cpp:218  p_imu = R_L_I * p_lidar + T_L_I
so mid360.yaml's extrinsic_T is the LiDAR origin expressed in the IMU frame:
  extrinsic_T = [-0.011, -0.02329, +0.04412]
=> the IMU sits 4.41 cm BELOW the LiDAR origin, 1.1 cm forward, 2.33 cm left.

  FC IMU --10.5cm--> dome seam --d--> LiDAR origin --4.41cm down--> MID360 IMU
  height of IMU above FC = 10.5 + d - 4.41 cm

d (dome seam -> optical origin) is in Livox's mechanical drawing and is NOT
known here. d=0 gives 6.09 cm; the origin is genuinely above the seam, so the
true value is a little more. We use 7 cm, mid-band. Worst case that is ~2 cm
out, against 6-9 cm of error at the 0.0 it replaces.

PX4 body is FRD, so "above" is NEGATIVE z and "left" is NEGATIVE y:
  x = +0.011   (1.1 cm forward)
  y = -0.023   (2.33 cm left)
  z = -0.07    (7 cm above)

LIMITS: translation only. If the MID360 is rotated in yaw relative to the
airframe, this does not help -- that needs a rotation in px4_odom_bridge, which
today does only the FLU->FRD convention flip.

Refuses to write while armed. Verifies every write by read-back. Take a full
backup first: scripts/px4/dump_params.py
"""
import os
import struct
import sys
import time

from pymavlink import mavutil

CONN = os.environ.get("PX4_CONN", "udpin:127.0.0.1:14569")
REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

# (name, value, rationale)
CHANGES = [
    ("EKF2_EV_POS_X",  0.011, "1.1 cm forward  (MID360 IMU vs LiDAR origin)"),
    ("EKF2_EV_POS_Y", -0.023, "2.33 cm left -> negative in FRD"),
    ("EKF2_EV_POS_Z", -0.07,  "7 cm above the FC IMU -> negative in FRD"),
]

TOL = 1e-4


def pname(pv):
    pid = pv.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="ignore")
    return pid.rstrip("\x00")


def read_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(
        m.target_system, m.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        pv = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if pv is not None and pname(pv) == name:
            # REAL32 params come through as a real float; only INT32 is bit-packed.
            if pv.param_type == REAL32:
                return pv.param_value
            return struct.unpack("<i", struct.pack("<f", pv.param_value))[0]
    return None


def main():
    apply = "--apply" in sys.argv
    m = mavutil.mavlink_connection(CONN)
    print(f"connecting on {CONN} ...")
    end = time.time() + 20
    hb = None
    while time.time() < end:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if (msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
                and msg.get_srcComponent() == 1):
            hb = msg
            m.target_system, m.target_component = msg.get_srcSystem(), 1
            break
    if hb is None:
        sys.exit("ERROR: no PX4 autopilot heartbeat in 20s")

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"FC sysid={m.target_system} comp=1  armed={armed}\n")
    if apply and armed:
        sys.exit("ABORT: vehicle is ARMED -- refusing to change EKF2 geometry.")

    print("--- external-vision lever arm (body FRD, metres) ---")
    for name, want, why in CHANGES:
        cur = read_param(m, name)
        mark = "" if cur is not None and abs(cur - want) < TOL else "   <-- change"
        print(f"  {name:<15} {str(cur):>8}  ->  {want:<8}{mark}   # {why}")

    if not apply:
        print("\n[dry run] re-run with --apply to write.")
        return

    print("\n--- writing ---")
    ok = True
    for name, want, _ in CHANGES:
        m.mav.param_set_send(
            m.target_system, m.target_component, name.encode(), float(want), REAL32)
        time.sleep(0.35)
        got = read_param(m, name)
        good = got is not None and abs(got - want) < TOL
        ok = ok and good
        print(f"  [{'OK ' if good else 'FAIL'}] {name:<15} = {got}")

    print("\n--- verify (fresh read) ---")
    for name, want, _ in CHANGES:
        got = read_param(m, name)
        good = got is not None and abs(got - want) < TOL
        ok = ok and good
        print(f"  {name:<15} = {got}  {'OK' if good else 'MISMATCH'}")

    print("\nRESULT:", "lever arm applied" if ok else "SOME WRITES FAILED")
    print("NOTE: EKF2 reads these at init -- restart the estimator")
    print("      (px4_shell.py 'ekf2 stop' 'ekf2 start') or reboot the FC.")


if __name__ == "__main__":
    main()
