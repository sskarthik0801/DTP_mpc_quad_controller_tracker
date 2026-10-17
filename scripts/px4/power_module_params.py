#!/usr/bin/env python3
"""Configure PX4 for the ARK 12S Payload Power Module (INA238 over I2C).

Wiring on this airframe: the module's 6-pin CLIK-Mate goes to the FC's GPS/I2C
port, sharing SCL/SDA with the GPS+compass. That is fine -- the INA238 answers
at 0x45 and the compass at 0x1e. Confirmed present with `i2cdetect -b 1`
(bus 1 is ARK FPV's only *external* I2C bus: boards/ark/fpv/src/i2c.cpp does
`initI2CBusExternal(1)`).

PREREQUISITE -- READ THIS FIRST
    The stock ARK FPV firmware does NOT contain the ina238 driver, so
    SENS_EN_INA238 / INA238_SHUNT do not exist and this script will refuse to
    run. That is a firmware limitation, not a parameter one:
    boards/ark/fpv/default.px4board has no CONFIG_DRIVERS_POWER_MONITOR_*, and
    boards/ark/fpv/init/rc.board_sensors has no ina238 start line (ARKV6X /
    ark/fmu-v6x has both -- that is why ARK's docs assume the params exist).
    See docs/ark-12s-payload-power-module.md for the two-line board patch.

Dry-run by default; --apply writes. Refuses if armed. Verifies by read-back.
"""
import math
import struct
import sys
import time

from pymavlink import mavutil

CONN = "udpin:127.0.0.1:14569"
DRY_RUN = "--apply" not in sys.argv

INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

INT_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32, INT32,
}

# The param that only exists once the driver is compiled in. Checked first so a
# stock-firmware board fails with a useful message instead of three timeouts.
GATE_PARAM = "SENS_EN_INA238"

# (name, value, type, rationale)
CHANGES = [
    ("SENS_EN_INA238", 1, INT32,
     "start the ina238 driver from rc.board_sensors at boot"),
    ("INA238_SHUNT", 0.0001, REAL32,
     "ARK 12S Payload PM uses a 0.1 mOhm shunt (default is 0.0005)"),
    ("BAT1_V_CHANNEL", -2, INT32,
     "disable the onboard analog voltage sense; -2 is PX4's documented "
     "'use digital monitoring exclusively'. Without it the board ADC and the "
     "INA238 both publish battery_status id=1 and fight over the topic."),
]

# Read-only: reported so a human can sanity-check them, never written here.
REVIEW = [
    ("BAT1_SOURCE", "must stay 0 -- ina238.cpp constructs its Battery with "
                    "battery_status_s::SOURCE_POWER_MODULE"),
    ("BAT1_N_CELLS", "must match the pack actually fitted"),
    ("BAT1_CAPACITY", "mAh of the pack actually fitted"),
    ("INA238_CURRENT", "max expected current; 327.68 A default is correct for "
                       "a 0.1 mOhm shunt (keeps the INA238 in its +/-40.96 mV "
                       "high-resolution ADC range, 10 mA LSB)"),
    ("BAT1_A_PER_V", "irrelevant once BAT1_V_CHANNEL=-2 (analog path off)"),
]


def decode(pv):
    if pv.param_type in INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", pv.param_value))[0]
    return pv.param_value


def encode(value, ptype):
    """PX4 carries INT32 params bit-packed in the float param_value field."""
    if ptype in INT_TYPES:
        return struct.unpack("<f", struct.pack("<i", int(value)))[0]
    return float(value)


def same(got, want):
    if got is None:
        return False
    if isinstance(want, float):
        return math.isclose(got, want, rel_tol=1e-6, abs_tol=1e-12)
    return got == want


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

# The UDP bus is shared with mavlink-routerd and ARK OS components, which also
# emit HEARTBEATs. wait_heartbeat() grabs whichever arrives first and yields
# sysid 0, which makes every param read silently return None.
hb = None
end = time.time() + 20
while time.time() < end:
    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    if msg is None:
        continue
    if (msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
            and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
        hb = msg
        break
    print(f"  (ignoring heartbeat from sys={msg.get_srcSystem()} "
          f"comp={msg.get_srcComponent()} autopilot={msg.autopilot})")

if hb is None:
    sys.exit("ERROR: no PX4 autopilot heartbeat seen in 20s")

m.target_system = hb.get_srcSystem()
m.target_component = hb.get_srcComponent()
print(f"FC sysid={m.target_system} comp={m.target_component}")

if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
    sys.exit("ABORT: vehicle is ARMED -- refusing to touch battery params.")

print(f"\n--- firmware check: does {GATE_PARAM} exist? ---")
if read_param(m, GATE_PARAM) is None:
    print(f"  {GATE_PARAM}: NOT PRESENT")
    sys.exit(
        "\nABORT: this firmware has no INA238 driver, so there is nothing to\n"
        "configure. Check with `python3 scripts/px4/px4_shell.py 'ina238 status'`\n"
        "-- 'command not found' confirms it. Fix: rebuild ark_fpv_default with\n"
        "CONFIG_DRIVERS_POWER_MONITOR_INA238=y plus the rc.board_sensors start\n"
        "line, then flash. See docs/ark-12s-payload-power-module.md.")
print(f"  {GATE_PARAM}: present -- driver is in this firmware.")

print("\n--- current values ---")
before = {}
for name, want, _t, _why in CHANGES:
    before[name] = read_param(m, name)
    mark = "" if same(before[name], want) else "   <- change"
    print(f"  {name:<15} = {before[name]}   ->  {want}{mark}")

print("\n--- for review (not written) ---")
for name, why in REVIEW:
    print(f"  {name:<15} = {read_param(m, name)}   ({why})")

if DRY_RUN:
    print("\n[dry run] re-run with --apply to write.")
    sys.exit(0)

print("\n--- writing ---")
ok = True
for name, want, ptype, why in CHANGES:
    if same(before[name], want):
        print(f"  {name:<15} already {want}, skipping")
        continue
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), encode(want, ptype), ptype)
    time.sleep(0.35)
    got = read_param(m, name)
    if not same(got, want):
        ok = False
    print(f"  [{'OK ' if same(got, want) else 'FAIL'}] {name:<15} = {got}   ({why})")

print("\n--- verify (fresh read) ---")
for name, want, _t, _why in CHANGES:
    got = read_param(m, name)
    print(f"  {name:<15} = {got}  {'OK' if same(got, want) else 'MISMATCH'}")
    if not same(got, want):
        ok = False

print("\nRESULT:", "all params set" if ok else "SOME WRITES FAILED")
print("""
NEXT: reboot the FC, then confirm the driver actually bound to the module:

  python3 scripts/px4/px4_shell.py "ina238 status" "listener battery_status"

Expect ina238 running on bus 1 @ 0x45, and battery_status with a NON-ZERO
current_a and a sane voltage_v. current_a stuck at 0.00000 means the driver
is not the one publishing -- re-check BAT1_V_CHANNEL=-2.

ARK's docs ask for a reboot after enabling the driver AND a second one after
setting the shunt. This script writes both at once, so reboot twice to be safe:
the shunt is only read in INA238::init().""")
