#!/usr/bin/env python3
"""Measure the pack's idle drain from voltage decay, and convert it to an implied current.

WHY VOLTAGE AND NOT CURRENT
---------------------------
`BAT1_I_CHANNEL` is `nan` on this aircraft, so PX4 does not actually measure pack current -- the
`current_a` field read a constant 0.37-0.40 A all through 2026-09-07 regardless of an 8.9 W swing
in compute load, which is what a dead sensor looks like. `BAT1_V_DIV` IS configured, so voltage
is a real measurement and is all we can trust.

WHAT IT IS FOR
--------------
On 2026-09-07 a pack went 24.85 V -> 20.93 V in ~4.5 h with **no motors**, and with neither the
Jetson nor the flight controller on it (both run from the Jetson's separate supply -- proven by
disconnecting the pack and watching the NX stay up with unbroken uptime). Roughly 2400 mAh of a
3300 mAh pack, about **0.5 A**, unaccounted for. That empties a full pack in ~6 h just sitting
connected, and leaves one over-discharged if forgotten overnight.

HOW TO USE IT
-------------
Take a BASELINE with everything connected, then disconnect ONE suspect and re-run. The difference
between runs is that subsystem's draw. Absolute numbers here are approximate; the DIFFERENCE
between two runs under identical conditions is the trustworthy part, which is why this prints the
raw decay rate alongside the derived current.

WHY THE CURRENT FIGURE IS APPROXIMATE
-------------------------------------
It maps volts to state-of-charge through a piecewise-linear 6S LiPo curve. In the flat middle of
that curve a small voltage error is a large charge error, so a short run at light load has wide
error bars. Run for at least ~15 minutes, and treat a single reading as indicative.
"""
import argparse
import sys
import time

# Per-cell resting voltage -> fraction of capacity remaining. Coarse but adequate: we need the
# SLOPE over a small interval, not absolute SoC.
LIPO_CURVE = [
    (4.20, 1.00), (4.10, 0.90), (4.00, 0.80), (3.92, 0.70), (3.85, 0.60),
    (3.80, 0.50), (3.75, 0.40), (3.70, 0.30), (3.65, 0.20), (3.55, 0.10),
    (3.40, 0.05), (3.20, 0.00),
]


def soc_from_cell_v(v):
    if v >= LIPO_CURVE[0][0]:
        return 1.0
    if v <= LIPO_CURVE[-1][0]:
        return 0.0
    for (v_hi, s_hi), (v_lo, s_lo) in zip(LIPO_CURVE, LIPO_CURVE[1:]):
        if v_lo <= v <= v_hi:
            f = (v - v_lo) / (v_hi - v_lo)
            return s_lo + f * (s_hi - s_lo)
    return 0.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--minutes', type=float, default=15.0)
    ap.add_argument('--capacity_mah', type=float, default=3300.0)
    ap.add_argument('--label', default='baseline', help='what was connected for this run')
    ap.add_argument('--out', default='/tmp/pack_drain.csv')
    args = ap.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from px4_msgs.msg import BatteryStatus
    except ImportError as e:
        print("ERROR: %s\n  Source the workspace (px4_msgs)." % e, file=sys.stderr)
        return 2

    rclpy.init()
    node = Node('measure_pack_drain')
    state = {'last': None}
    node.create_subscription(BatteryStatus, '/fmu/out/battery_status_v1',
                             lambda m: state.__setitem__('last', m), qos_profile_sensor_data)

    print("run: %s   duration %.0f min   pack %.0f mAh" % (args.label, args.minutes, args.capacity_mah))
    print("waiting for telemetry ...")
    t_wait = time.time()
    while state['last'] is None and time.time() - t_wait < 30:
        rclpy.spin_once(node, timeout_sec=0.2)
    if state['last'] is None:
        print("ERROR: no battery telemetry. Is the pack connected and the agent up?", file=sys.stderr)
        return 1

    samples = []
    t0 = time.time()
    end = t0 + args.minutes * 60.0
    with open(args.out, 'w') as fh:
        fh.write('t_s,voltage_v,cell_v,soc,discharged_mah\n')
        nxt = 0.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            el = time.time() - t0
            if el < nxt:
                continue
            nxt = el + 10.0
            m = state['last']
            v = float(m.voltage_v)
            cells = int(m.cell_count) or 6
            cv = v / cells
            soc = soc_from_cell_v(cv)
            samples.append((el, v, soc, float(m.discharged_mah)))
            fh.write('%.1f,%.4f,%.4f,%.5f,%.2f\n' % (el, v, cv, soc, m.discharged_mah))
            fh.flush()
            if int(el) % 120 < 10:
                print("  [%5.0fs] %.3f V  (%.3f V/cell, SoC ~%.1f%%)" % (el, v, cv, soc * 100))

    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 6:
        print("\nERROR: too few samples (%d)." % len(samples), file=sys.stderr)
        return 1

    (t_a, v_a, s_a, d_a) = samples[0]
    (t_b, v_b, s_b, d_b) = samples[-1]
    hours = (t_b - t_a) / 3600.0
    dv = v_a - v_b
    dsoc = s_a - s_b
    implied_a = (dsoc * args.capacity_mah / 1000.0) / hours if hours > 0 else 0.0
    px4_a = ((d_b - d_a) / 1000.0) / hours if hours > 0 else 0.0

    print("\n" + "=" * 70)
    print("PACK DRAIN  --  %s" % args.label)
    print("=" * 70)
    print("  window        %.1f min (%d samples)" % ((t_b - t_a) / 60.0, len(samples)))
    print("  voltage       %.3f -> %.3f V   (drop %.0f mV, %.1f mV/min)"
          % (v_a, v_b, dv * 1000, dv * 1000 / max((t_b - t_a) / 60.0, 1e-9)))
    print("  SoC (curve)   %.1f%% -> %.1f%%" % (s_a * 100, s_b * 100))
    print("  IMPLIED DRAW  %.2f A   (from the voltage curve -- the trustworthy signal)" % implied_a)
    print("  PX4 coulomb   %.2f A   (from discharged_mah; suspect -- BAT1_I_CHANNEL is nan)" % px4_a)
    print("  wrote %s" % args.out)

    if dv * 1000 < 15:
        print("\n  ⚠️  Only %.0f mV of decay. In the flat part of a LiPo curve that is close to"
              % (dv * 1000))
        print("      measurement noise -- run longer before trusting the current figure.")
    print("\n  COMPARE RUNS, don't trust one: re-run with a suspect disconnected and take the")
    print("  DIFFERENCE. Absolute values inherit the curve's error; the difference does not.")
    print("=" * 70)
    return 0


if __name__ == '__main__':
    sys.exit(main())
