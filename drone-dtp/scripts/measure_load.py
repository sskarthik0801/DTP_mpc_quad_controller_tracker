#!/usr/bin/env python3
"""Measure CONCURRENT-load power and thermals on the drone NX -- the number the airframe needs.

WHY THIS EXISTS
---------------
plan/dtp-imp/02 and 05 size the power and cooling budget from an 11 W figure that was
**GPU-only**, measured with nothing else running. The number that actually matters is the whole
edge stack at once: Jetson + Orbbec camera + Livox driver + TensorRT inference. plan/dtp-imp/21
lists this under "measurements we are missing (do not design around guesses)".

It also answers a question the telemetry alone could not: **is the Jetson fed from the flight
pack?** Idle battery current has twice matched Jetson VDD_IN almost exactly (0.39 A @ 8.6 W /
22 V, then 0.36 A @ 8.7 W / 24.2 V). Coincidence and causation look identical at one operating
point, so this ramps the compute load and checks whether pack current follows. If it does, every
watt of perception comes straight out of hover endurance.

METHOD
------
Three phases -- idle baseline, concurrent load, cooldown -- sampling tegrastats, thermal zones,
fan, and the FC's battery telemetry throughout. The cooldown phase is not padding: the recovery
slope is what tells you whether the idle temperature was a steady state or a still-rising one.

READ THE PREREQUISITES
----------------------
* **GPU clocks must be pinned** (`sudo jetson_clocks`). Unpinned, DVFS makes power and FPS both
  meaningless -- that mistake already invalidated a full round of benchmarks (plan 21,
  2026-09-07). This script records the min/max GPU frequency so the reader can check.
* Throttling is detected by GPU frequency dropping BELOW the pinned value, not by temperature
  alone -- the whole point is to find out whether it throttles, not to assume a threshold.
"""
import argparse
import csv
import os
import re
import subprocess
import sys
import time

TEGRA_RE = {
    'vdd_in_mw': re.compile(r'VDD_IN (\d+)mW'),
    'vdd_cpu_gpu_cv_mw': re.compile(r'VDD_CPU_GPU_CV (\d+)mW'),
    'vdd_soc_mw': re.compile(r'VDD_SOC (\d+)mW'),
    'gr3d_pct': re.compile(r'GR3D_FREQ (\d+)%'),
    'tj_c': re.compile(r'tj@([\d.]+)C'),
    'gpu_c': re.compile(r'gpu@([\d.]+)C'),
    'cpu_c': re.compile(r'cpu@([\d.]+)C'),
    'ram_mb': re.compile(r'RAM (\d+)/'),
}
CPU_RE = re.compile(r'CPU \[([^\]]+)\]')


def read_fan():
    for base in ('/sys/devices/platform/pwm-fan/hwmon/hwmon0',
                 '/sys/class/hwmon/hwmon0'):
        try:
            pwm = int(open(os.path.join(base, 'pwm1')).read().strip())
            rpm = None
            try:
                rpm = int(open(os.path.join(base, 'rpm')).read().strip())
            except OSError:
                pass
            return pwm, rpm
        except OSError:
            continue
    return None, None


def gpu_freq_mhz():
    for p in ('/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq',
              '/sys/class/devfreq/17000000.gpu/cur_freq'):
        try:
            return int(open(p).read().strip()) // 1_000_000
        except (OSError, ValueError):
            continue
    return None


def parse_tegra(line):
    out = {}
    for k, rx in TEGRA_RE.items():
        m = rx.search(line)
        out[k] = float(m.group(1)) if m else None
    m = CPU_RE.search(line)
    if m:
        loads = [int(p.split('%')[0]) for p in m.group(1).split(',') if '%' in p]
        out['cpu_mean_pct'] = sum(loads) / len(loads) if loads else None
        out['cpu_max_pct'] = max(loads) if loads else None
    return out


class Battery:
    """Reads the FC battery over ROS 2. Optional -- absent FC must not abort the measurement."""

    def __init__(self):
        self.ok = False
        try:
            import rclpy
            from rclpy.qos import qos_profile_sensor_data
            from px4_msgs.msg import BatteryStatus
            rclpy.init()
            self._rclpy = rclpy
            self.node = rclpy.create_node('load_measure_battery')
            self.latest = None

            def cb(msg):
                self.latest = msg
            # PX4 publishes BEST_EFFORT; a RELIABLE subscriber never connects and reads as a
            # dead FC with no error at all (plan 21 gotchas).
            self.node.create_subscription(
                BatteryStatus, '/fmu/out/battery_status_v1', cb, qos_profile_sensor_data)
            self.ok = True
        except Exception as exc:
            print("  (battery telemetry unavailable: %s)" % str(exc)[:80], file=sys.stderr)

    def sample(self):
        if not self.ok:
            return None, None
        self._rclpy.spin_once(self.node, timeout_sec=0.05)
        m = self.latest
        return (float(m.voltage_v), float(m.current_a)) if m is not None else (None, None)

    def close(self):
        if self.ok:
            try:
                self.node.destroy_node()
                self._rclpy.shutdown()
            except Exception:
                pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', default=os.path.expanduser('~/models/yolo11s.engine'),
                    help='TensorRT engine used as the GPU load')
    ap.add_argument('--idle', type=float, default=60.0, help='idle baseline seconds')
    ap.add_argument('--load', type=float, default=300.0, help='loaded seconds')
    ap.add_argument('--cool', type=float, default=90.0, help='cooldown seconds')
    ap.add_argument('--out', default=os.path.expanduser('~/load_runs'), help='output directory')
    ap.add_argument('--bench', default=os.path.expanduser(
        '~/super_ws/src/drone-dtp/scripts/bench_yolo.py'), help='load generator')
    args = ap.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(args.out, 'load_%s.csv' % stamp)

    gmin = gpu_freq_mhz()
    print("GPU freq at start: %s MHz  (clocks must be PINNED -- see docstring)" % gmin)
    fan0 = read_fan()
    print("fan at start: pwm=%s rpm=%s" % fan0)

    teg = subprocess.Popen(['tegrastats', '--interval', '1000'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    bat = Battery()
    load_proc = None
    log_path = None
    rows = []
    t0 = time.time()
    phase = 'idle'
    try:
        with open(csv_path, 'w', newline='') as fh:
            w = None
            while True:
                line = teg.stdout.readline()
                if not line:
                    break
                el = time.time() - t0

                if phase == 'idle' and el >= args.idle:
                    phase = 'load'
                    print("\n[%.0fs] --- starting load: %s ---" % (el, os.path.basename(args.model)))
                    # Capture the load generator's output. Sending it to DEVNULL hid an
                    # ImportError (libcudss) and the run then reported a confident -0.0 W
                    # "result" from a load that never started.
                    log_path = os.path.join(args.out, 'loadgen_%s.log' % stamp)
                    load_log = open(log_path, 'w')
                    load_proc = subprocess.Popen(
                        [sys.executable, args.bench, '--model', args.model,
                         '--duration', str(args.load + 30), '--prewarm', '0',
                         '--target_hz', '1000'],
                        stdout=load_log, stderr=subprocess.STDOUT)
                elif phase == 'load' and el >= args.idle + args.load:
                    phase = 'cool'
                    print("\n[%.0fs] --- load off, cooling ---" % el)
                    if load_proc and load_proc.poll() is None:
                        load_proc.terminate()
                elif phase == 'cool' and el >= args.idle + args.load + args.cool:
                    break

                rec = parse_tegra(line)
                pwm, rpm = read_fan()
                v, a = bat.sample()
                rec.update(t_s=round(el, 1), phase=phase, fan_pwm=pwm, fan_rpm=rpm,
                           gpu_mhz=gpu_freq_mhz(), batt_v=v, batt_a=a)
                if w is None:
                    w = csv.DictWriter(fh, fieldnames=sorted(rec))
                    w.writeheader()
                w.writerow(rec)
                fh.flush()
                rows.append(rec)
                if int(el) % 15 == 0:
                    print("  [%4.0fs] %-5s VDD_IN %5.1fW  tj %5.1fC  GPU %3s%% @%sMHz  fan %s  batt %s V %s A"
                          % (el, phase, (rec.get('vdd_in_mw') or 0) / 1000.0,
                             rec.get('tj_c') or 0, rec.get('gr3d_pct'), rec.get('gpu_mhz'),
                             pwm, ('%.2f' % v) if v else '-', ('%.2f' % a) if a else '-'))
    finally:
        if load_proc and load_proc.poll() is None:
            load_proc.terminate()
        teg.terminate()
        bat.close()

    ok = summarise(rows, csv_path, gmin, log_path)
    return 0 if ok else 1


def _stat(rows, key, phase, fn):
    vals = [r[key] for r in rows if r.get('phase') == phase and r.get(key) is not None]
    return fn(vals) if vals else None


def summarise(rows, csv_path, gpu_pinned_mhz, log_path=None):
    def line(label, key, unit='', scale=1.0):
        out = "  %-22s" % label
        for ph in ('idle', 'load', 'cool'):
            mx = _stat(rows, key, ph, max)
            mean = _stat(rows, key, ph, lambda v: sum(v) / len(v))
            out += ("  %s: %6.1f/%-6.1f" % (ph, (mean or 0) * scale, (mx or 0) * scale))
        return out + "  " + unit + "   (mean/max)"

    print("\n" + "=" * 78)
    print("CONCURRENT-LOAD SUMMARY   (%d samples)  ->  %s" % (len(rows), csv_path))
    print("=" * 78)

    # ---- VALIDITY GATE -----------------------------------------------------
    # Refuse to report conclusions from a load that never happened. The first run of this
    # script did exactly that: the generator died on an ImportError and the summary still
    # announced "load costs -0.0 W" and "the Jetson is NOT on this pack" -- both nonsense,
    # both stated with total confidence. A measurement harness that cannot tell "no effect"
    # from "no experiment" is worse than no harness.
    gpu_load = [r['gr3d_pct'] for r in rows
                if r.get('phase') == 'load' and r.get('gr3d_pct') is not None]
    peak_gpu = max(gpu_load) if gpu_load else 0.0
    if peak_gpu < 5.0:
        print("\n  *** RUN INVALID -- THE LOAD NEVER RAN ***")
        print("  Peak GPU utilisation during the 'load' phase was %.1f%%." % peak_gpu)
        print("  Every figure below describes an idle machine. Do NOT quote them.")
        if log_path and os.path.exists(log_path):
            print("\n  load generator output (%s):" % log_path)
            try:
                tail = open(log_path, encoding='utf-8', errors='replace').read().strip().split('\n')
                for l in tail[-8:]:
                    print("    %s" % l[:110])
            except OSError:
                pass
        print("\n  Most likely: the generator could not import torch. Source the Jetson env")
        print("  FIRST -- source ~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh -- which sets")
        print("  LD_LIBRARY_PATH for libcudss; torch will not import without it.")
        print("=" * 78)
        return False

    print(line("total board power", 'vdd_in_mw', 'W', 0.001))
    print(line("CPU+GPU+CV rail", 'vdd_cpu_gpu_cv_mw', 'W', 0.001))
    print(line("junction temp", 'tj_c', 'C'))
    print(line("GPU utilisation", 'gr3d_pct', '%'))
    print(line("fan pwm", 'fan_pwm', '/255'))

    idle_w = _stat(rows, 'vdd_in_mw', 'idle', lambda v: sum(v) / len(v))
    load_w = _stat(rows, 'vdd_in_mw', 'load', lambda v: sum(v) / len(v))
    peak_w = _stat(rows, 'vdd_in_mw', 'load', max)
    if idle_w and load_w:
        print("\n  CONCURRENT LOAD COSTS %+.1f W over idle (%.1f -> %.1f W, peak %.1f W)"
              % ((load_w - idle_w) / 1000.0, idle_w / 1000.0, load_w / 1000.0, peak_w / 1000.0))

    tj_i = _stat(rows, 'tj_c', 'idle', lambda v: sum(v) / len(v))
    tj_l = _stat(rows, 'tj_c', 'load', max)
    if tj_i and tj_l:
        print("  THERMAL RISE %+.1f C (idle %.1f -> peak %.1f C)" % (tj_l - tj_i, tj_i, tj_l))

    # Throttling is a frequency event, not a temperature guess.
    if gpu_pinned_mhz:
        low = [r for r in rows if r.get('phase') == 'load' and r.get('gpu_mhz')
               and r['gpu_mhz'] < gpu_pinned_mhz * 0.95]
        if low:
            print("  !! THROTTLED: GPU dropped below %d MHz on %d samples (min %d MHz)"
                  % (gpu_pinned_mhz, len(low), min(r['gpu_mhz'] for r in low)))
        else:
            print("  NO THROTTLING: GPU held %d MHz for the whole loaded phase." % gpu_pinned_mhz)

    # Is the Jetson fed from the flight pack? Only answerable if the current sensor works.
    pairs = [(r['vdd_in_mw'] / 1000.0, r['batt_a']) for r in rows
             if r.get('vdd_in_mw') and r.get('batt_a')]
    if len(pairs) > 20:
        ai = [a for w, a in pairs if w < (idle_w or 0) / 1000.0 + 2]
        al = [a for w, a in pairs if w > (load_w or 0) / 1000.0 - 2]
        if ai and al:
            di = sum(ai) / len(ai)
            dl = sum(al) / len(al)
            currents = [a for _, a in pairs]
            spread = max(currents) - min(currents)
            print("\n  BATTERY CURRENT  idle %.2f A -> loaded %.2f A  (delta %+.2f A, "
                  "full-run spread %.3f A)" % (di, dl, dl - di, spread))

            # A DEAD current sensor reads flat for exactly the same reason a disconnected load
            # does, so "no change" is NOT evidence of topology on its own. PX4 reports a
            # current even when BAT1_I_CHANNEL is unassigned. Check the sensor before inferring.
            if spread < 0.05:
                print("  -> INCONCLUSIVE. The reading barely moved across the whole run, which is")
                print("     equally consistent with (a) the Jetson being on a separate supply and")
                print("     (b) the current sensor not working at all. CHECK FIRST:")
                print("       BAT1_I_CHANNEL must not be nan  (unassigned == no real measurement)")
                print("     Measured 2026-09-07: it WAS nan, so this axis told us nothing.")
                print("     Resolve by bringing up INA238 telemetry (CLAUDE.md blocking gate 3),")
                print("     or measure the 12 V rail with a meter.")
            elif abs(dl - di) < 0.05:
                print("  -> pack current is live but did NOT follow compute load: the Jetson is on")
                print("     a separate supply, so perception power does not cost hover time.")
            else:
                print("  -> pack current FOLLOWS compute load: the Jetson IS on the flight pack,")
                print("     so every watt of perception comes out of endurance. Budget accordingly.")
    print("=" * 78)
    return True


if __name__ == '__main__':
    sys.exit(main())
