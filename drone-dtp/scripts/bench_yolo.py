#!/usr/bin/env python3
"""YOLO TensorRT throughput benchmark -- the M0.2 Step 0.2.2 FPS gate.

WHY THIS EXISTS IN THE REPO: the gate was declared PASSED on 2026-08-15 with this script living
only in /tmp on the Jetson. /tmp was later cleared and it was gone -- a passed gate with nothing
reproducible behind it. Anything that gates a milestone belongs in version control.

WHAT IT MEASURES, and why the breakdown matters more than the FPS:
  Reports pre / inference / post separately, not just frames-per-second. On the Orin NX the
  2026-08-15 run showed ~6.8 ms (~35 %) of each frame was pre+post, NOT inference -- which is the
  actual lever for M1.4's concurrent budget. That finding reproduced on a second machine (30-39 %).

  !!! RUN `sudo jetson_clocks` FIRST OR THE NUMBERS ARE MEANINGLESS.
  MEASURED 2026-09-07 on the drone NX: pinning clocks **more than DOUBLED** every result
  (yolo11s 42.9 -> 95.2 FPS, yolo11n 59.7 -> 127.8, yolo26n 62.4 -> 134.5). MAXN raises the power
  CEILING; it does NOT pin frequencies, and a benchmark with brief gaps between inferences lets the
  governor scale down. Unpinned, the same engine varied 42.8-49.1 FPS and ordering changed results
  by up to 23 %. Pinned, forward vs reversed ordering agreed within 0.8 %.
  jetson_clocks does NOT survive reboot -- re-apply it every session.

  !! A CROSS-MODEL CLAIM FROM THIS SCRIPT WAS ONCE WRONG. The 2026-08-15 run concluded yolo11s costs
  only ~+1.1 ms inference over yolo11n, i.e. "model capacity is nearly free". On 2026-09-06 the same
  engine file measured 42.8 FPS when benchmarked FIRST and 49.1 FPS when benchmarked SECOND -- a
  15 % ordering artefact from cold SoC clocks, large enough to manufacture or erase exactly that
  kind of per-model difference. --prewarm now exists to prevent it. Treat any cross-model comparison
  made WITHOUT a global pre-warm as unreliable, including the original one.

  Splits first-half vs second-half FPS as a THERMAL THROTTLING check. A benchmark that only reports
  a mean will happily average away a device that is degrading under sustained load.

THREE CAVEATS -- do not misquote the output (plan/dtp-imp/09 M1.4):
  1. ISOLATED. The real figure is the CONCURRENT rate with FAST-LIO2 + SUPER + fusion sharing one
     time-slicing GPU (~10-15 Hz expected). NEVER quote this number as the system rate.
  2. SYNTHETIC input by default -- random frames. Valid for throughput at a fixed TensorRT shape;
     says NOTHING about detection accuracy. That is M1.3's canopy-lit/occluded eval set.
  3. Engines are NOT PORTABLE (R-F8-engine). Build the .engine on the device it will run on.

Usage:
    # single model
    python3 scripts/bench_yolo.py --model ~/dtp_ws/models/yolo11s.engine --duration 60

    # three-way comparison (as run 2026-08-15)
    python3 scripts/bench_yolo.py --duration 30 \
        --model ~/dtp_ws/models/yolo11s.engine \
        --model ~/dtp_ws/models/yolo11n.engine \
        --model ~/dtp_ws/models/yolo26n.engine
"""
import argparse
import os
import statistics
import sys
import time

import numpy as np

PERSON_CLASS = 0


def _ensure_torch_libs():
    """Make `import torch` work when CUDA libs ship in pip wheels the loader never searches.

    This script has now died twice on `ImportError: libcudss.so.0`, and the FIRST time the
    failure was invisible to the caller: `measure_load.py` sent this script's output to
    /dev/null, saw no load, and cheerfully reported "-0.0 W" for a benchmark that never ran a
    single inference. A harness that cannot tell "no load" from "no process" is worse than none.

    The library is present, merely unreachable -- pip's nvidia wheels drop it in
    site-packages/nvidia/*/lib, which the dynamic loader does not search. LD_LIBRARY_PATH is read
    at process start, so assigning to os.environ here would be far too late; dlopen'ing the file
    by absolute path instead puts it in the process image, which satisfies the dependency when
    torch's own .so asks for it moments later.
    """
    import ctypes
    import glob
    try:
        ctypes.CDLL('libcudss.so.0')
        return                                  # already reachable
    except OSError:
        pass
    roots = glob.glob(os.path.join(os.path.dirname(os.__file__), 'site-packages',
                                   'nvidia', '*', 'lib'))
    roots += glob.glob(os.path.expanduser('~/.local/lib/python3.*/site-packages/nvidia/*/lib'))
    for d in roots:
        for so in sorted(glob.glob(os.path.join(d, 'libcudss*.so*'))):
            try:
                ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
    try:
        ctypes.CDLL('libcudss.so.0')
    except OSError:
        print('WARNING: libcudss.so.0 still not loadable -- import torch will likely '
              'fail; searched: %s' % (', '.join(roots) or '(no nvidia wheel dirs found)'),
              file=sys.stderr)


def read_temp_c():
    """Peak SoC temperature, for the throttling check. Jetson exposes thermal zones in sysfs;
    returning None rather than guessing keeps a missing sensor from being reported as 0 degrees."""
    best = None
    base = '/sys/class/thermal'
    if not os.path.isdir(base):
        return None
    for z in os.listdir(base):
        p = os.path.join(base, z, 'temp')
        try:
            with open(p) as f:
                v = int(f.read().strip()) / 1000.0
            if -20.0 < v < 150.0:
                best = v if best is None else max(best, v)
        except (OSError, ValueError):
            continue
    return best


def _prewarm(model_path, seconds, imgsz):
    """Run sustained inference purely to bring SoC/GPU clocks up before any measurement.
    Results are discarded. See --prewarm for why this exists."""
    from ultralytics import YOLO
    import numpy as _np
    m = YOLO(model_path, task='detect')
    frame = _np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=_np.uint8)
    t0 = time.time()
    while time.time() - t0 < seconds:
        m.predict(frame, imgsz=imgsz, classes=[PERSON_CLASS], verbose=False)


def bench_one(model_path, duration, imgsz, warmup):
    from ultralytics import YOLO

    model = YOLO(model_path, task='detect')
    frame = np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=np.uint8)

    # Warm-up is not optional: the first inferences include TensorRT context setup and allocator
    # warm-up, and folding them into the measurement understates steady-state throughput.
    for _ in range(warmup):
        model.predict(frame, imgsz=imgsz, classes=[PERSON_CLASS], verbose=False)

    t_start = time.time()
    temp_start = read_temp_c()
    lat, pre, inf, post = [], [], [], []
    half_time = t_start + duration / 2.0
    first_half = 0

    while True:
        now = time.time()
        if now - t_start >= duration:
            break
        t0 = time.perf_counter()
        r = model.predict(frame, imgsz=imgsz, classes=[PERSON_CLASS], verbose=False)
        lat.append((time.perf_counter() - t0) * 1000.0)
        s = r[0].speed          # ultralytics reports ms for each stage
        pre.append(s.get('preprocess', 0.0))
        inf.append(s.get('inference', 0.0))
        post.append(s.get('postprocess', 0.0))
        if now < half_time:
            first_half += 1

    elapsed = time.time() - t_start
    temp_end = read_temp_c()
    n = len(lat)
    second_half = n - first_half

    return {
        'model': os.path.basename(model_path),
        'frames': n,
        'elapsed': elapsed,
        'fps': n / elapsed if elapsed > 0 else 0.0,
        'fps_first': first_half / (duration / 2.0),
        'fps_second': second_half / (duration / 2.0),
        'p50': statistics.median(lat) if lat else 0.0,
        'p95': (sorted(lat)[int(0.95 * (n - 1))] if n > 1 else (lat[0] if lat else 0.0)),
        'pre': statistics.mean(pre) if pre else 0.0,
        'inf': statistics.mean(inf) if inf else 0.0,
        'post': statistics.mean(post) if post else 0.0,
        'temp_start': temp_start,
        'temp_end': temp_end,
    }


def main(argv=None):
    _ensure_torch_libs()      # before anything can import torch
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', action='append', required=True,
                    help='path to a .engine (repeat the flag to compare several)')
    ap.add_argument('--duration', type=float, default=60.0, help='seconds per model (default 60)')
    ap.add_argument('--imgsz', type=int, default=640,
                    help='MUST match the size the engine was exported at (default 640)')
    ap.add_argument('--warmup', type=int, default=30, help='warm-up iterations (default 30)')
    ap.add_argument('--target_hz', type=float, default=15.0,
                    help='gate threshold from plan 05 Step 0.2.2 (default 15)')
    ap.add_argument('--prewarm', type=float, default=25.0,
                    help='seconds of GLOBAL load before ANY model is measured, to settle SoC '
                         'clocks (default 25). MEASURED 2026-09-06: without this the FIRST model '
                         'benchmarked is penalised ~15%% by cold clocks -- the same engine scored '
                         '42.8 FPS first and 49.1 FPS second. Set 0 only to reproduce that bug.')
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    missing = [m for m in map(os.path.expanduser, a.model) if not os.path.exists(m)]
    if missing:
        print("ERROR: engine(s) not found: %s" % ", ".join(missing))
        print("Engines are device-specific and must be built on this machine (R-F8-engine).")
        return 1

    print("imgsz=%d  duration=%.0fs/model  warmup=%d  person-class only"
          % (a.imgsz, a.duration, a.warmup))
    print("NOTE: isolated + synthetic. Never quote this as the system rate (see M1.4).\n")

    # GLOBAL pre-warm. Per-model warm-up is NOT enough: Jetson SoC/GPU clocks ramp over tens of
    # seconds, so whichever model runs first absorbs the penalty and looks slower than it is. This
    # invalidates cross-model comparison in a single run -- which is exactly how a 15 % ordering
    # artefact was once mistaken for a real per-model regression.
    if a.prewarm > 0:
        print("pre-warming for %.0fs to settle SoC clocks (see --prewarm) ..." % a.prewarm)
        _prewarm(os.path.expanduser(a.model[0]), a.prewarm, a.imgsz)

    results = []
    for m in map(os.path.expanduser, a.model):
        print("benchmarking %s ..." % os.path.basename(m))
        results.append(bench_one(m, a.duration, a.imgsz, a.warmup))

    print("")
    print("%-16s %8s %8s %8s %8s %8s %8s %8s" %
          ("model", "FPS", "p50 ms", "p95 ms", "pre", "infer", "post", "frames"))
    print("-" * 84)
    for r in results:
        print("%-16s %8.1f %8.2f %8.2f %8.2f %8.2f %8.2f %8d" %
              (r['model'], r['fps'], r['p50'], r['p95'], r['pre'], r['inf'], r['post'], r['frames']))

    print("")
    for r in results:
        overhead = r['pre'] + r['post']
        total = overhead + r['inf']
        drift = r['fps_second'] - r['fps_first']
        print("%s:" % r['model'])
        print("  throttling check : first half %.1f -> second half %.1f FPS (%+.1f)"
              % (r['fps_first'], r['fps_second'], drift))
        if r['temp_start'] is not None and r['temp_end'] is not None:
            print("  temperature      : %.0f -> %.0f C (%+.0f)"
                  % (r['temp_start'], r['temp_end'], r['temp_end'] - r['temp_start']))
        if total > 0:
            print("  pre+post share   : %.1f ms of %.1f ms (%.0f%%) -- the M1.4 optimisation lever,"
                  % (overhead, total, 100.0 * overhead / total))
            print("                     not model choice")
        margin = r['fps'] / a.target_hz if a.target_hz > 0 else 0.0
        verdict = "PASS" if r['fps'] >= a.target_hz else "FAIL"
        print("  gate (>=%.0f Hz)    : %s, %.1fx margin" % (a.target_hz, verdict, margin))
        if drift < -0.1 * r['fps_first']:
            print("  WARNING: >10%% slowdown across the run -- suspect thermal throttling.")
        print("")

    return 0 if all(r['fps'] >= a.target_hz for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
