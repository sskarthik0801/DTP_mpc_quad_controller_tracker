#!/usr/bin/env python3
"""Can the NX run the SUPER stack and the follower stack at once? Measure, don't guess.

THE QUESTION
------------
SUPER (FAST-LIO2 + ROG-Map + the planner FSM) and our follower (camera + YOLO + fusion) have
never run together on this box. Before designing around "it'll fit", establish whether it does --
and under which sensor configuration:

  A  depth from LiDAR only, RGB from camera
  B  depth from LiDAR **and** camera, RGB from camera

WHAT "IT FITS" ACTUALLY MEANS
-----------------------------
Not "CPU < 100%". A stack can sit at 70% CPU and still be broken, because the thing that matters
is whether each component HOLDS ITS RATE. So this measures rate compliance per topic alongside
the resource totals, and judges on the rates:

  /livox/lidar          ~10 Hz   sensor
  /cloud_registered     ~10 Hz   FAST-LIO keeping up with the LiDAR
  /lidar_slam/odom      ~10 Hz   odometry -- if this sags, SUPER is flying blind
  camera colour         ~30 Hz
  camera depth          ~30 Hz   (config B only)

A component that drops below ~85% of its nominal rate is failing, whatever the CPU says.

THE FOLLOWER LOAD IS A PROXY, AND SAYS SO
-----------------------------------------
`fusion_node` / `perception_node` are still M0.1 scaffolds -- they store parameters and do no
work. Measuring them would measure nothing. So the follower side is represented by its two real
costs, which ARE known: YOLO11s TensorRT inference at camera rate (13.6 ms/frame measured), and
the LiDAR->camera projection fusion will perform. That is an honest lower bound on the real
thing, not the real thing, and the report says so.

A NOTE ON THE CLOUD TOPIC
-------------------------
Fusion should consume **`/cloud_registered_body`** (FAST-LIO's deskewed cloud in the body frame),
not the raw driver output. Two reasons: FAST-LIO needs `/livox/lidar` as CustomMsg while a
projection needs PointCloud2 -- they cannot share the topic -- and the registered cloud is
already motion-compensated, which fusion_node's own docstring says it requires.
"""
import argparse
import re
import statistics
import subprocess
import sys
import time

# nominal Hz per topic, and how far below nominal still counts as healthy
EXPECT = {
    '/livox/lidar': 10.0,
    '/cloud_registered_body': 10.0,
    '/lidar_slam/odom': 10.0,
    '/camera/color/image_raw/compressed': 30.0,
    '/camera/depth/image_raw': 30.0,
}
RATE_FLOOR = 0.85

TEGRA = {
    'vdd_in_mw': re.compile(r'VDD_IN (\d+)mW'),
    'gr3d_pct': re.compile(r'GR3D_FREQ (\d+)%'),
    'tj_c': re.compile(r'tj@([\d.]+)C'),
    'ram_mb': re.compile(r'RAM (\d+)/'),
}
CPU_RE = re.compile(r'CPU \[([^\]]+)\]')


def sample_tegra(seconds=8.0):
    """Average tegrastats over a window. Returns dict of means plus peak tj."""
    p = subprocess.Popen(['tegrastats', '--interval', '1000'],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    acc = {k: [] for k in TEGRA}
    cpu = []
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            line = p.stdout.readline()
            if not line:
                break
            for k, rx in TEGRA.items():
                m = rx.search(line)
                if m:
                    acc[k].append(float(m.group(1)))
            m = CPU_RE.search(line)
            if m:
                loads = [int(x.split('%')[0]) for x in m.group(1).split(',') if '%' in x]
                if loads:
                    cpu.append(sum(loads) / len(loads))
    finally:
        p.terminate()
    out = {k: (statistics.mean(v) if v else 0.0) for k, v in acc.items()}
    out['tj_peak'] = max(acc['tj_c']) if acc['tj_c'] else 0.0
    out['cpu_mean_pct'] = statistics.mean(cpu) if cpu else 0.0
    out['cpu_peak_pct'] = max(cpu) if cpu else 0.0
    return out


# Topics whose payload is large enough that a PYTHON subscriber may fail to keep up on a loaded
# system. `ros2 topic hz` is Python, so a low or zero reading on these can be the TOOL, not the
# system. Measured 2026-09-07: /camera/depth/image_raw is 24 MB/s and read 0.00 Hz during a busy
# phase while the driver was publishing 28 Hz perfectly well.
HEAVY_TOPICS = ('/camera/depth/image_raw', '/camera/color/image_raw',
                '/camera/depth/points', '/cloud_registered', '/cloud_registered_body',
                '/livox/lidar')
# /livox/lidar is heavy in CustomMsg form too (~20k points, ~280 KB/msg). It read 8.33 Hz while
# FAST-LIO's OWN outputs held 9.98 and 10.09 Hz -- and a consumer cannot sustain 10 Hz from an
# 8.33 Hz input. That contradiction is the tell.
#
# GENERAL RULE, worth more than the topic list: when a heavy topic reads low, check whether
# anything DOWNSTREAM of it is holding its rate. If it is, the upstream is fine and the reading
# is the tool. Downstream health is a stronger signal than an upstream Python rate probe.

# Better still: don't probe the payload at all. A heavy stream usually has a TINY companion that
# shares its cadence exactly -- measured 2026-09-07 with sensor_watchdog,
# /camera/depth/camera_info ran 29.96 Hz against the image stream's 29.97, at 1/2000th the bytes.
# Probing the companion is cheaper AND more accurate, so prefer it wherever one exists.
COMPANION = {
    '/camera/depth/image_raw': '/camera/depth/camera_info',
    '/camera/color/image_raw': '/camera/color/camera_info',
}


def measure_rate(topic, seconds=10.0, retries=2):
    """Publish rate via `ros2 topic hz`. Returns (Hz, confident, probed_topic).

    `ros2 topic hz` is a Python subscriber, and on a multi-MB/s topic it can only ever
    UNDER-report -- it cannot invent messages it did not receive, but it very much can miss ones
    it did. That asymmetry decides what to believe:

      * a reading AT nominal is trustworthy whatever the topic weighs;
      * a LOW reading on a heavy topic is not -- it cannot separate a real stall from the tool's
        own deserialisation limit.

    On 2026-09-07 this bit twice: /camera/depth/image_raw read 0.00 Hz in one phase and 1.46 Hz in
    another, while its camera_info companion held 29.96 Hz. An earlier version of this function
    only distrusted an exact zero, so the 1.46 sailed through as a rate FAILURE. Both are the same
    artefact, and declaring either a failure is a false alarm -- which is how a test gets ignored.
    """
    probe = COMPANION.get(topic, topic)
    heavy = probe in HEAVY_TOPICS
    window = seconds * (2.0 if heavy else 1.0)
    best = 0.0
    for attempt in range(retries + 1):
        try:
            r = subprocess.run(['ros2', 'topic', 'hz', probe],
                               capture_output=True, text=True, timeout=window)
            out = r.stdout
        except subprocess.TimeoutExpired as e:
            out = (e.stdout or b'').decode() if isinstance(e.stdout, bytes) else (e.stdout or '')
        m = re.search(r'average rate:\s*([\d.]+)', out or '')
        if m:
            best = max(best, float(m.group(1)))
        if best > 0 and not heavy:
            return best, True, probe
        time.sleep(1.0)          # heavy: keep retrying and take the best, don't stop at the first
    if not heavy:
        return best, True, probe
    exp = EXPECT.get(topic, 0.0)
    return best, (best >= exp * RATE_FLOOR if exp else best > 0), probe


def phase(name, topics, tegra_secs, rate_secs):
    print("\n%s" % ("-" * 76))
    print("PHASE: %s" % name)
    print("-" * 76)
    res = sample_tegra(tegra_secs)
    print("  CPU mean %5.1f%%  peak %5.1f%%   RAM %5.0f MB   GPU %4.1f%%   tj %.1f C (peak %.1f)   %.1f W"
          % (res['cpu_mean_pct'], res['cpu_peak_pct'], res['ram_mb'], res['gr3d_pct'],
             res['tj_c'], res['tj_peak'], res['vdd_in_mw'] / 1000.0))
    rates, unsure = {}, set()
    for t in topics:
        hz, confident, probe = measure_rate(t, rate_secs)
        rates[t] = hz
        if not confident:
            unsure.add(t)
        shown = t if probe == t else '%s <- %s' % (t, probe.rsplit('/', 1)[-1])
        exp = EXPECT.get(t, 0.0)
        if exp:
            if not confident:
                tag = "?? tool-limited (heavy topic, Python hz under-reports under load)"
            elif hz >= exp * RATE_FLOOR:
                tag = "ok"
            else:
                tag = "*** LOW ***"
            print("    %-44s %6.2f Hz  (want %.0f)  %s" % (shown, hz, exp, tag))
        else:
            print("    %-44s %6.2f Hz" % (shown, hz))
    res['rates'] = rates
    res['unsure'] = unsure
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--label', required=True, help='what is running for this phase')
    ap.add_argument('--topics', default='', help='comma-separated topics to rate-check')
    ap.add_argument('--tegra_secs', type=float, default=15.0)
    ap.add_argument('--rate_secs', type=float, default=10.0)
    args = ap.parse_args(argv)

    topics = [t for t in args.topics.split(',') if t.strip()]
    r = phase(args.label, topics, args.tegra_secs, args.rate_secs)

    print("\n  VERDICT for this phase:")
    # A topic we could not measure confidently is NOT a failure -- see measure_rate.
    bad = [t for t, hz in r['rates'].items()
           if EXPECT.get(t) and hz < EXPECT[t] * RATE_FLOOR and t not in r['unsure']]
    if r['unsure']:
        print("    (could not measure confidently: %s -- verify separately, do not read as"
              % ', '.join(sorted(r['unsure'])))
        print("     failure; `ros2 topic hz` is Python and these topics are multi-MB/s)")
    if bad:
        print("    *** RATE FAILURE on: %s" % ', '.join(bad))
        print("    The stack does NOT fit in this configuration, whatever the CPU headroom says.")
    else:
        print("    all measured topics holding rate")
    if r['cpu_mean_pct'] > 80:
        print("    ! CPU mean %.0f%% -- little headroom for the real fusion/tracking work that"
              % r['cpu_mean_pct'])
        print("      the scaffolds do not yet do.")
    if r['tj_peak'] > 85:
        print("    ! tj peak %.1f C -- approaching throttle." % r['tj_peak'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
