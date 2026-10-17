#!/usr/bin/env python3
"""Measure detection on LIVE camera frames -- the rate M1.4 actually needs.

WHY THIS EXISTS
---------------
`bench_yolo.py` carries its own disclaimer: "isolated + synthetic. Never quote this as the
system rate (see M1.4)." It feeds the engine a preallocated array in a tight loop. That
measures the engine, not the system.

The system rate is different in ways that matter for a follow controller:
  * frames arrive over ROS 2 at the camera's pace, not as fast as the GPU can chew them,
  * every frame costs a serialisation + copy the synthetic loop never pays,
  * the LiDAR driver, FAST-LIO and the camera driver compete for the same CPU,
  * real 1280x720 scenes are not the synthetic input the engine was warmed on.

So this subscribes to the real topic and reports three DIFFERENT things that are easy to
conflate:

  1. ARRIVAL rate  -- how fast frames actually show up (a camera/transport property)
  2. INFERENCE time -- how long one frame takes on the GPU (an engine property)
  3. THROUGHPUT     -- how many frames per second we actually finish (the system rate)

Throughput is capped by arrival rate. If the camera delivers 27 Hz, a 95 FPS engine still
yields 27 Hz -- and the useful finding is then the HEADROOM, not the FPS. Conversely if
throughput is below arrival rate we are dropping frames, which is the number that hurts.

It also measures END-TO-END AGE: from the image's own header stamp to the moment inference
finishes. That is what a controller actually consumes, and it includes transport delay the
other two miss entirely.

PREREQUISITES
-------------
Source the Jetson env first -- torch will not import without the libcudss LD_LIBRARY_PATH:
    source ~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh
GPU clocks should be pinned (`sudo jetson_clocks`), or the numbers are DVFS noise.
"""
import argparse
import statistics
import sys
import time


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--model', default='/home/ideas-ad/models/yolo11s.engine')
    ap.add_argument('--topic', default='/camera/color/image_raw')
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--raw', action='store_true',
                    help='use the raw Image topic instead of compressed (for A/B comparison)')
    ap.add_argument('--warmup', type=int, default=20)
    args = ap.parse_args(argv)

    try:
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage, Image
        from ultralytics import YOLO
        from dtp_perception import image_io
    except ImportError as e:
        print("ERROR: %s\n  Did you source dtp_env_jetson.sh? torch needs the libcudss path."
              % e, file=sys.stderr)
        return 2

    print("loading %s ..." % args.model)
    model = YOLO(args.model, task='detect')
    blank = np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8)
    for _ in range(args.warmup):
        model.predict(blank, imgsz=args.imgsz, classes=[0], verbose=False)
    print("warmed up (%d iters). listening on %s for %.0fs ...\n" % (
        args.warmup, args.topic, args.duration))

    rclpy.init()
    node = Node('measure_detect_live')

    state = {
        'arrivals': [],      # wall-clock arrival timestamps
        'infer_ms': [],      # pure inference time
        'age_ms': [],        # header stamp -> inference done
        'ndet': [],          # persons found per processed frame
        'received': 0,
        'processed': 0,
        'last_arrival': None,
        'decode_ms': [],
    }

    def on_image(msg):
        now = time.time()
        state['received'] += 1
        if state['last_arrival'] is not None:
            state['arrivals'].append(now - state['last_arrival'])
        state['last_arrival'] = now

        # image_io.decode() handles BOTH transports and always returns BGR, so switching
        # --raw cannot silently invert channel order.
        t_dec = time.perf_counter()
        try:
            img = image_io.decode(msg)
        except ValueError:
            return
        state['decode_ms'].append((time.perf_counter() - t_dec) * 1000.0)

        t0 = time.perf_counter()
        r = model.predict(img, imgsz=args.imgsz, classes=[0], verbose=False)
        dt = (time.perf_counter() - t0) * 1000.0

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0:
            # ROS time and time.time() are both wall clock here (no sim time in this stack).
            state['age_ms'].append((time.time() - stamp) * 1000.0)
        state['infer_ms'].append(dt)
        state['ndet'].append(int(len(r[0].boxes)) if r and r[0].boxes is not None else 0)
        state['processed'] += 1

    topic, mtype = image_io.image_topic_and_type(args.topic, use_compressed=not args.raw)
    print("  transport: %s  (%s)" % (topic, mtype.__name__))
    node.create_subscription(mtype, topic, on_image, qos_profile_sensor_data)

    t_start = time.time()
    try:
        while time.time() - t_start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = time.time() - t_start
    node.destroy_node()
    rclpy.shutdown()

    return report(state, elapsed, args)


def _pct(v, p):
    if not v:
        return 0.0
    s = sorted(v)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def report(st, elapsed, args):
    n = st['processed']
    if n < 10:
        print("\n*** TOO FEW FRAMES (%d) -- is the camera publishing on %s? ***"
              % (n, args.topic))
        return 1

    arr = st['arrivals']
    inf = st['infer_ms']
    age = st['age_ms']

    arrival_hz = (1.0 / statistics.mean(arr)) if arr else 0.0
    throughput = n / elapsed
    infer_mean = statistics.mean(inf)
    engine_ceiling = 1000.0 / infer_mean if infer_mean else 0.0

    print("=" * 74)
    print("LIVE DETECTION  (%s, %s, %.0fs)" % (
        args.model.split('/')[-1], args.topic, elapsed))
    print("=" * 74)
    print("  1. ARRIVAL   frames received %d  -> %.1f Hz mean" % (st['received'], arrival_hz))
    if arr:
        print("     inter-arrival  p50 %.1f ms  p95 %.1f ms  max %.1f ms"
              % (_pct(arr, 50) * 1000, _pct(arr, 95) * 1000, max(arr) * 1000))
    dec = st.get('decode_ms') or []
    if dec:
        print("  1b. DECODE   mean %.1f ms  p95 %.1f  max %.1f" % (
            statistics.mean(dec), _pct(dec, 95), max(dec)))
    print("  2. INFERENCE mean %.1f ms  p50 %.1f  p95 %.1f  max %.1f   -> engine ceiling %.1f FPS"
          % (infer_mean, _pct(inf, 50), _pct(inf, 95), max(inf), engine_ceiling))
    if age:
        print("  3. END-TO-END AGE (header stamp -> inference done)")
        print("     mean %.1f ms  p50 %.1f  p95 %.1f  max %.1f"
              % (statistics.mean(age), _pct(age, 50), _pct(age, 95), max(age)))
    print("  4. THROUGHPUT processed %d -> %.1f FPS   (dropped %d of %d received, %.1f%%)"
          % (n, throughput, st['received'] - n, st['received'],
             100.0 * (st['received'] - n) / max(st['received'], 1)))

    det = st['ndet']
    frames_with = sum(1 for d in det if d > 0)
    print("  5. DETECTIONS person in %d/%d frames (%.0f%%), mean %.2f per frame"
          % (frames_with, n, 100.0 * frames_with / n, statistics.mean(det)))

    print("\n  READ IT THIS WAY:")
    if throughput < arrival_hz * 0.9:
        print("  * THROUGHPUT IS BELOW ARRIVAL -- frames are being dropped. Inference or the")
        print("    ROS callback is the bottleneck, and the controller sees a stale world.")
    else:
        print("  * Throughput tracks arrival: the engine keeps up and the CAMERA is the limit.")
        print("    Extra engine speed buys nothing here; headroom is %.1fx (%.1f vs %.1f FPS)."
              % (engine_ceiling / max(arrival_hz, 1e-6), engine_ceiling, arrival_hz))
    if age and _pct(age, 95) > 150:
        print("  * p95 end-to-end age is %.0f ms. For a person walking at 1.5 m/s that is %.2f m"
              % (_pct(age, 95), 1.5 * _pct(age, 95) / 1000.0))
        print("    of position error before the planner even sees the frame.")
    if arr and max(arr) > 0.2:
        print("  * WORST inter-arrival gap %.0f ms -- a stall this long is a real hazard for a"
              % (max(arr) * 1000))
        print("    follow loop; investigate before trusting the mean.")
    if frames_with == 0:
        print("  * NO PERSON DETECTED in any frame. If someone was in view, that is the finding;")
        print("    if not, this run measured throughput only, not detection quality.")
    print("=" * 74)
    return 0


if __name__ == '__main__':
    sys.exit(main())
