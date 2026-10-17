#!/usr/bin/env python3
"""Does JPEG compression change DETECTION? Same frames, both transports, box by box.

WHY THIS AND NOT A LIVE A/B
---------------------------
Two live runs cannot answer this. Detection depends on what is in front of the camera, and two
sequential 45 s runs see different scenes -- our first attempt showed a person in 33 % of raw
frames and 100 % of compressed frames, which says nothing about the codec and everything about
who happened to be standing there. Asking someone to hold still identically for two runs does
not fix it either.

A bag does. Record `/camera/color/image_raw` and `/camera/color/image_raw/compressed`
simultaneously and both topics carry the SAME source frame, JPEG-encoded or not. Reading the bag
directly (rather than replaying it) also removes replay timing and drops from the comparison
entirely: every frame is processed, deterministically, as fast as the GPU allows.

WHAT IT MEASURES
----------------
Per matched frame, YOLO is run on both decodings and the results compared:
  * count agreement -- does either transport find a person the other misses?
  * confidence delta on matched detections
  * bounding-box IoU on matched detections

Counts alone would be too coarse: compression could shift a box or shave confidence without
changing how many people are found, and a follow controller cares about WHERE the person is,
not just that one exists.

Matching is by header stamp -- both topics are published from one source frame, so the stamps
are identical and there is no tolerance to tune.
"""
import argparse
import statistics
import sys

RAW_TOPIC = '/camera/color/image_raw'
CMP_TOPIC = '/camera/color/image_raw/compressed'
# Two boxes overlapping this much are treated as the same detection. Generous on purpose: the
# question is whether compression MOVES a box, so the pairing must not itself reject shifts.
MATCH_IOU = 0.30


def iou(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    ua = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / ua if ua > 0 else 0.0


def detect(model, img, imgsz, conf):
    r = model.predict(img, imgsz=imgsz, classes=[0], conf=conf, verbose=False)
    out = []
    if r and r[0].boxes is not None and len(r[0].boxes):
        for b in r[0].boxes:
            xy = [float(v) for v in b.xyxy[0].tolist()]
            out.append((xy, float(b.conf[0])))
    # Sort by confidence so greedy pairing starts from the most certain detection.
    return sorted(out, key=lambda t: -t[1])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True)
    ap.add_argument('--model', default='/home/ideas-ad/models/yolo11s.engine')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--limit', type=int, default=0, help='stop after N matched frames (0 = all)')
    args = ap.parse_args(argv)

    try:
        import numpy as np
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import CompressedImage, Image
        from ultralytics import YOLO
        from dtp_perception import image_io
    except ImportError as e:
        print("ERROR: %s\n  Source dtp_env_jetson.sh (torch needs libcudss) and the workspace."
              % e, file=sys.stderr)
        return 2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    raw, cmp_ = {}, {}
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == RAW_TOPIC:
            m = deserialize_message(data, Image)
            raw[(m.header.stamp.sec, m.header.stamp.nanosec)] = m
        elif topic == CMP_TOPIC:
            m = deserialize_message(data, CompressedImage)
            cmp_[(m.header.stamp.sec, m.header.stamp.nanosec)] = m

    keys = sorted(set(raw) & set(cmp_))
    print("bag: %d raw, %d compressed, %d MATCHED by header stamp"
          % (len(raw), len(cmp_), len(keys)))
    if not keys:
        print("\nERROR: no frames matched by stamp. Were both topics recorded together?",
              file=sys.stderr)
        return 1
    if args.limit:
        keys = keys[:args.limit]

    print("loading %s ..." % args.model)
    model = YOLO(args.model, task='detect')
    blank = np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8)
    for _ in range(20):
        model.predict(blank, imgsz=args.imgsz, classes=[0], verbose=False)

    n_raw_only = n_cmp_only = n_both_none = n_agree_count = 0
    ious, dconf, raw_counts, cmp_counts = [], [], [], []

    for i, k in enumerate(keys):
        dr = detect(model, image_io.decode(raw[k]), args.imgsz, args.conf)
        dc = detect(model, image_io.decode(cmp_[k]), args.imgsz, args.conf)
        raw_counts.append(len(dr))
        cmp_counts.append(len(dc))
        if len(dr) == len(dc):
            n_agree_count += 1
        if not dr and not dc:
            n_both_none += 1
        if dr and not dc:
            n_raw_only += 1
        if dc and not dr:
            n_cmp_only += 1

        # Greedy highest-confidence-first pairing.
        used = set()
        for box_r, conf_r in dr:
            best, best_j = 0.0, None
            for j, (box_c, _cc) in enumerate(dc):
                if j in used:
                    continue
                v = iou(box_r, box_c)
                if v > best:
                    best, best_j = v, j
            if best_j is not None and best >= MATCH_IOU:
                used.add(best_j)
                ious.append(best)
                dconf.append(dc[best_j][1] - conf_r)

        if (i + 1) % 100 == 0:
            print("  %d/%d frames ..." % (i + 1, len(keys)))

    n = len(keys)
    print("\n" + "=" * 74)
    print("TRANSPORT DETECTION PARITY  (%d identical frames, %s)" % (n, args.model.split('/')[-1]))
    print("=" * 74)
    print("  persons found   raw %d total, compressed %d total" % (sum(raw_counts), sum(cmp_counts)))
    print("  frames with a person   raw %d, compressed %d"
          % (sum(1 for c in raw_counts if c), sum(1 for c in cmp_counts if c)))
    print("  count agreement        %d/%d frames (%.1f%%)" % (n_agree_count, n, 100.0 * n_agree_count / n))
    print("  frames neither found anything : %d" % n_both_none)
    print("  DISAGREEMENTS  raw-only %d   compressed-only %d" % (n_raw_only, n_cmp_only))
    if ious:
        print("\n  matched detections: %d" % len(ious))
        print("    IoU        mean %.4f  min %.4f  p05 %.4f"
              % (statistics.mean(ious), min(ious), sorted(ious)[max(0, len(ious) // 20)]))
        print("    conf delta (compressed - raw)  mean %+.4f  max |%.4f|"
              % (statistics.mean(dconf), max(abs(d) for d in dconf)))
    print("\n  VERDICT:")
    # A comparison with nothing to detect is NOT a pass. "0 disagreements" across frames that
    # contain no person says nothing about whether compression harms detection -- the same
    # vacuous-result trap as a benchmark whose load never started.
    if sum(raw_counts) == 0 and sum(cmp_counts) == 0:
        print("  * INCONCLUSIVE -- NO PERSON DETECTED IN ANY FRAME, on either transport.")
        print("    This run measured nothing about parity. Re-record with someone in view,")
        print("    ideally at the standoff distances that matter (2-4 m) and beyond.")
        print("=" * 74)
        return 1
    # Normalise over frames where EITHER transport saw a person, never over all frames.
    # Empty frames agree for free: in the 2026-09-07 bag, 957 of 972 contained nobody, and
    # scoring against 972 turned a 20 % miss rate into a reassuring "99.6 % agreement".
    positives = sum(1 for a, b in zip(raw_counts, cmp_counts) if a or b)
    disagree = n_raw_only + n_cmp_only
    print("  * SCORED OVER FRAMES CONTAINING A PERSON (%d), not all %d -- empty frames agree"
          % (positives, n))
    print("    for free and would dilute any real miss into invisibility.")
    if positives < 50:
        print("  * ⚠️  SMALL SAMPLE: only %d frames contain a person. Treat every rate below as"
              % positives)
        print("    indicative, not conclusive. Re-record with sustained presence at 2-4 m+.")
    if disagree == 0 and ious and statistics.mean(ious) > 0.95:
        print("  * PARITY over this sample: compression changed no detection outcome, and matched")
        print("    boxes overlap almost exactly.")
    else:
        pct = 100.0 * disagree / positives if positives else 0.0
        print("  * COMPRESSION CHANGED THE OUTCOME on %d of %d person-frames (%.1f%%)."
              % (disagree, positives, pct))
        print("    raw-only %d (compression LOST a detection), compressed-only %d."
              % (n_raw_only, n_cmp_only))
        if ious and statistics.mean(ious) > 0.95:
            print("    Note the matched boxes agree almost exactly (IoU %.3f), so this is not"
                  % statistics.mean(ious))
            print("    boxes moving -- it is MARGINAL detections crossing the confidence")
            print("    threshold. Expect it to bite hardest on distant or partly-occluded people.")
    print("=" * 74)
    return 0


if __name__ == '__main__':
    sys.exit(main())
