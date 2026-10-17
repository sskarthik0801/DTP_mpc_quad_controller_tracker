#!/usr/bin/env python3
"""Where does JPEG quality actually start costing detections? Same frames, quality swept.

WHY
---
We pinned the driver to `color_mjpeg_quality: 100` and measured detection parity against raw
over 1180 person-frames -- zero disagreements. But that did NOT prove quality was the reason:
the earlier run that showed a 20 % loss also had a different (15-frame) sample, so quality and
sample size changed together and neither could be blamed.

This isolates quality. Every level re-encodes the SAME raw frames pulled from a bag, so the
scene, the distances, the lighting and the person are identical across the whole sweep. The
only thing that varies is the compression.

WHAT IT ANSWERS
---------------
  * how much margin does q100 actually buy -- does detection survive to 70? to 40?
  * where is the knee, so a future bandwidth decision is informed rather than superstitious
  * how many bytes each quality costs, so the trade is visible in both directions

Detection on the UNCOMPRESSED raw frame is the reference. A "loss" is a frame where raw finds a
person and the encoded version does not; a "gain" is the reverse (compression artefacts can
occasionally push a marginal detection UP, which is noise, not benefit).

NOTE ON THE DRIVER'S KNOB: `color_mjpeg_quality` is a TARGET SIZE, not the standard quality
factor cv2 uses here. The two scales are not interchangeable -- compare by BYTES, not by number.
The driver at q100 emits 168212 bytes/frame; find that figure in the bytes column to locate
where the live stream actually sits on this curve.
"""
import argparse
import statistics
import sys

RAW_TOPIC = '/camera/color/image_raw'
MATCH_IOU = 0.30
DEFAULT_QUALITIES = [100, 95, 90, 85, 75, 65, 50, 40, 30, 20]


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
            out.append(([float(v) for v in b.xyxy[0].tolist()], float(b.conf[0])))
    return sorted(out, key=lambda t: -t[1])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True)
    ap.add_argument('--model', default='/home/ideas-ad/models/yolo11s.engine')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--stride', type=int, default=4,
                    help='use every Nth frame (default 4) -- the sweep is qualities x frames')
    ap.add_argument('--qualities', default=','.join(str(q) for q in DEFAULT_QUALITIES))
    args = ap.parse_args(argv)

    try:
        import cv2
        import numpy as np
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        from ultralytics import YOLO
        from dtp_perception import image_io
    except ImportError as e:
        print("ERROR: %s\n  Source dtp_env_jetson.sh and the workspace." % e, file=sys.stderr)
        return 2

    qualities = [int(q) for q in args.qualities.split(',') if q.strip()]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    frames = []
    i = 0
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != RAW_TOPIC:
            continue
        if i % args.stride == 0:
            frames.append(image_io.decode(deserialize_message(data, Image)))
        i += 1
    if not frames:
        print("ERROR: no %s messages in the bag" % RAW_TOPIC, file=sys.stderr)
        return 1
    print("loaded %d raw frames (every %dth of %d)" % (len(frames), args.stride, i))

    print("loading %s ..." % args.model)
    model = YOLO(args.model, task='detect')
    blank = np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8)
    for _ in range(20):
        model.predict(blank, imgsz=args.imgsz, classes=[0], verbose=False)

    # Reference: detection on the UNCOMPRESSED frames.
    ref = [detect(model, f, args.imgsz, args.conf) for f in frames]
    ref_pos = sum(1 for d in ref if d)
    print("reference (uncompressed): %d of %d frames contain a person\n"
          % (ref_pos, len(frames)))
    if ref_pos == 0:
        print("ERROR: no person in ANY reference frame -- this bag cannot answer the question.",
              file=sys.stderr)
        return 1

    print("%-6s %10s %8s %8s %8s %9s %9s" %
          ('qual', 'bytes', 'lost', 'gained', 'net%', 'IoU', 'dconf'))
    print("-" * 64)
    rows = []
    for q in qualities:
        sizes, lost, gained, ious, dconf = [], 0, 0, [], []
        for f, dr in zip(frames, ref):
            ok, buf = cv2.imencode('.jpg', f, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if not ok:
                continue
            sizes.append(len(buf))
            dq = detect(model, cv2.imdecode(buf, cv2.IMREAD_COLOR), args.imgsz, args.conf)
            if dr and not dq:
                lost += 1
            if dq and not dr:
                gained += 1
            used = set()
            for box_r, conf_r in dr:
                best, bj = 0.0, None
                for j, (box_q, _c) in enumerate(dq):
                    if j in used:
                        continue
                    v = iou(box_r, box_q)
                    if v > best:
                        best, bj = v, j
                if bj is not None and best >= MATCH_IOU:
                    used.add(bj)
                    ious.append(best)
                    dconf.append(dq[bj][1] - conf_r)
        pct = 100.0 * lost / ref_pos
        rows.append((q, statistics.mean(sizes) if sizes else 0, lost, gained, pct,
                     statistics.mean(ious) if ious else 0.0,
                     statistics.mean(dconf) if dconf else 0.0))
        print("%-6d %10.0f %8d %8d %7.1f%% %9.4f %+9.4f" % rows[-1])

    print("-" * 64)
    print("  lost   = raw found a person, this quality did not (out of %d person-frames)" % ref_pos)
    print("  gained = the reverse; treat as noise, not benefit")
    print("  IoU / dconf are over MATCHED detections only\n")

    clean = [r for r in rows if r[2] == 0]
    if clean:
        knee = min(clean, key=lambda r: r[0])
        print("  LOWEST LOSSLESS QUALITY: %d  (%.0f bytes/frame)" % (knee[0], knee[1]))
        print("  The driver's q100 emits 168212 bytes/frame; compare by BYTES, not by the number")
        print("  -- its knob is a target size, not this quality factor.")
        if knee[0] <= 50:
            print("  Detection survives aggressive compression here, so bandwidth is available")
            print("  if a future need (a second camera, a downlink) calls for it.")
    else:
        print("  NO tested quality was lossless. Compression costs detections even at the top of")
        print("  the range -- prefer raw where bandwidth allows, and re-examine the pipeline.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
