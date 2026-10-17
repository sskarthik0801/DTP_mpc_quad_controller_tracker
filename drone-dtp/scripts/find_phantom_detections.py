#!/usr/bin/env python3
"""Find and SHOW the detections that are not people, by re-running the detector over a bag.

WHY THIS EXISTS
---------------
Measured 2026-09-09 on a 194 s recording with exactly ONE person in the room: 3435 detections
across 2037 frames -- **1.69 per frame** -- with two tracks living 95 s and 46 s simultaneously.
Something in that lab is being detected as a person, persistently, and target_manager has
therefore been CHOOSING AMONG CANDIDATES rather than tracking one human. That is a plausible
cause of every lock instability we have seen, and it is a real failure mode for any follower
deployed indoors: rooms contain pictures of people.

/dtp/targets carries no bounding box, so the bag cannot answer "what was it looking at". This
re-runs the detector over the recorded colour frames, which can.

HOW IT SEPARATES PHANTOMS FROM PEOPLE, AND WHY THAT TEST AND NOT CONFIDENCE
--------------------------------------------------------------------------
A poster of a person is a perfectly good person as far as a detector is concerned -- it will
often score HIGHER than a real human who is turning, partly occluded, or motion-blurred. So
confidence is the wrong discriminator and using it would filter the wrong thing.

What separates them is MOTION. A picture on a wall occupies the same pixels for the whole
recording; a human walking a 6 m line does not. So this clusters detections by image position
across the run and reports how far each cluster's centre ever travels. A cluster that never moves
is furniture.

It writes crops so the answer is visual and checkable, not a number to be trusted. If the crop
shows a poster, the fix is a static mask or a motion test. If it shows a real second person, there
is no bug and the lock logic simply needs to handle two candidates.

    python3 scripts/find_phantom_detections.py /tmp/bag_follow2 --out /tmp/phantoms
"""
import argparse
import collections
import glob
import math
import os
import sqlite3
import sys

STRIDE = 10               # every Nth colour frame; a wall does not move between frames
STATIC_RADIUS_PX = 60     # cluster centres within this are "the same thing"
STATIC_TRAVEL_PX = 90     # a cluster whose centre never travels further than this is furniture
MIN_CLUSTER_FRAMES = 8    # ignore one-off detections; we are hunting persistent things


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out', default='/tmp/phantoms')
    ap.add_argument('--model', default=os.path.expanduser('~/models/yolo11s.engine'))
    ap.add_argument('--conf', type=float, default=0.25)
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    import numpy as np
    import cv2
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from ultralytics import YOLO

    files = sorted(glob.glob(os.path.join(args.bag, '*.db3')))
    if not files:
        raise SystemExit('no .db3 in %s' % args.bag)
    con = sqlite3.connect(files[0])
    tid = {r[1]: r[0] for r in con.execute('SELECT id,name FROM topics').fetchall()}
    topic = '/camera/color/image_raw/compressed'
    if topic not in tid:
        raise SystemExit('bag has no %s' % topic)

    model = YOLO(args.model, task='detect')
    rows = con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',
                       (tid[topic],)).fetchall()
    print('colour frames in bag: %d, sampling every %d' % (len(rows), STRIDE))

    # cluster centre -> [n, sum_x, sum_y, min_x, max_x, min_y, max_y, best(conf, frame_idx, box)]
    clusters = []
    n_frames = n_det = 0
    for i in range(0, len(rows), STRIDE):
        _ts, data = rows[i]
        msg = deserialize_message(bytes(data), CompressedImage)
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        n_frames += 1
        res = model.predict(img, verbose=False, conf=args.conf)[0]
        for b in res.boxes:
            if int(b.cls[0]) != 0:                       # person class only
                continue
            n_det += 1
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            conf = float(b.conf[0])
            for c in clusters:
                if math.hypot(cx - c['cx'], cy - c['cy']) < STATIC_RADIUS_PX:
                    c['n'] += 1
                    c['xs'].append(cx); c['ys'].append(cy)
                    c['cx'] = sum(c['xs']) / len(c['xs'])
                    c['cy'] = sum(c['ys']) / len(c['ys'])
                    if conf > c['best'][0]:
                        c['best'] = (conf, i, (x1, y1, x2, y2))
                    break
            else:
                clusters.append(dict(n=1, cx=cx, cy=cy, xs=[cx], ys=[cy],
                                     best=(conf, i, (x1, y1, x2, y2))))

    print('sampled %d frames, %d person detections, %d clusters'
          % (n_frames, n_det, len(clusters)))
    print()
    print('CLUSTERS (a cluster that never travels is furniture, not a person)')
    print('  #   frames   centre         travel_px   best_conf   verdict')
    keep = [c for c in clusters if c['n'] >= MIN_CLUSTER_FRAMES]
    keep.sort(key=lambda c: -c['n'])
    for k, c in enumerate(keep):
        travel = max(max(c['xs']) - min(c['xs']), max(c['ys']) - min(c['ys']))
        static = travel < STATIC_TRAVEL_PX
        verdict = '*** STATIC -- PHANTOM ***' if static else 'moves (real person?)'
        print('  %-3d %6d   (%4.0f,%4.0f) %10.0f %11.2f   %s'
              % (k, c['n'], c['cx'], c['cy'], travel, c['best'][0], verdict))

        conf, fi, (x1, y1, x2, y2) = c['best']
        _ts, data = rows[fi]
        img = cv2.imdecode(np.frombuffer(
            deserialize_message(bytes(data), CompressedImage).data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        pad = 24
        h, w = img.shape[:2]
        crop = img[max(0, int(y1) - pad):min(h, int(y2) + pad),
                   max(0, int(x1) - pad):min(w, int(x2) + pad)]
        if crop.size:
            name = '%s_%02d_n%d_travel%03d_conf%.2f.png' % (
                'STATIC' if static else 'moving', k, c['n'], travel, conf)
            cv2.imwrite(os.path.join(args.out, name), crop)

    n_static = sum(1 for c in keep
                   if max(max(c['xs']) - min(c['xs']), max(c['ys']) - min(c['ys']))
                   < STATIC_TRAVEL_PX)
    print()
    print('%d of %d persistent clusters never move.' % (n_static, len(keep)))
    print('Crops written to %s -- LOOK AT THEM before choosing a filter.' % args.out)
    print('A poster scores as high as a person, so confidence is the wrong discriminator;')
    print('if these crops are wall art, the fix is a static mask or a motion test.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
