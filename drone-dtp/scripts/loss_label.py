#!/usr/bin/env python3
"""M6.2 labelling tool: ground-truth the WHY of every recorded loss, then score the oracle (D1).

  loss_label.py sheets BAG EVENTS.jsonl --out DIR [--window 2.0] [--gui]
      For every coast episode in EVENTS.jsonl (from loss_replay.py) write one contact sheet of
      the colour frames +-window s around the coast onset -- twelve thumbnails, the onset frame
      marked, the oracle's verdict and the recorded outcome in the header -- and a labels.csv
      with one row per episode and an empty `label` column. With --gui (needs a display: run it
      on the laptop, or with X forwarding) each sheet is shown and ONE keypress labels it:
          o  OCCLUDED      the person went behind mapped geometry (trunk, tube, wall, furniture)
          l  LOST          clear view: walked out of range, detector miss, crouch/turn
          v  OUT_OF_VIEW   left the camera's cone (behind the drone, above/below the frame)
          u  UNMAPPED      went into space the map has never seen (round a corner, into the dark)
          s  skip / unsure     q  quit (labels so far are kept)
      Without a display, open the PNGs anywhere and fill labels.csv by hand. --tile-width (default
      480 px) sets the thumbnail size; 320 was too small to tell a chair from a person at 6 m.

  loss_label.py score LABELS.csv EVENTS.jsonl
      The D1 numbers (docs/m5-m6-plan.md SS2.2): confusion matrix of the oracle's ONSET verdict
      (and its majority verdict over the coast) against the labels; E1 OCCLUDED recall; E2
      false-OCCLUDED rate on LOST-labelled episodes; E3 OUT_OF_VIEW correct rate.

The onset frame is chosen by the bag time of the FIRST coasting /dtp/follow_target of the
episode (events.jsonl's bag_t), which is one detection period after the last frame the person
was seen in -- the frame that answers "why did the detector stop seeing them" is the one just
BEFORE that, so look at the marked frame and the two to its left.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

CLS = ['OCCLUDED', 'UNMAPPED', 'LOST', 'OUT_OF_VIEW']
KEYS = {'o': 'OCCLUDED', 'l': 'LOST', 'v': 'OUT_OF_VIEW', 'u': 'UNMAPPED'}


def load_episodes(events_path):
    eps = {}
    with open(events_path) as f:
        for line in f:
            d = json.loads(line)
            eps.setdefault(d['episode'], []).append(d)
    out = []
    for ep, ds in sorted(eps.items()):
        first = ds[0]
        counts = {c: sum(1 for d in ds if d['cls_name'] == c) for c in CLS}
        out.append({'episode': ep, 'track_id': first['track_id'], 'onset_bag_t': first['bag_t'],
                    'first': first['cls_name'], 'majority': max(counts, key=counts.get),
                    'blocked': first['blocked_fraction'], 'unknown': first['unknown_fraction'],
                    'n': len(ds), 'events': ds})
    return out


def read_recorded_outcomes(episodes_csv):
    out = {}
    if not os.path.exists(episodes_csv):
        return out
    with open(episodes_csv) as f:
        for r in csv.DictReader(f):
            out[int(r['episode'])] = (r['recorded_outcome'], r['recorded_outcome_dt'])
    return out


def sheets(args):
    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    eps = [e for e in load_episodes(args.events) if e['onset_bag_t'] is not None]
    outcomes = read_recorded_outcomes(os.path.join(os.path.dirname(args.events), 'episodes.csv'))
    os.makedirs(args.out, exist_ok=True)
    windows = [(e['onset_bag_t'] - args.window, e['onset_bag_t'] + args.window, e['episode']) for e in eps]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.image_topic]))
    frames = {e['episode']: [] for e in eps}     # episode -> [(rel_t, jpeg bytes)]
    bag_t0 = None
    # bag_t0 in loss_replay is the first message of ITS topic set; the image stream starts at
    # about the same time, so align on the bag's own metadata start instead.
    meta = os.path.join(args.bag, 'metadata.yaml')
    if os.path.exists(meta):
        for line in open(meta):
            if 'nanoseconds_since_epoch' in line:
                bag_t0 = int(line.split(':')[1]) * 1e-9
                break
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if bag_t0 is None:
            bag_t0 = t
        rel = t - bag_t0
        for lo, hi, ep in windows:
            if lo <= rel <= hi:
                frames[ep].append((rel, data))
    labels_path = os.path.join(args.out, 'labels.csv')
    existing = {}
    if os.path.exists(labels_path):
        with open(labels_path) as f:
            for r in csv.DictReader(f):
                existing[int(r['episode'])] = r
    rows = []
    for e in eps:
        fr = frames[e['episode']]
        if not fr:
            continue
        idx = np.linspace(0, len(fr) - 1, min(12, len(fr))).astype(int)
        onset_i = int(np.argmin([abs(fr[i][0] - e['onset_bag_t']) for i in idx]))
        tiles = []
        for j, i in enumerate(idx):
            rel, data = fr[i]
            m = deserialize_message(data, CompressedImage)
            img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                img = np.zeros((args.tile_width * 9 // 16, args.tile_width, 3), np.uint8)
            tw, th = args.tile_width, args.tile_width * 9 // 16
            img = cv2.resize(img, (tw, th))
            col = (0, 0, 255) if j == onset_i else (255, 255, 255)
            cv2.putText(img, '%+.2f s%s' % (rel - e['onset_bag_t'], '  ONSET' if j == onset_i else ''),
                        (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
            if j == onset_i:
                cv2.rectangle(img, (0, 0), (tw - 1, th - 1), col, 3)
            tiles.append(img)
        while len(tiles) < 12:
            tiles.append(np.zeros((args.tile_width * 9 // 16, args.tile_width, 3), np.uint8))
        grid = np.vstack([np.hstack(tiles[:6]), np.hstack(tiles[6:12])])
        header = np.zeros((44, grid.shape[1], 3), np.uint8)
        rec = outcomes.get(e['episode'], ('?', ''))
        cv2.putText(header, 'episode %d | track %d | onset %.1f s | oracle: %s at onset (blocked %.2f, unknown %.2f),'
                    ' majority %s over %d | recorded: %s %s'
                    % (e['episode'], e['track_id'], e['onset_bag_t'], e['first'], e['blocked'], e['unknown'],
                       e['majority'], e['n'], rec[0], ('after %s s' % rec[1]) if rec[1] else ''),
                    (8, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        sheet = np.vstack([header, grid])
        png = os.path.join(args.out, 'episode_%03d.png' % e['episode'])
        cv2.imwrite(png, sheet)
        row = existing.get(e['episode'], {})
        rows.append({'episode': e['episode'], 'track_id': e['track_id'], 'onset_bag_t': round(e['onset_bag_t'], 3),
                     'oracle_first': e['first'], 'oracle_majority': e['majority'],
                     'recorded_outcome': rec[0], 'label': row.get('label', ''), 'notes': row.get('notes', ''),
                     'sheet': os.path.basename(png)})
        if args.gui:
            cv2.imshow('loss_label  (o l v u s q)', sheet)
            while True:
                k = cv2.waitKey(0) & 0xFF
                ch = chr(k) if k < 256 else ''
                if ch == 'q':
                    args.gui = False
                    break
                if ch == 's':
                    break
                if ch in KEYS:
                    rows[-1]['label'] = KEYS[ch]
                    break
    with open(labels_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['episode', 'track_id', 'onset_bag_t', 'oracle_first', 'oracle_majority',
                                          'recorded_outcome', 'label', 'notes', 'sheet'])
        w.writeheader()
        w.writerows(rows)
    print('%d sheets in %s; labels in %s (%d labelled)'
          % (len(rows), args.out, labels_path, sum(1 for r in rows if r['label'])))
    return 0


def score(args):
    """Labels are matched to episodes by ONSET TIME (within 0.25 s), not by episode index, so a
    labels.csv made from one replay scores the events of another replay of the same bag (e.g. a
    re-run with --virtual-ground); episode numbering can shift by a dropped message."""
    labels = {}
    with open(args.labels) as f:
        for r in csv.DictReader(f):
            if r['label'].strip():
                labels[float(r['onset_bag_t'])] = r['label'].strip().upper()
    by_t = [(e['onset_bag_t'], e) for e in load_episodes(args.events) if e['onset_bag_t'] is not None]
    eps = {}
    unmatched = 0
    for t, lab in labels.items():
        best = min(by_t, key=lambda te: abs(te[0] - t), default=None)
        if best is None or abs(best[0] - t) > 0.25:
            unmatched += 1
            continue
        eps[t] = best[1]
    if not labels:
        print('no labels in %s' % args.labels)
        return 1
    print('%d labels, %d matched to episodes in %s%s'
          % (len(labels), len(eps), args.events, '' if not unmatched else ', %d UNMATCHED' % unmatched))
    for key in ('first', 'majority'):
        conf = {a: {b: 0 for b in CLS} for a in CLS}
        for t, lab in labels.items():
            if t in eps and lab in CLS:
                conf[lab][eps[t][key]] += 1
        print('\noracle %s verdict vs label (rows = label, cols = oracle):' % key)
        print('%-13s' % '' + ''.join('%13s' % c for c in CLS) + '%8s' % 'n')
        for a in CLS:
            n = sum(conf[a].values())
            print('%-13s' % a + ''.join('%13d' % conf[a][b] for b in CLS) + '%8d' % n)
        n_occ = sum(conf['OCCLUDED'].values())
        n_lost = sum(conf['LOST'].values())
        n_oov = sum(conf['OUT_OF_VIEW'].values())
        if n_occ:
            print('E1 OCCLUDED recall        %.2f (%d/%d)  target >= 0.85'
                  % (conf['OCCLUDED']['OCCLUDED'] / n_occ, conf['OCCLUDED']['OCCLUDED'], n_occ))
        if n_lost:
            print('E2 false-OCCLUDED on LOST %.2f (%d/%d)  target <= 0.10'
                  % (conf['LOST']['OCCLUDED'] / n_lost, conf['LOST']['OCCLUDED'], n_lost))
        if n_oov:
            print('E3 OUT_OF_VIEW correct    %.2f (%d/%d)  target >= 0.90'
                  % (conf['OUT_OF_VIEW']['OUT_OF_VIEW'] / n_oov, conf['OUT_OF_VIEW']['OUT_OF_VIEW'], n_oov))
        total = sum(sum(r.values()) for r in conf.values())
        correct = sum(conf[c][c] for c in CLS)
        print('overall %.2f (%d/%d)' % (correct / max(total, 1), correct, total))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('sheets')
    s.add_argument('bag')
    s.add_argument('events')
    s.add_argument('--out', required=True)
    s.add_argument('--window', type=float, default=2.0)
    s.add_argument('--image-topic', default='/camera/color/image_raw/compressed')
    s.add_argument('--tile-width', type=int, default=480)
    s.add_argument('--gui', action='store_true')
    s.set_defaults(func=sheets)
    c = sub.add_parser('score')
    c.add_argument('labels')
    c.add_argument('events')
    c.set_defaults(func=score)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
