#!/usr/bin/env python3
"""compare_replay_targets.py -- pairwise agreement of replay_perception.sh runs.

    python3 scripts/compare_replay_targets.py <out_dir> numpyA numpyB cudaA cudaB

For every pair of labels: on the colour stamps BOTH runs published, the per-target max absolute
difference over (x, y, z, distance_m), matched by sorted order within a frame. Compare
cross-arm pairs against the WITHIN-arm pairs, never against zero -- see replay_perception.sh for
why a replay is not frame-deterministic. Also prints publish counts, perception_ok and CPU.
"""
import itertools
import json
import os
import sys

import numpy as np


def load(out_dir, label):
    targets, ok = {}, []
    with open(os.path.join(out_dir, 'targets_%s.jsonl' % label)) as f:
        for line in f:
            rec = json.loads(line)
            if rec['k'] == 't':
                targets[round(rec['s'], 6)] = rec['targets']
            else:
                ok.append(rec['v'])
    cpu_path = os.path.join(out_dir, 'cpu_%s.txt' % label)
    cpu = open(cpu_path).read().strip() if os.path.exists(cpu_path) else ''
    return targets, ok, cpu


def diffs(a, b):
    out = []
    for stamp in sorted(set(a) & set(b)):
        ta, tb = sorted(a[stamp]), sorted(b[stamp])
        if not ta or len(ta) != len(tb):
            continue
        out += [float(np.max(np.abs(np.array(p[1:5]) - np.array(q[1:5])))) for p, q in zip(ta, tb)]
    return np.array(out)


def main():
    out_dir, labels = sys.argv[1], sys.argv[2:]
    runs = {lab: load(out_dir, lab) for lab in labels}
    for lab, (targets, ok, cpu) in runs.items():
        print('%-8s %4d TargetArrays, %4d with a person, perception_ok %d/%d   %s'
              % (lab, len(targets), sum(1 for v in targets.values() if v), sum(ok), len(ok), cpu))
    print()
    for x, y in itertools.combinations(labels, 2):
        d = diffs(runs[x][0], runs[y][0])
        if d.size == 0:
            print('%8s vs %-8s no common non-empty stamps' % (x, y))
            continue
        print('%8s vs %-8s n=%3d  median %5.2f cm  p95 %5.2f cm  max %5.2f cm  exact %d'
              % (x, y, d.size, np.median(d) * 100, np.percentile(d, 95) * 100, d.max() * 100,
                 int(np.sum(d == 0))))


if __name__ == '__main__':
    main()
