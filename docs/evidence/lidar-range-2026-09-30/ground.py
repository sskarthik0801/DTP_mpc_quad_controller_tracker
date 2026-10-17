#!/usr/bin/env python3
"""Ground height along +x in lidar_init: the 5th percentile of points with |y| < 1.5, per 2 m.
    python3 ground.py <bag_dir>"""
import sys

import numpy as np

from _load import load

P, od_t, od = load(sys.argv[1].rstrip('/'))
odx = np.array([o[0] for o in od])
odz = np.array([o[2] for o in od])
print('x bin   ground z (p5 / p2 of pts, |y|<1.5)   n     aircraft z flown here')
for x0 in np.arange(-2, 58, 2.0):
    m = (P[:, 0] >= x0) & (P[:, 0] < x0 + 2) & (np.abs(P[:, 1]) < 1.5)
    a = (odx >= x0) & (odx < x0 + 2) & (odz > 0.5)
    if m.sum() < 20:
        print('%5.1f-%4.1f  (too few points: %d)' % (x0, x0 + 2, m.sum()))
        continue
    zs = P[m, 2]
    print('%5.1f-%4.1f  p5 %+.2f  p2 %+.2f   n %6d   %s' % (
        x0, x0 + 2, np.percentile(zs, 5), np.percentile(zs, 2), m.sum(),
        '%.2f-%.2f' % (odz[a].min(), odz[a].max()) if a.any() else '-'))
