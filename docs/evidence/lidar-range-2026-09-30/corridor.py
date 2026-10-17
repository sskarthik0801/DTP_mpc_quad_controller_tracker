#!/usr/bin/env python3
"""ASCII top-down obstacle map of the venue in lidar_init, x -4..70, y +-8, 0.5 m cells.
# = >= 15 points 0.6-4 m above the local ground, . = ground seen, | = x 28/40/50/60, - = y +-2.
    python3 corridor.py <bag_dir>"""
import sys
from collections import defaultdict

import numpy as np

from _load import load

P, _, _ = load(sys.argv[1].rstrip('/'))
C, X0, X1, Y0, Y1 = 0.5, -4, 70, -8, 8
NX, NY = int((X1 - X0) / C), int((Y1 - Y0) / C)
ix = np.floor((P[:, 0] - X0) / C).astype(int)
iy = np.floor((P[:, 1] - Y0) / C).astype(int)
ok = (ix >= 0) & (ix < NX) & (iy >= 0) & (iy < NY)
P, ix, iy = P[ok], ix[ok], iy[ok]
cells = defaultdict(list)
for k in range(len(P)):
    cells[(ix[k], iy[k])].append(P[k, 2])
ground = np.full((NX, NY), np.nan)
seen = np.zeros((NX, NY), bool)
for (a, b), zs in cells.items():
    seen[a, b] = True
    ground[a, b] = np.percentile(zs, 5)
occ = np.zeros((NX, NY), int)
for (a, b), zs in cells.items():
    g = np.nanmin(ground[max(0, a - 2):a + 3, max(0, b - 2):b + 3])
    zs = np.array(zs)
    occ[a, b] = int(np.count_nonzero((zs > g + 0.6) & (zs < g + 4.0)))
for b in range(NY - 1, -1, -1):
    y = Y0 + (b + 0.5) * C
    row = ''
    for a in range(NX):
        x = X0 + (a + 0.5) * C
        ch = '#' if occ[a, b] >= 15 else ('.' if seen[a, b] else ' ')
        if ch != '#' and any(abs(x - v) < 0.26 for v in (28, 40, 50, 60)):
            ch = '|'
        if ch in '. ' and abs(abs(y) - 2.0) < 0.26:
            ch = '-'
        row += ch
    print('%5.1f %s' % (y, row))
print('      ' + ''.join(str(int(X0 + a * C) % 10) if (X0 + a * C) % 5 == 0 else ' '
                         for a in range(NX)))
