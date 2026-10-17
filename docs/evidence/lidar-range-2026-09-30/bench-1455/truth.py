import pickle, math, collections, sys
import numpy as np
D = pickle.load(open('bag1455.pkl', 'rb'))
t0 = D['t0']; scans = D['scans']
P = np.concatenate([a for _, a in scans]); T = np.concatenate([np.full(len(a), t, np.float64) for t, a in scans])
ok = np.isfinite(P).all(1); P, T = P[ok], T[ok]
r = np.hypot(P[:, 0], P[:, 1])
# floor: points far enough that the -7 deg ray reaches the floor
lo = P[(r > 9) & (P[:, 2] < -0.7)]
h, e = np.histogram(lo[:, 2], bins=np.arange(-2.0, -0.6, 0.02))
k = np.argmax(h); print('floor candidates (r>9, z<-0.7): mode %.2f..%.2f n %d of %d' % (e[k], e[k + 1], h[k], len(lo)))
print('  z pct of those', np.percentile(lo[:, 2], [1, 5, 25, 50, 75]).round(2))
# per 2 m cell lowest z (5th pct) for r>8
cells = collections.defaultdict(list)
for p in lo: cells[(int(p[0] // 2), int(p[1] // 2))].append(p[2])
for c in sorted(cells, key=lambda c: -len(cells[c]))[:10]:
    print('  cell', c, 'n', len(cells[c]), 'p5 %.2f med %.2f' % (np.percentile(cells[c], 5), np.median(cells[c])))
# static / dynamic by voxel occupancy over 1 s bins
V = np.floor(P / 0.2).astype(np.int32); B = np.floor(T - T.min()).astype(np.int32); nb = B.max() + 1
key = (V[:, 0].astype(np.int64) * 1000003 + V[:, 1]) * 1000003 + V[:, 2]
kb = np.unique(np.stack([key, B], 1), axis=0)
kk, cnt = np.unique(kb[:, 0], return_counts=True)
occ = dict(zip(kk.tolist(), cnt.tolist()))
nbins = np.array([occ[x] for x in key.tolist()])
dyn = nbins < 0.25 * nb
print('bins', nb, 'dynamic frac', dyn.mean())
np.savez('lidar1455.npz', P=P, T=T, dyn=dyn)
