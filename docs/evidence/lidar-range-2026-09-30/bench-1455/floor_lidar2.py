import numpy as np
L = np.load('lidar1455.npz'); P, dyn = L['P'], L['dyn']
S = P[~dyn]; r = np.hypot(S[:, 0], S[:, 1])
for lo, hi in [(5, 6), (6, 7), (7, 8), (8, 9), (9, 10), (10, 12), (12, 16), (16, 25)]:
    s = S[(r >= lo) & (r < hi)]
    h, e = np.histogram(s[:, 2], bins=np.arange(-1.6, 0.0, 0.05))
    top = np.argsort(h)[::-1][:3]
    print('r %2d-%2d n %6d  z p0.5 %.2f p2 %.2f | lowest ray z %.2f | modes %s' % (lo, hi, len(s), np.percentile(s[:, 2], 0.5), np.percentile(s[:, 2], 2), -hi * np.tan(np.radians(7)),
          ', '.join('%.2f(%d)' % (e[k] + 0.025, h[k]) for k in top)))
C = S[(r > 6.5) & (S[:, 2] < -0.7)]
rng = np.random.default_rng(1); best = None
for _ in range(4000):
    s = C[rng.choice(len(C), 3, replace=False)]
    n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
    if nn < 1e-9: continue
    n /= nn
    if abs(n[2]) < 0.95: continue
    d = -n @ s[0]; inl = np.abs(C @ n + d) < 0.04
    if best is None or inl.sum() > best[0]: best = (inl.sum(), n, d, inl)
cnt, n, d, inl = best
A = C[inl]; c = A.mean(0); _, _, vt = np.linalg.svd(A - c); n = vt[2]; d = -n @ c
if n[2] < 0: n, d = -n, -d
print('floor (z<-0.7) plane: inliers %d of %d  normal %s tilt %.2f deg  z at origin %.3f  dz/dx %.4f dz/dy %.4f' % (cnt, len(C), n.round(4), np.degrees(np.arccos(n[2])), -d / n[2], -n[0] / n[2], -n[1] / n[2]))
print('  inlier extent x %.1f..%.1f y %.1f..%.1f' % (A[:, 0].min(), A[:, 0].max(), A[:, 1].min(), A[:, 1].max()))
np.save('floor_lidar.npy', np.r_[n, d])
