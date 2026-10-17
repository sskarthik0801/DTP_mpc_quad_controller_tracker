"""Independent check, no ICP: the tabletop plane as each sensor sees it ON ITS OWN.
Camera: the sparse near-range depth on the tabletop (bottom of the depth image).
LiDAR:  its returns on the tabletop (below the LiDAR, ahead).
A plane gives 2 rotation DOF (pitch, roll) and 1 translation DOF (height along the normal), so it
checks exactly the axes the ICP is weakest on."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import yaml

d = np.load(sys.argv[1], allow_pickle=True)
Ts = {n: (np.array(yaml.safe_load(open(p))['T_camera_lidar'], float) if p.endswith('.yaml') else np.loadtxt(p))
      for n, p in (x.split('=') for x in sys.argv[2:])}
rng = np.random.default_rng(0)

def ransac(Q, thr, iters=2000):
    best = None
    for _ in range(iters):
        s = Q[rng.choice(len(Q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9: continue
        n /= nn; dd = -n @ s[0]
        inl = np.abs(Q @ n + dd) < thr
        if best is None or inl.sum() > best[0]: best = (inl.sum(), inl)
    A = Q[best[1]]; c = A.mean(0); _, _, vt = np.linalg.svd(A - c, full_matrices=False); n = vt[2]
    return n, -n @ c, best[1]

# camera: bottom band of the depth image, near range
D = d['depth_med_mm'] / 1000.0; K = d['K_dep']; H, W = D.shape
v, u = np.mgrid[0:H, 0:W]
sel = np.isfinite(D) & (v > 0.62 * H) & (D > 0.3) & (D < 2.5)
z = D[sel]; x = (u[sel] - K[0, 2]) * z / K[0, 0]; y = (v[sel] - K[1, 2]) * z / K[1, 1]
P = np.c_[x, y, z] @ d['T_col_dep'][:3, :3].T + d['T_col_dep'][:3, 3]
nc, dc, inl = ransac(P, 0.015)
if nc[1] > 0: nc, dc = -nc, -dc                  # up = -y in optical
print('CAMERA table: %d/%d inliers, range %.2f-%.2f m, height of colour optical centre above table %.4f m, normal(optical) %s'
      % (inl.sum(), len(P), P[inl, 2].min(), P[inl, 2].max(), dc, nc.round(4)))
r = np.abs(P[inl] @ nc + dc); print('   plane residual rms %.4f m' % np.sqrt(np.mean(r ** 2)))

# LiDAR: returns below the sensor, ahead, near the table height
C = d['cloud'][:, :3].astype(float)
selL = (C[:, 0] > 0.5) & (C[:, 0] < 4.0) & (np.abs(C[:, 1]) < 0.5) & (C[:, 2] < -0.05) & (C[:, 2] > -0.6)
Q = C[selL]; Q = Q[rng.choice(len(Q), min(len(Q), 60000), replace=False)]
nl, dl, inlL = ransac(Q, 0.01)
if nl[2] < 0: nl, dl = -nl, -dl
print('LIDAR  table: %d/%d inliers, x %.2f-%.2f m, height of LiDAR origin above table %.4f m, normal(lidar) %s'
      % (inlL.sum(), len(Q), Q[inlL, 0].min(), Q[inlL, 0].max(), dl, nl.round(4)))
r = np.abs(Q[inlL] @ nl + dl); print('   plane residual rms %.4f m' % np.sqrt(np.mean(r ** 2)))

for name, T in Ts.items():
    R = T[:3, :3]; t = T[:3, 3]
    n_pred = R @ nl                                 # LiDAR table normal expressed in camera optical
    # camera origin in lidar frame, and its height above the LiDAR's table plane
    o_l = -R.T @ t
    h_pred = nl @ o_l + dl
    ang = np.degrees(np.arccos(np.clip(n_pred @ nc, -1, 1)))
    # split the normal disagreement into pitch (about optical x) and roll (about optical z)
    pitch = np.degrees(np.arctan2(n_pred[2], -n_pred[1]) - np.arctan2(nc[2], -nc[1]))
    roll = np.degrees(np.arctan2(n_pred[0], -n_pred[1]) - np.arctan2(nc[0], -nc[1]))
    print('%-10s normal angle %.3f deg (pitch %+.3f, roll %+.3f)   camera height above table: via T %.4f m, camera itself %.4f m, diff %+.4f m'
          % (name, ang, pitch, roll, h_pred, dc, h_pred - dc))
