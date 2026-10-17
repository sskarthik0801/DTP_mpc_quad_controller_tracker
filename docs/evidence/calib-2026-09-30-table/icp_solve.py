"""Solve T_camera_lidar by registering the Gemini's own depth cloud (factory depth->colour chain,
tier 1) against the accumulated MID360 cloud: robust point-to-plane ICP, several viewpoints
jointly. Independent of image features and of gravity.

usage: python3 icp_solve.py <init T_camera_lidar .txt|.yaml> vp0.npz [vp1.npz ...] [--out T.txt]
"""
import sys, argparse
import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

ap = argparse.ArgumentParser()
ap.add_argument('init'); ap.add_argument('vps', nargs='+'); ap.add_argument('--out')
ap.add_argument('--zmin', type=float, default=0.5); ap.add_argument('--zmax', type=float, default=6.0)
ap.add_argument('--iters', type=int, default=120)
ap.add_argument('--perturb', default=None, help='rx,ry,rz deg,tx,ty,tz m added to init (camera->lidar)')
ap.add_argument('--scale', action='store_true', help='also fit a depth scale factor (diagnostic)')
ap.add_argument('--fix_t', default=None, help='fix translation (camera origin in LiDAR frame) x,y,z')
a = ap.parse_args()

def load_T(p):
    if p.endswith('.yaml'):
        return np.array(yaml.safe_load(open(p))['T_camera_lidar'], float)
    return np.loadtxt(p)

T_cl0 = load_T(a.init)
T_lc = np.linalg.inv(T_cl0)          # camera optical -> lidar
if a.perturb:
    pp = [float(x) for x in a.perturb.split(',')]
    T_lc[:3, :3] = Rot.from_euler('xyz', pp[:3], degrees=True).as_matrix() @ T_lc[:3, :3]; T_lc[:3, 3] += pp[3:]

def depth_cloud(d):
    D = d['depth_med_mm'] / 1000.0; S = d['depth_std_mm'] / 1000.0
    K = d['K_dep']; H, W = D.shape
    v, u = np.mgrid[0:H, 0:W]
    # reject depth discontinuities: mixed pixels at edges disagree between sensors
    Dn = np.nan_to_num(D, nan=0.0)
    from scipy import ndimage as ndi
    mx = ndi.maximum_filter(Dn, 5); mn = ndi.minimum_filter(np.where(Dn > 0, Dn, 99), 5)
    edge = (mx - mn) > 0.08 * np.maximum(Dn, 0.5)
    ok = np.isfinite(D) & (D > a.zmin) & (D < a.zmax) & ~edge & (S < 0.02 + 0.01 * D ** 2)
    ok[::2, :] = False; ok[:, ::2] = False          # decimate 4x
    z = D[ok]; x = (u[ok] - K[0, 2]) * z / K[0, 0]; y = (v[ok] - K[1, 2]) * z / K[1, 1]
    P = np.c_[x, y, z]
    Tcd = d['T_col_dep']; P = P @ Tcd[:3, :3].T + Tcd[:3, 3]      # into colour optical
    return P

def lidar_map(d):
    C = d['cloud'][:, :3].astype(np.float64)
    C = C[(C[:, 0] > 0.2) & (np.linalg.norm(C, axis=1) < 12)]
    # voxel 1.5 cm, keep centroid
    k = np.floor(C / 0.015).astype(np.int64)
    key = k[:, 0] * 73856093 ^ k[:, 1] * 19349663 ^ k[:, 2] * 83492791
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    V = np.zeros((cnt.size, 3)); np.add.at(V, inv, C); V /= cnt[:, None]
    tree = cKDTree(V)
    _, nb = tree.query(V, k=20, workers=-1)
    Q = V[nb] - V[nb].mean(1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', Q, Q) / 20
    w, e = np.linalg.eigh(cov)
    N = e[:, :, 0]
    planarity = (w[:, 1] - w[:, 0]) / (w[:, 2] + 1e-12)
    good = (w[:, 0] < 0.01 ** 2 * 1.5) & (planarity > 0.3)
    return V, N, good, tree

VP = []
for p in a.vps:
    d = np.load(p, allow_pickle=True)
    P = depth_cloud(d); V, N, good, tree = lidar_map(d)
    VP.append(dict(name=p, P=P, V=V, N=N, good=good, tree=tree))
    print('%s: depth pts %d, lidar voxels %d (planar %d)' % (p, len(P), len(V), good.sum()))

fix_t = None if a.fix_t is None else np.array([float(s) for s in a.fix_t.split(',')])
if fix_t is not None:
    T_lc[:3, 3] = fix_t

def residuals(T, vp, scale=1.0, maxd=0.25):
    P = vp['P'] * scale
    L = P @ T[:3, :3].T + T[:3, 3]
    dist, idx = vp['tree'].query(L, k=1, distance_upper_bound=maxd, workers=-1)
    ok = np.isfinite(dist)
    ok[ok] &= vp['good'][idx[ok]]
    idx = idx[ok]; L = L[ok]; Pc = P[ok]
    n = vp['N'][idx]; q = vp['V'][idx]
    r = np.einsum('ij,ij->i', n, L - q)
    z = Pc[:, 2]
    ray = Pc / np.linalg.norm(Pc, axis=1, keepdims=True)
    ray_l = ray @ T[:3, :3].T
    sig = np.sqrt(0.015 ** 2 + (0.0025 * z ** 2 * np.einsum('ij,ij->i', n, ray_l)) ** 2)
    return r, n, L, Pc, sig

scale = 1.0
for it in range(a.iters):
    maxd = 0.25 if it < 10 else (0.12 if it < 20 else 0.08)
    H = np.zeros((7, 7)); g = np.zeros(7); tot = 0; rs = []
    for vp in VP:
        r, n, L, Pc, sig = residuals(T_lc, vp, scale, maxd)
        u = r / sig; c = 3.0
        w = np.where(np.abs(u) < c, (1 - (u / c) ** 2) ** 2, 0.0) / sig ** 2     # Tukey IRLS
        J = np.c_[np.cross(L, n), n, np.einsum("ij,ij->i", n, Pc @ T_lc[:3, :3].T)]
        H += (J * w[:, None]).T @ J; g += (J * w[:, None]).T @ r; tot += (w > 0).sum(); rs.append(r[w > 0])
    act = [0, 1, 2] + ([] if fix_t is not None else [3, 4, 5]) + ([6] if a.scale else [])
    dx = np.zeros(7)
    dx[act] = -np.linalg.solve(H[np.ix_(act, act)] + 1e-9 * np.eye(len(act)), g[act])
    dR = Rot.from_rotvec(dx[:3]).as_matrix()
    T_lc[:3, :3] = dR @ T_lc[:3, :3]; T_lc[:3, 3] = dR @ T_lc[:3, 3] + dx[3:6]
    if fix_t is not None: T_lc[:3, 3] = fix_t
    scale *= (1 + dx[6])
    rr = np.concatenate(rs)
    if it % 20 == 0 or np.abs(dx).max() < 1e-7:
        print('it %2d  inliers %d  rms %.4f m  med|r| %.4f  step rot %.4f deg  t %.5f m  scale %.4f'
              % (it, tot, np.sqrt(np.mean(rr ** 2)), np.median(np.abs(rr)), np.degrees(np.linalg.norm(dx[:3])), np.linalg.norm(dx[3:6]), scale))
    if it > 22 and np.abs(dx).max() < 1e-6:
        break

# conditioning of the final problem
ev, evec = np.linalg.eigh(H[:6, :6])
print('weakest direction (rot rad, trans m):', evec[:, 0].round(3))
print('Hessian eigenvalues (rot rad^-2 / trans m^-2 mixed):', np.array2string(ev, precision=3))
cov = np.linalg.inv(H[np.ix_(act, act)])
sd = np.sqrt(np.diag(cov))
print('formal 1-sigma: rot', np.degrees(sd[:3]).round(4), 'deg   trans', sd[3:6].round(4) if len(sd) >= 6 else '-', 'm')

T_cl = np.linalg.inv(T_lc)
np.set_printoptions(suppress=True, precision=6)
print('\nT_camera_lidar (solved):\n', T_cl)
print('camera origin in LiDAR frame (x fwd, y left, z up): ', T_lc[:3, 3].round(4))
print('deployed camera origin in LiDAR frame:              ', np.linalg.inv(T_cl0)[:3, 3].round(4))
dR = T_cl[:3, :3] @ T_cl0[:3, :3].T      # rotation change, in camera optical coords
rv = Rot.from_matrix(dR).as_rotvec(degrees=True)
print('rotation change vs deployed (camera optical axes x=right,y=down,z=fwd) rotvec deg:', rv.round(3),
      ' |%.3f| deg' % np.linalg.norm(rv))
print('  -> about optical x (pitch, + = nose up in image? see note) %.3f, about y (yaw) %.3f, about z (roll) %.3f' % tuple(rv))
# camera optical axis direction in the LiDAR frame: elevation angle
zc = T_lc[:3, :3] @ np.array([0, 0, 1.0]); zc0 = np.linalg.inv(T_cl0)[:3, :3] @ np.array([0, 0, 1.0])
print('optical axis elevation vs LiDAR x-y plane: solved %.3f deg, deployed %.3f deg' % (np.degrees(np.arcsin(zc[2])), np.degrees(np.arcsin(zc0[2]))))
print('optical axis azimuth in LiDAR frame:       solved %.3f deg, deployed %.3f deg' % (np.degrees(np.arctan2(zc[1], zc[0])), np.degrees(np.arctan2(zc0[1], zc0[0]))))
if a.out:
    np.savetxt(a.out, T_cl, fmt='%.9f'); print('wrote', a.out)
