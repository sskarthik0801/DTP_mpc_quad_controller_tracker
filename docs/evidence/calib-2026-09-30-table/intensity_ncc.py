"""Photometric check independent of the ICP: normalised cross-correlation between the LiDAR's
INTENSITY image (projected with an extrinsic) and the colour image's grey level, swept over
(du, dv). Regions: whole image, and named crops. Peak at (0,0) = the extrinsic is unbiased there."""
import sys, argparse
import numpy as np, cv2, yaml
from scipy import ndimage as ndi

ap = argparse.ArgumentParser()
ap.add_argument('vp'); ap.add_argument('Ts', nargs='+'); ap.add_argument('--R', type=int, default=40)
ap.add_argument('--box', action='append', default=[], help='name:y0,y1,x0,x1')
ap.add_argument('--rmin', type=float, default=0.8)
a = ap.parse_args()
d = np.load(a.vp, allow_pickle=True)
gray = cv2.cvtColor(d['image'], cv2.COLOR_RGB2GRAY).astype(float); H, W = gray.shape
K = d['K_col']; dist = d['D_col'][:5]
C = d['cloud']; P = C[:, :3].astype(float); I = C[:, 3].astype(float)
boxes = {'all': (0, H, 0, W)}
for b in a.box:
    n, v = b.split(':'); boxes[n] = tuple(int(x) for x in v.split(','))

def lidar_img(T):
    cam = P @ T[:3, :3].T + T[:3, 3]; r = np.linalg.norm(cam, axis=1)
    k = (cam[:, 2] > 0.3) & (r > a.rmin); cam = cam[k]; ii = I[k]; r = r[k]
    uv, _ = cv2.projectPoints(cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist); uv = uv.reshape(-1, 2)
    ui = np.round(uv[:, 0]).astype(int); vi = np.round(uv[:, 1]).astype(int)
    ok = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
    # z-buffer: keep the nearest return per pixel (occlusion-correct for the camera's view)
    order = np.argsort(-r[ok]); Z = np.full((H, W), np.nan)
    Z[vi[ok][order], ui[ok][order]] = ii[ok][order]
    m = np.isfinite(Z).astype(float); f = np.nan_to_num(Z)
    num = ndi.gaussian_filter(f, 2.0); den = ndi.gaussian_filter(m, 2.0)
    L = np.where(den > 0.05, num / np.maximum(den, 1e-6), np.nan)
    return L

G = ndi.gaussian_filter(gray, 2.0)
S = np.arange(-a.R, a.R + 1, 2)
for tp in a.Ts:
    name, p = tp.split('=')
    T = np.array(yaml.safe_load(open(p))['T_camera_lidar'], float) if p.endswith('.yaml') else np.loadtxt(p)
    L = lidar_img(T)
    for bn, (y0, y1, x0, x1) in boxes.items():
        M = np.full((len(S), len(S)), np.nan)
        for i, dv in enumerate(S):
            for j, du in enumerate(S):
                # LiDAR image shifted by (du, dv), compared on the box
                ys = slice(max(y0, y0 + dv, 0), min(y1, y1 + dv, H)); xs = slice(max(x0, x0 + du, 0), min(x1, x1 + du, W))
                Ls = L[ys.start - dv:ys.stop - dv, xs.start - du:xs.stop - du]; Gs = G[ys, xs]
                ok = np.isfinite(Ls)
                if ok.sum() < 500: continue
                x = Ls[ok] - Ls[ok].mean(); y = Gs[ok] - Gs[ok].mean()
                M[i, j] = (x @ y) / np.sqrt((x @ x) * (y @ y) + 1e-12)
        c = len(S) // 2; i, j = np.unravel_index(np.nanargmax(M), M.shape)
        def sub(v, k):
            if 0 < k < len(v) - 1 and np.all(np.isfinite(v[k - 1:k + 2])):
                den = v[k - 1] - 2 * v[k] + v[k + 1]
                return S[k] + (0 if den == 0 else (v[k - 1] - v[k + 1]) / den)
            return S[k]
        print('%-10s %-8s NCC(0,0) %.3f  peak %.3f at du %+.1f dv %+.1f px' % (name, bn, M[c, c], M[i, j], sub(M[i, :], j), sub(M[:, j], i)))
