import pickle, math, collections
import numpy as np
D = pickle.load(open('bag1455.pkl', 'rb')); t0 = D['t0']
L = np.load('lidar1455.npz'); P, T, dyn = L['P'], L['T'], L['dyn']
CAM = np.array([0.088, -0.005, 0.033]); H_CAM = 0.897; TILT = math.radians(2.67)
def floor_z(x, y):   # floor plane in lidar_init: 0.897 m below the camera, normal tilted by PX4's nose-up pitch
    return CAM[2] - H_CAM / math.cos(TILT) - (x - CAM[0]) * math.tan(TILT)
# check the floor model against the LiDAR: points within +-6 cm of it at r > 6.5
r = np.hypot(P[:, 0], P[:, 1]); fz = CAM[2] - H_CAM / math.cos(TILT) - (P[:, 0] - CAM[0]) * math.tan(TILT)
near = (r > 6.5) & (np.abs(P[:, 2] - fz) < 0.06); below = (r > 6.5) & (P[:, 2] < fz - 0.10)
print('LiDAR points r>6.5 within 6 cm of the floor model: %d; >10 cm below it: %d (mirror ghosts on a glossy floor)' % (near.sum(), below.sum()))
Pd, Td = P[dyn], T[dyn]
def truth(ts, xyz):
    b = math.atan2(xyz[1] - CAM[1], xyz[0] - CAM[0])
    s = np.abs(Td - ts) < 0.3
    Q = Pd[s]
    if not len(Q): return None
    az = np.arctan2(Q[:, 1] - CAM[1], Q[:, 0] - CAM[0]); rr = np.hypot(Q[:, 0] - CAM[0], Q[:, 1] - CAM[1])
    fzq = CAM[2] - H_CAM / math.cos(TILT) - (Q[:, 0] - CAM[0]) * math.tan(TILT)
    m = (np.abs(np.angle(np.exp(1j * (az - b)))) < math.radians(5)) & (rr > 0.8) & (rr < 14) & (Q[:, 2] > fzq + 0.15) & (Q[:, 2] < fzq + 2.3)
    if m.sum() < 3: return None
    rs = np.sort(rr[m]); best = None
    for i in range(len(rs)):
        j = np.searchsorted(rs, rs[i] + 0.6); n = j - i
        if best is None or n > best[0] + 2: best = (n, rs[i:j])
    n, cl = best
    if n < 3: return None
    rc = float(np.median(cl)); sel = m & (np.abs(rr - rc) < 0.5)
    return dict(r=rc, n=int(n), ztop=float(Q[sel, 2].max()), zlow=float(Q[sel, 2].min()), fz=float(np.median(fzq[sel])))
# per-frame rows keyed by image stamp: detections_2d (box, xyz), range_source (src, cam_r, r)
rsd = {}
for j in D['rs']:
    for tr in j['tracks']: rsd[(round(j['t_img'], 3), tr['id'])] = tr
rows = []
for j in D['dets']:
    if not j.get('ok') or not j.get('dets'): continue
    for d in j['dets']:
        ts = j['t_img']; x, y, z = d['xyz']
        tr = rsd.get((round(ts, 3), d['id']))
        tru = truth(ts, d['xyz'])
        dist = math.hypot(x - CAM[0], y - CAM[1])
        rows.append(dict(t=ts - t0, id=d['id'], box=d['box'], patch=d['patch'], x=x, y=y, z=z, dist=dist, fz=floor_z(x, y), src=tr and tr['src'], cam_r=tr and tr.get('cam_r'), r=tr and tr.get('r'), why=tr and tr.get('why'), tru=tru))
print('%6s %3s %6s %6s %6s %6s %6s | %6s %4s %6s %6s | %6s %6s %6s' % ('t', 'id', 'src', 'cam_r', 'r', 'pub_d', 'truth', 'box_h', 'npt', 'z_pub', 'floor', 'z-flr', 'head', 'cylbot'))
for w in rows:
    tr = w['tru']
    print('%6.2f %3d %6s %6s %6s %6.2f %6s | %6.0f %4s %6.2f %6.2f | %6.2f %6s %6.2f' % (w['t'], w['id'], w['src'], w['cam_r'], w['r'], w['dist'], '%.2f' % tr['r'] if tr else '-', w['box'][3] - w['box'][1], tr['n'] if tr else '-',
          w['z'], w['fz'], w['z'] - w['fz'], '%.2f' % (tr['ztop'] - w['fz']) if tr else '-', w['z'] - 0.85 - w['fz']))
pickle.dump(rows, open('rows1455.pkl', 'wb'))
