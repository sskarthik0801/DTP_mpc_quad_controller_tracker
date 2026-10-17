"""Score replay runs (targets_<L>.jsonl + probe_<L>.jsonl) against the LiDAR walker truth of the
2026-09-30 14:55 bag. Truth: dynamic LiDAR points within +-5 deg of the published bearing,
+-0.3 s of the frame stamp, 0.15-2.3 m above the floor; densest 0.6 m range cluster."""
import json, math, sys, collections
import numpy as np
RUN = sys.argv[1]; LABELS = sys.argv[2:]
L = np.load('lidar1455.npz'); P, T, dyn = L['P'], L['T'], L['dyn']; Pd, Td = P[dyn], T[dyn]
CAM = np.array([0.088, -0.005, 0.033]); H_CAM = 0.897; TILT = math.radians(2.67)
def floor_z(x): return CAM[2] - H_CAM / math.cos(TILT) - (x - CAM[0]) * math.tan(TILT)
_tc = {}
def truth(ts, x, y):
    b = math.atan2(y - CAM[1], x - CAM[0]); key = (round(ts, 3), round(b, 2))
    if key in _tc: return _tc[key]
    s = np.abs(Td - ts) < 0.3; Q = Pd[s]; out = None
    if len(Q):
        az = np.arctan2(Q[:, 1] - CAM[1], Q[:, 0] - CAM[0]); rr = np.hypot(Q[:, 0] - CAM[0], Q[:, 1] - CAM[1])
        fz = CAM[2] - H_CAM / math.cos(TILT) - (Q[:, 0] - CAM[0]) * math.tan(TILT)
        m = (np.abs(np.angle(np.exp(1j * (az - b)))) < math.radians(5)) & (rr > 0.8) & (rr < 14) & (Q[:, 2] > fz + 0.15) & (Q[:, 2] < fz + 2.3)
        if m.sum() >= 3:
            rs = np.sort(rr[m]); best = None
            for i in range(len(rs)):
                j = np.searchsorted(rs, rs[i] + 0.6)
                if best is None or j - i > best[0] + 2: best = (j - i, rs[i:j])
            if best[0] >= 3: out = float(np.median(best[1]))
    _tc[key] = out; return out
BINS = [(0, 2), (2, 4), (4, 6), (6, 8), (8, 10)]
def load(label):
    fr = {}
    for l in open('%s/targets_%s.jsonl' % (RUN, label)):
        j = json.loads(l)
        if j['k'] == 't': fr[round(j['s'], 3)] = j['targets']
    pr = {}
    try:
        for l in open('%s/probe_%s.jsonl' % (RUN, label)):
            j = json.loads(l); pr[(round(j['s'], 3), j['id'])] = j
    except FileNotFoundError: pass
    return fr, pr
res = {}
for lab in LABELS:
    fr, pr = load(lab); rows = []
    for s, tg in fr.items():
        for tid, x, y, z, d, c in tg:
            tr = truth(s, x, y)
            pb = pr.get((s, tid)) or pr.get((round(s, 4), tid)) or {}
            src = pb.get('src') or ('camera' if not pb.get('held') else 'held')
            if pb.get('held') and pb.get('src') not in ('hold',): src = 'held'
            rows.append(dict(s=s, id=tid, r=math.hypot(x - CAM[0], y - CAM[1]), z=z, zf=z - floor_z(x), tr=tr, src=src))
    res[lab] = rows
    nfr = len(fr); nwith = sum(1 for v in fr.values() if v)
    print('\n== %s: %d frames, %d with a person, %d person-frames, %d with LiDAR truth' % (lab, nfr, nwith, len(rows), sum(r['tr'] is not None for r in rows)))
    print('   bin      n  src(cam/lidar/hold/held)   |err| p50  p90   within0.3  >1m long  >1m short   z-above-floor p50')
    for lo, hi in BINS:
        rr = [r for r in rows if r['tr'] is not None and lo <= r['tr'] < hi]
        if not rr: continue
        e = np.array([r['r'] - r['tr'] for r in rr]); c = collections.Counter(r['src'] for r in rr)
        print('   %2d-%-2d %4d   %3d/%3d/%3d/%3d          %5.2f %5.2f    %4.0f%%     %3d       %3d        %.2f' % (lo, hi, len(rr), c['camera'], c['lidar'], c['hold'], c['held'],
              np.median(np.abs(e)), np.percentile(np.abs(e), 90), 100 * np.mean(np.abs(e) <= 0.3), (e > 1).sum(), (e < -1).sum(), np.median([r['zf'] for r in rr])))
    bad = [r for r in rows if r['tr'] is not None and abs(r['r'] - r['tr']) > 1.0]
    for r in bad[:12]:
        print('   gross: s %.2f id %d pub %.2f truth %.2f src %s' % (r['s'] - 1790794507.681919, r['id'], r['r'], r['tr'], r['src']))
import pickle; pickle.dump(res, open('%s/scored.pkl' % RUN, 'wb'))
