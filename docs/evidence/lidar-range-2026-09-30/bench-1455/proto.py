exec(open('repro.py').read().split('for stamp, d, tr in frames:')[0])
import numpy as np
orig_strip, orig_rfb = lbr.strip_ground, lbr.LidarBoxRange.range_for_box
CUR = {}
def strip_guard(*a, **k):
    p = orig_strip(*a, **k)
    if CUR.get('on'):
        g = p.g.copy(); g[g > CUR['cz'] - 0.30] = np.nan
        p = lbr.GroundProfile(p.s0, p.bin_m, g)
    return p
lbr.strip_ground = strip_guard
def rfb(self, box, stamp, fc=float('nan'), vel=None):
    f = self._frame(float(stamp)); CUR['cz'] = f.c_w[2] if f.c_w is not None else np.inf
    return orig_rfb(self, box, stamp, fc, vel)
lbr.LidarBoxRange.range_for_box = rfb
# every detection frame in the bag, not only 36-41
frames = []
for j in D['dets']:
    if j.get('ok') and j.get('dets'):
        for d in j['dets']:
            frames.append((j['t_img'], d, rsd.get((round(j['t_img'], 3), d['id']))))
rows = pickle.load(open('rows1455.pkl', 'rb')); tru = {(round(w['t'], 2), w['id']): w['tru'] for w in rows}
print('   t    truth | flown rule r (n)      | guarded r (n)')
for stamp, d, tr in frames:
    fc = tr.get('fc_r') if tr else None; fc = float('nan') if fc is None else fc
    CUR['on'] = False; a = run(stamp, d['box'], fc, None)
    CUR['on'] = True;  b = run(stamp, d['box'], fc, None)
    t = tru.get((round(stamp - t0, 2), d['id']))
    fa = '%.2f (%d)' % (a.range_m, a.n_points) if a.valid else 'none'
    fb = '%.2f (%d)' % (b.range_m, b.n_points) if b.valid else 'none'
    flag = ''
    if t and a.valid and abs(a.range_m - t['r']) > 1: flag += ' <- flown rule off by %+.1f' % (a.range_m - t['r'])
    if t and b.valid and abs(b.range_m - t['r']) > 1: flag += ' <- guarded off by %+.1f' % (b.range_m - t['r'])
    print('%6.2f %6s | %-20s | %-14s%s' % (stamp - t0, '%.2f' % t['r'] if t else '-', fa, fb, flag))
