exec(open('repro.py').read().split('for stamp, d, tr in frames:')[0])
import numpy as np
orig = lbr.strip_ground
def spy(s, lat, z, s_lo, s_hi, **kw):
    p = orig(s, lat, z, s_lo, s_hi, **kw)
    print('  strip: %d pts, s %.1f..%.1f' % (len(s), s_lo, s_hi))
    for b, gz in enumerate(p.g):
        m = (s >= s_lo + b * 2) & (s < s_lo + (b + 1) * 2)
        print('   bin %.1f-%.1f m: n %3d  lat %.1f..%.1f  z p10 %s  z range %.2f..%.2f -> ground %.2f (floor is ~%.2f)' % (
            s_lo + b * 2, s_lo + b * 2 + 2, m.sum(), lat[m].min() if m.any() else 0, lat[m].max() if m.any() else 0,
            '%.2f' % np.percentile(z[m], 10) if m.any() else '-', z[m].min() if m.any() else 0, z[m].max() if m.any() else 0, gz,
            0.033 - 0.897 / np.cos(np.radians(2.67)) - (s_lo + b * 2 + 1) * np.tan(np.radians(2.67))))
    return p
lbr.strip_ground = spy
for stamp, d, tr in frames:
    if abs(stamp - t0 - 40.72) < 0.05 or abs(stamp - t0 - 39.65) < 0.05:
        print('t %.2f box %s' % (stamp - t0, d['box']))
        s = run(stamp, d['box'], tr['fc_r'], None)
        print('  -> r %.2f n %d box %d gated %d ground %d' % (s.range_m, s.n_points, s.n_box, s.n_gated, s.n_ground))
        s = run(stamp, d['box'], tr['fc_r'], None, ground=0.0)
        print('  no ground rejection -> r %.2f n %d' % (s.range_m, s.n_points))
