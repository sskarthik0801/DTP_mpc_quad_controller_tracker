"""Second-level scoring of score.py's output: signed error, and walker speed from the truth series."""
import pickle, sys, collections, math
import numpy as np
RUN = sys.argv[1]; LABELS = sys.argv[2:]
res = pickle.load(open('%s/scored.pkl' % RUN, 'rb'))
# a common truth time series across all arms (same LiDAR truth, one per stamp)
tt = sorted({(r['s'], r['tr']) for lab in res for r in res[lab] if r['tr'] is not None})
ts = np.array([a for a, b in tt]); tr = np.array([b for a, b in tt])
def speed(s):
    m = np.abs(ts - s) <= 0.35
    if m.sum() < 3: return float('nan')
    return abs(np.polyfit(ts[m] - s, tr[m], 1)[0])
BINS = [(0, 2), (2, 4), (4, 6), (6, 8), (8, 10)]
for lab in LABELS:
    rows = [r for r in res[lab] if r['tr'] is not None]
    print('\n== %s' % lab)
    print('   bin    | STILL (<0.4 m/s): n  err p50  |err|p50  within0.3  >1m far | MOVING: n  err p50  |err|p50  within0.3  >1m far')
    for lo, hi in BINS:
        out = '   %2d-%-2d  |' % (lo, hi)
        for still in (True, False):
            rr = [r for r in rows if lo <= r['tr'] < hi and ((speed(r['s']) < 0.4) == still)]
            if not rr: out += '        -                                         |'; continue
            e = np.array([r['r'] - r['tr'] for r in rr])
            out += ' %4d  %+6.2f    %5.2f      %4.0f%%    %3d    |' % (len(rr), np.median(e), np.median(np.abs(e)), 100 * np.mean(np.abs(e) <= 0.3), (e > 1).sum())
        print(out)
