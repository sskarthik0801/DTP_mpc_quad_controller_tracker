import sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG = sys.argv[1]
db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
topics = {r[0]: (r[1], r[2]) for r in db.execute('select id,name,type from topics')}
name2id = {v[0]: k for k, v in topics.items()}
t0 = db.execute('select min(timestamp) from messages').fetchone()[0] / 1e9
def hs(name):
    tid = name2id[name]; cls = get_message(topics[tid][1]); out = []
    for ts, data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        m = deserialize_message(data, cls); out.append((ts / 1e9 - t0, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t0))
    return np.array(out)
for name in ['/camera/color/image_raw/compressed', '/camera/color/image_raw', '/camera/depth/image_raw', '/camera/depth/points']:
    a = hs(name)
    lag = a[:, 0] - a[:, 1]; gaps = np.diff(a[:, 1])
    print('%-38s n %4d  header %.2f..%.2f  rx-stamp lag p50 %.3f p90 %.3f max %.3f  stamp gaps p50 %.3f max %.3f (n>0.2 s: %d)' % (name, len(a), a[0, 1], a[-1, 1], np.median(lag), np.percentile(lag, 90), lag.max(), np.median(gaps), gaps.max(), (gaps > 0.2).sum()))
    if 'depth/image_raw' in name: D = a
    if name.endswith('compressed'): C = a
# skew: for each colour stamp, nearest depth stamp at or before
ds = np.sort(D[:, 1])
sk = []
for c in C[:, 1]:
    i = np.searchsorted(ds, c) 
    cand = [abs(c - ds[j]) for j in (i - 1, i) if 0 <= j < len(ds)]
    sk.append(min(cand))
sk = np.array(sk); print('colour->nearest depth stamp: p50 %.3f p90 %.3f  frac > 0.2 s: %.2f' % (np.median(sk), np.percentile(sk, 90), (sk > 0.2).mean()))
big = np.nonzero(np.diff(ds) > 0.2)[0]
print('depth stamp gaps > 0.2 s at t:', [(round(ds[i], 2), round(ds[i + 1] - ds[i], 2)) for i in big][:30])
