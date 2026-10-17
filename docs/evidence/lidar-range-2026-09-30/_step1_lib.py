"""Geometry shared by the Step 1 measurement scripts. Works on the _step1_cache.py cache.

Conventions: everything in lidar_init (FAST-LIO world). "Range" is HORIZONTAL range from the
aircraft (odom position at the camera frame's stamp), because perception's position z is the
lower-body patch and command_mux/follow_planner use horizontal distance.
"""
import numpy as np

from _step1_cache import load

BAGS = ['stage_b_2026-09-29_122511', 'stage_b_2026-09-26_195108', 'stage_b_2026-09-26_200650',
        'stage_b_2026-09-21_183532', 'stage_b_2026-09-21_195658_salvaged']
BAG_DIR = '/home/ideas-ad/SUPER-Human-DTP/bags/'
# lidar_on_person.py's body band: 0.3-2.1 m above the local ground.
BODY_LO, BODY_HI = 0.3, 2.1
# depth_source.py's nearest-surface rule (CLUSTER_SEED_PCTL, CLUSTER_BAND_M).
SEED_PCTL, BAND_M = 10.0, 0.6
# A FAST-LIO scan is deskewed to its END (lidar_end_time = the stamp); a moving walker's points
# were taken over the 0.1 s before it, on average 0.05 s before.
SCAN_MID_OFFSET_S = 0.05


def nearest_cluster(r):
    """depth_source.range_in_roi's statistic on a 1-D array of ranges: (median, n, mask)."""
    r = np.asarray(r, float)
    if r.size == 0:
        return np.nan, 0, np.zeros(0, bool)
    seed = np.percentile(r, SEED_PCTL)
    m = (r >= seed - BAND_M) & (r <= seed + BAND_M)
    return float(np.median(r[m])), int(m.sum()), m


class Bag:
    def __init__(self, name):
        self.name = name
        c = load(BAG_DIR + name)
        self.c = c
        self.start = float(c['start'])
        od = c['od']
        self.od_t = od[:, 0]
        self.od_p = od[:, 1:4]
        w, x, y, z = od[:, 4], od[:, 5], od[:, 6], od[:, 7]
        self.od_yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        self.sc_t = c['sc_t']
        self.sc_off = c['sc_off']
        self.pts = c['sc_pts']
        self.sc_pos = c['sc_pos']
        self.tg = c['tg']
        self.ft = c['ft']
        self.lk_t, self.lk_s = c['lk_t'], c['lk_s']
        self.K = c['K']
        self._ground()

    # -- odom -----------------------------------------------------------------------------------
    def pos(self, t):
        return np.array([np.interp(t, self.od_t, self.od_p[:, k]) for k in range(3)]).T

    def yaw(self, t):
        return np.interp(t, self.od_t, self.od_yaw)

    # -- ground ---------------------------------------------------------------------------------
    def _ground(self, cell=2.0):
        """Local ground: 5th percentile z of every 4th scan's points per 2 m cell, holes filled
        from the nearest filled cell. (ground.py's profile is 1-D and for 09-29 only.)"""
        idx = np.concatenate([np.arange(self.sc_off[i], self.sc_off[i + 1])
                              for i in range(0, len(self.sc_t), 4)])
        p = self.pts[idx]
        p = p[np.hypot(p[:, 0], p[:, 1]) < 80]
        ix = np.floor(p[:, 0] / cell).astype(np.int64)
        iy = np.floor(p[:, 1] / cell).astype(np.int64)
        self.g_x0, self.g_y0 = ix.min(), iy.min()
        nx, ny = ix.max() - self.g_x0 + 1, iy.max() - self.g_y0 + 1
        key = (ix - self.g_x0) * ny + (iy - self.g_y0)
        order = np.lexsort((p[:, 2], key))
        ks, zs = key[order], p[order, 2]
        u, first, cnt = np.unique(ks, return_index=True, return_counts=True)
        g = np.full(nx * ny, np.nan)
        ok = cnt >= 15
        g[u[ok]] = zs[first[ok] + (0.05 * (cnt[ok] - 1)).astype(int)]
        g = g.reshape(nx, ny)
        # fill holes by nearest filled cell
        fi, fj = np.nonzero(np.isfinite(g))
        hi, hj = np.nonzero(~np.isfinite(g))
        if len(hi):
            d = (hi[:, None] - fi[None, :]) ** 2 + (hj[:, None] - fj[None, :]) ** 2
            j = d.argmin(axis=1)
            g[hi, hj] = g[fi[j], fj[j]]
        self.g, self.g_cell, self.g_nx, self.g_ny = g, cell, nx, ny

    def ground(self, x, y):
        i = np.clip(np.floor(np.asarray(x) / self.g_cell).astype(np.int64) - self.g_x0, 0,
                    self.g_nx - 1)
        j = np.clip(np.floor(np.asarray(y) / self.g_cell).astype(np.int64) - self.g_y0, 0,
                    self.g_ny - 1)
        return self.g[i, j]

    # -- scans ----------------------------------------------------------------------------------
    def scans_between(self, t0, t1):
        """Indices of scans whose mid-acquisition time lies in [t0, t1]."""
        tm = self.sc_t - SCAN_MID_OFFSET_S
        return np.nonzero((tm >= t0) & (tm <= t1))[0]

    def window_points(self, t0, t1, centre_xy=None, radius=None):
        """World points of the scans in [t0, t1] (optionally within `radius` of centre_xy),
        with each point's scan mid-time. Returns (pts Nx3 float64, t N)."""
        out, tt = [], []
        for i in self.scans_between(t0, t1):
            a = self.pts[self.sc_off[i]:self.sc_off[i + 1]]
            if centre_xy is not None:
                m = (np.abs(a[:, 0] - centre_xy[0]) < radius) & (np.abs(a[:, 1] - centre_xy[1]) < radius)
                a = a[m]
            out.append(a.astype(np.float64))
            tt.append(np.full(len(a), self.sc_t[i] - SCAN_MID_OFFSET_S))
        if not out:
            return np.zeros((0, 3)), np.zeros(0)
        return np.vstack(out), np.concatenate(tt)

    def lock_at(self, t_recv):
        i = np.searchsorted(self.lk_t, t_recv, side='right') - 1
        return self.lk_s[i].split('|')[0] if i >= 0 else ''

    def camera_frames(self, min_conf=0.3):
        """The camera's measured person positions: every Target that is confirmed, above the
        confidence floor (0.25 marks a track with no detector score), and not a HELD frame
        (perception republishes the last position unchanged when the patch is unranged).
        Returns a structured dict of arrays, with horizontal range and bearing from the aircraft."""
        tg = self.tg
        keep = (tg[:, 10] > 0.5) & (tg[:, 8] > min_conf)
        held = np.zeros(len(tg), bool)
        last = {}
        for k in range(len(tg)):
            tid = int(tg[k, 2])
            p = (tg[k, 3], tg[k, 4])
            if tid in last and last[tid] == p:
                held[k] = True
            last[tid] = p
        keep &= ~held
        tg = tg[keep]
        a = self.pos(tg[:, 0])
        dx, dy = tg[:, 3] - a[:, 0], tg[:, 4] - a[:, 1]
        return dict(t=tg[:, 0], t_recv=tg[:, 1], tid=tg[:, 2].astype(int), x=tg[:, 3],
                    y=tg[:, 4], z=tg[:, 5], vx=tg[:, 6], vy=tg[:, 7], conf=tg[:, 8],
                    ax=a[:, 0], ay=a[:, 1], az=a[:, 2], r=np.hypot(dx, dy),
                    brg=np.arctan2(dy, dx), n_held=int(held.sum()))


# -- static / dynamic ---------------------------------------------------------------------------
# A point is STATIC when its voxel was also occupied in >= STATIC_MIN_BINS distinct 1-s bins more
# than STATIC_EXCLUDE_S away from the query time. The walker occupies a given 0.25 m voxel for a
# fraction of a second per pass; the fence, trees, a parked car or a standing bystander occupy it
# for as long as they are in view. It is a heuristic: a person who stood still for several seconds
# somewhere is "static" there at every other time too, and a far, sparsely hit static object can
# read as dynamic. Used only to identify the walker's points, never as a range.
VOX = 0.25
STATIC_MIN_BINS = 4
STATIC_EXCLUDE_S = 3.0


def _vkey(p):
    q = np.floor(np.asarray(p, np.float64) / VOX).astype(np.int64) + (1 << 19)
    return (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]


def build_static(b):
    """Per bag: sorted voxel keys, and for each key the sorted 1-s bins it was occupied in."""
    n = np.diff(b.sc_off)
    tbin = np.repeat(np.floor(b.sc_t - b.start).astype(np.int64), n)
    keys = _vkey(b.pts)
    o = np.lexsort((tbin, keys))
    keys, tbin = keys[o], tbin[o]
    first = np.ones(len(keys), bool)
    first[1:] = (keys[1:] != keys[:-1]) | (tbin[1:] != tbin[:-1])
    b.st_key = keys[first]
    b.st_bin = tbin[first]


def is_static(b, P, t):
    """Boolean per point of P (Nx3) at time t (epoch s)."""
    if not hasattr(b, 'st_key'):
        build_static(b)
    k = _vkey(P)
    lo = np.searchsorted(b.st_key, k, 'left')
    hi = np.searchsorted(b.st_key, k, 'right')
    tb = t - b.start
    out = np.zeros(len(P), bool)
    for i in range(len(P)):
        if hi[i] - lo[i] < STATIC_MIN_BINS:
            continue
        bins = b.st_bin[lo[i]:hi[i]]
        out[i] = np.count_nonzero(np.abs(bins + 0.5 - tb) > STATIC_EXCLUDE_S) >= STATIC_MIN_BINS
    return out


def pitch_deg(b, t):
    """Nose-UP positive pitch of the MID360/IMU body (FLU), degrees."""
    od = b.c['od']
    i = min(np.searchsorted(b.od_t, t), len(b.od_t) - 1)
    w, x, y, z = od[i, 4:8]
    return -np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))


def densest_cluster(r, win=0.6, band=0.45):
    """Truth statistic (not the design's): the densest `win`-wide group of ranges, then the median
    of the points within `band` of its median. Robust to a stray point in front, which the design's
    nearest-surface rule is not meant to be. Returns (median, n)."""
    r = np.sort(np.asarray(r, float))
    if r.size == 0:
        return np.nan, 0
    j = np.searchsorted(r, r + win, 'right')
    i = int(np.argmax(j - np.arange(r.size)))
    c = np.median(r[i:j[i]])
    m = np.abs(r - c) <= band
    return float(np.median(r[m])), int(m.sum())


MID360_FLOOR_DEG = -7.0


def lidar_floor_deg(b, t, azimuth):
    """World elevation of the MID360's lowest ray (-7 deg in its own frame) toward a world
    azimuth, given the attitude at t. Exact rotation of the sensor's floor cone."""
    od = b.c['od']
    i = min(np.searchsorted(b.od_t, t), len(b.od_t) - 1)
    w, x, y, z = od[i, 4:8]
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    phi = np.radians(np.arange(0, 360, 0.5))
    e = np.radians(MID360_FLOOR_DEG)
    d = np.stack([np.cos(e) * np.cos(phi), np.cos(e) * np.sin(phi), np.full_like(phi, np.sin(e))])
    dw = R @ d
    az = np.arctan2(dw[1], dw[0])
    el = np.degrees(np.arcsin(np.clip(dw[2], -1, 1)))
    k = np.argmin(np.abs(np.angle(np.exp(1j * (az - azimuth)))))
    return float(el[k])
