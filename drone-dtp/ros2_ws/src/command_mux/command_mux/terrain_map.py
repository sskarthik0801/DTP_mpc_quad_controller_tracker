"""A ground-height map built from FAST-LIO's registered scans, with no ROS in it.

WHY IT EXISTS (2026-09-21, user decision)
-----------------------------------------
Every follow goal's altitude is absolute in lidar_init: z = 0 is the LiDAR's height where the
stack started. On the 2026-09-21 18:41 flight the field sloped away ~1.2 m along the corridor,
so a goal held at a fixed lidar_init height put the aircraft 3.4-3.7 m above the ground at the
far end, and the walker slid out of the bottom of the camera frame. terrain_estimator_node feeds
this map, and command_mux shifts the follow height by the ground it reports (terrain.py).

WHY NOT THE DOWNWARD RANGEFINDER
--------------------------------
A single beam straight down reads whatever is under it. It reads long the instant the aircraft
crosses the edge of a ditch, so a goal slaved to it drops by the ditch's depth, and SUPER sees
the ditch as free space and flies into it. It reads short over a bush, a car or a person, so the
goal pops up and drops again past it. Those are the failure modes the user asked about.

HOW THE ESTIMATE IS MADE, AND WHAT EACH STEP REJECTS
----------------------------------------------------
  1. Points go into 0.5 m cells, and each cell keeps its last `point_history` points' z ACROSS
     scans. Its ground is a LOW PERCENTILE (20 %) of those, not the minimum -- one multipath
     return below the ground must not become the ground. A tree trunk, a wall or a person
     standing in a cell still leaves ground returns at its foot, and the low percentile picks
     those. A cell needs `min_points_per_cell` points before it counts.
  2. Pooling across scans is not optional. In flight the MID360 sees the ground only in its
     bottom two degrees, 17-30 m out, at about one point per cell per scan. The first version
     asked for 3 points per cell in ONE scan, and replaying the 2026-09-21 flight it learned no
     ground ahead of the aircraft at all (terrain_replay.py).
  3. The ground at a point is the MEDIAN over every known cell within `patch_radius_m` (1.5 m),
     and needs at least `min_cells` (8 of the 29) of them. A narrow ditch or a bush covers a
     minority of the patch and does not move a median. A slope, or a ditch wider than the patch,
     does.
  4. Points above the sensor are dropped, so a canopy overhead never enters the map. A canopy
     BELOW the sensor with no ground seen under it reads high -- the safe direction, and SUPER
     avoids the canopy itself because it is in ROG-Map.
  5. Cells with no returns are UNKNOWN, never "a hole". The MID360 sees down to about -7 deg, so
     from 2 m up the nearest ground it sees is ~16 m away, and the bottom of a ditch seen at that
     grazing angle is usually in the shadow of its near edge. An unknown patch reports None and
     the consumer keeps what it had.

PEOPLE ARE CUT OUT BEFORE THEY GO IN (found replaying 2026-09-21 with the estimator started in
the air). From 2 m up the MID360 cannot see the ground within ~17 m, but it sees a person's
body there. The walker waited ~30 s at 5-6 m before takeoff: three cells filled with 32 points
at ~0.7 m (the p20 of a body), the patch under the goal had no other known cell, and the
"ground" read +0.70 against a true -0.35. The goal climbed 0.9 m for 10 s. The walker is in the
scene for the whole follow, walking over the cells the aircraft is about to fly over, so
add_scan takes `exclude` -- the tracked people's positions (terrain_estimator_node feeds
/dtp/targets) -- and drops every point within `exclude_radius_m` of one. min_cells is 8, not
4: one person or one bush (1-4 cells) can never make a patch on its own.

What it does not do: model anything else that moves. A car or a dog that stood in a cell leaves
its points there until `point_history` newer points replace them.
"""
import collections
import math

import numpy as np

# Cells are keyed by one int64: ix * _KEY_SPAN + iy, with both indices offset to be >= 0. 2^20
# cells of 0.5 m is 524 km per axis either side of the origin -- no venue reaches that.
_KEY_OFFSET = 1 << 20
_KEY_SPAN = 1 << 21


class TerrainMap:
    """See the module docstring. Coordinates are lidar_init metres, z up."""

    def __init__(self, cell_m=0.5, point_history=32, cell_percentile=0.2, min_points_per_cell=3,
                 patch_radius_m=1.5, min_cells=8, max_range_m=30.0, min_below_sensor_m=0.1,
                 exclude_radius_m=1.0):
        for name, value in (('cell_m', cell_m), ('patch_radius_m', patch_radius_m),
                            ('max_range_m', max_range_m)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('terrain map %s must be finite and > 0, got %r' % (name, value))
        if not 0.0 <= cell_percentile < 1.0:
            raise ValueError('cell_percentile must be in [0, 1), got %r' % cell_percentile)
        if (int(point_history) < 1 or int(min_points_per_cell) < 1 or int(min_cells) < 1
                or int(min_points_per_cell) > int(point_history)):
            raise ValueError('point_history, min_points_per_cell and min_cells must be >= 1, and '
                             'min_points_per_cell <= point_history')
        self.cell_m = float(cell_m)
        self.point_history = int(point_history)
        self.cell_percentile = float(cell_percentile)
        self.min_points_per_cell = int(min_points_per_cell)
        self.patch_radius_m = float(patch_radius_m)
        self.min_cells = int(min_cells)
        self.max_range_m = float(max_range_m)
        self.min_below_sensor_m = float(min_below_sensor_m)
        self.exclude_radius_m = float(exclude_radius_m)
        self._cells = {}
        r = int(math.ceil(self.patch_radius_m / self.cell_m))
        # The cell offsets whose CENTRE lies within the patch radius of the query cell's centre.
        self._patch = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                       if math.hypot(dx, dy) * self.cell_m <= self.patch_radius_m]

    def __len__(self):
        return len(self._cells)

    def _index(self, v):
        return int(math.floor(v / self.cell_m))

    def add_scan(self, points, sensor, exclude=()):
        """Fold one scan into the map. points: (N, 3) world-frame array. sensor: (x, y, z).
        exclude: (x, y) of people; their points (within exclude_radius_m) are dropped.

        Returns the number of cells this scan updated.
        """
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        sx, sy, sz = sensor
        ok = np.isfinite(pts).all(axis=1)
        pts = pts[ok]
        if pts.size == 0:
            return 0
        rng = np.hypot(pts[:, 0] - sx, pts[:, 1] - sy)
        keep = (rng <= self.max_range_m) & (pts[:, 2] <= sz - self.min_below_sensor_m)
        for ex, ey in exclude:
            if math.isfinite(ex) and math.isfinite(ey):
                keep &= np.hypot(pts[:, 0] - ex, pts[:, 1] - ey) > self.exclude_radius_m
        pts = pts[keep]
        if pts.shape[0] == 0:
            return 0
        ix = np.floor(pts[:, 0] / self.cell_m).astype(np.int64) + _KEY_OFFSET
        iy = np.floor(pts[:, 1] / self.cell_m).astype(np.int64) + _KEY_OFFSET
        keys = ix * _KEY_SPAN + iy
        order = np.lexsort((pts[:, 2], keys))          # by cell, then by z within the cell
        keys = keys[order]
        zs = pts[order, 2]
        starts = np.flatnonzero(np.r_[True, keys[1:] != keys[:-1]])
        ends = np.r_[starts[1:], keys.shape[0]]
        cells = self._cells
        n = self.point_history
        for key, a, b in zip(keys[starts].tolist(), starts.tolist(), ends.tolist()):
            hist = cells.get(key)
            if hist is None:
                hist = cells[key] = collections.deque(maxlen=n)
            if b - a <= n:
                hist.extend(zs[a:b].tolist())
            else:
                # More points than the cell keeps: n samples spread evenly over this scan's
                # sorted z, so what is kept has the scan's distribution. Keeping the last n in
                # cloud order kept whatever the sensor happened to scan last.
                hist.extend(zs[np.linspace(a, b - 1, n).round().astype(np.int64)].tolist())
        return len(starts)

    def _ground(self, hist):
        if hist is None or len(hist) < self.min_points_per_cell:
            return None
        s = sorted(hist)
        return s[int(len(s) * self.cell_percentile)]

    def cell_ground(self, x, y):
        """One cell's ground, or None if it has fewer than min_points_per_cell points."""
        return self._ground(self._cells.get((self._index(x) + _KEY_OFFSET) * _KEY_SPAN
                                            + self._index(y) + _KEY_OFFSET))

    def ground_at(self, x, y):
        """(ground z, number of known cells in the patch). z is None below min_cells."""
        cx, cy = self._index(x), self._index(y)
        values = []
        for dx, dy in self._patch:
            z = self._ground(self._cells.get((cx + dx + _KEY_OFFSET) * _KEY_SPAN
                                             + cy + dy + _KEY_OFFSET))
            if z is not None:
                values.append(z)
        if len(values) < self.min_cells:
            return None, len(values)
        return float(np.median(values)), len(values)
