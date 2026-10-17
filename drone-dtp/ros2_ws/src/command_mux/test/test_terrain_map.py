"""Terrain map. No ROS.

The properties that matter, each as a test: flat ground reads as itself; a slope is followed; a
NARROW ditch, a bush and a single low outlier do not move the estimate, and a WIDE ditch does;
a canopy above the sensor never enters the map; a patch with too few known cells is unknown,
never a guess; a person is never the ground (min_cells, and the tracked-people cut-out); and
cells pool their points across scans (the 2026-09-21 replay: at one point
per cell per scan, a per-scan minimum learned no ground ahead of the aircraft at all).
"""
import math

import numpy as np
import pytest

from command_mux.terrain_map import TerrainMap

SENSOR = (0.0, 0.0, 2.0)


def _grid(x0, x1, y0, y1, step, zfun):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    xs, ys = xs.ravel(), ys.ravel()
    return np.column_stack([xs, ys, zfun(xs, ys)])


def _flat(z):
    return lambda xs, ys: np.full(xs.shape, z)


def _fed(pts, scans=4, **kw):
    m = TerrainMap(**kw)
    for _ in range(scans):
        m.add_scan(pts, SENSOR)
    return m


def test_flat_ground_reads_as_itself():
    m = _fed(_grid(-5, 5, -5, 5, 0.1, _flat(-0.15)))
    z, n = m.ground_at(1.0, 1.0)
    assert z == pytest.approx(-0.15, abs=1e-9)
    assert n >= 25


def test_a_slope_is_followed():
    m = _fed(_grid(-2, 30, -3, 3, 0.1, lambda x, y: -0.05 * x - 0.15))
    for x in (0.0, 10.0, 20.0):
        z, _ = m.ground_at(x, 0.0)
        # the cell percentile sits low inside each cell: within one cell of slope (2.5 cm)
        assert z == pytest.approx(-0.05 * x - 0.15, abs=0.03)


def test_narrow_ditch_does_not_move_the_estimate():
    # 0.5 m wide, 1 m deep, straight under the query point.
    def ditch(xs, ys):
        return np.where(np.abs(xs - 5.0) < 0.25, -1.15, -0.15)
    m = _fed(_grid(0, 10, -3, 3, 0.1, ditch))
    z, _ = m.ground_at(5.0, 0.0)
    assert z == pytest.approx(-0.15, abs=1e-9)


def test_wide_ditch_is_followed():
    def ditch(xs, ys):
        return np.where(np.abs(xs - 10.0) < 2.5, -1.15, -0.15)
    m = _fed(_grid(0, 20, -3, 3, 0.1, ditch))
    z, _ = m.ground_at(10.0, 0.0)
    assert z == pytest.approx(-1.15, abs=1e-9)


def test_bush_does_not_move_the_estimate():
    # A 1 x 1 m bush 0.8 m tall, dense: no ground seen under it.
    pts = _grid(0, 10, -3, 3, 0.1, _flat(-0.15))
    under = (np.abs(pts[:, 0] - 5.0) < 0.5) & (np.abs(pts[:, 1]) < 0.5)
    pts[under, 2] = 0.65
    m = _fed(pts)
    assert m.cell_ground(5.0, 0.0) == pytest.approx(0.65)   # the cell itself reads the bush
    z, _ = m.ground_at(5.0, 0.0)
    assert z == pytest.approx(-0.15, abs=1e-9)                # the patch does not


def _person(x, y, top=1.7):
    """A standing person's returns: a 0.6 m column from the knees up, densely sampled."""
    zs = np.arange(0.3, top, 0.05)
    xs, ys, zz = np.meshgrid(np.arange(x - 0.3, x + 0.3, 0.05),
                             np.arange(y - 0.3, y + 0.3, 0.05), zs)
    return np.column_stack([xs.ravel(), ys.ravel(), zz.ravel()])


def test_a_person_alone_is_never_the_ground():
    # The 2026-09-21 replay started in the air: no ground seen near the walker, only them. At
    # (6.0, -0.5) the column straddles four 0.5 m cells.
    person = _person(6.0, -0.5)
    m = _fed(person)
    assert m.ground_at(6.0, -0.5)[0] is None           # 4 cells < min_cells 8
    m4 = _fed(person, min_cells=4)
    assert m4.ground_at(6.0, -0.5)[0] > 0.3            # what min_cells 4 did: read the body


def test_tracked_people_are_cut_out():
    # A person standing on known ground. Uncut, their cells read the body; cut, those cells are
    # empty (the cut takes the ground within 1 m with them) and the patch reads the ground.
    ground = _grid(0, 12, -3, 3, 0.1, _flat(-0.15))
    person = _person(6.0, -0.5)
    scene = np.vstack([person, person, person, ground])
    uncut = _fed(scene)
    assert uncut.cell_ground(6.1, -0.4) > 0.3
    cut = TerrainMap()
    for _ in range(4):
        cut.add_scan(scene, SENSOR, exclude=[(6.0, -0.5)])
    assert cut.cell_ground(6.1, -0.4) is None
    assert cut.ground_at(6.0, -0.5)[0] == pytest.approx(-0.15)


def test_one_low_outlier_is_not_the_ground():
    pts = _grid(0, 4, 0, 4, 0.05, _flat(-0.15))
    pts = np.vstack([pts, [[1.1, 1.1, -3.0]]])                 # one multipath return
    m = _fed(pts, scans=1)
    assert m.cell_ground(1.1, 1.1) == pytest.approx(-0.15)


def test_canopy_above_the_sensor_never_enters():
    ground = _grid(0, 6, -3, 3, 0.1, _flat(-0.15))
    canopy = _grid(0, 6, -3, 3, 0.1, _flat(3.5))                # overhead, above the 2 m sensor
    m = _fed(np.vstack([ground, canopy]))
    z, _ = m.ground_at(3.0, 0.0)
    assert z == pytest.approx(-0.15)
    m2 = _fed(canopy)
    assert m2.ground_at(3.0, 0.0) == (None, 0)


def test_too_few_known_cells_is_unknown():
    pts = _grid(3.0, 3.5, 0.0, 1.0, 0.05, _flat(-0.15))         # two cells only
    m = _fed(pts)
    z, n = m.ground_at(3.2, 0.3)
    assert z is None and 0 < n < 4


def test_unseen_ground_is_unknown_not_a_hole():
    m = _fed(_grid(0, 5, -3, 3, 0.1, _flat(-0.15)))
    assert m.ground_at(20.0, 0.0) == (None, 0)


def test_cells_pool_points_across_scans():
    # One point per cell per scan: known after min_points_per_cell scans, not before.
    pts = _grid(10.25, 14, -1.75, 2, 0.5, _flat(-1.0))
    m = TerrainMap()
    m.add_scan(pts, SENSOR)
    m.add_scan(pts, SENSOR)
    assert m.ground_at(12.0, 0.0)[0] is None
    m.add_scan(pts, SENSOR)
    assert m.ground_at(12.0, 0.0)[0] == pytest.approx(-1.0)


def test_range_and_height_filters():
    far = _grid(40, 45, -2, 2, 0.1, _flat(-0.15))               # beyond max_range_m 30
    m = _fed(far)
    assert len(m) == 0
    near_sensor = _grid(0, 4, -2, 2, 0.1, _flat(1.95))          # within 0.1 m of the sensor
    m = _fed(near_sensor)
    assert len(m) == 0


def test_non_finite_points_are_dropped():
    pts = _grid(0, 4, -2, 2, 0.1, _flat(-0.15))
    pts[::7, 2] = math.nan
    pts[::11, 0] = math.inf
    m = _fed(pts)
    assert m.ground_at(2.0, 0.0)[0] == pytest.approx(-0.15)


@pytest.mark.parametrize('bad', [dict(cell_m=0.0), dict(patch_radius_m=math.nan),
                                 dict(cell_percentile=1.0), dict(min_cells=0),
                                 dict(point_history=2, min_points_per_cell=3)])
def test_constructor_refuses_bad_numbers(bad):
    with pytest.raises(ValueError):
        TerrainMap(**bad)
