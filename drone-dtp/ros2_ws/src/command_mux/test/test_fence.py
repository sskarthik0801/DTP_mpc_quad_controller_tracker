"""Fence geometry. No ROS.

The properties that matter, each as a test: a goal inside is untouched; a goal outside lands on
the inset edge, never on the true edge; the clamp is the nearest inset point (so a person walking
along outside the fence drags the goal along the edge, not to a corner); rotation is applied and
inverted consistently; breach distance is Euclidean, so a corner is not closer than a side; a
NaN reads as the worst case in every direction (inf breach, centre clamp), never as "inside";
and the constructor refuses a box with no room to fly in.
"""
import math

import pytest

from command_mux.fence import Fence


def _box(**kw):
    args = dict(x_min=-2.0, x_max=28.0, y_min=-2.0, y_max=2.0, yaw_rad=0.0, margin_m=0.5,
                breach_land_m=3.0)
    args.update(kw)
    return Fence(**args)


def test_inside_is_untouched():
    f = _box()
    for x, y in ((0.0, 0.0), (10.0, 1.4), (27.4, -1.49), (-1.5, 0.0)):
        cx, cy, moved = f.clamp(x, y)
        assert (cx, cy) == (x, y)
        assert moved == 0.0
        assert f.outside_by(x, y) == 0.0
        assert f.contains(x, y)
        assert not f.breached(x, y)


def test_outside_lands_on_the_inset_edge_not_the_true_edge():
    f = _box()
    cx, cy, moved = f.clamp(10.0, 5.0)          # 3 m past the +y side
    assert cx == pytest.approx(10.0)
    assert cy == pytest.approx(1.5)              # y_max 2.0 - margin 0.5
    assert moved == pytest.approx(3.5)
    cx, cy, moved = f.clamp(40.0, 0.0)          # 12 m past the far end
    assert (cx, cy) == pytest.approx((27.5, 0.0))
    cx, cy, moved = f.clamp(-9.0, -9.0)         # past a corner
    assert (cx, cy) == pytest.approx((-1.5, -1.5))


def test_between_edge_and_inset_is_pulled_in():
    # A goal 0.2 m inside the true edge is still outside the INSET box and is moved. This is
    # what keeps SUPER's planned path -- which wanders around the goal -- off the fence line.
    f = _box()
    cx, cy, moved = f.clamp(10.0, 1.8)
    assert (cx, cy) == pytest.approx((10.0, 1.5))
    assert moved == pytest.approx(0.3)
    assert f.outside_by(10.0, 1.8) == 0.0        # ...but it is NOT a breach


def test_clamp_is_nearest_point_so_it_slides_along_the_edge():
    f = _box()
    ys = [f.clamp(x, 6.0)[1] for x in (5.0, 6.0, 7.0)]
    xs = [f.clamp(x, 6.0)[0] for x in (5.0, 6.0, 7.0)]
    assert ys == pytest.approx([1.5, 1.5, 1.5])
    assert xs == pytest.approx([5.0, 6.0, 7.0])


def test_breach_distance_is_euclidean():
    f = _box()
    assert f.outside_by(10.0, 4.0) == pytest.approx(2.0)          # side
    assert f.outside_by(31.0, 0.0) == pytest.approx(3.0)          # end
    assert f.outside_by(31.0, 6.0) == pytest.approx(5.0)          # corner: hypot(3, 4)
    assert not f.breached(10.0, 4.0)           # 2.0 < 3.0
    assert not f.breached(31.0, 0.0)           # 3.0 is not > 3.0
    assert f.breached(31.0, 6.0)               # 5.0 > 3.0
    assert f.breached(10.0, 5.01)


def test_hard_boundary_can_be_disabled_but_clamp_cannot():
    f = _box(breach_land_m=-1.0)
    assert not f.breached(1000.0, 1000.0)
    assert f.outside_by(1000.0, 0.0) == pytest.approx(972.0)
    assert f.clamp(1000.0, 0.0)[:2] == pytest.approx((27.5, 0.0))


def test_rotation_round_trips_and_rotates_the_box():
    f = _box(yaw_rad=math.radians(90.0))
    # The 30 m side now runs along world +y; world +x is the box's -v side.
    for x, y in ((0.3, 5.0), (-1.2, 27.0), (1.9, -1.9)):
        u, v = f.to_box(x, y)
        assert f.from_box(u, v) == pytest.approx((x, y))
    assert f.contains(0.0, 27.0)
    assert not f.contains(27.0, 0.0)
    assert f.outside_by(27.0, 0.0) == pytest.approx(25.0)
    cx, cy, moved = f.clamp(5.0, 10.0)                   # 3 m past what is now the -v side
    assert (cx, cy) == pytest.approx((1.5, 10.0))
    assert moved == pytest.approx(3.5)
    # 45 degrees: a point straight down the corridor axis is inside, its mirror is not.
    g = _box(yaw_rad=math.radians(45.0))
    d = 20.0 / math.sqrt(2.0)
    assert g.contains(d, d)
    assert not g.contains(d, -d)


def test_nan_is_the_worst_case_never_inside():
    f = _box()
    for x, y in ((math.nan, 0.0), (0.0, math.nan), (math.inf, 0.0)):
        assert f.outside_by(x, y) == math.inf
        assert not f.contains(x, y)
        assert f.breached(x, y)
        cx, cy, moved = f.clamp(x, y)
        assert moved == math.inf
        assert f.contains(cx, cy)
        assert (cx, cy) == pytest.approx((13.0, 0.0))     # the centre


def test_refuses_a_box_with_no_room():
    with pytest.raises(ValueError):
        _box(x_min=0.0, x_max=1.0, margin_m=0.5)           # 1.0 - 1.0 = 0 < 0.5
    with pytest.raises(ValueError):
        _box(y_min=2.0, y_max=-2.0)                        # inverted
    with pytest.raises(ValueError):
        _box(margin_m=-0.1)
    with pytest.raises(ValueError):
        _box(x_max=math.nan)
    # Exactly the minimum interior is accepted.
    Fence(0.0, 1.5, 0.0, 1.5, margin_m=0.5)


def test_plan_default_box_contains_origin_with_room():
    f = _box()
    assert f.contains(0.0, 0.0)
    assert f.clamp(0.0, 0.0)[2] == 0.0
    assert '30.0 x 4.0 m' in f.describe()
