"""Terrain offset. No ROS.

The properties that matter, each as a test: off Offboard it takes the estimate at once; in
Offboard it moves at no more than the down / up rates, climbing faster than descending; a change
inside the deadband is not taken; it is clamped; a stale, NaN or missing estimate HOLDS it (never
decays to 0); and a long gap between ticks cannot become one big step.
"""
import math

import pytest

from command_mux.terrain import TerrainOffset

REF = -0.15


def _offset(**kw):
    args = dict(ref_z=REF, max_up_m=1.5, max_down_m=1.2, rate_up_mps=0.3, rate_down_mps=0.2,
                deadband_m=0.05, timeout_s=1.0)
    args.update(kw)
    return TerrainOffset(**args)


def _run(o, t0, t1, ground, engaged=True, dt=0.1):
    t = t0
    while t <= t1 + 1e-9:
        o.update(t, ground, t, engaged)
        t += dt
    return o.offset


def test_takeoff_ground_is_zero():
    o = _offset()
    assert o.update(0.0, REF, 0.0, False) == 0.0
    assert o.status == 'ok'


def test_not_engaged_snaps():
    o = _offset()
    assert o.update(0.0, -1.15, 0.0, False) == pytest.approx(-1.0)


def test_engaged_descends_at_the_down_rate():
    o = _offset()
    o.update(0.0, REF, 0.0, True)
    off = _run(o, 0.1, 2.0, -1.15)
    assert off == pytest.approx(-0.4, abs=0.021)          # 2 s x 0.2 m/s
    off = _run(o, 2.1, 10.0, -1.15)
    assert off == pytest.approx(-1.0, abs=0.05)            # arrives, inside the deadband


def test_engaged_climbs_at_the_up_rate():
    o = _offset()
    o.update(0.0, REF, 0.0, True)
    off = _run(o, 0.1, 2.0, 0.85)
    assert off == pytest.approx(0.6, abs=0.031)            # 2 s x 0.3 m/s


def test_deadband():
    o = _offset()
    o.update(0.0, REF, 0.0, True)
    assert _run(o, 0.1, 5.0, REF + 0.04) == 0.0


@pytest.mark.parametrize('ground,expected', [(-3.0, -1.2), (3.0, 1.5)])
def test_clamped(ground, expected):
    o = _offset()
    assert o.update(0.0, ground, 0.0, False) == pytest.approx(expected)
    assert o.status == 'clamped'


def test_stale_nan_and_missing_hold():
    o = _offset()
    assert o.update(0.0, None, None, True) == 0.0 and o.status == 'none'
    o.update(0.1, -1.15, 0.1, False)                       # -1.0
    assert o.update(5.0, 0.85, 3.5, True) == pytest.approx(-1.0)   # 1.5 s old
    assert o.status == 'stale'
    assert o.update(5.1, math.nan, 5.1, True) == pytest.approx(-1.0)
    assert o.status == 'unknown'


def test_a_long_gap_is_not_a_big_step():
    o = _offset()
    o.update(0.0, REF, 0.0, True)
    o.update(30.0, -1.15, 30.0, True)                      # 30 s between ticks
    assert o.offset == pytest.approx(-0.1)                 # capped at 0.5 s x 0.2 m/s


@pytest.mark.parametrize('bad', [dict(rate_up_mps=-0.1), dict(timeout_s=math.nan),
                                 dict(ref_z=math.inf)])
def test_constructor_refuses_bad_numbers(bad):
    with pytest.raises(ValueError):
        _offset(**bad)
