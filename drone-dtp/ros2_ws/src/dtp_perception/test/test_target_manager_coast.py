"""Tests for target_manager's coast floor (min_coast_horizon_s).

*** WHY THIS FILE EXISTS ***

On 2026-09-12 every fresh lock that missed one frame in its first second went LOST with
"coasted 0.07 s of a 0.20 s horizon" -- indoors at 8 Hz and outdoors at dusk. A new track arrives
with a one-point forecast at dt = latency (perception will not invent a path before the predictor
has warmed up), and target_manager coasted only as far as that forecast reached. With
auto_reacquire_after_lost false, each of those was a manual re-engage in the field.

These tests drive _remember_seen -- the one place the coast polyline is finalized -- on a bare
object (no ROS graph, no executor) and check the polyline it produces. The case that matters is
test_one_point_forecast_is_extended_to_the_floor: the exact message shape from the field.
"""
import math
import types

import pytest

from dtp_msgs.msg import Target, TargetArray
from dtp_perception import target_manager as tm


def _bare(min_floor=1.2, coast_timeout=1.5):
    """A TargetManagerNode with only the state _remember_seen touches, and no rclpy."""
    node = object.__new__(tm.TargetManagerNode)
    node._coast_ts = []
    node._coast_pts = []
    node._coast_frames = 0
    node._coast_timeout = coast_timeout
    node._min_coast_horizon = min_floor
    node._latency_s = 0.0
    node._locked_msg = None
    node._last_seen_arrival = None
    quiet = types.SimpleNamespace(warn=lambda *a, **k: None, error=lambda *a, **k: None)
    node.get_logger = lambda: quiet
    return node


def _target(x, y, z, vx, vy, path):
    """path: list of (dt, x, y) exactly as perception_node._predicted_path emits it."""
    t = Target()
    t.track_id = 7
    t.header.frame_id = 'lidar_init'
    t.header.stamp.sec = 100
    t.position.x, t.position.y, t.position.z = x, y, z
    t.velocity.x, t.velocity.y = vx, vy
    for dt, px, py in path:
        p = type(t.position)()
        p.x, p.y, p.z = px, py, z
        t.predicted_path.append(p)
        t.predicted_path_dt.append(dt)
    arr = TargetArray()
    arr.header = t.header
    arr.targets.append(t)
    return t, arr


def test_one_point_forecast_is_extended_to_the_floor():
    """THE FIELD CASE: a fresh track, one forecast point at dt=0.20, zero velocity."""
    node = _bare()
    tgt, arr = _target(4.0, 0.0, 0.9, 0.0, 0.0, [(0.20, 4.0, 0.0)])
    node._remember_seen(tgt, arr, now=1000.0)
    assert node._coast_ts[0] == 0.0
    assert node._coast_ts[-1] == pytest.approx(1.2), 'horizon must reach the floor'
    # Stationary extension: zero velocity means the person is assumed to stay put.
    assert node._coast_pts[-1] == pytest.approx((4.0, 0.0, 0.9))
    # And the coast reference exists well past 0.20 s, where it previously returned None.
    node._last_seen_arrival = 1000.0
    assert node._coast_reference(now=1000.0 + 0.8) is not None
    assert node._coast_reference(now=1000.0 + 1.1) is not None


def test_extension_follows_the_reported_velocity():
    """A track with velocity is coasted along it -- the zero-turn-rate case of CTRV."""
    node = _bare()
    tgt, arr = _target(4.0, 0.0, 0.9, 1.0, 0.5, [(0.20, 4.2, 0.1)])
    node._remember_seen(tgt, arr, now=1000.0)
    # From the last forecast point (4.2, 0.1) at 0.20 s, 1.0 s more at (1.0, 0.5) m/s.
    assert node._coast_pts[-1] == pytest.approx((5.2, 0.6, 0.9))
    assert node._coast_ts[-1] == pytest.approx(1.2)


def test_full_forecast_is_left_alone():
    """A warmed-up predictor's 1.5 s forecast is longer than the floor: nothing is appended."""
    node = _bare()
    path = [(0.20 + 0.075 * i, 4.0 + 0.1 * i, 0.0) for i in range(18)]   # ends at ~1.475 s
    tgt, arr = _target(4.0, 0.0, 0.9, 1.3, 0.0, path)
    node._remember_seen(tgt, arr, now=1000.0)
    assert len(node._coast_ts) == 1 + len(path), 'no extension point should be added'
    assert node._coast_ts[-1] == pytest.approx(path[-1][0])


def test_floor_is_capped_by_coast_timeout():
    """The outer bound still wins: a floor above coast_timeout_s does not extend past it."""
    node = _bare(min_floor=5.0, coast_timeout=1.5)
    tgt, arr = _target(4.0, 0.0, 0.9, 0.0, 0.0, [(0.20, 4.0, 0.0)])
    node._remember_seen(tgt, arr, now=1000.0)
    assert node._coast_ts[-1] == pytest.approx(1.5)


def test_floor_zero_restores_old_behaviour():
    """min_coast_horizon_s: 0 must reproduce the pre-2026-09-12 polyline exactly."""
    node = _bare(min_floor=0.0)
    tgt, arr = _target(4.0, 0.0, 0.9, 0.0, 0.0, [(0.20, 4.0, 0.0)])
    node._remember_seen(tgt, arr, now=1000.0)
    assert node._coast_ts == pytest.approx([0.0, 0.20])
    node._last_seen_arrival = 1000.0
    assert node._coast_reference(now=1000.0 + 0.5) is None, 'old behaviour: exhausted past 0.20 s'


def test_non_finite_velocity_coasts_stationary():
    """A NaN velocity must not poison the polyline; extension falls back to stationary."""
    node = _bare()
    tgt, arr = _target(4.0, 0.0, 0.9, float('nan'), 0.0, [(0.20, 4.0, 0.0)])
    node._remember_seen(tgt, arr, now=1000.0)
    x, y, z = node._coast_pts[-1]
    assert all(math.isfinite(v) for v in (x, y, z))
    assert (x, y) == pytest.approx((4.0, 0.0))


def test_latency_is_still_read_from_the_message():
    """The extension must not disturb _latency_s, which is predicted_path_dt[0] by contract."""
    node = _bare()
    tgt, arr = _target(4.0, 0.0, 0.9, 0.0, 0.0, [(0.20, 4.0, 0.0)])
    node._remember_seen(tgt, arr, now=1000.0)
    assert node._latency_s == pytest.approx(0.20)
