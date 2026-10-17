"""The braking envelope and the speed-scaled reach (command_mux/brake_envelope.py). No ROS.

The formula table is the SAME table follow_planner's test_speed_standoff.cpp asserts: the two
implementations must not drift. The rest: a steady follow is untouched; the closing speed ignores a
coasting track's velocity and survives a glitch burst; the extra grows at once and shrinks slowly;
the envelope vetoes a person the aircraft could not stop short of, and nobody it could.
"""
import math

import pytest

from command_mux import brake_envelope as be

# (closing, ground) -> (reach, extra), defaults = no acceleration feed-forward (track 0.4 s/m)
TABLE_NO_FF = [((0.0, 0.0), (1.50, 0.0)), ((1.0, 1.0), (2.65, 0.15)), ((2.0, 2.0), (4.30, 1.5)),
               ((3.0, 3.0), (6.45, 1.5)), ((4.0, 4.0), (9.10, 1.5))]
# with the feed-forward values (track 0.25 s/m, capped 0.25 m)
TABLE_FF = [((0.0, 0.0), (1.50, 0.0)), ((1.0, 1.0), (2.50, 0.0)), ((2.0, 2.0), (3.75, 1.25)),
            ((3.0, 3.0), (5.50, 1.5)), ((4.0, 4.0), (7.75, 1.5))]


@pytest.mark.parametrize('vs,expect', TABLE_NO_FF)
def test_formula_table_defaults(vs, expect):
    p = be.SpeedParams()
    assert be.reach(*vs, p) == pytest.approx(expect[0], abs=1e-9)
    assert be.extra(*vs, p) == pytest.approx(expect[1], abs=1e-9)


@pytest.mark.parametrize('vs,expect', TABLE_FF)
def test_formula_table_feed_forward(vs, expect):
    p = be.SpeedParams(track_s=0.25, track_max_m=0.25)
    assert be.reach(*vs, p) == pytest.approx(expect[0], abs=1e-9)
    assert be.extra(*vs, p) == pytest.approx(expect[1], abs=1e-9)


def test_steady_follow_is_untouched():
    # 09-26 steady follow: closing 0-0.3 m/s at 0.8-0.95 m/s ground speed.
    p = be.SpeedParams()
    for vc in (0.0, 0.1, 0.2, 0.3):
        assert be.extra(vc, 0.95, p) == 0.0
    # the onset: extra stays 0 up to ~0.89 m/s closing at a 0.9 m/s ground speed
    assert be.extra(0.88, 0.9, p) == 0.0
    assert be.extra(0.90, 0.9, p) > 0.0


@pytest.mark.parametrize('bad', [dict(brake_mps2=0.0), dict(release_mps=0.0),
                                 dict(react_s=-0.1), dict(track_s=float('nan')),
                                 dict(max_extra_m=float('inf'))])
def test_bad_parameters_are_refused(bad):
    with pytest.raises(ValueError):
        be.SpeedParams(**bad)


def test_closing_speed():
    ac = (0.0, 0.0)
    p = (5.0, 0.0)
    assert be.closing_speed(ac, (2.0, 0.0), p, [(0.8, 0.0)]) == pytest.approx(1.2)
    assert be.closing_speed(ac, (0.8, 0.0), p, [(1.5, 0.0)]) == 0.0        # walks away faster
    assert be.closing_speed(ac, (1.0, 0.0), p, [(-0.8, 0.0)]) == pytest.approx(1.8)   # toward
    assert be.closing_speed(ac, (0.0, 0.0), p, [(-0.8, 0.0)]) == pytest.approx(0.8)   # we hover
    assert be.closing_speed(ac, (0.0, 2.0), p, [(0.0, 0.0)]) == pytest.approx(0.0)    # sideways
    # a coasting track's velocity is a forecast, and no reports at all: taken as stopped
    assert be.closing_speed(ac, (2.0, 0.0), p, [(0.8, 0.0)], coasting=True) == pytest.approx(2.0)
    assert be.closing_speed(ac, (2.0, 0.0), p, []) == pytest.approx(2.0)


def test_closing_speed_takes_the_slowest_away_report():
    ac, p = (0.0, 0.0), (5.0, 0.0)
    walker = [(0.85, 0.0), (0.87, 0.0), (0.86, 0.0), (0.86, 0.0), (0.86, 0.0)]
    assert be.closing_speed(ac, (2.0, 0.0), p, walker) == pytest.approx(2.0 - 0.85)
    # the 2026-09-26 burst (2.82 / 2.37 / 1.99 m/s for 3 reports) does not shrink the closing speed
    burst = walker[:2] + [(2.82, 0.0), (2.37, 0.0), (1.99, 0.0)]
    assert be.closing_speed(ac, (2.0, 0.0), p, burst) == pytest.approx(2.0 - 0.85)
    # one report that the walker stopped counts at once
    stop = walker[:4] + [(0.0, 0.0)]
    assert be.closing_speed(ac, (2.0, 0.0), p, stop) == pytest.approx(2.0)


def test_velocity_from_poses():
    v = be.VelocityEstimator()
    assert v.velocity() is None
    for k in range(6):
        v.add(10.0 + 0.1 * k, 2.0 * 0.1 * k, -1.0 * 0.1 * k)
    vx, vy = v.velocity()
    assert (vx, vy) == (pytest.approx(2.0), pytest.approx(-1.0))


def test_velocity_needs_a_window_and_forgets_a_gap():
    v = be.VelocityEstimator()
    v.add(0.0, 0.0, 0.0)
    v.add(0.1, 0.2, 0.0)
    assert v.velocity() is None                 # only 0.1 s apart
    v.add(5.0, 3.0, 0.0)                        # a 4.9 s gap: nothing recent enough to pair
    assert v.velocity() is None
    v.add(5.1, 3.1, 0.0)
    v.add(5.2, 3.2, 0.0)
    assert v.velocity()[0] == pytest.approx(1.0)


def test_velocity_clock_backwards_starts_again():
    v = be.VelocityEstimator()
    for k in range(4):
        v.add(100.0 + 0.1 * k, 0.1 * k, 0.0)
    v.add(1.0, 0.0, 0.0)                        # a restarted clock
    assert v.velocity() is None


def test_person_velocities_keep_the_last_five_and_forget():
    pv = be.PersonVelocities()
    for k, vx in enumerate((0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)):
        pv.add(0.1 * k, 7, vx, 0.0)
    assert [v[0] for v in pv.get(7)] == pytest.approx([0.3, 0.4, 0.5, 0.6, 0.7])
    assert pv.get(99) == []
    pv.add(5.0, 8, 1.0, 0.0)                    # track 7 not seen for > 1 s: forgotten
    assert pv.get(7) == []
    pv.add(5.1, 8, float('nan'), 0.0)           # a NaN report is dropped
    assert len(pv.get(8)) == 1


def test_a_track_id_seen_again_after_a_gap_starts_afresh():
    # found by brake_envelope_check B3: a standing person's reports (v 0) survived under the same
    # id, and made a walker going away at 2.5 m/s read as closing at 2.5 m/s
    pv = be.PersonVelocities()
    for k in range(5):
        pv.add(0.1 * k, 1, 0.0, 0.0)
    pv.add(5.0, 1, 2.5, 0.0)
    assert pv.get(1) == [(2.5, 0.0)]


def test_reach_filter_grows_at_once_holds_then_shrinks_slowly():
    p = be.SpeedParams()                        # hold 1.0 s, shrink 0.5 m/s
    f = be.ReachFilter(p)
    assert f.update(0.0, 0.0) == 0.0
    assert f.update(0.02, 1.2) == pytest.approx(1.2)       # at once
    assert f.update(0.5, 0.0) == pytest.approx(1.2)        # held
    assert f.update(1.0, 0.0) == pytest.approx(1.2)        # still inside hold_s of the peak
    v = f.update(1.2, 0.0)
    assert v == pytest.approx(1.2 - 0.5 * 0.2)             # shrinking at 0.5 m/s
    assert f.update(1.4, 0.9) == pytest.approx(max(0.9, 1.1 - 0.1))
    t, last = 1.4, f.value
    while t < 6.0:
        t += 0.02
        now = f.update(t, 0.0)
        assert now <= last + 1e-12 and last - now <= 0.5 * 0.02 + 1e-9
        last = now
    assert f.value == 0.0


def test_reach_filter_does_not_breathe_on_a_noisy_signal():
    p = be.SpeedParams()
    f = be.ReachFilter(p)
    want = [0.6 + 0.3 * math.sin(k) for k in range(200)]     # +/-0.3 m of noise at 50 Hz
    out = [f.update(0.02 * k, w) for k, w in enumerate(want)]
    # the input swings 0.6 m peak to peak; after the first second the output stays within the
    # goal deadband (0.15 m), so the goal does not breathe with it
    assert max(want[50:]) - min(want[50:]) > 0.55
    assert max(out[50:]) - min(out[50:]) <= 0.15


def test_braking_positions():
    p = be.SpeedParams()
    at_rest = be.braking_positions(1.0, 2.0, 0.0, 0.0, p)
    assert all((x, y) == (1.0, 2.0) for _, x, y in at_rest)
    path = be.braking_positions(0.0, 0.0, 3.0, 0.0, p)
    ds = [x for _, x, _ in path]
    assert ds == sorted(ds)
    assert ds[-1] == pytest.approx(be.stop_distance(3.0, p) + be.track_margin(3.0, p))
    assert path[-1][0] == pytest.approx(0.5 + 3.0 / 2.0)


def _standing(x, y=0.0, track=1):
    return (track, [(0.0, x, y, 0.2)])


def _walking(x, vx, y=0.0, vy=0.0, track=1):
    fc = [(k * 0.075, x + vx * k * 0.075, y + vy * k * 0.075, 0.2) for k in range(1, 21)]
    return (track, [(0.0, x, y, 0.2)] + fc)


def test_envelope_standing_person_dead_ahead():
    p = be.SpeedParams()
    path = be.braking_positions(0.0, 0.0, 3.0, 0.0, p)
    stop = be.stop_distance(3.0, p) + be.track_margin(3.0, p)     # 4.95 m
    assert be.envelope_violation(path, [_standing(stop + 1.4)], 1.5) is not None
    assert be.envelope_violation(path, [_standing(stop + 1.6)], 1.5) is None


def test_envelope_walking_away_at_our_speed_is_clear():
    p = be.SpeedParams()
    path = be.braking_positions(0.0, 0.0, 3.0, 0.0, p)
    assert be.envelope_violation(path, [_walking(4.0, 3.0)], 1.5) is None


def test_envelope_person_behind_is_clear():
    p = be.SpeedParams()
    path = be.braking_positions(0.0, 0.0, 3.0, 0.0, p)
    assert be.envelope_violation(path, [_standing(-2.0)], 1.5) is None


def test_envelope_crosser_timing():
    p = be.SpeedParams()
    path = be.braking_positions(0.0, 0.0, 2.0, 0.0, p)            # stops after 1.5 s, 2.8 m
    # crosses the path at x = 2.0 about 0.9 s from now: in the window
    assert be.envelope_violation(path, [_walking(2.0, 0.0, y=-1.2, vy=1.4)], 1.5) is not None
    # crosses far ahead (x = 6): the aircraft stops short of it
    assert be.envelope_violation(path, [_walking(6.0, 0.0, y=-1.2, vy=1.4)], 1.5) is None


def test_envelope_at_rest_is_no_wider_than_the_point_check():
    p = be.SpeedParams()
    path = be.braking_positions(0.0, 0.0, 0.0, 0.0, p)
    assert be.envelope_violation(path, [_standing(1.49)], 1.5) is not None
    assert be.envelope_violation(path, [_standing(1.51)], 1.5) is None


def test_person_at_holds_the_last_forecast_point():
    pts = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (1.5, 1.5, 0.0, 0.0)]
    assert be.person_at(pts, 0.5)[0] == pytest.approx(0.5)
    assert be.person_at(pts, 3.0)[0] == pytest.approx(1.5)


def test_extend_segment():
    assert be.extend_segment(0.0, 0.0, 3.0, 4.0, 1.0) == (pytest.approx(3.6), pytest.approx(4.8))
    assert be.extend_segment(0.0, 0.0, 3.0, 4.0, 0.0) == (3.0, 4.0)
    assert be.extend_segment(1.0, 1.0, 1.0, 1.0, 1.0) == (1.0, 1.0)
