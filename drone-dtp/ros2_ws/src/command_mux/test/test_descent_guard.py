"""Descent guard latch. No ROS.

The properties that matter, each as a test: nobody near means no change; a goal below the drone
with a person near latches the drone's altitude; the floor does not flap when the person sits on
the radius (the 2026-09-15 flight); the floor does not follow the drone down or creep up on
altitude-hold noise, but does rise on a real climb; it releases only after the margin AND the
dwell; a goal above the floor is never lowered; a NaN person is skipped; no odom keeps the floor;
reset() drops it. And (2026-09-21) a climb made by the mux's own HOLD neither raises the floor nor
sets a new latch at the climbed altitude, until the drone is back down.
"""
import math

import pytest

from command_mux.descent_guard import DescentGuard

DRONE = (0.0, 0.0, 1.5)
GOAL = (1.0, 0.0, 0.5)


def _guard(**kw):
    args = dict(radius_m=3.0, release_margin_m=0.5, release_dwell_s=1.0, raise_margin_m=0.3)
    args.update(kw)
    return DescentGuard(**args)


def _person_from_goal(d, track=1):
    """A person on the +x axis, d metres from GOAL and further than that from DRONE."""
    return (track, GOAL[0] + d, 0.0)


def _apply(g, t, people, drone=DRONE, goal=GOAL):
    return g.apply(t, goal[0], goal[1], goal[2], drone, people)


def test_nobody_near_changes_nothing():
    g = _guard()
    z, event, _ = _apply(g, 0.0, [])
    assert (z, event, g.active) == (0.5, None, False)
    z, event, _ = _apply(g, 0.1, [_person_from_goal(3.2)])
    assert (z, event, g.active) == (0.5, None, False)


def test_goal_below_with_person_near_latches_drone_altitude():
    g = _guard()
    z, event, cause = _apply(g, 0.0, [_person_from_goal(2.8, track=7)])
    assert z == pytest.approx(1.5)
    assert event == 'latched'
    assert cause[0] == 7
    assert g.floor_z == pytest.approx(1.5)


def test_person_near_the_drone_also_latches():
    g = _guard()
    z, event, _ = _apply(g, 0.0, [(2, -2.0, 0.0)])   # 2 m behind the drone, 3 m from the goal
    assert (z, event) == (pytest.approx(1.5), 'latched')


def test_goal_above_the_drone_does_not_latch_and_is_never_lowered():
    g = _guard()
    z, event, _ = _apply(g, 0.0, [_person_from_goal(1.0)], goal=(1.0, 0.0, 2.0))
    assert (z, event, g.active) == (2.0, None, False)
    _apply(g, 0.1, [_person_from_goal(2.8)])              # latch at 1.5
    z, _, _ = _apply(g, 0.2, [_person_from_goal(2.8)], goal=(1.0, 0.0, 2.5))
    assert z == 2.5


def test_person_on_the_radius_does_not_flap():
    # The 2026-09-15 20:08 flight: the goal sat 2.77-3.07 m from the walker. The old guard
    # alternated 0.5 / drone altitude; the latch must hold one value throughout.
    g = _guard()
    out = []
    for i in range(60):
        d = 2.8 if i % 2 == 0 else 3.2
        z, _, _ = _apply(g, i * 0.1, [_person_from_goal(d)])
        out.append(z)
    assert out == [pytest.approx(1.5)] * 60


def test_floor_does_not_follow_the_drone_down():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    z, event, _ = _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 0.7))  # a dip
    assert (z, event) == (pytest.approx(1.5), None)
    assert g.floor_z == pytest.approx(1.5)


def test_floor_ignores_small_rises_and_follows_real_climbs():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    z, event, _ = _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.72))
    assert (z, event) == (pytest.approx(1.5), None)       # +0.22, the measured tracking error
    z, event, _ = _apply(g, 0.2, [_person_from_goal(2.8)], drone=(0.0, 0.0, 2.25))
    assert (z, event) == (pytest.approx(2.25), 'raised')  # a climb over an obstacle
    z, _, _ = _apply(g, 0.3, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.9))
    assert z == pytest.approx(2.25)


def test_release_needs_the_margin_and_the_dwell():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    # 3.3 m: outside the trip radius but inside radius + margin -> never releases.
    for i in range(1, 40):
        z, event, _ = _apply(g, i * 0.1, [_person_from_goal(3.3)])
        assert (z, event) == (pytest.approx(1.5), None)
    # 5 m: clear. Still latched until the dwell has passed.
    z, event, _ = _apply(g, 10.0, [_person_from_goal(5.0)])
    assert (z, event) == (pytest.approx(1.5), None)
    z, event, _ = _apply(g, 10.9, [_person_from_goal(5.0)])
    assert (z, event) == (pytest.approx(1.5), None)
    z, event, _ = _apply(g, 11.0, [_person_from_goal(5.0)])
    assert (z, event, g.active) == (0.5, 'released', False)


def test_a_person_returning_restarts_the_dwell():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    _apply(g, 1.0, [])                                   # clear from t=1.0
    _apply(g, 1.8, [_person_from_goal(3.2)])             # back inside radius + margin
    z, event, _ = _apply(g, 2.5, [])                     # clear again from t=2.5
    assert (z, event) == (pytest.approx(1.5), None)
    z, event, _ = _apply(g, 3.4, [])
    assert (z, event) == (pytest.approx(1.5), None)
    z, event, _ = _apply(g, 3.5, [])
    assert event == 'released'


def test_relatches_after_release():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    _apply(g, 1.0, [])
    _apply(g, 2.0, [])
    assert not g.active
    z, event, _ = _apply(g, 2.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.1))
    assert (z, event) == (pytest.approx(1.1), 'latched')


def test_nan_person_is_skipped():
    g = _guard()
    z, event, _ = _apply(g, 0.0, [(1, math.nan, 0.0), (2, 1.5, math.inf)])
    assert (z, event, g.active) == (0.5, None, False)


def test_no_odom_keeps_the_floor_and_restarts_the_dwell():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    _apply(g, 1.0, [])                                   # clear from t=1.0
    z, event, _ = _apply(g, 1.5, [], drone=None)
    assert (z, event) == (pytest.approx(1.5), None)
    z, event, _ = _apply(g, 2.2, [])                     # dwell restarted at 2.2
    assert (z, event) == (pytest.approx(1.5), None)
    z, event, _ = _apply(g, 3.2, [])
    assert event == 'released'


def test_no_odom_never_latches():
    g = _guard()
    z, event, _ = _apply(g, 0.0, [_person_from_goal(1.0)], drone=None)
    assert (z, event, g.active) == (0.5, None, False)


def test_reset_drops_the_floor():
    g = _guard()
    assert g.reset() is None
    _apply(g, 0.0, [_person_from_goal(2.8)])
    assert g.reset() == pytest.approx(1.5)
    assert not g.active
    z, _, _ = _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.0))
    assert z == pytest.approx(1.0)                       # re-latched at the new altitude


def test_hold_climb_does_not_raise_the_floor():
    # The 2026-09-21 18:41 flight: floor 1.46, a HOLD climbed the drone 0.75 m, the floor went to
    # 2.10 and stayed there for the rest of the follow.
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])                       # floor 1.5
    g.hold_started(1.5)
    z, event, _ = _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 2.25))
    assert (z, event) == (pytest.approx(1.5), None)                # during the HOLD
    g.hold_ended()
    z, event, _ = _apply(g, 0.2, [_person_from_goal(2.8)], drone=(0.0, 0.0, 2.25))
    assert (z, event) == (pytest.approx(1.5), None)                # FOLLOW again, still up there
    assert g.floor_z == pytest.approx(1.5)


def test_back_down_after_a_hold_real_climbs_raise_again():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    g.hold_started(1.5)
    _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 2.25))
    g.hold_ended()
    _apply(g, 0.2, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.6))   # back down
    assert g.hold_base_z is None
    z, event, _ = _apply(g, 0.3, [_person_from_goal(2.8)], drone=(0.0, 0.0, 2.1))
    assert (z, event) == (pytest.approx(2.1), 'raised')


def test_back_down_during_the_hold_does_not_end_it():
    # At HOLD entry the drone is AT the entry altitude; that must not clear the exclusion before
    # the climb has even started.
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    g.hold_started(1.5)
    _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.5))
    z, event, _ = _apply(g, 0.2, [_person_from_goal(2.8)], drone=(0.0, 0.0, 2.25))
    assert (z, event) == (pytest.approx(1.5), None)


def test_hold_climb_does_not_set_a_new_latch_up_there():
    # No floor yet: following at 1.2, a HOLD climbs to 1.95, FOLLOW comes back with the goal at
    # 1.2 and a person near. The latch belongs at 1.2, not 1.95.
    g = _guard()
    g.hold_started(1.2)
    g.hold_ended()
    z, event, _ = _apply(g, 0.0, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.95),
                         goal=(1.0, 0.0, 1.0))
    assert (z, event) == (pytest.approx(1.2), 'latched')


def test_two_holds_keep_the_lower_entry_altitude():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    g.hold_started(1.5)
    g.hold_ended()
    g.hold_started(2.25)                                           # second HOLD, still up
    g.hold_ended()
    z, event, _ = _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 3.0))
    assert (z, event, g.hold_base_z) == (pytest.approx(1.5), None, pytest.approx(1.5))


def test_release_keeps_the_hold_exclusion():
    g = _guard()
    _apply(g, 0.0, [_person_from_goal(2.8)])
    g.hold_started(1.5)
    g.hold_ended()
    up = (0.0, 0.0, 2.25)
    _apply(g, 1.0, [], drone=up)
    z, event, _ = _apply(g, 2.0, [], drone=up)
    assert event == 'released'
    z, event, _ = _apply(g, 2.1, [_person_from_goal(2.8)], drone=up)
    assert (z, event) == (pytest.approx(1.5), 'latched')


def test_clear_hold_forgets_a_hold_that_commanded_nothing():
    # A HOLD on the ground (z 0.02), then a takeoff under the pilot. Without clear_hold the guard
    # would see the drone at 0.02 all flight and never trip; the node calls it while PX4 is not
    # flying the mux's goals.
    g = _guard()
    g.hold_started(0.02)
    z, event, _ = _apply(g, 0.0, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.5),
                         goal=(1.0, 0.0, 1.2))
    assert (z, event) == (1.2, None)                               # the trap, demonstrated
    g.clear_hold()
    z, event, _ = _apply(g, 0.1, [_person_from_goal(2.8)], drone=(0.0, 0.0, 1.5),
                         goal=(1.0, 0.0, 1.2))
    assert (z, event) == (pytest.approx(1.5), 'latched')


def test_reset_forgets_the_hold():
    g = _guard()
    g.hold_started(1.0)
    g.reset()
    assert g.hold_base_z is None


def test_non_finite_hold_altitude_is_ignored():
    g = _guard()
    g.hold_started(math.nan)
    assert g.hold_base_z is None


@pytest.mark.parametrize('bad', [dict(radius_m=-1.0), dict(release_dwell_s=math.nan),
                                 dict(raise_margin_m=math.inf)])
def test_constructor_refuses_bad_numbers(bad):
    with pytest.raises(ValueError):
        _guard(**bad)


# ---- SPEED-SCALED RADII (2026-09-27): extra_radius_m ----

def _script(g, extra=None):
    """One fixed sequence of goals and people; returns every (z, event) the guard gave."""
    out = []
    seq = [(0.0, [_person_from_goal(3.2)]), (0.1, [_person_from_goal(2.8)]),
           (0.2, [_person_from_goal(3.4)]), (0.5, [_person_from_goal(5.0)]),
           (1.6, [_person_from_goal(5.0)]), (1.7, [])]
    for t, people in seq:
        if extra is None:
            z, event, _ = _apply(g, t, people)
        else:
            z, event, _ = g.apply(t, GOAL[0], GOAL[1], GOAL[2], DRONE, people,
                                  extra_radius_m=extra)
        out.append((z, event))
    return out


def test_extra_radius_zero_is_exactly_the_old_behaviour():
    assert _script(_guard()) == _script(_guard(), extra=0.0)


def test_extra_radius_widens_the_trip():
    g = _guard()
    z, event, _ = g.apply(0.0, *GOAL, DRONE, [_person_from_goal(3.3)], extra_radius_m=0.0)
    assert (z, event) == (0.5, None)
    z, event, _ = g.apply(0.1, *GOAL, DRONE, [_person_from_goal(3.3)], extra_radius_m=0.5)
    assert (z, event) == (pytest.approx(1.5), 'latched')


def test_extra_radius_widens_the_release_by_the_same_amount():
    g = _guard()
    g.apply(0.0, *GOAL, DRONE, [_person_from_goal(2.8)], extra_radius_m=1.0)
    # 4.2 m: outside the plain release radius (3.5), inside 3.5 + 1.0: held
    for k in range(20):
        z, event, _ = g.apply(0.1 + 0.1 * k, *GOAL, DRONE, [_person_from_goal(4.2)],
                              extra_radius_m=1.0)
    assert g.active and event is None
    # the extra shrinks back to 0: now 4.2 m is clear, released after the dwell
    for k in range(12):
        z, event, _ = g.apply(2.2 + 0.1 * k, *GOAL, DRONE, [_person_from_goal(4.2)])
    assert not g.active and z == pytest.approx(0.5)


@pytest.mark.parametrize('bad', [float('nan'), -1.0, float('inf')])
def test_bad_extra_radius_is_zero(bad):
    g = _guard()
    z, event, _ = g.apply(0.0, *GOAL, DRONE, [_person_from_goal(3.3)], extra_radius_m=bad)
    assert (z, event) == (0.5, None)
