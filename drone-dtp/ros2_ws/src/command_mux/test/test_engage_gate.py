"""EngageGate with a fake clock. No ROS.

The properties that matter: shut until the first fresh armed + Offboard report; open on it; a
stale report never changes it (never opens it, and above all never shuts it in the air); a fresh
takeover or disarm shuts it; a re-engage opens it again.
"""
from command_mux.engage_gate import EngageGate, ParkBurst

STALE = 3.0


def test_shut_until_engaged():
    g = EngageGate(STALE)
    assert not g.is_open
    assert g.update(0.0, None, False, False) is None           # never a report
    assert g.update(1.0, 1.0, True, False) is None             # armed, Position
    assert g.update(1.5, 1.5, False, True) is None             # Offboard, disarmed
    assert not g.is_open


def test_opens_on_fresh_armed_offboard():
    g = EngageGate(STALE)
    assert g.update(10.0, 10.0, True, True) == 'opened'
    assert g.is_open and g.opened == 1
    assert g.update(10.5, 10.5, True, True) is None            # no edge on a repeat


def test_stale_engaged_report_does_not_open():
    g = EngageGate(STALE)
    assert g.update(10.0, 6.0, True, True) is None
    assert not g.is_open


def test_stale_report_never_shuts_it():
    """The DDS agent dropping mid-flight must not silence the mux's HOLD goals."""
    g = EngageGate(STALE)
    g.update(10.0, 10.0, True, True)
    for t in (13.1, 20.0, 600.0):
        assert g.update(t, 10.0, True, True) is None
        assert g.is_open
    # even when the last report said something else, a stale one is not believed
    assert g.update(700.0, 10.0, False, False) is None
    assert g.is_open


def test_takeover_and_disarm_shut_it():
    g = EngageGate(STALE)
    g.update(10.0, 10.0, True, True)
    assert g.update(20.0, 20.0, True, False) == 'closed'       # pilot flips to Position
    assert not g.is_open
    assert g.update(21.0, 21.0, False, False) is None
    g.update(30.0, 30.0, True, True)
    assert g.update(31.0, 31.0, False, True) == 'closed'       # disarmed in Offboard


def test_reengage_opens_again():
    g = EngageGate(STALE)
    g.update(10.0, 10.0, True, True)
    g.update(20.0, 20.0, True, False)
    assert g.update(25.0, 25.0, True, True) == 'opened'
    assert g.is_open and g.opened == 2


def test_boundary_is_fresh():
    g = EngageGate(STALE)
    assert g.update(13.0, 10.0, True, True) == 'opened'        # exactly stale_s old: fresh


def test_park_burst_idle_until_started():
    b = ParkBurst(3, 0.3)
    assert not b.due(0.0) and not b.due(100.0)
    assert b.sent == 0


def test_park_burst_sends_repeats_at_the_period():
    b = ParkBurst(3, 0.3)
    b.start(10.0)
    assert b.due(10.0)                       # the first one on the closing tick itself
    assert not b.due(10.02) and not b.due(10.29)
    assert b.due(10.30)
    assert not b.due(10.5)
    assert b.due(10.61)
    assert not b.due(11.0) and not b.due(60.0)   # the burst is over
    assert b.sent == 3


def test_park_burst_cancelled_by_a_reengage():
    """A park that lands after the gate re-opened would drop the goal SUPER is flying."""
    b = ParkBurst(3, 0.3)
    b.start(10.0)
    assert b.due(10.0)
    b.cancel()
    assert not b.due(10.3) and not b.due(11.0)
    assert b.sent == 1


def test_park_burst_restarts_on_the_next_takeover():
    b = ParkBurst(2, 0.5)
    b.start(10.0)
    assert b.due(10.0) and b.due(10.5) and not b.due(11.0)
    b.start(20.0)
    assert b.due(20.0) and b.due(20.5) and not b.due(21.0)
    assert b.sent == 4
