"""AbortEscalation with a fake clock. No ROS, no px4_msgs.

The properties that matter, each as a test: it never fires without authority; it never fires
on PROXIMITY; it needs the condition to PERSIST, not merely recur; a hiccup shorter than the
window lands nobody; it fires exactly once; it retries on TEMPORARILY_REJECTED and stops on
ACCEPTED / DENIED / exhaustion; and it is terminal.
"""
import pytest

from command_mux.abort import (AbortEscalation, RESULT_ACCEPTED, RESULT_DENIED,
                               RESULT_IN_PROGRESS, RESULT_TEMPORARILY_REJECTED)


def _esc():
    return AbortEscalation(after_s=2.0, repeat_period_s=0.5, max_sends=4)


def _drive(esc, codes, authority, t0, t1, dt=0.02):
    """Tick from t0 to t1; return the (time, reason) of the fire, or None."""
    t = t0
    while t <= t1 + 1e-9:
        r = esc.observe(t, codes, authority)
        if r is not None:
            return t, r
        t += dt
    return None


def test_no_authority_never_fires():
    esc = _esc()
    assert _drive(esc, ['SENSORS'], False, 0.0, 30.0) is None
    assert not esc.fired


def test_proximity_alone_never_fires():
    esc = _esc()
    assert _drive(esc, ['PROXIMITY'], True, 0.0, 30.0) is None
    assert _drive(esc, ['GOAL_INVALID'], True, 0.0, 30.0) is None


def test_fires_after_window_and_only_once():
    esc = _esc()
    hit = _drive(esc, ['SENSORS'], True, 0.0, 10.0)
    assert hit is not None
    t, reason = hit
    assert 2.0 <= t < 2.05
    assert 'SENSORS' in reason
    # subsequent ticks never report a second fire
    assert _drive(esc, ['SENSORS'], True, t, t + 5.0) is None
    assert esc.fired


def test_short_hiccup_does_not_fire_and_timer_resets():
    esc = _esc()
    assert _drive(esc, ['SENSORS'], True, 0.0, 1.5) is None       # 1.5 s blind
    assert _drive(esc, [], True, 1.52, 1.6) is None                 # recovers
    assert esc.since is None
    assert _drive(esc, ['SENSORS'], True, 1.62, 3.5) is None       # 1.9 s blind again
    assert not esc.fired
    # a recurring-but-never-persistent fault never lands the drone


def test_losing_authority_mid_window_resets():
    esc = _esc()
    assert _drive(esc, ['ODOM'], True, 0.0, 1.9) is None
    assert _drive(esc, ['ODOM'], False, 1.92, 2.5) is None         # pilot took it
    assert esc.since is None
    hit = _drive(esc, ['ODOM'], True, 2.52, 10.0)
    assert hit is not None and hit[0] >= 4.5                        # full window again


def test_send_schedule_retry_then_accept():
    esc = _esc()
    _drive(esc, ['TARGETS'], True, 0.0, 3.0)
    assert esc.fired
    t = 3.0
    assert esc.send_due(t)
    esc.note_sent(t)
    assert not esc.send_due(t + 0.1)
    assert esc.note_ack(RESULT_TEMPORARILY_REJECTED) == 'retry'
    assert esc.send_due(t + 0.5)
    esc.note_sent(t + 0.5)
    assert esc.note_ack(RESULT_ACCEPTED) == 'accepted'
    assert esc.done
    assert not esc.send_due(t + 5.0)
    assert esc.sends == 2


def test_in_progress_counts_as_accepted():
    esc = _esc()
    _drive(esc, ['SENSORS'], True, 0.0, 3.0)
    esc.note_sent(3.0)
    assert esc.note_ack(RESULT_IN_PROGRESS) == 'accepted'
    assert esc.done


def test_final_rejection_stops_resending():
    esc = _esc()
    _drive(esc, ['SENSORS'], True, 0.0, 3.0)
    esc.note_sent(3.0)
    assert esc.note_ack(RESULT_DENIED) == 'rejected'
    assert esc.done
    assert not esc.send_due(10.0)


def test_exhaustion_stops_resending():
    esc = _esc()
    _drive(esc, ['SENSORS'], True, 0.0, 3.0)
    t = 3.0
    n = 0
    while esc.send_due(t):
        esc.note_sent(t)
        n += 1
        t += 0.5
    assert n == 4
    assert esc.done


def test_bad_config_rejected():
    with pytest.raises(ValueError):
        AbortEscalation(after_s=-1.0, repeat_period_s=0.5, max_sends=1)
    with pytest.raises(ValueError):
        AbortEscalation(after_s=1.0, repeat_period_s=0.0, max_sends=1)
    with pytest.raises(ValueError):
        AbortEscalation(after_s=1.0, repeat_period_s=0.5, max_sends=0)


def test_fence_escalates_like_a_sensor_loss():
    # M5.2 (2026-09-14): the geofence's hard boundary lands the aircraft through this path.
    esc = _esc()
    hit = _drive(esc, ['FENCE'], True, 0.0, 10.0)
    assert hit is not None
    t, reason = hit
    assert 2.0 <= t < 2.05
    assert 'FENCE' in reason
    # ...and only with authority, like everything else.
    assert _drive(_esc(), ['FENCE'], False, 0.0, 30.0) is None
