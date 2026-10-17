"""Tests for target_manager's M6.5 loss conditioning (loss_conditioning) and the M6.8 stock arm.

Same shape as test_target_manager_coast.py: a bare TargetManagerNode (no ROS graph, no
executor), driven through _remember_seen / _coast / _on_loss_event with a fake clock. What is
asserted is the CONTRACT in the module docstring's LOSS CONDITIONING block:

  * conditioning OFF (the carry-forward arm) ignores every LossEvent;
  * LOST / UNMAPPED end the coast only once the verdict has stood lost_confirm_s, and an
    OCCLUDED answer in between resets that clock;
  * OCCLUDED extends the coast to min(cap, t_max + margin), never below the plain timeout;
  * inside the belief's window the re-acquisition gate is the REGION, not the band -- a
    candidate in the band but outside the region is refused, and one inside the region but
    outside the band (same id) is accepted;
  * outside the window the ordinary band is back;
  * an event whose clock disagrees with ours is refused;
  * the stock baseline drops the lock on the first missed frame and re-acquires at once.
"""
import types

import pytest

from dtp_msgs.msg import LossEvent, Target, TargetArray
from dtp_perception import target_manager as tm
from geometry_msgs.msg import Point


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _bare(conditioning=True, stock=False, now=1000.0):
    node = object.__new__(tm.TargetManagerNode)
    quiet = types.SimpleNamespace(warn=lambda *a, **k: None, error=lambda *a, **k: None,
                                  info=lambda *a, **k: None)
    node.get_logger = lambda: quiet
    node._clock = {'t': now}
    node._now = lambda: node._clock['t']
    node._ignore_radius = 3.0
    node._acq_min_m, node._acq_max_m = 2.5, 10.0
    node._acq_motion_m, node._acq_motion_window_s = 0.0, 6.0
    node._acq_frames = 1
    node._coast_timeout = 1.5
    node._min_coast_horizon = 1.2
    node._band_m, node._band_growth, node._band_max_m = 1.0, 1.0, 2.5
    node._abstain_timeout = 0.5
    node._tiebreak_on, node._tiebreak_margin = False, 0.25
    node._auto_reacquire = False
    node._world_frame = 'lidar_init'
    node._conditioning = conditioning
    node._occluded_cap = 3.0
    node._window_margin = 0.5
    node._region_min_r = 0.5
    node._region_sigma_k = 2.0
    node._lost_confirm = 0.3
    node._loss_event_max_age = 0.5
    node._stock_baseline = stock
    node._state = tm.LockState.IDLE
    node._engaged = True
    node._locked_track_id = None
    node._locked_msg = None
    node._last_seen_arrival = None
    node._coast_frames = 0
    node._coast_ts, node._coast_pts = [], []
    node._latency_s = 0.0
    node._abstain_since = None
    node._acq_candidate_id, node._acq_streak = None, 0
    node._loss_event, node._loss_event_arrival = None, None
    node._loss_class, node._lost_verdict_since = None, None
    node._occluded_extended = False
    node._loss_events_seen = 0
    node._clock_skew_warned = False
    node._launch_xy = (0.0, 0.0)
    node._acq_motion = {}
    node._odom_xy = (0.0, 0.0)
    node._last_state_text, node._last_state_pub, node._state_detail = None, 0.0, ''
    node._follow_target_pub = _Pub()
    node._state_pub = _Pub()
    return node


def _target(track, x, y, z=0.9, vx=1.0, vy=0.0, path_len=18, confirmed=True):
    t = Target()
    t.track_id = track
    t.header.frame_id = 'lidar_init'
    t.header.stamp.sec = 100
    t.position.x, t.position.y, t.position.z = x, y, z
    t.velocity.x, t.velocity.y = vx, vy
    t.track_confirmed = confirmed
    t.distance_m = (x * x + y * y) ** 0.5
    for i in range(path_len):
        dt = 0.2 + 0.075 * i
        t.predicted_path.append(Point(x=x + vx * dt, y=y + vy * dt, z=z))
        t.predicted_path_dt.append(dt)
    return t


def _array(*targets):
    arr = TargetArray()
    arr.header.frame_id = 'lidar_init'
    arr.header.stamp.sec = 100
    arr.targets.extend(targets)
    return arr


def _lock(node, track=7, x=4.0, y=0.0):
    tgt = _target(track, x, y)
    node._locked_track_id = track
    node._remember_seen(tgt, _array(tgt), node._now())
    node._state = tm.LockState.LOCKED
    return tgt


def _event(node, cls, track=7, reappear=None, dt=(0.8, 1.2, 0.5), sigma=0.1, eval_offset=0.0):
    """A LossEvent evaluated at now + eval_offset. reappear=(x, y) adds one dominant cluster."""
    ev = LossEvent()
    ev.track_id = track
    ev.cls = cls
    t = node._now() + eval_offset
    ev.eval_stamp.sec = int(t)
    ev.eval_stamp.nanosec = int((t - int(t)) * 1e9)
    ev.blocked_fraction = 1.0 if cls == LossEvent.CLS_OCCLUDED else 0.0
    if reappear is not None:
        ev.reappear_p.append(Point(x=reappear[0], y=reappear[1], z=0.9))
        ev.reappear_dt_min.append(dt[0])
        ev.reappear_dt_max.append(dt[1])
        ev.reappear_dt.append(dt[2])
        ev.reappear_sigma_m.append(sigma)
        ev.reappear_w.append(0.9)
        ev.reappear_side.append(-1)
    return ev


def _coast_at(node, age, *others):
    node._clock['t'] = 1000.0 + age
    node._coast(_array(*others), node._now())


# --- the carry-forward arm ------------------------------------------------------------------

def test_off_arm_ignores_lost_verdicts():
    node = _bare(conditioning=False)
    _lock(node)
    node._on_loss_event(_event(node, LossEvent.CLS_LOST))
    for age in (0.1, 0.4, 0.8, 1.2):
        node._on_loss_event(_event(node, LossEvent.CLS_LOST))
        _coast_at(node, age)
        assert node._state is tm.LockState.COASTING, \
            'the OFF arm must coast through a LOST verdict'
    _coast_at(node, 1.6)
    assert node._state is tm.LockState.LOST
    assert 'coast timeout' in node._state_detail


# --- LOST / UNMAPPED ------------------------------------------------------------------------

def test_lost_verdict_ends_coast_after_confirm_window():
    node = _bare()
    _lock(node)
    node._clock['t'] = 1000.1
    node._on_loss_event(_event(node, LossEvent.CLS_LOST))
    _coast_at(node, 0.1)
    assert node._state is tm.LockState.COASTING
    node._clock['t'] = 1000.3
    node._on_loss_event(_event(node, LossEvent.CLS_LOST))
    _coast_at(node, 0.3)
    assert node._state is tm.LockState.COASTING, '0.2 s of LOST is under lost_confirm_s'
    node._clock['t'] = 1000.45
    node._on_loss_event(_event(node, LossEvent.CLS_UNMAPPED))   # UNMAPPED counts the same
    _coast_at(node, 0.45)
    assert node._state is tm.LockState.LOST
    assert 'loss oracle' in node._state_detail and 'UNMAPPED' in node._state_detail


def test_occluded_answer_resets_lost_persistence():
    node = _bare()
    _lock(node)
    node._clock['t'] = 1000.1
    node._on_loss_event(_event(node, LossEvent.CLS_LOST))
    node._clock['t'] = 1000.3
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED, reappear=(5.0, -1.0)))
    assert node._lost_verdict_since is None
    node._clock['t'] = 1000.4
    node._on_loss_event(_event(node, LossEvent.CLS_LOST))
    _coast_at(node, 0.5)
    assert node._state is tm.LockState.COASTING, 'the LOST clock restarted at 0.4 s'


def test_stale_event_is_not_used():
    node = _bare()
    _lock(node)
    node._clock['t'] = 1000.1
    node._on_loss_event(_event(node, LossEvent.CLS_LOST))
    # No further events: at 1.0 s the one we have is 0.9 s old, past loss_event_max_age_s.
    _coast_at(node, 1.0)
    assert node._state is tm.LockState.COASTING


def test_clock_skew_refuses_event():
    node = _bare()
    _lock(node)
    node._on_loss_event(_event(node, LossEvent.CLS_LOST, eval_offset=10.0))
    assert node._loss_event is None and node._clock_skew_warned


def test_event_for_another_track_is_ignored():
    node = _bare()
    _lock(node, track=7)
    node._on_loss_event(_event(node, LossEvent.CLS_LOST, track=8))
    assert node._loss_event is None


# --- OCCLUDED: coast extension --------------------------------------------------------------

def test_occluded_extends_coast_to_belief_plus_margin():
    node = _bare()
    _lock(node)
    node._clock['t'] = 1000.2
    # Belief: re-emergence 1.0-1.6 s after an evaluation at coast age 0.2 -> window 0.7-2.3 s.
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.0), dt=(1.0, 1.6, 1.3)))
    _coast_at(node, 0.2)
    assert node._state is tm.LockState.COASTING
    assert node._coast_ts[-1] >= 2.3, 'the polyline must reach the extended coast limit'
    # Well past the carry-forward coast (1.5 s), still coasting and still publishing.
    n_before = len(node._follow_target_pub.msgs)
    node._clock['t'] = 1002.0
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.0), dt=(1.0, 1.6, 1.3),
                               eval_offset=0.0))
    _coast_at(node, 2.0)
    assert node._state is tm.LockState.COASTING
    assert len(node._follow_target_pub.msgs) == n_before + 1
    assert node._follow_target_pub.msgs[-1].missed_frames >= 2
    # The limit follows the NEWEST event: an event evaluated at 2.4 s that still expects a
    # re-emergence 1.0-1.6 s later pushes the window to 4.5 s, so the 3.0 s cap is what ends it.
    node._clock['t'] = 1002.4
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.0), dt=(1.0, 1.6, 1.3)))
    _coast_at(node, 2.4)
    assert node._state is tm.LockState.COASTING
    node._clock['t'] = 1003.1
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.0), dt=(1.0, 1.6, 1.3)))
    _coast_at(node, 3.1)
    assert node._state is tm.LockState.LOST
    assert 'coast timeout (3.00 s)' in node._state_detail


def test_occluded_extension_needs_a_fresh_event():
    """An oracle that stops answering must not keep the coast extended: the limit is re-derived
    every frame from a FRESH event (loss_event_max_age_s), else it is the plain timeout."""
    node = _bare()
    _lock(node)
    node._clock['t'] = 1000.2
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.0), dt=(1.0, 1.6, 1.3)))
    _coast_at(node, 0.2)                     # window [0.7, 2.3] s -> limit 2.3 s for this frame
    assert node._coast_ts[-1] >= 2.3
    _coast_at(node, 1.6)                     # the only event is now 1.4 s old
    assert node._state is tm.LockState.LOST
    assert 'coast timeout (1.50 s)' in node._state_detail


def test_occluded_never_shortens_below_plain_timeout():
    node = _bare()
    _lock(node)
    node._clock['t'] = 1000.1
    # An optimistic belief: out again within 0.2-0.4 s. Coast must still last the plain 1.5 s.
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(4.5, -0.5), dt=(0.2, 0.4, 0.3)))
    _coast_at(node, 0.1)
    for age in (0.5, 1.0, 1.4):
        node._clock['t'] = 1000.0 + age
        node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                                   reappear=(4.5, -0.5), dt=(0.2, 0.4, 0.3)))
        _coast_at(node, age)
        assert node._state is tm.LockState.COASTING


def test_occluded_without_belief_coasts_to_cap():
    node = _bare()
    _lock(node)
    node._clock['t'] = 1000.1
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED))
    _coast_at(node, 0.1)
    node._clock['t'] = 1002.5
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED))
    _coast_at(node, 2.5)
    assert node._state is tm.LockState.COASTING
    node._clock['t'] = 1003.1
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED))
    _coast_at(node, 3.1)
    assert node._state is tm.LockState.LOST


# --- OCCLUDED: the region gate ---------------------------------------------------------------

def _occluded_with_window(node, reappear=(6.0, -1.5), dt=(0.7, 1.1, 0.9), sigma=0.1):
    """Belief evaluated at coast age 0.2: window [0.4, 1.8] s, region r 0.5 about `reappear`."""
    node._clock['t'] = 1000.2
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=reappear, dt=dt, sigma=sigma))
    _coast_at(node, 0.2)


def test_region_refuses_candidate_in_band_but_outside_region():
    node = _bare()
    _lock(node)                       # walking +x from (4, 0): CTRV ref at age 1.0 is ~(5, 0)
    _occluded_with_window(node)
    node._clock['t'] = 1001.0
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.5), dt=(-0.1, 0.3, 0.1)))
    stranger = _target(9, 5.0, 1.0)   # 1.0 m from the CTRV point: inside the 2.0 m band
    _coast_at(node, 1.0, stranger)
    assert node._state is tm.LockState.COASTING, 'inside the window the band does not apply'


def test_region_accepts_sole_confirmed_candidate_inside_region():
    node = _bare()
    _lock(node)
    _occluded_with_window(node)
    node._clock['t'] = 1001.0
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.5), dt=(-0.1, 0.3, 0.1)))
    back = _target(9, 6.2, -1.3)      # 0.28 m from the region centre
    _coast_at(node, 1.0, back)
    assert node._state is tm.LockState.LOCKED
    assert node._locked_track_id == 9


def test_region_still_abstains_on_two_candidates():
    node = _bare()
    _lock(node)
    _occluded_with_window(node)
    node._clock['t'] = 1001.0
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.5), dt=(-0.1, 0.3, 0.1)))
    _coast_at(node, 1.0, _target(9, 6.2, -1.3), _target(10, 5.8, -1.7))
    assert node._state is tm.LockState.ABSTAIN


def test_band_returns_outside_window():
    node = _bare()
    _lock(node)
    _occluded_with_window(node, dt=(1.5, 1.9, 1.7))    # window [1.2, 2.6] s
    node._clock['t'] = 1000.8
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(6.0, -1.5), dt=(0.9, 1.3, 1.1)))
    stranger = _target(9, 4.9, 0.8)   # in the band about the CTRV point (~(4.8, 0) at 0.8 s)
    _coast_at(node, 0.8, stranger)
    assert node._state is tm.LockState.LOCKED, 'before the window opens the ordinary band applies'


def test_same_id_back_inside_region_outside_band():
    node = _bare()
    _lock(node, track=7)
    _occluded_with_window(node, reappear=(8.0, -2.5))   # far from any CTRV point
    node._clock['t'] = 1001.0
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED,
                               reappear=(8.0, -2.5), dt=(-0.1, 0.3, 0.1)))
    same = _target(7, 8.1, -2.4)   # 3.3 m from the CTRV point: out of band, in the region
    node._service_lock(_array(same), node._now())
    assert node._state is tm.LockState.LOCKED
    assert 'region' in node._state_detail


def test_seen_frame_clears_the_verdict():
    node = _bare()
    tgt = _lock(node)
    node._clock['t'] = 1000.1
    node._on_loss_event(_event(node, LossEvent.CLS_OCCLUDED, reappear=(6.0, -1.5)))
    node._remember_seen(tgt, _array(tgt), 1000.2)
    assert node._loss_event is None and node._loss_class is None and not node._occluded_extended


# --- M6.8 stock baseline ---------------------------------------------------------------------

def test_stock_baseline_drops_lock_on_first_miss_and_reacquires():
    node = _bare(conditioning=False, stock=True)
    _lock(node, track=7)
    other = _target(9, 5.0, 2.0)      # the nearest confirmed body, outside the launch disc
    _coast_at(node, 0.1, other)
    assert node._state is tm.LockState.LOCKED
    assert node._locked_track_id == 9, 'stock: no coast, no abstain, nearest confirmed person'


def test_stock_baseline_with_nobody_stays_acquiring():
    node = _bare(conditioning=False, stock=True)
    _lock(node, track=7)
    _coast_at(node, 0.1)
    assert node._state is tm.LockState.ACQUIRING
    assert node._locked_track_id is None


# --- parameter validation ---------------------------------------------------------------------

def test_cap_below_coast_timeout_is_rejected():
    node = _bare()
    node._occluded_cap = 1.0
    with pytest.raises(ValueError):
        node._validate_parameters()


def test_extend_coast_to_is_idempotent_and_constant_velocity():
    node = _bare()
    _lock(node)                        # polyline ends at ~1.475 s
    end = node._coast_ts[-1]
    node._extend_coast_to(3.0)
    assert node._coast_ts[-1] == pytest.approx(3.0)
    x_end, _, _ = node._coast_pts[-1]
    assert x_end == pytest.approx(node._coast_pts[-2][0] + 1.0 * (3.0 - end), abs=0.05)
    n = len(node._coast_ts)
    node._extend_coast_to(3.0)
    assert len(node._coast_ts) == n
