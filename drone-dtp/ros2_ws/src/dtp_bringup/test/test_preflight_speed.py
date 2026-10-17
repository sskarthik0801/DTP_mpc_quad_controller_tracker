"""Tests for the max_vel gate in dtp_bringup.preflight.

*** WHY THIS FILE EXISTS ***

On 2026-09-12 an audit found that our own preflight told the operator to fix a missing follow path
by switching to `config_name:=follow_test_ros2.yaml`. That file carries SUPER's benchmark tune
(max_vel 5.0) while the config the aircraft actually flies carries the gate-signed first-flight
limit (max_vel 0.1). Our advice was a 50x speed increase with nothing downstream clamping it, and
it was unnecessary -- the safe config had gained the follow path two days earlier.

The bug was not in the logic. The bug was that the envelope lived in a COMMENT. These tests exist
so it stays a gate: the case that matters is test_refuses_the_repo_benchmark_tune, which runs
against the real YAML in the repo rather than a fixture, so the gate cannot drift away from the
file it is protecting against.
"""
import os

import pytest
import yaml

from dtp_bringup import preflight

# The real files, because a gate tested only against fixtures proves nothing about the config that
# would actually have flown. Resolved relative to this file so it works from any cwd.
_CONFIG_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', 'super_planner', 'config'))


def _load(name):
    path = os.path.join(_CONFIG_DIR, name)
    if not os.path.exists(path):
        pytest.skip('%s not present in this checkout' % name)
    with open(path) as handle:
        return yaml.safe_load(handle), path


# Kevin's deployed first-flight limits, sed-applied by super_bringup/scripts/setup.sh on every
# install and tabulated in his docs/flight-test-plan.md. The gate must never refuse these.
GATE_SIGNED = {
    'fsm': {'follow_goal_en': True},
    'super_planner': {'yaw_dot_max': 0.5},
    'traj_opt': {'boundary': {'max_vel': 0.1, 'max_acc': 0.3, 'max_omg': 1.5}},
}


def test_refuses_the_repo_benchmark_tune():
    """The whole point. follow_test_ros2.yaml must not pass the default ceiling."""
    cfg, path = _load('follow_test_ros2.yaml')
    with pytest.raises(preflight.PreflightError) as excinfo:
        preflight.assert_speed_envelope(cfg, path)
    message = str(excinfo.value)
    # The refusal has to be actionable on a flight line, so it must name the measured value, the
    # ceiling, and the fact that nothing downstream saves you.
    assert '5' in message
    assert '1.5' in message
    assert 'clamps' in message


def test_accepts_the_gate_signed_envelope():
    """A gate that also refuses the config we DO want to fly would just get bypassed."""
    preflight.assert_speed_envelope(GATE_SIGNED, 'deployed click_smooth_ros2.yaml')


def test_ceiling_is_inclusive():
    cfg = {'traj_opt': {'boundary': {'max_vel': 1.5}}}
    preflight.assert_speed_envelope(cfg, '<test>', 1.5)
    with pytest.raises(preflight.PreflightError):
        preflight.assert_speed_envelope({'traj_opt': {'boundary': {'max_vel': 1.5001}}},
                                        '<test>', 1.5)


def test_raising_the_ceiling_is_possible_but_explicit():
    """The override exists so the gate does not get disabled wholesale instead."""
    cfg, path = _load('follow_test_ros2.yaml')
    preflight.assert_speed_envelope(cfg, path, 5.0)


def test_absent_max_vel_is_refused_not_assumed_safe():
    """fsm/config.hpp would supply a compiled default we cannot read, so unknown != safe."""
    with pytest.raises(preflight.PreflightError) as excinfo:
        preflight.assert_speed_envelope({'traj_opt': {'boundary': {'max_acc': 1.0}}}, '<test>')
    assert 'absent' in str(excinfo.value)


def test_unreadable_max_vel_says_so_rather_than_absent():
    """Telling someone to add a key that is already there sends them after the wrong thing."""
    with pytest.raises(preflight.PreflightError) as excinfo:
        preflight.assert_speed_envelope({'traj_opt': {'boundary': {'max_vel': 'fast'}}}, '<test>')
    message = str(excinfo.value)
    assert 'unreadable' in message
    assert 'fast' in message


def test_empty_config_is_refused():
    with pytest.raises(preflight.PreflightError):
        preflight.assert_speed_envelope({}, '<test>')


def test_envelope_reports_all_four_limits():
    """The refusal prints acc/omg/yaw too, because max_vel is never the only thing that moved."""
    shown = preflight.format_speed_envelope(GATE_SIGNED)
    for key in ('max_vel', 'max_acc', 'max_omg', 'yaw_dot_max'):
        assert key in shown
    assert 'max_vel=0.1' in shown


def test_follow_path_error_no_longer_recommends_the_fast_config():
    """The original defect, pinned. This message must not send anyone to follow_test_ros2.yaml."""
    with pytest.raises(preflight.PreflightError) as excinfo:
        preflight.assert_follow_path_enabled({'fsm': {'follow_goal_en': False}}, '<test>')
    message = str(excinfo.value)
    assert 'Do NOT switch to follow_test_ros2.yaml' in message
    assert 'follow_goal_en: true' in message
