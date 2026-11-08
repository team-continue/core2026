"""Check full-turn trial bounds without connecting to ROS or hardware."""
import math
from types import SimpleNamespace

import pytest

from core_tools.base_sysid_probe import FULL_TURN_EXCURSION, Probe, make_plan


def test_full_turn_trial_has_three_second_pulses_and_stationary_intervals():
    plan = make_plan('waist', 'turns', 6.28)
    assert [(v, t) for v, t in plan if v] == [(6.28, 3), (-6.28, 3)]
    assert plan[0] == (0, 1)
    assert plan[2] == (0, 3)
    assert plan[-1] == (0, 3)
    displacement = 0.0
    for speed, duration in plan:
        displacement += speed * duration
        assert abs(displacement) < FULL_TURN_EXCURSION
    assert displacement == pytest.approx(0.0)


@pytest.mark.parametrize('axis', ['vx', 'vy', 'wz'])
@pytest.mark.parametrize('profile', ['turns', 'turns-valid'])
def test_full_turn_profiles_cannot_raise_drive_limits(axis, profile):
    with pytest.raises(ValueError, match='only available for waist'):
        make_plan(axis, profile, 6.28)


@pytest.mark.parametrize('amplitude', [0, -0.1, 6.29, float('inf'), float('nan')])
def test_invalid_full_turn_speed_is_rejected(amplitude):
    with pytest.raises(ValueError, match='amplitude'):
        make_plan('waist', 'turns', amplitude)


@pytest.mark.parametrize('profile', ['step', 'multistep'])
def test_original_profiles_keep_small_motion_limit(profile):
    with pytest.raises(ValueError, match='amplitude'):
        make_plan('waist', profile, 6.28)
    assert make_plan('waist', profile, 0.35)


def test_four_turn_encoder_bound_still_aborts():
    state = SimpleNamespace(started=True, is_waist=True, waist_origin=3.0,
                            waist_excursion_limit=FULL_TURN_EXCURSION,
                            tripped=False, samples={})
    Probe.on_joints(state, SimpleNamespace(position=[0, 0, 0, 0, 3.0 + 6 * math.pi]))
    assert not state.tripped
    Probe.on_joints(state, SimpleNamespace(position=[0, 0, 0, 0, 3.0 + 8 * math.pi + 0.01]))
    assert state.tripped


def test_validation_uses_independent_excitation_within_travel_bound():
    plan = make_plan('waist', 'turns-valid', 6.28)
    assert plan != make_plan('waist', 'turns', 6.28)
    displacement = 0.0
    for speed, duration in plan:
        displacement += speed * duration
        assert abs(displacement) < FULL_TURN_EXCURSION
        if speed:
            assert duration == 3
