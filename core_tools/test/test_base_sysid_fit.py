import numpy as np
import pytest

from core_tools.base_sysid_fit import body_speed, fit, prepare_run, response


def test_gain_lag_and_delay_recovery():
    grid = np.arange(0, 6, 0.01)
    speed = np.zeros_like(grid)
    speed[(grid >= 1) & (grid < 2)] = 0.16
    speed[(grid >= 3) & (grid < 4)] = -0.16
    run = (grid, speed, grid, np.zeros_like(grid))
    y = -1.2 * response(run, 0.18, 5)
    gain, tau, delay = fit((grid, speed, grid, y))
    assert gain == pytest.approx(-1.2, abs=0.005)
    assert tau == pytest.approx(0.18, abs=0.005)
    assert delay == 5


@pytest.mark.parametrize('failure', ['hazard', 'stale', 'gap'])
def test_reject_incomplete_experiment(failure):
    t = np.arange(0, 7, 0.02)
    u = np.where((t > 1) & (t < 2), 0.2, np.where((t > 3) & (t < 4), -0.2, 0))
    hazards = [(x, failure == 'hazard' and 2 < x < 2.2) for x in t
               if not (failure == 'stale' and 2 < x < 3)]
    ty = t[(t < 2) | (t > 3)] if failure == 'gap' else t
    with pytest.raises(ValueError, match='hazard|gap'):
        prepare_run(t, u, ty, np.zeros_like(ty), hazards)


@pytest.mark.parametrize('axis,signs', [('vx', [-1, -1, 1, 1]), ('vy', [1, -1, -1, 1]),
                                        ('wz', [1, 1, 1, 1])])
def test_wheel_directions_are_decoupled(axis, signs):
    speed = 0.5304 / np.sqrt(2) if axis == 'wz' else 1 / np.sqrt(2)
    refs = {i: s * speed / 0.065 for i, s in enumerate(signs)}
    for component in ('vx', 'vy', 'wz'):
        assert body_speed(refs, component) == pytest.approx(float(axis == component))
