import math

from core_msgs.msg import LocalizationHealth
from core_nav_launch.localization_status import (
    HealthSample,
    LEVEL_DEGRADED,
    LEVEL_FAILED,
    LEVEL_LABELS,
    LEVEL_OK,
    status_label,
    UNKNOWN_RGBA,
)


def sample(
    level: int, fitness: float = 0.042, inlier_ratio: float = 0.93, gyro: float = 0.01, since: float = 0.12,
) -> HealthSample:
    return HealthSample(level, fitness, inlier_ratio, gyro, since)


def test_levels_match_message() -> None:
    assert (LEVEL_OK, LEVEL_DEGRADED, LEVEL_FAILED) == (
        LocalizationHealth.LEVEL_OK, LocalizationHealth.LEVEL_DEGRADED, LocalizationHealth.LEVEL_FAILED)


def test_each_level_has_its_own_color_and_shows_values() -> None:
    label = status_label(sample(LEVEL_OK), age_s=0.1, stale_after_s=1.0)
    assert label.text == 'LOC OK\nfitness 0.042  inlier 93%\nlast match 0.1 s  gyro diff 0.01 rad/s'
    assert label.rgba == LEVEL_LABELS[LEVEL_OK][1]
    colors = {status_label(sample(level), 0.1, 1.0).rgba for level in (LEVEL_OK, LEVEL_DEGRADED, LEVEL_FAILED)}
    assert len(colors | {UNKNOWN_RGBA}) == 4
    assert status_label(sample(LEVEL_FAILED), 0.1, 1.0).text.startswith('LOC FAILED\n')


def test_missing_values_are_dashes() -> None:
    # 地図照合の評価値は未取得のとき負、インライア率は出力が無いと NaN
    missing = sample(LEVEL_DEGRADED, fitness=-1.0, inlier_ratio=math.nan, gyro=math.nan, since=math.inf)
    assert status_label(missing, 0.1, 1.0).text == (
        'LOC DEGRADED\nfitness -  inlier -\nlast match -  gyro diff -')


def test_no_or_stale_health_is_gray() -> None:
    never = status_label(None, math.inf, 1.0)
    assert never.rgba == UNKNOWN_RGBA
    assert never.text == 'LOC ?\nno /localization/health'
    stale = status_label(sample(LEVEL_OK), age_s=2.5, stale_after_s=1.0)
    assert stale.rgba == UNKNOWN_RGBA
    assert stale.text == 'LOC ?\nhealth stale 2.5 s'
    # ちょうど stale_after_s はまだ新しい。時間が NaN なら古いとみなす
    assert status_label(sample(LEVEL_OK), 1.0, 1.0).rgba == LEVEL_LABELS[LEVEL_OK][1]
    assert status_label(sample(LEVEL_OK), math.nan, 1.0).rgba == UNKNOWN_RGBA


def test_unknown_level_is_gray() -> None:
    label = status_label(sample(7), 0.1, 1.0)
    assert label.rgba == UNKNOWN_RGBA
    assert label.text.startswith('LOC level 7\n')
