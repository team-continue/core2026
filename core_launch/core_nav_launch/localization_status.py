"""自己位置の状態（core_msgs/LocalizationHealth）を、RViz に出す文字と色にする（ROS に依存しない部分）.

localization_status_marker_node.py が使う。RViz の文字（Ogre のフォント）は日本語を描けないので英字で出し、
理由（reason。日本語）は出さない。理由は `ros2 topic echo /localization/health` で見る。
"""

from dataclasses import dataclass
import math
from typing import Final

# core_msgs/LocalizationHealth の level
LEVEL_OK: Final = 0
LEVEL_DEGRADED: Final = 1
LEVEL_FAILED: Final = 2

Rgba = tuple[float, float, float, float]

LEVEL_LABELS: Final[dict[int, tuple[str, Rgba]]] = {
    LEVEL_OK: ('OK', (0.30, 0.90, 0.45, 1.0)),
    LEVEL_DEGRADED: ('DEGRADED', (1.00, 0.78, 0.20, 1.0)),
    LEVEL_FAILED: ('FAILED', (1.00, 0.30, 0.30, 1.0)),
}
# 状態が届かない・古い・知らない level のときの色
UNKNOWN_RGBA: Final[Rgba] = (0.62, 0.62, 0.66, 1.0)


@dataclass(frozen=True)
class HealthSample:
    """受けた LocalizationHealth の値."""

    level: int
    fitness: float
    inlier_ratio: float
    gyro_difference: float
    time_since_match_s: float


@dataclass(frozen=True)
class StatusLabel:
    """RViz に出す文字と色."""

    text: str
    rgba: Rgba


def _quantity(value: float, digits: int, unit: str = '') -> str:
    """値が無い（NaN・無限・負）なら '-'. 地図照合の評価値は、未取得のとき負になる."""
    if not (math.isfinite(value) and value >= 0.0):
        return '-'
    return f'{value:.{digits}f}{" " + unit if unit else ""}'


def _percent(ratio: float) -> str:
    return f'{ratio * 100.0:.0f}%' if math.isfinite(ratio) and ratio >= 0.0 else '-'


def status_label(sample: HealthSample | None, age_s: float, stale_after_s: float) -> StatusLabel:
    """自己位置の状態の文字と色.

    sample が無い（まだ届いていない）か、age_s（受けてからの時間）が stale_after_s を超えたら、
    状態を判定するノード（localization_health）が止まっているので、灰色で届いていないことを出す。
    """
    if sample is None:
        return StatusLabel('LOC ?\nno /localization/health', UNKNOWN_RGBA)
    if not age_s <= stale_after_s:
        return StatusLabel(f'LOC ?\nhealth stale {_quantity(age_s, 1, "s")}', UNKNOWN_RGBA)
    name, rgba = LEVEL_LABELS.get(sample.level, (f'level {sample.level}', UNKNOWN_RGBA))
    return StatusLabel(
        f'LOC {name}\n'
        f'fitness {_quantity(sample.fitness, 3)}  inlier {_percent(sample.inlier_ratio)}\n'
        f'last match {_quantity(sample.time_since_match_s, 1, "s")}  '
        f'gyro diff {_quantity(sample.gyro_difference, 2, "rad/s")}',
        rgba,
    )
