"""自己位置の跳びの指標（map→odom の段差、地図照合の採否、動きの区分ごとの集計）.

ROS に依存しない。記録した値から計算するので、シミュレーションの試験でも、実機の rosbag の再生でも
同じ指標になる。真値（シミュレーション）があれば、位置の誤差も区分ごとに出す。

区分は、その時刻の LIO の速さと角速度、近くの追跡の数で決める。1 つの値が複数の区分に入ることがある。
"""

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import math
from typing import Final

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]

# /localization/health の level
LEVEL_DEGRADED: Final = 1
LEVEL_FAILED: Final = 2
# /localization/health（ノードの時計）と /lio/odom（センサのヘッダ）の時刻がこれ以上ずれていたら、時刻の基準が違う
TIME_BASE_GAP_S: Final = 60.0


class JumpError(RuntimeError):
    pass


@dataclass(frozen=True)
class PlanarPose:
    """時刻つきの平面の姿勢。map→odom にも、odom→base_link や真値にも使う."""

    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Alignment:
    """地図照合の 1 回分（/alignment_status）."""

    t: float
    converged: bool
    fitness: float
    correction_m: float
    correction_deg: float
    consecutive_rejected: int
    # 速く動いていたので照合の結果を使わなかった（motion_gate_active）。捨てた回には数えられない
    motion_gated: bool = False


@dataclass(frozen=True)
class Motion:
    """LIO の速さと角速度（/lio/odom の twist）."""

    t: float
    speed_mps: float
    yaw_rate: float


@dataclass(frozen=True)
class Level:
    """自己位置の状態（/localization/health）."""

    t: float
    level: int
    reason: str = ''


@dataclass(frozen=True)
class Crowd:
    """機体の近くにいる追跡の数（/perception/tracks）."""

    t: float
    near: int


@dataclass(frozen=True)
class Band:
    """動きの区分。速さと角速度は絶対値の [下限, 上限)、混雑は近くの追跡の数の下限."""

    name: str
    min_speed: float = 0.0
    max_speed: float = math.inf
    min_yaw_rate: float = 0.0
    max_yaw_rate: float = math.inf
    min_near: int = 0


BANDS: Final = (
    Band('全体'),
    Band('静止', max_speed=0.1, max_yaw_rate=0.2),
    Band('並進 1〜2.5m/s', min_speed=1.0, max_speed=2.5),
    Band('並進 2.5m/s 以上', min_speed=2.5),
    Band('旋回 1.5〜3rad/s', min_yaw_rate=1.5, max_yaw_rate=3.0),
    Band('旋回 3rad/s 以上', min_yaw_rate=3.0),
    Band('混雑（近くに 3 台以上）', min_near=3),
)


def wrap(angle: float | FloatArray) -> float | FloatArray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def parse_alignment(t: float, values: Mapping[str, str]) -> Alignment | None:
    """/alignment_status の値から 1 回分を読む。照合の結果を含まない状態の通知なら None."""
    required = ('has_converged', 'fitness_score', 'correction_translation_m', 'correction_yaw_deg',
                'consecutive_rejected_updates')
    if not all(key in values for key in required):
        return None
    try:
        return Alignment(
            t=t,
            converged=values['has_converged'] == 'true',
            fitness=float(values['fitness_score']),
            correction_m=float(values['correction_translation_m']),
            correction_deg=float(values['correction_yaw_deg']),
            consecutive_rejected=int(values['consecutive_rejected_updates']),
            motion_gated=values.get('motion_gate_active', 'false') == 'true',
        )
    except ValueError as error:
        raise JumpError(f'/alignment_status の値を読めません（時刻 {t:.3f}）: {error}') from error


def compose(a: PlanarPose, b: PlanarPose, t: float) -> PlanarPose:
    """a の座標系で b を表した姿勢（a ∘ b）."""
    cos, sin = math.cos(a.yaw), math.sin(a.yaw)
    return PlanarPose(t, a.x + cos * b.x - sin * b.y, a.y + sin * b.x + cos * b.y, float(wrap(a.yaw + b.yaw)))


def steps(
    map_odom: Sequence[PlanarPose], odom_base: Sequence[PlanarPose] = (),
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """map→odom が変わるたびに、機体の地図上の位置がどれだけ跳んだか（時刻、並進 [m]、yaw [rad]）.

    map→odom の並進そのものは odom の原点を中心に回るので、原点から遠いと小さな向きの補正でも大きく
    変わる。そこで、その時刻までの最新の LIO の姿勢（odom→base_link）を新旧の map→odom で地図に写し、
    その差を跳びとする。LIO の姿勢がまだなければ、odom の原点での差（map→odom の並進の差）にする。
    同じ時刻の重複は、後に届いた値を使う。
    """
    unique = sorted({sample.t: sample for sample in map_odom}.values(), key=lambda sample: sample.t)
    if len(unique) < 2:
        return np.empty(0), np.empty(0), np.empty(0)
    base = sorted(odom_base, key=lambda pose: pose.t)
    base_times = np.array([pose.t for pose in base])
    t = np.array([sample.t for sample in unique[1:]])
    index = latest_before(t, base_times) if base else np.full(t.shape, -1)
    origin = PlanarPose(0.0, 0.0, 0.0, 0.0)
    jumps = []
    for k, (old, new) in enumerate(zip(unique, unique[1:])):
        robot = base[index[k]] if index[k] >= 0 else origin
        before, after = compose(old, robot, new.t), compose(new, robot, new.t)
        jumps.append(math.hypot(after.x - before.x, after.y - before.y))
    dyaw = np.abs(wrap(np.diff(np.unwrap([sample.yaw for sample in unique]))))
    return t, np.array(jumps), np.asarray(dyaw, dtype=np.float64)


def latest_before(times: FloatArray, sample_times: FloatArray) -> NDArray[np.intp]:
    """各時刻までに来た最新の値の位置。まだ来ていなければ -1."""
    return np.searchsorted(sample_times, times, side='right') - 1


def at_times(times: FloatArray, sample_times: FloatArray, values: FloatArray, default: float) -> FloatArray:
    index = latest_before(times, sample_times)
    return np.where(index >= 0, values[np.clip(index, 0, None)], default) if sample_times.size else (
        np.full(times.shape, default))


def band_mask(band: Band, speed: FloatArray, yaw_rate: FloatArray, near: FloatArray) -> BoolArray:
    rate = np.abs(yaw_rate)
    return (
        (speed >= band.min_speed) & (speed < band.max_speed)
        & (rate >= band.min_yaw_rate) & (rate < band.max_yaw_rate)
        & (near >= band.min_near)
    )


def map_pose_errors(
    map_odom: Sequence[PlanarPose], odom_base: Sequence[PlanarPose], truth: Sequence[PlanarPose],
) -> tuple[FloatArray, FloatArray]:
    """map→base_link（その時刻までの最新の map→odom ∘ LIO の姿勢）と真値の位置の差（時刻、[m]）."""
    if not map_odom or not odom_base or len(truth) < 2:
        return np.empty(0), np.empty(0)
    corrections = sorted(map_odom, key=lambda sample: sample.t)
    correction_times = np.array([sample.t for sample in corrections])
    truth_t = np.array([sample.t for sample in truth])
    truth_x = np.array([sample.x for sample in truth])
    truth_y = np.array([sample.y for sample in truth])
    usable = [p for p in odom_base if truth_t[0] <= p.t <= truth_t[-1] and p.t >= correction_times[0]]
    if not usable:
        return np.empty(0), np.empty(0)
    t = np.array([p.t for p in usable])
    index = latest_before(t, correction_times)
    poses = [compose(corrections[i], p, p.t) for i, p in zip(index, usable)]
    errors = np.hypot(
        np.array([p.x for p in poses]) - np.interp(t, truth_t, truth_x),
        np.array([p.y for p in poses]) - np.interp(t, truth_t, truth_y),
    )
    return t, errors


def level_durations(levels: Sequence[Level], end: float) -> tuple[FloatArray, FloatArray]:
    """各 level の値が続いた時間（各値の時刻、長さ [s]）。最後の値は end まで続くとみなす."""
    if not levels:
        return np.empty(0), np.empty(0)
    t = np.array([level.t for level in levels])
    return t, np.diff(np.append(t, max(end, t[-1])))


def time_base_gap(levels: Sequence[Level], motion: Sequence[Motion]) -> float:
    """最後の /localization/health と最後の /lio/odom の時刻の差 [s]。どちらかがなければ 0.

    /localization/health の時刻はノードの時計（再生では /clock）、/lio/odom の時刻はセンサのヘッダから来る。
    シミュレーションの rosbag を --use-sim-time なしで録ると、再生の /clock が録ったときの実時間になり、
    この差が大きくなる。そのときは区分ごとの DEGRADED と FAILED の時間が正しくない。
    """
    if not levels or not motion:
        return 0.0
    return levels[-1].t - motion[-1].t


def percentile(values: FloatArray, q: float) -> float:
    return float(np.percentile(values, q)) if values.size else math.nan


def maximum(values: FloatArray) -> float:
    return float(np.max(values)) if values.size else math.nan


@dataclass(frozen=True)
class Record:
    """1 回の走行で集めた値."""

    map_odom: Sequence[PlanarPose]
    odom_base: Sequence[PlanarPose]
    motion: Sequence[Motion]
    alignments: Sequence[Alignment]
    levels: Sequence[Level]
    crowd: Sequence[Crowd]
    truth: Sequence[PlanarPose]
    end: float


def summarize(record: Record, bands: Sequence[Band] = BANDS) -> dict[str, dict[str, float]]:
    """区分ごとの指標。時間の単位は秒、長さは cm、角度は度."""
    motion_t = np.array([m.t for m in record.motion])
    speed = np.array([m.speed_mps for m in record.motion])
    rate = np.array([m.yaw_rate for m in record.motion])
    crowd_t = np.array([c.t for c in record.crowd])
    near = np.array([float(c.near) for c in record.crowd])

    def context(times: FloatArray) -> Callable[[Band], BoolArray]:
        s = at_times(times, motion_t, speed, 0.0)
        r = at_times(times, motion_t, rate, 0.0)
        n = at_times(times, crowd_t, near, 0.0)
        return lambda band: band_mask(band, s, r, n)

    step_t, step_m, step_rad = steps(record.map_odom, record.odom_base)
    align_t = np.array([a.t for a in record.alignments])
    rejected = np.array([float(a.consecutive_rejected) for a in record.alignments])
    gated = np.array([a.motion_gated for a in record.alignments], dtype=np.bool_)
    corrections_m = np.array([a.correction_m for a in record.alignments])
    level_t, level_s = level_durations(record.levels, record.end)
    level_value = np.array([float(level.level) for level in record.levels])
    error_t, error_m = map_pose_errors(record.map_odom, record.odom_base, record.truth)
    duration_t = motion_t
    duration_s = np.diff(np.append(motion_t, max(record.end, motion_t[-1]))) if motion_t.size else np.empty(0)

    step_band, align_band, level_band, error_band, duration_band = (
        context(step_t), context(align_t), context(level_t), context(error_t), context(duration_t))
    summary: dict[str, dict[str, float]] = {}
    for band in bands:
        step_mask, align_mask, level_mask = step_band(band), align_band(band), level_band(band)
        error_mask = error_band(band)
        summary[band.name] = {
            '時間_s': float(np.sum(duration_s[duration_band(band)])) if duration_s.size else 0.0,
            '段差_最大_cm': maximum(step_m[step_mask]) * 100.0,
            '段差_99_cm': percentile(step_m[step_mask], 99) * 100.0,
            '段差_yaw_最大_deg': math.degrees(maximum(step_rad[step_mask])),
            '照合の補正_最大_cm': maximum(corrections_m[align_mask]) * 100.0,
            '捨てた照合_回': float(np.count_nonzero(rejected[align_mask] > 0)),
            '続けて捨てた_最大_回': maximum(rejected[align_mask]),
            '動きで使わなかった照合_回': float(np.count_nonzero(gated[align_mask])),
            'DEGRADED_s': float(np.sum(level_s[level_mask & (level_value == LEVEL_DEGRADED)])),
            'FAILED_s': float(np.sum(level_s[level_mask & (level_value == LEVEL_FAILED)])),
            '位置の誤差_最大_cm': maximum(error_m[error_mask]) * 100.0,
        }
    return summary


def format_table(summary: Mapping[str, Mapping[str, float]]) -> list[str]:
    keys = next(iter(summary.values())).keys() if summary else ()
    lines = ['| 区分 | ' + ' | '.join(keys) + ' |', '|---' * (len(keys) + 1) + '|']
    for name, values in summary.items():
        cells = ' | '.join('–' if math.isnan(value) else f'{value:.2f}' for value in values.values())
        lines.append(f'| {name} | {cells} |')
    return lines
