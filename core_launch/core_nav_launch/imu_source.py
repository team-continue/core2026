"""LIO（FAST-LIO）に入れる IMU の選択を、各ノードのパラメータに直す.

config/imu_source.yaml の IMU ごとの値（トピック、座標系、外部パラメータ、単位、雑音、飽和）を読み、
launch 引数 imu_source で選んだものに合わせて次を決める。
- FAST-LIO: common.imu_topic、mapping.extrinsic_T・extrinsic_R、common.accel_clip（IMU の単位）
- lio_odom_adapter: imu_frame（FAST-LIO の body の座標系）と、TF と比べる外部パラメータ
- scan_filter: output_frame（/cloud_registered_body の座標系）
- fused のときは imu_fusion ノード

外部パラメータは IMU ごとに extrinsic_T・extrinsic_R で書くか、extrinsic: urdf でロボットの記述（URDF）から求める。
LiDAR と一体でない IMU（Damiao）は LiDAR の取り付け（xacro の lidar_x・lidar_height）で値が変わるので、urdf にする。
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Final, Mapping

from launch_ros.actions import Node
import yaml

from .self_filter import compose, fixed_transform, invert, LIDAR_FRAME, SelfFilterError

IMU_SOURCES: Final = ('livox', 'damiao', 'fused')
DEFAULT_IMU_SOURCE: Final = 'livox'
FUSED: Final = 'fused'
# 実在する IMU。fused はこの 2 つから作る
SENSORS: Final = ('livox', 'damiao')
# 回転行列とみなす R Rᵀ と単位行列の差、行列式と 1 の差の上限
ROTATION_TOLERANCE: Final = 1e-3
# extrinsic にこの値を書くと、外部パラメータをロボットの記述から求める
EXTRINSIC_FROM_URDF: Final = 'urdf'
# 外部パラメータを求めるときに、IMU と LiDAR をつなぐ根のフレーム
ROBOT_BASE_FRAME: Final = 'base_link'


class ImuSourceError(RuntimeError):
    """imu_source の指定か、IMU の設定ファイルが正しくないときの例外."""


@dataclass(frozen=True)
class ImuSensor:
    name: str
    topic: str
    # 取り付け位置の TF のフレーム
    frame: str
    # IMU の座標系で表した LiDAR の位置と、LiDAR→IMU の回転（行優先の 3x3）
    extrinsic_t: tuple[float, float, float]
    extrinsic_r: tuple[float, ...]
    # 加速度を m/s2 に直す倍率
    accel_scale: float
    gyro_noise_std_radps: float
    accel_noise_std_mps2: float
    gyro_saturation_radps: float
    accel_saturation_mps2: float


@dataclass(frozen=True)
class ImuSelection:
    """選んだ IMU と、FAST-LIO に入るトピックの性質."""

    name: str
    # FAST-LIO が読むトピック
    topic: str
    # FAST-LIO の body になる IMU。fused では主
    body: ImuSensor
    # FAST-LIO に入る加速度を m/s2 に直す倍率。fused の出力は m/s2 なので 1
    accel_scale: float
    # fused のときの (主, 補助)
    fusion: tuple[ImuSensor, ImuSensor] | None


def _require(mapping: Mapping[str, object], key: str, where: str) -> object:
    if key not in mapping:
        raise ImuSourceError(f'{where} に {key} がありません')
    return mapping[key]


def _mapping(value: object, where: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise ImuSourceError(f'{where} は項目の辞書にしてください: {value!r}')
    return value


def _text(value: object, where: str) -> str:
    if not isinstance(value, str) or not value:
        raise ImuSourceError(f'{where} は空でない文字列にしてください: {value!r}')
    return value


def _number(value: object, where: str) -> float:
    # YAML の true/false は int の仲間なので、数値として受けない
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ImuSourceError(f'{where} は数値にしてください: {value!r}')
    number = float(value)
    if not math.isfinite(number):
        raise ImuSourceError(f'{where} が有限の値ではありません: {value!r}')
    return number


def _positive(value: object, where: str) -> float:
    number = _number(value, where)
    if number <= 0.0:
        raise ImuSourceError(f'{where} は正の値にしてください: {number}')
    return number


def _numbers(value: object, count: int, where: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise ImuSourceError(f'{where} は {count} 個の数値の配列にしてください: {value!r}')
    return tuple(_number(item, f'{where}[{index}]') for index, item in enumerate(value))


def _check_rotation(values: tuple[float, ...], where: str) -> None:
    rows = (values[0:3], values[3:6], values[6:9])
    orthogonality_error = max(
        abs(sum(rows[i][k] * rows[j][k] for k in range(3)) - (1.0 if i == j else 0.0))
        for i in range(3)
        for j in range(3)
    )
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if orthogonality_error > ROTATION_TOLERANCE or abs(determinant - 1.0) > ROTATION_TOLERANCE:
        raise ImuSourceError(
            f'{where} が回転行列ではありません（R Rᵀ と単位行列の差 {orthogonality_error:.3g}、'
            f'行列式 {determinant:.4f}）。行優先の 3x3 で書いてください'
        )


Extrinsic = tuple[tuple[float, float, float], tuple[float, ...]]


def extrinsic_from_urdf(robot_description: str, imu_frame: str) -> Extrinsic:
    """ロボットの記述から、FAST-LIO の外部パラメータ（IMU の座標系での LiDAR の位置と、LiDAR→IMU の回転）を求める.

    base_link から IMU と LiDAR（livox_frame）までの固定の関節を合成し、IMU から見た LiDAR の変換にする。
    """
    try:
        base_from_imu = fixed_transform(robot_description, ROBOT_BASE_FRAME, imu_frame)
        base_from_lidar = fixed_transform(robot_description, ROBOT_BASE_FRAME, LIDAR_FRAME)
    except SelfFilterError as error:
        raise ImuSourceError(
            f'IMU（{imu_frame}）の外部パラメータをロボットの記述から求められません: {error}'
        ) from error
    imu_from_lidar = compose(invert(base_from_imu), base_from_lidar)
    return imu_from_lidar.translation, imu_from_lidar.rotation


def _parse_extrinsic(
    entry: Mapping[str, object], frame: str, where: str, robot_description: str | None,
) -> Extrinsic:
    """extrinsic_T・extrinsic_R か、extrinsic: urdf（ロボットの記述から求める）を読む."""
    if 'extrinsic' not in entry:
        extrinsic_t = _numbers(_require(entry, 'extrinsic_T', where), 3, f'{where}.extrinsic_T')
        extrinsic_r = _numbers(_require(entry, 'extrinsic_R', where), 9, f'{where}.extrinsic_R')
        _check_rotation(extrinsic_r, f'{where}.extrinsic_R')
        return (extrinsic_t[0], extrinsic_t[1], extrinsic_t[2]), extrinsic_r
    if entry['extrinsic'] != EXTRINSIC_FROM_URDF:
        raise ImuSourceError(
            f'{where}.extrinsic は {EXTRINSIC_FROM_URDF} にするか、書かずに extrinsic_T・extrinsic_R を'
            f'書いてください: {entry["extrinsic"]!r}'
        )
    if 'extrinsic_T' in entry or 'extrinsic_R' in entry:
        raise ImuSourceError(
            f'{where} に extrinsic: {EXTRINSIC_FROM_URDF} と extrinsic_T・extrinsic_R の両方があります。'
            'どちらかにしてください'
        )
    if robot_description is None:
        raise ImuSourceError(
            f'{where}.extrinsic は {EXTRINSIC_FROM_URDF} なので、ロボットの記述（URDF）を渡してください'
        )
    return extrinsic_from_urdf(robot_description, frame)


def _parse_sensor(
    name: str, value: object, path: Path, robot_description: str | None,
) -> ImuSensor:
    where = f'{path} の {name}'
    entry = _mapping(value, where)
    topic = _text(_require(entry, 'topic', where), f'{where}.topic')
    if not topic.startswith('/'):
        raise ImuSourceError(f'{where}.topic は / で始まる名前にしてください: {topic}')
    frame = _text(_require(entry, 'frame', where), f'{where}.frame')
    extrinsic_t, extrinsic_r = _parse_extrinsic(entry, frame, where, robot_description)

    def positive(key: str) -> float:
        return _positive(_require(entry, key, where), f'{where}.{key}')

    return ImuSensor(
        name=name,
        topic=topic,
        frame=frame,
        extrinsic_t=extrinsic_t,
        extrinsic_r=extrinsic_r,
        accel_scale=positive('accel_scale'),
        gyro_noise_std_radps=positive('gyro_noise_std_radps'),
        accel_noise_std_mps2=positive('accel_noise_std_mps2'),
        gyro_saturation_radps=positive('gyro_saturation_radps'),
        accel_saturation_mps2=positive('accel_saturation_mps2'),
    )


def load_imu_selection(
    config_path: Path, name: str, robot_description: str | None = None,
) -> ImuSelection:
    """設定ファイルを読み、imu_source の name に合う選択を返す.

    robot_description はロボットの記述（URDF の文字列）。extrinsic: urdf の IMU があるときに必要。
    """
    if name not in IMU_SOURCES:
        raise ImuSourceError(f'imu_source は {" / ".join(IMU_SOURCES)} のどれかにしてください: {name}')
    try:
        document = yaml.safe_load(config_path.read_text())
    except OSError as error:
        raise ImuSourceError(f'IMU の設定 {config_path} を読めません: {error}') from error
    except yaml.YAMLError as error:
        raise ImuSourceError(f'IMU の設定 {config_path} の YAML が正しくありません: {error}') from error
    root = _mapping(document, str(config_path))
    sensors = {
        sensor: _parse_sensor(
            sensor, _require(root, sensor, str(config_path)), config_path, robot_description,
        )
        for sensor in SENSORS
    }
    if name != FUSED:
        sensor = sensors[name]
        return ImuSelection(name, sensor.topic, sensor, sensor.accel_scale, None)

    where = f'{config_path} の {FUSED}'
    entry = _mapping(_require(root, FUSED, str(config_path)), where)
    topic = _text(_require(entry, 'topic', where), f'{where}.topic')
    roles = {
        role: _text(_require(entry, role, where), f'{where}.{role}')
        for role in ('primary', 'secondary')
    }
    for role, sensor in roles.items():
        if sensor not in SENSORS:
            raise ImuSourceError(
                f'{where}.{role} は {" / ".join(SENSORS)} のどれかにしてください: {sensor}'
            )
    if roles['primary'] == roles['secondary']:
        raise ImuSourceError(f'{where} の primary と secondary が同じです: {roles["primary"]}')
    if topic in (sensor.topic for sensor in sensors.values()):
        raise ImuSourceError(f'{where}.topic が入力の IMU のトピックと同じです: {topic}')
    primary = sensors[roles['primary']]
    # 融合の出力は主の座標系なので、FAST-LIO の外部パラメータも主のものを使う
    return ImuSelection(FUSED, topic, primary, 1.0, (primary, sensors[roles['secondary']]))


def read_accel_clip_mps2(fast_lio_config: Path) -> float:
    """fast_lio.yaml の common.accel_clip（m/s2 で書く）を読む."""
    try:
        document = yaml.safe_load(fast_lio_config.read_text())
        value = document['/**']['ros__parameters']['common']['accel_clip']
    except OSError as error:
        raise ImuSourceError(f'FAST-LIO の設定 {fast_lio_config} を読めません: {error}') from error
    except (KeyError, TypeError) as error:
        raise ImuSourceError(f'{fast_lio_config} に common.accel_clip がありません') from error
    clip = _number(value, f'{fast_lio_config} の common.accel_clip')
    if clip < 0.0:
        raise ImuSourceError(f'{fast_lio_config} の common.accel_clip は 0 以上にしてください: {clip}')
    return clip


def fast_lio_parameters(selection: ImuSelection, accel_clip_mps2: float) -> dict[str, object]:
    """選んだ IMU に合わせて上書きする FAST-LIO のパラメータ."""
    return {
        'common.imu_topic': selection.topic,
        # accel_clip は IMU の単位で比べるので、m/s2 の値を直す（g 単位の IMU なら約 1/9.8）
        'common.accel_clip': accel_clip_mps2 / selection.accel_scale,
        'mapping.extrinsic_T': list(selection.body.extrinsic_t),
        'mapping.extrinsic_R': list(selection.body.extrinsic_r),
    }


def adapter_parameters(selection: ImuSelection) -> dict[str, object]:
    """lio_odom_adapter のパラメータ。FAST-LIO の姿勢は body（IMU）の座標系なので、その名前を渡す."""
    return {
        'imu_frame': selection.body.frame,
        'lio_extrinsic_T': list(selection.body.extrinsic_t),
        'lio_extrinsic_R': list(selection.body.extrinsic_r),
    }


def scan_filter_parameters(selection: ImuSelection) -> dict[str, object]:
    """scan_filter のパラメータ。/cloud_registered_body は body（IMU）の座標系の点群."""
    return {'output_frame': selection.body.frame}


def fusion_node(selection: ImuSelection, fusion_config: Path, use_sim_time: bool) -> Node | None:
    """fused のときの imu_fusion ノード。それ以外は None."""
    if selection.fusion is None:
        return None
    primary, secondary = selection.fusion
    parameters: dict[str, object] = {
        'use_sim_time': use_sim_time,
        'primary.frame': primary.frame,
        'secondary.frame': secondary.frame,
    }
    for role, sensor in (('primary', primary), ('secondary', secondary)):
        parameters.update({
            f'{role}.accel_scale': sensor.accel_scale,
            f'{role}.gyro_noise_std_radps': sensor.gyro_noise_std_radps,
            f'{role}.accel_noise_std_mps2': sensor.accel_noise_std_mps2,
            f'{role}.gyro_saturation_radps': sensor.gyro_saturation_radps,
            f'{role}.accel_saturation_mps2': sensor.accel_saturation_mps2,
        })
    return Node(
        package='core_localization',
        executable='imu_fusion',
        name='imu_fusion',
        namespace='sensing',
        output='screen',
        parameters=[str(fusion_config), parameters],
        remappings=[
            ('primary', primary.topic),
            ('secondary', secondary.topic),
            ('output', selection.topic),
        ],
    )
