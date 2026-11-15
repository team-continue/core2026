"""自機の除去（機体の箱）を、FAST-LIO の前処理のパラメータに直す.

検出（perception_node）と FAST-LIO は、同じ箱の中の点を自機として除く。
- 箱の大きさ: config/footprint.yaml の footprint.length_m・width_m の半分に self_filter_margin_m を足す。
  上面は footprint.height_m（機体の上面の床からの高さ）に self_filter_margin_m を足す
- LiDAR の取り付け: ロボットの記述（URDF。xacro なら処理する）の、base_link から LiDAR のフレームまでの
  固定の関節の変換を合成する
- 判定: base_link（床に投影した機体中心）で xy が箱の中にあり、高さが箱の上面以下の点を除く（FAST-LIO の
  preprocess.cpp の in_self_box と、core_perception の selectObstaclePoints で同じ）。LiDAR は機体の前の端の
  低い所にあるので、LiDAR より上の機体の面も除く

launch 引数 lio_self_filter で FAST-LIO の除き方を選ぶ。
- box: 機体の箱の中の点を除く。blind は fast_lio.yaml の値（近すぎる雑音だけを除く）（既定）
- legacy_blind: 以前の動き。箱を使わず、blind を LEGACY_BLIND_M の球にする（比べるためだけに残す）
"""

from dataclasses import dataclass
from functools import reduce
import math
from pathlib import Path
from typing import Final, Mapping
from xml.etree import ElementTree

import xacro
import yaml

LIO_SELF_FILTER_BOX: Final = 'box'
LIO_SELF_FILTER_LEGACY_BLIND: Final = 'legacy_blind'
LIO_SELF_FILTER_MODES: Final = (LIO_SELF_FILTER_BOX, LIO_SELF_FILTER_LEGACY_BLIND)
DEFAULT_LIO_SELF_FILTER: Final = LIO_SELF_FILTER_BOX
# 以前の fast_lio.yaml の blind [m]。自機の天板の角（約 0.57m）まで含む球
LEGACY_BLIND_M: Final = 0.65
# 機体のフレームと、点群のフレーム（実機のドライバと同じ名前）
BASE_FRAME: Final = 'base_link'
LIDAR_FRAME: Final = 'livox_frame'


class SelfFilterError(RuntimeError):
    """自機の除去の設定（機体の形、ロボットの記述、launch 引数）が正しくないときの例外."""


# 自機の除去の形（footprint.yaml の footprint.self_filter_shape）。検出の perception_node と同じ値
SELF_FILTER_BOX: Final = 'box'
SELF_FILTER_CYLINDER: Final = 'cylinder'
SELF_FILTER_SHAPES: Final = (SELF_FILTER_BOX, SELF_FILTER_CYLINDER)

@dataclass(frozen=True)
class SelfBox:
    """base_link の座標軸に沿った箱。xy は半分の長さ、top_height_m は上面の床からの高さ [m]."""

    half_length_m: float
    half_width_m: float
    top_height_m: float
    # 0 より大きければ、xy は箱ではなくこの半径の円柱（footprint.self_filter_shape: cylinder）
    radius_m: float = 0.0


@dataclass(frozen=True)
class Transform:
    """子の座標系の点を親へ移す変換。p_parent = rotation p_child + translation.

    rotation は行優先の 3x3。
    """

    rotation: tuple[float, ...]
    translation: tuple[float, float, float]


IDENTITY: Final = Transform((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0))


def compose(outer: Transform, inner: Transform) -> Transform:
    """inner の子の点を outer の親へ移す変換（outer・inner）."""
    r, s = outer.rotation, inner.rotation
    rotation = tuple(
        sum(r[3 * i + k] * s[3 * k + j] for k in range(3)) for i in range(3) for j in range(3)
    )
    x, y, z = (
        sum(r[3 * i + k] * inner.translation[k] for k in range(3)) + outer.translation[i]
        for i in range(3)
    )
    return Transform(rotation, (x, y, z))


def invert(transform: Transform) -> Transform:
    """逆の変換（親の点を子へ移す）。回転は直交行列なので転置を使う."""
    r, t = transform.rotation, transform.translation
    rotation = tuple(r[3 * j + i] for i in range(3) for j in range(3))
    x, y, z = (-sum(rotation[3 * i + k] * t[k] for k in range(3)) for i in range(3))
    return Transform(rotation, (x, y, z))


def rpy_matrix(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """URDF の rpy（固定軸で x, y, z の順に回す）の回転行列。R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    )


def _number(value: object, where: str) -> float:
    # YAML の true/false は int の仲間なので、数値として受けない
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SelfFilterError(f'{where} は数値にしてください: {value!r}')
    number = float(value)
    if not math.isfinite(number):
        raise SelfFilterError(f'{where} が有限の値ではありません: {value!r}')
    return number


def load_self_box(footprint_config: Path) -> SelfBox:
    """footprint.yaml から箱を作る。検出の perception_node と同じ計算."""
    try:
        document = yaml.safe_load(footprint_config.read_text())
    except OSError as error:
        raise SelfFilterError(f'機体の形の設定 {footprint_config} を読めません: {error}') from error
    except yaml.YAMLError as error:
        raise SelfFilterError(
            f'機体の形の設定 {footprint_config} の YAML が正しくありません: {error}'
        ) from error
    try:
        footprint = document['/**']['ros__parameters']['footprint']
    except (KeyError, TypeError) as error:
        raise SelfFilterError(
            f'{footprint_config} に /**.ros__parameters.footprint がありません'
        ) from error
    if not isinstance(footprint, dict):
        raise SelfFilterError(f'{footprint_config} の footprint は項目の辞書にしてください: {footprint!r}')

    def value(key: str) -> float:
        if key not in footprint:
            raise SelfFilterError(f'{footprint_config} に footprint.{key} がありません')
        return _number(footprint[key], f'{footprint_config} の footprint.{key}')

    length, width, height = value('length_m'), value('width_m'), value('height_m')
    margin = value('self_filter_margin_m')
    if length <= 0.0 or width <= 0.0 or height <= 0.0:
        raise SelfFilterError(
            f'{footprint_config} の footprint.length_m、width_m、height_m は正の値にしてください: '
            f'{length}, {width}, {height}'
        )
    if margin < 0.0:
        raise SelfFilterError(
            f'{footprint_config} の footprint.self_filter_margin_m は 0 以上にしてください: {margin}'
        )
    shape = footprint.get('self_filter_shape', SELF_FILTER_BOX)
    if shape not in SELF_FILTER_SHAPES:
        raise SelfFilterError(
            f'{footprint_config} の footprint.self_filter_shape は {" / ".join(SELF_FILTER_SHAPES)} の'
            f'どれかにしてください: {shape!r}'
        )
    radius = math.hypot(0.5 * length, 0.5 * width) + margin if shape == SELF_FILTER_CYLINDER else 0.0
    return SelfBox(0.5 * length + margin, 0.5 * width + margin, height + margin, radius)


def parse_xacro_args(text: str) -> dict[str, str]:
    """'name:=value name2:=value2' を xacro の引数の辞書にする。空なら xacro の既定値を使う."""
    mappings: dict[str, str] = {}
    for token in text.split():
        name, separator, value = token.partition(':=')
        if not separator or not name:
            raise SelfFilterError(
                f'robot_description_args は name:=value を空白で区切って書いてください: {token}'
            )
        mappings[name] = value
    return mappings


def load_robot_description(path: Path, xacro_args: Mapping[str, str]) -> str:
    """ロボットの記述を URDF の文字列にする。.xacro なら引数を渡して処理する."""
    if not path.is_file():
        raise SelfFilterError(f'ロボットの記述 {path} が見つかりません')
    if path.suffix != '.xacro':
        if xacro_args:
            raise SelfFilterError(f'xacro の引数は .xacro のときだけ使えます: {path} {dict(xacro_args)}')
        try:
            return path.read_text()
        except OSError as error:
            raise SelfFilterError(f'ロボットの記述 {path} を読めません: {error}') from error
    try:
        return xacro.process_file(str(path), mappings=dict(xacro_args)).toxml()
    except Exception as error:  # xacro は書式の誤りなどで様々な例外を投げる
        raise SelfFilterError(
            f'ロボットの記述 {path} を xacro で処理できません（引数 {dict(xacro_args)}）: {error}'
        ) from error


def _vector(text: str, where: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) != 3:
        raise SelfFilterError(f'{where} は 3 個の数値にしてください: {text!r}')
    try:
        x, y, z = (float(part) for part in parts)
    except ValueError as error:
        raise SelfFilterError(f'{where} は 3 個の数値にしてください: {text!r}') from error
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise SelfFilterError(f'{where} が有限の値ではありません: {text!r}')
    return (x, y, z)


def _joint_origin(joint: ElementTree.Element, name: str) -> Transform:
    origin = joint.find('origin')
    if origin is None:
        return IDENTITY
    xyz = _vector(origin.get('xyz', '0 0 0'), f'関節 {name} の origin xyz')
    roll, pitch, yaw = _vector(origin.get('rpy', '0 0 0'), f'関節 {name} の origin rpy')
    return Transform(rpy_matrix(roll, pitch, yaw), xyz)


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    origin: Transform


def _joint_link(joint: ElementTree.Element, tag: str, name: str) -> str:
    element = joint.find(tag)
    link = element.get('link') if element is not None else None
    if not link:
        raise SelfFilterError(f'関節 {name} に {tag} のリンクがありません')
    return link


def fixed_transform(urdf_xml: str, parent: str, child: str) -> Transform:
    """URDF の parent から child までの固定の関節を合成し、child の点を parent へ移す変換を返す."""
    try:
        robot = ElementTree.fromstring(urdf_xml)
    except ElementTree.ParseError as error:
        raise SelfFilterError(f'ロボットの記述の XML が正しくありません: {error}') from error
    links = {link.get('name') for link in robot.findall('link')}
    for frame in (parent, child):
        if frame not in links:
            raise SelfFilterError(
                f'ロボットの記述にフレーム {frame} がありません（リンク: {", ".join(sorted(map(str, links)))}）'
            )
    joints: dict[str, _Joint] = {}
    for element in robot.findall('joint'):
        name = element.get('name', '')
        joint = _Joint(
            name=name,
            kind=element.get('type', ''),
            parent=_joint_link(element, 'parent', name),
            origin=_joint_origin(element, name),
        )
        joints[_joint_link(element, 'child', name)] = joint

    # child から親をたどる。木なので、関節の数より長くなれば輪になっている
    chain: list[_Joint] = []
    frame = child
    while frame != parent:
        joint = joints.get(frame)
        if joint is None:
            raise SelfFilterError(
                f'{child} は {parent} の下にありません（{frame} の親の関節がありません）'
            )
        if joint.kind != 'fixed':
            raise SelfFilterError(
                f'{parent} から {child} までに固定でない関節 {joint.name}（{joint.kind}）があるので、'
                '取り付けの変換を決められません'
            )
        if len(chain) >= len(joints):
            raise SelfFilterError(f'ロボットの記述の関節が輪になっています（{child} から {parent} へ）')
        chain.append(joint)
        frame = joint.parent
    # chain は child 側から並ぶので、parent 側から合成する
    return reduce(compose, (joint.origin for joint in reversed(chain)), IDENTITY)


def self_filter_parameters(box: SelfBox, base_from_lidar: Transform) -> dict[str, object]:
    """FAST-LIO の preprocess.self_filter.* の値."""
    return {
        'preprocess.self_filter.enabled': True,
        'preprocess.self_filter.half_length': box.half_length_m,
        'preprocess.self_filter.half_width': box.half_width_m,
        'preprocess.self_filter.top_height': box.top_height_m,
        'preprocess.self_filter.radius': box.radius_m,
        'preprocess.self_filter.extrinsic_T': list(base_from_lidar.translation),
        'preprocess.self_filter.extrinsic_R': list(base_from_lidar.rotation),
    }


def fast_lio_self_filter_parameters(
    mode: str, footprint_config: Path, robot_description: Path, xacro_args: Mapping[str, str],
) -> dict[str, object]:
    """launch 引数 lio_self_filter に合わせて上書きする FAST-LIO のパラメータ."""
    if mode not in LIO_SELF_FILTER_MODES:
        raise SelfFilterError(
            f'lio_self_filter は {" / ".join(LIO_SELF_FILTER_MODES)} のどれかにしてください: {mode}'
        )
    if mode == LIO_SELF_FILTER_LEGACY_BLIND:
        return {'preprocess.self_filter.enabled': False, 'preprocess.blind': LEGACY_BLIND_M}
    urdf = load_robot_description(robot_description, xacro_args)
    return self_filter_parameters(
        load_self_box(footprint_config), fixed_transform(urdf, BASE_FRAME, LIDAR_FRAME),
    )
