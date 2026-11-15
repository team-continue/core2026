"""点群抽出（地図照合の入力 scan_filter から、照合に使う点だけを取り出す処理）の選択と、観客席の箱の読み込み.

launch 引数 point_extraction でモードを選ぶ。FAST-LIO の手前にはかけない（地図照合の入力だけ）。
- off: かけない（既定。今までと同じ）
- ceiling（天井）: 床と、床から 2.2m より上だけを残す。人の帯（0.1〜2.2m）と、観客席の箱
  （config/exclude_regions.yaml）の中を除く。柵の外（config/field.yaml の範囲から 1.0m より外）では、
  人の帯を 3.0m まで広げる
- field_objects（フィールドオブジェクト）: フィールドとその周り（config/field.yaml の範囲に余裕を足した
  map の座標系の長方形）の、床から 2.2m までの点だけを残す。フィールドの中の相手は、/perception/tracks の
  追跡の箱を点群の時刻まで進めた柱で除く
値は config/point_extraction.yaml。観客席の箱は ROS のパラメータで構造を渡せないので、ここで読んで
point_extraction.ceiling.region.names と boxes（箱ごとに x_min, x_max, y_min, y_max, z_min, z_max）に直す。
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Final, Mapping

import yaml

from core_nav_launch.composition import field_overlay

POINT_EXTRACTION_OFF: Final = 'off'
POINT_EXTRACTION_CEILING: Final = 'ceiling'
POINT_EXTRACTION_FIELD_OBJECTS: Final = 'field_objects'
POINT_EXTRACTION_MODES: Final = (
    POINT_EXTRACTION_OFF, POINT_EXTRACTION_CEILING, POINT_EXTRACTION_FIELD_OBJECTS,
)
DEFAULT_POINT_EXTRACTION: Final = POINT_EXTRACTION_OFF
AXES: Final = ('x', 'y', 'z')


class PointExtractionError(RuntimeError):
    """point_extraction の指定か、観客席の箱の設定ファイルが正しくないときの例外."""


@dataclass(frozen=True)
class ExcludeRegion:
    """map の座標軸に沿った箱。各軸は (下限, 上限) [m]."""

    name: str
    x: tuple[float, float]
    y: tuple[float, float]
    z: tuple[float, float]


def select_point_extraction(mode: str) -> str:
    """launch 引数 point_extraction の値を確かめて返す."""
    if mode not in POINT_EXTRACTION_MODES:
        raise PointExtractionError(
            f'point_extraction は {" / ".join(POINT_EXTRACTION_MODES)} のどれかにしてください: {mode}'
        )
    return mode


def _number(value: object, where: str) -> float:
    # YAML の true/false は int の仲間なので、数値として受けない
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PointExtractionError(f'{where} は数値にしてください: {value!r}')
    number = float(value)
    if not math.isfinite(number):
        raise PointExtractionError(f'{where} が有限の値ではありません: {value!r}')
    return number


def _range(value: object, where: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise PointExtractionError(f'{where} は [下限, 上限] の 2 個の数値にしてください: {value!r}')
    low = _number(value[0], f'{where}[0]')
    high = _number(value[1], f'{where}[1]')
    if low > high:
        raise PointExtractionError(f'{where} の下限 {low} が上限 {high} を超えています')
    return low, high


def _parse_region(index: int, value: object, path: Path) -> ExcludeRegion:
    where = f'{path} の regions[{index}]'
    if not isinstance(value, dict):
        raise PointExtractionError(f'{where} は name、x、y、z を持つ項目にしてください: {value!r}')
    entry: Mapping[str, object] = value
    unknown = sorted(set(entry) - {'name', *AXES})
    if unknown:
        raise PointExtractionError(f'{where} に知らない項目があります: {unknown}')
    name = entry.get('name')
    if not isinstance(name, str) or not name:
        raise PointExtractionError(f'{where}.name は空でない文字列にしてください: {name!r}')
    missing = [axis for axis in AXES if axis not in entry]
    if missing:
        raise PointExtractionError(f'{where}（{name}）に {missing} がありません')
    x, y, z = (_range(entry[axis], f'{where}（{name}）.{axis}') for axis in AXES)
    return ExcludeRegion(name=name, x=x, y=y, z=z)


def load_exclude_regions(path: Path) -> tuple[ExcludeRegion, ...]:
    """観客席の箱の設定ファイルを読む."""
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except OSError as error:
        raise PointExtractionError(f'観客席の箱の設定 {path} を読めません: {error}') from error
    except yaml.YAMLError as error:
        raise PointExtractionError(f'観客席の箱の設定 {path} の YAML が正しくありません: {error}') from error
    if not isinstance(document, dict) or 'regions' not in document:
        raise PointExtractionError(f'{path} に regions（箱の一覧）がありません')
    items = document['regions'] or []
    if not isinstance(items, list):
        raise PointExtractionError(f'{path} の regions は一覧にしてください: {items!r}')
    regions = tuple(_parse_region(index, item, path) for index, item in enumerate(items))
    names = [region.name for region in regions]
    duplicated = sorted({name for name in names if names.count(name) > 1})
    if duplicated:
        raise PointExtractionError(f'{path} の箱の名前が重複しています: {duplicated}')
    return regions


def region_parameters(regions: tuple[ExcludeRegion, ...]) -> dict[str, object]:
    """観客席の箱を scan_filter のパラメータに直す.

    空の配列は ROS のパラメータの型が決まらず起動できないので、箱がなければ何も渡さない
    （ノードは箱なしとして警告する）。
    """
    if not regions:
        return {}
    return {
        'point_extraction.ceiling.region.names': [region.name for region in regions],
        'point_extraction.ceiling.region.boxes': [
            value for region in regions for value in (*region.x, *region.y, *region.z)
        ],
    }


def point_extraction_parameters(
    mode: str, config_dir: Path, exclude_regions_file: Path, field_file: str = '',
) -> list[object]:
    """scan_filter に渡す点群抽出のパラメータ（設定ファイルと上書き）.

    off ならノードの既定値（off）のままにするので、何も渡さない。
    """
    selected = select_point_extraction(mode)
    if selected == POINT_EXTRACTION_OFF:
        return []
    parameters: list[object] = [
        str(config_dir / 'point_extraction.yaml'),
        # field_objects の残す範囲と ceiling の柵の外の帯は、検出と同じフィールドの範囲から作る
        str(config_dir / 'field.yaml'),
        *field_overlay(field_file),
        {'point_extraction.mode': selected},
    ]
    if selected == POINT_EXTRACTION_CEILING:
        parameters.append(region_parameters(load_exclude_regions(exclude_regions_file)))
    return parameters
