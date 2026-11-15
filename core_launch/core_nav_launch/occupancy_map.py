"""起動した地図の占有地図を読む（map_publisher_node.py と launch で使う）.

地図は 2 通りで指定できる。
- ROS の map_server の yaml（image・resolution・origin・occupied_thresh・free_thresh・negate）。Gazebo の
  arena_*.yaml
- 画像と解像度・原点（navigation.launch.py の map_name のプリセット）。しきい値は map_server の既定値

画像は上の行が map の y の大きい側。OccupancyGrid は下の行（原点の側）から並べるので、上下を返す。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Final

import numpy as np
from numpy.typing import NDArray
import yaml

# map_server の既定値
DEFAULT_OCCUPIED_THRESH: Final = 0.65
DEFAULT_FREE_THRESH: Final = 0.25

OCCUPIED: Final = 100
FREE: Final = 0
UNKNOWN: Final = -1


class OccupancyMapError(Exception):
    """占有地図を読めない."""


@dataclass(frozen=True)
class MapSource:
    image: Path
    resolution: float
    origin_x: float
    origin_y: float
    occupied_thresh: float = DEFAULT_OCCUPIED_THRESH
    free_thresh: float = DEFAULT_FREE_THRESH
    negate: bool = False


@dataclass(frozen=True)
class OccupancyMap:
    resolution: float
    origin_x: float
    origin_y: float
    # 下の行（原点の側）から並べた占有の値（-1・0・100）。形は (高さ, 幅)
    cells: NDArray[np.int8]


def _number(value: object, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise OccupancyMapError(f'{where} は数値にしてください: {value!r}')
    number = float(value)
    if not math.isfinite(number):
        raise OccupancyMapError(f'{where} が有限の値ではありません: {value!r}')
    return number


def load_map_yaml(path: Path) -> MapSource:
    """map_server の yaml を読む。image は yaml からの相対の場所でもよい."""
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except OSError as error:
        raise OccupancyMapError(f'地図の yaml {path} を読めません: {error}') from error
    except yaml.YAMLError as error:
        raise OccupancyMapError(f'地図の yaml {path} の書式が正しくありません: {error}') from error
    if not isinstance(document, dict):
        raise OccupancyMapError(f'地図の yaml {path} は項目の辞書にしてください')
    for key in ('image', 'resolution', 'origin'):
        if key not in document:
            raise OccupancyMapError(f'地図の yaml {path} に {key} がありません')
    origin = document['origin']
    if not isinstance(origin, list) or len(origin) < 2:
        raise OccupancyMapError(f'地図の yaml {path} の origin は [x, y, yaw] にしてください: {origin!r}')
    image = Path(str(document['image']))
    if not image.is_absolute():
        image = path.parent / image
    return MapSource(
        image=image,
        resolution=_number(document['resolution'], f'{path} の resolution'),
        origin_x=_number(origin[0], f'{path} の origin[0]'),
        origin_y=_number(origin[1], f'{path} の origin[1]'),
        occupied_thresh=_number(
            document.get('occupied_thresh', DEFAULT_OCCUPIED_THRESH), f'{path} の occupied_thresh'),
        free_thresh=_number(document.get('free_thresh', DEFAULT_FREE_THRESH), f'{path} の free_thresh'),
        negate=bool(document.get('negate', 0)),
    )


def occupancy_from_pixels(pixels: NDArray[np.uint8], source: MapSource) -> NDArray[np.int8]:
    """画像の画素（上の行から）を、占有の値（下の行から）にする。map_server の trinary と同じ判定."""
    if source.resolution <= 0.0:
        raise OccupancyMapError(f'resolution は正の値にしてください: {source.resolution}')
    if not 0.0 <= source.free_thresh < source.occupied_thresh <= 1.0:
        raise OccupancyMapError(
            f'free_thresh < occupied_thresh の 0〜1 の値にしてください: '
            f'{source.free_thresh}, {source.occupied_thresh}')
    value = pixels.astype(np.float64) / 255.0
    # 暗いほど占有（negate なら逆）
    occupancy_ratio = value if source.negate else 1.0 - value
    cells = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    cells[occupancy_ratio > source.occupied_thresh] = OCCUPIED
    cells[occupancy_ratio < source.free_thresh] = FREE
    return np.flipud(cells).copy()


def load_occupancy_map(source: MapSource) -> OccupancyMap:
    """画像を読み、占有地図にする."""
    # PIL は rclpy のノードを作る前でなくてもよいが、使うときだけ読む（launch で読まない）
    from PIL import Image

    try:
        with Image.open(source.image) as image:
            pixels = np.asarray(image.convert('L'), dtype=np.uint8)
    except OSError as error:
        raise OccupancyMapError(f'地図の画像 {source.image} を読めません: {error}') from error
    return OccupancyMap(
        resolution=source.resolution,
        origin_x=source.origin_x,
        origin_y=source.origin_y,
        cells=occupancy_from_pixels(pixels, source),
    )
