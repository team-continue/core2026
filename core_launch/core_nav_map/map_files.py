"""地図の PCD の読み書きと、PCD から UI の占有地図・フィールドの範囲を作る処理（make_map_files.py が使う）.

地図は床を z=0 にした座標（lio_map_saver_node.py が保存する座標）を前提にする。
- 占有: 床から min_height_m〜max_height_m の高さに点のあるセル（壁や机など、機体が当たる物）
- 空き: 床（|z| ≤ floor_tolerance_m）の点から free_fill_m 以内で、占有でないセル（地図の点はボクセルに 1 点なので、
  セルが細かいと床の点のないセルが残る。少し広げて埋める）
- 不明: どちらでもないセル（走って見ていない所）
フィールドの範囲は空きのセルの外接の長方形にする（計画がゴールや経路を置ける範囲になる）。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Final

import numpy as np
from numpy.typing import NDArray
from PIL import Image
import yaml

# map_server の trinary の画素の値（make_core1_map.py の占有地図と同じ）
PIXEL_OCCUPIED: Final = 0
PIXEL_FREE: Final = 254
PIXEL_UNKNOWN: Final = 205

_PCD_TYPES: Final = {
    ('F', 4): '<f4', ('F', 8): '<f8',
    ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
    ('U', 1): '<u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
}


class MapFileError(Exception):
    """地図のファイルを読めない、または地図にできない."""


# ---------------------------------------------------------------- PCD

def write_pcd(points: NDArray[np.floating], out: Path) -> None:
    """x y z（float32）の binary の PCD を書く."""
    data = np.ascontiguousarray(np.asarray(points, dtype=np.float32).reshape(-1, 3))
    count = data.shape[0]
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n'
        'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n'
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    temporary = out.with_name(out.name + '.tmp')
    temporary.write_bytes(header.encode('ascii') + data.tobytes())
    # 書きかけのファイルを地図として読まないよう、書き終えてから名前を変える
    temporary.replace(out)


def read_pcd(path: Path) -> NDArray[np.float64]:
    """PCD（ascii か binary）の x y z を読む。ほかの欄（intensity など）は捨てる."""
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise MapFileError(f'PCD を読めません: {path}: {error}') from error
    header: dict[str, list[str]] = {}
    offset = 0
    while True:
        end = raw.find(b'\n', offset)
        if end < 0:
            raise MapFileError(f'PCD のヘッダが DATA で終わっていません: {path}')
        line = raw[offset:end].decode('ascii', errors='replace').strip()
        offset = end + 1
        if not line or line.startswith('#'):
            continue
        key, *values = line.split()
        header[key.upper()] = values
        if key.upper() == 'DATA':
            break
    try:
        fields = header['FIELDS']
        sizes = [int(value) for value in header['SIZE']]
        types = header['TYPE']
        counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
        points = int(header['POINTS'][0])
        data_format = header['DATA'][0].lower()
    except (KeyError, IndexError, ValueError) as error:
        raise MapFileError(f'PCD のヘッダを読めません: {path}: {error}') from error
    if not {'x', 'y', 'z'} <= set(fields):
        raise MapFileError(f'PCD に x y z の欄がありません: {path}: {fields}')
    if data_format == 'ascii':
        text = raw[offset:].decode('ascii', errors='replace').split()
        width = sum(counts)
        if len(text) < width * points:
            raise MapFileError(f'PCD の点が足りません: {path}')
        values = np.array(text[:width * points], dtype=np.float64).reshape(points, width)
        columns = np.cumsum([0] + counts[:-1])
        return np.stack([values[:, columns[fields.index(axis)]] for axis in 'xyz'], axis=1)
    if data_format != 'binary':
        raise MapFileError(
            f'PCD の DATA {data_format} は読めません（ascii か binary にしてください。pcl_convert_pcd_ascii_binary '
            f'で変換できます）: {path}')
    try:
        dtype = np.dtype([
            (name, _PCD_TYPES[(kind, size)], (count,)) if count > 1 else (name, _PCD_TYPES[(kind, size)])
            for name, kind, size, count in zip(fields, types, sizes, counts)
        ])
    except KeyError as error:
        raise MapFileError(f'PCD の欄の型を読めません: {path}: {error}') from error
    body = raw[offset:offset + dtype.itemsize * points]
    if len(body) < dtype.itemsize * points:
        raise MapFileError(f'PCD の点が足りません: {path}')
    records = np.frombuffer(body, dtype=dtype, count=points)
    cloud = np.stack([records[axis].astype(np.float64) for axis in 'xyz'], axis=1)
    return cloud[np.all(np.isfinite(cloud), axis=1)]


# ---------------------------------------------------------------- 占有地図

@dataclass(frozen=True)
class OccupancyImage:
    """画素（上の行が y の大きい側）と、左下の角の座標."""

    pixels: NDArray[np.uint8]
    resolution: float
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class Bounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float


def _dilate(mask: NDArray[np.bool_], radius_cells: int) -> NDArray[np.bool_]:
    """radius_cells 以内（円）のセルへ広げる."""
    if radius_cells <= 0:
        return mask
    height, width = mask.shape
    padded = np.pad(mask, radius_cells)
    out = np.zeros_like(mask)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                out |= padded[radius_cells + dy:radius_cells + dy + height, radius_cells + dx:radius_cells + dx + width]
    return out


def occupancy_image(
    points: NDArray[np.float64], resolution: float, min_height_m: float, max_height_m: float,
    floor_tolerance_m: float, margin_m: float, free_fill_m: float = 0.0,
) -> OccupancyImage:
    """点群から trinary の占有地図を作る."""
    if not resolution > 0.0:
        raise MapFileError(f'解像度は正の値にしてください: {resolution}')
    if not 0.0 < min_height_m < max_height_m:
        raise MapFileError(f'高さの範囲が正しくありません: {min_height_m}〜{max_height_m}')
    floor = points[np.abs(points[:, 2]) <= floor_tolerance_m]
    obstacles = points[(points[:, 2] >= min_height_m) & (points[:, 2] <= max_height_m)]
    if floor.shape[0] == 0:
        raise MapFileError(
            f'床（|z| ≤ {floor_tolerance_m}m）の点がありません。床が z=0 の座標の地図か確かめてください')
    used = np.concatenate([floor, obstacles])
    x_min = math.floor((used[:, 0].min() - margin_m) / resolution) * resolution
    y_min = math.floor((used[:, 1].min() - margin_m) / resolution) * resolution
    width = int(math.ceil((used[:, 0].max() + margin_m - x_min) / resolution)) + 1
    height = int(math.ceil((used[:, 1].max() + margin_m - y_min) / resolution)) + 1

    def cells(cloud: NDArray[np.float64]) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
        column = np.floor((cloud[:, 0] - x_min) / resolution).astype(np.int64)
        row_from_bottom = np.floor((cloud[:, 1] - y_min) / resolution).astype(np.int64)
        return height - 1 - row_from_bottom, column

    if free_fill_m < 0.0:
        raise MapFileError(f'床を広げる距離は 0 以上にしてください: {free_fill_m}')
    floor_mask = np.zeros((height, width), dtype=bool)
    floor_mask[cells(floor)] = True
    pixels = np.full((height, width), PIXEL_UNKNOWN, dtype=np.uint8)
    pixels[_dilate(floor_mask, int(round(free_fill_m / resolution)))] = PIXEL_FREE
    if obstacles.shape[0] > 0:
        pixels[cells(obstacles)] = PIXEL_OCCUPIED
    return OccupancyImage(pixels, resolution, x_min, y_min)


def free_bounds(image: OccupancyImage) -> Bounds:
    """空きのセルの外接の長方形."""
    rows, columns = np.nonzero(image.pixels == PIXEL_FREE)
    if rows.shape[0] == 0:
        raise MapFileError('空きのセルがありません')
    height = image.pixels.shape[0]
    return Bounds(
        x_min=image.origin_x + float(columns.min()) * image.resolution,
        x_max=image.origin_x + float(columns.max() + 1) * image.resolution,
        y_min=image.origin_y + float(height - 1 - rows.max()) * image.resolution,
        y_max=image.origin_y + float(height - rows.min()) * image.resolution,
    )


def write_occupancy(image: OccupancyImage, name: str, out_dir: Path, source: str) -> Path:
    """<name>.png と map_server の <name>.yaml を書き、yaml の場所を返す."""
    out_dir.mkdir(parents=True, exist_ok=True)
    # uint8 の 2 次元の配列はグレースケール（L）になる
    Image.fromarray(image.pixels).save(out_dir / f'{name}.png')
    yaml_path = out_dir / f'{name}.yaml'
    yaml_path.write_text(
        f'# make_map_files.py が {source} から作った、UI に出す占有地図（壁などが占有、走って見た床が空き）\n'
        f'image: {name}.png\nresolution: {image.resolution}\n'
        f'origin: [{image.origin_x:.4f}, {image.origin_y:.4f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n', encoding='utf-8')
    return yaml_path


def write_field_config(bounds: Bounds, out: Path, source: str) -> None:
    """フィールドの範囲（config/field.yaml に重ねる設定）を書く."""
    document = {'/**': {'ros__parameters': {'field': {
        'x_min': round(bounds.x_min, 3), 'x_max': round(bounds.x_max, 3),
        'y_min': round(bounds.y_min, 3), 'y_max': round(bounds.y_max, 3),
    }}}}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        f'# make_map_files.py が {source} から作ったフィールドの範囲（走って見た床の外接の長方形）。\n'
        '# navigation.launch.py の map_name:=<地図の名前> で config/field.yaml に重ねる。\n'
        + yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding='utf-8')
