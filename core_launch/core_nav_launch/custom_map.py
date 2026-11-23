"""navigation.launch.py の map_name に、プリセットにない名前（make_map_files.py で作った地図）を渡したときの解決.

core_launch/maps に、同じ名前の 3 つのファイルがそろっていれば、その名前で選べる。
- <name>.pcd: 地図照合と静止物の地図（lio_map_saver_node.py が保存する。床が z=0）
- <name>.yaml（と画像）: UI に出す占有地図。地図照合が LIO を失ったときに捕まえ直す範囲にも使う
- <name>_field.yaml: フィールドの範囲（config/field.yaml に重ねる）
初期姿勢は地図の原点（地図を作り始めたときの機体の位置と向き）。navigation.launch.py の init_x・init_y・
init_yaw で変えられる。
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Final

from core_nav_launch.occupancy_map import load_map_yaml

_NAME: Final = re.compile(r'^[A-Za-z0-9_\-]+$')


class CustomMapError(Exception):
    """作った地図を選べない."""


def custom_map_files(maps_dir: Path, name: str) -> tuple[Path, Path, Path]:
    """地図の PCD、占有地図の yaml、フィールドの範囲の yaml の場所."""
    if not _NAME.match(name):
        raise CustomMapError(f'地図の名前に使えるのは英数字と _ - だけです: {name!r}')
    return maps_dir / f'{name}.pcd', maps_dir / f'{name}.yaml', maps_dir / f'{name}_field.yaml'


def available_custom_maps(maps_dir: Path) -> list[str]:
    """3 つのファイルがそろっている地図の名前."""
    names = sorted(path.stem for path in maps_dir.glob('*.pcd'))
    return [
        name for name in names
        if _NAME.match(name) and all(path.is_file() for path in custom_map_files(maps_dir, name))
    ]


def custom_map_preset(maps_dir: Path, name: str) -> dict[str, str | float]:
    """navigation.launch.py の MAP_PRESETS と同じ形の値。ファイルの場所は絶対パス."""
    pcd, occupancy_yaml, field = custom_map_files(maps_dir, name)
    missing = [path.name for path in (pcd, occupancy_yaml, field) if not path.is_file()]
    if missing:
        raise CustomMapError(
            f"地図 '{name}' のファイルが {maps_dir} にありません: {', '.join(missing)}"
            '（lio_map_saver_node.py で PCD を保存し、make_map_files.py で残りを作ってから、'
            'core_launch をビルドし直してください）')
    source = load_map_yaml(occupancy_yaml)
    return {
        'pcd': str(pcd),
        'field': str(field),
        'image': str(source.image),
        'resolution': float(source.resolution),
        'origin_x': float(source.origin_x),
        'origin_y': float(source.origin_y),
        'init_x': 0.0,
        'init_y': 0.0,
        'init_yaw': 0.0,
    }
