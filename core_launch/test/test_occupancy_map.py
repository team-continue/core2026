from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from core_nav_launch.occupancy_map import (
    FREE,
    load_map_yaml,
    load_occupancy_map,
    MapSource,
    OCCUPIED,
    occupancy_from_pixels,
    OccupancyMapError,
    UNKNOWN,
)
import numpy as np
import pytest

MAPS = Path(get_package_share_directory('core_launch')) / 'maps'


def test_pixels_follow_map_server_trinary_and_flip_rows() -> None:
    # 上の行: 黒（占有）・白（空き）、下の行: 灰色 205（不明）・白。しきい値は arena_*.yaml と同じ
    # （free_thresh 0.196。map_server の既定 0.25 なら 205 は空き）
    pixels = np.array([[0, 255], [205, 254]], dtype=np.uint8)
    cells = occupancy_from_pixels(pixels, MapSource(Path('x.png'), 0.1, 0.0, 0.0, 0.65, 0.196))
    # OccupancyGrid は下の行（原点の側）から並べる
    assert cells.tolist() == [[UNKNOWN, FREE], [OCCUPIED, FREE]]
    negated = occupancy_from_pixels(
        pixels, MapSource(Path('x.png'), 0.1, 0.0, 0.0, 0.65, 0.196, negate=True))
    assert negated[1].tolist() == [FREE, OCCUPIED]


def test_invalid_thresholds_are_rejected() -> None:
    with pytest.raises(OccupancyMapError, match='free_thresh'):
        occupancy_from_pixels(
            np.zeros((1, 1), dtype=np.uint8), MapSource(Path('x.png'), 0.1, 0.0, 0.0, 0.2, 0.3))
    with pytest.raises(OccupancyMapError, match='resolution'):
        occupancy_from_pixels(np.zeros((1, 1), dtype=np.uint8), MapSource(Path('x.png'), 0.0, 0.0, 0.0))


def test_repository_map_yaml_loads() -> None:
    source = load_map_yaml(MAPS / 'arena_static.yaml')
    assert source.image == MAPS / 'arena_static.pgm'
    assert source.resolution == pytest.approx(0.2)
    occupancy = load_occupancy_map(source)
    assert occupancy.cells.shape == (241, 347)
    assert int(np.sum(occupancy.cells == OCCUPIED)) > 0
    assert int(np.sum(occupancy.cells == FREE)) > 0


def test_preset_image_loads() -> None:
    # navigation.launch.py の core1_field のプリセット
    occupancy = load_occupancy_map(MapSource(MAPS / 'core1_field.png', 0.025, -13.675, -9.15))
    assert occupancy.cells.shape == (732, 1104)


def test_missing_yaml_key_is_reported(tmp_path: Path) -> None:
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.1\n', encoding='utf-8')
    with pytest.raises(OccupancyMapError, match='origin'):
        load_map_yaml(path)
