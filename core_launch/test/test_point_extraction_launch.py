"""core_nav_launch.point_extraction（点群抽出のモードと観客席の箱）の試験."""

from pathlib import Path

from core_nav_launch.point_extraction import (
    load_exclude_regions,
    point_extraction_parameters,
    PointExtractionError,
    region_parameters,
    select_point_extraction,
)
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def test_modes() -> None:
    for mode in ('off', 'ceiling', 'field_objects'):
        assert select_point_extraction(mode) == mode
    # 以前の crowd_filter の値（FAST-LIO の手前にもかける lio_and_map など）は受けない
    for mode in ('map', 'lio_and_map', ''):
        with pytest.raises(PointExtractionError, match='point_extraction'):
            select_point_extraction(mode)


def test_parameters_for_each_mode() -> None:
    regions_file = CONFIG / 'exclude_regions.yaml'
    assert point_extraction_parameters('off', CONFIG, regions_file) == []
    ceiling = point_extraction_parameters('ceiling', CONFIG, regions_file)
    assert ceiling[:3] == [
        str(CONFIG / 'point_extraction.yaml'), str(CONFIG / 'field.yaml'),
        {'point_extraction.mode': 'ceiling'},
    ]
    assert ceiling[3]['point_extraction.ceiling.region.names'] == ['stand_north']
    assert len(ceiling[3]['point_extraction.ceiling.region.boxes']) == 6
    # field_objects は観客席の箱を使わないので、ファイルがなくても起動できる
    field = point_extraction_parameters('field_objects', CONFIG, Path('/nonexistent.yaml'))
    assert field == [
        str(CONFIG / 'point_extraction.yaml'), str(CONFIG / 'field.yaml'),
        {'point_extraction.mode': 'field_objects'},
    ]
    with pytest.raises(PointExtractionError, match='読めません'):
        point_extraction_parameters('ceiling', CONFIG, Path('/nonexistent.yaml'))


def test_config_values() -> None:
    document = yaml.safe_load((CONFIG / 'point_extraction.yaml').read_text())
    params = document['/**']['ros__parameters']['point_extraction']
    # 人の帯の上限と field_objects の高さの上限は同じ 2.2m、残す範囲の余裕は 0.5m、追跡の余裕は 0.2m
    assert params['ceiling']['band_max_m'] == pytest.approx(2.2)
    assert params['field_objects']['max_height_m'] == pytest.approx(2.2)
    assert params['field_objects']['field_margin_m'] == pytest.approx(0.5)
    assert params['field_objects']['tracks']['margin_m'] == pytest.approx(0.2)
    assert params['field_objects']['tracks']['enabled'] is True
    # モードは launch が決める
    assert 'mode' not in params


def test_outside_band_config_values() -> None:
    document = yaml.safe_load((CONFIG / 'point_extraction.yaml').read_text())
    ceiling = document['/**']['ros__parameters']['point_extraction']['ceiling']
    # 柵の外（フィールドの範囲から 1.0m より外）は、床から 0.1〜3.0m を除く（決め打ちの仮の値）
    assert ceiling['outside']['enabled'] is True
    assert ceiling['outside']['field_margin_m'] == pytest.approx(1.0)
    assert ceiling['outside']['band_max_m'] == pytest.approx(3.0)
    assert ceiling['outside']['band_max_m'] >= ceiling['band_max_m']


@pytest.mark.parametrize(
    ('text', 'message'),
    [
        ('regions:\n  - {name: a, x: [0, 1], y: [0, 1]}\n', 'がありません'),
        ('regions:\n  - {name: a, x: [1, 0], y: [0, 1], z: [0, 1]}\n', '超えています'),
        ('regions:\n  - {name: a, x: [0, 1], y: [0, 1], z: [0, 1], w: 1}\n', '知らない項目'),
        ('regions:\n  - {name: a, x: [0, 1], y: [0, 1], z: [0, 1]}\n'
         '  - {name: a, x: [0, 1], y: [0, 1], z: [0, 1]}\n', '重複'),
        ('other: 1\n', 'regions'),
    ],
)
def test_region_errors(tmp_path: Path, text: str, message: str) -> None:
    path = tmp_path / 'regions.yaml'
    path.write_text(text)
    with pytest.raises(PointExtractionError, match=message):
        load_exclude_regions(path)


def test_region_parameters() -> None:
    assert region_parameters(()) == {}
    regions = load_exclude_regions(CONFIG / 'exclude_regions.yaml')
    values = region_parameters(regions)['point_extraction.ceiling.region.boxes']
    # x_min, x_max, y_min, y_max, z_min, z_max の順
    assert values == [-21.0, 11.0, 13.0, 18.9, 5.1, 7.5]
