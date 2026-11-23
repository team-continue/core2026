import math
from pathlib import Path

from core_nav_launch.custom_map import (
    available_custom_maps,
    custom_map_files,
    custom_map_preset,
    CustomMapError,
)
from core_nav_launch.livox_driver import LIVOX_PARAMETERS, LIVOX_REMAPPINGS
from core_nav_launch.occupancy_map import FREE, load_map_yaml, load_occupancy_map, OCCUPIED, UNKNOWN
from core_nav_map.map_files import (
    free_bounds,
    MapFileError,
    occupancy_image,
    PIXEL_FREE,
    PIXEL_OCCUPIED,
    PIXEL_UNKNOWN,
    read_pcd,
    write_field_config,
    write_occupancy,
    write_pcd,
)
from core_nav_map.voxel_map import (
    add_frames,
    apply_transform,
    empty_map,
    floor_height,
    floor_plane,
    map_points,
    planar_pose_matrix,
    transform_matrix,
    voxel_keys,
    within_range,
)
import numpy as np
import pytest
import yaml


# ---------------------------------------------------------------- 点を貯める

def test_points_in_one_voxel_merge_to_their_mean() -> None:
    voxel_map = add_frames(empty_map(0.1), [np.array([[0.01, 0.02, 0.03], [0.05, 0.06, 0.07]])])
    assert voxel_map.size == 1
    np.testing.assert_allclose(map_points(voxel_map, 1), [[0.03, 0.04, 0.05]])


def test_hits_count_frames_not_points() -> None:
    # 1 フレームに同じボクセルの点が 3 つあっても、見たフレームは 1
    frame = np.array([[0.01, 0.0, 0.0], [0.02, 0.0, 0.0], [0.03, 0.0, 0.0]])
    voxel_map = add_frames(empty_map(0.1), [frame])
    assert voxel_map.hits.tolist() == [1]
    assert voxel_map.point_counts.tolist() == [3]
    voxel_map = add_frames(voxel_map, [frame, frame])
    assert voxel_map.hits.tolist() == [3]


def test_min_hits_drops_briefly_seen_points() -> None:
    wall = np.array([[1.0, 0.0, 0.5]])
    person = np.array([[2.0, 0.0, 1.0]])
    voxel_map = add_frames(empty_map(0.1), [np.concatenate([wall, person]), wall, wall])
    np.testing.assert_allclose(map_points(voxel_map, 3), wall)
    assert map_points(voxel_map, 1).shape == (2, 3)


def test_add_frames_does_not_change_the_original_map() -> None:
    original = add_frames(empty_map(0.1), [np.array([[0.0, 0.0, 0.0]])])
    add_frames(original, [np.array([[5.0, 5.0, 5.0]])])
    assert original.size == 1


def test_negative_coordinates_get_their_own_voxels() -> None:
    keys = voxel_keys(np.array([[-0.05, 0.0, 0.0], [0.05, 0.0, 0.0]]), 0.1)
    assert keys[0] != keys[1]


def test_nan_points_are_ignored() -> None:
    voxel_map = add_frames(empty_map(0.1), [np.array([[np.nan, 0.0, 0.0], [1.0, 1.0, 1.0]])])
    assert voxel_map.size == 1


def test_invalid_inputs_raise() -> None:
    with pytest.raises(ValueError):
        empty_map(0.0)
    with pytest.raises(ValueError):
        add_frames(empty_map(0.1), [np.zeros((3, 2))])
    with pytest.raises(ValueError):
        map_points(empty_map(0.1), 0)
    with pytest.raises(ValueError):
        voxel_keys(np.array([[1.0e6, 0.0, 0.0]]), 0.1)


def test_within_range_uses_the_sensor_position() -> None:
    points = np.array([[1.0, 0.0, 0.0], [12.0, 0.0, 0.0]])
    np.testing.assert_allclose(within_range(points, np.array([10.0, 0.0, 0.0]), 5.0), [[12.0, 0.0, 0.0]])


def test_transforms_match_rotation_and_translation() -> None:
    point = np.array([[1.0, 0.0, 0.0]])
    # z 軸まわりに 90 度、x に 2m
    planar = planar_pose_matrix(2.0, 0.0, math.pi / 2)
    np.testing.assert_allclose(apply_transform(planar, point), [[2.0, 1.0, 0.0]], atol=1e-12)
    quaternion = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    np.testing.assert_allclose(transform_matrix((2.0, 0.0, 0.0), quaternion), planar, atol=1e-12)
    with pytest.raises(ValueError):
        transform_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))


def test_floor_height_finds_the_densest_low_layer() -> None:
    rng = np.random.default_rng(0)
    floor = np.column_stack([rng.uniform(-5, 5, 2000), rng.uniform(-5, 5, 2000), rng.normal(0.03, 0.005, 2000)])
    wall = np.column_stack([np.full(500, 5.0), rng.uniform(-5, 5, 500), rng.uniform(0.0, 2.0, 500)])
    height = floor_height(np.concatenate([floor, wall]))
    assert height is not None and abs(height - 0.03) <= 0.02
    assert floor_height(np.array([[0.0, 0.0, 3.0]])) is None



def test_floor_plane_reports_the_tilt() -> None:
    rng = np.random.default_rng(1)
    xy = rng.uniform(-10, 10, (5000, 2))
    level = np.column_stack([xy, rng.normal(0.0, 0.01, 5000)])
    plane = floor_plane(level)
    assert plane is not None and plane.tilt_deg < 0.1 and abs(plane.height_at_origin_m) < 0.01
    # 0.5 度傾いた床（x 方向）と、床から外れた机の点
    tilted = np.column_stack([xy, xy[:, 0] * math.tan(math.radians(0.5))])
    desk = np.column_stack([rng.uniform(0, 1, (300, 2)), np.full(300, 0.1)])
    plane = floor_plane(np.concatenate([tilted, desk]))
    assert plane is not None and abs(plane.tilt_deg - 0.5) < 0.05
    assert floor_plane(np.array([[0.0, 0.0, 0.0]])) is None


# ---------------------------------------------------------------- PCD

def test_pcd_round_trip(tmp_path: Path) -> None:
    points = np.array([[1.0, 2.0, 3.0], [-4.5, 0.25, 0.0]])
    path = tmp_path / 'map.pcd'
    write_pcd(points, path)
    np.testing.assert_allclose(read_pcd(path), points)
    assert not (tmp_path / 'map.pcd.tmp').exists()


def test_read_pcd_with_extra_fields_and_ascii(tmp_path: Path) -> None:
    # FAST-LIO や GLIM の PCD は intensity などの欄も持つ
    binary = tmp_path / 'binary.pcd'
    records = np.array([(1.0, 2.0, 3.0, 7.0, 9)], dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                                          ('intensity', '<f4'), ('ring', '<u2')])
    header = ('VERSION 0.7\nFIELDS x y z intensity ring\nSIZE 4 4 4 4 2\nTYPE F F F F U\nCOUNT 1 1 1 1 1\n'
              'WIDTH 1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA binary\n')
    binary.write_bytes(header.encode('ascii') + records.tobytes())
    np.testing.assert_allclose(read_pcd(binary), [[1.0, 2.0, 3.0]])
    ascii_pcd = tmp_path / 'ascii.pcd'
    ascii_pcd.write_text(
        'VERSION 0.7\nFIELDS intensity x y z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n5 1 2 3\n6 4 5 6\n', encoding='ascii')
    np.testing.assert_allclose(read_pcd(ascii_pcd), [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])


def test_read_pcd_rejects_compressed_and_missing_xyz(tmp_path: Path) -> None:
    compressed = tmp_path / 'compressed.pcd'
    compressed.write_text('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 0\nDATA binary_compressed\n', encoding='ascii')
    with pytest.raises(MapFileError, match='binary_compressed'):
        read_pcd(compressed)
    no_xyz = tmp_path / 'no_xyz.pcd'
    no_xyz.write_text('FIELDS a b\nSIZE 4 4\nTYPE F F\nPOINTS 0\nDATA ascii\n', encoding='ascii')
    with pytest.raises(MapFileError, match='x y z'):
        read_pcd(no_xyz)
    with pytest.raises(MapFileError):
        read_pcd(tmp_path / 'missing.pcd')


# ---------------------------------------------------------------- 占有地図とフィールドの範囲

def room_points() -> np.ndarray:
    """x 0〜4m・y 0〜2m の床と、x=4m の壁（高さ 0〜2m）."""
    xs, ys = np.meshgrid(np.arange(0.0, 4.0, 0.05), np.arange(0.0, 2.0, 0.05))
    floor = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    wy, wz = np.meshgrid(np.arange(0.0, 2.0, 0.05), np.arange(0.0, 2.0, 0.05))
    wall = np.column_stack([np.full(wy.size, 4.02), wy.ravel(), wz.ravel()])
    ceiling = np.array([[1.0, 1.0, 2.8]])
    return np.concatenate([floor, wall, ceiling])


def test_occupancy_marks_walls_floor_and_unknown() -> None:
    image = occupancy_image(room_points(), 0.1, 0.15, 1.5, 0.08, 1.0)
    height = image.pixels.shape[0]

    def pixel(x: float, y: float) -> int:
        column = int((x - image.origin_x) / image.resolution)
        row = height - 1 - int((y - image.origin_y) / image.resolution)
        return int(image.pixels[row, column])

    assert pixel(1.0, 1.0) == PIXEL_FREE  # 床（天井の点は高さの上限より上なので占有にしない）
    assert pixel(4.02, 1.0) == PIXEL_OCCUPIED  # 壁
    assert pixel(-0.5, 1.0) == PIXEL_UNKNOWN  # 見ていない所
    bounds = free_bounds(image)
    assert bounds.x_min <= 0.0 and 3.9 <= bounds.x_max <= 4.1
    assert bounds.y_min <= 0.0 and 1.9 <= bounds.y_max <= 2.1


def test_free_fill_closes_gaps_between_sparse_floor_points() -> None:
    # 0.1m のボクセルの床の点を 0.05m のセルに描くと、点のないセルが残る
    xs, ys = np.meshgrid(np.arange(0.05, 2.0, 0.1), np.arange(0.05, 2.0, 0.1))
    floor = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    # 余白（0.5m = 10 セル）の外側を除いた、床の点の並ぶ範囲
    inner = (slice(12, -12), slice(12, -12))
    sparse = occupancy_image(floor, 0.05, 0.15, 1.5, 0.08, 0.5)
    assert np.any(sparse.pixels[inner] == PIXEL_UNKNOWN)
    filled = occupancy_image(floor, 0.05, 0.15, 1.5, 0.08, 0.5, free_fill_m=0.1)
    assert np.all(filled.pixels[inner] == PIXEL_FREE)
    with pytest.raises(MapFileError):
        occupancy_image(floor, 0.05, 0.15, 1.5, 0.08, 0.5, free_fill_m=-1.0)


def test_occupancy_needs_a_floor_at_zero() -> None:
    lifted = room_points() + np.array([0.0, 0.0, 0.5])
    with pytest.raises(MapFileError, match='床'):
        occupancy_image(lifted, 0.1, 0.15, 1.5, 0.08, 1.0)


def test_written_files_load_as_a_navigation_map(tmp_path: Path) -> None:
    # make_map_files.py が書くファイルを、navigation.launch.py と同じ読み方で読む
    name = 'lab'
    pcd, occupancy_yaml, field = custom_map_files(tmp_path, name)
    points = room_points()
    write_pcd(points, pcd)
    image = occupancy_image(points, 0.1, 0.15, 1.5, 0.08, 1.0)
    write_occupancy(image, name, tmp_path, 'lab.pcd')
    write_field_config(free_bounds(image), field, 'lab.pcd')

    assert available_custom_maps(tmp_path) == ['lab']
    preset = custom_map_preset(tmp_path, name)
    assert preset['pcd'] == str(pcd)
    assert preset['field'] == str(field)
    assert (preset['init_x'], preset['init_y'], preset['init_yaw']) == (0.0, 0.0, 0.0)
    occupancy = load_occupancy_map(load_map_yaml(occupancy_yaml))
    assert {OCCUPIED, FREE, UNKNOWN} <= set(np.unique(occupancy.cells).tolist())
    assert occupancy.origin_x == pytest.approx(preset['origin_x'])
    document = yaml.safe_load(field.read_text(encoding='utf-8'))
    bounds = document['/**']['ros__parameters']['field']
    assert set(bounds) == {'x_min', 'x_max', 'y_min', 'y_max'}


def test_custom_map_errors_name_the_missing_files(tmp_path: Path) -> None:
    write_pcd(np.zeros((1, 3)), tmp_path / 'half.pcd')
    assert available_custom_maps(tmp_path) == []
    with pytest.raises(CustomMapError, match='half.yaml'):
        custom_map_preset(tmp_path, 'half')
    with pytest.raises(CustomMapError, match='英数字'):
        custom_map_files(tmp_path, '../etc')


def test_livox_driver_sends_custom_msg_to_the_sensing_topics() -> None:
    # FAST-LIO と検出は CustomMsg（xfer_format 1）を /sensing/livox/lidar で受ける
    assert LIVOX_PARAMETERS['xfer_format'] == 1
    assert dict(LIVOX_REMAPPINGS) == {'livox/lidar': '/sensing/livox/lidar', 'livox/imu': '/sensing/livox/imu'}
