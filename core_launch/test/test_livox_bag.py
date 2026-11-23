"""core_nav_check.livox_bag の確認（PointCloud2 の Livox の点を CustomMsg の値にする）."""

import math

from core_nav_check.livox_bag import (
    FieldSpec,
    LivoxBagError,
    point_dtype,
    quaternion_from_rpy,
    to_custom_points,
)
import numpy as np
import pytest

# livox_ros_driver2 の xfer_format 0（PointXYZRTLT）の並び
FIELDS = [
    FieldSpec('x', 0, 7), FieldSpec('y', 4, 7), FieldSpec('z', 8, 7), FieldSpec('intensity', 12, 7),
    FieldSpec('tag', 16, 2), FieldSpec('line', 17, 2), FieldSpec('timestamp', 18, 8),
]
HEADER_NS = 1_773_144_477_110_876_416


def cloud(rows: list[tuple[float, float, float, float, int, int, float]]) -> np.ndarray:
    points = np.zeros(len(rows), dtype=point_dtype(FIELDS, 26, big_endian=False))
    for i, row in enumerate(rows):
        points[i] = row
    return points


def test_offsets_are_from_header_and_clipped_at_zero() -> None:
    points = cloud([
        (1.0, 2.0, 3.0, 12.0, 0, 1, float(HEADER_NS - 256)),        # 丸めでヘッダより 256ns 早い
        (0.0, 0.0, 0.0, 0.0, 0, 2, float(HEADER_NS + 50_000_000)),  # 返りのない点（そのまま残す）
        (4.0, 5.0, 6.0, 300.0, 4, 3, float(HEADER_NS + 99_000_000)),
    ])
    custom = to_custom_points(points, HEADER_NS)
    assert custom.timebase_ns == HEADER_NS
    assert custom.offset_ns.tolist()[0] == 0
    # float64 の ns は 256ns 刻みなので、その範囲で合えばよい
    assert abs(int(custom.offset_ns[1]) - 50_000_000) <= 256
    assert abs(int(custom.offset_ns[2]) - 99_000_000) <= 256
    assert custom.xyz[1].tolist() == [0.0, 0.0, 0.0]
    assert custom.reflectivity.tolist() == [12, 0, 255]
    assert custom.tag.tolist() == [0, 0, 4]
    assert custom.line.tolist() == [1, 2, 3]


def test_rejects_cloud_without_livox_fields() -> None:
    plain = np.zeros(2, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')]))
    with pytest.raises(LivoxBagError):
        to_custom_points(plain, HEADER_NS)


def test_quaternion_for_upside_down_mount() -> None:
    x, y, z, w = quaternion_from_rpy(math.pi, 0.0, 0.0)
    assert x == pytest.approx(1.0)
    assert (y, z, w) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    # yaw だけなら z 軸まわり
    assert quaternion_from_rpy(0.0, 0.0, math.pi / 2.0) == pytest.approx(
        (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)))
