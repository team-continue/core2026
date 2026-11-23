"""Livox の PointCloud2（livox_ros_driver2 の xfer_format 0）を、CustomMsg（xfer_format 1）の値に直す計算.

ROS に依存しない。PointCloud2 の点は x, y, z, intensity（float32）、tag, line（uint8）、timestamp（float64、
点ごとの絶対時刻 [ns]）。CustomMsg は timebase [ns] と、点ごとの offset_time（timebase からの ns、uint32）と
reflectivity（uint8）を持つ。返りのない (0,0,0) の点も、実機のドライバと同じくそのまま残す。

timestamp は float64 の ns なので、2026 年の時刻では 256ns 刻みになる。ヘッダの時刻（最初の点）より最大 256ns
早い点が出るので、offset_time は 0 で切る。
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Final

import numpy as np
from numpy.typing import NDArray

POINT_FIELDS: Final = ('x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp')
# sensor_msgs/PointField の datatype
DATATYPES: Final = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
UINT32_MAX: Final = 2**32 - 1
# Mid-360 の内蔵 IMU の位置（LiDAR の座標系、回転なし）。core_launch の URDF の livox_imu_* と同じ
MID360_IMU_OFFSET_M: Final = (0.011, 0.02329, -0.04412)


class LivoxBagError(ValueError):
    pass


@dataclass(frozen=True)
class FieldSpec:
    name: str
    offset: int
    datatype: int


@dataclass(frozen=True)
class CustomPoints:
    """CustomMsg の中身（点は時刻の順のまま）."""

    timebase_ns: int
    offset_ns: NDArray[np.uint32]
    xyz: NDArray[np.float32]
    reflectivity: NDArray[np.uint8]
    tag: NDArray[np.uint8]
    line: NDArray[np.uint8]


def point_dtype(fields: Sequence[FieldSpec], point_step: int, big_endian: bool) -> np.dtype:
    """PointCloud2 の fields と point_step から、1 点の numpy の型を作る."""
    unknown = [f.name for f in fields if f.datatype not in DATATYPES]
    if unknown:
        raise LivoxBagError(f'扱えない datatype のフィールドがあります: {unknown}')
    order = '>' if big_endian else '<'
    return np.dtype({
        'names': [f.name for f in fields],
        'formats': [order + DATATYPES[f.datatype] for f in fields],
        'offsets': [f.offset for f in fields],
        'itemsize': point_step,
    })


def to_custom_points(points: np.ndarray, header_stamp_ns: int) -> CustomPoints:
    """PointCloud2 の点（point_dtype の配列）を、ヘッダの時刻を timebase にした CustomMsg の値にする."""
    names = points.dtype.names or ()
    missing = [n for n in POINT_FIELDS if n not in names]
    if missing:
        raise LivoxBagError(f'Livox の PointCloud2（xfer_format 0）のフィールドがありません: {missing}')
    stamps = np.round(points['timestamp'].astype(np.float64)).astype(np.int64)
    offsets = np.clip(stamps - header_stamp_ns, 0, UINT32_MAX).astype(np.uint32)
    reflectivity = np.clip(np.round(points['intensity'].astype(np.float64)), 0, 255).astype(np.uint8)
    xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float32)
    return CustomPoints(
        timebase_ns=int(header_stamp_ns), offset_ns=offsets, xyz=xyz, reflectivity=reflectivity,
        tag=points['tag'].astype(np.uint8), line=points['line'].astype(np.uint8),
    )


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """roll・pitch・yaw [rad]（URDF と同じ固定軸 x→y→z の順）から (x, y, z, w) の四元数."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
