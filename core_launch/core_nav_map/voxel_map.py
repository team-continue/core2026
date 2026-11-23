"""走りながら受けた点群を、ボクセルごとに 1 点へまとめて地図にする（lio_map_saver_node.py が使う）.

同じボクセルに入った点は平均の位置にまとめ、そのボクセルを見たフレームの数（hits）を数える。地図に出すときは、
hits が min_hits 以上のボクセルだけを残す（通り過ぎた人など、少しの間だけ見えた点を除く）。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Final

import numpy as np
from numpy.typing import NDArray

# 格子の番号を各軸 21 ビットに詰める（符号つきの番号を 2^20 だけずらす）。0.1m の格子で ±100km まで
_BITS: Final = 21
_OFFSET: Final = 1 << (_BITS - 1)
_MASK: Final = (1 << _BITS) - 1

FloatPoints = NDArray[np.float64]


@dataclass(frozen=True)
class VoxelMap:
    """ボクセルごとの点の和・点の数・見たフレームの数。keys は昇順で重複しない."""

    voxel_size_m: float
    keys: NDArray[np.int64]
    sums: FloatPoints
    point_counts: NDArray[np.int64]
    hits: NDArray[np.int64]

    @property
    def size(self) -> int:
        return int(self.keys.shape[0])


def _check_points(points: NDArray[np.floating]) -> FloatPoints:
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f'点は (N, 3) の配列で渡してください: {array.shape}')
    return array[np.all(np.isfinite(array), axis=1)]


def voxel_keys(points: FloatPoints, voxel_size_m: float) -> NDArray[np.int64]:
    """点の入るボクセルの番号を 1 つの整数に詰める."""
    if not voxel_size_m > 0.0:
        raise ValueError(f'ボクセルの大きさは正の値にしてください: {voxel_size_m}')
    index = np.floor(points / voxel_size_m).astype(np.int64) + _OFFSET
    if index.size and (index.min() < 0 or index.max() > _MASK):
        raise ValueError('点がボクセルの番号の範囲（約 ±100km）の外です')
    return (index[:, 0] << (2 * _BITS)) | (index[:, 1] << _BITS) | index[:, 2]


def empty_map(voxel_size_m: float) -> VoxelMap:
    if not voxel_size_m > 0.0:
        raise ValueError(f'ボクセルの大きさは正の値にしてください: {voxel_size_m}')
    return VoxelMap(
        voxel_size_m=voxel_size_m,
        keys=np.zeros(0, dtype=np.int64),
        sums=np.zeros((0, 3), dtype=np.float64),
        point_counts=np.zeros(0, dtype=np.int64),
        hits=np.zeros(0, dtype=np.int64),
    )


def _frame_voxels(points: FloatPoints, voxel_size_m: float) -> VoxelMap:
    """1 フレームの点をボクセルにまとめる。どのボクセルも hits は 1."""
    keys = voxel_keys(points, voxel_size_m)
    unique, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
    sums = np.stack([np.bincount(inverse, weights=points[:, axis], minlength=unique.shape[0])
                     for axis in range(3)], axis=1)
    return VoxelMap(voxel_size_m, unique, sums, counts.astype(np.int64), np.ones(unique.shape[0], dtype=np.int64))


def add_frames(voxel_map: VoxelMap, frames: Sequence[NDArray[np.floating]]) -> VoxelMap:
    """フレームの点群を足した新しい地図を返す（元の地図は変えない）."""
    parts = [voxel_map] + [
        _frame_voxels(points, voxel_map.voxel_size_m)
        for points in (_check_points(frame) for frame in frames) if points.shape[0] > 0
    ]
    if len(parts) == 1:
        return voxel_map
    keys = np.concatenate([part.keys for part in parts])
    unique, inverse = np.unique(keys, return_inverse=True)
    size = unique.shape[0]
    sums_all = np.concatenate([part.sums for part in parts])
    sums = np.stack([np.bincount(inverse, weights=sums_all[:, axis], minlength=size) for axis in range(3)], axis=1)
    point_counts = np.bincount(
        inverse, weights=np.concatenate([part.point_counts for part in parts]), minlength=size).astype(np.int64)
    hits = np.bincount(inverse, weights=np.concatenate([part.hits for part in parts]), minlength=size).astype(np.int64)
    return VoxelMap(voxel_map.voxel_size_m, unique, sums, point_counts, hits)


def map_points(voxel_map: VoxelMap, min_hits: int) -> FloatPoints:
    """min_hits 以上のフレームで見たボクセルの、点の平均の位置."""
    if min_hits < 1:
        raise ValueError(f'min_hits は 1 以上にしてください: {min_hits}')
    keep = voxel_map.hits >= min_hits
    return voxel_map.sums[keep] / voxel_map.point_counts[keep, None]


def within_range(points: FloatPoints, origin: FloatPoints, max_range_m: float) -> FloatPoints:
    """センサーの位置から max_range_m 以内の点だけを残す（遠い点は位置の誤差が大きい）."""
    if not max_range_m > 0.0:
        raise ValueError(f'距離の上限は正の値にしてください: {max_range_m}')
    distance = np.linalg.norm(points - origin.reshape(1, 3), axis=1)
    return points[distance <= max_range_m]


def planar_pose_matrix(x: float, y: float, yaw: float) -> NDArray[np.float64]:
    """平面上の位置と向き（z 軸まわり）を 4x4 の変換にする."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0, x], [s, c, 0.0, y], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


def transform_matrix(translation: Sequence[float], quaternion_xyzw: Sequence[float]) -> NDArray[np.float64]:
    """平行移動と四元数（x, y, z, w）を 4x4 の変換にする."""
    qx, qy, qz, qw = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError('四元数の大きさが 0 です')
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [float(value) for value in translation]
    return matrix


def apply_transform(matrix: NDArray[np.float64], points: FloatPoints) -> FloatPoints:
    return points @ matrix[:3, :3].T + matrix[:3, 3]


@dataclass(frozen=True)
class FloorPlane:
    """床の面（z = a x + b y + c）の傾きと、原点での高さ."""

    tilt_deg: float
    height_at_origin_m: float
    points: int


def floor_plane(
    points: FloatPoints, search_m: float = 0.15, radius_m: float = 15.0, iterations: int = 3,
) -> FloorPlane | None:
    """床の面を最小二乗で当てはめる（外れた点を除きながら数回）。床の点が少なければ None.

    作り始めに機体が傾いていると、地図全体が傾き、遠くの床が静止物の高さの下限（計画は既定 0.15m）を超える。
    """
    near = points[(np.abs(points[:, 2]) <= search_m) & (np.hypot(points[:, 0], points[:, 1]) <= radius_m)]
    for _ in range(iterations):
        if near.shape[0] < 100:
            return None
        design = np.column_stack([near[:, 0], near[:, 1], np.ones(near.shape[0])])
        coefficients, *_ = np.linalg.lstsq(design, near[:, 2], rcond=None)
        residual = near[:, 2] - design @ coefficients
        near = near[np.abs(residual) <= max(0.03, 2.5 * float(np.std(residual)))]
    a, b, c = (float(value) for value in coefficients)
    return FloorPlane(math.degrees(math.atan(math.hypot(a, b))), c, int(near.shape[0]))


def floor_height(points: FloatPoints, search_m: float = 0.5, bin_m: float = 0.02) -> float | None:
    """床の高さの目安（|z| ≤ search_m の点の z の最頻値）。点がなければ None.

    地図は床を z=0 にして作るので、0 から大きくずれていれば、作り始めに機体が水平でなかったか、取り付けの値が違う。
    """
    z = points[np.abs(points[:, 2]) <= search_m, 2]
    if z.shape[0] == 0:
        return None
    bins = int(math.ceil(2 * search_m / bin_m))
    counts, edges = np.histogram(z, bins=bins, range=(-search_m, search_m))
    peak = int(np.argmax(counts))
    return float((edges[peak] + edges[peak + 1]) / 2.0)
