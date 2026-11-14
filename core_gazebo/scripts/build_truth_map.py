#!/usr/bin/env python3
"""真値の姿勢で点群を積算して、地図照合に使う PCD を作る.

sim.launch.py を起動した状態で実行する。機体をフィールド内で走らせ、/sim/points を
真値の姿勢で世界座標に重ねる。動く物体がない状態で作ること。
実機ではこの方法は使えないので、GLIM などで作った地図に置き換える。
"""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time
from typing import Final

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

FloatArray = NDArray[np.float64]

# 全域を見るために走る経路。柵に当たらないよう内側を回る
ROUTE: Final = (
    (-10.0, -5.5), (10.0, -5.5), (10.0, 5.5), (-10.0, 5.5), (-10.0, 0.0), (10.0, 0.0), (0.0, 0.0),
)
SPEED_MPS: Final = 2.0
BRAKE_ACCEL_MPS2: Final = 4.0
# 機体に当たった点を除く範囲（機体座標系）
BODY_HALF_M: Final = 0.45
# 自機の除去の箱の上面（機体の上面 0.8m と余裕 0.05m。footprint.yaml と同じ）
BODY_TOP_M: Final = 0.85
MIN_RANGE_M: Final = 0.1
MAX_RANGE_M: Final = 40.0
WALL_TIMEOUT_S: Final = 180.0


@dataclass(frozen=True)
class Pose:
    t: float
    x: float
    y: float
    z: float
    yaw: float


class Collector(Node):
    def __init__(self, lidar_x: float, lidar_height: float, frame_stride: int) -> None:
        super().__init__('build_truth_map')
        self._lidar_x = lidar_x
        self._lidar_height = lidar_height
        self._frame_stride = frame_stride
        self._frame_count = 0
        self.truth: list[Pose] = []
        self.chunks: list[NDArray[np.float32]] = []
        self.collecting = False
        self._pending: list[PointCloud2] = []
        self._cmd = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.create_subscription(
            Odometry, '/ground_truth/odom', self._on_truth,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            PointCloud2, '/sensing/sim/points', self._on_cloud,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_truth(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.truth.append(Pose(
            t=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            x=p.x, y=p.y, z=p.z,
            yaw=math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
        ))

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self.collecting:
            return
        self._frame_count += 1
        if self._frame_count % self._frame_stride == 0:
            self._pending.append(msg)

    def process_pending(self) -> None:
        """受信した点群を、真値が揃ったものから世界座標に直す."""
        if not self._pending or not self.truth:
            return
        times = np.array([p.t for p in self.truth[-2000:]])
        recent = self.truth[-2000:]
        remaining: list[PointCloud2] = []
        for cloud in self._pending:
            t = float(cloud.header.stamp.sec) + float(cloud.header.stamp.nanosec) * 1e-9
            if t > times[-1]:
                remaining.append(cloud)
                continue
            index = int(np.clip(np.searchsorted(times, t), 0, len(recent) - 1))
            self.chunks.append(self._to_world(cloud, recent[index]))
        self._pending = remaining

    def _to_world(self, cloud: PointCloud2, pose: Pose) -> NDArray[np.float32]:
        raw = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=False)
        points = np.asarray(raw, dtype=np.float64)
        ranges = np.linalg.norm(points, axis=1)
        valid = np.isfinite(ranges) & (ranges >= MIN_RANGE_M) & (ranges <= MAX_RANGE_M)
        base = points[valid] + np.array([self._lidar_x, 0.0, self._lidar_height])
        # 機体の上面までの箱の中を自機として除く（LiDAR は機体の前の端の低い所にある）
        on_body = (
            (np.abs(base[:, 0]) <= BODY_HALF_M)
            & (np.abs(base[:, 1]) <= BODY_HALF_M)
            & (base[:, 2] <= BODY_TOP_M)
        )
        base = base[~on_body]
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        world = np.column_stack((
            c * base[:, 0] - s * base[:, 1] + pose.x,
            s * base[:, 0] + c * base[:, 1] + pose.y,
            base[:, 2] + pose.z,
        ))
        return world.astype(np.float32)

    def command_world(self, vx: float, vy: float) -> None:
        yaw = self.truth[-1].yaw
        msg = Twist()
        msg.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        msg.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        self._cmd.publish(msg)

    def drive_route(self) -> None:
        for x, y in ROUTE:
            wall_start = time.monotonic()
            while True:
                now = self.truth[-1]
                dx, dy = x - now.x, y - now.y
                distance = math.hypot(dx, dy)
                if distance < 0.05:
                    break
                if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                    raise RuntimeError(f'目標 ({x}, {y}) に到達できません')
                v = min(SPEED_MPS, math.sqrt(2.0 * BRAKE_ACCEL_MPS2 * distance))
                self.command_world(v * dx / distance, v * dy / distance)
                self.process_pending()
                time.sleep(0.02)
        self.command_world(0.0, 0.0)


def voxel_average(points: NDArray[np.float32], leaf: float) -> NDArray[np.float32]:
    """ボクセルごとに平均を取り、距離ノイズを減らす."""
    indices = np.floor(points.astype(np.float64) / leaf).astype(np.int64)
    _, inverse = np.unique(indices, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    count = np.bincount(inverse).astype(np.float64)
    averaged = np.column_stack([
        np.bincount(inverse, weights=points[:, axis].astype(np.float64)) / count
        for axis in range(3)
    ])
    return averaged.astype(np.float32)


def save_pcd(path: Path, points: NDArray[np.float32]) -> None:
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\n'
        'DATA binary\n'
    )
    data = np.column_stack((points, np.zeros((len(points), 1), dtype=np.float32)))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('wb') as file:
        file.write(header.encode('ascii'))
        file.write(data.astype(np.float32).tobytes())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='書き出す PCD')
    # sim.launch.py の lidar_x・lidar_height と同じ値にする（既定は機体の前の端、床から 0.25m）
    parser.add_argument('--lidar-x', type=float, default=0.44)
    parser.add_argument('--lidar-height', type=float, default=0.25)
    parser.add_argument('--leaf', type=float, default=0.2, help='地図のボクセルの大きさ [m]')
    parser.add_argument('--frame-stride', type=int, default=5, help='N フレームに 1 回積算する')
    args = parser.parse_args()

    rclpy.init()
    node = Collector(args.lidar_x, args.lidar_height, args.frame_stride)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        wall_start = time.monotonic()
        while not node.truth:
            if time.monotonic() - wall_start > 60.0:
                raise RuntimeError('60秒待っても /ground_truth/odom を受信できません')
            time.sleep(0.1)
        node.collecting = True
        node.drive_route()
        time.sleep(1.0)
        node.process_pending()
        if not node.chunks:
            raise RuntimeError('点群を 1 フレームも積算できませんでした')
        all_points = np.concatenate(node.chunks)
        print(f'積算したフレーム {len(node.chunks)}、点数 {len(all_points)}', flush=True)
        averaged = voxel_average(all_points, args.leaf)
        save_pcd(args.output, averaged)
        print(f'{args.output} に {len(averaged)} 点を書き出しました', flush=True)
        low = averaged.min(axis=0)
        high = averaged.max(axis=0)
        print(f'範囲: x {low[0]:.2f}〜{high[0]:.2f}, y {low[1]:.2f}〜{high[1]:.2f}, z {low[2]:.2f}〜{high[2]:.2f}')
        return 0
    except RuntimeError as error:
        print(f'地図を作れませんでした: {error}', file=sys.stderr)
        return 1
    finally:
        if rclpy.ok() and node.truth:
            node.command_world(0.0, 0.0)
            time.sleep(0.1)
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
