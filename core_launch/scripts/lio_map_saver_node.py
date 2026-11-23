#!/usr/bin/env python3
"""FAST-LIO の点群を貯めて、地図照合に使う地図（PCD）を作る（mapping.launch.py が起動する）.

FAST-LIO の歪みを補正した点群（/localization/cloud_registered、camera_init の座標）を、lio_odom_adapter の出す
odom の座標（地図を作り始めたときの base_link。床が z=0、x が機体の前）に直して、ボクセルごとに 1 点へまとめる。
origin_x・origin_y・origin_yaw で、作り始めの機体を地図のどこに置くかを決められる（既定は原点）。

- 保存: ~/save（std_srvs/Trigger）を呼ぶか、ノードを止めたとき（save_on_shutdown）に output へ書く
- 確認: 貯めた地図を ~/map（PointCloud2、map_frame）に出す。RViz で壁が二重になっていないかを見る
- 少しの間だけ見えた点（通り過ぎた人など）は、min_hits フレーム以上で見たボクセルだけを残して除く
"""

from __future__ import annotations

from pathlib import Path
import sys

from core_nav_map.map_files import write_pcd
from core_nav_map.voxel_map import (
    add_frames,
    apply_transform,
    empty_map,
    floor_height,
    floor_plane,
    map_points,
    planar_pose_matrix,
    transform_matrix,
    VoxelMap,
    within_range,
)
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import tf2_ros


# 床の傾きがこれより大きければ、作り始めに機体が水平でなかった（10m 先で約 5cm）
TILT_WARN_DEG = 0.3


class MapSaverError(Exception):
    """地図を保存できない."""


class LioMapSaverNode(Node):
    def __init__(self) -> None:
        super().__init__('map_saver')
        self._output = Path(str(self.declare_parameter('output', '').value)).expanduser()
        if not str(self.get_parameter('output').value):
            raise MapSaverError('output（保存する PCD の場所）を指定してください')
        if self._output.suffix != '.pcd':
            raise MapSaverError(f'output は .pcd にしてください: {self._output}')
        self._map_frame = str(self.declare_parameter('map_frame', 'odom').value)
        voxel_size_m = float(self.declare_parameter('voxel_size_m', 0.1).value)
        self._min_hits = int(self.declare_parameter('min_hits', 3).value)
        self._max_range_m = float(self.declare_parameter('max_range_m', 30.0).value)
        merge_period_s = float(self.declare_parameter('merge_period_s', 1.0).value)
        publish_period_s = float(self.declare_parameter('publish_period_s', 2.0).value)
        self._save_on_shutdown = bool(self.declare_parameter('save_on_shutdown', True).value)
        self._origin = planar_pose_matrix(
            float(self.declare_parameter('origin_x', 0.0).value),
            float(self.declare_parameter('origin_y', 0.0).value),
            float(self.declare_parameter('origin_yaw', 0.0).value),
        )
        input_topic = str(self.declare_parameter('input_topic', '/localization/cloud_registered').value)
        odometry_topic = str(self.declare_parameter('odometry_topic', '/localization/Odometry').value)

        self._map: VoxelMap = empty_map(voxel_size_m)
        self._pending: list[NDArray[np.float64]] = []
        self._frames = 0
        self._sensor_origin: NDArray[np.float64] | None = None
        # camera_init → map_frame（odom）。lio_odom_adapter が出す固定の変換なので、一度取れたら使い続ける
        self._to_map: NDArray[np.float64] | None = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, input_topic, self._on_cloud, reliable)
        self.create_subscription(Odometry, odometry_topic, self._on_odometry, reliable)
        self._map_publisher = self.create_publisher(PointCloud2, '~/map', 1)
        self.create_service(Trigger, '~/save', self._on_save)
        self.create_timer(merge_period_s, self._merge)
        self.create_timer(publish_period_s, self._publish_map)
        self.get_logger().info(
            f'{input_topic} を {self._map_frame} の座標で貯めます（ボクセル {voxel_size_m}m、{self._min_hits} フレーム以上、'
            f'{self._max_range_m}m 以内）。保存先: {self._output}')

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._sensor_origin = np.array([position.x, position.y, position.z])

    def _lookup_to_map(self, cloud_frame: str) -> NDArray[np.float64] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, cloud_frame, Time(), timeout=Duration(seconds=0.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            self.get_logger().warning(
                f'{cloud_frame} → {self._map_frame} の変換を待っています（lio_odom_adapter が出す）: {error}',
                throttle_duration_sec=5.0)
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return transform_matrix(
            (translation.x, translation.y, translation.z), (rotation.x, rotation.y, rotation.z, rotation.w))

    def _on_cloud(self, message: PointCloud2) -> None:
        if self._sensor_origin is None:
            return
        if self._to_map is None:
            self._to_map = self._lookup_to_map(message.header.frame_id)
            if self._to_map is None:
                return
        # read_points は Humble でも Jazzy でも numpy の構造化配列を返す（read_points_numpy は版によってない）
        records = point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.stack([np.asarray(records[axis], dtype=np.float64) for axis in ('x', 'y', 'z')], axis=1)
        points = within_range(points, self._sensor_origin, self._max_range_m)
        self._pending.append(apply_transform(self._origin @ self._to_map, points))
        self._frames += 1

    def _merge(self) -> None:
        if not self._pending:
            return
        pending, self._pending = self._pending, []
        self._map = add_frames(self._map, pending)
        self.get_logger().info(
            f'{self._frames} フレーム、{self._map.size} ボクセル', throttle_duration_sec=10.0)

    def _publish_map(self) -> None:
        if self._map_publisher.get_subscription_count() == 0 or self._map.size == 0:
            return
        header = Header(frame_id=self._map_frame, stamp=self.get_clock().now().to_msg())
        points = map_points(self._map, self._min_hits).astype(np.float32)
        self._map_publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

    def save(self) -> str:
        """貯めた地図を書き、結果の文を返す."""
        self._merge()
        points = map_points(self._map, self._min_hits)
        if points.shape[0] == 0:
            raise MapSaverError('保存する点がありません（FAST-LIO の点群が届いているか確かめてください）')
        write_pcd(points, self._output)
        floor = floor_height(points)
        floor_text = '床の点が見つかりません' if floor is None else f'床の高さの目安 {floor:+.2f}m（0 に近いはず）'
        plane = floor_plane(points)
        tilt_text = '' if plane is None else f'、床の傾き {plane.tilt_deg:.2f}°（{TILT_WARN_DEG}° 以下のはず）'
        return f'{self._output} に {points.shape[0]} 点を保存しました（{self._frames} フレーム）。{floor_text}{tilt_text}'

    def _on_save(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            response.message = self.save()
            response.success = True
            self.get_logger().info(response.message)
        except (MapSaverError, OSError) as error:
            response.success = False
            response.message = f'保存できませんでした: {error}'
            self.get_logger().error(response.message)
        return response

    def save_on_shutdown(self) -> None:
        if not self._save_on_shutdown:
            return
        try:
            # 止めるときはロガーが使えないことがあるので、標準出力に書く
            print(f'[map_saver] {self.save()}', flush=True)
        except (MapSaverError, OSError) as error:
            print(f'[map_saver] 止めるときに保存できませんでした: {error}', file=sys.stderr, flush=True)


def main() -> int:
    rclpy.init()
    try:
        node = LioMapSaverNode()
    except MapSaverError as error:
        print(f'[map_saver] {error}', file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_on_shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
