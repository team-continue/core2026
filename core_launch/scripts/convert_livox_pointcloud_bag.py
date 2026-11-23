#!/usr/bin/env python3
"""Livox の PointCloud2（livox_ros_driver2 の xfer_format 0）で録った bag を、CustomMsg（xfer_format 1）の bag にする.

FAST-LIO と検出は CustomMsg を読むので、xfer_format 0 で録った実機の bag は、このまま再生できない。
変えたあとは replay_localization.py で再生できる。

- /livox/lidar の PointCloud2 を CustomMsg にする。点ごとの時刻と返りのない (0,0,0) の点はそのまま残す
- --copy のトピック（IMU など）はそのまま写す。/tf（録ったときの FAST-LIO などの出力）は写さない
- /tf_static は 1 つにまとめる。map・odom を親にした変換（録ったときの自己位置の出力）を除き、Mid-360 の内蔵 IMU の
  フレーム（livox_frame→livox_imu_link）がなければ足す。--mount で base_link→livox_frame を置き換えられる
  （録ったときの TF が機体の取り付けと違うとき。2026-03 の auto4_pcl は TF が 0.5m、床の面から測ると 0.533m）

使い方: convert_livox_pointcloud_bag.py <入力の bag> <出力の bag（新しいディレクトリ）>
        [--mount X Y Z ROLL PITCH YAW] [--copy TOPIC ...]
"""

import argparse
from pathlib import Path
import sys
from typing import Final

from geometry_msgs.msg import TransformStamped
from core_nav_check.livox_bag import (
    FieldSpec,
    LivoxBagError,
    MID360_IMU_OFFSET_M,
    point_dtype,
    quaternion_from_rpy,
    to_custom_points,
)
from livox_ros_driver2.msg import CustomMsg, CustomPoint
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

LIDAR_TOPIC: Final = '/sensing/livox/lidar'
DEFAULT_COPY: Final = ('/sensing/livox/imu', '/sensing/imu', '/control/cmd_vel')
BASE_FRAME: Final = 'base_link'
LIDAR_FRAME: Final = 'livox_frame'
IMU_FRAME: Final = 'livox_imu_link'
# 録ったときの自己位置の出力。再生で動かす自己位置と重なるので写さない
DROPPED_STATIC_PARENTS: Final = ('map', 'odom')


def to_message(cloud: object) -> CustomMsg:
    fields = [FieldSpec(f.name, f.offset, f.datatype) for f in cloud.fields]
    dtype = point_dtype(fields, cloud.point_step, cloud.is_bigendian)
    points = np.frombuffer(bytes(cloud.data), dtype=dtype, count=cloud.width * cloud.height)
    stamp_ns = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
    custom = to_custom_points(points, stamp_ns)
    message = CustomMsg()
    message.header = cloud.header
    message.timebase = custom.timebase_ns
    message.lidar_id = 0
    converted = []
    for i in range(len(custom.offset_ns)):
        point = CustomPoint()
        point.offset_time = int(custom.offset_ns[i])
        point.x = float(custom.xyz[i, 0])
        point.y = float(custom.xyz[i, 1])
        point.z = float(custom.xyz[i, 2])
        point.reflectivity = int(custom.reflectivity[i])
        point.tag = int(custom.tag[i])
        point.line = int(custom.line[i])
        converted.append(point)
    message.points = converted
    message.point_num = len(converted)
    return message


def static_transform(parent: str, child: str, xyz: tuple[float, ...], rpy: tuple[float, ...]) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
    q = quaternion_from_rpy(*rpy)
    rotation = transform.transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = q
    return transform


def merge_static(
    recorded: list[TransformStamped], mount: tuple[float, ...] | None, stamp: object,
) -> tuple[TFMessage, list[str]]:
    """録った /tf_static を 1 つにまとめ、取り付けと内蔵 IMU のフレームを整える。変えた所の説明も返す."""
    notes = []
    by_child: dict[str, TransformStamped] = {}
    for transform in recorded:
        if transform.header.frame_id in DROPPED_STATIC_PARENTS:
            notes.append(f'除いた: {transform.header.frame_id}→{transform.child_frame_id}')
            continue
        by_child[transform.child_frame_id] = transform
    if mount is not None:
        by_child[LIDAR_FRAME] = static_transform(BASE_FRAME, LIDAR_FRAME, mount[:3], mount[3:])
        notes.append(f'置き換えた: {BASE_FRAME}→{LIDAR_FRAME} 位置 {mount[:3]} roll・pitch・yaw {mount[3:]}')
    if LIDAR_FRAME not in by_child:
        raise LivoxBagError(f'{BASE_FRAME}→{LIDAR_FRAME} の TF がありません。--mount で与えてください')
    if IMU_FRAME not in by_child:
        by_child[IMU_FRAME] = static_transform(LIDAR_FRAME, IMU_FRAME, MID360_IMU_OFFSET_M, (0.0, 0.0, 0.0))
        notes.append(f'足した: {LIDAR_FRAME}→{IMU_FRAME}（Mid-360 の内蔵 IMU）')
    message = TFMessage()
    for transform in by_child.values():
        transform.header.stamp = stamp
        message.transforms.append(transform)
    return message, notes


def convert(source: Path, target: Path, mount: tuple[float, ...] | None, copy: tuple[str, ...]) -> int:
    if target.exists():
        raise LivoxBagError(f'出力のディレクトリがすでにあります: {target}')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if types.get(LIDAR_TOPIC) != 'sensor_msgs/msg/PointCloud2':
        raise LivoxBagError(f'{LIDAR_TOPIC} が PointCloud2 ではありません（{types.get(LIDAR_TOPIC)}）')
    copied = [topic for topic in copy if topic in types]
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(target), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = [(LIDAR_TOPIC, 'livox_ros_driver2/msg/CustomMsg'), ('/tf_static', 'tf2_msgs/msg/TFMessage')]
    topics += [(topic, types[topic]) for topic in copied]
    for topic_id, (name, kind) in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(id=topic_id, name=name, type=kind, serialization_format='cdr'))

    cloud_type = get_message(types[LIDAR_TOPIC])
    recorded_static: list[TransformStamped] = []
    first_ns: int | None = None
    first_stamp = None
    clouds = 0
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if first_ns is None:
            first_ns = stamp_ns
        if topic == LIDAR_TOPIC:
            cloud = deserialize_message(data, cloud_type)
            if first_stamp is None:
                first_stamp = cloud.header.stamp
            writer.write(LIDAR_TOPIC, serialize_message(to_message(cloud)), stamp_ns)
            clouds += 1
        elif topic == '/tf_static':
            recorded_static.extend(deserialize_message(data, TFMessage).transforms)
        elif topic in copied:
            writer.write(topic, data, stamp_ns)
    if clouds == 0 or first_ns is None or first_stamp is None:
        raise LivoxBagError(f'{LIDAR_TOPIC} の点群がありません')
    static, notes = merge_static(recorded_static, mount, first_stamp)
    writer.write('/tf_static', serialize_message(static), first_ns)
    del writer
    print(f'点群 {clouds} 回を CustomMsg にしました。写したトピック: {", ".join(copied) or "なし"}')
    for note in notes:
        print(f'/tf_static を {note}')
    print(f'出力: {target}')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('source', type=Path, help='入力の bag のディレクトリ')
    parser.add_argument('target', type=Path, help='出力の bag のディレクトリ（まだないもの）')
    parser.add_argument('--mount', type=float, nargs=6, metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                        help='base_link→livox_frame を置き換える取り付け [m, rad]')
    parser.add_argument('--copy', nargs='*', default=list(DEFAULT_COPY), help='そのまま写すトピック（あるものだけ）')
    args = parser.parse_args()
    try:
        return convert(args.source, args.target, tuple(args.mount) if args.mount else None, tuple(args.copy))
    except LivoxBagError as error:
        print(f'[エラー] {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
