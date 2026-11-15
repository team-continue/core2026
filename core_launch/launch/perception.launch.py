"""生点群から他ロボットと人間を検出して追跡するノードを起動する.

rear_lidar:=true なら、検出のための 2 台目の LiDAR（別案3-1）の点群（/livox/lidar2）を lidar_merge で 1 台目
（/livox/lidar）と合わせ、検出の入力を合わせた点群（/livox/lidar_merged）にする（値は config/lidar_merge.yaml）。
LIO と地図照合は 1 台目だけを使うので変わらない。

composition:=true なら、検出（perception_node）を点群の入力と同じプロセス（コンテナ lidar_container）に入れ、
点群をコピーせずに受け取る（core_nav_launch/composition.py）。container で、点群の入力を起動した launch が
用意したコンテナを指定する。空なら、検出だけのコンテナを起動する。lidar_merge はいつも別のプロセスで動かす。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core_nav_launch.composition import (
    COMPOSITION_MODES,
    container,
    DEFAULT_COMPOSITION,
    field_overlay,
    LIDAR_CONTAINER,
    load_into,
    NodeSpec,
    parse_composition,
    PERCEPTION_NS,
    standalone_node,
)
import yaml


def merged_topic(merge_config: Path) -> str:
    """lidar_merge の出力のトピック（検出の入力）を設定ファイルから読む."""
    try:
        parameters = yaml.safe_load(merge_config.read_text())['/**']['ros__parameters']
        return str(parameters['output_topic'])
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError(f'{merge_config} から output_topic を読めません: {error}') from error


def build_nodes(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    config = Path(get_package_share_directory('core_launch')) / 'config'
    rear_lidar = LaunchConfiguration('rear_lidar').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    merge_config = config / 'lidar_merge.yaml'
    perception_overrides: dict[str, object] = {
        'use_sim_time': use_sim_time,
        'map_path': LaunchConfiguration('map_path').perform(context),
    }
    nodes: list[LaunchDescriptionEntity] = []
    if rear_lidar:
        rear_frame = LaunchConfiguration('rear_lidar_frame').perform(context)
        nodes.append(Node(
            package='core_perception',
            executable='lidar_merge_node',
            name='lidar_merge',
            namespace=PERCEPTION_NS,
            output='screen',
            parameters=[
                str(merge_config),
                # 実機の livox_ros_driver2 は全 LiDAR に同じ frame_id を付けるので、2 台目のフレームはここで決める
                {'use_sim_time': use_sim_time, 'secondary_frame_id': rear_frame},
            ],
        ))
        perception_overrides.update({
            'input_topic': merged_topic(merge_config),
            # クラスタごとに近いほうの LiDAR から見た向きで中心を求める
            'additional_sensor_frames': [rear_frame],
        })
    perception = NodeSpec(
        package='core_perception',
        executable='perception_node',
        plugin='core_perception::PerceptionNode',
        name='perception_node',
        namespace=PERCEPTION_NS,
        parameters=(
            *(
                str(config / f'{name}.yaml')
                for name in (
                    'field', 'footprint', 'outlier_filter', 'cluster_split', 'tracking',
                    'perception',
                )
            ),
            *field_overlay(LaunchConfiguration('field_file').perform(context)),
            perception_overrides,
        ),
    )
    target = LaunchConfiguration('container').perform(context)
    if not parse_composition(LaunchConfiguration('composition').perform(context)):
        nodes.append(standalone_node(perception))
    elif target:
        nodes.append(load_into(target, [perception]))
    else:
        nodes.append(container(LIDAR_CONTAINER, [perception]))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_path', description='地図の PCD'),
        DeclareLaunchArgument(
            'field_file', default_value='',
            description='フィールドの範囲（field.x_min など）を config/field.yaml の後から重ねる設定ファイル'
                        '（空なら重ねない。navigation.launch.py は map_name のプリセットで選ぶ）',
        ),
        DeclareLaunchArgument(
            'rear_lidar', default_value='false', choices=['true', 'false'],
            description='2 台目の LiDAR の点群を合わせて検出の入力にする（lidar_merge）',
        ),
        DeclareLaunchArgument(
            'rear_lidar_frame', default_value='livox_rear_frame',
            description='2 台目の LiDAR のフレーム（lidar_merge が 1 台目へ写す TF と、検出がクラスタごとに近い LiDAR を選ぶのに使う）',
        ),
        DeclareLaunchArgument(
            'composition', default_value=DEFAULT_COMPOSITION, choices=list(COMPOSITION_MODES),
            description='検出を点群の入力と同じプロセス（コンテナ）に入れる',
        ),
        DeclareLaunchArgument(
            'container', default_value='',
            description=f'composition:=true で検出を入れる、起動済みのコンテナ（空なら {LIDAR_CONTAINER} を起動する）',
        ),
        OpaqueFunction(function=build_nodes),
    ])
