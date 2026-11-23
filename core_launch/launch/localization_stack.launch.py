"""FAST-LIO、地図照合、点群フィルタ、自己位置の異常判定を起動する.

filter_layout でフィルタの場所を選ぶ。
- map_matching: 地図照合の入力だけに入れる
- lio_input: FAST-LIO の手前に入れる

imu_source で FAST-LIO に入れる IMU を選ぶ（値は config/imu_source.yaml）。
- livox: Mid-360 内蔵の IMU
- damiao: Damiao IMU
- fused: 2 つを imu_fusion で融合した値（Damiao の座標系・時刻・周期）

point_extraction で、地図照合の入力（scan_filter）から照合に使う点だけを取り出す点群抽出のモードを選ぶ
（値は config/point_extraction.yaml、core_nav_launch/point_extraction.py）。FAST-LIO の手前にはかけない。
- off: かけない（既定）
- ceiling: 床と、床から 2.2m より上だけを残す（人の帯と、exclude_regions_file の観客席の箱の中を除く）
- field_objects: フィールドとその周り（config/field.yaml の範囲 + 余裕）の、床から 2.2m までだけを残す。
  フィールドの中の相手は /perception/tracks の追跡の柱で除く

lio_self_filter で、FAST-LIO の前処理での自機の除き方を選ぶ（core_nav_launch/self_filter.py）。
- box: 検出と同じ機体の箱（config/footprint.yaml）の中の点を除く。LiDAR の取り付けは
  robot_description_file（xacro なら robot_description_args を渡して処理する）から読む（既定）
- legacy_blind: 以前の動き（箱なし、blind 0.65m の球）。比べるためだけに残す

composition で、FAST-LIO より後ろのノード（lio_odom_adapter、scan_filter、地図照合、localization_health）を
1 つのプロセス（コンテナ localization_container）にまとめるかを選ぶ（core_nav_launch/composition.py）。
- true: 1 つのプロセスにまとめ、地図照合の入力の点群をコピーせずに渡す（既定）
- false: ノードごとに別のプロセス

state_source で、自己位置の出し方を選ぶ（core_nav_launch/state_source.py）。
- lio: FAST-LIO と地図照合（既定）
- ekf: さらに、車輪の速度（/vehicle/twist）と IMU の角速度を合わせた速度（gyro_odometer）と、地図照合の姿勢を
  EKF（ekf_localizer）で合わせ、/localization/kinematic_state に出す。角速度は融合した IMU（/imu/fused）を使うので、
  imu_source が fused でなくても融合のノードを起動する。制御はまだ LIO の姿勢を使う
"""

import math
from pathlib import Path
from typing import Final

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core_nav_launch.composition import (
    COMPOSITION_MODES,
    DEFAULT_COMPOSITION,
    FAST_LIO_REMAPPINGS,
    launch_nodes,
    LOCALIZATION_CONTAINER,
    LOCALIZATION_NS,
    NodeSpec,
    parse_composition,
)
from core_nav_launch.imu_source import (
    adapter_parameters,
    DEFAULT_IMU_SOURCE,
    fast_lio_parameters,
    FUSED,
    fusion_node,
    IMU_SOURCES,
    load_imu_selection,
    read_accel_clip_mps2,
    scan_filter_parameters,
)
from core_nav_launch.point_extraction import (
    DEFAULT_POINT_EXTRACTION,
    point_extraction_parameters,
    POINT_EXTRACTION_MODES,
)
from core_nav_launch.self_filter import (
    DEFAULT_LIO_SELF_FILTER,
    fast_lio_self_filter_parameters,
    LIO_SELF_FILTER_MODES,
    load_robot_description,
    parse_xacro_args,
)
from core_nav_launch.state_source import (
    DEFAULT_STATE_SOURCE,
    ekf_node_specs,
    parse_state_source,
    STATE_SOURCES,
)

LAYOUT_MAP_MATCHING: Final = 'map_matching'
LAYOUT_LIO_INPUT: Final = 'lio_input'


def build_nodes(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    def flag(name: str) -> bool:
        return arg(name).lower() == 'true'

    layout = arg('filter_layout')
    if layout not in (LAYOUT_MAP_MATCHING, LAYOUT_LIO_INPUT):
        raise RuntimeError(
            f'filter_layout は {LAYOUT_MAP_MATCHING} か {LAYOUT_LIO_INPUT} を指定してください: {layout}'
        )
    map_path = arg('map_path')
    if not Path(map_path).is_file():
        raise RuntimeError(f'地図が見つかりません: {map_path}')

    # 地図と同じ名前の占有格子があれば、LIO を失ったときに捕まえ直す機能を使う
    occupancy_yaml = Path(map_path).with_suffix('.yaml')
    # 地図照合と自己位置の異常の判定に、既定の設定の後から重ねる設定（試験で新しい働きを入れるとき）
    extra_params_file = arg('localization_params_file')
    if extra_params_file and not Path(extra_params_file).is_file():
        raise RuntimeError(f'localization_params_file が見つかりません: {extra_params_file}')
    extra_params = (extra_params_file,) if extra_params_file else ()

    share = Path(get_package_share_directory('core_launch'))
    use_sim_time = flag('use_sim_time')
    use_ekf = parse_state_source(arg('state_source'))
    robot_description_file = Path(arg('robot_description_file'))
    robot_description_args = parse_xacro_args(arg('robot_description_args'))
    robot_description = load_robot_description(robot_description_file, robot_description_args)
    imu_config = share / 'config' / 'imu_source.yaml'
    # extrinsic: urdf の IMU（Damiao）の外部パラメータは、LiDAR の取り付けと同じロボットの記述から求める
    imu = load_imu_selection(imu_config, arg('imu_source'), robot_description)
    fast_lio_file = share / 'config' / 'fast_lio.yaml'
    filter_params = {
        'every_n_frames': int(arg('every_n_frames')),
        'voxel.enabled': flag('voxel_enabled'),
        'voxel.leaf_size_m': float(arg('voxel_leaf_size_m')),
        'random.enabled': flag('random_enabled'),
        'random.target_points': int(arg('random_target_points')),
        'use_sim_time': use_sim_time,
    }
    scan_filter_file = str(share / 'config' / 'scan_filter.yaml')
    # 点群抽出は地図照合の入力（scan_filter）だけにかける
    extraction_parameters = point_extraction_parameters(
        arg('point_extraction'), share / 'config', Path(arg('exclude_regions_file')),
        arg('field_file'),
    )

    fast_lio_overrides: dict[str, object] = {
        'use_sim_time': use_sim_time,
        # 選んだ IMU のトピック、外部パラメータ、加速度の単位に合わせる
        **fast_lio_parameters(imu, read_accel_clip_mps2(fast_lio_file)),
        # 地図照合の入力に歪み補正後の点群を使う
        'publish.scan_publish_en': True,
        'publish.scan_bodyframe_pub_en': True,
        # 自機の除去。検出と同じ機体の箱と、URDF の LiDAR の取り付け
        **fast_lio_self_filter_parameters(
            arg('lio_self_filter'),
            share / 'config' / 'footprint.yaml',
            robot_description_file,
            robot_description_args,
        ),
    }
    nodes: list[LaunchDescriptionEntity] = []
    if layout == LAYOUT_LIO_INPUT:
        nodes.append(Node(
            package='core_localization',
            executable='livox_filter',
            name='livox_filter',
            namespace=LOCALIZATION_NS,
            output='screen',
            parameters=[scan_filter_file, filter_params],
            remappings=[('input', '/sensing/livox/lidar'), ('output', '/localization/livox/lidar_filtered')],
        ))
        fast_lio_overrides['common.lid_topic'] = '/localization/livox/lidar_filtered'
        fast_lio_overrides.update({
            # 間引きはフィルタに任せる。内部のボクセルが比較の邪魔をしないよう細かくする
            'point_filter_num': 1,
            'filter_size_surf': 0.1,
        })
        # 点は FAST-LIO の手前で減らしたので、地図照合の入力では減らさない。
        # scan_filter.yaml の既定値は減らす設定なので、ここで明示的に止める
        scan_filter_params = {
            'use_sim_time': use_sim_time, 'voxel.enabled': False, 'random.enabled': False,
        }
    else:
        scan_filter_params = filter_params

    fusion = fusion_node(imu, share / 'config' / 'imu_fusion.yaml', use_sim_time)
    if fusion is not None:
        nodes.append(fusion)
    # EKF の角速度は融合した IMU を使う。FAST-LIO が融合を使わないときは、EKF のために融合を起動する
    ekf_imu = load_imu_selection(imu_config, FUSED, robot_description) if use_ekf else None
    if ekf_imu is not None and fusion is None:
        ekf_fusion = fusion_node(ekf_imu, share / 'config' / 'imu_fusion.yaml', use_sim_time)
        if ekf_fusion is None:
            raise RuntimeError('融合した IMU の設定から、融合のノードを作れませんでした')
        nodes.append(ekf_fusion)

    yaw = float(arg('initial_yaw'))
    nodes.append(Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        namespace=LOCALIZATION_NS,
        output='screen',
        parameters=[str(fast_lio_file), fast_lio_overrides],
        remappings=FAST_LIO_REMAPPINGS,
    ))
    # composition:=true なら、FAST-LIO より後ろの段を 1 つのコンテナにまとめる
    nodes.extend(launch_nodes(parse_composition(arg('composition')), LOCALIZATION_CONTAINER, [
        NodeSpec(
            package='core_localization',
            executable='lio_odom_adapter',
            plugin='core_localization::LioOdomAdapter',
            name='lio_odom_adapter',
            namespace=LOCALIZATION_NS,
            parameters=({'use_sim_time': use_sim_time, **adapter_parameters(imu)},),
        ),
        NodeSpec(
            package='core_localization',
            executable='scan_filter',
            plugin='core_localization::ScanFilter',
            name='scan_filter',
            namespace=LOCALIZATION_NS,
            parameters=(
                scan_filter_file,
                {'every_n_frames': int(arg('every_n_frames'))},
                scan_filter_params,
                scan_filter_parameters(imu),
                *extraction_parameters,
            ),
            remappings=(('input', '/localization/cloud_registered_body'), ('output', '/localization/cloud')),
        ),
        NodeSpec(
            package='lidar_localization_ros2',
            executable='lidar_localization_node',
            plugin='PCLLocalization',
            name='lidar_localization',
            namespace=LOCALIZATION_NS,
            parameters=(
                str(share / 'config' / 'lidar_localization.yaml'),
                {
                    'use_sim_time': use_sim_time,
                    'map_path': map_path,
                    'enable_local_reacquisition': occupancy_yaml.is_file(),
                    'local_reacquisition_occupancy_yaml':
                        str(occupancy_yaml) if occupancy_yaml.is_file() else '',
                    'initial_pose_x': float(arg('initial_x')),
                    'initial_pose_y': float(arg('initial_y')),
                    'initial_pose_z': 0.0,
                    'initial_pose_qx': 0.0,
                    'initial_pose_qy': 0.0,
                    'initial_pose_qz': math.sin(yaw / 2.0),
                    'initial_pose_qw': math.cos(yaw / 2.0),
                },
                *extra_params,
            ),
            # 地図照合のトピックは相対名なので、名前空間の下（/localization/cloud・pcl_pose など）になる。
            # RViz の既定の /initialpose は直下のまま受ける
            remappings=(('initialpose', '/initialpose'),),
        ),
        NodeSpec(
            package='core_localization',
            executable='localization_health',
            plugin='core_localization::LocalizationHealthNode',
            name='localization_health',
            namespace=LOCALIZATION_NS,
            parameters=(
                str(share / 'config' / 'localization_health.yaml'),
                {'use_sim_time': use_sim_time},
                *extra_params,
            ),
        ),
        *(ekf_node_specs(share / 'config', ekf_imu.topic, use_sim_time) if ekf_imu is not None else ()),
    ]))
    nodes.append(Node(
        package='lidar_localization_ros2',
        executable='start_lifecycle_node.py',
        name='start_lidar_localization',
        namespace=LOCALIZATION_NS,
        arguments=[f'/{LOCALIZATION_NS}/lidar_localization'],
        output='screen',
    ))
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
        DeclareLaunchArgument('initial_x', default_value='0.0', description='地図上の初期位置 x [m]'),
        DeclareLaunchArgument('initial_y', default_value='0.0', description='地図上の初期位置 y [m]'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0', description='地図上の初期方位 [rad]'),
        DeclareLaunchArgument(
            'filter_layout', default_value=LAYOUT_MAP_MATCHING,
            description='点群フィルタの場所（map_matching か lio_input）',
        ),
        DeclareLaunchArgument('every_n_frames', default_value='1', description='地図照合に渡す間隔'),
        DeclareLaunchArgument('voxel_enabled', default_value='true'),
        DeclareLaunchArgument('voxel_leaf_size_m', default_value='0.5'),
        DeclareLaunchArgument('random_enabled', default_value='true'),
        DeclareLaunchArgument('random_target_points', default_value='3000'),
        DeclareLaunchArgument(
            'imu_source', default_value=DEFAULT_IMU_SOURCE, choices=list(IMU_SOURCES),
            description='FAST-LIO に入れる IMU（値は config/imu_source.yaml）',
        ),
        DeclareLaunchArgument(
            'point_extraction', default_value=DEFAULT_POINT_EXTRACTION,
            choices=list(POINT_EXTRACTION_MODES),
            description='地図照合の入力の点群抽出（off / ceiling: 床と人の頭より上 / field_objects: フィールドとその周り）',
        ),
        DeclareLaunchArgument(
            'exclude_regions_file',
            default_value=str(
                Path(get_package_share_directory('core_launch')) / 'config' / 'exclude_regions.yaml'
            ),
            description='点群抽出の ceiling で除く観客席の箱の設定ファイル',
        ),
        DeclareLaunchArgument(
            'lio_self_filter', default_value=DEFAULT_LIO_SELF_FILTER, choices=list(LIO_SELF_FILTER_MODES),
            description='FAST-LIO での自機の除き方（box: 機体の箱 / legacy_blind: 以前の blind 0.65m の球）',
        ),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value=str(
                Path(get_package_share_directory('core_launch')) / 'urdf' / 'core2026.urdf.xacro'
            ),
            description='LiDAR の取り付けを読むロボットの記述（URDF か xacro）',
        ),
        DeclareLaunchArgument(
            'robot_description_args', default_value='',
            description='xacro の引数（name:=value を空白で区切る。空なら xacro の既定値）',
        ),
        DeclareLaunchArgument(
            'composition', default_value=DEFAULT_COMPOSITION, choices=list(COMPOSITION_MODES),
            description='FAST-LIO より後ろのノードを 1 つのプロセス（コンテナ）にまとめる',
        ),
        DeclareLaunchArgument(
            'state_source', default_value=DEFAULT_STATE_SOURCE, choices=list(STATE_SOURCES),
            description='自己位置の出し方（lio / ekf: 車輪の速度・IMU・地図照合の EKF も起動し、'
                        '/localization/kinematic_state に出す）',
        ),
        DeclareLaunchArgument(
            'localization_params_file', default_value='',
            description='地図照合（lidar_localization）と localization_health に、既定の設定の後から重ねる '
                        'パラメータファイル（ノード名の節に書く。空なら重ねない）',
        ),
        OpaqueFunction(function=build_nodes),
    ])
