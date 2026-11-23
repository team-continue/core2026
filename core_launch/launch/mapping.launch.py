"""実機（または録った rosbag）で、地図照合に使う地図（PCD）を FAST-LIO で作る.

起動するもの:
- Livox のドライバ（livox_driver:=true。rosbag やシミュレーションの点群を使うときは false）
- robot_state_publisher（robot_state_publisher:=true。base_link→センサの固定 TF。rosbag の /tf_static を使うときは false）
- FAST-LIO（navigation.launch.py の自己位置と同じ設定。IMU の選択と自機の除去も同じ）と lio_odom_adapter
- lio_map_saver_node.py（点群を odom の座標＝作り始めの base_link で貯め、output に PCD を保存する）

地図照合（lidar_localization）と自律移動は起動しない。navigation.launch.py と同時には起動しない（FAST-LIO が 2 つになる）。
作り始めは機体を水平な床に止めておく（odom の z 軸が鉛直になり、床が z=0 になる）。
ゆっくり（0.5m/s 前後）走り、近くの物は周りを回って見せる。速く走ると薄い物や裏側の面の点が欠け、自律移動で
物に近づきすぎる（sim の CoRE-1 で、最高 3m/s の地図は 6 回のうち 3 回で構造物に触れ、平均 0.5m/s の地図は 0 回）。

保存: ros2 service call /mapping/map_saver/save std_srvs/srv/Trigger（止めたときにも保存する）
次に make_map_files.py で占有地図とフィールドの範囲を作る。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from core_nav_launch.composition import FAST_LIO_REMAPPINGS, LOCALIZATION_NS
from core_nav_launch.imu_source import (
    adapter_parameters,
    DEFAULT_IMU_SOURCE,
    fast_lio_parameters,
    fusion_node,
    IMU_SOURCES,
    load_imu_selection,
    read_accel_clip_mps2,
)
from core_nav_launch.livox_driver import livox_driver_node
from core_nav_launch.self_filter import (
    DEFAULT_LIO_SELF_FILTER,
    fast_lio_self_filter_parameters,
    LIO_SELF_FILTER_MODES,
    load_robot_description,
    parse_xacro_args,
)


def build_nodes(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    def flag(name: str) -> bool:
        return arg(name).lower() == 'true'

    output = Path(arg('output')).expanduser()
    if not arg('output'):
        raise RuntimeError('output:=<保存する PCD の場所> を指定してください（例: output:=~/maps/lab.pcd）')
    if output.suffix != '.pcd':
        raise RuntimeError(f'output は .pcd にしてください: {output}')
    if output.exists() and not flag('overwrite'):
        raise RuntimeError(f'{output} がすでにあります。別の名前にするか overwrite:=true を付けてください')

    share = Path(get_package_share_directory('core_launch'))
    use_sim_time = flag('use_sim_time')
    robot_description_file = Path(arg('robot_description_file'))
    robot_description_args = parse_xacro_args(arg('robot_description_args'))
    robot_description = load_robot_description(robot_description_file, robot_description_args)
    imu = load_imu_selection(share / 'config' / 'imu_source.yaml', arg('imu_source'), robot_description)
    fast_lio_file = share / 'config' / 'fast_lio.yaml'

    nodes: list[LaunchDescriptionEntity] = []
    if flag('livox_driver'):
        nodes.append(livox_driver_node())
    if flag('robot_state_publisher'):
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
                'use_sim_time': use_sim_time,
            }],
            # 腰の角度（base_to_chassis）は chassis_joint_state が関節名を付けて出す（navigation.launch.py と同じ）
            remappings=[('joint_states', '/control/chassis/joint_states')],
        ))
    fusion = fusion_node(imu, share / 'config' / 'imu_fusion.yaml', use_sim_time)
    if fusion is not None:
        nodes.append(fusion)
    nodes.append(Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        namespace=LOCALIZATION_NS,
        output='screen',
        parameters=[str(fast_lio_file), {
            'use_sim_time': use_sim_time,
            # 選んだ IMU のトピック、外部パラメータ、加速度の単位に合わせる（localization_stack.launch.py と同じ）
            **fast_lio_parameters(imu, read_accel_clip_mps2(fast_lio_file)),
            # 地図にする点群（camera_init の座標）。間引く前の、歪みを補正した点を出す
            'publish.scan_publish_en': True,
            'publish.dense_publish_en': True,
            **fast_lio_self_filter_parameters(
                arg('lio_self_filter'),
                share / 'config' / 'footprint.yaml',
                robot_description_file,
                robot_description_args,
            ),
        }],
        remappings=FAST_LIO_REMAPPINGS,
    ))
    nodes.append(Node(
        package='core_localization',
        executable='lio_odom_adapter',
        name='lio_odom_adapter',
        namespace=LOCALIZATION_NS,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, **adapter_parameters(imu)}],
    ))
    nodes.append(Node(
        package='core_launch',
        executable='lio_map_saver_node.py',
        name='map_saver',
        namespace='mapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output': str(output),
            'voxel_size_m': float(arg('voxel_size_m')),
            'min_hits': int(arg('min_hits')),
            'max_range_m': float(arg('max_range_m')),
            'origin_x': float(arg('origin_x')),
            'origin_y': float(arg('origin_y')),
            'origin_yaw': float(arg('origin_yaw')),
        }],
    ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='', description='保存する PCD の場所（例: ~/maps/lab.pcd）'),
        DeclareLaunchArgument('overwrite', default_value='false', choices=['true', 'false'],
                              description='output がすでにあれば書き換える'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false'],
                              description='rosbag（--clock で再生）やシミュレーションの時刻を使う'),
        DeclareLaunchArgument('livox_driver', default_value='true', choices=['true', 'false'],
                              description='実機の Livox のドライバを起動する（rosbag やシミュレーションでは false）'),
        DeclareLaunchArgument('robot_state_publisher', default_value='true', choices=['true', 'false'],
                              description='URDF から機体の固定 TF を出す（rosbag の /tf_static を使うときは false）'),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value=str(Path(get_package_share_directory('core_launch')) / 'urdf' / 'core2026.urdf.xacro'),
            description='機体の URDF（xacro）。LiDAR の取り付けと自機の除去に使う',
        ),
        DeclareLaunchArgument('robot_description_args', default_value='',
                              description='xacro の引数（例: "lidar_x:=0.13 lidar_height:=0.48"）'),
        DeclareLaunchArgument('imu_source', default_value=DEFAULT_IMU_SOURCE, choices=list(IMU_SOURCES),
                              description='FAST-LIO に入れる IMU（config/imu_source.yaml）'),
        DeclareLaunchArgument('lio_self_filter', default_value=DEFAULT_LIO_SELF_FILTER,
                              choices=list(LIO_SELF_FILTER_MODES), description='FAST-LIO の前処理での自機の除き方'),
        DeclareLaunchArgument('voxel_size_m', default_value='0.1', description='地図のボクセルの大きさ [m]'),
        DeclareLaunchArgument('min_hits', default_value='3',
                              description='このフレーム数以上で見たボクセルだけを残す（通り過ぎた人などを除く）'),
        DeclareLaunchArgument('max_range_m', default_value='30.0', description='地図に入れる点の、センサーからの距離の上限 [m]'),
        DeclareLaunchArgument('origin_x', default_value='0.0', description='作り始めの機体の、地図上の位置 x [m]'),
        DeclareLaunchArgument('origin_y', default_value='0.0', description='作り始めの機体の、地図上の位置 y [m]'),
        DeclareLaunchArgument('origin_yaw', default_value='0.0', description='作り始めの機体の、地図上の向き [rad]'),
        OpaqueFunction(function=build_nodes),
    ])
