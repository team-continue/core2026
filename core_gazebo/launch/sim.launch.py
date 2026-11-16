"""Gazeboのフィールド、機体、ROS 2ブリッジを起動する."""

import atexit
import os
from pathlib import Path
import re
import subprocess
from typing import Final

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WSL_GPU_DEVICE: Final = Path('/dev/dxg')
# 指定しないとWSL2のMesaはllvmpipe（CPU描画）を選ぶ
WSL_GPU_ENV: Final = {
    'GALLIUM_DRIVER': 'd3d12',
    'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA',
}
# 駆動の無駄時間と床との滑り（omni_force_drive の設定。core2026.urdf.xacro の同じ名前の引数へ渡す）。
# 既定は遅れなし・滑らない
SLIP_ARGUMENTS: Final = {
    'drive_dead_time': ('0.0', '速度指令を受けてから駆動が応え始めるまでの時間（無駄時間）[s]'),
    'slip_longitudinal_ratio': ('1.0', '指令の前後の速度のうち、床で出せる割合'),
    'slip_lateral_ratio': ('1.0', '指令の横の速度のうち、床で出せる割合'),
    'slip_yaw_ratio': ('1.0', '指令の角速度のうち、床で出せる割合'),
    'slip_time_constant': ('0.0', '目標の速度との差を埋める時定数 [s]（0 で 1 ステップ）'),
    'slip_static_friction_accel': ('0.0', '床が出せる水平の加速度（静止摩擦）[m/s2]。0 で滑らない'),
    'slip_kinetic_friction_accel': ('0.0', '滑っている間に出る加速度（動摩擦）[m/s2]。0 なら静止摩擦と同じ'),
    'slip_noise_ratio': ('0.0', '目標の速度に足す床のむらの雑音（目標の速さに対する割合）'),
    'slip_noise_time': ('0.5', '床のむらの雑音の相関時間 [s]'),
}


def wsl_gpu_env_actions() -> list[SetEnvironmentVariable]:
    """WSL2ならGPU描画の環境変数を設定する。利用者が設定した値は上書きしない."""
    if not WSL_GPU_DEVICE.exists():
        return []
    return [
        SetEnvironmentVariable(name, value)
        for name, value in WSL_GPU_ENV.items()
        if name not in os.environ
    ]


def in_same_partition(pid: str) -> bool:
    """プロセスの GZ_PARTITION が、この launch と同じか調べる."""
    try:
        environment = Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
    except OSError:
        return False
    variables = dict(item.split(b'=', 1) for item in environment if b'=' in item)
    current = os.environ.get('GZ_PARTITION', '').encode()
    return variables.get(b'GZ_PARTITION', b'') == current


def running_gazebo_pids() -> list[str]:
    """同じパーティションで動作中の Gazebo の PID を返す.

    GZ_PARTITION と ROS_DOMAIN_ID を分ければ、複数のシミュレーションを同時に動かせる。
    起動コマンドの文字列に一致しないよう行頭で照合する。
    """
    result = subprocess.run(
        ['pgrep', '-f', '^gz sim'], capture_output=True, text=True, check=False
    )
    return [pid for pid in result.stdout.split() if in_same_partition(pid)]


def stop_server_left_behind(world: str) -> None:
    """止まらずに残った、同じパーティションのこのワールドの Gazebo サーバーを止める."""
    result = subprocess.run(
        ['pgrep', '-f', f'^gz sim .*{re.escape(world)}'], capture_output=True, text=True, check=False
    )
    for pid in result.stdout.split():
        if in_same_partition(pid):
            subprocess.run(['kill', '-KILL', pid], check=False)


def register_server_cleanup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """launch の終了処理がすべて終わった後に片付けるよう、atexit に登録する."""
    atexit.register(stop_server_left_behind, LaunchConfiguration('world').perform(context))
    return []


def generate_launch_description() -> LaunchDescription:
    # 終了時に Gazebo のサーバーが止まらず、launch が SIGTERM を送るラッパーだけが終わって
    # サーバーが残ることがある（原因は未特定。約 40 回の停止で 2 回）。残ったまま起動すると、
    # 機体の生成先やトピックが混ざって原因の分かりにくい不具合になるので、ここで止める
    stale_pids = running_gazebo_pids()
    if stale_pids:
        pids = ' '.join(stale_pids)
        raise RuntimeError(
            f'Gazebo が既に動いています（PID: {pids}）。前回の終了時に残ったものなら、'
            f'kill -9 {pids} で止めてから起動してください。'
        )

    share = Path(get_package_share_directory('core_gazebo'))
    description_share = Path(get_package_share_directory('core_launch'))
    gz_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py'
    plugin_dir = Path(get_package_prefix('core_gazebo')) / 'lib' / 'core_gazebo'

    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command([
            'xacro ', str(description_share / 'urdf' / 'core2026.urdf.xacro'),
            ' lidar_x:=', LaunchConfiguration('lidar_x'),
            ' lidar_height:=', LaunchConfiguration('lidar_height'),
            ' lidar_roll:=', LaunchConfiguration('lidar_roll'),
            ' body_height:=', LaunchConfiguration('body_height'),
            ' rear_lidar:=', LaunchConfiguration('rear_lidar'),
            ' gazebo:=true',
            *[part for name in SLIP_ARGUMENTS for part in (f' {name}:=', LaunchConfiguration(name))],
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        launch_arguments={
            'gz_args': [
                PythonExpression([
                    "'-r -s --headless-rendering ' if '", headless, "' == 'true' else '-r '",
                ]),
                LaunchConfiguration('world'),
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'field',
            '-topic', 'robot_description',
            '-name', 'core2026_robot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.0',
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sim_bridge',
        output='screen',
        parameters=[{'config_file': str(share / 'config' / 'bridge.yaml')}],
    )

    # 手順2以降は FAST-LIO が odom→base_link を出すので、真値のTFは切り替えられるようにする
    ground_truth_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_tf_bridge',
        output='screen',
        arguments=['/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/ground_truth/tf', '/tf')],
        condition=IfCondition(LaunchConfiguration('publish_ground_truth_tf')),
    )

    # 検出のための 2 台目の LiDAR（rear_lidar:=true）。点群は 1 台目と同じ形で /sim/points_rear に出す
    rear_points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rear_points_bridge',
        output='screen',
        arguments=['/sim/lidar_rear/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[('/sim/lidar_rear/points', '/sensing/sim/points_rear')],
        condition=IfCondition(LaunchConfiguration('rear_lidar')),
    )

    # 実機の足回りが返す車輪の速度の代わり（state_source:=ekf の EKF が使う）
    wheel_odometry = Node(
        package='core_gazebo',
        executable='wheel_odometry_sim',
        name='wheel_odometry_sim',
        output='screen',
        parameters=[str(share / 'config' / 'wheel_odometry.yaml'), {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('wheel_odometry')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(share / 'rviz' / 'sim.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'field.sdf')),
        DeclareLaunchArgument('headless', default_value='true', description='GazeboのGUIを出さない'),
        DeclareLaunchArgument('rviz', default_value='false', description='RVizを起動する'),
        DeclareLaunchArgument('x', default_value='-6.0', description='初期位置 x [m]'),
        DeclareLaunchArgument('y', default_value='-2.5', description='初期位置 y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='初期方位 [rad]'),
        # Mid-360 の既定は Unity の機体の取り付け（core2026.urdf.xacro の既定値と同じ）
        DeclareLaunchArgument('lidar_x', default_value='0.1321', description='LiDARの前後の位置（base_link の x）[m]'),
        DeclareLaunchArgument('lidar_height', default_value='0.484173', description='LiDARの床からの高さ [m]'),
        DeclareLaunchArgument(
            'lidar_roll', default_value='3.14159',
            description='LiDAR の前後の軸まわりの回転 [rad]。3.14159 で上下逆さ（既定。Unity の機体と 2026-03 の'
                        '実機の bag の付け方）。左右の位置・ほかの軸の回転は URDF の既定値',
        ),
        DeclareLaunchArgument(
            'body_height', default_value='0.95',
            description='機体の上面の床からの高さ [m]（points_to_livox で車体に当たった点を見分ける）',
        ),
        DeclareLaunchArgument(
            'rear_lidar', default_value='false', choices=['true', 'false'],
            description='検出のための 2 台目の LiDAR を機体の後ろの端に付ける（取り付けは URDF の rear_lidar_*）',
        ),
        *[
            DeclareLaunchArgument(name, default_value=default, description=description)
            for name, (default, description) in SLIP_ARGUMENTS.items()
        ],
        DeclareLaunchArgument(
            'publish_ground_truth_tf',
            default_value='true',
            description='真値の world→base_link を /tf に出す',
        ),
        DeclareLaunchArgument(
            'wheel_odometry', default_value='false', choices=['true', 'false'],
            description='車輪の速度（/vehicle/twist、真値に車輪の誤差を足したもの）を出す',
        ),
        DeclareLaunchArgument(
            'scenario_file',
            default_value='',
            description='他ロボットと人間の動きの YAML（空なら出さない）',
        ),
        SetEnvironmentVariable('CORE_SCENARIO_FILE', LaunchConfiguration('scenario_file')),
        OpaqueFunction(function=register_server_cleanup),
        *wsl_gpu_env_actions(),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', str(plugin_dir)),
        # URDF のメッシュ（package://core_launch/...）を Gazebo が見つけられるよう、share を足す
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(description_share.parent)),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rear_points_bridge,
        ground_truth_tf_bridge,
        wheel_odometry,
        rviz_node,
    ])
