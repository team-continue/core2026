"""
aim_bot の画像追尾を仮想砲塔・仮想カメラで試す.

    ROS_LOCALHOST_ONLY=1 ros2 launch core_shooter aim_bot_tracking_sim.launch.py \
        side:=left scenario:=triangle csv_path:=/tmp/track.csv

aim_bot は本番と同じ shooter.params.yaml を読み、aim_bot_tracking_sim.yaml の
/**/aim_bot セクションで上書きする。仮想カメラの px 換算は shooter.params.yaml の
同じ値を使う。仮想ノードが終了すると launch 全体を終了する。

実機のハードウェアや敵検出と同じネットワークで起動しないこと。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# 仮想カメラが aim_bot と同じ換算を使うために shooter.params.yaml から引き継ぐ値
SHARED_KEYS = (
    'yaw_motor_id', 'pitch_motor_id', 'yaw_image_gain', 'pitch_image_gain',
    'yaw_direction', 'pitch_direction', 'image_width', 'image_height',
    'image_center_x', 'image_center_y',
    'image_shoot_tolerance_x_px', 'image_shoot_tolerance_y_px',
)


def _config(name):
    return os.path.join(get_package_share_directory('core_shooter'), 'config', name)


def _aim_bot_params(shooter_params_path, side):
    with open(shooter_params_path) as f:
        data = yaml.safe_load(f)
    merged = dict(data.get('/**', {}).get('ros__parameters', {}))
    merged.update(data.get(f'/**/{side}/aim_bot', {}).get('ros__parameters', {}))
    if merged.get('target_input_mode', 'image') != 'image':
        raise RuntimeError('aim_bot_tracking_sim supports target_input_mode="image" only')
    missing = [key for key in SHARED_KEYS if key not in merged]
    if missing:
        raise RuntimeError(f'shooter.params.yaml lacks aim_bot params for {side}: {missing}')
    return {key: merged[key] for key in SHARED_KEYS}


def _flatten(params, prefix=''):
    flat = {}
    for key, value in params.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(_flatten(value, f'{name}.'))
        else:
            flat[name] = value
    return flat


def _sim_node_params(sim_params_path):
    with open(sim_params_path) as f:
        data = yaml.safe_load(f)
    section = data.get('/**/aim_bot_tracking_sim', {}).get('ros__parameters')
    if section is None:
        raise RuntimeError(f'{sim_params_path} lacks /**/aim_bot_tracking_sim.ros__parameters')
    return _flatten(section)


def _launch(context):
    side = LaunchConfiguration('side').perform(context)
    if side not in ('left', 'right'):
        raise RuntimeError(f"side must be 'left' or 'right' (got '{side}')")
    shooter_params = _config('shooter.params.yaml')
    sim_params = LaunchConfiguration('sim_params_file').perform(context)

    # 仮想ノードのパラメータは1つの辞書にまとめて渡す。launch_ros は辞書を
    # --params-file より前に置くので、YAML と辞書を混ぜると launch 引数の上書きが効かない。
    sim_node_params = _sim_node_params(sim_params)
    sim_node_params.update(_aim_bot_params(shooter_params, side))
    for name, key in (('scenario', 'target.scenario'), ('csv_path', 'csv_path')):
        value = LaunchConfiguration(name).perform(context)
        if value:
            sim_node_params[key] = value

    sim = Node(
        package='core_shooter',
        executable='aim_bot_tracking_sim',
        name='aim_bot_tracking_sim',
        output='screen',
        parameters=[sim_node_params],
        remappings=[('target_pose', f'/perception/enemy_detection/{side}/target_pose')],
    )
    aim_bot = Node(
        package='core_shooter',
        executable='aim_bot',
        name='aim_bot',
        output='screen',
        parameters=[shooter_params, sim_params],
        remappings=[
            ('hazard_status', '/system/emergency/hazard_status'),
            ('/can/tx', '/hardware/can/tx'),
            ('/test_mode', '/ui/test_mode'),
            ('target_image_position', f'/perception/enemy_detection/{side}/target_pose'),
        ],
    )
    return [
        GroupAction([PushRosNamespace(f'mecha/shooter/{side}'), aim_bot, sim]),
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(
            event=Shutdown(reason='tracking sim finished'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='static / step / triangle / sine（空で設定ファイルの値）'),
        DeclareLaunchArgument('csv_path', default_value='',
                              description='空でなければ全サンプルを CSV に保存する'),
        DeclareLaunchArgument('sim_params_file',
                              default_value=_config('aim_bot_tracking_sim.yaml')),
        OpaqueFunction(function=_launch),
    ])
