"""
Navigation launch file.

Launches the full navigation pipeline with configurable control modes and map presets.
Sensors (real) and the Unity ROS-TCP-Endpoint (sim) are launched by core2026_system.launch.xml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

# ── Map presets ──────────────────────────────────────────────────────
MAP_PRESETS = {
    'core1_field': {
        'image': 'core1_field.png',
        'resolution': 0.025,
        'origin_x': -13.675,
        'origin_y': -9.15,
        'init_x': -10.7,
        'init_y': 5.9,
    },
    'curious_house': {
        'image': 'curious_house.png',
        'resolution': 0.025,
        'origin_x': -4.5,
        'origin_y': -7.5,
        'init_x': 0.0,
        'init_y': 0.0,
    },
}


def _launch_nodes(context):
    """Resolve map preset and return all node actions."""
    # ── Resolve launch arguments ─────────────────────────────────────
    map_name = context.launch_configurations['map_name']
    use_rviz = context.launch_configurations['use_rviz']
    use_localization = context.launch_configurations.get(
        'use_localization', 'false').lower() == 'true'

    # ── Map preset ───────────────────────────────────────────────────
    preset = MAP_PRESETS.get(map_name)
    if preset is None:
        available = ', '.join(MAP_PRESETS.keys())
        raise RuntimeError(
            f"Unknown map_name '{map_name}'. Available: {available}")

    core_launch_share = get_package_share_directory('core_launch')

    # ── Package directories ──────────────────────────────────────────
    body_ctrl_share = get_package_share_directory('core_body_controller')

    rviz_config = os.path.join(core_launch_share, 'config', 'navigation.rviz')

    actions = []

    # ── 無効化中のノードについて ─────────────────────────────────────
    # 以下のコメントアウト済みブロックは名前空間対応済みの形で記述してあります。
    # そのままコメントを外せば docs/architecture/topics.md のトピック一覧と
    # 一致した名前空間（localization / map / planning）で起動します。

    # ── 3. FAST-LIO (when use_fastlio) ──────────────────────────────
    # if use_fastlio:
    #     actions.append(GroupAction([
    #         PushRosNamespace('localization'),
    #         Node(
    #             package='fast_lio',
    #             executable='fastlio_mapping',
    #             name='fastlio_mapping',
    #             output='screen',
    #             parameters=[
    #                 PathJoinSubstitution([
    #                     FindPackageShare('fast_lio'), 'config', 'mid360.yaml',
    #                 ]),
    #                 {
    #                     'common.lid_topic': '/sensing/livox/lidar',
    #                     'common.imu_topic': imu_topic,
    #                     'preprocess.lidar_type': lidar_type,
    #                 },
    #             ],
    #         ),
    #     ]))

    # ── 4. Static TF / Localization ──────────────────────────────────
    if use_localization:
        # Dynamic map→odom from NDT/ICP localization node
        localization_share = get_package_share_directory('core_localization')
        localization_config = os.path.join(
            localization_share, 'config', 'localization_params.yaml')

        # Resolve PCD map path from map_name → pcd_maps/<map_name>.pcd
        resolved_pcd_path = os.path.join(
            localization_share, 'pcd_maps', f'{map_name}.pcd')

        if not os.path.isfile(resolved_pcd_path):
            raise RuntimeError(
                f"PCD map not found: '{resolved_pcd_path}'. "
                f"Place the PCD file at core_localization/pcd_maps/{map_name}.pcd "
                f"and rebuild.")

        actions.append(GroupAction([
            PushRosNamespace('localization'),
            Node(
                package='core_localization',
                executable='localization_node',
                name='localization_node',
                output='screen',
                parameters=[
                    localization_config,
                    {
                        'global_map_path': resolved_pcd_path,
                        'initial_pose_x': preset['init_x'],
                        'initial_pose_y': preset['init_y'],
                    },
                ],
            ),
        ]))
    else:
        # Static identity map→odom (no global localization)
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_odom',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ))

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_livox',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.5',
            '--roll', '3.141592653589793', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'livox_frame',
        ],
    ))

    # ── 5. Map server ───────────────────────────────────────────────
    # ソース内はいずれも絶対名なので PushRosNamespace では移動しない。
    # /map はグローバル据え置き、グローバルコストマップのみリマップする。
    # actions.append(GroupAction([
    #     PushRosNamespace('map'),
    #     Node(
    #         package='core_launch',
    #         executable='map_server_node.py',
    #         name='map_server_node',
    #         output='screen',
    #         remappings=[
    #             ('/costmap/global', '/map/costmap/global'),
    #         ],
    #         parameters=[{
    #             'image_path': map_image_path,
    #             'resolution': preset['resolution'],
    #             'origin_x': preset['origin_x'],
    #             'origin_y': preset['origin_y'],
    #             'inflation_radius_m': 0.40,
    #             'decay_margin_m': 0.20,
    #         }],
    #     ),
    # ]))

    # ── 6. Odom bridge ──────────────────────────────────────────────
    # 入力（/sim_odom, /Odometry）は外部境界なのでグローバル据え置き。
    # actions.append(GroupAction([
    #     PushRosNamespace('localization'),
    #     Node(
    #         package='core_launch',
    #         executable='odom_bridge_node.py',
    #         name='odom_bridge_node',
    #         output='screen',
    #         remappings=[
    #             ('/odom', '/localization/odom'),
    #             ('/start_pose', '/localization/start_pose'),
    #         ],
    #         parameters=[{
    #             'odom_source': effective_odom_source,
    #             'init_x': preset['init_x'],
    #             'init_y': preset['init_y'],
    #             'init_yaw': float(init_yaw),
    #         }],
    #     ),
    # ]))

    # ── 7. Path planner ─────────────────────────────────────────────
    # actions.append(GroupAction([
    #     PushRosNamespace('planning'),
    #     Node(
    #         package='core_path_planner',
    #         executable='path_planner_node',
    #         name='core_path_planner_node',
    #         output='screen',
    #         parameters=[{
    #             'goal_topic': '/behavior/goal_pose',
    #             'start_topic': '/localization/start_pose',
    #             'path_topic': 'planned_path',
    #             'local_costmap_topic': 'costmap/local',
    #             'global_map_topic': '/map',
    #             'publish_in_global_frame': True,
    #             'global_frame_id': 'odom',
    #             'cost_weight': 2.0,
    #         }],
    #     ),
    # ]))

    # ── 8. MPPI controller ──────────────────────────────────────────
    # use_smoother = context.launch_configurations.get('use_smoother', 'true')
    # mppi_cmd_vel_topic = (
    #     'cmd_vel_raw' if use_smoother.lower() == 'true' else '/control/cmd_vel')

    # actions.append(GroupAction([
    #     PushRosNamespace('planning'),
    #     Node(
    #         package='core_mppi',
    #         executable='core_mppi_node',
    #         name='core_mppi_node',
    #         output='screen',
    #         remappings=[('/goal_reached', '/behavior/goal_reached')],
    #         parameters=[mppi_params, {'cmd_vel_topic': mppi_cmd_vel_topic}],
    #     ),
    # ]))

    # ── 9. cmd_vel smoother ──────────────────────────────────────────
    # if use_smoother.lower() == 'true':
    #     actions.append(GroupAction([
    #         PushRosNamespace('planning'),
    #         Node(
    #             package='core_cmd_vel_smoother',
    #             executable='cmd_vel_smoother_node',
    #             name='cmd_vel_smoother_node',
    #             output='screen',
    #             parameters=[{
    #                 'alpha': 1.0,
    #                 'input_topic': 'cmd_vel_raw',
    #                 'output_topic': '/control/cmd_vel',
    #                 'timeout_sec': 0.2,
    #             }],
    #         ),
    #     ]))

    # ── 10. Costmap builder ─────────────────────────────────────────
    # actions.append(GroupAction([
    #     PushRosNamespace('planning'),
    #     Node(
    #         package='core_costmap_builder',
    #         executable='costmap_build_node',
    #         name='costmap_build_node',
    #         output='screen',
    #         parameters=[costmap_params],
    #     ),
    # ]))

    # ── 11. Body controller ─────────────────────────────────────────
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(body_ctrl_share, 'launch', 'body_controller.launch.py'),
        ),
    ))

    # ── 12. RViz ────────────────────────────────────────────────────
    if use_rviz.lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_source', default_value='sim',
            description="'sim' (/sim_odom) or 'fastlio' (/Odometry). Forced 'fastlio' when real.",
        ),
        DeclareLaunchArgument(
            'map_name', default_value='core1_field',
            description=f"Map preset: {', '.join(MAP_PRESETS.keys())}",
        ),
        DeclareLaunchArgument(
            'init_yaw', default_value='0.0',
            description='Initial yaw in odom frame [rad] (FAST-LIO mode)',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'use_smoother', default_value='true',
            description='Enable cmd_vel EMA smoother between MPPI and body_controller',
        ),
        DeclareLaunchArgument(
            'use_localization', default_value='false',
            description='Enable NDT/ICP localization (dynamic map→odom TF)',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
