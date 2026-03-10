"""Navigation launch: path_planner + costmap_builder + MPPI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    core_launch_share = get_package_share_directory('core_launch')
    mppi_share = get_package_share_directory('core_mppi')
    costmap_share = get_package_share_directory('core_costmap_builder')

    rviz_config = os.path.join(
        core_launch_share, 'config', 'integration_test.rviz')
    mppi_params = os.path.join(mppi_share, 'param', 'default_params.yaml')
    costmap_params = os.path.join(
        costmap_share, 'config', 'costmap_build_node.yaml')
    livox_user_config = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'])

    # --- Launch arguments ---
    # Default: look for map image installed alongside core_launch share,
    # but typically users should pass the actual path via launch argument.
    map_image_arg = DeclareLaunchArgument(
        'map_image',
        default_value=os.path.join(
            core_launch_share, 'curious_house.png'),
        description='Path to global_map.png',
    )

    odom_source_arg = DeclareLaunchArgument(
        'odom_source',
        default_value='sim',
        description='Odometry source: "sim" (Unity /sim_odom) or "fastlio" (/Odometry)',
    )

    init_yaw_arg = DeclareLaunchArgument(
        'init_yaw',
        default_value='0.0',
        description='Initial yaw in odom frame [rad] (used in fastlio mode)',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true',
    )

    # Condition: odom_source == "fastlio"
    is_fastlio = PythonExpression(
        ["'", LaunchConfiguration('odom_source'), "' == 'fastlio'"])

    # 1a. ROS-TCP-Endpoint (Unity bridge) -- always launched
    tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': '127.0.0.1',
            'ROS_TCP_PORT': 10000,
        }],
    )

    # 1b. Livox driver -- only in fastlio mode
    # 外部 rviz_MID360_launch.py は RViz も同時起動するため、
    # ここでは PointCloud2 を出すドライバ本体だけを直接起動する。
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': livox_user_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
        condition=IfCondition(is_fastlio),
    )

    # 1c. FAST-LIO -- only in fastlio mode
    # FindPackageShare はノード実行時に評価されるため、
    # fast_lio 未インストール環境でも sim モードなら問題なし
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'config', 'mid360.yaml']),
            {
                'common.lid_topic': '/livox/lidar/no_self',
                'preprocess.lidar_type': 4,
            },
        ],
        condition=IfCondition(is_fastlio),
    )

    # 2. Static TF: map -> odom (identity)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    # 3. Static TF: base_link -> livox_frame
    static_tf_base_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_livox',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.5',
            '--roll', '3.141592653589793', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
    )

    # 4. Map server (PNG -> OccupancyGrid)
    map_server = Node(
        package='core_launch',
        executable='map_server_node.py',
        name='map_server_node',
        output='screen',
        parameters=[{
            'image_path': LaunchConfiguration('map_image'),
            'resolution': 0.025,
            'origin_x': -4.5,
            'origin_y': -7.5,
        }],
    )

    # 5. Odom bridge (sim_odom or FAST-LIO -> odom + TF + start_pose)
    odom_bridge = Node(
        package='core_launch',
        executable='odom_bridge_node.py',
        name='odom_bridge_node',
        output='screen',
        parameters=[{
            'odom_source': LaunchConfiguration('odom_source'),
            'init_x': 0.0,
            'init_y': 0.0,
            'init_yaw': LaunchConfiguration('init_yaw'),
        }],
    )

    # 6. Goal pose: use RViz "2D Goal Pose" button (publishes to /goal_pose)

    # 7. Path planner
    path_planner = Node(
        package='core_path_planner',
        executable='path_planner_node',
        name='core_path_planner_node',
        output='screen',
        parameters=[{
            'local_costmap_topic': '/costmap/local',
            'publish_in_global_frame': True,
            'global_frame_id': 'odom',
        }],
    )

    # 8. MPPI controller
    mppi_node = Node(
        package='core_mppi',
        executable='core_mppi_node',
        name='core_mppi_node',
        output='screen',
        parameters=[mppi_params],
    )

    # 9. Costmap builder (node only, no static TF — odom_bridge handles it)
    costmap_build = Node(
        package='core_costmap_builder',
        executable='costmap_build_node',
        name='costmap_build_node',
        output='screen',
        parameters=[costmap_params],
    )

    # 10. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        map_image_arg,
        odom_source_arg,
        init_yaw_arg,
        use_rviz_arg,
        # tcp_endpoint,
        livox_driver,
        fast_lio_node,
        static_tf_map_odom,
        static_tf_base_livox,
        map_server,
        odom_bridge,
        path_planner,
        mppi_node,
        costmap_build,
        rviz,
    ])
