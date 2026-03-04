"""Integration test launch: path_planner + costmap_builder + MPPI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    core_launch_share = get_package_share_directory('core_launch')
    mppi_share = get_package_share_directory('core_mppi')
    costmap_share = get_package_share_directory('core_costmap_builder')

    rviz_config = os.path.join(
        core_launch_share, 'config', 'integration_test.rviz')
    mppi_params = os.path.join(mppi_share, 'param', 'default_params.yaml')
    costmap_params = os.path.join(
        costmap_share, 'config', 'costmap_build_node.yaml')

    # --- Launch arguments ---
    # Default: look for global_map.png installed alongside core_launch share,
    # but typically users should pass the actual path via launch argument.
    map_image_arg = DeclareLaunchArgument(
        'map_image',
        default_value=os.path.join(
            core_launch_share, 'global_map.png'),
        description='Path to global_map.png',
    )
    # 1. ROS-TCP-Endpoint (Unity bridge)
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
            '--x', '0', '--y', '0', '--z', '0.6',
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
            'origin_x': -13.675,
            'origin_y': -9.15,
        }],
    )

    # 5. Odom bridge (sim_odom -> odom + TF + start_pose, Unity→ROS conversion)
    odom_bridge = Node(
        package='core_launch',
        executable='odom_bridge_node.py',
        name='odom_bridge_node',
        output='screen',
        parameters=[{
            'init_x': -10.7,
            'init_y': 5.9,
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
    )

    return LaunchDescription([
        map_image_arg,
        tcp_endpoint,
        static_tf_map_odom,
        static_tf_base_livox,
        map_server,
        odom_bridge,
        path_planner,
        mppi_node,
        costmap_build,
        rviz,
    ])
