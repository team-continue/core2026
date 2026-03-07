from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # LaunchArgument で砲塔名と入力トピック名を外部から受け取る
    turret_name_arg = DeclareLaunchArgument(
        'turret_name',
        default_value='turret',
        description='Name of the turret (used for namespace and topics)'
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='image_input',
        description='Input camera topic for this turret'
    )
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='target_pose',
        description='Targete Pose topic for this turret'
    )

    turret_name = LaunchConfiguration('turret_name')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    # パラメータ YAML のフルパス
    pkg_dir = get_package_share_directory('core_enemy_detection')
    param_file = os.path.join(pkg_dir, 'config', 'target_detector_parameter.yaml')

    return LaunchDescription([
        turret_name_arg,
        input_topic_arg,
        output_topic_arg,

        # target_detector
        Node(
            package='core_enemy_detection',
            executable='target_detector',
            name=['target_detector'],
            namespace=turret_name,
            output='screen',
            parameters=[param_file],
            remappings=[
                ('raw_image/compressed', input_topic),
            ]
        ),

        # target_selector
        Node(
            package='core_enemy_detection',
            executable='target_selector',
            name=['target_selector'],
            namespace=turret_name,
            output='screen',
            remappings=[
                ('damage_panel_pose', output_topic),
            ]
        ),
    ])