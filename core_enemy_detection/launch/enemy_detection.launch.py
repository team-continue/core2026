from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('core_enemy_detection')
    target_detector_param_file = os.path.join(pkg_dir, 'launch', 'target_detector_parameter.yaml')
    return LaunchDescription([
        # target_detector ノード
        Node(
            package='core_enemy_detection',
            executable='target_detector',
            name='target_detector',
            output='screen',
            parameters=[target_detector_param_file],
            remappings=[
                ('/raw_image', '/image_input'),        # 入力トピックを remap
            ]
        ),
        # target_selector ノード
        Node(
            package='core_enemy_detection',
            executable='target_selector',
            name='target_selector',
            output='screen',
            remappings=[
                ('/damage_panel_pose', '/target_pose')
            ]
        ),
    ])