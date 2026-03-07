from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # target_detector ノード
        Node(
            package='core_enemy_detection',
            executable='target_detector',
            name='target_detector',
            output='screen',
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