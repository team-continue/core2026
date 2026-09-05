from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # パラメータ YAML のフルパス
    pkg_dir = get_package_share_directory('core_enemy_detection')
    param_file = os.path.join(pkg_dir, 'config', 'sim_param2.yaml')

    # =========================================================
    # OAK-D 系
    # oakd_panel_localizer
    #       ↓ detected_panel_info
    # oakd_target_detector
    #       ↓ damage_panels_infomation
    # target_selector
    # =========================================================

    oakd_panel_localizer = Node(
        package='core_enemy_detection',
        executable='oakd_panel_localizer',
        namespace='oakd',
        name='oakd_panel_localizer',
        output='screen',
        remappings=[
            ('detected_panel_info', '/oakd/detected_panel_info'),
        ],
    )

    oakd_target_detector = Node(
        package='core_enemy_detection',
        executable='oakd_target_detector',
        namespace='oakd',
        name='oakd_target_detector',
        parameters=[param_file],
        output='screen',
        remappings=[
            ('detected_panel_info', '/oakd/detected_panel_info'),
            ('color', 'color'),
            ('damage_panels_infomation', '/oakd/damage_panels_infomation'),
        ],
    )

    oakd_target_selector = Node(
        package='core_enemy_detection',
        executable='target_selector',
        namespace='oakd',
        name='oakd_target_selector',
        output='screen',
        remappings=[
            ('damage_panels_infomation', '/oakd/damage_panels_infomation'),
            ('damage_panel_pose', '/right/target_pose'),
        ],
    )


    # =========================================================
    # 従来の target_detector 系
    # target_detector
    #       ↓
    # target_selector
    # =========================================================

    target_detector = Node(
        package='core_enemy_detection',
        executable='target_detector',
        namespace='normal',
        name='target_detector',
        parameters=[param_file],
        output='screen',
        remappings=[
            ('raw_image', '/turret_camera_right/color/image'),
            ('target_pose', '/normal/detected_target'),
        ],
    )

    target_selector = Node(
        package='core_enemy_detection',
        executable='target_selector',
        namespace='normal',
        name='target_selector',
        output='screen',
        remappings=[
            ('target_pose', '/normal/detected_target'),
            ('damage_panel_pose', '/left/target_pose'),
        ],
    )


    return LaunchDescription([
        # OAK-D
        oakd_panel_localizer,
        oakd_target_detector,
        oakd_target_selector,

        # 従来方式
        target_detector,
        target_selector,
    ])