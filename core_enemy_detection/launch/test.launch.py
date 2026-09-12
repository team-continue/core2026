from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # パラメータ YAML のフルパス
    pkg_dir = get_package_share_directory('core_enemy_detection')
    param_file = os.path.join(pkg_dir, 'config', 'sim_param2.yaml')

    # =========================================================
    # OAK-D 系（right 砲塔向け）
    # oakd_panel_localizer
    #       ↓ detected_panel_info
    # oakd_target_detector
    #       ↓ damage_panels_infomation
    # target_selector
    # =========================================================
    RIGHT_NS = 'perception/enemy_detection/right'

    oakd_panel_localizer = Node(
        package='core_enemy_detection',
        executable='oakd_panel_localizer',
        namespace=RIGHT_NS,
        name='oakd_panel_localizer',
        output='screen',
    )

    oakd_target_detector = Node(
        package='core_enemy_detection',
        executable='oakd_target_detector',
        namespace=RIGHT_NS,
        name='oakd_target_detector',
        parameters=[param_file],
        output='screen',
        remappings=[
            ('color', '/hardware/color'),
        ],
    )

    oakd_target_selector = Node(
        package='core_enemy_detection',
        executable='target_selector',
        namespace=RIGHT_NS,
        name='oakd_target_selector',
        output='screen',
        remappings=[
            ('damage_panel_pose', 'target_pose'),
        ],
    )


    # =========================================================
    # 従来の target_detector 系（left 砲塔向け）
    # target_detector
    #       ↓
    # target_selector
    # =========================================================
    LEFT_NS = 'perception/enemy_detection/left'

    target_detector = Node(
        package='core_enemy_detection',
        executable='target_detector',
        namespace=LEFT_NS,
        name='target_detector',
        parameters=[param_file],
        output='screen',
        remappings=[
            ('raw_image', '/turret_camera_left/color/image'),
            ('color', '/hardware/color'),
        ],
    )

    target_selector = Node(
        package='core_enemy_detection',
        executable='target_selector',
        namespace=LEFT_NS,
        name='target_selector',
        output='screen',
        remappings=[
            ('damage_panel_pose', 'target_pose'),
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
