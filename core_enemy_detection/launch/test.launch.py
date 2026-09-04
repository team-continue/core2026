from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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
        output='screen',
        remappings=[
            ('detected_panel_info', '/oakd/detected_panel_info'),
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
            ('target_pose', '/oakd/target_pose'),
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
        output='screen',
        remappings=[
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
            ('selected_target', '/normal/selected_target'),
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