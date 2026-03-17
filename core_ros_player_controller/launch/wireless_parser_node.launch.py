import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory("core_ros_player_controller"),
        "config",
        "wireless_parser_params.yaml",
    )

    wireless_arg = DeclareLaunchArgument(
        "wireless",
        default_value="/wireless",
        description="Input topic to remap node subscription target /wireless",
    )
    rotation_arg = DeclareLaunchArgument(
        "rotation",
        default_value="/rotation",
        description="Output topic to remap /rotation",
    )
    ads_arg = DeclareLaunchArgument(
        "ads",
        default_value="/ads",
        description="Output topic to remap /ads",
    )
    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel",
        default_value="/cmd_vel",
        description="Output topic to remap /cmd_vel",
    )
    manual_mode_arg = DeclareLaunchArgument(
        "manual_mode",
        default_value="/manual_mode",
        description="Output topic to remap /manual_mode",
    )
    manual_pitch_arg = DeclareLaunchArgument(
        "manual_pitch",
        default_value="/manual_pitch",
        description="Output topic to remap /manual_pitch",
    )
    shoot_motor_arg = DeclareLaunchArgument(
        "shoot_motor",
        default_value="/shoot_motor",
        description="Output topic to remap /shoot_motor",
    )
    left_shoot_once_arg = DeclareLaunchArgument(
        "left_shoot_once",
        default_value="/left/shoot_once",
        description="Output topic to remap /left/shoot_once",
    )
    reloading_arg = DeclareLaunchArgument(
        "reloading",
        default_value="/reloading",
        description="Output topic to remap /reloading",
    )
    hazard_status_arg = DeclareLaunchArgument(
        "hazard_status",
        default_value="/system/emergency/hazard_status",
        description="Output topic to remap /system/emergency/hazard_status",
    )
    test_mode_arg = DeclareLaunchArgument(
        "test_mode",
        default_value="/test_mode",
        description="Output topic to remap /test_mode",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_param_file,
        description="Path to the parameter file",
    )

    node = Node(
        package="core_ros_player_controller",
        executable="wireless_parser_node",
        name="wireless_parser_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            ("/wireless", LaunchConfiguration("wireless")),
            ("/rotation", LaunchConfiguration("rotation")),
            ("/ads", LaunchConfiguration("ads")),
            ("/cmd_vel", LaunchConfiguration("cmd_vel")),
            ("/manual_mode", LaunchConfiguration("manual_mode")),
            ("/manual_pitch", LaunchConfiguration("manual_pitch")),
            ("/shoot_motor", LaunchConfiguration("shoot_motor")),
            ("/left/shoot_once", LaunchConfiguration("left_shoot_once")),
            ("/reloading", LaunchConfiguration("reloading")),
            ("/system/emergency/hazard_status", LaunchConfiguration("hazard_status")),
            ("/test_mode", LaunchConfiguration("test_mode")),
        ],
    )

    return LaunchDescription([
        wireless_arg,
        rotation_arg,
        ads_arg,
        cmd_vel_arg,
        manual_mode_arg,
        manual_pitch_arg,
        shoot_motor_arg,
        left_shoot_once_arg,
        reloading_arg,
        hazard_status_arg,
        test_mode_arg,
        params_file_arg,
        node,
    ])
