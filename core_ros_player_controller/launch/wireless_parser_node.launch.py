import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory("core_ros_player_controller"),
        "config",
        "wireless_parser_params.yaml",
    )

    wireless_arg = DeclareLaunchArgument(
        "wireless",
        default_value="/hardware/wireless",
        description="Input topic to remap node subscription target /wireless",
    )
    rotation_arg = DeclareLaunchArgument(
        "rotation",
        default_value="/control/rotation",
        description="Output topic to remap /rotation",
    )
    ads_arg = DeclareLaunchArgument(
        "ads",
        default_value="/ui/ads",
        description="Output topic to remap /ads",
    )
    left_turret_auto_arg = DeclareLaunchArgument(
        "left_turret_auto",
        default_value="/ui/left/turret_auto",
        description="Output topic to remap /left/turret_auto",
    )
    right_turret_auto_arg = DeclareLaunchArgument(
        "right_turret_auto",
        default_value="/ui/right/turret_auto",
        description="Output topic to remap /right/turret_auto",
    )
    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel",
        default_value="/control/cmd_vel",
        description="Output topic to remap /cmd_vel",
    )
    manual_mode_arg = DeclareLaunchArgument(
        "manual_mode",
        default_value="/ui/manual_mode",
        description="Output topic to remap /manual_mode",
    )
    manual_pitch_arg = DeclareLaunchArgument(
        "manual_pitch",
        default_value="/ui/manual_pitch",
        description="Output topic to remap /manual_pitch",
    )
    shoot_motor_arg = DeclareLaunchArgument(
        "shoot_motor",
        default_value="/ui/shoot_motor_state",
        description="Output topic to remap /shoot_motor",
    )
    left_shoot_fullauto_arg = DeclareLaunchArgument(
        "left_shoot_fullauto",
        default_value="/mecha/shooter/left/shoot_fullauto",
        description="Output topic to remap /left/shoot_fullauto",
    )
    right_shoot_fullauto_arg = DeclareLaunchArgument(
        "right_shoot_fullauto",
        default_value="/mecha/shooter/right/shoot_fullauto",
        description="Output topic to remap /right/shoot_fullauto",
    )
    reloading_arg = DeclareLaunchArgument(
        "reloading",
        default_value="/ui/reloading",
        description="Output topic to remap /reloading",
    )
    auto_point_select_arg = DeclareLaunchArgument(
        "auto_point_select",
        default_value="/ui/auto_point_select",
        description="Output topic to remap /auto_point_select",
    )
    selected_pose_arg = DeclareLaunchArgument(
        "selected_pose",
        default_value="/ui/selected_pose",
        description="Output topic to remap /selected_pose",
    )
    hazard_status_arg = DeclareLaunchArgument(
        "hazard_status",
        default_value="/system/emergency/software_emergency",
        description="Output topic to remap /system/emergency/hazard_status"
    )
    test_mode_arg = DeclareLaunchArgument(
        "test_mode",
        default_value="/ui/test_mode",
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
            ("/left/turret_auto", LaunchConfiguration("left_turret_auto")),
            ("/right/turret_auto", LaunchConfiguration("right_turret_auto")),
            ("/cmd_vel", LaunchConfiguration("cmd_vel")),
            ("/manual_mode", LaunchConfiguration("manual_mode")),
            ("/manual_pitch", LaunchConfiguration("manual_pitch")),
            ("/shoot_motor", LaunchConfiguration("shoot_motor")),
            ("/left/shoot_fullauto", LaunchConfiguration("left_shoot_fullauto")),
            ("/right/shoot_fullauto", LaunchConfiguration("right_shoot_fullauto")),
            ("/reloading", LaunchConfiguration("reloading")),
            ("/auto_point_select", LaunchConfiguration("auto_point_select")),
            ("/selected_pose", LaunchConfiguration("selected_pose")),
            ("/system/emergency/hazard_status", LaunchConfiguration("hazard_status")),
            ("/test_mode", LaunchConfiguration("test_mode")),
        ],
    )

    return LaunchDescription([
        wireless_arg,
        rotation_arg,
        ads_arg,
        left_turret_auto_arg,
        right_turret_auto_arg,
        cmd_vel_arg,
        manual_mode_arg,
        manual_pitch_arg,
        shoot_motor_arg,
        left_shoot_fullauto_arg,
        right_shoot_fullauto_arg,
        reloading_arg,
        auto_point_select_arg,
        selected_pose_arg,
        hazard_status_arg,
        test_mode_arg,
        params_file_arg,
        GroupAction([
            PushRosNamespace("ui"),
            node,
        ]),
    ])
