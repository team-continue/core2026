from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    acceleration_arg = DeclareLaunchArgument(
        "acceleration",
        default_value="2.0",
        description="Linear acceleration limit [m/s^2]",
    )
    rotation_acceleration_arg = DeclareLaunchArgument(
        "rotation_acceleration",
        default_value="1.0",
        description="Angular acceleration limit [rad/s^2]",
    )
    auto_rotation_velocity_arg = DeclareLaunchArgument(
        "auto_rotation_velocity",
        default_value="-3.141592653589793",
        description="Auto rotation angular velocity [rad/s]",
    )
    high_rotation_velocity_arg = DeclareLaunchArgument(
        "high_rotation_velocity",
        default_value="-6.28",
        description="High rotation angular velocity [rad/s]",
    )
    yaw_rotation_velocity_arg = DeclareLaunchArgument(
        "yaw_rotation_velocity",
        default_value="6.28",
        description="Max yaw angular velocity [rad/s]",
    )
    yaw_rotation_acceleration_arg = DeclareLaunchArgument(
        "yaw_rotation_acceleration",
        default_value="9.42477796076938",
        description="Max yaw angular acceleration [rad/s^2]",
    )
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        "cmd_vel_timeout_sec",
        default_value="0.2",
        description="Maximum accepted age of cmd_vel [s]",
    )
    acceleration_in_world_frame_arg = DeclareLaunchArgument(
        "acceleration_in_world_frame",
        default_value="false",
        description=(
            "Apply the linear acceleration limit in the world frame using the upper-body "
            "IMU (/sensing/imu). Keeps TURRET turns and stops from lagging while the upper "
            "body rotates"
        ),
    )
    imu_timeout_arg = DeclareLaunchArgument(
        "imu_timeout_sec",
        default_value="0.2",
        description="Maximum accepted age of IMU data [s]",
    )
    body_omega_timeout_arg = DeclareLaunchArgument(
        "body_omega_timeout_sec",
        default_value="0.2",
        description="Maximum accepted age of body_omega feedforward [s]",
    )
    # sim（core_gazebo の sim_nav.launch.py の waist:=true）から使うときの設定。実機の既定は変えない
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock (sim only)",
    )
    use_control_mode_manager_arg = DeclareLaunchArgument(
        "use_control_mode_manager",
        default_value="true",
        description="Launch control_mode_manager (false when the caller launches its own)",
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    waist_front_offset_arg = DeclareLaunchArgument(
        "waist_front_offset",
        default_value="3.7822790145874023",
        description="Waist motor position when the upper body faces the chassis front [rad]",
    )
    waist_joint_direction_arg = DeclareLaunchArgument(
        "waist_joint_direction",
        default_value="1.0",
        description="1.0 if a positive waist motor position turns the upper body to positive yaw, "
        "otherwise -1.0",
    )
    yaw_mode_arg = DeclareLaunchArgument(
        "yaw_mode",
        default_value="1",
        description="Initial waist yaw mode (0: front, 1: turret); "
        "switch at runtime with /control/yaw_mode",
    )
    use_damiao_imu_arg = DeclareLaunchArgument(
        "use_damiao_imu",
        default_value="true",
        description="Launch the DM-IMU-L1 USB driver",
    )
    imu_port_arg = DeclareLaunchArgument(
        "imu_port",
        default_value=(
            "/dev/serial/by-id/"
            "usb-DM-IMU_DM-IMU_USB_CDC_2025021200-if00"
        ),
        description="DM-IMU-L1 USB serial device",
    )
    imu_baudrate_arg = DeclareLaunchArgument(
        "imu_baudrate",
        default_value="921600",
        description="DM-IMU-L1 USB serial baud rate",
    )
    imu_frame_id_arg = DeclareLaunchArgument(
        "imu_frame_id",
        default_value="damiao_imu_link",
        description="Frame ID assigned to /imu messages",
    )
    imu_output_rate_arg = DeclareLaunchArgument(
        "imu_output_rate_hz",
        default_value="200",
        description="DM-IMU-L1 output rate [Hz]",
    )

    damiao_imu = Node(
        package="core_damiao_imu",
        executable="damiao_imu_node",
        name="damiao_imu_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_damiao_imu")),
        remappings=[("imu", "/sensing/imu")],
        parameters=[
            {
                "port": LaunchConfiguration("imu_port"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("imu_baudrate"), value_type=int
                ),
                "frame_id": LaunchConfiguration("imu_frame_id"),
                "output_rate_hz": ParameterValue(
                    LaunchConfiguration("imu_output_rate_hz"), value_type=int
                ),
            }
        ],
    )

    body_control = Node(
        package="core_body_controller",
        executable="body_control_node",
        name="body_control_node",
        output="screen",
        remappings=[
            ("can/tx", "/hardware/can/tx"),
            ("joint_states", "/hardware/joint_states"),
            ("/rotation", "/control/rotation"),
            ("imu", "/sensing/imu"),
        ],
        parameters=[
            {
                "acceleration": LaunchConfiguration("acceleration"),
                "acceleration_in_world_frame": ParameterValue(
                    LaunchConfiguration("acceleration_in_world_frame"), value_type=bool
                ),
                "imu_timeout_sec": LaunchConfiguration("imu_timeout_sec"),
                "rotation_acceleration": LaunchConfiguration("rotation_acceleration"),
                "auto_rotation_velocity": LaunchConfiguration("auto_rotation_velocity"),
                "high_rotation_velocity": LaunchConfiguration("high_rotation_velocity"),
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                "use_sim_time": use_sim_time,
                "waist_front_offset": ParameterValue(
                    LaunchConfiguration("waist_front_offset"), value_type=float
                ),
                "waist_joint_direction": ParameterValue(
                    LaunchConfiguration("waist_joint_direction"), value_type=float
                ),
            }
        ],
    )

    target_angle = Node(
        package="core_body_controller",
        executable="target_angle_node",
        name="target_angle_node",
        output="screen",
        remappings=[
            ("imu", "/sensing/imu"),
            ("can/tx", "/hardware/can/tx"),
            ("/rotation", "/control/rotation"),
            ("joint_states", "/hardware/joint_states"),
        ],
        parameters=[
            {
                "yaw_rotation_velocity": LaunchConfiguration("yaw_rotation_velocity"),
                "yaw_rotation_acceleration": LaunchConfiguration(
                    "yaw_rotation_acceleration"
                ),
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                "imu_timeout_sec": LaunchConfiguration("imu_timeout_sec"),
                "body_omega_timeout_sec": LaunchConfiguration(
                    "body_omega_timeout_sec"
                ),
                "use_sim_time": use_sim_time,
                "yaw_mode": ParameterValue(
                    LaunchConfiguration("yaw_mode"), value_type=int
                ),
                "waist_front_offset": ParameterValue(
                    LaunchConfiguration("waist_front_offset"), value_type=float
                ),
                "waist_joint_direction": ParameterValue(
                    LaunchConfiguration("waist_joint_direction"), value_type=float
                ),
            }
        ],
    )

    # 手動（/control/manual/cmd_vel）と自動（/control/auto/cmd_vel）のどちらを
    # /control/cmd_vel へ通すかを決める。UI の auto で自動に入り、到達・失敗・停止で手動に戻る
    # 腰の角度に関節名を付けて /control/chassis/joint_states に出す（robot_state_publisher が読む）
    chassis_joint_state = Node(
        package="core_body_controller",
        executable="chassis_joint_state_node",
        name="chassis_joint_state",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "waist_front_offset": ParameterValue(
                    LaunchConfiguration("waist_front_offset"), value_type=float
                ),
                "waist_joint_direction": ParameterValue(
                    LaunchConfiguration("waist_joint_direction"), value_type=float
                ),
            }
        ],
    )

    control_mode_manager = Node(
        package="core_body_controller",
        executable="control_mode_manager_node",
        name="control_mode_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_control_mode_manager")),
    )

    return LaunchDescription(
        [
            acceleration_arg,
            rotation_acceleration_arg,
            yaw_rotation_velocity_arg,
            yaw_rotation_acceleration_arg,
            auto_rotation_velocity_arg,
            high_rotation_velocity_arg,
            cmd_vel_timeout_arg,
            acceleration_in_world_frame_arg,
            imu_timeout_arg,
            body_omega_timeout_arg,
            use_sim_time_arg,
            use_control_mode_manager_arg,
            waist_front_offset_arg,
            waist_joint_direction_arg,
            yaw_mode_arg,
            use_damiao_imu_arg,
            imu_port_arg,
            imu_baudrate_arg,
            imu_frame_id_arg,
            imu_output_rate_arg,
            GroupAction([
                PushRosNamespace("sensing"),
                damiao_imu,
            ]),
            GroupAction([
                PushRosNamespace("control"),
                body_control,
                target_angle,
                control_mode_manager,
                chassis_joint_state,
            ]),
        ]
    )
