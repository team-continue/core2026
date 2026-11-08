"""Isolated waist and drive command inputs for bench commissioning."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    common = [
        ('can/tx', '/hardware/can/tx'),
        ('/rotation', '/sysid/unused_rotation'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'velocity_mode', default_value='true', choices=['true', 'false'],
            description='Use waist speed input; false holds upper-body world yaw for validation.'),
        DeclareLaunchArgument(
            'waist_velocity_limit', default_value='0.35',
            description='Waist test speed ceiling in rad/s; up to 6.28 for full-turn trials.'),
        Node(package='core_body_controller', executable='body_control_node',
             name='sysid_body', namespace='control', output='screen',
             remappings=common + [('cmd_vel', '/sysid/drive'),
                                  ('joint_states', '/hardware/joint_states')]),
        Node(package='core_body_controller', executable='target_angle_node',
             name='sysid_yaw', namespace='control', output='screen',
             parameters=[{'sysid_velocity_mode': ParameterValue(
                 LaunchConfiguration('velocity_mode'), value_type=bool),
                 'sysid_velocity_limit': ParameterValue(
                     LaunchConfiguration('waist_velocity_limit'), value_type=float)}],
             remappings=common + [('cmd_vel', '/sysid/unused_twist'),
                                  ('yaw_target_angle', '/sysid/unused_reference'),
                                  ('imu', '/sensing/imu'),
                                  ('sysid_velocity', '/sysid/waist_velocity')]),
    ])
