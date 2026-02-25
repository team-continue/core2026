from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
	body_control = Node(
		package="core_body_controller",
		executable="body_control_node",
		name="body_control_node",
		output="screen",
	)

	target_angle = Node(
		package="core_body_controller",
		executable="target_angle_node",
		name="target_angle_node",
		output="screen",
	)

	return LaunchDescription([
		body_control,
		target_angle,
	])
