from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パラメータファイルのパスを取得
    shooter_params = os.path.join(
        get_package_share_directory("core_shooter"),
        "config",
        "shooter.params.yaml"
    )

    shoot_controller_node = Node(
        package="core_shooter",
        executable="shoot_controller",
        name="shoot_controller",
        output="screen",
        parameters=[shooter_params]
    )

    return LaunchDescription([
        shoot_controller_node
    ])
