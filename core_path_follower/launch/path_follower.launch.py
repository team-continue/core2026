from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(__file__), '..')
    param_file = os.path.join(pkg_share, 'config', 'default_params.yaml')

    node = Node(
        package='core_path_follower',
        executable='path_follower_node',
        name='path_follower',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([node])
