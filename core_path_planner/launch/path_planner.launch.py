from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    path_planner_node = Node(
        package="core_path_planner",
        executable="path_planner_node",
        name="core_path_planner_node",
        output="screen",
        parameters=[
            {
                "goal_topic": "/behavior/goal_pose",
                "start_topic": "/localization/start_pose",
                "path_topic": "planned_path",
                "local_costmap_topic": "costmap/local",
                "global_map_topic": "/map",
            }
        ],
    )

    return LaunchDescription([
        GroupAction([
            PushRosNamespace("planning"),
            path_planner_node,
        ]),
    ])
