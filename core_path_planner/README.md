# Core Path Planner

A ROS 2 path planning package that implements A* algorithm for global path planning with local costmap integration.

## Overview

This package provides a path planner node that:
- Uses A* algorithm for path finding on occupancy grids
- Supports diagonal movement (configurable)
- Integrates local costmap for dynamic obstacle avoidance
- Publishes planned paths as `nav_msgs/Path`

## Node: `path_planner_node`

### Subscribed Topics

| Topic            | Type                        | Description                          |
| ---------------- | --------------------------- | ------------------------------------ |
| `/map`           | `nav_msgs/OccupancyGrid`    | Global map for path planning         |
| `/local_costmap` | `nav_msgs/OccupancyGrid`    | Local costmap for obstacle avoidance |
| `/start_pose`    | `geometry_msgs/PoseStamped` | Start pose for planning              |
| `/goal_pose`     | `geometry_msgs/PoseStamped` | Goal pose for planning               |

### Published Topics

| Topic           | Type            | Description                     |
| --------------- | --------------- | ------------------------------- |
| `/planned_path` | `nav_msgs/Path` | Planned path from start to goal |

### Parameters

| Parameter                   | Type   | Default          | Description                                                                                                                       |
| --------------------------- | ------ | ---------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `global_map_topic`          | string | `/map`           | Topic name for global map                                                                                                          |
| `local_costmap_topic`       | string | `/local_costmap` | Topic name for local costmap                                                                                                       |
| `start_topic`               | string | `/start_pose`    | Topic name for start pose                                                                                                          |
| `goal_topic`                | string | `/goal_pose`     | Topic name for goal pose                                                                                                           |
| `path_topic`                | string | `/planned_path`  | Topic name for output path                                                                                                         |
| `local_frame_id`            | string | `chassis_link`   | Frame id of the published path when `publish_in_global_frame` is false                                                             |
| `publish_in_global_frame`   | bool   | `false`          | Publish the path in the global frame instead of the robot-local frame                                                              |
| `global_frame_id`           | string | `odom`           | Frame id of the published path when `publish_in_global_frame` is true                                                              |
| `occupied_threshold`        | int    | `50`             | Threshold for considering a cell occupied (0-100)                                                                                  |
| `allow_unknown`             | bool   | `true`           | Whether to allow planning through unknown cells                                                                                    |
| `use_diagonal`              | bool   | `true`           | Whether to allow diagonal movement                                                                                                 |
| `cost_weight`               | double | `0.0`            | Weight of cell cost values added to step costs (0 disables cost-aware planning)                                                    |
| `search_window_margin`      | int    | `-1`             | Cells added around the start-goal bounding box for a windowed first search attempt. Falls back to a full-map search only when the window contains no path, so windowed results may be costlier than the global optimum. Negative disables windowing (always globally optimal) |
| `replan_rate_hz`            | double | `5.0`            | Rate of the replan timer; planning runs at most this often                                                                         |
| `replan_position_tolerance` | double | `0.0`            | Start-pose translation (m) beyond which a replan is requested (compared against the start pose of the last successful plan)        |
| `replan_yaw_tolerance`      | double | `0.0`            | Start-pose yaw change (rad) beyond which a replan is requested                                                                     |

### Replanning behavior

Planning is decoupled from topic rates: callbacks only record inputs and set
a dirty flag, and a wall timer (`replan_rate_hz`) runs A* only when an input
actually changed since the last plan. Receiving a map, goal, or local
costmap always requests a replan; a start pose requests one only when it
moved beyond the tolerances relative to the last successful plan. The last
planned path is additionally re-transformed and republished on every start
pose message (cheap, no search), so local-frame consumers always see the
path relative to the current robot pose even between replans.

## Usage

```bash
ros2 run core_path_planner path_planner_node
```

With parameters:
```bash
ros2 run core_path_planner path_planner_node --ros-args \
  -p global_map_topic:=/custom_map \
  -p occupied_threshold:=60 \
  -p use_diagonal:=false
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    PathPlannerNode                          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                   PathPlanner                        │   │
│  │  - A* Algorithm                                      │   │
│  │  - Grid/World coordinate conversion                  │   │
│  │  - Local costmap integration                         │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
        ▲                ▲               ▲           │
        │                │               │           │
   /map (global)   /local_costmap   /start_pose, /goal_pose   │
                                                     ▼
                                              /planned_path
```

## Dependencies

- rclcpp
- geometry_msgs
- nav_msgs
- std_msgs
