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

| Parameter             | Type   | Default          | Description                                       |
| --------------------- | ------ | ---------------- | ------------------------------------------------- |
| `global_map_topic`    | string | `/map`           | Topic name for global map                         |
| `local_costmap_topic` | string | `/local_costmap` | Topic name for local costmap                      |
| `start_topic`         | string | `/start_pose`    | Topic name for start pose                         |
| `goal_topic`          | string | `/goal_pose`     | Topic name for goal pose                          |
| `path_topic`          | string | `/planned_path`  | Topic name for output path                        |
| `occupied_threshold`  | int    | `50`             | Threshold for considering a cell occupied (0-100) |
| `allow_unknown`       | bool   | `false`          | Whether to allow planning through unknown cells   |
| `use_diagonal`        | bool   | `true`           | Whether to allow diagonal movement                |

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
