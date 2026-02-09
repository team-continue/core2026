# core_path_follower

Follows a `nav_msgs/Path` from `core_path_planner` (chassis_link frame) and publishes `geometry_msgs/Twist` on `cmd_vel`.
`core_body_controller` consumes `cmd_vel` to drive the mecanum base.

## Data flow

```
core_path_planner          core_path_follower          core_body_controller
 /planned_path  ──────────▶  PathFollowerNode  ────────▶  BodyControlNode
 (chassis_link)              └── Controller              cmd_vel (Twist)
                  /odom ────────┘    │                    → CAN commands
                                     ▼
                              /goal_reached (Bool)
```

## Package structure

```
core_path_follower/
├── include/core_path_follower/
│   ├── controller.hpp          # Control algorithm (no ROS dependency)
│   ├── interpolation.hpp       # Catmull-Rom / Bézier interpolation
│   ├── path_follower_node.hpp  # ROS 2 node
│   └── pid.hpp                 # PID controller
├── src/
│   ├── controller.cpp
│   ├── interpolation.cpp
│   ├── node.cpp                # main()
│   └── path_follower_node.cpp
├── launch/
│   └── path_follower.launch.py
├── param/
│   └── default_params.yaml
└── schema/
    └── core_path_follower.schema.json
```

## Coordinate frame

| Mode | `use_local_frame` | Frame | Use case |
|:-----|:------------------|:------|:---------|
| Local (recommended) | `true` | chassis_link (robot origin) | `core_path_planner` |
| World | `false` | map / odom | nav2-style planners |

In local mode the robot is always at (0,0); odom is used only for angular velocity feedback.

## Controller types

### cascade (recommended)

Outer PID: heading error → $\omega_{\text{ref}}$, Inner PID: $\omega_{\text{ref}} - \omega_{\text{current}}$ → $\omega_{\text{cmd}}$

### pid

Single PID: heading error → $\omega_{\text{cmd}}$

### pure_pursuit

$$\omega_{\text{ref}} = k_{\text{pure}} \cdot \frac{2v\sin\alpha}{L}$$

Inner PID tracks $\omega_{\text{ref}}$.

## Interpolation

Smooths jagged A* paths before tracking.

| Method | Description |
|:-------|:------------|
| `none` | Use path as-is |
| `spline` | Catmull-Rom spline |
| `bezier` | Global Bézier curve |

## Goal detection

Stops and publishes `true` on `/goal_reached` when distance to the last waypoint < `goal_tolerance`.

## Topics

### Input

| Topic | Type | Description |
|:------|:-----|:------------|
| `/planned_path` | `nav_msgs/Path` | Path to follow (chassis_link) |
| `/odom` | `nav_msgs/Odometry` | Robot pose & angular velocity |

### Output

| Topic | Type | Description |
|:------|:-----|:------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command (linear.x, linear.y, angular.z) |
| `/goal_reached` | `std_msgs/Bool` | Goal reached flag |

## Parameters

Defaults in `param/default_params.yaml`. Schema in `schema/core_path_follower.schema.json`.

| Parameter | Type | Default | Description |
|:----------|:-----|:--------|:------------|
| `path_topic` | string | `/planned_path` | Path topic |
| `odom_topic` | string | `/odom` | Odom topic |
| `cmd_vel_topic` | string | `/cmd_vel` | Cmd vel topic |
| `use_local_frame` | bool | `true` | Local frame mode |
| `goal_tolerance` | double | `0.15` | Goal distance threshold [m] |
| `reset_on_new_path` | bool | `true` | Reset PID on new path |
| `controller_type` | string | `cascade` | Controller type |
| `linear_speed` | double | `0.6` | Forward speed [m/s] |
| `lookahead_dist` | double | `0.5` | Lookahead distance [m] |
| `control_rate` | double | `20.0` | Control frequency [Hz] |
| `outer_kp/ki/kd` | double | `1.2/0.0/0.15` | Outer PID gains |
| `inner_kp/ki/kd` | double | `2.0/0.0/0.05` | Inner PID gains |
| `pure_k` | double | `1.0` | Pure pursuit gain |
| `interpolation` | string | `spline` | Interpolation method |
| `spline_samples_per_segment` | int | `10` | Spline samples per segment |
| `bezier_samples` | int | `100` | Bézier total samples |

## Usage

```bash
colcon build --packages-select core_path_follower
source install/setup.bash
ros2 launch core_path_follower path_follower.launch.py
```

## Notes

- Local mode assumes path is in chassis_link frame
- Output Twist is for omnidirectional base (linear.x, linear.y, angular.z)
- Obstacle avoidance is the planner's responsibility
