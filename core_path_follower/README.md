# core_path_follower

## Purpose / Use cases

The `core_path_follower` node receives a `nav_msgs/msg/Path` (typically from `core_path_planner`) and generates velocity commands (`geometry_msgs/msg/Twist`) to make the robot follow the planned path. It supports both **local-frame** (robot-relative) and **world-frame** (map/odom) path inputs, and provides three controller modes: cascade PID, single PID, and pure pursuit.

## Design / Inner-workings / Algorithms

### Architecture

```
core_path_planner                    core_path_follower
┌────────────────┐  /planned_path   ┌───────────────────┐  /cmd_vel
│  A* Planner    │─────────────────▶│  Path Follower    │──────────▶ Robot
│ (chassis_link) │  nav_msgs/Path   │  (cascade / pid / │
└────────────────┘                  │   pure_pursuit)   │
                                    └───────────────────┘
                  /odom                      │
         ┌─────────────────┐                 ▼ /goal_reached
         │  Odometry Source │            std_msgs/Bool
         └─────────────────┘
```

### Coordinate frame modes

#### Local-frame mode (`use_local_frame: true`) — recommended

Designed for direct integration with `core_path_planner`. The path is expressed in the robot body frame (`chassis_link`).

- The robot position is treated as the origin `(0, 0)` at all times.
- Heading error is computed directly as `atan2(target_y, target_x)`.
- On each new path received, the tracking index resets to `0`.
- Odometry is used only for angular velocity feedback (cascade / pure_pursuit controllers).

#### World-frame mode (`use_local_frame: false`)

For use when the path is expressed in a global frame (`map` or `odom`).

- The robot's current pose is obtained from `/odom`.
- The closest point on the path is tracked and advanced forward.
- Compatible with `nav2`-style path planners.

### Controller types

#### cascade (recommended)

A two-loop PID controller.

1. **Outer loop**: Computes the heading error to the lookahead point and produces a desired angular velocity $\omega_{\text{ref}}$ via the outer PID.
2. **Inner loop**: Reads the current angular velocity from odometry and tracks $\omega_{\text{ref}}$ via the inner PID, producing the commanded angular velocity.
3. **Linear velocity**: Based on `linear_speed`, scaled down by a heading-error factor to prioritize rotation when the error is large.

$$\omega_{\text{ref}} = \text{PID}_{\text{outer}}(e_{\theta}), \quad \omega_{\text{cmd}} = \text{PID}_{\text{inner}}(\omega_{\text{ref}} - \omega_{\text{current}})$$

#### pid

A single PID controller that maps heading error directly to an angular velocity command (no inner/outer separation).

$$\omega_{\text{cmd}} = \text{PID}(e_{\theta})$$

#### pure_pursuit

Selects a lookahead point at distance $L$ and computes curvature in the body frame:

$$\omega_{\text{ref}} = k_{\text{pure}} \cdot \frac{2 \, v \, \sin(\alpha)}{L}$$

where $\alpha$ is the angle to the lookahead point in the body frame, and $v$ is the reference linear speed. The inner PID is used to track $\omega_{\text{ref}}$.

### Path interpolation

Grid-based planners (e.g. A\*) produce jagged waypoint sequences. The node can internally upsample these into a smooth, dense path before tracking.

| Method   | Description                                                                                  |
| :------- | :------------------------------------------------------------------------------------------- |
| `none`   | No interpolation. Use the path as-is.                                                        |
| `spline` | Catmull–Rom cubic spline. Inserts `spline_samples_per_segment` points between each waypoint. |
| `bezier` | Global Bézier curve through all waypoints. Generates `bezier_samples` total points.          |

### Goal detection

When the distance from the robot (or body-frame origin) to the last waypoint falls below `goal_tolerance`, the node:

1. Publishes a zero-velocity `Twist` to stop the robot.
2. Publishes `true` on the `/goal_reached` topic.
3. Ignores further control cycles until a new path is received.

---

## Inputs / Outputs / API

### Input

| Topic                       | Type                       | QoS        | Description                                                                                       |
| :-------------------------- | :------------------------- | :--------- | :------------------------------------------------------------------------------------------------ |
| `/planned_path` (remappable) | `nav_msgs/msg/Path`        | reliable/10 | Reference path to follow. From `core_path_planner` (local frame) or any global planner (world frame). |
| `/odom` (remappable)         | `nav_msgs/msg/Odometry`    | best_effort/50 | Robot pose & twist. Required for world-frame mode; angular velocity used in cascade / pure_pursuit.   |

### Output

| Topic            | Type                        | QoS         | Description                                                 |
| :--------------- | :-------------------------- | :---------- | :---------------------------------------------------------- |
| `/cmd_vel` (remappable) | `geometry_msgs/msg/Twist`   | reliable/10 | Velocity command (`linear.x`, `linear.y`, `angular.z`).     |
| `/goal_reached`  | `std_msgs/msg/Bool`         | reliable/10 | Publishes `true` when the robot reaches the final waypoint. |

---

## Parameters

Default values are defined in `config/default_params.yaml`.

### Topic configuration

| Name            | Type   | Description                              | Default value    |
| :-------------- | :----- | :--------------------------------------- | :--------------- |
| `path_topic`    | string | Subscribed path topic name               | `"/planned_path"` |
| `odom_topic`    | string | Subscribed odometry topic name           | `"/odom"`        |
| `cmd_vel_topic` | string | Published velocity command topic name    | `"/cmd_vel"`     |

### Frame & goal

| Name               | Type   | Description                                                                           | Default value |
| :----------------- | :----- | :------------------------------------------------------------------------------------ | :------------ |
| `use_local_frame`  | bool   | `true`: path is in robot-local frame (chassis_link). `false`: path is in world frame. | `true`        |
| `goal_tolerance`   | double | Distance to last waypoint at which the robot is considered to have reached the goal [m]. | `0.15`      |
| `reset_on_new_path`| bool   | Whether to reset PID integrators when a new path is received.                         | `true`        |

### Controller

| Name              | Type   | Description                                                   | Default value |
| :---------------- | :----- | :------------------------------------------------------------ | :------------ |
| `controller_type` | string | Controller mode: `"cascade"`, `"pid"`, or `"pure_pursuit"`.  | `"cascade"`   |
| `linear_speed`    | double | Reference forward speed [m/s].                                | `0.6`         |
| `lookahead_dist`  | double | Lookahead distance for target point selection [m].            | `0.5`         |
| `control_rate`    | double | Control loop frequency [Hz].                                  | `20.0`        |

### PID gains

| Name       | Type   | Description                                        | Default value |
| :--------- | :----- | :------------------------------------------------- | :------------ |
| `outer_kp` | double | Outer PID proportional gain (heading → ω_ref)      | `1.2`         |
| `outer_ki` | double | Outer PID integral gain                            | `0.0`         |
| `outer_kd` | double | Outer PID derivative gain                          | `0.15`        |
| `inner_kp` | double | Inner PID proportional gain (ω tracking)           | `2.0`         |
| `inner_ki` | double | Inner PID integral gain                            | `0.0`         |
| `inner_kd` | double | Inner PID derivative gain                          | `0.05`        |
| `pure_k`   | double | Pure pursuit curvature-to-angular-velocity gain    | `1.0`         |

### Interpolation

| Name                         | Type   | Description                                              | Default value |
| :--------------------------- | :----- | :------------------------------------------------------- | :------------ |
| `interpolation`              | string | Interpolation method: `"none"`, `"spline"`, or `"bezier"`. | `"spline"`  |
| `spline_samples_per_segment` | int    | Number of samples per segment for Catmull–Rom spline.    | `10`          |
| `bezier_samples`             | int    | Total number of samples for global Bézier curve.         | `100`         |

---

## Assumptions / Known limits

1. In local-frame mode, the path must be expressed in the robot body frame (`chassis_link`). The node assumes the robot is at the origin.
2. In world-frame mode, accurate odometry is required. Poor odometry will degrade tracking performance.
3. The path is assumed to be updated frequently by the planner. Stale paths may cause the robot to track outdated waypoints.
4. The output `Twist` assumes a holonomic or omnidirectional base (`linear.x`, `linear.y`, `angular.z`). For differential-drive robots, `linear.y` should be ignored downstream.
5. No obstacle avoidance is performed. Obstacle handling is the responsibility of the upstream planner.

---

## How to launch

```bash
# Build
colcon build --packages-select core_path_follower

# Source
source install/setup.bash

# Launch with default parameters
ros2 launch core_path_follower path_follower.launch.py
```

## Debugging

```bash
# Echo velocity commands
ros2 topic echo /cmd_vel

# Echo planned path
ros2 topic echo /planned_path

# Check goal reached status
ros2 topic echo /goal_reached

# Visualize in RViz: add a Path display with topic /planned_path
```

## Tuning tips

1. Start with `controller_type: cascade` and the default gains.
2. Increase `outer_kp` gradually until heading convergence is achieved without oscillation.
3. Tune `inner_kp` to match the robot's rotational dynamics; higher values improve responsiveness but may cause oscillation.
4. Increase `lookahead_dist` for smoother tracking; decrease for tighter corner following.
5. Set `goal_tolerance` according to the required stopping precision of the application.
6. For A\*-generated paths, `interpolation: spline` is recommended to smooth out grid artefacts.

---

## Package structure

```
core_path_follower/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── default_params.yaml
├── include/
│   └── core_path_follower/
│       ├── path_follower.hpp
│       ├── pid.hpp
│       └── interpolation.hpp
├── launch/
│   └── path_follower.launch.py
└── src/
    ├── path_follower_node.cpp
    └── interpolation.cpp
```

## Dependencies

| Package         | Purpose                           |
| :-------------- | :-------------------------------- |
| `rclcpp`        | ROS 2 C++ client library          |
| `nav_msgs`      | `Path`, `Odometry` message types  |
| `geometry_msgs` | `Twist`, `Pose` message types     |
| `std_msgs`      | `Bool` message type               |

## Future extensions / Unimplemented parts

- Dynamic parameter reconfiguration (on-the-fly PID tuning).
- Stanley controller mode.
- Velocity profiling along the path (decelerate before sharp turns).
- TF-based frame transformation (eliminate `use_local_frame` flag).

## Related issues

<!-- Link related GitHub issues here -->
