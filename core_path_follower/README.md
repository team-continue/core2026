# core_path_follower

## Purpose / Use cases

Tracks a `nav_msgs/Path` published by `core_path_planner` and outputs `geometry_msgs/Twist` for the mecanum base.
Designed for the `chassis_link`-local coordinate convention used throughout the core stack.

```
core_path_planner        core_path_follower        core_body_controller
 /planned_path ────────▶ PathFollowerNode ────────▶ BodyControlNode
 (chassis_link)           └── Controller             cmd_vel → CAN
               /odom ────────┘   │
                                 ▼
                          /goal_reached
```

## Design / Inner-workings / Algorithms

### Coordinate frames

| Mode | `use_local_frame` | Frame | Description |
|:-----|:------------------|:------|:------------|
| Local (default) | `true` | `chassis_link` | Robot at origin; no odom required |
| World | `false` | `map` / `odom` | For nav2-style global planners |

### Controller types

| Type | Algorithm |
|:-----|:----------|
| `cascade` (default) | Outer PID: heading error → $\omega_\text{ref}$, Inner PID: $\omega_\text{ref} - \omega_\text{cur}$ → $\omega_\text{cmd}$ |
| `pid` | Single PID: heading error → $\omega_\text{cmd}$ |
| `pure_pursuit` | $\omega = k \cdot 2v\sin\alpha / L$, inner PID tracks reference |

### Interpolation

Smooths A\* paths before tracking.

| Method | Description |
|:-------|:------------|
| `none` | As-is |
| `spline` | Catmull-Rom |
| `bezier` | Global Bézier |

### Goal detection

Publishes `true` on `/goal_reached` when distance to the last waypoint < `goal_tolerance`, then stops.

## Inputs / Outputs / API

### Input

| Topic | Type | QoS | Description |
|:------|:-----|:----|:------------|
| `/planned_path` | `nav_msgs/Path` | reliable (10) | Target path in `chassis_link` |
| `/odom` | `nav_msgs/Odometry` | reliable (50) | Pose & angular velocity |

### Output

| Topic | Type | QoS | Description |
|:------|:-----|:----|:------------|
| `/cmd_vel` | `geometry_msgs/Twist` | reliable (10) | `linear.x`, `linear.y`, `angular.z` |
| `/goal_reached` | `std_msgs/Bool` | reliable (10) | `true` when goal is reached |

## Parameters

Defaults: `param/default_params.yaml` &ensp; Schema: `schema/core_path_follower.schema.json`

| Parameter | Type | Default | Description |
|:----------|:-----|:--------|:------------|
| `use_local_frame` | bool | `true` | Local-frame mode |
| `controller_type` | string | `cascade` | `cascade` / `pid` / `pure_pursuit` |
| `linear_speed` | double | `0.6` | Forward speed [m/s] |
| `lookahead_dist` | double | `0.5` | Lookahead distance [m] |
| `goal_tolerance` | double | `0.15` | Goal threshold [m] |
| `control_rate` | double | `20.0` | Control loop frequency [Hz] |
| `reset_on_new_path` | bool | `true` | Reset PID on new path |
| `outer_kp` / `ki` / `kd` | double | `1.2` / `0.0` / `0.15` | Outer PID gains |
| `inner_kp` / `ki` / `kd` | double | `2.0` / `0.0` / `0.05` | Inner PID gains |
| `pure_k` | double | `1.0` | Pure-pursuit gain |
| `interpolation` | string | `spline` | `none` / `spline` / `bezier` |
| `spline_samples_per_segment` | int | `10` | Spline density |
| `bezier_samples` | int | `100` | Bézier density |

## Testing

A Python helper node `test_path_publisher` is provided for standalone verification without a simulator or hardware.

### Quick start

```bash
# Build
colcon build --packages-select core_path_follower --symlink-install

# Launch with test publisher
ros2 launch core_path_follower test_path_follower.launch.py

# Change path shape
ros2 launch core_path_follower test_path_follower.launch.py path_type:=figure8

# Full pipeline (includes body_controller)
ros2 launch core_path_follower test_path_follower.launch.py with_body_controller:=true
```

### Preset paths

| Name | Shape | Test target |
|:-----|:------|:------------|
| `straight` | +x line | Basic tracking |
| `square` | Rectangle | 90° turns |
| `slalom` | Sine wave | Continuous steering |
| `circle` | Circle | Steady-state turning |
| `figure8` | Lemniscate | Left/right switching |
| `diamond` | Rhombus (diagonal) | Mecanum diagonal drive |
| `lateral` | +y line | Mecanum lateral drive |

### Test publisher parameters

| Parameter | Type | Default | Description |
|:----------|:-----|:--------|:------------|
| `mode` | string | `path` | `path`: publish path directly / `planner`: publish `/map`, `/start`, `/goal` |
| `path_type` | string | `straight` | Preset name |
| `use_local_frame` | bool | `true` | When `false`, publishes simulated odom |
| `frame_id` | string | `chassis_link` | Path frame ID |
| `path_scale` | double | `1.0` | Scale factor |
| `one_shot` | bool | `false` | Publish path only once |
| `cycle` | bool | `false` | Auto-advance to next path on goal |
| `monitor_can` | bool | `true` | Log `can/tx` output |
| `publish_joint_states` | bool | `false` | Publish simulated joint states |

## Assumptions / Known limits

- Local mode assumes path is expressed in `chassis_link`.
- Obstacle avoidance is the planner's responsibility.
- Output Twist uses all three omnidirectional components (`vx`, `vy`, `ω`).

## Future extensions / Unimplemented parts

<!-- Add planned features here -->
