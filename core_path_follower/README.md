# core_path_follower

This package provides a ROS 2 node that follows a `nav_msgs/msg/Path` by generating velocity commands. The controller primarily uses a PID-based heading controller, with an optional pure-pursuit mode.

## Quickstart

- Build the workspace: `colcon build` from the workspace root.
- Source the install: `source install/setup.bash`.
- Launch the node: `ros2 launch core_path_follower path_follower.launch.py`

You can override parameters at launch, for example:

```bash
ros2 launch core_path_follower path_follower.launch.py controller_type:=pure_pursuit linear_speed:=0.5
```

## Node & Files

- Node name: `core_path_follower` (may vary depending on launch)
- Key files:
  - `launch/path_follower.launch.py` — launch file
  - `config/default_params.yaml` — default parameters

---

## Topics

- Subscribed:
  - `/path` (`nav_msgs/msg/Path`) — target path (sequence of `PoseStamped`)
  - `/odom` (`nav_msgs/msg/Odometry`) — robot pose/velocity
- Published:
  - `/cmd_vel` (`geometry_msgs/msg/Twist`) — velocity commands (`linear.x`, `angular.z`)

## Parameters (see `config/default_params.yaml`)

- `controller_type` (string, default: `"cascade"`): controller mode: `"cascade"`, `"pid"`, or `"pure_pursuit"`.
- `linear_speed` (double, m/s): reference forward speed used during path following.
- `lookahead_dist` (double, m): lookahead distance for pure pursuit.
- `control_rate` (int, Hz): controller execution frequency.

- Outer (heading) PID (used in `cascade` mode):
  - `outer_kp`, `outer_ki`, `outer_kd`
- Inner (angular rate) PID (used in `cascade` mode):
  - `inner_kp`, `inner_ki`, `inner_kd`
- `pure_k` (double): gain used to convert curvature to angular velocity in `pure_pursuit` mode.

(Refer to `config/default_params.yaml` for concrete default values.)

## Controller Specification (summary)

- cascade (recommended):
  1. Outer loop: compute heading error to the next target point on the path and use the outer PID to compute a desired angular velocity (`omega_ref`).
  2. Inner loop: read current angular velocity and use the inner PID to track `omega_ref`, producing the commanded angular velocity.
  3. Linear velocity is based on `linear_speed` and may be reduced by any braking logic if necessary.

- pid:
  - A single PID controller maps heading error directly to angular velocity command (no inner/outer separation).

- pure_pursuit:
  - Select a lookahead point along the path at distance `lookahead_dist` and compute curvature to that point, then convert curvature to angular velocity using `pure_k` and the current linear speed.

## Expected Inputs & Behavior

- The `Path` message should contain a sequence of poses in a world frame (commonly `map` or `odom`).
- `odom` must provide accurate robot pose and angular velocity for good tracking.
- Output `cmd_vel` uses `linear.x` and `angular.z` to drive the robot base.

## Tuning Tips

- Start with `controller_type:=cascade`. Tune `outer_kp` first (small -> increase) to get heading convergence.
- Tune inner PID (`inner_kp`) to match robot dynamics; higher values increase responsiveness but may induce oscillation.
- `pure_pursuit` works well for smooth curves and higher speeds; for sharp corners adjust `lookahead_dist` and `pure_k`.

## Debugging & Visualization

- Check outputs using: `ros2 topic echo /cmd_vel`.
- Visualize the `Path` in RViz to verify the target path.
- Confirm TF frames and `odom` correctness (e.g., `ros2 run tf2_tools view_frames.py`).

## Dependencies

Typical ROS 2 message dependencies: `nav_msgs`, `geometry_msgs`, `rclpy`/`rclcpp`. Optionally `nav2_util` if used by the implementation.

## Common Issues

- No `cmd_vel` published: verify topic names, node namespace, and that input topics are published.
- Robot does not follow path: check frame consistency between `Path` and `odom`, time synchronization, and TF availability.

---

If you prefer, I can:
- Replace the existing `README.md` with this English version, or
- Keep both and add a short English header in `README.md` linking to `README.EN.md`.

Would you like me to add default parameter values from `config/default_params.yaml` into this README?