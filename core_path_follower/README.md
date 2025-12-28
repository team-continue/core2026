# core_path_follower

Node to follow a `nav_msgs/msg/Path` using a PID-based heading controller.

Usage
- Build the workspace: `colcon build` from your workspace root.
- Source the install: `source install/setup.bash`.
- Launch with params: `ros2 launch core_path_follower path_follower.launch.py`.

Topics
- Subscribes: `/path` (nav_msgs/Path), `/odom` (nav_msgs/Odometry)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

Parameters (in `config/default_params.yaml`)
- `controller_type`: controller selection: `'cascade'`, `'pid'`, or `'pure_pursuit'`.
- `linear_speed`, `lookahead_dist`, `control_rate`
- `outer_kp/outer_ki/outer_kd`: outer heading PID (heading -> desired omega)
- `inner_kp/inner_ki/inner_kd`: inner rate PID (omega tracking)
- `pure_k`: pure-pursuit gain (used when `controller_type` is `'pure_pursuit'`)
