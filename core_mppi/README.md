# core_mppi

Local MPPI controller package for CoRE2026.

## Inputs
- `/planned_path` (`nav_msgs/Path`)
- `/odom` (`nav_msgs/Odometry`)
- `/costmap/local` (`nav_msgs/OccupancyGrid`)
- `/costmap/global` (`nav_msgs/OccupancyGrid`)

## Output
- `/cmd_vel` (`geometry_msgs/Twist`)

## Run
```bash
ros2 launch core_mppi mppi.launch.py
```
