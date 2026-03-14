# トピック・メッセージ一覧

## 主要トピック

### ナビゲーションパイプライン

| トピック | 型 | Publisher | Subscriber | QoS |
|---------|------|-----------|------------|-----|
| `/sim_odom` | `nav_msgs/Odometry` | Unity (TCP) | odom_bridge | reliable(10) |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO | odom_bridge | reliable(10) |
| `/odom` | `nav_msgs/Odometry` | odom_bridge | mppi, path_follower | reliable(10) |
| `/start_pose` | `geometry_msgs/PoseStamped` | odom_bridge | path_planner | reliable(10) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 / behavior system | path_planner, mppi | reliable(10) |
| `/map` | `nav_msgs/OccupancyGrid` | map_server | path_planner | transient_local(1) |
| `/costmap/global` | `nav_msgs/OccupancyGrid` | map_server | mppi | transient_local(1) |
| `/costmap/local` | `nav_msgs/OccupancyGrid` | costmap_builder | mppi | reliable |
| `/planned_path` | `nav_msgs/Path` | path_planner | mppi, path_follower | reliable |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | mppi or path_follower | cmd_vel_smoother | reliable(10) |
| `/cmd_vel` | `geometry_msgs/Twist` | cmd_vel_smoother | body_controller | reliable(10) |

### センサ

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox Mid-360 | costmap_builder |
| `/livox/lidar/no_self` | `sensor_msgs/PointCloud2` | costmap_builder | (デバッグ) |
| `/lidar/points_filtered` | `sensor_msgs/PointCloud2` | costmap_builder | (デバッグ) |

### 車体制御

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/can/tx` | `core_msgs/CANArray` | body_controller | core_hardware |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | (外部) | body_controller |
| `/body_target_angle` | `std_msgs/Float64` | (外部) | body_controller |
| `/body_omega` | `std_msgs/Float64` | body_controller | (外部) |
| `/joint_states` | `sensor_msgs/JointState` | core_hardware | body_controller |
| `/goal_reached` | `std_msgs/Bool` | mppi, path_follower | (外部) |

### 敵検出

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `raw_image/compressed` | `sensor_msgs/Image` | カメラ | target_detector |
| `damage_panels_infomation` | `core_msgs/DamagePanelInfoArray` | target_detector | target_selector |
| `damage_panel_pose` | `geometry_msgs/PointStamped` | target_selector | aim_bot |

### シューター

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/left/shoot_once` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/shoot_motor` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/manual_mode` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/manual_pitch` | `std_msgs/Float32` | wireless_parser | shooter_cmd_gate |
| `/reloading` | `std_msgs/Bool` | wireless_parser | magazine_manager |
| `shoot_cmd` | `std_msgs/Int32` | shooter_controller | (CAN) |
| `shoot_motor` | `std_msgs/Float32` | shooter_cmd_gate | shooter_controller |

### コントローラ入力

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/wireless` | `std_msgs/UInt8MultiArray` | 受信機 | wireless_parser, diagnostic |
| `/rotation_flag` | `std_msgs/Bool` | wireless_parser | body_controller |

### システム管理

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/system/emergency/hazard_status` | `std_msgs/Bool` | emergency_handler | body_controller, aim_bot |
| `/system/emergency/hazard_states` | `std_msgs/Int8MultiArray` | emergency_handler | (外部) |
| `/system/emergency/hazard_label` | `std_msgs/String` | emergency_handler | (外部) |
| `/emergency` | `std_msgs/Bool` | ハードウェアスイッチ | emergency_handler |

## カスタムメッセージ（core_msgs）

### CAN.msg

```
uint8 id
float32[] data
```

CANフレームの抽象化。`id` はモータID、`data` は指令値。

### CANArray.msg

```
CAN[] array
```

複数のCANメッセージをまとめて送信。

### Path.msg

```
std_msgs/Header header
PoseWithWeight[] pose
```

重み付き経路。

### PoseWithWeight.msg

```
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
float64 distance_to_obstable
```

障害物までの距離情報を持つ姿勢。
