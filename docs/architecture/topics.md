# トピック・メッセージ一覧

**このページは各トピックのインタフェース仕様（型・Publisher / Subscriber・QoS・リマップ）を扱います。** ノード構成とデータの流れ、起動構成は[システム概要](overview.md)を参照してください。

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
| `/cmd_vel_raw` | `geometry_msgs/Twist` | mppi | cmd_vel_smoother | reliable(10) |
| `/cmd_vel` | `geometry_msgs/Twist` | cmd_vel_smoother, path_follower, wireless_parser | body_controller | reliable(10) |

!!! note "`/cmd_vel` には複数のPublisherが存在します"
    `core_path_follower` の `cmd_vel_topic` はデフォルトで `/cmd_vel`（`cmd_vel_smoother` を経由しない）、`wireless_parser` も手動操縦時に `/cmd_vel` を発行します。MPPIを使う場合のみ `/cmd_vel_raw` → `cmd_vel_smoother` → `/cmd_vel` の経路になります。

### センサ

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox Mid-360 | costmap_builder |
| `/livox/lidar/no_self` | `sensor_msgs/PointCloud2` | costmap_builder | (デバッグ) |
| `/lidar/points_filtered` | `sensor_msgs/PointCloud2` | costmap_builder | (デバッグ) |

### 車体制御

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/can/tx` | `core_msgs/CANArray` | body_controller, shooter_controller, magazine_manager, aim_bot | core_hardware |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | emergency_handler, wireless_parser | body_controller |
| `/body_target_angle` | `std_msgs/Float64` | target_angle_node | body_controller |
| `/body_omega` | `std_msgs/Float64` | body_controller | (外部) |
| `/joint_states` | `sensor_msgs/JointState` | core_hardware | body_controller, diagnostic |
| `/goal_reached` | `std_msgs/Bool` | mppi, path_follower | behavior_system |
| `/livox/imu` | `sensor_msgs/Imu` | Livox Mid-360 | target_angle_node（`imu` からリマップ） |

### 敵検出

`target_detector` / `target_selector` は砲塔ごと（`left` / `right` 名前空間）に1組ずつ起動します。以下はノード内部でのトピック名です。

| トピック（内部名） | 型 | Publisher | Subscriber | リマップ先 |
|---------|------|-----------|------------|-----------|
| `raw_image` | `sensor_msgs/Image` | usb_cam | target_detector | `/turret_camera_{side}/color/image` |
| `damage_panels_infomation` | `core_msgs/DamagePanelInfoArray` | target_detector | target_selector | （そのまま、名前空間内） |
| `damage_panel_pose` | `geometry_msgs/PointStamped` | target_selector | aim_bot, enemy_detection_coordinator | `/{side}/target_pose` |
| `target_image_position` | `geometry_msgs/PointStamped` | （上記） | aim_bot | `/{side}/target_pose` |

カメラのトピックは以下の通りです。

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/turret_camera_left/color/image` | `sensor_msgs/Image` | usb_cam (camera_left) | left/target_detector |
| `/turret_camera_right/color/image` | `sensor_msgs/Image` | usb_cam (camera_right) | right/target_detector, gui_qt |
| `/turret_camera_tps/color/image` | `sensor_msgs/Image` | usb_cam (camera_tps) | gui_qt |

### シューター

`{side}` は `left` / `right` を表します。

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/right/shoot_fullauto` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/{side}/shoot_fullauto` | `std_msgs/Bool` | attack_shoot_manager | shooter_cmd_gate |
| `/shoot_motor` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/manual_mode` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/manual_pitch` | `std_msgs/Float32` | wireless_parser | shooter_cmd_gate |
| `/{side}/shoot_cmd` | `std_msgs/Int32` | shooter_cmd_gate | shooter_controller |
| `/{side}/shoot_motor` | `std_msgs/Float32` | shooter_cmd_gate | shooter_controller |
| `/{side}/manual_mode` | `std_msgs/Bool` | shooter_cmd_gate | aim_bot |
| `/{side}/manual_pitch_angle` | `std_msgs/Float32` | shooter_cmd_gate | aim_bot |
| `/{side}/shoot_status` | `std_msgs/Bool` | shooter_controller | magazine_manager |
| `/{side}/regrip_active` | `std_msgs/Bool` | magazine_manager | shooter_controller |
| `/{side}/remaining_disk` | `std_msgs/Int8` | magazine_manager | GUI |
| `/reloading` | `std_msgs/Bool` | wireless_parser | （要リマップ）magazine_manager |

### コントローラ入力

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/wireless` | `std_msgs/UInt8MultiArray` | 受信機 | wireless_parser, diagnostic |
| `/rotation_flag` | `std_msgs/Bool` | wireless_parser | body_controller |

### 局在化（Localization）

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO | localization_node |
| `/localization/pose` | `geometry_msgs/PoseStamped` | localization_node | (外部) |
| `/localization/aligned_cloud` | `sensor_msgs/PointCloud2` | localization_node | (デバッグ) |
| `/localization/global_map` | `sensor_msgs/PointCloud2` | localization_node | (デバッグ) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz2 | localization_node |

### システム管理

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/system/emergency/hazard_status` | `std_msgs/Bool` | emergency_handler, **wireless_parser** | body_controller, shooter_controller, magazine_manager, aim_bot, gui_qt, status_display_gui |
| `/system/emergency/hazard_states` | `std_msgs/Int8MultiArray` | emergency_handler | (外部) |
| `/system/emergency/hazard_label` | `std_msgs/String` | emergency_handler | gui_qt, status_display_gui |
| `/emergency` | `std_msgs/Bool` | ハードウェアスイッチ | emergency_handler |
| `/software_emergency` | `std_msgs/Bool` | (外部) | emergency_handler |
| `/destroy` | `std_msgs/Bool` | (外部) | emergency_handler, gui_qt |

!!! warning "`/system/emergency/hazard_status` は2箇所から発行されます"
    `emergency_handler`（`core_mode`）に加えて `wireless_parser`（`core_ros_player_controller`）も同トピックを直接発行します。片方が `false` を出し続けると非常停止が解除されうるため、変更時は両方を確認してください。

`emergency_handler` と `diagnostic` は `mode.launch.py` で `/system/emergency` 名前空間に配置され、入力は以下にリマップされます。

| ノード内部名 | リマップ先 |
|---|---|
| `emergency_switch` | `/emergency` |
| `destroy` | `/destroy` |
| `software_emergency` | `/software_emergency` |
| `microcontroller_monitor` | `/joint_states` |
| `receive_module_monitor` | `/wireless` |

### 行動計画（core_behavior_system）

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/selected_pose` | `geometry_msgs/PoseStamped` | waypoint_selector | behavior_system |
| `/goal_pose` | `geometry_msgs/PoseStamped` | behavior_system | path_planner |
| `/goal_reached` | `std_msgs/Bool` | mppi, path_follower | behavior_system |
| `/enemy_detected` | `std_msgs/Bool` | enemy_detection_coordinator | behavior_system, attack_shoot_manager |
| `/behavior_system/state` | `std_msgs/Int32` | behavior_system | (外部) |
| `/behavior_system/state_name` | `std_msgs/String` | behavior_system | status_display_gui |
| `/waypoint_selector/pause` | `std_msgs/Bool` | (外部) | waypoint_selector |
| `waypoints` | `visualization_msgs/Marker` | waypoint_selector | RViz2 |
| `/led/upper` | `std_msgs/UInt8` | behavior_system | (外部) |

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
