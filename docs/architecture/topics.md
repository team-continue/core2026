# トピック・メッセージ一覧

**このページは各トピックのインタフェース仕様（型・Publisher / Subscriber・QoS・リマップ）を扱います。** ノード構成とデータの流れ、起動構成は[システム概要](overview.md)を参照してください。

## 名前空間とトピック名の決まり

ノードは[システム概要](overview.md#名前空間)の名前空間ツリーに配置され、トピック名も原則としてその配下に置かれます。名前空間は各パッケージのlaunchファイルの `PushRosNamespace` で与えられるため、**トピック名の正はlaunchファイル**です。C++ソース内のトピック名（`declare_parameter` のデフォルト値やハードコードされた絶対名）は、launchを経由せず `ros2 run` した場合のフォールバックとして残っています。

トピックがどの名前空間に属するかは以下の規則で決まります。

| ケース | 置き場所 | 例 |
|---|---|---|
| Publisherが1つ | 発行元の名前空間 | `/ui/manual_mode`（wireless_parser） |
| Publisherが複数・Subscriberが1つ（指令系） | 受け手の名前空間 | `/hardware/can/tx`, `/control/cmd_vel` |

### 名前空間を付けないトピック

外部システム（Unityシミュレータ、外部ドライバ）との境界にあるトピックと、ROS標準トピックは名前空間を付けずグローバルのままです。

| トピック | 理由 |
|---|---|
| `/imu` | 実機は `core_damiao_imu`、simはUnityが発行するため名前は共通である必要がある |
| `/turret_camera_{left,right,tps}/color/image` | 同上（実機は `usb_cam`、simはUnity） |
| `/livox/lidar` | 外部パッケージ `livox_ros_driver2` のトピック |
| `/joint_states`, `/tf`, `/tf_static`, `/initialpose` | ROS標準トピック |
| `/sim_odom`, `/Odometry`, `/cloud_registered` | Unity / FAST-LIO 由来 |

## トピック一覧

launch経由で起動したときに実際に流れる名前で記載します。`{side}` は `left` / `right` を表します。

### ナビゲーションパイプライン

| トピック | 型 | Publisher | Subscriber | QoS |
|---------|------|-----------|------------|-----|
| `/sim_odom` | `nav_msgs/Odometry` | Unity (TCP) | odom_bridge | reliable(10) |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO | odom_bridge | reliable(10) |
| `/localization/odom` | `nav_msgs/Odometry` | odom_bridge | mppi, path_follower | reliable(10) |
| `/localization/start_pose` | `geometry_msgs/PoseStamped` | odom_bridge | path_planner | reliable(10) |
| `/behavior/goal_pose` | `geometry_msgs/PoseStamped` | RViz2 / behavior system | path_planner, mppi | reliable(10) |
| `/map` | `nav_msgs/OccupancyGrid` | map_server | path_planner | transient_local(1) |
| `/map/costmap/global` | `nav_msgs/OccupancyGrid` | map_server | mppi | transient_local(1) |
| `/planning/costmap/local` | `nav_msgs/OccupancyGrid` | costmap_builder | mppi | reliable |
| `/planning/planned_path` | `nav_msgs/Path` | path_planner | mppi, path_follower | reliable |
| `/planning/cmd_vel_raw` | `geometry_msgs/Twist` | mppi | cmd_vel_smoother | reliable(10) |
| `/control/cmd_vel` | `geometry_msgs/Twist` | cmd_vel_smoother, path_follower, wireless_parser | body_controller | reliable(10) |

!!! note "`/control/cmd_vel` には複数のPublisherが存在します"
    `core_path_follower` の `cmd_vel_topic` はデフォルトで `/control/cmd_vel`（`cmd_vel_smoother` を経由しない）、`wireless_parser` も手動操縦時に `/control/cmd_vel` を発行します。MPPIを使う場合のみ `/planning/cmd_vel_raw` → `cmd_vel_smoother` → `/control/cmd_vel` の経路になります。

### センサ

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox Mid-360 | costmap_builder |
| `/livox/lidar/no_self` | `sensor_msgs/PointCloud2` | costmap_builder | (デバッグ) |
| `/lidar/points_filtered` | `sensor_msgs/PointCloud2` | costmap_builder | (デバッグ) |
| `/livox/imu` | `sensor_msgs/Imu` | Livox Mid-360 | FAST-LIO（無効中） |
| `/imu` | `sensor_msgs/Imu` | core_damiao_imu（実機）/ Unity（sim） | imu_filter_madgwick, target_angle_node, hardware_ui_converter_node |
| `/sensing/filtered_imu` | `sensor_msgs/Imu` | imu_filter_madgwick | （現在Subscriberなし） |

### 車体制御

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/hardware/can/tx` | `core_msgs/CANArray` | body_controller, shooter_controller, magazine_manager, aim_bot | core_hardware |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | emergency_handler | body_controller, target_angle_node |
| `/control/body_omega` | `std_msgs/Float64` | body_controller | target_angle_node |
| `/joint_states` | `sensor_msgs/JointState` | core_hardware | body_controller, diagnostic |
| `/control/rotation` | `std_msgs/Int32` | wireless_parser, behavior_system | body_controller, target_angle_node |
| `/behavior/goal_reached` | `std_msgs/Bool` | mppi, path_follower | behavior_system |
| `/control/target_omega` | `std_msgs/Float64` | target_angle_node | (デバッグ) |
| `/control/yaw_target_angle` | `std_msgs/Float64` | (外部) | target_angle_node |

### 敵検出

`target_detector` / `target_selector` は砲塔ごと（`/perception/enemy_detection/left` / `/perception/enemy_detection/right` 名前空間）に1組ずつ起動します。以下はノード内部でのトピック名です。

| トピック（内部名） | 型 | Publisher | Subscriber | リマップ先 |
|---------|------|-----------|------------|-----------|
| `raw_image` | `sensor_msgs/Image` | usb_cam | target_detector | `/turret_camera_{side}/color/image` |
| `damage_panels_infomation` | `core_msgs/DamagePanelInfoArray` | target_detector | target_selector | （そのまま、名前空間内） |
| `damage_panel_pose` | `geometry_msgs/PointStamped` | target_selector | aim_bot, enemy_detection_coordinator | `/perception/enemy_detection/{side}/target_pose` |
| `target_image_position` | `geometry_msgs/PointStamped` | （上記） | aim_bot | `/perception/enemy_detection/{side}/target_pose` |
| `color` | `std_msgs/UInt8` | core_hardware | target_detector, oakd_target_detector | `/hardware/color` |

!!! note "`/enemy_poses` はPublisher未実装"
    `gui_qt` は敵位置オーバーレイ用に `~/input/enemy_poses`（`hud.launch.py` で `/enemy_poses` にリマップ）を購読していますが、これを発行するノードはリポジトリ内に存在しません。発行元を実装する際に名前空間（`/perception` 配下）を決めてください。

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
| `/mecha/shooter/right/shoot_fullauto` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/mecha/shooter/{side}/shoot_fullauto` | `std_msgs/Bool` | attack_shoot_manager | shooter_cmd_gate |
| `/ui/shoot_motor_state` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate |
| `/ui/manual_mode` | `std_msgs/Bool` | wireless_parser | shooter_cmd_gate, behavior_system |
| `/ui/manual_pitch` | `std_msgs/Float32` | wireless_parser | shooter_cmd_gate |
| `/mecha/shooter/{side}/shoot_cmd` | `std_msgs/Int32` | shooter_cmd_gate | shooter_controller |
| `/mecha/shooter/{side}/shoot_motor` | `std_msgs/Float32` | shooter_cmd_gate | shooter_controller |
| `/mecha/shooter/{side}/manual_mode` | `std_msgs/Bool` | shooter_cmd_gate | aim_bot |
| `/mecha/shooter/{side}/manual_pitch_angle` | `std_msgs/Float32` | shooter_cmd_gate | aim_bot |
| `/mecha/shooter/{side}/shoot_status` | `std_msgs/Bool` | shooter_controller | magazine_manager |
| `/mecha/shooter/{side}/regrip_active` | `std_msgs/Bool` | magazine_manager | shooter_controller |
| `/mecha/shooter/{side}/remaining_disk` | `std_msgs/Int8` | magazine_manager | GUI |
| `/mecha/shooter/{side}/shoot_once` | `std_msgs/Bool` | デバッグGUI | shooter_cmd_gate |
| `/mecha/shooter/{side}/shoot_burst` | `std_msgs/Bool` | デバッグGUI | shooter_cmd_gate |
| `/mecha/shooter/{side}/reloading` | `std_msgs/Bool` | デバッグGUI | magazine_manager |
| `/mecha/shooter/{side}/reloading_increment` | `std_msgs/Int8` | デバッグGUI | magazine_manager |
| `/mecha/shooter/{side}/disk_hold_state` | `std_msgs/Bool` | デバッグGUI | magazine_manager |
| `/mecha/shooter/{side}/distance` | `std_msgs/Int32` | 距離センサ（CAN経由） | magazine_manager |
| `/mecha/shooter/{side}/jam` | `std_msgs/Bool` | ジャムセンサ | shooter_controller |
| `/mecha/shooter/{side}/jam_state` | `std_msgs/Bool` | shooter_controller | (診断) |
| `/mecha/shooter/{side}/loading_motor_error_state` | `std_msgs/Bool` | shooter_controller | (診断) |
| `/mecha/shooter/{side}/shoot_motor_error_state` | `std_msgs/Bool` | shooter_controller | (診断) |
| `/mecha/shooter/{side}/test_yaw_angle` | `std_msgs/Float32` | デバッグGUI | aim_bot |
| `/mecha/shooter/{side}/test_pitch_angle` | `std_msgs/Float32` | デバッグGUI | aim_bot |
| `/ui/reloading` | `std_msgs/Bool` | wireless_parser | （Subscriberなし） |

!!! warning "リロードは左右個別のトピック"
    `magazine_manager` は左右それぞれ自身の名前空間で `reloading` を購読するため、実体は `/mecha/shooter/left/reloading` と `/mecha/shooter/right/reloading` の2本です。一方 `wireless_parser` のリロード出力は `/ui/reloading` 1本しかなく、どちらにも接続されていません（この未接続は名前空間移行前からの既存の状態です）。操縦者のリロードボタンを機能させるには、左右どちらに割り当てるか、あるいは左右別トピックを発行するかを決めた上で配線する必要があります。

### コントローラ入力

操縦者入力（`wireless_parser_node` の出力）は `/ui` 名前空間に集約されています。

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/hardware/wireless` | `std_msgs/UInt8MultiArray` | core_hardware（受信機） | wireless_parser, diagnostic |
| `/ui/ads` | `std_msgs/Bool` | wireless_parser | gui_qt |
| `/ui/test_mode` | `std_msgs/Bool` | wireless_parser | shooter_controller, aim_bot |
| `/ui/{side}/turret_auto` | `std_msgs/Bool` | wireless_parser | attack_shoot_manager |
| `/ui/auto_point_select` | `std_msgs/Bool` | （Publisherなし） | behavior_system |
| `/ui/selected_pose` | `geometry_msgs/PoseStamped` | （Publisherなし） | behavior_system |
| `/ui/yaw_degree` | `std_msgs/Float32` | hardware_ui_converter_node | gui_qt |
| `/ui/speed_mps` | `std_msgs/Float32` | hardware_ui_converter_node | gui_qt |
| `/ui/qe_degree` | `std_msgs/Float32` | hardware_ui_converter_node | gui_qt |
| `/ui/gui_debug/log` | `std_msgs/String` | (外部) | gui_qt |

!!! warning "自動目標選択の入力は現在Publisherがいない"
    `wireless_parser` はかつて `/auto_point_select` と `/selected_pose` を発行していましたが、`a0136f0 操作系のプロトコル変更を反映` で削除されました。`wireless_parser_node.launch.py` には両トピックのリマップ引数（`auto_point_select` / `selected_pose`）が残っていますが、対応するPublisherが無いため現状は効果がありません。`behavior_system` の `AUTO_SELECTED` 系の状態遷移は、これらを発行する手段を用意するまで到達しません。

### ハードウェア（core_hardware）

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/hardware/can/tx` | `core_msgs/CANArray` | body_control_node, target_angle_node, shooter_controller, magazine_manager, aim_bot | core_hardware |
| `/hardware/can/rx` | `core_msgs/CANArray` | core_hardware | motor_tool（デバッグ） |
| `/hardware/wireless` | `std_msgs/UInt8MultiArray` | core_hardware（受信機） | wireless_parser, diagnostic |
| `/hardware/destroy` | `std_msgs/Bool` | core_hardware | emergency_handler, gui_qt |
| `/hardware/hp` | `std_msgs/UInt8` | core_hardware | gui_qt |
| `/hardware/color` | `std_msgs/UInt8` | core_hardware | target_detector |
| `/hardware/hardware_emergency` | `std_msgs/Bool` | core_hardware | emergency_handler（`emergency_switch` からリマップ） |
| `/hardware/led/{upper,bottom,bottom2}` | `std_msgs/UInt8` | （Publisherなし） | core_hardware |

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
| `/system/emergency/hazard_status` | `std_msgs/Bool` | emergency_handler | body_controller, shooter_controller, magazine_manager, aim_bot, gui_qt, status_display_gui |
| `/system/emergency/hazard_states` | `std_msgs/Int8MultiArray` | emergency_handler | (外部) |
| `/system/emergency/hazard_label` | `std_msgs/String` | emergency_handler | gui_qt, status_display_gui |
| `/hardware/hardware_emergency` | `std_msgs/Bool` | core_hardware（非常停止スイッチ） | emergency_handler |
| `/system/emergency/software_emergency` | `std_msgs/Bool` | wireless_parser | emergency_handler |
| `/hardware/destroy` | `std_msgs/Bool` | core_hardware | emergency_handler, gui_qt |
| `/system/emergency/microcontroller_emergency` | `std_msgs/Bool` | diagnostic | emergency_handler |
| `/system/emergency/receiver_emergency` | `std_msgs/Bool` | diagnostic | emergency_handler |

!!! warning "ハザード状態の発行元が2系統ある"
    `wireless_parser`（`core_ros_player_controller`）はソース上 `/system/emergency/hazard_status` を直接発行しますが、`wireless_parser_node.launch.py` の `hazard_status` 引数（既定 `/system/emergency/software_emergency`）でリマップされ、`emergency_handler` が集約する形になっています。launchを経由せず `ros2 run` した場合は `/system/emergency/hazard_status` を直接発行してしまうため、非常停止が解除されうる点に注意してください。

`emergency_handler` と `diagnostic` は `mode.launch.py` で `/system/emergency` 名前空間に配置され、入力は以下にリマップされます。

| ノード内部名 | リマップ先 |
|---|---|
| `emergency_switch` | `/hardware/hardware_emergency` |
| `destroy` | `/hardware/destroy` |
| `software_emergency` | （リマップなし → `/system/emergency/software_emergency`） |
| `microcontroller_monitor` | `/joint_states` |
| `receive_module_monitor` | `/hardware/wireless` |

### 行動計画（core_behavior_system）

| トピック | 型 | Publisher | Subscriber |
|---------|------|-----------|------------|
| `/ui/selected_pose` | `geometry_msgs/PoseStamped` | （Publisherなし） | behavior_system |
| `/behavior/goal_pose` | `geometry_msgs/PoseStamped` | behavior_system | path_planner |
| `/behavior/goal_reached` | `std_msgs/Bool` | mppi, path_follower | behavior_system |
| `/behavior/enemy_detected` | `std_msgs/Bool` | enemy_detection_coordinator | behavior_system, attack_shoot_manager |
| `/behavior/state` | `std_msgs/Int32` | behavior_system | (外部) |
| `/behavior/state_name` | `std_msgs/String` | behavior_system | status_display_gui |
| `/behavior/waypoint_selector/goal_pose` | `geometry_msgs/PoseStamped` | waypoint_selector | behavior_system |
| `/behavior/waypoint_selector/pause` | `std_msgs/Bool` | behavior_system | waypoint_selector |
| `/behavior/waypoints` | `visualization_msgs/Marker` | waypoint_selector | RViz2 |
| `/led/upper` | `std_msgs/UInt8` | behavior_system | （未接続） |

!!! warning "`behavior_system` のLED出力は未接続"
    `behavior_system_manager.cpp` は `/led/upper` を**絶対名**で発行するため `PushRosNamespace` の影響を受けません。一方 `core_hardware` は相対名 `led/upper`（= `/hardware/led/upper`）を購読しており、両者は繋がっていません。`behavior_system_node` を起動する際は launch で `("/led/upper", "/hardware/led/upper")` のリマップを追加してください（現在 `behavior_system.launch.py` は `attack_shoot_manager` しか起動しないため、このノードは未起動です）。

## 未接続トピック

Publisher と Subscriber のどちらかが存在しないトピックの一覧です。**いずれも名前空間の導入が原因ではなく、それ以前から繋がっていませんでした。**

| トピック | 状態 | 対応 |
|---|---|---|
| `/led/upper` | `behavior_system` が絶対名で発行。`core_hardware` は `/hardware/led/upper` を購読 | 未対応。当該ノードは未起動のため実害なし。起動時は `("/led/upper", "/hardware/led/upper")` のリマップが必要 |
| `/ui/reloading` | `wireless_parser` が発行。Subscriberなし | 未対応。`magazine_manager` は左右個別に `/mecha/shooter/{side}/reloading` を購読するため 1対2 となり、割り当ての判断が必要 |
| `/ui/auto_point_select` | Publisherなし。`behavior_system` が購読 | 未対応。`a0136f0 操作系のプロトコル変更を反映` で `wireless_parser` から削除済み |
| `/ui/selected_pose` | 同上 | 同上 |
| `/enemy_poses` | Publisherなし。`gui_qt` が購読 | 未対応。発行元の実装時に `/perception` 配下の名前を決める |
| `/hardware/led/{upper,bottom,bottom2}` | Publisherなし。`core_hardware` が購読 | 上記 `/led/upper` の対向 |
| `/sensing/filtered_imu` | `imu_filter_madgwick` が発行。Subscriberなし | `target_angle_node` は生の `/imu` を購読している |
| `/emergency` | Publisherなし（旧構成） | **修正済み。** `emergency_handler` の `emergency_switch` を `/hardware/hardware_emergency` にリマップ |
| `/color2` | Publisherなし（旧構成） | **修正済み。** `test.launch.py` の参照を `/hardware/color` に統一 |

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
