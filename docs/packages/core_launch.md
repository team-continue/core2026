# core_launch

ランチファイルとヘルパーノードを提供するメタパッケージです。

## ランチファイル

| ファイル | 用途 |
|---------|------|
| `core_2026.launch.py` | ナビゲーションに加えて、状態配信・sensing・USBカメラ・状態表示GUI・モード管理・無線入力・射撃・敵検出・実機ハードウェアをまとめて起動 |
| `sensing.launch.py` | Mid-360 と IMU フィルタをまとめて起動 |
| `navigation.launch.py` | ナビゲーションパイプライン全体を起動 |
| `imu_filter.launch.py` | IMUフィルタリング |
| `state_publisher.launch.py` | ロボットステートパブリッシャ（URDF/JointState） |

## core_2026.launch.py が起動する launch

### 直接 include する launch

| launch | デフォルト | 条件 | 説明 |
|--------|-----------|------|------|
| `state_publisher.launch.py` | 有効 | `launch_state_publisher:=true` | URDF を読み込み、`robot_state_publisher` と `joint_state_publisher` を起動します。TF可視化やRViz表示の土台です。 |
| `sensing.launch.py` | 有効 | `launch_imu_filter:=true` または `environment:=real` かつ `launch_mid360:=true` | sensing 系の親 launch です。`imu_filter.launch.py` と外部の Mid-360 launch を条件付きでまとめて起動します。 |
| `usb_cam.launch.py` | 有効 | `environment:=real` かつ `launch_usb_camera:=true` | 左・TPS・右の USB カメラノードを起動します。`detection.launch.py` が前提にする実機カメラ入力トピックを供給します。 |
| `navigation.launch.py` | 有効 | `launch_navigation:=true` | ナビゲーション系の中核 launch です。地図配信、オドメトリ統合、経路計画、MPPI、ローカルコストマップ、body controller までをまとめて起動します。 |
| `mode.launch.py` | 有効 | `launch_mode:=true` | 緊急停止や診断監視を担当する `emergency_handler` と `diagnostic` を起動します。 |
| `status_display_gui.launch.py` | 有効 | `environment:=real` かつ `launch_status_gui:=true` | 運用時の状態表示 GUI を起動します。behavior 状態と hazard 状態を全画面表示します。 |
| `wireless_parser_node.launch.py` | 有効 | `launch_wireless_parser:=true` | ワイヤレスコントローラ入力を解析し、`/cmd_vel` や射撃系の操作トピックへ変換します。 |
| `shooter.launch.py` | 有効 | `launch_shooter:=true` | 左右の `shooter_controller`、`magazine_manager`、`aim_bot` と `shooter_cmd_gate` を起動し、射撃系全体を立ち上げます。 |
| `detection.launch.py` | 有効 | `launch_enemy_detection:=true` | 左右タレット用の敵検出 launch をまとめて起動します。カメラ画像からターゲット候補を生成します。 |
| `core_hardware.launch.py` | 有効 | `environment:=real` かつ `launch_hardware:=true` | 実機通信用の `core_hardware` を起動します。シミュレータモードでは起動しません。 |

### 間接的に起動される launch

| launch | 起動元 | 説明 |
|--------|--------|------|
| `imu_filter.launch.py` | `sensing.launch.py` | `imu_filter_madgwick` を起動し、生のIMU値から姿勢推定済みの `/filtered_imu` を生成します。 |
| `body_controller.launch.py` | `navigation.launch.py` | 車体制御用の `body_control_node` と `target_angle_node` を起動します。`launch_navigation:=true` のときに間接的に起動され、`/cmd_vel` を実際の車体制御信号へ変換します。 |
| `enemy_detection.launch.py` | `detection.launch.py` | 左右それぞれのタレット名前空間で `target_detector` と `target_selector` を起動します。`detection.launch.py` から左右2回 include されます。 |

### 補足

- `core_2026.launch.py` は「すべてのノードを直接起動する」構成ではなく、既存の package ごとの launch を include して束ねる親 launch です。
- `core_2026.launch.py` から `navigation.launch.py` には `launch_mid360:=false` を渡し、sensing 系の二重起動を避けています。
- `navigation.launch.py` の中では launch file 以外にも、`path_planner_node`、`core_mppi_node`、`costmap_build_node`、`map_server_node`、`odom_bridge_node` が直接起動され、real時は `sensing.launch.py` を経由して Mid-360 を起動できます。
- `sensing.launch.py` は `imu_filter.launch.py` を include し、real時は別途インストールされている `livox_ros2_driver` の `livox_lidar_rviz_launch.py` も include します。
- `sensing.launch.py`、`usb_cam.launch.py`、`status_display_gui.launch.py` の個別引数は、現時点では `core_2026.launch.py` から必要最小限のみ forward しています。
- `detection.launch.py` は `/turret_camera_left/...` と `/turret_camera_right/...` の画像入力を前提にしているため、実機運用の親 launch では `usb_cam.launch.py` をあわせて起動する構成にしています。
- `status_display_gui.launch.py` は運用用 GUI のため、`environment:=real` のときだけ標準で起動します。

## ヘルパーノード

### odom_bridge_node

オドメトリソースを `/odom` にブリッジし、TF（odom→base_link）と `/start_pose` をパブリッシュします。

**入力:**

| トピック | 型 | 条件 |
|---------|------|------|
| `/sim_odom` | `nav_msgs/Odometry` | simモード |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIOモード |

**出力:**

| トピック | 型 |
|---------|------|
| `/odom` | `nav_msgs/Odometry` |
| `/start_pose` | `geometry_msgs/PoseStamped` |

**TFブロードキャスト:**

- `odom → base_link`（動的）
- `odom → camera_init`（FAST-LIOモードのみ、初回メッセージ受信時に一度だけ配信）

**パラメータ:**

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `odom_source` | string | `sim` | `sim` or `fastlio` |
| `odom_frame` | string | `odom` | 親フレーム名 |
| `base_frame` | string | `base_link` | 子フレーム名 |
| `init_x` | double | `0.0` | 初期X座標 [m] |
| `init_y` | double | `0.0` | 初期Y座標 [m] |
| `init_yaw` | double | `0.0` | 初期ヨー角 [rad] |

### map_server_node

PNG画像をOccupancyGridに変換し、`/map` と `/costmap/global` にパブリッシュします。

**出力:**

| トピック | 型 | QoS | フレーム |
|---------|------|-----|---------|
| `/map` | `nav_msgs/OccupancyGrid` | transient_local | `map` |
| `/costmap/global` | `nav_msgs/OccupancyGrid` | transient_local | `odom` |

**パラメータ:**

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `image_path` | string | `core1_field.png` | マップ画像の絶対パス（navigation.launch.pyではプリセットから自動設定） |
| `resolution` | double | `0.05` | 解像度 [m/px] |
| `origin_x` | double | `0.0` | マップ原点X [m] |
| `origin_y` | double | `0.0` | マップ原点Y [m] |
| `occupied_thresh` | double | `0.65` | 占有判定しきい値 |
| `free_thresh` | double | `0.25` | 自由判定しきい値 |
| `inflation_radius_m` | double | `0.0` | 障害物膨張半径（LETHAL zone）[m]。navigation.launch.pyでは`0.40` |
| `decay_margin_m` | double | `0.0` | LETHAL zone外の線形減衰幅 [m]。navigation.launch.pyでは`0.20` |

!!! info "navigation.launch.pyでの設定値"
    `map_name` プリセットにより自動設定されます。

    - `core1_field`: `resolution=0.025`, `origin_x=-13.675`, `origin_y=-9.15`
    - `curious_house`: `resolution=0.025`, `origin_x=-4.5`, `origin_y=-7.5`
