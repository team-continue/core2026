# core_launch

ランチファイルとヘルパーノードを提供するメタパッケージです。

## ランチファイル

| ファイル | 用途 |
|---------|------|
| `navigation.launch.py` | ナビゲーションパイプライン全体を起動 |
| `imu_filter.launch.py` | IMUフィルタリング |
| `state_publisher.launch.py` | ロボットステートパブリッシャ（URDF/JointState） |

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
