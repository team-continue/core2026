# ナビゲーション起動ガイド

## 起動モード一覧

| モード | コマンド | TCP EP | odom | 用途 |
|--------|---------|--------|------|------|
| sim（デフォルト） | `navigation.launch.py` | o | sim | 開発テスト |
| sim + FAST-LIO | `navigation.launch.py odom_source:=fastlio` | o | FAST-LIO | LiDARテスト |
| 実機 | `navigation.launch.py environment:=real` | x | FAST-LIO | 実機走行 |

## シミュレータモード

### 前提条件

- Unity シミュレータが起動済み
- TCP接続先: `127.0.0.1:10000`

### 起動

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch core_launch navigation.launch.py
```

### 起動されるノード

| ノード | パッケージ | 役割 | 条件 |
|--------|-----------|------|------|
| `ros_tcp_endpoint` | ROS-TCP-Endpoint | Unity接続 | sim時のみ |
| `livox_lidar_publisher` | livox_ros_driver2 | LiDARドライバ | real時のみ |
| `fastlio_mapping` | fast_lio | FAST-LIOオドメトリ | FAST-LIO時 |
| `odom_bridge_node` | core_launch | odom変換 | 常時 |
| `map_server_node` | core_launch | マップ配信 | 常時 |
| `path_planner_node` | core_path_planner | A*経路計画 | 常時 |
| `core_mppi_node` | core_mppi | MPPI制御 | 常時 |
| `cmd_vel_smoother_node` | core_cmd_vel_smoother | cmd_vel平滑化 | use_smoother時 |
| `costmap_build_node` | core_costmap_builder | ローカルコストマップ | 常時 |
| `body_control_node` | core_body_controller | cmd_vel→CAN変換 | canarray時 |
| `target_angle_node` | core_body_controller | 車体角度計算 | canarray時 |
| `rviz2` | rviz2 | 可視化 | use_rviz時 |
| `static_tf_map_odom` | tf2_ros | map→odom TF | 常時 |
| `static_tf_base_livox` | tf2_ros | base_link→livox_frame TF | 常時 |

## 実機モード

### 前提条件

- Livox Mid-360 LiDARが接続済み
- `fast_lio` パッケージがインストール済み
- `core_hardware` デーモンが起動済み

### 起動

```bash
ros2 launch core_launch navigation.launch.py environment:=real
```

初期ヨー角（ロボットの向き）を指定する場合:

```bash
ros2 launch core_launch navigation.launch.py environment:=real init_yaw:=1.5708
```

GUIラッパー（非対話シェル用）:

```bash
ros2 run core_launch navigation.sh
```

### 実機モードの動作

- TCP endpoint は起動しない
- Livox driver が起動し `/livox/lidar` を出力
- FAST-LIO が起動し `/Odometry` を出力（IMUトピック: `/livox/imu`）
- body_controller が起動し `/cmd_vel` → `/can/tx` に変換
- odom_source は自動的に `fastlio` に設定

!!! note "sim + FAST-LIO モードとの違い"
    sim + FAST-LIO（`odom_source:=fastlio`）ではシミュレータが `/livox/lidar` と `/imu` を直接出力するため Livox driver は起動しません。
    実機モードでは Livox driver が起動し、IMUトピック名が `/livox/imu` になります。

## ゴール設定

1. RViz2 で **2D Goal Pose** ボタン（上部ツールバー）をクリック
2. マップ上でクリック＆ドラッグしてゴール位置と向きを指定
3. `/goal_pose` にパブリッシュされ、自動的に経路計画→追従が開始

## RViz2の設定

`navigation.launch.py` は `core_launch/config/integration_test.rviz` を自動でロードします。以下が表示されます:

- マップ（OccupancyGrid）
- ロボット位置（TF）
- 計画経路（Path）
- ローカルコストマップ
- LiDAR点群

## マップ切替

`map_name` 引数でマッププリセットを切り替えます。プリセットにはマップ画像、原点、ロボット初期位置が含まれます。

```bash
# CoRE-1フィールド（デフォルト）
ros2 launch core_launch navigation.launch.py

# curious_house
ros2 launch core_launch navigation.launch.py map_name:=curious_house
```

プリセット定義は `navigation.launch.py` の `MAP_PRESETS` 辞書を参照してください。

## トラブルシューティング

### 経路が計画されない

- `/goal_pose` がパブリッシュされているか確認: `ros2 topic echo /goal_pose`
- `/map` が配信されているか確認: `ros2 topic echo /map --once`
- ゴールが障害物上にないか確認

### ロボットの動きがガタガタ

- スムーザーが有効か確認: `ros2 node list | grep smoother`
- `alpha` を下げて平滑化を強化: launch引数でオーバーライドは不可のため、`navigation.launch.py` 内のパラメータを変更
- スムーザーを無効化してMPPI出力を直接確認: `use_smoother:=false`

### ロボットが動かない

- `/cmd_vel` が出力されているか確認: `ros2 topic echo /cmd_vel`
- `/odom` が更新されているか確認: `ros2 topic hz /odom`
- body_controllerが起動しているか確認: `ros2 node list | grep body_control`

### TFエラー

- `ros2 run tf2_tools view_frames` でTFツリーを確認
- `odom → base_link` が存在するか確認
