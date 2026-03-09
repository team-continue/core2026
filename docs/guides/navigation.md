# ナビゲーション起動ガイド

## 起動モード一覧

| モード | コマンド | オドメトリソース | 用途 |
|-------|---------|----------------|------|
| シミュレータ | `navigation.launch.py` | `/sim_odom` (Unity) | 開発・テスト |
| FAST-LIO | `navigation.launch.py odom_source:=fastlio` | `/Odometry` (LiDAR) | 実機走行 |

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

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| `ros_tcp_endpoint` | ROS-TCP-Endpoint | Unity接続 |
| `odom_bridge_node` | core_launch | /sim_odom → /odom変換 |
| `map_server_node` | core_launch | マップ配信 |
| `path_planner_node` | core_path_planner | A*経路計画 |
| `core_mppi_node` | core_mppi | MPPI制御 |
| `costmap_build_node` | core_costmap_builder | ローカルコストマップ |
| `rviz2` | rviz2 | 可視化 |
| `static_tf_map_odom` | tf2_ros | map→odom TF |
| `static_tf_base_livox` | tf2_ros | base_link→livox_frame TF |

## FAST-LIOモード

### 前提条件

- Livox Mid-360 LiDARが接続済み
- `fast_lio` パッケージがインストール済み

### 起動

```bash
ros2 launch core_launch navigation.launch.py odom_source:=fastlio
```

初期ヨー角（ロボットの向き）を指定する場合:

```bash
ros2 launch core_launch navigation.launch.py odom_source:=fastlio init_yaw:=1.5708
```

### 追加で起動されるノード

シミュレータモードのノードに加えて:

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| `fastlio_mapping` | fast_lio | FAST-LIOオドメトリ |

odom_bridge_node は `/Odometry` を購読するモードに切り替わります。

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

## トラブルシューティング

### 経路が計画されない

- `/goal_pose` がパブリッシュされているか確認: `ros2 topic echo /goal_pose`
- `/map` が配信されているか確認: `ros2 topic echo /map --once`
- ゴールが障害物上にないか確認

### ロボットが動かない

- `/cmd_vel` が出力されているか確認: `ros2 topic echo /cmd_vel`
- `/odom` が更新されているか確認: `ros2 topic hz /odom`
- body_controllerが起動しているか確認（navigation.launch.pyには含まれないため別途起動が必要）

### TFエラー

- `ros2 run tf2_tools view_frames` でTFツリーを確認
- `odom → base_link` が存在するか確認
