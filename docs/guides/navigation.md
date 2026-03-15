# ナビゲーション起動ガイド

## 起動モード一覧

| モード | コマンド | TCP EP | odom | 用途 |
|--------|---------|--------|------|------|
| sim（デフォルト） | `navigation.launch.py` | o | sim | 開発テスト |
| sim + FAST-LIO | `navigation.launch.py odom_source:=fastlio` | o | FAST-LIO | LiDARテスト |
| 実機 | `navigation.launch.py environment:=real` | x | FAST-LIO | 実機走行 |
| 実機 + localization | `navigation.launch.py environment:=real use_localization:=true` | x | FAST-LIO | グローバル局在化付き実機走行 |

## シミュレータモード

### 前提条件

- Unity シミュレータが起動済み
- TCP接続先: `127.0.0.1:10000`

### 起動

```bash
source ~/core_ws/install/setup.bash
ros2 launch core_launch navigation.launch.py
```

## 実機モード

### 前提条件

- Livox Mid-360 LiDARが接続済み
- `fast_lio` パッケージがインストール済み
- `core_hardware` デーモンが起動済み

### 起動（通常）

ターミナルで直接起動する場合:

```bash
source /opt/ros/humble/setup.bash
source ~/core_ws/install/setup.bash
ros2 launch core_launch navigation.launch.py environment:=real
```

初期ヨー角（ロボットの向き）を指定する場合:

```bash
ros2 launch core_launch navigation.launch.py environment:=real init_yaw:=1.5708
```

### 起動（navigation.sh を使う場合）

SSH やデスクトップショートカットなど、**`.bashrc` が読み込まれない環境**では `ros2 launch` が直接動きません。
`navigation.sh` は ROS 2 / core_ws / Livox_ws のセットアップを自動で行ってから launch を実行するラッパースクリプトです。

```bash
ros2 run core_launch navigation.sh
```

!!! tip "実機で launch しても起動しないとき"
    SSH 経由でミニPCにアクセスして `ros2 launch` を実行したときに「コマンドが見つからない」「パッケージが見つからない」等のエラーが出る場合は、`navigation.sh` を使ってください。
    SSH のデフォルトシェルは non-interactive のため `.bashrc` の `source setup.bash` が実行されず、ROS 2 の環境変数が未設定になることが原因です。

ミニPCでGUI負荷を避けたい場合:

```bash
ros2 run core_launch navigation.sh use_rviz:=false
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

## 実機テストの手順

ミニPCに直接モニタ・キーボードを接続して操作することもできますが、SSH経由で起動してリモートPCの RViz2 で可視化・操作することもできます。

### 1. ミニPCでナビゲーションを起動

SSHでミニPCにログインし、RViz なしで起動します:

```bash
ssh <user>@<mini-pc-ip>
ros2 run core_launch navigation.sh use_rviz:=false
```

!!! warning "ROS_DOMAIN_ID を揃える"
    ミニPCとリモートPCで `ROS_DOMAIN_ID` が異なるとトピックが見えません。両方で同じ値を設定してください。

    ```bash
    export ROS_DOMAIN_ID=0  # デフォルトは0
    ```

### 2. リモートPCで RViz2 を起動

手元のPCで、プロジェクト付属の RViz 設定ファイルを指定して起動します:

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/core_ws/src/core2026/core_launch/config/navigation.rviz
```

この設定ファイルには以下の表示が含まれています:

- マップ（OccupancyGrid）
- ロボット位置（TF）
- 計画経路（Path）
- ローカルコストマップ
- LiDAR点群

!!! tip "RViz2 で何も表示されない場合"
    - `ROS_DOMAIN_ID` がミニPCと一致しているか確認
    - リモートPCで `ros2 topic list` を実行してトピックが見えるか確認
    - ファイアウォールが DDS のマルチキャスト通信をブロックしていないか確認

### 3. ゴール設定

RViz2 の **2D Goal Pose** ボタン（上部ツールバー）をクリックし、マップ上でクリック＆ドラッグしてゴール位置と向きを指定します。
`/goal_pose` にパブリッシュされ、自動的に経路計画→追従が開始されます。

## 起動されるノード

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
| `localization_node` | core_localization | NDT/ICPグローバル局在化 | use_localization時 |
| `static_tf_map_odom` | tf2_ros | map→odom TF（静的） | use_localization無効時 |
| `static_tf_base_livox` | tf2_ros | base_link→livox_frame TF | 常時 |

## Launch引数一覧

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `environment` | `sim` | `sim` or `real` |
| `odom_source` | `sim` | `sim` or `fastlio`（real時は強制FAST-LIO） |
| `map_name` | `core1_field` | マッププリセット名。2Dマップ・初期位置・PCD地図を一括選択 |
| `init_yaw` | `0.0` | 初期ヨー角 [rad]（FAST-LIOモードで使用） |
| `use_rviz` | `true` | RViz2を起動するか |
| `use_smoother` | `true` | cmd_vel平滑化を有効にするか |
| `use_localization` | `false` | グローバル局在化を有効にするか。PCD地図は `map_name` に対応する `pcd_maps/<map_name>.pcd` が自動で使用される |

## マップ切替

`map_name` 引数でマッププリセットを切り替えます。プリセットにはマップ画像、原点、ロボット初期位置が含まれます。

```bash
# CoRE-1フィールド（デフォルト）
ros2 launch core_launch navigation.launch.py

# curious_house
ros2 launch core_launch navigation.launch.py map_name:=curious_house
```

| プリセット | origin | ロボット初期位置 |
|-----------|--------|----------------|
| `core1_field` | (-13.675, -9.15) | (-10.7, 5.9) |
| `curious_house` | (-4.5, -7.5) | (0.0, 0.0) |

プリセット定義は `navigation.launch.py` の `MAP_PRESETS` 辞書を参照してください。

## トラブルシューティング

### ros2 コマンドが見つからない / パッケージが見つからない

SSH経由やデスクトップショートカットからの起動で発生しやすい問題です。

- **原因**: non-interactive シェルでは `.bashrc` が読み込まれず、ROS 2の環境変数が未設定
- **対処**: `navigation.sh` を使って起動する（→ [navigation.sh を使う場合](#起動navigation.sh-を使う場合)）

### 経路が計画されない

- `/goal_pose` がパブリッシュされているか確認: `ros2 topic echo /goal_pose`
- `/map` が配信されているか確認: `ros2 topic echo /map --once`
- ゴールが障害物上にないか確認

### ロボットが動かない

- `/cmd_vel` が出力されているか確認: `ros2 topic echo /cmd_vel`
- `/odom` が更新されているか確認: `ros2 topic hz /odom`
- body_controllerが起動しているか確認: `ros2 node list | grep body_control`

### ロボットの動きがガタガタ

- スムーザーが有効か確認: `ros2 node list | grep smoother`
- `alpha` を下げて平滑化を強化: launch引数でオーバーライドは不可のため、`navigation.launch.py` 内のパラメータを変更
- スムーザーを無効化してMPPI出力を直接確認: `use_smoother:=false`

### TFエラー

- `ros2 run tf2_tools view_frames` でTFツリーを確認
- `odom → base_link` が存在するか確認

### リモートPCからトピックが見えない

- `ROS_DOMAIN_ID` が一致しているか確認
- 同一ネットワーク上にいるか確認
- ファイアウォールの設定を確認（DDS はマルチキャスト UDP を使用）
- `ros2 topic list` で確認
