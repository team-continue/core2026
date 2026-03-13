# クイックスタート

## 前提条件

- ROS 2 Humble
- Ubuntu 22.04

## ビルド

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

## 起動方法

すべての起動は `navigation.launch.py` に集約されています。`environment` と `odom_source` の組み合わせで動作モードが決まります。

### モード一覧

| environment | odom_source | 用途 | コマンド |
|:-----------:|:-----------:|------|---------|
| `sim` | `sim` | **シミュレータ標準**（デフォルト） | `ros2 launch core_launch navigation.launch.py` |
| `sim` | `fastlio` | シミュレータ + FAST-LIO オドメトリ | `... odom_source:=fastlio` |
| `real` | (強制 `fastlio`) | **実機** — Livox + FAST-LIO | `... environment:=real` |

> `...` は `ros2 launch core_launch navigation.launch.py` の省略です。

- `real` モードでは `odom_source` は自動的に `fastlio` に強制されます。
- FAST-LIO 使用時は `init_yaw` で初期ヨー角 [rad] を指定できます（例: `init_yaw:=1.5708`）。

### 地図の切替

`map_name` 引数で地図を切り替えられます。地図ごとにマップ原点とロボット初期位置が自動設定されます。

```bash
# CoRE-1 フィールド（デフォルト）
ros2 launch core_launch navigation.launch.py

# curious_house
ros2 launch core_launch navigation.launch.py map_name:=curious_house
```

### 実機モード（GUI・ショートカットから）

`.bashrc` は非対話シェルで早期 return するため、デスクトップショートカットや `bash -c` からは `navigation.sh` を使います。ROS / core_ws / Livox_ws を明示的に source してから `navigation.launch.py environment:=real` を実行するラッパーです。

```bash
ros2 run core_launch navigation.sh
```

ミニ PC で GUI 負荷を切り分ける場合:

```bash
ros2 run core_launch navigation.sh use_rviz:=false
```

### 個別ノードの起動

| コンポーネント | コマンド |
|---------------|---------|
| ボディコントローラ単体 | `ros2 launch core_body_controller body_controller.launch.py` |
| ハードウェアインターフェース単体 | `ros2 launch core_hardware core_hardware.launch.py` |
| ROS-TCP-Endpoint（Unity 接続、sim モードで自動起動） | `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0` |

## ゴール設定

RViz2 の **2D Goal Pose** ボタンでゴールを設定すると、`/goal_pose` にパブリッシュされ、自動的に経路計画・追従が開始されます。

## Launch引数一覧

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `environment` | `sim` | `sim` or `real` |
| `odom_source` | `sim` | `sim` or `fastlio`（real時は強制FAST-LIO） |
| `map_name` | `core1_field` | マッププリセット名（`core1_field`, `curious_house`） |
| `init_yaw` | `0.0` | 初期ヨー角 [rad]（FAST-LIOモードで使用） |
| `use_rviz` | `true` | RViz2を起動するか |

### マッププリセット

| プリセット | origin | ロボット初期位置 |
|-----------|--------|----------------|
| `core1_field` | (-13.675, -9.15) | (-10.7, 5.9) |
| `curious_house` | (-4.5, -7.5) | (0.0, 0.0) |
