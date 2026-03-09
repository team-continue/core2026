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

### シミュレータモード（デフォルト）

Unity シミュレータの `/sim_odom` を使用します。

```bash
ros2 launch core_launch navigation.launch.py
```

### FAST-LIO モード（実機LiDAR）

Livox Mid-360 LiDAR の `/Odometry` を使用します。

```bash
ros2 launch core_launch navigation.launch.py odom_source:=fastlio
```

初期ヨー角を指定する場合:

```bash
ros2 launch core_launch navigation.launch.py odom_source:=fastlio init_yaw:=1.5708
```

### ボディコントローラ単体

```bash
ros2 launch core_body_controller body_controller.launch.py
```

### ハードウェアインターフェース単体

```bash
ros2 launch core_hardware core_hardware.launch.py
```

### ROS-TCP-Endpoint（Unity接続）

`navigation.launch.py` に含まれていますが、単体起動する場合:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## ゴール設定

RViz2 の **2D Goal Pose** ボタンでゴールを設定すると、`/goal_pose` にパブリッシュされ、自動的に経路計画・追従が開始されます。

## Launch引数一覧

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `map_image` | `global_map.png` | マップ画像のパス |
| `odom_source` | `sim` | オドメトリソース: `sim` or `fastlio` |
| `init_yaw` | `0.0` | 初期ヨー角 [rad]（FAST-LIOモードで使用） |
