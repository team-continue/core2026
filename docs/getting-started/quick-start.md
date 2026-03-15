# クイックスタート

## 前提条件

- ROS 2 Humble
- Ubuntu 22.04

## ビルド

```bash
cd ~/core_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

## 起動方法

すべての起動は `navigation.launch.py` に集約されています。`environment` と `odom_source` の組み合わせで動作モードが決まります。

```bash
# シミュレータモード（デフォルト）
ros2 launch core_launch navigation.launch.py

# 実機モード
ros2 launch core_launch navigation.launch.py environment:=real
```

> 各モードの詳細、実機テスト手順、リモート RViz2 の使い方は[ナビゲーション起動ガイド](../guides/navigation.md)を参照してください。

全Launch引数の一覧は[ナビゲーション起動ガイド](../guides/navigation.md#launch引数一覧)を参照してください。

## 個別ノードの起動

| コンポーネント | コマンド |
|---------------|---------|
| ボディコントローラ単体 | `ros2 launch core_body_controller body_controller.launch.py` |
| ハードウェアインターフェース単体 | `ros2 launch core_hardware core_hardware.launch.py` |
| ROS-TCP-Endpoint（Unity 接続、sim モードで自動起動） | `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0` |

## ゴール設定

RViz2 の **2D Goal Pose** ボタンでゴールを設定すると、`/goal_pose` にパブリッシュされ、自動的に経路計画・追従が開始されます。
