# core2026
CoRE2026用のメインレポジトリです．

## To contributers
開発はブランチを切ってPull requestsしてmainにマージする形にしてください．
Pull requests以外でのmainコミットは禁止です．
### ブランチ名
仕様追加は`feature/hogehoge`
バグ修正は`fix/hogehoge`
で行きましょう．

### パッケージ名
パッケージ名は`core_hoge`
にしましょう，タブ補完で一覧で出たら何も考えなくて良いので．

## Formatting (CI)
Push時にCIが`ament_clang_format`を実行し、必要ならブランチへ自動コミットします。
mainブランチには自動pushしません。

## 使い方 (ロボットの起動)
```bash
# ナビゲーションシミュレータモード（デフォルト: /sim_odom使用）
ros2 launch core_launch navigation.launch.py

# ナビゲーションFAST-LIOモード（実機LiDAR: /Odometry使用）
ros2 launch core_launch navigation.launch.py odom_source:=fastlio

# ナビゲーションFAST-LIOモード + 初期ヨー指定
ros2 launch core_launch navigation.launch.py odom_source:=fastlio init_yaw:=1.5708

# ボディコントローラ
ros2 launch core_body_controller body_controller.launch.py

# 実機ハードウェア
ros2 launch core_hardware core_hardware.launch.py

# ROS-TCP-Endpointの起動
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```