# core2026

CoRE2026用のメインレポジトリです。

詳細はドキュメントサイトを参照してください。

## 開発ルール

- 開発はブランチを切って Pull Request で main にマージしてください。main への直接コミットは禁止です。
- ブランチ名: 機能追加は `feature/hogehoge`、バグ修正は `fix/hogehoge`
- パッケージ名: `core_hoge`（タブ補完で一覧表示できるように）

## Formatting (CI)

Push 時に CI が `ament_clang_format` を実行し、必要ならブランチへ自動コミットします。
main ブランチには自動 push しません。

## ビルド

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

## 起動方法

```bash
# シミュレータモード（デフォルト: /sim_odom使用）
ros2 launch core_launch navigation.launch.py

# シミュレータ + FAST-LIOオドメトリ
ros2 launch core_launch navigation.launch.py odom_source:=fastlio

# 実機モード（FAST-LIO + body_controller、TCP endpoint無し）
ros2 launch core_launch navigation.launch.py environment:=real

# 実機モード（GUI・ショートカットから）
# .bashrc が非対話シェルで早期 return するため、このラッパーを使用
ros2 run core_launch navigation.sh

# ボディコントローラ単体
ros2 launch core_body_controller body_controller.launch.py

# 実機ハードウェア
ros2 launch core_hardware core_hardware.launch.py
```

Launch 引数の詳細はドキュメントサイトの「クイックスタート」を参照してください。
