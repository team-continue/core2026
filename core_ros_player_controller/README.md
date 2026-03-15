# core_ros_player_controller

CORE 2026 スタック向けのワイヤレスコントローラ解析ノードです。

## 起動時の動作

`wireless_parser_node` は `/wireless`（`std_msgs/msg/UInt8MultiArray`）を購読し、ボディ系・シューター系の制御トピックを発行します。

動作概要:
- `values[0]` のビットフラグからキー入力（W/A/S/D, reload, click, roller, emergency）を取得します。
- `values[1]`（X）と `values[2]`（Y）からマウス入力を取り出し、`[-1.0, 1.0]` に正規化して感度と反転を適用します。
- W/A/S/D とマウス X から `cmd_vel` を生成します。
- UI の自動フラグが OFF のとき、マウス Y からシューターのピッチ入力を生成します。
- UI の自動フラグが ON のときは、`manual_mode` / `test_mode` / `hazard_status` のみ publish します。
- UI の自動フラグが OFF のときは、ボディ・シューター系のトピックを publish します。
- `reloading` は立ち上がりエッジのみ publish します（手動モード時のみ）。

### パラメータ

- `mouse_x_sensitivity` (double, default: `1.0`)
- `mouse_y_sensitivity` (double, default: `1.0`)
- `mouse_x_inverse` (bool, default: `false`)
- `mouse_y_inverse` (bool, default: `false`)

デフォルトのパラメータは以下にあります:
- `config/wireless_parser_params.yaml`

## 起動方法（launch）

同梱の launch ファイルから起動する場合:

```bash
ros2 launch core_ros_player_controller wireless_parser_node.launch.py
```

パラメータファイルを指定する場合:

```bash
ros2 launch core_ros_player_controller wireless_parser_node.launch.py \
  params_file:=/path/to/params.yaml
```

## 別の launch ファイルから呼び出す remapping 例

別パッケージの launch からこの launch を呼び出し、トピックを remap する例です:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wireless_parser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("core_ros_player_controller"),
                "launch",
                "wireless_parser_node.launch.py",
            ])
        ),
        launch_arguments={
            "wireless": "/my_robot/wireless",
            "rotation": "/my_robot/rotation",
            "ads": "/my_robot/ads",
            "cmd_vel": "/my_robot/cmd_vel",
            "manual_mode": "/my_robot/manual_mode",
            "manual_pitch": "/my_robot/manual_pitch",
            "shoot_motor": "/my_robot/shoot_motor",
            "left_shoot_once": "/my_robot/left/shoot_once",
            "reloading": "/my_robot/reloading",
            "hazard_status": "/my_robot/system/emergency/hazard_status",
            "test_mode": "/my_robot/test_mode",
            "params_file": "/path/to/params.yaml",
        }.items(),
    )

    return LaunchDescription([
        wireless_parser_launch,
    ])
```
