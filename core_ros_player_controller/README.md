# core_ros_player_controller

CORE 2026 スタック向けのワイヤレスコントローラ解析ノードです。

## 起動時の動作

`wireless_parser_node` は `/wireless`（`std_msgs/msg/UInt8MultiArray`）を購読し、ボディ系・シューター系の制御トピックを発行します。

動作概要（7Byteプロトコル）:
- `values[0]` のビットフラグから EStop / Roller / Reload / Shoot / ADS / turret-auto を取得します。
- `values[1]`（X）と `values[2]`（Y）からマウス入力を取り出し、`[-1.0, 1.0]` に正規化して感度と反転を適用します。
- W/A/S/D とマウス X から `cmd_vel` を生成します。
- `values[3]` の bit0-3 から W/A/S/D、bit4-5 から InfiniteRotate を取得します。
- `reloading` は立ち上がりエッジのみ publish します。

AutoMode 時の座標送信:
- `values[1..2]` を X、`values[5..6]` を Y として座標を取り出します（little endian）。
- 値は mm→m に変換して `/selected_pose`（`geometry_msgs/msg/PoseStamped`）へ送信します。
- 座標に変化があったときのみ publish します。
- `values[3]` の bit2 を `/auto_point_select`（`std_msgs/msg/Bool`）へ送信します（AutoMode 時のみ）。

### rotation の詳細

`/rotation` は `std_msgs/msg/Int32` で publish され、InfiniteRotate（0=OFF, 1=R1, 2=R2）を出力します。

- 参照元の入力: `values[3]` の bit4-5
- `manual_mode_target_side` に指定した側（`left` / `right`）のTurretAuto反転値を `manual_mode` に出力します。

### パラメータ

- `mouse_x_sensitivity` (double, default: `1.0`)
- `mouse_y_sensitivity` (double, default: `1.0`)
- `mouse_x_inverse` (bool, default: `false`)
- `mouse_y_inverse` (bool, default: `false`)
- `manual_mode_target_side` (string, default: `right`)

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
            "left_shoot_fullauto": "/my_robot/left/shoot_fullauto",
            "right_shoot_fullauto": "/my_robot/right/shoot_fullauto",
            "reloading": "/my_robot/reloading",
            "auto_point_select": "/my_robot/auto_point_select",
            "selected_pose": "/my_robot/selected_pose",
            "hazard_status": "/my_robot/system/emergency/hazard_status",
            "test_mode": "/my_robot/test_mode",
            "params_file": "/path/to/params.yaml",
        }.items(),
    )

    return LaunchDescription([
        wireless_parser_launch,
    ])
```
