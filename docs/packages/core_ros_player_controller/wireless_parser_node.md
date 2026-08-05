# wireless_parser_node

## Purpose

`/wireless` の7Byte入力を解析し、車体移動・砲塔操作・射撃・緊急停止などのROSトピックへ変換するノードです。7バイト未満のパケットは無視します。

## Inner-workings / Algorithms

パケット受信時にbyte 0とbyte 3のビットを展開し、マウス入力を正規化して各トピックへ発行します。Reloadは立ち上がりエッジでのみ発行するため、ボタンを押し続けても連続発火しません。Shootは`manual_mode_target_side`で指定した側の`shoot_fullauto`にだけ発行されます。

## Input format

パケットは次の形式です。

`[flags, mouse_x, mouse_y, movement, reserved, reserved, reserved]`

`mouse_x` と `mouse_y` は符号付き8bitの2の補数で、`-127..127`を`-1.0..1.0`へ正規化します。

### byte 0: flags

| Bit | 内容 |
|---:|---|
| 0 | EStop |
| 1 | Roller |
| 2 | Reload |
| 3 | Shoot |
| 4 | ADS |
| 5 | LeftTurretAuto |
| 6 | RightTurretAuto |
| 7 | Reserved |

### byte 3: movement

| Bit | 内容 |
|---:|---|
| 0 | W（前進） |
| 1 | A（左移動） |
| 2 | S（後退） |
| 3 | D（右移動） |
| 4-5 | InfiniteRotate（`0=OFF`, `1=R1`, `2=R2`） |

## Inputs / Outputs

### Input

| Topic / Service | Type | QoS | 内容 |
|---|---|---|---|
| `/wireless` | `std_msgs/msg/UInt8MultiArray` | reliable(10) | 7バイト以上のワイヤレス入力 |

### Output

有効なパケットを受信するたびに、次のトピックを発行します。

| Topic / Service | Type | QoS | 内容 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | reliable(10) | WASDによる移動と`mouse_x`による旋回 |
| `/rotation` | `std_msgs/msg/Int32` | reliable(10) | InfiniteRotateの値 |
| `/ads` | `std_msgs/msg/Bool` | reliable(10) | ADS状態 |
| `/left/turret_auto` | `std_msgs/msg/Bool` | reliable(10) | 左砲塔の自動制御フラグ |
| `/right/turret_auto` | `std_msgs/msg/Bool` | reliable(10) | 右砲塔の自動制御フラグ |
| `/manual_pitch` | `std_msgs/msg/Float32` | reliable(10) | `mouse_y`による手動ピッチ入力 |
| `/shoot_motor` | `std_msgs/msg/Bool` | reliable(10) | ローラーモーター制御 |
| `/reloading` | `std_msgs/msg/Bool` | reliable(10) | Reloadの立ち上がりエッジでのみ発行 |
| `/system/emergency/hazard_status` | `std_msgs/msg/Bool` | reliable(10) | EStop状態 |
| `/test_mode` | `std_msgs/msg/Bool` | reliable(10) | 常に`false` |
| `/manual_mode` | `std_msgs/msg/Bool` | reliable(10) | 対象側の自動制御フラグを反転した手動モード |

Shootは`manual_mode_target_side`で指定した側にだけ発行します。

| Topic / Service | Type | QoS | 内容 |
|---|---|---|---|
| `/left/shoot_fullauto` | `std_msgs/msg/Bool` | reliable(10) | 左側を選択した場合の射撃トリガー |
| `/right/shoot_fullauto` | `std_msgs/msg/Bool` | reliable(10) | 右側を選択した場合の射撃トリガー |

`/manual_mode` も常に発行され、選択側の砲塔自動制御フラグを反転した値になります。例えば対象側が`right`の場合、`/right/turret_auto=false`で`/manual_mode=true`です。

## Parameters

設定ファイルは`config/wireless_parser_params.yaml`です。

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---:|---|
| `mouse_x_sensitivity` | double | `1.0` | 水平マウス感度 |
| `mouse_y_sensitivity` | double | `1.0` | 垂直マウス感度 |
| `mouse_x_inverse` | bool | `false` | 水平入力の反転 |
| `mouse_y_inverse` | bool | `false` | 垂直入力の反転 |
| `cmd_vel_xy_scale` | double | `1.0` | XY速度のスケール |
| `manual_mode_target_side` | string | `right` | `/manual_mode`とShootの対象側（`left`または`right`） |

## Assumptions / Known limits

- `/wireless` のペイロードは7バイト以上である必要があります。短いパケットは警告を出して無視します。
- マウス入力は受信したパケットごとに発行されます。入力が途絶えた場合、このノード自身はゼロ速度を発行しません。
- `/manual_mode` と射撃トリガーの対象側は、`manual_mode_target_side` の値（`left`または`right`）で決まります。
- Reloadは立ち上がりエッジのみ発行されるため、押下中の繰り返し発行はありません。

## Launch

```bash
ros2 launch core_ros_player_controller wireless_parser_node.launch.py
```
