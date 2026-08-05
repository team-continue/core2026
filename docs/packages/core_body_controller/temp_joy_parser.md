# temp_joy_parser

## Purpose

試合用のワイヤレス受信機がなくても車体やモータの動作確認ができるよう、汎用ゲームパッド（`sensor_msgs/Joy`）から直接指令を出すための開発用ノードです。名前の通り暫定的な位置づけで、試合構成には含まれません。

## Inner-workings / Algorithms

`joy` トピックを購読し、ボタンとスティックの状態を個別のトピックに展開します。左スティックを `cmd_vel` の並進成分に、各ボタンを `pad/<ボタン名>` のBoolトピックにマッピングする単純な変換です。

ボタンごとに独立したトピックへ展開しているため、デバッグ時に任意のボタンを任意のノードへ直接つなぎ替えられます。一部のボタンはモータへのCAN指令（`can/tx`）に直結しています。

### ボタンマッピング

| トピック | 対応 |
|---------|------|
| `pad/circle`, `pad/cross`, `pad/square`, `pad/triangle` | ○×□△ボタン |
| `pad/up`, `pad/down`, `pad/left`, `pad/right` | 十字キー |
| `pad/l1`, `pad/l2`, `pad/l3` | 左トリガー・スティック押込 |
| `pad/r1`, `pad/r2`, `pad/r3` | 右トリガー・スティック押込 |
| `pad/share`, `pad/options`, `pad/ps` | システムボタン |
| `pad/r_stick_vertical` | 右スティック垂直軸 |

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `joy` | `sensor_msgs/Joy` | ゲームパッド入力（`joy_node` から） |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | 車体速度指令 |
| `can/tx` | `core_msgs/CANArray` | モータCAN指令（一部ボタンに直結） |
| `pad/*` | `std_msgs/Bool` ほか | 各ボタン・軸の状態（上表参照） |

## Parameters

パラメータはありません。マッピングはソースにハードコードされています。

## Assumptions / Known limits

- 開発・デバッグ専用のノードです。試合用のランチファイルには含めないでください。手動操縦の正規の入口は [core_ros_player_controller](../core_ros_player_controller/index.md) です。
- 緊急停止（`hazard_status`）を購読しません。このノードを起動している間は、緊急停止をかけてもこのノード経由のCAN指令は止まりません。実機での使用時は十分注意してください。
- `joy_node`（`joy` パッケージ）が別途起動している必要があります。
- ボタン配列のインデックスはDualShock系のコントローラを前提としています。他社製パッドではマッピングがずれます。

## 起動

```bash
# 別途 joy_node が必要
ros2 run joy joy_node
ros2 run core_body_controller temp_joy_parser
```
