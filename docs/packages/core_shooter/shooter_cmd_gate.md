# shooter_cmd_gate

## Purpose

操縦側は「単発を撃つ」「バーストを撃つ」といった意図をBool形式で送ってきますが、`shooter_controller` は発射数を表す数値コマンドを必要とします。またデュアルタレット構成では、手動照準モード時にどちらのタレットを操作対象にするかを決める必要があります。このノードは両者の間に立ち、射撃意図を数値コマンドに変換して適切なタレットへ振り分けます。

## Inner-workings / Algorithms

各射撃トピックを受信すると、モードに応じた発射数を左右それぞれの `shoot_cmd`（`std_msgs/Int32`）として発行します。

| 入力 | 発行される値 |
|------|------------|
| `shoot_once` | 1 |
| `shoot_burst` | `burst_count`（デフォルト3） |
| `shoot_fullauto` | フルオート継続を示す値 |

手動照準モード（`manual_mode`）では、`manual_mode_target_side` パラメータで指定された側のタレットにのみ `manual_mode` / `manual_pitch_angle` を転送します。これにより、操縦者のピッチ入力が両タレットに同時に効いてしまうことを防ぎます。

`shoot_motor_state` がONの場合、`shoot_motor_on_command` の値をシューターモーター指令として左右に発行します。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `left/shoot_once`, `right/shoot_once` | `std_msgs/Bool` | 単発射撃コマンド |
| `left/shoot_burst`, `right/shoot_burst` | `std_msgs/Bool` | バースト射撃コマンド |
| `left/shoot_fullauto`, `right/shoot_fullauto` | `std_msgs/Bool` | フルオート射撃コマンド |
| `manual_mode` | `std_msgs/Bool` | 手動照準モード切替（`/manual_mode` にリマップ） |
| `manual_pitch` | `std_msgs/Float32` | 手動ピッチ入力（`/manual_pitch` にリマップ） |
| `shoot_motor_state` | `std_msgs/Bool` | シューターモーターON/OFF状態 |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `left_shoot_cmd` → `/left/shoot_cmd` | `std_msgs/Int32` | 左タレットの発射数コマンド |
| `right_shoot_cmd` → `/right/shoot_cmd` | `std_msgs/Int32` | 右タレットの発射数コマンド |
| `left_manual_mode` → `/left/manual_mode` | `std_msgs/Bool` | 左タレットの手動モード状態 |
| `right_manual_mode` → `/right/manual_mode` | `std_msgs/Bool` | 右タレットの手動モード状態 |
| `left_manual_pitch_angle` → `/left/manual_pitch_angle` | `std_msgs/Float32` | 左タレットへの手動ピッチ指令 |
| `right_manual_pitch_angle` → `/right/manual_pitch_angle` | `std_msgs/Float32` | 右タレットへの手動ピッチ指令 |
| `/left/shoot_motor` | `std_msgs/Float32` | 左シューターモーター指令 |
| `/right/shoot_motor` | `std_msgs/Float32` | 右シューターモーター指令 |

## Parameters

設定ファイル: `config/shooter.params.yaml`

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `burst_count` | int | `3` | バーストモードで発行する発射数 |
| `shoot_motor_on_command` | double | `2000.0` | シューターモーターON時に発行する指令値 |
| `manual_mode_target_side` | string | `right` | 手動照準モードで操作対象とするタレット（`left` / `right`） |

## Assumptions / Known limits

- このノードは射撃可否の判断を行いません。緊急停止や残弾の確認は下流の `shooter_controller` と `magazine_manager` の責務です。
- 手動照準の操作対象は `manual_mode_target_side` で固定されます。実行中に左右を切り替えることはできません。
- 発射数コマンドを発行するだけで、実際に何発発射されたかのフィードバックは受け取りません。
