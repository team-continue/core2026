# body_control_node

## Purpose

上流のコントローラ（MPPIや手動操縦）が出力する速度指令は、車体座標系での並進・回転速度です。オムニホイール機構でこれを実現するには各ホイールの回転速度に分解する必要があります。このノードは逆運動学変換と、急激な指令変化を機構が許容できる範囲に抑えるレートリミットを担当します。

## Inner-workings / Algorithms

1. **受信**: `/cmd_vel` から並進速度（vx, vy）と角速度（ω）を取得
2. **緊急停止判定**: `/system/emergency/hazard_status` が true の場合、以降の処理をスキップして全モータへ停止指令を送出
3. **レートリミット**: 前回指令との差分を、並進は `acceleration`、回転は `rotation_acceleration` で制限。指令が急変してもホイールがスリップせず、駆動系への衝撃を抑える
4. **逆運動学**: オムニホイール配置に基づき、(vx, vy, ω) を各ホイールの回転速度に分解
5. **CAN指令生成**: 各ホイールのモータIDと速度をまとめて `/can/tx` に発行

`/rotation` の状態に応じてベースへ加算する回転速度を切り替えます（0=加算なし、1=通常回転、2=高速回転）。[target_angle_node](target_angle_node.md) は同じモードを受け取り、回転中だけベース角速度指令をYaw制御へフィードフォワードします。

レート制限適用後のベース角速度指令は `/body_omega` として発行します。この値は実測角速度ではなく、`target_angle_node` の回転補償に使用するフィードフォワード値です。

`/cmd_vel` が `cmd_vel_timeout_sec` を超えて更新されない場合、`/cmd_vel` 由来の並進速度と角速度だけを既存の加速度制限に従ってゼロへ戻します。`/rotation` は独立した保持状態であり、モード1または2は明示的にモード0を受信するまで継続します。非常停止中はモードにかかわらず全速度指令を即座にゼロにします。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | 速度指令（linear.x, linear.y, angular.z） |
| `joint_states` | `sensor_msgs/JointState` | ホイールエンコーダのフィードバック |
| `/rotation` | `std_msgs/Int32` | 回転モード（[behavior_system_node](../core_behavior_system/behavior_system_node.md) または操縦入力から） |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | 緊急停止フラグ |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `can/tx` | `core_msgs/CANArray` | ホイールモータのCAN指令 |
| `body_omega` | `std_msgs/Float64` | レート制限後のベース角速度指令 [rad/s] |

## Parameters

| パラメータ | 型 | 説明 |
|-----------|------|------|
| `acceleration` | double | 並進加速度制限 [m/s²] |
| `rotation_acceleration` | double | 回転加速度制限 [rad/s²] |
| `auto_rotation_velocity` | double | 自動回転モード時の回転速度 [rad/s] |
| `high_rotation_velocity` | double | 高速回転時の回転速度 [rad/s] |
| `cmd_vel_timeout_sec` | double | `/cmd_vel` を有効とみなす最大経過時間 [s] |

## Assumptions / Known limits

- オムニホイール（全方向移動）機構を前提とした逆運動学です。差動二輪やメカナム以外の構成には適用できません。
- レートリミットは指令値に対してのみ作用します。実際のホイールが指令に追従できているかは検証しません（オープンループ）。
- 緊急停止時はCAN指令をゼロにするのみで、機械的なブレーキは作動させません。惰性による移動は残ります。
- `joint_states.position[4]` が不足または非有限値の場合、そのメッセージを無視して最後の正常な車体角を維持します。
