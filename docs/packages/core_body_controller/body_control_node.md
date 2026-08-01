# body_control_node

## Purpose

上流のコントローラ（MPPIや手動操縦）が出力する速度指令は、車体座標系での並進・回転速度です。オムニホイール機構でこれを実現するには各ホイールの回転速度に分解する必要があります。このノードは逆運動学変換と、急激な指令変化を機構が許容できる範囲に抑えるレートリミットを担当します。

## Inner-workings / Algorithms

1. **受信**: `/cmd_vel` から並進速度（vx, vy）と角速度（ω）を取得
2. **緊急停止判定**: `/system/emergency/hazard_status` が true の場合、以降の処理をスキップして全モータへ停止指令を送出
3. **レートリミット**: 前回指令との差分を、並進は `acceleration`、回転は `rotation_acceleration` で制限。指令が急変してもホイールがスリップせず、駆動系への衝撃を抑える
4. **逆運動学**: オムニホイール配置に基づき、(vx, vy, ω) を各ホイールの回転速度に分解
5. **CAN指令生成**: 各ホイールのモータIDと速度をまとめて `/can/tx` に発行

`/rotation` の状態に応じて回転速度の扱いを切り替え、回転モードでは `/cmd_vel` の角速度成分を [target_angle_node](target_angle_node.md) 側の制御に委ねます。

現在の車体角速度は `joint_states` のホイールエンコーダから逆算し、`/body_omega` として発行します。この値は `target_angle_node` の内側ループで使用されます。

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
| `body_omega` | `std_msgs/Float64` | 現在の車体角速度 [rad/s] |

## Parameters

| パラメータ | 型 | 説明 |
|-----------|------|------|
| `acceleration` | double | 並進加速度制限 [m/s²] |
| `rotation_acceleration` | double | 回転加速度制限 [rad/s²] |
| `auto_rotation_velocity` | double | 自動回転モード時の回転速度 [rad/s] |
| `high_rotation_velocity` | double | 高速回転時の回転速度 [rad/s] |

## Assumptions / Known limits

- オムニホイール（全方向移動）機構を前提とした逆運動学です。差動二輪やメカナム以外の構成には適用できません。
- レートリミットは指令値に対してのみ作用します。実際のホイールが指令に追従できているかは検証しません（オープンループ）。
- 緊急停止時はCAN指令をゼロにするのみで、機械的なブレーキは作動させません。惰性による移動は残ります。
- `/cmd_vel` が途絶えた場合、このノードは自動停止しません。タイムアウトによる停止は上流の [core_cmd_vel_smoother](../core_cmd_vel_smoother/index.md) が担当します。
