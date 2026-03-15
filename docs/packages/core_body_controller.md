# core_body_controller

車体モータ制御パッケージです。`/cmd_vel` をオムニホイールのCAN指令に変換します。

## 概要

- `/cmd_vel`（Twist）を受信し、4輪オムニホイールの逆運動学で各モータ指令に変換
- レートリミッタで加速度制限を適用
- 緊急停止機能
- 車体回転制御（PID）

## 入力

| トピック | 型 | 説明 |
|---------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令（vx, vy, ω） |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | 緊急停止フラグ |
| `/body_target_angle` | `std_msgs/Float64` | 車体回転目標角度 |
| `/joint_states` | `sensor_msgs/JointState` | 現在の車体角度 |
| `/rotation_flag` | `std_msgs/Bool` | 回転モードフラグ |
| `/pad/up` | `std_msgs/Bool` | ゲームパッド入力 |

## 出力

| トピック | 型 | 説明 |
|---------|------|------|
| `/can/tx` | `core_msgs/CANArray` | モータCAN指令 |
| `/body_omega` | `std_msgs/Float64` | 現在の角速度 |

## パラメータ

| パラメータ | 型 | 説明 |
|-----------|------|------|
| `acceleration` | double | 並進加速度制限 |
| `rotation_acceleration` | double | 回転加速度制限 |
| `yaw_rotation_velocity` | double | ヨー回転速度 |
| `auto_rotation_velocity` | double | 自動回転速度 |

## 起動

```bash
ros2 launch core_body_controller body_controller.launch.py
```
