# core_body_controller

車体モータ制御パッケージです。`/cmd_vel` をオムニホイールのCAN指令に変換します。

## 概要

- `/cmd_vel`（Twist）を受信し、4輪オムニホイールの逆運動学で各モータ指令に変換
- レートリミッタで加速度制限を適用
- 緊急停止機能
- 車体回転制御（PID）

## ノード

### body_control_node

`/cmd_vel` をオムニホイールのCAN指令に変換するメインノードです。

### target_angle_node

車体の回転角度を制御するノードです。IMUとエンコーダから姿勢を推定し、PID制御で目標角度に追従します。

- **回転モード**（`/rotation` が `true`）: IMUベースのワールド座標系で目標角度を追従。マウス入力（`cmd_vel.angular.z`）で目標角度を更新
- **非回転モード**（`/rotation` が `false`）: エンコーダベースで最寄りの90°角度に自動復帰

## 入力

### body_control_node

| トピック | 型 | 説明 |
|---------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令（vx, vy, ω） |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | 緊急停止フラグ |

### target_angle_node

| トピック | 型 | 説明 |
|---------|------|------|
| `/rotation` | `std_msgs/Bool` | 回転モード切替（プレイヤー操作） |
| `cmd_vel` | `geometry_msgs/Twist` | マウス入力からの角速度 |
| `imu` | `sensor_msgs/Imu` | IMU角速度（ヨー推定用） |
| `yaw_target_angle` | `std_msgs/Float64` | 車体回転目標角度（外部指定） |
| `body_omega` | `std_msgs/Float64` | 現在の車体角速度 |
| `joint_states` | `sensor_msgs/JointState` | エンコーダからの車体角度 |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | 緊急停止フラグ |

## 出力

### body_control_node

| トピック | 型 | 説明 |
|---------|------|------|
| `/can/tx` | `core_msgs/CANArray` | モータCAN指令 |
| `/body_omega` | `std_msgs/Float64` | 現在の角速度 |

### target_angle_node

| トピック | 型 | 説明 |
|---------|------|------|
| `can/tx` | `core_msgs/CANArray` | 回転モータCAN指令（ID=4） |
| `rotation_flag` | `std_msgs/Bool` | 現在の回転モード状態 |
| `target_omega` | `std_msgs/Float64` | 目標角速度（デバッグ用） |

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
