# target_angle_node

## Purpose

DM-IMU-L1内部EKFのYawと目標角の差から、車体無限回転Yawモータ（CAN ID 4）の角速度指令を生成します。`cmd_vel.angular.z` は目標角の変化率として扱われるため、入力を止めると最後の目標方位を保持します。

## Inner-workings / Algorithms

1. `imu.orientation`のQuaternionを正規化してYawを抽出し、最初の正常サンプルをYaw=0として相対角を作ります。±πをまたぐ差分はアンラップして連続角を維持します。
2. 新しい `/cmd_vel` が届いている間だけ `angular.z × 制御周期` を目標角へ加算します。
3. 正規化した目標角誤差をPIDへ入力し、`/rotation` が1または2なら `/body_omega` をフィードフォワードとして加えます。
4. 合成後の指令へ `yaw_rotation_velocity` と `yaw_rotation_acceleration` の制限を適用し、`/can/tx` と `/target_omega` へ発行します。

`/rotation` は `std_msgs/Int32` で、0は通常、1は通常回転、2は高速回転です。1と2はいずれもYaw側の回転補償を有効にし、ベースの速度差は `body_control_node` が選択します。

IMUが未受信または `imu_timeout_sec` より古い場合と、非常停止中は、保持されている古い制御値を使わずID 4へゼロ指令を継続送信します。タイムアウト後にIMUが復帰した最初のサンプルは、それまでの推定角を保ったまま新しいYaw基準として扱います。このためUSB再接続でIMU内部Yawが変化しても、復帰サンプル自体のジャンプは制御へ入りません。`cmd_vel` だけが期限切れになった場合は、IMUによる方位保持を継続しつつ目標角の更新を停止します。`body_omega` が期限切れになった場合は、PID制御を継続しつつ回転フィードフォワードだけを無効化します。

NaN、ゼロQuaternion、非現実的なノルムのQuaternionは破棄し、そのサンプルではIMU受信時刻を更新しません。`angular_velocity.z`は制御角の計算には使用しません。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | `angular.z`を目標角の変化率として使用 |
| `imu` | `sensor_msgs/Imu` | 正規化可能な姿勢Quaternion（launchでは`/imu`） |
| `yaw_target_angle` | `std_msgs/Float64` | ワールド基準の目標角 [rad] |
| `body_omega` | `std_msgs/Float64` | ベース角速度指令のフィードフォワード値 [rad/s] |
| `/rotation` | `std_msgs/Int32` | 回転モード（0=OFF、1=通常、2=高速） |
| `/system/emergency/hazard_status` | `std_msgs/Bool` | 非常停止フラグ |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `can/tx` | `core_msgs/CANArray` | ID 4のYawモータ指令。`data=[3, omega]` |
| `target_omega` | `std_msgs/Float64` | 制限適用後のYaw角速度指令 [rad/s] |

## Parameters

| パラメータ | 既定値 | 説明 |
|-----------|--------|------|
| `yaw_rotation_velocity` | `6.28` | 最終Yaw角速度の絶対上限 [rad/s] |
| `yaw_rotation_acceleration` | `3π` | 最終Yaw角速度の変化率上限 [rad/s²] |
| `cmd_vel_timeout_sec` | `0.2` | `/cmd_vel` を有効とみなす最大経過時間 [s] |
| `imu_timeout_sec` | `0.2` | IMUを有効とみなす最大経過時間 [s] |
| `body_omega_timeout_sec` | `0.2` | `/body_omega` を有効とみなす最大経過時間 [s] |

## Assumptions / Known limits

- DM-IMU-L1は6軸IMUのため、内部EKFのYawも絶対方位ではなく時間とともにドリフトします。
- 起動時の最初の正常姿勢が制御上のYaw=0です。
- PIDゲイン、デッドバンド、CAN IDは現在パラメータ化していません。
- `/body_omega` は実測値ではなく、`body_control_node` が生成したベース角速度指令です。
