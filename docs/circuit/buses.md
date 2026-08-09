# CANバスとモータID

## CANバス構成

いずれも FlexCAN_T4（受信バッファ 256、送信バッファ 16）を使用し、1 ms 間隔でラウンドロビン送信します。

### CAN3（足回り + 無限回転Yaw + bottom）

| 定数 | 値 |
|------|-----|
| `CAN3_NUM_DAMIAO` | 4 |
| `CAN3_NUM_ROBOSTRIDE` | 1 |
| `CAN3_NUM_BOTTOM` | 1 |
| `CAN3_RESEND_INTERVAL_MS` | 1 |
| `CAN3_TIMEOUT_MS` | 1000 |
| `CAN3_RS05_SPEED_LIMIT` | 1.0 rad/s |
| `CAN3_RS05_ACC_LIMIT` | 3.0 rad/s² |
| 初期制御モード | `Speed_control_mode` |

### CAN2（砲台Yaw）

| 定数 | 値 |
|------|-----|
| `CAN2_NUM_MOTOR` | 2（すべて RoboStride） |
| `CAN2_RESEND_INTERVAL_MS` | 1 |
| `CAN2_TIMEOUT_MS` | 1000 |
| `CAN2_RS05_SPEED_LIMIT` | 1.0 rad/s |
| `CAN2_RS05_ACC_LIMIT` | 3.0 rad/s² |
| 初期制御モード | `PosPP_control_mode` |

## モータID割り当て

EtherCAT の float パケットIDが、そのままモータIDになります。

| ID | アクチュエータ | バス / 接続 | 用途 |
|----|--------------|-----------|------|
| 0–3 | Damiao × 4 | CAN3 | 足回りオムニホイール |
| 4 | RoboStride 06 | CAN3 | 車体無限回転Yaw |
| 5 | RoboStride 05 | CAN2 | 左砲台Yaw |
| 6 | RoboStride 05 | CAN2 | 右砲台Yaw |
| 7 | Feetech STS | Serial7 | 右砲塔 Pitch |
| 8 | Feetech STS | Serial7 | 右砲塔 装填 |
| 9 | Feetech STS | Serial7 | 右砲塔 ディスク保持（右） |
| 10 | Feetech STS | Serial7 | 右砲塔 ディスク保持（左） |
| 11 | Feetech STS | Serial7 | 左砲塔 Pitch |
| 12 | Feetech STS | Serial7 | 左砲塔 装填 |
| 13 | Feetech STS | Serial7 | 左砲塔 ディスク保持（右） |
| 14 | Feetech STS | Serial7 | 左砲塔 ディスク保持（左） |
| 15 | ESC | PWM（ピン24） | 左砲塔 発射モータ |
| 16 | ESC | — | 右砲塔 発射モータ（**ファームウェア未実装**） |
| 17 | — | GPIO 32 | 非常停止（**ファームウェアでコメントアウト**） |

ID 0–4 は `can3_motor[]`、5–6 は `can2_motor[id-5]`、7–14 は `sts.setRefPos(id-7, …)` にディスパッチされます（`upper/src/main.cpp` の `ecat_PacketCallBack`）。

!!! warning "ID 16 と 17 は現在のファームウェアで無効です"
    `ecat_PacketCallBack` の `case 16:`（右ESC）と `case 17:`（非常停止のGPIO出力）はどちらもコメントアウトされています。ROS2側は `shooter.launch.py` で ID 16 を設定していますが、指令は届きません。

### Feetechサーボ

| 項目 | 値 |
|------|-----|
| 台数（`LEN_SERVO`） | 8 |
| ボーレート | 1 Mbps（Serial7） |
| 制御周期 | 10 ms（`STS_CONTROL_INTERVAL_US`） |
| 位置PID P項 | 2.0 |
| 速度上限（STS3215） | 67 rpm 相当 |
| 速度上限（STS3020） | 100 rpm 相当 |
| 接続タイムアウト | 1000 ms |
| 再接続間隔 | 500 ms |
| SyncReadタイムアウト | 50 ms |

## EtherCAT PDO / パケット構成

### 状態フィードバック（Teensy → PC、float × 6）

全15軸（ID 0–14）について、各軸6要素のfloat配列を送信します。

| 要素 | 内容 |
|------|------|
| `[0]` | 予約（未使用） |
| `[1]` | 実トルク / 電流 |
| `[2]` | 目標速度 |
| `[3]` | 実速度 |
| `[4]` | 目標位置 |
| `[5]` | 実位置 |

### 状態フィードバック（Teensy → PC、uint8）

| パケットID | 内容 | サイズ |
|-----------|------|-------|
| 100 | damage（HP） | 1 B |
| 101 | destroy（撃破フラグ） | 1 B |
| 102 | wireless（受信機データ） | 7 B |
| 103 | color（チーム色） | 1 B |
| 104 | hardware_enable | 1 B |

### 指令（PC → Teensy、uint8）

| パケットID | 内容 |
|-----------|------|
| 100 | `data[0]`: upper LED、`data[1..2]`: bottom LED（`setLedBytes`） |

!!! warning "wireless と color はPDO上で多重化されています"
    `core_hardware/README.md` の記載通り、`wireless` 7バイトに加えて末尾1バイトを `color` に使い、`motor_state_torque[0..3]` の先頭8バイトに載せています。多重化は `core_hardware_daemon` と Teensy の間で吸収され、ROS2側の `/wireless` トピックのインタフェースは維持されます。

    そのため **EtherCATレイアウトを変更するときは `utypes.h` だけでなく `objectlist.c` / `esi.json` / PC側 `ecat.cpp` を必ず同時に揃えてください**。ID 0–3 の `torque` は通常用途では使いません。

## 死活監視と非常停止

| 監視対象 | 条件 | 動作 |
|---------|------|------|
| ROS2接続（upper） | 最終受信から 500 ms 経過 | `LED_BUILTIN` 点灯、Feetechサーボを `disable` |
| CAN3受信（bottom） | 最終受信から 500 ms 経過 | `LED_BUILTIN` 点灯 |
| CANモータ | `CAN2/CAN3_TIMEOUT_MS` = 1000 ms | 該当モータを未接続扱い |

`led_timer` は upper / bottom ともに 50 ms 周期（20 Hz）で上記の判定を行います。ROS2側の死活監視は `core_mode` の `diagnostic` ノードが `/joint_states` と `/wireless` のハートビートで実施します（[システム概要](../architecture/overview.md) を参照）。

## 関連ページ

- [基板とピン配置](boards.md)
- [メカ構成：駆動系・旋回機構](../mechanics/drivetrain.md)
- [メカ構成：砲塔・装填・発射機構](../mechanics/turret.md)
- [トピック・メッセージ一覧](../architecture/topics.md) — `core_msgs/CANArray` の定義
