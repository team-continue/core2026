# core_hardware

EtherCAT（SOEM）によるハードウェアインターフェースパッケージです。

## 概要

- SOEM ライブラリを使用したEtherCAT通信
- Teensy41スレーブコントローラとの通信
- モータESC制御
- HP・破壊判定・緊急停止の受信
- LEDテープ制御

## 構成

```
core_hardware_node ──EtherCAT──▶ Teensy41スレーブ ──▶ モータESC
```

## 入力（Subscribe）

| トピック | 型 | 説明 |
|---------|------|------|
| `can/tx` | `core_msgs/CANArray` | モータ指令（CAN ID別） |
| `led/upper` | `std_msgs/UInt8` | 上部LEDテープ制御 |
| `led/bottom` | `std_msgs/UInt8` | 下部LEDテープ制御 |
| `led/bottom2` | `std_msgs/UInt8` | 下部LEDテープ2制御 |

## 出力（Publish）

| トピック | 型 | 説明 |
|---------|------|------|
| `can/rx` | `core_msgs/CANArray` | モータフィードバック |
| `/joint_states` | `sensor_msgs/JointState` | 全モータの角度・速度・トルク |
| `wireless` | `std_msgs/UInt8MultiArray` | ワイヤレスコントローラ生データ |
| `hp` | `std_msgs/UInt8` | ロボットHP値 |
| `destroy` | `std_msgs/Bool` | 破壊判定フラグ |
| `hardware_emergency` | `std_msgs/Bool` | ハードウェア緊急停止 |

## パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `socket_path` | `/tmp/core_hardware.sock` | ハードウェアデーモンとのIPCソケットパス |

## 起動

```bash
ros2 launch core_hardware core_hardware.launch.py
```

!!! warning "権限"
    EtherCAT通信には `NET_RAW` / `NET_ADMIN` 権限が必要です。Docker環境では `docker-compose.yaml` で設定済みです。

セットアップ手順（EEPROM書き込み、通信テスト、LiDAR接続など）は[ハードウェアセットアップガイド](../guides/hardware-setup.md)を参照してください。
