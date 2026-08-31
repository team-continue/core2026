# core_hardware

ROS2と実機ハードウェアの境界を担うパッケージです。上位ノードが発行するCAN指令をマイコンへ送出し、モータのフィードバックやセンサ値・試合情報をROSトピックとして公開します。

## ノード一覧

| ノード | 役割 |
|-------|------|
| [`core_hardware`](core_hardware.md) | EtherCAT（SOEM）経由のハードウェアインターフェース。実機の標準構成 |
| [`core_hardware_usb`](core_hardware_usb.md) | USBシリアル経由のハードウェアインターフェース。EtherCATを使わない簡易構成 |

`core_hardware_daemon` は権限分離のためのデーモンプロセスで、ROSノードではありません。EtherCAT通信に必要な権限を持つ側で実行され、`core_hardware` とはUNIXドメインソケットで通信します。

## データフロー

```
上位ノード ──/can/tx──▶ core_hardware ──EtherCAT──▶ Teensy41スレーブ ──▶ モータESC
                              ▲                            │
                              └──/can/rx, /joint_states ────┘
```

CAN指令を発行するのは [core_body_controller](../core_body_controller/index.md) と [core_shooter](../core_shooter/index.md) の各ノードで、それらが同一の `/can/tx` トピックに集約されます。逆方向のフィードバック（`/joint_states`）は各制御ノードと [core_mode](../core_mode/index.md) の `diagnostic` が参照します。

## 構成の選択

| 構成 | ノード | 用途 |
|------|-------|------|
| EtherCAT | `core_hardware` | 実機の標準構成。多数のモータを低遅延で制御 |
| USB | `core_hardware_usb` | 単体テストや簡易構成。EtherCAT環境が不要 |

## 起動

```bash
ros2 launch core_hardware core_hardware.launch.py
```

!!! warning "権限"
    EtherCAT通信には `NET_RAW` / `NET_ADMIN` 権限が必要です。Docker環境では `docker-compose.yaml` で設定済みです。

EtherCATデーモンを含む実機システムの起動手順は[クイックスタート](../../getting-started/quick-start.md)を参照してください。
