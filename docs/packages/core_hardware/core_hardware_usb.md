# core_hardware_usb

## Purpose

EtherCAT構成は特権とネットワーク設定を必要とし、単体テストや開発機では準備が煩雑です。このノードはUSBシリアル経由で同等のインターフェースを提供し、EtherCAT環境なしでモータ制御と状態取得を行えるようにします。

## Inner-workings / Algorithms

`port` で指定したシリアルポート（デフォルト `/dev/ttyACM0`）を開き、マイコンとバイト列でやり取りします。

1. `can/tx` で受け取ったCAN指令をシリアルフレームにエンコードして送信
2. マイコンからの応答フレームをデコード
3. [core_hardware](core_hardware.md) と同じトピック（`can/rx`, `/joint_states`, `wireless`, `hp`, `destroy`, `hardware_emergency`）に分解して発行

EtherCAT版と同一のトピックインターフェースを提供するため、上位ノードはどちらの構成で動いているかを意識する必要がありません。ランチ側でどちらを起動するかを選ぶだけで切り替えられます。

LED制御トピックは扱いません。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `can/tx` | `core_msgs/CANArray` | モータ指令（CAN ID別） |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `can/rx` | `core_msgs/CANArray` | モータフィードバック |
| `/joint_states` | `sensor_msgs/JointState` | 全モータの角度・速度・トルク |
| `wireless` | `std_msgs/UInt8MultiArray` | ワイヤレスコントローラ生データ |
| `hp` | `std_msgs/UInt8` | ロボットHP値 |
| `destroy` | `std_msgs/Bool` | 破壊判定フラグ |
| `hardware_emergency` | `std_msgs/Bool` | ハードウェア緊急停止 |

## Parameters

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `port` | string | `/dev/ttyACM0` | 接続するシリアルポート |

## Assumptions / Known limits

- LEDテープ制御（`led/upper`, `led/bottom`, `led/bottom2`）には対応していません。LED制御が必要な場合は [core_hardware](core_hardware.md) を使用してください。
- USBシリアルはEtherCATに比べて帯域・遅延の面で劣ります。多数のモータを高レートで制御する試合構成には適しません。
- デバイスファイル（`/dev/ttyACM*`）への読み書き権限が必要です。ユーザーを `dialout` グループに追加するか、udevルールを設定してください。
- ポート番号はUSBの接続順で変動します。複数のシリアルデバイスを接続している場合は `/dev/serial/by-id/` の永続パスを指定することを推奨します。
- `core_hardware` と同時に起動しないでください。同一の `/can/tx` を購読して指令が二重に送出されます。
