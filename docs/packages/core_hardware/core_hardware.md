# core_hardware

## Purpose

ROS2側で生成されたモータ指令を実機に届け、実機の状態をROS2側に返すための境界ノードです。EtherCAT（SOEMライブラリ）を用いてTeensy41スレーブと通信し、モータ制御・LED制御・試合情報の受信を一手に担います。

## Inner-workings / Algorithms

### 権限分離とIPC

EtherCATの生パケット送受信には `NET_RAW` / `NET_ADMIN` 権限が必要です。ROSノード自体を特権で動かすことを避けるため、実際のEtherCAT通信は別プロセスの `core_hardware_daemon` が担当し、このノードとは `socket_path` のUNIXドメインソケット経由で通信します。

### 通信サイクル

1. `can/tx` で受け取ったCAN指令をモータIDごとに集約
2. LED制御トピック（`led/upper`, `led/bottom`, `led/bottom2`）の値を出力データに反映
3. デーモン経由でEtherCATのプロセスデータを送信し、スレーブからの応答を受信
4. 受信データを以下に分解して発行
   - モータのフィードバック → `can/rx` および `/joint_states`
   - ワイヤレスコントローラの生データ → `wireless`
   - 試合情報（HP、破壊判定） → `hp`, `destroy`
   - ハードウェア緊急停止スイッチ → `hardware_emergency`

`/joint_states` は全モータの角度・速度・トルクを含み、各制御ノードのフィードバックとしてだけでなく、[core_mode](../core_mode/index.md) の `diagnostic` によるマイコン生存監視にも使われます。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `can/tx` | `core_msgs/CANArray` | モータ指令（CAN ID別） |
| `led/upper` | `std_msgs/UInt8` | 上部LEDテープ制御 |
| `led/bottom` | `std_msgs/UInt8` | 下部LEDテープ制御 |
| `led/bottom2` | `std_msgs/UInt8` | 下部LEDテープ2制御 |
| `color` | `std_msgs/UInt8` | チームカラー設定（LED表示に反映） |

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
| `socket_path` | string | `/tmp/core_hardware.sock` | `core_hardware_daemon` とのIPCソケットパス |

## Assumptions / Known limits

- `core_hardware_daemon` が起動しており、指定した `socket_path` で待ち受けていることが前提です。デーモンが落ちるとモータ指令は一切届きません。
- EtherCATスレーブ（Teensy41）のEEPROMが正しく書き込まれている必要があります。手順は[ハードウェアセットアップガイド](../../guides/hardware-setup.md)を参照してください。
- `can/tx` に届いた指令の妥当性検証（速度上限や可動範囲）は行いません。安全確認は各制御ノードの責務です。
- 通信周期はEtherCATサイクルに従います。ROS側のトピック発行レートを上げても実効的な制御周期は変わりません。
