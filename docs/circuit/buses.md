# CANバスとモータID

upper TeensyはCAN2とCAN3の2系統を1 Mbpsで使用します。どちらもFlexCAN_T4の受信キュー256、送信キュー16で動作します。

!!! important "論理IDとCAN IDは別物です"
    `/can/tx`、`motor_ref[]`、`ecat_PacketCallBack()` が使う **論理ID** と、CANフレーム内でモータを識別する **CANプロトコルID** は異なります。たとえば論理ID 0のDamiaoは、ファームウェア上では `master_id=0x11`、`slave_id=0x01` です。

## CAN3

CAN3にはupper Teensy、Damiao 4台、RoboStride 06 1台、bottom Teensyが同一バス上に接続されます。

| 項目 | 現在値 |
|------|-------|
| ボーレート | 1 Mbps |
| Damiao台数 | 4 |
| RoboStride台数 | 1 |
| モータ送信間隔 | `CAN3_SEND_INTERVAL_US = 1000` µs |
| bottom問い合わせ周期 | `BOTTOM_REQUEST_INTERVAL_MS = 50` ms |
| bottom接続タイムアウト | `BOTTOM_CAN_TIMEOUT_MS = 200` ms |
| RoboStride速度上限 | 1.0 rad/s |
| RoboStride加速度上限 | 3.0 rad/s² |
| RoboStride初期モード | `Speed_control_mode` |

CAN3モータは5台をラウンドロビンし、1 ms以上の間隔で1台ずつ送信します。送信処理はモータの返信待ちに依存しません。bottomへの問い合わせもモータ送信とは独立して50 ms周期で実行します。

### CAN3デバイスID

| 論理ID | デバイス | ファームウェアのID設定 | 用途 |
|-------:|---------|-------------------------|------|
| 0 | Damiao | `master_id=0x11`, `slave_id=0x01` | 足回りモータ0 |
| 1 | Damiao | `master_id=0x12`, `slave_id=0x02` | 足回りモータ1 |
| 2 | Damiao | `master_id=0x13`, `slave_id=0x03` | 足回りモータ2 |
| 3 | Damiao | `master_id=0x14`, `slave_id=0x04` | 足回りモータ3 |
| 4 | RoboStride 06 | `master_id=0x01`, `motor_id=0x01` | 車体無限回転Yaw |
| — | bottom Teensy | 問い合わせ `0xff`、応答 `0x00`（標準ID） | 競技状態とLED |

DamiaoとRoboStrideは異なるCANプロトコルを使用するため、同じ数値のIDが含まれていてもフレーム形式と照合方法が異なります。

### upper ↔ bottomフレーム

upperは標準ID `0xff`、長さ2 Bでbottomへ問い合わせます。

| リクエスト | 現在の用途 |
|-----------|-----------|
| `buf[0]` | bottom LEDモード。upper LEDと同じ `LED_TAPE[0]` の下位8 bit |
| `buf[1]` | `LED_TAPE[2]` の下位8 bitを送信するが、bottom側では現在未使用 |

bottomは標準ID `0x00`、長さ3 Bで応答します。

| レスポンス | 内容 |
|-----------|------|
| `buf[0]` | destroy（撃破フラグ） |
| `buf[1]` | hp（残HP） |
| `buf[2]` | color（チーム色） |

## CAN2

CAN2にはupper TeensyとRoboStride 05 2台が接続されます。

| 項目 | 現在値 |
|------|-------|
| ボーレート | 1 Mbps |
| RoboStride台数 | 2 |
| 最小再送間隔 | `CAN2_RESEND_INTERVAL_MS = 1` ms |
| 応答待ちの送信フォールバック | `CAN2_TIMEOUT_MS = 1000` ms |
| 速度上限 | 1.0 rad/s |
| 加速度上限 | 3.0 rad/s² |
| 初期モード | `PosPP_control_mode` |
| 位置オフセット（論理ID 5） | `M_PI + 1.25` rad |
| 位置オフセット（論理ID 6） | `M_PI + 0.1` rad |

CAN2は1台への送信後に返信を待ち、返信を受けたら次のモータへ進みます。返信がない場合でも1000 ms経過後に送信を再開します。`CAN2_TIMEOUT_MS` はこの送信スケジューラのフォールバックであり、モータ接続判定のタイムアウトではありません。

### CAN2デバイスID

| 論理ID | デバイス | ファームウェアのID設定 | 用途 |
|-------:|---------|-------------------------|------|
| 5 | RoboStride 05 | `master_id=0xfd`, `motor_id=0x01` | 左砲台Yaw |
| 6 | RoboStride 05 | `master_id=0xfd`, `motor_id=0x02` | 右砲台Yaw |

## アプリ側論理ID

論理IDはROS 2の `/can/tx`、EtherCATの `motor_ref[]`、upperの指令ディスパッチで共通して使用します。CANの生IDではありません。

| 論理ID | アクチュエータ | 物理接続 | デバイスID | 用途 |
|-------:|--------------|---------|-----------|------|
| 0–3 | Damiao × 4 | CAN3 | slave ID 1–4 | 足回りオムニホイール |
| 4 | RoboStride 06 | CAN3 | motor ID 1 | 車体無限回転Yaw |
| 5 | RoboStride 05 | CAN2 | motor ID 1 | 左砲台Yaw |
| 6 | RoboStride 05 | CAN2 | motor ID 2 | 右砲台Yaw |
| 7 | Feetech STS | Serial7 | servo ID 1 | 右砲塔 Pitch |
| 8 | Feetech STS | Serial7 | servo ID 2 | 右砲塔 装填 |
| 9 | Feetech STS | Serial7 | servo ID 3 | 右砲塔 ディスク保持（右） |
| 10 | Feetech STS | Serial7 | servo ID 4 | 右砲塔 ディスク保持（左） |
| 11 | Feetech STS | Serial7 | servo ID 5 | 左砲塔 Pitch |
| 12 | Feetech STS | Serial7 | servo ID 6 | 左砲塔 装填 |
| 13 | Feetech STS | Serial7 | servo ID 7 | 左砲塔 ディスク保持（右） |
| 14 | Feetech STS | Serial7 | servo ID 8 | 左砲塔 ディスク保持（左） |
| 15 | ESC | PWM、ピン24 | — | 左砲塔 発射モータ |
| 16 | 未実装 | — | — | 右砲塔 発射モータ用としてlaunchに設定されるが、PDOとupper処理に未実装 |
| 17 | 非常停止指令 | GPIO 32 | — | `system_ref` として転送されるが、upperのGPIO処理はコメントアウト |

ID 0–4は `can3_motor[]`、5–6は `can2_motor[id-5]`、7–14は `sts.setRefPos(id-7, …)`、15は `esc.write()` にディスパッチされます。

!!! warning "論理ID 16と17はアクチュエータを動かしません"
    EtherCATの `motor_ref` はID 0–15までです。ID 16用の右ESC指令はPDOに存在せず、upperの `case 16` もコメントアウトされています。ID 17は別の `system_ref` でupperへ届きますが、非常停止GPIOの処理がコメントアウトされています。

## Feetechサーボ

| 項目 | 現在値 |
|------|-------|
| 台数 | 8（servo ID 1–8） |
| ボーレート | 1 Mbps（Serial7） |
| 制御周期 | 10 ms |
| 位置誤差が0.5 rad未満のPゲイン | 2.0 |
| 位置誤差が0.5 rad以上のPゲイン | 4.0 |
| 速度上限（STS3215） | 67 rpm相当 |
| 速度上限（STS3020） | 100 rpm相当 |
| 接続タイムアウト | 1000 ms |
| 再接続間隔 | 500 ms |
| SyncReadタイムアウト | 50 ms |

## 死活監視と安全動作

| 監視対象 | タイムアウト | 現在の動作 |
|---------|-------------|-----------|
| upperへのEtherCAT指令 | 500 ms | `hardware_enable=0`、`LED_BUILTIN`点灯、Feetechをdisable |
| bottomのCAN応答 | 200 ms | 全アクチュエータ指令を拒否し、上記と同じ安全状態へ移行 |
| upperから各CANモータへの指令 | 200 ms | Damiao / RoboStride側で指令切断として扱う |
| Damiao / RoboStrideのCAN応答 | 100 ms | 該当モータをCAN未接続として扱う |
| upperからbottomへの問い合わせ | 500 ms | bottomの `LED_BUILTIN` を点灯 |
| 無線受信 | 3000 ms | 接続状態は計算するが、無線機器未導入のため安全条件から一時的に除外 |

bottom応答はモータ応答より先に専用処理されます。これによりbottomの応答がモータの応答待ち状態を誤って解除することはありません。

EtherCATマスタ側の周期、WKC監視、PDO構成は [EtherCATとPDO](ethercat.md) を参照してください。

## 関連ページ

- [回路・通信構成](index.md)
- [基板とピン配置](boards.md)
- [EtherCATとPDO](ethercat.md)
- [メカ構成：駆動系・旋回機構](../mechanics/drivetrain.md)
- [メカ構成：砲塔・装填・発射機構](../mechanics/turret.md)
- [トピック・メッセージ一覧](../architecture/topics.md) — `core_msgs/CANArray` の定義
