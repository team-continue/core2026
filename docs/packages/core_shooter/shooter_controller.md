# shooter_controller

## Purpose

ディスクを射出するには、シューターモーターを目標速度まで加速させたうえで、ローディングモーターを適切なタイミングと間隔で回して装填する必要があります。このノードは発射数コマンドを受け取り、この一連の機構動作をCAN指令として組み立てます。ディスク詰まり（ジャム）の検出と復帰もここで扱います。左右タレットに1つずつ起動されます。

## Inner-workings / Algorithms

### 射撃シーケンス

1. `shoot_cmd`（発射数）を受信し、内部の残発射カウントに加算
2. `shoot_motor` で指定された目標速度までシューターモーターを加速。`shoot_motor_rotation_cmd_activation_delay_sec` の間は装填を開始せず、回転が乗るのを待つ
3. 発射モードに応じた間隔（`shoot_interval_ms` / `burst_interval_ms` / `fullauto_interval_ms`）でローディングモーターを `loading_motor_speed` で駆動
4. 1発ごとに `shoot_status` を発行し、`magazine_manager` に残弾を減算させる
5. 残発射カウントが0になるまで繰り返す

### ジャム検出

`enable_jam_detection` が有効な場合、`joint_states` のローディングモーター状態を監視し、指令を出しているにもかかわらず `jam_detect_time_sec` の間だけ回転が進まない状態をジャムと判定します。ジャム時は `jam_state` を発行して射撃を中断します。

### リグリップとの排他

`magazine_manager` がリグリップ動作中（`regrip_active` が true）の間は装填を行いません。ディスク保持機構が動いている最中に装填するとディスクを噛み込むため、両ノード間の排他制御をこのフラグで実現しています。

### 緊急停止・テストモード

`hazard_status` が true の間は全てのモータ指令を停止します。`/test_mode` が true の場合、シューターモーター回転の待機時間チェックなど一部の安全チェックをバイパスします。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `shoot_cmd` | `std_msgs/Int32` | 発射数コマンド（`shooter_cmd_gate` から） |
| `shoot_motor` | `std_msgs/Float32` | シューターモーター目標速度指令 |
| `/joint_states` | `sensor_msgs/JointState` | 全モータの角度・速度フィードバック |
| `jam` | `std_msgs/Bool` | 外部ジャムセンサ入力 |
| `regrip_active` | `std_msgs/Bool` | `magazine_manager` のリグリップ動作中フラグ |
| `hazard_status` | `std_msgs/Bool` | 緊急停止状態（`/system/emergency/hazard_status` にリマップ） |
| `/test_mode` | `std_msgs/Bool` | テストモード切替 |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `/can/tx` | `core_msgs/CANArray` | シューター・ローディングモーターのCAN指令 |
| `shoot_status` | `std_msgs/Bool` | 1発発射したことを示す通知 |
| `jam_state` | `std_msgs/Bool` | ジャム検出状態 |
| `loading_motor_error_state` | `std_msgs/Bool` | ローディングモーター異常 |
| `shoot_motor_error_state` | `std_msgs/Bool` | シューターモーター異常 |

## Parameters

設定ファイル: `config/shooter.params.yaml`

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `shoot_motor_id` | int | `15`（左）/ `16`（右） | シューターモーターCAN ID |
| `loading_motor_id` | int | `12`（左）/ `8`（右） | ローディングモーターCAN ID |
| `burst_count` | int | `3` | バーストモードの発数 |
| `shoot_interval_ms` | int | `500` | 単発射撃間隔 [ms] |
| `burst_interval_ms` | int | `500` | バースト射撃間隔 [ms] |
| `fullauto_interval_ms` | int | `500` | フルオート射撃間隔 [ms] |
| `shoot_motor_rotation_cmd_activation_delay_sec` | double | `2.0` | シューターモーター加速待ち時間 [s] |
| `loading_motor_speed` | double | `6.5` | ローディングモーター速度 [rev/s] |
| `target_speed` | double[] | `[2000, 1750, 1500]` | シューターモーター目標速度 |
| `limit_rad` | double[] | `[0,0,0,0]` | 可動範囲制限 [rad] |
| `set_initial_rad` | double | `0.0` | 初期角度オフセット [rad] |
| `enable_jam_detection` | bool | `false` | ジャム検出有効化 |
| `jam_detect_time_sec` | double | `0.1` | ジャム判定時間 [s] |
| `enable_panel_synchronizer` | bool | `true` | パネル同期機能の有効化 |
| `enable_test_mode` | bool | `false` | 起動時のテストモード既定値 |

## Assumptions / Known limits

- `/joint_states` が [core_hardware](../core_hardware/index.md) から届いていることが前提です。届かない場合ジャム検出は機能しません。
- ジャム検出は「指令に対して回転が進まない」ことのみを見ます。空撃ち（ディスクがないまま装填動作が完了する）はジャムとして検出されません。残弾の管理は `magazine_manager` の責務です。
- `enable_jam_detection` はデフォルトで無効です。実機で有効化する場合は `jam_detect_time_sec` を機構に合わせて調整してください。
- テストモード時は安全チェックの一部がバイパスされます。実機の機構に負荷がかかる可能性があるため、動作確認以外では使用しないでください。
