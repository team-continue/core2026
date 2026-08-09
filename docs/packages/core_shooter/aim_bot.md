# aim_bot

## Purpose

ビジョンが検出した敵ダメージパネルの画像位置に砲身を向けるための照準ノードです。画像座標の誤差をヨー・ピッチのモータ角度指令に変換し、機構やルール上の可動制限を守りながら追尾します。手動照準モードでは操縦者のピッチ入力に従います。左右タレットに1つずつ起動されます。

## Inner-workings / Algorithms

`rate`（デフォルト30Hz）の周期で制御ループを回します。

### 自動追尾

1. `target_image_position` で受け取ったターゲットの画像座標と画像中心（`image_center_x` / `image_center_y`）との差分を求める
2. `use_fov_image_tracking` が有効な場合、`horizontal_fov_deg` を用いてピクセル誤差を角度誤差に換算する
3. 誤差が `image_tolerance_x` / `image_tolerance_y` 以内なら不感帯として指令を出さない
4. `yaw_image_gain` / `pitch_image_gain` を掛けて角度指令の増分とし、`max_yaw_rate` / `max_pitch_rate` で変化率を制限
5. `joint_states` の現在角度と突き合わせ、CAN指令として `/can/tx` に発行

ターゲットの移動速度を `target_velocity_ema_alpha` で平滑推定し、`target_lead_time_sec` の分だけ未来位置を予測する偏差リード補正を持ちます（デフォルトは0で無効）。

`target_timeout_sec` の間ターゲットが検出されないとロスト扱いになり、`target_lost_return_to_startup_delay_sec` 経過後に `startup_release_yaw_angle` / `startup_release_pitch_angle` の待機姿勢へ戻ります。

### ゾーン別角度制限

`enable_zone_angle_limit` が有効な場合、ヨー角を3つのゾーンに分割し、ゾーンごとに異なるピッチ可動範囲を適用します。自陣方向を向いているときに砲身を上げられないようにするなど、機構干渉とルール上の制約を表現するための仕組みです。

ゾーン境界での指令のばたつきを防ぐため、`control.hysteresis_rad` のヒステリシスを持たせています。

### 手動モード

`manual_mode` が true の間は、ヨーを `manual_mode_yaw_fixed_angle` に固定し、ピッチのみ `manual_pitch_angle` の入力に従います。

### 緊急停止・テストモード

`hazard_status` が true の間はモータ指令を停止します。`/test_mode` が true の場合、`test_yaw_angle` / `test_pitch_angle` で直接角度を与えられます。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `target_image_position` | `geometry_msgs/PointStamped` | ターゲットの画像座標（launchで `/left/target_pose` 等にリマップ） |
| `manual_mode` | `std_msgs/Bool` | 手動照準モード切替 |
| `manual_pitch_angle` | `std_msgs/Float32` | 手動ピッチ角指令 |
| `/joint_states` | `sensor_msgs/JointState` | ヨー・ピッチモータの現在角度 |
| `hazard_status` | `std_msgs/Bool` | 緊急停止状態（`/system/emergency/hazard_status` にリマップ） |
| `/test_mode` | `std_msgs/Bool` | テストモード切替 |
| `test_yaw_angle` | `std_msgs/Float32` | テストモード時のヨー角直接指定 |
| `test_pitch_angle` | `std_msgs/Float32` | テストモード時のピッチ角直接指定 |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `/can/tx` | `core_msgs/CANArray` | ヨー・ピッチモータのCAN指令 |

## Parameters

設定ファイル: `config/shooter.params.yaml`

### モータ・基本設定

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `pitch_motor_id` | int | `11`（左）/ `7`（右） | ピッチモーターCAN ID |
| `yaw_motor_id` | int | `5`（左）/ `6`（右） | ヨーモーターCAN ID |
| `rate` | double | `30.0` | 制御ループ周波数 [Hz] |
| `pitch_offset` | double | `0.0` | ピッチ角オフセット [rad] |
| `yaw_min_angle` / `yaw_max_angle` | double | `-π` / `π` | ヨー可動範囲 [rad] |
| `pitch_min_angle` / `pitch_max_angle` | double | `-π` / `π` | ピッチ可動範囲 [rad] |

### 画像追尾

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `image_width` / `image_height` | double | `1280.0` / `720.0` | 入力画像サイズ [px] |
| `image_center_x` / `image_center_y` | double | `0.5` / `0.5` | 照準中心（正規化座標） |
| `horizontal_fov_deg` | double | `100.0` | カメラ水平視野角 [deg] |
| `use_fov_image_tracking` | bool | `true` | FOVを用いたピクセル→角度換算を使う |
| `image_tolerance_x` / `image_tolerance_y` | double | `8.0` / `8.0` | 追尾の不感帯 [px] |
| `yaw_image_gain` / `pitch_image_gain` | double | `0.0005` | 画像追尾ゲイン |
| `yaw_direction` / `pitch_direction` | double | `1.0` | 指令方向の符号反転用 |
| `max_yaw_rate` / `max_pitch_rate` | double | `0.5` | 角度指令の最大変化率 [rad/step] |

### ターゲット速度予測

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `target_lead_time_sec` | double | `0.0` | 偏差リード時間 [s]（0で無効） |
| `target_velocity_min_dt_sec` | double | `0.01` | 速度推定に使う最小時間差 [s] |
| `target_velocity_max_px_per_sec` | double | `1500.0` | 速度推定の上限 [px/s] |
| `target_velocity_ema_alpha` | double | `0.25` | 速度推定のEMA係数 |

### ロスト・待機姿勢

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `target_timeout_sec` | double | `0.2` | ターゲットロスト判定時間 [s] |
| `target_lost_return_to_startup_delay_sec` | double | `2.0` | 待機姿勢へ戻るまでの遅延 [s] |
| `startup_release_yaw_angle` | double | `0.0` | 待機時のヨー角 [rad] |
| `startup_release_pitch_angle` | double | `0.0` | 待機時のピッチ角 [rad] |

### 手動・テストモード

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `manual_mode_yaw_fixed_angle` | double | `0.0` | 手動モード時の固定ヨー角 [rad] |
| `manual_mode_pitch_initial_angle` | double | `0.0` | 手動モード開始時のピッチ角 [rad] |
| `enable_test_mode` | bool | `false` | 起動時のテストモード既定値 |
| `test_yaw_gain` / `test_pitch_gain` | double | `0.05` | テストモード時のゲイン |

### ゾーン別角度制限

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `enable_zone_angle_limit` | bool | `true` | ゾーン別角度制限を有効化 |
| `zone.yaw_reversed` | bool | `false` | ゾーン判定のヨー方向を反転 |
| `zone.yaw_zone1_start` | double | `-π` | ゾーン1の開始ヨー角 [rad] |
| `zone.yaw_boundary` | double | `-π/2` | ゾーン1と2の境界 [rad] |
| `zone.yaw_zone2_end` | double | `π/2` | ゾーン2の終了ヨー角 [rad] |
| `zone.yaw_zone3_end` | double | `π` | ゾーン3の終了ヨー角 [rad] |
| `zone.pitch_zone1_upper` | double | `π` | ゾーン1のピッチ上限 [rad] |
| `zone.pitch_zone2_lower` / `zone.pitch_zone2_upper` | double | `-π` / `π/6` | ゾーン2のピッチ範囲 [rad] |
| `zone.pitch_zone2_upper_limit` | double | `π` | ゾーン2のピッチ上限（拡張） [rad] |
| `zone.pitch_zone3_lower` / `zone.pitch_zone3_upper` | double | `-π/6` / `π` | ゾーン3のピッチ範囲 [rad] |
| `zone.pitch_lower_limit` | double | `-π` | 全ゾーン共通のピッチ下限 [rad] |
| `control.hysteresis_rad` | double | `0.0175` | ゾーン境界のヒステリシス [rad]（約1度） |
| `control.pitch_correct_tolerance` | double | `0.01` | ピッチ補正の許容誤差 [rad] |

## Assumptions / Known limits

- 追尾は画像座標ベースで、ターゲットまでの距離は考慮しません。弾道の落下補正は `pitch_offset` による静的な調整に留まります。
- `image_width` / `image_height` および `horizontal_fov_deg` が実際のカメラ設定と一致していることが前提です。ずれていると角度換算に系統誤差が生じます。
- ゾーン別角度制限のパラメータはフィールドとロボットの機構に強く依存します。機体やレギュレーションが変わった場合は必ず再設定してください。
- 制御は画像誤差に対する比例制御で、モータ側の追従遅れは補償しません。高速に動くターゲットには追いつかない場合があります。
- ターゲットロスト時は待機姿勢に戻るのみで、探索スキャン動作は行いません。
