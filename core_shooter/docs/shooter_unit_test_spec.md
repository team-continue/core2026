# core_shooter 単体テスト仕様書

- 作成日: 2026-03-14
- 対象パッケージ: `core_shooter`
- 対象実装:
  - `core_shooter/src/shooter_cmd_gate.cpp`
  - `core_shooter/src/shooter_controller.cpp`
  - `core_shooter/src/magazine_manager.cpp`
  - `core_shooter/src/aim_bot.cpp`
- 関連資料:
  - `core_shooter/docs/shooter_design.md`
  - `core_shooter/config/shooter.params.yaml`
  - `core_test/cmake/CoreTest.cmake`

## 1. 目的

`core_shooter` の制御ロジックについて、改修時に安全条件・状態遷移・左右振り分け・弾数管理・角度制限が壊れていないことを確認するための単体テスト仕様を定義する。

本仕様書は現行実装ベースであり、理想仕様ではなく「2026-03-14 時点のコードが持つ振る舞い」を基準とする。

## 2. スコープ

### 2.1 単体テスト対象

| ノード | 主対象 | 推奨テスト形態 |
|---|---|---|
| `shooter_cmd_gate` | UI入力の左右振り分け、フルオート立上り/立下り判定、手動モード転送 | Node起動 + topic入出力観測 |
| `shooter_controller` | 射撃可否判定、ジャム判定、発射状態機械、発射モータ指令ゲート | ロジック分離テスト + Node起動 |
| `magazine_manager` | 残弾推定、hold/regrip 状態機械、優先順位制御 | ロジック分離テスト + Node起動 |
| `aim_bot` | モード優先順位、manual/test/auto 制御、timeout、zone角度制限 | ロジック分離テスト + Node起動 |

### 2.2 単体テスト対象外

- `core_shooter/launch/shooter.launch.py` の remap / namespace 組み合わせ
- `/can/tx` を複数ノードが同時 publish したときの順序競合
- `scripts/shooter_debug_topic_gui.py` の GUI 操作性
- 実機モータ応答、センサ遅延、CAN ドライバ依存挙動

上記は結合試験または手動試験で別途確認する。

## 3. テスト方針

### 3.1 フレームワーク

- C++テストは `ament_cmake_gtest` を使用する。
- テストターゲット登録は `core_test/cmake/CoreTest.cmake` の `core_add_gtest(...)` に合わせる。
- ROS topic 入出力確認が必要な項目は `rclcpp::executors::SingleThreadedExecutor` 上でノードを回し、テスト subscriber で観測する。

### 3.2 実装前提

現行実装は各ノードのロジックが `.cpp` 内 private メソッドに閉じているため、単体テスト実装は以下のいずれかを前提とする。

1. 判定ロジックをヘッダ付きの純粋クラスまたは free function に抽出する。
2. ノード単位テストとして publisher/subscriber と timer 駆動で観測する。
3. 時刻依存ロジックは clock 注入または内部時刻状態を直接設定できる形へ最小限整理する。

本仕様書のテストケースは上記のどの実装方式でも成立するよう、入力条件と期待結果を振る舞いベースで定義する。

### 3.3 優先度

- `P1`: 安全・発射禁止条件・弾数管理・制御モード優先順位に直結する。最初に実装する。
- `P2`: 境界値・回帰防止として重要。
- `P3`: 補助的。必要に応じて追加する。

### 3.4 命名規則

- `SCG-xxx`: `shooter_cmd_gate`
- `SCT-xxx`: `shooter_controller`
- `MGM-xxx`: `magazine_manager`
- `ABT-xxx`: `aim_bot`

## 4. テスト環境

| 項目 | 内容 |
|---|---|
| ビルド | `BUILD_TESTING=ON` |
| 実行単位 | 1テストファイル = 1ターゲット |
| 時刻依存 | `last_*_time_` / `start_time_` / timeout 判定を制御可能にする |
| 観測対象 | topic publish 内容、内部状態、例外送出 |
| 入力生成 | パラメータ上書き、topic publish、直接関数呼出し |

## 5. テストケース

### 5.1 `shooter_cmd_gate`

#### 5.1.1 重点観点

- 左右射撃指令の振り分け
- フルオート入力のエッジ検出
- `manual_mode` / `manual_pitch` の片側転送
- `shoot_motor_state` の左右同時出力

#### 5.1.2 テストケース一覧

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `SCG-001` | P1 | `burst_count` 妥当性 | `burst_count=0` または負数で生成 | `std::runtime_error` を送出する |
| `SCG-002` | P1 | `shoot_motor_on_command` 妥当性 | `shoot_motor_on_command<0` で生成 | `std::runtime_error` を送出する |
| `SCG-003` | P1 | `manual_mode_target_side` 妥当性 | `left/right` 以外を設定して生成 | `std::runtime_error` を送出する |
| `SCG-004` | P1 | 単発指令 | `left_shoot_once=true` を publish | `left_shoot_cmd=1` を1回 publish し、`false` 入力では publish しない |
| `SCG-005` | P1 | バースト指令 | `right_shoot_burst=true`、`burst_count=3` | `right_shoot_cmd=3` を1回 publish する |
| `SCG-006` | P1 | フルオート立上り | `left_shoot_fullauto: false -> true -> true` | 最初の `true` でのみ `left_shoot_cmd=-1` を publish し、維持中は再 publish しない |
| `SCG-007` | P1 | フルオート立下り | `right_shoot_fullauto: false -> true -> false` | 立上りで `-1`、立下りで `0` を publish する |
| `SCG-008` | P1 | `manual_mode` 左右転送 | `manual_mode_target_side=right` で `manual_mode=true/false` | `/right/manual_mode` のみ入力値を反映し、`/left/manual_mode` は常に `false` を publish する。`left` 設定時は左右が反転する |
| `SCG-009` | P2 | `manual_pitch` 転送 | `manual_mode_target_side=left` で `manual_pitch=0.25` | `/left/manual_pitch_angle=0.25` のみ publish する |
| `SCG-010` | P1 | 発射モータ一括 ON/OFF | `shoot_motor_state=true/false`、`shoot_motor_on_command=2000.0` | `true` で左右とも `2000.0`、`false` で左右とも `0.0` を publish する |

### 5.2 `shooter_controller`

#### 5.2.1 重点観点

- 発射許可条件 `canShoot()`
- 射撃状態機械 `INIT -> SHOOT -> CMD_WAIT -> EMERGENCY`
- ジャム・hazard・regrip・発射モータ起動遅延の相互作用
- panel synchronizer の禁止角判定

#### 5.2.2 パラメータ検証

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `SCT-001` | P1 | モータID妥当性 | `shoot_motor_id<0` または `loading_motor_id<0` | `std::runtime_error` を送出する |
| `SCT-002` | P1 | 射撃パラメータ妥当性 | `burst_count<=0`、各 interval<0、`shoot_motor_rotation_cmd_activation_delay_sec<0` のいずれか | `std::runtime_error` を送出する |
| `SCT-003` | P1 | ベクタ長妥当性 | `limit_rad.size()!=4`、`loading_motor_initial_angle.size()!=3`、`target_speed.size()!=3` | それぞれ `std::runtime_error` を送出する |
| `SCT-004` | P1 | ジャム検知パラメータ妥当性 | `jam_detect_time_sec<0` | `std::runtime_error` を送出する |

#### 5.2.3 コールバック/判定ロジック

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `SCT-005` | P1 | `joint_states` サイズ不足 | `position.size() <= 4` または `loading_motor_id` が範囲外 | `loading_motor_rad_` と `turret_angle_from_chassis_` を更新しない |
| `SCT-006` | P1 | ジャム検知しきい値 | `jam=true` 継続時間が `jam_detect_time_sec` 未満/超過、その後 `false` | 未満では `is_jam_detected_=false`、超過で `true`、`false` 受信で追跡状態をリセットする |
| `SCT-007` | P1 | hazard 解除エッジ | `hazard_state_: true -> false` | `shoot_completed_=true` へ更新される |
| `SCT-008` | P1 | test mode 優先順位 | `enable_test_mode=false`、topic で `true`/`false` を投入 | 有効判定は topic 値を優先し、topic 未受信時のみパラメータ値を使う |
| `SCT-009` | P1 | 発射モータ段階指令 | `state!=EMERGENCY`、`target_speed=[2000,1750,1500]`、入力 `0.8/0.5/0.2/0.0` | それぞれ `2000/1750/1500/0` を publish する。`EMERGENCY` では常に `0` |
| `SCT-010` | P1 | 発射モータ起動遅延 | 非ゼロ速度指令を受信し、起動遅延前後を評価、その後 0 指令 | 遅延前は `shoot_motor_rotation_cmd_active_=false`、遅延到達後に `true`、0 指令で requested/active の両方をリセットする |
| `SCT-011` | P2 | `getShootMotorRotationCount()` | `loading_motor_rad_` を `0`, `0.49π`, `0.51π`, `1.49π`, `1.51π` に設定 | `π/2` 境界をまたいだときに回転数が切り上がる現行実装どおりの値を返す |
| `SCT-012` | P1 | `isValidAngle()` 通常区間 | `enable_panel_synchronizer=true`、`limit_rad=[a,b,c,d]`、`turret_angle` が `[a,b]` 内 | `false` を返す |
| `SCT-013` | P1 | `isValidAngle()` wrap 区間 | `start>end` になる禁止区間を設定 | `rad>=start` または `rad<=end` で `false` を返す |
| `SCT-014` | P1 | `isShootIntervalElapsed()` | `shoot_repeat_count_` を `1`, `2`, `-1`, `0` で評価 | 単発/バースト/フルオートごとに対応 interval を使い、`0` は `false` を返す |
| `SCT-015` | P1 | `canShoot()` 総合条件 | hazard, regrip, angle NG, interval未到達, motor inactive, jam を個別に不成立にする | いずれか1条件でも不成立なら `false`。test mode 有効時は jam / motor inactive / regrip をバイパスするが hazard / angle / interval は残る |
| `SCT-016` | P1 | `shootDecision()` の repeat 減算 | `shoot_repeat_count_=3` で `canShoot()==false` | `false` を返し、`shoot_repeat_count_` を `2` に減算する。負値の fullauto 指令は減算しない |

#### 5.2.4 状態機械

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `SCT-017` | P1 | `INIT` 遷移 | `state=INIT`、`loading_motor_rad_` を任意値に設定して timer 実行 | `setAngle(loading_motor_id_, rotation_count*pi)` 相当の指令を出し、`shoot_completed_=false`、`state=SHOOT` になる |
| `SCT-018` | P1 | `CMD_WAIT` で単発/バースト開始 | `state=CMD_WAIT`、`shoot_repeat_count_>0`、`shootDecision()==true` | `shoot_cnt` を加算し、目標角を `+π` 進め、`shoot_status=false` を publish して `state=SHOOT` へ遷移する |
| `SCT-019` | P1 | `CMD_WAIT` で指令なし | `state=CMD_WAIT`、`shoot_repeat_count_=0` | 何も publish せず状態維持する |
| `SCT-020` | P1 | `SHOOT` 完了判定 | `loading_motor_rad_ >= target_angle - 0.05π` | `shoot_status=true` を publish し、`shoot_repeat_count_>=1` のとき 1 減算して `state=CMD_WAIT` へ遷移する |
| `SCT-021` | P1 | `CMD_WAIT` 中 emergency 遷移 | `state=CMD_WAIT`、`hazard_state_=true` | `state=EMERGENCY` となり、発射モータ回転コマンド状態をリセットする |
| `SCT-022` | P1 | `SHOOT` 中 emergency 遷移 | `state=SHOOT`、`hazard_state_=true` | `state=EMERGENCY`、発射モータ回転コマンド状態をリセットし、`setAngle(loading_motor_id_, loading_motor_rad_)` を publish する |
| `SCT-023` | P1 | `EMERGENCY` 復帰 | `state=EMERGENCY`、`hazard_state_=false` | `shoot_repeat_count_=0`、発射モータ速度 `0` を publish し、`state=INIT` へ遷移する |
| `SCT-024` | P1 | fullauto 継続 | `shoot_repeat_count_=-1` で `CMD_WAIT -> SHOOT -> CMD_WAIT` を1周させる | `SHOOT` 完了後も `shoot_repeat_count_` は `-1` のままで、次の interval 経過後に再度射撃可能状態へ戻る |

### 5.3 `magazine_manager`

#### 5.3.1 重点観点

- 残弾カウントとセンサ同期の整合
- hold / release / regrip の優先順位
- `remaining_disks_ <= 10` と hazard による強制 release

#### 5.3.2 パラメータ検証

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `MGM-001` | P1 | パラメータ妥当性 | `max_disks<0` または `>127`、`window_size<=0`、`disk_thickness<=0`、`disk_hold_motor_*_angle.size()!=2`、`regrip_release_ms<0` | 各条件で `std::runtime_error` を送出する |
| `MGM-002` | P2 | 初期 publish | ノード生成直後 | `remaining_disk=max_disks` と `regrip_active=false` を publish する |

#### 5.3.3 残弾推定

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `MGM-003` | P1 | `shoot_status` 初回同期 | 最初の `shoot_status=true` を受信 | 残弾を減算せず、初期同期だけ行う |
| `MGM-004` | P1 | `shoot_status` 立上り減算 | `prev=false` 後に `true` を受信 | `remaining_disks_` を 1 減算し publish する |
| `MGM-005` | P1 | `reloading` | `reloading=true` | `remaining_disks_=max_disks_` に戻し publish する |
| `MGM-006` | P1 | `reloading_increment` | `+n`, `0`, `-n` を順に投入 | 正値だけ加算して clamp し publish、`0` と負値は無視する |
| `MGM-007` | P1 | `clampRemainingDisks()` | `-1`, `max_disks_+1` | `0` と `max_disks_` に clamp する |
| `MGM-008` | P1 | センサ移動平均更新条件 | `state != REGRIP_RELEASING` で距離センサ入力 | `buffer_` と `distance_` を更新しない |
| `MGM-009` | P1 | センサ移動平均 | `state=REGRIP_RELEASING` で `window_size` 件以上の入力 | `buffer_` を窓長で保ち、平均値を `distance_` に反映する |
| `MGM-010` | P1 | センサ同期計算 | `remainingDiskEstimator(0)`、`sensor_height-distance > 0` | `round((sensor_height-distance)/disk_thickness)` で残弾を更新し clamp、`last_sensor_*` も更新する |
| `MGM-011` | P2 | センサ同期の異常値 | `sensor_height-distance <= 0` | 残弾を更新しない |

#### 5.3.4 hold / regrip 状態機械

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `MGM-012` | P1 | hold 要求入力 | `disk_hold_state=true` | `hold_request_on_=true`、`hold_on_=false`、`state=IDLE_RELEASED` に寄せる |
| `MGM-013` | P1 | hazard 強制 release | `hazard_status: false -> true` | 即時に `hold_on_=false`、`state=IDLE_RELEASED`、`hold_shots_since_grip_=0`、release 指令 publish |
| `MGM-014` | P1 | hazard 解除 | `hazard_status: true -> false` | `on_timer()` を即時反映し、通常ロジックへ復帰する |
| `MGM-015` | P1 | `on_timer()` 優先順位 | `hazard_active_`, `remaining<=10`, `hold_request_on_`, 正常時を個別に設定 | 優先順位が `hazard > remaining<=10 > hold_request > normal` になる |
| `MGM-016` | P1 | 通常 hold 開始 | `hazard=false`, `remaining>10`, `hold_request=false`, `state=IDLE_RELEASED` | `state=HOLDING` へ遷移し、close 角度を publish する |
| `MGM-017` | P1 | regrip 開始条件 | `state=HOLDING`, `hold_on_=true`, `remaining>10`, `hazard=false`, 立上り射撃10回 | `state=REGRIP_RELEASING`、`hold_shots_since_grip_=0`、`buffer_.clear()`、release 指令、`regrip_active=true` を publish する |
| `MGM-018` | P1 | regrip 期間中の同期 | `state=REGRIP_RELEASING`、`buffer_.size()>=window_size_` | `remainingDiskEstimator(0)` を実行する |
| `MGM-019` | P1 | regrip 完了 | `state=REGRIP_RELEASING`、現在時刻が `regrip_release_until_` 以上 | `state=HOLDING` に戻り、以後 close 指令へ復帰する |
| `MGM-020` | P2 | hold 指令角 | `publish_hold_command(true/false)` | `true` で index 0 の角度、`false` で index 1 の角度を左右モータへ publish する |

### 5.4 `aim_bot`

#### 5.4.1 重点観点

- 制御モード優先順位 `Emergency > Manual > Test > AutoTrack`
- timeout と初期化動作
- gain / FOV 2方式の画像追尾
- zone ベース角度制限と pitch 補正

#### 5.4.2 パラメータ検証

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `ABT-001` | P1 | 基本パラメータ妥当性 | `rate<=0`、モータID負値、`yaw_min>yaw_max`、`pitch_min>pitch_max` | `std::runtime_error` を送出する |
| `ABT-002` | P1 | 画像追尾パラメータ妥当性 | `image_width<=0`、`image_height<=0`、`test_*_gain<0`、`image_tolerance<0`、`target_timeout_sec<0` | `std::runtime_error` を送出する |
| `ABT-003` | P1 | FOV 妥当性 | `use_fov_image_tracking=true` かつ `horizontal_fov_deg<=0` または `>=180` | `std::runtime_error` を送出する |
| `ABT-004` | P1 | zone 設定妥当性 | yaw 区間順序不正、pitch 上下限が同値、`hysteresis_rad<0`、hysteresis が zone 幅以上、pitch cap と重ならない設定 | `std::runtime_error` を送出する |

#### 5.4.3 入力コールバックとモード選択

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `ABT-005` | P1 | ターゲット検出入力 | `PointStamped(x,y,z)` で `z<0.5` / `z>=0.5` | 検出時は `has_target_=true` と座標更新、未検出時は `has_target_=false` と座標リセット |
| `ABT-006` | P1 | test mode 上書き | `enable_test_mode=false`、topic で `true/false` | 有効判定は topic 値を優先する |
| `ABT-007` | P1 | manual mode エッジ | `manual_mode: false -> true -> false` | 立上りで `manual_mode_init_pending_=true`、OFF で pending をクリアする |
| `ABT-008` | P1 | `joint_states` サイズ不足 | `position.size()` が `yaw_motor_id_` または `pitch_motor_id_` を満たさない | `yaw_angle_` / `pitch_angle_` / `has_joint_state_` を更新しない |
| `ABT-009` | P1 | モード優先順位 | hazard, manual_mode, test_mode, target 有効の組合せを与える | 優先順位は常に `Emergency > Manual > Test > AutoTrack` となる |

#### 5.4.4 モード別制御

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `ABT-010` | P1 | 初回 hazard 解除初期化 | 再起動直後 `hazard: true -> false`、`startup_release_*` 設定済み | 1回だけ `command_yaw/pitch` を startup_release 角に設定し、`startup_release_hold_active_=true` にする |
| `ABT-011` | P2 | hazard 解除の一回性 | `startup_release_init_pending_` 消化後に再度 `hazard: true -> false` | startup_release 初期化を再実行しない |
| `ABT-012` | P1 | Manual 初期化 | Manual へ遷移、`manual_mode_yaw_fixed_angle` / `manual_mode_pitch_initial_angle` 設定済み | 遷移時に yaw 固定角・初期 pitch を設定し、既存 manual pitch 入力を破棄する |
| `ABT-013` | P1 | Manual pitch 積分 | Manual 中に `manual_pitch_angle=+1.5/-1.5/0.5` を投入 | 入力を `[-1,1]` に clamp し、`pitch_direction * test_pitch_gain` で内部 command pitch に加算する |
| `ABT-014` | P1 | Manual pitch timeout | Manual 中、最後の manual pitch 入力から `target_timeout_sec` 超過 | `has_manual_pitch_target_=false` に戻し、`joint_states` があれば実 pitch を保持対象にする |
| `ABT-015` | P1 | Test 入力初期化 | Test へ遷移 | 以前の `test_yaw/pitch` を無効化し、新しい入力を要求する |
| `ABT-016` | P1 | Test 入力積分 | Test 中に fresh な `test_yaw` / `test_pitch` を投入 | 入力を `[-1,1]` に clamp し、`yaw_direction/test_yaw_gain` と `pitch_direction/test_pitch_gain` で command を積分する |
| `ABT-017` | P1 | Test timeout | Test 中、最後の test 入力から `target_timeout_sec` 超過 | 該当軸の fresh フラグを下ろし、両軸とも未入力なら command を保持するだけで新規移動しない |
| `ABT-018` | P1 | Auto timeout | Auto 中に target 未受信または timeout | 最初の1回だけ現在 joint 角をラッチして publish、その後は直前 command を保持する |
| `ABT-019` | P1 | Auto 遷移時ベースライン | 他モードから Auto へ遷移し `has_joint_state_=true` | baseline を `yaw_angle_` と `pitch_angle_ + pitch_offset_` に設定する |
| `ABT-020` | P1 | Auto gain 方式 | `use_fov_image_tracking=false`、誤差が tolerance を超える | `yaw_image_gain` / `pitch_image_gain` に比例した command 変化を出す。tolerance 以内の軸は変化しない |
| `ABT-021` | P1 | Auto FOV 方式 | `use_fov_image_tracking=true`、画像誤差を与える | `atan(x_norm*tan(hfov/2))` と画像比率由来の縦画角で角度変換する |

#### 5.4.5 角度制限

| ID | 優先度 | 観点 | 入力・事前条件 | 期待結果 |
|---|---|---|---|---|
| `ABT-022` | P1 | 基本 clamp | `setCommandTarget()` に global cap を超える yaw/pitch を与える | `yaw_min/max`、`pitch_min/max` に clamp される |
| `ABT-023` | P1 | `setManualModeCommandTarget()` | zone 制限有効時に manual yaw/pitch を設定 | yaw は zone frame で clamp、pitch は global cap のみ適用し、pitch 補正状態を解除する |
| `ABT-024` | P1 | yaw zone ヒステリシス | `classifyYawZone()` に boundary 付近の値を連続投入 | `last_yaw_zone_` を使って hysteresis 幅内のチャタリングを抑止する |
| `ABT-025` | P1 | zone 跨ぎ時の pitch 補正 | 現在 zone と目標 zone の pitch 許容範囲が異なる状態で遷移 | まず yaw を保持しつつ `correction_target_pitch_` へ補正し、許容誤差到達後に yaw 遷移を再開する |
| `ABT-026` | P2 | `zone_yaw_reversed` | `zone_yaw_reversed=true` で zone clamp/classify を評価 | zone 判定フレームのみ yaw 符号反転して扱い、最終出力は元フレームへ戻す |
| `ABT-027` | P2 | command 未初期化時 publish | `has_command_target_=false` で `publishCommandTarget()` 相当を実行 | CAN publish を行わず、保持動作に留める |

## 6. 実装順序

1. `P1` のパラメータ妥当性テストを追加する。
2. `shooter_cmd_gate` の callback routing を追加する。
3. `shooter_controller` の `canShoot()` / interval / state machine を追加する。
4. `magazine_manager` の priority / regrip / remaining disk estimator を追加する。
5. `aim_bot` の mode priority / timeout / zone 制限を追加する。
6. `P2` を回帰防止目的で段階的に埋める。

## 7. 実装時の注意

- `shooter_controller`, `magazine_manager`, `aim_bot` は time 依存が強い。sleep ベースではなく、clock 注入または内部時刻の直接設定で determinism を確保する。
- publish 内容の確認は topic subscriber だけでなく、必要に応じて CAN メッセージの `id` と `data[0]` まで検証する。
- `aim_bot` の zone 補正は条件分岐が多いため、純粋関数化してから単体テストする方が保守しやすい。
- `manual` / `test` / `auto` の優先順位は仕様固定とし、ログ文言ではなく出力 command と内部状態で検証する。

## 8. 未定義・別試験扱い事項

- `core_shooter/config/shooter.params.yaml` にある `max_yaw_rate` / `max_pitch_rate` は、現行 `aim_bot.cpp` では宣言・使用されていないため本仕様の対象外とする。
- `shooter_controller` の `loading_motor_error_state` / `shoot_motor_error_state` publisher は現行コードで publish されていないため、本仕様ではテスト対象外とする。
- `magazine_manager` の `hold_disable_height_margin_mm` は現行コードでロジック未使用のため、本仕様ではテスト対象外とする。
