# magazine_manager

## Purpose

発射通知とリロード入力から残弾数を追跡し、残弾表示と保持機構（ディスクホールド）の自律制御を行います。また、装填不良を防ぐためのリグリップ動作もこのノードが管理します。左右タレットに1つずつ起動されます。

## Inner-workings / Algorithms

### 残弾推定

`shoot_status` を受信するたびにカウントを1減らし、`reloading`（満タンにリセット）/ `reloading_increment`（指定枚数を加算）でリロード時のカウントを補正します。推定値は `max_disks` を上限にクランプされます。

!!! note "距離センサによる高さ推定について"
    マガジン上部の距離センサ値からディスク山の高さを求め、`残弾数 = (sensor_height - 測定距離) / disk_thickness` の式で枚数を再推定するロジックも実装されています（リグリップ動作で保持を解放している間だけ、`window_size` 分の移動平均を取って同期）。ただし現状 `core_hardware` から実センサ値は発行されておらず、[shooter_debug_topic_gui](shooter_debug_topic_gui.md) からの手動発行以外に入力元がないため、実機では今回使用していません。残弾数は発射カウントとリロード入力のみで管理されています。

### リグリップ

`regrip_enabled` が有効な場合、`regrip_trigger_shots` 発ごとにディスク保持機構を一時的に解放し（`regrip_release_ms`）、上部のディスクを支えます。連続射撃でディスクが傾いて装填不良になるのを防ぐための動作です。

リグリップ中は `regrip_active` を発行し、[shooter_controller](shooter_controller.md) 側で装填を停止させます。

### ディスクホールド

残弾が11枚以上ある間は自動的に保持機構をON（HOLDING状態）にし、`remaining_disks_` が10枚以下になると自動でOFFにします。緊急停止中とリグリップ解放中も一時的にOFFになります。

!!! note "手動オーバーライドと高さマージンについて"
    操作者が `disk_hold_state` でホールドを強制解放する入力も実装されていますが、実機でこのトピックを発行する構成が今回はなく未使用です。また `hold_disable_height_margin_mm` によるセンサ高さベースの無効化判定も、距離センサ値が入力されないため実質的に発動しません。実機では残弾数によるしきい値（11枚未満で自動解放）のみが有効です。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `shoot_status` | `std_msgs/Bool` | 1発発射通知（`shooter_controller` から） |
| `reloading` | `std_msgs/Bool` | リロードトリガー |
| `reloading_increment` | `std_msgs/Int8` | リロード時の残弾加算値 |
| `hazard_status` | `std_msgs/Bool` | 緊急停止状態（`/system/emergency/hazard_status` にリマップ） |
| `disk_distance_sensor` | `std_msgs/Int32` | マガジン充填高さセンサ値（launchで `distance` にリマップ）。※今回未使用（備考参照） |
| `disk_hold_state` | `std_msgs/Bool` | ディスク保持機構の手動オーバーライド。※今回未使用（備考参照） |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `remaining_disk` | `std_msgs/Int8` | 推定残弾数 |
| `regrip_active` | `std_msgs/Bool` | リグリップ動作中フラグ（`shooter_controller` の装填を抑止） |
| `/can/tx` | `core_msgs/CANArray` | ディスクホールドモーターのCAN指令 |

## Parameters

設定ファイル: `config/shooter.params.yaml`

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `max_disks` | int | `30` | 最大ディスク容量 |
| `disk_thickness` | double | `20.0` | ディスク厚さ [mm] |
| `sensor_height` | double | `500.0` | センサ取り付け高さ [mm] |
| `window_size` | int | `3` | センサ値の移動平均window |
| `disk_hold_right_motor_id` | int | `100` | 右ホールドモーターCAN ID |
| `disk_hold_left_motor_id` | int | `101` | 左ホールドモーターCAN ID |
| `regrip_enabled` | bool | `true` | リグリップ機能有効化 |
| `regrip_release_ms` | int | `1000` | リグリップの解放保持時間 [ms] |
| `regrip_trigger_shots` | int | `6` | リグリップを発動する発射数間隔 |
| `hold_disable_height_margin_mm` | double | `0.5` | ホールド無効化する高さマージン [mm] |

## Assumptions / Known limits

- 残弾数は発射カウントとリロード入力による積算値であり、実際の枚数とのズレを検知・補正する手段が実機にはありません。カウントの前提が崩れる操作（電源再投入なしのマガジン差し替えなど）があると実数とずれ続けます。
- 距離センサによる高さ推定・センサ同期のロジックはコード上に存在しますが、`core_hardware` から実センサ値が発行されないため今回は使用していません（詳細は各節の備考を参照）。
- `disk_hold_state` による手動オーバーライドも同様に、実機で発行する構成が今回はないため未使用です。
- リグリップは発射数カウントに基づく開放ループ制御で、実際に積み直しが成功したかは検証しません。
- ホールドモーターIDのデフォルト値（`100` / `101`）はプレースホルダです。実機起動時はランチファイルまたは設定ファイルで必ず上書きしてください。
