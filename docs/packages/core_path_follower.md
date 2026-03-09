# core_path_follower

経路追従コントローラパッケージです。`nav_msgs/Path` を追従して `geometry_msgs/Twist` を出力します。

## 概要

```
core_path_planner        core_path_follower        core_body_controller
 /planned_path ────────▶ PathFollowerNode ────────▶ BodyControlNode
 (chassis_link)           └── Controller             cmd_vel → CAN
               /odom ────────┘   │
                                 ▼
                          /goal_reached
```

!!! note "MPPIとの使い分け"
    navigation.launch.py ではMPPIコントローラが使用されます。path_followerは単体テストやMPPIを使わない構成で利用します。

## 座標フレームモード

| モード | `use_local_frame` | フレーム | 説明 |
|-------|-------------------|---------|------|
| ローカル（デフォルト） | `true` | `chassis_link` | ロボットが原点、odom不要 |
| ワールド | `false` | `map` / `odom` | nav2スタイルのグローバルプランナ用 |

## コントローラタイプ

| タイプ | アルゴリズム |
|-------|------------|
| `cascade`（デフォルト） | 外側PID: 方位誤差→ω_ref、内側PID: ω_ref - ω_cur→ω_cmd |
| `pid` | 単一PID: 方位誤差→ω_cmd |
| `pure_pursuit` | ω = k * 2v*sin(α)/L、内側PIDで追従 |

## 補間

A*経路のスムージング:

| 方式 | 説明 |
|------|------|
| `none` | そのまま |
| `spline` | Catmull-Romスプライン |
| `bezier` | グローバルベジエ |

## 入力

| トピック | 型 | QoS | 説明 |
|---------|------|-----|------|
| `/planned_path` | `nav_msgs/Path` | reliable(10) | 追従経路 |
| `/odom` | `nav_msgs/Odometry` | reliable(50) | 姿勢・角速度 |

## 出力

| トピック | 型 | QoS | 説明 |
|---------|------|-----|------|
| `/cmd_vel` | `geometry_msgs/Twist` | reliable(10) | linear.x, linear.y, angular.z |
| `/goal_reached` | `std_msgs/Bool` | reliable(10) | ゴール到達時にtrue |

## パラメータ

設定ファイル: `param/default_params.yaml`

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `use_local_frame` | bool | `true` | ローカルフレームモード |
| `controller_type` | string | `cascade` | `cascade` / `pid` / `pure_pursuit` |
| `linear_speed` | double | `0.6` | 前進速度 [m/s] |
| `lookahead_dist` | double | `0.5` | ルックアヘッド距離 [m] |
| `goal_tolerance` | double | `0.15` | ゴール判定距離 [m] |
| `control_rate` | double | `20.0` | 制御ループ周波数 [Hz] |
| `reset_on_new_path` | bool | `true` | 新経路受信時にPIDリセット |
| `outer_kp` / `ki` / `kd` | double | `1.2` / `0.0` / `0.15` | 外側PIDゲイン |
| `inner_kp` / `ki` / `kd` | double | `2.0` / `0.0` / `0.05` | 内側PIDゲイン |
| `pure_k` | double | `1.0` | Pure Pursuitゲイン |
| `interpolation` | string | `spline` | `none` / `spline` / `bezier` |
| `spline_samples_per_segment` | int | `10` | スプライン密度 |
| `bezier_samples` | int | `100` | ベジエ密度 |

## テスト

テスト用パスパブリッシャが付属しています:

```bash
# デフォルト（直進）
ros2 launch core_path_follower test_path_follower.launch.py

# 形状を変更
ros2 launch core_path_follower test_path_follower.launch.py path_type:=figure8

# body_controllerも含めたフルパイプライン
ros2 launch core_path_follower test_path_follower.launch.py with_body_controller:=true
```

### プリセット経路

| 名前 | 形状 | テスト対象 |
|------|------|-----------|
| `straight` | 直線 | 基本追従 |
| `square` | 四角形 | 90度旋回 |
| `slalom` | 正弦波 | 連続ステアリング |
| `circle` | 円 | 定常旋回 |
| `figure8` | 8の字 | 左右切替 |
| `diamond` | 菱形（斜め） | メカナム斜め走行 |
| `lateral` | Y方向直線 | メカナム横移動 |
