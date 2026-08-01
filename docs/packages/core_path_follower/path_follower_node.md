# path_follower_node

## Purpose

`nav_msgs/Path` を追従して `geometry_msgs/Twist` を出力する経路追従コントローラです。MPPIより軽量で挙動が決定的なため、単体テストやMPPIを使わない構成での経路追従に用います。

!!! note "MPPIとの使い分け"
    `navigation.launch.py` ではMPPIコントローラが使用されます。path_followerは単体テストやMPPIを使わない構成で利用します。

## Inner-workings / Algorithms

`control_rate` の周期で、経路上のルックアヘッド点に対する方位誤差を求め、選択したコントローラで角速度指令に変換します。並進速度は `linear_speed` 固定です。

### 座標フレームモード

| モード | `use_local_frame` | フレーム | 説明 |
|-------|-------------------|---------|------|
| ローカル（デフォルト） | `true` | `chassis_link` | ロボットが原点、odom不要 |
| ワールド | `false` | `map` / `odom` | nav2スタイルのグローバルプランナ用 |

### コントローラタイプ

| タイプ | アルゴリズム |
|-------|------------|
| `cascade`（デフォルト） | 外側PID: 方位誤差→ω_ref、内側PID: ω_ref - ω_cur→ω_cmd |
| `pid` | 単一PID: 方位誤差→ω_cmd |
| `pure_pursuit` | ω = k * 2v*sin(α)/L、内側PIDで追従 |

### 補間

A*経路は折れ線のため、そのまま追従すると角の部分で指令が不連続になります。追従前にスムージングを適用します。

| 方式 | 説明 |
|------|------|
| `none` | そのまま |
| `spline` | Catmull-Romスプライン |
| `bezier` | グローバルベジエ |

```
core_path_planner        core_path_follower        core_body_controller
 /planned_path ────────▶ PathFollowerNode ────────▶ BodyControlNode
 (chassis_link)           └── Controller             cmd_vel → CAN
               /odom ────────┘   │
                                 ▼
                          /goal_reached
```

## Inputs / Outputs

### Input

| トピック | 型 | QoS | 説明 |
|---------|------|-----|------|
| `/planned_path` | `nav_msgs/Path` | reliable(10) | 追従経路 |
| `/odom` | `nav_msgs/Odometry` | reliable(50) | 姿勢・角速度 |

### Output

| トピック | 型 | QoS | 説明 |
|---------|------|-----|------|
| `/cmd_vel` | `geometry_msgs/Twist` | reliable(10) | linear.x, linear.y, angular.z |
| `/goal_reached` | `std_msgs/Bool` | reliable(10) | ゴール到達時にtrue |

## Parameters

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

## Assumptions / Known limits

- 障害物回避は行いません。与えられた経路をそのまま追従するため、動的障害物への対応は上流のプランナ側の責務です。
- 並進速度は `linear_speed` 固定で、曲率に応じた減速は行いません。急カーブでは `lookahead_dist` と併せて調整が必要です。
- `use_local_frame:=false` の場合のみ `/odom` のワールド姿勢に依存します。ローカルモードでは経路が `chassis_link` 基準で届くことが前提です。
- ゴール判定は距離のみで、最終姿勢（ヨー角）の収束は保証しません。

## テスト

テスト用パスパブリッシャが付属しています。

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
