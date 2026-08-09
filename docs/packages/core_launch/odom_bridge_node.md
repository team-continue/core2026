# odom_bridge_node

## Purpose

オドメトリの供給元はシミュレータ（`/sim_odom`）とFAST-LIO（`/Odometry`）の2系統があり、トピック名もフレーム構成も異なります。下流のプランナやコントローラがこの違いを意識せずに済むよう、このノードが両者を `/odom` に統一し、併せてTFと経路計画用の開始位置を発行します。

## Inner-workings / Algorithms

`odom_source` パラメータに応じて購読するトピックを切り替えます。

| `odom_source` | 購読トピック | 用途 |
|--------------|------------|------|
| `sim` | `/sim_odom` | Unityシミュレータ |
| `fastlio` | `/Odometry` | 実機（FAST-LIO） |

受信したオドメトリに対して以下を行います。

1. 初期姿勢オフセット（`init_x` / `init_y` / `init_yaw`）を適用し、`/odom` として再発行
2. 同じ姿勢を `/start_pose` として発行。[core_path_planner](../core_path_planner/index.md) がA*探索の開始点として使用する
3. `odom_frame` → `base_frame` のTFを動的にブロードキャスト

FAST-LIOモードでは、初回メッセージ受信時に `odom → camera_init` のTFを一度だけ配信します。FAST-LIOは独自の `camera_init` フレームを基準に点群を出力するため、この変換がないと [core_localization](../core_localization/index.md) が点群を地図座標系に持ち込めません。

## Inputs / Outputs

### Input

| トピック | 型 | 条件 |
|---------|------|------|
| `/sim_odom` | `nav_msgs/Odometry` | `odom_source:=sim` |
| `/Odometry` | `nav_msgs/Odometry` | `odom_source:=fastlio` |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `/odom` | `nav_msgs/Odometry` | 統一されたオドメトリ |
| `/start_pose` | `geometry_msgs/PoseStamped` | 経路計画の開始位置 |

### TF

| TF | 種類 | 説明 |
|----|------|------|
| `odom → base_link` | 動的 | ロボットのオドメトリ姿勢 |
| `odom → camera_init` | 静的（一度だけ） | FAST-LIOモードのみ。初回メッセージ受信時に配信 |

## Parameters

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `odom_source` | string | `sim` | `sim` または `fastlio` |
| `odom_frame` | string | `odom` | 親フレーム名 |
| `base_frame` | string | `base_link` | 子フレーム名 |
| `init_x` | double | `0.0` | 初期X座標 [m] |
| `init_y` | double | `0.0` | 初期Y座標 [m] |
| `init_yaw` | double | `0.0` | 初期ヨー角 [rad] |

## Assumptions / Known limits

- `odom_source` は起動時に固定されます。実行中にシミュレータと実機を切り替えることはできません。
- `odom → camera_init` は初回メッセージ時の一度きりの配信です。FAST-LIOを再起動した場合、このノードも再起動しないとフレーム関係がずれます。
- オドメトリの積算誤差（ドリフト）は補正しません。グローバルな位置補正は [core_localization](../core_localization/index.md) が `map → odom` で行います。
- 上流のオドメトリが途絶えても、このノードは異常を通知しません。TFが更新されなくなるだけです。
