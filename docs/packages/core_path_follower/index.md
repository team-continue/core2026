# core_path_follower

経路追従コントローラパッケージです。`nav_msgs/Path` を追従して速度指令を出力します。MPPIより軽量で挙動が決定的なため、単体テストやMPPIを使わない構成で使用します。

## 位置づけ

[core_mppi](../core_mppi/index.md) と同じ「経路をどう走るか」の層に位置する代替コントローラです。障害物回避を持たない代わりに、動作が決定的で挙動を追いやすいという特性があります。

```mermaid
graph LR
    PP["core_path_planner"] -->|"/planned_path"| PF["path_follower_node"]
    OB["odom_bridge_node"] -->|"/odom"| PF
    PF -->|"/cmd_vel"| BC["core_body_controller"]
    PF -->|"/goal_reached"| BS["core_behavior_system"]
```

!!! note "MPPIとの使い分け"
    `navigation.launch.py` ではMPPIコントローラが使用されます。path_followerは単体テストやMPPIを使わない構成で利用します。

| コントローラ | 障害物回避 | 挙動 | 使いどころ |
|------------|----------|------|-----------|
| [core_mppi](../core_mppi/index.md) | あり | 確率的 | 本番構成 |
| **core_path_follower** | なし | 決定的 | テスト・デバッグ |

## ノード一覧

| ノード | 実行ファイル | 役割 |
|-------|------------|------|
| [`path_follower_node`](path_follower_node.md) | `core_path_follower_node` | 経路追従による速度指令の生成 |

## 処理の流れ

```mermaid
flowchart LR
    A["/planned_path 受信"] --> B["補間<br/>spline / bezier"]
    B --> C["ルックアヘッド点を探索<br/>lookahead_dist"]
    C --> D["方位誤差を算出"]
    D --> E["コントローラ<br/>cascade / pid / pure_pursuit"]
    E --> F["/cmd_vel 発行"]
    C --> G{"ゴール距離 <<br/>goal_tolerance?"}
    G -->|"はい"| H["/goal_reached = true"]
```

A*が出力する折れ線経路は角で方向が不連続に変わるため、追従前にスプラインまたはベジエで補間して滑らかにします。

### コントローラの選択肢

| タイプ | アルゴリズム | 特徴 |
|-------|------------|------|
| `cascade`（デフォルト） | 外側PID（方位誤差→ω_ref）＋内側PID（ω誤差→ω_cmd） | 負荷変動に強い |
| `pid` | 単一PID（方位誤差→ω_cmd） | 単純・調整が容易 |
| `pure_pursuit` | 幾何的な追従則＋内側PID | 曲線追従が滑らか |

### 座標フレームモード

| モード | `use_local_frame` | フレーム | 説明 |
|-------|-------------------|---------|------|
| ローカル（デフォルト） | `true` | `chassis_link` | ロボットが原点、odom不要 |
| ワールド | `false` | `map` / `odom` | nav2スタイルのグローバルプランナ用 |

## テスト構成

テスト用のパスパブリッシャが付属し、7種類のプリセット経路で追従性能を確認できます。

```mermaid
graph LR
    TP["test_path_publisher<br/>straight/square/slalom/<br/>circle/figure8/diamond/lateral"] -->|"/planned_path"| PF["path_follower_node"]
    PF -->|"/cmd_vel"| BC["body_control_node<br/>（オプション）"]
```

```bash
ros2 launch core_path_follower test_path_follower.launch.py path_type:=figure8
```

詳細は[ノードページのテスト節](path_follower_node.md#テスト)を参照してください。

## 起動

```bash
ros2 launch core_path_follower path_follower.launch.py
```

設定ファイル: `param/default_params.yaml`
