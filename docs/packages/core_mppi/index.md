# core_mppi

MPPI（Model Predictive Path Integral）によるローカルコントローラパッケージです。グローバル経路を、障害物を避けながら実際に走行可能な速度指令に変換します。

## 位置づけ

ナビゲーションスタックの「経路をどう走るか」を担う層です。上流のプランナが引いた幾何的な経路を、ロボットの運動特性とリアルタイムの障害物情報を踏まえた速度指令に落とし込みます。

```mermaid
graph LR
    subgraph Plan["経路計画"]
        PP["core_path_planner<br/>A*"]
    end
    subgraph Perceive["環境認識"]
        CB["core_costmap_builder"]
        MS["core_launch<br/>map_server_node"]
        OB["core_launch<br/>odom_bridge_node"]
    end
    subgraph Control["ローカル制御（本パッケージ）"]
        MPPI["core_mppi_node"]
    end
    subgraph Actuate["車体"]
        SM["core_cmd_vel_smoother"]
        BC["core_body_controller"]
    end

    PP -->|"/planned_path"| MPPI
    CB -->|"/costmap/local"| MPPI
    MS -->|"/costmap/global"| MPPI
    OB -->|"/odom"| MPPI
    MPPI -->|"/cmd_vel_raw"| SM
    SM -->|"/cmd_vel"| BC
```

## ノード一覧

| ノード | 実行ファイル | 役割 |
|-------|------------|------|
| [`core_mppi_node`](core_mppi_node.md) | `core_mppi_node` | MPPIによる速度指令の生成 |

## アルゴリズムの流れ

制御周期ごとに、多数の候補軌道をサンプリングしてコストで重み付けし、最適な制御列を求めます。

```mermaid
flowchart LR
    A["直前の制御列"] --> B["ノイズを加えて<br/>N本サンプリング"]
    B --> C["運動モデルで<br/>ホライズン先まで予測"]
    C --> D["コスト評価<br/>経路/ゴール/障害物/平滑性"]
    D --> E["exp(-cost/温度)で<br/>重み付き平均"]
    E --> F["先頭ステップを<br/>/cmd_vel_raw に発行"]
    F -.->|"次周期"| A
```

勾配計算を必要とせず、障害物コストのような不連続な評価関数をそのまま扱えるのがMPPIの利点です。一方で確率的サンプリングのため出力にフレーム間の揺らぎが出るので、[core_cmd_vel_smoother](../core_cmd_vel_smoother/index.md) での平滑化を前提としています。

## 他のコントローラとの使い分け

| コントローラ | 特徴 | 使いどころ |
|------------|------|-----------|
| **core_mppi** | 障害物回避を内包、全方向移動に対応 | `navigation.launch.py` の既定構成 |
| [core_path_follower](../core_path_follower/index.md) | 軽量・決定的、障害物回避なし | 単体テスト、MPPIを使わない構成 |

## 起動

```bash
ros2 launch core_mppi mppi.launch.py
```

設定ファイル: `param/default_params.yaml`

コスト重みのチューニング手順は[パラメータチューニングガイド](../../guides/tuning.md#mppi-コントローラ)を参照してください。
