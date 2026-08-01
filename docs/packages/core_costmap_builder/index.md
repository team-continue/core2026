# core_costmap_builder

LiDAR点群から、ロボット周辺のローリングウィンドウ式ローカルコストマップを生成するパッケージです。

## 位置づけ

事前に用意したフィールド地図には、試合中に現れる相手ロボットや動く障害物は含まれていません。このパッケージが動的障害物をリアルタイムに検出し、プランナとコントローラに伝える役割を担います。

```mermaid
graph LR
    subgraph Static["静的情報"]
        MS["core_launch<br/>map_server_node"] -->|"/map<br/>/costmap/global"| Plan
    end
    subgraph Dynamic["動的情報（本パッケージ）"]
        LiDAR["Livox LiDAR"] -->|"/livox/lidar"| CB["costmap_build_node"]
        TF["TFツリー"] -->|"TF: odom→base_link"| CB
    end
    CB -->|"/costmap/local"| Plan["core_path_planner<br/>経路計画"]
    CB -->|"/costmap/local"| MPPI["core_mppi<br/>障害物コスト評価"]
```

静的地図と動的コストマップの2層構成にすることで、フィールドの壁は事前情報として持ちつつ、移動する障害物にはセンサで反応できます。

## ノード一覧

| ノード | 実行ファイル | 役割 |
|-------|------------|------|
| [`costmap_build_node`](costmap_build_node.md) | `costmap_build_node` | 点群のフィルタリングとOccupancyGridへの変換 |

## 処理パイプライン

```mermaid
flowchart TD
    A["LiDAR点群<br/>/livox/lidar"] --> B["自己除去<br/>self_crop_min_range_m 以内を除去"]
    B --> C["フィルタリング<br/>高さ min_z_m〜max_z_m<br/>距離 max_range_m<br/>範囲 crop_xy_m"]
    C --> D["ボクセル化<br/>voxel_leaf_m"]
    D --> E["グリッド投影<br/>resolution_m のセルへ"]
    E --> F["膨張処理<br/>inflation_radius_m + decay_margin_m"]
    F --> G["/costmap/local 発行"]
```

## コストの構造

膨張処理により、障害物の周囲に2段階のゾーンを作ります。

```
     障害物セル
         │
    ┌────┴────┐
    │ LETHAL  │  コスト100 : 進入不可
    │ (半径 inflation_radius_m)
    └────┬────┘
    ┌────┴────┐
    │  decay  │  コスト1〜49 : 通れるが避けたい
    │ (幅 decay_margin_m、外側ほど低コスト)
    └─────────┘
         │
      自由空間     コスト0
```

decayゾーンは [core_path_planner](../core_path_planner/index.md) のコスト考慮型A*（`cost_weight`）や [core_mppi](../core_mppi/index.md) の障害物コストで「通れるが避けたい領域」として評価され、壁ぎりぎりではなく通路の中央寄りを走る挙動を生みます。

## ローリングウィンドウ

コストマップはロボットに追従して移動する固定サイズの窓です。フィールド全体を保持しないことで計算量とメモリを一定に保ちます。

```mermaid
graph LR
    TF["TFツリー"] -->|"TF: odom→base_link<br>（窓の中心を決定）"| CB["costmap_build_node"]
    CB -->|"/costmap/local<br>（update_hz の周期で更新）"| Cons["core_path_planner<br>core_mppi"]
```

## 起動

```bash
ros2 launch core_costmap_builder costmap_build.launch.py
```

設定ファイル: `config/costmap_build_node.yaml`

チューニングのポイントは[パラメータチューニングガイド](../../guides/tuning.md#ローカルコストマップ)を参照してください。
