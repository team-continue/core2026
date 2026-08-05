# path_planner_node

## Purpose

ゴール地点が与えられても、障害物を避けてどう到達するかは自明ではありません。このノードは静的なフィールド地図と動的なローカルコストマップを重ね合わせたグリッド上でA*探索を行い、走行可能な経路を生成します。

## Inner-workings / Algorithms

### 経路探索

`/goal_pose` を受信するたびに、`/start_pose` から目標までのA*探索を実行します。

1. **グリッド統合**: `/map`（静的）と `/local_costmap`（動的障害物）を重ね合わせて探索用グリッドを構成
2. **通行判定**: セル値が `occupied_threshold` 以上を障害物として扱う。未知セル（-1）は `allow_unknown` に従う
3. **A*探索**: `use_diagonal` が有効なら8近傍、無効なら4近傍で展開
4. **座標変換**: 得られたグリッド経路をワールド座標に変換し、`nav_msgs/Path` として発行

`publish_in_global_frame` が有効な場合、経路を `global_frame_id`（通常 `odom`）のフレームで出力します。

### コスト考慮型A*

`cost_weight > 0` の場合、各ステップのコストにセルの占有値に比例したペナルティが加算されます。

```
step_cost = base_cost + cost_weight × (cell_value / 100)
```

これにより、[core_costmap_builder](../core_costmap_builder/index.md) が生成するdecayゾーン（コスト1〜49）を通る経路はペナルティを受け、壁ぎりぎりではなく通路の中央寄りを走る安全な経路が選ばれます。`cost_weight=0.0` では従来の等コストA*と同一動作です。

```
┌──────────────────────────────────────────┐
│            PathPlannerNode               │
│  ┌────────────────────────────────────┐  │
│  │           PathPlanner              │  │
│  │  - A*アルゴリズム                    │  │
│  │  - グリッド/ワールド座標変換          │  │
│  │  - ローカルコストマップ統合           │  │
│  └────────────────────────────────────┘  │
└──────────────────────────────────────────┘
      ▲            ▲              ▲    │
      │            │              │    ▼
   /map     /local_costmap   /start_pose  /planned_path
                             /goal_pose
```

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | グローバルマップ |
| `/local_costmap` | `nav_msgs/OccupancyGrid` | ローカルコストマップ（トピック名はパラメータで変更可） |
| `/start_pose` | `geometry_msgs/PoseStamped` | 開始位置（odom_bridgeから） |
| `/goal_pose` | `geometry_msgs/PoseStamped` | ゴール位置（RViz2またはbehavior systemから） |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `/planned_path` | `nav_msgs/Path` | 計画された経路 |

## Parameters

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `global_map_topic` | string | `/map` | グローバルマップトピック名 |
| `local_costmap_topic` | string | `/local_costmap` | ローカルコストマップトピック名 |
| `start_topic` | string | `/start_pose` | 開始位置トピック名 |
| `goal_topic` | string | `/goal_pose` | ゴール位置トピック名 |
| `path_topic` | string | `/planned_path` | 出力経路トピック名 |
| `occupied_threshold` | int | `50` | セル占有判定しきい値（0-100） |
| `allow_unknown` | bool | `false` | 未知セルの通過を許可 |
| `use_diagonal` | bool | `true` | 斜め移動を許可 |
| `cost_weight` | double | `0.0` | コスト考慮の重み（0.0で従来動作、2.0推奨） |
| `publish_in_global_frame` | bool | `false` | odomフレームで経路を出力 |
| `global_frame_id` | string | `odom` | グローバルフレーム名 |

!!! note "navigation.launch.pyでの設定"
    `local_costmap_topic=/costmap/local`, `publish_in_global_frame=true`, `global_frame_id=odom`, `cost_weight=2.0`

## Assumptions / Known limits

- 経路探索はグリッドベースで、ロボットの運動学的制約（旋回半径や横移動の可否）を考慮しません。生成された経路が実際に追従可能かは下流のコントローラ次第です。
- 出力される経路は折れ線であり、角の部分で方向が不連続に変化します。平滑化は [core_path_follower](../core_path_follower/index.md) の補間機能や [core_mppi](../core_mppi/index.md) 側で行います。
- ロボットのサイズは考慮しません。障害物からのクリアランス確保は、コストマップ側の膨張（`inflation_radius_m`）に依存します。
- 再計画は `/goal_pose` の受信を契機とします。走行中に障害物が現れても自動では再探索しません。
- ゴールが到達不能な位置（障害物内など）の場合、探索は失敗し経路は発行されません。
