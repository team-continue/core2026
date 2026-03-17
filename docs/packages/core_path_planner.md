# core_path_planner

A*アルゴリズムによるグローバル経路計画パッケージです。

## 概要

- OccupancyGrid上でA*経路探索を実行
- 斜め移動対応（設定可能）
- ローカルコストマップとの統合による動的障害物回避
- 結果を `nav_msgs/Path` としてパブリッシュ

## アーキテクチャ

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

## 入力

| トピック | 型 | 説明 |
|---------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | グローバルマップ |
| `/local_costmap` | `nav_msgs/OccupancyGrid` | ローカルコストマップ（トピック名はパラメータで変更可） |
| `/start_pose` | `geometry_msgs/PoseStamped` | 開始位置（odom_bridgeから） |
| `/goal_pose` | `geometry_msgs/PoseStamped` | ゴール位置（RViz2から） |

## 出力

| トピック | 型 | 説明 |
|---------|------|------|
| `/planned_path` | `nav_msgs/Path` | 計画された経路 |

## パラメータ

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

## コスト考慮型A*

`cost_weight > 0` の場合、各ステップのコストにセルの占有値に比例したペナルティが加算されます。
これにより、decay zone（コスト1-49）のセルを通るパスはペナルティを受け、通路の中央寄りの安全なパスが生成されます。

```
step_cost = base_cost + cost_weight × (cell_value / 100)
```

`cost_weight=0.0` では従来の等コストA*と同一動作です。

## 起動

```bash
ros2 run core_path_planner path_planner_node
```
