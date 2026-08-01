# costmap_publisher_node

## Purpose

`path_planner_node` の動作確認には地図とコストマップが必要ですが、実機のLiDARやフィールド地図を毎回用意するのは手間がかかります。このノードは矩形障害物を配置した合成のOccupancyGridを発行し、プランナ単体でのテストを可能にします。

## Inner-workings / Algorithms

タイマー周期で、パラメータで指定されたサイズ・解像度・原点を持つ2枚のOccupancyGridを生成して発行します。

- **グローバルマップ**: `global_*` パラメータに従ったグリッド。フィールド地図の代替
- **ローカルコストマップ**: `local_*` パラメータに従ったグリッド。動的障害物の代替

グリッド内には矩形の障害物領域が塗り込まれます。実データではないため、探索アルゴリズムやパラメータの挙動を決定的に確認できるのが利点です。

## Inputs / Outputs

### Input

購読するトピックはありません。パラメータのみで動作します。

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `global_map_topic`（デフォルト `/map`） | `nav_msgs/OccupancyGrid` | 合成グローバルマップ |
| `local_costmap_topic`（デフォルト `/local_costmap`） | `nav_msgs/OccupancyGrid` | 合成ローカルコストマップ |

## Parameters

### トピック・フレーム

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `global_map_topic` | string | `/map` | グローバルマップの発行先 |
| `local_costmap_topic` | string | `/local_costmap` | ローカルコストマップの発行先 |
| `global_frame_id` | string | `map` | グローバルマップのフレーム名 |
| `local_frame_id` | string | `map` | ローカルコストマップのフレーム名 |

### グローバルマップ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `global_width` | int | `100` | 幅 [cell] |
| `global_height` | int | `100` | 高さ [cell] |
| `global_resolution` | double | `0.05` | 解像度 [m/cell] |
| `global_origin_x` | double | `-2.5` | 原点X [m] |
| `global_origin_y` | double | `-2.5` | 原点Y [m] |

### ローカルコストマップ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `local_width` | int | `40` | 幅 [cell] |
| `local_height` | int | `40` | 高さ [cell] |
| `local_resolution` | double | `0.05` | 解像度 [m/cell] |
| `local_origin_x` | double | `-1.0` | 原点X [m] |
| `local_origin_y` | double | `-1.0` | 原点Y [m] |

## Assumptions / Known limits

- テスト専用のノードです。実機やシミュレータのランチ構成には含めないでください。[core_launch](../core_launch/map_server_node.md) の `map_server_node` や [core_costmap_builder](../core_costmap_builder/index.md) と同じトピックに発行するため、同時起動するとマップが競合します。
- 障害物の配置はソースにハードコードされており、パラメータでは変更できません。
- コストマップはロボットの位置に追従しません。原点は固定です。
