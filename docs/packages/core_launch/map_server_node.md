# map_server_node

## Purpose

フィールドのレイアウトはPNG画像として管理するのが編集しやすく、バージョン管理もしやすい形式です。このノードは画像ファイルをROSの `nav_msgs/OccupancyGrid` に変換し、経路計画用の地図とグローバルコストマップとして供給します。

## Inner-workings / Algorithms

1. **画像読み込み**: `image_path` のPNG画像をグレースケールで読み込む
2. **占有値変換**: 各ピクセルの輝度を `occupied_thresh` / `free_thresh` と比較し、占有（100）／自由（0）／未知（-1）に分類
3. **膨張処理**: `inflation_radius_m` の範囲を障害物として膨張（LETHALゾーン）し、その外側 `decay_margin_m` を線形に減衰させる
4. **発行**: 2つのトピックに、それぞれ異なるフレームで発行

`/map` は `map` フレーム、`/costmap/global` は `odom` フレームで発行されます。同じ内容を2フレームで出しているのは、経路計画が `map` 系で行われる一方、[core_mppi](../core_mppi/index.md) のコスト評価が `odom` 系で完結するためです。

QoSは `transient_local` を使用しており、このノードより後に起動したノードも地図を受け取れます。

膨張処理を地図側で行うことで、[core_path_planner](../core_path_planner/index.md) はロボットのサイズを意識せずにセル単位の探索ができます。

## Inputs / Outputs

### Input

購読するトピックはありません。画像ファイルとパラメータのみで動作します。

### Output

| トピック | 型 | QoS | フレーム | 説明 |
|---------|------|-----|---------|------|
| `/map` | `nav_msgs/OccupancyGrid` | transient_local | `map` | 経路計画用のフィールド地図 |
| `/costmap/global` | `nav_msgs/OccupancyGrid` | transient_local | `odom` | MPPIのコスト評価用グローバルコストマップ |

## Parameters

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `image_path` | string | `core1_field.png` | マップ画像の絶対パス（`navigation.launch.py` ではプリセットから自動設定） |
| `resolution` | double | `0.05` | 解像度 [m/px] |
| `origin_x` | double | `0.0` | マップ原点X [m] |
| `origin_y` | double | `0.0` | マップ原点Y [m] |
| `occupied_thresh` | double | `0.65` | 占有判定しきい値 |
| `free_thresh` | double | `0.25` | 自由判定しきい値 |
| `inflation_radius_m` | double | `0.0` | 障害物膨張半径（LETHALゾーン）[m]。`navigation.launch.py` では `0.40` |
| `decay_margin_m` | double | `0.0` | LETHALゾーン外の線形減衰幅 [m]。`navigation.launch.py` では `0.20` |

!!! info "navigation.launch.pyでの設定値"
    `map_name` プリセットにより自動設定されます。

    - `core1_field`: `resolution=0.025`, `origin_x=-13.675`, `origin_y=-9.15`
    - `curious_house`: `resolution=0.025`, `origin_x=-4.5`, `origin_y=-7.5`

## Assumptions / Known limits

- 地図は起動時に一度読み込まれるだけで、実行中の更新はできません。地図を変更するには再起動が必要です。
- `resolution` と `origin_x` / `origin_y` が実際のフィールド寸法と一致していることが前提です。これらがずれると、ロボットの自己位置は正しくても地図上の障害物位置が全体的にずれます。
- 動的障害物は含みません。相手ロボットなどへの対応は [core_costmap_builder](../core_costmap_builder/index.md) のローカルコストマップが担当します。
- `/costmap/global` を `odom` フレームで発行する都合上、`map` と `odom` が一致していることを前提としています。[core_localization](../core_localization/index.md) で `map → odom` が動的に補正される構成では、グローバルコストマップと実際の地図位置にずれが生じます。
