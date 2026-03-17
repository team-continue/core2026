# パラメータチューニング

各パラメータの定義は個別パッケージのドキュメントを参照してください。ここではチューニングの考え方と手順を説明します。

## MPPI コントローラ

設定ファイル: `core_mppi/param/default_params.yaml`（パラメータ一覧は[core_mppi](../packages/core_mppi.md#パラメータ)を参照）

### コスト重みのチューニング

MPPIの挙動を最も大きく左右するのはコスト重み（`w_path`, `w_goal`, `w_obstacle`, `w_control`, `w_smooth`）です。

!!! tip "チューニングの順序"
    1. まず `w_obstacle` で安全な回避を確保
    2. `w_path` と `w_goal` で経路追従とゴール到達のバランスを調整
    3. `w_control` と `w_smooth` で動きの滑らかさを仕上げる

| 重み | 上げた場合 | 下げた場合 |
|------|-----------|-----------|
| `w_path` | 経路に忠実に追従 | 経路から外れてもショートカット |
| `w_goal` | ゴールに向かいやすい | 経路追従優先 |
| `w_obstacle` | 障害物から大きく離れる | ギリギリを通る |
| `w_control` | 動きが穏やか・速度が出にくい | 急加速・急旋回しやすい |
| `w_smooth` | 軌道が滑らか | ジグザグが出やすい |

## PathFollower（PID制御）

設定ファイル: `core_path_follower/param/default_params.yaml`（パラメータ一覧は[core_path_follower](../packages/core_path_follower.md#パラメータ)を参照）

カスケードPID構成です:

- **外側PID**（`outer_kp/ki/kd`）: 方位誤差 → 角速度参照値を生成
- **内側PID**（`inner_kp/ki/kd`）: 角速度追従

`lookahead_dist` が小さいと旋回が鋭くなり、大きいとなだらかになります。

## cmd_vel スムーザー

`navigation.launch.py` 内のパラメータで設定（デフォルト有効）。パラメータ一覧は[core_cmd_vel_smoother](../packages/core_cmd_vel_smoother.md#パラメータ)を参照。

`alpha`（EMAフィルタ係数）が最も重要なパラメータです:

| alpha値 | 動作 |
|---------|------|
| 0.15-0.2 | 非常に滑らか、遅延大 |
| **0.3** | **バランス（デフォルト）** |
| 0.5-0.7 | 応答性重視、軽いスムージング |
| 1.0 | パススルー（無効化） |

!!! tip "チューニング手順"
    1. `use_smoother:=false` でスムーザーなしの挙動を確認
    2. スムーザー有効にして `alpha=0.3` から開始
    3. ガタつきが残るなら `alpha` を下げる、応答が鈍いなら上げる

## グローバルマップ膨張（map_server_node）

`navigation.launch.py` のmap_server_nodeパラメータで設定。パラメータ一覧は[core_launch — map_server_node](../packages/core_launch.md#map_server_node)を参照。

- `inflation_radius_m`（デフォルト `0.40`）: 障害物周囲のLETHAL膨張半径。A*はこの範囲内を通れない
- `decay_margin_m`（デフォルト `0.20`）: LETHAL外側の減衰幅。`cost_weight` と組み合わせて障害物から離れたパスを誘導

!!! warning "スタート/ゴールがLETHAL領域内に入る場合"
    膨張が大きすぎると壁近くのスタート/ゴールが占有扱いになりプラン失敗します。
    その場合は `inflation_radius_m` を縮小（例: `0.30`）してください。

## A* コスト重み（path_planner_node）

`navigation.launch.py` のpath_planner_nodeパラメータで設定。パラメータ一覧は[core_path_planner](../packages/core_path_planner.md#パラメータ)を参照。

`cost_weight`（デフォルト `2.0`）は decay zone のペナルティ倍率です。大きいほど障害物から離れたパスを選び、`0.0` で従来動作（コスト無視）になります。

## ローカルコストマップ

設定ファイル: `core_costmap_builder/config/costmap_build_node.yaml`（パラメータ一覧は[core_costmap_builder](../packages/core_costmap_builder.md#パラメータ)を参照）

チューニングのポイント:

- `inflation_radius_m`: ロボット半径以上に設定。大きいとMPPIが障害物を早めに避ける
- `decay_margin_m`: 大きいと遠くの障害物も避けるが、狭い通路を通りにくくなる
- `min_z_m`: 地面が障害物として検出される場合は値を上げる
- `voxel_leaf_m`: 大きいと処理が軽くなるが精度低下
