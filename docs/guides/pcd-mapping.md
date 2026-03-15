# PCD地図構築ガイド

`core_localization` パッケージを使ったグローバル局在化には、事前に構築したPCD（Point Cloud Data）地図が必要です。このガイドでは、FAST-LIOを使ったPCD地図の構築手順を説明します。

シミュレータ環境でも実機環境でも同じ流れで構築できます。

## 前提条件

=== "シミュレータ"

    - Unity シミュレータが起動済み
    - `fast_lio` パッケージがビルド済み

=== "実機"

    - Livox Mid-360 LiDARが接続済み
    - `fast_lio` パッケージがビルド済み
    - ロボットが手動操作可能な状態

## 手順

### 1. マッピング走行

FAST-LIOを有効にしてナビゲーションスタックを起動し、ロボットを手動操作してフィールド全体を走行します。

=== "シミュレータ"

    ```bash
    ros2 launch core_launch navigation.launch.py odom_source:=fastlio
    ```

    Unity 上でロボットを操作してフィールドを走行してください。

=== "実機"

    ```bash
    ros2 launch core_launch navigation.launch.py environment:=real
    ```

    SSH経由で起動する場合は `navigation.sh` を使用してください:

    ```bash
    ros2 run core_launch navigation.sh
    ```

    > SSH経由で `ros2 launch` が動かない場合の詳細は[ナビゲーション起動ガイド](navigation.md#起動navigation.sh-を使う場合)を参照。

!!! tip "走行のコツ"
    - フィールドの隅々まで走行してください
    - 急激な旋回を避け、ゆっくり移動すると精度が上がります
    - 同じ場所を複数回通ると地図の密度が上がります

### 2. PCD地図の保存

走行が完了したら、FAST-LIOの `map_save` サービスを呼び出してPCDファイルを保存します。

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

PCD ファイルはワークスペースのルート（`~/core_ws/`）に `scans.pcd` として保存されます。

!!! note "保存先の確認"
    FAST-LIOのログ出力でPCDファイルの保存先パスを確認してください。

### 3. PCD地図の配置

保存したPCDファイルを `core_localization/pcd_maps/` にコピーします。**ファイル名は `map_name` プリセットに合わせてください**。`use_localization:=true` で起動すると `map_name` に対応するPCD地図が自動的に使用されます。

```bash
# 例: CoRE-1フィールドで構築した場合
cp /path/to/scans.pcd ~/core_ws/src/core2026/core_localization/pcd_maps/core1_field.pcd

# 例: curious_house で構築した場合
cp /path/to/scans.pcd ~/core_ws/src/core2026/core_localization/pcd_maps/curious_house.pcd
```

配置後にビルドすると install ディレクトリにもコピーされます。

!!! warning "注意"
    PCD ファイルは大きい（数十MB〜数百MB）ため、`.gitignore` でGit管理から除外しています。チーム共有には別の手段（NAS、クラウドストレージなど）を使用してください。

### 4. 座標系について

FAST-LIOの `map_save` で保存されるPCD地図は `camera_init` フレーム（FAST-LIOの起動時のIMU位置が原点）で保存されます。

`core_localization` ノードは内部で `camera_init` フレームのままNDTマッチングを実施し、`odom→camera_init` TF（odom_bridgeが発行）を使って `map→odom` 補正に変換します。そのため、**PCD地図をマップフレームに変換する必要はありません**。

ただし、`initial_pose_x/y/yaw` パラメータでマッチングの初期推定を正しく設定する必要があります（`map_name` プリセットから自動設定されます）。

## localizationの起動

PCD地図が準備できたら、`use_localization:=true` を指定して起動します。`map_name` に対応する PCD 地図（`pcd_maps/<map_name>.pcd`）が自動的に使用されます。

=== "シミュレータ"

    ```bash
    # core1_field（デフォルト）
    ros2 launch core_launch navigation.launch.py \
      odom_source:=fastlio \
      use_localization:=true

    # curious_house
    ros2 launch core_launch navigation.launch.py \
      odom_source:=fastlio \
      map_name:=curious_house \
      use_localization:=true
    ```

=== "実機"

    ```bash
    ros2 launch core_launch navigation.launch.py \
      environment:=real \
      use_localization:=true
    ```

    SSH経由など `ros2 launch` で起動しない場合は `navigation.sh` を使ってください:

    ```bash
    ros2 run core_launch navigation.sh use_localization:=true
    ```

    > 詳細は[ナビゲーション起動ガイド — navigation.sh を使う場合](navigation.md#起動navigation.sh-を使う場合)を参照。

### RVizでの確認

起動後、RVizで以下を確認してください:

- `/localization/global_map` — ダウンサンプリング済みPCD地図が表示されること
- `/localization/aligned_cloud` — NDT整列済みスキャンが地図と重なること
- `map→odom` TFが動的に更新されること

> リモートPCからの RViz2 起動方法は[ナビゲーション起動ガイド — リモートPCで RViz2 を起動](navigation.md#2-リモートpcで-rviz2-を起動)を参照。

### 初期位置の手動設定

初期推定が不正確でマッチングが失敗する場合、RVizの「2D Pose Estimate」ツールで `/initialpose` を発行して初期位置を手動設定できます。

## トラブルシューティング

| 症状 | 原因 | 対策 |
|------|------|------|
| マッチングが収束しない | 初期推定が遠すぎる | `initial_pose_x/y/yaw` を調整、またはRVizで手動設定 |
| fitness_score が高い | スキャンと地図の重なりが少ない | `fitness_score_threshold` を緩和、または地図を再構築 |
| TFが飛ぶ | 平滑化が不十分 | `smooth_alpha` を小さく（例: 0.1） |
| ノードが起動しない | PCD地図が見つからない | `pcd_maps/<map_name>.pcd` が存在するか確認し、ビルドし直す |
