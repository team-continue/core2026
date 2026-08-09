# core_localization

事前構築したPCD点群マップに対してNDT/ICPマッチングを行い、グローバル自己位置推定（`map→odom` の動的補正）を提供するパッケージです。

## 位置づけ

FAST-LIOによるオドメトリは積算誤差でドリフトするため、走行時間が長くなるほどマップ上の位置がずれます。このパッケージはドリフトを吸収する補正層として、TFツリーの `map→odom` を担当します。

```mermaid
graph TB
    subgraph Local["ローカル推定（ドリフトする）"]
        LIO["FAST-LIO"] -->|"/Odometry"| OB["core_launch<br/>odom_bridge_node"]
        OB -->|"TF: odom→base_link"| TF1
    end
    subgraph Global["グローバル補正（本パッケージ）"]
        PCD(["PCD地図（事前構築）"]) --> LOC["localization_node"]
        LIO2["FAST-LIO"] -->|"/cloud_registered"| LOC
        LOC -->|"TF: map→odom"| TF1["TFツリー"]
    end
    TF1 -->|"TF参照"| USE["core_path_planner<br/>core_behavior_system<br/>（map座標系で動作）"]
```

## ノード一覧

| ノード | 実行ファイル | 役割 |
|-------|------------|------|
| [`localization_node`](localization_node.md) | `localization_node` | NDT/ICPによる位置合わせと `map→odom` の発行 |

## TFツリーの変化

`use_localization` の値によってTFツリーの構成が変わります。

```mermaid
graph LR
    subgraph OFF["use_localization:=false（デフォルト）"]
        M1["map"] -->|"静的な恒等変換"| O1["odom"] -->|"FAST-LIO"| B1["base_link"]
    end
```

```mermaid
graph LR
    subgraph ON["use_localization:=true"]
        M2["map"] -->|"動的<br/>NDT/ICP補正"| O2["odom"] -->|"FAST-LIO"| B2["base_link"]
    end
```

無効時は `map` と `odom` が一致するため、オドメトリのドリフトがそのままマップ上の位置誤差になります。有効時はこのノードが差分を補正し続けます。

## 処理サイクル

```mermaid
flowchart TD
    A["起動時: PCD地図読み込み<br/>map_voxel_size でダウンサンプリング"] --> B
    B["/cloud_registered 受信<br/>scan_voxel_size でダウンサンプリング"] --> C{"点数 ≧<br/>min_scan_points?"}
    C -->|"いいえ"| SKIP["スキップ"]
    C -->|"はい"| D["現在のTFから初期姿勢を計算"]
    D --> E["NDT または ICP で位置合わせ"]
    E --> F{"フィットネススコア ≦<br/>fitness_score_threshold?"}
    F -->|"いいえ（誤収束）"| REJ["リジェクト<br/>TFを更新しない"]
    F -->|"はい"| G["smooth_alpha で指数平滑"]
    G --> H["TF: map→odom をブロードキャスト"]
```

フィットネススコアによるリジェクトと指数平滑の2段構えで、誤ったマッチング結果がTFに反映されるのを防いでいます。

## 関連ガイド

PCD地図の構築・配置方法やフィールドごとの地図選択については[PCD地図構築ガイド](../../guides/pcd-mapping.md)を参照してください。

## 起動

### navigation.launch.py から起動（推奨）

```bash
ros2 launch core_launch navigation.launch.py \
  environment:=real \
  use_localization:=true
```

### 単体起動（テスト・開発用）

```bash
ros2 launch core_localization localization.launch.py \
  global_map_path:=/path/to/field.pcd
```

設定ファイル: `config/localization_params.yaml`、PCD地図: `pcd_maps/`
