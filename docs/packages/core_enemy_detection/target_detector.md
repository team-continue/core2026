# target_detector

## Purpose

敵ロボットのダメージパネルは、黄色いパネル本体とチームカラー（赤／青）のLEDという色の組み合わせで識別できます。このノードはカメラ画像からその色特徴を持つ領域を抽出し、パネル候補の位置と大きさを下流に渡します。

## Inner-workings / Algorithms

`image_transport` 経由で受け取った画像に対し、以下のパイプラインを適用します。

1. **色空間変換**: LEDの検出にはHSV、パネル本体の検出にはLAB色空間を使用。照明変化に対する頑健性を色空間の使い分けで確保しています
2. **カラーフィルタリング**: 
   - LED: `enemy` パラメータに応じて赤（`red_range_*`）または青（`blue_range_*`）の範囲で2値化。赤はHue環の両端にまたがるため、`lower1/upper1` と `lower2/upper2` の2区間で判定します
   - パネル: `panel_lab_range_lower/upper` の範囲で2値化
3. **モルフォロジー処理**: LEDには `led_kernel_matrix_size`、パネルには `panel_kernel_matrix_size` のカーネルでノイズ除去と穴埋めを実施
4. **連結成分分析**: 残った領域をラベリングし、各成分の重心・面積・バウンディングボックスを算出
5. **出力**: 検出した全候補を `core_msgs/DamagePanelInfoArray` として発行

`color` トピックで実行中に検出対象チームカラーを切り替えられます。試合ごとに赤陣営／青陣営が入れ替わるため、再起動なしで対応できるようにしています。

`debug_mode` が有効な場合、検出結果を重畳した画像を発行します。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `raw_image` | `sensor_msgs/Image` | カメラ画像（`image_transport`、launchでリマップ） |
| `color` | `std_msgs/UInt8` | 検出対象チームカラーの切替 |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `damage_panels_infomation` | `core_msgs/DamagePanelInfoArray` | 検出されたダメージパネル候補の情報 |
| `result` | `sensor_msgs/Image` | デバッグ: 検出結果を重畳した画像（`debug_mode` 有効時） |

## Parameters

設定ファイル: `config/sim_param.yaml`

パラメータはランタイムでの動的再設定に対応しており、実機で色範囲を調整しながら検出結果を確認できます。

### カラーフィルタ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `enemy` | `[0]` | 検出モード（0=赤、1=青） |
| `red_range_lower1` | `[0, 100, 125]` | 赤色HSV下限1 [H, S, V] |
| `red_range_upper1` | `[10, 255, 255]` | 赤色HSV上限1 [H, S, V] |
| `red_range_lower2` | `[175, 125, 125]` | 赤色HSV下限2 [H, S, V] |
| `red_range_upper2` | `[180, 255, 255]` | 赤色HSV上限2 [H, S, V] |
| `blue_range_lower` | `[105, 64, 125]` | 青色HSV下限 [H, S, V] |
| `blue_range_upper` | `[135, 255, 255]` | 青色HSV上限 [H, S, V] |
| `panel_lab_range_lower` | `[0, 130, 95]` | パネルLAB下限 [L, A, B] |
| `panel_lab_range_upper` | `[100, 175, 150]` | パネルLAB上限 [L, A, B] |

### 画像処理

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `image_size` | `[1280, 720]` | 入力画像サイズ [幅, 高さ] |
| `led_kernel_matrix_size` | `[5, 5]` | LEDモルフォロジーカーネルサイズ |
| `panel_kernel_matrix_size` | `[9, 9]` | パネルモルフォロジーカーネルサイズ |
| `debug_mode` | `false` | デバッグ画像の発行を有効化 |

## Assumptions / Known limits

- 色ベースの検出のため、照明条件に強く依存します。会場の照明が想定と異なる場合は、実機で色範囲パラメータの再調整が必須です。
- 背景に検出対象と同系色の物体があると誤検出します。形状の妥当性検証は行っていません。
- `image_size` が実際の入力画像サイズと一致していることが前提です。不一致の場合、出力される座標がずれます。
- 検出は単一フレームごとに独立して行われ、フレーム間の追跡（トラッキング）は行いません。同一パネルに一貫したIDは付与されません。
- 距離情報は持ちません。出力されるのは画像上の位置と面積のみです。
