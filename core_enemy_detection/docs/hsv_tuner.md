# HSV調整GUI

## 起動

各ターミナルで `source /opt/ros/humble/setup.bash` と
`source ~/ros2_ws/install/setup.bash` を実行します。
GUIはビルドしてから起動します。画像のプレビューだけなら検出器は不要です。

```bash
ros2 run core_enemy_detection hsv_tuner
```

画像トピックや対象ノードを変更する場合:

```bash
ros2 run core_enemy_detection hsv_tuner --ros-args \
  -p image_topic:=/raw_image -p detector:=/target_detector
```

rosbag → detected_panel_image_republisher → /raw_image の再生を先に開始します。
GUIはPanel、Blue LED、Red LED 1/2の各タブに上下限H/S/Vのスライダーがあります。
赤色は色相の端をまたぐため、二つの範囲のORを表示します。
Hは0～179、S/Vは0～255。黒いパネルはまずV上限で暗い部分を抽出します。
初期値は調整用で、実行中の検出器と自動同期しません。

- 「プレビューを固定」: 最後の画像でしきい値を調整。bag再生は停止しません。
- 「検出器から読込」: 現在の8個のHSV配列を取得。不正な範囲はエラー表示します。
- 「検出器へ反映」: 全HSV配列を一つのatomicサービス要求で送信。
- 「YAML保存」: 現在の対象ノード名とHSV配列を保存（既存設定へ追記はしません）。
- 「YAML読込」: 対象ノード名または `/**` のHSV設定を読込。反映ボタンは別操作です。

スライダーはプレビューだけを変更し、反映ボタンを押すまで検出器には送信しません。
プレビューは色抽出のみで、ノイズ除去・上下判定後の検出結果とは異なります。
検出器の結果は従来どおり `/result` で確認してください。
GUIは `/color` を変更しません（青81・赤67の指定は従来どおり必要）。

検出器側のパラメータ検証も修正しているので、反映機能を使う前に再ビルドし、
実行中の検出器を終了して新しいバイナリを起動してください。

```bash
cd ~/ros2_ws
colcon build --packages-up-to core_enemy_detection --symlink-install
source install/setup.bash
ros2 run core_enemy_detection target_detector --ros-args --params-file /absolute/path/hsv_params.yaml -p debug_mode:=true
```

## コードの説明

C++版の `src/hsv_tuner.cpp` 内に日本語コメントで説明しています。
