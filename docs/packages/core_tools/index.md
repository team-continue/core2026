# core_tools

## Purpose

モータの挙動を調べるには、指令値と実際の回転をリアルタイムに突き合わせる必要があります。ログ出力を目で追うのは非効率なため、モータ状態をグラフで可視化するデバッグ用GUIツールを提供します。

## ノード一覧

| ノード | 役割 |
|-------|------|
| `motor_tool` | モータ状態をリアルタイムにグラフ表示するGUIツール |

## motor_tool

### Inner-workings

pythonQwtを使用したGUIアプリケーションです。ROS2ノードとして動作し、`/joint_states` などのモータ状態トピックを購読して時系列グラフに描画します。

### Parameters

パラメータはありません。表示対象はGUI上で選択します。

### Assumptions / Known limits

- デバッグ専用ツールです。試合運用のランチには含めないでください。
- pythonQwtの手動インストールが必要です（下記「環境構築」参照）。
- GUI環境が必要です。ヘッドレス環境では起動できません。

### 環境構築

```bash
sudo apt update && sudo apt install -y --no-install-recommends python3-pip
pip3 install --no-cache-dir pythonQwt
```

### 起動

```bash
ros2 run core_tools motor_tool
```
