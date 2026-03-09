# core_tools

デバッグ・診断用ユーティリティパッケージです。

## motor_tool

GUIベースのモータデバッグツールです。pythonQwtを使用してリアルタイムにモータ状態を可視化します。

### 環境構築

```bash
sudo apt update && sudo apt install -y --no-install-recommends python3-pip
pip3 install --no-cache-dir pythonQwt
```

### 起動

```bash
ros2 run core_tools motor_tool
```
