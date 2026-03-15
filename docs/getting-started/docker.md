# Docker環境

Docker Compose を使用した開発環境です。

## 起動

```bash
cd ~/core_ws/src/core2026
docker compose up -d
docker exec -it core_container bash
```

## コンテナ内での操作

コンテナ内には便利なエイリアスが設定されています:

| コマンド | 動作 |
|---------|------|
| `cw` | ワークスペースルート (`/ros2_ws`) に移動 |
| `cs` | ソースディレクトリ (`/ros2_ws/src`) に移動 |
| `cb` | ワークスペース全体をビルド (`colcon build --symlink-install`) |
| `cb <pkg>` | 指定パッケージのみビルド |
| `cbd` | Debugビルド |
| `cl` | ビルド成果物をクリーン |

## 構成

- **ベースイメージ**: `ros:humble-ros-base`
- **ネットワーク**: `host` モード（EtherCAT/SOEM用）
- **ボリューム**:
    - ソースコードはホストからマウント（`.:/ros2_ws/src/`）
    - `build/` と `install/` はDockerボリュームで永続化
- **GUI対応**: X11フォワーディングでRViz2等を表示可能

## EtherCAT権限

EtherCAT（SOEM）を使用するため、`NET_RAW` と `NET_ADMIN` のケーパビリティが付与されています。
