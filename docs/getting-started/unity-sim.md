# Unityシミュレータ接続

## 概要

ROS-TCP-Endpoint を介して Unity シミュレータと ROS2 を TCP ソケットで接続します。

## 接続構成

```
Unity アプリ ──TCP:10000──▶ ROS-TCP-Endpoint ──▶ ROS2トピック
```

## 起動手順

### 1. ROS2側

`navigation.launch.py` を起動すると、ROS-TCP-Endpoint が自動的に起動します。

```bash
ros2 launch core_launch navigation.launch.py
```

デフォルト設定:

- IP: `127.0.0.1`
- ポート: `10000`

### 2. Unity側

Unity アプリ内の ROS Connection 設定で、ROS2 側の IP とポート（10000）を指定してください。

## データフロー

Unity から ROS2 に送信されるトピック:

| トピック | 型 | 内容 |
|---------|------|------|
| `/sim_odom` | `nav_msgs/Odometry` | ロボットのオドメトリ |

!!! note "座標変換"
    Unity の座標系（X=右, Y=上, Z=前）は odom_bridge で ROS2 の座標系（X=前, Y=左, Z=上）に変換されます。詳細は[座標系](../architecture/tf-tree.md)を参照。
