# ROS-TCP-Endpoint

Unity-ROS2間のTCPソケットブリッジパッケージです。

## 概要

Unity アプリと ROS2 をTCPソケット経由で接続し、トピックの双方向通信を可能にします。

## 設定

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `ROS_IP` | `127.0.0.1` | リッスンIPアドレス |
| `ROS_TCP_PORT` | `10000` | リッスンポート |

## 起動

`navigation.launch.py` に含まれていますが、単体起動する場合:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

外部ホストから接続する場合は `ROS_IP:=0.0.0.0` を指定してください。

## Unity側との連携

詳細は[Unityシミュレータ](../getting-started/unity-sim.md)を参照してください。
