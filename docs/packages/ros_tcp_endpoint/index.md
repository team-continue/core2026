# ROS-TCP-Endpoint

## Purpose

UnityシミュレータはROS2のDDS通信に直接参加できません。TCPソケットを介したブリッジを提供し、Unity側のセンサ出力やオドメトリをROS2トピックとして流通させるための外部パッケージです。

!!! note "外部パッケージ"
    このパッケージは [Unity Robotics Hub](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) 由来の外部コードです。プロジェクト固有の改変は行っていないため、詳細な仕様は上流のドキュメントを参照してください。

## Inner-workings

`ROS_IP`:`ROS_TCP_PORT` でTCPソケットを待ち受け、接続してきたUnityクライアントとの間でメッセージをシリアライズして双方向に中継します。Unity側で発行されたトピックはROS2側の通常のトピックとして見え、その逆も同様です。

このプロジェクトでは、Unity側から `/sim_odom`（オドメトリ）や点群などを受け取り、ROS2側の `/cmd_vel` などをUnityに返す経路として使用します。

## Parameters

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `ROS_IP` | `127.0.0.1` | リッスンIPアドレス |
| `ROS_TCP_PORT` | `10000` | リッスンポート |

## Assumptions / Known limits

- TCPブリッジを経由するため、DDS直結に比べて遅延とジッタが大きくなります。制御の実時間性が問われる検証には向きません。
- デフォルトの `ROS_IP` は `127.0.0.1` で、同一ホストからの接続のみ受け付けます。別ホストのUnityから接続する場合は `0.0.0.0` を指定してください。
- 認証機構はありません。`0.0.0.0` で待ち受ける場合は、信頼できるネットワーク内でのみ使用してください。

## 起動

`navigation.launch.py` に含まれていますが、単体起動する場合:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Unity側との連携の詳細は[Unityシミュレータ](../../getting-started/unity-sim.md)を参照してください。
