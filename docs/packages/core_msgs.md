# core_msgs

カスタムROS2メッセージ定義パッケージです。

## メッセージ型

### CAN.msg

```
uint8 id
float32[] data
```

CANフレームの抽象化。モータ制御コマンドの送受信に使用します。

### CANArray.msg

```
CAN[] array
```

複数のCANメッセージをまとめて送信するための配列型。`body_control_node` が `/can/tx` にパブリッシュします。

### Path.msg

```
std_msgs/Header header
PoseWithWeight[] pose
```

重み付き経路メッセージ。

### PoseWithWeight.msg

```
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
float64 distance_to_obstable
```

障害物までの距離情報を持つ姿勢メッセージ。
