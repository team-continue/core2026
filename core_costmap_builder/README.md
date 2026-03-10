コストマップの生成を行うnodeです。

## Memo
コストマップに関して，いい感じのトピックは存在指定なさそうなので独自トピックでも可能です．
グローバルコストマップとローカルコストマップをフュージョンして出してくれるとめっちゃ嬉しい．

ROS 2のNav2のコストマップ冗長な上に実装がエグゼキュータなせいでめちゃでかいのであんまり参考にならんかも -- curious

## Node
- costmap_build_node

## Subscribe
- points_in_topic (sensor_msgs/PointCloud2) ※パラメータでトピック名を変更可能

## Publish
- local_topic (ローカルコストマップ) ※パラメータでトピック名を変更可能
- points_filt_topic (デバッグ用フィルタ済み点群)

## TF
- map -> odom -> base_link -> lidar