コストマップの生成を行うnodeです。

## Memo
コストマップに関して，いい感じのトピックは存在指定なさそうなので独自トピックでも可能です．
グローバルコストマップとローカルコストマップをフュージョンして出してくれるとめっちゃ嬉しい．

ROS 2のNav2のコストマップ冗長な上に実装がエグゼキュータなせいでめちゃでかいのであんまり参考にならんかも -- curious

## Node
- costmap_builder

## Subscribe
- /map (nav_msgs/OccupancyGrid)
- /points (sensor_msgs/PointCloud2)

## Publish
- /costmap/global
- /costmap/local
- /costmap/fused

## TF
- map -> odom -> base_link -> lidar