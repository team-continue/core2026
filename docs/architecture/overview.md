# システム概要

## ナビゲーションパイプライン

```mermaid
graph TB
    subgraph 入力ソース
        Unity["Unity Sim<br/>/sim_odom"]
        FASTLIO["FAST-LIO<br/>/Odometry"]
        LiDAR["Livox Mid-360<br/>/livox/lidar"]
        MapPNG["global_map.png"]
    end

    subgraph core_launch
        OdomBridge["odom_bridge_node<br/><i>オドメトリ変換・TFブロードキャスト</i>"]
        MapServer["map_server_node<br/><i>PNG→OccupancyGrid変換</i>"]
    end

    subgraph ナビゲーション
        PathPlanner["path_planner_node<br/><i>A*グローバル経路計画</i>"]
        MPPI["core_mppi_node<br/><i>MPPIローカル制御</i>"]
        CostmapBuilder["costmap_build_node<br/><i>ローカルコストマップ生成</i>"]
    end

    subgraph 制御・ハードウェア
        BodyController["body_control_node<br/><i>オムニホイール逆運動学</i>"]
        Hardware["core_hardware<br/><i>EtherCAT通信</i>"]
    end

    subgraph 可視化
        RViz["RViz2"]
    end

    Unity -->|"/sim_odom"| OdomBridge
    FASTLIO -->|"/Odometry"| OdomBridge
    MapPNG --> MapServer

    OdomBridge -->|"/odom"| MPPI
    OdomBridge -->|"/start_pose"| PathPlanner
    OdomBridge -->|"TF: odom→base_link"| RViz

    MapServer -->|"/map"| PathPlanner
    MapServer -->|"/costmap/global"| MPPI

    LiDAR -->|"/livox/lidar"| CostmapBuilder
    CostmapBuilder -->|"/costmap/local"| MPPI

    PathPlanner -->|"/planned_path"| MPPI

    MPPI -->|"/cmd_vel"| BodyController
    BodyController -->|"CAN (can/tx)"| Hardware

    style Unity fill:#e1f5fe
    style FASTLIO fill:#e1f5fe
    style LiDAR fill:#e1f5fe
    style MapPNG fill:#e1f5fe
```

## ノード一覧

| ノード | パッケージ | 言語 | 役割 |
|--------|-----------|------|------|
| `odom_bridge_node` | core_launch | Python | オドメトリソース切替、座標変換、TFブロードキャスト |
| `map_server_node` | core_launch | Python | PNG画像をOccupancyGridに変換してパブリッシュ |
| `path_planner_node` | core_path_planner | C++ | A*アルゴリズムによるグローバル経路計画 |
| `core_mppi_node` | core_mppi | C++ | MPPI（Model Predictive Path Integral）ローカル制御 |
| `costmap_build_node` | core_costmap_builder | C++ | LiDAR点群からローリングウィンドウ式ローカルコストマップ生成 |
| `body_control_node` | core_body_controller | C++ | cmd_vel→オムニホイールCAN指令変換、レートリミッタ |
| `core_hardware` | core_hardware | C++ | EtherCAT（SOEM）によるTeensy41スレーブ通信 |
| `ros_tcp_endpoint` | ROS-TCP-Endpoint | Python | Unity-ROS2 TCPブリッジ |

## 起動モード

### シミュレータモード（デフォルト）

```bash
ros2 launch core_launch navigation.launch.py
```

起動ノード: ros_tcp_endpoint, odom_bridge, map_server, path_planner, mppi, costmap_builder, rviz2

### FAST-LIOモード

```bash
ros2 launch core_launch navigation.launch.py odom_source:=fastlio
```

シミュレータモードに加えて `fastlio_mapping` ノードが起動し、odom_bridge が `/Odometry` を購読します。

## 静的TF

`navigation.launch.py` で以下の静的TFがブロードキャストされます:

| 親フレーム | 子フレーム | 変換 |
|-----------|-----------|------|
| `map` | `odom` | 恒等変換（x=0, y=0, yaw=0） |
| `base_link` | `livox_frame` | z=+0.6m, roll=π（上下反転） |

動的TFは[TFフレームと座標系](tf-tree.md)を参照してください。
