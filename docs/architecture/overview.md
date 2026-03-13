# システム概要

## ナビゲーションパイプライン

```mermaid
graph TB
    subgraph Inputs
        Unity["Unity Sim\n/sim_odom"]
        FASTLIO["FAST-LIO\n/Odometry"]
        MapPNG["core1_field.png"]
        LiDAR["Livox Mid-360\n/livox/lidar"]
    end

    BehaviorSystem["Behavior System"]

    subgraph Bridge["データ変換"]
        OdomBridge["odom_bridge_node"]
        MapServer["map_server_node"]
    end

    subgraph Planning["経路計画・コストマップ"]
        PathPlanner["path_planner_node"]
        CostmapBuilder["costmap_build_node"]
    end

    MPPI["core_mppi_node"]
    Smoother["cmd_vel_smoother_node"]

    subgraph Control["車体制御"]
        BodyController["body_control_node"]
        Hardware["core_hardware"]
    end

    RViz["RViz2"]

    Unity -->|/sim_odom| OdomBridge
    FASTLIO -->|/Odometry| OdomBridge
    MapPNG --> MapServer
    LiDAR -->|/livox/lidar| CostmapBuilder

    BehaviorSystem -->|/goal_pose| PathPlanner

    OdomBridge -->|/start_pose| PathPlanner
    OdomBridge -->|/odom| MPPI
    OdomBridge -->|TF| RViz
    MapServer -->|/map| PathPlanner
    MapServer -->|/costmap/global| MPPI

    PathPlanner -->|/planned_path| MPPI
    CostmapBuilder -->|/costmap/local| MPPI

    MPPI -->|/cmd_vel_raw| Smoother
    Smoother -->|/cmd_vel| BodyController
    BodyController -->|can/tx| Hardware

    style MapPNG fill:#e1f5fe,color:#333
    style Unity fill:#e1f5fe,color:#333
    style FASTLIO fill:#e1f5fe,color:#333
    style LiDAR fill:#e1f5fe,color:#333
    subgraph Shooter["射撃系"]
        Camera["カメラ"] -->|画像| TargetDetector["target_detector"]
        TargetDetector --> TargetSelector["target_selector"]
        TargetSelector -->|target_pose| AimBot["aim_bot"]
        ShooterGate["shooter_cmd_gate"] --> ShooterCtrl["shooter_controller"]
        MagazineMgr["magazine_manager"]
    end

    subgraph SystemMode["システム管理"]
        WirelessParser["wireless_parser_node"]
        EmergencyHandler["emergency_handler_node"]
        Diagnostic["diagnostic_node"]
        Diagnostic --> EmergencyHandler
    end

    WirelessParser -->|/cmd_vel| BodyController
    EmergencyHandler -->|/hazard_status| BodyController

    AimBot -->|can/tx| Hardware
    ShooterCtrl -->|can/tx| Hardware

    style BehaviorSystem fill:#fff3e0,color:#333
```

## ノード一覧

| ノード | パッケージ | 言語 | 役割 |
|--------|-----------|------|------|
| `odom_bridge_node` | core_launch | Python | オドメトリソース切替、座標変換、TFブロードキャスト |
| `map_server_node` | core_launch | Python | PNG画像をOccupancyGridに変換してパブリッシュ |
| `path_planner_node` | core_path_planner | C++ | A*アルゴリズムによるグローバル経路計画 |
| `core_mppi_node` | core_mppi | C++ | MPPI（Model Predictive Path Integral）ローカル制御、ゴール到達判定 |
| `cmd_vel_smoother_node` | core_cmd_vel_smoother | C++ | cmd_vel EMA平滑化フィルタ |
| `costmap_build_node` | core_costmap_builder | C++ | LiDAR点群からローリングウィンドウ式ローカルコストマップ生成 |
| `body_control_node` | core_body_controller | C++ | cmd_vel→オムニホイールCAN指令変換、レートリミッタ |
| `core_hardware` | core_hardware | C++ | EtherCAT（SOEM）によるTeensy41スレーブ通信 |
| `ros_tcp_endpoint` | ROS-TCP-Endpoint | Python | Unity-ROS2 TCPブリッジ |
| `localization_node` | core_localization | C++ | NDT/ICPによるPCDマップベースのグローバル局在化（`map→odom` 動的TF） |
| `target_detector` | core_enemy_detection | C++ | カメラ画像からダメージパネル検出 |
| `target_selector` | core_enemy_detection | C++ | 最大面積パネルのターゲット選択 |
| `emergency_handler_node` | core_mode | C++ | 緊急信号集約・ハザード状態管理 |
| `diagnostic` | core_mode | C++ | マイコン/受信機ハートビート監視 |
| `wireless_parser_node` | core_ros_player_controller | C++ | ワイヤレスコントローラ入力パーサー |
| `shooter_cmd_gate` | core_shooter | C++ | 射撃コマンドゲート（左右振り分け） |
| `shooter_controller` | core_shooter | C++ | 射撃モーター・ローディング制御（左右各1） |
| `magazine_manager` | core_shooter | C++ | ディスクマガジン管理（左右各1） |
| `aim_bot` | core_shooter | C++ | ビジョンベースターゲット追尾（左右各1） |

## 起動モード

| モード | コマンド | TCP EP | odom | localization |
|--------|---------|--------|------|-------------|
| sim（デフォルト） | `navigation.launch.py` | o | sim | x |
| sim + FAST-LIO | `navigation.launch.py odom_source:=fastlio` | o | FAST-LIO | x |
| 実機 | `navigation.launch.py environment:=real` | x | FAST-LIO | x |
| 実機 + localization | `navigation.launch.py environment:=real use_localization:=true pcd_map_path:=...` | x | FAST-LIO | o |

### シミュレータモード（デフォルト）

```bash
ros2 launch core_launch navigation.launch.py
```

起動ノード: ros_tcp_endpoint, odom_bridge, map_server, path_planner, mppi, cmd_vel_smoother, costmap_builder, rviz2

### 実機モード

```bash
ros2 launch core_launch navigation.launch.py environment:=real
```

シミュレータモードとの違い: TCP endpoint非起動、Livox driver起動、body_controller起動、odom_sourceはFAST-LIO固定。

## 静的TF

`navigation.launch.py` で以下の静的TFがブロードキャストされます:

| 親フレーム | 子フレーム | 変換 |
|-----------|-----------|------|
| `map` | `odom` | 恒等変換（デフォルト）。`use_localization:=true` 時は `localization_node` が動的に更新 |
| `base_link` | `livox_frame` | z=+0.5m, roll=π（上下反転） |

動的TFは[TFフレームと座標系](tf-tree.md)を参照してください。
