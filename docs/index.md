# core2026 ドキュメント

CoRE2026の機体コンセプト、メカ、回路、制御システム、および開発手順をまとめたドキュメントです。

## 機体コンセプト

[2026年度 機体コンセプト](robot-concept.md)では、機体全体で目指す姿と、メカ・回路・制御に共通する設計方針を説明します。

## はじめに

環境構築からシステムの起動までを案内します。

| ページ | 内容 |
|-------|------|
| [クイックスタート](getting-started/quick-start.md) | ROS 2ワークスペースのビルドと基本的な起動方法 |
| [Docker環境](getting-started/docker.md) | Docker Composeを利用した開発環境 |
| [Unityシミュレータ](getting-started/unity-sim.md) | ROS-TCP-Endpointを介したUnityとの接続方法 |

## メカ

[メカ構成](mechanics/index.md)を入口として、ソフトウェアが前提とする機体寸法や可動範囲を説明します。

| ページ | 内容 |
|-------|------|
| [駆動系・旋回機構](mechanics/drivetrain.md) | 4輪オムニ、無限回転機構、逆運動学、加減速制限 |
| [砲塔・装填・発射機構](mechanics/turret.md) | 左右砲塔の可動範囲、マガジン、発射機構 |

## 回路

[回路構成](circuit/index.md)を入口として、制御PC、マイコン、モーター間の接続とID割り当てを説明します。

| ページ | 内容 |
|-------|------|
| [基板とピン配置](circuit/boards.md) | Teensy 4.1のピン割り当てとファームウェアのビルド |
| [CANバスとモーターID](circuit/buses.md) | CANバス、モーターID、EtherCAT PDO、死活監視 |

## 制御

ROS 2上のノード構成、トピック、座標系、および各パッケージの仕様を説明します。

### アーキテクチャ

| ページ | 内容 |
|-------|------|
| [システム概要](architecture/overview.md) | 機体全体のノード構成、データフロー、現在の起動構成 |
| [トピック・メッセージ一覧](architecture/topics.md) | Publisher、Subscriber、型、QoS、リマップ |

### パッケージ一覧

`docs/packages/` にあるパッケージドキュメントを、システム上の役割に沿って分類しています。

| レイヤー | パッケージ |
|---------|-----------|
| Launch | [core_launch](packages/core_launch/index.md) |
| Sensing | [core_damiao_imu](packages/core_damiao_imu/index.md)、[core_camera](packages/core_camera/index.md) |
| Localization | [core_localization](packages/core_localization/index.md) |
| Perception | [core_enemy_detection](packages/core_enemy_detection/index.md) |
| Planning | [core_costmap_builder](packages/core_costmap_builder/index.md)、[core_path_planner](packages/core_path_planner/index.md)、[core_mppi](packages/core_mppi/index.md)、[core_path_follower](packages/core_path_follower/index.md)、[core_cmd_vel_smoother](packages/core_cmd_vel_smoother/index.md) |
| Behavior | [core_behavior_system](packages/core_behavior_system/index.md) |
| Mecha | [core_shooter](packages/core_shooter/index.md) |
| Control | [core_body_controller](packages/core_body_controller/index.md) |
| System | [core_mode](packages/core_mode/index.md) |
| UI | [core_qt_gui](packages/core_qt_gui/index.md)、[core_status_gui](packages/core_status_gui/index.md)、[core_ros_player_controller](packages/core_ros_player_controller/index.md) |
| Hardware | [core_hardware](packages/core_hardware/index.md) |
| Common | [core_msgs](packages/core_msgs/index.md)、[core_tools](packages/core_tools/index.md)、[core_test](packages/core_test/index.md)、[ROS-TCP-Endpoint](packages/ros_tcp_endpoint/index.md) |

## 開発

リポジトリへ変更を加える際のルールと、ドキュメントの記述方法を説明します。

| ページ | 内容 |
|-------|------|
| [開発ワークフロー](contributing/workflow.md) | ブランチ規則、ビルド、テスト、Pull Request |
| [コードスタイル](contributing/code-style.md) | C++とPythonのフォーマット規則 |
| [ノードドキュメントの書き方](contributing/node-documentation.md) | パッケージ・ノード文書の構成と追加方法 |
