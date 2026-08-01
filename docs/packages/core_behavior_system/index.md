# core_behavior_system

自律行動の意思決定を担うパッケージです。「今どのモードで動くべきか」を判断し、走行のゴールと射撃の指示をナビゲーション層・シューター層に振り分けます。

## 位置づけ

このパッケージはシステムの最上位に位置し、下位のナビゲーション・シューターに対して「何をするか」だけを指示します。「どう動くか」は下位パッケージが決めるという責務分担になっています。

```mermaid
graph TB
    subgraph Perception["認識層"]
        ED["core_enemy_detection<br/>敵検出"]
    end
    subgraph Decision["意思決定層（本パッケージ）"]
        BS["behavior_system_node<br/>状態機械"]
        EDC["enemy_detection_coordinator_node"]
        WS["waypoint_selector_node"]
        ASM["attack_shoot_manager_node"]
    end
    subgraph Action["実行層"]
        PP["core_path_planner<br/>経路計画"]
        SH["core_shooter<br/>射撃"]
        BC["core_body_controller<br/>車体制御"]
    end

    ED -->|"/left/target_pose<br>/right/target_pose"| EDC
    ED -->|"/left/target_pose<br>/right/damage_panel_pose"| ASM
    EDC -->|"/enemy_detected"| BS
    WS -->|"/waypoint_selector/goal_pose"| BS
    BS -->|"/waypoint_selector/pause"| WS
    BS -->|"/goal_pose"| PP
    BS -->|"/rotation"| BC
    BS -->|"/behavior_system/state"| ASM
    ASM -->|"/left/shoot_fullauto<br>/right/shoot_fullauto"| SH
```

## ノード一覧

| ノード | 実行ファイル | 役割 |
|-------|------------|------|
| [`behavior_system_node`](behavior_system_node.md) | `behavior_system_node` | 状態機械。5つの行動状態を判定しゴールと回転モードを指示 |
| [`waypoint_selector_node`](waypoint_selector_node.md) | `waypoint_selector_node` | 巡回ウェイポイントをスコアリングして次の目標を選択 |
| [`enemy_detection_coordinator_node`](enemy_detection_coordinator_node.md) | `enemy_detection_coordinator_node` | 左右タレットの検出結果を統合し、敵の有無を1つのフラグにまとめる |
| [`attack_shoot_manager_node`](attack_shoot_manager_node.md) | `attack_shoot_manager_node` | 攻撃状態かつ照準が合ったときに射撃を指示 |

## 制御フロー

意思決定は「敵の有無の判定 → 状態決定 → 状態に応じた指令の発行」という一方向の流れで構成されます。

```mermaid
sequenceDiagram
    participant ED as core_enemy_detection
    participant EDC as enemy_detection_coordinator
    participant BS as behavior_system
    participant WS as waypoint_selector
    participant ASM as attack_shoot_manager
    participant Nav as core_path_planner
    participant Sh as core_shooter

    ED->>EDC: /left,right/target_pose
    EDC->>BS: /enemy_detected
    WS->>BS: waypoint goal (巡回先)
    Note over BS: 状態を判定

    alt 敵を検出（ATTACK）
        BS->>WS: pause = true（巡回を停止）
        BS->>Nav: 停止ゴール
        BS->>ASM: state = ATTACK
        ED->>ASM: target_pose（照準位置）
        ASM->>Sh: shoot_fullauto
    else 巡回中（AUTO_WAYPOINT）
        BS->>WS: pause = false
        BS->>Nav: /goal_pose（巡回先）
    end
```

## 行動状態

`behavior_system_node` が管理する5つの状態です。詳細は[ノードページ](behavior_system_node.md)を参照してください。

```mermaid
stateDiagram-v2
    [*] --> MANUAL
    MANUAL --> AUTO_IDLE: manual_mode=false
    AUTO_IDLE --> AUTO_WAYPOINT: ウェイポイント目標あり
    AUTO_WAYPOINT --> ATTACK: 敵検出
    ATTACK --> AUTO_WAYPOINT: 敵ロスト
    AUTO_IDLE --> AUTO_SELECTED: 手動点選択で目標指定
    AUTO_SELECTED --> AUTO_IDLE: 到達
    AUTO_WAYPOINT --> MANUAL: manual_mode=true
    ATTACK --> MANUAL: manual_mode=true
    AUTO_SELECTED --> MANUAL: manual_mode=true
```

| 状態 | 値 | 意味 | LED |
|------|---|------|-----|
| `MANUAL` | 0 | 手動操縦中。自律指令を出さない | 2 |
| `ATTACK` | 1 | 敵を検出。その場で車体回転しつつ射撃 | 9 |
| `AUTO_SELECTED` | 2 | 手動で選択された地点へ移動 | 8 |
| `AUTO_WAYPOINT` | 3 | ウェイポイント巡回中 | 7 |
| `AUTO_IDLE` | 4 | 自律だが目標なし。待機 | 6 |

緊急停止中は状態に関わらずLEDが `4` になります。

## 起動

```bash
ros2 launch core_behavior_system behavior_system.launch.py
```

設定ファイル: `config/behavior_system.yaml`（全4ノード共通）
