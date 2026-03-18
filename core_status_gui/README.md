# core_status_gui

## 使用方法

```bash
ros2 run core_status_gui status_display_gui
```

```bash
ros2 launch core_status_gui status_display_gui.launch.py
```

## 概要

- 通常時は `/behavior_system/state_name` を全画面表示
- 表示対象の状態名は `MANUAL` / `ATTACK` / `AUTO_SELECTED` / `AUTO_WAYPOINT` / `AUTO_IDLE`
- 既定では `/system/emergency/hazard_status` を購読し、互換用途で `/system/emergency/hazard_state` も購読可能
- `/system/emergency/hazard_label` を最大2行 + `+N MORE` で表示
- hazard 系は transient local / volatile のどちらの publisher とも接続可能
- hazard 状態を未受信の間は fail-safe で `STATUS UNKNOWN` を表示
