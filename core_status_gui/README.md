# core_status_gui

## 使用方法

```bash
ros2 run core_status_gui status_display_gui
```

## 概要

- 通常時は `/system/behavior` を全画面表示
- 緊急時は `/system/emergency/hazard_state` または `/system/emergency/hazard_status` を優先して `EMERGENCY` 表示
- `/system/emergency/hazard_label` を最大2行 + `+N MORE` で表示
