# LED テープ 1 本分の表示指令。
# pattern: led.h の Pattern (0 = kOff は「指令なし」の意味でも使う)
# color: 0 = 白, 1 = 赤, 2 = 黄, 3 = 緑, 4 = 青。レインボー系のパターンでは無視される
uint8 pattern
uint8 color
# true なら、一方向に流れるパターン (kChase / kMultiChase / kRainbowChase / kRainbowMultiChase) を区間の末尾から先頭へ流す。
# 左右対称のパターン (kInwardChase / kCenterOut*) や点滅には効かない
bool reverse
