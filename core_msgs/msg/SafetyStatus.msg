# 安全停止の状態

uint8 STATE_PASS=0
uint8 STATE_SLOW=1
uint8 STATE_STOP=2
uint8 STATE_WATCHDOG_STOP=3
uint8 STATE_CONTACT_STOP=4
uint8 STATE_LOCALIZATION_STOP=5

std_msgs/Header header
uint8 state
string reason
# 最も近い衝突までの予測時間 [s]。衝突の予測がなければ負
float32 ttc_s
# 速度指令に掛けた倍率
float32 speed_scale
