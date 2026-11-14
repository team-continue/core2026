# 局所計画の状態

uint8 STATE_IDLE=0
uint8 STATE_DRIVING=1
uint8 STATE_STUCK_WAIT=2
uint8 STATE_BACKOFF=3
uint8 STATE_GOAL_WAIT=4
uint8 STATE_REACHED=5
uint8 STATE_FAILED=6

std_msgs/Header header
uint8 state
# 行き詰まりからの復帰を試みた回数
uint32 retries
# 今の周期で使った速度の上限 [m/s]
float32 speed_limit_mps
# 計画の計算時間 [ms]
float32 planning_ms
# ゴールまでの距離 [m]
float32 distance_to_goal_m
