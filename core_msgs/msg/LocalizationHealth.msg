# 自己位置の状態

uint8 LEVEL_OK=0
uint8 LEVEL_DEGRADED=1
uint8 LEVEL_FAILED=2

std_msgs/Header header
uint8 level
string reason

# 地図照合の評価値（小さいほどよい）。未取得なら負
float32 fitness
# 地図照合のインライア率。出力がなければ NaN
float32 inlier_ratio
# Mid-360 内蔵 IMU と Damiao IMU の角速度の差 [rad/s]
float32 gyro_difference
# 最後に地図照合が成功してからの時間 [s]
float32 time_since_match_s
