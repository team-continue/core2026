# 追跡中の物体。座標は TrackedObjectArray の header.frame_id 基準

uint8 CLASS_UNKNOWN=0
uint8 CLASS_ROBOT=1
uint8 CLASS_HUMAN=2

uint32 id
uint8 classification

# 物体の中心（z は床面）と速度
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# 観測から推定した大きさ [m]
float32 length
float32 width
float32 height

# 位置の標準偏差 [m]。隠れている間は大きくなる
float32 position_sigma

# 速度の推定の不確かさ [m/s]。速度の標準偏差のうち、追跡が落ち着いたときの値を超える分。
# 追跡を始めた直後や長く隠れた後は正、落ち着けば 0。使う側は時間を掛けて予測の広がりに足す
float32 velocity_sigma

# 追跡を始めてからの時間 [s]
float32 age_s

# 今のフレームで観測できず、予測だけで保っているとき true
bool coasting
