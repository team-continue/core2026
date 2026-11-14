# 各処理の 1 周期分の統計。性能の計測に使う
std_msgs/Header header
uint32 input_points
uint32 output_points
float32 processing_ms
# 計算が間に合わず飛ばしたフレームの累積数
uint32 skipped_frames
