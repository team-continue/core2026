// 車輪の速度と IMU の角速度を合わせて、機体の速度を出す計算（Autoware の gyro_odometer と同じ分け方）。
// 並進（vx, vy）は車輪から、回転（wz）はジャイロから取る。全方向に動く機体なので、横の速度 vy も車輪から取る。
// ROS に依存しないので、ノード（gyro_odometer_node.cpp）と分けてテストする。

#ifndef CORE_LOCALIZATION__GYRO_ODOMETER_HPP_
#define CORE_LOCALIZATION__GYRO_ODOMETER_HPP_

#include <deque>
#include <optional>

namespace core_localization
{
struct GyroOdometerParams
{
  // IMU のメッセージに角速度の分散がない（0 以下の）ときに使う分散 [(rad/s)^2]
  double gyro_variance{1.0e-4};
  // 車輪のメッセージに分散がないときに使う分散 [(m/s)^2]、[(rad/s)^2]
  double wheel_linear_variance{0.01};
  double wheel_angular_variance{0.04};
  // 最後のジャイロからこの時間を超えたら、ジャイロが途絶えたとみなし、車輪の角速度を使う [s]
  double gyro_timeout_s{0.1};
};

// 車輪の速度（base_link の座標系）
struct WheelTwist
{
  double stamp_s{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  // 分散。0 以下なら GyroOdometerParams の値を使う
  double var_vx{0.0};
  double var_vy{0.0};
  double var_wz{0.0};
};

// ジャイロの角速度（base_link の座標系に直した z 成分）
struct GyroRate
{
  double stamp_s{0.0};
  double wz{0.0};
  // 分散。0 以下なら GyroOdometerParams の値を使う
  double variance{0.0};
};

// 合わせた速度
struct TwistEstimate
{
  double stamp_s{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double var_vx{0.0};
  double var_vy{0.0};
  double var_wz{0.0};
  // false なら、ジャイロが途絶えていて車輪の角速度を使った
  bool used_gyro{false};
};

class GyroOdometer
{
public:
  explicit GyroOdometer(const GyroOdometerParams & params);

  // ジャイロの値をためる
  void addGyro(const GyroRate & gyro);

  // 車輪の速度を受けたら、ためたジャイロのうち車輪の時刻までの値の平均と合わせる。前回に合わせた後に届いた値は、
  // 時刻が前回の車輪より古くても入れる（融合した IMU は 2 つ目の IMU を最大 20ms 待つので遅れて届く。
  // 捨てると角速度が古くなる。Autoware の gyro_odometer も、前回の出力から後に届いた値をすべて平均する）。
  // ジャイロが一つもなければ、途絶えていない間は最後のジャイロの値を、途絶えていれば車輪の角速度を使う
  TwistEstimate combine(const WheelTwist & wheel);

  const GyroOdometerParams & params() const {return params_;}

private:
  GyroOdometerParams params_;
  std::deque<GyroRate> pending_;
  std::optional<GyroRate> last_gyro_;
};
}  // namespace core_localization

#endif  // CORE_LOCALIZATION__GYRO_ODOMETER_HPP_
