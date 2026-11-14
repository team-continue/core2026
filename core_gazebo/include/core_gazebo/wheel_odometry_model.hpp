// シミュレーションの車輪の速度（実機の足回りが返す速度の代わり）を、真値の姿勢から作る計算。
// 真値の姿勢の差分で機体の速度を求め、実機の車輪で起きる誤差を足す。
// - 倍率の誤差: 車輪の径や取り付けの誤差。起動時に軸ごとに 1 回引く
// - ゆっくり変わる偏り: 床の状態など。1 次のマルコフ過程（定常の標準偏差と時定数）
// - 加減速の滑り: 加速すると車輪が速く回り、減速すると遅く回る。加速度 × accel_slip_s
// - ときどき起きる大きな滑り: ぶつかったときの空転など。指定の頻度で起き、向き・大きさ・長さはランダム。並進だけ
// - 白色雑音
// ROS に依存しないので、ノード（wheel_odometry_sim.cpp）と分けてテストする。

#ifndef CORE_GAZEBO__WHEEL_ODOMETRY_MODEL_HPP_
#define CORE_GAZEBO__WHEEL_ODOMETRY_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>

namespace core_gazebo::wheel_odometry
{
struct PlanarPose
{
  double stamp_s{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 機体の座標の速度（または加速度）
struct BodyTwist
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

inline double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 2 つの姿勢の差分から、その間の平均の速度を、間の真ん中の向きの機体の座標で返す
inline BodyTwist bodyTwistBetween(const PlanarPose & from, const PlanarPose & to)
{
  const double dt = to.stamp_s - from.stamp_s;
  if (!(dt > 0.0)) {
    throw std::invalid_argument("bodyTwistBetween: 姿勢の時刻が進んでいません");
  }
  const double dyaw = wrapAngle(to.yaw - from.yaw);
  const double mid_yaw = from.yaw + 0.5 * dyaw;
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double c = std::cos(mid_yaw);
  const double s = std::sin(mid_yaw);
  return BodyTwist{(c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt};
}

struct ErrorParams
{
  // 倍率の誤差の標準偏差（0.02 で 2%）
  double scale_std_x{0.02};
  double scale_std_y{0.03};
  double scale_std_yaw{0.03};
  // ゆっくり変わる偏りの定常の標準偏差と時定数
  double drift_std_linear_mps{0.01};
  double drift_std_angular_radps{0.01};
  double drift_time_constant_s{5.0};
  // 加減速の滑り [s]。加速度にこれを掛けた分だけ速く出る
  double accel_slip_s{0.01};
  // 大きな滑りの頻度 [1/s]、大きさ [m/s]、長さ [s]
  double burst_rate_hz{1.0 / 30.0};
  double burst_speed_min_mps{0.2};
  double burst_speed_max_mps{0.6};
  double burst_duration_min_s{0.1};
  double burst_duration_max_s{0.5};
  // 白色雑音の標準偏差
  double noise_std_linear_mps{0.02};
  double noise_std_angular_radps{0.02};
};

inline void validate(const ErrorParams & params)
{
  const auto require = [](const bool ok, const std::string & message) {
      if (!ok) {
        throw std::invalid_argument("車輪の速度の誤差の設定: " + message);
      }
    };
  require(
    params.scale_std_x >= 0.0 && params.scale_std_y >= 0.0 && params.scale_std_yaw >= 0.0,
    "scale_std_* は 0 以上にしてください");
  require(
    params.drift_std_linear_mps >= 0.0 && params.drift_std_angular_radps >= 0.0,
    "drift_std_* は 0 以上にしてください");
  require(params.drift_time_constant_s > 0.0, "drift_time_constant_s は正にしてください");
  require(params.accel_slip_s >= 0.0, "accel_slip_s は 0 以上にしてください");
  require(params.burst_rate_hz >= 0.0, "burst_rate_hz は 0 以上にしてください");
  require(
    0.0 <= params.burst_speed_min_mps && params.burst_speed_min_mps <= params.burst_speed_max_mps,
    "0 <= burst_speed_min_mps <= burst_speed_max_mps にしてください");
  require(
    0.0 < params.burst_duration_min_s && params.burst_duration_min_s <= params.burst_duration_max_s,
    "0 < burst_duration_min_s <= burst_duration_max_s にしてください");
  require(
    params.noise_std_linear_mps >= 0.0 && params.noise_std_angular_radps >= 0.0,
    "noise_std_* は 0 以上にしてください");
}

class WheelErrorModel
{
public:
  WheelErrorModel(const ErrorParams & params, const std::uint64_t seed)
  : params_((validate(params), params)),
    rng_(seed)
  {
    scale_ = BodyTwist{
      1.0 + params_.scale_std_x * normal_(rng_), 1.0 + params_.scale_std_y * normal_(rng_),
      1.0 + params_.scale_std_yaw * normal_(rng_)};
    // 偏りは定常の分布から始める
    drift_ = BodyTwist{
      params_.drift_std_linear_mps * normal_(rng_), params_.drift_std_linear_mps * normal_(rng_),
      params_.drift_std_angular_radps * normal_(rng_)};
  }

  // 真値の速度と加速度（機体の座標）に、前回から dt [s] 経った分の誤差を足す
  BodyTwist apply(const BodyTwist & truth, const BodyTwist & accel, const double dt)
  {
    advanceDrift(dt);
    advanceBurst(dt);
    return BodyTwist{
      scale_.vx * truth.vx + drift_.vx + params_.accel_slip_s * accel.vx + burst_.vx +
      params_.noise_std_linear_mps * normal_(rng_),
      scale_.vy * truth.vy + drift_.vy + params_.accel_slip_s * accel.vy + burst_.vy +
      params_.noise_std_linear_mps * normal_(rng_),
      scale_.wz * truth.wz + drift_.wz + params_.accel_slip_s * accel.wz +
      params_.noise_std_angular_radps * normal_(rng_)};
  }

  // 足回りが分かっているはずの誤差（倍率、偏り、雑音）の分散。大きな滑りと加減速の滑りは入れない
  BodyTwist variance(const BodyTwist & truth) const
  {
    const auto square = [](const double value) {return value * value;};
    const double linear = square(params_.noise_std_linear_mps) + square(params_.drift_std_linear_mps);
    const double angular =
      square(params_.noise_std_angular_radps) + square(params_.drift_std_angular_radps);
    return BodyTwist{
      linear + square(params_.scale_std_x * truth.vx), linear + square(params_.scale_std_y * truth.vy),
      angular + square(params_.scale_std_yaw * truth.wz)};
  }

  const BodyTwist & scale() const {return scale_;}
  bool inBurst() const {return burst_remaining_s_ > 0.0;}

private:
  void advanceDrift(const double dt)
  {
    // 1 次のマルコフ過程を厳密に離散化する（dt によらず定常の標準偏差が保たれる）
    const double keep = std::exp(-std::max(dt, 0.0) / params_.drift_time_constant_s);
    const double spread = std::sqrt(1.0 - keep * keep);
    drift_.vx = keep * drift_.vx + spread * params_.drift_std_linear_mps * normal_(rng_);
    drift_.vy = keep * drift_.vy + spread * params_.drift_std_linear_mps * normal_(rng_);
    drift_.wz = keep * drift_.wz + spread * params_.drift_std_angular_radps * normal_(rng_);
  }

  void advanceBurst(const double dt)
  {
    if (burst_remaining_s_ > 0.0) {
      burst_remaining_s_ -= dt;
      // 周期の足し引きの丸めで、長さちょうどの回に残らないようにする
      if (burst_remaining_s_ <= 1e-9) {
        burst_remaining_s_ = 0.0;
        burst_ = BodyTwist{};
      }
      return;
    }
    const double start_probability = 1.0 - std::exp(-params_.burst_rate_hz * std::max(dt, 0.0));
    if (uniform_(rng_) >= start_probability) {
      return;
    }
    const double speed = lerp(params_.burst_speed_min_mps, params_.burst_speed_max_mps, uniform_(rng_));
    const double direction = 2.0 * M_PI * uniform_(rng_);
    burst_ = BodyTwist{speed * std::cos(direction), speed * std::sin(direction), 0.0};
    burst_remaining_s_ =
      lerp(params_.burst_duration_min_s, params_.burst_duration_max_s, uniform_(rng_));
  }

  static double lerp(const double from, const double to, const double ratio)
  {
    return from + (to - from) * ratio;
  }

  ErrorParams params_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  BodyTwist scale_;
  BodyTwist drift_;
  BodyTwist burst_;
  double burst_remaining_s_{0.0};
};
}  // namespace core_gazebo::wheel_odometry

#endif  // CORE_GAZEBO__WHEEL_ODOMETRY_MODEL_HPP_
