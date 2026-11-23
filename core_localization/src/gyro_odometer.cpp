#include "core_localization/gyro_odometer.hpp"

namespace core_localization
{
namespace
{
double orDefault(const double value, const double fallback)
{
  return value > 0.0 ? value : fallback;
}
}  // namespace

GyroOdometer::GyroOdometer(const GyroOdometerParams & params)
: params_(params)
{
}

void GyroOdometer::addGyro(const GyroRate & gyro)
{
  pending_.push_back(gyro);
  if (!last_gyro_.has_value() || gyro.stamp_s >= last_gyro_->stamp_s) {
    last_gyro_ = gyro;
  }
}

TwistEstimate GyroOdometer::combine(const WheelTwist & wheel)
{
  TwistEstimate estimate;
  estimate.stamp_s = wheel.stamp_s;
  estimate.vx = wheel.vx;
  estimate.vy = wheel.vy;
  estimate.var_vx = orDefault(wheel.var_vx, params_.wheel_linear_variance);
  estimate.var_vy = orDefault(wheel.var_vy, params_.wheel_linear_variance);

  // 車輪の時刻までの、ためたジャイロの平均（1 つの IMU の値は時刻の順に届く）
  double sum = 0.0;
  double variance_sum = 0.0;
  int count = 0;
  while (!pending_.empty() && pending_.front().stamp_s <= wheel.stamp_s) {
    sum += pending_.front().wz;
    variance_sum += orDefault(pending_.front().variance, params_.gyro_variance);
    ++count;
    pending_.pop_front();
  }
  if (count > 0) {
    estimate.wz = sum / count;
    // 平均で雑音は減るが、ジャイロの偏りは減らないので、1 つ分の分散のままにする
    estimate.var_wz = variance_sum / count;
    estimate.used_gyro = true;
  } else if (last_gyro_.has_value() && wheel.stamp_s - last_gyro_->stamp_s <= params_.gyro_timeout_s) {
    estimate.wz = last_gyro_->wz;
    estimate.var_wz = orDefault(last_gyro_->variance, params_.gyro_variance);
    estimate.used_gyro = true;
  } else {
    estimate.wz = wheel.wz;
    estimate.var_wz = orDefault(wheel.var_wz, params_.wheel_angular_variance);
    estimate.used_gyro = false;
  }
  return estimate;
}
}  // namespace core_localization
