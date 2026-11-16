#include "core_planning/yaw_hold.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace core_planning
{
void validateYawHoldParams(const YawHoldParams & params)
{
  const auto require = [](const char * name, const double value, const double min, const double max) {
      if (!std::isfinite(value) || value < min || value > max) {
        std::ostringstream message;
        message << "指令の向きの保持の " << name << " は " << min << "〜" << max << " にしてください: " <<
          value;
        throw std::invalid_argument(message.str());
      }
    };
  require("timeout_s", params.timeout_s, 1e-3, 10.0);
  require("max_correction_rad", params.max_correction_rad, 0.0, M_PI);
  require("max_imu_gap_s", params.max_imu_gap_s, 1e-4, 1.0);
}

BodyVelocity holdWorldDirection(
  const BodyVelocity & command, const double yaw_at_command, const double yaw_now,
  const double max_correction_rad)
{
  // 機体が +Δ 回ったら、機体の座標の並進を −Δ 回すと、世界の座標の向きが変わらない
  const double turned = std::clamp(yaw_now - yaw_at_command, -max_correction_rad, max_correction_rad);
  const double c = std::cos(turned);
  const double s = std::sin(turned);
  return BodyVelocity{c * command.vx + s * command.vy, -s * command.vx + c * command.vy, command.wz};
}

YawHold::YawHold(const YawHoldParams & params)
: params_(params)
{
  validateYawHoldParams(params_);
}

void YawHold::onYawRate(const double stamp_s, const double yaw_rate_radps)
{
  if (!std::isfinite(stamp_s) || !std::isfinite(yaw_rate_radps)) {
    return;
  }
  if (yaw_stamp_s_.has_value()) {
    const double dt = stamp_s - *yaw_stamp_s_;
    if (dt > 0.0 && dt <= params_.max_imu_gap_s) {
      yaw_ += yaw_rate_radps * dt;
    }
  }
  yaw_stamp_s_ = stamp_s;
}

void YawHold::onCommand(const BodyVelocity & command, const double time_s)
{
  command_ = command;
  command_time_s_ = time_s;
  yaw_at_command_ = yaw_;
}

std::optional<BodyVelocity> YawHold::output(const double now_s) const
{
  if (!command_.has_value() || now_s - command_time_s_ > params_.timeout_s) {
    return std::nullopt;
  }
  return holdWorldDirection(*command_, yaw_at_command_, yaw_, params_.max_correction_rad);
}
}  // namespace core_planning
