#include "core_planning/yaw_hold.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
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
  require("max_reference_age_s", params.max_reference_age_s, 0.0, 5.0);
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
    // 時計が戻ったら、記録を捨てて始め直す（時刻の順を保つ）
    if (dt <= 0.0) {
      history_.clear();
    }
  }
  yaw_stamp_s_ = stamp_s;
  history_.emplace_back(stamp_s, yaw_);
  while (!history_.empty() && history_.front().first < stamp_s - params_.max_reference_age_s) {
    history_.pop_front();
  }
}

std::optional<double> YawHold::yawAt(const double stamp_s) const
{
  if (history_.empty() || !std::isfinite(stamp_s) || stamp_s < history_.front().first ||
    stamp_s > history_.back().first)
  {
    return std::nullopt;
  }
  const auto after = std::lower_bound(
    history_.begin(), history_.end(), stamp_s,
    [](const std::pair<double, double> & entry, const double t) {return entry.first < t;});
  if (after == history_.begin()) {
    return after->second;
  }
  const auto before = std::prev(after);
  const double span = after->first - before->first;
  const double ratio = span > 0.0 ? (stamp_s - before->first) / span : 0.0;
  return before->second + ratio * (after->second - before->second);
}

void YawHold::onCommand(
  const BodyVelocity & command, const double time_s, const std::optional<double> reference_s)
{
  command_ = command;
  command_time_s_ = time_s;
  yaw_at_command_ = yaw_;
  if (reference_s.has_value() && *reference_s <= time_s &&
    time_s - *reference_s <= params_.max_reference_age_s)
  {
    if (const auto yaw = yawAt(*reference_s)) {
      yaw_at_command_ = *yaw;
    }
  }
}

std::optional<BodyVelocity> YawHold::output(const double now_s) const
{
  if (!command_.has_value() || now_s - command_time_s_ > params_.timeout_s) {
    return std::nullopt;
  }
  return holdWorldDirection(*command_, yaw_at_command_, yaw_, params_.max_correction_rad);
}
}  // namespace core_planning
