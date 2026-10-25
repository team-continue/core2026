#pragma once

#include <cmath>
#include <optional>

#include "core_body_controller/target_angle_controller.hpp"

namespace core_body_controller
{

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

// base_link 座標系の速度 (vx, vy) を dt 秒積分する。
// new_yaw は積分後の yaw (IMU)。途中の向きは前後の yaw の中間で近似する。
inline std::optional<Pose2D> integrate_pose(
  const Pose2D & pose, double vx, double vy, double new_yaw, double dt)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
    !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(new_yaw) ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    return std::nullopt;
  }

  const double heading = pose.yaw + normalize_angle(new_yaw - pose.yaw) / 2;
  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);
  return Pose2D{
    pose.x + (vx * cos_h - vy * sin_h) * dt,
    pose.y + (vx * sin_h + vy * cos_h) * dt,
    new_yaw,
  };
}

}  // namespace core_body_controller
