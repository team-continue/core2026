#include "core_path_follower/controller.hpp"

#include <algorithm>
#include <cmath>

namespace core_path_follower
{

  Controller::Controller(const ControllerParam &param)
      : param_(param),
        outer_pid_(param.outer_kp, param.outer_ki, param.outer_kd),
        inner_pid_(param.inner_kp, param.inner_ki, param.inner_kd)
  {
  }

  void Controller::setParam(const ControllerParam &param)
  {
    param_ = param;
    outer_pid_ = PID(param.outer_kp, param.outer_ki, param.outer_kd);
    inner_pid_ = PID(param.inner_kp, param.inner_ki, param.inner_kd);
  }

  void Controller::reset()
  {
    outer_pid_.reset();
    inner_pid_.reset();
  }

  ControlOutput Controller::computeWorldFrame(
      const std::vector<geometry_msgs::msg::Pose> &path,
      const geometry_msgs::msg::Pose &current_pose,
      double angular_z, double dt,
      size_t &current_idx, bool &goal_reached)
  {
    auto [cx, cy] = poseXY(current_pose);

    // Goal check
    if (checkGoal(path, cx, cy))
    {
      goal_reached = true;
      return {};
    }

    // Clamp index
    if (current_idx >= path.size())
    {
      current_idx = path.empty() ? 0 : path.size() - 1;
    }

    advanceIndex(path, cx, cy, current_idx);
    const size_t look_idx = findLookaheadIndex(path, cx, cy, current_idx);

    auto [tx, ty] = poseXY(path[look_idx]);

    // Transform target to body frame
    const double yaw = quaternionToYaw(current_pose.orientation);
    const double dx = tx - cx;
    const double dy = ty - cy;
    const double tx_body = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double ty_body = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    const double desired_yaw = std::atan2(dy, dx);
    const double heading_err = wrapAngle(desired_yaw - yaw);

    auto output = computeCommand(tx_body, ty_body, heading_err, angular_z, dt);

    // Advance index if close to current target
    auto [tpx, tpy] = poseXY(path[current_idx]);
    if (std::hypot(tpx - cx, tpy - cy) < kCloseThresh &&
        current_idx + 1 < path.size())
    {
      current_idx++;
    }

    return output;
  }

  ControlOutput Controller::computeLocalFrame(
      const std::vector<geometry_msgs::msg::Pose> &path,
      double angular_z, double dt,
      size_t &current_idx, bool &goal_reached)
  {
    constexpr double cx = 0.0;
    constexpr double cy = 0.0;

    // Goal check
    if (checkGoal(path, cx, cy))
    {
      goal_reached = true;
      return {};
    }

    // Clamp index
    if (current_idx >= path.size())
    {
      current_idx = path.size() - 1;
    }

    advanceIndex(path, cx, cy, current_idx);
    const size_t look_idx = findLookaheadIndex(path, cx, cy, current_idx);

    auto [tx, ty] = poseXY(path[look_idx]);

    // In local frame, target is already in body coordinates
    const double heading_err = std::atan2(ty, tx);

    auto output = computeCommand(tx, ty, heading_err, angular_z, dt);

    // Advance index if close to current target
    auto [tpx, tpy] = poseXY(path[current_idx]);
    if (std::hypot(tpx - cx, tpy - cy) < kCloseThresh &&
        current_idx + 1 < path.size())
    {
      current_idx++;
    }

    return output;
  }

  void Controller::advanceIndex(
      const std::vector<geometry_msgs::msg::Pose> &path,
      double cx, double cy, size_t &current_idx) const
  {
    double best_d = 1e9;
    size_t best_i = current_idx;
    const size_t search_end = std::min(path.size(), current_idx + 20);
    for (size_t i = current_idx; i < search_end; ++i)
    {
      auto [px, py] = poseXY(path[i]);
      const double d = std::hypot(px - cx, py - cy);
      if (d < best_d)
      {
        best_d = d;
        best_i = i;
      }
    }
    if (best_d < param_.lookahead_dist * 2.0)
    {
      current_idx = best_i;
    }
  }

  size_t Controller::findLookaheadIndex(
      const std::vector<geometry_msgs::msg::Pose> &path,
      double cx, double cy, size_t current_idx) const
  {
    size_t look_idx = current_idx;
    for (size_t i = current_idx; i < path.size(); ++i)
    {
      auto [px, py] = poseXY(path[i]);
      if (std::hypot(px - cx, py - cy) >= param_.lookahead_dist)
      {
        return i;
      }
      look_idx = i;
    }
    return look_idx;
  }

  bool Controller::checkGoal(
      const std::vector<geometry_msgs::msg::Pose> &path,
      double cx, double cy) const
  {
    auto [gx, gy] = poseXY(path.back());
    return std::hypot(gx - cx, gy - cy) < param_.goal_tolerance;
  }

  ControlOutput Controller::computeCommand(
      double tx_body, double ty_body, double heading_err,
      double angular_z, double dt)
  {
    double omega_des = 0.0;
    double ang_z_cmd = 0.0;

    if (param_.controller_type == "pid")
    {
      ang_z_cmd = outer_pid_.update(heading_err, dt);
    }
    else if (param_.controller_type == "cascade")
    {
      omega_des = outer_pid_.update(heading_err, dt);
      ang_z_cmd = inner_pid_.update(omega_des - angular_z, dt);
    }
    else if (param_.controller_type == "pure_pursuit")
    {
      const double alpha = std::atan2(ty_body, tx_body);
      const double L = std::max(std::hypot(tx_body, ty_body), 1e-6);
      omega_des = param_.pure_k * 2.0 * param_.linear_speed * std::sin(alpha) / L;
      ang_z_cmd = inner_pid_.update(omega_des - angular_z, dt);
    }
    else
    {
      // Fallback to proportional
      ang_z_cmd = param_.outer_kp * heading_err;
    }

    // Translational velocity
    const double dist = std::hypot(tx_body, ty_body);
    double vx = 0.0;
    double vy = 0.0;
    if (dist > 1e-6)
    {
      const double heading_factor =
          std::max(0.0, 1.0 - std::min(1.0, std::abs(heading_err) / (M_PI / 2.0)));
      const double speed = param_.linear_speed * heading_factor;

      if (param_.controller_type == "pure_pursuit")
      {
        const double alpha = std::atan2(ty_body, tx_body);
        vx = speed * std::cos(alpha);
        vy = speed * std::sin(alpha);
      }
      else
      {
        vx = speed * (tx_body / dist);
        vy = speed * (ty_body / dist);
      }
    }

    return {vx, vy, ang_z_cmd};
  }

  double Controller::quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::pair<double, double> Controller::poseXY(const geometry_msgs::msg::Pose &p)
  {
    return {p.position.x, p.position.y};
  }

  double Controller::wrapAngle(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

} // namespace core_path_follower
