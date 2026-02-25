#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "core_path_follower/pid.hpp"

namespace core_path_follower
{

struct ControlOutput
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

struct ControllerParam
{
  std::string controller_type{"cascade"};
  double linear_speed{0.6};
  double lookahead_dist{0.5};
  double goal_tolerance{0.15};
  // outer PID
  double outer_kp{1.2};
  double outer_ki{0.0};
  double outer_kd{0.15};
  // inner PID
  double inner_kp{2.0};
  double inner_ki{0.0};
  double inner_kd{0.05};
  // pure pursuit
  double pure_k{1.0};
};

/// Path-following controller (no ROS dependency)
class Controller
{
public:
  explicit Controller(const ControllerParam & param);

  void setParam(const ControllerParam & param);
  void reset();

  /// Compute control output in world-frame mode
  ControlOutput computeWorldFrame(
    const std::vector<geometry_msgs::msg::Pose> & path,
    const geometry_msgs::msg::Pose & current_pose,
    double angular_z, double dt,
    size_t & current_idx, bool & goal_reached);

  /// Compute control output in local-frame mode (path in chassis_link)
  ControlOutput computeLocalFrame(
    const std::vector<geometry_msgs::msg::Pose> & path,
    double angular_z, double dt,
    size_t & current_idx, bool & goal_reached);

private:
  size_t findLookaheadIndex(
    const std::vector<geometry_msgs::msg::Pose> & path,
    double cx, double cy, size_t current_idx) const;

  void advanceIndex(
    const std::vector<geometry_msgs::msg::Pose> & path,
    double cx, double cy, size_t & current_idx) const;

  bool checkGoal(
    const std::vector<geometry_msgs::msg::Pose> & path,
    double cx, double cy) const;

  ControlOutput computeCommand(
    double tx_body, double ty_body, double heading_err,
    double angular_z, double dt);

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);
  static std::pair<double, double> poseXY(const geometry_msgs::msg::Pose & p);
  static double wrapAngle(double angle);

  ControllerParam param_;
  PID outer_pid_;
  PID inner_pid_;

  static constexpr double kCloseThresh = 0.3;   // [m] advance-index threshold
};

} // namespace core_path_follower
