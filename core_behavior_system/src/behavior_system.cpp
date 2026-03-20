#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "core_behavior_system/waypoint_selector.hpp"

#include <cmath>

class WaypointSelectorNode : public rclcpp::Node {
public:
  explicit WaypointSelectorNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("waypoint_selector", options) {
    declare_parameter<int>("update_rate_hz", 10);
    declare_parameter<std::string>("mode", "idle");
    declare_parameter<double>("risk_threshold", 0.5);
    declare_parameter<double>("w_distance", 1.0);
    declare_parameter<double>("w_angle", 3.0);
    declare_parameter<double>("w_time", 5.0);
    declare_parameter<double>("revisit_cooldown", 15.0);
    declare_parameter<bool>("use_exponential_decay", true);
    declare_parameter<bool>("forward_only", false);
    declare_parameter<double>("random_jitter", 0.01);
    declare_parameter<std::vector<double>>("weights", {1.0, 1.0, 1.0});
    declare_parameter<std::vector<double>>("waypoints", {});
    declare_parameter<double>("current_x", 0.0);
    declare_parameter<double>("current_y", 0.0);
    declare_parameter<double>("current_yaw", 0.0);
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    declare_parameter<std::string>("pause_topic", "/waypoint_selector/pause");
    declare_parameter<double>("arrival_radius", 0.5);

    update_rate_hz_ = get_parameter("update_rate_hz").as_int();
    mode_ = get_parameter("mode").as_string();
    risk_threshold_ = get_parameter("risk_threshold").as_double();
    weights_ = get_parameter("weights").as_double_array();
    waypoints_ = get_parameter("waypoints").as_double_array();
    if (waypoints_.size() % 2 != 0) {
      RCLCPP_WARN(get_logger(),
                  "waypoints parameter should have even length (x,y pairs).");
    }

    RCLCPP_INFO(get_logger(), "Parameters loaded from YAML or defaults:");
    RCLCPP_INFO(get_logger(), "  update_rate_hz: %d", update_rate_hz_);
    RCLCPP_INFO(get_logger(), "  mode: %s", mode_.c_str());
    RCLCPP_INFO(get_logger(), "  risk_threshold: %.3f", risk_threshold_);
    RCLCPP_INFO(get_logger(), "  weights size: %zu", weights_.size());
    RCLCPP_INFO(get_logger(), "  waypoints size: %zu", waypoints_.size());

    current_x_ = get_parameter("current_x").as_double();
    current_y_ = get_parameter("current_y").as_double();
    current_yaw_ = get_parameter("current_yaw").as_double();
    odom_topic_ = get_parameter("odom_topic").as_string();
    goal_pose_topic_ = get_parameter("goal_pose_topic").as_string();
    pause_topic_ = get_parameter("pause_topic").as_string();
    arrival_radius_ = get_parameter("arrival_radius").as_double();

    selector_config_.w_distance = get_parameter("w_distance").as_double();
    selector_config_.w_angle = get_parameter("w_angle").as_double();
    selector_config_.w_time = get_parameter("w_time").as_double();
    selector_config_.revisit_cooldown =
        get_parameter("revisit_cooldown").as_double();
    selector_config_.use_exponential_decay =
        get_parameter("use_exponential_decay").as_bool();
    selector_config_.forward_only = get_parameter("forward_only").as_bool();
    selector_config_.random_jitter = get_parameter("random_jitter").as_double();
    selector_.set_config(selector_config_);

    rclcpp::QoS goal_qos(1);
    goal_qos.transient_local();
    goal_qos.reliable();
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, goal_qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_x_ = msg->pose.pose.position.x;
          current_y_ = msg->pose.pose.position.y;
          current_yaw_ = YawFromQuat(msg->pose.pose.orientation);
          have_odom_ = true;
        });

    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        pause_topic_, rclcpp::QoS(1),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          const bool was_paused = paused_;
          paused_ = msg->data;
          if (was_paused && !paused_) {
            current_target_id_ = -1;
          }
        });

    marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("waypoints", 10);

    const int safe_rate = update_rate_hz_ > 0 ? update_rate_hz_ : 1;
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / safe_rate),
        [this]() { publish_waypoints(); });
  }

private:
  void publish_waypoints() {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for odom on %s", odom_topic_.c_str());
    }
    if (paused_) {
      return;
    }
    SelectWaypoint();
    PublishGoalPose();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.9;
    marker.color.b = 0.1;

    const size_t pair_count = waypoints_.size() / 2;
    marker.points.reserve(pair_count);
    for (size_t i = 0; i + 1 < waypoints_.size(); i += 2) {
      geometry_msgs::msg::Point p;
      p.x = waypoints_[i];
      p.y = waypoints_[i + 1];
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_pub_->publish(marker);
  }

  void SelectWaypoint() {
    if (current_target_id_ >= 0 && !ReachedCurrentTarget()) {
      return;
    }

    std::vector<core_behavior_system::WaypointCandidate> candidates;
    candidates.reserve(waypoints_.size() / 2);

    const double forward_x = std::cos(current_yaw_);
    const double forward_y = std::sin(current_yaw_);

    for (size_t i = 0; i + 1 < waypoints_.size(); i += 2) {
      const double wx = waypoints_[i];
      const double wy = waypoints_[i + 1];
      const double dx = wx - current_x_;
      const double dy = wy - current_y_;
      const double dist = std::hypot(dx, dy);
      const double norm = dist > 1e-9 ? dist : 1.0;
      const double dir_x = dx / norm;
      const double dir_y = dy / norm;
      const double dot = forward_x * dir_x + forward_y * dir_y;

      core_behavior_system::WaypointCandidate c;
      c.id = static_cast<int>(i / 2);
      c.path_length = dist;
      c.first_dir_dot = dot;
      c.path_complete = true;
      candidates.push_back(c);
    }

    const double now_sec = now().seconds();
    auto best = selector_.SelectBestWaypoint(current_target_id_, now_sec,
                                             candidates);
    if (best) {
      if (*best != current_target_id_) {
        current_target_id_ = *best;
        selector_.MarkVisited(current_target_id_, now_sec);
        RCLCPP_INFO(get_logger(), "Selected waypoint id: %d",
                    current_target_id_);
      }
    }
  }

  bool ReachedCurrentTarget() const {
    const size_t idx = static_cast<size_t>(current_target_id_) * 2;
    if (idx + 1 >= waypoints_.size()) {
      return true;
    }
    const double dx = waypoints_[idx] - current_x_;
    const double dy = waypoints_[idx + 1] - current_y_;
    const double dist = std::hypot(dx, dy);
    return dist <= arrival_radius_;
  }

  void PublishGoalPose() {
    if (current_target_id_ < 0) {
      return;
    }
    const size_t idx = static_cast<size_t>(current_target_id_) * 2;
    if (idx + 1 >= waypoints_.size()) {
      RCLCPP_WARN(get_logger(), "Selected waypoint id out of range: %d",
                  current_target_id_);
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "map";
    goal.pose.position.x = waypoints_[idx];
    goal.pose.position.y = waypoints_[idx + 1];
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;

    goal_pub_->publish(goal);
  }

  int update_rate_hz_{};
  std::string mode_;
  double risk_threshold_{};
  std::vector<double> weights_;
  std::vector<double> waypoints_;
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  int current_target_id_{-1};
  bool have_odom_{false};
  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::string goal_pose_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  bool paused_{false};
  std::string pause_topic_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  double arrival_radius_{0.5};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  core_behavior_system::WaypointSelectorConfig selector_config_;
  core_behavior_system::WaypointSelector selector_;

  static double YawFromQuat(const geometry_msgs::msg::Quaternion &q) {
    // yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(false);

  auto node = std::make_shared<WaypointSelectorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
