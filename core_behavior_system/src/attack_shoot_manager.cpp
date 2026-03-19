#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <cmath>
#include <optional>
#include <string>

class AttackShootManager : public rclcpp::Node {
public:
  AttackShootManager() : rclcpp::Node("attack_shoot_manager") {
    state_topic_ = declare_parameter<std::string>("state_topic", "/behavior_system/state");
    left_target_topic_ =
        declare_parameter<std::string>("left_target_topic", "/left/damage_panel_pose");
    right_target_topic_ =
        declare_parameter<std::string>("right_target_topic", "/right/damage_panel_pose");
    left_shoot_once_topic_ =
        declare_parameter<std::string>("left_shoot_once_topic", "/left_shoot_once");
    right_shoot_once_topic_ =
        declare_parameter<std::string>("right_shoot_once_topic", "/right_shoot_once");

    image_width_ = declare_parameter<double>("image_width", 1280.0);
    image_height_ = declare_parameter<double>("image_height", 720.0);
    image_center_x_ = declare_parameter<double>("image_center_x", 0.5);
    image_center_y_ = declare_parameter<double>("image_center_y", 0.5);
    center_tolerance_x_px_ =
        declare_parameter<double>("center_tolerance_x_px", 20.0);
    center_tolerance_y_px_ =
        declare_parameter<double>("center_tolerance_y_px", 20.0);

    detected_z_threshold_ = declare_parameter<double>("detected_z_threshold", 0.5);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.2);
    shoot_cooldown_sec_ = declare_parameter<double>("shoot_cooldown_sec", 0.5);
    attack_state_value_ = declare_parameter<int>("attack_state_value", 1);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    state_sub_ = create_subscription<std_msgs::msg::Int32>(
        state_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Int32::SharedPtr msg) { current_state_ = msg->data; });

    left_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        left_target_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          last_left_target_ = *msg;
          last_left_target_time_ = now();
        });

    right_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        right_target_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          last_right_target_ = *msg;
          last_right_target_time_ = now();
        });

    left_shoot_once_pub_ =
        create_publisher<std_msgs::msg::Bool>(left_shoot_once_topic_, 10);
    right_shoot_once_pub_ =
        create_publisher<std_msgs::msg::Bool>(right_shoot_once_topic_, 10);

    const double safe_rate = publish_rate_hz_ > 0.0 ? publish_rate_hz_ : 10.0;
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / safe_rate)),
        [this]() { onTimer(); });

    RCLCPP_INFO(get_logger(), "attack_shoot_manager started");
  }

private:
  void onTimer() {
    if (!current_state_.has_value() || current_state_.value() != attack_state_value_) {
      return;
    }

    const auto now_time = now();
    if (shouldFire(last_left_target_, last_left_target_time_, now_time, last_left_fire_time_)) {
      publishShootOnce(left_shoot_once_pub_);
      last_left_fire_time_ = now_time;
    }

    if (shouldFire(last_right_target_, last_right_target_time_, now_time, last_right_fire_time_)) {
      publishShootOnce(right_shoot_once_pub_);
      last_right_fire_time_ = now_time;
    }
  }

  bool shouldFire(const std::optional<geometry_msgs::msg::PointStamped> &target,
                  const rclcpp::Time &target_time, const rclcpp::Time &now_time,
                  const rclcpp::Time &last_fire_time) const {
    if (!target.has_value()) {
      return false;
    }

    if ((now_time - target_time).seconds() > stale_timeout_sec_) {
      return false;
    }

    if ((now_time - last_fire_time).seconds() < shoot_cooldown_sec_) {
      return false;
    }

    if (target->point.z >= detected_z_threshold_) {
      return false;
    }

    if (!isNearCenter(*target)) {
      return false;
    }

    return true;
  }

  bool isNearCenter(const geometry_msgs::msg::PointStamped &target) const {
    if (image_width_ <= 0.0 || image_height_ <= 0.0) {
      return false;
    }

    const double center_x_px = (image_center_x_ - 0.5) * image_width_;
    const double center_y_px = (image_center_y_ - 0.5) * image_height_;
    const double dx = target.point.x - center_x_px;
    const double dy = target.point.y - center_y_px;

    return std::fabs(dx) <= center_tolerance_x_px_ && std::fabs(dy) <= center_tolerance_y_px_;
  }

  void publishShootOnce(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    pub->publish(msg);
  }

  // Parameters
  std::string state_topic_;
  std::string left_target_topic_;
  std::string right_target_topic_;
  std::string left_shoot_once_topic_;
  std::string right_shoot_once_topic_;
  double image_width_{1280.0};
  double image_height_{720.0};
  double image_center_x_{0.5};
  double image_center_y_{0.5};
  double center_tolerance_x_px_{20.0};
  double center_tolerance_y_px_{20.0};
  double detected_z_threshold_{0.5};
  double stale_timeout_sec_{0.2};
  double shoot_cooldown_sec_{0.5};
  double publish_rate_hz_{20.0};
  int attack_state_value_{1};

  // State
  std::optional<int> current_state_;
  std::optional<geometry_msgs::msg::PointStamped> last_left_target_;
  std::optional<geometry_msgs::msg::PointStamped> last_right_target_;
  rclcpp::Time last_left_target_time_{};
  rclcpp::Time last_right_target_time_{};
  rclcpp::Time last_left_fire_time_{};
  rclcpp::Time last_right_fire_time_{};

  // ROS
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr left_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr right_target_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_shoot_once_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_shoot_once_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttackShootManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
