#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <optional>
#include <string>

class AttackShootManager : public rclcpp::Node {
public:
  AttackShootManager() : rclcpp::Node("attack_shoot_manager") {
    //---------------------------------
    // Parameter
    //---------------------------------
    left_target_topic_ =
        declare_parameter<std::string>("left_target_topic", "/left/target_pose");
    right_target_topic_ =
        declare_parameter<std::string>("right_target_topic", "/right/target_pose");
    left_shoot_fullauto_topic_ =
        declare_parameter<std::string>("left_shoot_fullauto_topic", "/left/shoot_fullauto");
    right_shoot_fullauto_topic_ =
        declare_parameter<std::string>("right_shoot_fullauto_topic", "/right/shoot_fullauto");
    left_turret_auto_topic_ =
        declare_parameter<std::string>("left_turret_auto_topic", "/left/turret_auto");
    right_turret_auto_topic_ =
        declare_parameter<std::string>("right_turret_auto_topic", "/right/turret_auto");

    image_width_ = declare_parameter<double>("image_width", 1280.0);
    image_height_ = declare_parameter<double>("image_height", 720.0);
    image_center_x_ = declare_parameter<double>("image_center_x", 0.5);
    image_center_y_ = declare_parameter<double>("image_center_y", 0.5);
    center_tolerance_x_px_ = declare_parameter<double>("center_tolerance_x_px", 20.0);
    center_tolerance_y_px_ = declare_parameter<double>("center_tolerance_y_px", 20.0);

    detected_z_threshold_ = declare_parameter<double>("detected_z_threshold", 0.5);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.2);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    //---------------------------------
    // Subscriber
    //---------------------------------
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

    left_turret_auto_sub_ = create_subscription<std_msgs::msg::Bool>(
        left_turret_auto_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          left_turret_auto_ = msg->data;
        });
    right_turret_auto_sub_ = create_subscription<std_msgs::msg::Bool>(
        right_turret_auto_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          right_turret_auto_ = msg->data;
        });

    left_shoot_fullauto_pub_ =
        create_publisher<std_msgs::msg::Bool>(left_shoot_fullauto_topic_, 10);
    right_shoot_fullauto_pub_ =
        create_publisher<std_msgs::msg::Bool>(right_shoot_fullauto_topic_, 10);

    const double safe_rate = publish_rate_hz_ > 0.0 ? publish_rate_hz_ : 10.0;
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / safe_rate)),
        [this]() { onTimer(); });

    RCLCPP_INFO(get_logger(), "attack_shoot_manager started");
  }

private:
  void onTimer() {
    const auto now_time = now();
    std_msgs::msg::Bool right_msg;
    std_msgs::msg::Bool left_msg;

    const bool left_auto = left_turret_auto_;
    const bool right_auto = right_turret_auto_;
    left_msg.data = shouldFire(
        left_auto, last_left_target_, last_left_target_time_, now_time);
    right_msg.data = shouldFire(
        right_auto, last_right_target_, last_right_target_time_, now_time);

    left_shoot_fullauto_pub_->publish(left_msg);
    right_shoot_fullauto_pub_->publish(right_msg);
  }

  bool shouldFire(bool turret_auto,
                  const std::optional<geometry_msgs::msg::PointStamped> &target,
                  const rclcpp::Time &target_time, const rclcpp::Time &now_time) const {
    if (!turret_auto) {
      return false;
    }

    if (!target.has_value()) {
      return false;
    }

    if ((now_time - target_time).seconds() > stale_timeout_sec_) {
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

    return std::fabs(dx) <= center_tolerance_x_px_ &&
           std::fabs(dy) <= center_tolerance_y_px_;
  }

  // Parameters
  std::string left_target_topic_;
  std::string right_target_topic_;
  std::string left_shoot_fullauto_topic_;
  std::string right_shoot_fullauto_topic_;
  double image_width_{1280.0};
  double image_height_{720.0};
  double image_center_x_{0.5};
  double image_center_y_{0.5};
  double center_tolerance_x_px_{20.0};
  double center_tolerance_y_px_{20.0};
  double detected_z_threshold_{0.5};
  double stale_timeout_sec_{0.2};
  double publish_rate_hz_{20.0};
  std::string left_turret_auto_topic_;
  std::string right_turret_auto_topic_;

  // State
  std::optional<geometry_msgs::msg::PointStamped> last_left_target_;
  std::optional<geometry_msgs::msg::PointStamped> last_right_target_;
  rclcpp::Time last_left_target_time_{};
  rclcpp::Time last_right_target_time_{};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr left_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr right_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_turret_auto_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_turret_auto_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_shoot_fullauto_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_shoot_fullauto_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool left_turret_auto_{false};
  bool right_turret_auto_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttackShootManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
