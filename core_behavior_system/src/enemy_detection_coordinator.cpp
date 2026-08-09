#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <optional>
#include <string>

class EnemyDetectionCoordinator : public rclcpp::Node {
public:
  EnemyDetectionCoordinator() : rclcpp::Node("enemy_detection_coordinator") {
    left_target_topic_ =
        declare_parameter<std::string>("left_target_topic", "/left/target_pose");
    right_target_topic_ =
        declare_parameter<std::string>("right_target_topic", "/right/target_pose");
    enemy_detected_topic_ =
        declare_parameter<std::string>("enemy_detected_topic", "/enemy_detected");
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.5);
    detected_z_threshold_ = declare_parameter<double>("detected_z_threshold", 0.5);

    left_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        left_target_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          last_left_ = *msg;
          last_left_time_ = now();
        });

    right_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        right_target_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          last_right_ = *msg;
          last_right_time_ = now();
        });

    enemy_detected_pub_ =
        create_publisher<std_msgs::msg::Bool>(enemy_detected_topic_, 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               [this]() { onTimer(); });

    RCLCPP_INFO(get_logger(), "enemy_detection_coordinator started");
  }

private:
  void onTimer() {
    const auto now_time = now();
    const bool left_detected =
        last_left_.has_value() &&
        (now_time - last_left_time_).seconds() <= stale_timeout_sec_ &&
        isDetected(*last_left_);
    const bool right_detected =
        last_right_.has_value() &&
        (now_time - last_right_time_).seconds() <= stale_timeout_sec_ &&
        isDetected(*last_right_);

    const bool detected = left_detected || right_detected;
    publishIfChanged(detected);
  }

  bool isDetected(const geometry_msgs::msg::PointStamped &msg) const {
    return msg.point.z < detected_z_threshold_;
  }

  void publishIfChanged(bool detected) {
    if (last_detected_.has_value() && last_detected_.value() == detected) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = detected;
    enemy_detected_pub_->publish(msg);
    last_detected_ = detected;
  }

  // Parameters
  std::string left_target_topic_;
  std::string right_target_topic_;
  std::string enemy_detected_topic_;
  double stale_timeout_sec_{0.5};
  double detected_z_threshold_{0.5};

  // State
  std::optional<geometry_msgs::msg::PointStamped> last_left_;
  std::optional<geometry_msgs::msg::PointStamped> last_right_;
  rclcpp::Time last_left_time_{};
  rclcpp::Time last_right_time_{};
  std::optional<bool> last_detected_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr right_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enemy_detected_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EnemyDetectionCoordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
