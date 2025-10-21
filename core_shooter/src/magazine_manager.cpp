#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include <deque>
#include <cmath>

class MagazineManager : public rclcpp::Node
{
public:
  MagazineManager()
  : Node("magazine_manager")
  {
    //========================================
    // parameters
    //========================================
    declare_parameter("remaining_disks", 27);
    declare_parameter("disk_thickness", 1.0);
    declare_parameter("sensor_height", 100.0);
    declare_parameter("window_size", 3);

    remaining_disks_ = this->get_parameter("remaining_disks").as_int();
    disk_thickness_ = this->get_parameter("disk_thickness").as_double();
    sensor_height_ = this->get_parameter("sensor_height").as_double();
    window_size_ = this->get_parameter("window_size").as_int();

    RCLCPP_INFO(
      this->get_logger(), "remaining_disks: %d, disk_thickness: %f, sensor_height: %f", remaining_disks_, disk_thickness_,
      sensor_height_);

    //========================================
    // subscribers ui
    //========================================
    shoot_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_status", 10,
      std::bind(&MagazineManager::shootStatusCallback, this, std::placeholders::_1));

    reloading_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "reloading", 10,
      std::bind(&MagazineManager::reloadingCallback, this, std::placeholders::_1));

    disk_distance_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "disk_distance_sensor", 10,
      std::bind(&MagazineManager::diskDistanceSensorCallback, this, std::placeholders::_1));

    //========================================
    // publishers
    //========================================
    remaining_disk_pub_ = this->create_publisher<std_msgs::msg::Int8>("remaining_disk", 10);

    //========================================
    // initialize
    //========================================
    remainingDiskEstimator();
  }

private:
  //========================================
  // callbacks
  //========================================
  void shootStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      remainingDiskEstimator();
    }
  }

  void reloadingCallback(std_msgs::msg::Int8::SharedPtr msg)
  {
    if (msg->data > 0) {
      remainingDiskEstimator();
    }
  }

  void diskDistanceSensorCallback(std_msgs::msg::Int32::SharedPtr msg)
  {
    double val = static_cast<double>(msg->data);

    // 移動平均フィルタ
    buffer_.push_back(val);
    if (buffer_.size() > (size_t)window_size_) {
      buffer_.pop_front();
    }

    double sum = 0.0;
    for (double v : buffer_) {sum += v;}
    distance_ = sum / buffer_.size();
  }

  //========================================
  // remaining disk estimator
  //========================================
  void remainingDiskEstimator()
  {
    double height = sensor_height_ - distance_;

    int estimated_disks = std::round(height / disk_thickness_);
    if (estimated_disks < 0) {
      estimated_disks = 0;
    }

    remainingDisksPublish(estimated_disks);
  }

  void remainingDisksPublish(int data)
  {
    std_msgs::msg::Int8 message;
    message.data = data;
    remaining_disk_pub_->publish(message);
  }


  //========================================
  // Subscription valids
  //========================================
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr reloading_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr disk_distance_sensor_sub_;

  //========================================
  // publisher valids
  //========================================
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr remaining_disk_pub_;

  //========================================
  // subscription valids
  //========================================
  double distance_ = 0.0;

  //========================================
  // parameter valids
  //========================================
  int remaining_disks_;
  double disk_thickness_;
  double sensor_height_;
  int window_size_;

  //========================================
  // valids
  //========================================
  std::deque<double> buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagazineManager>());
  rclcpp::shutdown();
  return 0;
}
