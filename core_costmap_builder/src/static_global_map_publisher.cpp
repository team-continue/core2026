#include "core_costmap_builder/static_global_map_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <stdexcept>

namespace core_costmap_builder
{

StaticGlobalMapPublisher::StaticGlobalMapPublisher(const rclcpp::NodeOptions & options)
: Node("static_global_map_publisher", options)
{
  // =========================
  // Parameters
  // =========================
  topic_ = declare_parameter<std::string>("topic", "/map");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  resolution_ = declare_parameter<double>("resolution", 0.05);
  width_m_ = declare_parameter<double>("width_m", 27.6);
  height_m_ = declare_parameter<double>("height_m", 18.3);

  origin_x_ = declare_parameter<double>("origin_x", -13.8);
  origin_y_ = declare_parameter<double>("origin_y", -9.15);
  origin_yaw_ = declare_parameter<double>("origin_yaw", 0.0);

  wall_thickness_m_ = declare_parameter<double>("wall_thickness_m", 0.20);
  publish_times_ = declare_parameter<int>("publish_times", 1);

  if (resolution_ <= 0.0) {
    throw std::runtime_error("resolution must be > 0");
  }
  if (width_m_ <= 0.0 || height_m_ <= 0.0) {
    throw std::runtime_error("width_m/height_m must be > 0");
  }

  width_ = static_cast<int>(std::round(width_m_ / resolution_));
  height_ = static_cast<int>(std::round(height_m_ / resolution_));
  if (width_ <= 0 || height_ <= 0) {
    throw std::runtime_error("invalid grid size");
  }

  // =========================
  // Publisher QoS（late join対応）
  // =========================
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);

  // =========================
  // Build & Publish
  // =========================
  buildMap();

  for (int i = 0; i < std::max(1, publish_times_); ++i) {
    publishOnce();
  }

  RCLCPP_INFO(
    get_logger(),
    "static_global_map_publisher: %dx%d res=%.3f origin=(%.2f,%.2f) wall=%.2fm topic=%s",
    width_, height_, resolution_, origin_x_, origin_y_, wall_thickness_m_,
    topic_.c_str());
}

void StaticGlobalMapPublisher::setOccSafe(int x, int y)
{
  if (x < 0 || y < 0 || x >= width_ || y >= height_) {
    return;
  }
  grid_.data[idx(x, y)] = 100;  // occupied
}

void StaticGlobalMapPublisher::buildMap()
{
  grid_.header.frame_id = frame_id_;
  grid_.info.resolution = static_cast<float>(resolution_);
  grid_.info.width = static_cast<uint32_t>(width_);
  grid_.info.height = static_cast<uint32_t>(height_);

  grid_.info.origin.position.x = origin_x_;
  grid_.info.origin.position.y = origin_y_;
  grid_.info.origin.position.z = 0.0;

  // origin yaw を quaternion に
  tf2::Quaternion q;
  q.setRPY(0, 0, origin_yaw_);
  grid_.info.origin.orientation.x = q.x();
  grid_.info.origin.orientation.y = q.y();
  grid_.info.origin.orientation.z = q.z();
  grid_.info.origin.orientation.w = q.w();

  // 外周だけ壁にしたいので、内部は free(0) で埋める
  grid_.data.assign(width_ * height_, 0);

  // 壁厚をセル数に変換
  int t = static_cast<int>(std::ceil(wall_thickness_m_ / resolution_));
  t = std::max(1, t);

  // 下端・上端
  for (int y = 0; y < t; ++y) {
    for (int x = 0; x < width_; ++x) {
      setOccSafe(x, y);
    }
  }
  for (int y = height_ - t; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      setOccSafe(x, y);
    }
  }

  // 左端・右端
  for (int x = 0; x < t; ++x) {
    for (int y = 0; y < height_; ++y) {
      setOccSafe(x, y);
    }
  }
  for (int x = width_ - t; x < width_; ++x) {
    for (int y = 0; y < height_; ++y) {
      setOccSafe(x, y);
    }
  }
}

void StaticGlobalMapPublisher::publishOnce()
{
  grid_.header.stamp = now();
  pub_->publish(grid_);
}

}  // namespace core_costmap_builder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<core_costmap_builder::StaticGlobalMapPublisher>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
