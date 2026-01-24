#pragma once

#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
namespace core_costmap_builder
{

// 外周だけの矩形フィールドマップを生成して /map に publish するノード
class StaticGlobalMapPublisher : public rclcpp::Node
{
public:
  explicit StaticGlobalMapPublisher(const rclcpp::NodeOptions & options);

private:
  // ----- helpers -----
  inline int idx(int x, int y) const {return y * width_ + x;}
  void setOccSafe(int x, int y);
  void buildMap();
  void publishOnce();

  // ----- parameters -----
  std::string topic_;
  std::string frame_id_;

  double resolution_{0.05};
  double width_m_{27.6};
  double height_m_{18.3};

  // OccupancyGrid の origin（左下の座標）
  double origin_x_{-13.8};
  double origin_y_{-9.15};
  double origin_yaw_{0.0};

  // 外周壁の太さ（m）
  double wall_thickness_m_{0.20};

  // 起動時 publish 回数（transient_localなら基本1でOK）
  int publish_times_{1};

  // ----- derived -----
  int width_{552};  // cells
  int height_{366};  // cells

  // ----- ROS interfaces -----
  nav_msgs::msg::OccupancyGrid grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

}  // namespace core_costmap_builder
