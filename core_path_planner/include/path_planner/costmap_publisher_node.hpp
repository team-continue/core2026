#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace path_planner {

class CostmapPublisherNode : public rclcpp::Node {
 public:
  explicit CostmapPublisherNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void publishCostmaps();

  nav_msgs::msg::OccupancyGrid buildGlobalMap() const;
  nav_msgs::msg::OccupancyGrid buildLocalCostmap() const;

  void fillRect(nav_msgs::msg::OccupancyGrid& map, int x0, int y0, int x1,
                int y1, int8_t value) const;

  std::string global_map_topic_;
  std::string local_costmap_topic_;
  std::string global_frame_id_;
  std::string local_frame_id_;

  int global_width_;
  int global_height_;
  double global_resolution_;
  double global_origin_x_;
  double global_origin_y_;

  int local_width_;
  int local_height_;
  double local_resolution_;
  double local_origin_x_;
  double local_origin_y_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace path_planner
