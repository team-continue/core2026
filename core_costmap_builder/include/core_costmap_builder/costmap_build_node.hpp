#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ==============================
// Voxel 用キー（PCLのVoxelGrid相当を自作）
// ==============================
struct VoxelKey
{
  int32_t ix;
  int32_t iy;
  int32_t iz;

  bool operator==(const VoxelKey & o) const {return ix == o.ix && iy == o.iy && iz == o.iz;}
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & k) const noexcept;
};

namespace core_costmap_builder
{

class CostmapBuildNode : public rclcpp::Node
{
public:
  explicit CostmapBuildNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ====== ROS Callbacks / Loop ======
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onGlobal(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onUpdate();

  // ====== TF Utils ======
  bool get2DTranslation(
    const std::string & target, const std::string & source, double & x,
    double & y);
  bool transformPointCloudToFrame(
    const sensor_msgs::msg::PointCloud2 & in,
    const std::string & target_frame,
    sensor_msgs::msg::PointCloud2 & out);

  // ====== PointCloud2 Utils ======
  bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const;

  // ====== Local Grid Utils ======
  bool worldToLocalCell(double wx, double wy, int & ix, int & iy) const;
  inline int lidx(int ix, int iy) const {return iy * size_x_ + ix;}

  void setOccupied(double wx, double wy);
  void raytraceFree(double sx, double sy, double ex, double ey);
  void applyInflation();

  // ====== Publish ======
  void publishLocal();
  void publishFilteredPoints(
    const std::vector<float> & xs, const std::vector<float> & ys,
    const std::vector<float> & zs);
  void publishFused();

private:
  // Occupancy values
  static constexpr int8_t UNKNOWN = -1;
  static constexpr int8_t FREE = 0;
  static constexpr int8_t LETHAL = 100;

  // ====== TF ======
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ====== Sub/Pub ======
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_fused_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_filt_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_in_map_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ====== Topics / Frames ======
  std::string points_in_topic_, global_topic_;
  std::string points_filt_topic_, local_topic_, fused_topic_, local_in_map_topic_;
  std::string map_frame_, odom_frame_, base_frame_, lidar_frame_;

  // ====== Params ======
  double local_width_m_{10.0}, local_height_m_{10.0}, resolution_m_{0.05}, update_hz_{15.0};

  double crop_xy_m_{6.0}, min_z_m_{0.10}, max_z_m_{1.60}, voxel_leaf_m_{0.05};
  double min_range_m_{0.30}, max_range_m_{6.0};

  double robot_radius_m_{0.71}, inflation_radius_m_{0.90};

  bool enable_clearing_{true};
  double points_timeout_sec_{0.2};
  int tf_timeout_ms_{50};

  bool publish_filtered_points_{true};
  bool publish_local_in_map_{true};
  int max_debug_points_{20000};

  // ====== Local Grid ======
  int size_x_{0}, size_y_{0};
  double local_origin_x_{0.0}, local_origin_y_{0.0};
  std::vector<int8_t> local_grid_;

  // ====== Latest Data ======
  sensor_msgs::msg::PointCloud2::SharedPtr last_points_;
  builtin_interfaces::msg::Time last_points_stamp_{};
  nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
};

}  // namespace core_costmap_builder
