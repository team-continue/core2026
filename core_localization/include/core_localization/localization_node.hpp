#ifndef CORE_LOCALIZATION__LOCALIZATION_NODE_HPP_
#define CORE_LOCALIZATION__LOCALIZATION_NODE_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace core_localization
{

class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  // Initialization
  void declare_params();
  bool load_global_map();

  // Callbacks
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void relocalize_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void timer_callback();

  // Core logic
  bool perform_matching();
  void publish_map_odom_tf(const Eigen::Matrix4f & T_map_ci);
  Eigen::Matrix4f smooth_transform(
    const Eigen::Matrix4f & T_prev, const Eigen::Matrix4f & T_new, double alpha);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Point clouds
  CloudT::Ptr global_map_;
  CloudT::Ptr latest_scan_;
  std::mutex scan_mutex_;

  // NDT / ICP
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  pcl::IterativeClosestPoint<PointT, PointT> icp_;
  std::string registration_method_;

  // State
  Eigen::Matrix4f T_map_ci_;    // NDT result: camera_init → map
  Eigen::Matrix4f T_map_odom_;  // Current map→odom correction
  std::atomic<bool> scan_received_{false};
  bool initial_transform_set_{false};
  bool map_loaded_{false};

  // Parameters
  std::string global_map_path_;
  double map_voxel_size_;
  double scan_voxel_size_;
  double ndt_resolution_;
  double ndt_step_size_;
  int max_iterations_;
  double transformation_epsilon_;
  double fitness_score_threshold_;
  double relocalize_rate_;
  int min_scan_points_;
  double smooth_alpha_;
  std::string map_frame_;
  std::string odom_frame_;
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_yaw_;
};

}  // namespace core_localization

#endif  // CORE_LOCALIZATION__LOCALIZATION_NODE_HPP_
