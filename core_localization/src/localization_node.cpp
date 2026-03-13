#include "core_localization/localization_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>

namespace core_localization
{

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("localization_node", options), global_map_(new CloudT), latest_scan_(new CloudT)
{
  declare_params();

  T_map_ci_ = Eigen::Matrix4f::Identity();
  T_map_odom_ = Eigen::Matrix4f::Identity();

  // Set initial guess from parameters
  {
    Eigen::AngleAxisf yaw_rot(static_cast<float>(initial_pose_yaw_), Eigen::Vector3f::UnitZ());
    T_map_ci_.block<3, 3>(0, 0) = yaw_rot.toRotationMatrix();
    T_map_ci_(0, 3) = static_cast<float>(initial_pose_x_);
    T_map_ci_(1, 3) = static_cast<float>(initial_pose_y_);
    T_map_ci_(2, 3) = static_cast<float>(initial_pose_z_);
  }

  // TF
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load global map
  if (!load_global_map()) {
    RCLCPP_ERROR(get_logger(), "Failed to load global PCD map. Node will not function.");
    return;
  }

  // Configure NDT
  ndt_.setResolution(static_cast<float>(ndt_resolution_));
  ndt_.setStepSize(ndt_step_size_);
  ndt_.setMaximumIterations(max_iterations_);
  ndt_.setTransformationEpsilon(transformation_epsilon_);
  ndt_.setInputTarget(global_map_);

  // Configure ICP (fallback)
  icp_.setMaxCorrespondenceDistance(2.0);
  icp_.setMaximumIterations(max_iterations_);
  icp_.setTransformationEpsilon(transformation_epsilon_);
  icp_.setInputTarget(global_map_);

  // Subscribers
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", rclcpp::SensorDataQoS(),
    std::bind(&LocalizationNode::cloud_callback, this, std::placeholders::_1));

  initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1,
    std::bind(&LocalizationNode::initialpose_callback, this, std::placeholders::_1));

  // Publishers
  auto latched_qos = rclcpp::QoS(1).transient_local();
  aligned_cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("/localization/aligned_cloud", 1);
  global_map_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("/localization/global_map", latched_qos);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 1);

  // Service
  relocalize_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/relocalize",
    std::bind(
      &LocalizationNode::relocalize_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Publish global map for RViz
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*global_map_, map_msg);
  map_msg.header.frame_id = map_frame_;
  map_msg.header.stamp = now();
  global_map_pub_->publish(map_msg);

  // Timer for periodic relocalization
  auto period_ms = static_cast<int>(1000.0 / relocalize_rate_);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&LocalizationNode::timer_callback, this));

  // Publish initial identity map->odom TF until first match succeeds
  publish_map_odom_tf(T_map_ci_);

  RCLCPP_INFO(
    get_logger(), "Localization node initialized. method=%s, map_points=%zu, rate=%.1fHz",
    registration_method_.c_str(), global_map_->size(), relocalize_rate_);
}

void LocalizationNode::declare_params()
{
  // Map
  global_map_path_ = declare_parameter("global_map_path", std::string(""));
  map_voxel_size_ = declare_parameter("map_voxel_size", 0.4);
  scan_voxel_size_ = declare_parameter("scan_voxel_size", 0.3);

  // Registration
  registration_method_ = declare_parameter("registration_method", std::string("ndt"));
  ndt_resolution_ = declare_parameter("ndt_resolution", 1.0);
  ndt_step_size_ = declare_parameter("ndt_step_size", 0.1);
  max_iterations_ = declare_parameter("max_iterations", 30);
  transformation_epsilon_ = declare_parameter("transformation_epsilon", 0.01);
  fitness_score_threshold_ = declare_parameter("fitness_score_threshold", 1.0);

  // Timing
  relocalize_rate_ = declare_parameter("relocalize_rate", 1.0);
  min_scan_points_ = declare_parameter("min_scan_points", 100);

  // Smoothing
  smooth_alpha_ = declare_parameter("smooth_alpha", 0.3);

  // Frames
  map_frame_ = declare_parameter("map_frame", std::string("map"));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));

  // Initial pose
  initial_pose_x_ = declare_parameter("initial_pose_x", 0.0);
  initial_pose_y_ = declare_parameter("initial_pose_y", 0.0);
  initial_pose_z_ = declare_parameter("initial_pose_z", 0.0);
  initial_pose_yaw_ = declare_parameter("initial_pose_yaw", 0.0);
}

bool LocalizationNode::load_global_map()
{
  if (global_map_path_.empty()) {
    RCLCPP_ERROR(get_logger(), "global_map_path is empty");
    return false;
  }

  CloudT::Ptr raw_map(new CloudT);
  if (pcl::io::loadPCDFile<PointT>(global_map_path_, *raw_map) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to load PCD file: %s", global_map_path_.c_str());
    return false;
  }
  RCLCPP_INFO(
    get_logger(), "Loaded PCD map: %zu points from %s", raw_map->size(), global_map_path_.c_str());

  // Voxel grid downsample
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(raw_map);
  voxel.setLeafSize(
    static_cast<float>(map_voxel_size_), static_cast<float>(map_voxel_size_),
    static_cast<float>(map_voxel_size_));
  voxel.filter(*global_map_);

  RCLCPP_INFO(
    get_logger(), "Downsampled global map: %zu -> %zu points (voxel=%.2fm)", raw_map->size(),
    global_map_->size(), map_voxel_size_);

  map_loaded_ = true;
  return true;
}

void LocalizationNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  CloudT::Ptr cloud(new CloudT);
  pcl::fromROSMsg(*msg, *cloud);

  if (static_cast<int>(cloud->size()) < min_scan_points_) {
    return;
  }

  // Downsample
  CloudT::Ptr filtered(new CloudT);
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(
    static_cast<float>(scan_voxel_size_), static_cast<float>(scan_voxel_size_),
    static_cast<float>(scan_voxel_size_));
  voxel.filter(*filtered);

  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = filtered;
    scan_received_ = true;
  }
}

void LocalizationNode::initialpose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Set initial guess from RViz 2D Pose Estimate
  // The /initialpose is in map frame
  Eigen::Isometry3d pose_map;
  tf2::fromMsg(msg->pose.pose, pose_map);

  // We need T_map_ci. Get T_odom_ci from TF, then:
  // T_map_ci = T_map_odom * T_odom_ci
  // For the initial guess, assume the user is setting T_map_base.
  // T_map_ci = T_map_base * inv(T_ci_base)
  // But we don't have T_ci_base easily here. Instead, since map≈odom initially:
  // Just use the pose as an approximate T_map_ci initial guess for XY and yaw.
  Eigen::Matrix4f T_init = pose_map.matrix().cast<float>();

  T_map_ci_ = T_init;
  initial_transform_set_ = false;  // Force re-matching with new initial guess

  RCLCPP_INFO(
    get_logger(), "Initial pose set from /initialpose: (%.2f, %.2f, yaw=%.1f deg)", T_init(0, 3),
    T_init(1, 3), std::atan2(T_init(1, 0), T_init(0, 0)) * 180.0 / M_PI);
}

void LocalizationNode::relocalize_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!scan_received_) {
    response->success = false;
    response->message = "No scan data received yet";
    return;
  }

  bool ok = perform_matching();
  response->success = ok;
  response->message = ok ? "Relocalization succeeded" : "Relocalization failed (fitness too high)";
}

void LocalizationNode::timer_callback()
{
  if (!map_loaded_ || !scan_received_) {
    // Keep publishing current estimate even before first match
    publish_map_odom_tf(T_map_ci_);
    return;
  }

  perform_matching();
}

bool LocalizationNode::perform_matching()
{
  CloudT::Ptr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!latest_scan_ || latest_scan_->empty()) {
      return false;
    }
    scan = latest_scan_;
  }

  // Run NDT or ICP
  CloudT::Ptr aligned(new CloudT);
  Eigen::Matrix4f T_result;
  double fitness_score;

  if (registration_method_ == "ndt") {
    ndt_.setInputSource(scan);
    ndt_.align(*aligned, T_map_ci_);
    T_result = ndt_.getFinalTransformation();
    fitness_score = ndt_.getFitnessScore();
  } else {
    icp_.setInputSource(scan);
    icp_.align(*aligned, T_map_ci_);
    T_result = icp_.getFinalTransformation();
    fitness_score = icp_.getFitnessScore();
  }

  bool converged = (registration_method_ == "ndt") ? ndt_.hasConverged() : icp_.hasConverged();

  if (!converged || fitness_score > fitness_score_threshold_) {
    RCLCPP_WARN(
      get_logger(), "Matching failed: converged=%d, fitness=%.3f (threshold=%.3f)", converged,
      fitness_score, fitness_score_threshold_);
    // Keep publishing previous estimate
    publish_map_odom_tf(T_map_ci_);
    return false;
  }

  // Apply exponential smoothing to avoid jumps
  if (initial_transform_set_) {
    T_map_ci_ = smooth_transform(T_map_ci_, T_result, smooth_alpha_);
  } else {
    T_map_ci_ = T_result;
    initial_transform_set_ = true;
  }

  // Publish
  publish_map_odom_tf(T_map_ci_);

  // Publish aligned cloud for debugging
  sensor_msgs::msg::PointCloud2 aligned_msg;
  pcl::toROSMsg(*aligned, aligned_msg);
  aligned_msg.header.frame_id = map_frame_;
  aligned_msg.header.stamp = now();
  aligned_cloud_pub_->publish(aligned_msg);

  RCLCPP_DEBUG(get_logger(), "Match OK: fitness=%.4f, points=%zu", fitness_score, scan->size());

  return true;
}

void LocalizationNode::publish_map_odom_tf(const Eigen::Matrix4f & T_map_ci)
{
  // T_map_odom = T_map_ci * inv(T_odom_ci)
  // T_odom_ci is the static TF published by odom_bridge (odom → camera_init)
  Eigen::Matrix4f T_map_odom;

  try {
    auto tf_odom_ci = tf_buffer_->lookupTransform(odom_frame_, "camera_init", tf2::TimePointZero);

    Eigen::Isometry3d T_odom_ci_d;
    T_odom_ci_d = tf2::transformToEigen(tf_odom_ci);
    Eigen::Matrix4f T_odom_ci = T_odom_ci_d.matrix().cast<float>();

    T_map_odom = T_map_ci * T_odom_ci.inverse();
  } catch (const tf2::TransformException & ex) {
    // odom→camera_init TF not yet available (odom_bridge hasn't started)
    // Fall back: assume camera_init ≈ odom (identity relationship)
    T_map_odom = T_map_ci;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "odom->camera_init TF not available, using T_map_ci as T_map_odom: %s", ex.what());
  }

  T_map_odom_ = T_map_odom;

  // Broadcast map → odom TF
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = odom_frame_;

  Eigen::Isometry3d T_iso(T_map_odom.cast<double>());
  tf_msg.transform = tf2::eigenToTransform(T_iso).transform;

  tf_broadcaster_->sendTransform(tf_msg);

  // Publish pose in map frame
  // T_map_base = T_map_odom * T_odom_base
  try {
    auto tf_odom_base = tf_buffer_->lookupTransform(odom_frame_, "base_link", tf2::TimePointZero);

    Eigen::Isometry3d T_odom_base;
    T_odom_base = tf2::transformToEigen(tf_odom_base);
    Eigen::Isometry3d T_map_base_iso(T_map_odom.cast<double>());
    T_map_base_iso = T_map_base_iso * T_odom_base;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(T_map_base_iso);
    pose_pub_->publish(pose_msg);
  } catch (const tf2::TransformException &) {
    // odom→base_link not yet available, skip pose publish
  }
}

Eigen::Matrix4f LocalizationNode::smooth_transform(
  const Eigen::Matrix4f & T_prev, const Eigen::Matrix4f & T_new, double alpha)
{
  // Linear interpolation for translation
  Eigen::Vector3f t_prev = T_prev.block<3, 1>(0, 3);
  Eigen::Vector3f t_new = T_new.block<3, 1>(0, 3);
  Eigen::Vector3f t_smooth = t_prev + static_cast<float>(alpha) * (t_new - t_prev);

  // SLERP for rotation
  Eigen::Quaternionf q_prev(T_prev.block<3, 3>(0, 0));
  Eigen::Quaternionf q_new(T_new.block<3, 3>(0, 0));
  Eigen::Quaternionf q_smooth = q_prev.slerp(static_cast<float>(alpha), q_new);

  Eigen::Matrix4f T_smooth = Eigen::Matrix4f::Identity();
  T_smooth.block<3, 3>(0, 0) = q_smooth.toRotationMatrix();
  T_smooth.block<3, 1>(0, 3) = t_smooth;
  return T_smooth;
}

}  // namespace core_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<core_localization::LocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
