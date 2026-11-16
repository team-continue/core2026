// 生点群（既定は /livox/lidar、input_topic で変える）から他ロボットと人間を検出して追跡するノード。
//
// 点群は LIO の姿勢が届いてから処理する。1 フレームの点は時刻が違うので、前後の姿勢の
// 補間で自機の動きによる歪みを補正する。シミュレーションでは全点が同時刻なので補正されない。
// 2 台の LiDAR を lidar_merge で合わせた点群（1 台目のフレーム）を読むときは、2 台目のフレームを
// additional_sensor_frames に渡す。クラスタごとに最も近い LiDAR から見た向きで、見えている面の奥の中心を求める。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <core_msgs/msg/process_stats.hpp>
#include <core_msgs/msg/tracked_object.hpp>
#include <core_msgs/msg/tracked_object_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "core_nav_common/parameter_updates.hpp"
#include "core_nav_common/parameters.hpp"
#include "core_perception/detection.hpp"
#include "core_perception/pose_timing.hpp"
#include "core_perception/static_map.hpp"
#include "core_perception/track_frame.hpp"
#include "core_perception/tracker.hpp"

namespace core_perception
{
namespace
{
using core_msgs::msg::ProcessStats;
using core_msgs::msg::TrackedObject;
using core_msgs::msg::TrackedObjectArray;
using livox_ros_driver2::msg::CustomMsg;
using nav_msgs::msg::Odometry;
using visualization_msgs::msg::Marker;

constexpr std::size_t kMaxQueuedClouds = 3U;

struct StampedPose
{
  std::uint64_t stamp_ns{0U};
  Eigen::Isometry3f pose{Eigen::Isometry3f::Identity()};
};

Eigen::Isometry3f toIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d result;
  tf2::fromMsg(pose, result);
  return result.cast<float>();
}

std::uint64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

Eigen::Isometry3f interpolate(
  const StampedPose & before, const StampedPose & after, const std::uint64_t stamp_ns)
{
  if (after.stamp_ns <= before.stamp_ns || stamp_ns >= after.stamp_ns) {
    return after.pose;
  }
  if (stamp_ns <= before.stamp_ns) {
    return before.pose;
  }
  const float ratio = static_cast<float>(stamp_ns - before.stamp_ns) /
    static_cast<float>(after.stamp_ns - before.stamp_ns);
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.translation() = before.pose.translation() +
    ratio * (after.pose.translation() - before.pose.translation());
  pose.linear() = Eigen::Quaternionf(before.pose.rotation()).slerp(
    ratio, Eigen::Quaternionf(after.pose.rotation())).toRotationMatrix();
  return pose;
}

std::vector<FramePoint> toFramePoints(
  const CustomMsg & cloud, const StampedPose & before, const StampedPose & after)
{
  std::vector<FramePoint> points;
  points.reserve(cloud.points.size());
  // 全点が同時刻のときに補間を繰り返さない
  std::uint32_t cached_offset = 0U;
  Eigen::Isometry3f cached_pose = interpolate(before, after, cloud.timebase);
  for (const auto & point : cloud.points) {
    if (point.offset_time != cached_offset) {
      cached_offset = point.offset_time;
      cached_pose = interpolate(before, after, cloud.timebase + point.offset_time);
    }
    points.push_back(FramePoint{Eigen::Vector3f(point.x, point.y, point.z), cached_pose});
  }
  return points;
}
geometry_msgs::msg::Point makePoint(const Eigen::Vector2f & xy, const float z)
{
  geometry_msgs::msg::Point point;
  point.x = xy.x();
  point.y = xy.y();
  point.z = z;
  return point;
}

// 計測した箱の 12 本の辺を、LINE_LIST の点の組として足す。高さは床から上面まで
void appendBoxEdges(const Detection & detection, std::vector<geometry_msgs::msg::Point> & points)
{
  const Eigen::Vector2f axis(std::cos(detection.box_yaw), std::sin(detection.box_yaw));
  const Eigen::Vector2f normal(-axis.y(), axis.x());
  const Eigen::Vector2f half_depth = 0.5F * detection.depth * axis;
  const Eigen::Vector2f half_width = 0.5F * detection.width * normal;
  const std::array<Eigen::Vector2f, 4> corners{
    detection.box_center - half_depth - half_width, detection.box_center + half_depth - half_width,
    detection.box_center + half_depth + half_width, detection.box_center - half_depth + half_width};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const Eigen::Vector2f & a = corners[i];
    const Eigen::Vector2f & b = corners[(i + 1U) % corners.size()];
    for (const float z : {0.0F, detection.height}) {
      points.push_back(makePoint(a, z));
      points.push_back(makePoint(b, z));
    }
    points.push_back(makePoint(a, 0.0F));
    points.push_back(makePoint(a, detection.height));
  }
}
}  // namespace

class PerceptionNode final : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options)
  : Node("perception_node", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareDouble;
    using core_nav_common::declareInt;
    using core_nav_common::declareString;

    declareString(*this, "map_path", "", "地図の PCD", true);
    declareString(*this, "input_topic", "/sensing/livox/lidar", "入力の点群（Livox の CustomMsg）", true);
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description =
        "入力に点を合わせたほかの LiDAR のフレーム（lidar_merge の 2 台目）。空なら入力のフレームの 1 台だけ";
      descriptor.read_only = true;
      declare_parameter<std::vector<std::string>>(
        "additional_sensor_frames", std::vector<std::string>{}, descriptor);
    }
    declareString(*this, "base_frame", "base_link", "機体のフレーム", true);
    declareString(*this, "odom_frame", "odom", "追跡のフレーム", true);
    declareString(*this, "map_frame", "map", "地図のフレーム", true);
    declareDouble(*this, "map_leaf_size_m", 0.2, 0.05, 2.0, "地図のボクセルの大きさ", true);

    declareDouble(*this, "field.x_min", -12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.x_max", 12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_min", -7.5, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_max", 7.5, -1000.0, 1000.0, "フィールドの範囲", true);

    declareDouble(*this, "footprint.length_m", 0.8, 0.1, 5.0, "機体の前後の長さ");
    declareDouble(*this, "footprint.width_m", 0.8, 0.1, 5.0, "機体の左右の幅");
    declareDouble(*this, "footprint.height_m", 0.8, 0.1, 5.0, "機体の上面の床からの高さ（自機の除去に使う）");
    // FAST-LIO の前処理と同じ箱にするため、機体の形と同じ footprint.yaml に置く
    declareDouble(*this, "footprint.self_filter_margin_m", 0.05, 0.0, 1.0, "自機の除去の余裕");
    declareString(
      *this, "footprint.self_filter_shape", "box",
      "自機の除去の形（box: base_link の向きの箱、cylinder: 外接円の円柱）");
    declareDouble(
      *this, "min_range_m", 0.1, 0.0, 5.0, "LiDAR からこの距離未満の点（返りのない (0,0,0) の点を含む）を除く");

    declareDouble(*this, "ground_height_m", 0.12, 0.0, 1.0, "これ以下の高さを地面として除く");
    declareDouble(*this, "max_height_m", 2.2, 0.5, 10.0, "これより高い点を除く");
    declareDouble(*this, "cluster.cell_size_m", 0.2, 0.05, 1.0, "クラスタリングの格子の大きさ");
    declareInt(*this, "cluster.min_points", 3, 1, 1000, "クラスタの最小点数");
    declareDouble(*this, "cluster.min_height_m", 0.25, 0.0, 2.0, "クラスタの最小の高さ");
    declareDouble(*this, "cluster.wide_connect_range_m", 0.0, 0.0, 100.0, "セルを広くつなげる距離");
    declareDouble(*this, "cluster.merge_gap_m", 0.7, 0.0, 5.0, "同じ相手としてまとめる隙間");
    declareDouble(*this, "human_size_m", 0.5, 0.1, 2.0, "人の大きさ");
    declareDouble(*this, "human_center_min_height_m", 1.2, 0.1, 3.0, "人の大きさを使う高さ");

    declareBool(*this, "outlier_filter.enabled", true, "孤立した点を除く");
    declareDouble(*this, "outlier_filter.radius_m", 0.2, 0.01, 5.0, "近傍を数える半径");
    declareInt(*this, "outlier_filter.min_neighbors", 2, 0, 1000, "必要な近傍の数");

    declareBool(*this, "cluster_split.enabled", true, "接近したクラスタを分ける");
    declareDouble(*this, "cluster_split.max_cluster_size_m", 1.2, 0.1, 20.0, "分ける大きさ");
    declareDouble(*this, "cluster_split.expected_robot_size_m", 0.8, 0.1, 5.0, "ロボットの大きさ");

    declareDouble(*this, "tracking.max_coast_s", 1.0, 0.0, 30.0, "観測がなくても追跡を続ける時間");
    declareDouble(*this, "tracking.coast_sigma_growth_mps", 1.0, 0.0, 10.0, "隠れている間の不確かさの増え方");
    declareInt(*this, "tracking.min_hits", 2, 1, 100, "追跡を確定する観測回数");
    declareDouble(*this, "tracking.association_gate_m", 1.5, 0.1, 10.0, "観測と対応させる距離");
    declareDouble(*this, "tracking.accel_sigma_mps2", 3.0, 0.01, 50.0, "相手の加速度の標準偏差");
    declareDouble(*this, "tracking.measurement_sigma_m", 0.2, 0.01, 5.0, "観測の標準偏差");
    declareDouble(*this, "tracking.initial_velocity_sigma_mps", 3.0, 0.01, 20.0, "追跡開始時の速度の標準偏差");
    declareDouble(*this, "tracking.max_speed_mps", 4.0, 0.1, 30.0, "相手の速度の上限");
    declareDouble(
      *this, "tracking.velocity_settled_sigma_mps", 0.5, 0.0, 10.0,
      "速度が定まったとみなす速度の標準偏差");
    declareBool(*this, "tracking.handover.enabled", true, "消えた追跡の速度と ID を引き継ぐ");
    declareDouble(*this, "tracking.handover.max_gap_s", 0.5, 0.0, 10.0, "引き継ぐ時間の門");
    declareDouble(*this, "tracking.handover.max_distance_m", 1.5, 0.0, 10.0, "引き継ぐ距離の門");
    declareDouble(
      *this, "tracking.handover.extra_velocity_sigma_mps", 0.5, 0.0, 10.0,
      "引き継いだ速度の標準偏差に足す値");

    declareDouble(*this, "classification.robot_max_height_m", 1.0, 0.1, 5.0, "ロボットの高さの上限");
    declareDouble(*this, "classification.robot_min_width_m", 0.2, 0.0, 5.0, "ロボットの幅の下限");
    declareDouble(*this, "classification.robot_max_width_m", 1.3, 0.1, 5.0, "ロボットの幅の上限");
    declareDouble(*this, "classification.human_min_height_m", 1.4, 0.1, 5.0, "人の高さの下限");
    declareDouble(*this, "classification.human_max_width_m", 0.7, 0.1, 5.0, "人の幅の上限");

    loadParams();
    parameter_callback_ = core_nav_common::onParametersSet(
      *this, [this](const std::vector<rclcpp::Parameter> &) {loadParams();});

    FieldBounds field;
    field.x_min = get_parameter("field.x_min").as_double();
    field.x_max = get_parameter("field.x_max").as_double();
    field.y_min = get_parameter("field.y_min").as_double();
    field.y_max = get_parameter("field.y_max").as_double();
    const std::string map_path = get_parameter("map_path").as_string();
    if (map_path.empty()) {
      RCLCPP_WARN(get_logger(), "地図がないので、固定物の除去をしません");
      static_map_.setField(field);
    } else {
      static_map_ = StaticMap::load(
        map_path, field, get_parameter("map_leaf_size_m").as_double(),
        detection_params_.height.ground_height_m, detection_params_.height.max_height_m);
      RCLCPP_INFO(get_logger(), "フィールド内の固定物のボクセル: %zu", static_map_.size());
    }

    tracks_publisher_ = create_publisher<TrackedObjectArray>("/perception/tracks", rclcpp::QoS(10));
    // 同じ追跡結果を地図の座標に直したもの（操縦の UI が地図に重ねて描く）
    map_tracks_publisher_ =
      create_publisher<TrackedObjectArray>("/perception/tracks_map", rclcpp::QoS(10));
    stats_publisher_ = create_publisher<ProcessStats>("/perception/stats", rclcpp::QoS(10));
    detections_publisher_ = create_publisher<Marker>("/perception/detections", rclcpp::QoS(10));
    cloud_subscription_ = create_subscription<CustomMsg>(
      get_parameter("input_topic").as_string(), rclcpp::QoS(2).reliable(),
      [this](const CustomMsg::ConstSharedPtr & cloud) {
        clouds_.push_back(cloud);
        if (clouds_.size() > kMaxQueuedClouds) {
          clouds_.pop_front();
          ++skipped_frames_;
        }
        processReadyClouds();
      });
    odometry_subscription_ = create_subscription<Odometry>(
      "/localization/lio/odom", rclcpp::QoS(10),
      [this](const Odometry::ConstSharedPtr & odometry) {
        previous_pose_ = latest_pose_;
        latest_pose_ = StampedPose{
          toNanoseconds(odometry->header.stamp), toIsometry(odometry->pose.pose)};
        processReadyClouds();
      });
  }

private:
  void loadParams()
  {
    DetectionParams detection;
    const std::string self_shape = get_parameter("footprint.self_filter_shape").as_string();
    if (self_shape != "box" && self_shape != "cylinder") {
      throw std::invalid_argument(
              "footprint.self_filter_shape は box か cylinder にしてください: " + self_shape);
    }
    const bool cylinder = self_shape == "cylinder";
    detection.self = (cylinder ? cylinderSelfFilterFromFootprint : selfFilterFromFootprint)(
      get_parameter("footprint.length_m").as_double(),
      get_parameter("footprint.width_m").as_double(),
      get_parameter("footprint.height_m").as_double(),
      get_parameter("footprint.self_filter_margin_m").as_double());
    detection.self.min_range_m = get_parameter("min_range_m").as_double();
    detection.height.ground_height_m = get_parameter("ground_height_m").as_double();
    detection.height.max_height_m = get_parameter("max_height_m").as_double();
    detection.cluster.cell_size_m = get_parameter("cluster.cell_size_m").as_double();
    detection.cluster.min_points =
      static_cast<std::size_t>(get_parameter("cluster.min_points").as_int());
    detection.cluster.min_height_m = get_parameter("cluster.min_height_m").as_double();
    detection.cluster.wide_connect_range_m =
      get_parameter("cluster.wide_connect_range_m").as_double();
    detection.cluster.merge_gap_m = get_parameter("cluster.merge_gap_m").as_double();
    detection.human_size_m = get_parameter("human_size_m").as_double();
    detection.human_min_height_m = get_parameter("human_center_min_height_m").as_double();
    detection.outlier.enabled = get_parameter("outlier_filter.enabled").as_bool();
    detection.outlier.radius_m = get_parameter("outlier_filter.radius_m").as_double();
    detection.outlier.min_neighbors =
      static_cast<std::size_t>(get_parameter("outlier_filter.min_neighbors").as_int());
    detection.split.enabled = get_parameter("cluster_split.enabled").as_bool();
    detection.split.max_cluster_size_m =
      get_parameter("cluster_split.max_cluster_size_m").as_double();
    detection.split.expected_robot_size_m =
      get_parameter("cluster_split.expected_robot_size_m").as_double();
    detection_params_ = detection;

    TrackingParams tracking;
    tracking.max_coast_s = get_parameter("tracking.max_coast_s").as_double();
    tracking.coast_sigma_growth_mps = get_parameter("tracking.coast_sigma_growth_mps").as_double();
    tracking.min_hits = static_cast<std::size_t>(get_parameter("tracking.min_hits").as_int());
    tracking.association_gate_m = get_parameter("tracking.association_gate_m").as_double();
    tracking.accel_sigma_mps2 = get_parameter("tracking.accel_sigma_mps2").as_double();
    tracking.measurement_sigma_m = get_parameter("tracking.measurement_sigma_m").as_double();
    tracking.initial_velocity_sigma_mps =
      get_parameter("tracking.initial_velocity_sigma_mps").as_double();
    tracking.max_speed_mps = get_parameter("tracking.max_speed_mps").as_double();
    tracking.velocity_settled_sigma_mps =
      get_parameter("tracking.velocity_settled_sigma_mps").as_double();
    tracking.handover.enabled = get_parameter("tracking.handover.enabled").as_bool();
    tracking.handover.max_gap_s = get_parameter("tracking.handover.max_gap_s").as_double();
    tracking.handover.max_distance_m =
      get_parameter("tracking.handover.max_distance_m").as_double();
    tracking.handover.extra_velocity_sigma_mps =
      get_parameter("tracking.handover.extra_velocity_sigma_mps").as_double();
    tracking_params_ = tracking;

    ClassificationParams classification;
    classification.robot_max_height_m =
      get_parameter("classification.robot_max_height_m").as_double();
    classification.robot_min_width_m =
      get_parameter("classification.robot_min_width_m").as_double();
    classification.robot_max_width_m =
      get_parameter("classification.robot_max_width_m").as_double();
    classification.human_min_height_m =
      get_parameter("classification.human_min_height_m").as_double();
    classification.human_max_width_m =
      get_parameter("classification.human_max_width_m").as_double();
    classification_params_ = classification;
  }

  std::optional<Eigen::Isometry3f> lookup(const std::string & target, const std::string & source)
  {
    try {
      return tf2::transformToEigen(
        tf_buffer_.lookupTransform(target, source, tf2::TimePointZero)).cast<float>();
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s から %s への変換を取得できません: %s",
        target.c_str(), source.c_str(), error.what());
      return std::nullopt;
    }
  }

  // additional_sensor_frames の取り付け。1 つでも取れなければ nullopt（次の点群で取り直す）
  std::optional<std::vector<Eigen::Isometry3f>> lookupAdditionalSensors()
  {
    std::vector<Eigen::Isometry3f> transforms;
    const std::vector<std::string> frames =
      get_parameter("additional_sensor_frames").as_string_array();
    for (const auto & frame : frames) {
      const auto transform = lookup(get_parameter("base_frame").as_string(), frame);
      if (!transform.has_value()) {
        return std::nullopt;
      }
      transforms.push_back(*transform);
    }
    return transforms;
  }

  void processReadyClouds()
  {
    if (!latest_pose_.has_value()) {
      return;
    }
    while (!clouds_.empty()) {
      const auto cloud = clouds_.front();
      // 姿勢より新しい点群は、次の姿勢が届くまで待つ。FAST-LIO の時刻の切り捨ての分は待たない
      if (waitsForNewerPose(toNanoseconds(cloud->header.stamp), latest_pose_->stamp_ns)) {
        return;
      }
      clouds_.pop_front();
      processCloud(*cloud);
    }
  }

  void processCloud(const CustomMsg & cloud)
  {
    const auto start = std::chrono::steady_clock::now();
    if (!base_to_lidar_.has_value()) {
      const auto frame = cloud.header.frame_id;
      base_to_lidar_ = lookup(get_parameter("base_frame").as_string(), frame);
      if (!base_to_lidar_.has_value()) {
        return;
      }
    }
    if (!base_to_additional_sensors_.has_value()) {
      base_to_additional_sensors_ = lookupAdditionalSensors();
      if (!base_to_additional_sensors_.has_value()) {
        return;
      }
    }
    const auto map_to_odom = lookup(
      get_parameter("map_frame").as_string(), get_parameter("odom_frame").as_string());
    if (!map_to_odom.has_value()) {
      return;
    }

    FrameTransforms transforms;
    transforms.base_to_lidar = *base_to_lidar_;
    transforms.map_to_odom = *map_to_odom;

    const StampedPose before = previous_pose_.value_or(*latest_pose_);
    const auto points = toFramePoints(cloud, before, *latest_pose_);
    SelectionCounts counts;
    const auto selected = selectObstaclePoints(
      points, transforms, detection_params_, static_map_, counts);
    std::vector<Eigen::Vector2f> sensors{
      (latest_pose_->pose * transforms.base_to_lidar).translation().head<2>()};
    for (const auto & base_to_sensor : *base_to_additional_sensors_) {
      sensors.push_back((latest_pose_->pose * base_to_sensor).translation().head<2>());
    }
    const auto detections = clusterObstaclePoints(selected, sensors, detection_params_);
    publishDetections(cloud, detections);

    const std::uint64_t stamp_ns = toNanoseconds(cloud.header.stamp);
    const double dt = last_stamp_ns_.has_value() && stamp_ns > *last_stamp_ns_ ?
      static_cast<double>(stamp_ns - *last_stamp_ns_) * 1e-9 : 0.0;
    last_stamp_ns_ = stamp_ns;
    tracker_.update(detections, dt, tracking_params_);

    publishTracks(cloud, *map_to_odom);
    ProcessStats stats;
    stats.header = cloud.header;
    stats.input_points = static_cast<std::uint32_t>(counts.input);
    stats.output_points = static_cast<std::uint32_t>(counts.after_outlier);
    stats.processing_ms = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    stats.skipped_frames = skipped_frames_;
    stats_publisher_->publish(stats);
  }

  // 計測した箱（クラスタの見えている点を囲む箱）を線で出す。確認用なので、購読者がいなければ作らない
  void publishDetections(const CustomMsg & cloud, const std::vector<Detection> & detections)
  {
    if (detections_publisher_->get_subscription_count() == 0U) {
      return;
    }
    Marker marker;
    marker.header.stamp = cloud.header.stamp;
    marker.header.frame_id = get_parameter("odom_frame").as_string();
    marker.ns = "detections";
    marker.id = 0;
    marker.type = Marker::LINE_LIST;
    marker.action = detections.empty() ? Marker::DELETE : Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.color.r = 0.2F;
    marker.color.g = 0.9F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.points.reserve(detections.size() * 24U);
    for (const auto & detection : detections) {
      appendBoxEdges(detection, marker.points);
    }
    detections_publisher_->publish(marker);
  }

  void publishTracks(const CustomMsg & cloud, const Eigen::Isometry3f & map_to_odom)
  {
    TrackedObjectArray output;
    output.header.stamp = cloud.header.stamp;
    output.header.frame_id = get_parameter("odom_frame").as_string();
    for (const auto & track : tracker_.tracks()) {
      if (!MultiObjectTracker::isConfirmed(track, tracking_params_)) {
        continue;
      }
      TrackedObject object;
      object.id = track.id;
      object.classification = static_cast<std::uint8_t>(classify(track, classification_params_));
      object.position.x = track.state(0);
      object.position.y = track.state(1);
      object.velocity.x = track.state(2);
      object.velocity.y = track.state(3);
      object.length = track.depth.value();
      object.width = track.width.value();
      object.height = track.height.value();
      object.position_sigma = static_cast<float>(
        std::sqrt(0.5 * (track.covariance(0, 0) + track.covariance(1, 1))) +
        tracking_params_.coast_sigma_growth_mps * track.coast_s);
      object.velocity_sigma =
        static_cast<float>(uncertainVelocitySigma(track, tracking_params_));
      object.age_s = static_cast<float>(track.age_s);
      object.coasting = track.coast_s > 0.0;
      output.objects.push_back(object);
    }
    tracks_publisher_->publish(output);
    if (map_tracks_publisher_->get_subscription_count() == 0U) {
      return;
    }
    TrackedObjectArray in_map = output;
    in_map.header.frame_id = get_parameter("map_frame").as_string();
    for (auto & object : in_map.objects) {
      const PlanarTrackState odom_state{
        object.position.x, object.position.y, object.velocity.x, object.velocity.y};
      const auto state = transformTrackState(odom_state, map_to_odom);
      object.position.x = state.x;
      object.position.y = state.y;
      object.velocity.x = state.vx;
      object.velocity.y = state.vy;
    }
    map_tracks_publisher_->publish(in_map);
  }

  DetectionParams detection_params_;
  TrackingParams tracking_params_;
  ClassificationParams classification_params_;
  StaticMap static_map_;
  MultiObjectTracker tracker_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<Eigen::Isometry3f> base_to_lidar_;
  std::optional<std::vector<Eigen::Isometry3f>> base_to_additional_sensors_;
  std::optional<StampedPose> latest_pose_;
  std::optional<StampedPose> previous_pose_;
  std::optional<std::uint64_t> last_stamp_ns_;
  std::deque<CustomMsg::ConstSharedPtr> clouds_;
  std::uint32_t skipped_frames_{0U};
  core_nav_common::ParameterUpdateHandle parameter_callback_;
  rclcpp::Publisher<TrackedObjectArray>::SharedPtr tracks_publisher_;
  rclcpp::Publisher<TrackedObjectArray>::SharedPtr map_tracks_publisher_;
  rclcpp::Publisher<ProcessStats>::SharedPtr stats_publisher_;
  rclcpp::Publisher<Marker>::SharedPtr detections_publisher_;
  rclcpp::Subscription<CustomMsg>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
};
}  // namespace core_perception

RCLCPP_COMPONENTS_REGISTER_NODE(core_perception::PerceptionNode)
