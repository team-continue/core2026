// scan_filter の点群抽出（point_extraction.hpp）のパラメータ、TF、追跡の扱い。
//
// パラメータは point_extraction.*（モードは point_extraction.mode）と、フィールドの範囲の field.*
// （field.yaml）。既定は off で、点群に触れず、TF と追跡も購読しない（今までと同じ動きと負荷）。
// 入力は FAST-LIO の出力（/cloud_registered_body）なので、点群の時刻の odom←点群がある。map←odom は
// 地図照合がこの点群を照合した後に出すので、map だけは最新を使う（odom を固定とみなす）。
//
// 変換や追跡が得られない周期の扱い:
// - ceiling: 床の座標系（既定は odom）←点群が得られなければ、帯をかけない。map←点群が得られなければ
//   （地図照合が 1 度も成功していない間など）、観客席の箱をかけない
// - field_objects: 床と map の変換のどちらかがなければ、その周期は全点を通す。追跡がない、古い、または
//   追跡の座標系への変換がなければ、追跡の柱だけをかけない
// どれも外したことを警告する（同じ警告は 5 秒に 1 回）。

#ifndef CORE_LOCALIZATION__POINT_EXTRACTION_STAGE_HPP_
#define CORE_LOCALIZATION__POINT_EXTRACTION_STAGE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <core_msgs/msg/process_stats.hpp>
#include <core_msgs/msg/tracked_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_localization/point_extraction.hpp"
#include "core_nav_common/parameter_updates.hpp"

namespace core_localization
{
struct PointExtractionStageParams
{
  point_extraction::Params extraction;
  std::string floor_frame{"odom"};
  // 床の座標系での床の高さ
  float floor_z_m{0.0F};
  std::string map_frame{"map"};
  // map の変換を求める間は動かないとみなす座標系
  std::string odom_frame{"odom"};
  // 点群の時刻の TF を待つ時間の上限（実時間）
  double tf_wait_s{0.05};
  std::string tracks_topic{"/perception/tracks"};
};

class PointExtractionStage
{
public:
  explicit PointExtractionStage(rclcpp::Node & node);

  bool active() const;

  // 選んだモードで抽出した点を返す。off なら入力をそのまま返す
  template<typename Point, typename PositionOf>
  std::vector<Point> apply(
    std::vector<Point> points, const std::string & cloud_frame, const rclcpp::Time & stamp,
    PositionOf position_of)
  {
    if (!active()) {
      return points;
    }
    const auto start = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector3f> positions;
    positions.reserve(points.size());
    for (const auto & point : points) {
      const std::array<float, 3> position = position_of(point);
      positions.emplace_back(position[0], position[1], position[2]);
    }
    const point_extraction::Result result = extract(positions, cloud_frame, stamp);
    auto output = crowd::selectKept(points, result.keep);
    publishStats(result, stamp, start);
    return output;
  }

private:
  point_extraction::Result extract(
    const std::vector<Eigen::Vector3f> & positions, const std::string & cloud_frame,
    const rclcpp::Time & stamp);
  point_extraction::Transforms lookupTransforms(
    const std::string & cloud_frame, const rclcpp::Time & stamp);
  // 点群の時刻の odom←点群を使い、odom から target までは最新を使った target←点群。
  // 得られなければ警告して nullopt
  std::optional<Eigen::Isometry3f> lookupAtCloud(
    const std::string & target, const std::string & cloud_frame, const rclcpp::Time & stamp,
    const char * purpose);
  // 点群の時刻の odom←点群が届くのを tf_wait_s まで待つ
  void waitForCloudPose(const std::string & cloud_frame, const rclcpp::Time & stamp) const;
  // 点群の時刻まで進めた追跡の柱。追跡がない、古いときは nullopt（警告する）
  std::optional<std::vector<point_extraction::Cylinder>> trackCylinders(const rclcpp::Time & stamp);
  void ensureTf();
  void ensureTracks();
  void report(const point_extraction::Result & result);
  void publishStats(
    const point_extraction::Result & result, const rclcpp::Time & stamp,
    std::chrono::steady_clock::time_point start);
  void logSettings() const;

  rclcpp::Node & node_;
  // 警告の間引きは実時間で数える。シミュレーション時間だと、時刻 0 から間引きの間隔までの警告が出ない
  rclcpp::Clock warn_clock_{RCL_STEADY_TIME};
  PointExtractionStageParams params_;
  std::uint64_t fallback_count_{0U};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<core_msgs::msg::TrackedObjectArray>::SharedPtr tracks_subscription_;
  core_msgs::msg::TrackedObjectArray::ConstSharedPtr latest_tracks_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_callback_;
  core_nav_common::ParameterUpdateHandle update_callback_;
  rclcpp::Publisher<core_msgs::msg::ProcessStats>::SharedPtr stats_publisher_;
};
}  // namespace core_localization

#endif  // CORE_LOCALIZATION__POINT_EXTRACTION_STAGE_HPP_
