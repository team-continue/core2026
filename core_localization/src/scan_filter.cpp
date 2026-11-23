// FAST-LIO の歪み補正後の点群を減らして、地図照合に渡すノード。
//
// 地図照合より処理が遅いときは、キューに溜めず最新のフレームだけを処理する。
// 点群抽出（point_extraction.*、既定は off）を選ぶと、ボクセルとランダムサンプルの前にかける。
// 入力は FAST-LIO の出力なので、点群の時刻の odom→base_link がある（point_extraction_stage.hpp）。

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <core_msgs/msg/process_stats.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "core_localization/point_extraction_stage.hpp"
#include "core_localization/filter_params.hpp"

namespace core_localization
{
namespace
{
using core_msgs::msg::ProcessStats;
using sensor_msgs::msg::PointCloud2;

struct PointXyzi
{
  float x;
  float y;
  float z;
  float intensity;
};

std::vector<PointXyzi> readPoints(const PointCloud2 & cloud)
{
  std::vector<PointXyzi> points;
  points.reserve(cloud.width * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");
  for (; x != x.end(); ++x, ++y, ++z, ++intensity) {
    points.push_back(PointXyzi{*x, *y, *z, *intensity});
  }
  return points;
}

PointCloud2 writePoints(const std::vector<PointXyzi> & points, const PointCloud2 & input)
{
  PointCloud2 output;
  output.header = input.header;
  output.height = 1;
  output.width = static_cast<std::uint32_t>(points.size());
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z(output, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(output, "intensity");
  for (const auto & point : points) {
    *x = point.x;
    *y = point.y;
    *z = point.z;
    *intensity = point.intensity;
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return output;
}
}  // namespace

class ScanFilter final : public rclcpp::Node
{
public:
  explicit ScanFilter(const rclcpp::NodeOptions & options)
  : Node("scan_filter", options)
  {
    declareFilterParams(*this);
    // FAST-LIO の body は IMU の座標系なので、TF で扱える固定のフレーム名に付け替える
    output_frame_ = core_nav_common::declareString(
      *this, "output_frame", "livox_imu_link", "出力する点群のフレーム", true);
    params_ = readFilterParams(*this);
    parameter_callback_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {params_ = readFilterParams(*this);});
    extraction_ = std::make_unique<PointExtractionStage>(*this);

    publisher_ = create_publisher<PointCloud2>("output", rclcpp::QoS(1));
    stats_publisher_ = create_publisher<ProcessStats>("~/stats", rclcpp::QoS(10));
    subscription_ = create_subscription<PointCloud2>(
      "input", rclcpp::QoS(1).reliable(),
      std::bind(&ScanFilter::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const PointCloud2::ConstSharedPtr & cloud)
  {
    if ((frame_count_++ % static_cast<std::uint64_t>(params_.every_n_frames)) != 0U) {
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    const auto input = readPoints(*cloud);
    const auto position_of = [](const PointXyzi & point) {
        return std::array<float, 3>{point.x, point.y, point.z};
      };
    // 入力の座標系 body は TF にないので、同じ位置の output_frame で TF を引く
    const auto filtered = applyFilters(
      extraction_->apply(input, output_frame_, rclcpp::Time(cloud->header.stamp), position_of),
      params_, random_, position_of);
    auto output = std::make_unique<PointCloud2>(writePoints(filtered, *cloud));
    output->header.frame_id = output_frame_;
    ProcessStats stats;
    stats.header = output->header;
    // 同じプロセスの地図照合（composition:=true）へは、コピーせずにポインタで渡す
    publisher_->publish(std::move(output));

    stats.input_points = static_cast<std::uint32_t>(input.size());
    stats.output_points = static_cast<std::uint32_t>(filtered.size());
    stats.processing_ms = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    stats_publisher_->publish(stats);
  }

  std::string output_frame_;
  FilterParams params_;
  std::unique_ptr<PointExtractionStage> extraction_;
  std::uint64_t frame_count_{0U};
  // 同じ入力なら同じ結果になるよう、シードは固定する
  std::mt19937 random_{0U};
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<ProcessStats>::SharedPtr stats_publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};
}  // namespace core_localization

RCLCPP_COMPONENTS_REGISTER_NODE(core_localization::ScanFilter)
