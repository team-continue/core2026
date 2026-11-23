// Livox の CustomMsg を減らして FAST-LIO に渡すノード。
//
// FAST-LIO の手前でフィルタをかける構成（filter_layout:=lio_input）で使う。残す点は実際の点のままなので、
// 点ごとの時刻を保つ。点群抽出（point_extraction）は地図照合の入力（scan_filter）だけにかけ、ここではかけない。

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <vector>

#include <core_msgs/msg/process_stats.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>

#include "core_localization/filter_params.hpp"
#include "core_nav_common/parameter_updates.hpp"

namespace core_localization
{
using core_msgs::msg::ProcessStats;
using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;

class LivoxFilter final : public rclcpp::Node
{
public:
  LivoxFilter()
  : Node("livox_filter")
  {
    declareFilterParams(*this);
    params_ = readFilterParams(*this);
    parameter_callback_ = core_nav_common::onParametersSet(
      *this, [this](const std::vector<rclcpp::Parameter> &) {params_ = readFilterParams(*this);});

    publisher_ = create_publisher<CustomMsg>("output", rclcpp::QoS(10));
    stats_publisher_ = create_publisher<ProcessStats>("~/stats", rclcpp::QoS(10));
    subscription_ = create_subscription<CustomMsg>(
      "input", rclcpp::QoS(1).reliable(),
      std::bind(&LivoxFilter::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const CustomMsg::ConstSharedPtr & cloud)
  {
    if ((frame_count_++ % static_cast<std::uint64_t>(params_.every_n_frames)) != 0U) {
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    CustomMsg output;
    output.header = cloud->header;
    output.timebase = cloud->timebase;
    output.lidar_id = cloud->lidar_id;
    const auto position_of = [](const CustomPoint & point) {
        return std::array<float, 3>{point.x, point.y, point.z};
      };
    output.points = applyFilters(cloud->points, params_, random_, position_of);
    output.point_num = static_cast<std::uint32_t>(output.points.size());
    publisher_->publish(output);

    ProcessStats stats;
    stats.header = output.header;
    stats.input_points = cloud->point_num;
    stats.output_points = output.point_num;
    stats.processing_ms = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    stats_publisher_->publish(stats);
  }

  FilterParams params_;
  std::uint64_t frame_count_{0U};
  // 同じ入力なら同じ結果になるよう、シードは固定する
  std::mt19937 random_{0U};
  core_nav_common::ParameterUpdateHandle parameter_callback_;
  rclcpp::Publisher<CustomMsg>::SharedPtr publisher_;
  rclcpp::Publisher<ProcessStats>::SharedPtr stats_publisher_;
  rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
};
}  // namespace core_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<core_localization::LivoxFilter>());
  rclcpp::shutdown();
  return 0;
}
