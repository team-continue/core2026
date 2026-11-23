// 2 台の LiDAR（前の 1 台目と後ろの 2 台目）の点群を時刻でそろえて 1 つにし、検出に渡すノード。
//
// LIO と地図照合は 1 台目（/livox/lidar）だけを使う。このノードの出力は検出（perception_node）だけが読む。
// - 組にする: 1 台目のスキャンごとに、時刻（timebase）の差が max_time_difference_s 以内で最も近い 2 台目と
//   組にする。実機は 2 台の時計を PTP で合わせる前提で、時刻の差をそのまま比べる
// - 組にできないとき: 2 台目が届いていれば max_wait_s まで待ち、来なければ 1 台目だけを出す。2 台目が
//   secondary_timeout_s の間届いていなければ（止まった、起動していない）、待たずに 1 台目だけを出す。
//   1 台目が届かなければ何も出さない（LIO も止まり、検出は姿勢がないと処理できないため）
// - 2 台目の点は静的な TF（1 台目のフレーム ← 2 台目のフレーム）で 1 台目のフレームへ写す。TF が
//   まだ取れなければ 1 台目だけを出す。2 台目のフレームはヘッダの frame_id か、secondary_frame_id（実機の
//   livox_ros_driver2 は全 LiDAR に同じ frame_id を付けるので、そのときに指定する）
// - 点ごとの時刻は、2 つのうち早いほうの timebase からの差に直す（lidar_merge.hpp）。voxel で間引き、
//   voxel ごとに時刻の最も早い点を残す

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_nav_common/parameters.hpp"
#include "core_perception/lidar_merge.hpp"

namespace core_perception
{
namespace
{
using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;

// 2 台目のスキャンを手元に置く数。10Hz で 0.5 秒分
constexpr std::size_t kMaxQueuedSecondary = 5U;
// 統計をログに出す間隔
constexpr std::chrono::seconds kReportInterval{30};

std::uint64_t secondsToNanoseconds(const double seconds)
{
  return static_cast<std::uint64_t>(std::llround(seconds * 1e9));
}

Scan toScan(const CustomMsg & message)
{
  Scan scan;
  scan.timebase_ns = message.timebase;
  scan.points.reserve(message.points.size());
  for (const auto & point : message.points) {
    ScanPoint converted;
    converted.position = Eigen::Vector3f(point.x, point.y, point.z);
    converted.offset_ns = point.offset_time;
    converted.reflectivity = point.reflectivity;
    converted.tag = point.tag;
    converted.line = point.line;
    scan.points.push_back(converted);
  }
  return scan;
}

// 1 台目のヘッダ（フレーム）を引き継ぎ、時刻は共通の timebase にする
CustomMsg toMessage(const Scan & scan, const CustomMsg & primary)
{
  CustomMsg output;
  output.header.frame_id = primary.header.frame_id;
  output.header.stamp.sec = static_cast<std::int32_t>(scan.timebase_ns / 1000000000ULL);
  output.header.stamp.nanosec = static_cast<std::uint32_t>(scan.timebase_ns % 1000000000ULL);
  output.timebase = scan.timebase_ns;
  output.lidar_id = primary.lidar_id;
  output.rsvd = primary.rsvd;
  output.points.reserve(scan.points.size());
  for (const auto & point : scan.points) {
    CustomPoint converted;
    converted.offset_time = point.offset_ns;
    converted.x = point.position.x();
    converted.y = point.position.y();
    converted.z = point.position.z();
    converted.reflectivity = point.reflectivity;
    converted.tag = point.tag;
    converted.line = point.line;
    output.points.push_back(converted);
  }
  output.point_num = static_cast<std::uint32_t>(output.points.size());
  return output;
}
}  // namespace

class LidarMergeNode final : public rclcpp::Node
{
public:
  LidarMergeNode()
  : Node("lidar_merge"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    using core_nav_common::declareDouble;
    using core_nav_common::declareString;

    const auto primary_topic = declareString(
      *this, "primary_topic", "/sensing/livox/lidar", "1 台目（LIO に使う LiDAR）の点群", true);
    const auto secondary_topic = declareString(
      *this, "secondary_topic", "/livox/lidar2", "2 台目（検出だけに使う LiDAR）の点群", true);
    const auto output_topic = declareString(
      *this, "output_topic", "/perception/livox/lidar_merged", "合わせた点群（1 台目のフレーム）", true);
    max_difference_ns_ = secondsToNanoseconds(declareDouble(
        *this, "max_time_difference_s", 0.05, 0.0, 1.0,
        "組にする 2 つのスキャンの時刻（timebase）の差の上限", true));
    max_wait_ = std::chrono::duration<double>(declareDouble(
        *this, "max_wait_s", 0.05, 0.0, 1.0,
        "組にする 2 台目を待つ時間の上限。過ぎたら 1 台目だけを出す", true));
    secondary_timeout_ = std::chrono::duration<double>(declareDouble(
        *this, "secondary_timeout_s", 0.5, 0.0, 10.0,
        "2 台目がこの間届いていなければ、待たずに 1 台目だけを出す", true));
    voxel_size_m_ = declareDouble(
      *this, "voxel_size_m", 0.1, 0.0, 2.0, "間引く voxel の大きさ（0 で間引かない）", true);
    secondary_frame_id_ = declareString(
      *this, "secondary_frame_id", "",
      "2 台目の点群のフレーム。空ならヘッダの frame_id を使う（ドライバが 1 台目と同じ名前を付けるとき指定する）",
      true);

    publisher_ = create_publisher<CustomMsg>(output_topic, rclcpp::QoS(10));
    // 1 メッセージが大きく、Best Effort だと欠落するので Reliable で受ける（検出と同じ）
    primary_subscription_ = create_subscription<CustomMsg>(
      primary_topic, rclcpp::QoS(5).reliable(),
      [this](const CustomMsg::ConstSharedPtr & message) {onPrimary(message);});
    secondary_subscription_ = create_subscription<CustomMsg>(
      secondary_topic, rclcpp::QoS(5).reliable(),
      [this](const CustomMsg::ConstSharedPtr & message) {onSecondary(message);});
    report_timer_ = create_wall_timer(kReportInterval, [this]() {report();});
    RCLCPP_INFO(
      get_logger(), "%s と %s を合わせて %s に出します（時刻の差 %.0fms 以内、待つ %.0fms まで、voxel %.2fm）",
      primary_topic.c_str(), secondary_topic.c_str(), output_topic.c_str(),
      static_cast<double>(max_difference_ns_) * 1e-6, max_wait_.count() * 1e3, voxel_size_m_);
  }

private:
  using Clock = std::chrono::steady_clock;

  void onPrimary(const CustomMsg::ConstSharedPtr & message)
  {
    // 前のスキャンを待っている間に次が来たら、前のスキャンは 1 台目だけで出す
    if (pending_primary_ != nullptr) {
      publishPrimaryOnly("次の 1 台目のスキャンが来た");
    }
    const bool secondary_alive = last_secondary_arrival_.has_value() &&
      Clock::now() - *last_secondary_arrival_ <= secondary_timeout_;
    const auto decision = decidePairing(
      message->timebase, secondaryStamps(), max_difference_ns_, secondary_alive);
    switch (decision.action) {
      case PairingAction::kMerge:
        publishMerged(*message, decision.secondary_index);
        return;
      case PairingAction::kWait:
        pending_primary_ = message;
        wait_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(max_wait_),
          [this]() {
            if (pending_primary_ != nullptr) {
              publishPrimaryOnly("2 台目が待つ時間の上限までに届かなかった");
            }
          });
        return;
      case PairingAction::kPrimaryOnly:
        pending_primary_ = message;
        publishPrimaryOnly(
          secondary_alive ? "組にできる時刻の 2 台目がない" : "2 台目が届いていない");
        return;
    }
  }

  void onSecondary(const CustomMsg::ConstSharedPtr & message)
  {
    last_secondary_arrival_ = Clock::now();
    secondaries_.push_back(message);
    while (secondaries_.size() > kMaxQueuedSecondary) {
      secondaries_.pop_front();
    }
    if (pending_primary_ == nullptr) {
      return;
    }
    if (const auto index = findClosestStamp(
        secondaryStamps(), pending_primary_->timebase, max_difference_ns_))
    {
      const auto primary = pending_primary_;
      clearPending();
      publishMerged(*primary, *index);
    }
  }

  std::vector<std::uint64_t> secondaryStamps() const
  {
    std::vector<std::uint64_t> stamps;
    stamps.reserve(secondaries_.size());
    for (const auto & secondary : secondaries_) {
      stamps.push_back(secondary->timebase);
    }
    return stamps;
  }

  void clearPending()
  {
    pending_primary_.reset();
    if (wait_timer_ != nullptr) {
      wait_timer_->cancel();
      wait_timer_.reset();
    }
  }

  std::optional<Eigen::Isometry3f> lookupPrimaryFromSecondary(
    const std::string & primary_frame, const std::string & secondary_frame)
  {
    if (primary_from_secondary_.has_value()) {
      return primary_from_secondary_;
    }
    try {
      // 取り付けは動かないので、最新の値を 1 度だけ取って使い続ける
      primary_from_secondary_ = tf2::transformToEigen(
        tf_buffer_.lookupTransform(primary_frame, secondary_frame, tf2::TimePointZero)).cast<float>();
      RCLCPP_INFO(
        get_logger(), "%s から %s への取り付けの変換を取得しました",
        secondary_frame.c_str(), primary_frame.c_str());
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s から %s への変換を取得できないので、1 台目だけを出します: %s",
        secondary_frame.c_str(), primary_frame.c_str(), error.what());
    }
    return primary_from_secondary_;
  }

  void publishMerged(const CustomMsg & primary, const std::size_t secondary_index)
  {
    const auto secondary = secondaries_.at(secondary_index);
    // 使った 2 台目と、それより古い 2 台目は捨てる
    secondaries_.erase(
      secondaries_.begin(), secondaries_.begin() + static_cast<std::ptrdiff_t>(secondary_index) + 1);
    const auto transform = lookupPrimaryFromSecondary(
      primary.header.frame_id,
      secondary_frame_id_.empty() ? secondary->header.frame_id : secondary_frame_id_);
    if (!transform.has_value()) {
      publish(primary, nullptr, false);
      return;
    }
    publish(primary, secondary, true);
  }

  void publishPrimaryOnly(const char * reason)
  {
    const auto primary = pending_primary_;
    clearPending();
    if (primary == nullptr) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "1 台目だけを出します（%s）。これまでに %lu 回", reason,
      static_cast<unsigned long>(primary_only_count_ + 1U));  // NOLINT(runtime/int)
    publish(*primary, nullptr, false);
  }

  void publish(
    const CustomMsg & primary, const CustomMsg::ConstSharedPtr & secondary, const bool paired)
  {
    const auto start = Clock::now();
    bool merged_both = paired;
    MergeCounts counts;
    std::optional<Scan> secondary_scan;
    if (secondary != nullptr) {
      secondary_scan = toScan(*secondary);
    }
    try {
      const Scan merged = mergeScans(
        toScan(primary), secondary_scan,
        primary_from_secondary_.value_or(Eigen::Isometry3f::Identity()), voxel_size_m_, counts);
      publisher_->publish(toMessage(merged, primary));
    } catch (const std::invalid_argument & error) {
      // 時刻が壊れたスキャンの組。1 台目だけでも出す
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "2 つのスキャンを合わせられないので、1 台目だけを出します: %s",
        error.what());
      const Scan alone = mergeScans(
        toScan(primary), std::nullopt, Eigen::Isometry3f::Identity(), voxel_size_m_, counts);
      publisher_->publish(toMessage(alone, primary));
      merged_both = false;
    }
    const double elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    if (merged_both) {
      ++paired_count_;
    } else {
      ++primary_only_count_;
    }
    if (counts.dropped_invalid > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "座標が不正な点を %zu 点捨てました", counts.dropped_invalid);
    }
    processing_ms_sum_ += elapsed_ms;
    processing_ms_max_ = std::max(processing_ms_max_, elapsed_ms);
    input_points_sum_ += counts.primary_input + counts.secondary_input;
    output_points_sum_ += counts.output;
    ++reported_frames_;
  }

  void report()
  {
    if (reported_frames_ == 0U) {
      RCLCPP_WARN(get_logger(), "この %ld 秒に 1 台目の点群が届いていません", kReportInterval.count());
      return;
    }
    const double frames = static_cast<double>(reported_frames_);
    RCLCPP_INFO(
      get_logger(),
      "組にした %lu、1 台目だけ %lu（累計）。この %ld 秒: 入力 平均 %.0f 点、出力 平均 %.0f 点、"
      "処理 平均 %.2fms 最大 %.2fms",
      static_cast<unsigned long>(paired_count_),  // NOLINT(runtime/int)
      static_cast<unsigned long>(primary_only_count_),  // NOLINT(runtime/int)
      kReportInterval.count(), static_cast<double>(input_points_sum_) / frames,
      static_cast<double>(output_points_sum_) / frames, processing_ms_sum_ / frames, processing_ms_max_);
    reported_frames_ = 0U;
    processing_ms_sum_ = 0.0;
    processing_ms_max_ = 0.0;
    input_points_sum_ = 0U;
    output_points_sum_ = 0U;
  }

  std::uint64_t max_difference_ns_{0U};
  std::chrono::duration<double> max_wait_{0.0};
  std::chrono::duration<double> secondary_timeout_{0.0};
  double voxel_size_m_{0.0};
  std::string secondary_frame_id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<Eigen::Isometry3f> primary_from_secondary_;
  CustomMsg::ConstSharedPtr pending_primary_;
  std::deque<CustomMsg::ConstSharedPtr> secondaries_;
  std::optional<Clock::time_point> last_secondary_arrival_;
  std::uint64_t paired_count_{0U};
  std::uint64_t primary_only_count_{0U};
  std::uint64_t reported_frames_{0U};
  double processing_ms_sum_{0.0};
  double processing_ms_max_{0.0};
  std::uint64_t input_points_sum_{0U};
  std::uint64_t output_points_sum_{0U};
  rclcpp::Publisher<CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<CustomMsg>::SharedPtr primary_subscription_;
  rclcpp::Subscription<CustomMsg>::SharedPtr secondary_subscription_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};
}  // namespace core_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<core_perception::LidarMergeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_merge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
