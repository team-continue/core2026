// 等速カルマンフィルタとハンガリアン法による複数物体の追跡。ROS に依存しない。

#ifndef CORE_PERCEPTION__TRACKER_HPP_
#define CORE_PERCEPTION__TRACKER_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core_perception/detection.hpp"

namespace core_perception
{
enum class ObjectClass : std::uint8_t {kUnknown = 0, kRobot = 1, kHuman = 2};

// 確定した追跡が消えた直後に、近くで生まれた追跡へ速度と ID を引き継ぐ条件
struct HandoverParams
{
  bool enabled{true};
  // 消えてからこの時間までに生まれた（または生まれていた）追跡に引き継ぐ
  double max_gap_s{0.5};
  // 消えた追跡を等速で進めた位置と、新しい追跡の位置の距離の上限
  double max_distance_m{1.5};
  // 引き継いだ速度の標準偏差に足す値。別の物体に引き継いでも、数回の観測で直るようにする
  double extra_velocity_sigma_mps{0.5};
};

struct TrackingParams
{
  double max_coast_s{1.0};
  double coast_sigma_growth_mps{1.0};
  std::size_t min_hits{2};
  double association_gate_m{1.5};
  // 加速度の標準偏差（プロセスノイズ）と観測の標準偏差
  double accel_sigma_mps2{3.0};
  double measurement_sigma_m{0.2};
  // 追跡を始めたときの速度の標準偏差。大きいと 1 回の観測のぶれがそのまま速度になる
  double initial_velocity_sigma_mps{3.0};
  // 相手の速度の上限。観測のぶれで非現実的な速度になり、追跡が離れていくのを防ぐ
  double max_speed_mps{4.0};
  // 速度の標準偏差がこれを下回れば、速度が定まったとみなす。追跡が落ち着いたときの値より少し大きくする
  double velocity_settled_sigma_mps{0.5};
  HandoverParams handover;
};

struct ClassificationParams
{
  double robot_max_height_m{1.0};
  double robot_min_width_m{0.2};
  double robot_max_width_m{1.3};
  double human_min_height_m{1.4};
  double human_max_width_m{0.7};
};

// 直近の値の 75% 点を返す。他の物体と一時的に混ざった観測に引きずられない
class RecentPercentile
{
public:
  void add(const float value)
  {
    values_[next_ % values_.size()] = value;
    ++next_;
  }

  float value() const
  {
    const std::size_t count = std::min(next_, values_.size());
    if (count == 0U) {
      return 0.0F;
    }
    std::array<float, kCapacity> sorted{};
    std::copy_n(values_.begin(), count, sorted.begin());
    const std::size_t index = (count * 3U) / 4U;
    std::nth_element(sorted.begin(), sorted.begin() + index, sorted.begin() + count);
    return sorted[index];
  }

private:
  // 10Hz で 3 秒分
  static constexpr std::size_t kCapacity = 30U;
  std::array<float, kCapacity> values_{};
  std::size_t next_{0U};
};

struct Track
{
  std::uint32_t id{0};
  // 位置 x, y と速度 vx, vy
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};
  std::size_t hits{0};
  double age_s{0.0};
  double coast_s{0.0};
  // 追跡器の時計で、追跡を始めた時刻
  double birth_s{0.0};
  // 消えた追跡から速度を引き継いだか
  bool inherited{false};
  RecentPercentile height;
  RecentPercentile width;
  RecentPercentile depth;
};

ObjectClass classify(const Track & track, const ClassificationParams & params);

// 速度の標準偏差（x と y の分散の平均の平方根）
double velocitySigma(const Track & track);

// 速度の不確かさ。速度の標準偏差のうち velocity_settled_sigma_mps を超える分で、定まっていれば 0
double uncertainVelocitySigma(const Track & track, const TrackingParams & params);

class MultiObjectTracker
{
public:
  // dt 秒進めて観測を反映する
  void update(
    const std::vector<Detection> & detections, double dt, const TrackingParams & params);

  const std::vector<Track> & tracks() const {return tracks_;}

  // 確定した（min_hits 回以上観測した）追跡か
  static bool isConfirmed(const Track & track, const TrackingParams & params)
  {
    return track.hits >= params.min_hits;
  }

private:
  // 消えた確定済みの追跡。引き継ぎの相手を探す間だけ持つ
  struct LostTrack
  {
    Track track;
    // 消した時刻と、最後に観測した時刻（追跡器の時計）
    double deleted_s{0.0};
    double last_seen_s{0.0};
  };

  void predict(double dt, const TrackingParams & params);
  void createTrack(const Detection & detection, const TrackingParams & params);
  // 消えた追跡を、その後に生まれた近くの追跡へ引き継ぐ
  void handOver(const TrackingParams & params);

  std::vector<Track> tracks_;
  std::vector<LostTrack> lost_;
  std::uint32_t next_id_{1U};
  double time_s_{0.0};
};
}  // namespace core_perception

#endif  // CORE_PERCEPTION__TRACKER_HPP_
