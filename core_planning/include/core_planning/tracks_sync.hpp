// 局所計画（mppi_lite）が、姿勢と同じスキャンの追跡結果を待つかどうかを決める。ROS に依存しない。
//
// 検出（perception_node）も局所計画も、同じ /lio/odom を受けてから計算する。そのため局所計画が姿勢を
// 受けた時点では、同じスキャンの追跡結果はまだ出ていない（検出は約 1ms 後に出す）。待たずに計算すると、
// いつも 1 つ前のスキャン（約 100ms 前）の追跡結果を使うことになるので、短い上限時間まで待つ。

#ifndef CORE_PLANNING__TRACKS_SYNC_HPP_
#define CORE_PLANNING__TRACKS_SYNC_HPP_

#include <cstdint>
#include <optional>

namespace core_planning
{

// FAST-LIO は時刻を double の秒から戻すときに ns を切り捨てるので、姿勢の時刻が点群（追跡結果）より
// 1ns ほど早くなることがある。追跡結果の時刻がこれだけ早くても、同じスキャンとみなす [ns]
constexpr std::int64_t kScanStampToleranceNs = 1000;

struct TracksWaitParams
{
  // 待つ上限 [s]。0 なら待たない
  double wait_s{0.02};
  // 最新の追跡結果がこれより古ければ、検出が止まっているとみなして待たない [s]
  double alive_s{1.0};
};

// 時刻 tracks_ns の追跡結果が、時刻 pose_ns の姿勢と同じか新しいスキャンのものか
constexpr bool isTracksForPose(const std::int64_t tracks_ns, const std::int64_t pose_ns)
{
  return tracks_ns + kScanStampToleranceNs >= pose_ns;
}

// 時刻 pose_ns の姿勢で計算する前に、同じスキャンの追跡結果を待つか。latest_tracks_ns は、
// これまでに受けた最新の追跡結果の時刻（まだなければ nullopt）
constexpr bool shouldWaitForTracks(
  const std::optional<std::int64_t> & latest_tracks_ns, const std::int64_t pose_ns,
  const TracksWaitParams & params)
{
  if (params.wait_s <= 0.0 || !latest_tracks_ns.has_value()) {
    return false;
  }
  if (isTracksForPose(*latest_tracks_ns, pose_ns)) {
    return false;
  }
  const double age_s = static_cast<double>(pose_ns - *latest_tracks_ns) * 1e-9;
  return age_s <= params.alive_s;
}

}  // namespace core_planning

#endif  // CORE_PLANNING__TRACKS_SYNC_HPP_
