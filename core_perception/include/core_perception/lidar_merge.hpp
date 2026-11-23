// 2 台の LiDAR の点群を、検出のために 1 つにまとめる処理。ROS に依存しない。
//
// 1 台目（LIO と地図照合に使う前の LiDAR）のフレームを基準にする。2 台目の点は静的な取り付けの変換で
// 1 台目のフレームへ写す。点ごとの時刻（offset_time）は、2 つのスキャンのうち早いほうの timebase を共通の
// 基準にして、そこからの差に直す。点の絶対時刻（timebase + offset）は変えないので、検出の歪み補正
// （点ごとの時刻と LIO の姿勢の補間）はそのまま使える。最後に voxel で間引き、voxel ごとに時刻の最も早い
// 点を、実際の位置と時刻のまま残す（重心にすると、動く相手を別の時刻の点と混ぜてぼかすため）。

#ifndef CORE_PERCEPTION__LIDAR_MERGE_HPP_
#define CORE_PERCEPTION__LIDAR_MERGE_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace core_perception
{
// Livox の CustomPoint と同じ中身。位置はそのスキャンの LiDAR のフレーム
struct ScanPoint
{
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
  // スキャンの timebase からの時刻 [ns]
  std::uint32_t offset_ns{0U};
  std::uint8_t reflectivity{0U};
  std::uint8_t tag{0U};
  std::uint8_t line{0U};
};

struct Scan
{
  // 点の時刻の基準 [ns]（Livox の timebase）
  std::uint64_t timebase_ns{0U};
  std::vector<ScanPoint> points;
};

struct MergeCounts
{
  std::size_t primary_input{0U};
  std::size_t secondary_input{0U};
  // 座標が有限でない、または voxel の番号の範囲を超えて捨てた点
  std::size_t dropped_invalid{0U};
  std::size_t output{0U};
};

// 1 台目のスキャンを受けたときの扱い
enum class PairingAction
{
  // secondary_index の 2 台目と組にして出す
  kMerge,
  // 組にできる 2 台目がまだ届いていないので、届くまで（上限 max_wait）待つ
  kWait,
  // 2 台目が届いていない（止まった、まだ起動していない）ので、待たずに 1 台目だけを出す
  kPrimaryOnly,
};

struct PairingDecision
{
  PairingAction action{PairingAction::kPrimaryOnly};
  std::size_t secondary_index{0U};
};

// 候補の時刻のうち target_ns に最も近いものの番号を返す。差が max_difference_ns を超えるならなし
std::optional<std::size_t> findClosestStamp(
  const std::vector<std::uint64_t> & candidates_ns, std::uint64_t target_ns,
  std::uint64_t max_difference_ns);

// 1 台目のスキャン（時刻 primary_ns）を受けたときの扱いを決める。
// secondary_ns は手元にある 2 台目のスキャンの時刻、secondary_alive は 2 台目が最近届いているか。
// 組にできる 2 台目があれば kMerge。なければ、2 台目が動いていて、手元の 2 台目がすべて 1 台目より古い
// （組になるものがこれから届きうる）なら kWait、それ以外は kPrimaryOnly
PairingDecision decidePairing(
  std::uint64_t primary_ns, const std::vector<std::uint64_t> & secondary_ns,
  std::uint64_t max_difference_ns, bool secondary_alive);

// voxel ごとに、並びの最も前の点を 1 つ残す。points は時刻の順に並んでいること（並びの前の点を
// 残すので、時刻の最も早い点が残る）。voxel_size_m が 0 以下ならそのまま返す。
// 座標が有限でない点と、voxel の番号の範囲（各軸 ±2^20 個）を超える点は捨て、dropped に足す
std::vector<ScanPoint> voxelDownsample(
  const std::vector<ScanPoint> & points, double voxel_size_m, std::size_t & dropped);

// 2 台目の点を primary_from_secondary で 1 台目のフレームへ写し、2 つのスキャンを共通の timebase
// （早いほう）からの時刻に直して時刻の順に並べ、voxel_size_m で間引く。secondary がなければ 1 台目だけを
// 間引いて返す。時刻の差が offset の範囲（uint32 の ns、約 4.29 秒）を超えると std::invalid_argument
Scan mergeScans(
  const Scan & primary, const std::optional<Scan> & secondary,
  const Eigen::Isometry3f & primary_from_secondary, double voxel_size_m, MergeCounts & counts);
}  // namespace core_perception

#endif  // CORE_PERCEPTION__LIDAR_MERGE_HPP_
