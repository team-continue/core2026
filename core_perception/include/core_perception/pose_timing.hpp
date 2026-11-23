// 点群と LIO の姿勢の時刻を比べる。
//
// 検出は、点群の時刻以上の姿勢が届いてから点群を処理する。FAST-LIO は時刻を double の秒から
// 戻すときに ns を切り捨てるので、同じスキャンの姿勢の時刻が点群より 1ns ほど早くなることがある。
// 少しの余裕を持たせないと、その点群は次の姿勢（約 100ms 後）まで待ってしまう。
// シミュレーションでは点ごとの時刻がすべて 0 で、点群と同じスキャンの姿勢の時刻がほぼ同じになるので、
// 約 4 割のスキャンでこれが起きていた。実機の Mid-360 は、前のフレームの最後の点と次のフレームの
// 最初の点が数 µs 離れているので、1µs の余裕なら前のフレームの姿勢を同じスキャンの姿勢と取り違えない。

#ifndef CORE_PERCEPTION__POSE_TIMING_HPP_
#define CORE_PERCEPTION__POSE_TIMING_HPP_

#include <cstdint>

namespace core_perception
{

// 姿勢の時刻が点群よりこれだけ早くても、同じ時刻とみなす [ns]
constexpr std::uint64_t kPoseStampToleranceNs = 1000U;

// 時刻 cloud_ns の点群を処理するのに、時刻 pose_ns の姿勢より新しい姿勢を待つ必要があるか
constexpr bool waitsForNewerPose(const std::uint64_t cloud_ns, const std::uint64_t pose_ns)
{
  return cloud_ns > pose_ns + kPoseStampToleranceNs;
}

}  // namespace core_perception

#endif  // CORE_PERCEPTION__POSE_TIMING_HPP_
