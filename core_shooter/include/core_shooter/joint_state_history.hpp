#ifndef CORE_SHOOTER__JOINT_STATE_HISTORY_HPP_
#define CORE_SHOOTER__JOINT_STATE_HISTORY_HPP_

#include <algorithm>
#include <deque>
#include <optional>

namespace core_shooter
{

/// ある時刻の砲塔の yaw / pitch 角 [rad]。
struct AxisPair
{
  double yaw = 0.0;
  double pitch = 0.0;
};

/// 時刻付きの yaw / pitch 角。
struct TimedAngles
{
  double time_sec = 0.0;
  AxisPair angles;
};

/// 指定時刻を問い合わせたときの結果。
struct JointLookup
{
  AxisPair angles;
  /// 問い合わせ時刻が履歴の範囲からはみ出した量 [s]。範囲内なら 0、
  /// 最古より前なら負、最新より後なら正。はみ出したときは端の値で代用している。
  double outside_sec = 0.0;
};

/// 関節角の履歴。画像の撮影時刻に砲塔がどこを向いていたかを引くために使う。
///
/// 画像の誤差は撮影時刻の砲塔の向きに対する値なので、現在角に足すと
/// 遅延のあいだに回った分だけずれる。撮影時刻の角度に足せば正しい目標角になる。
class JointStateHistory
{
public:
  explicit JointStateHistory(double max_age_sec)
  : max_age_sec_(max_age_sec) {}

  /// サンプルを追加する。時刻が前のサンプル以前なら捨てる（時刻の巻き戻り対策）。
  void push(const TimedAngles & sample)
  {
    if (!samples_.empty() && sample.time_sec <= samples_.back().time_sec) {
      return;
    }
    samples_.push_back(sample);
    while (samples_.size() > 2 &&
      samples_.back().time_sec - samples_[1].time_sec >= max_age_sec_)
    {
      samples_.pop_front();
    }
  }

  void clear() {samples_.clear();}

  bool empty() const {return samples_.empty();}

  /// 指定時刻の関節角を前後のサンプルから線形補間で求める。
  /// 履歴の範囲外なら端の値を返し、はみ出し量を outside_sec に入れる。履歴が空なら nullopt。
  std::optional<JointLookup> at(double time_sec) const
  {
    if (samples_.empty()) {
      return std::nullopt;
    }
    if (time_sec <= samples_.front().time_sec) {
      return JointLookup{samples_.front().angles, time_sec - samples_.front().time_sec};
    }
    if (time_sec >= samples_.back().time_sec) {
      return JointLookup{samples_.back().angles, time_sec - samples_.back().time_sec};
    }

    const auto upper = std::lower_bound(
      samples_.begin(), samples_.end(), time_sec,
      [](const TimedAngles & sample, double t) {return sample.time_sec < t;});
    const TimedAngles & after = *upper;
    const TimedAngles & before = *(upper - 1);
    const double ratio = (time_sec - before.time_sec) / (after.time_sec - before.time_sec);
    return JointLookup{
      AxisPair{
        before.angles.yaw + (after.angles.yaw - before.angles.yaw) * ratio,
        before.angles.pitch + (after.angles.pitch - before.angles.pitch) * ratio},
      0.0};
  }

private:
  double max_age_sec_;
  std::deque<TimedAngles> samples_;
};

}  // namespace core_shooter

#endif  // CORE_SHOOTER__JOINT_STATE_HISTORY_HPP_
