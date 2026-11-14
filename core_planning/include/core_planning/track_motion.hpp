// 追跡の速度を、安全停止（safety_gate）の動いている相手の判定（正面の相手への減速、衝撃の緩和）に使って
// よいかを決める。ROS に依存しない。局所計画は追跡の速度をそのまま使う（速度を 0 にすると、追跡を始めた直後や
// 見えなくなった間の、本当に速く動いているロボットの動きを見込めず、横切る相手に当てた）。
//
// 追跡の速度は、止まっている相手にも誤った値が付くことがある。speed_limit の試験（2026-11-14）では、
// 止まっているロボットの追跡に、追跡を始めた直後や、見えなくなって予測だけで続けている間に、経路へ
// 向かう 0.4〜2.3m/s の速度が付いた。局所計画はそれを「経路へ入ってくる相手」として扱い、経路から
// 大きく外れた。ここでは、速さが min_speed_mps 以上で、速度の不確かさ（velocity_sigma）が
// max_velocity_sigma_mps 以下で、見えている（coasting でない）状態が persist_s 続いた追跡だけを、
// 動いているとみなす。使う側はロボットの追跡だけにかける（人は速度をそのまま信じる）

#ifndef CORE_PLANNING__TRACK_MOTION_HPP_
#define CORE_PLANNING__TRACK_MOTION_HPP_

#include <cstdint>
#include <iterator>
#include <unordered_map>

namespace core_planning
{

struct MotionFilterParams
{
  // false なら、速さが min_speed_mps 以上なら動いているとみなす（前の動き）
  bool enabled{true};
  // 動いているとみなす速さの下限 [m/s]。局所計画の「動いている相手」と同じ値
  double min_speed_mps{0.3};
  // 速度の不確かさの上限 [m/s]。追跡を始めた直後（約 1.5m/s）や長く隠れた後に正になる
  double max_velocity_sigma_mps{0.2};
  // 上の条件が続く時間 [s]
  double persist_s{0.5};
};

class TrackMotionFilter
{
public:
  // 追跡 id の今の状態から、動いているとみなすかを返す。time_s は追跡結果の時刻 [s]。
  // 同じ時刻で何度呼んでもよい
  bool update(
    const std::uint32_t id, const double time_s, const double speed, const double velocity_sigma,
    const bool coasting, const MotionFilterParams & params)
  {
    const bool candidate = speed >= params.min_speed_mps;
    if (!params.enabled) {
      return candidate;
    }
    if (!candidate || velocity_sigma > params.max_velocity_sigma_mps || coasting) {
      entries_.erase(id);
      return false;
    }
    const auto [entry, inserted] = entries_.try_emplace(id, Entry{time_s, time_s});
    if (!inserted && time_s < entry->second.last_s) {
      // 時刻が戻った（シミュレーションの再起動など）。数え直す
      entry->second = Entry{time_s, time_s};
    }
    entry->second.last_s = time_s;
    return time_s - entry->second.since_s >= params.persist_s;
  }

  // forget_after_s より長く update されていない id を忘れる
  void prune(const double time_s, const double forget_after_s)
  {
    for (auto it = entries_.begin(); it != entries_.end(); ) {
      it = time_s - it->second.last_s > forget_after_s ? entries_.erase(it) : std::next(it);
    }
  }

  void reset() {entries_.clear();}

private:
  struct Entry
  {
    // 条件を満たし始めた時刻
    double since_s;
    // 最後に条件を満たした時刻
    double last_s;
  };
  std::unordered_map<std::uint32_t, Entry> entries_;
};

}  // namespace core_planning

#endif  // CORE_PLANNING__TRACK_MOTION_HPP_
