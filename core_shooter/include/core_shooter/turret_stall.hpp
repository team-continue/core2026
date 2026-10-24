#ifndef CORE_SHOOTER__TURRET_STALL_HPP_
#define CORE_SHOOTER__TURRET_STALL_HPP_

#include <cmath>
#include <optional>

namespace core_shooter
{

/// 「指令に追従できずに止まっている」の判定条件。
struct StallParams
{
  double error_threshold = 0.0;     ///< |指令角 - 実角| がこれを超えたら候補 [rad]
  double velocity_threshold = 0.0;  ///< |速度| がこれ未満なら止まっているとみなす [rad/s]
  double hold_sec = 0.0;            ///< 候補がこの時間続いたら stalled [s]
};

struct StallState
{
  double candidate_sec = 0.0;  ///< 候補が続いている時間 [s]
  bool stalled = false;
};

/// 指令角と実角の差が大きいのに動いていない状態が続いたかを 1 周期ぶん進める。
///
/// 大きく動いている最中の追従遅れは速度が出ているので候補にならない。
/// 原点合わせのストッパ接触と、稼働中の衝突検知の両方で使う。
inline StallState stepStall(
  const StallState & state, double command, double actual, double velocity, double dt,
  const StallParams & params)
{
  const bool candidate =
    std::fabs(command - actual) > params.error_threshold &&
    std::fabs(velocity) < params.velocity_threshold;
  if (!candidate) {
    return StallState{};
  }
  const double candidate_sec = state.candidate_sec + dt;
  return StallState{candidate_sec, candidate_sec >= params.hold_sec};
}

/// 衝突で塞がれた向きの限界角。当たった向きにだけ片側の制限を置く。
struct DirectionalBlock
{
  std::optional<double> lower;  ///< これより小さい指令角を出さない [rad]
  std::optional<double> upper;  ///< これより大きい指令角を出さない [rad]
};

/// stall を検知したときに、当たった向きを実角で塞ぐ。
///
/// 当たった向きは指令角 - 実角 の符号で決める。指令がそれより先へ行かないので
/// 押し付けが止まり、逆向きの指令はそのまま通る。
inline DirectionalBlock blockOnStall(DirectionalBlock block, double command, double actual)
{
  if (command > actual) {
    block.upper = actual;
  } else if (command < actual) {
    block.lower = actual;
  }
  return block;
}

/// 実角が塞いだ角度から逆向きに margin 以上離れたら、その向きの制限を外す。
///
/// 解除の操作は要らない。障害物が無くなっていれば次はそのまま通り、
/// 残っていればもう一度 stall を検知して塞ぐ。
inline DirectionalBlock releaseBlock(DirectionalBlock block, double actual, double margin)
{
  if (block.upper && actual < *block.upper - margin) {
    block.upper.reset();
  }
  if (block.lower && actual > *block.lower + margin) {
    block.lower.reset();
  }
  return block;
}

inline double clampByBlock(double command, const DirectionalBlock & block)
{
  if (block.upper && command > *block.upper) {
    command = *block.upper;
  }
  if (block.lower && command < *block.lower) {
    command = *block.lower;
  }
  return command;
}

inline bool isBlocked(const DirectionalBlock & block)
{
  return block.lower.has_value() || block.upper.has_value();
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__TURRET_STALL_HPP_
