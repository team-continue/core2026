#include "core_planning/traction_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>

namespace core_planning
{
namespace
{
// これより少ない組では推定しない
constexpr std::size_t kMinPairs = 5U;
// 誤差が最小からこの割合以内の候補は、同じくらい合うとみなす（その中で最も高い値を選ぶ）
constexpr double kTieRatio = 1.02;
// 速度の割合を見るのに要る、指令がほぼ一定の組の数と、一定とみなす指令の変化 [m/s]。割合は止まらずに
// 走り続けた所でしか見えない（加速の間は、割合と滑りのどちらでも速度の遅れを説明できる）
constexpr std::size_t kMinSteadyPairs = 5U;
constexpr double kSteadyCommandChangeMps = 0.1;
// 組の前のサンプルより前に、指令が一定であってほしい時間 [s]（駆動の一次遅れが収まるまで）
constexpr double kSteadySettleS = 0.2;
// 割合を求めるのに要る、その軸の指令の大きさ（二乗平均）[m/s]と、割合の下限
constexpr double kMinRatioCommandMps = 0.5;
constexpr double kMinRatio = 0.5;
// 1 つの窓で試す割合の幅（前に分かった割合のまわり）
constexpr double kRatioStep = 0.04;

// 時刻 time_s に効いている指令（time_s + dead_time_s で効き始める最後の指令）。なければ nullptr
const SentCommand * activeCommand(
  const std::vector<SentCommand> & sent, const double time_s, const double dead_time_s)
{
  const auto after = std::upper_bound(
    sent.begin(), sent.end(), time_s - dead_time_s,
    [](const double t, const SentCommand & command) {return t < command.time_s;});
  return after == sent.begin() ? nullptr : &*std::prev(after);
}

// 時刻 from_s〜to_s に効いていた指令（time_s + dead_time_s で効き始める）の、前後・横の速度の変化の最大
double commandChange(
  const std::vector<SentCommand> & sent, const double from_s, const double to_s,
  const double dead_time_s)
{
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -min_x;
  double min_y = min_x;
  double max_y = -min_x;
  bool found = false;
  for (std::size_t i = 0; i < sent.size(); ++i) {
    const double start = sent[i].time_s + dead_time_s;
    const double end = i + 1U < sent.size() ? sent[i + 1U].time_s + dead_time_s :
      std::numeric_limits<double>::infinity();
    if (end <= from_s || start > to_s) {
      continue;
    }
    found = true;
    min_x = std::min(min_x, sent[i].vx);
    max_x = std::max(max_x, sent[i].vx);
    min_y = std::min(min_y, sent[i].vy);
    max_y = std::max(max_y, sent[i].vy);
  }
  return found ? std::max(max_x - min_x, max_y - min_y) : std::numeric_limits<double>::infinity();
}
}  // namespace

std::optional<std::string> tractionParamsError(const TractionParams & params)
{
  if (!(params.update_period_s > 0.0) || !(params.window_s > 0.0) || !(params.horizon_s > 0.0) ||
    !(params.horizon_s < params.window_s))
  {
    return "traction の update_period_s、window_s、horizon_s は正で、horizon_s は window_s より短くしてください";
  }
  if (!(params.min_improvement_ratio > 0.0) || !(params.min_improvement_ratio <= 1.0) ||
    !(params.min_improvement_mps >= 0.0) || !(params.min_excitation_mps >= 0.0))
  {
    return "traction の min_improvement_ratio は 0 より大きく 1 以下、min_improvement_mps と "
           "min_excitation_mps は 0 以上にしてください";
  }
  if (params.raise_after < 1 || !(params.initial_friction_mps2 > 0.0)) {
    return "traction の raise_after は 1 以上、initial_friction_mps2 は正にしてください";
  }
  if (!(params.accel_ratio > 0.0) || !(params.decel_ratio > 0.0) || !(params.lateral_ratio > 0.0)) {
    return "traction の accel_ratio、decel_ratio、lateral_ratio は正にしてください";
  }
  if (!(params.min_accel_mps2 > 0.0) || !(params.min_accel_mps2 <= params.max_accel_mps2) ||
    !(params.min_decel_mps2 > 0.0) || !(params.min_decel_mps2 <= params.max_decel_mps2) ||
    !(params.min_lateral_accel_mps2 > 0.0) ||
    !(params.min_lateral_accel_mps2 <= params.max_lateral_accel_mps2))
  {
    return "traction の上限の範囲（min_* と max_*）は正で、min_* は max_* 以下にしてください";
  }
  return std::nullopt;
}

std::optional<TractionEstimate> estimateTraction(
  const std::vector<OdometrySample> & samples, const std::vector<SentCommand> & sent,
  const DriveParams & drive, const TractionParams & params, const double now_s,
  const std::array<double, 2> & known_ratio)
{
  struct Pair
  {
    std::size_t from;
    std::size_t to;
  };
  const auto earlier = [](const OdometrySample & sample, const double time_s) {
      return sample.time_s < time_s;
    };
  const auto first = std::lower_bound(
    samples.begin(), samples.end(), now_s - params.window_s, earlier);
  std::vector<Pair> pairs;
  double excitation = 0.0;
  // 指令が一定の組の終わりで、機体座標の速度と指令から、床で出せる速度の割合を最小二乗で求める
  std::size_t steady_pairs = 0U;
  double command_xx = 0.0;
  double command_x_velocity = 0.0;
  double command_yy = 0.0;
  double command_y_velocity = 0.0;
  for (auto it = first; it != samples.end() && it->time_s <= now_s; ++it) {
    const auto from = std::lower_bound(
      samples.begin(), it, it->time_s - params.horizon_s, earlier);
    if (from == it || it->time_s - from->time_s < 0.5 * params.horizon_s) {
      continue;
    }
    pairs.push_back(
      Pair{static_cast<std::size_t>(from - samples.begin()),
        static_cast<std::size_t>(it - samples.begin())});
    excitation = std::max(
      excitation, std::hypot(it->state.vx - from->state.vx, it->state.vy - from->state.vy));
    if (commandChange(sent, from->time_s - kSteadySettleS, it->time_s, drive.dead_time_s) >
      kSteadyCommandChangeMps)
    {
      continue;
    }
    const SentCommand * command = activeCommand(sent, it->time_s, drive.dead_time_s);
    if (command == nullptr) {
      continue;
    }
    const double c = std::cos(it->state.yaw);
    const double s = std::sin(it->state.yaw);
    command_xx += command->vx * command->vx;
    command_x_velocity += command->vx * (c * it->state.vx + s * it->state.vy);
    command_yy += command->vy * command->vy;
    command_y_velocity += command->vy * (-s * it->state.vx + c * it->state.vy);
    ++steady_pairs;
  }
  if (pairs.size() < kMinPairs || excitation < params.min_excitation_mps) {
    return std::nullopt;
  }

  // 割合の中心。指令が一定の組が足りて、その軸の指令が十分大きければ最小二乗で求めた値、ほかは前に分かった値
  std::array<double, 2> center = known_ratio;
  if (steady_pairs >= kMinSteadyPairs) {
    const double enough = kMinRatioCommandMps * kMinRatioCommandMps * static_cast<double>(steady_pairs);
    if (command_xx >= enough) {
      center[0] = std::clamp(command_x_velocity / command_xx, kMinRatio, 1.0);
    }
    if (command_yy >= enough) {
      center[1] = std::clamp(command_y_velocity / command_yy, kMinRatio, 1.0);
    }
  }
  // 試す割合: 中心と、その ±kRatioStep（前後・横の組み合わせ）
  std::vector<std::array<double, 2>> ratios;
  for (const double dx : {0.0, -kRatioStep, kRatioStep}) {
    for (const double dy : {0.0, -kRatioStep, kRatioStep}) {
      const std::array<double, 2> ratio{
        std::clamp(center[0] + dx, kMinRatio, 1.0), std::clamp(center[1] + dy, kMinRatio, 1.0)};
      if (std::find(ratios.begin(), ratios.end(), ratio) == ratios.end()) {
        ratios.push_back(ratio);
      }
    }
  }

  const auto rms_error = [&](const FloorTraction & floor) {
      double total = 0.0;
      for (const Pair & pair : pairs) {
        const OdometrySample & start = samples[pair.from];
        const OdometrySample & end = samples[pair.to];
        const MppiState predicted =
          predictDelayedState(start.state, start.time_s, end.time_s, sent, drive, floor);
        const double ex = predicted.vx - end.state.vx;
        const double ey = predicted.vy - end.state.vy;
        total += ex * ex + ey * ey;
      }
      return std::sqrt(total / static_cast<double>(pairs.size()));
    };

  // errors[i]: 静止摩擦の候補 i（最後は上限なし）の、最も合う割合での誤差。best_ratio[i] はその割合
  constexpr std::size_t kFrictions = kFrictionCandidates.size() + 1U;
  std::array<double, kFrictions> errors{};
  std::array<std::array<double, 2>, kFrictions> best_ratio{};
  for (std::size_t i = 0; i < kFrictions; ++i) {
    errors[i] = std::numeric_limits<double>::infinity();
    for (const auto & ratio : ratios) {
      FloorTraction floor;
      floor.static_accel_mps2 = i < kFrictionCandidates.size() ? kFrictionCandidates[i] : 0.0;
      floor.longitudinal_ratio = ratio[0];
      floor.lateral_ratio = ratio[1];
      const double error = rms_error(floor);
      if (error < errors[i]) {
        errors[i] = error;
        best_ratio[i] = ratio;
      }
    }
  }
  const std::size_t none = kFrictions - 1U;
  const double no_slip = errors[none];
  const double best = *std::min_element(errors.begin(), errors.end());
  // no_slip より明らかに小さい（滑りが見えた）か、明らかに大きい（予測では滑るが、滑らなかった）か
  const auto clearly_better = [&](const double error) {
      return error <= params.min_improvement_ratio * no_slip &&
             no_slip - error >= params.min_improvement_mps;
    };
  const auto clearly_worse = [&](const double error) {
      return no_slip <= params.min_improvement_ratio * error &&
             error - no_slip >= params.min_improvement_mps;
    };

  TractionEstimate estimate;
  estimate.pairs = static_cast<int>(pairs.size());
  estimate.no_slip_error_mps = no_slip;
  std::size_t chosen = none;
  if (clearly_better(best)) {
    // 同じくらい合う候補のうち、最も高い静止摩擦（滑りを小さく見る側）を選ぶ
    estimate.slipping = true;
    for (std::size_t i = kFrictionCandidates.size(); i-- > 0U; ) {
      if (errors[i] <= best * kTieRatio) {
        chosen = i;
        break;
      }
    }
    estimate.friction_mps2 = kFrictionCandidates[chosen];
  } else {
    // 明らかに合わない静止摩擦のうち最も高いものの、1 つ上の候補までは滑らずに出せた
    estimate.friction_mps2 = kFrictionCandidates.front();
    for (std::size_t i = kFrictionCandidates.size(); i-- > 0U; ) {
      if (clearly_worse(errors[i])) {
        estimate.friction_mps2 = i + 1U < kFrictionCandidates.size() ?
          kFrictionCandidates[i + 1U] : std::numeric_limits<double>::infinity();
        break;
      }
    }
  }
  estimate.error_mps = errors[chosen];
  estimate.longitudinal_ratio = best_ratio[chosen][0];
  estimate.lateral_ratio = best_ratio[chosen][1];
  // 滑りが見えない窓では、速度が出きらない分は割合のせいなので、割合を覚えてよい
  estimate.ratio_observed = !estimate.slipping;
  return estimate;
}

TractionState::TractionState(const double initial_friction_mps2)
: friction_(initial_friction_mps2)
{
}

double TractionState::update(
  const std::optional<TractionEstimate> & estimate, const TractionParams & params)
{
  if (!estimate.has_value()) {
    return friction_;
  }
  if (estimate->ratio_observed) {
    // 1 つの窓で動く幅は推定の側で ±0.04 に絞っている
    ratio_ = {estimate->longitudinal_ratio, estimate->lateral_ratio};
  }
  const double value = estimate->friction_mps2;
  if (estimate->slipping && value < friction_) {
    // 滑りが増えたらすぐ下げる
    friction_ = value;
    votes_ = 0;
  } else if (!estimate->slipping || value > friction_) {
    // 滑りが見えなければ、少なくとも 1 段上（床が良いかを試す）へ上げる候補にする
    const auto above = std::upper_bound(
      kFrictionCandidates.begin(), kFrictionCandidates.end(), friction_);
    const double step = above != kFrictionCandidates.end() ?
      *above : std::numeric_limits<double>::infinity();
    const double target = estimate->slipping ? value : std::max(value, step);
    if (!(target > friction_)) {
      votes_ = 0;
      return friction_;
    }
    pending_ = votes_ == 0 ? target : std::min(pending_, target);
    ++votes_;
    if (votes_ >= std::max(params.raise_after, 1)) {
      friction_ = pending_;
      votes_ = 0;
    }
  } else {
    votes_ = 0;
  }
  return friction_;
}

void TractionState::reset(const double initial_friction_mps2)
{
  friction_ = initial_friction_mps2;
  ratio_ = {1.0, 1.0};
  pending_ = 0.0;
  votes_ = 0;
}

MppiParams applyTraction(
  const MppiParams & base, const double friction_mps2, const TractionParams & params)
{
  const auto scaled = [friction_mps2](const double ratio, const double low, const double high) {
      return std::clamp(friction_mps2 * ratio, low, std::max(low, high));
    };
  MppiParams result = base;
  result.max_accel_mps2 = scaled(params.accel_ratio, params.min_accel_mps2, params.max_accel_mps2);
  result.reference_decel_mps2 =
    scaled(params.decel_ratio, params.min_decel_mps2, params.max_decel_mps2);
  if (base.curve.max_lateral_accel_mps2 > 0.0) {
    result.curve.max_lateral_accel_mps2 = scaled(
      params.lateral_ratio, params.min_lateral_accel_mps2, params.max_lateral_accel_mps2);
  }
  return result;
}

FloorTraction floorFor(const double friction_mps2)
{
  FloorTraction floor;
  floor.static_accel_mps2 = std::isfinite(friction_mps2) ? friction_mps2 : 0.0;
  return floor;
}
}  // namespace core_planning
