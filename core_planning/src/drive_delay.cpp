#include "core_planning/drive_delay.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>

namespace core_planning
{
namespace
{
// 刻みがこれより短くなるのは、指令が変わる時刻の丸めだけ
constexpr double kMinStepS = 1e-9;
}  // namespace

double lagGain(const double dt, const double time_constant_s)
{
  if (!(time_constant_s > 0.0)) {
    return 1.0;
  }
  return 1.0 - std::exp(-std::max(dt, 0.0) / time_constant_s);
}

std::optional<std::string> driveParamsError(const DriveParams & params)
{
  if (!(params.dead_time_s >= 0.0) || !(params.time_constant_s >= 0.0)) {
    return "駆動の無駄時間（dead_time_s）と時定数（time_constant_s）は 0 以上にしてください";
  }
  if (!(params.max_accel_mps2 > 0.0) || !(params.max_angular_accel_radps2 > 0.0)) {
    return "駆動の加速度の上限（max_accel_mps2、max_angular_accel_radps2）は正にしてください";
  }
  return std::nullopt;
}

MppiState predictDelayedState(
  const MppiState & measured, const double measured_s, const double target_s,
  const std::vector<SentCommand> & sent, const DriveParams & params, const FloorTraction & floor,
  const double step_s, const std::optional<double> yaw_rate)
{
  MppiState state = measured;
  if (yaw_rate.has_value()) {
    state.wz = *yaw_rate;
  }
  if (!(target_s > measured_s)) {
    return state;
  }
  const double step = step_s > 0.0 ? step_s : 0.005;
  // sent[next] がまだ効いていない最初の指令。active はいま効いている指令
  std::size_t next = 0U;
  const SentCommand * active = nullptr;
  const auto advance = [&](const double t) {
      while (next < sent.size() && sent[next].time_s + params.dead_time_s <= t) {
        active = &sent[next];
        ++next;
      }
    };

  double t = measured_s;
  advance(t);
  while (target_s - t > kMinStepS) {
    // 指令が変わる時刻で刻みを切る
    double h = std::min(step, target_s - t);
    if (next < sent.size()) {
      h = std::min(h, std::max(sent[next].time_s + params.dead_time_s - t, kMinStepS));
    }
    if (active != nullptr) {
      const double c = std::cos(state.yaw);
      const double s = std::sin(state.yaw);
      const double bx = active->vx * floor.longitudinal_ratio;
      const double by = active->vy * floor.lateral_ratio;
      const double ux = c * bx - s * by;
      const double uy = s * bx + c * by;
      const double gain = lagGain(h, params.time_constant_s);
      double dvx = (ux - state.vx) * gain;
      double dvy = (uy - state.vy) * gain;
      const double max_dv = params.max_accel_mps2 * h;
      const double dv = std::hypot(dvx, dvy);
      if (dv > max_dv) {
        dvx *= max_dv / dv;
        dvy *= max_dv / dv;
      }
      // 床が出せる加速度を超えたら滑り、動摩擦の分しか出ない
      const double demand = std::min(dv, max_dv) / h;
      if (floor.static_accel_mps2 > 0.0 && demand > floor.static_accel_mps2) {
        const double kinetic = floor.kinetic_ratio * floor.static_accel_mps2;
        dvx *= kinetic / demand;
        dvy *= kinetic / demand;
      }
      state.vx += dvx;
      state.vy += dvy;
      if (!yaw_rate.has_value()) {
        const double max_dw = params.max_angular_accel_radps2 * h;
        state.wz += std::clamp((active->wz - state.wz) * gain, -max_dw, max_dw);
      }
    }
    state.x += state.vx * h;
    state.y += state.vy * h;
    state.yaw += state.wz * h;
    t += h;
    advance(t);
  }
  return state;
}

void pruneSentCommands(std::vector<SentCommand> & sent, const double before_s, const double dead_time_s)
{
  // before_s までに効き始めた指令の数。最後の 1 つは before_s に効いているので残す
  const auto started = std::partition_point(
    sent.begin(), sent.end(),
    [&](const SentCommand & command) {return command.time_s + dead_time_s <= before_s;});
  const auto count = std::distance(sent.begin(), started);
  if (count > 1) {
    sent.erase(sent.begin(), std::prev(started));
  }
}
}  // namespace core_planning
