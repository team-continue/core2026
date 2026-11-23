// 駆動の遅れ（無駄時間と一次遅れ）を見込んで、局所計画の始めの状態を先へ進める。ROS に依存しない。
//
// 指令を出してから機体の速度が変わるまでには、無駄時間（何も起きない時間）と一次遅れ（指令の速さへ近づく
// 鈍さ）がある。どちらも床ではなく機体（駆動と、自己位置の速度が出るまで）の値なので、走りながら推定せず、
// 滑らない床で段差の指令を出して測った値を入れる。
// 計画は、今送る指令が効き始める時刻（今 + 無駄時間）の状態から始める。それまでの間は、送ったがまだ
// 効いていない指令で状態を進める（predictDelayedState）。

#ifndef CORE_PLANNING__DRIVE_DELAY_HPP_
#define CORE_PLANNING__DRIVE_DELAY_HPP_

#include <optional>
#include <string>
#include <vector>

#include "core_planning/mppi.hpp"

namespace core_planning
{
struct DriveParams
{
  // true なら、送った指令で状態を先へ進めてから計画する。false なら前の動き（遅れの補償の時間だけ、
  // 今の速度で位置を進める）
  bool predict_delay{false};
  // 速度指令（/cmd_vel）を出してから駆動が応え始めるまでの時間 [s]
  double dead_time_s{0.0};
  // 指令の速度へ近づく一次遅れの時定数 [s]。0 なら加速度の上限だけで近づく
  double time_constant_s{0.0};
  // 駆動の加速度と角加速度の上限。送った指令で状態を進めるときに使う（計画の max_accel_mps2 とは別）
  double max_accel_mps2{5.0};
  double max_angular_accel_radps2{10.0};
};

// 床との滑り（tracking_bench と omni_force_drive の床と同じ形）。static_accel_mps2 を超える加速度を求めると
// 滑り、その kinetic_ratio 倍しか出ない（0 以下なら滑らない）。機体座標の指令の前後・横の速度のうち、床で
// 出せる割合が longitudinal_ratio と lateral_ratio
struct FloorTraction
{
  double static_accel_mps2{0.0};
  double kinetic_ratio{0.7};
  double longitudinal_ratio{1.0};
  double lateral_ratio{1.0};
};

// 送った速度指令（機体座標）。time_s は指令を出した時刻
struct SentCommand
{
  double time_s{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

// 一次遅れで、時間 dt の間に指令との差を詰める割合（1 − exp(−dt / τ)）。τ が 0 以下なら 1
double lagGain(double dt, double time_constant_s);

// 設定が正しくなければ理由を返す
std::optional<std::string> driveParamsError(const DriveParams & params);

// 時刻 measured_s に測った状態 measured を、時刻 target_s まで進める。各時刻 t には、sent（出した時刻の順）の
// うち time_s + dead_time_s <= t の最後の指令が効いているとして、速度をその指令（機体座標なので、その時の
// 向きで世界座標に直す）へ一次遅れと加速度の上限で近づけ（床が出せる加速度を超えれば滑る）、位置と向きを
// 進める。効いている指令がまだない間は、測った速度を保つ。target_s <= measured_s なら measured をそのまま
// 返す。step_s は積分の刻み [s]。
// yaw_rate があれば、向きは指令の回る速さではなくこの速さで進める。腰が TURRET の間は、指令の機体座標
// （上半身）が回る速さは指令の wz（足回りを回す速さ）ではなく、操作や自律移動が上半身を回す速さなので、
// 測った上半身の回る速さを渡す
MppiState predictDelayedState(
  const MppiState & measured, double measured_s, double target_s,
  const std::vector<SentCommand> & sent, const DriveParams & params,
  const FloorTraction & floor = FloorTraction{}, double step_s = 0.005,
  std::optional<double> yaw_rate = std::nullopt);

// before_s より前に効き始めた指令を捨てる。ただし before_s に効いている指令（その中の最後の 1 つ）は残す
void pruneSentCommands(std::vector<SentCommand> & sent, double before_s, double dead_time_s);
}  // namespace core_planning

#endif  // CORE_PLANNING__DRIVE_DELAY_HPP_
