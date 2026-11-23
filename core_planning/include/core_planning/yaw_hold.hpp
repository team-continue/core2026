// 速度指令の並進の向きを、IMU の速い周期で世界の座標に保つ。ROS に依存しない。
//
// 局所計画（MPPI）は、指令の並進の速さを 1 周期（0.1 秒）の間、世界の座標で一定と見込んで計画し、出すときに
// 機体の座標へ直す。足回りは機体の座標の指令を次の指令まで保つので、その間に機体が回ると、回った分だけ
// 世界の座標での向きがずれる（Gazebo で、回る速さ 1rad/s あたり約 5°）。
// 指令を受けた時の向きを覚え、IMU の角速度を積んだ今の向きとの差だけ、並進を逆に回して出し直す。
// 回る速さ（wz）はそのまま。
// 指令を作った側が、機体の座標に直した時刻（reference_s）を渡せば、受けた時ではなくその時の向きから回った分を
// 戻す（作ってから届くまでの遅れの間に回った分も戻せる）。そのため、積んだ向きを時刻つきで覚えておく。

#ifndef CORE_PLANNING__YAW_HOLD_HPP_
#define CORE_PLANNING__YAW_HOLD_HPP_

#include <deque>
#include <optional>
#include <utility>

namespace core_planning
{
// 機体の座標の速度指令
struct BodyVelocity
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct YawHoldParams
{
  // 指令を受けてからこの時間を過ぎたら出し直さない [s]（足回りの cmd_vel_timeout_sec と同じ）
  double timeout_s{0.2};
  // 逆に回す角度の上限 [rad]。これを超えて回ったら、上限まで回して出す
  double max_correction_rad{0.5};
  // IMU の時刻の間隔がこれを超えたら、その間は積まない（抜けや時計の戻り）[s]
  double max_imu_gap_s{0.05};
  // 指令を作った時刻（reference_s）として受ける、受けた時刻からの古さの上限 [s]。これより古い・新しい時刻は使わず、
  // 受けた時の向きにする。積んだ向きもこの長さだけ覚えておく
  double max_reference_age_s{0.5};
};

// 値が範囲外なら std::invalid_argument を投げる
void validateYawHoldParams(const YawHoldParams & params);

// 指令を受けた時の向き yaw_at_command から今の向き yaw_now までに回った分だけ、並進を逆に回す。
// 回った角度は ±max_correction_rad に収める。回る速さはそのまま
BodyVelocity holdWorldDirection(
  const BodyVelocity & command, double yaw_at_command, double yaw_now, double max_correction_rad);

// 指令と IMU を受けて、今出す指令を決める
class YawHold
{
public:
  explicit YawHold(const YawHoldParams & params);

  // IMU の機体の座標の z 軸まわりの角速度 [rad/s] を、時刻 stamp_s の順に受ける
  void onYawRate(double stamp_s, double yaw_rate_radps);

  // 指令を時刻 time_s に受ける（time_s は output と同じ時計）。reference_s があれば、指令を作った側が機体の座標に
  // 直した時刻（IMU と同じ時計）。覚えている向きの範囲にあれば、その時の向きを基準にする
  void onCommand(
    const BodyVelocity & command, double time_s, std::optional<double> reference_s = std::nullopt);

  // 時刻 stamp_s に積んでいた向き（覚えている範囲で、前後の IMU の間を直線でつなぐ）。範囲の外なら nullopt
  std::optional<double> yawAt(double stamp_s) const;

  // IMU を一度でも受けたか（受けていなければ、指令をそのまま出す）
  bool hasYaw() const {return yaw_stamp_s_.has_value();}

  // 時刻 now_s に出す指令。指令をまだ受けていない、または timeout_s を過ぎたら nullopt
  std::optional<BodyVelocity> output(double now_s) const;

private:
  YawHoldParams params_;
  // IMU の角速度を積んだ向き（始まりを 0 とした相対の値）
  double yaw_{0.0};
  std::optional<double> yaw_stamp_s_;
  // 積んだ向きの記録（時刻の順）。max_reference_age_s より古いものは捨てる
  std::deque<std::pair<double, double>> history_;
  std::optional<BodyVelocity> command_;
  double command_time_s_{0.0};
  double yaw_at_command_{0.0};
};
}  // namespace core_planning

#endif  // CORE_PLANNING__YAW_HOLD_HPP_
