// sim の足回りと腰の、CAN の指令から機体の動きへの換算。ROS と Gazebo に依存しないので、ノードと分けてテストする。
//
// 実機では core_hardware が /hardware/can/tx の指令をマイコンへ送る。sim では sim_hardware がその代わりに、
// body_control_node（車輪、id 0〜3）と target_angle_node（腰、id 4）の指令を読み、Gazebo の上半身（base_link）
// の速度の指令と、腰の関節の回る速さにする。
//
// 向きの約束（core_body_controller と同じ）:
// - 腰の角度 q は、上半身（base_link）から見た足回り（chassis_link）の向き（z 軸まわり、反時計回りが正）。
//   body_control_node は、上半身の座標の指令を −q だけ回して足回りの座標にしてから車輪の速さにする
//   （invert_kinematics_calc）。/hardware/joint_states の 5 番目（id 4）は q に初期角を足した値
// - 腰の指令の回る速さは dq/dt。target_angle_node は回転モードの間、足回りの回る速さ（body_omega）を先回りで
//   足し、上半身の向きの誤差には負の帰還をかける（TargetAngleController::update）。上半身の回る速さは
//   「足回りの回る速さ − dq/dt」なので、先回りで 0 になり、帰還で目標の向きへ戻る
#ifndef CORE_GAZEBO__SIM_HARDWARE_MODEL_HPP_
#define CORE_GAZEBO__SIM_HARDWARE_MODEL_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace core_gazebo::sim_hardware
{
// 車輪の寸法。body_control_node の invert_kinematics_calc と同じ値（変えるときは両方を変える）
struct WheelGeometry
{
  double wheel_radius_m{0.13 / 2.0};
  double body_width_m{0.5304};
};

// 平面の速度。座標系は使う所で決める（足回りの座標か、上半身の座標）
struct PlanarTwist
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

// CAN の指令の種類（実機の core_hardware と同じ番号と形）
inline constexpr int kWheelCount = 4;
// 車輪: id 0〜3、data = {20（速度の制御）, 車輪の回る速さ [rad/s]}
inline constexpr float kWheelVelocityMode = 20.0F;
// 腰: id 4、data = {3（速度の制御）, 腰の回る速さ dq/dt [rad/s]}
inline constexpr int kWaistId = 4;
inline constexpr float kWaistVelocityMode = 3.0F;

enum class CommandKind
{
  kWheel,
  kWaist,
};

struct DecodedCommand
{
  CommandKind kind{CommandKind::kWheel};
  // kWheel のときの車輪の番号（0〜3）
  int wheel{0};
  double value{0.0};
};

// CAN の 1 つの指令を読む。車輪と腰の速度の指令でなければ（砲塔・射出など、ほかの番号やモード）nullopt
inline std::optional<DecodedCommand> decode(const int id, const std::vector<float> & data)
{
  if (data.size() < 2U || !std::isfinite(data[1])) {
    return std::nullopt;
  }
  if (id >= 0 && id < kWheelCount && data[0] == kWheelVelocityMode) {
    return DecodedCommand{CommandKind::kWheel, id, static_cast<double>(data[1])};
  }
  if (id == kWaistId && data[0] == kWaistVelocityMode) {
    return DecodedCommand{CommandKind::kWaist, 0, static_cast<double>(data[1])};
  }
  return std::nullopt;
}

namespace detail
{
// 車輪の位置の、回転の項の腕の長さ（body_control_node の SQRT2 * BODY_WIDTH / 2）
inline double turnArm(const WheelGeometry & geometry)
{
  return std::sqrt(2.0) * geometry.body_width_m / 2.0;
}
}  // namespace detail

// 足回りの座標の速度から、車輪の回る速さ。body_control_node の invert_kinematics_calc の、
// 足回りの座標に回した後の部分と同じ式（試験で往復を確かめるために持つ）
inline std::array<double, kWheelCount> wheelsFromChassisTwist(
  const PlanarTwist & chassis, const WheelGeometry & geometry)
{
  const double c = std::cos(M_PI / 4.0);
  const double s = std::sin(M_PI / 4.0);
  const double turn = detail::turnArm(geometry) * chassis.wz;
  const double r = geometry.wheel_radius_m;
  return {
    -(chassis.vx * c - chassis.vy * s - turn) / r,
    -(chassis.vx * c + chassis.vy * s - turn) / r,
    -(-chassis.vx * c + chassis.vy * s - turn) / r,
    -(-chassis.vx * c - chassis.vy * s - turn) / r,
  };
}

// 車輪の回る速さ（id 0〜3 の並び）から、足回りの座標の速度。wheelsFromChassisTwist の逆（4 本の車輪の式を
// 最小二乗で解く。車輪が滑らず 4 本がそろっていれば、ちょうど元に戻る）
inline PlanarTwist chassisTwistFromWheels(
  const std::array<double, kWheelCount> & wheels, const WheelGeometry & geometry)
{
  // u_i = −r w_i は、車輪の向きの速さ
  std::array<double, kWheelCount> u{};
  for (std::size_t i = 0; i < u.size(); ++i) {
    u[i] = -geometry.wheel_radius_m * wheels[i];
  }
  const double c = std::cos(M_PI / 4.0);
  const double s = std::sin(M_PI / 4.0);
  return {
    (u[0] + u[1] - u[2] - u[3]) / (4.0 * c),
    (-u[0] + u[1] + u[2] - u[3]) / (4.0 * s),
    -(u[0] + u[1] + u[2] + u[3]) / (4.0 * detail::turnArm(geometry)),
  };
}

// 足回りの座標の速度と腰の角度 q・回る速さ dq/dt から、上半身（base_link）の座標の速度。
// 並進は足回りの座標を +q だけ回して上半身の座標にする。上半身の回る速さは足回りの回る速さ − dq/dt
inline PlanarTwist upperTwist(
  const PlanarTwist & chassis, const double waist_angle, const double waist_rate)
{
  const double c = std::cos(waist_angle);
  const double s = std::sin(waist_angle);
  return {
    c * chassis.vx - s * chassis.vy,
    s * chassis.vx + c * chassis.vy,
    chassis.wz - waist_rate,
  };
}
}  // namespace core_gazebo::sim_hardware

#endif  // CORE_GAZEBO__SIM_HARDWARE_MODEL_HPP_
