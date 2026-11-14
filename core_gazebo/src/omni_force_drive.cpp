// 全方位移動の速度指令を、力とトルクに変換して機体に加える Gazebo プラグイン。
//
// 標準の VelocityControl は速度を毎ステップ直接書き換えるので、物理エンジンの加速度に
// 現れず、IMU の水平加速度が常に 0 になる。力で動かすと IMU に実際の加速度が出る。
// 指令は機体座標系の vx, vy, wz。機体側の制御を想定して、加速度に上限をかける。
// <dead_time>: 指令を受けてから効き始めるまでの時間（駆動の無駄時間）[s]。省略すると 0（受けたステップから効く。
// 前の動き）。計算は command_delay.hpp（試験は test_command_delay.cpp）
//
// 車輪と床の接触は再現しないので、機体が床に置かれている状態は、高さと傾きを力で保って
// 再現する。これを弱くすると、低い柵に押し付けたときに機体が前転して飛び越える。
//
// 床との滑り（どれも省略できる。省略すると滑らない、前の動き）。計算は slip_model.hpp（試験は
// test_slip_model.cpp）。
// - <longitudinal_ratio> <lateral_ratio> <yaw_ratio>: 指令の前後・横・回転の速度のうち、床で出せる割合
//   （1 で指令どおり）。全方位の車輪は横に動くときに滑りやすい
// - <time_constant>: 目標の速度との差を埋める時定数 [s]（0 なら 1 ステップで埋める）。駆動の上限
//   <max_linear_accel> で抑える
// - <static_friction_accel>: 床が出せる水平の加速度（静止摩擦、摩擦円）[m/s2]。0 なら滑らない。
//   これを超える加速度を求めると滑り、<kinetic_friction_accel>（動摩擦）しか出ない
// - <noise_ratio> <noise_time>: 目標の速度に足す、時間相関のある雑音（床のむら）の大きさ（目標の速さに対する
//   割合、標準偏差。止まっていれば 0）と相関時間 [s]
// - <seed>: 雑音の乱数の種

#include <gz/msgs/twist.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "core_gazebo/command_delay.hpp"
#include "core_gazebo/slip_model.hpp"

namespace core_gazebo
{
namespace
{
using gz::math::Vector3d;

struct BodyVelocity
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};
}  // namespace

class OmniForceDrive final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "OmniForceDrive はモデルに付けてください。\n";
      return;
    }
    const gz::sim::Link link(model.CanonicalLink(ecm));
    if (!link.Valid(ecm)) {
      gzerr << "OmniForceDrive: モデル [" << model.Name(ecm) << "] に基準リンクがありません。\n";
      return;
    }
    const auto * inertial = ecm.Component<gz::sim::components::Inertial>(link.Entity());
    if (inertial == nullptr) {
      gzerr << "OmniForceDrive: リンクに慣性がありません。\n";
      return;
    }

    const std::string topic = sdf->Get<std::string>("topic", "").first;
    const double max_linear_accel = sdf->Get<double>("max_linear_accel", 0.0).first;
    const double max_angular_accel = sdf->Get<double>("max_angular_accel", 0.0).first;
    if (topic.empty() || max_linear_accel <= 0.0 || max_angular_accel <= 0.0) {
      gzerr << "OmniForceDrive: <topic>、正の <max_linear_accel> と <max_angular_accel> が必要です。\n";
      return;
    }
    if (!node_.Subscribe(topic, &OmniForceDrive::onCommand, this)) {
      gzerr << "OmniForceDrive: トピック [" << topic << "] を購読できません。\n";
      return;
    }

    const double dead_time = sdf->Get<double>("dead_time", 0.0).first;
    if (!(dead_time >= 0.0)) {
      gzerr << "OmniForceDrive: <dead_time>（駆動の無駄時間）は 0 以上にしてください: " << dead_time << "\n";
      return;
    }

    slip::SlipParams slip;
    slip.longitudinal_ratio = sdf->Get<double>("longitudinal_ratio", 1.0).first;
    slip.lateral_ratio = sdf->Get<double>("lateral_ratio", 1.0).first;
    slip.yaw_ratio = sdf->Get<double>("yaw_ratio", 1.0).first;
    slip.time_constant_s = sdf->Get<double>("time_constant", 0.0).first;
    slip.static_friction_accel = sdf->Get<double>("static_friction_accel", 0.0).first;
    // 0 なら静止摩擦と同じ（slip::kineticAccel）
    slip.kinetic_friction_accel = sdf->Get<double>("kinetic_friction_accel", 0.0).first;
    slip.noise_ratio = sdf->Get<double>("noise_ratio", 0.0).first;
    slip.noise_time_s = sdf->Get<double>("noise_time", 0.5).first;
    if (const auto error = slip::validate(slip)) {
      gzerr << "OmniForceDrive: 滑りの設定が正しくありません: " << *error << "\n";
      return;
    }
    random_.seed(static_cast<std::mt19937::result_type>(sdf->Get<int>("seed", 0).first));
    gzmsg << "OmniForceDrive: 無駄時間 " << dead_time << " s、滑り 前後 " << slip.longitudinal_ratio <<
      "、横 " << slip.lateral_ratio << "、回転 " << slip.yaw_ratio << "、時定数 " << slip.time_constant_s <<
      " s、摩擦 " << slip.static_friction_accel << "/" << slip::kineticAccel(slip) << " m/s2、雑音 " <<
      slip.noise_ratio << "（" << slip.noise_time_s << " s）\n";

    link.EnableVelocityChecks(ecm);
    link_ = link;
    mass_ = inertial->Data().MassMatrix().Mass();
    max_linear_accel_ = max_linear_accel;
    max_angular_accel_ = max_angular_accel;
    slip_ = slip;
    delayed_ = DelayedCommand<BodyVelocity>(dead_time);
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (!link_.has_value() || info.paused) {
      return;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    const auto pose = link_->WorldPose(ecm);
    const auto linear_velocity = link_->WorldLinearVelocity(ecm);
    const auto angular_velocity = link_->WorldAngularVelocity(ecm);
    const auto inertia = link_->WorldInertiaMatrix(ecm);
    if (dt <= 0.0 || !pose || !linear_velocity || !angular_velocity || !inertia) {
      return;
    }
    if (!hold_height_.has_value()) {
      hold_height_ = pose->Pos().Z();
    }

    // 受けた指令は、このステップの時刻に受けたとして、無駄時間の後から効かせる
    const double now_s = std::chrono::duration<double>(info.simTime).count();
    for (const BodyVelocity & received : takeReceived()) {
      delayed_.push(now_s, received);
    }
    const BodyVelocity command = delayed_.update(now_s);
    const double yaw = pose->Rot().Yaw();

    // 水平方向と yaw は、指令（床で出せる割合をかけ、床のむらを足したもの）との差を時定数で埋めようと
    // して、加速度の上限で抑える。実際の速度との差から毎回求めるので、壁に押し付けても力が溜まらない。
    // 床が出せる加速度を超えたら滑り、動摩擦の加速度しか出ない
    const slip::PlanarAccel planar = slip::accelerationFor(
      slip_, max_linear_accel_, max_angular_accel_, yaw, linear_velocity->X(), linear_velocity->Y(),
      angular_velocity->Z(), command.vx, command.vy, command.wz, noise_x_, noise_y_, dt);
    noise_x_ = slip::updateNoise(noise_x_, slip_, dt, normal_(random_));
    noise_y_ = slip::updateNoise(noise_y_, slip_, dt, normal_(random_));

    // 高さと傾きは床に置かれている状態を表すので、上限をかけずに元へ戻す
    const double target_vz = -kHoldGainPerS * (pose->Pos().Z() - *hold_height_);
    const Vector3d level_axis = pose->Rot().RotateVector(Vector3d::UnitZ).Cross(Vector3d::UnitZ);

    const Vector3d linear_accel(planar.ax, planar.ay, (target_vz - linear_velocity->Z()) / dt);
    const Vector3d angular_accel(
      (kHoldGainPerS * level_axis.X() - angular_velocity->X()) / dt,
      (kHoldGainPerS * level_axis.Y() - angular_velocity->Y()) / dt,
      planar.yaw_accel);

    link_->AddWorldForce(ecm, mass_ * linear_accel);
    link_->AddWorldWrench(ecm, Vector3d::Zero, (*inertia) * angular_accel);
  }

private:
  void onCommand(const gz::msgs::Twist & msg)
  {
    const std::lock_guard<std::mutex> lock(command_mutex_);
    received_.push_back(BodyVelocity{msg.linear().x(), msg.linear().y(), msg.angular().z()});
  }

  // 前のステップから受けた指令（受けた順）
  std::vector<BodyVelocity> takeReceived()
  {
    std::vector<BodyVelocity> received;
    const std::lock_guard<std::mutex> lock(command_mutex_);
    received.swap(received_);
    return received;
  }

  // 高さと傾きのずれを元へ戻す速さ。ずれの約 1/e まで 0.05 秒で戻る
  static constexpr double kHoldGainPerS = 20.0;

  gz::transport::Node node_;
  std::optional<gz::sim::Link> link_;
  std::optional<double> hold_height_;
  double mass_{0.0};
  double max_linear_accel_{0.0};
  double max_angular_accel_{0.0};
  slip::SlipParams slip_;
  std::mt19937 random_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  double noise_x_{0.0};
  double noise_y_{0.0};
  std::mutex command_mutex_;
  std::vector<BodyVelocity> received_;
  DelayedCommand<BodyVelocity> delayed_;
};
}  // namespace core_gazebo

GZ_ADD_PLUGIN(
  core_gazebo::OmniForceDrive,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
