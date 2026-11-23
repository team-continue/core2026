// 全方向に動く機体の自己位置の EKF（Autoware の ekf_localizer と同じ役割）。
// 状態は地図の座標の位置と向き、機体の座標の速度 [x, y, yaw, vx, vy, wz]。速度一定の運動で先へ進め、
// gyro_odometer の速度 [vx, vy, wz] と、NDT の姿勢 [x, y, yaw] で直す。
// NDT の姿勢はスキャンの時刻（過去）で届くので、その時刻の状態に当て、そのあとの測定をやり直す
// （測定ごとの状態を history_s だけ残す）。外れた測定はマハラノビス距離で捨てる。
// 姿勢を続けて捨てたら、測定の姿勢に置き換えて立て直す（車輪が大きく滑ってずれたまま戻らないのを防ぐ）。
// ROS に依存しないので、ノード（ekf_localizer_node.cpp）と分けてテストする。

#ifndef CORE_LOCALIZATION__EKF_LOCALIZER_HPP_
#define CORE_LOCALIZATION__EKF_LOCALIZER_HPP_

#include <cstdint>
#include <variant>
#include <vector>

#include <Eigen/Core>

namespace core_localization
{
struct EkfParams
{
  // 速度の変わりやすさ（予測の雑音）。機体の加速度と角加速度の上限に近い値
  double accel_std_mps2{3.0};
  double angular_accel_std_radps2{6.0};
  // 位置と向きそのものの予測の雑音 [m/√s]、[rad/√s]。車輪の滑りなど、速度で表せない分
  double position_noise_std{0.02};
  double yaw_noise_std{0.01};
  // 測定を捨てるマハラノビス距離の 2 乗。Autoware の ekf_localizer の既定（3 自由度で有意水準 1e-10 ほど）。
  // 小さいと、車輪が滑ってずれたときに NDT を捨て続けて戻らない
  double pose_gate_chi2{49.5};
  double twist_gate_chi2{46.1};
  // 姿勢の分散の下限（標準偏差で）
  double min_pose_std_xy_m{0.01};
  double min_pose_std_yaw_rad{0.002};
  // 初めの速度の不確かさ（標準偏差）
  double initial_velocity_std{1.0};
  // 遅れて届く測定のために残す時間 [s]
  double history_s{1.0};
  // 姿勢の測定をこの回数続けて捨てたら、測定の姿勢に置き換える。0 で置き換えない
  int pose_rejections_before_reset{10};
};

struct PoseMeasurement
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  // [x, y, yaw] の共分散
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

struct TwistMeasurement
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  // [vx, vy, wz] の共分散
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

enum class UpdateResult : std::uint8_t
{
  kAccepted,
  // マハラノビス距離がしきい値を超えた
  kRejectedByGate,
  // 姿勢を続けて捨てたので、測定の姿勢に置き換えた
  kReset,
  // 残している状態より古い
  kTooOld,
  // まだ初めの姿勢を受けていない
  kNotInitialized,
};

class EkfLocalizer
{
public:
  using State = Eigen::Matrix<double, 6, 1>;
  using Covariance = Eigen::Matrix<double, 6, 6>;

  struct Belief
  {
    double stamp_s{0.0};
    State x{State::Zero()};
    Covariance p{Covariance::Identity()};
  };

  explicit EkfLocalizer(const EkfParams & params);

  bool initialized() const {return initialized_;}
  // 初めの姿勢で始める。速度は 0 とし、initial_velocity_std の不確かさを持たせる
  void initialize(double stamp_s, const PoseMeasurement & pose);
  // 姿勢の測定で直す。初めの姿勢を受けていなければ、それで始める
  UpdateResult updatePose(double stamp_s, const PoseMeasurement & pose);
  // 速度の測定で直す
  UpdateResult updateTwist(double stamp_s, const TwistMeasurement & twist);
  // 今の状態を stamp_s まで進めた予測。状態は変えない。今より前なら今の状態を返す
  Belief predicted(double stamp_s) const;

  const State & state() const {return current_.x;}
  const Covariance & covariance() const {return current_.p;}
  // 最後に測定を当てた（捨てた測定も含めて進めた）時刻
  double stamp() const {return current_.stamp_s;}
  const EkfParams & params() const {return params_;}

private:
  struct Measurement
  {
    std::variant<PoseMeasurement, TwistMeasurement> value;
    // true なら、姿勢を測定に置き換える（続けて捨てたあとの立て直しと、初めの姿勢）
    bool reset_pose{false};
  };
  // 測定と、それを当てた後の状態
  struct Entry
  {
    Belief belief;
    Measurement measurement;
  };

  UpdateResult update(double stamp_s, const Measurement & measurement);
  void predict(Belief & belief, double stamp_s) const;
  bool apply(Belief & belief, const Measurement & measurement) const;
  void trimHistory();

  EkfParams params_;
  bool initialized_{false};
  Belief current_;
  std::vector<Entry> history_;
  int consecutive_pose_rejections_{0};
};

// 角度を [-π, π) に直す
double normalizeAngle(double angle);
}  // namespace core_localization

#endif  // CORE_LOCALIZATION__EKF_LOCALIZER_HPP_
