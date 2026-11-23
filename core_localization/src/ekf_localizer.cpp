#include "core_localization/ekf_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include <Eigen/Dense>

namespace core_localization
{
namespace
{
constexpr int kStateSize = 6;
constexpr int kX = 0;
constexpr int kY = 1;
constexpr int kYaw = 2;
constexpr int kVx = 3;
constexpr int kVy = 4;
constexpr int kWz = 5;

// 予測の 1 歩の上限 [s]。長い間を一度に進めると、回りながら進む分を取りこぼす
constexpr double kMaxPredictStepS = 0.01;
// 予測の歩数の上限。時計が大きく跳んだときに止まらないようにする
constexpr int kMaxPredictSteps = 1000;

using Matrix36 = Eigen::Matrix<double, 3, kStateSize>;
using Matrix63 = Eigen::Matrix<double, kStateSize, 3>;

// 測定のマハラノビス距離の 2 乗がしきい値以下なら、EKF の更新を当てる（共分散は Joseph 形式）
bool kalmanUpdate(
  EkfLocalizer::State & x, EkfLocalizer::Covariance & p, const Matrix36 & h,
  const Eigen::Vector3d & innovation, const Eigen::Matrix3d & r, const double gate_chi2)
{
  const Eigen::Matrix3d s = h * p * h.transpose() + r;
  const Eigen::LDLT<Eigen::Matrix3d> s_ldlt(s);
  if (s_ldlt.info() != Eigen::Success) {
    return false;
  }
  const double distance2 = innovation.dot(s_ldlt.solve(innovation));
  if (!std::isfinite(distance2) || distance2 > gate_chi2) {
    return false;
  }
  const Matrix63 k = (s_ldlt.solve(h * p)).transpose();
  x += k * innovation;
  const EkfLocalizer::Covariance i_kh = EkfLocalizer::Covariance::Identity() - k * h;
  p = i_kh * p * i_kh.transpose() + k * r * k.transpose();
  return true;
}

Eigen::Matrix3d poseCovarianceWithFloor(const Eigen::Matrix3d & covariance, const EkfParams & params)
{
  Eigen::Matrix3d r = 0.5 * (covariance + covariance.transpose());
  const double xy_floor = params.min_pose_std_xy_m * params.min_pose_std_xy_m;
  const double yaw_floor = params.min_pose_std_yaw_rad * params.min_pose_std_yaw_rad;
  r(0, 0) = std::max(r(0, 0), xy_floor);
  r(1, 1) = std::max(r(1, 1), xy_floor);
  r(2, 2) = std::max(r(2, 2), yaw_floor);
  return r;
}

// 姿勢を測定に置き換える。速度とその不確かさは残し、姿勢と速度の相関は捨てる
void resetPose(EkfLocalizer::Belief & belief, const PoseMeasurement & pose, const EkfParams & params)
{
  belief.x(kX) = pose.x;
  belief.x(kY) = pose.y;
  belief.x(kYaw) = normalizeAngle(pose.yaw);
  belief.p.topRightCorner<3, 3>().setZero();
  belief.p.bottomLeftCorner<3, 3>().setZero();
  belief.p.topLeftCorner<3, 3>() = poseCovarianceWithFloor(pose.covariance, params);
}

// 速度一定のまま dt だけ進める（機体の座標の速度を、地図の座標に回して足す）
void predictStep(EkfLocalizer::Belief & belief, const double dt, const EkfParams & params)
{
  EkfLocalizer::State & x = belief.x;
  const double c = std::cos(x(kYaw));
  const double s = std::sin(x(kYaw));
  const double vx = x(kVx);
  const double vy = x(kVy);

  EkfLocalizer::Covariance f = EkfLocalizer::Covariance::Identity();
  f(kX, kYaw) = -(vx * s + vy * c) * dt;
  f(kX, kVx) = c * dt;
  f(kX, kVy) = -s * dt;
  f(kY, kYaw) = (vx * c - vy * s) * dt;
  f(kY, kVx) = s * dt;
  f(kY, kVy) = c * dt;
  f(kYaw, kWz) = dt;

  x(kX) += (vx * c - vy * s) * dt;
  x(kY) += (vx * s + vy * c) * dt;
  x(kYaw) = normalizeAngle(x(kYaw) + x(kWz) * dt);

  EkfLocalizer::Covariance q = EkfLocalizer::Covariance::Zero();
  q(kX, kX) = params.position_noise_std * params.position_noise_std * dt;
  q(kY, kY) = q(kX, kX);
  q(kYaw, kYaw) = params.yaw_noise_std * params.yaw_noise_std * dt;
  q(kVx, kVx) = params.accel_std_mps2 * params.accel_std_mps2 * dt;
  q(kVy, kVy) = q(kVx, kVx);
  q(kWz, kWz) = params.angular_accel_std_radps2 * params.angular_accel_std_radps2 * dt;

  belief.p = f * belief.p * f.transpose() + q;
}
}  // namespace

double normalizeAngle(const double angle)
{
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

EkfLocalizer::EkfLocalizer(const EkfParams & params)
: params_(params)
{
}

void EkfLocalizer::initialize(const double stamp_s, const PoseMeasurement & pose)
{
  current_.stamp_s = stamp_s;
  current_.x = State::Zero();
  current_.p = Covariance::Zero();
  const double velocity_variance = params_.initial_velocity_std * params_.initial_velocity_std;
  current_.p(kVx, kVx) = velocity_variance;
  current_.p(kVy, kVy) = velocity_variance;
  current_.p(kWz, kWz) = velocity_variance;
  resetPose(current_, pose, params_);
  history_.clear();
  history_.push_back(Entry{current_, Measurement{pose, true}});
  consecutive_pose_rejections_ = 0;
  initialized_ = true;
}

void EkfLocalizer::predict(Belief & belief, const double stamp_s) const
{
  const double duration = stamp_s - belief.stamp_s;
  if (!(duration > 0.0)) {
    return;
  }
  // 周期の丸め（0.02 / 0.01 が 2.0000000000000018 になるなど）で歩数が 1 つ増えないよう、わずかに切り下げる。
  // 歩数が変わると、同じ時間でも予測の雑音の位置への入り方が変わる
  const int steps = std::clamp(
    static_cast<int>(std::ceil(duration / kMaxPredictStepS - 1e-6)), 1, kMaxPredictSteps);
  const double dt = duration / steps;
  for (int step = 0; step < steps; ++step) {
    predictStep(belief, dt, params_);
  }
  belief.stamp_s = stamp_s;
}

bool EkfLocalizer::apply(Belief & belief, const Measurement & measurement) const
{
  if (const auto * pose = std::get_if<PoseMeasurement>(&measurement.value)) {
    if (measurement.reset_pose) {
      resetPose(belief, *pose, params_);
      return true;
    }
    Matrix36 h = Matrix36::Zero();
    h(0, kX) = 1.0;
    h(1, kY) = 1.0;
    h(2, kYaw) = 1.0;
    const Eigen::Vector3d innovation(
      pose->x - belief.x(kX), pose->y - belief.x(kY), normalizeAngle(pose->yaw - belief.x(kYaw)));
    if (!kalmanUpdate(
        belief.x, belief.p, h, innovation, poseCovarianceWithFloor(pose->covariance, params_),
        params_.pose_gate_chi2))
    {
      return false;
    }
    belief.x(kYaw) = normalizeAngle(belief.x(kYaw));
    return true;
  }
  const auto & twist = std::get<TwistMeasurement>(measurement.value);
  Matrix36 h = Matrix36::Zero();
  h(0, kVx) = 1.0;
  h(1, kVy) = 1.0;
  h(2, kWz) = 1.0;
  const Eigen::Vector3d innovation(
    twist.vx - belief.x(kVx), twist.vy - belief.x(kVy), twist.wz - belief.x(kWz));
  const Eigen::Matrix3d r = 0.5 * (twist.covariance + twist.covariance.transpose());
  return kalmanUpdate(belief.x, belief.p, h, innovation, r, params_.twist_gate_chi2);
}

EkfLocalizer::Belief EkfLocalizer::predicted(const double stamp_s) const
{
  Belief belief = current_;
  predict(belief, stamp_s);
  return belief;
}

UpdateResult EkfLocalizer::updatePose(const double stamp_s, const PoseMeasurement & pose)
{
  if (!initialized_) {
    initialize(stamp_s, pose);
    return UpdateResult::kAccepted;
  }
  const UpdateResult result = update(stamp_s, Measurement{pose, false});
  if (result == UpdateResult::kAccepted) {
    consecutive_pose_rejections_ = 0;
    return result;
  }
  if (result != UpdateResult::kRejectedByGate) {
    return result;
  }
  ++consecutive_pose_rejections_;
  if (
    params_.pose_rejections_before_reset <= 0 ||
    consecutive_pose_rejections_ < params_.pose_rejections_before_reset)
  {
    return result;
  }
  consecutive_pose_rejections_ = 0;
  // 捨てた直後なので、同じ時刻の状態が残っている（kTooOld にはならない）
  update(stamp_s, Measurement{pose, true});
  return UpdateResult::kReset;
}

UpdateResult EkfLocalizer::updateTwist(const double stamp_s, const TwistMeasurement & twist)
{
  if (!initialized_) {
    return UpdateResult::kNotInitialized;
  }
  return update(stamp_s, Measurement{twist, false});
}

UpdateResult EkfLocalizer::update(const double stamp_s, const Measurement & measurement)
{
  if (stamp_s >= current_.stamp_s) {
    // 今より新しい（遅れのない）測定: 今の状態を進めて当てる。捨てても、時刻は進める
    Belief belief = current_;
    predict(belief, stamp_s);
    const bool accepted = apply(belief, measurement);
    current_ = belief;
    if (!accepted) {
      return UpdateResult::kRejectedByGate;
    }
    history_.push_back(Entry{belief, measurement});
    trimHistory();
    return UpdateResult::kAccepted;
  }

  // 遅れて届いた測定: その時刻より前の最後の状態から当て直し、そのあとの測定をやり直す
  const auto after = std::upper_bound(
    history_.begin(), history_.end(), stamp_s,
    [](const double stamp, const Entry & entry) {return stamp < entry.belief.stamp_s;});
  if (after == history_.begin()) {
    return UpdateResult::kTooOld;
  }
  Belief belief = std::prev(after)->belief;
  predict(belief, stamp_s);
  if (!apply(belief, measurement)) {
    return UpdateResult::kRejectedByGate;
  }
  const auto inserted = history_.insert(after, Entry{belief, measurement});
  for (auto entry = std::next(inserted); entry != history_.end(); ++entry) {
    Belief replayed = std::prev(entry)->belief;
    predict(replayed, entry->belief.stamp_s);
    // やり直しで外れた測定は、予測だけにする
    apply(replayed, entry->measurement);
    entry->belief = replayed;
  }
  const double now = current_.stamp_s;
  current_ = history_.back().belief;
  predict(current_, now);
  trimHistory();
  return UpdateResult::kAccepted;
}

void EkfLocalizer::trimHistory()
{
  const double oldest = current_.stamp_s - params_.history_s;
  const auto keep_from = std::find_if(
    history_.begin(), history_.end(),
    [oldest](const Entry & entry) {return entry.belief.stamp_s >= oldest;});
  // 範囲の始まりの直前の状態から当て直せるよう、範囲の外でも 1 つは残す
  if (keep_from != history_.begin()) {
    history_.erase(history_.begin(), std::prev(keep_from));
  }
}
}  // namespace core_localization
