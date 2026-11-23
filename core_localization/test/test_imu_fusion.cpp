#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "core_localization/imu_fusion.hpp"

namespace
{
using core_localization::imu_fusion::accumulateUnitCheck;
using core_localization::imu_fusion::Contribution;
using core_localization::imu_fusion::estimateTimeOffset;
using core_localization::imu_fusion::Extrinsic;
using core_localization::imu_fusion::FusedSample;
using core_localization::imu_fusion::FusionParams;
using core_localization::imu_fusion::ImuFusion;
using core_localization::imu_fusion::ImuSample;
using core_localization::imu_fusion::interpolate;
using core_localization::imu_fusion::inverseVarianceMean;
using core_localization::imu_fusion::isSaturated;
using core_localization::imu_fusion::kStandardGravity;
using core_localization::imu_fusion::Measurement;
using core_localization::imu_fusion::OffsetEstimationParams;
using core_localization::imu_fusion::OffsetStatus;
using core_localization::imu_fusion::RateSample;
using core_localization::imu_fusion::transferAcceleration;
using core_localization::imu_fusion::UnitCheckState;
using core_localization::imu_fusion::UnitCheckStatus;
using core_localization::imu_fusion::validateParams;

// 200Hz
constexpr double kPeriodS = 0.005;
const Eigen::Vector3d kGravity(0.0, 0.0, kStandardGravity);

using VectorOfTime = std::function<Eigen::Vector3d(double)>;

// 単位の確認を省いた設定。確認そのものは別のテストで見る
FusionParams testParams()
{
  FusionParams params;
  params.unit_check_samples = 0U;
  return params;
}

ImuSample makeSample(const double stamp_s, const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel)
{
  ImuSample sample;
  sample.stamp_s = stamp_s;
  sample.gyro = gyro;
  sample.accel = accel;
  return sample;
}

void append(std::vector<FusedSample> & all, const std::vector<FusedSample> & more)
{
  all.insert(all.end(), more.begin(), more.end());
}

// 同じ時刻に主と補助を 1 つずつ、主から先に入れる。受信の時刻は測定の時刻と同じとする
std::vector<FusedSample> runBoth(
  ImuFusion & fusion, const int count, const VectorOfTime & primary_gyro,
  const VectorOfTime & primary_accel, const VectorOfTime & secondary_gyro,
  const VectorOfTime & secondary_accel, const double start_s = 0.0)
{
  std::vector<FusedSample> outputs;
  for (int k = 0; k < count; ++k) {
    const double t = start_s + k * kPeriodS;
    append(outputs, fusion.addPrimary(makeSample(t, primary_gyro(t), primary_accel(t)), t));
    append(outputs, fusion.addSecondary(makeSample(t, secondary_gyro(t), secondary_accel(t)), t));
  }
  return outputs;
}

VectorOfTime constant(const Eigen::Vector3d & value)
{
  return [value](double) {return value;};
}

void expectVectorNear(const Eigen::Vector3d & actual, const Eigen::Vector3d & expected, const double tolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

TEST(TransferAcceleration, RemovesCentripetalAccelerationOfPureRotation)
{
  // 主を通る z 軸まわりに 2rad/s で回る。補助は主から -x に 0.5m なので、軸へ向かう 2m/s2 を受ける
  const Eigen::Vector3d gyro(0.0, 0.0, 2.0);
  const Eigen::Vector3d lever_arm(0.5, 0.0, 0.0);
  const Eigen::Vector3d at_secondary(2.0, 0.0, 0.0);
  expectVectorNear(
    transferAcceleration(at_secondary, gyro, Eigen::Vector3d::Zero(), lever_arm),
    Eigen::Vector3d::Zero(), 1e-12);
}

TEST(TransferAcceleration, RemovesTangentialAccelerationOfAngularAcceleration)
{
  // 静止から 3rad/s2 で回り始める。補助は接線方向に 1.5m/s2 を受ける
  const Eigen::Vector3d angular_accel(0.0, 0.0, 3.0);
  const Eigen::Vector3d lever_arm(0.5, 0.0, 0.0);
  const Eigen::Vector3d at_secondary(0.0, -1.5, 0.0);
  expectVectorNear(
    transferAcceleration(at_secondary, Eigen::Vector3d::Zero(), angular_accel, lever_arm),
    Eigen::Vector3d::Zero(), 1e-12);
}

TEST(InverseVarianceMean, WeightsByInverseOfVariance)
{
  // 分散 4e-6 と 1e-6 の重みは 1:4
  const auto mean = inverseVarianceMean({
      Measurement{Eigen::Vector3d(1.0, 0.0, 0.0), 4e-6},
      Measurement{Eigen::Vector3d(2.0, 0.0, 0.0), 1e-6}});
  ASSERT_TRUE(mean.has_value());
  EXPECT_NEAR(mean->value.x(), 1.8, 1e-12);
  EXPECT_NEAR(mean->variance, 0.8e-6, 1e-15);
  EXPECT_FALSE(inverseVarianceMean({}).has_value());
}

TEST(Interpolate, IsLinearBetweenSamples)
{
  const auto before = makeSample(1.0, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.0, 0.0, 9.0));
  const auto after = makeSample(1.01, Eigen::Vector3d(2.0, 2.0, 1.0), Eigen::Vector3d(1.0, 0.0, 10.0));
  const auto middle = interpolate(before, after, 1.0025);
  EXPECT_DOUBLE_EQ(middle.stamp_s, 1.0025);
  expectVectorNear(middle.gyro, Eigen::Vector3d(1.25, 2.0, 2.5), 1e-12);
  expectVectorNear(middle.accel, Eigen::Vector3d(0.25, 0.0, 9.25), 1e-12);
}

TEST(IsSaturated, ChecksEachAxisAgainstThreshold)
{
  EXPECT_FALSE(isSaturated(Eigen::Vector3d(10.0, -10.0, 10.0), 34.0));
  EXPECT_TRUE(isSaturated(Eigen::Vector3d(0.0, -34.0, 0.0), 34.0));
}

TEST(ImuFusion, SameMeasurementGivesSameOutput)
{
  ImuFusion fusion(testParams(), Extrinsic{});
  const Eigen::Vector3d gyro(0.1, -0.2, 0.3);
  const Eigen::Vector3d accel(0.5, 0.1, 9.8);
  const auto outputs = runBoth(
    fusion, 100, constant(gyro), constant(accel), constant(gyro), constant(accel));
  ASSERT_EQ(outputs.size(), 100U);
  // 最初の主の測定の時点では補助がまだ 1 つも届いていないので、待たずに主だけで出す
  EXPECT_EQ(outputs.front().gyro_from, Contribution::kPrimary);
  for (std::size_t k = 0U; k < outputs.size(); ++k) {
    EXPECT_NEAR(outputs[k].stamp_s, static_cast<double>(k) * kPeriodS, 1e-12);
    expectVectorNear(outputs[k].gyro, gyro, 1e-12);
    expectVectorNear(outputs[k].accel, accel, 1e-12);
    if (k > 0U) {
      EXPECT_EQ(outputs[k].gyro_from, Contribution::kBoth);
      EXPECT_EQ(outputs[k].accel_from, Contribution::kBoth);
    }
  }
}

TEST(ImuFusion, WeightsSourcesByNoise)
{
  auto params = testParams();
  params.primary.gyro_noise_std_radps = 0.002;
  params.secondary.gyro_noise_std_radps = 0.001;
  ImuFusion fusion(params, Extrinsic{});
  const auto outputs = runBoth(
    fusion, 10, constant(Eigen::Vector3d(1.0, 0.0, 0.0)), constant(kGravity),
    constant(Eigen::Vector3d(2.0, 0.0, 0.0)), constant(kGravity));
  ASSERT_EQ(outputs.size(), 10U);
  EXPECT_NEAR(outputs.back().gyro.x(), 1.8, 1e-9);
  EXPECT_NEAR(outputs.back().gyro_variance, 0.8e-6, 1e-15);
}

TEST(ImuFusion, RotatesSecondaryIntoPrimaryFrame)
{
  // 補助は主に対して z 軸まわりに 90 度回して取り付けてある
  Extrinsic extrinsic;
  extrinsic.rotation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  ImuFusion fusion(testParams(), extrinsic);
  const Eigen::Vector3d gyro(0.3, 0.1, 0.5);
  const Eigen::Vector3d accel(1.0, -2.0, 9.8);
  // 補助は自分の座標系で測る
  const Eigen::Vector3d secondary_gyro = extrinsic.rotation.transpose() * gyro;
  const Eigen::Vector3d secondary_accel = extrinsic.rotation.transpose() * accel;
  ASSERT_GT((secondary_gyro - gyro).norm(), 0.1);
  const auto outputs = runBoth(
    fusion, 20, constant(gyro), constant(accel), constant(secondary_gyro),
    constant(secondary_accel));
  ASSERT_EQ(outputs.size(), 20U);
  expectVectorNear(outputs.back().gyro, gyro, 1e-12);
  expectVectorNear(outputs.back().accel, accel, 1e-12);
}

// 主を通る軸まわりの回転で、補助が受ける加速度（てこの腕の効果）を返す
Eigen::Vector3d accelAtSecondary(
  const Eigen::Vector3d & gyro, const Eigen::Vector3d & angular_accel, const Eigen::Vector3d & lever_arm)
{
  // 補助は主から -lever_arm の位置にある
  const Eigen::Vector3d offset = -lever_arm;
  return kGravity + angular_accel.cross(offset) + gyro.cross(gyro.cross(offset));
}

TEST(ImuFusion, CorrectsLeverArmForPureRotation)
{
  Extrinsic extrinsic;
  extrinsic.lever_arm = Eigen::Vector3d(0.3, -0.2, 0.4);
  auto params = testParams();
  // 補助の重みを大きくして、補正の誤りが出力に出やすくする
  params.primary.accel_noise_std_mps2 = 0.02;
  params.secondary.accel_noise_std_mps2 = 0.01;
  ImuFusion fusion(params, extrinsic);
  const Eigen::Vector3d gyro(0.5, -0.3, 2.0);
  const Eigen::Vector3d at_secondary = accelAtSecondary(gyro, Eigen::Vector3d::Zero(), extrinsic.lever_arm);
  // 補正しなければ 1m/s2 以上ずれる
  ASSERT_GT((at_secondary - kGravity).norm(), 1.0);
  const auto outputs = runBoth(
    fusion, 50, constant(gyro), constant(kGravity), constant(gyro), constant(at_secondary));
  ASSERT_EQ(outputs.size(), 50U);
  for (const auto & output : outputs) {
    expectVectorNear(output.accel, kGravity, 1e-9);
  }
}

TEST(ImuFusion, CorrectsLeverArmWithAngularAcceleration)
{
  Extrinsic extrinsic;
  extrinsic.lever_arm = Eigen::Vector3d(0.3, -0.2, 0.4);
  ImuFusion fusion(testParams(), extrinsic);
  // 一定の角加速度で回り始める。差分を平滑化した角加速度は時定数 0.02 秒で真の値に近づく
  const Eigen::Vector3d angular_accel(1.0, 0.5, 3.0);
  const auto gyro = [angular_accel](double t) -> Eigen::Vector3d {return angular_accel * t;};
  const auto at_secondary = [&](double t) {
      return accelAtSecondary(gyro(t), angular_accel, extrinsic.lever_arm);
    };
  const auto outputs = runBoth(fusion, 100, gyro, constant(kGravity), gyro, at_secondary);
  ASSERT_EQ(outputs.size(), 100U);
  // 0.2 秒（時定数の 10 倍）以降は補正が効いている
  for (std::size_t k = 40U; k < outputs.size(); ++k) {
    expectVectorNear(outputs[k].accel, kGravity, 1e-3);
  }
  // 平滑化の立ち上がりでは補正が足りず、ずれが残る
  EXPECT_GT((outputs[1].accel - kGravity).norm(), 0.1);
}

TEST(ImuFusion, UsesSecondaryAccelerationWhenPrimaryIsSaturated)
{
  ImuFusion fusion(testParams(), Extrinsic{});
  const Eigen::Vector3d gyro(0.0, 0.0, 0.5);
  const Eigen::Vector3d saturated(200.0, 0.0, 9.8);
  const Eigen::Vector3d secondary_accel(1.0, 0.0, 9.8);
  const auto outputs = runBoth(
    fusion, 10, constant(gyro), constant(saturated), constant(gyro), constant(secondary_accel));
  // 最初の主の測定には補助がなく、使える加速度がないので出さない
  ASSERT_EQ(outputs.size(), 9U);
  expectVectorNear(outputs.back().accel, secondary_accel, 1e-12);
  EXPECT_EQ(outputs.back().accel_from, Contribution::kSecondary);
  EXPECT_EQ(outputs.back().gyro_from, Contribution::kBoth);
}

TEST(ImuFusion, UsesPrimaryGyroWhenSecondaryIsSaturated)
{
  ImuFusion fusion(testParams(), Extrinsic{});
  const Eigen::Vector3d gyro(0.0, 0.0, 0.5);
  const auto outputs = runBoth(
    fusion, 10, constant(gyro), constant(kGravity), constant(Eigen::Vector3d(0.0, 0.0, 40.0)),
    constant(kGravity));
  ASSERT_EQ(outputs.size(), 10U);
  expectVectorNear(outputs.back().gyro, gyro, 1e-12);
  EXPECT_EQ(outputs.back().gyro_from, Contribution::kPrimary);
  EXPECT_EQ(outputs.back().accel_from, Contribution::kBoth);
}

TEST(ImuFusion, OutputsNothingWhenBothAreSaturated)
{
  ImuFusion fusion(testParams(), Extrinsic{});
  const Eigen::Vector3d saturated(0.0, 0.0, 200.0);
  const auto outputs = runBoth(
    fusion, 10, constant(Eigen::Vector3d::Zero()), constant(saturated),
    constant(Eigen::Vector3d::Zero()), constant(saturated));
  EXPECT_TRUE(outputs.empty());
}

TEST(ImuFusion, IgnoresBrokenValues)
{
  ImuFusion fusion(testParams(), Extrinsic{});
  const Eigen::Vector3d broken(std::nan(""), 0.0, 9.8);
  const auto outputs = runBoth(
    fusion, 10, constant(Eigen::Vector3d::Zero()), constant(broken),
    constant(Eigen::Vector3d::Zero()), constant(kGravity));
  // 最初の主の測定には補助がなく、使える加速度がないので出さない
  ASSERT_EQ(outputs.size(), 9U);
  EXPECT_EQ(outputs.back().accel_from, Contribution::kSecondary);
  EXPECT_TRUE(outputs.back().accel.allFinite());
}

TEST(ImuFusion, FallsBackToPrimaryWhenSecondaryStops)
{
  const auto params = testParams();
  ImuFusion fusion(params, Extrinsic{});
  const Eigen::Vector3d gyro(0.0, 0.0, 0.2);
  auto outputs = runBoth(fusion, 100, constant(gyro), constant(kGravity), constant(gyro), constant(kGravity));
  ASSERT_EQ(outputs.size(), 100U);

  // 補助が止まる。待つ上限までは出力が遅れ、途絶とみなした後は主の測定ごとにすぐ出る
  std::vector<double> immediate;
  for (int k = 100; k < 200; ++k) {
    const double t = k * kPeriodS;
    const auto more = fusion.addPrimary(makeSample(t, gyro, kGravity), t);
    if (!more.empty() && std::abs(more.back().stamp_s - t) < 1e-12) {
      immediate.push_back(t);
    }
    append(outputs, more);
  }
  // 主の測定はどれも 1 回だけ、時刻順に出る
  ASSERT_EQ(outputs.size(), 200U);
  for (std::size_t k = 0U; k < outputs.size(); ++k) {
    EXPECT_NEAR(outputs[k].stamp_s, static_cast<double>(k) * kPeriodS, 1e-12);
  }
  EXPECT_EQ(outputs.back().gyro_from, Contribution::kPrimary);
  EXPECT_EQ(outputs.back().accel_from, Contribution::kPrimary);
  // 途絶とみなしてからは待たない
  ASSERT_FALSE(immediate.empty());
  EXPECT_LE(immediate.front(), 0.495 + params.timeout_s + params.max_wait_s + 1e-9);
}

TEST(ImuFusion, FallsBackToSecondaryWhenPrimaryStops)
{
  Extrinsic extrinsic;
  extrinsic.rotation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  extrinsic.lever_arm = Eigen::Vector3d(0.0, 0.0, 0.4);
  const auto params = testParams();
  ImuFusion fusion(params, extrinsic);
  const Eigen::Vector3d gyro(0.0, 0.0, 0.0);
  const Eigen::Vector3d secondary_accel = extrinsic.rotation.transpose() * kGravity;
  auto outputs = runBoth(
    fusion, 100, constant(gyro), constant(kGravity), constant(gyro), constant(secondary_accel));
  ASSERT_EQ(outputs.size(), 100U);

  // 主が止まり、補助だけが届く。主の途絶は受信の時刻で判定する
  std::vector<FusedSample> secondary_only;
  for (int k = 100; k < 200; ++k) {
    const double t = k * kPeriodS;
    append(secondary_only, fusion.addSecondary(makeSample(t, gyro, secondary_accel), t));
  }
  ASSERT_FALSE(secondary_only.empty());
  // 途絶とみなすまでは出さず、その後は補助の時刻ごとに出す
  EXPECT_GT(secondary_only.front().stamp_s, 0.495 + params.timeout_s - 1e-9);
  EXPECT_LE(secondary_only.front().stamp_s, 0.495 + params.timeout_s + kPeriodS + 1e-9);
  for (std::size_t k = 1U; k < secondary_only.size(); ++k) {
    EXPECT_NEAR(secondary_only[k].stamp_s - secondary_only[k - 1U].stamp_s, kPeriodS, 1e-9);
  }
  // 補助の値は主の座標系に回して出す
  for (const auto & output : secondary_only) {
    expectVectorNear(output.accel, kGravity, 1e-9);
    EXPECT_EQ(output.accel_from, Contribution::kSecondary);
  }

  // 主が戻ると、主の時刻で両方を使う
  const auto resumed = runBoth(
    fusion, 20, constant(gyro), constant(kGravity), constant(gyro), constant(secondary_accel),
    200 * kPeriodS);
  ASSERT_FALSE(resumed.empty());
  EXPECT_GT(resumed.front().stamp_s, secondary_only.back().stamp_s);
  EXPECT_EQ(resumed.back().accel_from, Contribution::kBoth);
}

TEST(ImuFusion, OutputsNothingWithoutMeasurements)
{
  ImuFusion fusion(testParams(), Extrinsic{});
  // 補助の値が壊れていて、主も来ない
  const Eigen::Vector3d broken(std::nan(""), 0.0, 0.0);
  std::vector<FusedSample> outputs;
  for (int k = 0; k < 20; ++k) {
    const double t = k * kPeriodS;
    append(outputs, fusion.addSecondary(makeSample(t, broken, broken), t));
  }
  EXPECT_TRUE(outputs.empty());
}

TEST(ImuFusion, InterpolatesSecondaryToPrimaryStamps)
{
  auto params = testParams();
  params.primary.gyro_noise_std_radps = 1.0;
  params.secondary.gyro_noise_std_radps = 0.001;
  ImuFusion fusion(params, Extrinsic{});
  // 補助は主より 2.5ms 遅れた時刻で測り、主のあとに届く
  const auto gyro = [](double t) {return Eigen::Vector3d(t, 2.0 * t, -t);};
  std::vector<FusedSample> outputs;
  for (int k = 0; k < 50; ++k) {
    const double t = k * kPeriodS;
    const double ts = t + 0.0025;
    append(outputs, fusion.addPrimary(makeSample(t, Eigen::Vector3d::Zero(), kGravity), t));
    append(outputs, fusion.addSecondary(makeSample(ts, gyro(ts), kGravity), ts));
  }
  // 最初の主の測定には前の補助がないので、補助を使わない
  ASSERT_EQ(outputs.size(), 50U);
  EXPECT_EQ(outputs.front().gyro_from, Contribution::kPrimary);
  for (std::size_t k = 1U; k < outputs.size(); ++k) {
    EXPECT_EQ(outputs[k].gyro_from, Contribution::kBoth);
    // 主の重みはほぼ 0 なので、補助を主の時刻に補間した値になる
    expectVectorNear(outputs[k].gyro, gyro(outputs[k].stamp_s), 1e-5);
  }
}

// 時刻のずれの推定に使う、回転の速さが不規則に変わる動き
Eigen::Vector3d wobblingGyro(const double t)
{
  return Eigen::Vector3d(
    0.4 * std::sin(2.0 * M_PI * 0.7 * t),
    0.3 * std::cos(2.0 * M_PI * 1.3 * t + 0.5),
    1.0 + 0.8 * std::sin(2.0 * M_PI * 0.45 * t) + 0.3 * std::sin(2.0 * M_PI * 2.9 * t + 1.0));
}

TEST(EstimateTimeOffset, FindsKnownOffset)
{
  // 補助の時刻は真の時刻より 12.3ms 早い（足すと主の時刻になる値は +12.3ms）
  const double true_offset = 0.0123;
  std::vector<RateSample> primary;
  std::vector<RateSample> secondary;
  for (int k = 0; k < 2000; ++k) {
    const double t = k * kPeriodS;
    primary.push_back(RateSample{t, wobblingGyro(t).norm()});
    // 補助は主と違う刻みで測る
    const double ts = t + 0.0021;
    secondary.push_back(RateSample{ts, wobblingGyro(ts + true_offset).norm()});
  }
  OffsetEstimationParams params;
  const auto estimate = estimateTimeOffset(primary, secondary, 0.0, params);
  EXPECT_EQ(estimate.status, OffsetStatus::kAccepted);
  EXPECT_NEAR(estimate.offset_s, true_offset, 0.0003);
  EXPECT_GT(estimate.correlation, 0.99);
}

TEST(EstimateTimeOffset, RejectsWithoutRotation)
{
  std::vector<RateSample> primary;
  std::vector<RateSample> secondary;
  for (int k = 0; k < 2000; ++k) {
    const double t = k * kPeriodS;
    primary.push_back(RateSample{t, 0.01});
    secondary.push_back(RateSample{t, 0.01});
  }
  const auto estimate = estimateTimeOffset(primary, secondary, 0.0, OffsetEstimationParams{});
  EXPECT_EQ(estimate.status, OffsetStatus::kLowExcitation);
}

TEST(EstimateTimeOffset, ReportsOffsetOutsideSearchRange)
{
  std::vector<RateSample> primary;
  std::vector<RateSample> secondary;
  for (int k = 0; k < 2000; ++k) {
    const double t = k * kPeriodS;
    primary.push_back(RateSample{t, wobblingGyro(t).norm()});
    secondary.push_back(RateSample{t, wobblingGyro(t + 0.2).norm()});
  }
  const auto estimate = estimateTimeOffset(primary, secondary, 0.0, OffsetEstimationParams{});
  EXPECT_EQ(estimate.status, OffsetStatus::kAtSearchEdge);
}

TEST(ImuFusion, EstimatesAndAppliesTimeOffset)
{
  auto params = testParams();
  params.offset_estimation.enabled = true;
  params.primary.gyro_noise_std_radps = 1.0;
  params.secondary.gyro_noise_std_radps = 0.001;
  ImuFusion fusion(params, Extrinsic{});
  const double true_offset = 0.0123;
  std::vector<FusedSample> outputs;
  for (int k = 0; k < 3000; ++k) {
    const double t = k * kPeriodS;
    append(outputs, fusion.addPrimary(makeSample(t, wobblingGyro(t), kGravity), t));
    // 補助の時刻は真の時刻より早く、受信は遅れない
    append(
      outputs, fusion.addSecondary(makeSample(t - true_offset, wobblingGyro(t), kGravity), t));
  }
  ASSERT_TRUE(fusion.lastOffsetEstimate().has_value());
  EXPECT_EQ(fusion.lastOffsetEstimate()->status, OffsetStatus::kAccepted);
  EXPECT_NEAR(fusion.timeOffset(), true_offset, 0.0003);
  // 推定したずれで補助を合わせると、補助の値は主の時刻の真の値になる
  ASSERT_FALSE(outputs.empty());
  const auto & last = outputs.back();
  EXPECT_EQ(last.gyro_from, Contribution::kBoth);
  expectVectorNear(last.gyro, wobblingGyro(last.stamp_s), 0.003);
}

TEST(UnitCheck, RejectsAccelerationInG)
{
  UnitCheckState in_g;
  UnitCheckState in_si;
  for (int k = 0; k < 200; ++k) {
    in_g = accumulateUnitCheck(in_g, Eigen::Vector3d(0.0, 0.0, 1.0), 200U, 0.3);
    in_si = accumulateUnitCheck(in_si, kGravity, 200U, 0.3);
    if (k < 199) {
      EXPECT_EQ(in_g.result.status, UnitCheckStatus::kPending);
    }
  }
  EXPECT_EQ(in_g.result.status, UnitCheckStatus::kFailed);
  EXPECT_NEAR(in_g.result.mean_accel_norm_mps2, 1.0, 1e-12);
  EXPECT_EQ(in_si.result.status, UnitCheckStatus::kPassed);
}

TEST(ImuFusion, ExcludesSourceWithWrongUnit)
{
  auto params = testParams();
  params.unit_check_samples = 20U;
  ImuFusion fusion(params, Extrinsic{});
  // 補助の加速度が g 単位のまま入る
  const Eigen::Vector3d gyro(0.0, 0.0, 0.1);
  const auto outputs = runBoth(
    fusion, 40, constant(gyro), constant(kGravity), constant(gyro), constant(kGravity / kStandardGravity));
  EXPECT_EQ(fusion.primaryUnitCheck().status, UnitCheckStatus::kPassed);
  EXPECT_EQ(fusion.secondaryUnitCheck().status, UnitCheckStatus::kFailed);
  // 確認が済むまでは出さず、その後は主だけで出す
  ASSERT_EQ(outputs.size(), 21U);
  EXPECT_NEAR(outputs.front().stamp_s, 19 * kPeriodS, 1e-12);
  EXPECT_EQ(outputs.back().accel_from, Contribution::kPrimary);
  expectVectorNear(outputs.back().accel, kGravity, 1e-12);
}

TEST(ImuFusion, WaitsForUnitChecksOfBothSources)
{
  auto params = testParams();
  params.unit_check_samples = 20U;
  ImuFusion fusion(params, Extrinsic{});
  const Eigen::Vector3d gyro(0.0, 0.0, 0.1);
  // 補助が主より先に届くので、補助の確認が 1 サンプル先に済む
  std::vector<FusedSample> outputs;
  for (int k = 0; k < 40; ++k) {
    const double t = k * kPeriodS;
    append(outputs, fusion.addSecondary(makeSample(t, gyro, kGravity), t));
    append(outputs, fusion.addPrimary(makeSample(t, gyro, kGravity), t));
  }
  // 両方の確認が済んでから、両方を使って出す
  ASSERT_EQ(outputs.size(), 21U);
  EXPECT_NEAR(outputs.front().stamp_s, 19 * kPeriodS, 1e-12);
  for (const auto & output : outputs) {
    EXPECT_EQ(output.gyro_from, Contribution::kBoth);
    EXPECT_EQ(output.accel_from, Contribution::kBoth);
  }
}

TEST(ImuFusion, DoesNotWaitForUnitCheckOfAbsentSource)
{
  auto params = testParams();
  params.unit_check_samples = 20U;
  ImuFusion fusion(params, Extrinsic{});
  // 補助が 1 度も届かなくても、主の確認が済めば主だけで出す
  std::vector<FusedSample> outputs;
  for (int k = 0; k < 40; ++k) {
    const double t = k * kPeriodS;
    append(outputs, fusion.addPrimary(makeSample(t, Eigen::Vector3d::Zero(), kGravity), t));
  }
  ASSERT_EQ(outputs.size(), 21U);
  EXPECT_EQ(outputs.back().gyro_from, Contribution::kPrimary);
}

TEST(ImuFusion, DropsReorderedSamplesAndRestartsAfterClockJump)
{
  ImuFusion fusion(testParams(), Extrinsic{});
  const Eigen::Vector3d gyro(0.0, 0.0, 0.1);
  auto outputs = runBoth(fusion, 10, constant(gyro), constant(kGravity), constant(gyro), constant(kGravity), 10.0);
  ASSERT_EQ(outputs.size(), 10U);
  // 少し前の時刻の測定は捨てる
  EXPECT_TRUE(fusion.addPrimary(makeSample(10.0, gyro, kGravity), 10.1).empty());
  EXPECT_TRUE(fusion.addSecondary(makeSample(10.0, gyro, kGravity), 10.1).empty());
  // 大きく戻ったら、記録の再生のやり直しとみなして最初から出す
  const auto restarted = runBoth(fusion, 10, constant(gyro), constant(kGravity), constant(gyro), constant(kGravity), 0.0);
  ASSERT_EQ(restarted.size(), 10U);
  EXPECT_NEAR(restarted.front().stamp_s, 0.0, 1e-12);
}

TEST(ValidateParams, RejectsInconsistentEstimationSettings)
{
  auto params = testParams();
  EXPECT_FALSE(validateParams(params).has_value());
  params.offset_estimation.step_s = 0.1;
  EXPECT_TRUE(validateParams(params).has_value());
  params = testParams();
  params.offset_estimation.window_s = 0.05;
  EXPECT_TRUE(validateParams(params).has_value());
  params = testParams();
  params.secondary.accel_noise_std_mps2 = 0.0;
  EXPECT_TRUE(validateParams(params).has_value());
  EXPECT_THROW(static_cast<void>(ImuFusion(params, Extrinsic{})), std::invalid_argument);
}
}  // namespace
