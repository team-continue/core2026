// core_shooter の原点合わせ（turret_origin.hpp）と衝突検知（turret_stall.hpp）の単体テスト。
//
// ストッパ押し当ての状態遷移は、指令へ一定速度で追従しストッパで止まる簡単なモータの
// 模型で検証する。ROS の起動は不要。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "core_shooter/turret_origin.hpp"
#include "core_shooter/turret_stall.hpp"

using core_shooter::DirectionalBlock;
using core_shooter::OriginAxisParams;
using core_shooter::OriginAxisState;
using core_shooter::OriginOffsets;
using core_shooter::OriginPhase;
using core_shooter::StallParams;
using core_shooter::StallState;

namespace
{

constexpr double kDt = 1.0 / 30.0;

StallParams stallParams()
{
  return StallParams{0.1, 0.05, 0.2};
}

OriginAxisParams originParams(double direction)
{
  OriginAxisParams p;
  p.direction = direction;
  p.limit_angle = 0.0;
  p.limit_angle_measured = true;
  p.approach_speed = 0.2;
  p.lead = 0.15;
  p.contact = StallParams{0.08, 0.05, 0.2};
  p.min_travel = 0.02;
  p.max_travel = 3.0;
  p.backoff = 0.1;
  p.settle_sec = 0.5;
  p.timeout_sec = 30.0;
  return p;
}

/// 指令へ max_speed で追従し、ストッパ（direction 側）を越えられないモータの模型。
struct MotorModel
{
  double position = 0.0;
  double velocity = 0.0;
  double max_speed = 1.0;
  double stopper = 0.0;
  double direction = 1.0;  ///< ストッパがある向き

  void step(double command, double dt)
  {
    const double step = std::clamp(command - position, -max_speed * dt, max_speed * dt);
    double next = position + step;
    if (direction * (next - stopper) > 0.0) {
      next = stopper;
    }
    velocity = (next - position) / dt;
    position = next;
  }
};

/// 押し当てを Done / Failed まで回す。
OriginAxisState runOriginAxis(MotorModel & motor, const OriginAxisParams & p)
{
  OriginAxisState state = core_shooter::startOriginAxis(motor.position);
  for (int i = 0; i < 100000; ++i) {
    state = core_shooter::stepOriginAxis(state, motor.position, motor.velocity, kDt, p);
    if (state.phase == OriginPhase::Done || state.phase == OriginPhase::Failed) {
      return state;
    }
    motor.step(state.target_raw, kDt);
  }
  ADD_FAILURE() << "origin axis did not finish";
  return state;
}

/// テスト用の一時ファイルのパス。デストラクタで消す。
class TempPath
{
public:
  explicit TempPath(const std::string & name)
  {
    static int counter = 0;
    path_ = (std::filesystem::temp_directory_path() /
      ("core_test_turret_origin_" + std::to_string(++counter)) / name).string();
  }
  ~TempPath()
  {
    std::error_code ec;
    std::filesystem::remove_all(std::filesystem::path(path_).parent_path(), ec);
  }
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

}  // namespace

// ============================================================================
// stall 判定
// ============================================================================

TEST(TurretStall, BecomesStalledAfterHoldTime)
{
  StallState state;
  state = core_shooter::stepStall(state, 1.0, 0.5, 0.0, 0.1, stallParams());
  EXPECT_FALSE(state.stalled);
  state = core_shooter::stepStall(state, 1.0, 0.5, 0.0, 0.1, stallParams());
  EXPECT_TRUE(state.stalled);
}

TEST(TurretStall, MovingIsNotStalledEvenWithLargeError)
{
  // 大きく動いている最中の追従遅れ
  StallState state;
  for (int i = 0; i < 10; ++i) {
    state = core_shooter::stepStall(state, 2.0, 0.5, 1.0, 0.1, stallParams());
  }
  EXPECT_FALSE(state.stalled);
  EXPECT_DOUBLE_EQ(state.candidate_sec, 0.0);
}

TEST(TurretStall, SmallErrorIsNotStalled)
{
  StallState state;
  for (int i = 0; i < 10; ++i) {
    state = core_shooter::stepStall(state, 0.55, 0.5, 0.0, 0.1, stallParams());
  }
  EXPECT_FALSE(state.stalled);
}

TEST(TurretStall, CandidateResetsWhenConditionBreaks)
{
  StallState state;
  state = core_shooter::stepStall(state, 1.0, 0.5, 0.0, 0.15, stallParams());
  state = core_shooter::stepStall(state, 0.5, 0.5, 0.0, 0.15, stallParams());
  state = core_shooter::stepStall(state, 1.0, 0.5, 0.0, 0.15, stallParams());
  EXPECT_FALSE(state.stalled);
}

TEST(TurretStall, UnknownVelocityIsNeverStalled)
{
  StallState state;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 10; ++i) {
    state = core_shooter::stepStall(state, 1.0, 0.5, nan, 0.1, stallParams());
  }
  EXPECT_FALSE(state.stalled);
}

// ============================================================================
// 方向別リミット
// ============================================================================

TEST(DirectionalBlock, BlocksOnlyTheCollidingDirection)
{
  DirectionalBlock block = core_shooter::blockOnStall({}, 1.0, 0.4);
  ASSERT_TRUE(block.upper.has_value());
  EXPECT_FALSE(block.lower.has_value());
  EXPECT_DOUBLE_EQ(*block.upper, 0.4);

  EXPECT_DOUBLE_EQ(core_shooter::clampByBlock(1.0, block), 0.4);   // 当たった向きは止まる
  EXPECT_DOUBLE_EQ(core_shooter::clampByBlock(0.1, block), 0.1);   // 逆向きはそのまま
}

TEST(DirectionalBlock, NegativeCollisionBlocksLowerSide)
{
  DirectionalBlock block = core_shooter::blockOnStall({}, -1.0, -0.4);
  ASSERT_TRUE(block.lower.has_value());
  EXPECT_FALSE(block.upper.has_value());
  EXPECT_DOUBLE_EQ(core_shooter::clampByBlock(-1.0, block), -0.4);
  EXPECT_DOUBLE_EQ(core_shooter::clampByBlock(0.0, block), 0.0);
}

TEST(DirectionalBlock, ReleasedAfterMovingBackByMargin)
{
  DirectionalBlock block = core_shooter::blockOnStall({}, 1.0, 0.4);
  block = core_shooter::releaseBlock(block, 0.35, 0.1);  // 押し返されただけ
  EXPECT_TRUE(core_shooter::isBlocked(block));
  block = core_shooter::releaseBlock(block, 0.29, 0.1);
  EXPECT_FALSE(core_shooter::isBlocked(block));
  EXPECT_DOUBLE_EQ(core_shooter::clampByBlock(1.0, block), 1.0);
}

TEST(DirectionalBlock, MovingFurtherDoesNotRelease)
{
  DirectionalBlock block = core_shooter::blockOnStall({}, -1.0, -0.4);
  block = core_shooter::releaseBlock(block, -0.45, 0.1);
  EXPECT_TRUE(core_shooter::isBlocked(block));
}

// ============================================================================
// 押し当て
// ============================================================================

TEST(TurretOrigin, ApproachTargetIsLimitedByLead)
{
  EXPECT_DOUBLE_EQ(core_shooter::nextApproachTarget(0.0, 0.0, 1.0, 1.0, 0.15, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(core_shooter::nextApproachTarget(0.1, 0.0, 1.0, 1.0, 0.15, 0.1), 0.15);
  EXPECT_DOUBLE_EQ(core_shooter::nextApproachTarget(-0.1, 0.0, -1.0, 1.0, 0.15, 0.1), -0.15);
}

TEST(TurretOrigin, FindsStopperInPositiveDirection)
{
  MotorModel motor{0.3, 0.0, 1.0, 1.2, 1.0};
  const auto p = originParams(1.0);
  const auto state = runOriginAxis(motor, p);

  ASSERT_EQ(state.phase, OriginPhase::Done) << state.error;
  EXPECT_NEAR(state.contact_raw, 1.2, 1e-9);
  EXPECT_NEAR(state.target_raw, 1.2 - p.backoff, 1e-9);
}

TEST(TurretOrigin, FindsStopperInNegativeDirection)
{
  MotorModel motor{0.5, 0.0, 1.0, -0.7, -1.0};
  const auto p = originParams(-1.0);
  const auto state = runOriginAxis(motor, p);

  ASSERT_EQ(state.phase, OriginPhase::Done) << state.error;
  EXPECT_NEAR(state.contact_raw, -0.7, 1e-9);
  EXPECT_NEAR(state.target_raw, -0.7 + p.backoff, 1e-9);
}

TEST(TurretOrigin, PushingForceIsBoundedByLead)
{
  // 接触後も目標角は実角 + lead を超えない
  MotorModel motor{0.0, 0.0, 1.0, 0.5, 1.0};
  const auto p = originParams(1.0);
  OriginAxisState state = core_shooter::startOriginAxis(motor.position);
  double max_error = 0.0;
  while (state.phase == OriginPhase::Approach) {
    state = core_shooter::stepOriginAxis(state, motor.position, motor.velocity, kDt, p);
    max_error = std::max(max_error, state.target_raw - motor.position);
    motor.step(state.target_raw, kDt);
  }
  EXPECT_LE(max_error, p.lead + 1e-9);
}

TEST(TurretOrigin, FailsWhenNoStopperWithinMaxTravel)
{
  MotorModel motor{0.0, 0.0, 1.0, 100.0, 1.0};
  auto p = originParams(1.0);
  p.max_travel = 0.5;
  const auto state = runOriginAxis(motor, p);
  ASSERT_EQ(state.phase, OriginPhase::Failed);
  EXPECT_NE(state.error.find("max_travel"), std::string::npos);
}

TEST(TurretOrigin, FailsWhenAlreadyAtStopper)
{
  MotorModel motor{1.0, 0.0, 1.0, 1.0, 1.0};
  const auto state = runOriginAxis(motor, originParams(1.0));
  ASSERT_EQ(state.phase, OriginPhase::Failed);
  EXPECT_NE(state.error.find("min_travel"), std::string::npos);
  EXPECT_DOUBLE_EQ(state.target_raw, 1.0);  // その場で止める
}

TEST(TurretOrigin, FailsOnTimeout)
{
  MotorModel motor{0.0, 0.0, 1.0, 100.0, 1.0};
  auto p = originParams(1.0);
  p.timeout_sec = 1.0;
  const auto state = runOriginAxis(motor, p);
  ASSERT_EQ(state.phase, OriginPhase::Failed);
  EXPECT_NE(state.error.find("timeout"), std::string::npos);
}

TEST(TurretOrigin, MeasureThenCalibrateRecoversTheOrigin)
{
  // 原点の生角が 0.25 の機体。ストッパは原点から見て +0.9。
  const double true_offset = 0.25;
  const double stopper_raw = true_offset + 0.9;
  const auto p = originParams(1.0);

  // 原点に置いて実測する
  MotorModel measure_motor{true_offset, 0.0, 1.0, stopper_raw, 1.0};
  const auto measured = runOriginAxis(measure_motor, p);
  ASSERT_EQ(measured.phase, OriginPhase::Done) << measured.error;
  const double limit = core_shooter::computeLimitAngle(measured.contact_raw, true_offset);
  EXPECT_NEAR(limit, 0.9, 1e-9);

  // 別の姿勢から原点合わせをする
  MotorModel motor{-0.4, 0.0, 1.0, stopper_raw, 1.0};
  const auto calibrated = runOriginAxis(motor, p);
  ASSERT_EQ(calibrated.phase, OriginPhase::Done) << calibrated.error;
  EXPECT_NEAR(core_shooter::computeOriginOffset(calibrated.contact_raw, limit), true_offset, 1e-9);
}

TEST(TurretOrigin, ValidatesParams)
{
  EXPECT_FALSE(core_shooter::validateOriginAxisParams(originParams(1.0)).has_value());

  auto p = originParams(1.0);
  p.direction = 0.5;
  EXPECT_TRUE(core_shooter::validateOriginAxisParams(p).has_value());

  p = originParams(1.0);
  p.contact.error_threshold = p.lead;  // 永遠に接触と判定できない
  EXPECT_TRUE(core_shooter::validateOriginAxisParams(p).has_value());

  p = originParams(1.0);
  p.min_travel = p.max_travel;
  EXPECT_TRUE(core_shooter::validateOriginAxisParams(p).has_value());
}

TEST(TurretOrigin, ChecksMeasuredLimitAgainstCad)
{
  auto p = originParams(1.0);
  p.cad_limit_angle = 0.9;
  p.cad_tolerance = 0.05;
  EXPECT_FALSE(core_shooter::checkLimitAgainstCad(0.93, p).has_value());
  EXPECT_TRUE(core_shooter::checkLimitAgainstCad(1.0, p).has_value());

  p.cad_tolerance = 0.0;  // 照合しない
  EXPECT_FALSE(core_shooter::checkLimitAgainstCad(5.0, p).has_value());
}

// ============================================================================
// 結果ファイル
// ============================================================================

TEST(TurretOriginFile, WriteThenLoadRoundTrip)
{
  TempPath file("local/turret_origin_left.state.yaml");  // ディレクトリも作る
  std::string reason;
  ASSERT_TRUE(
    core_shooter::writeOriginOffsets(
      file.path(), OriginOffsets{0.123456789, -1.5}, "2026-10-25T12:00:00", reason)) << reason;
  EXPECT_FALSE(std::filesystem::exists(file.path() + ".tmp"));

  OriginOffsets loaded;
  ASSERT_TRUE(core_shooter::loadOriginOffsets(file.path(), loaded, reason)) << reason;
  EXPECT_NEAR(loaded.yaw, 0.123456789, 1e-9);
  EXPECT_NEAR(loaded.pitch, -1.5, 1e-9);
}

TEST(TurretOriginFile, OverwritesExistingFile)
{
  TempPath file("turret_origin.state.yaml");
  std::string reason;
  ASSERT_TRUE(core_shooter::writeOriginOffsets(file.path(), {1.0, 2.0}, "t1", reason)) << reason;
  ASSERT_TRUE(core_shooter::writeOriginOffsets(file.path(), {3.0, 4.0}, "t2", reason)) << reason;
  OriginOffsets loaded;
  ASSERT_TRUE(core_shooter::loadOriginOffsets(file.path(), loaded, reason)) << reason;
  EXPECT_DOUBLE_EQ(loaded.yaw, 3.0);
  EXPECT_DOUBLE_EQ(loaded.pitch, 4.0);
}

TEST(TurretOriginFile, MissingFileFails)
{
  TempPath file("missing.yaml");
  OriginOffsets loaded{9.0, 9.0};
  std::string reason;
  EXPECT_FALSE(core_shooter::loadOriginOffsets(file.path(), loaded, reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_DOUBLE_EQ(loaded.yaw, 9.0);  // 失敗時は書き換えない
}

TEST(TurretOriginFile, InvalidContentFails)
{
  TempPath file("broken.yaml");
  std::filesystem::create_directories(std::filesystem::path(file.path()).parent_path());
  {
    std::ofstream stream(file.path());
    stream << "yaw_origin_offset: abc\n";
  }
  OriginOffsets loaded;
  std::string reason;
  EXPECT_FALSE(core_shooter::loadOriginOffsets(file.path(), loaded, reason));
  EXPECT_NE(reason.find("pitch_origin_offset"), std::string::npos);
}
