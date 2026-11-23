#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/grid_map.hpp"
#include "core_planning/safety.hpp"

namespace
{
using core_planning::safety::applySpeedScale;
using core_planning::safety::BodyVelocity;
using core_planning::safety::Decision;
using core_planning::safety::evaluateSafety;
using core_planning::safety::firstReachableViolation;
using core_planning::safety::Footprint;
using core_planning::safety::inTurnedFrame;
using core_planning::safety::limitDeceleration;
using core_planning::safety::Obstacle;
using core_planning::safety::ObstacleKind;
using core_planning::safety::PlanarPose;
using core_planning::safety::reachableRadius;
using core_planning::safety::SafetyParams;
using core_planning::safety::SafetyResult;
using core_planning::safety::staticClearance;
using core_planning::safety::superviseCommand;

// 前方 distance に止まっているロボット
std::vector<Obstacle> robotAhead(const double distance)
{
  return {Obstacle{distance, 0.0, 0.0, 0.0, ObstacleKind::kRobot}};
}

const BodyVelocity kForward{2.0, 0.0, 0.0};

TEST(EvaluateSafety, StopsWhenCollisionIsImminent)
{
  // 隙間は 1.5 - 0.4 - 0.45 - 0.1 = 0.55m で、2m/s なら約 0.3 秒で当たる
  const auto result = evaluateSafety(
    robotAhead(1.5), PlanarPose{}, kForward, kForward, Footprint{}, SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kStop);
  EXPECT_DOUBLE_EQ(result.speed_scale, 0.0);
  EXPECT_NEAR(result.ttc_s, 0.3, 0.06);
}

TEST(EvaluateSafety, SlowsDownBetweenThresholds)
{
  // 隙間は 2.05m で、2m/s なら約 1.05 秒で当たる
  const auto result = evaluateSafety(
    robotAhead(3.0), PlanarPose{}, kForward, kForward, Footprint{}, SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kSlow);
  EXPECT_NEAR(result.speed_scale, 1.05 / 1.5, 0.05);
}

TEST(EvaluateSafety, PassesWhenCollisionIsFarAway)
{
  const auto result = evaluateSafety(
    robotAhead(6.0), PlanarPose{}, kForward, kForward, Footprint{}, SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kPass);
  EXPECT_DOUBLE_EQ(result.speed_scale, 1.0);
  EXPECT_LT(result.ttc_s, 0.0);
}

TEST(EvaluateSafety, PassesWhenObstacleDoesNotCross)
{
  // 横にいて同じ向きに動く相手とは当たらない
  const std::vector<Obstacle> obstacles{Obstacle{2.0, 3.0, 2.0, 0.0, ObstacleKind::kRobot}};
  const auto result = evaluateSafety(
    obstacles, PlanarPose{}, kForward, kForward, Footprint{}, SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kPass);
}

TEST(EvaluateSafety, FollowsYawOfThePose)
{
  // 90 度向いているので、機体の前方は世界座標の +y
  const std::vector<Obstacle> ahead{Obstacle{0.0, 1.5, 0.0, 0.0, ObstacleKind::kRobot}};
  const auto result = evaluateSafety(
    ahead, PlanarPose{0.0, 0.0, 1.5707963267948966}, kForward, kForward, Footprint{},
    SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kStop);
}

TEST(EvaluateSafety, UsesLargerRadiusForUnknown)
{
  // 不明の半径は人より 0.2m 大きいので、早く当たる
  const auto gap_for = [](const ObstacleKind kind) {
      const std::vector<Obstacle> obstacles{Obstacle{1.0, 0.0, 0.0, 0.0, kind}};
      return evaluateSafety(
        obstacles, PlanarPose{}, BodyVelocity{1.0, 0.0, 0.0}, BodyVelocity{1.0, 0.0, 0.0},
        Footprint{}, SafetyParams{}).ttc_s;
    };
  EXPECT_LT(gap_for(ObstacleKind::kUnknown), gap_for(ObstacleKind::kHuman));
}

TEST(EvaluateSafety, DoesNotStopEvasiveMotion)
{
  // 相手が 3m/s で向かってくる。止まると約 0.7 秒で当たるので、下がる指令を止めない
  const std::vector<Obstacle> obstacles{Obstacle{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity stopped{0.0, 0.0, 0.0};
  const BodyVelocity backward{-1.0, 0.0, 0.0};
  const auto result = evaluateSafety(
    obstacles, PlanarPose{}, stopped, backward, Footprint{}, SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kPass);
  EXPECT_DOUBLE_EQ(result.speed_scale, 1.0);
  EXPECT_GT(result.ttc_s, 0.7);
}

TEST(EvaluateSafety, DoesNotPassEvasiveMotionThatHitsSoon)
{
  // 相手が 3m/s で 2m 先から向かってくる。止まると約 0.35 秒で当たり、下がる指令（0.5m/s）でも
  // 約 0.42 秒で当たる。止まるほうが早く当たるが、指令も ttc_stop_s 以内に当たるので通さない
  const std::vector<Obstacle> obstacles{Obstacle{2.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}};
  const auto result = evaluateSafety(
    obstacles, PlanarPose{}, BodyVelocity{}, BodyVelocity{-0.5, 0.0, 0.0}, Footprint{},
    SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kStop);
  EXPECT_NEAR(result.ttc_s, 0.42, 0.06);
  // 相手へ向かう成分だけを落とすので、下がる指令はそのまま残る
  EXPECT_NEAR(applySpeedScale(BodyVelocity{-0.5, 0.0, 0.0}, result).vx, -0.5, 1e-9);
}

TEST(EvaluateSafety, StopsMotionTowardApproachingObstacle)
{
  // 向かってくる相手へ進む指令は止める。相対 6m/s なので約 0.35 秒で当たる
  const std::vector<Obstacle> obstacles{Obstacle{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}};
  const auto result = evaluateSafety(
    obstacles, PlanarPose{}, BodyVelocity{}, BodyVelocity{3.0, 0.0, 0.0}, Footprint{},
    SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kStop);
  EXPECT_DOUBLE_EQ(result.speed_scale, 0.0);
}

TEST(EvaluateSafety, AllowsLeavingAnOverlap)
{
  // すでに余裕の内側にいても、離れる指令なら通す
  const auto result = evaluateSafety(
    robotAhead(0.9), PlanarPose{}, BodyVelocity{}, BodyVelocity{-1.0, 0.0, 0.0}, Footprint{},
    SafetyParams{});
  EXPECT_EQ(result.decision, Decision::kPass);
}

TEST(ApplySpeedScale, KeepsSidewaysMotionWhileStoppingTowardThreat)
{
  SafetyResult result;
  result.decision = Decision::kStop;
  result.speed_scale = 0.0;
  result.threat_x = 1.0;
  const auto output = applySpeedScale(BodyVelocity{2.0, 1.0, 0.5}, result);
  EXPECT_NEAR(output.vx, 0.0, 1e-9);
  EXPECT_NEAR(output.vy, 1.0, 1e-9);
}

TEST(ApplySpeedScale, DoesNotSlowMotionAwayFromThreat)
{
  SafetyResult result;
  result.decision = Decision::kSlow;
  result.speed_scale = 0.5;
  result.threat_x = 1.0;
  const auto output = applySpeedScale(BodyVelocity{-2.0, 0.0, 0.0}, result);
  EXPECT_NEAR(output.vx, -2.0, 1e-9);
}

TEST(ApplySpeedScale, ScalesEverythingWithoutDirection)
{
  SafetyResult result;
  result.decision = Decision::kStop;
  result.speed_scale = 0.0;
  const auto output = applySpeedScale(BodyVelocity{2.0, 1.0, 0.5}, result);
  EXPECT_NEAR(output.vx, 0.0, 1e-9);
  EXPECT_NEAR(output.vy, 0.0, 1e-9);
}

TEST(LimitDeceleration, LimitsBraking)
{
  const auto result = limitDeceleration(
    BodyVelocity{3.0, 0.0, 0.0}, BodyVelocity{0.0, 0.0, 0.0}, 3.0, 0.05);
  EXPECT_NEAR(result.vx, 2.85, 1e-9);
}

TEST(LimitDeceleration, DoesNotLimitAcceleration)
{
  const auto result = limitDeceleration(
    BodyVelocity{1.0, 0.0, 0.0}, BodyVelocity{3.0, 0.0, 0.5}, 3.0, 0.05);
  EXPECT_DOUBLE_EQ(result.vx, 3.0);
  EXPECT_DOUBLE_EQ(result.wz, 0.5);
}

TEST(LimitDeceleration, LimitsReversal)
{
  const auto result = limitDeceleration(
    BodyVelocity{1.0, 0.0, 0.0}, BodyVelocity{-1.0, 0.0, 0.0}, 3.0, 0.05);
  EXPECT_NEAR(result.vx, 0.85, 1e-9);
}

TEST(LimitDeceleration, FrameRotationIsNotBraking)
{
  // 世界の座標で 2.5m/s のまま、機体（上半身）が 0.1 秒で 0.09rad 回る。機体の座標の指令は −0.09rad 回る
  const double turned = 0.09;
  const BodyVelocity previous{2.5, 0.0, 0.0};
  const BodyVelocity target{2.5 * std::cos(turned), -2.5 * std::sin(turned), 0.0};
  // 前の出力を今の座標へ直せば、同じ速度なので制限しない
  const auto result = limitDeceleration(inTurnedFrame(previous, turned), target, 2.0, 0.1);
  EXPECT_NEAR(result.vx, target.vx, 1e-9);
  EXPECT_NEAR(result.vy, target.vy, 1e-9);
  // 直さずに比べると、座標が回った分（約 0.22m/s）を減速とみなして、前の向きへ残してしまう
  const auto unrotated = limitDeceleration(previous, target, 2.0, 0.1);
  EXPECT_GT(std::atan2(unrotated.vy, unrotated.vx), std::atan2(target.vy, target.vx) + 0.005);
}

TEST(InTurnedFrame, KeepsWorldVelocity)
{
  const BodyVelocity velocity{1.0, 0.5, 0.3};
  const double yaw_before = 0.4;
  const double yaw_after = 1.1;
  const auto turned = inTurnedFrame(velocity, yaw_after - yaw_before);
  const auto world = [](const BodyVelocity & v, const double yaw) {
      return std::make_pair(
        std::cos(yaw) * v.vx - std::sin(yaw) * v.vy, std::sin(yaw) * v.vx + std::cos(yaw) * v.vy);
    };
  EXPECT_NEAR(world(turned, yaw_after).first, world(velocity, yaw_before).first, 1e-12);
  EXPECT_NEAR(world(turned, yaw_after).second, world(velocity, yaw_before).second, 1e-12);
  EXPECT_DOUBLE_EQ(turned.wz, 0.3);
}

TEST(LimitDeceleration, ReachesSmallTargetChange)
{
  const auto result = limitDeceleration(
    BodyVelocity{0.1, 0.0, 0.0}, BodyVelocity{0.0, 0.0, 0.0}, 3.0, 0.05);
  EXPECT_DOUBLE_EQ(result.vx, 0.0);
}
// 候補の速度 target（odom 座標系）が、止まる軌道か保ち続ける軌道のどちらかで範囲に入らないか
bool keepsOutOfReach(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose, const BodyVelocity & velocity,
  const BodyVelocity & target, const SafetyParams & params)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double vx = c * velocity.vx - s * velocity.vy;
  const double vy = s * velocity.vx + c * velocity.vy;
  const double tx = c * target.vx - s * target.vy;
  const double ty = s * target.vx + c * target.vy;
  const auto braking = firstReachableViolation(
    obstacles, pose, vx, vy, tx, ty, params.reachable.commit_time_s, 3.0, Footprint{}, params);
  const auto holding = firstReachableViolation(
    obstacles, pose, vx, vy, tx, ty, params.reachable.escape_horizon_s,
    params.reachable.escape_horizon_s, Footprint{}, params);
  return !braking.has_value() || !holding.has_value();
}

TEST(ReachableRadius, GrowsWithTimeAndTrackAge)
{
  const SafetyParams params;
  const Obstacle robot{0.0, 0.0, 0.0, 0.0, ObstacleKind::kRobot, 0.1, 0.0, 0.0};
  // 0.45 + 余裕 0.1 + 位置の標準偏差 0.1、1 秒後に 0.5 * 1.5 * 1^2 を足す
  EXPECT_NEAR(reachableRadius(robot, 0.0, params), 0.65, 1e-9);
  EXPECT_NEAR(reachableRadius(robot, 1.0, params), 1.4, 1e-9);
  Obstacle late = robot;
  late.age_s = 0.2;
  EXPECT_NEAR(reachableRadius(late, 0.0, params), 0.65 + 0.75 * 0.04, 1e-9);
  Obstacle uncertain = robot;
  uncertain.velocity_sigma = 1.0;
  EXPECT_NEAR(reachableRadius(uncertain, 1.0, params), 2.4, 1e-9);
  // 遅い不明の物体（止まった物）は加速しないとみなす
  const Obstacle box{0.0, 0.0, 0.0, 0.0, ObstacleKind::kUnknown, 0.0, 0.0, 0.0};
  EXPECT_NEAR(reachableRadius(box, 1.0, params), 0.6, 1e-9);
}

TEST(SuperviseCommand, PassesCommandWithRoomToStop)
{
  const std::vector<Obstacle> far{Obstacle{12.0, 4.0, 0.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity forward{3.0, 0.0, 0.2};
  const auto supervised = superviseCommand(
    far, PlanarPose{}, forward, forward, Footprint{}, SafetyParams{});
  EXPECT_FALSE(supervised.replaced);
  EXPECT_EQ(supervised.decision, Decision::kPass);
  EXPECT_DOUBLE_EQ(supervised.output.vx, 3.0);
  EXPECT_DOUBLE_EQ(supervised.output.wz, 0.2);
}

TEST(SuperviseCommand, SlowsForStoppedRobotThatMayStartMoving)
{
  // 端で止まったロボットの横を 3m/s で通る。等速の予測では当たらないが、相手が 1.5m/s^2 で動き出すと、
  // 止まるまでの 1.15 秒の間に範囲へ入るので、範囲に入らない速度に替える
  const std::vector<Obstacle> obstacles{Obstacle{3.0, 1.4, 0.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity forward{3.0, 0.0, 0.0};
  const SafetyParams params;
  const auto supervised = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  EXPECT_EQ(supervised.nominal.decision, Decision::kPass);
  EXPECT_TRUE(supervised.replaced);
  EXPECT_TRUE(supervised.feasible);
  EXPECT_EQ(supervised.decision, Decision::kStop);
  // 相手の側（+y）へは寄らない
  EXPECT_LE(supervised.output.vy, 1e-9);
  EXPECT_TRUE(keepsOutOfReach(obstacles, PlanarPose{}, forward, supervised.output, params));
  EXPECT_FALSE(keepsOutOfReach(obstacles, PlanarPose{}, forward, forward, params));
}

TEST(SuperviseCommand, ReplacedVelocityFollowsYaw)
{
  // 上と同じ場面を 90 度回した。機体座標系で同じ結果になる
  const std::vector<Obstacle> obstacles{Obstacle{-1.4, 3.0, 0.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity forward{3.0, 0.0, 0.0};
  const PlanarPose pose{0.0, 0.0, 1.5707963267948966};
  const auto rotated = superviseCommand(
    obstacles, pose, forward, forward, Footprint{}, SafetyParams{});
  const auto original = superviseCommand(
    std::vector<Obstacle>{Obstacle{3.0, 1.4, 0.0, 0.0, ObstacleKind::kRobot}}, PlanarPose{},
    forward, forward, Footprint{}, SafetyParams{});
  ASSERT_TRUE(rotated.replaced);
  EXPECT_NEAR(rotated.output.vx, original.output.vx, 1e-6);
  EXPECT_NEAR(rotated.output.vy, original.output.vy, 1e-6);
}

TEST(SuperviseCommand, AvoidsEnteringPathOfCrossingRobot)
{
  // 右前から自機の進路へ 0.8m/s で出てくるロボットの前を、3m/s で通る指令。等速の予測では当たらないが、
  // 相手が加速すると、止まる前に相手の進む道で範囲に入る。相手の側（-y）へ寄る成分を落とし、
  // 範囲に入らない速度のうち指令に最も近いものにする
  const std::vector<Obstacle> obstacles{Obstacle{3.5, -2.0, 0.0, 0.8, ObstacleKind::kRobot}};
  const BodyVelocity forward{3.0, 0.0, 0.0};
  const SafetyParams params;
  const auto supervised = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  EXPECT_EQ(supervised.nominal.decision, Decision::kPass);
  ASSERT_TRUE(supervised.replaced);
  EXPECT_TRUE(supervised.feasible);
  EXPECT_GE(supervised.output.vy, 0.0);
  EXPECT_TRUE(keepsOutOfReach(obstacles, PlanarPose{}, forward, supervised.output, params));
  EXPECT_FALSE(keepsOutOfReach(obstacles, PlanarPose{}, forward, forward, params));
}

TEST(SuperviseCommand, DoesNotMoveWhileStoppedAtGoal)
{
  // ゴールで止まっている間に正面から来るロボットは避けない（止まっていれば近づく向きに動かない）
  const std::vector<Obstacle> obstacles{Obstacle{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}};
  const auto supervised = superviseCommand(
    obstacles, PlanarPose{}, BodyVelocity{}, BodyVelocity{}, Footprint{}, SafetyParams{});
  EXPECT_FALSE(supervised.replaced);
  EXPECT_DOUBLE_EQ(supervised.output.vx, 0.0);
  EXPECT_DOUBLE_EQ(supervised.output.vy, 0.0);
}

TEST(SuperviseCommand, KeepsMotionAwayFromRobot)
{
  // 向かってくる相手から下がる指令は、範囲の中でも近づかないので替えない
  const std::vector<Obstacle> obstacles{Obstacle{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity backward{-1.0, 0.0, 0.0};
  const auto supervised = superviseCommand(
    obstacles, PlanarPose{}, BodyVelocity{}, backward, Footprint{}, SafetyParams{});
  EXPECT_FALSE(supervised.replaced);
  EXPECT_DOUBLE_EQ(supervised.output.vx, -1.0);
}

TEST(SuperviseCommand, UsesVelocityUncertaintyOfTrack)
{
  // 横に止まって見えるロボット。速度が定まっていれば通すが、追跡を始めた直後で速度が 1.5m/s ほど
  // 不確かなら、動いているかもしれないので替える
  Obstacle robot{3.0, 1.3, 0.0, 0.0, ObstacleKind::kRobot};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  const auto settled = superviseCommand(
    std::vector<Obstacle>{robot}, PlanarPose{}, forward, forward, Footprint{}, SafetyParams{});
  EXPECT_FALSE(settled.replaced);
  robot.velocity_sigma = 1.5;
  const auto uncertain = superviseCommand(
    std::vector<Obstacle>{robot}, PlanarPose{}, forward, forward, Footprint{}, SafetyParams{});
  EXPECT_TRUE(uncertain.replaced);
  EXPECT_LT(std::hypot(uncertain.output.vx, uncertain.output.vy), 2.0);
}

TEST(SuperviseCommand, ChoosesLargestSeparationWhenNothingKeepsOut)
{
  // 正面の近くから 2m/s で来る相手へ 2m/s で進んでいる。どの候補でも止まる前に範囲へ入るので、
  // 範囲から離れる距離が最大の候補を選ぶ。真っすぐ止まるより、減速しながら横へ外れるほうが離れる。
  // 正面の相手には減速だけで応じる設定（oncoming.brake_only、既定）では横へ外れないので、元の動き
  // （brake_only: false）で確かめる。既定の動きは OncomingBrakeOnly の試験で確かめる
  const std::vector<Obstacle> obstacles{Obstacle{1.6, 0.0, -2.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  SafetyParams params;
  params.oncoming.brake_only = false;
  const auto supervised = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  EXPECT_TRUE(supervised.replaced);
  EXPECT_FALSE(supervised.feasible);
  EXPECT_LT(supervised.output.vx, 1.9);
  EXPECT_GT(std::abs(supervised.output.vy), 0.5);
}

TEST(OncomingBrakeOnly, HeadOnRobotIsAnsweredByBrakingStraight)
{
  // 上と同じ形（正面の 1.6m 先から 2m/s で来る相手へ 2m/s で進む）。既定では横へ外れず、今の向きのまま
  // 減速する（道を開けない）
  const std::vector<Obstacle> obstacles{Obstacle{1.6, 0.0, -2.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  const auto supervised = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, SafetyParams{});
  EXPECT_TRUE(supervised.replaced);
  EXPECT_LT(supervised.output.vx, 2.0);
  EXPECT_GE(supervised.output.vx, 0.0);
  EXPECT_NEAR(supervised.output.vy, 0.0, 1e-9);
}

TEST(OncomingBrakeOnly, OtherCausesKeepEscapeCandidates)
{
  // 動いているとみなせない追跡（始めた直後など）は正面の相手として扱わず、元の候補から選ぶ
  Obstacle uncertain{1.6, 0.0, -2.0, 0.0, ObstacleKind::kRobot};
  uncertain.moving_reliable = false;
  const BodyVelocity forward{2.0, 0.0, 0.0};
  const auto supervised = superviseCommand(
    {uncertain}, PlanarPose{}, forward, forward, Footprint{}, SafetyParams{});
  EXPECT_TRUE(supervised.replaced);
  EXPECT_GT(std::abs(supervised.output.vy), 0.5);
}

TEST(OncomingBrakeOnly, HeadOnNeedsOppositeCourseAndOwnLine)
{
  using core_planning::safety::isHeadOnObstacle;
  const SafetyParams params;
  const Footprint footprint;
  const PlanarPose pose;
  // 正面で、進む線の上
  EXPECT_TRUE(isHeadOnObstacle(
      Obstacle{4.0, 0.5, -2.0, 0.0, ObstacleKind::kRobot}, pose, 2.0, 0.0, footprint, params));
  // 進む線から横へ 2m（自機の半幅 0.4 + 余裕 0.1 + 相手 0.45 より外）
  EXPECT_FALSE(isHeadOnObstacle(
      Obstacle{4.0, 2.0, -2.0, 0.0, ObstacleKind::kRobot}, pose, 2.0, 0.0, footprint, params));
  // 同じ向き、後ろ、人、止まっている自機
  EXPECT_FALSE(isHeadOnObstacle(
      Obstacle{4.0, 0.0, 1.0, 0.0, ObstacleKind::kRobot}, pose, 2.0, 0.0, footprint, params));
  EXPECT_FALSE(isHeadOnObstacle(
      Obstacle{-4.0, 0.0, -2.0, 0.0, ObstacleKind::kRobot}, pose, 2.0, 0.0, footprint, params));
  EXPECT_FALSE(isHeadOnObstacle(
      Obstacle{4.0, 0.0, -2.0, 0.0, ObstacleKind::kHuman}, pose, 2.0, 0.0, footprint, params));
  EXPECT_FALSE(isHeadOnObstacle(
      Obstacle{4.0, 0.0, -2.0, 0.0, ObstacleKind::kRobot}, pose, 0.0, 0.0, footprint, params));
}

TEST(ImpactMitigation, FindsFastHitOnStoppedRobot)
{
  using core_planning::safety::findImpactThreat;
  const SafetyParams params;
  const Footprint footprint;
  // 止まっている自機へ、3m/s の相手が 3m 先から来る。前の端（0.4m）と相手（0.45m）の隙間 2.15m を
  // 約 0.72 秒で詰め、3m/s で当たる
  const auto threat = findImpactThreat(
    {Obstacle{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}}, PlanarPose{}, 0.0, 0.0, footprint,
    params);
  ASSERT_TRUE(threat.has_value());
  EXPECT_NEAR(threat->time_s, 2.15 / 3.0, 0.06);
  EXPECT_NEAR(threat->impact_speed_mps, 3.0, 1e-6);
  // 遅い相手（1m/s）は下限 1.5m/s より遅い。遠い相手は 1 秒以内に当たらない
  EXPECT_FALSE(findImpactThreat(
      {Obstacle{1.5, 0.0, -1.0, 0.0, ObstacleKind::kRobot}}, PlanarPose{}, 0.0, 0.0, footprint,
      params).has_value());
  EXPECT_FALSE(findImpactThreat(
      {Obstacle{6.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}}, PlanarPose{}, 0.0, 0.0, footprint,
      params).has_value());
  // 動いているとみなせない追跡は見ない
  Obstacle uncertain{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot};
  uncertain.moving_reliable = false;
  EXPECT_FALSE(
    findImpactThreat({uncertain}, PlanarPose{}, 0.0, 0.0, footprint, params).has_value());
}

TEST(ImpactMitigation, MovesALittleToSoftenAndStopsWithinShift)
{
  using core_planning::safety::ImpactMitigator;
  const SafetyParams params;
  const Footprint footprint;
  ImpactMitigator mitigator;
  PlanarPose pose;
  BodyVelocity velocity;
  Obstacle other{3.0, 0.3, -3.0, 0.0, ObstacleKind::kRobot};
  constexpr double kDt = 0.05;
  bool started = false;
  double max_shift = 0.0;
  for (int i = 0; i < 40; ++i) {
    const auto command = mitigator.update(
      i * kDt, {other}, pose, velocity, footprint, params, nullptr, PlanarPose{});
    if (command.has_value()) {
      started = true;
      EXPECT_LT(mitigator.chosenImpact(), mitigator.baselineImpact() - 0.3 + 1e-9);
      velocity = *command;
    } else {
      velocity = BodyVelocity{};
    }
    pose.x += velocity.vx * kDt;
    pose.y += velocity.vy * kDt;
    other.x += other.vx * kDt;
    max_shift = std::max(max_shift, std::hypot(pose.x, pose.y));
  }
  EXPECT_TRUE(started);
  // 少しだけ動く（0.5m と、指令の遅れの分）
  EXPECT_GT(max_shift, 0.1);
  EXPECT_LT(max_shift, 0.6);
}

TEST(ImpactMitigation, DisabledDoesNothing)
{
  using core_planning::safety::ImpactMitigator;
  SafetyParams params;
  params.mitigation.enabled = false;
  ImpactMitigator mitigator;
  EXPECT_FALSE(
    mitigator.update(
      0.0, {Obstacle{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}}, PlanarPose{}, BodyVelocity{},
      Footprint{}, params, nullptr, PlanarPose{}).has_value());
}

TEST(SuperviseCommand, DisabledReachableKeepsTimeToCollisionResult)
{
  SafetyParams params;
  params.reachable.enabled = false;
  const std::vector<Obstacle> obstacles{Obstacle{3.0, 1.4, 0.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity forward{3.0, 0.0, 0.0};
  const auto supervised = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  EXPECT_FALSE(supervised.replaced);
  EXPECT_DOUBLE_EQ(supervised.output.vx, 3.0);
}
// フィールド（柵）に、x ∈ [x0, x1]、y ∈ [y0, y1] の箱を足した距離の場（格子 0.05m）
core_planning::DistanceField fieldWithBoxes(
  const std::vector<std::vector<double>> & boxes)
{
  auto grid = core_planning::makeFieldGrid(
    core_planning::FieldBounds{-12.0, 12.0, -7.5, 7.5}, 0.05);
  const auto & info = grid.info();
  for (int cx = 0; cx < info.width; ++cx) {
    for (int cy = 0; cy < info.height; ++cy) {
      const double x = info.centerX(cx);
      const double y = info.centerY(cy);
      for (const auto & box : boxes) {
        if (x >= box[0] && x <= box[1] && y >= box[2] && y <= box[3]) {
          grid.setOccupied(core_planning::GridCell{cx, cy});
        }
      }
    }
  }
  return core_planning::computeDistanceField(grid);
}

// 出した速度（機体座標系）へ max_decel で近づき commit_time_s 保ってから止まる軌道の、静止物までの
// 最小距離（向きは変えない）。安全停止の候補の確かめと同じ運動を、テストの側で別に進める
double brakingStaticClearance(
  const core_planning::DistanceField & field, const PlanarPose & pose,
  const BodyVelocity & velocity, const BodyVelocity & target, const SafetyParams & params)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  double vx = c * velocity.vx - s * velocity.vy;
  double vy = s * velocity.vx + c * velocity.vy;
  const double tx = c * target.vx - s * target.vy;
  const double ty = s * target.vx + c * target.vy;
  constexpr double kDt = 0.01;
  const double max_change = params.max_decel_mps2 * kDt;
  PlanarPose current = pose;
  double closest = std::numeric_limits<double>::infinity();
  for (double t = 0.0; t < 5.0; t += kDt) {
    const bool hold = t < params.reachable.commit_time_s;
    const double gx = hold ? tx - vx : -vx;
    const double gy = hold ? ty - vy : -vy;
    const double change = std::hypot(gx, gy);
    const double ratio = change > max_change ? max_change / change : 1.0;
    vx += gx * ratio;
    vy += gy * ratio;
    current.x += vx * kDt;
    current.y += vy * kDt;
    closest = std::min(closest, staticClearance(field, PlanarPose{}, current, Footprint{}));
    if (!hold && std::hypot(vx, vy) < 1e-6) {
      break;
    }
  }
  return closest;
}

TEST(StaticCheck, EscapeDoesNotHeadIntoNearbyBox)
{
  // 第2段の obstacle_goals に似せた場面。2m/s で進む自機の右前から、相手が 1.5m/s で横切ってくる。
  // 左には箱があり、機体の左の面から 0.2m。距離の場がなければ、相手の範囲から離れる左（箱）へ寄る
  // 候補を選ぶ。距離の場があれば、止まる軌道で箱までの余裕 0.1m を割る候補は選ばない
  const auto field = fieldWithBoxes({{-1.0, 3.0, 0.6, 1.6}});
  const std::vector<Obstacle> obstacles{Obstacle{1.8, -1.8, 0.0, 1.5, ObstacleKind::kRobot}};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  const SafetyParams params;
  ASSERT_NEAR(staticClearance(field, PlanarPose{}, PlanarPose{}, Footprint{}), 0.2, 0.026);

  const auto without = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  ASSERT_TRUE(without.replaced);
  EXPECT_FALSE(without.static_checked);
  EXPECT_GT(without.output.vy, 0.5);
  EXPECT_LT(brakingStaticClearance(field, PlanarPose{}, forward, without.output, params), 0.1);

  const auto with = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params, &field, PlanarPose{});
  ASSERT_TRUE(with.replaced);
  EXPECT_TRUE(with.static_checked);
  EXPECT_TRUE(with.static_ok);
  EXPECT_TRUE(with.feasible);
  EXPECT_LE(with.output.vy, 0.05);
  EXPECT_GE(with.static_clearance_m, params.static_check.margin_m);
  EXPECT_GE(
    brakingStaticClearance(field, PlanarPose{}, forward, with.output, params),
    params.static_check.margin_m - 0.03);
  EXPECT_TRUE(keepsOutOfReach(obstacles, PlanarPose{}, forward, with.output, params));
}

TEST(StaticCheck, MapFromOdomMovesTheBox)
{
  // 上と同じ場面を、odom が地図から (5, 2) ずれ、90 度回った座標で表す。地図の箱は同じ向きに見える
  const PlanarPose map_from_odom{5.0, 2.0, 1.5707963267948966};
  // odom の (x, y) は地図の (5 - y, 2 + x)。箱は地図で、odom の x ∈ [-1, 3]、y ∈ [0.6, 1.6] に当たる所
  const auto field = fieldWithBoxes({{3.4, 4.4, 1.0, 5.0}});
  const std::vector<Obstacle> obstacles{Obstacle{1.8, -1.8, 0.0, 1.5, ObstacleKind::kRobot}};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  const SafetyParams params;
  ASSERT_NEAR(staticClearance(field, map_from_odom, PlanarPose{}, Footprint{}), 0.2, 0.026);
  const auto with = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params, &field, map_from_odom);
  ASSERT_TRUE(with.replaced);
  EXPECT_TRUE(with.static_ok);
  EXPECT_LE(with.output.vy, 0.05);
}

TEST(StaticCheck, NarrowCorridorWithoutRobotsUnchanged)
{
  // 幅 1.2m（片側 0.2m）の通路を 2.5m/s で進む。相手がいない、または遠い間は、指令をそのまま通す
  // （距離の場は指令を替えるときだけ使い、通す指令は確かめない）
  const auto field = fieldWithBoxes({{-5.0, 5.0, 0.6, 7.6}, {-5.0, 5.0, -7.6, -0.6}});
  const BodyVelocity forward{2.5, 0.0, 0.1};
  const SafetyParams params;
  for (const auto & obstacles : {
      std::vector<Obstacle>{},
      std::vector<Obstacle>{Obstacle{-8.0, 0.0, 0.0, 0.0, ObstacleKind::kRobot}}})
  {
    const auto without = superviseCommand(
      obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
    const auto with = superviseCommand(
      obstacles, PlanarPose{}, forward, forward, Footprint{}, params, &field, PlanarPose{});
    EXPECT_FALSE(with.replaced);
    EXPECT_FALSE(with.static_checked);
    EXPECT_DOUBLE_EQ(with.output.vx, without.output.vx);
    EXPECT_DOUBLE_EQ(with.output.vy, without.output.vy);
    EXPECT_DOUBLE_EQ(with.output.wz, without.output.wz);
    EXPECT_DOUBLE_EQ(with.output.vx, 2.5);
  }
}

TEST(StaticCheck, NarrowCorridorWithApproachingRobotKeepsOffWalls)
{
  // 幅 1.2m（片側 0.2m）の通路（x ≤ 1.5）を 2m/s で進み、出口の右から相手が 1.5m/s で横切ってくる。
  // 距離の場がなければ、相手から離れる左の壁へ寄る候補を選ぶ。距離の場があれば、壁へ寄らない
  // （今の距離から格子 1 段より近づかない）候補で減速する
  const auto field = fieldWithBoxes({{-5.0, 1.5, 0.6, 7.6}, {-5.0, 1.5, -7.6, -0.6}});
  const std::vector<Obstacle> obstacles{Obstacle{1.8, -1.8, 0.0, 1.5, ObstacleKind::kRobot}};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  const SafetyParams params;
  const double now = staticClearance(field, PlanarPose{}, PlanarPose{}, Footprint{});
  ASSERT_NEAR(now, 0.2, 0.026);
  const auto without = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  ASSERT_TRUE(without.replaced);
  EXPECT_GT(without.output.vy, 0.5);
  const auto with = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params, &field, PlanarPose{});
  ASSERT_TRUE(with.replaced);
  EXPECT_TRUE(with.static_ok);
  const double required = std::min(
    params.static_check.margin_m, now - params.static_check.tolerance_m);
  EXPECT_GE(with.static_clearance_m, required);
  EXPECT_GE(
    brakingStaticClearance(field, PlanarPose{}, forward, with.output, params), required - 0.03);
  EXPECT_LE(std::abs(with.output.vy), 0.05);
  EXPECT_LT(with.output.vx, 2.0);
}

TEST(StaticCheck, AlreadyTooCloseKeepsMotionParallelToWall)
{
  // 壁まで 0.05m（余裕より近い）で壁と平行に進んでいる。今より近づかない候補は使えるので、壁から
  // 離れるか平行な候補を選び、壁へは寄らない
  const auto field = fieldWithBoxes({{-5.0, 5.0, 0.45, 7.6}});
  const std::vector<Obstacle> obstacles{Obstacle{1.8, -1.8, 0.0, 1.5, ObstacleKind::kRobot}};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  const SafetyParams params;
  const double now = staticClearance(field, PlanarPose{}, PlanarPose{}, Footprint{});
  ASSERT_LT(now, params.static_check.margin_m);
  const auto with = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params, &field, PlanarPose{});
  ASSERT_TRUE(with.replaced);
  EXPECT_TRUE(with.static_ok);
  // 壁のセル（距離 0）には入らない
  EXPECT_GT(with.static_clearance_m, 0.0);
  EXPECT_GT(brakingStaticClearance(field, PlanarPose{}, forward, with.output, params), 0.0);
  EXPECT_LE(with.output.vy, 0.1);
}

TEST(StaticCheck, WithoutFieldOrDisabledSameAsBefore)
{
  // 距離の場がないとき、static_check を切ったときは、元の選び方と同じ
  const auto field = fieldWithBoxes({{-1.0, 3.0, 0.6, 1.6}});
  const std::vector<Obstacle> obstacles{Obstacle{1.8, -1.8, 0.0, 1.5, ObstacleKind::kRobot}};
  const BodyVelocity forward{2.0, 0.0, 0.0};
  SafetyParams params;
  const auto original = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  const auto no_field = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params, nullptr, PlanarPose{});
  params.static_check.enabled = false;
  const auto disabled = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params, &field, PlanarPose{});
  for (const auto & result : {no_field, disabled}) {
    EXPECT_EQ(result.replaced, original.replaced);
    EXPECT_EQ(result.feasible, original.feasible);
    EXPECT_FALSE(result.static_checked);
    EXPECT_DOUBLE_EQ(result.output.vx, original.output.vx);
    EXPECT_DOUBLE_EQ(result.output.vy, original.output.vy);
  }
}

TEST(StaticClearance, MeasuresCornersAndCenter)
{
  // 柵（y = 7.5 の外周 1 セル）に、機体の左の面が 0.5m まで寄った姿勢。角の距離は格子の段（0.05m）の範囲
  const auto field = fieldWithBoxes({});
  const double clearance = staticClearance(
    field, PlanarPose{}, PlanarPose{0.0, 7.5 - 0.4 - 0.5, 0.0}, Footprint{});
  EXPECT_NEAR(clearance, 0.5, 0.05);
  // 45 度回すと角が柵に近づく
  const double rotated = staticClearance(
    field, PlanarPose{}, PlanarPose{0.0, 7.5 - 0.4 - 0.5, 0.7853981633974483}, Footprint{});
  EXPECT_LT(rotated, clearance - 0.1);
}

TEST(SuperviseCommand, LimitsBlindSideEscape)
{
  // SlowsForStoppedRobotThatMayStartMoving を前後に反転した場面。真後ろ（見えない側）へ 3m/s で進んでいて、
  // 左後ろに止まったロボットの範囲に入るので指令を替える。上限がなければ後ろへ 3m/s の候補を選ぶが、
  // 見えない側へ向かう候補は 0.5m/s までに縮める
  const std::vector<Obstacle> obstacles{Obstacle{-3.0, -1.4, 0.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity backward{-3.0, 0.0, 0.0};
  SafetyParams params;
  params.blind_side.enabled = false;
  const auto unlimited = superviseCommand(
    obstacles, PlanarPose{}, backward, backward, Footprint{}, params);
  ASSERT_TRUE(unlimited.replaced);
  EXPECT_GT(std::hypot(unlimited.output.vx, unlimited.output.vy), 2.0);
  EXPECT_LT(unlimited.output.vx, 0.0);

  params.blind_side.enabled = true;
  for (const double yaw : {0.0, 1.0}) {
    // 機体の向きを回しても、機体座標系で同じ結果になる
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    std::vector<Obstacle> rotated = obstacles;
    rotated[0].x = c * obstacles[0].x - s * obstacles[0].y;
    rotated[0].y = s * obstacles[0].x + c * obstacles[0].y;
    const auto limited = superviseCommand(
      rotated, PlanarPose{0.0, 0.0, yaw}, backward, backward, Footprint{}, params);
    ASSERT_TRUE(limited.replaced) << yaw;
    const double speed = std::hypot(limited.output.vx, limited.output.vy);
    const double angle = std::abs(std::atan2(limited.output.vy, limited.output.vx));
    // 見えない側（前から 96° より外）なら 0.5m/s まで
    if (angle > params.blind_side.visible_half_angle_deg * M_PI / 180.0) {
      EXPECT_LE(speed, params.blind_side.speed_mps + 1e-9) << yaw;
    }
    EXPECT_LT(speed, 1.0) << yaw;
  }
}

TEST(SuperviseCommand, BlindSideLimitKeepsForwardEscape)
{
  // 前へ進む場面（SlowsForStoppedRobotThatMayStartMoving）では、上限の有無で選ぶ候補が変わらない
  const std::vector<Obstacle> obstacles{Obstacle{3.0, 1.4, 0.0, 0.0, ObstacleKind::kRobot}};
  const BodyVelocity forward{3.0, 0.0, 0.0};
  SafetyParams params;
  const auto limited = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  params.blind_side.enabled = false;
  const auto unlimited = superviseCommand(
    obstacles, PlanarPose{}, forward, forward, Footprint{}, params);
  ASSERT_TRUE(limited.replaced);
  EXPECT_DOUBLE_EQ(limited.output.vx, unlimited.output.vx);
  EXPECT_DOUBLE_EQ(limited.output.vy, unlimited.output.vy);
}

TEST(ImpactMitigation, NoRoomMeansNoMove)
{
  using core_planning::safety::ImpactMitigator;
  // 左右と後ろを、機体から 0.15m の箱で囲む。前（相手の来る側）へ動くと速く当たるので、動かない
  const auto field = fieldWithBoxes(
    {{-1.0, -0.55, -1.0, 1.0}, {-1.0, 1.0, 0.55, 1.0}, {-1.0, 1.0, -1.0, -0.55}});
  ImpactMitigator mitigator;
  const auto command = mitigator.update(
    0.0, {Obstacle{3.0, 0.0, -3.0, 0.0, ObstacleKind::kRobot}}, PlanarPose{}, BodyVelocity{},
    Footprint{}, SafetyParams{}, &field, PlanarPose{});
  EXPECT_FALSE(command.has_value());
}
}  // namespace
