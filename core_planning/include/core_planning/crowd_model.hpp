// シナリオの相手の中を走らせて、接触の頻度を Gazebo を使わずに統計で比べるモデル。ROS に依存しない。
//
// 計画と安全停止は実機と同じライブラリを使い、mppi_lite_node と safety_gate_node と同じ順に呼ぶ。
// 次のものは近似または省略する。
// - 追跡: 真の速度に一次遅れとノイズを足す。見失い、IDの入れ替わり、相手どうしの隠れは再現しない。
//   機体の陰（LiDAR から見える範囲の外）は model.visible_half_angle_deg で入れられる（既定は全周が見える）
// - 経路: 今の位置からゴールへの直線。静止物は柵だけで、止まった相手を回り込む経路は作らない
// - 機体: 加速度の上限だけ。指令の遅れは追跡の外挿の分しか入らない
//
// そのため接触は Gazebo より少なく出る。crowd_goals の既定の設定で、このモデルが 0.2〜0.3 回/100秒、
// Gazebo が約 1〜1.5 回/100秒だった。絶対値ではなく、パラメータの比較に使う。

#ifndef CORE_PLANNING__CROWD_MODEL_HPP_
#define CORE_PLANNING__CROWD_MODEL_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "core_planning/grid_map.hpp"
#include "core_planning/mppi.hpp"
#include "core_planning/safety.hpp"
#include "core_planning/track_motion.hpp"

namespace core_planning::crowd_model
{
struct Goal
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 相手の動き。start と end の間を余弦波の速度で往復する
struct Lane
{
  Point2 start;
  Point2 end;
  double peak_speed_mps{0.0};
  double phase{0.0};
  ObjectKind kind{ObjectKind::kRobot};
};

struct LaneState
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
};

struct Scenario
{
  Goal start;
  std::vector<Goal> goals;
  std::vector<Lane> lanes;
  // goals を 1 回りする制限時間
  double timeout_s{120.0};
  int repeat{1};
};

// 追跡の結果の近似
struct TrackingModel
{
  // 速度の推定の遅れ。2m/s2 で加速する相手に 0.5m/s 遅れるという Gazebo の結果に合わせた
  double velocity_lag_s{0.25};
  double velocity_noise_mps{0.15};
  double position_noise_m{0.05};
  // 走査の時刻から計画の時刻まで、推定した速度で外挿する時間
  double extrapolation_s{0.083};
  double position_sigma_m{0.12};
  // 機体の陰。LiDAR（機体の中心から前へ lidar_x_m）から見た相手の方位が、機体の前から
  // visible_half_angle_deg（相手の大きさの分を足す）より外なら見えない。180 で全周が見える（前の動き）
  double visible_half_angle_deg{180.0};
  double lidar_x_m{0.44};
  // 見えなくなった相手は、最後に見えた位置と速度から max_coast_s まで予測で保つ（位置の不確かさは
  // coast_sigma_growth_mps で増やす）。過ぎたら計画と安全停止に渡さない（tracking.yaml と同じ値）
  double max_coast_s{1.0};
  double coast_sigma_growth_mps{1.0};
};

// 計画と安全停止の設定。名前は applyParameter を通して、実機の設定ファイルと同じものを使う
struct StackParams
{
  MppiParams mppi;
  SpeedLimitParams speed_limit;
  StuckParams stuck;
  GoalApproachParams goal_approach;
  bool safety_enabled{true};
  safety::SafetyParams safety;
  safety::Footprint footprint;
  FieldBounds field;
  double grid_resolution_m{0.05};
  double max_speed_mps{3.0};
  double goal_tolerance_m{0.15};
  double goal_speed_tolerance_mps{0.2};
  double backoff_speed_mps{0.5};
  double plan_period_s{0.5};
  bool evade_while_stopped{false};
  // mppi_lite の mppi.evade_humans_while_stopped と同じ。false なら止まっている間は人も避けない
  bool evade_humans_while_stopped{false};
  // safety_gate の track_motion と同じ（正面の相手と衝撃の緩和の判定だけに使う）。モデルの追跡は
  // coasting を見えない間とし、速度の不確かさは 0 とする
  MotionFilterParams motion_filter;
  TrackingModel tracking;
};

struct RunResult
{
  int contacts{0};
  int own_fault_contacts{0};
  int goals_reached{0};
  int goals_total{0};
  double duration_s{0.0};
  double distance_m{0.0};
  double recovery_s{0.0};
  double min_gap_m{std::numeric_limits<double>::infinity()};
  double min_fence_clearance_m{std::numeric_limits<double>::infinity()};
  // 向かい合ってすれ違った回数。規則の側と、その逆の側
  int passes_on_rule_side{0};
  int passes_on_other_side{0};
  // 接触し始めたときの近づく速さ（中心を結ぶ向きの相対速度）の合計と、kHardImpactSpeedMps 以上の回数
  double impact_speed_sum_mps{0.0};
  int hard_contacts{0};
  // 衝撃の緩和を始めた回数と、停止線で止まり始めた回数
  int mitigations{0};
  int yields{0};
  // 人との接触の回数。機体の外形が人の中心から kHumanRegionM 以内にいた時間と、その間の最高の速さ
  int human_contacts{0};
  int human_own_fault_contacts{0};
  double human_region_s{0.0};
  double human_region_max_speed_mps{0.0};
};

struct Summary
{
  int runs{0};
  double contacts_per_100s{0.0};
  // 接触の頻度の 95% 信頼区間の半分の幅
  double contacts_per_100s_margin{0.0};
  double own_fault_per_100s{0.0};
  double contact_free_ratio{0.0};
  double deep_contact_ratio{0.0};
  double goals_reached_mean{0.0};
  int goals_total{0};
  double mean_duration_s{0.0};
  double mean_speed_mps{0.0};
  double mean_recovery_s{0.0};
  double min_fence_clearance_m{std::numeric_limits<double>::infinity()};
  int fence_touch_runs{0};
  int passes_on_rule_side{0};
  int passes_total{0};
  // 接触し始めたときの近づく速さの平均 [m/s] と、kHardImpactSpeedMps 以上で当たった頻度 [回/100秒]
  double mean_impact_speed_mps{0.0};
  double hard_contacts_per_100s{0.0};
  // 衝撃の緩和を始めた頻度と、停止線で止まり始めた頻度 [回/100秒]
  double mitigations_per_100s{0.0};
  double yields_per_100s{0.0};
  // 人との接触の頻度 [回/100秒]、機体の外形が人の中心から kHumanRegionM 以内にいた時間 [秒/100秒]、
  // その間の最高の速さ [m/s]（すべての実行の最大）
  double human_contacts_per_100s{0.0};
  double human_own_fault_per_100s{0.0};
  double human_region_per_100s{0.0};
  double human_region_max_speed_mps{0.0};
};

// 速く当たったとみなす、接触し始めたときの近づく速さ [m/s]（衝撃の緩和の下限と同じ）
inline constexpr double kHardImpactSpeedMps = 1.5;

// 人の近さを数える、人の中心から機体の外形までの距離 [m]。設定（speed_limit.human_radius_m）によらず
// 同じ値で数え、設定の違う build どうしを比べられるようにする
inline constexpr double kHumanRegionM = 1.0;

// sim（core_field_sim）の core_field_sim/src/moving_obstacles.cpp と同じ式。変えるときは両方を直す
LaneState laneStateAt(const Lane & lane, double time_s);

// 自機の箱と相手の間の距離。重なっていれば負。core_nav_check/scenario.py の footprint_gap と同じ
double gapToObject(double own_x, double own_y, double own_yaw, ObjectKind kind, double x, double y);

// 機体の角から柵の内側の面までの最小距離
double fenceClearance(const FieldBounds & field, double x, double y, double yaw);

// シナリオの YAML を読む。必要な項目がなければ std::invalid_argument を投げる
Scenario parseScenario(const YAML::Node & root);

// 名前が "mppi.samples" の形の値を設定する。名前を知らなければ false を返す。
// 値を読めなければ std::invalid_argument を投げる
bool applyParameter(StackParams & params, const std::string & name, const std::string & value);

// ROS のパラメータファイルの値を設定して、モデルで使わなかった名前を返す
std::vector<std::string> applyParameterFile(StackParams & params, const YAML::Node & root);

// 1 回走らせる。同じ seed なら同じ結果になる
RunResult simulateRun(
  const Scenario & scenario, const StackParams & params, const DistanceField & field,
  std::uint32_t seed);

Summary summarize(const std::vector<RunResult> & results);
}  // namespace core_planning::crowd_model

#endif  // CORE_PLANNING__CROWD_MODEL_HPP_
