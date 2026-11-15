// 計画とは独立に、速度指令の安全を判定する処理。ROS に依存しない。
//
// 指令どおりに進んだときの衝突までの時間（TTC）で、そのまま通す、減速する、止めるを決める。
// そのうえで、相手を「推定の遅れ、速度の不確かさ、加速度の上限の分だけ広がる範囲」として扱い、
// 指令の後に相手の方向へ動いたまま範囲に入らずに止まれるかを確かめる（方向別の受動安全）。
// 止まれなければ、範囲に入らない候補の速度のうち指令に最も近いものに替える。
// 静止物の距離の場を渡せば、替える候補は静止物（地図と柵）にも寄らないものから選ぶ。
// 替える候補の速度には、局所計画と同じ見えない側の速さの上限（blind_side.hpp）をかける。

#ifndef CORE_PLANNING__SAFETY_HPP_
#define CORE_PLANNING__SAFETY_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "core_planning/blind_side.hpp"
#include "core_planning/footprint.hpp"
#include "core_planning/grid_map.hpp"

namespace core_planning::safety
{
enum class ObstacleKind : std::uint8_t {kUnknown, kRobot, kHuman};

// 追跡中の相手。位置と速度は自機の姿勢と同じ座標系（odom）
struct Obstacle
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  ObstacleKind kind{ObstacleKind::kUnknown};
  // 位置の標準偏差 [m]
  double position_sigma{0.0};
  // 速度の推定の不確かさ [m/s]（追跡を始めた直後などに正）
  double velocity_sigma{0.0};
  // 位置と速度を推定した時刻から今までの時間 [s]
  double age_s{0.0};
  // 速度を、動いている相手の判定（正面の相手、衝撃の緩和）に使ってよいか（TrackMotionFilter）。
  // 到達範囲の確かめには使わない（追跡を始めた直後の速い相手も、今までどおり速度で広げる）
  bool moving_reliable{true};
};

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 機体座標系の速度
struct BodyVelocity
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct Footprint
{
  double half_length_m{0.4};
  double half_width_m{0.4};
  // 駆動部の形（orientFootprint で決める）。大きさが 0 なら上の長方形を base_link の向きのまま使う
  ChassisShape chassis{};
};

// 相手の到達範囲で確かめる設定
struct ReachableParams
{
  bool enabled{true};
  // 相手の加速度の上限。範囲の半径に 0.5 * a * t^2 を足す
  double obstacle_accel_mps2{1.5};
  // 速度の不確かさに掛ける倍率。範囲の半径に gain * velocity_sigma * t を足す
  double velocity_sigma_gain{1.0};
  // 指令を保つ時間。この後に max_decel で止まれるかを確かめる（周期と指令の遅れ）
  double commit_time_s{0.15};
  // 候補の速度を保って確かめる時間
  double escape_horizon_s{1.0};
  // 候補の速度は、今の速度から max_decel * この時間まで変えたもの
  double escape_change_s{0.5};
  // 候補の速度の向きの数
  int directions{16};
};

// 相手の到達範囲のために指令を候補の速度に替えるとき、候補を静止物（地図と柵）で確かめる設定。
// 候補を commit_time_s 保ってから max_decel で止まる軌道で、機体の 4 隅と中心の静止物までの最小距離が
// min(margin_m, 今の距離 - tolerance_m) 以上で、かつ 0 より大きい（静止物のセルに入らない）なら、
// 静止物について使える
struct StaticCheckParams
{
  bool enabled{true};
  // 機体と静止物の間に保つ余裕 [m]
  double margin_m{0.1};
  // 今の距離から近づくのを許す距離 [m]。距離の場の格子の段で、平行に動いても距離が変わるため
  double tolerance_m{0.05};
};

// 正面から来るロボット（進む向きの差 150° 以上で、自機の進む線の上）だけが原因で指令を替えるときの
// 設定。brake_only なら、候補を今の向きのまま減速するものに限り、横へ逃がさない（道を開けない。
// 2026-11-14 のユーザーの判断）。false で元の動き（範囲に入らない候補のうち指令に最も近いもの）
struct OncomingSafetyParams
{
  bool brake_only{true};
};

// 高速で当てられそうなときの衝撃の緩和（2026-11-14 のユーザーの判断）。止まっているか遅い（0.5m/s 未満の）
// ときに、動いているとみなせる相手が、今の速度のままだと horizon_s 以内に機体と重なり、そのときの近づく
// 速さが min_impact_speed_mps 以上なら、相手から離れる向きのうち当たるときの近づく速さが最も小さくなる
// ものへ、緩和を始めた位置から max_shift_m まで、max_speed_mps までの速さで動く。動く先で静止物の余裕
// （static_check）が取れ、ほかの相手の到達範囲へ向かわず、当たるときに自機が相手の方向へ動いていない
// （自機が原因の接触にならない）候補だけを使う。止まっている機体でも動かす（ほかの判定の「止まった機体は
// 動き出さない」の例外）。走っている間は、計画の停止線と安全停止の減速で応じる
struct MitigationParams
{
  bool enabled{true};
  double min_impact_speed_mps{1.5};
  double horizon_s{1.0};
  double max_shift_m{0.5};
  double max_speed_mps{1.5};
  // 何もしないときより、当たるときの近づく速さがこれ以上小さくならなければ動かない [m/s]
  double min_reduction_mps{0.3};
  // 候補の向きの数
  int directions{16};
};

struct SafetyParams
{
  double ttc_slow_s{1.5};
  double ttc_stop_s{0.6};
  double footprint_margin_m{0.1};
  double max_decel_mps2{3.0};
  ReachableParams reachable;
  StaticCheckParams static_check;
  OncomingSafetyParams oncoming;
  MitigationParams mitigation;
  // 替える候補の速度のうち、機体の前から見えない側へ向かうものの速さの上限
  BlindSideParams blind_side;
};

enum class Decision : std::uint8_t {kPass, kSlow, kStop};

struct SafetyResult
{
  Decision decision{Decision::kPass};
  // 速度指令に掛ける倍率
  double speed_scale{1.0};
  // 衝突する相手へ向かう向き（機体座標系の単位ベクトル）。この向きの速度だけを抑えれば、
  // 横へ避ける動きを止めずに済む。衝突の予測がなければ 0
  double threat_x{0.0};
  double threat_y{0.0};
  // 指令どおりに進んだときの衝突までの時間。衝突の予測がなければ負
  double ttc_s{-1.0};
};

// 相手の種類ごとの半径 [m]
double obstacleRadius(ObstacleKind kind);

// 速度指令を確かめた結果
struct SupervisedCommand
{
  // 衝突までの時間による判定
  SafetyResult nominal;
  // 最終の判定。範囲の確かめで指令を替えたときは kStop
  Decision decision{Decision::kPass};
  // 出す速度（機体座標系）。減速度の制限はかける前
  BodyVelocity output;
  // 相手の範囲に入るので、指令を候補の速度に替えたか
  bool replaced{false};
  // 替えた候補が、どの相手の範囲にも入らないか。false なら離れる距離が最大の候補
  bool feasible{true};
  // 替えた候補を静止物で確かめたか（距離の場があり、static_check.enabled のとき）
  bool static_checked{false};
  // 替えた候補が、静止物について使えるか。false なら静止物までの最小距離が最大の候補。
  // 確かめていなければ true
  bool static_ok{true};
  // 替えた候補を保ってから止まる軌道での、静止物までの最小距離 [m]。確かめていなければ負
  double static_clearance_m{-1.0};
};

// 指令どおりに進んだときの衝突を調べ、通す、減速する、止めるを決める。
// 相手が向かってきていて止まるほうが早く当たり、指令が ttc_stop_s 以内には当たらないなら、
// 避ける動きを止めないよう指令を通す。
SafetyResult evaluateSafety(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose,
  const BodyVelocity & own_velocity, const BodyVelocity & commanded, const Footprint & footprint,
  const SafetyParams & params);

// 相手へ向かう向きの速度に倍率を掛ける。向きがなければ全体に掛ける
BodyVelocity applySpeedScale(const BodyVelocity & commanded, const SafetyResult & result);

// 時刻 t（今から）の相手の到達範囲の半径。相手の大きさと余裕を含む
double reachableRadius(const Obstacle & obstacle, double t, const SafetyParams & params);

// 速度の目標 target（odom 座標系）へ max_decel で近づき、hold_s 秒保ったあと max_decel で止まる軌道で、
// 相手の方向へ動いたまま相手の到達範囲に入る最初の時刻を返す。入らなければ nullopt。
// hold_s が horizon_s 以上なら止まらずに horizon_s まで保つ
std::optional<double> firstReachableViolation(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose, double velocity_x,
  double velocity_y, double target_x, double target_y, double hold_s, double horizon_s,
  const Footprint & footprint, const SafetyParams & params);

// evaluateSafety と applySpeedScale のあとに到達範囲で確かめ、出す速度を決める。
// 指令の後に止まれないなら、到達範囲に入らない候補の速度のうち指令に最も近いものを選ぶ。
// どれも入るなら、範囲から離れる距離が最大の候補を選ぶ。候補の速度は、機体の前（pose.yaw）から見えない側へ
// 向かうなら blind_side の上限までに縮めてから確かめる。
// field は静止物の距離の場（地図の座標系）、map_from_odom は地図の座標系での odom の姿勢。field があり
// static_check.enabled なら、候補を選ぶときだけ静止物も確かめる（指令をそのまま通すときは見ない）。
// 1. 範囲に入らず、静止物について使える候補のうち、指令に最も近いもの
// 2. なければ、静止物について使える候補のうち、範囲から離れる距離が最大のもの
// 3. 静止物について使える候補がなければ、静止物までの最小距離が最大のもの
// field が nullptr なら静止物を見ない
SupervisedCommand superviseCommand(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose,
  const BodyVelocity & own_velocity, const BodyVelocity & commanded, const Footprint & footprint,
  const SafetyParams & params, const DistanceField * field = nullptr,
  const PlanarPose & map_from_odom = PlanarPose{});

// 機体（4 隅と中心）と静止物の最小距離。pose は odom 座標系、map_from_odom は地図の座標系での odom の姿勢
double staticClearance(
  const DistanceField & field, const PlanarPose & map_from_odom, const PlanarPose & pose,
  const Footprint & footprint);

// 並進の減速を max_decel 以内にする。加速は制限しない
BodyVelocity limitDeceleration(
  const BodyVelocity & previous, const BodyVelocity & target, double max_decel, double dt);

// 正面から来るロボットか。動いているとみなせる（moving_reliable）ロボットが 0.3m/s 以上で、自機の速度
// （odom の vx, vy。0.3m/s 以上）と進む向きが 150° 以上違い、自機の前方で、進む線からの横のずれが
// 自機の半幅と余裕と相手の大きさ以内
bool isHeadOnObstacle(
  const Obstacle & obstacle, const PlanarPose & pose, double velocity_x, double velocity_y,
  const Footprint & footprint, const SafetyParams & params);

// 高速で当てられそうな相手
struct ImpactThreat
{
  std::size_t index{0U};
  // 当たるまでの時間 [s] と、そのときの近づく速さ（中心を結ぶ向きの相対速度）[m/s]
  double time_s{0.0};
  double impact_speed_mps{0.0};
};

// 自機が今の速度（odom の vx, vy）のまま、相手が等速で進むとき、mitigation.horizon_s 以内に機体
// （余裕なし）と相手が重なり、近づく速さが mitigation.min_impact_speed_mps 以上になる相手のうち、
// 最も早く当たるもの。moving_reliable の相手だけを見る
std::optional<ImpactThreat> findImpactThreat(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose, double velocity_x,
  double velocity_y, const Footprint & footprint, const SafetyParams & params);

// 衝撃の緩和（MitigationParams）。緩和を始めた位置を覚え、そこから max_shift_m を超えて動かない
class ImpactMitigator
{
public:
  // 緩和するなら、出す速度（機体座標系）を返す。time_s は今の時刻 [s]。field は静止物の距離の場
  // （無ければ nullptr）。止まっているか遅い（0.5m/s 未満の）ときにだけ始め、始めた位置から
  // max_shift_m までで止まる
  std::optional<BodyVelocity> update(
    double time_s, const std::vector<Obstacle> & obstacles, const PlanarPose & pose,
    const BodyVelocity & own_velocity, const Footprint & footprint, const SafetyParams & params,
    const DistanceField * field, const PlanarPose & map_from_odom);

  // 直前の update で緩和したか
  bool active() const {return active_;}
  // 直前の update で緩和したときの、何もしないときと選んだ候補の当たる速さ [m/s]
  double baselineImpact() const {return baseline_impact_;}
  double chosenImpact() const {return chosen_impact_;}

  void reset()
  {
    origin_.reset();
    active_ = false;
  }

private:
  struct Origin
  {
    // 緩和を始めた位置（odom）と時刻
    double x{0.0};
    double y{0.0};
    double time_s{0.0};
  };
  std::optional<Origin> origin_;
  bool active_{false};
  double baseline_impact_{0.0};
  double chosen_impact_{0.0};
};
}  // namespace core_planning::safety

#endif  // CORE_PLANNING__SAFETY_HPP_
