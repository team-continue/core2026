#include "core_planning/mppi.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <numeric>
#include <sstream>
#include <utility>

#include "core_planning/drive_delay.hpp"
#include "core_planning/footprint.hpp"

namespace core_planning
{
namespace
{
constexpr int kControlSize = 3;
// passing_rule.weight が 1 のときの、1 ステップあたりの罰
constexpr double kPassingBaseCost = 10.0;
// すれ違い規則を適用する、相手の向かってくる速さの下限
constexpr double kOncomingSpeedMps = 0.3;
// 止まっている相手にすれ違いの規則を当てるのは、ゴールがその相手よりこの距離以上先にあるときだけ
// （経路に沿って）。止まった相手のすぐ先にあるゴールへ、規則の側から回り込んで近づけなくなるため
constexpr double kPassingGoalClearanceM = 1.0;
// 見通しの速度制限で、通り道にいるとみなす横の距離（機体の半分 + 相手の半径 + 余裕）
constexpr double kSpeedLimitCorridorM = 1.2;
// 平均の系列がこれ以上悪ければ、平均をやめて最良の系列を使う（weight_collision に対する割合）。
// 左右どちらにも避けられるとき、平均すると相手に向かって直進してしまう
constexpr double kMeanRejectRatio = 0.5;
// 温度の下限を決める、コストのばらつきに対する比率
constexpr double kAdaptiveTemperatureRatio = 0.2;
constexpr double kMinDirectionSpeedMps = 0.1;
// これより遅い相手には衝突コースの罰を与えない。止まった相手の位置はほぼ確かなので
// 位置のコストで足り、罰を与えると近くのゴールへ進めなくなる
constexpr double kMovingObjectSpeedMps = 0.3;
// 合流とみなす、進路の交わる角度の下限（sin 15°）。追跡の速度の向きのぶれは、3m/s の相手で約 3°、
// 1m/s の相手で約 9°（速度の誤差 0.15m/s）
constexpr double kMinMergeSine = 0.26;
// 進む向きの差がこれ以上（cos 150°）なら、横切りではなく正面とする。正面はすれ違い規則で側を決め、
// 自機が近づく向きに動いている間は避ける。30° 以内で交わる進路は交点が遠く、向きのぶれで大きく動く
constexpr double kHeadOnCos = -0.866;
// 自機の速度の、相手の方向の成分がこれを超えれば、その相手へ近づいているとみなす（avoid_being_hit が
// false のとき）。run_scenario.py の classify_contact は、近づく成分が 0 より大きく速さが 0.5m/s 以上の
// 接触を自機の原因とするので、同じ境界の 0 にする。仕様の例の 0.1m/s では、横からの接触のうち相手が
// わずかに前にいるものを数えず、crowd_goals のモデル（240 回、横切りの規則を入れる前）で自機が原因の
// 接触が 0.00 から 0.07 回/100秒に増えた。止まっている候補は速度がちょうど 0 になるので、0 でも数えない
constexpr double kApproachSpeedMps = 0.0;
// 柵に当たるコストの、相手に当たるコストに対する倍率
constexpr double kStaticCollisionScale = 3.0;
// 距離の場の勾配の大きさの下限。正しく求まれば約 1（斜めの角で約 0.7）で、占有の中、通路の真ん中の
// 尾根、平らな所ではほぼ 0 になる。これより小さければ向きが求まらないとして全方向の速さを使う
constexpr double kMinStaticGradient = 0.5;
// すれ違う場所を見積もるときの、近づく速さの下限
constexpr double kMinClosingSpeedMps = 0.5;
// 下がる方向の候補の数と、見る距離
constexpr int kBackoffDirections = 16;
constexpr double kBackoffProbeM = 0.5;
constexpr double kBackoffPathPreference = 0.3;
constexpr double kBackoffTimeoutS = 3.0;
// 行き詰まりから抜けたとみなす前進量
constexpr double kRetryResetProgressM = 1.0;
// 進路の交点を探すときに、これより短い区間は次の点とまとめる。止まりかけの軌跡の数 cm の区間は
// 向きが定まらず、交点が誤って見つかる
constexpr double kMinCourseSegmentM = 0.05;
// ゴールに着くまでの時間を見積もるときの、ステップ数の上限（dt 0.1 秒で 30 秒）
constexpr int kMaxArrivalSteps = 300;
// ゴールの手前で待った時間を数える、待つ位置までの経路に沿った距離。参照軌道はそこで止まるが、機体は
// 少し手前で止まることがある
constexpr double kHoldArrivalM = 0.5;
// 待った時間に足す 1 回分の上限。周期が途切れた後に、まとめて数えないようにする
constexpr double kMaxHoldStepS = 0.5;
// 進む向きを取る弦がこれより短ければ（経路の終わりの近く）、経路の終わりの区間の向きを使う
constexpr double kMinHeadingChordM = 0.05;
// 通路の余裕と速さの上限を、経路に沿って調べる間隔 [m]
constexpr double kPassageStepM = 0.1;
// 曲がりの速さを求める経路の間隔 [m]
constexpr double kCurveStepM = 0.1;
// 通路の中で縮めた罰の範囲の下限 [m]。0 にすると、余裕のすぐ外で罰が急に跳ぶ
constexpr double kMinPassageBandM = 0.02;
// 通路の余裕をしきい値と比べるときの許し。セルの境界から求めた余裕の丸めの誤差で、ちょうどの値を落とさない
constexpr double kPassageClearanceToleranceM = 1e-6;
// 通路の左右の静止物を探す光線の、左右の幅 [m]。地図の点は 0.2m おきなので、半分の幅で必ず点に当たる
constexpr double kPassageRayHalfWidthM = 0.1;

// 経路に沿った通路の余裕と、そこで出せる参照軌道の速さ。start_s から kPassageStepM ごと
struct PassageProfile
{
  double start_s{0.0};
  // 機体を真ん中に置いたときの片側の余裕。通路でなければ無限大
  std::vector<double> clearance;
  // その先の通路の速さの上限まで reference_decel_mps2 で落とせる速さ
  std::vector<double> allowed_speed;

  // 範囲の外は、通路ではない（無限大）とする。間の点は、両隣の小さいほう
  double clearanceAt(const double s) const {return lookup(clearance, s);}
  double allowedSpeedAt(const double s) const {return lookup(allowed_speed, s);}

private:
  double lookup(const std::vector<double> & values, const double s) const
  {
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    if (values.empty() || !(s >= start_s)) {
      return kInfinity;
    }
    const auto index = static_cast<std::size_t>(std::floor((s - start_s) / kPassageStepM));
    if (index >= values.size()) {
      return kInfinity;
    }
    return index + 1U < values.size() ? std::min(values[index], values[index + 1U]) : values[index];
  }
};

// 経路の曲がりで出せる参照軌道の速さ（CurveSpeedParams）。start_s から kCurveStepM おき
struct CurveProfile
{
  double start_s{0.0};
  // その先の曲がりの速さの上限まで reference_decel_mps2 で落とせる速さ
  std::vector<double> allowed_speed;

  // 範囲の外と、曲がりの上限を使わないときは無限大。間の点は、両隣の小さいほう
  double allowedSpeedAt(const double s) const
  {
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    if (allowed_speed.empty() || !(s >= start_s)) {
      return kInfinity;
    }
    const auto index = static_cast<std::size_t>(std::floor((s - start_s) / kCurveStepM));
    if (index >= allowed_speed.size()) {
      return kInfinity;
    }
    return index + 1U < allowed_speed.size() ?
           std::min(allowed_speed[index], allowed_speed[index + 1U]) : allowed_speed[index];
  }
};

// 経路に沿って、今の位置の射影 now_s から reach 先まで、曲がりで出せる速さを求める。
// 曲がりの上限を使わないなら空（どこでも無限大）
CurveProfile makeCurveProfile(
  const ReferencePath & path, const double now_s, const double reach, const MppiParams & params)
{
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  CurveProfile profile;
  if (!(params.curve.max_lateral_accel_mps2 > 0.0) || path.points().size() < 2U) {
    return profile;
  }
  profile.start_s = std::max(now_s, 0.0);
  const double end_s = std::min(profile.start_s + std::max(reach, 0.0), path.length());
  std::vector<double> cap;
  for (double s = profile.start_s; s <= end_s + 1e-9; s += kCurveStepM) {
    cap.push_back(curveSpeedLimit(pathCurvature(path, s, params.curve.window_m), params));
  }
  profile.allowed_speed.assign(cap.size(), kInfinity);
  const double decel = std::max(params.reference_decel_mps2, 0.0);
  for (std::size_t i = cap.size(); i-- > 0U; ) {
    const double ahead = i + 1U < cap.size() ? profile.allowed_speed[i + 1U] : kInfinity;
    profile.allowed_speed[i] =
      std::min(cap[i], std::sqrt(ahead * ahead + 2.0 * decel * kCurveStepM));
  }
  return profile;
}

struct ObjectStep
{
  double x;
  double y;
  double radius;
  double margin;
  double vx;
  double vy;
  // まだ離れていて、規則の側に空きがあり、すれ違う側を選べる相手か
  bool passing_applicable;
  // 衝突コースの判定に使う相手の大きさ（予測の広がりは含めない）
  double course_radius;
  bool moving;
  // 自機がこの相手へ近づく向きに動いているステップだけ、重なり・接近・衝突コースを数えるか
  bool approach_only;
  // 合流で自機が先に入る相手か。予測の広がりと衝突コースの罰の代わりに、垂直二等分線を見る
  bool own_first;
  // 譲る相手（合流で相手が先、横切り）が加速度の上限で速くなると、等速の予測より先へ進みうる距離と、
  // その向き。重なりと接近は、等速の位置からその先までの線分で見る（それ以外の相手は 0）
  double ahead;
  double unit_x;
  double unit_y;
  // すれ違いの規則で回り込む側を決める、経路をふさいでいる止まっている（遅い）ロボットか
  // （oncoming.enabled のとき）。ゴールがその相手の kPassingGoalClearanceM 先より近ければ当てない
  bool pass_stationary;
  // 最小の余裕。この内側では weight_collision まで上がる罰を足す（ロボットだけ。ほかは 0）
  double min_margin;
};

// 速さ speed の相手が、accel で max_speed まで速くなると t 秒で進む距離
double acceleratedDistance(
  const double speed, const double accel, const double max_speed, const double t)
{
  const double accel_time = accel > 0.0 ? std::clamp((max_speed - speed) / accel, 0.0, t) : 0.0;
  return speed * t + 0.5 * accel * accel_time * accel_time + accel * accel_time * (t - accel_time);
}

// 動いているロボットとの出会い方。kNone は正面、ほぼ平行、交点が自機の進路の前方（参照軌道の範囲）に
// ない相手
enum class Encounter : std::uint8_t {kNone, kMergeOwnFirst, kMergeOtherFirst, kCrossing};

double objectRadius(const ObjectCostParams & params, const ObjectKind kind)
{
  switch (kind) {
    case ObjectKind::kRobot:
      return params.robot_radius_m;
    case ObjectKind::kHuman:
      return params.human_radius_m;
    case ObjectKind::kUnknown:
      break;
  }
  return params.unknown_radius_m;
}

double objectMinMargin(const ObjectCostParams & params, const ObjectKind kind)
{
  return kind == ObjectKind::kRobot ?
         std::clamp(params.robot_min_margin_m, 0.0, params.robot_margin_m) : 0.0;
}

double objectMargin(const ObjectCostParams & params, const ObjectKind kind)
{
  switch (kind) {
    case ObjectKind::kRobot:
      return params.robot_margin_m;
    case ObjectKind::kHuman:
      return params.human_margin_m;
    case ObjectKind::kUnknown:
      break;
  }
  return params.unknown_margin_m;
}

// 相手の規則の側に、機体が通れるだけの空きがあるかを調べる。
// 柵の近くを走る相手の外側へ避けようとすると、柵に当たる
bool hasRoomOnPassingSide(
  const MppiState & state, const PredictedObject & object, const DistanceField * field,
  const SE2 & map_from_odom, const MppiParams & params)
{
  const double dx = object.x - state.x;
  const double dy = object.y - state.y;
  const double distance = std::hypot(dx, dy);
  if (field == nullptr || distance <= 1e-6) {
    return true;
  }
  const double unit_x = dx / distance;
  const double unit_y = dy / distance;
  const double closing = std::max(
    (state.vx - object.vx) * unit_x + (state.vy - object.vy) * unit_y, kMinClosingSpeedMps);
  const double time_to_meet = std::min(distance / closing, params.collision_course_lookahead_s);
  // 自機から相手へ向かう向きの右（side_right）か左
  const double side = params.passing.side_right ? -1.0 : 1.0;
  const double half_width = params.footprint.enabled ? params.footprint.half_width_m : 0.0;
  const double body_clearance = half_width + params.footprint.margin_m;
  const double offset = body_clearance + objectRadius(params.objects, object.kind) + object.sigma +
    objectMargin(params.objects, object.kind);
  const Point2 passing_point = map_from_odom.apply(
    object.x + object.vx * time_to_meet - side * unit_y * offset,
    object.y + object.vy * time_to_meet + side * unit_x * offset);
  return static_cast<double>(field->at(passing_point.x, passing_point.y)) > body_clearance;
}

bool isMovingRobot(const PredictedObject & object)
{
  return object.kind == ObjectKind::kRobot &&
         std::hypot(object.vx, object.vy) >= kMovingObjectSpeedMps;
}

// 自機の進路と、相手の今の速度の直線の交点
struct CourseCrossing
{
  // 自機が交点に着く時刻と、進路に沿った交点までの距離
  double own_time_s;
  double own_distance_m;
  // 相手の直線に沿った交点までの距離
  double object_distance_m;
  // 交点での進む向きの差の cos
  double heading_cos;
};

// 自機の進路は start（時刻 0）から points をたどる折れ線で、points[k] は (k + 1) * dt 秒後の位置。
// 相手の進路は今の速度の直線。交点が折れ線の上（相手から見て前方）になければ nullopt
std::optional<CourseCrossing> findCourseCrossing(
  const PredictedObject & object, const Point2 & start, const std::vector<Point2> & points,
  const double dt)
{
  const double object_speed = std::hypot(object.vx, object.vy);
  if (object_speed <= 0.0) {
    return std::nullopt;
  }
  const double direction_x = object.vx / object_speed;
  const double direction_y = object.vy / object_speed;
  Point2 previous = start;
  double previous_time = 0.0;
  double travelled = 0.0;
  for (std::size_t k = 0; k < points.size(); ++k) {
    const double segment_x = points[k].x - previous.x;
    const double segment_y = points[k].y - previous.y;
    const double segment_length = std::hypot(segment_x, segment_y);
    const double time = static_cast<double>(k + 1U) * dt;
    if (segment_length < kMinCourseSegmentM) {
      continue;
    }
    // 線分 previous + ratio * segment と、相手の直線 object + along * direction の交点
    const double denominator = segment_x * direction_y - segment_y * direction_x;
    // ほぼ平行な進路は、同じ線の上の追従か追い越し、または正面として扱う。追跡の速度の向きのぶれで、
    // 交点の位置が大きく動くため
    if (std::abs(denominator) > kMinMergeSine * segment_length) {
      const double offset_x = object.x - previous.x;
      const double offset_y = object.y - previous.y;
      const double ratio = (offset_x * direction_y - offset_y * direction_x) / denominator;
      const double along = (offset_x * segment_y - offset_y * segment_x) / denominator;
      if (ratio >= 0.0 && ratio <= 1.0 && along >= 0.0) {
        return CourseCrossing{
          previous_time + ratio * (time - previous_time), travelled + ratio * segment_length, along,
          (segment_x * direction_x + segment_y * direction_y) / segment_length};
      }
    }
    previous = points[k];
    previous_time = time;
    travelled += segment_length;
  }
  return std::nullopt;
}

// 動いているロボットとの出会い方を、進路の交点で分ける。相手の進路は今の速度の直線とする。
// 自機の進路は、まず前回の MPPI の軌跡（今の状態から前回の系列を進めたもの。実際に進む線）で調べる。
// 軌跡の範囲に交点がなければ、経路への射影（時刻 0）から参照軌道（0.1 秒ごとの目標点）をたどる折れ線で
// 調べ、自機が着く時刻は軌跡の終わり以降とする（軌跡で届かないので、それより早くは着かない）。
// どちらにもなければ kNone。自機が交点を通り過ぎていれば、交点は後ろにあるので見つからない。
// - 合流: 交点で進む向きの差が 90° 未満で、今互いに近づいている。交点に着く時刻を比べて順番を決める。
//   相手の時刻は、objects.accel_mps2 で objects.max_speed_mps まで速くなるとして早めに見積もる。
//   軌跡の上の交点が自機の機体の中（中心から前へ半分の長さと余裕の内側）にあれば、相手は自機の
//   後ろや横から当たってくる形なので kNone
// - 横切り: 向きの差が 90° 以上 150° 未満
// - 正面: 150° 以上（kNone）
// 参照軌道を経路の上の点から始めるのは、機体の数 cm の横ぶれで短い最初の区間が斜めになり、同じ線を
// 後ろから来る相手に交点ができてしまうため。軌跡では短い区間をまとめ、機体の中の交点を除いて防ぐ
Encounter classifyEncounter(
  const MppiState & state, const PredictedObject & object, const std::vector<Point2> & trajectory,
  const Point2 & path_start, const std::vector<Point2> & reference, const MppiParams & params)
{
  const double object_speed = std::hypot(object.vx, object.vy);
  if (object_speed <= 0.0) {
    return Encounter::kNone;
  }
  const double relative_x = object.x - state.x;
  const double relative_y = object.y - state.y;
  const bool approaching =
    relative_x * (object.vx - state.vx) + relative_y * (object.vy - state.vy) < 0.0;
  auto crossing = findCourseCrossing(object, Point2{state.x, state.y}, trajectory, params.dt);
  const bool on_trajectory = crossing.has_value();
  if (!on_trajectory) {
    crossing = findCourseCrossing(object, path_start, reference, params.dt);
    if (!crossing.has_value()) {
      return Encounter::kNone;
    }
    crossing->own_time_s =
      std::max(crossing->own_time_s, static_cast<double>(trajectory.size()) * params.dt);
  }
  if (crossing->heading_cos > 0.0) {
    if (!params.merge.enabled || !approaching) {
      return Encounter::kNone;
    }
    const double body_front = (params.footprint.enabled ? params.footprint.half_length_m : 0.0) +
      params.footprint.margin_m;
    if (on_trajectory && crossing->own_distance_m < body_front) {
      return Encounter::kNone;
    }
    const double object_time = earliestArrivalTime(
      crossing->object_distance_m, object_speed, params.objects.accel_mps2,
      params.objects.max_speed_mps);
    return object_time - crossing->own_time_s >= params.merge.lead_time_s ?
           Encounter::kMergeOwnFirst : Encounter::kMergeOtherFirst;
  }
  return params.crossing.enabled && crossing->heading_cos > kHeadOnCos ?
         Encounter::kCrossing : Encounter::kNone;
}

// 相手の各ステップの予測位置を、ステップごとにまとめて返す。trajectory は前回の MPPI の軌跡
// （今の状態の次のステップから）
std::vector<std::vector<ObjectStep>> predictObjects(
  const MppiState & state, const std::vector<PredictedObject> & objects,
  const std::vector<Point2> & trajectory, const ReferencePath & path, const Point2 & path_start,
  const std::vector<Point2> & reference, const DistanceField * field, const SE2 & map_from_odom,
  const MppiParams & params)
{
  std::vector<std::uint8_t> passing_applicable;
  std::vector<std::uint8_t> approach_only;
  std::vector<std::uint8_t> own_first;
  std::vector<std::uint8_t> yield_to;
  std::vector<std::uint8_t> pass_stationary;
  passing_applicable.reserve(objects.size());
  approach_only.reserve(objects.size());
  own_first.reserve(objects.size());
  yield_to.reserve(objects.size());
  pass_stationary.reserve(objects.size());
  const double path_half_width =
    (params.footprint.enabled ? params.footprint.half_width_m : 0.0) + params.footprint.margin_m;
  for (const auto & object : objects) {
    // 経路をふさいでいる（通り道の上にいる）、止まっている（遅い）ロボット。すれ違いの規則の側から
    // 回り込む（oncoming.enabled のとき）。経路の横にいるだけの相手には当てない（規則と逆の側にいると、
    // 罰のかからない所まで経路から大きく外れる。speed_limit の試験で列から 3m の外へ出た）。
    // ゴールがその相手の kPassingGoalClearanceM 先より近ければ当てない（近くのゴールへ進めなくなるため）
    const double s_object = path.project(object.x, object.y);
    const Point2 base = path.pointAt(s_object);
    const Point2 along = path.directionAt(s_object);
    const double lateral =
      std::abs(along.x * (object.y - base.y) - along.y * (object.x - base.x));
    const bool blocking_stationary = params.oncoming.enabled &&
      object.kind == ObjectKind::kRobot && !object.hold &&
      std::hypot(object.vx, object.vy) < kMovingObjectSpeedMps &&
      lateral <= path_half_width + objectRadius(params.objects, object.kind) &&
      path.length() - s_object > kPassingGoalClearanceM;
    // 止まっている相手を回り込む側は、近づいても決め直さない（commit_distance_m は、速く近づき合う
    // 正面の相手で、避け始めてから側を変えないためのもの）。止まっている相手へは自機だけが近づくので、
    // 離れているうちは予測の範囲に入らず、側を決められない
    passing_applicable.push_back(
      (blocking_stationary ||
      std::hypot(object.x - state.x, object.y - state.y) > params.passing.commit_distance_m) &&
      hasRoomOnPassingSide(state, object, field, map_from_odom, params) ? 1U : 0U);
    // 合流と横切りの規則は動いているロボットだけに当てる。人には当てない。
    // 予測の位置をよけない相手（hold）は OncomingMonitor が停止線で応じるので、規則を当てない
    const Encounter encounter =
      (params.merge.enabled || params.crossing.enabled) && isMovingRobot(object) && !object.hold ?
      classifyEncounter(state, object, trajectory, path_start, reference, params) :
      Encounter::kNone;
    // 人、止まっている相手、分類できない相手には、責任によらず今までどおり避ける。
    // 合流で相手が先なら、相手が自機の後ろや横にいても、今までどおり避けて後ろにつく。
    // 横切りの相手も、自機がまだ交点の手前にいる間は今までどおり避ける（自機が相手の前を横切る形も含む）。
    // hold の相手は、今の位置へ自機が近づくときだけ数える（横へ道を開けない）
    approach_only.push_back(
      object.hold ||
      (!params.avoid_being_hit && isMovingRobot(object) &&
      encounter != Encounter::kMergeOtherFirst && encounter != Encounter::kCrossing) ? 1U : 0U);
    own_first.push_back(encounter == Encounter::kMergeOwnFirst ? 1U : 0U);
    yield_to.push_back(
      encounter == Encounter::kMergeOtherFirst || encounter == Encounter::kCrossing ? 1U : 0U);
    pass_stationary.push_back(params.passing.enabled && blocking_stationary ? 1U : 0U);
  }
  std::vector<std::vector<ObjectStep>> predicted(static_cast<std::size_t>(params.horizon_steps));
  for (int k = 0; k < params.horizon_steps; ++k) {
    const double t = static_cast<double>(k + 1) * params.dt;
    auto & step = predicted[static_cast<std::size_t>(k)];
    step.reserve(objects.size());
    for (std::size_t i = 0; i < objects.size(); ++i) {
      const auto & object = objects[i];
      if (object.hold) {
        // 予測の位置をよけない相手: 今の位置に止まっているとして、自機が近づくときだけ重なりと余裕を
        // 数える（相手が近づいてくる分はよけない。自機から近づいて当てることはしない）
        const double size = objectRadius(params.objects, object.kind) + object.sigma;
        const double margin = objectMargin(params.objects, object.kind);
        step.push_back(ObjectStep{
            object.x, object.y, size, margin, 0.0, 0.0, false, size + margin, false, true, false, 0.0,
            0.0, 0.0, false, objectMinMargin(params.objects, object.kind)});
        continue;
      }
      const double speed = std::hypot(object.vx, object.vy);
      const double ahead = yield_to[i] != 0U && speed > 0.0 ?
        acceleratedDistance(speed, params.objects.accel_mps2, params.objects.max_speed_mps, t) -
        speed * t : 0.0;
      step.push_back(ObjectStep{
          object.x + object.vx * t,
          object.y + object.vy * t,
          // 止まっている相手の位置は時間が経っても不確かにならない。増やすと、止まった相手の
          // 近くにあるゴールへ近づけなくなる。追跡の速度がまだ定まっていない間は、止まって見える
          // 相手でも、その不確かさの分を広げる
          objectRadius(params.objects, object.kind) + object.sigma +
          (params.objects.prediction_sigma_growth_mps *
          std::min(1.0, std::hypot(object.vx, object.vy)) +
          params.objects.velocity_sigma_gain * object.velocity_sigma) * t,
          objectMargin(params.objects, object.kind),
          object.vx,
          object.vy,
          passing_applicable[i] != 0U,
          objectRadius(params.objects, object.kind) + object.sigma +
          objectMargin(params.objects, object.kind),
          std::hypot(object.vx, object.vy) >= kMovingObjectSpeedMps,
          approach_only[i] != 0U,
          own_first[i] != 0U,
          ahead,
          speed > 0.0 ? object.vx / speed : 0.0,
          speed > 0.0 ? object.vy / speed : 0.0,
          pass_stationary[i] != 0U,
          objectMinMargin(params.objects, object.kind)});
    }
  }
  return predicted;
}

// 前後 window_steps 点の中で最も狭い余裕
std::vector<double> narrowestAround(const std::vector<double> & raw, const std::size_t window_steps)
{
  std::vector<double> narrowest(raw.size(), std::numeric_limits<double>::infinity());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const std::size_t first = i >= window_steps ? i - window_steps : 0U;
    const std::size_t last = std::min(i + window_steps, raw.size() - 1U);
    narrowest[i] = *std::min_element(
      raw.begin() + static_cast<std::ptrdiff_t>(first),
      raw.begin() + static_cast<std::ptrdiff_t>(last) + 1);
  }
  return narrowest;
}

// 経路に沿って、今の位置の射影 now_s から reach 先まで、通路の余裕と参照軌道で出せる速さを求める。
// 機体の中心が通路の手前にあっても、前の端は先に入る（出るときは後ろの端が残る）。
// - 速さ: 前後に機体の半分の長さと余裕を足した範囲の中で、最も狭い所の余裕から決める
// - 罰の範囲: 入口の手前でも、機体の角は入口の壁の端に罰の範囲（余裕 + 速さの分 + keep_out_range_m）の
//   内側まで近づく。そこで元の範囲の罰を受けると手前で止まるので、さらに罰の範囲の分だけ前後を見る
PassageProfile makePassageProfile(
  const ReferencePath & path, const double now_s, const double reach, const double speed_limit,
  const DistanceField & field, const SE2 & map_from_odom, const MppiParams & params)
{
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  const FootprintParams & footprint = params.footprint;
  const double half_length = footprint.enabled ? footprint.half_length_m : 0.0;
  const double half_width = footprint.enabled ? footprint.half_width_m : 0.0;
  const double speed_window = half_length + std::max(footprint.margin_m, 0.0);
  const double band_window = speed_window + std::max(params.keep_out_range_m, 0.0) +
    std::max(params.static_margin_time_s, 0.0) * std::max(speed_limit, 0.0);

  PassageProfile profile;
  profile.start_s = std::max(now_s - band_window, 0.0);
  const double end_s = std::min(now_s + std::max(reach, 0.0) + band_window, path.length());
  std::vector<double> raw;
  for (double s = profile.start_s; s <= end_s + 1e-9; s += kPassageStepM) {
    raw.push_back(
      passageSideClearance(field, map_from_odom, path.pointAt(s), path.directionAt(s), half_width)
      .value_or(kInfinity));
  }
  profile.clearance =
    narrowestAround(raw, static_cast<std::size_t>(std::ceil(band_window / kPassageStepM)));
  const auto speed_clearance =
    narrowestAround(raw, static_cast<std::size_t>(std::ceil(speed_window / kPassageStepM)));
  // 先の速さの上限まで、参照軌道と同じ減速度で落とせる速さ
  profile.allowed_speed.assign(raw.size(), kInfinity);
  for (std::size_t i = raw.size(); i-- > 0U; ) {
    const double cap = std::isinf(speed_clearance[i]) ? kInfinity :
      passageSpeedLimit(speed_clearance[i], params);
    const double ahead = i + 1U < raw.size() ? profile.allowed_speed[i + 1U] : kInfinity;
    profile.allowed_speed[i] = std::min(
      cap, std::sqrt(ahead * ahead + 2.0 * params.reference_decel_mps2 * kPassageStepM));
  }
  return profile;
}

// 参照軌道: 現在位置の射影から、速度を上げ下げしながら経路を進めた各ステップの目標点。
// 経路の向きが、そのステップの機体の前（forward）から見えない側なら、速さを blind の上限までにする。
// 狭い通路では passage の速さまでに下げる。arc_lengths に各点の経路に沿った位置を入れる
std::vector<Point2> makeReference(
  const MppiState & state, const ReferencePath & path, const MppiParams & params,
  const double speed_limit, const std::vector<Point2> & forward, const BlindSideLimit & blind,
  const PassageProfile & passage, const CurveProfile & curve, std::vector<double> & arc_lengths)
{
  std::vector<Point2> reference;
  reference.reserve(static_cast<std::size_t>(params.horizon_steps));
  arc_lengths.clear();
  arc_lengths.reserve(static_cast<std::size_t>(params.horizon_steps));
  double s = path.project(state.x, state.y);
  const Point2 direction = path.directionAt(s);
  double v = std::max(state.vx * direction.x + state.vy * direction.y, 0.0);
  for (int k = 0; k < params.horizon_steps; ++k) {
    const double remaining = std::max(path.length() - s, 0.0);
    const double braking = std::sqrt(2.0 * params.reference_decel_mps2 * remaining);
    v = std::min(
      {speed_limit, braking, passage.allowedSpeedAt(s), curve.allowedSpeedAt(s),
        v + params.max_accel_mps2 * params.dt});
    const Point2 along = path.directionAt(s);
    const Point2 & front = forward[static_cast<std::size_t>(k)];
    v *= blind.scale(along.x * v, along.y * v, front.x, front.y);
    s = std::min(s + v * params.dt, path.length());
    reference.push_back(path.pointAt(s));
    arc_lengths.push_back(s);
  }
  return reference;
}

// 向きの指令の系列。目標の向き（headingTarget）があればそちらへ回り（yawRateToward）、なければ今の
// 向きを保つ。全方位に動けるので、避けるために向きを変える必要はない。サンプルに含めると向きが乱れ、
// 斜めになった機体は横に広がる。すべての系列で同じなので、回るときの機体の角の動きは
// rolloutCost の静止物と相手のコストに入る
std::vector<double> makeYawRates(
  const MppiState & state, const std::optional<double> & target_yaw, const MppiParams & params)
{
  std::vector<double> rates(static_cast<std::size_t>(params.horizon_steps), 0.0);
  const double max_dw = params.max_angular_accel_radps2 * params.dt;
  double yaw = state.yaw;
  double wz = state.wz;
  for (double & rate : rates) {
    const double target = target_yaw.has_value() ?
      yawRateToward(wrapAngle(*target_yaw - yaw), params) : 0.0;
    wz += std::clamp(target - wz, -max_dw, max_dw);
    yaw += wz * params.dt;
    rate = wz;
  }
  return rates;
}

// 各ステップの始めの機体の前の向き（単位ベクトル）。rolloutCost と同じく、向きは角速度で進める
std::vector<Point2> forwardDirections(
  const MppiState & state, const std::vector<double> & yaw_rates, const double dt)
{
  std::vector<Point2> forward;
  forward.reserve(yaw_rates.size());
  double yaw = state.yaw;
  for (const double rate : yaw_rates) {
    forward.push_back(Point2{std::cos(yaw), std::sin(yaw)});
    yaw += rate * dt;
  }
  return forward;
}

// 参照軌道をそのまま追う制御系列
std::vector<double> makePathControls(
  const MppiState & state, const std::vector<Point2> & reference, const MppiParams & params)
{
  std::vector<double> controls(reference.size() * kControlSize, 0.0);
  Point2 previous{state.x, state.y};
  for (std::size_t k = 0; k < reference.size(); ++k) {
    controls[k * kControlSize] = (reference[k].x - previous.x) / params.dt;
    controls[k * kControlSize + 1] = (reference[k].y - previous.y) / params.dt;
    previous = reference[k];
  }
  return controls;
}

// 見えない側へ向かう指令を blind の上限までに縮める。controls は forward.size() ステップの系列を
// 並べたもので、forward はステップごとの機体の前の向き
void limitBlindSide(
  std::vector<double> & controls, const std::vector<Point2> & forward, const BlindSideLimit & blind)
{
  const std::size_t steps = forward.size();
  const std::size_t count = controls.size() / kControlSize;
  for (std::size_t j = 0; j < count && steps > 0U; ++j) {
    double * u = controls.data() + j * kControlSize;
    const Point2 & front = forward[j % steps];
    const double scale = blind.scale(u[0], u[1], front.x, front.y);
    if (scale < 1.0) {
      u[0] *= scale;
      u[1] *= scale;
    }
  }
}

// 系列（forward.size() ステップの系列を並べたもの）の各ステップの並進の速さを speed_limit までにし、
// 見えない側へ向かう速度を blind の上限までにする。回転は上限に収める
void clampControls(
  std::vector<double> & controls, const double speed_limit, const double max_angular_speed,
  const std::vector<Point2> & forward, const BlindSideLimit & blind)
{
  const std::size_t count = controls.size() / kControlSize;
  for (std::size_t j = 0; j < count; ++j) {
    double * u = controls.data() + j * kControlSize;
    const double speed = std::hypot(u[0], u[1]);
    if (speed > speed_limit) {
      const double scale = speed > 0.0 ? speed_limit / speed : 0.0;
      u[0] *= scale;
      u[1] *= scale;
    }
    u[2] = std::clamp(u[2], -max_angular_speed, max_angular_speed);
  }
  limitBlindSide(controls, forward, blind);
}

struct RolloutContext
{
  const MppiState & state;
  const ReferencePath & path;
  const std::vector<Point2> & reference;
  const std::vector<std::vector<ObjectStep>> & predicted;
  const DistanceField * field;
  const SE2 & map_from_odom;
  const MppiParams & params;
  double half_length;
  double half_width;
  double circumscribed_radius;
  double map_cos;
  double map_sin;
  // 機体の 4 隅を調べる範囲に足す距離。kKeepOutCheckMarginM と、系列の中の機体の点の速さの上限に
  // static_margin_time_s を掛けた距離の大きいほう（設定が staticMarginError を満たせば前者）
  double keep_out_check_margin;
  // ステップごとの、参照軌道の点での通路の片側の余裕（通路でなければ無限大）と、経路の左向きの法線（odom）。
  // 狭い通路を使わないときは空
  const std::vector<double> & passage_clearance;
  const std::vector<Point2> & passage_normal;
};

// ステップ step の参照軌道の点が、入れる通路（片側の余裕が min_side_clearance_m 以上）の中か
std::optional<double> enterablePassage(const RolloutContext & context, const std::size_t step)
{
  if (step >= context.passage_clearance.size() || step >= context.passage_normal.size()) {
    return std::nullopt;
  }
  const double clearance = context.passage_clearance[step];
  if (std::isinf(clearance) ||
    !(clearance + kPassageClearanceToleranceM >= context.params.narrow.min_side_clearance_m))
  {
    return std::nullopt;
  }
  return clearance;
}

// 通路の中の静止物の罰の範囲。機体の点の距離（距離の場はセルの中心どうしなので、真ん中にいても 1 セル
// ほど短く読める）が「通路の余裕 − lateral_tolerance_m」以上なら罰をかけず、そこから余裕までの間に罰をかける。
// 通路が罰の範囲より広ければ元の範囲
double passageBand(const MppiParams & params, const double clearance, const double static_margin)
{
  const double free_from = clearance - std::max(params.narrow.lateral_tolerance_m, 0.0);
  return std::clamp(free_from - static_margin, kMinPassageBandM, params.keep_out_range_m);
}

// 機体の 4 隅と中心のうち、静的な障害物に最も近い点
struct KeepOutPoint
{
  // 最も近い障害物までの距離
  double distance;
  // 点の位置（地図の座標系）
  double map_x;
  double map_y;
  // 機体の中心から点までの位置（地図の座標系）と、その長さ。中心なら 0
  double offset_x;
  double offset_y;
  double radius;
};

// 機体の 4 隅と中心のうち、静的な障害物に最も近い点を返す
// 静止物の距離の場を調べる機体の点（base_link）。長方形なら 4 隅、回転中は外接円の上の 8 点
std::vector<Point2> footprintProbePoints(const RolloutContext & context)
{
  const FootprintParams & footprint = context.params.footprint;
  const ChassisShape & chassis = footprint.chassis;
  if (!footprint.enabled || chassis.half_length <= 0.0 || chassis.half_width <= 0.0) {
    return {
      {-context.half_length, -context.half_width}, {-context.half_length, context.half_width},
      {context.half_length, -context.half_width}, {context.half_length, context.half_width}};
  }
  std::vector<Point2> points;
  if (chassis.circle) {
    constexpr int kCirclePoints = 8;
    const double radius = std::hypot(chassis.half_length, chassis.half_width);
    points.reserve(kCirclePoints);
    for (int i = 0; i < kCirclePoints; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / kCirclePoints;
      points.push_back({radius * std::cos(angle), radius * std::sin(angle)});
    }
    return points;
  }
  const double c = std::cos(chassis.yaw);
  const double s = std::sin(chassis.yaw);
  points.reserve(4U);
  for (const double along : {-chassis.half_length, chassis.half_length}) {
    for (const double side : {-chassis.half_width, chassis.half_width}) {
      points.push_back({c * along - s * side, s * along + c * side});
    }
  }
  return points;
}

KeepOutPoint keepOutPoint(
  const RolloutContext & context, const double x, const double y, const double yaw)
{
  const auto & params = context.params;
  const double map_x = context.map_from_odom.x + context.map_cos * x - context.map_sin * y;
  const double map_y = context.map_from_odom.y + context.map_sin * x + context.map_cos * y;
  KeepOutPoint closest{
    static_cast<double>(context.field->at(map_x, map_y)), map_x, map_y, 0.0, 0.0, 0.0};
  // 中心が十分離れていれば、4 隅を調べても結果は変わらない
  const double reach = context.circumscribed_radius + params.footprint.margin_m +
    context.keep_out_check_margin + params.keep_out_range_m;
  if (closest.distance > reach || context.circumscribed_radius <= 0.0) {
    return closest;
  }
  const double c = std::cos(yaw + context.map_from_odom.yaw);
  const double s = std::sin(yaw + context.map_from_odom.yaw);
  for (const Point2 & corner : footprintProbePoints(context)) {
    const double corner_x = map_x + c * corner.x - s * corner.y;
    const double corner_y = map_y + s * corner.x + c * corner.y;
    const double distance = static_cast<double>(context.field->at(corner_x, corner_y));
    if (distance < closest.distance) {
      closest = KeepOutPoint{
        distance, corner_x, corner_y, c * corner.x - s * corner.y, s * corner.x + c * corner.y,
        context.circumscribed_radius};
    }
  }
  return closest;
}

// 静止物の余裕に足す速さ。static_margin_approach_only なら、静止物に最も近い点の速度（並進と回転の分
// ω × r。odom の速度 (vx, vy) と角速度 wz から求める）のうち、その点の距離の場の勾配の逆向き（静止物へ
// 近づく向き）の成分。勾配が求まらない所では、元の動きと同じ全方向の速さ（中心の並進の速さ）にする
double staticMarginSpeed(
  const RolloutContext & context, const KeepOutPoint & point, const double vx, const double vy,
  const double wz, const double speed)
{
  const auto & params = context.params;
  if (!params.static_margin_approach_only) {
    return speed;
  }
  // 点の速さの上限で見積もった余裕と罰の範囲の外側の境より遠ければ、向きによらず罰はない。
  // 計算時間を増やさないよう、勾配は求めない
  const double point_speed_bound = speed + std::abs(wz) * point.radius;
  if (point.distance >= params.footprint.margin_m +
    point_speed_bound * params.static_margin_time_s + params.keep_out_range_m)
  {
    return speed;
  }
  // 中心差分の勾配（地図の座標系）。距離の場はセルごとに一定なので、点の入るセルの両隣の値の差にする
  const DistanceField & field = *context.field;
  const GridInfo & info = field.info();
  const GridCell cell = info.cellAt(point.map_x, point.map_y);
  const double gradient_x = (static_cast<double>(field.at(GridCell{cell.x + 1, cell.y})) -
    static_cast<double>(field.at(GridCell{cell.x - 1, cell.y}))) / (2.0 * info.resolution);
  const double gradient_y = (static_cast<double>(field.at(GridCell{cell.x, cell.y + 1})) -
    static_cast<double>(field.at(GridCell{cell.x, cell.y - 1}))) / (2.0 * info.resolution);
  const double gradient_norm = std::sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
  if (gradient_norm < kMinStaticGradient) {
    return speed;
  }
  const double map_vx = context.map_cos * vx - context.map_sin * vy;
  const double map_vy = context.map_sin * vx + context.map_cos * vy;
  const double point_vx = map_vx - wz * point.offset_y;
  const double point_vy = map_vy + wz * point.offset_x;
  return std::max(0.0, -(gradient_x * point_vx + gradient_y * point_vy) / gradient_norm);
}

// 今の相対速度のまま進んだときの、最接近での相手の位置（自機から見た向き）を返す。
// 遠ざかる相手と、最接近が lookahead より先の相手には nullopt を返す
std::optional<Point2> closestApproach(
  const double relative_x, const double relative_y, const double relative_vx,
  const double relative_vy, const double lookahead)
{
  const double squared_speed = relative_vx * relative_vx + relative_vy * relative_vy;
  const double closing = -(relative_x * relative_vx + relative_y * relative_vy);
  if (closing <= 0.0 || closing >= lookahead * squared_speed) {
    return std::nullopt;
  }
  const double time_to_closest = closing / squared_speed;
  return Point2{
    relative_x + relative_vx * time_to_closest, relative_y + relative_vy * time_to_closest};
}

// 自機の速度 (vx, vy) の、相手（自機から見て relative の位置）の方向の成分。中心が重なっていれば
// 向きが決まらないので、動いていれば近づいているとみなす
double approachSpeed(
  const double relative_x, const double relative_y, const double squared_distance, const double vx,
  const double vy, const double speed)
{
  if (squared_distance <= 1e-12) {
    return speed;
  }
  return (vx * relative_x + vy * relative_y) / std::sqrt(squared_distance);
}

// 2 台の中心を結ぶ線の垂直二等分線を、自機の機体（footprint の余裕を含む）が越えるときの罰。
// 相手の大きさや予測の広がりは見ず、2 台の間の半分を自機の側とする。(c, s) は自機の向きの cos と sin
double bisectorCost(
  const RolloutContext & context, const double relative_x, const double relative_y,
  const double squared_distance, const double c, const double s)
{
  const auto & params = context.params;
  const double hard_margin = params.footprint.margin_m;
  const double distance = std::sqrt(squared_distance);
  // 中心がこれだけ離れていれば、どの向きでも機体は線に届かない
  if (distance >= 2.0 * (context.circumscribed_radius + hard_margin)) {
    return 0.0;
  }
  if (distance <= 1e-6) {
    return 2.0 * params.weight_collision;
  }
  const double local_x = (c * relative_x + s * relative_y) / distance;
  const double local_y = (-s * relative_x + c * relative_y) / distance;
  // 相手の方向への機体の張り出し（長方形の支持関数）
  const double extent = context.half_length * std::abs(local_x) +
    context.half_width * std::abs(local_y) + hard_margin;
  const double depth = extent - 0.5 * distance;
  if (depth <= 0.0) {
    return 0.0;
  }
  // 深く越えるほど大きくして、越えた状態からも戻る差をつける
  return params.weight_collision * (1.0 + depth / extent);
}

// 動いている相手が、今の速度のまま進むと collision_course_lookahead_s 以内に機体の近くを通るか
bool objectOnCollisionCourse(
  const MppiState & state, const PredictedObject & object, const MppiParams & params)
{
  if (std::hypot(object.vx, object.vy) < kMovingObjectSpeedMps) {
    return false;
  }
  const double half_width = params.footprint.enabled ? params.footprint.half_width_m : 0.0;
  const double reach = half_width + params.footprint.margin_m +
    objectRadius(params.objects, object.kind) + object.sigma +
    objectMargin(params.objects, object.kind);
  const auto closest = closestApproach(
    object.x - state.x, object.y - state.y, object.vx - state.vx, object.vy - state.vy,
    params.collision_course_lookahead_s);
  return closest.has_value() && std::hypot(closest->x, closest->y) < reach;
}

// 1 本の制御系列のコストを計算する。collision に衝突の有無を返す
// rolloutCost と同じ運動（加速度の上限で指令の速度へ近づき、駆動の一次遅れの分だけ遅れる）で制御系列を
// 進め、各ステップの状態を返す。先頭は今の状態
std::vector<MppiState> simulateControls(
  const MppiState & state, const double * controls, const MppiParams & params)
{
  const double dt = params.dt;
  const double max_dv = params.max_accel_mps2 * dt;
  const double max_dw = params.max_angular_accel_radps2 * dt;
  const double lag = lagGain(dt, params.drive_time_constant_s);
  std::vector<MppiState> trajectory;
  trajectory.reserve(static_cast<std::size_t>(std::max(params.horizon_steps, 0)) + 1U);
  trajectory.push_back(state);
  MppiState current = state;
  for (int k = 0; k < params.horizon_steps; ++k) {
    const double * u = controls + static_cast<std::size_t>(k) * kControlSize;
    double dvx = u[0] - current.vx;
    double dvy = u[1] - current.vy;
    const double dv = std::hypot(dvx, dvy);
    if (dv > max_dv) {
      dvx *= max_dv / dv;
      dvy *= max_dv / dv;
    }
    current.vx += lag * dvx;
    current.vy += lag * dvy;
    current.wz += lag * std::clamp(u[2] - current.wz, -max_dw, max_dw);
    current.x += current.vx * dt;
    current.y += current.vy * dt;
    current.yaw += current.wz * dt;
    trajectory.push_back(current);
  }
  return trajectory;
}

double rolloutCost(const RolloutContext & context, const double * controls, bool & collision)
{
  const auto & params = context.params;
  const auto & state = context.state;
  const double dt = params.dt;
  const double max_dv = params.max_accel_mps2 * dt;
  const double max_dw = params.max_angular_accel_radps2 * dt;
  const double lag = lagGain(dt, params.drive_time_constant_s);
  const double hard_margin = params.footprint.margin_m;

  double x = state.x;
  double y = state.y;
  double yaw = state.yaw;
  // 衝突判定の形を置く向き。腰の TURRET では足回りの向きで、上半身の回り（wz）では回さない
  double footprint_yaw = state.yaw;
  double vx = state.vx;
  double vy = state.vy;
  double wz = state.wz;
  double previous_ux = state.vx;
  double previous_uy = state.vy;
  double cost = 0.0;
  double discount = 1.0;
  collision = false;

  for (int k = 0; k < params.horizon_steps; ++k) {
    const double * u = controls + static_cast<std::size_t>(k) * kControlSize;

    // 指令の速度へ、加速度の上限で近づく（駆動の一次遅れがあれば、その分だけ遅れる）
    double dvx = u[0] - vx;
    double dvy = u[1] - vy;
    const double dv = std::hypot(dvx, dvy);
    if (dv > max_dv) {
      dvx *= max_dv / dv;
      dvy *= max_dv / dv;
    }
    vx += lag * dvx;
    vy += lag * dvy;
    wz += lag * std::clamp(u[2] - wz, -max_dw, max_dw);
    x += vx * dt;
    y += vy * dt;
    yaw += wz * dt;
    // 形の角が回る速さ（余裕の見積もりに使う）
    const double footprint_rate = params.chassis_decoupled ? params.chassis_rate_radps : wz;
    footprint_yaw = params.chassis_decoupled ? footprint_yaw + footprint_rate * dt : yaw;

    const auto index = static_cast<std::size_t>(k);
    const Point2 & target = context.reference[index];
    const double tracking =
      (x - target.x) * (x - target.x) + (y - target.y) * (y - target.y);
    cost += params.weight_tracking * tracking;

    const double c = std::cos(footprint_yaw);
    const double s = std::sin(footprint_yaw);
    const double speed = std::hypot(vx, vy);

    if (context.field != nullptr) {
      // 計画から実際の動きまでの遅れの間に進む分だけ、速いほど柵から離れる。approach_only なら
      // 静止物へ近づく向きの速さだけを見る（壁と平行に走っても余裕を増やさない）。
      // 入れる通路の中では、両側の壁は経路と平行なので、経路に直角な速さ（と回転で角が動く速さ）だけを
      // 見る。通路の速さの上限（passageSpeedLimit）は参照軌道にかけている
      const KeepOutPoint point = keepOutPoint(context, x, y, footprint_yaw);
      const auto passage = enterablePassage(context, index);
      double margin_speed = 0.0;
      if (passage.has_value()) {
        const Point2 & normal = context.passage_normal[index];
        margin_speed = std::abs(vx * normal.x + vy * normal.y) +
          std::abs(footprint_rate) * context.circumscribed_radius;
      } else {
        margin_speed = staticMarginSpeed(context, point, vx, vy, footprint_rate, speed);
      }
      const double static_margin = hard_margin + margin_speed * params.static_margin_time_s;
      const double distance = point.distance;
      const double band = passage.has_value() ?
        passageBand(params, *passage, static_margin) : params.keep_out_range_m;
      if (distance <= static_margin) {
        // 相手の大きさには余裕が入っているが、柵は確実に当たる。相手との間で挟まれたら柵を避ける。
        // 深く入るほど大きくして、中にいても外へ向かう差をつける
        const double depth = static_margin > 0.0 ? 1.0 - distance / static_margin : 1.0;
        cost += kStaticCollisionScale * params.weight_collision * (1.0 + depth);
        collision = true;
      } else if (distance < static_margin + band) {
        const double ratio = 1.0 - (distance - static_margin) / band;
        cost += params.weight_keep_out * ratio * ratio;
      }
    }

    // 相手が後ろから当たる予測では、相手が自機を通り抜けて前へ出た後のステップで、自機が近づく向きに
    // 見えて罰を受ける。この罰で横からの接触にも慎重になり、crowd_stats で自機が原因の接触が
    // 0.09 から 0 回/100秒に減ったので、当たった後のステップも数える
    for (const auto & object : context.predicted[index]) {
      const double relative_x = object.x - x;
      const double relative_y = object.y - y;
      const double squared = relative_x * relative_x + relative_y * relative_y;
      // 自機が近づく向きに動いていないステップでは、動いているロボットと重なっても相手の責任とする
      const bool responsible = !object.approach_only ||
        approachSpeed(relative_x, relative_y, squared, vx, vy, speed) > kApproachSpeedMps;
      if (responsible && object.own_first) {
        // 合流で自機が先に入る相手には、予測の広がりや衝突コースで譲らない
        cost += bisectorCost(context, relative_x, relative_y, squared, c, s) * discount;
      }
      const bool evaluate_object = responsible && !object.own_first;
      const double reach = context.circumscribed_radius + hard_margin + object.radius +
        object.margin;
      // 譲る相手は、等速の位置から加速したときの位置までの線分のうち、自機に最も近い点で重なりを見る
      const double along_ahead = object.ahead > 0.0 ?
        std::clamp(-(relative_x * object.unit_x + relative_y * object.unit_y), 0.0, object.ahead) :
        0.0;
      const double near_x = relative_x + object.unit_x * along_ahead;
      const double near_y = relative_y + object.unit_y * along_ahead;
      const double near_squared = near_x * near_x + near_y * near_y;
      if (evaluate_object && near_squared <= reach * reach) {
        const double local_x = c * near_x + s * near_y;
        const double local_y = -s * near_x + c * near_y;
        const double distance = signedDistanceToFootprint(
          local_x, local_y, params.footprint.enabled ? params.footprint : FootprintParams{false, 0.0, 0.0}) -
          object.radius - hard_margin;
        if (distance <= 0.0) {
          cost += params.weight_collision * discount;
          collision = true;
        } else if (distance < object.margin) {
          const double ratio = 1.0 - distance / object.margin;
          cost += params.weight_object * ratio * ratio * discount;
          // 最小の余裕の内側は、重なりと同じ重みまで上がる罰を足す（狭い所では目安より近づいてよいが、
          // 最小より近づかない）
          if (distance < object.min_margin) {
            const double inner = 1.0 - distance / object.min_margin;
            cost += params.weight_collision * inner * inner * discount;
          }
        }
      }

      // 合流で先に入る相手でも、今の速度のままでは機体どうしが重なる（余裕を除いた大きさで当たる）
      // 予測なら、衝突コースの罰を数えて譲る。重ならなければ数えず、垂直二等分線だけを見る
      if ((evaluate_object || (responsible && object.own_first)) && object.moving &&
        params.weight_collision_course > 0.0)
      {
        const auto closest = closestApproach(
          relative_x, relative_y, object.vx - vx, object.vy - vy,
          params.collision_course_lookahead_s);
        const double course_reach = context.half_width + hard_margin + object.course_radius -
          (object.own_first ? object.margin : 0.0);
        const double miss = closest.has_value() ? std::hypot(closest->x, closest->y) : course_reach;
        if (miss < course_reach) {
          double ratio = 1.0 - miss / course_reach;
          if (params.passing.enabled && object.passing_applicable && speed > kMinDirectionSpeedMps) {
            const double direction_x = vx / speed;
            const double direction_y = vy / speed;
            const double lateral = direction_x * closest->y - direction_y * closest->x;
            const bool wrong_side = params.passing.side_right ? lateral < 0.0 : lateral > 0.0;
            const bool oncoming =
              object.vx * direction_x + object.vy * direction_y < -kOncomingSpeedMps;
            if (oncoming && wrong_side) {
              // 規則と逆の側へずれるほど罰を大きくする。避け始めのわずかなずれで側が決まらないように、
              // 少し逆へずれていても規則の側へ戻る
              ratio += 2.0 * params.passing.weight * miss / course_reach;
            }
          }
          cost += params.weight_collision_course * ratio * ratio * discount;
        }
      }

      if (params.passing.enabled && speed > kMinDirectionSpeedMps) {
        const double direction_x = vx / speed;
        const double direction_y = vy / speed;
        const double along = relative_x * direction_x + relative_y * direction_y;
        // 正なら相手は進行方向の左にいる
        const double lateral = direction_x * relative_y - direction_y * relative_x;
        const bool wrong_side = params.passing.side_right ? lateral < 0.0 : lateral > 0.0;
        // 向かってくる相手とは規則の側ですれ違う。oncoming.enabled なら、経路をふさいでいる止まっている（遅い）
        // ロボットも規則の側から回り込む。予測の位置をよけない相手（ゴールの近くで出会う正面の相手など。停止線で
        // 止まる）には passing_applicable を付けないので、ここでは数えない。無効なら元の動き: 止まった相手にも
        // 効かせると、近くのゴールへ進めなくなる
        const bool oncoming_object =
          object.vx * direction_x + object.vy * direction_y < -kOncomingSpeedMps;
        const bool applies = params.oncoming.enabled ?
          object.pass_stationary || oncoming_object : oncoming_object;
        if (object.passing_applicable && applies && wrong_side && along > 0.0 &&
          along < params.passing.range_m &&
          std::abs(lateral) < params.passing.lateral_range_m)
        {
          // 横の距離で弱めない。避ける量が大きいと罰がなくなり、どちらの側でも同じになる
          cost += kPassingBaseCost * params.passing.weight *
            (1.0 - along / params.passing.range_m);
        }
      }
    }

    cost += params.weight_effort *
      ((u[0] - previous_ux) * (u[0] - previous_ux) + (u[1] - previous_uy) * (u[1] - previous_uy));
    previous_ux = u[0];
    previous_uy = u[1];
    discount *= params.objects.discount;
  }

  const Point2 & final_target = context.reference.back();
  cost += params.weight_terminal *
    ((x - final_target.x) * (x - final_target.x) + (y - final_target.y) * (y - final_target.y));
  return cost;
}
}  // namespace

Point2 SE2::apply(const double px, const double py) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Point2{x + c * px - s * py, y + s * px + c * py};
}

SE2 SE2::inverse() const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return SE2{-c * x - s * y, s * x - c * y, -yaw};
}

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double pathCurvature(const ReferencePath & path, const double s, const double window_m)
{
  if (path.points().size() < 2U) {
    return 0.0;
  }
  const double window = std::max(window_m, 1e-3);
  const double before = std::clamp(s - window, 0.0, path.length());
  const double after = std::clamp(s + window, 0.0, path.length());
  if (!(after - before > 1e-6)) {
    return 0.0;
  }
  const Point2 a = path.directionAt(before);
  const Point2 b = path.directionAt(after);
  const double turn = std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
  return std::abs(turn) / (after - before);
}

double curveSpeedLimit(const double curvature, const MppiParams & params)
{
  if (!(params.curve.max_lateral_accel_mps2 > 0.0) || !(curvature > 1e-9)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(params.curve.max_lateral_accel_mps2 / curvature);
}

std::optional<HeadingMode> parseHeadingMode(const std::string & text)
{
  if (text == "travel") {
    return HeadingMode::kTravel;
  }
  if (text == "goal") {
    return HeadingMode::kGoal;
  }
  if (text == "forward") {
    return HeadingMode::kForward;
  }
  if (text == "fixed") {
    return HeadingMode::kFixed;
  }
  return std::nullopt;
}

std::optional<double> headingTarget(
  const MppiState & state, const ReferencePath & path, const MppiParams & params)
{
  if (params.heading.yaw_free) {
    return std::nullopt;
  }
  if (params.heading.mode == HeadingMode::kGoal || path.points().size() < 2U) {
    return path.goalYaw();
  }
  const double s = path.project(state.x, state.y);
  const double to_goal = std::max(path.length() - s, 0.0) + path.distanceBeyondEnd();
  if (params.heading.mode == HeadingMode::kFixed) {
    // ゴールの向きの指定がなければ、着くまで保つ向きのまま
    if (to_goal <= params.heading.goal_align_distance_m && path.goalYaw().has_value()) {
      return path.goalYaw();
    }
    return params.heading.fixed_yaw;
  }
  if (to_goal <= params.heading.goal_align_distance_m) {
    return path.goalYaw();
  }
  const Point2 from = path.pointAt(s);
  const Point2 to = path.pointAt(s + std::max(params.heading.lookahead_m, 0.0));
  if (std::hypot(to.x - from.x, to.y - from.y) >= kMinHeadingChordM) {
    return std::atan2(to.y - from.y, to.x - from.x);
  }
  // ゴールの手前で待つ位置など、経路の終わりにいる。終わりの区間の向き（ゴールへ向かう向き）を使う
  const Point2 last = path.directionAt(path.length());
  if (std::hypot(last.x, last.y) < 0.5) {
    return std::nullopt;
  }
  return std::atan2(last.y, last.x);
}

double chassisFaceRate(
  const double chassis_yaw, const double travel_yaw, const double speed,
  const ChassisFaceParams & params)
{
  if (!params.enabled || !(speed >= params.min_speed_mps)) {
    return 0.0;
  }
  // 4 つの面のうち近い面へ（差を ±45° に収める）
  const double quarter = M_PI / 2.0;
  double error = wrapAngle(travel_yaw - chassis_yaw);
  error -= quarter * std::round(error / quarter);
  if (std::abs(error) <= params.deadband_rad) {
    return 0.0;
  }
  const double limit = std::max(params.max_rate_radps, 0.0);
  return std::clamp(params.gain_per_s * error, -limit, limit);
}

double yawRateToward(const double error, const MppiParams & params)
{
  const double magnitude = std::abs(error);
  double rate = std::min(
    params.yaw_gain_per_s * magnitude, std::max(params.max_angular_speed_radps, 0.0));
  const double brake = params.max_angular_accel_radps2 * params.yaw_brake_accel_ratio;
  if (brake > 0.0) {
    // makeYawRates と同じく dt ごとに速さを決めて回る。速さ w から dt ごとに brake × dt ずつ落とすと、
    // 止まるまでに w² / (2 brake) + w × dt / 2 回る。これが |error| 以下になる w の上限
    const double half_step = 0.5 * brake * std::max(params.dt, 0.0);
    rate = std::min(rate, std::sqrt(half_step * half_step + 2.0 * brake * magnitude) - half_step);
  }
  return std::copysign(rate, error);
}

double forwardTurnSpeedLimit(
  const MppiState & state, const std::optional<double> & target_yaw, const MppiParams & params,
  const double speed_limit)
{
  if (params.heading.mode != HeadingMode::kForward || !target_yaw.has_value() ||
    !(std::abs(wrapAngle(*target_yaw - state.yaw)) > params.heading.forward_turn_angle_rad))
  {
    return speed_limit;
  }
  return std::min(speed_limit, std::max(params.heading.forward_turn_speed_mps, 0.0));
}

std::optional<std::string> staticMarginError(const MppiParams & params, const double max_speed_mps)
{
  if (!(params.static_margin_time_s >= 0.0)) {
    std::ostringstream message;
    message << "mppi.static_margin_time_s は 0 以上にしてください: " << params.static_margin_time_s;
    return message.str();
  }
  const double half_length = params.footprint.enabled ? params.footprint.half_length_m : 0.0;
  const double half_width = params.footprint.enabled ? params.footprint.half_width_m : 0.0;
  const double point_speed = std::max(max_speed_mps, 0.0) +
    std::max(params.max_angular_speed_radps, 0.0) * std::hypot(half_length, half_width);
  const double margin = point_speed * params.static_margin_time_s;
  if (margin < kKeepOutCheckMarginM) {
    return std::nullopt;
  }
  std::ostringstream message;
  message << std::fixed << std::setprecision(3) << "mppi.static_margin_time_s（" <<
    params.static_margin_time_s << " 秒）と機体の点の速さの上限（max_speed_mps と、"
    "max_angular_speed_radps × 機体の外接円の半径の和 " << point_speed << " m/s）の積 " << margin <<
    " m が、機体の 4 隅を調べる範囲に足す距離 " << kKeepOutCheckMarginM << " m 以上です。"
    "static_margin_time_s を " << std::floor(1000.0 * kKeepOutCheckMarginM / point_speed) / 1000.0 <<
    " 秒以下にしてください";
  return message.str();
}

std::optional<double> passageSideClearance(
  const DistanceField & field, const SE2 & map_from_odom, const Point2 & point,
  const Point2 & direction, const double half_width)
{
  const double norm = std::hypot(direction.x, direction.y);
  if (!(norm > 0.0)) {
    return std::nullopt;
  }
  const double c = std::cos(map_from_odom.yaw);
  const double s = std::sin(map_from_odom.yaw);
  const double map_x = map_from_odom.x + c * point.x - s * point.y;
  const double map_y = map_from_odom.y + s * point.x + c * point.y;
  // 経路の左向きの法線（地図の座標系）
  const double normal_x = -(s * direction.x + c * direction.y) / norm;
  const double normal_y = (c * direction.x - s * direction.y) / norm;
  // 経路の向き（地図の座標系）
  const double tangent_x = normal_y;
  const double tangent_y = -normal_x;
  const double search = std::max(half_width, 0.0) + kPassageSearchMarginM;
  const GridInfo & info = field.info();
  const double cell = info.resolution;
  // 地図の固定物は、点（0.2m おき）の入ったセルだけが占有になる（点線の壁）。光線が点の間を抜けて
  // 壁の中（点がない）へ入らないよう、半セルずつ進みながら光線の左右 kPassageRayHalfWidthM の幅で占有の
  // セルを探す。最初に見つかったセルの中心までの、法線に沿った距離を、その側の面とする（地図の点は面の
  // 上にあり、点のセルの中心は面から半セル以内。隙間なく埋まった占有では半セル広く見積もる）。
  // 1 セルずつ進むと、標本がセルの境界に乗ったときに丸めで 1 セル飛ばすことがある
  const int lateral_steps = static_cast<int>(std::round(kPassageRayHalfWidthM / cell));
  std::array<double, 2> free_distance{};
  for (std::size_t side = 0; side < free_distance.size(); ++side) {
    const double ux = (side == 0U ? 1.0 : -1.0) * normal_x;
    const double uy = (side == 0U ? 1.0 : -1.0) * normal_y;
    std::optional<double> surface;
    for (double t = 0.0; t <= search + cell && !surface.has_value(); t += 0.5 * cell) {
      for (int lateral = -lateral_steps; lateral <= lateral_steps; ++lateral) {
        const double x = map_x + ux * t + tangent_x * lateral * cell;
        const double y = map_y + uy * t + tangent_y * lateral * cell;
        if (static_cast<double>(field.at(x, y)) > 0.0) {
          continue;
        }
        const GridCell hit = info.cellAt(x, y);
        const double along = (info.centerX(hit.x) - map_x) * ux + (info.centerY(hit.y) - map_y) * uy;
        surface = std::min(surface.value_or(along), along);
      }
    }
    if (!surface.has_value() || *surface > search) {
      return std::nullopt;
    }
    free_distance[side] = std::max(*surface, 0.0);
  }
  return 0.5 * (free_distance[0] + free_distance[1]) - half_width;
}

double passageSpeedLimit(const double side_clearance, const MppiParams & params)
{
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  if (!params.narrow.enabled) {
    return kInfinity;
  }
  if (!(side_clearance + kPassageClearanceToleranceM >= params.narrow.min_side_clearance_m)) {
    return 0.0;
  }
  if (!(params.static_margin_time_s > 0.0)) {
    return kInfinity;
  }
  const double room =
    side_clearance - params.footprint.margin_m - std::max(params.narrow.lateral_tolerance_m, 0.0);
  return std::max(room, 0.0) / params.static_margin_time_s;
}

ReferencePath::ReferencePath(std::vector<Point2> points, std::optional<double> goal_yaw)
: points_(std::move(points)), goal_yaw_(goal_yaw)
{
  cumulative_.reserve(points_.size());
  double length = 0.0;
  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (i > 0U) {
      length += std::hypot(points_[i].x - points_[i - 1U].x, points_[i].y - points_[i - 1U].y);
    }
    cumulative_.push_back(length);
  }
  total_length_ = length;
}

double ReferencePath::project(const double x, const double y) const
{
  if (points_.size() < 2U) {
    return 0.0;
  }
  double best_s = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1U < points_.size(); ++i) {
    const double dx = points_[i + 1U].x - points_[i].x;
    const double dy = points_[i + 1U].y - points_[i].y;
    const double squared = dx * dx + dy * dy;
    const double ratio = squared > 0.0 ?
      std::clamp(((x - points_[i].x) * dx + (y - points_[i].y) * dy) / squared, 0.0, 1.0) : 0.0;
    const double distance = std::hypot(points_[i].x + ratio * dx - x, points_[i].y + ratio * dy - y);
    if (distance < best_distance) {
      best_distance = distance;
      best_s = cumulative_[i] + ratio * (cumulative_[i + 1U] - cumulative_[i]);
    }
  }
  return best_s;
}

Point2 ReferencePath::pointAt(const double s) const
{
  if (points_.empty()) {
    return Point2{};
  }
  if (s <= 0.0 || points_.size() == 1U) {
    return points_.front();
  }
  if (s >= total_length_) {
    return points_.back();
  }
  const auto upper = std::upper_bound(cumulative_.begin(), cumulative_.end(), s);
  const auto i = static_cast<std::size_t>(std::distance(cumulative_.begin(), upper)) - 1U;
  const double segment = cumulative_[i + 1U] - cumulative_[i];
  const double ratio = segment > 0.0 ? (s - cumulative_[i]) / segment : 0.0;
  return Point2{
    points_[i].x + ratio * (points_[i + 1U].x - points_[i].x),
    points_[i].y + ratio * (points_[i + 1U].y - points_[i].y)};
}

ReferencePath ReferencePath::truncated(const double length) const
{
  if (points_.size() < 2U || length >= total_length_) {
    return *this;
  }
  const double clamped = std::max(length, 0.0);
  std::vector<Point2> points;
  points.reserve(points_.size());
  for (std::size_t i = 0; i < points_.size() && cumulative_[i] < clamped; ++i) {
    points.push_back(points_[i]);
  }
  points.push_back(pointAt(clamped));
  ReferencePath result(std::move(points), goal_yaw_);
  result.beyond_end_m_ = beyond_end_m_ + (total_length_ - clamped);
  return result;
}

Point2 ReferencePath::directionAt(const double s) const
{
  if (points_.size() < 2U || total_length_ <= 0.0) {
    return Point2{};
  }
  const double clamped = std::clamp(s, 0.0, total_length_);
  const auto upper = std::upper_bound(cumulative_.begin(), cumulative_.end(), clamped);
  auto i = static_cast<std::size_t>(std::distance(cumulative_.begin(), upper));
  i = std::clamp<std::size_t>(i, 1U, points_.size() - 1U);
  const double dx = points_[i].x - points_[i - 1U].x;
  const double dy = points_[i].y - points_[i - 1U].y;
  const double length = std::hypot(dx, dy);
  return length > 0.0 ? Point2{dx / length, dy / length} : Point2{};
}

MppiCommand limitCommand(
  const MppiState & state, const double vx, const double vy, const double wz,
  const MppiParams & params)
{
  const double max_dv = params.max_accel_mps2 * params.dt;
  const double max_dw = params.max_angular_accel_radps2 * params.dt;
  double dvx = vx - state.vx;
  double dvy = vy - state.vy;
  const double dv = std::hypot(dvx, dvy);
  if (dv > max_dv) {
    dvx *= max_dv / dv;
    dvy *= max_dv / dv;
  }
  MppiCommand command;
  command.vx = state.vx + dvx;
  command.vy = state.vy + dvy;
  command.wz = state.wz + std::clamp(wz - state.wz, -max_dw, max_dw);
  return command;
}

MppiController::MppiController(const std::uint32_t seed)
: random_(seed)
{
}

void MppiController::reset()
{
  mean_.clear();
  last_trajectory_.clear();
}

MppiCommand MppiController::solve(
  const MppiState & state, const ReferencePath & path,
  const std::vector<PredictedObject> & objects, const DistanceField * field,
  const SE2 & map_from_odom, const MppiParams & params, const double speed_limit)
{
  if (path.empty() || params.horizon_steps <= 0 || params.samples <= 0) {
    mean_.clear();
    last_trajectory_.clear();
    return limitCommand(state, 0.0, 0.0, 0.0, params);
  }

  const auto steps = static_cast<std::size_t>(params.horizon_steps);
  const std::size_t sequence_size = steps * kControlSize;
  const auto samples = static_cast<std::size_t>(std::max(params.samples, 3));

  // 向きはすべての系列で同じ。先に決めて、各ステップの機体の前の向きで見えない側の上限をかける
  const auto target_yaw = headingTarget(state, path, params);
  // forward では、向きの差が大きい間は並進を抑えて、その場で向いてから走る。回りながら速く走ると、
  // 機体座標で保たれる指令の速度が向きと一緒に回り、指令の変化の上限（limitCommand）をその引きずりで
  // 使い切って、経路から外れる（Gazebo の CoRE-1 のフィールド、北向きで走り出して東へ進む経路で 0.8〜1.2m）
  const double limit = forwardTurnSpeedLimit(state, target_yaw, params, std::max(speed_limit, 0.0));
  const auto yaw_rates = makeYawRates(state, target_yaw, params);
  const auto forward = forwardDirections(state, yaw_rates, params.dt);
  const BlindSideLimit blind(params.blind_side);

  // 狭い通路: 参照軌道が届く範囲（上限の速さで進む距離と、そこから止まる距離）の通路の余裕を調べる
  PassageProfile passage;
  if (params.narrow.enabled && field != nullptr) {
    const double reach = limit * params.dt * static_cast<double>(params.horizon_steps) +
      limit * limit / (2.0 * std::max(params.reference_decel_mps2, 1e-3)) + kPassageStepM;
    passage = makePassageProfile(
      path, path.project(state.x, state.y), reach, limit, *field, map_from_odom, params);
  }
  // 曲がり: 同じ範囲で、横の加速度の上限から決まる速さを調べる
  const CurveProfile curve = makeCurveProfile(
    path, path.project(state.x, state.y),
    limit * params.dt * static_cast<double>(params.horizon_steps) +
    limit * limit / (2.0 * std::max(params.reference_decel_mps2, 1e-3)) + kCurveStepM, params);
  std::vector<double> reference_s;
  const auto reference = makeReference(
    state, path, params, limit, forward, blind, passage, curve, reference_s);
  std::vector<double> passage_clearance;
  std::vector<Point2> passage_normal;
  if (!passage.clearance.empty()) {
    passage_clearance.reserve(reference_s.size());
    passage_normal.reserve(reference_s.size());
    for (const double s : reference_s) {
      passage_clearance.push_back(passage.clearanceAt(s));
      const Point2 direction = path.directionAt(s);
      passage_normal.push_back(Point2{-direction.y, direction.x});
    }
  }
  auto path_controls = makePathControls(state, reference, params);
  clampControls(path_controls, limit, params.max_angular_speed_radps, forward, blind);

  if (mean_.size() != sequence_size) {
    mean_ = path_controls;
  }
  clampControls(mean_, limit, params.max_angular_speed_radps, forward, blind);

  // 前回の MPPI の軌跡（前回の系列を 1 ステップずらしたものを今の状態から進めた線）。合流と横切りの
  // 交点を、経路ではなく実際に進む線で取る
  std::vector<Point2> trajectory;
  if (params.merge.enabled || params.crossing.enabled) {
    const auto previous_plan = simulateControls(state, mean_.data(), params);
    trajectory.reserve(previous_plan.size());
    std::transform(
      std::next(previous_plan.begin()), previous_plan.end(), std::back_inserter(trajectory),
      [](const MppiState & point) {return Point2{point.x, point.y};});
  }
  const auto predicted = predictObjects(
    state, objects, trajectory, path, path.pointAt(path.project(state.x, state.y)), reference, field,
    map_from_odom, params);

  const FootprintParams & footprint = params.footprint;
  const double half_length = footprint.enabled ? footprint.half_length_m : 0.0;
  const double half_width = footprint.enabled ? footprint.half_width_m : 0.0;
  // 駆動部の形が決まっていれば、その外接円（囲む長方形の外接円より小さい）
  const bool chassis_known = footprint.enabled && footprint.chassis.half_length > 0.0 &&
    footprint.chassis.half_width > 0.0;
  const double circumscribed_radius = chassis_known ?
    std::hypot(footprint.chassis.half_length, footprint.chassis.half_width) :
    std::hypot(half_length, half_width);
  // 系列の速度は今の速度から上限の中の指令へ近づくだけなので、機体の点の速さはこれを超えない
  const double point_speed_bound = std::max(limit, std::hypot(state.vx, state.vy)) +
    std::max(params.max_angular_speed_radps, std::abs(state.wz)) * circumscribed_radius;
  const RolloutContext context{
    state, path, reference, predicted, field, map_from_odom, params, half_length, half_width,
    circumscribed_radius, std::cos(map_from_odom.yaw), std::sin(map_from_odom.yaw),
    std::max(kKeepOutCheckMarginM, point_speed_bound * std::max(params.static_margin_time_s, 0.0)),
    passage_clearance, passage_normal};

  // 候補: 前回の平均、ブレーキ、参照軌道。残りはそれらにノイズを加えたもの
  std::vector<double> sequences(samples * sequence_size, 0.0);
  std::copy(mean_.begin(), mean_.end(), sequences.begin());
  // 2 番目は速度 0 のブレーキ
  std::copy(
    path_controls.begin(), path_controls.end(),
    sequences.begin() + static_cast<std::ptrdiff_t>(2U * sequence_size));

  std::normal_distribution<double> standard_normal(0.0, 1.0);
  const double correlation = params.noise_correlation;
  const double innovation = std::sqrt(std::max(1.0 - correlation * correlation, 0.0));
  for (std::size_t i = 3U; i < samples; ++i) {
    // ブレーキと参照軌道の周りも探すと、前回の結果に引きずられにくい
    const double * base = i % 10U == 0U ? nullptr :
      (i % 5U == 0U ? path_controls.data() : mean_.data());
    double noise_x = standard_normal(random_) * params.noise_linear;
    double noise_y = standard_normal(random_) * params.noise_linear;
    double * sequence = sequences.data() + i * sequence_size;
    for (std::size_t k = 0; k < steps; ++k) {
      double * u = sequence + k * kControlSize;
      const double * b = base != nullptr ? base + k * kControlSize : nullptr;
      u[0] = (b != nullptr ? b[0] : 0.0) + noise_x;
      u[1] = (b != nullptr ? b[1] : 0.0) + noise_y;
      noise_x = correlation * noise_x +
        innovation * standard_normal(random_) * params.noise_linear;
      noise_y = correlation * noise_y +
        innovation * standard_normal(random_) * params.noise_linear;
    }
  }
  clampControls(sequences, limit, params.max_angular_speed_radps, forward, blind);
  for (std::size_t i = 0; i < samples; ++i) {
    for (std::size_t k = 0; k < steps; ++k) {
      sequences[(i * steps + k) * kControlSize + 2U] = yaw_rates[k];
    }
  }

  std::vector<double> costs(samples, 0.0);
  std::vector<std::uint8_t> collisions(samples, 0U);
  for (std::size_t i = 0; i < samples; ++i) {
    bool collision = false;
    costs[i] = rolloutCost(context, sequences.data() + i * sequence_size, collision);
    collisions[i] = collision ? 1U : 0U;
  }
  const auto best_iterator = std::min_element(costs.begin(), costs.end());
  const auto best = static_cast<std::size_t>(std::distance(costs.begin(), best_iterator));
  const double best_cost = *best_iterator;

  // 重み付き平均
  // ゴールの近くではコストの差が小さく、固定の温度だと重みがほぼ均等になって指令が動かない。
  // 差が小さいときは温度を下げて、よい系列を選べるようにする
  const double mean_sample_cost =
    std::accumulate(costs.begin(), costs.end(), 0.0) / static_cast<double>(costs.size());
  const double temperature = std::clamp(
    kAdaptiveTemperatureRatio * (mean_sample_cost - best_cost), 1e-6,
    std::max(params.temperature, 1e-6));
  std::vector<double> weighted_mean(sequence_size, 0.0);
  double weight_sum = 0.0;
  for (std::size_t i = 0; i < samples; ++i) {
    const double weight = std::exp(-(costs[i] - best_cost) / temperature);
    if (weight < 1e-12) {
      continue;
    }
    weight_sum += weight;
    const double * sequence = sequences.data() + i * sequence_size;
    for (std::size_t j = 0; j < sequence_size; ++j) {
      weighted_mean[j] += weight * sequence[j];
    }
  }
  for (double & value : weighted_mean) {
    value /= weight_sum;
  }
  // 見える向きの速い系列と、見えない側の遅い系列の平均は、見えない側へ上限より速く向かうことがある
  limitBlindSide(weighted_mean, forward, blind);

  bool mean_collision = false;
  const double mean_cost = rolloutCost(context, weighted_mean.data(), mean_collision);
  const bool use_best = mean_cost > best_cost + kMeanRejectRatio * params.weight_collision;
  const double * selected = use_best ? sequences.data() + best * sequence_size :
    weighted_mean.data();

  MppiCommand command = limitCommand(state, selected[0], selected[1], selected[2], params);
  command.best_cost = use_best ? best_cost : mean_cost;
  command.collision_free = use_best ? collisions[best] == 0U : !mean_collision;
  last_trajectory_ = simulateControls(state, selected, params);

  // 次の周期のために 1 ステップずらす
  std::vector<double> shifted(sequence_size);
  std::copy(selected + kControlSize, selected + sequence_size, shifted.begin());
  std::copy(
    selected + sequence_size - kControlSize, selected + sequence_size,
    shifted.end() - kControlSize);
  mean_ = std::move(shifted);
  return command;
}

bool isOnCollisionCourse(
  const MppiState & state, const std::vector<PredictedObject> & objects, const MppiParams & params)
{
  return std::any_of(
    objects.begin(), objects.end(),
    [&](const PredictedObject & object) {return objectOnCollisionCourse(state, object, params);});
}

bool needsEvasionWhileStopped(
  const MppiState & state, const std::vector<PredictedObject> & objects, const MppiParams & params,
  const bool evade_all)
{
  return std::any_of(
    objects.begin(), objects.end(),
    [&](const PredictedObject & object) {
      return (evade_all || object.kind == ObjectKind::kHuman) &&
             objectOnCollisionCourse(state, object, params);
    });
}

bool shouldEvadeWhileStopped(
  const MppiState & state, const std::vector<PredictedObject> & objects, const MppiParams & params,
  const bool evade_all, const bool evade_humans)
{
  if (!evade_all && !evade_humans) {
    return false;
  }
  return needsEvasionWhileStopped(state, objects, params, evade_all);
}

namespace
{
// 人の領域が機体にかかるかを調べる時間の刻み [s]
constexpr double kHumanApproachStepS = 0.05;

// 点 (px, py) から機体の外形（余裕を含まない長方形）までの距離。中にあれば 0。
// footprint が無効なら機体の中心までの距離
double distanceToBody(
  const MppiState & state, const FootprintParams & footprint, const double px, const double py)
{
  const double dx = px - state.x;
  const double dy = py - state.y;
  if (!footprint.enabled) {
    return std::hypot(dx, dy);
  }
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  return std::hypot(
    std::max(std::abs(local_x) - footprint.half_length_m, 0.0),
    std::max(std::abs(local_y) - footprint.half_width_m, 0.0));
}

// 人の中心が自機に対して (vx, vy) で動くとき、機体の外形（向きは今のまま）が人の中心から radius 以内に
// 入る最初の時刻。horizon 以内に入らなければ nullopt
std::optional<double> firstRegionEntry(
  const MppiState & state, const FootprintParams & footprint, const PredictedObject & object,
  const double vx, const double vy, const double radius, const double horizon)
{
  for (double t = kHumanApproachStepS; t <= horizon + 1e-9; t += kHumanApproachStepS) {
    if (distanceToBody(state, footprint, object.x + vx * t, object.y + vy * t) <= radius) {
      return t;
    }
  }
  return std::nullopt;
}

// 人の近くでの速度の上限（SpeedLimitParams の human_* の説明を参照）。かからなければ無限大
double humanSpeedLimit(
  const SpeedLimitParams & params, const std::vector<PredictedObject> & objects,
  const MppiState & state, const FootprintParams & footprint)
{
  double limit = std::numeric_limits<double>::infinity();
  for (const auto & object : objects) {
    if (object.kind != ObjectKind::kHuman) {
      continue;
    }
    if (distanceToBody(state, footprint, object.x, object.y) <= params.human_radius_m) {
      limit = std::min(limit, params.human_speed_mps);
      continue;
    }
    // 人と自機がどちらも今の速度のまま進むとき、領域が機体にかかる時刻。かからなければ（横を通る、
    // 自機のほうが速く離れていく）下げない
    const auto entry = firstRegionEntry(
      state, footprint, object, object.vx - state.vx, object.vy - state.vy, params.human_radius_m,
      params.human_approach_time_s);
    if (!entry.has_value()) {
      continue;
    }
    // 人が向かってくる: 人の速度の自機へ向かう成分が下限以上で、自機が止まっているとしても
    // human_approach_time_s 以内に領域が機体にかかる
    const double to_own_x = state.x - object.x;
    const double to_own_y = state.y - object.y;
    const double center_distance = std::max(std::hypot(to_own_x, to_own_y), 1e-6);
    const double toward_own = (object.vx * to_own_x + object.vy * to_own_y) / center_distance;
    if (toward_own >= params.human_approach_speed_mps &&
      firstRegionEntry(
        state, footprint, object, object.vx, object.vy, params.human_radius_m,
        params.human_approach_time_s).has_value())
    {
      limit = std::min(limit, params.human_speed_mps);
      continue;
    }
    // 領域が機体にかかる時刻までに、反応の時間のあと max_decel_mps2 で human_speed_mps に落とせる速さ
    limit = std::min(
      limit, params.human_speed_mps +
      params.max_decel_mps2 * std::max(*entry - params.reaction_time_s, 0.0));
  }
  return limit;
}
}  // namespace

double computeSpeedLimit(
  const SpeedLimitParams & params, const std::vector<PredictedObject> & objects,
  const MppiState & state, const FootprintParams & footprint, const double max_speed)
{
  if (!params.enabled) {
    return max_speed;
  }
  const double x = state.x;
  const double y = state.y;
  const double vx = state.vx;
  const double vy = state.vy;
  double limit = max_speed;

  // 混雑: 近くにいる相手の数だけ倍率を掛ける
  const auto nearby = std::count_if(
    objects.begin(), objects.end(),
    [&](const PredictedObject & object) {
      return std::hypot(object.x - x, object.y - y) <= params.density_radius_m;
    });
  limit *= std::pow(params.density_speed_scale, static_cast<double>(nearby));

  // 見通し: 進行方向の相手の手前で止まれる速度
  const double speed = std::hypot(vx, vy);
  if (speed > kMinDirectionSpeedMps) {
    const double cos_limit = std::cos(M_PI / 3.0);
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & object : objects) {
      const double relative_x = object.x - x;
      const double relative_y = object.y - y;
      const double distance = std::hypot(relative_x, relative_y);
      if (distance <= 1e-6) {
        nearest = 0.0;
        continue;
      }
      const double along = (relative_x * vx + relative_y * vy) / (distance * speed);
      // 通り道から外れた相手と、向かってくる相手では減速しない。
      // 向かってくる相手は止まっても避けられず、減速すると横へ避ける速度まで落ちる
      const double lateral = std::abs(relative_x * vy - relative_y * vx) / speed;
      const double object_along = (object.vx * vx + object.vy * vy) / speed;
      if (along >= cos_limit && lateral <= kSpeedLimitCorridorM &&
        object_along >= -kOncomingSpeedMps)
      {
        nearest = std::min(nearest, distance);
      }
    }
    if (std::isfinite(nearest)) {
      const double available = std::max(nearest - params.stopping_margin_m, 0.0);
      const double decel = params.max_decel_mps2;
      const double reaction = params.reaction_time_s;
      // v * reaction + v^2 / (2 * decel) = available の解
      const double visible_speed =
        decel * (-reaction + std::sqrt(reaction * reaction + 2.0 * available / decel));
      limit = std::min(limit, visible_speed);
    }
  }
  const double clamped = std::clamp(limit, std::min(params.min_speed_mps, max_speed), max_speed);
  // 人の近くの上限は min_speed_mps より優先する
  return params.human_enabled ?
         std::min(clamped, humanSpeedLimit(params, objects, state, footprint)) : clamped;
}

double earliestArrivalTime(
  const double distance, const double speed, const double accel, const double max_speed)
{
  const double clamped = std::max(distance, 0.0);
  if (accel <= 0.0 || speed >= max_speed) {
    return speed > 1e-9 ? clamped / speed : std::numeric_limits<double>::infinity();
  }
  const double start_speed = std::max(speed, 0.0);
  const double accel_time = (max_speed - start_speed) / accel;
  const double accel_distance = start_speed * accel_time + 0.5 * accel * accel_time * accel_time;
  if (clamped <= accel_distance) {
    return (-start_speed + std::sqrt(start_speed * start_speed + 2.0 * accel * clamped)) / accel;
  }
  return accel_time + (clamped - accel_distance) / max_speed;
}

namespace
{
// 経路に沿って、今の速さから max_accel で speed_limit まで上げ、reference_decel で終点に止まる
// として、距離 s_target を越える時刻を返す（makeReference と同じ動き）。届かなければ上限の時刻
double timeToReach(
  const double s_start, const double speed, const double s_target, const double length,
  const MppiParams & params, const double speed_limit)
{
  double s = s_start;
  double v = std::max(speed, 0.0);
  for (int k = 0; k < kMaxArrivalSteps; ++k) {
    if (s >= s_target - 1e-6) {
      return static_cast<double>(k) * params.dt;
    }
    const double braking = std::sqrt(2.0 * params.reference_decel_mps2 * std::max(length - s, 0.0));
    v = std::min({speed_limit, braking, v + params.max_accel_mps2 * params.dt});
    // 終点の手前で速さが 0 に近づいても着けるよう、1 ステップの下限を設ける
    s = std::min(s + std::max(v, kMinDirectionSpeedMps) * params.dt, length);
  }
  return static_cast<double>(kMaxArrivalSteps) * params.dt;
}

// 点 p から、線分 a + t (b - a)（0 <= t <= 1）までの距離
double distanceToSegment(const Point2 & p, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double squared = dx * dx + dy * dy;
  const double t = squared > 1e-12 ?
    std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / squared, 0.0, 1.0) : 0.0;
  return std::hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
}
}  // namespace

std::optional<double> goalHoldLength(
  const MppiState & state, const ReferencePath & path,
  const std::vector<PredictedObject> & objects, const MppiParams & params,
  const GoalApproachParams & goal, const double speed_limit, const bool check_hold_point)
{
  if (!goal.enabled || path.points().size() < 2U) {
    return std::nullopt;
  }
  const double length = path.length();
  const double hold_length = length - goal.hold_distance_m;
  const double s_now = path.project(state.x, state.y);
  // 待つ位置より内側にいれば、そのままゴールへ進む
  if (hold_length <= 0.0 || s_now >= hold_length) {
    return std::nullopt;
  }
  const Point2 direction = path.directionAt(s_now);
  const double speed = state.vx * direction.x + state.vy * direction.y;
  const double enter_s = timeToReach(s_now, speed, hold_length, length, params, speed_limit);
  const double arrive_s = timeToReach(s_now, speed, length, length, params, speed_limit);
  const Point2 goal_point = path.points().back();
  const Point2 hold_point = path.pointAt(hold_length);
  const double body_clearance =
    (params.footprint.enabled ?
    std::hypot(params.footprint.half_length_m, params.footprint.half_width_m) : 0.0) +
    params.footprint.margin_m;
  const double accel = params.objects.accel_mps2;
  return std::any_of(
    objects.begin(), objects.end(), [&](const PredictedObject & object) {
      if (!isMovingRobot(object)) {
        return false;
      }
      const double object_speed = std::hypot(object.vx, object.vy);
      const Point2 unit{object.vx / object_speed, object.vy / object_speed};
      // 直線の上で最も先へ進んだ位置（max_speed まで速くなる）と、最も手前の位置（同じ加速度で
      // 遅くなり、逆向きにもなる）。自機が待つ位置を越えてからゴールに着くまでの間に通りうる範囲
      const double farthest =
        acceleratedDistance(object_speed, accel, params.objects.max_speed_mps, arrive_s);
      const auto nearest_at = [&](const double t) {return object_speed * t - 0.5 * accel * t * t;};
      const double nearest = std::min(nearest_at(enter_s), nearest_at(arrive_s));
      const Point2 from{object.x + unit.x * nearest, object.y + unit.y * nearest};
      const Point2 to{object.x + unit.x * farthest, object.y + unit.y * farthest};
      if (distanceToSegment(goal_point, from, to) > goal.radius_m + object.sigma) {
        return false;
      }
      if (!check_hold_point) {
        return true;
      }
      // 待つ位置も相手の通り道の上なら、待っても離れられないので理由にしない
      const Point2 line_from{object.x - unit.x * 100.0, object.y - unit.y * 100.0};
      const Point2 line_to{object.x + unit.x * 100.0, object.y + unit.y * 100.0};
      return distanceToSegment(hold_point, line_from, line_to) >
             body_clearance + objectRadius(params.objects, object.kind) + object.sigma;
    }) ? std::optional<double>(hold_length) : std::nullopt;
}

void GoalHoldMonitor::reset()
{
  last_update_s_.reset();
  waited_s_ = 0.0;
  holding_ = false;
}

std::optional<double> GoalHoldMonitor::update(
  const double time_s, const MppiState & state, const ReferencePath & path,
  const std::vector<PredictedObject> & objects, const MppiParams & params,
  const GoalApproachParams & goal, const double speed_limit)
{
  const auto requested =
    goalHoldLength(state, path, objects, params, goal, speed_limit, !holding_);
  if (requested.has_value()) {
    last_request_s_ = time_s;
  }
  const double hold_length = path.length() - goal.hold_distance_m;
  const double s_now = path.project(state.x, state.y);
  // 待つ位置の近くで待っていた時間を数える。待つと決めても、待つ位置へ向かって走っている間は数えない
  // （遠くから待ち始めると、着く前に上限に達して、相手が通る最中にゴールへ入ってしまう）
  if (holding_ && last_update_s_.has_value() && s_now >= hold_length - kHoldArrivalM) {
    waited_s_ += std::clamp(time_s - *last_update_s_, 0.0, kMaxHoldStepS);
  }
  last_update_s_ = time_s;
  // 通る予測が途切れても release_s は待ち続ける。待つ位置より内側に入っていれば待たない
  const bool keep = holding_ && time_s - last_request_s_ < goal.release_s && s_now < hold_length;
  const bool expired = waited_s_ >= goal.max_hold_s;
  holding_ = goal.enabled && (requested.has_value() || keep) && !expired;
  return holding_ ? std::optional<double>(hold_length) : std::nullopt;
}

double oncomingSafeSpeed(
  const double gap_m, const double closing_mps, const double margin_m, const double decel)
{
  const double room = gap_m - margin_m;
  if (room <= 0.0 || decel <= 0.0) {
    return 0.0;
  }
  const double closing = std::max(closing_mps, 0.0);
  return -closing + std::sqrt(closing * closing + 2.0 * decel * room);
}

namespace
{
// 交点を探すときに経路をたどる間隔 [m]
constexpr double kCrossingSampleM = 0.2;

// 経路の s_from から range 先までと、相手の今の速度の直線の交点。own_distance_m は s_from からの距離
std::optional<CourseCrossing> pathCourseCrossing(
  const ReferencePath & path, const double s_from, const double range,
  const PredictedObject & object)
{
  const double s_end = std::min(s_from + range, path.length());
  std::vector<Point2> points;
  for (double s = s_from + kCrossingSampleM; s < s_end + kCrossingSampleM; s += kCrossingSampleM) {
    points.push_back(path.pointAt(std::min(s, s_end)));
  }
  return findCourseCrossing(object, path.pointAt(s_from), points, 1.0);
}
}  // namespace

YieldPlan OncomingMonitor::update(
  const double time_s, const MppiState & state, const ReferencePath & path,
  const std::vector<PredictedObject> & objects, const MppiParams & params, const double speed_limit)
{
  YieldPlan plan;
  const OncomingParams & oncoming = params.oncoming;
  if (!oncoming.enabled || path.empty()) {
    encounters_.clear();
    return plan;
  }
  const FootprintParams & footprint = params.footprint;
  const double half_length = footprint.enabled ? footprint.half_length_m : 0.0;
  const double half_width = footprint.enabled ? footprint.half_width_m : 0.0;
  // 機体の中心から前の端（余裕を含む）までの距離
  const double front = half_length + footprint.margin_m;
  const double s_self = path.project(state.x, state.y);
  const Point2 direction = path.directionAt(s_self);
  const double own_speed = std::max(state.vx * direction.x + state.vy * direction.y, 0.0);
  const auto propose_stop = [&](Encounter & encounter, const double stop_s) {
      encounter.last_yield_s = time_s;
      encounter.stop_point = path.pointAt(std::clamp(stop_s, s_self, path.length()));
    };

  for (const auto & object : objects) {
    if (!isMovingRobot(object)) {
      continue;
    }
    const double s_object = path.project(object.x, object.y);
    const double ahead = s_object - s_self;
    if (ahead <= 0.0 || ahead > oncoming.range_m) {
      continue;
    }
    const double speed = std::hypot(object.vx, object.vy);
    const Point2 along = path.directionAt(s_object);
    const double along_speed = object.vx * along.x + object.vy * along.y;
    const double radius = objectRadius(params.objects, object.kind);
    Encounter & encounter = encounters_[object.id];

    if (along_speed <= kHeadOnCos * speed) {
      // 正面: ゴールから遠い所で出会う、通り道に入る相手は、すれ違いの規則の側へよけて通る（止まらない。
      // 予測の位置をよける元の動き。出会いの間は決め直さない）。ゴールの近くで出会う相手と、横を通る相手は
      // 予測の位置をよけず、通り道（自機の半幅と余裕と相手の大きさ）に入るなら停止線と速さの上限で応じる
      if (encounter.passing) {
        encounter.last_pass_s = time_s;
        continue;
      }
      const Point2 base = path.pointAt(s_object);
      const double lateral = along.x * (object.y - base.y) - along.y * (object.x - base.x);
      const double lateral_speed = along.x * object.vy - along.y * object.vx;
      const double closing = std::max(own_speed - along_speed, kMinClosingSpeedMps);
      const double lateral_at_meet = lateral + lateral_speed * ahead / closing;
      const double corridor = half_width + footprint.margin_m + radius;
      const bool on_path = (lateral > 0.0) != (lateral_at_meet > 0.0) ||
        std::min(std::abs(lateral), std::abs(lateral_at_meet)) <= corridor;
      if (on_path && !encounter.midpoint.has_value()) {
        encounter.midpoint = Point2{0.5 * (state.x + object.x), 0.5 * (state.y + object.y)};
      }
      if (on_path && oncoming.pass_goal_margin_m >= 0.0 &&
        path.length() - path.project(encounter.midpoint->x, encounter.midpoint->y) >=
        oncoming.pass_goal_margin_m)
      {
        encounter.passing = true;
        encounter.last_pass_s = time_s;
        continue;
      }
      encounter.last_held_s = time_s;
      if (!on_path) {
        continue;
      }
      propose_stop(
        encounter,
        path.project(encounter.midpoint->x, encounter.midpoint->y) -
        oncoming.stop_before_midpoint_m - front);
      plan.speed_limit = std::min(
        plan.speed_limit,
        oncomingSafeSpeed(
          ahead - front - radius, -along_speed, oncoming.stop_before_midpoint_m,
          params.reference_decel_mps2));
      continue;
    }

    // 合流・横切り: 相手の進路が、自機の経路の前方と交わるか。合流は互いに近づいているときだけ
    const auto crossing = pathCourseCrossing(path, s_self, oncoming.range_m, object);
    if (!crossing.has_value()) {
      continue;
    }
    const double relative_x = object.x - state.x;
    const double relative_y = object.y - state.y;
    const bool approaching =
      relative_x * (object.vx - state.vx) + relative_y * (object.vy - state.vy) < 0.0;
    if (crossing->heading_cos > 0.0 && !approaching) {
      continue;
    }
    encounter.last_held_s = time_s;
    const double s_cross = s_self + crossing->own_distance_m;
    const double own_time =
      timeToReach(s_self, own_speed, s_cross, path.length(), params, speed_limit);
    const double object_time = earliestArrivalTime(
      crossing->object_distance_m, speed, params.objects.accel_mps2, params.objects.max_speed_mps);
    if (object_time - own_time >= params.merge.lead_time_s) {
      // 自機が先に交点を抜ける: 進路を保つ（当てられても相手の責任）
      continue;
    }
    // 相手が先: 交点の手前で、自機の前の端が相手の通り道（大きさと余裕）にかからない所で止まる
    const double sine =
      std::sqrt(std::max(1.0 - crossing->heading_cos * crossing->heading_cos, 0.0));
    propose_stop(
      encounter, s_cross - front - (radius + footprint.margin_m) / std::max(sine, kMinMergeSine));
  }

  // 止まる理由がなくなっても resume_after_s は停止線と hold を保つ（追跡の速度のぶれで止まったり
  // 走ったりしないように）。過ぎた出会いは忘れ、次に出会ったら中点を取り直す
  for (auto it = encounters_.begin(); it != encounters_.end(); ) {
    const Encounter & encounter = it->second;
    const bool yielding = encounter.stop_point.has_value() &&
      time_s - encounter.last_yield_s < oncoming.resume_after_s;
    const bool held = time_s - encounter.last_held_s < oncoming.resume_after_s;
    const bool passing = encounter.passing &&
      time_s - encounter.last_pass_s < oncoming.resume_after_s;
    if (yielding) {
      const double stop =
        std::max(path.project(encounter.stop_point->x, encounter.stop_point->y), s_self);
      plan.stop_length = std::min(plan.stop_length.value_or(stop), stop);
      ++plan.yielding;
    }
    if (!yielding && !held) {
      // 規則の側へよけて通る相手は、予測の位置をよける（hold にしない）。出会いが続く間は覚えておく
      it = passing ? std::next(it) : encounters_.erase(it);
      continue;
    }
    plan.held_ids.push_back(it->first);
    ++it;
  }
  return plan;
}

std::vector<PredictedObject> markHeld(
  std::vector<PredictedObject> objects, const std::vector<std::uint32_t> & held_ids)
{
  for (auto & object : objects) {
    object.hold = std::find(held_ids.begin(), held_ids.end(), object.id) != held_ids.end();
  }
  return objects;
}

void StuckMonitor::reset()
{
  state_ = StuckState::kDriving;
  started_ = false;
  retries_ = 0;
}

void StuckMonitor::holdProgress(const double time_s, const double distance_to_goal)
{
  if (started_ && state_ == StuckState::kDriving) {
    restartProgress(time_s, distance_to_goal);
  }
}

void StuckMonitor::restartProgress(const double time_s, const double distance_to_goal)
{
  best_distance_ = distance_to_goal;
  progress_time_s_ = time_s;
  approach_time_s_ = time_s;
}

StuckState StuckMonitor::update(
  const double time_s, const double distance_to_goal, const double x, const double y,
  const StuckParams & params)
{
  if (!params.enabled) {
    state_ = StuckState::kDriving;
    started_ = false;
    return state_;
  }
  if (!started_) {
    started_ = true;
    state_ = StuckState::kDriving;
    closest_distance_ = distance_to_goal;
    closest_time_s_ = time_s;
    distance_at_last_stuck_ = distance_to_goal;
    restartProgress(time_s, distance_to_goal);
    anchor_ = Point2{x, y};
    return state_;
  }

  if (state_ != StuckState::kFailed && state_ != StuckState::kNearGoal) {
    if (distance_to_goal <= closest_distance_ - params.min_progress_m) {
      closest_distance_ = distance_to_goal;
      closest_time_s_ = time_s;
    } else if (params.give_up_s > 0.0 && time_s - closest_time_s_ >= params.give_up_s) {
      // 塞がれた通路の間を動き回るなど、動いていてもゴールに近づけないままなら、あきらめる
      state_ = StuckState::kFailed;
      return state_;
    }
  }
  switch (state_) {
    case StuckState::kDriving:
      if (distance_to_goal <= best_distance_ - params.min_progress_m) {
        restartProgress(time_s, distance_to_goal);
        anchor_ = Point2{x, y};
      } else if (
        params.moving_distance_m > 0.0 &&
        std::hypot(x - anchor_.x, y - anchor_.y) >= params.moving_distance_m &&
        time_s - approach_time_s_ < params.moving_timeout_s)
      {
        // すき間を行き過ぎて引き返すなど、残りの距離が増えても実際に動いていれば進んでいるとみなす。
        // 残りの距離の基準（best_distance_）は変えない
        progress_time_s_ = time_s;
        anchor_ = Point2{x, y};
      }
      if (distance_to_goal <= distance_at_last_stuck_ - kRetryResetProgressM) {
        // 十分に進めたら、次の行き詰まりは別のものとして数える
        retries_ = 0;
        distance_at_last_stuck_ = distance_to_goal;
      }
      if (time_s - progress_time_s_ >= params.no_progress_timeout_s) {
        if (goal_hard_to_reach_ && params.near_goal_accept_m > 0.0 &&
          distance_to_goal <= params.near_goal_accept_m)
        {
          // 固定物に近いゴールで、いちばん近くまで来て進めないなら、着いたことにする
          state_ = StuckState::kNearGoal;
          break;
        }
        distance_at_last_stuck_ = distance_to_goal;
        state_ = StuckState::kWait;
        wait_start_s_ = time_s;
        wait_until_s_ = time_s + params.wait_s;
      }
      break;
    case StuckState::kWait: {
        // 待っている間に道が空いて前進できるようになっていても、次の判定は走行に戻ってから行う。
        // 計画し直しを待つときは、待ち始めた後の経路を受けるか、上限の時間が過ぎるまで待つ
        const bool replanned = params.replan_timeout_s <= 0.0 ||
          last_plan_time_s_ >= wait_start_s_ ||
          time_s >= wait_start_s_ + params.replan_timeout_s;
        if (time_s < wait_until_s_ || !replanned) {
          break;
        }
        if (retries_ >= params.max_retries) {
          state_ = StuckState::kFailed;
        } else if (params.backoff_distance_m <= 0.0) {
          ++retries_;
          state_ = StuckState::kDriving;
          restartProgress(time_s, distance_to_goal);
          anchor_ = Point2{x, y};
        } else {
          ++retries_;
          state_ = StuckState::kBackoff;
          backoff_start_ = Point2{x, y};
          backoff_start_time_s_ = time_s;
        }
        break;
      }
    case StuckState::kBackoff:
      // 下がれない場合に止まり続けないよう、時間でも打ち切る
      if (std::hypot(x - backoff_start_.x, y - backoff_start_.y) >= params.backoff_distance_m ||
        time_s - backoff_start_time_s_ >= kBackoffTimeoutS)
      {
        state_ = StuckState::kDriving;
        restartProgress(time_s, distance_to_goal);
        anchor_ = Point2{x, y};
      }
      break;
    case StuckState::kFailed:
    case StuckState::kNearGoal:
      break;
  }
  return state_;
}

GoalArrival GoalArrivalMonitor::update(
  const double time_s, const MppiState & state, const Point2 & goal,
  const std::optional<double> & goal_yaw, const GoalArrivalParams & params)
{
  const bool at_position =
    std::hypot(goal.x - state.x, goal.y - state.y) <= params.position_tolerance_m;
  const bool settled = at_position && std::hypot(state.vx, state.vy) <= params.speed_tolerance_mps;
  if (!goal_yaw.has_value() || params.yaw_tolerance_rad < 0.0) {
    return settled ? GoalArrival::kReached : GoalArrival::kApproaching;
  }
  // 時計が戻った（シミュレーションのやり直しなど）ときは、いま着いたとみなす
  if ((settled && !position_reached_s_.has_value()) ||
    (position_reached_s_.has_value() && time_s < *position_reached_s_))
  {
    position_reached_s_ = time_s;
  }
  if (!position_reached_s_.has_value() || !at_position) {
    return GoalArrival::kApproaching;
  }
  const bool aligned = std::abs(wrapAngle(*goal_yaw - state.yaw)) <= params.yaw_tolerance_rad &&
    std::abs(state.wz) <= params.yaw_rate_tolerance_radps;
  if (settled && aligned) {
    return GoalArrival::kReached;
  }
  if (time_s - *position_reached_s_ >= params.yaw_timeout_s) {
    return GoalArrival::kTimedOut;
  }
  return GoalArrival::kAligning;
}

Point2 chooseBackoffDirection(
  const MppiState & state, const ReferencePath & path,
  const std::vector<PredictedObject> & objects, const DistanceField * field,
  const SE2 & map_from_odom, const MppiParams & params)
{
  const Point2 path_direction = path.directionAt(path.project(state.x, state.y));
  Point2 best_direction{-path_direction.x, -path_direction.y};
  double best_score = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < kBackoffDirections; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kBackoffDirections;
    const Point2 direction{std::cos(angle), std::sin(angle)};
    const double probe_x = state.x + kBackoffProbeM * direction.x;
    const double probe_y = state.y + kBackoffProbeM * direction.y;

    double clearance = std::numeric_limits<double>::infinity();
    for (const auto & object : objects) {
      clearance = std::min(
        clearance,
        std::hypot(object.x - probe_x, object.y - probe_y) -
        objectRadius(params.objects, object.kind));
    }
    if (field != nullptr) {
      const Point2 map_point = map_from_odom.apply(probe_x, probe_y);
      clearance = std::min(clearance, static_cast<double>(field->at(map_point.x, map_point.y)));
    }
    // 離れていれば差がつかないよう、評価する距離に上限を設ける
    constexpr double kClearanceCapM = 2.0;
    const double score = std::min(clearance, kClearanceCapM) -
      kBackoffPathPreference * (direction.x * path_direction.x + direction.y * path_direction.y);
    if (score > best_score) {
      best_score = score;
      best_direction = direction;
    }
  }
  return best_direction;
}
}  // namespace core_planning
