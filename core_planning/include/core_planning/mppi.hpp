// 予測付きの局所計画 MPPI-lite と、速度の上限、行き詰まりの判定。ROS に依存しない。
//
// 状態と制御は世界座標（odom）で扱う。全方位移動なので、並進の指令と機体の向きを分けて考えられる。
// 相手の将来位置を直接コストに入れるので、相手の速度を計画に反映できる。

#ifndef CORE_PLANNING__MPPI_HPP_
#define CORE_PLANNING__MPPI_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "core_planning/blind_side.hpp"
#include "core_planning/footprint.hpp"
#include "core_planning/grid_map.hpp"

namespace core_planning
{
struct Point2
{
  double x{0.0};
  double y{0.0};
};

// 2 次元の座標変換。apply はこの座標系の点を親の座標系に直す
struct SE2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  Point2 apply(double px, double py) const;
  SE2 inverse() const;
};

double wrapAngle(double angle);

enum class ObjectKind : std::uint8_t {kUnknown = 0, kRobot = 1, kHuman = 2};

// 相手の現在の状態。計画の座標系（odom）で表す
struct PredictedObject
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  // 位置の標準偏差。隠れている相手ほど大きい
  double sigma{0.0};
  // 速度の推定の不確かさ [m/s]。追跡を始めた直後などに正
  double velocity_sigma{0.0};
  ObjectKind kind{ObjectKind::kUnknown};
  // 追跡の id。OncomingMonitor が相手ごとの出会いを覚えるのに使う
  std::uint32_t id{0U};
  // 予測の位置をよけない相手か（OncomingMonitor が決める）。今の位置だけを、自機が近づく向きに
  // 動くときに数える。正面から来る相手や、合流・横切りの相手に、横へ道を開けないため
  bool hold{false};
};

// 計画の座標系（odom）での折れ線の経路
class ReferencePath
{
public:
  ReferencePath() = default;
  explicit ReferencePath(std::vector<Point2> points, std::optional<double> goal_yaw = std::nullopt);

  bool empty() const {return points_.empty();}
  double length() const {return total_length_;}
  const std::vector<Point2> & points() const {return points_;}
  const std::optional<double> & goalYaw() const {return goal_yaw_;}

  // 最も近い点の、始点からの距離
  double project(double x, double y) const;
  Point2 pointAt(double s) const;
  // 距離 s での進行方向（単位ベクトル）。経路が 1 点なら (0, 0)
  Point2 directionAt(double s) const;
  // (x, y) からゴールまでの残りの道のり
  double remaining(double x, double y) const {return total_length_ - project(x, y);}
  // 始点から距離 length までの経路（終わりの向きはそのまま）。length が全長以上なら同じ経路。
  // 切った分の長さを distanceBeyondEnd に足す
  ReferencePath truncated(double length) const;
  // この経路の終点から本当のゴールまでの、経路に沿った距離。ゴールの手前で切った経路では正、
  // それ以外は 0。ゴールの向きへ回り始める距離を、切る前の経路で数えるために使う
  double distanceBeyondEnd() const {return beyond_end_m_;}

private:
  std::vector<Point2> points_;
  std::vector<double> cumulative_;
  double total_length_{0.0};
  std::optional<double> goal_yaw_;
  double beyond_end_m_{0.0};
};

struct MppiState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

// 世界座標の速度指令
struct MppiCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double best_cost{0.0};
  // 選んだ系列が、予測の範囲で衝突しないとき true
  bool collision_free{true};
};

struct FootprintParams
{
  // false なら機体を点として扱う
  bool enabled{true};
  double half_length_m{0.4};
  double half_width_m{0.4};
  double margin_m{0.1};
  // 駆動部の形（orientFootprint で決める）。大きさが 0 なら上の長方形を base_link の向きのまま使う
  ChassisShape chassis{};
};

struct ObjectCostParams
{
  // 相手の向きは分からないので、0.8m 角の箱の外接円に近い大きさにする
  double robot_radius_m{0.55};
  double human_radius_m{0.3};
  double unknown_radius_m{0.5};
  // ロボットとの間は目安 0.5m、最小 0.2m（2026-11-14 のユーザーの判断）。どちらも機体の余裕
  // （footprint.margin_m の 0.1m）込みで、相手は向きが分からないので外接円（robot_radius_m）で数える。
  // 余裕（目安から機体の余裕を引いた値）の内側は weight_object の罰、最小の内側は重なりと同じ
  // weight_collision まで上がる罰を足す。最小を 0 にすると前と同じ（余裕だけ）
  double robot_margin_m{0.4};
  double robot_min_margin_m{0.1};
  // 人の中心から機体の外形まで 1.1m（人は 1m 以上よける。2026-11-14 のユーザーの判断）
  double human_margin_m{0.8};
  double unknown_margin_m{0.6};
  // 予測の不確かさの増え方
  double prediction_sigma_growth_mps{0.3};
  // 追跡の速度の不確かさに掛ける倍率。予測の広がりに倍率 * velocity_sigma * t を足す
  double velocity_sigma_gain{1.0};
  // 1 ステップごとに掛ける割引。近い将来ほど重くする
  double discount{0.95};
  // 動いている相手の加速度と速さの上限。合流で相手が交点に着く時刻と、ゴールの近くを通るかを、
  // 相手がこの加速度でこの速さまで速くなる（またはこの加速度で遅くなる）として早めに見積もる
  double accel_mps2{1.5};
  double max_speed_mps{3.0};
};

struct PassingRuleParams
{
  bool enabled{true};
  // true なら自機が右へ避ける（相手が自機の左を通る）
  bool side_right{true};
  double weight{1.0};
  double range_m{4.0};
  double lateral_range_m{3.0};
  // 相手がこの距離より近づいたら規則を適用しない。側を変える時間がなく、回避と打ち消し合って
  // 直進してしまうため
  double commit_distance_m{6.0};
};

// 合流の規則。進む向きの差が 90° 未満で互いに近づいている、動いているロボットとの間に当てる。
// 自機が先に進路の交点を抜けられる相手には、その相手を理由に減速しない。代わりに、2 台の中心を
// 結ぶ線の垂直二等分線を自機の機体（footprint の余裕を含む）が越えるときだけ罰を与える。
// 先に入る相手でも、今の速度のままでは機体どうしが重なる予測になる系列には、衝突コースの罰を与える。
// 相手が先なら、今までどおり避けて後ろにつく。
// 交点は前回の MPPI の軌跡（実際に進む線）で取り、相手が交点に着く時刻は、相手が
// objects.accel_mps2 で objects.max_speed_mps まで速くなるとして早めに見積もる
struct MergeRuleParams
{
  bool enabled{true};
  // 交点を通る時刻が、相手よりこの時間以上早ければ自機が先とみなす
  double lead_time_s{0.5};
};

// 正面から来るロボットと、合流・横切りのロボットへの応じ方（2026-11-14 のユーザーの判断）。
// 予測の位置をよけて横へ道を開けることはせず、経路の上で止まって応じる（OncomingMonitor）。
// ただし、正面の相手とゴールから遠い所（pass_goal_margin_m 以上手前）で出会うときは、止まらずに
// すれ違いの規則の側へよけて通る（予測の位置をよける元の動き）。
// - 正面（進む向きの差 150° 以上）で通り道に入る相手（ゴールの近くで出会うとき）: 出会ったときの 2 台の中点より、自機の前の端
//   （余裕を含む）が stop_before_midpoint_m 手前で止まる。相手がそのまま来ても届く前に止まり終える
//   速さまでに抑える（相手が止まらないと、中点だけでは自機が動いているうちに当たり、自機が原因になる）
// - 合流・横切りで相手が先に交点を通る: 交点の手前で止まって先に通す
// - 合流・横切りで自機が先: 進路を保つ（当てられても相手の責任）
// 相手が止まる理由がなくなってから resume_after_s は停止線を保つ。止まった相手は、止まっている相手として
// すれ違いの規則（passing_rule）の側から回り込む。enabled: false で元の動き
struct OncomingParams
{
  bool enabled{true};
  double stop_before_midpoint_m{0.3};
  double resume_after_s{0.5};
  // 相手を見る、経路に沿った前方の距離 [m]
  double range_m{15.0};
  // 正面で通り道に入る相手と出会う所（出会ったときの 2 台の中点）から、経路に沿ってゴールまでがこれ以上なら、
  // 止まらずにすれ違いの規則の側へよけて通る（予測の位置をよける元の動き。2026-11-14 のユーザーの判断）。
  // 負なら常に停止線で止まる
  double pass_goal_margin_m{3.0};
};

// 横切りの規則。合流（向きの差 90° 未満）でも正面（150° 以上）でもなく、自機の進路の前方で進路が
// 交わる、動いているロボットには、近づく向きの判定を当てずに今までどおり避ける。自機がその交点を
// すでに通り過ぎていれば当てない（後ろや横から当たってくる形は避けない）
struct CrossingRuleParams
{
  bool enabled{true};
};

// 機体の向きの決め方。向きはサンプルせず、決めた目標へ yaw_gain_per_s の比例で回る
enum class HeadingMode : std::uint8_t
{
  // 走っている間は進む向き（参照の経路の先の向き）へ回り、ゴールの近くではゴールの向きへ回る
  kTravel,
  // ゴールの向きがあればそちらへ回り、なければ今の向きを保つ（前の動き）
  kGoal,
  // kTravel と同じ向きへ回り、その向きとの差が forward_turn_angle_rad を超える間は、並進の速さを
  // forward_turn_speed_mps までに抑える（その場で向いてから走る）
  kForward,
  // 走っている間は fixed_yaw を保ち、ゴールの近くではゴールの向きへ回る。fixed_yaw がなければ今の向きを保つ
  kFixed,
};

struct HeadingParams
{
  HeadingMode mode{HeadingMode::kTravel};
  // 進む向きを取る距離 [m]。経路の今の位置（射影）からこの距離先までの弦の向きへ回る
  double lookahead_m{1.0};
  // ゴールまでの経路に沿った残りがこの距離以下なら、ゴールの向きへ回る [m]。ゴールの向きがなければ
  // 今の向きを保つ
  double goal_align_distance_m{1.0};
  // kForward で、向きの目標との差がこの角度を超える間、並進の速さを抑える [rad]
  double forward_turn_angle_rad{M_PI / 4.0};
  // kForward で、向きの差が大きい間の並進の速さの上限 [m/s]
  double forward_turn_speed_mps{0.5};
  // kFixed で保つ向き（odom の座標系）[rad]。ノードがゴールを受けたときの向きを入れる
  std::optional<double> fixed_yaw;
};

// 経路の曲がりで、参照軌道の速さを横の加速度の上限まで落とす。曲率 κ の所の速さを √(上限 / κ) までにし、
// その手前から reference_decel_mps2 で落とす（狭い通路の速さと同じ後ろ向きの計算）。
// 全方位移動の機体は向きを変えずに曲がれるが、速度の向きを変えるのに要る加速度（v² × κ）は変わらない。
// 計画（max_accel_mps2）や床（滑り）が出せる加速度を超えると、曲がりの外へ膨らむ
struct CurveSpeedParams
{
  // 曲がりで出す横の加速度の上限 [m/s²]。0 以下なら速さを落とさない（前の動き）
  double max_lateral_accel_mps2{0.0};
  // 曲率を測る前後の距離 [m]。s − window_m と s + window_m の経路の向きの差を、その間の道のりで割る
  double window_m{0.5};
};

// 狭い通路。経路に直角に左右の静止物を探し、両側が閉じている所を通路とする。機体を通路の真ん中に
// 置いたときの片側の余裕（左右の静止物までの距離の平均 − 機体の半幅）が min_side_clearance_m 以上なら入る。
// 通路の中（入口の手前で機体の角が入口の壁の端に近づく範囲を含む）では:
// - 静止物の余裕（footprint.margin_m + 速さ × static_margin_time_s）の速さを、経路に直角な速さにする
//   （両側の壁は経路と平行なので、壁に沿って走っても余裕を増やさない）
// - 静止物の罰を、機体の点の距離が「通路の余裕 − lateral_tolerance_m」以上ならかけず、そこから余裕までの
//   間にかける（距離の場はセルの中心どうしなので、真ん中にいても 1 セルほど短く読める。片寄ると罰がかかる）
// - 参照軌道の速さを「通路の余裕 − footprint.margin_m − lateral_tolerance_m」を static_margin_time_s で
//   進む速さまでに下げる（片側 0.2m で 0.5m/s、0.3m で 1.5m/s）
// 余裕が min_side_clearance_m 未満の所では、参照軌道がその手前で止まり、罰も元のまま（元の動き）
struct NarrowPassageParams
{
  bool enabled{true};
  double min_side_clearance_m{0.2};
  // 罰をかけない、真ん中からのずれ（格子の丸めの分を含む）。速さの上限もこの分を残す
  double lateral_tolerance_m{0.05};
};

struct MppiParams
{
  int samples{300};
  int horizon_steps{20};
  double dt{0.1};
  // 大きいと左右に割れた候補を平均してしまい、混雑で進めなくなる
  double temperature{1.0};
  double noise_linear{0.8};
  double noise_correlation{0.7};
  double max_accel_mps2{3.0};
  double max_angular_speed_radps{1.5};
  double max_angular_accel_radps2{6.0};
  // 予測で、速度が指令へ近づく一次遅れの時定数 [s]。指令は今の速度から加速度の上限（max_accel_mps2 ×
  // dt）までしか変えない（limitCommand）ので、1 ステップで速度はその変化の 1 − exp(−dt / τ) だけ進む。
  // 0 なら 1 ステップで進む（前の動き）。機体の駆動の時定数（drive_delay.hpp の DriveParams）を入れる
  double drive_time_constant_s{0.0};
  // ゴールの向きへ回る速さの、向きの誤差に対する比
  double yaw_gain_per_s{1.5};
  // 参照軌道がゴールの手前で減速するときの減速度
  double reference_decel_mps2{2.0};
  double keep_out_range_m{0.5};
  // 静止物との余裕（footprint.margin_m）に足す「速さ × この時間」[s]。計画から機体が動くまでの遅れの間に
  // 進む分。速さ × この時間が kKeepOutCheckMarginM を超えないこと（staticMarginError で確かめる）
  double static_margin_time_s{0.1};
  // true なら、上の速さを静止物へ近づく向きの速さだけにする。機体の 4 隅と中心のうち静止物に最も近い点の
  // 速度（回転の分を含む）と、その点の距離の場の勾配の内積から求める。壁と平行に走るときは余裕を
  // 増やさない。勾配が求まらない所では全方向の速さを使う。false で全方向の速さ（元の動き）。
  // 既定は false。true では、相手を避けるときに静止物へ速く向かって止まりきれないことがあった（mppi.yaml）
  bool static_margin_approach_only{false};
  double weight_tracking{1.0};
  double weight_terminal{5.0};
  CurveSpeedParams curve;
  double weight_keep_out{200.0};
  double weight_collision{1000.0};
  double weight_object{50.0};
  double weight_effort{0.1};
  // 動いている相手に向かう速度で走ることの重み。位置のコストだけだと、ぎりぎりまで経路を追って
  // 最大の加速度で避ける系列が最も安くなり、相手が加速して予測が外れると避けきれない
  double weight_collision_course{10.0};
  // 最接近がこの時間より先の相手は評価しない
  double collision_course_lookahead_s{2.5};
  // false なら、動いているロボットとの重なり・接近・衝突コースのコストを、自機がその相手へ近づく
  // 向きに動いているときだけ数える。後ろから追いつかれる、横から当たってくるなど、自機が近づいて
  // いない相手は避けない（当てられても相手の責任）。true で元の動き（常に避ける）。
  // 人、止まっている相手、分類できない相手、合流で先に入る相手、横切りの相手は、この値によらず常に避ける
  bool avoid_being_hit{false};
  FootprintParams footprint;
  ObjectCostParams objects;
  PassingRuleParams passing;
  MergeRuleParams merge;
  CrossingRuleParams crossing;
  OncomingParams oncoming;
  HeadingParams heading;
  // 見えない側へ進むときの速さの上限。系列の各ステップの指令に、そのステップで予測した機体の向きで
  // かける（参照軌道も同じ）
  BlindSideParams blind_side;
  NarrowPassageParams narrow;
};

// 通路を探す、経路から左右への距離 [m]。機体の半幅、余裕、速さの分（3m/s × 0.1 秒）、罰の範囲を
// 足した距離より先に静止物がなければ、その側は開いている（通路ではない）とする
inline constexpr double kPassageSearchMarginM = 0.9;

// 経路の点 point（odom）で、経路の向き direction（odom、単位ベクトル）に直角に左右の静止物までの距離を
// 距離の場でたどり、機体を真ん中に置いたときの片側の余裕（左右の距離の平均 − half_width）を返す。
// 片側でも half_width + kPassageSearchMarginM 以内に静止物がなければ（通路でなければ）nullopt
std::optional<double> passageSideClearance(
  const DistanceField & field, const SE2 & map_from_odom, const Point2 & point,
  const Point2 & direction, double half_width);

// 通路の片側の余裕から決める参照軌道の速さの上限 [m/s]。narrow.enabled でなければ無限大。
// min_side_clearance_m 未満なら 0（その手前で止まる）。それ以上なら、真ん中から lateral_tolerance_m 外れても
// 機体の側面が静止物の余裕（footprint.margin_m + 速さ × static_margin_time_s）の外にある速さ
double passageSpeedLimit(double side_clearance, const MppiParams & params);

// 経路の距離 s での曲率 [1/m]（向きの差 / 道のり）。s − window と s + window（経路の両端で切る）の
// 向きの差を、その間の道のりで割る。直線や 1 点の経路は 0
double pathCurvature(const ReferencePath & path, double s, double window_m);

// 曲率 curvature で横の加速度を curve.max_lateral_accel_mps2 に収める速さ [m/s]。無効か直線なら無限大
double curveSpeedLimit(double curvature, const MppiParams & params);

// "travel"・"goal"・"forward"・"fixed" を HeadingMode に直す。それ以外は nullopt
std::optional<HeadingMode> parseHeadingMode(const std::string & text);

// 機体の向きの目標 [rad]。nullopt なら今の向きを保つ。
// heading.mode が kGoal なら、ゴールの向き（なければ nullopt）。
// kFixed なら、ゴールまでの残りが goal_align_distance_m 以下でゴールの向き（なければ fixed_yaw）。
// それより遠ければ fixed_yaw（なければ nullopt）。
// kTravel と kForward なら、ゴールまでの残り（経路に沿った残りと distanceBeyondEnd の和）が goal_align_distance_m
// 以下でゴールの向き（なければ nullopt）。それより遠ければ、経路の今の位置から lookahead_m 先までの弦の
// 向き。経路の終わり（ゴールの手前で待つ位置）で弦が短ければ、経路の終わりの区間の向き
std::optional<double> headingTarget(
  const MppiState & state, const ReferencePath & path, const MppiParams & params);

// heading.mode が kForward で、向きの目標 target_yaw と今の向きの差が forward_turn_angle_rad を超えるなら、
// speed_limit を forward_turn_speed_mps までに下げて返す。それ以外は speed_limit のまま
double forwardTurnSpeedLimit(
  const MppiState & state, const std::optional<double> & target_yaw, const MppiParams & params,
  double speed_limit);

// 機体の 4 隅を調べる範囲に足す距離 [m]。中心が静止物からこの分と機体の外接円、余裕、罰の範囲を
// 足したより離れていれば、4 隅を調べない。静止物の余裕に足す「速さ × static_margin_time_s」より大きくする
inline constexpr double kKeepOutCheckMarginM = 0.5;

// static_margin_time_s が、機体の点の速さの上限（並進の上限 max_speed_mps と、回転の上限で機体の角が
// 動く速さの和）に掛けても kKeepOutCheckMarginM 未満に収まるかを確かめる。収まらなければ理由を返す
std::optional<std::string> staticMarginError(const MppiParams & params, double max_speed_mps);

// 指令の変化を加速度の上限に収める。機体側の制限より先に抑えて、計画の予測と揃える
MppiCommand limitCommand(
  const MppiState & state, double vx, double vy, double wz, const MppiParams & params);

class MppiController
{
public:
  explicit MppiController(std::uint32_t seed = 0U);

  // 次の周期の速度指令を求める。field と map_from_odom は静的な障害物の距離の場（無ければ nullptr）
  MppiCommand solve(
    const MppiState & state, const ReferencePath & path,
    const std::vector<PredictedObject> & objects, const DistanceField * field,
    const SE2 & map_from_odom, const MppiParams & params, double speed_limit);

  // 前回の結果を捨てる。ゴールが変わったときに呼ぶ
  void reset();

  // 直前の solve で選んだ制御系列を、評価と同じ運動で進めた軌跡（先頭は今の状態）。表示に使う。
  // 系列を選ばなかったときと reset の後は空
  const std::vector<MppiState> & lastTrajectory() const {return last_trajectory_;}

private:
  std::mt19937 random_;
  // 前回の平均の制御系列（ステップごとに vx, vy, wz）
  std::vector<double> mean_;
  std::vector<MppiState> last_trajectory_;
};

struct SpeedLimitParams
{
  bool enabled{true};
  double reaction_time_s{0.2};
  double max_decel_mps2{3.0};
  double density_radius_m{3.0};
  double density_speed_scale{0.9};
  double min_speed_mps{0.5};
  // 相手との距離から引く分（両者の大きさと余裕）
  double stopping_margin_m{1.0};
  // 人の近くの上限（min_speed_mps より優先する。2026-11-14 のユーザーの判断で、人の中心から 3m 以内を
  // 一律に下げるのをやめ、次の 3 つにした）。
  // - 機体の外形（余裕を含まない長方形）が、人の中心から human_radius_m の領域に入っていれば human_speed_mps
  // - 人と自機が今の速度のまま進むと human_approach_time_s 以内に領域が機体にかかるなら、かかる時刻までに
  //   human_speed_mps に落とせる速さ（reaction_time_s のあと max_decel_mps2 で落とす）に抑える。領域の横を
  //   通るだけ、自機のほうが速く離れていくなら抑えない
  // - そのうち人が向かってくる（人の速度の、自機の中心へ向かう成分が human_approach_speed_mps 以上で、
  //   自機が止まっていても human_approach_time_s 以内に領域が機体にかかる）なら human_speed_mps
  bool human_enabled{true};
  double human_radius_m{1.0};
  double human_speed_mps{1.0};
  double human_approach_time_s{2.0};
  double human_approach_speed_mps{0.3};
};

// 動いている相手が、今の速度のまま進むと collision_course_lookahead_s 以内に機体の近くを通るなら
// true を返す。自機が近づいているかは問わない（行き詰まりの判定を止める条件に使う）
bool isOnCollisionCourse(
  const MppiState & state, const std::vector<PredictedObject> & objects, const MppiParams & params);

// 止まっている間（到達後、行き詰まりで待つ・下がる間）に、その場で避ける相手がいるかを返す。
// evade_all が true なら衝突コースの相手すべて、false でも衝突コースの人は避ける
bool needsEvasionWhileStopped(
  const MppiState & state, const std::vector<PredictedObject> & objects, const MppiParams & params,
  bool evade_all);

// 止まっている間に、その場で避けるかを決める。evade_all（mppi.evade_while_stopped）が true なら衝突コースの
// 相手すべてを、evade_humans（mppi.evade_humans_while_stopped）が true なら衝突コースの人を避ける。
// どちらも false なら、人が向かってきても動き出さない（既定。ユーザーの判断）
bool shouldEvadeWhileStopped(
  const MppiState & state, const std::vector<PredictedObject> & objects, const MppiParams & params,
  bool evade_all, bool evade_humans);

// 見通しと混雑と人の近さから速度の上限を決める。state の速度は世界座標。footprint は人の領域を
// 機体の外形で測るのに使う（enabled が false なら機体の中心で測る）
double computeSpeedLimit(
  const SpeedLimitParams & params, const std::vector<PredictedObject> & objects,
  const MppiState & state, const FootprintParams & footprint, double max_speed);

enum class StuckState : std::uint8_t {kDriving, kWait, kBackoff, kFailed};

struct StuckParams
{
  bool enabled{true};
  double no_progress_timeout_s{2.0};
  double min_progress_m{0.2};
  double wait_s{1.0};
  double backoff_distance_m{0.5};
  int max_retries{3};
  // 前に進んだとみなした位置からこの距離以上動けば、ゴールまでの残りが減っていなくても進んでいると
  // みなす（すき間を行き過ぎて引き返すと残りが増える）。0 で残りの距離だけで判定する
  double moving_distance_m{0.5};
  // 動いていることで進んでいるとみなすのは、残りの距離が最後に減ってからこの時間まで。
  // 動き回るだけでゴールへ近づけないときは、行き詰まりと判定する
  double moving_timeout_s{10.0};
};

// 行き詰まりの判定と復帰の状態遷移
class StuckMonitor
{
public:
  // ゴールが変わったときに呼ぶ
  void reset();

  // 相手を避けている間や、ゴールの手前で待っている間に呼ぶ。進めなくても行き詰まりとは数えない
  void holdProgress(double time_s, double distance_to_goal);

  // time_s の時点の、ゴールまでの残りの距離と現在位置で状態を進める
  StuckState update(
    double time_s, double distance_to_goal, double x, double y, const StuckParams & params);

  StuckState state() const {return state_;}
  int retries() const {return retries_;}

private:
  void restartProgress(double time_s, double distance_to_goal);

  StuckState state_{StuckState::kDriving};
  bool started_{false};
  int retries_{0};
  double best_distance_{0.0};
  double progress_time_s_{0.0};
  // 残りの距離が最後に減った（または数えないと決めた）時刻
  double approach_time_s_{0.0};
  double wait_until_s_{0.0};
  double backoff_start_time_s_{0.0};
  double distance_at_last_stuck_{0.0};
  Point2 backoff_start_;
  // 進んだとみなした最後の位置
  Point2 anchor_;
};

// ゴールの手前で待つ判定。ゴールの近くを、動いているロボットが自機の着くまでの間に通る予測なら、
// 経路をゴールの手前で切って待つ（MPPI の参照軌道がそこで止まる）。止まって着いた後に来る相手は
// 避けない方針なので、着くまでの間だけを見る
struct GoalApproachParams
{
  bool enabled{true};
  // 相手の中心が通りうる範囲が、ゴールからこの距離より近ければ、ゴールの近くを通るとする
  double radius_m{1.5};
  // 待つ位置の、経路に沿ったゴールまでの距離。これより内側に入っていれば待たずにゴールへ進む
  double hold_distance_m{2.0};
  // 1 つのゴールで、待つ位置の近く（経路に沿って 0.5m 以内）で待つ時間の上限。過ぎたら待たずに進む
  double max_hold_s{6.0};
  // 待っている間に通る予測がなくなっても、この時間は待ち続ける。端で折り返す相手は、向きを変える間
  // 追跡の速度が小さく向きも定まらないので、通る予測が途切れる
  double release_s{1.0};
};

// ゴールの手前で待つべきなら、待つ位置（経路の始点からの距離）を返す。
// 自機は今の速さから max_accel_mps2 で speed_limit まで上げ、reference_decel_mps2 で止まるとして、
// 待つ位置を越える時刻からゴールに着く時刻までを見る。相手は今の向きの直線の上を、
// objects.accel_mps2 で速く（objects.max_speed_mps まで）も遅く（逆向きまで）もなりうるとする。
// check_hold_point が true なら、待つ位置が相手の通り道の上にあって、待っても離れられない相手は理由に
// しない（待ち始めるときだけ調べる。遅い相手の追跡の向きはぶれ、通り道の判定が変わるため）
std::optional<double> goalHoldLength(
  const MppiState & state, const ReferencePath & path,
  const std::vector<PredictedObject> & objects, const MppiParams & params,
  const GoalApproachParams & goal, double speed_limit, bool check_hold_point = true);

// ゴールの手前で待つかの状態。goalHoldLength に、待ち続ける時間（release_s）と、1 つのゴールで待つ位置の
// 近くで待つ時間の上限（max_hold_s）を足す
class GoalHoldMonitor
{
public:
  // ゴールが変わったときに呼ぶ
  void reset();

  // 待つなら待つ位置（経路の始点からの距離）を返す
  std::optional<double> update(
    double time_s, const MppiState & state, const ReferencePath & path,
    const std::vector<PredictedObject> & objects, const MppiParams & params,
    const GoalApproachParams & goal, double speed_limit);

  bool holding() const {return holding_;}
  // 今のゴールで、待つ位置の近くで待った時間の合計
  double waited() const {return waited_s_;}

private:
  std::optional<double> last_update_s_;
  double last_request_s_{0.0};
  double waited_s_{0.0};
  bool holding_{false};
};

// OncomingMonitor::update の結果
struct YieldPlan
{
  // 経路の始点からの、自機の中心を止める位置 [m]。止まらなくてよければ nullopt
  std::optional<double> stop_length;
  // 速さの上限 [m/s]。正面の相手がそのまま来ても、届く前に止まり終えられる速さ。なければ無限大
  double speed_limit{std::numeric_limits<double>::infinity()};
  // 予測の位置をよけない相手の id
  std::vector<std::uint32_t> held_ids;
  // 止まる理由になっている相手の数（停止線を保っている間を含む）
  int yielding{0};
};

// 正面の相手が近づく速さ closing_mps のまま来ても、前の端どうしの隙間 gap_m が margin_m 残るうちに、
// 減速度 decel で止まり終えられる自機の速さの上限。v·v/(2·decel) + closing·v/decel ≤ gap − margin を解く
double oncomingSafeSpeed(double gap_m, double closing_mps, double margin_m, double decel);

// 正面から来るロボットと、合流・横切りのロボットへの応じ方（OncomingParams）。相手ごとに出会いを覚える
class OncomingMonitor
{
public:
  // ゴールが変わったときに呼ぶ
  void reset() {encounters_.clear();}

  // 止まる位置と速さの上限、予測の位置をよけない相手を決める。objects の動いているロボット
  // （0.3m/s 以上）だけを見る。speed_limit は今の速さの上限で、自機が交点に着く時刻の見積もりに使う
  YieldPlan update(
    double time_s, const MppiState & state, const ReferencePath & path,
    const std::vector<PredictedObject> & objects, const MppiParams & params, double speed_limit);

private:
  struct Encounter
  {
    // 正面の相手と出会ったときの 2 台の中点（odom）。出会いの間は動かさない（相手が近づくほど中点が
    // 自機の側へ動き、下がらなければならなくなるため）
    std::optional<Point2> midpoint;
    // 最後に止まる理由があった時刻と、そのときの停止線の点（odom。経路が引き直されても位置を保つ）
    double last_yield_s{-std::numeric_limits<double>::infinity()};
    std::optional<Point2> stop_point;
    // 最後に予測の位置をよけない相手とした時刻
    double last_held_s{-std::numeric_limits<double>::infinity()};
    // ゴールから遠い所で出会う正面の相手で、規則の側へよけて通ると決めたか（出会いの間は決め直さない）と、
    // 最後にそう判定した時刻
    bool passing{false};
    double last_pass_s{-std::numeric_limits<double>::infinity()};
  };
  std::unordered_map<std::uint32_t, Encounter> encounters_;
};

// held_ids の相手に hold を付けた objects を返す
std::vector<PredictedObject> markHeld(
  std::vector<PredictedObject> objects, const std::vector<std::uint32_t> & held_ids);

// 相手が直線の上を、今の速さ speed から accel で max_speed まで速くなって distance 進むのにかかる
// 時間。accel が 0 以下か、すでに max_speed 以上なら今の速さのまま進む
double earliestArrivalTime(double distance, double speed, double accel, double max_speed);

// 下がる方向を選ぶ。相手と静的な障害物から離れ、経路の進行方向の逆を少し優先する
Point2 chooseBackoffDirection(
  const MppiState & state, const ReferencePath & path,
  const std::vector<PredictedObject> & objects, const DistanceField * field,
  const SE2 & map_from_odom, const MppiParams & params);
}  // namespace core_planning

#endif  // CORE_PLANNING__MPPI_HPP_
