// 床との滑り（床が出せる水平の加速度）を、送った指令と測った速度から推定し、計画の加減速度と曲がりの
// 横の加速度の上限に割り当てる。ROS に依存しない。
//
// 駆動の遅れ（drive_delay.hpp の DriveParams。無駄時間と時定数）は機体の値として固定し、床だけを推定する
// （遅れを入れないと、遅れの分を滑りと取り違える）。窓（window_s 秒）の中で horizon_s 秒離れた速度の
// サンプルの組ごとに、前のサンプルの速度から送った指令で進めた予測（predictDelayedState に床の候補を
// 入れたもの）と、後のサンプルの速度との差の二乗平均を求める。床の候補は静止摩擦（kFrictionCandidates と
// 上限なし）。指令の前後・横の速度のうち床で出せる割合（入れないと、床で速度が出きらない分を滑りと取り違える）
// は、前に分かった割合（指令がほぼ一定の組があれば、その終わりの機体座標の速度と指令から最小二乗で求めた値）を
// 中心に ±0.04 を試し、静止摩擦の候補ごとに最も合うものを使う。
// 上限なしより明らかに合う（誤差が min_improvement_ratio 倍以下で、min_improvement_mps 以上小さい）静止摩擦が
// あれば、滑っているとして最も合う値（同じくらい合うなら高い方）を推定にする。
// 滑りが見えなければ、上限なしより明らかに合わない（予測では滑るのに、実際は滑らなかった）静止摩擦より上の
// 値を「滑らずに出せた床の値」（下限）とする。窓の中の速度の変化が min_excitation_mps 未満なら推定しない。
// 割合を覚える（TractionState。初めは 1）のは、滑りが見えない窓だけ（その窓では、速度が出きらない分は割合の
// せい）。1 つの窓で動かす幅を ±0.04 に絞るのは、加速の間は割合と滑りのどちらでも速度の遅れを説明でき、
// 走り始めの滑りを割合として見逃すため。
// 使う床の値は、滑りの推定が今より低ければすぐ下げる。滑りが見えない窓では、下限と「今の 1 段上の候補」の
// 高い方を、滑りの推定が今より高い窓ではその値を上げる候補とし、raise_after 回続いたら、その中の最も低い値へ
// 上げる（低い上限のままでは床の良さが見えないので、1 段ずつ試す。滑れば次の推定で下がる）。

#ifndef CORE_PLANNING__TRACTION_ESTIMATOR_HPP_
#define CORE_PLANNING__TRACTION_ESTIMATOR_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "core_planning/drive_delay.hpp"
#include "core_planning/mppi.hpp"

namespace core_planning
{
struct TractionParams
{
  bool enabled{false};
  // 推定する間隔と、使う記録の長さ [s]
  double update_period_s{2.0};
  double window_s{3.0};
  // 比べる 2 つのサンプルの間隔 [s]
  double horizon_s{0.3};
  double min_excitation_mps{0.3};
  // 滑っているとみなす、上限なしからの誤差の減り（比と差）。LIO の速度のばらつきで誤差の底が約 0.1m/s あり、
  // Gazebo の氷（2.0）でも滑りの分は 5〜10% しか減らない。0.9・0.02 では氷の窓の 1 割しか滑りと見なかった
  double min_improvement_ratio{0.95};
  double min_improvement_mps{0.005};
  int raise_after{2};
  // 推定を始める前の床の静止摩擦 [m/s2]。氷くらい（2.0。上限は加速度 1.67、減速度 1.2、曲がり 1.33）から始め、
  // 滑りが見えなければ 1 段ずつ上げる。3.0（今の既定の上限）から始めると、氷で走り始めに止まりきれないことがあった
  double initial_friction_mps2{2.0};
  // 床の静止摩擦に掛けて、計画の上限にする比
  double accel_ratio{2.5 / 3.0};
  double decel_ratio{0.6};
  double lateral_ratio{2.0 / 3.0};
  // 上限の範囲。上は滑らない床で使う値（加速度と減速度は見直しの前の 3.0、2.0）。曲がりは 3.0 だと、Gazebo の
  // 5m/s で 2m/s 以上のはみ出しの 95% 点が 0.15→0.53m に増えたので、2.5 にする（試験台の周回は 0.5 秒延びる）
  double max_accel_mps2{3.0};
  double max_decel_mps2{2.0};
  double max_lateral_accel_mps2{2.5};
  double min_accel_mps2{1.0};
  double min_decel_mps2{0.8};
  double min_lateral_accel_mps2{1.0};
};

// 推定の候補（床の静止摩擦 [m/s2]）。このほかに上限なしを比べる
inline constexpr std::array<double, 8> kFrictionCandidates{1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0};

// 測った状態（位置・向き・世界座標の速度）と、その時刻
struct OdometrySample
{
  double time_s{0.0};
  MppiState state;
};

struct TractionEstimate
{
  // 窓の中で滑りが見えたか
  bool slipping{false};
  // slipping なら床の静止摩擦の推定、そうでなければ滑らずに出せた床の値（下限。無限大もある）[m/s2]
  double friction_mps2{0.0};
  // 床で出せる速度の割合（前後、横）。ratio_observed（滑りが見えない窓）なら覚えてよい値
  double longitudinal_ratio{1.0};
  double lateral_ratio{1.0};
  bool ratio_observed{false};
  // 選んだ候補と、上限なし（割合は最も合うもの）の速度の誤差（二乗平均）[m/s]
  double error_mps{0.0};
  double no_slip_error_mps{0.0};
  int pairs{0};
};

// 設定が正しくなければ理由を返す
std::optional<std::string> tractionParamsError(const TractionParams & params);

// 時刻 now_s までの window_s 秒の記録（samples は時刻の順）から床を推定する。見分けられなければ nullopt。
// known_ratio は前に分かった割合（前後、横）で、指令が一定の組が少ない窓ではこれだけを使う
std::optional<TractionEstimate> estimateTraction(
  const std::vector<OdometrySample> & samples, const std::vector<SentCommand> & sent,
  const DriveParams & drive, const TractionParams & params, double now_s,
  const std::array<double, 2> & known_ratio = {1.0, 1.0});

// 推定から、使う床の値を決める
class TractionState
{
public:
  explicit TractionState(double initial_friction_mps2 = 3.0);

  // 推定（なければ今の値のまま）を受けて、使う床の値を返す。下げるのは滑りの推定だけ
  double update(const std::optional<TractionEstimate> & estimate, const TractionParams & params);
  double friction() const {return friction_;}
  // 前に分かった速度の割合（前後、横）。滑りが見えない窓の推定を受けるたびに変わる
  const std::array<double, 2> & ratio() const {return ratio_;}
  void reset(double initial_friction_mps2);

private:
  double friction_;
  std::array<double, 2> ratio_{1.0, 1.0};
  // 上げる候補（続いた推定の中の最も低い値）と、続いた回数
  double pending_{0.0};
  int votes_{0};
};

// 床の値 friction_mps2（無限大なら滑らない）を、計画の加減速度と曲がりの横の加速度の上限に割り当てる。
// base の曲がりの上限が 0 以下（使わない設定）なら、曲がりはそのまま
MppiParams applyTraction(
  const MppiParams & base, double friction_mps2, const TractionParams & params);

// 先読みの予測に入れる床。無限大なら滑らない
FloorTraction floorFor(double friction_mps2);
}  // namespace core_planning

#endif  // CORE_PLANNING__TRACTION_ESTIMATOR_HPP_
