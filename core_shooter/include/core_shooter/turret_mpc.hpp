#ifndef CORE_SHOOTER__TURRET_MPC_HPP_
#define CORE_SHOOTER__TURRET_MPC_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "core_shooter/turret_model.hpp"

/// 砲塔1軸の位置指令を決めるモデル予測制御（MPC）。
///
/// ## 問題設定
/// 同定した「位置指令 u → 実角度 y」のむだ時間付きモデル（turret_model.hpp）を使い、
/// 的の予測角度軌道 r に実角度が重なるよう、次の N 周期分の指令列を毎周期最適化する。
///
///   min  sum_{j=1..N} q_j (y_j - r_j)^2 + sum_{j=0..N-1} w_du (Δu_j - Δr_j)^2
///   s.t. command_min <= u_j <= command_max
///        |Δu_j| <= max_command_step            （Δu_j = u_j - u_{j-1}）
///
/// q_j は tracking_weight（最終段だけ terminal_weight を加算）。
/// 変化量の項は Δu そのものではなく「目標軌道の1周期あたり変化量 Δr との差」に掛ける。
/// Δu だけに罰則を掛けると、等速で動く的を追うとき必要な一定の Δu まで抑えてしまい
/// 定常的な遅れが残るため（Δr = 0 の静止目標では通常の Δu 罰則と同じ）。
///
/// ## ソルバの選定理由
/// - 外部ソルバ（OSQP, qpOASES など）に依存させないため、問題を小さく保って自前で解く。
/// - 入力ブロッキング: N 周期の指令を数個のブロック（同じ値を保持する区間）で表し、
///   決定変数を N から nb（既定 7）に減らす。指令の自由度を手前に集中させても、
///   むだ時間 + 低次遅れの系では追従性能はほとんど落ちない。
/// - 無制約解析解を優先: 二次計画のヘッセ行列はモデルと重みだけで決まるので
///   configure() 時に逆行列を前計算しておく。毎周期は行列ベクトル積だけで無制約最適解が出て、
///   それが制約を満たしていれば（通常の追従時はほぼこれ）厳密な最適解としてそのまま使う。
/// - 制約が効くときだけ ADMM: 変化量制約 |v_i - v_{i-1}| <= du は箱制約ではないので、
///   単純な射影勾配法では射影が解析的に求まらない。ADMM（OSQP と同じ分割）なら
///   箱への射影（クランプ）だけで一般の線形不等式を扱え、係数行列の逆行列も前計算できる。
///   nb=7 なら 1 反復あたり数百 flop で、上限 500 反復でも 1ms 程度。
///   30Hz 周期（33ms）に対して十分小さい（実測は無制約時 数µs、制約時 平均 0.1ms 前後）。
/// - 最後に先頭の指令だけを厳密に範囲・変化量でクランプするので、ADMM が収束途中でも
///   実際に送る指令は必ず制約を満たす。
///
/// ## むだ時間と外乱
/// - むだ時間は過去の指令を状態に含める状態拡大で扱う（DiscreteTurretModel）。
/// - 状態推定は「適用した指令でモデルを回す内部モデル + 出力外乱 d の推定」とする。
///   位置ループ付きサーボは安定系なので内部モデルで十分で、d が定常ゲイン誤差や重力による
///   たわみを吸収してオフセットなし追従になる（DMC と同じ考え方）。
///
/// ROS には依存しない（yaml-cpp のみ）。
namespace core_shooter
{

/// MPC のチューニングパラメータ。
struct TurretMpcConfig
{
  double sample_time_sec = 1.0 / 30.0;   ///< 制御周期 [s]
  int horizon_steps = 15;                ///< 予測ホライズン N [周期]
  /// 入力ブロック長 [周期]。合計は horizon_steps と一致させる。空なら 1 周期ずつ（ブロッキングなし）。
  std::vector<int> block_lengths = {1, 1, 1, 2, 2, 3, 5};
  double tracking_weight = 1.0;          ///< 追従誤差の重み q [1/rad^2]
  double terminal_weight = 0.0;          ///< 最終段に追加する追従誤差の重み [1/rad^2]
  double rate_weight = 0.02;             ///< 指令変化量（目標変化量との差）の重み w_du [1/rad^2]
  double command_min = -M_PI;            ///< 指令角の下限 [rad]
  double command_max = M_PI;             ///< 指令角の上限 [rad]
  /// 1周期あたりの指令変化量の上限 [rad/周期]。inf で無制限。
  double max_command_step = std::numeric_limits<double>::infinity();
  /// 出力外乱推定のゲイン [0..1]。1 で毎周期の誤差を全量採用、0 で外乱推定なし。
  double disturbance_gain = 0.5;
  /// モデル出力と実測の差がこれを超えたら内部モデルを実測へ同期し直す [rad]。
  double resync_threshold_rad = 0.3;
  int admm_max_iterations = 500;
  /// ADMM のペナルティ係数（ヘッセ行列の対角平均に対する比）。
  double admm_rho = 0.3;
  double admm_tolerance = 1e-5;          ///< ADMM の収束判定 [rad]
};

/// MPC 1周期分の結果。
struct TurretMpcSolution
{
  bool valid = false;                ///< false なら入力が不正。command は前回指令（保持）
  double command = 0.0;              ///< 今周期に送る位置指令 [rad]
  bool constrained = false;          ///< 無制約解が制約に掛かり ADMM で解いたか
  bool converged = true;             ///< ADMM が許容誤差内に収束したか
  bool resynced = false;             ///< 今周期に内部モデルを実測へ同期し直したか
  int iterations = 0;                ///< ADMM 反復回数
  double disturbance = 0.0;          ///< 推定した出力外乱 [rad]
  std::vector<double> predicted_output;  ///< y_1..y_N の予測 [rad]
  std::vector<double> planned_commands;  ///< u_0..u_{N-1} の計画 [rad]
};

/// 等速運動する的の予測角度軌道 r_j = angle + rate * (time_offset + j*Ts), j=1..N。
///
/// time_offset_sec は「angle を観測した時刻から今周期までの経過時間」。
/// 画像遅延を補償する場合に正の値を入れる。
inline std::vector<double> makeConstantVelocityReference(
  double angle, double rate, int horizon_steps, double sample_time_sec,
  double time_offset_sec = 0.0)
{
  std::vector<double> reference;
  reference.reserve(static_cast<std::size_t>(std::max(horizon_steps, 0)));
  for (int j = 1; j <= horizon_steps; ++j) {
    reference.push_back(angle + rate * (time_offset_sec + j * sample_time_sec));
  }
  return reference;
}

namespace mpc_detail
{

/// l <= A x <= u の制約付き凸二次計画 min 1/2 x^T P x + q^T x を ADMM で解く。
///
/// P と A は固定で、q / l / u だけが毎回変わる用途向け。(P + sigma I + rho A^T A) の
/// 逆行列を configure() で前計算するので、solve() は行列ベクトル積とクランプだけで済む。
/// 分割は OSQP（Stellato et al., 2020）と同じで、過緩和係数 alpha = 1.6 を使う。
class BoundedQpSolver
{
public:
  struct Result
  {
    std::vector<double> x;
    int iterations = 0;
    bool converged = false;
  };

  bool configure(
    const DenseMatrix & p, const DenseMatrix & a, double rho, std::string & reason)
  {
    if (p.rows != p.cols || a.cols != p.rows || !(rho > 0.0)) {
      reason = "invalid QP dimensions or rho";
      return false;
    }
    a_ = a;
    at_ = transpose(a);
    rho_ = rho;
    DenseMatrix k = multiply(at_, a_);
    for (double & v : k.data) {
      v *= rho_;
    }
    for (std::size_t i = 0; i < k.data.size(); ++i) {
      k.data[i] += p.data[i];
    }
    for (std::size_t i = 0; i < k.rows; ++i) {
      k(i, i) += kSigma;
    }
    if (!choleskyInverse(k, k_inverse_)) {
      reason = "ADMM system matrix is not positive definite";
      return false;
    }
    return true;
  }

  Result solve(
    const std::vector<double> & q, const std::vector<double> & lower,
    const std::vector<double> & upper, const std::vector<double> & x0,
    int max_iterations, double tolerance) const
  {
    const std::size_t n = a_.cols;
    const std::size_t m = a_.rows;
    Result result;
    result.x = x0;
    std::vector<double> z = multiply(a_, x0);
    clampInPlace(z, lower, upper);
    std::vector<double> y(m, 0.0);
    std::vector<double> rhs(n, 0.0);
    std::vector<double> combined(m, 0.0);

    for (int it = 1; it <= max_iterations; ++it) {
      for (std::size_t i = 0; i < m; ++i) {
        combined[i] = rho_ * z[i] - y[i];
      }
      const std::vector<double> at_combined = multiply(at_, combined);
      for (std::size_t i = 0; i < n; ++i) {
        rhs[i] = kSigma * result.x[i] - q[i] + at_combined[i];
      }
      const std::vector<double> x_tilde = multiply(k_inverse_, rhs);
      const std::vector<double> z_tilde = multiply(a_, x_tilde);

      double primal_residual = 0.0;
      double dual_change = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        result.x[i] = kAlpha * x_tilde[i] + (1.0 - kAlpha) * result.x[i];
      }
      for (std::size_t i = 0; i < m; ++i) {
        const double z_relaxed = kAlpha * z_tilde[i] + (1.0 - kAlpha) * z[i];
        const double z_next = std::clamp(z_relaxed + y[i] / rho_, lower[i], upper[i]);
        y[i] += rho_ * (z_relaxed - z_next);
        dual_change = std::max(dual_change, std::fabs(z_next - z[i]));
        z[i] = z_next;
      }
      const std::vector<double> ax = multiply(a_, result.x);
      for (std::size_t i = 0; i < m; ++i) {
        primal_residual = std::max(primal_residual, std::fabs(ax[i] - z[i]));
      }
      result.iterations = it;
      if (primal_residual < tolerance && dual_change < tolerance) {
        result.converged = true;
        break;
      }
    }
    return result;
  }

private:
  static constexpr double kSigma = 1e-6;
  static constexpr double kAlpha = 1.6;

  static void clampInPlace(
    std::vector<double> & v, const std::vector<double> & lower, const std::vector<double> & upper)
  {
    for (std::size_t i = 0; i < v.size(); ++i) {
      v[i] = std::clamp(v[i], lower[i], upper[i]);
    }
  }

  DenseMatrix a_;
  DenseMatrix at_;
  DenseMatrix k_inverse_;
  double rho_ = 1.0;
};

}  // namespace mpc_detail

/// 砲塔1軸の MPC。
///
/// 使い方（1制御周期ごと）:
/// ```cpp
/// const auto ref = makeConstantVelocityReference(target_angle, target_rate, mpc.horizon(), ts);
/// const TurretMpcSolution sol = mpc.compute(measured_angle, ref);
/// double cmd = sol.command;          // 必要なら呼び出し側でさらに制限（ゾーン制限など）
/// publish(cmd);
/// mpc.commit(cmd);                   // 実際に送った指令で内部モデルを進める
/// ```
/// commit() を呼ばずに次の compute() を呼んだ場合は、前回の sol.command を送ったものとみなす。
class TurretMpc
{
public:
  /// モデルとパラメータから予測行列・ヘッセ行列を前計算する。失敗時は reason に理由。
  bool configure(
    const TurretAxisModel & model, const TurretMpcConfig & config, std::string & reason)
  {
    configured_ = false;
    if (!validateConfig(config, reason)) {
      return false;
    }
    DiscreteTurretModel discrete;
    if (!discretizeTurretModel(model, config.sample_time_sec, discrete, reason)) {
      return false;
    }
    const std::size_t n_steps = static_cast<std::size_t>(config.horizon_steps);
    if (n_steps < discrete.delay_steps + 2) {
      reason = "horizon_steps (" + std::to_string(config.horizon_steps) +
        ") must be at least delay_steps + 2 (" + std::to_string(discrete.delay_steps + 2) +
        "); increase horizon_steps or the control rate";
      return false;
    }

    // ブロック構成 M (N x nb): u_j = v_{block(j)}
    std::vector<int> blocks = config.block_lengths;
    if (blocks.empty()) {
      blocks.assign(n_steps, 1);
    }
    const std::size_t nb = blocks.size();
    DenseMatrix blocking(n_steps, nb);
    {
      std::size_t row = 0;
      for (std::size_t b = 0; b < nb; ++b) {
        for (int k = 0; k < blocks[b]; ++k) {
          blocking(row++, b) = 1.0;
        }
      }
    }

    // 予測行列: y_j = c^T A^j z0 + sum_{i<j} h_{j-1-i} u_i （h_m = c^T A^m b）
    const std::size_t nz = discrete.stateDim();
    DenseMatrix free_response(n_steps, nz);
    std::vector<double> markov(n_steps, 0.0);
    {
      // 行ベクトル c^T A^j を順に作る
      std::vector<double> row = discrete.c;
      for (std::size_t j = 0; j < n_steps; ++j) {
        markov[j] = dot(row, discrete.b);   // h_j = c^T A^j b
        std::vector<double> next(nz, 0.0);
        for (std::size_t col = 0; col < nz; ++col) {
          double sum = 0.0;
          for (std::size_t k = 0; k < nz; ++k) {
            sum += row[k] * discrete.a(k, col);
          }
          next[col] = sum;
        }
        row = next;                          // c^T A^{j+1}
        for (std::size_t col = 0; col < nz; ++col) {
          free_response(j, col) = row[col];
        }
      }
    }
    DenseMatrix forced(n_steps, n_steps);
    for (std::size_t j = 0; j < n_steps; ++j) {      // 行 j は y_{j+1}
      for (std::size_t i = 0; i <= j; ++i) {
        forced(j, i) = markov[j - i];
      }
    }
    const DenseMatrix g = multiply(forced, blocking);   // N x nb

    std::vector<double> q_weights(n_steps, config.tracking_weight);
    q_weights.back() += config.terminal_weight;

    // 変化量 ΔU = D M v - e0 u_prev。D M の各行はブロック境界でだけ非ゼロ。
    // 罰則は w_du ||ΔU - ΔR||^2 なので、一次項は -w_du (DM)^T (e0 u_prev + ΔR)。
    DenseMatrix diff_blocking(n_steps, nb);
    for (std::size_t j = 0; j < n_steps; ++j) {
      for (std::size_t b = 0; b < nb; ++b) {
        diff_blocking(j, b) = blocking(j, b) - (j > 0 ? blocking(j - 1, b) : 0.0);
      }
    }

    // P = G^T Q G + w_du (DM)^T (DM)
    DenseMatrix gt_q = transpose(g);
    for (std::size_t b = 0; b < nb; ++b) {
      for (std::size_t j = 0; j < n_steps; ++j) {
        gt_q(b, j) *= q_weights[j];
      }
    }
    DenseMatrix hessian = multiply(gt_q, g);
    const DenseMatrix dm_t = transpose(diff_blocking);
    const DenseMatrix dm_t_dm = multiply(dm_t, diff_blocking);
    for (std::size_t i = 0; i < hessian.data.size(); ++i) {
      hessian.data[i] += config.rate_weight * dm_t_dm.data[i];
    }
    double mean_diag = 0.0;
    for (std::size_t i = 0; i < nb; ++i) {
      mean_diag += hessian(i, i);
    }
    mean_diag = std::max(mean_diag / static_cast<double>(nb), 1e-12);
    // 予測区間に影響しないブロックがあっても逆行列が存在するよう、ごく小さな正則化を足す。
    for (std::size_t i = 0; i < nb; ++i) {
      hessian(i, i) += 1e-9 * mean_diag;
    }

    DenseMatrix hessian_inverse;
    if (!choleskyInverse(hessian, hessian_inverse)) {
      reason = "MPC Hessian is not positive definite (check weights)";
      return false;
    }

    // 制約行列: 上 nb 行は v_i 自身（範囲）、下 nb 行は v_i - v_{i-1}（変化量, v_{-1}=u_prev）
    DenseMatrix constraint(2 * nb, nb);
    for (std::size_t i = 0; i < nb; ++i) {
      constraint(i, i) = 1.0;
      constraint(nb + i, i) = 1.0;
      if (i > 0) {
        constraint(nb + i, i - 1) = -1.0;
      }
    }
    mpc_detail::BoundedQpSolver solver;
    if (!solver.configure(hessian, constraint, config.admm_rho * mean_diag, reason)) {
      return false;
    }

    // 変化量項の一次項の係数 -w_du (DM)^T（nb x N）
    DenseMatrix rate_linear = dm_t;
    for (double & value : rate_linear.data) {
      value *= -config.rate_weight;
    }

    config_ = config;
    config_.block_lengths = blocks;
    model_ = discrete;
    blocking_ = blocking;
    free_response_ = free_response;
    g_ = g;
    gt_q_ = gt_q;
    hessian_inverse_ = hessian_inverse;
    constraint_ = constraint;
    rate_linear_ = rate_linear;
    solver_ = solver;
    configured_ = true;
    initialized_ = false;
    pending_commit_ = false;
    return true;
  }

  /// 内部モデルを「角度 measured_angle で静止、指令 held_command を保持中」に初期化する。
  void reset(double measured_angle, double held_command)
  {
    state_ = model_.restState(measured_angle, held_command);
    disturbance_ = 0.0;
    last_command_ = held_command;
    planned_.assign(blocking_.cols, held_command);
    initialized_ = true;
    pending_commit_ = false;
  }

  /// 指令角の範囲を周期ごとに変える（ゾーン制限で範囲が変わる場合など）。
  void setCommandLimits(double command_min, double command_max)
  {
    config_.command_min = command_min;
    config_.command_max = command_max;
  }

  /// 1周期分の最適化を行い、今周期に送る指令を返す。
  ///
  /// reference は y_1..y_N（今から 1..N 周期後）の目標角。N より短ければ最後の値で埋める。
  TurretMpcSolution compute(double measured_angle, const std::vector<double> & reference)
  {
    TurretMpcSolution sol;
    if (!configured_) {
      return sol;
    }
    if (!std::isfinite(measured_angle) || reference.empty() ||
      !std::all_of(reference.begin(), reference.end(), [](double r) {return std::isfinite(r);}))
    {
      sol.command = last_command_;
      return sol;
    }
    if (!initialized_) {
      reset(measured_angle, measured_angle);
    }
    if (pending_commit_) {
      commit(pending_command_);
    }

    // 出力外乱の更新（ずれが大きすぎるときは内部モデルを同期し直す）
    const double innovation = measured_angle - (model_.output(state_) + disturbance_);
    if (std::fabs(innovation) > config_.resync_threshold_rad) {
      reset(measured_angle, last_command_);
      sol.resynced = true;
    } else {
      disturbance_ += config_.disturbance_gain * innovation;
    }

    const std::size_t n_steps = static_cast<std::size_t>(config_.horizon_steps);
    const std::size_t nb = blocking_.cols;
    const double u_prev = last_command_;

    std::vector<double> y_free = multiply(free_response_, state_);
    std::vector<double> error(n_steps, 0.0);
    for (std::size_t j = 0; j < n_steps; ++j) {
      y_free[j] += disturbance_;
      const double r = reference[std::min(j, reference.size() - 1)];
      error[j] = y_free[j] - r;
    }
    // 目標の1周期あたり変化量 ΔR（先頭は u_prev からの変化なので Δr_0 = r_2 - r_1 で代用）
    std::vector<double> rate_target(n_steps, 0.0);
    for (std::size_t j = 0; j < n_steps; ++j) {
      const std::size_t hi = std::min(std::max<std::size_t>(j, 1), reference.size() - 1);
      const std::size_t lo = hi > 0 ? hi - 1 : 0;
      rate_target[j] = reference[hi] - reference[lo];
    }
    rate_target[0] += u_prev;   // e0 u_prev + ΔR
    std::vector<double> q = multiply(gt_q_, error);
    const std::vector<double> q_rate = multiply(rate_linear_, rate_target);
    for (std::size_t b = 0; b < nb; ++b) {
      q[b] += q_rate[b];
    }

    // 無制約最適解 v = -P^{-1} q
    std::vector<double> v = multiply(hessian_inverse_, q);
    for (double & value : v) {
      value = -value;
    }

    // 制約の上下限
    const double du = config_.max_command_step;
    const double big = 1e9;
    const double du_bound = std::isfinite(du) ? du : big;
    double box_min = config_.command_min;
    double box_max = config_.command_max;
    // 前回指令が範囲外で 1 周期では戻れない場合、範囲制約を外して変化量制約だけで解く
    // （最後に先頭指令を範囲へ向けて最大変化量で戻す）。
    const bool box_reachable =
      std::max(box_min, u_prev - du_bound) <= std::min(box_max, u_prev + du_bound);
    if (!box_reachable) {
      box_min = -big;
      box_max = big;
    }
    std::vector<double> lower(2 * nb), upper(2 * nb);
    for (std::size_t i = 0; i < nb; ++i) {
      lower[i] = box_min;
      upper[i] = box_max;
      lower[nb + i] = -du_bound;
      upper[nb + i] = du_bound;
    }
    lower[nb] += u_prev;
    upper[nb] += u_prev;

    const std::vector<double> av = multiply(constraint_, v);
    bool feasible = true;
    for (std::size_t i = 0; i < av.size(); ++i) {
      if (av[i] < lower[i] - 1e-12 || av[i] > upper[i] + 1e-12) {
        feasible = false;
        break;
      }
    }
    if (!feasible) {
      // 初期値: 無制約解を先頭から順に範囲・変化量でクランプした実行可能点
      std::vector<double> x0 = v;
      double prev = u_prev;
      for (std::size_t i = 0; i < nb; ++i) {
        x0[i] = std::clamp(x0[i], prev - du_bound, prev + du_bound);
        x0[i] = std::clamp(x0[i], box_min, box_max);
        prev = x0[i];
      }
      const auto result = solver_.solve(
        q, lower, upper, x0, config_.admm_max_iterations, config_.admm_tolerance);
      v = result.x;
      sol.constrained = true;
      sol.converged = result.converged;
      sol.iterations = result.iterations;
    }

    sol.valid = true;
    sol.command = limitFirstCommand(v[0], u_prev);
    sol.disturbance = disturbance_;
    sol.planned_commands = multiply(blocking_, v);
    sol.predicted_output = multiply(g_, v);
    for (std::size_t j = 0; j < n_steps; ++j) {
      sol.predicted_output[j] += y_free[j];
    }
    planned_ = v;
    pending_command_ = sol.command;
    pending_commit_ = true;
    return sol;
  }

  /// 実際に送った指令で内部モデルを1周期進める。
  void commit(double applied_command)
  {
    if (!configured_ || !initialized_) {
      return;
    }
    state_ = model_.step(state_, applied_command);
    last_command_ = applied_command;
    pending_commit_ = false;
  }

  bool configured() const {return configured_;}
  bool initialized() const {return initialized_;}
  int horizon() const {return config_.horizon_steps;}
  double sampleTime() const {return config_.sample_time_sec;}
  double lastCommand() const {return last_command_;}
  const TurretMpcConfig & config() const {return config_;}
  const DiscreteTurretModel & discreteModel() const {return model_;}

private:
  static bool validateConfig(const TurretMpcConfig & c, std::string & reason)
  {
    if (!std::isfinite(c.sample_time_sec) || c.sample_time_sec <= 0.0) {
      reason = "sample_time_sec must be positive";
      return false;
    }
    if (c.horizon_steps < 2 || c.horizon_steps > 200) {
      reason = "horizon_steps must be within [2, 200]";
      return false;
    }
    if (!c.block_lengths.empty()) {
      int sum = 0;
      for (int len : c.block_lengths) {
        if (len <= 0) {
          reason = "block_lengths must be positive";
          return false;
        }
        sum += len;
      }
      if (sum != c.horizon_steps) {
        reason = "sum of block_lengths (" + std::to_string(sum) +
          ") must equal horizon_steps (" + std::to_string(c.horizon_steps) + ")";
        return false;
      }
    }
    if (!(c.tracking_weight > 0.0) || !(c.terminal_weight >= 0.0) || !(c.rate_weight >= 0.0)) {
      reason = "weights must be tracking_weight > 0, terminal_weight >= 0, rate_weight >= 0";
      return false;
    }
    if (!(c.command_min < c.command_max)) {
      reason = "command_min must be smaller than command_max";
      return false;
    }
    if (!(c.max_command_step > 0.0)) {
      reason = "max_command_step must be positive (use infinity for no limit)";
      return false;
    }
    if (!(c.disturbance_gain >= 0.0 && c.disturbance_gain <= 1.0)) {
      reason = "disturbance_gain must be within [0, 1]";
      return false;
    }
    if (!(c.resync_threshold_rad > 0.0)) {
      reason = "resync_threshold_rad must be positive";
      return false;
    }
    if (c.admm_max_iterations < 1 || !(c.admm_rho > 0.0) || !(c.admm_tolerance > 0.0)) {
      reason = "admm settings must be max_iterations >= 1, rho > 0, tolerance > 0";
      return false;
    }
    return true;
  }

  /// 先頭指令を範囲・変化量で厳密に制限する。両立しない場合は変化量を優先して範囲へ戻す。
  double limitFirstCommand(double command, double u_prev) const
  {
    const double du = config_.max_command_step;
    const double lo = std::max(config_.command_min, u_prev - du);
    const double hi = std::min(config_.command_max, u_prev + du);
    if (lo <= hi) {
      return std::clamp(command, lo, hi);
    }
    return (u_prev > config_.command_max) ? u_prev - du : u_prev + du;
  }

  bool configured_ = false;
  bool initialized_ = false;
  bool pending_commit_ = false;
  double pending_command_ = 0.0;
  double last_command_ = 0.0;
  double disturbance_ = 0.0;
  TurretMpcConfig config_;
  DiscreteTurretModel model_;
  DenseMatrix blocking_;
  DenseMatrix free_response_;
  DenseMatrix g_;
  DenseMatrix gt_q_;
  DenseMatrix hessian_inverse_;
  DenseMatrix constraint_;
  DenseMatrix rate_linear_;
  std::vector<double> state_;
  std::vector<double> planned_;
  mpc_detail::BoundedQpSolver solver_;
};

// ---------------------------------------------------------------------------
// YAML 読み込み
// ---------------------------------------------------------------------------

/// `turret_mpc:` 節を解釈する。command_min / command_max は呼び出し側（砲塔の可動範囲）が
/// 決めるのでファイルからは読まず、引数 config の値を維持する。
///
/// ```yaml
/// turret_mpc:
///   rate_hz: 30.0
///   horizon_steps: 15
///   block_lengths: [1, 1, 1, 2, 2, 3, 5]
///   tracking_weight: 1.0
///   terminal_weight: 0.0
///   rate_weight: 0.02
///   max_command_rate_rad_per_sec: 0.0   # 0 以下で無制限。rate_hz で 1 周期あたりへ換算
///   disturbance_gain: 0.5
///   resync_threshold_rad: 0.3
///   admm: {max_iterations: 500, rho: 0.3, tolerance: 1.0e-5}
/// ```
/// 省略したキーは config の現在値（既定値）のまま。
inline bool parseTurretMpcConfig(
  const YAML::Node & root, TurretMpcConfig & config, std::string & reason)
{
  try {
    const YAML::Node node = root["turret_mpc"];
    if (!node || !node.IsMap()) {
      reason = "missing 'turret_mpc' map";
      return false;
    }
    TurretMpcConfig parsed = config;
    if (node["rate_hz"]) {
      const double rate_hz = node["rate_hz"].as<double>();
      if (!(rate_hz > 0.0)) {
        reason = "rate_hz must be positive";
        return false;
      }
      parsed.sample_time_sec = 1.0 / rate_hz;
    }
    if (node["horizon_steps"]) {
      parsed.horizon_steps = node["horizon_steps"].as<int>();
    }
    if (node["block_lengths"]) {
      parsed.block_lengths = node["block_lengths"].as<std::vector<int>>();
    }
    if (node["tracking_weight"]) {
      parsed.tracking_weight = node["tracking_weight"].as<double>();
    }
    if (node["terminal_weight"]) {
      parsed.terminal_weight = node["terminal_weight"].as<double>();
    }
    if (node["rate_weight"]) {
      parsed.rate_weight = node["rate_weight"].as<double>();
    }
    if (node["max_command_rate_rad_per_sec"]) {
      const double rate = node["max_command_rate_rad_per_sec"].as<double>();
      parsed.max_command_step = rate > 0.0 ?
        rate * parsed.sample_time_sec :
        std::numeric_limits<double>::infinity();
    }
    if (node["disturbance_gain"]) {
      parsed.disturbance_gain = node["disturbance_gain"].as<double>();
    }
    if (node["resync_threshold_rad"]) {
      parsed.resync_threshold_rad = node["resync_threshold_rad"].as<double>();
    }
    if (const YAML::Node admm = node["admm"]) {
      if (admm["max_iterations"]) {
        parsed.admm_max_iterations = admm["max_iterations"].as<int>();
      }
      if (admm["rho"]) {
        parsed.admm_rho = admm["rho"].as<double>();
      }
      if (admm["tolerance"]) {
        parsed.admm_tolerance = admm["tolerance"].as<double>();
      }
    }
    config = parsed;
    return true;
  } catch (const YAML::Exception & e) {
    reason = std::string("YAML error: ") + e.what();
    return false;
  }
}

/// MPC 設定 YAML を読み込む。失敗時は reason に理由を入れて false を返す。
inline bool loadTurretMpcConfig(
  const std::string & path, TurretMpcConfig & config, std::string & reason)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    reason = "failed to load '" + path + "': " + e.what();
    return false;
  }
  if (!parseTurretMpcConfig(root, config, reason)) {
    reason = "'" + path + "': " + reason;
    return false;
  }
  return true;
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__TURRET_MPC_HPP_
