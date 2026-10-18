// core_shooter の砲塔モデル（turret_model.hpp）と MPC（turret_mpc.hpp）の単体テスト。
//
// 同定モデルの離散化（むだ時間の端数を含む）、YAML 読み込み、MPC の追従・制約・計算時間、
// 制約付き二次計画（ADMM）の解の正しさを ROS 起動なしで検証する。

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "core_shooter/turret_model.hpp"
#include "core_shooter/turret_mpc.hpp"

using core_shooter::DenseMatrix;
using core_shooter::DiscreteTurretModel;
using core_shooter::TurretAxisModel;
using core_shooter::TurretModelType;
using core_shooter::TurretMpc;
using core_shooter::TurretMpcConfig;
using core_shooter::TurretMpcSolution;

namespace
{

/// テスト用の YAML を一時ファイルへ書き出し、デストラクタで消す。
class TempYaml
{
public:
  explicit TempYaml(const std::string & body)
  {
    static int counter = 0;
    path_ = (std::filesystem::temp_directory_path() /
      ("core_test_turret_mpc_" + std::to_string(++counter) + ".yaml")).string();
    std::ofstream out(path_);
    out << body;
  }

  ~TempYaml() {std::remove(path_.c_str());}

  TempYaml(const TempYaml &) = delete;
  TempYaml & operator=(const TempYaml &) = delete;

  const std::string & path() const {return path_;}

private:
  std::string path_;
};

TurretAxisModel firstOrder(double gain, double tau, double delay)
{
  TurretAxisModel m;
  m.type = TurretModelType::FirstOrder;
  m.gain = gain;
  m.time_constant_sec = tau;
  m.delay_sec = delay;
  return m;
}

TurretAxisModel secondOrder(double gain, double wn, double zeta, double delay)
{
  TurretAxisModel m;
  m.type = TurretModelType::SecondOrder;
  m.gain = gain;
  m.natural_frequency_rad_per_sec = wn;
  m.damping_ratio = zeta;
  m.delay_sec = delay;
  return m;
}

DiscreteTurretModel discretizeOrFail(const TurretAxisModel & model, double ts)
{
  DiscreteTurretModel d;
  std::string reason;
  EXPECT_TRUE(core_shooter::discretizeTurretModel(model, ts, d, reason)) << reason;
  return d;
}

/// 静止状態から時刻 0 に単位ステップを入れたときの y[k] 列。
std::vector<double> stepResponse(const DiscreteTurretModel & d, int steps)
{
  std::vector<double> z = d.restState(0.0, 0.0);
  std::vector<double> y;
  for (int k = 0; k < steps; ++k) {
    y.push_back(d.output(z));
    z = d.step(z, 1.0);
  }
  return y;
}

/// 実機の代わりに使うプラント。細かい刻みで離散化した「真の」モデルを、
/// 制御周期ごとにゼロ次ホールドした指令で回す。
class FinePlant
{
public:
  FinePlant(const TurretAxisModel & model, double control_ts, int substeps, double initial)
  : substeps_(substeps)
  {
    fine_ = discretizeOrFail(model, control_ts / substeps);
    z_ = fine_.restState(initial * model.gain, initial);
  }

  /// 1制御周期だけ指令 u を保持して進め、周期終了時の角度を返す。
  double advance(double u)
  {
    for (int i = 0; i < substeps_; ++i) {
      z_ = fine_.step(z_, u);
    }
    return fine_.output(z_);
  }

  double output() const {return fine_.output(z_);}

private:
  int substeps_;
  DiscreteTurretModel fine_;
  std::vector<double> z_;
};

TurretMpcConfig baseConfig()
{
  TurretMpcConfig c;
  c.sample_time_sec = 1.0 / 30.0;
  c.horizon_steps = 15;
  c.block_lengths = {1, 1, 1, 2, 2, 3, 5};
  c.tracking_weight = 1.0;
  c.terminal_weight = 0.0;
  c.rate_weight = 0.01;
  c.command_min = -3.0;
  c.command_max = 3.0;
  return c;
}

TurretMpc configureOrFail(const TurretAxisModel & model, const TurretMpcConfig & config)
{
  TurretMpc mpc;
  std::string reason;
  EXPECT_TRUE(mpc.configure(model, config, reason)) << reason;
  return mpc;
}

}  // namespace

// ---------------------------------------------------------------------------
// 線形代数ヘルパ
// ---------------------------------------------------------------------------

TEST(TurretModelMath, MatrixExponentialMatchesRotation)
{
  // e^{[[0, -w],[w, 0]] t} は角度 w t の回転行列
  const double angle = 2.5;
  DenseMatrix a(2, 2);
  a(0, 1) = -angle;
  a(1, 0) = angle;
  const DenseMatrix e = core_shooter::matrixExponential(a);
  EXPECT_NEAR(e(0, 0), std::cos(angle), 1e-12);
  EXPECT_NEAR(e(0, 1), -std::sin(angle), 1e-12);
  EXPECT_NEAR(e(1, 0), std::sin(angle), 1e-12);
  EXPECT_NEAR(e(1, 1), std::cos(angle), 1e-12);
}

TEST(TurretModelMath, CholeskyInverseRejectsIndefinite)
{
  DenseMatrix spd(2, 2);
  spd(0, 0) = 4.0;
  spd(0, 1) = 1.0;
  spd(1, 0) = 1.0;
  spd(1, 1) = 3.0;
  DenseMatrix inv;
  ASSERT_TRUE(core_shooter::choleskyInverse(spd, inv));
  const DenseMatrix eye = core_shooter::multiply(spd, inv);
  EXPECT_NEAR(eye(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(eye(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(eye(1, 1), 1.0, 1e-12);

  DenseMatrix indefinite(2, 2);
  indefinite(0, 0) = 1.0;
  indefinite(0, 1) = 2.0;
  indefinite(1, 0) = 2.0;
  indefinite(1, 1) = 1.0;
  EXPECT_FALSE(core_shooter::choleskyInverse(indefinite, inv));
}

// ---------------------------------------------------------------------------
// 離散化
// ---------------------------------------------------------------------------

TEST(TurretModelDiscretize, FirstOrderWithoutDelayMatchesAnalyticStep)
{
  const double ts = 0.01;
  const double tau = 0.08;
  const auto d = discretizeOrFail(firstOrder(1.0, tau, 0.0), ts);
  EXPECT_EQ(d.delay_steps, 0u);
  EXPECT_EQ(d.history_length, 0u);
  const auto y = stepResponse(d, 50);
  for (int k = 0; k < 50; ++k) {
    EXPECT_NEAR(y[k], 1.0 - std::exp(-k * ts / tau), 1e-12) << "k=" << k;
  }
}

TEST(TurretModelDiscretize, FractionalDelayMatchesContinuousResponse)
{
  // 30Hz でむだ時間 50ms = 1周期 + 端数 16.7ms。サンプル時刻での応答は連続時間の解と一致するはず。
  const double ts = 1.0 / 30.0;
  const double tau = 0.06;
  const double delay = 0.05;
  const auto d = discretizeOrFail(firstOrder(0.9, tau, delay), ts);
  EXPECT_EQ(d.delay_steps, 1u);
  EXPECT_NEAR(d.fractional_delay_sec, delay - ts, 1e-12);
  EXPECT_EQ(d.history_length, 2u);

  const auto y = stepResponse(d, 30);
  for (int k = 0; k < 30; ++k) {
    const double t = k * ts - delay;
    const double expected = t > 0.0 ? 0.9 * (1.0 - std::exp(-t / tau)) : 0.0;
    EXPECT_NEAR(y[k], expected, 1e-10) << "k=" << k;
  }
}

TEST(TurretModelDiscretize, SecondOrderMatchesAnalyticUnderdampedStep)
{
  const double ts = 1.0 / 30.0;
  const double wn = 25.0;
  const double zeta = 0.4;
  const double delay = 2.0 * ts;   // 端数なしの整数遅延
  const auto d = discretizeOrFail(secondOrder(1.0, wn, zeta, delay), ts);
  EXPECT_EQ(d.delay_steps, 2u);
  EXPECT_EQ(d.history_length, 2u);
  EXPECT_DOUBLE_EQ(d.fractional_delay_sec, 0.0);

  const double wd = wn * std::sqrt(1.0 - zeta * zeta);
  const double phi = std::acos(zeta);
  const auto y = stepResponse(d, 40);
  for (int k = 0; k < 40; ++k) {
    const double t = k * ts - delay;
    const double expected = t > 0.0 ?
      1.0 - std::exp(-zeta * wn * t) * std::sin(wd * t + phi) / std::sqrt(1.0 - zeta * zeta) :
      0.0;
    EXPECT_NEAR(y[k], expected, 1e-9) << "k=" << k;
  }
}

TEST(TurretModelDiscretize, RejectsInvalidModel)
{
  DiscreteTurretModel d;
  std::string reason;
  EXPECT_FALSE(core_shooter::discretizeTurretModel(firstOrder(1.0, 0.0, 0.0), 0.01, d, reason));
  EXPECT_FALSE(core_shooter::discretizeTurretModel(firstOrder(-1.0, 0.1, 0.0), 0.01, d, reason));
  EXPECT_FALSE(core_shooter::discretizeTurretModel(firstOrder(1.0, 0.1, -0.01), 0.01, d, reason));
  EXPECT_FALSE(
    core_shooter::discretizeTurretModel(secondOrder(1.0, 20.0, 0.0, 0.0), 0.01, d, reason));
  EXPECT_FALSE(core_shooter::discretizeTurretModel(firstOrder(1.0, 0.1, 0.0), 0.0, d, reason));
}

// ---------------------------------------------------------------------------
// YAML 読み込み
// ---------------------------------------------------------------------------

TEST(TurretModelYaml, LoadsSecondOrderAndIgnoresExtraKeys)
{
  const TempYaml yaml(
    "turret_axis_model:\n"
    "  side: left\n"
    "  axis: yaw\n"
    "  model_type: second_order\n"
    "  gain: 0.98\n"
    "  natural_frequency_rad_per_sec: 22.5\n"
    "  damping_ratio: 0.75\n"
    "  delay_sec: 0.045\n"
    "  fit: {valid_fit_percent: 91.2}\n");
  TurretAxisModel model;
  std::string reason;
  ASSERT_TRUE(core_shooter::loadTurretAxisModel(yaml.path(), model, reason)) << reason;
  EXPECT_EQ(model.type, TurretModelType::SecondOrder);
  EXPECT_DOUBLE_EQ(model.gain, 0.98);
  EXPECT_DOUBLE_EQ(model.natural_frequency_rad_per_sec, 22.5);
  EXPECT_DOUBLE_EQ(model.damping_ratio, 0.75);
  EXPECT_DOUBLE_EQ(model.delay_sec, 0.045);
}

TEST(TurretModelYaml, RejectsMissingOrInvalidFields)
{
  TurretAxisModel model;
  std::string reason;
  {
    const TempYaml yaml("turret_axis_model:\n  model_type: first_order\n  gain: 1.0\n");
    EXPECT_FALSE(core_shooter::loadTurretAxisModel(yaml.path(), model, reason));
    EXPECT_NE(reason.find("delay_sec"), std::string::npos) << reason;
  }
  {
    const TempYaml yaml(
      "turret_axis_model:\n  model_type: third_order\n  gain: 1.0\n  delay_sec: 0.0\n");
    EXPECT_FALSE(core_shooter::loadTurretAxisModel(yaml.path(), model, reason));
  }
  {
    const TempYaml yaml(
      "turret_axis_model:\n  model_type: first_order\n  gain: 1.0\n  delay_sec: 0.0\n"
      "  time_constant_sec: -0.1\n");
    EXPECT_FALSE(core_shooter::loadTurretAxisModel(yaml.path(), model, reason));
  }
  EXPECT_FALSE(core_shooter::loadTurretAxisModel("/nonexistent/model.yaml", model, reason));
}

TEST(TurretMpcYaml, LoadsConfigAndConvertsRateLimit)
{
  const TempYaml yaml(
    "turret_mpc:\n"
    "  rate_hz: 50.0\n"
    "  horizon_steps: 10\n"
    "  block_lengths: [1, 2, 3, 4]\n"
    "  rate_weight: 0.1\n"
    "  max_command_rate_rad_per_sec: 5.0\n"
    "  admm: {max_iterations: 100}\n");
  TurretMpcConfig config;
  config.command_min = -1.0;
  config.command_max = 1.0;
  std::string reason;
  ASSERT_TRUE(core_shooter::loadTurretMpcConfig(yaml.path(), config, reason)) << reason;
  EXPECT_DOUBLE_EQ(config.sample_time_sec, 0.02);
  EXPECT_EQ(config.horizon_steps, 10);
  EXPECT_EQ(config.block_lengths, (std::vector<int>{1, 2, 3, 4}));
  EXPECT_DOUBLE_EQ(config.rate_weight, 0.1);
  EXPECT_NEAR(config.max_command_step, 0.1, 1e-12);
  EXPECT_EQ(config.admm_max_iterations, 100);
  // 可動範囲はファイルからは変えない
  EXPECT_DOUBLE_EQ(config.command_min, -1.0);
  EXPECT_DOUBLE_EQ(config.command_max, 1.0);
}

// ---------------------------------------------------------------------------
// 制約付き二次計画（ADMM）
// ---------------------------------------------------------------------------

namespace
{

double qpCost(const DenseMatrix & p, const std::vector<double> & q, const std::vector<double> & x)
{
  const auto px = core_shooter::multiply(p, x);
  return 0.5 * core_shooter::dot(x, px) + core_shooter::dot(q, x);
}

/// 有効制約の組み合わせを総当たりして厳密解を求める（3変数・6制約なら 3^6 通り）。
std::vector<double> bruteForceQp(
  const DenseMatrix & p, const std::vector<double> & q, const DenseMatrix & a,
  const std::vector<double> & lower, const std::vector<double> & upper)
{
  const std::size_t n = p.rows;
  const std::size_t m = a.rows;
  std::size_t combos = 1;
  for (std::size_t i = 0; i < m; ++i) {
    combos *= 3;
  }
  double best_cost = std::numeric_limits<double>::infinity();
  std::vector<double> best;
  for (std::size_t code = 0; code < combos; ++code) {
    // 有効制約 E x = e の下で KKT 系 [P E^T; E 0][x; λ] = [-q; e] を解く
    std::vector<std::size_t> rows;
    std::vector<double> rhs_e;
    std::size_t c = code;
    for (std::size_t i = 0; i < m; ++i) {
      const std::size_t state = c % 3;
      c /= 3;
      if (state == 1) {
        rows.push_back(i);
        rhs_e.push_back(lower[i]);
      } else if (state == 2) {
        rows.push_back(i);
        rhs_e.push_back(upper[i]);
      }
    }
    const std::size_t k = rows.size();
    if (k > n) {
      continue;
    }
    const std::size_t dim = n + k;
    std::vector<double> mat(dim * dim, 0.0);
    std::vector<double> rhs(dim, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < n; ++j) {
        mat[i * dim + j] = p(i, j);
      }
      rhs[i] = -q[i];
    }
    for (std::size_t r = 0; r < k; ++r) {
      for (std::size_t j = 0; j < n; ++j) {
        mat[(n + r) * dim + j] = a(rows[r], j);
        mat[j * dim + n + r] = a(rows[r], j);
      }
      rhs[n + r] = rhs_e[r];
    }
    // 部分ピボット付きガウス消去
    bool singular = false;
    for (std::size_t col = 0; col < dim && !singular; ++col) {
      std::size_t piv = col;
      for (std::size_t r = col + 1; r < dim; ++r) {
        if (std::fabs(mat[r * dim + col]) > std::fabs(mat[piv * dim + col])) {
          piv = r;
        }
      }
      if (std::fabs(mat[piv * dim + col]) < 1e-12) {
        singular = true;
        break;
      }
      for (std::size_t j = 0; j < dim; ++j) {
        std::swap(mat[col * dim + j], mat[piv * dim + j]);
      }
      std::swap(rhs[col], rhs[piv]);
      for (std::size_t r = 0; r < dim; ++r) {
        if (r == col) {
          continue;
        }
        const double f = mat[r * dim + col] / mat[col * dim + col];
        for (std::size_t j = 0; j < dim; ++j) {
          mat[r * dim + j] -= f * mat[col * dim + j];
        }
        rhs[r] -= f * rhs[col];
      }
    }
    if (singular) {
      continue;
    }
    std::vector<double> x(n);
    for (std::size_t i = 0; i < n; ++i) {
      x[i] = rhs[i] / mat[i * dim + i];
    }
    const auto ax = core_shooter::multiply(a, x);
    bool feasible = true;
    for (std::size_t i = 0; i < m; ++i) {
      if (ax[i] < lower[i] - 1e-9 || ax[i] > upper[i] + 1e-9) {
        feasible = false;
      }
    }
    if (!feasible) {
      continue;
    }
    const double cost = qpCost(p, q, x);
    if (cost < best_cost) {
      best_cost = cost;
      best = x;
    }
  }
  return best;
}

}  // namespace

TEST(TurretMpcQp, AdmmMatchesBruteForceWithRateConstraints)
{
  // MPC と同じ形（範囲 + 隣接差分）の制約で、いくつかの一次項について厳密解と比べる。
  DenseMatrix p(3, 3);
  p(0, 0) = 2.0;
  p(0, 1) = 0.5;
  p(1, 0) = 0.5;
  p(1, 1) = 1.5;
  p(1, 2) = 0.3;
  p(2, 1) = 0.3;
  p(2, 2) = 1.0;
  DenseMatrix a(6, 3);
  for (std::size_t i = 0; i < 3; ++i) {
    a(i, i) = 1.0;
    a(3 + i, i) = 1.0;
    if (i > 0) {
      a(3 + i, i - 1) = -1.0;
    }
  }
  const double u_prev = 0.2;
  const double du = 0.15;
  const std::vector<double> lower = {-0.5, -0.5, -0.5, u_prev - du, -du, -du};
  const std::vector<double> upper = {0.6, 0.6, 0.6, u_prev + du, du, du};

  core_shooter::mpc_detail::BoundedQpSolver solver;
  std::string reason;
  ASSERT_TRUE(solver.configure(p, a, 0.5, reason)) << reason;

  const std::vector<std::vector<double>> qs = {
    {-3.0, -2.0, -1.0}, {2.0, 0.0, -2.0}, {0.1, 0.1, 0.1}, {-0.2, 5.0, -4.0}};
  for (const auto & q : qs) {
    const auto exact = bruteForceQp(p, q, a, lower, upper);
    ASSERT_EQ(exact.size(), 3u);
    const auto result = solver.solve(q, lower, upper, {u_prev, u_prev, u_prev}, 2000, 1e-9);
    EXPECT_TRUE(result.converged);
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_NEAR(result.x[i], exact[i], 1e-5) << "q=" << q[0] << "," << q[1] << "," << q[2];
    }
  }
}

// ---------------------------------------------------------------------------
// MPC
// ---------------------------------------------------------------------------

TEST(TurretMpc, RejectsInvalidConfig)
{
  const auto model = firstOrder(1.0, 0.05, 0.04);
  TurretMpc mpc;
  std::string reason;

  auto config = baseConfig();
  config.block_lengths = {1, 2, 3};   // 合計 6 != 15
  EXPECT_FALSE(mpc.configure(model, config, reason));

  config = baseConfig();
  config.horizon_steps = 2;            // むだ時間 2 周期を覆えない
  config.block_lengths = {};
  EXPECT_FALSE(mpc.configure(model, config, reason));
  EXPECT_NE(reason.find("horizon_steps"), std::string::npos) << reason;

  config = baseConfig();
  config.command_min = 1.0;
  config.command_max = -1.0;
  EXPECT_FALSE(mpc.configure(model, config, reason));
}

TEST(TurretMpc, OffsetFreeStepTrackingWithGainMismatch)
{
  // モデルはゲイン 1.0 だが実機（プラント）は 0.9。外乱推定で定常偏差が消えること。
  const auto model = secondOrder(1.0, 20.0, 0.7, 0.05);
  auto plant_model = model;
  plant_model.gain = 0.9;
  const auto config = baseConfig();
  TurretMpc mpc = configureOrFail(model, config);
  FinePlant plant(plant_model, config.sample_time_sec, 20, 0.0);

  double y = plant.output();
  const double target = 0.4;
  for (int k = 0; k < 90; ++k) {
    const std::vector<double> ref(config.horizon_steps, target);
    const TurretMpcSolution sol = mpc.compute(y, ref);
    ASSERT_TRUE(sol.valid);
    mpc.commit(sol.command);
    y = plant.advance(sol.command);
  }
  EXPECT_NEAR(y, target, 2e-3);
}

TEST(TurretMpc, ConstantVelocityTrackingBeatsDirectCommand)
{
  // 等速で動く的を、MPC と「的の現在角をそのまま指令する」方式で追う。
  // むだ時間 + 遅れのぶん直接指令は遅れ続けるが、MPC は先読みで追いつくはず。
  const auto model = firstOrder(1.0, 0.06, 0.06);
  auto config = baseConfig();
  // 3 秒で 3rad 進むので、可動範囲の制約に当たらないよう広げておく
  config.command_min = -10.0;
  config.command_max = 10.0;
  const double ts = config.sample_time_sec;
  const double rate = 1.0;   // [rad/s]

  auto run = [&](bool use_mpc) {
      TurretMpc mpc = configureOrFail(model, config);
      FinePlant plant(model, ts, 20, 0.0);
      double y = plant.output();
      double max_late_error = 0.0;
      for (int k = 0; k < 90; ++k) {
        const double t = k * ts;
        const double target_now = rate * t;
        double u = target_now;
        if (use_mpc) {
          const auto ref = core_shooter::makeConstantVelocityReference(
            target_now, rate, config.horizon_steps, ts);
          const auto sol = mpc.compute(y, ref);
          u = sol.command;
          mpc.commit(u);
        }
        y = plant.advance(u);
        // 周期終了時刻での追従誤差（過渡の 1 秒以降）
        const double target_next = rate * (t + ts);
        if (t > 1.0) {
          max_late_error = std::max(max_late_error, std::fabs(y - target_next));
        }
      }
      return max_late_error;
    };

  const double mpc_error = run(true);
  const double direct_error = run(false);
  // 直接指令の定常遅れは rate*(L + T) = 0.12rad 程度
  EXPECT_GT(direct_error, 0.1);
  EXPECT_LT(mpc_error, 0.005);
}

TEST(TurretMpc, CommandsRespectRangeAndRateLimits)
{
  const auto model = firstOrder(1.0, 0.05, 0.04);
  auto config = baseConfig();
  config.command_min = -0.3;
  config.command_max = 0.5;
  config.max_command_step = 0.05;
  TurretMpc mpc = configureOrFail(model, config);
  FinePlant plant(model, config.sample_time_sec, 10, 0.0);

  double y = plant.output();
  double prev_u = 0.0;
  bool saw_constrained = false;
  // 範囲外の目標へ大きく振る（上限 → 下限）
  for (int k = 0; k < 120; ++k) {
    const double target = (k < 60) ? 1.0 : -1.0;
    const std::vector<double> ref(config.horizon_steps, target);
    const auto sol = mpc.compute(y, ref);
    ASSERT_TRUE(sol.valid);
    saw_constrained = saw_constrained || sol.constrained;
    EXPECT_GE(sol.command, config.command_min - 1e-12) << "k=" << k;
    EXPECT_LE(sol.command, config.command_max + 1e-12) << "k=" << k;
    EXPECT_LE(std::fabs(sol.command - prev_u), config.max_command_step + 1e-12) << "k=" << k;
    prev_u = sol.command;
    mpc.commit(sol.command);
    y = plant.advance(sol.command);
    if (k == 59) {
      EXPECT_NEAR(sol.command, config.command_max, 1e-4);  // ADMM の収束許容誤差程度
    }
  }
  EXPECT_TRUE(saw_constrained);
  EXPECT_NEAR(prev_u, config.command_min, 1e-4);
}

TEST(TurretMpc, ReturnsTowardRangeWhenPreviousCommandIsOutside)
{
  // 可動範囲が狭まり前回指令が範囲外になった場合、変化量制限を守りつつ範囲へ戻る。
  const auto model = firstOrder(1.0, 0.05, 0.0);
  auto config = baseConfig();
  config.max_command_step = 0.1;
  TurretMpc mpc = configureOrFail(model, config);
  mpc.reset(1.0, 1.0);
  mpc.setCommandLimits(-0.5, 0.5);
  const std::vector<double> ref(config.horizon_steps, 0.0);
  const auto sol = mpc.compute(1.0, ref);
  ASSERT_TRUE(sol.valid);
  EXPECT_NEAR(sol.command, 0.9, 1e-12);
}

TEST(TurretMpc, InvalidInputHoldsPreviousCommand)
{
  const auto model = firstOrder(1.0, 0.05, 0.03);
  TurretMpc mpc = configureOrFail(model, baseConfig());
  mpc.reset(0.2, 0.2);
  const auto sol = mpc.compute(std::numeric_limits<double>::quiet_NaN(), {0.5});
  EXPECT_FALSE(sol.valid);
  EXPECT_DOUBLE_EQ(sol.command, 0.2);
  const auto sol_empty = mpc.compute(0.2, {});
  EXPECT_FALSE(sol_empty.valid);
}

TEST(TurretMpc, ResyncsWhenMeasurementJumps)
{
  // 非常停止中に手で動かされた、などでモデルと実測が大きくずれたら同期し直す。
  const auto model = firstOrder(1.0, 0.05, 0.03);
  auto config = baseConfig();
  config.resync_threshold_rad = 0.2;
  TurretMpc mpc = configureOrFail(model, config);
  mpc.reset(0.0, 0.0);
  const std::vector<double> ref(config.horizon_steps, 0.0);
  const auto sol = mpc.compute(1.0, ref);
  EXPECT_TRUE(sol.resynced);
  EXPECT_DOUBLE_EQ(sol.disturbance, 0.0);
}

TEST(TurretMpc, ComputeIsFastEnoughFor30Hz)
{
  const auto model = secondOrder(1.0, 25.0, 0.6, 0.07);
  auto config = baseConfig();
  config.max_command_step = 0.05;   // 制約が頻繁に効く条件で計測する
  TurretMpc mpc = configureOrFail(model, config);
  FinePlant plant(model, config.sample_time_sec, 10, 0.0);
  double y = plant.output();

  const int cycles = 600;
  const auto start = std::chrono::steady_clock::now();
  for (int k = 0; k < cycles; ++k) {
    const double target = (k / 30) % 2 == 0 ? 0.8 : -0.8;
    const auto ref = core_shooter::makeConstantVelocityReference(
      target, 0.0, config.horizon_steps, config.sample_time_sec);
    const auto sol = mpc.compute(y, ref);
    mpc.commit(sol.command);
    y = plant.advance(sol.command);
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  // 30Hz の周期 33ms に対し、1周期あたり 1ms 未満（実測は数十 µs）であること。
  // シミュレーション（プラント計算）込みの時間なので MPC 単体はさらに短い。
  EXPECT_LT(elapsed_ms / cycles, 1.0);
}
