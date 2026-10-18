#ifndef CORE_SHOOTER__TURRET_MODEL_HPP_
#define CORE_SHOOTER__TURRET_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

/// 砲塔1軸（yaw または pitch）の「位置指令 → 実角度」モデル。
///
/// モータは位置指令しか受け付けず、ファームウェア内部の位置ループの応答は未知なので、
/// システム同定（scripts/turret_sysid_identify.py）で次の形のモデルを推定して使う。
///
///   first_order  : y(s)/u(s) = K e^{-Ls} / (T s + 1)
///   second_order : y(s)/u(s) = K wn^2 e^{-Ls} / (s^2 + 2 zeta wn s + wn^2)
///
/// u は位置指令 [rad]、y は /joint_states の実測角 [rad]、L はむだ時間 [s]。
/// ROS には依存しない（yaml-cpp のみ）ので単体テストからそのまま使える。
namespace core_shooter
{

// ---------------------------------------------------------------------------
// 小さな密行列
// ---------------------------------------------------------------------------

/// 行優先の小さな密行列。
///
/// MPC で扱う行列は高々数十次元なので、外部ライブラリに頼らず必要な演算だけを持つ。
struct DenseMatrix
{
  std::size_t rows = 0;
  std::size_t cols = 0;
  std::vector<double> data;

  DenseMatrix() = default;
  DenseMatrix(std::size_t r, std::size_t c, double value = 0.0)
  : rows(r), cols(c), data(r * c, value) {}

  static DenseMatrix identity(std::size_t n)
  {
    DenseMatrix m(n, n);
    for (std::size_t i = 0; i < n; ++i) {
      m(i, i) = 1.0;
    }
    return m;
  }

  double & operator()(std::size_t r, std::size_t c) {return data[r * cols + c];}
  double operator()(std::size_t r, std::size_t c) const {return data[r * cols + c];}
};

inline DenseMatrix multiply(const DenseMatrix & a, const DenseMatrix & b)
{
  DenseMatrix out(a.rows, b.cols);
  for (std::size_t i = 0; i < a.rows; ++i) {
    for (std::size_t k = 0; k < a.cols; ++k) {
      const double aik = a(i, k);
      if (aik == 0.0) {
        continue;
      }
      for (std::size_t j = 0; j < b.cols; ++j) {
        out(i, j) += aik * b(k, j);
      }
    }
  }
  return out;
}

inline std::vector<double> multiply(const DenseMatrix & a, const std::vector<double> & x)
{
  std::vector<double> out(a.rows, 0.0);
  for (std::size_t i = 0; i < a.rows; ++i) {
    double sum = 0.0;
    for (std::size_t j = 0; j < a.cols; ++j) {
      sum += a(i, j) * x[j];
    }
    out[i] = sum;
  }
  return out;
}

inline DenseMatrix transpose(const DenseMatrix & a)
{
  DenseMatrix out(a.cols, a.rows);
  for (std::size_t i = 0; i < a.rows; ++i) {
    for (std::size_t j = 0; j < a.cols; ++j) {
      out(j, i) = a(i, j);
    }
  }
  return out;
}

inline double dot(const std::vector<double> & a, const std::vector<double> & b)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
    sum += a[i] * b[i];
  }
  return sum;
}

/// 行列指数関数 e^A（スケーリング・スクエアリング + テイラー展開）。
///
/// 離散化に使う 2〜3 次元の行列が対象。ノルムを 0.5 以下に縮めてから 20 次まで展開するので
/// 倍精度の丸め誤差程度の精度が出る。
inline DenseMatrix matrixExponential(const DenseMatrix & a)
{
  const std::size_t n = a.rows;
  double norm = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    double row_sum = 0.0;
    for (std::size_t j = 0; j < n; ++j) {
      row_sum += std::fabs(a(i, j));
    }
    norm = std::max(norm, row_sum);
  }
  int squarings = 0;
  if (norm > 0.5) {
    squarings = static_cast<int>(std::ceil(std::log2(norm / 0.5)));
  }
  DenseMatrix scaled = a;
  const double factor = std::ldexp(1.0, -squarings);
  for (double & v : scaled.data) {
    v *= factor;
  }

  DenseMatrix result = DenseMatrix::identity(n);
  DenseMatrix term = DenseMatrix::identity(n);
  for (int k = 1; k <= 20; ++k) {
    term = multiply(term, scaled);
    for (double & v : term.data) {
      v /= static_cast<double>(k);
    }
    for (std::size_t i = 0; i < result.data.size(); ++i) {
      result.data[i] += term.data[i];
    }
  }
  for (int s = 0; s < squarings; ++s) {
    result = multiply(result, result);
  }
  return result;
}

/// 対称正定値行列の逆行列をコレスキー分解で求める。正定値でなければ false。
inline bool choleskyInverse(const DenseMatrix & spd, DenseMatrix & inverse)
{
  const std::size_t n = spd.rows;
  if (spd.cols != n) {
    return false;
  }
  DenseMatrix l(n, n);
  for (std::size_t j = 0; j < n; ++j) {
    double diag = spd(j, j);
    for (std::size_t k = 0; k < j; ++k) {
      diag -= l(j, k) * l(j, k);
    }
    if (!(diag > 0.0) || !std::isfinite(diag)) {
      return false;
    }
    l(j, j) = std::sqrt(diag);
    for (std::size_t i = j + 1; i < n; ++i) {
      double value = spd(i, j);
      for (std::size_t k = 0; k < j; ++k) {
        value -= l(i, k) * l(j, k);
      }
      l(i, j) = value / l(j, j);
    }
  }

  // L Y = I を前進代入、L^T X = Y を後退代入で解く。
  inverse = DenseMatrix(n, n);
  for (std::size_t col = 0; col < n; ++col) {
    std::vector<double> y(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      double value = (i == col) ? 1.0 : 0.0;
      for (std::size_t k = 0; k < i; ++k) {
        value -= l(i, k) * y[k];
      }
      y[i] = value / l(i, i);
    }
    for (std::size_t ii = n; ii-- > 0; ) {
      double value = y[ii];
      for (std::size_t k = ii + 1; k < n; ++k) {
        value -= l(k, ii) * inverse(k, col);
      }
      inverse(ii, col) = value / l(ii, ii);
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 連続時間モデル
// ---------------------------------------------------------------------------

/// 同定モデルの型。
enum class TurretModelType
{
  FirstOrder,   ///< むだ時間 + 1次遅れ
  SecondOrder,  ///< むだ時間 + 2次遅れ
};

/// 砲塔1軸の連続時間モデル（位置指令 [rad] → 実角度 [rad]）。
struct TurretAxisModel
{
  TurretModelType type = TurretModelType::FirstOrder;
  double gain = 1.0;                             ///< 定常ゲイン K [-]
  double time_constant_sec = 0.05;               ///< 1次遅れの時定数 T [s]（first_order）
  double natural_frequency_rad_per_sec = 20.0;   ///< 固有角周波数 wn [rad/s]（second_order）
  double damping_ratio = 0.7;                    ///< 減衰比 zeta [-]（second_order）
  double delay_sec = 0.0;                        ///< むだ時間 L [s]
};

/// モデルパラメータが物理的に意味のある範囲かを確認する。
inline bool validateTurretAxisModel(const TurretAxisModel & model, std::string & reason)
{
  if (!std::isfinite(model.gain) || model.gain <= 0.0) {
    reason = "gain must be positive";
    return false;
  }
  if (!std::isfinite(model.delay_sec) || model.delay_sec < 0.0 || model.delay_sec > 1.0) {
    reason = "delay_sec must be within [0, 1] s";
    return false;
  }
  if (model.type == TurretModelType::FirstOrder) {
    if (!std::isfinite(model.time_constant_sec) || model.time_constant_sec < 1e-4) {
      reason = "time_constant_sec must be >= 1e-4 s";
      return false;
    }
  } else {
    if (!std::isfinite(model.natural_frequency_rad_per_sec) ||
      model.natural_frequency_rad_per_sec <= 0.0 ||
      model.natural_frequency_rad_per_sec > 1e4)
    {
      reason = "natural_frequency_rad_per_sec must be within (0, 1e4]";
      return false;
    }
    if (!std::isfinite(model.damping_ratio) || model.damping_ratio <= 0.0 ||
      model.damping_ratio > 10.0)
    {
      reason = "damping_ratio must be within (0, 10]";
      return false;
    }
  }
  return true;
}

/// 連続時間の状態空間表現 x' = A x + B u, y = C x（むだ時間は含まない）。
struct ContinuousStateSpace
{
  DenseMatrix a;
  std::vector<double> b;
  std::vector<double> c;
};

inline ContinuousStateSpace toContinuousStateSpace(const TurretAxisModel & model)
{
  ContinuousStateSpace ss;
  if (model.type == TurretModelType::FirstOrder) {
    const double t = model.time_constant_sec;
    ss.a = DenseMatrix(1, 1, -1.0 / t);
    ss.b = {model.gain / t};
    ss.c = {1.0};
  } else {
    const double wn = model.natural_frequency_rad_per_sec;
    const double zeta = model.damping_ratio;
    ss.a = DenseMatrix(2, 2);
    ss.a(0, 1) = 1.0;
    ss.a(1, 0) = -wn * wn;
    ss.a(1, 1) = -2.0 * zeta * wn;
    ss.b = {0.0, model.gain * wn * wn};
    ss.c = {1.0, 0.0};
  }
  return ss;
}

/// e^{A tau} と ∫_0^tau e^{A s} ds B を拡大行列の指数関数でまとめて求める。
inline void zeroOrderHoldIntegrals(
  const ContinuousStateSpace & ss, double tau, DenseMatrix & phi, std::vector<double> & gamma)
{
  const std::size_t n = ss.a.rows;
  DenseMatrix augmented(n + 1, n + 1);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      augmented(i, j) = ss.a(i, j) * tau;
    }
    augmented(i, n) = ss.b[i] * tau;
  }
  const DenseMatrix e = matrixExponential(augmented);
  phi = DenseMatrix(n, n);
  gamma.assign(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      phi(i, j) = e(i, j);
    }
    gamma[i] = e(i, n);
  }
}

// ---------------------------------------------------------------------------
// 離散時間モデル（むだ時間を状態拡大で表現）
// ---------------------------------------------------------------------------

/// むだ時間込みの離散時間モデル z[k+1] = A z[k] + b u[k], y[k] = c^T z[k]。
///
/// 状態 z = [x; u[k-1]; u[k-2]; ...; u[k-m]]。x はプラント状態（1 or 2 次元）、
/// 後ろは過去の指令履歴（むだ時間ぶん）。むだ時間 L = d*Ts + theta（0 <= theta < Ts）の
/// 端数 theta も、1周期内で入力が切り替わる時刻として厳密に扱う（Astrom & Wittenmark の手法）:
///
///   x[k+1] = Phi(Ts) x[k] + Phi(Ts-theta) Gamma(theta) u[k-d-1] + Gamma(Ts-theta) u[k-d]
struct DiscreteTurretModel
{
  double sample_time_sec = 0.0;
  std::size_t plant_order = 0;      ///< x の次元
  std::size_t history_length = 0;   ///< 状態に含める過去指令の数 m
  std::size_t delay_steps = 0;      ///< むだ時間の整数部 d [周期]
  double fractional_delay_sec = 0.0;  ///< むだ時間の端数 theta [s]
  double gain = 1.0;                ///< 定常ゲイン（初期化に使う）
  DenseMatrix a;
  std::vector<double> b;
  std::vector<double> c;

  std::size_t stateDim() const {return a.rows;}

  /// 1周期進めた状態を返す（副作用なし）。
  std::vector<double> step(const std::vector<double> & z, double u) const
  {
    std::vector<double> next = multiply(a, z);
    for (std::size_t i = 0; i < next.size(); ++i) {
      next[i] += b[i] * u;
    }
    return next;
  }

  double output(const std::vector<double> & z) const {return dot(c, z);}

  /// 角度 output で静止し、過去の指令がすべて command だった状態を作る。
  ///
  /// 定常ゲインが 1 でない場合、output と gain*command の差は MPC の外乱推定が吸収する。
  std::vector<double> restState(double output_angle, double command) const
  {
    std::vector<double> z(stateDim(), 0.0);
    z[0] = output_angle;  // 1次・2次とも x[0] が角度、2次の x[1]（角速度）は 0
    for (std::size_t i = 0; i < history_length; ++i) {
      z[plant_order + i] = command;
    }
    return z;
  }
};

/// 連続時間モデルをサンプリング周期 sample_time_sec でむだ時間込みに離散化する。
inline bool discretizeTurretModel(
  const TurretAxisModel & model, double sample_time_sec, DiscreteTurretModel & out,
  std::string & reason)
{
  if (!validateTurretAxisModel(model, reason)) {
    return false;
  }
  if (!std::isfinite(sample_time_sec) || sample_time_sec <= 0.0) {
    reason = "sample_time_sec must be positive";
    return false;
  }

  const ContinuousStateSpace ss = toContinuousStateSpace(model);
  const std::size_t n = ss.a.rows;
  const double ts = sample_time_sec;

  // むだ時間を整数周期と端数へ分ける。浮動小数の誤差で端数が Ts ぎりぎりになる場合を丸める。
  constexpr double kEps = 1e-9;
  std::size_t d = static_cast<std::size_t>(std::floor(model.delay_sec / ts + kEps));
  double theta = model.delay_sec - static_cast<double>(d) * ts;
  if (theta < kEps) {
    theta = 0.0;
  }
  if (theta > ts - kEps) {
    ++d;
    theta = 0.0;
  }
  const std::size_t m = d + (theta > 0.0 ? 1 : 0);

  DenseMatrix phi_ts;
  std::vector<double> gamma_unused;
  zeroOrderHoldIntegrals(ss, ts, phi_ts, gamma_unused);

  // 現周期の後半 (Ts-theta) に効く指令 u[k-d] の係数
  DenseMatrix phi_rest;
  std::vector<double> b0;
  zeroOrderHoldIntegrals(ss, ts - theta, phi_rest, b0);
  // 現周期の前半 theta に効く指令 u[k-d-1] の係数 Phi(Ts-theta) Gamma(theta)
  std::vector<double> b1(n, 0.0);
  if (theta > 0.0) {
    DenseMatrix phi_theta;
    std::vector<double> gamma_theta;
    zeroOrderHoldIntegrals(ss, theta, phi_theta, gamma_theta);
    b1 = multiply(phi_rest, gamma_theta);
  }

  const std::size_t dim = n + m;
  out = DiscreteTurretModel{};
  out.sample_time_sec = ts;
  out.plant_order = n;
  out.history_length = m;
  out.delay_steps = d;
  out.fractional_delay_sec = theta;
  out.gain = model.gain;
  out.a = DenseMatrix(dim, dim);
  out.b.assign(dim, 0.0);
  out.c.assign(dim, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      out.a(i, j) = phi_ts(i, j);
    }
    out.c[i] = ss.c[i];
    // u[k-d]: d == 0 なら現在の入力、そうでなければ履歴の d-1 番目（履歴0番 = u[k-1]）
    if (d == 0) {
      out.b[i] += b0[i];
    } else {
      out.a(i, n + d - 1) += b0[i];
    }
    // u[k-d-1]: 履歴の d 番目
    if (theta > 0.0) {
      out.a(i, n + d) += b1[i];
    }
  }
  // 履歴のシフト: 新しい履歴0番 = u[k]、i 番 = 旧 i-1 番
  if (m > 0) {
    out.b[n] = 1.0;
    for (std::size_t i = 1; i < m; ++i) {
      out.a(n + i, n + i - 1) = 1.0;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// YAML 読み込み
// ---------------------------------------------------------------------------

/// `turret_axis_model:` 節を解釈する。同定スクリプトが出力する形式。
///
/// ```yaml
/// turret_axis_model:
///   model_type: second_order          # first_order | second_order
///   gain: 1.0
///   time_constant_sec: 0.05           # first_order のとき必須
///   natural_frequency_rad_per_sec: 25.0  # second_order のとき必須
///   damping_ratio: 0.8                # second_order のとき必須
///   delay_sec: 0.04
/// ```
/// それ以外のキー（適合度、取得日時など）は参照情報として無視する。
inline bool parseTurretAxisModel(
  const YAML::Node & root, TurretAxisModel & model, std::string & reason)
{
  try {
    const YAML::Node node = root["turret_axis_model"];
    if (!node || !node.IsMap()) {
      reason = "missing 'turret_axis_model' map";
      return false;
    }
    TurretAxisModel parsed;
    const auto type_node = node["model_type"];
    if (!type_node) {
      reason = "missing 'model_type'";
      return false;
    }
    const std::string type = type_node.as<std::string>();
    if (type == "first_order") {
      parsed.type = TurretModelType::FirstOrder;
    } else if (type == "second_order") {
      parsed.type = TurretModelType::SecondOrder;
    } else {
      reason = "unknown model_type '" + type + "' (expected first_order or second_order)";
      return false;
    }

    auto require = [&](const char * key, double & value) {
        const auto v = node[key];
        if (!v) {
          reason = std::string("missing '") + key + "'";
          return false;
        }
        value = v.as<double>();
        return true;
      };
    if (!require("gain", parsed.gain) || !require("delay_sec", parsed.delay_sec)) {
      return false;
    }
    if (parsed.type == TurretModelType::FirstOrder) {
      if (!require("time_constant_sec", parsed.time_constant_sec)) {
        return false;
      }
    } else {
      if (!require("natural_frequency_rad_per_sec", parsed.natural_frequency_rad_per_sec) ||
        !require("damping_ratio", parsed.damping_ratio))
      {
        return false;
      }
    }
    if (!validateTurretAxisModel(parsed, reason)) {
      return false;
    }
    model = parsed;
    return true;
  } catch (const YAML::Exception & e) {
    reason = std::string("YAML error: ") + e.what();
    return false;
  }
}

/// 同定結果の YAML ファイルを読み込む。失敗時は reason に理由を入れて false を返す。
inline bool loadTurretAxisModel(
  const std::string & path, TurretAxisModel & model, std::string & reason)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    reason = "failed to load '" + path + "': " + e.what();
    return false;
  }
  if (!parseTurretAxisModel(root, model, reason)) {
    reason = "'" + path + "': " + reason;
    return false;
  }
  return true;
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__TURRET_MODEL_HPP_
