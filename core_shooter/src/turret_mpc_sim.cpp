// 砲塔 MPC のオフライン・シミュレータ（ROS 非依存の CLI ツール）。
//
// 同定モデル（turret_axis_model YAML）と MPC 設定（turret_mpc YAML）を読み、
// 「同定モデルを真値とする仮想プラント」に対して MPC と直接指令（従来方式）を走らせ、
// 追従誤差などの指標を表で出す。実機に触れずに予測ホライズン・重み・制約を調整するためのもの。
//
// 仮想プラントのゲイン・時定数・むだ時間をずらしたり、モータ内部の速度制限を模擬したりして、
// モデル誤差に対する頑健性も確認できる。使い方は --help を参照。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "core_shooter/turret_model.hpp"
#include "core_shooter/turret_mpc.hpp"

namespace
{

using core_shooter::DiscreteTurretModel;
using core_shooter::TurretAxisModel;
using core_shooter::TurretModelType;
using core_shooter::TurretMpc;
using core_shooter::TurretMpcConfig;

struct Options
{
  std::string model_path;
  std::string mpc_path;
  std::string plant_path;            ///< 空なら model と同じ
  std::string scenario = "all";      ///< step | ramp | sine | all
  std::string preview = "constant_velocity";  ///< constant_velocity | exact
  std::string csv_path;
  double amplitude = 0.3;            ///< [rad]
  double rate = 1.0;                 ///< ramp の角速度 [rad/s]
  double frequency = 1.0;            ///< sine の周波数 [Hz]
  double duration = 4.0;             ///< [s]
  double command_min = -M_PI;
  double command_max = M_PI;
  double plant_gain_scale = 1.0;
  double plant_lag_scale = 1.0;      ///< 時定数（1次）/ 1/wn（2次）の倍率
  double plant_delay_offset = 0.0;   ///< [s]
  double plant_rate_limit = 0.0;     ///< モータ内部の速度制限の模擬 [rad/s]。0 以下で無効
  double noise_std = 0.0;            ///< 実測角の白色ノイズ [rad]
  int substeps = 20;
};

void printUsage()
{
  std::printf(
    "Usage: turret_mpc_sim --model <turret_axis_model.yaml> --mpc <turret_mpc.yaml> [options]\n"
    "\n"
    "Scenario options:\n"
    "  --scenario step|ramp|sine|all   (default all)\n"
    "  --amplitude <rad>      step height / ramp & sine amplitude (default 0.3)\n"
    "  --rate <rad/s>         ramp (triangle) speed (default 1.0)\n"
    "  --frequency <Hz>       sine frequency (default 1.0)\n"
    "  --duration <s>         simulated time per scenario (default 4.0)\n"
    "  --preview constant_velocity|exact\n"
    "                         reference given to MPC: extrapolate current angle & rate\n"
    "                         (like aim_bot would) or the exact future (default constant_velocity)\n"
    "  --command-min/--command-max <rad>  command range (default -pi/pi)\n"
    "\n"
    "Plant perturbation (robustness check):\n"
    "  --plant <yaml>         use another model as the true plant (default: --model)\n"
    "  --plant-gain-scale <x> --plant-lag-scale <x> --plant-delay-offset <s>\n"
    "  --plant-rate-limit <rad/s>  emulate the motor's internal velocity limit\n"
    "  --noise-std <rad>      measurement noise\n"
    "\n"
    "Output:\n"
    "  --csv <path>           write time series (scenario,controller,t,reference,command,angle)\n");
}

bool parseOptions(int argc, char ** argv, Options & o)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](std::string & out) {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "missing value for %s\n", arg.c_str());
          return false;
        }
        out = argv[++i];
        return true;
      };
    auto next_double = [&](double & out) {
        std::string s;
        if (!next(s)) {
          return false;
        }
        char * end = nullptr;
        out = std::strtod(s.c_str(), &end);
        if (end == s.c_str() || *end != '\0' || !std::isfinite(out)) {
          std::fprintf(stderr, "invalid number for %s: %s\n", arg.c_str(), s.c_str());
          return false;
        }
        return true;
      };
    bool ok = true;
    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else if (arg == "--model") {
      ok = next(o.model_path);
    } else if (arg == "--mpc") {
      ok = next(o.mpc_path);
    } else if (arg == "--plant") {
      ok = next(o.plant_path);
    } else if (arg == "--scenario") {
      ok = next(o.scenario);
    } else if (arg == "--preview") {
      ok = next(o.preview);
    } else if (arg == "--csv") {
      ok = next(o.csv_path);
    } else if (arg == "--amplitude") {
      ok = next_double(o.amplitude);
    } else if (arg == "--rate") {
      ok = next_double(o.rate);
    } else if (arg == "--frequency") {
      ok = next_double(o.frequency);
    } else if (arg == "--duration") {
      ok = next_double(o.duration);
    } else if (arg == "--command-min") {
      ok = next_double(o.command_min);
    } else if (arg == "--command-max") {
      ok = next_double(o.command_max);
    } else if (arg == "--plant-gain-scale") {
      ok = next_double(o.plant_gain_scale);
    } else if (arg == "--plant-lag-scale") {
      ok = next_double(o.plant_lag_scale);
    } else if (arg == "--plant-delay-offset") {
      ok = next_double(o.plant_delay_offset);
    } else if (arg == "--plant-rate-limit") {
      ok = next_double(o.plant_rate_limit);
    } else if (arg == "--noise-std") {
      ok = next_double(o.noise_std);
    } else {
      std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
      ok = false;
    }
    if (!ok) {
      return false;
    }
  }
  if (o.model_path.empty() || o.mpc_path.empty()) {
    std::fprintf(stderr, "--model and --mpc are required\n");
    return false;
  }
  if (o.scenario != "step" && o.scenario != "ramp" && o.scenario != "sine" &&
    o.scenario != "all")
  {
    std::fprintf(stderr, "unknown scenario: %s\n", o.scenario.c_str());
    return false;
  }
  if (o.preview != "constant_velocity" && o.preview != "exact") {
    std::fprintf(stderr, "unknown preview: %s\n", o.preview.c_str());
    return false;
  }
  return true;
}

/// 目標角の時間関数（角度と角速度）。
struct Scenario
{
  std::string name;
  std::function<double(double)> angle;
  std::function<double(double)> rate;
  double settle_skip_sec;            ///< 指標の集計から除く立ち上がり時間
};

std::vector<Scenario> makeScenarios(const Options & o)
{
  std::vector<Scenario> list;
  const double t0 = 0.2;   // 最初は静止状態から始める
  if (o.scenario == "step" || o.scenario == "all") {
    const double a = o.amplitude;
    list.push_back(
      {"step",
        [a, t0](double t) {return t >= t0 ? a : 0.0;},
        [](double) {return 0.0;},
        0.0});
  }
  if (o.scenario == "ramp" || o.scenario == "all") {
    // ±amplitude を rate で往復する三角波（区間内は等速）
    const double a = std::max(o.amplitude, 1e-6);
    const double v = std::max(std::fabs(o.rate), 1e-6);
    const double period = 4.0 * a / v;
    auto tri = [a, v, period, t0](double t) {
        if (t < t0) {
          return 0.0;
        }
        const double ph = std::fmod(t - t0, period);
        if (ph < a / v) {
          return v * ph;
        }
        if (ph < 3.0 * a / v) {
          return a - v * (ph - a / v);
        }
        return -a + v * (ph - 3.0 * a / v);
      };
    auto tri_rate = [a, v, period, t0](double t) {
        if (t < t0) {
          return 0.0;
        }
        const double ph = std::fmod(t - t0, period);
        return (ph < a / v || ph >= 3.0 * a / v) ? v : -v;
      };
    list.push_back({"ramp", tri, tri_rate, t0 + 0.5});
  }
  if (o.scenario == "sine" || o.scenario == "all") {
    const double a = o.amplitude;
    const double w = 2.0 * M_PI * o.frequency;
    list.push_back(
      {"sine",
        [a, w, t0](double t) {return t >= t0 ? a * std::sin(w * (t - t0)) : 0.0;},
        [a, w, t0](double t) {return t >= t0 ? a * w * std::cos(w * (t - t0)) : 0.0;},
        t0 + 0.5});
  }
  return list;
}

/// 細かい刻みで回す仮想プラント（モータ内部の速度制限の模擬つき）。
class Plant
{
public:
  bool configure(
    const TurretAxisModel & model, double fine_dt, double rate_limit,
    std::string & reason)
  {
    dt_ = fine_dt;
    rate_limit_ = rate_limit;
    return core_shooter::discretizeTurretModel(model, fine_dt, fine_, reason);
  }
  void reset(double angle) {z_ = fine_.restState(angle, angle); limited_ = angle;}
  void advance(double command, int substeps)
  {
    for (int i = 0; i < substeps; ++i) {
      if (rate_limit_ > 0.0) {
        const double step = rate_limit_ * dt_;
        limited_ += std::clamp(command - limited_, -step, step);
      } else {
        limited_ = command;
      }
      z_ = fine_.step(z_, limited_);
    }
  }
  double output() const {return fine_.output(z_);}

private:
  DiscreteTurretModel fine_;
  std::vector<double> z_;
  double dt_ = 0.001;
  double rate_limit_ = 0.0;
  double limited_ = 0.0;
};

struct Metrics
{
  double rmse = 0.0;
  double max_abs_error = 0.0;
  double overshoot = 0.0;           ///< step のみ [rad]
  double settling_time = -1.0;      ///< step のみ [s]（2% 帯）。-1 は未整定
  double max_command_step = 0.0;    ///< 1周期あたりの最大指令変化 [rad]
  double mean_compute_us = 0.0;
  double max_compute_us = 0.0;
  int constrained_cycles = 0;
  int unconverged_cycles = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parseOptions(argc, argv, opt)) {
    printUsage();
    return 2;
  }

  std::string reason;
  TurretAxisModel model;
  if (!core_shooter::loadTurretAxisModel(opt.model_path, model, reason)) {
    std::fprintf(stderr, "model load failed: %s\n", reason.c_str());
    return 1;
  }
  TurretAxisModel plant_model = model;
  if (!opt.plant_path.empty() &&
    !core_shooter::loadTurretAxisModel(opt.plant_path, plant_model, reason))
  {
    std::fprintf(stderr, "plant model load failed: %s\n", reason.c_str());
    return 1;
  }
  plant_model.gain *= opt.plant_gain_scale;
  plant_model.time_constant_sec *= opt.plant_lag_scale;
  plant_model.natural_frequency_rad_per_sec /= opt.plant_lag_scale;
  plant_model.delay_sec = std::max(0.0, plant_model.delay_sec + opt.plant_delay_offset);

  TurretMpcConfig config;
  config.command_min = opt.command_min;
  config.command_max = opt.command_max;
  if (!core_shooter::loadTurretMpcConfig(opt.mpc_path, config, reason)) {
    std::fprintf(stderr, "mpc config load failed: %s\n", reason.c_str());
    return 1;
  }
  TurretMpc probe;
  if (!probe.configure(model, config, reason)) {
    std::fprintf(stderr, "mpc configure failed: %s\n", reason.c_str());
    return 1;
  }
  const double ts = config.sample_time_sec;
  const auto & dm = probe.discreteModel();
  std::printf(
    "control period %.4f s (%.1f Hz), horizon %d steps (%.3f s), blocks %zu, "
    "model delay = %zu steps + %.4f s\n",
    ts, 1.0 / ts, config.horizon_steps, config.horizon_steps * ts,
    config.block_lengths.size(), dm.delay_steps, dm.fractional_delay_sec);
  std::printf(
    "plant: gain x%.3f, lag x%.3f, delay %+.4f s, rate limit %s, noise %.4f rad\n\n",
    opt.plant_gain_scale, opt.plant_lag_scale, opt.plant_delay_offset,
    opt.plant_rate_limit > 0.0 ? (std::to_string(opt.plant_rate_limit) + " rad/s").c_str() :
    "none", opt.noise_std);

  std::ofstream csv;
  if (!opt.csv_path.empty()) {
    csv.open(opt.csv_path);
    if (!csv) {
      std::fprintf(stderr, "cannot open csv: %s\n", opt.csv_path.c_str());
      return 1;
    }
    csv << "scenario,controller,t,reference,command,angle\n";
  }

  std::printf(
    "%-6s %-7s %9s %9s %9s %9s %9s %8s %8s %6s\n", "scen", "ctrl", "rmse", "max_err",
    "overshoot", "settle_s", "max_du", "mean_us", "max_us", "cons%");
  const int steps = static_cast<int>(std::ceil(opt.duration / ts));
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, std::max(opt.noise_std, 0.0));

  for (const Scenario & sc : makeScenarios(opt)) {
    for (const std::string controller : {"mpc", "direct"}) {
      Plant plant;
      if (!plant.configure(plant_model, ts / opt.substeps, opt.plant_rate_limit, reason)) {
        std::fprintf(stderr, "plant configure failed: %s\n", reason.c_str());
        return 1;
      }
      plant.reset(0.0);
      TurretMpc mpc;
      mpc.configure(model, config, reason);

      Metrics m;
      double sum_sq = 0.0;
      int count = 0;
      double prev_u = 0.0;
      double compute_total = 0.0;
      double last_outside = 0.0;
      for (int k = 0; k < steps; ++k) {
        const double t = k * ts;
        const double y_meas = plant.output() + (opt.noise_std > 0.0 ? noise(rng) : 0.0);
        const double r_now = sc.angle(t);
        double u = r_now;
        if (controller == "mpc") {
          std::vector<double> ref;
          if (opt.preview == "exact") {
            for (int j = 1; j <= config.horizon_steps; ++j) {
              ref.push_back(sc.angle(t + j * ts));
            }
          } else {
            ref = core_shooter::makeConstantVelocityReference(
              r_now, sc.rate(t), config.horizon_steps, ts);
          }
          const auto start = std::chrono::steady_clock::now();
          const auto sol = mpc.compute(y_meas, ref);
          const double us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - start).count();
          compute_total += us;
          m.max_compute_us = std::max(m.max_compute_us, us);
          m.constrained_cycles += sol.constrained ? 1 : 0;
          m.unconverged_cycles += sol.converged ? 0 : 1;
          u = sol.command;
          mpc.commit(u);
        } else {
          u = std::clamp(r_now, config.command_min, config.command_max);
          if (std::isfinite(config.max_command_step)) {
            u = std::clamp(u, prev_u - config.max_command_step, prev_u + config.max_command_step);
          }
        }
        m.max_command_step = std::max(m.max_command_step, std::fabs(u - prev_u));
        prev_u = u;
        plant.advance(u, opt.substeps);

        // 周期終了時刻の真の角度で誤差を評価する
        const double t_next = t + ts;
        const double y = plant.output();
        const double r = sc.angle(t_next);
        const double err = y - r;
        if (csv) {
          csv << sc.name << ',' << controller << ',' << t_next << ',' << r << ',' << u << ',' <<
            y << '\n';
        }
        if (t_next >= sc.settle_skip_sec) {
          sum_sq += err * err;
          ++count;
          m.max_abs_error = std::max(m.max_abs_error, std::fabs(err));
        }
        if (sc.name == "step" && t_next >= 0.2) {
          m.overshoot = std::max(m.overshoot, (y - opt.amplitude) * (opt.amplitude >= 0 ? 1 : -1));
          if (std::fabs(err) > 0.02 * std::fabs(opt.amplitude)) {
            last_outside = t_next;
          }
        }
      }
      m.rmse = count > 0 ? std::sqrt(sum_sq / count) : 0.0;
      m.mean_compute_us = controller == "mpc" ? compute_total / steps : 0.0;
      if (sc.name == "step") {
        m.settling_time = last_outside >= opt.duration - ts ? -1.0 : last_outside - 0.2;
      }
      std::printf(
        "%-6s %-7s %9.5f %9.5f %9s %9s %9.4f %8.1f %8.1f %6.1f\n", sc.name.c_str(),
        controller.c_str(), m.rmse, m.max_abs_error,
        sc.name == "step" ? std::to_string(m.overshoot).substr(0, 8).c_str() : "-",
        sc.name == "step" ?
        (m.settling_time < 0 ? "unsettled" : std::to_string(m.settling_time).substr(0, 6)).c_str() :
        "-",
        m.max_command_step, m.mean_compute_us, m.max_compute_us,
        100.0 * m.constrained_cycles / steps);
      if (m.unconverged_cycles > 0) {
        std::printf(
          "  warning: ADMM did not converge in %d cycles (increase admm.max_iterations)\n",
          m.unconverged_cycles);
      }
    }
  }
  return 0;
}
