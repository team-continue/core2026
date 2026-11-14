// パラメータの違いによる接触の頻度の差を、Gazebo を使わずに統計で比べるコマンド。
//
// 実機と同じ設定ファイルを読み、--case で指定した値だけを変えて、同じ乱数の列で比べる。
// モデルの範囲と Gazebo との差は crowd_model.hpp に書いた。

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "core_planning/crowd_model.hpp"

namespace
{
namespace crowd_model = core_planning::crowd_model;
namespace fs = std::filesystem;

constexpr int kDefaultRuns = 240;
constexpr const char * kUsage =
  "使い方: ros2 run core_planning crowd_stats [オプション]\n"
  "  --scenario FILE    シナリオ（既定は core_gazebo の crowd_goals.yaml）\n"
  "  --config-dir DIR   パラメータファイルの場所（既定は core_launch の config）\n"
  "  --runs N           条件ごとの実行回数（既定 240）\n"
  "  --jobs N           同時に動かすスレッドの数（既定は CPU の数）\n"
  "  --case A=1,B=2     既定と比べる条件。何度でも指定できる\n"
  "\n"
  "例: crowd_stats --case speed_limit.density_speed_scale=0.7 --case passing_rule.enabled=false\n"
  "    追跡の近似も変えられる: --case model.velocity_lag_s=0.01,model.velocity_noise_mps=0\n";

// モデルが読むパラメータファイル
const std::vector<std::string> kConfigFiles{
  "mppi.yaml", "passing_rule.yaml", "speed_limit.yaml", "stuck_recovery.yaml", "footprint.yaml",
  "safety_stop.yaml", "field.yaml", "planner.yaml",
};

struct Arguments
{
  bool help{false};
  std::string scenario;
  std::string config_dir;
  int runs{kDefaultRuns};
  int jobs{0};
  std::vector<std::string> cases;
};

int toPositiveInt(const std::string & option, const std::string & text)
{
  std::size_t used = 0U;
  int value = 0;
  try {
    value = std::stoi(text, &used);
  } catch (const std::logic_error &) {
    throw std::invalid_argument(option + " は正の整数で指定してください: " + text);
  }
  if (used != text.size() || value <= 0) {
    throw std::invalid_argument(option + " は正の整数で指定してください: " + text);
  }
  return value;
}

Arguments parseArguments(const std::vector<std::string> & tokens)
{
  Arguments arguments;
  for (std::size_t i = 0; i < tokens.size(); ++i) {
    const std::string & token = tokens[i];
    if (token == "--help" || token == "-h") {
      arguments.help = true;
      continue;
    }
    if (i + 1U >= tokens.size()) {
      throw std::invalid_argument(token + " の値がありません");
    }
    const std::string & value = tokens[++i];
    if (token == "--scenario") {
      arguments.scenario = value;
    } else if (token == "--config-dir") {
      arguments.config_dir = value;
    } else if (token == "--runs") {
      arguments.runs = toPositiveInt(token, value);
    } else if (token == "--jobs") {
      arguments.jobs = toPositiveInt(token, value);
    } else if (token == "--case") {
      arguments.cases.push_back(value);
    } else {
      throw std::invalid_argument("知らないオプションです: " + token);
    }
  }
  return arguments;
}

std::string packageShare(const std::string & package)
{
  try {
    return ament_index_cpp::get_package_share_directory(package);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    throw std::runtime_error(
            "パッケージ " + package + " が見つかりません。install/setup.bash を読み込むか、"
            "--scenario と --config-dir を指定してください");
  }
}

// 既定の設定を読む。モデルで使わなかった名前は unused に入れる
crowd_model::StackParams loadConfig(const fs::path & directory, std::vector<std::string> & unused)
{
  crowd_model::StackParams params;
  for (const auto & name : kConfigFiles) {
    const fs::path path = directory / name;
    if (!fs::exists(path)) {
      throw std::runtime_error("パラメータファイルがありません: " + path.string());
    }
    const auto names = crowd_model::applyParameterFile(params, YAML::LoadFile(path.string()));
    unused.insert(unused.end(), names.begin(), names.end());
  }
  return params;
}

// "A=1,B=2" の値を設定する
crowd_model::StackParams applyCase(crowd_model::StackParams params, const std::string & text)
{
  std::size_t begin = 0U;
  while (begin <= text.size()) {
    const std::size_t comma = std::min(text.find(',', begin), text.size());
    const std::string item = text.substr(begin, comma - begin);
    const std::size_t equal = item.find('=');
    if (equal == std::string::npos || equal == 0U || equal + 1U == item.size()) {
      throw std::invalid_argument("--case は 名前=値 の形で指定してください: " + item);
    }
    const std::string name = item.substr(0U, equal);
    if (!crowd_model::applyParameter(params, name, item.substr(equal + 1U))) {
      throw std::invalid_argument("知らないパラメータです: " + name);
    }
    begin = comma + 1U;
  }
  return params;
}

std::vector<crowd_model::RunResult> runAll(
  const crowd_model::Scenario & scenario, const crowd_model::StackParams & params,
  const core_planning::DistanceField & field, const int runs, const int jobs)
{
  std::vector<crowd_model::RunResult> results(static_cast<std::size_t>(runs));
  std::atomic<int> next{0};
  const auto worker = [&]() {
      for (int i = next++; i < runs; i = next++) {
        results[static_cast<std::size_t>(i)] =
          crowd_model::simulateRun(scenario, params, field, static_cast<std::uint32_t>(i));
      }
    };
  std::vector<std::future<void>> workers;
  workers.reserve(static_cast<std::size_t>(jobs));
  for (int i = 0; i < jobs; ++i) {
    workers.push_back(std::async(std::launch::async, worker));
  }
  // スレッドで起きた例外はここで投げ直される
  for (auto & running : workers) {
    running.get();
  }
  return results;
}

void printRow(const std::string & name, const crowd_model::Summary & summary)
{
  const std::string rule_side = summary.passes_total > 0 ?
    std::to_string(100 * summary.passes_on_rule_side / summary.passes_total) + "%（" +
    std::to_string(summary.passes_total) + "回）" : std::string{"なし"};
  std::printf(
    "%s | %.2f±%.2f | %.2f | %.0f%% | %.0f%% | %.2f/%d | %.1f | %.2f | %.1f | %.2f | %d | %s | "
    "%.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f\n",
    name.c_str(), summary.contacts_per_100s, summary.contacts_per_100s_margin,
    summary.own_fault_per_100s, 100.0 * summary.contact_free_ratio,
    100.0 * summary.deep_contact_ratio, summary.goals_reached_mean, summary.goals_total,
    summary.mean_duration_s, summary.mean_speed_mps, summary.mean_recovery_s,
    summary.min_fence_clearance_m, summary.fence_touch_runs, rule_side.c_str(),
    summary.mean_impact_speed_mps, summary.hard_contacts_per_100s, summary.yields_per_100s,
    summary.mitigations_per_100s, summary.human_contacts_per_100s,
    summary.human_own_fault_per_100s, summary.human_region_per_100s,
    summary.human_region_max_speed_mps);
  std::fflush(stdout);
}

void evaluate(
  const std::string & name, const crowd_model::Scenario & scenario,
  const crowd_model::StackParams & params, const int runs, const int jobs)
{
  const auto grid = core_planning::makeFieldGrid(params.field, params.grid_resolution_m);
  const auto field = core_planning::computeDistanceField(grid);
  printRow(name, crowd_model::summarize(runAll(scenario, params, field, runs, jobs)));
}

int run(const Arguments & arguments)
{
  const fs::path scenario_path = arguments.scenario.empty() ?
    fs::path(packageShare("core_gazebo")) / "scenarios" / "crowd_goals.yaml" :
    fs::path(arguments.scenario);
  const fs::path config_dir = arguments.config_dir.empty() ?
    fs::path(packageShare("core_launch")) / "config" : fs::path(arguments.config_dir);
  const int jobs = arguments.jobs > 0 ? arguments.jobs :
    std::max(static_cast<int>(std::thread::hardware_concurrency()), 1);

  const auto scenario = crowd_model::parseScenario(YAML::LoadFile(scenario_path.string()));
  std::vector<std::string> unused;
  const auto base = loadConfig(config_dir, unused);
  // 先に全部の条件を読み、書き間違いで途中まで走ってから止まらないようにする
  std::vector<crowd_model::StackParams> cases;
  cases.reserve(arguments.cases.size());
  for (const auto & text : arguments.cases) {
    cases.push_back(applyCase(base, text));
  }

  std::printf(
    "シナリオ: %s（相手 %zu、ゴール %zu）、条件ごとに %d 回\n",
    scenario_path.filename().c_str(), scenario.lanes.size(), scenario.goals.size(),
    arguments.runs);
  std::printf("モデルで使わない設定:");
  for (const auto & name : unused) {
    std::printf(" %s", name.c_str());
  }
  std::printf("\n");
  std::printf(
    "条件 | 接触/100秒 | 自機原因/100秒 | 無接触 | 深い接触 | 到達 | 周回[秒] | 平均[m/s] | "
    "復帰[秒] | 柵最小[m] | 柵接触の回 | 規則の側 | 当たる速さ[m/s] | 1.5m/s以上/100秒 | "
    "停止線/100秒 | 緩和/100秒 | 人と接触/100秒 | うち自機原因 | 人の1m内[秒/100秒] | "
    "人の1m内の最高[m/s]\n");
  evaluate("既定", scenario, base, arguments.runs, jobs);
  for (std::size_t i = 0; i < cases.size(); ++i) {
    evaluate(arguments.cases[i], scenario, cases[i], arguments.runs, jobs);
  }
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments arguments = parseArguments(std::vector<std::string>(argv + 1, argv + argc));
    if (arguments.help) {
      std::printf("%s", kUsage);
      return 0;
    }
    return run(arguments);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "[NG] %s\n%s", error.what(), kUsage);
    return 1;
  }
}
