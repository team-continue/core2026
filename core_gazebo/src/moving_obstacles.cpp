// シナリオファイルに従って、他ロボットと人間を動かす Gazebo プラグイン。
//
// 環境変数 CORE_SCENARIO_FILE で YAML を指定する。未指定なら何もしない。
// 動きが乱れないよう、衝突形状のない見た目だけのモデルにして、位置を時刻の関数で直接与える。
// LiDAR には写るが機体とは当たらないので、接触は確認スクリプトが位置から判定する。
//
// 位置: start と end の間を、速度が余弦波になるように往復する
//   s(t) = (1 - cos(w t + 2 pi phase)) / 2、w = 2 peak_speed / 距離
// core_nav_check.scenario に同じ式があるので、変えるときは両方を直す。
// 歩く人は start と end を離し、ほぼその場で揺れる人は 0.3m ほどの間を遅く往復させる。
//
// 観客（フィールドの外の人）を置くための項目:
//   z: 足元の高さ [m]（既定 0）。観客席の上の人に使う
//   spectator: true にすると観客として扱う（既定 false）。このプラグインの動きは変わらず、
//     run_scenario.py が接触と最小距離の計算から除く

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <yaml-cpp/yaml.h>

namespace core_gazebo
{
namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
// 10Hz の LiDAR に十分な間隔で位置を更新する
constexpr double kUpdatePeriodS = 0.01;

enum class ObstacleType {kRobot, kHuman};

struct ObstacleSpec
{
  std::string name;
  ObstacleType type{ObstacleType::kRobot};
  gz::math::Vector2d start;
  gz::math::Vector2d end;
  double peak_speed{0.0};
  double phase{0.0};
  // 足元の高さ
  double z{0.0};
};

std::string modelSdf(const ObstacleSpec & spec)
{
  const std::string geometry = spec.type == ObstacleType::kRobot ?
    "<box><size>0.8 0.8 0.8</size></box>" :
    "<cylinder><radius>0.25</radius><length>1.7</length></cylinder>";
  return "<?xml version='1.0'?><sdf version='1.9'><model name='" + spec.name + "'>"
         "<static>true</static><link name='link'><visual name='visual'><geometry>" + geometry +
         "</geometry></visual></link></model></sdf>";
}

double centerHeight(const ObstacleSpec & spec)
{
  return spec.z + (spec.type == ObstacleType::kRobot ? 0.4 : 0.85);
}

gz::math::Vector2d positionAt(const ObstacleSpec & spec, const double time_s)
{
  const gz::math::Vector2d offset = spec.end - spec.start;
  const double length = offset.Length();
  if (length < 1e-6 || spec.peak_speed <= 0.0) {
    return spec.start;
  }
  const double angular_rate = 2.0 * spec.peak_speed / length;
  const double ratio = 0.5 * (1.0 - std::cos(angular_rate * time_s + kTwoPi * spec.phase));
  return spec.start + offset * ratio;
}

gz::math::Vector2d readPoint(const YAML::Node & node)
{
  return gz::math::Vector2d(node[0].as<double>(), node[1].as<double>());
}

std::vector<ObstacleSpec> loadScenario(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  std::vector<ObstacleSpec> specs;
  for (const auto & item : root["obstacles"]) {
    ObstacleSpec spec;
    spec.name = item["name"].as<std::string>();
    const auto type = item["type"].as<std::string>();
    if (type != "robot" && type != "human") {
      throw YAML::Exception(item["type"].Mark(), "type は robot か human を指定してください: " + type);
    }
    spec.type = type == "robot" ? ObstacleType::kRobot : ObstacleType::kHuman;
    spec.start = readPoint(item["start"]);
    spec.end = item["end"] ? readPoint(item["end"]) : spec.start;
    spec.peak_speed = item["peak_speed"] ? item["peak_speed"].as<double>() : 0.0;
    spec.phase = item["phase"] ? item["phase"].as<double>() : 0.0;
    spec.z = item["z"] ? item["z"].as<double>() : 0.0;
    if (!std::isfinite(spec.z)) {
      throw YAML::Exception(item["z"].Mark(), spec.name + " の z が有限の値ではありません");
    }
    // 値の形だけを確かめる。観客かどうかで動きは変えない
    if (item["spectator"]) {
      (void)item["spectator"].as<bool>();
    }
    specs.push_back(spec);
  }
  return specs;
}
}  // namespace

class MovingObstacles final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & world, const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & event_manager) override
  {
    const char * path = std::getenv("CORE_SCENARIO_FILE");
    if (path == nullptr || std::string(path).empty()) {
      return;
    }
    try {
      specs_ = loadScenario(path);
    } catch (const YAML::Exception & error) {
      gzerr << "MovingObstacles: シナリオ [" << path << "] を読めません: " << error.what() << "\n";
      return;
    }
    gz::sim::SdfEntityCreator creator(ecm, event_manager);
    for (const auto & spec : specs_) {
      sdf::Root root;
      const auto errors = root.LoadSdfString(modelSdf(spec));
      if (!errors.empty() || root.Model() == nullptr) {
        gzerr << "MovingObstacles: モデル [" << spec.name << "] を作れません。\n";
        continue;
      }
      sdf::Model model = *root.Model();
      const auto position = positionAt(spec, 0.0);
      model.SetRawPose(gz::math::Pose3d(position.X(), position.Y(), centerHeight(spec), 0, 0, 0));
      const gz::sim::Entity entity = creator.CreateEntities(&model);
      creator.SetParent(entity, world);
      models_.emplace_back(entity);
      active_specs_.push_back(spec);
    }
    gzmsg << "MovingObstacles: " << active_specs_.size() << " 体を [" << path << "] から作りました。\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || models_.empty()) {
      return;
    }
    const double time_s = std::chrono::duration<double>(info.simTime).count();
    if (last_update_s_.has_value() && time_s - *last_update_s_ < kUpdatePeriodS - 1e-9) {
      return;
    }
    last_update_s_ = time_s;
    for (std::size_t i = 0; i < models_.size(); ++i) {
      const auto position = positionAt(active_specs_[i], time_s);
      models_[i].SetWorldPoseCmd(
        ecm,
        gz::math::Pose3d(position.X(), position.Y(), centerHeight(active_specs_[i]), 0, 0, 0));
    }
  }

private:
  std::vector<ObstacleSpec> specs_;
  std::vector<ObstacleSpec> active_specs_;
  std::vector<gz::sim::Model> models_;
  std::optional<double> last_update_s_;
};
}  // namespace core_gazebo

GZ_ADD_PLUGIN(
  core_gazebo::MovingObstacles,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
