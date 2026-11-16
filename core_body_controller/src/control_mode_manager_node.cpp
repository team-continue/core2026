// 手動と自動の運転モードを持ち、今のモードの速度指令だけを /control/cmd_vel へ通すノード。
//
// 入力:
//   /ui/auto/start（PoseStamped）   目標の位置と向き。受けたら自動に入り、目標を計画へ渡す
//   /ui/auto/stop（Bool, true）     自動をやめて手動に戻し、計画のゴールを消す
//   /control/manual/cmd_vel         手動の指令（操縦アプリ → ros2_ws_bridge）
//   /control/auto/cmd_vel           自動の指令（safety_gate）
//   /behavior/goal_reached（Bool）  到達。手動に戻す
//   /planning/navigation/status     走行の状態。失敗で手動に戻す
//   /system/emergency/hazard_status（Bool）  緊急停止。入ったら手動に戻し、計画のゴールを消す。
//                                   入っている間は auto を受け付けない
// 出力:
//   /control/cmd_vel                足回りへの指令
//   /behavior/goal_pose             計画へのゴール（停止では位置を NaN にしてゴールを消す）
//   /control/mode（String）         今のモード（"manual" / "auto"）

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "core_body_controller/control_mode.hpp"
#include "core_msgs/msg/navigation_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace core_body_controller
{
namespace
{
NavigationPhase toPhase(const std::uint8_t state)
{
  using core_msgs::msg::NavigationStatus;
  switch (state) {
    case NavigationStatus::STATE_DRIVING:
    case NavigationStatus::STATE_STUCK_WAIT:
    case NavigationStatus::STATE_BACKOFF:
    case NavigationStatus::STATE_GOAL_WAIT:
      return NavigationPhase::kActive;
    case NavigationStatus::STATE_FAILED:
      return NavigationPhase::kFailed;
    default:
      return NavigationPhase::kIdle;
  }
}
}  // namespace

class ControlModeManagerNode : public rclcpp::Node
{
public:
  ControlModeManagerNode()
  : Node("control_mode_manager")
  {
    const double mode_period_s = declare_parameter<double>("mode_publish_period_s", 0.5);
    if (!(mode_period_s > 0.0)) {
      throw std::invalid_argument("mode_publish_period_s は正の値にしてください");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel", rclcpp::QoS(10));
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/behavior/goal_pose", rclcpp::QoS(1).reliable().transient_local());
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/control/mode", rclcpp::QoS(1).reliable().transient_local());

    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/manual/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        forward(*msg, CommandSource::kManual);
      });
    auto_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/auto/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        forward(*msg, CommandSource::kAuto);
      });
    start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ui/auto/start", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {onStart(*msg);});
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/ui/auto/stop", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          onStop();
        }
      });
    reached_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/behavior/goal_reached", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          update([this]() {logic_.reached();}, "ゴールに着いた");
        }
      });
    navigation_sub_ = create_subscription<core_msgs::msg::NavigationStatus>(
      "/planning/navigation/status", rclcpp::QoS(10),
      [this](const core_msgs::msg::NavigationStatus::SharedPtr msg) {
        const auto phase = toPhase(msg->state);
        update([this, phase]() {logic_.navigation(phase);}, "走行に失敗した");
      });

    // emergency_handler は最後の状態を残して出す（transient_local）ので、後から起動しても今の状態を受ける
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/emergency/hazard_status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {onEmergency(msg->data);});

    // ブリッジや UI が後から繋いでも今のモードが分かるよう、決まった周期でも出す
    mode_timer_ = create_wall_timer(
      std::chrono::duration<double>(mode_period_s), [this]() {publishMode();});
    publishMode();
  }

private:
  void forward(const geometry_msgs::msg::Twist & cmd, const CommandSource source)
  {
    if (logic_.passes(source)) {
      cmd_pub_->publish(cmd);
    }
  }

  void onStart(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y)) {
      RCLCPP_WARN(get_logger(), "目標の位置が有限でないので、auto を受け付けません");
      return;
    }
    if (logic_.emergencyActive()) {
      RCLCPP_WARN(get_logger(), "緊急停止が入っているので、auto を受け付けません");
      return;
    }
    goal_pub_->publish(goal);
    update([this]() {logic_.start();}, "auto が押された");
  }

  void onStop()
  {
    cancelGoal();
    update([this]() {logic_.stop();}, "停止が押された");
  }

  void onEmergency(const bool active)
  {
    const bool was_auto = logic_.mode() == ControlMode::kAuto;
    update([this, active]() {logic_.emergency(active);}, "緊急停止が入った");
    // 解除したときに前の目標へ走り出さないよう、自動の途中なら計画のゴールも消す
    if (active && was_auto) {
      cancelGoal();
    }
  }

  // 計画のゴールを消す（global_planner は位置が有限でないゴールをゴールなしとして扱う）
  void cancelGoal()
  {
    geometry_msgs::msg::PoseStamped cancel;
    cancel.header.stamp = now();
    cancel.header.frame_id = "map";
    cancel.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    cancel.pose.position.y = std::numeric_limits<double>::quiet_NaN();
    cancel.pose.orientation.w = 1.0;
    goal_pub_->publish(cancel);
  }

  // モードを変える処理を行い、変わったら知らせる。切り替えのときは一度止める
  template<typename Change>
  void update(Change change, const char * reason)
  {
    const ControlMode before = logic_.mode();
    change();
    if (logic_.mode() == before) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "運転モード: %s → %s（%s）", toString(before).c_str(),
      toString(logic_.mode()).c_str(), reason);
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    publishMode();
  }

  void publishMode()
  {
    std_msgs::msg::String msg;
    msg.data = toString(logic_.mode());
    mode_pub_->publish(msg);
  }

  ControlModeLogic logic_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::Subscription<core_msgs::msg::NavigationStatus>::SharedPtr navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
};

}  // namespace core_body_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<core_body_controller::ControlModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
