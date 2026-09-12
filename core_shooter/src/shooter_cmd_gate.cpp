#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

#include "core_shooter/parameter_utils.hpp"

namespace
{
/// shoot_cmd の特殊値。正の値は「その回数だけ発射」を意味する。
constexpr int kShootCmdStop = 0;
constexpr int kShootCmdFullauto = -1;
constexpr int kShootCmdOnce = 1;
}  // namespace

class ShooterCmdGate : public rclcpp::Node
{
public:
  ShooterCmdGate()
  : Node("shooter_cmd_gate")
  {
    //========================================
    // parameters
    //========================================
    burst_count_ = core_shooter::declareAndGet<int>(*this, "burst_count", 3);
    shoot_motor_on_command_ =
      core_shooter::declareAndGet<double>(*this, "shoot_motor_on_command", 2000.0);
    manual_mode_target_side_ =
      core_shooter::declareAndGet<std::string>(*this, "manual_mode_target_side", "right");

    if (burst_count_ <= 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid parameter burst_count=%d (must be > 0)", burst_count_);
      throw std::runtime_error("invalid burst_count");
    }
    if (shoot_motor_on_command_ < 0.0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid parameter shoot_motor_on_command=%f (must be >= 0)",
        shoot_motor_on_command_);
      throw std::runtime_error("invalid shoot_motor_on_command");
    }
    if (manual_mode_target_side_ != "left" && manual_mode_target_side_ != "right") {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid parameter manual_mode_target_side='%s' (must be 'left' or 'right')",
        manual_mode_target_side_.c_str());
      throw std::runtime_error("invalid manual_mode_target_side");
    }

    //========================================
    // 左右系統（トピック名は従来のフラット名のまま。launch で各 namespace へ remap する）
    //========================================
    setupSide(left_, "left", "Left");
    setupSide(right_, "right", "Right");

    //========================================
    // subscribers ui (左右共通入力)
    //========================================
    manual_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "manual_mode", 1,
      std::bind(&ShooterCmdGate::manualModeCallback, this, std::placeholders::_1));
    manual_pitch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "manual_pitch", 1,
      std::bind(&ShooterCmdGate::manualPitchCallback, this, std::placeholders::_1));
    shoot_motor_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_motor_state", 1,
      std::bind(&ShooterCmdGate::shootMotorStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "ShooterCmdGate manual route: /manual_mode -> /%s/manual_mode, /manual_pitch -> /%s/manual_pitch_angle",
      manual_mode_target_side_.c_str(), manual_mode_target_side_.c_str());
  }

private:
  /// 片側系統の入出力をまとめて保持する。
  struct SideChannel
  {
    std::string name;          // "left" / "right"
    std::string display_name;  // ログ表示用 "Left" / "Right"

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr once_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr burst_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fullauto_sub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr manual_pitch_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoot_motor_pub;

    bool fullauto_enabled = false;
    bool fullauto_input_prev = false;
  };

  void setupSide(SideChannel & side, const std::string & name, const std::string & display_name)
  {
    side.name = name;
    side.display_name = display_name;

    //========================================
    // subscribers ui
    //========================================
    side.once_sub = this->create_subscription<std_msgs::msg::Bool>(
      name + "/shoot_once", 1,
      [this, &side](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          publishCmd(side, kShootCmdOnce);
          RCLCPP_INFO(this->get_logger(), "On trigger: %s Once", side.display_name.c_str());
        }
      });
    side.burst_sub = this->create_subscription<std_msgs::msg::Bool>(
      name + "/shoot_burst", 1,
      [this, &side](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          publishCmd(side, burst_count_);
          RCLCPP_INFO(
            this->get_logger(), "On trigger: %s Burst (%d)", side.display_name.c_str(),
            burst_count_);
        }
      });
    side.fullauto_sub = this->create_subscription<std_msgs::msg::Bool>(
      name + "/shoot_fullauto", 1,
      [this, &side](const std_msgs::msg::Bool::SharedPtr msg) {
        processFullautoInput(side, msg->data);
      });

    //========================================
    // publishers
    //========================================
    side.cmd_pub = this->create_publisher<std_msgs::msg::Int32>(name + "_shoot_cmd", 10);
    side.manual_mode_pub = this->create_publisher<std_msgs::msg::Bool>(name + "_manual_mode", 10);
    side.manual_pitch_pub =
      this->create_publisher<std_msgs::msg::Float32>(name + "_manual_pitch_angle", 10);
    side.shoot_motor_pub =
      this->create_publisher<std_msgs::msg::Float32>("/" + name + "/shoot_motor", 10);
  }

  //========================================
  // ui callbacks
  //========================================
  void manualModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // 選択されていない側は明示的に false を publish して片側だけ有効化する。
    publishBool(left_.manual_mode_pub, isManualTarget(left_) && msg->data);
    publishBool(right_.manual_mode_pub, isManualTarget(right_) && msg->data);
  }

  void manualPitchCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    SideChannel & target = isManualTarget(right_) ? right_ : left_;
    target.manual_pitch_pub->publish(*msg);
  }

  void shootMotorStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const float command = msg->data ? static_cast<float>(shoot_motor_on_command_) : 0.0F;
    publishFloat(left_.shoot_motor_pub, command);
    publishFloat(right_.shoot_motor_pub, command);
  }

  /// フルオートは入力レベルではなく立上り/立下りで開始・停止コマンドを生成する。
  void processFullautoInput(SideChannel & side, bool input)
  {
    const bool rising = input && !side.fullauto_input_prev;
    const bool falling = !input && side.fullauto_input_prev;
    side.fullauto_input_prev = input;

    if (rising && !side.fullauto_enabled) {
      side.fullauto_enabled = true;
      publishCmd(side, kShootCmdFullauto);
      RCLCPP_INFO(this->get_logger(), "On trigger: %s Fullauto", side.display_name.c_str());
    } else if (falling && side.fullauto_enabled) {
      side.fullauto_enabled = false;
      publishCmd(side, kShootCmdStop);
    }
  }

  bool isManualTarget(const SideChannel & side) const
  {
    return manual_mode_target_side_ == side.name;
  }

  //========================================
  // publish helpers
  //========================================
  void publishCmd(const SideChannel & side, int repeat_count)
  {
    std_msgs::msg::Int32 msg;
    msg.data = repeat_count;
    side.cmd_pub->publish(msg);
  }

  static void publishBool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & pub, bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    pub->publish(msg);
  }

  static void publishFloat(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & pub, float value)
  {
    std_msgs::msg::Float32 msg;
    msg.data = value;
    pub->publish(msg);
  }

  //========================================
  // 左右共通入力
  //========================================
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr manual_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_motor_state_sub_;

  //========================================
  // 左右系統
  //========================================
  SideChannel left_;
  SideChannel right_;

  //========================================
  // parameters
  //========================================
  int burst_count_ = 3;
  double shoot_motor_on_command_ = 2000.0;
  std::string manual_mode_target_side_ = "right";
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShooterCmdGate>());

  rclcpp::shutdown();
  return 0;
}
