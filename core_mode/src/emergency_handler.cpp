#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

class EmergencyHandlerNode : public rclcpp::Node
{
public:
  EmergencyHandlerNode()
  : Node("emergency_handler_node")
  {
    //========================================
    // subscribers
    //========================================
    emergency_switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_switch", 10,
      std::bind(&EmergencyHandlerNode::emergencySwitchCallback, this, std::placeholders::_1));

    software_emergency_on_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_button_on", 10,
      std::bind(&EmergencyHandlerNode::softwareEmergencyOnCallback, this, std::placeholders::_1));

    software_emergency_off_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_button_off", 10,
      std::bind(&EmergencyHandlerNode::softwareEmergencyOffCallback, this, std::placeholders::_1));

    destroy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "destroy", 10,
      std::bind(&EmergencyHandlerNode::destroyCallback, this, std::placeholders::_1));

    microcontroller_emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "microcontroller_emergency", 10,
      std::bind(&EmergencyHandlerNode::microcontrollerDiagCallback, this, std::placeholders::_1));

    receiver_emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "receiver_emergency", 10,
      std::bind(&EmergencyHandlerNode::receiverDiagCallback, this, std::placeholders::_1));

    //========================================
    // publishers
    //========================================
    hazard_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "hazard_status", 10);
    hazard_states_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(
      "hazard_states", 10);
    hazard_label_pub_ = this->create_publisher<std_msgs::msg::String>(
      "hazard_label", 10);

    //========================================
    // Initialization
    //========================================
    RCLCPP_INFO(this->get_logger(), "EmergencyHandlerNode initialized");
    evaluateHazardStates();
  }

private:
  //========================================
  // callbacks
  //========================================
  void emergencySwitchCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    emergency_switch_state_ = msg->data;
    if (msg->data) {
      high_emergency_level_ = true;
    }
    evaluateHazardStates();
  }

  void softwareEmergencyOnCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      software_emergency_state_ = true;
      high_emergency_level_ = true;
    }
    evaluateHazardStates();
  }

  void softwareEmergencyOffCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      if (!isEmergency()) {
        software_emergency_state_ = false;
        high_emergency_level_ = false;
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Software Emergency OFF ignored — other hazards still active");
      }
    }
    evaluateHazardStates();
  }

  void destroyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    destroy_state_ = msg->data;
    if (msg->data) {
      high_emergency_level_ = true;
    }
    evaluateHazardStates();
  }

  void microcontrollerDiagCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    microcontroller_emergency_state_ = msg->data;
    if (msg->data) {
      high_emergency_level_ = true;
    }
    evaluateHazardStates();
  }

  void receiverDiagCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    receiver_emergency_state_ = msg->data;
    evaluateHazardStates();
  }

  //========================================
  // main logic
  //========================================
  void evaluateHazardStates()
  {
    evaluateIndividualStates();
    publishHazardState();
  }

  void evaluateIndividualStates()
  {
    // 他のHazardがある場合、Software Emergencyは強制ON
    if (isEmergency()) {
      software_emergency_state_ = true;
    }

    // 軽微なエラーは自動復帰
    if (!isEmergency() && !high_emergency_level_) {
      software_emergency_state_ = false;
    }
  }

  void publishHazardState()
  {
    std_msgs::msg::Bool status_msg;
    std_msgs::msg::Int8MultiArray states_msg;
    std_msgs::msg::String label_msg;

    states_msg.data = {
      emergency_switch_state_,
      software_emergency_state_,
      microcontroller_emergency_state_,
      receiver_emergency_state_,
      destroy_state_};

    status_msg.data = software_emergency_state_;

    // ラベル作成
    std::vector<std::string> labels;
    if (emergency_switch_state_) {labels.push_back("Emergency Switch triggered");}
    if (microcontroller_emergency_state_) {labels.push_back("Microcontroller diagnostic failure");}
    if (receiver_emergency_state_) {labels.push_back("Receiver diagnostic failure");}
    if (destroy_state_) {labels.push_back("Destroy command received");}
    if (software_emergency_state_) {labels.push_back("Software Emergency active");}

    if (labels.empty()) {
      label_msg.data = "System normal";
    } else {
      std::string combined;
      for (size_t i = 0; i < labels.size(); ++i) {
        combined += labels[i];
        if (i < labels.size() - 1) {combined += ", ";}
      }
      label_msg.data = combined;
    }

    hazard_status_pub_->publish(status_msg);
    hazard_states_pub_->publish(states_msg);
    hazard_label_pub_->publish(label_msg);

    if (status_msg.data) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "%s",
        label_msg.data.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "%s",
        label_msg.data.c_str());
    }
  }

  bool isEmergency()
  {
    return emergency_switch_state_ ||
           destroy_state_ ||
           microcontroller_emergency_state_ ||
           receiver_emergency_state_;
  }

  //========================================
  // Subscription variables
  //========================================
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_switch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr software_emergency_on_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr software_emergency_off_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr destroy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr microcontroller_emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr receiver_emergency_sub_;

  //========================================
  // publisher variables
  //========================================
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hazard_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr hazard_states_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hazard_label_pub_;

  //========================================
  // variables
  //========================================
  bool emergency_switch_state_ = false;
  bool software_emergency_state_ = false;
  bool destroy_state_ = false;
  bool microcontroller_emergency_state_ = false;
  bool receiver_emergency_state_ = false;

  bool high_emergency_level_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyHandlerNode>());
  rclcpp::shutdown();
  return 0;
}
