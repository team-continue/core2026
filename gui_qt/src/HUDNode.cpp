#include "HUDNode.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

#define VAR_TO_STR(var) (#var)

HUDNode::HUDNode(std::shared_ptr<HUDCore> core, std::string global_battle_param_filepath, const rclcpp::NodeOptions &options)
    : Node("hud_node", options),
      global_battle_param_filepath_(global_battle_param_filepath),
      battle_param_(((Node*)this))
{
    RCLCPP_INFO(get_logger(), "Using Parameter = %s", global_battle_param_filepath.c_str());
    core_ = core;

    // Subscriber
    sub_hp_ = create_subscription<std_msgs::msg::UInt8>(
        "~/input/hp", rclcpp::QoS(10), std::bind(&HUDNode::onHP, this, std::placeholders::_1));

    sub_ammo_ = create_subscription<std_msgs::msg::Int8>(
        "~/input/ammo", rclcpp::QoS(10), std::bind(&HUDNode::onAmmo, this, std::placeholders::_1));

    sub_hp_max_ = create_subscription<std_msgs::msg::UInt8>(
        "~/input/max_hp", rclcpp::QoS(10), std::bind(&HUDNode::onMaxHP, this, std::placeholders::_1));

    sub_ammo_max_ = create_subscription<std_msgs::msg::Int8>(
        "~/input/max_ammo", rclcpp::QoS(10), std::bind(&HUDNode::onMaxAmmo, this, std::placeholders::_1));

    sub_compass_ = create_subscription<std_msgs::msg::Float32>(
        "~/input/compass", rclcpp::QoS(10), std::bind(&HUDNode::onCompass, this, std::placeholders::_1));

    sub_speed_ = create_subscription<std_msgs::msg::Float32>(
        "~/input/speed", rclcpp::QoS(10), std::bind(&HUDNode::onSpeed, this, std::placeholders::_1));

    sub_qe_ = create_subscription<std_msgs::msg::Float32>(
        "~/input/qe", rclcpp::QoS(10), std::bind(&HUDNode::onQE, this, std::placeholders::_1));

    sub_log_ = create_subscription<std_msgs::msg::String>(
        "~/input/log", rclcpp::QoS(10), std::bind(&HUDNode::onLog, this, std::placeholders::_1));

    sub_image_1_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        "~/input/camera", rclcpp::SensorDataQoS(), std::bind(&HUDNode::onImage1, this, std::placeholders::_1));

    sub_image_2_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        "~/input/camera_sub", rclcpp::SensorDataQoS(), std::bind(&HUDNode::onImage2, this, std::placeholders::_1));
    
    sub_enemy_poses_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "~/input/enemy_poses", rclcpp::QoS(10), std::bind(&HUDNode::onEnemyPoses, this, std::placeholders::_1));

    sub_hazard_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/hazard", rclcpp::QoS(10), std::bind(&HUDNode::onHazard, this, std::placeholders::_1));

    sub_input_camera1_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/camera_change_1", rclcpp::QoS(10), std::bind(&HUDNode::onInputCamera1, this, std::placeholders::_1));
    
    sub_input_camera2_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/camera_change_2", rclcpp::QoS(10), std::bind(&HUDNode::onInputCamera2, this, std::placeholders::_1));
    
    sub_input_down_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/cursor_next", rclcpp::QoS(10), std::bind(&HUDNode::onInputDown, this, std::placeholders::_1));
    
    sub_input_up_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/cursor_prev", rclcpp::QoS(10), std::bind(&HUDNode::onInputUp, this, std::placeholders::_1));

    sub_input_left_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/value_down", rclcpp::QoS(10), std::bind(&HUDNode::onInputLeft, this, std::placeholders::_1));

    sub_input_right_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/value_up", rclcpp::QoS(10), std::bind(&HUDNode::onInputRight, this, std::placeholders::_1));

    sub_input_ok_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/cursor_ok", rclcpp::QoS(10), std::bind(&HUDNode::onInputOK, this, std::placeholders::_1));

    sub_input_back_ = create_subscription<std_msgs::msg::Bool>(
        "~/input/cursor_back", rclcpp::QoS(10), std::bind(&HUDNode::onInputBack, this, std::placeholders::_1));
    

    // Parameter
    battle_param_.declareParameter();
    node_param_.range_aim_sensitivity = declare_parameter<std::vector<double>>("range_aim_sensitivity");
    node_param_.range_max_velocity = declare_parameter<std::vector<double>>("range_max_velocity");
    node_param_.range_max_omega = declare_parameter<std::vector<double>>("range_max_omega");
    node_param_.range_max_shooter = declare_parameter<std::vector<double>>("range_max_shooter");
    parameter_set_destination_ = declare_parameter<std::vector<std::string>>("parameter_set_destination");

    on_set_param_callback_ = add_on_set_parameters_callback(std::bind(&HUDNode::onSetParam, this, std::placeholders::_1));
    
    core_->setRangeAimSensitivity(node_param_.range_aim_sensitivity[0], node_param_.range_aim_sensitivity[1]);
    core_->setRangeMaxVelocity(node_param_.range_max_velocity[0], node_param_.range_max_velocity[1]);
    core_->setRangeMaxOmega(node_param_.range_max_omega[0], node_param_.range_max_omega[1]);
    core_->setRangeMaxShooter(node_param_.range_max_shooter[0], node_param_.range_max_shooter[1]);
    
    // create client
    for (auto str : parameter_set_destination_) {
        if (str == "") continue;
        RCLCPP_INFO(this->get_logger(), "Create AsyncParametersClient : %s", str.c_str());
        paramater_clients_.emplace_back(std::make_shared<rclcpp::AsyncParametersClient>(this, str));
    }
}

void HUDNode::onHP(const std_msgs::msg::UInt8::SharedPtr msg) {
    uint8_t value =msg->data;
    core_->setHP(value);
}

void HUDNode::onAmmo(const std_msgs::msg::Int8::SharedPtr msg) {
    uint8_t value =msg->data;
    core_->setAmmo(value);
}

void HUDNode::onMaxHP(const std_msgs::msg::UInt8::SharedPtr msg) {
    uint8_t value =msg->data;
    core_->setMaxHP(value);
}

void HUDNode::onMaxAmmo(const std_msgs::msg::Int8::SharedPtr msg) {
    uint8_t value =msg->data;
    core_->setMaxAmmo(value);

}

void HUDNode::onCompass(const std_msgs::msg::Float32::SharedPtr msg) {
    float value =msg->data;
    core_->setCompass(value);
}

void HUDNode::onSpeed(const std_msgs::msg::Float32::SharedPtr msg) {
    float value =msg->data;
    core_->setSpeed(value);
}

void HUDNode::onQE(const std_msgs::msg::Float32::SharedPtr msg) {
    float value =msg->data;
    core_->setQE(value);
}

void HUDNode::onLog(const std_msgs::msg::String::SharedPtr msg) {
    std::string value =msg->data;
    core_->setLog(value);
}

void HUDNode::onImage1(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    core_->setImage1(msg);
}

void HUDNode::onImage2(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    core_->setImage2(msg);
}

void HUDNode::onEnemyPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    core_->setEnemyPoses(msg->poses);
}

void HUDNode::onHazard(const std_msgs::msg::Bool::SharedPtr msg) {
    core_->setHazard(msg->data);
}

void HUDNode::onInputCamera1(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputCamera1();
}

void HUDNode::onInputCamera2(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputCamera2();
}

void HUDNode::onInputUp(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputUp();
}

void HUDNode::onInputDown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputDown();
}

void HUDNode::onInputLeft(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputLeft();
}

void HUDNode::onInputRight(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputRight();
}

void HUDNode::onInputBack(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputBack(battle_param_);
}

void HUDNode::onInputOK(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;
    core_->onInputOK();
}

rcl_interfaces::msg::SetParametersResult HUDNode::onSetParam(const std::vector<rclcpp::Parameter> &parameters) {

    for (auto&& param : parameters){
        if (param.get_name() == "range_aim_sensitivity") {
            node_param_.range_aim_sensitivity = param.as_double_array();
            core_->setRangeAimSensitivity(node_param_.range_aim_sensitivity[0], node_param_.range_aim_sensitivity[1]);
        } else if (param.get_name() == "range_max_velocity") {
            node_param_.range_max_velocity = param.as_double_array();
            core_->setRangeMaxVelocity(node_param_.range_max_velocity[0], node_param_.range_max_velocity[1]);
        } else if (param.get_name() == "range_max_omega") {
            node_param_.range_max_omega = param.as_double_array();
            core_->setRangeMaxOmega(node_param_.range_max_omega[0], node_param_.range_max_omega[1]);
        } else if (param.get_name() == "range_max_shooter") {
            node_param_.range_max_shooter = param.as_double_array();
            core_->setRangeMaxShooter(node_param_.range_max_shooter[0], node_param_.range_max_shooter[1]);
        }
    }

    battle_param_.updateParamater(parameters);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
}

bool HUDNode::requestSetBattleParameter(BattleParamData &battle_param) {
    std::vector<rclcpp::Parameter> params;

    params.emplace_back(rclcpp::Parameter("sensitivity", battle_param.sensitivity));
    params.emplace_back(rclcpp::Parameter("max_vel", battle_param.max_vel));
    params.emplace_back(rclcpp::Parameter("max_omega", battle_param.max_omega));
    params.emplace_back(rclcpp::Parameter("max_roller_speed", battle_param.max_roller_speed));
    params.emplace_back(rclcpp::Parameter("team", battle_param.team));
    params.emplace_back(rclcpp::Parameter("enable_auto_aim", battle_param.enable_auto_aim));
    
    for (const auto &client: paramater_clients_) {
        if (client->service_is_ready()) {
            auto future_result = client->set_parameters(params);
            auto results = future_result.get();

        }
    }

    saveBattleParamater2Yaml(battle_param);

    battle_param_.data = battle_param;

    return true;
}

void HUDNode::saveBattleParamater2Yaml(BattleParamData &battle_param) {
    if (global_battle_param_filepath_ == "") return;
    
    YAML::Node yaml_npde;
    yaml_npde["/**"]["ros__parameters"]["sensitivity"] = battle_param.sensitivity;
    yaml_npde["/**"]["ros__parameters"]["max_vel"] = battle_param.max_vel;
    yaml_npde["/**"]["ros__parameters"]["max_omega"] = battle_param.max_omega;
    yaml_npde["/**"]["ros__parameters"]["max_roller_speed"] = battle_param.max_roller_speed;
    yaml_npde["/**"]["ros__parameters"]["team"] = battle_param.team;
    yaml_npde["/**"]["ros__parameters"]["enable_auto_aim"] = battle_param.enable_auto_aim;
    RCLCPP_INFO(get_logger(), "sensitivity: %lf, max_vel: %lf, max_omega: %lf, max_roller_speed: %lf, team: %d, enable_auto_aim: %d",
        battle_param.sensitivity,
        battle_param.max_vel,
        battle_param.max_omega,
        battle_param.max_roller_speed,
        battle_param. team,
        battle_param.enable_auto_aim
    );
    try {
        std::ofstream fout(global_battle_param_filepath_);
        fout << yaml_npde;
        fout.close();
        RCLCPP_INFO(get_logger(), "YAML file saved: %s", global_battle_param_filepath_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "%s", e.what());
    }
}