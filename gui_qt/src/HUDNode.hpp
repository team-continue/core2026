#pragma once

#include "gui_qt/battle_param.hpp"
#include "HUDCore.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <vector>

class HUDNode : public rclcpp::Node{
    std::shared_ptr<HUDCore> core_;

    
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_hp_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_ammo_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_hp_max_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_ammo_max_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_compass_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_qe_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_log_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_1_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_enemy_poses_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_hazard_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_camera1_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_camera2_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_up_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_down_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_left_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_right_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_back_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_input_ok_;

    std::vector<rclcpp::AsyncParametersClient::SharedPtr> paramater_clients_;
    std::string global_battle_param_filepath_;

    BattleParam battle_param_;

    struct params {
        std::vector<double> range_aim_sensitivity;
        std::vector<double> range_max_velocity;
        std::vector<double> range_max_omega;
        std::vector<double> range_max_shooter;
    };
    params node_param_;
    std::vector<std::string> parameter_set_destination_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_param_callback_;

public:
    HUDNode(std::shared_ptr<HUDCore> core, std::string, const rclcpp::NodeOptions &options);
    void onHP(const std_msgs::msg::UInt8::SharedPtr msg);
    void onAmmo(const std_msgs::msg::Int8::SharedPtr msg);
    void onMaxHP(const std_msgs::msg::UInt8::SharedPtr msg);
    void onMaxAmmo(const std_msgs::msg::Int8::SharedPtr msg);
    void onCompass(const std_msgs::msg::Float32::SharedPtr msg);
    void onSpeed(const std_msgs::msg::Float32::SharedPtr msg);
    void onQE(const std_msgs::msg::Float32::SharedPtr msg);
    void onLog(const std_msgs::msg::String::SharedPtr msg);
    void onImage1(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onImage2(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onEnemyPoses(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void onHazard(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputCamera1(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputCamera2(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputUp(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputDown(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputLeft(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputRight(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputBack(const std_msgs::msg::Bool::SharedPtr msg);
    void onInputOK(const std_msgs::msg::Bool::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult onSetParam(const std::vector<rclcpp::Parameter> &parameters);
    bool requestSetBattleParameter(BattleParamData &battle_param);
    void saveBattleParamater2Yaml(BattleParamData &battle_param);
};