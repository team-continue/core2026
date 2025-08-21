#pragma once

#include <rclcpp/rclcpp.hpp>

struct BattleParamData {
    double sensitivity;
    double max_vel;
    double max_omega;
    double max_roller_speed;
    int team;
    bool enable_auto_aim;
};

class BattleParam {
    rclcpp::Node *node_;
public:
    BattleParamData data;

    BattleParam(rclcpp::Node *node)
     :  node_(node)
    {}

    void declareParameter() {
        data.sensitivity =  node_->declare_parameter<double>("sensitivity");
        data.max_vel = node_->declare_parameter<double>("max_vel");
        data.max_omega = node_->declare_parameter<double>("max_omega");
        data.max_roller_speed = node_->declare_parameter<double>("max_roller_speed");
        data.team = node_->declare_parameter<int>("team");
        data.enable_auto_aim = node_->declare_parameter<bool>("enable_auto_aim");
    }

    void updateParamater(const std::vector<rclcpp::Parameter> &parameters) {
        for (auto&& param : parameters){
            if (param.get_name() == "sensitivity") {
                data.sensitivity = param.as_double();
            } else if (param.get_name() == "range_max_velocity") {
                data.max_vel = param.as_double();
            } else if (param.get_name() == "range_max_omega") {
                data.max_omega = param.as_double();
            } else if (param.get_name() == "range_max_shooter") {
                data.max_roller_speed = param.as_double();
            } else if (param.get_name() == "team") {
                data.team = param.as_int();
            } else if (param.get_name() == "enable_auto_aim") {
                data.enable_auto_aim = param.as_bool();
            }
        }
    }
};