#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <mutex>
#include "core_path_follower/pid.hpp"

namespace core_path_follower
{
    // PID class moved to pid.hpp

    class PathFollower : public rclcpp::Node
    {
    public:
        PathFollower();

    private:
        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void controlTimer();
        double quaternionToYaw(const geometry_msgs::msg::Quaternion &q);
        std::pair<double, double> poseXY(const geometry_msgs::msg::Pose &p);

        // Control logic for local-frame path (from path_planner, robot-relative)
        void controlTimerLocalFrame();

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<geometry_msgs::msg::Pose> path_poses_;
        std::mutex path_mutex_;
        // index on the path we are currently tracking (closest / lookahead base)
        size_t current_target_idx_ = 0;
        geometry_msgs::msg::Pose current_pose_;
        bool have_odom_ = false;
        bool goal_reached_ = false;

        bool reset_on_new_path_ = false;

        PID heading_outer_pid_; // computes desired angular rate from heading error
        PID heading_inner_pid_; // rate PID to track angular velocity

        // runtime measured
        double latest_angular_z_ = 0.0;

        // parameters
        std::string path_topic_;
        std::string odom_topic_;
        std::string cmd_vel_topic_;
        double linear_speed_;
        double lookahead_dist_;
        std::string controller_type_;
        // coordinate frame mode: false = world frame (use odom), true = local/robot frame (from path_planner)
        bool use_local_frame_;
        // goal tolerance: stop when within this distance of the last waypoint
        double goal_tolerance_;
        // interpolation settings
        std::string interpolation_type_; // 'none', 'spline', 'bezier'
        int spline_samples_per_segment_;
        int bezier_samples_;
        // outer PID gains (heading -> desired omega)
        double outer_kp_, outer_ki_, outer_kd_;
        // inner PID gains (omega tracking)
        double inner_kp_, inner_ki_, inner_kd_;
        // pure pursuit gain
        double pure_k_;
        double control_rate_;

        // interpolation is provided in separate module interpolation.hpp
    };

} // namespace
