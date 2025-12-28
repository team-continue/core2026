#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <mutex>

namespace core_path_follower
{
    class PID
    {
    public:
        PID(double p = 0.0, double i = 0.0, double d = 0.0)
            : kp_(p), ki_(i), kd_(d), prev_(0.0), integral_(0.0) {}

        double update(double error, double dt)
        {
            if (dt <= 0.0)
                return 0.0;
            integral_ += error * dt;
            double derivative = (error - prev_) / dt;
            prev_ = error;
            return kp_ * error + ki_ * integral_ + kd_ * derivative;
        }

        void reset()
        {
            prev_ = 0.0;
            integral_ = 0.0;
        }

        double kp_, ki_, kd_;

    private:
        double prev_;
        double integral_;
    };

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

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<geometry_msgs::msg::Pose> path_poses_;
        std::mutex path_mutex_;
        // index on the path we are currently tracking (closest / lookahead base)
        size_t current_target_idx_ = 0;
        geometry_msgs::msg::Pose current_pose_;
        bool have_odom_ = false;

        bool reset_on_new_path_ = false;

        PID heading_outer_pid_; // computes desired angular rate from heading error
        PID heading_inner_pid_; // rate PID to track angular velocity

        // runtime measured
        double latest_angular_z_ = 0.0;

        // parameters
        double linear_speed_;
        double lookahead_dist_;
        std::string controller_type_;
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

        // interpolation helpers
        std::vector<geometry_msgs::msg::Pose> interpolateSpline(const std::vector<geometry_msgs::msg::Pose> &waypoints, int samples_per_segment);
        std::vector<geometry_msgs::msg::Pose> interpolateBezier(const std::vector<geometry_msgs::msg::Pose> &waypoints, int samples);
    };

} // namespace
