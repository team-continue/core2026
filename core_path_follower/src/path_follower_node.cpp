#include "core_path_follower/path_follower.hpp"
#include "core_path_follower/pid.hpp"
#include "core_path_follower/interpolation.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace core_path_follower
{
    PathFollower::PathFollower()
        : Node("core_path_follower")
    {
        // declare parameters with defaults (cascade PID: outer -> desired omega, inner -> rate)
        this->declare_parameter<double>("linear_speed", 0.5);
        this->declare_parameter<double>("lookahead_dist", 0.5);
        this->declare_parameter<std::string>("controller_type", "cascade");
        // outer PID (heading -> desired omega)
        this->declare_parameter<double>("outer_kp", 1.0);
        this->declare_parameter<double>("outer_ki", 0.0);
        this->declare_parameter<double>("outer_kd", 0.1);
        // inner PID (omega tracking)
        this->declare_parameter<double>("inner_kp", 2.0);
        this->declare_parameter<double>("inner_ki", 0.0);
        this->declare_parameter<double>("inner_kd", 0.05);
        this->declare_parameter<double>("pure_k", 1.0);
        this->declare_parameter<double>("control_rate", 20.0);
        this->declare_parameter<bool>("reset_on_new_path", false);
        // interpolation options: 'none', 'spline', 'bezier'
        this->declare_parameter<std::string>("interpolation", "none");
        this->declare_parameter<int>("spline_samples_per_segment", 10);
        this->declare_parameter<int>("bezier_samples", 100);

        linear_speed_ = this->get_parameter("linear_speed").as_double();
        lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
        controller_type_ = this->get_parameter("controller_type").as_string();
        outer_kp_ = this->get_parameter("outer_kp").as_double();
        outer_ki_ = this->get_parameter("outer_ki").as_double();
        outer_kd_ = this->get_parameter("outer_kd").as_double();
        inner_kp_ = this->get_parameter("inner_kp").as_double();
        inner_ki_ = this->get_parameter("inner_ki").as_double();
        inner_kd_ = this->get_parameter("inner_kd").as_double();
        pure_k_ = this->get_parameter("pure_k").as_double();
        control_rate_ = this->get_parameter("control_rate").as_double();
        reset_on_new_path_ = this->get_parameter("reset_on_new_path").as_bool();
        interpolation_type_ = this->get_parameter("interpolation").as_string();
        spline_samples_per_segment_ = this->get_parameter("spline_samples_per_segment").as_int();
        bezier_samples_ = this->get_parameter("bezier_samples").as_int();

        heading_outer_pid_ = PID(outer_kp_, outer_ki_, outer_kd_);
        heading_inner_pid_ = PID(inner_kp_, inner_ki_, inner_kd_);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10,
                                                                   std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 50,
                                                                       std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        auto period = std::chrono::duration<double>(1.0 / control_rate_);
        timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                         std::bind(&PathFollower::controlTimer, this));

        RCLCPP_INFO(this->get_logger(), "core_path_follower started (controller=%s)", controller_type_.c_str());
    }

    void PathFollower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        std::vector<geometry_msgs::msg::Pose> incoming;
        incoming.clear();
        for (const auto &p : msg->poses)
            incoming.push_back(p.pose);

        // If interpolation requested, upsample the coarse waypoints
        if (interpolation_type_ == "spline")
        {
            path_poses_ = core_path_follower::interp::splineCatmullRom(incoming, spline_samples_per_segment_);
        }
        else if (interpolation_type_ == "bezier")
        {
            path_poses_ = core_path_follower::interp::bezierGlobal(incoming, bezier_samples_);
        }
        else
        {
            path_poses_.clear();
            for (const auto &p : incoming)
                path_poses_.push_back(p);
        }
        // determine starting index: if we have odom, pick closest point, otherwise 0
        if (have_odom_ && !path_poses_.empty())
        {
            double cx = current_pose_.position.x;
            double cy = current_pose_.position.y;
            size_t best = 0;
            double best_d = 1e9;
            for (size_t i = 0; i < path_poses_.size(); ++i)
            {
                double dx = path_poses_[i].position.x - cx;
                double dy = path_poses_[i].position.y - cy;
                double d = std::hypot(dx, dy);
                if (d < best_d)
                {
                    best_d = d;
                    best = i;
                }
            }
            current_target_idx_ = best;
        }
        else
        {
            current_target_idx_ = 0;
        }
        if (reset_on_new_path_)
        {
            heading_outer_pid_.reset();
            heading_inner_pid_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "Received path with %zu poses, start_idx=%zu", path_poses_.size(), current_target_idx_);
    }

        void PathFollower::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        have_odom_ = true;
        latest_angular_z_ = msg->twist.twist.angular.z;
    }

    double PathFollower::quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    std::pair<double, double> PathFollower::poseXY(const geometry_msgs::msg::Pose &p)
    {
        return {p.position.x, p.position.y};
    }

    void PathFollower::controlTimer()
    {
        if (!have_odom_)
            return;

        std::lock_guard<std::mutex> lock(path_mutex_);
        if (path_poses_.empty())
            return;

        // current position
        auto [cx, cy] = poseXY(current_pose_);

        // ensure current_target_idx_ is within bounds
        if (current_target_idx_ >= path_poses_.size())
            current_target_idx_ = path_poses_.empty() ? 0 : path_poses_.size() - 1;

        // advance current_target_idx_ if earlier points are now behind us
        double best_d = 1e9;
        size_t best_i = current_target_idx_;
        // search from current index up to a small window ahead to find closest
        size_t search_end = std::min(path_poses_.size(), current_target_idx_ + 20);
        for (size_t i = current_target_idx_; i < search_end; ++i)
        {
            auto [px, py] = poseXY(path_poses_[i]);
            double d = std::hypot(px - cx, py - cy);
            if (d < best_d)
            {
                best_d = d;
                best_i = i;
            }
        }
        // allow stepping backwards a little if closer earlier in path
        if (best_d < lookahead_dist_ * 2.0)
            current_target_idx_ = best_i;

        // search forward from current_target_idx_ until lookahead distance
        size_t look_idx = current_target_idx_;
        for (size_t i = current_target_idx_; i < path_poses_.size(); ++i)
        {
            auto [px, py] = poseXY(path_poses_[i]);
            double d = std::hypot(px - cx, py - cy);
            if (d >= lookahead_dist_)
            {
                look_idx = i;
                break;
            }
        }

        auto [tx, ty] = poseXY(path_poses_[look_idx]);
        double desired_yaw = std::atan2(ty - cy, tx - cx);
        double current_yaw = quaternionToYaw(current_pose_.orientation);
        double err = desired_yaw - current_yaw;
        // wrap to [-pi,pi]
        while (err > M_PI)
            err -= 2 * M_PI;
        while (err < -M_PI)
            err += 2 * M_PI;

        double dt = 1.0 / control_rate_;

        // Controller selection: 'pid' (single PID), 'cascade' (outer->inner), 'pure_pursuit'
        double omega_des = 0.0;
        double ang_z_cmd = 0.0;
        double alpha = 0.0;

        if (controller_type_ == "pid")
        {
            // single PID produces angular velocity command directly
            ang_z_cmd = heading_outer_pid_.update(err, dt);
        }
        else if (controller_type_ == "cascade")
        {
            // Outer PID: heading error -> desired angular velocity (omega_des)
            omega_des = heading_outer_pid_.update(err, dt);
            // Inner PID: track angular velocity (use odom measurement)
            ang_z_cmd = heading_inner_pid_.update(omega_des - latest_angular_z_, dt);
        }
        else if (controller_type_ == "pure_pursuit")
        {
            // pure pursuit: compute target in body frame (we'll compute x_b,y_b below)
            // compute body-frame coordinates first
            double yaw = current_yaw;
            double x_b = std::cos(yaw) * (tx - cx) + std::sin(yaw) * (ty - cy);
            double y_b = -std::sin(yaw) * (tx - cx) + std::cos(yaw) * (ty - cy);
            double alpha = std::atan2(y_b, x_b); // lookahead angle in body frame
            // curvature -> desired omega: 2*v*sin(alpha)/L, scaled by pure_k_
            double L = std::max(lookahead_dist_, 1e-6);
            omega_des = pure_k_ * 2.0 * linear_speed_ * std::sin(alpha) / L;
            ang_z_cmd = heading_inner_pid_.update(omega_des - latest_angular_z_, dt);
        }
        else
        {
            // unknown controller: fallback to P
            ang_z_cmd = outer_kp_ * err;
        }

        // compute desired translational velocity in body frame towards lookahead point
        double dx = tx - cx;
        double dy = ty - cy;
        double dist = std::hypot(dx, dy);
        double vx = 0.0, vy = 0.0;
        if (dist > 1e-6)
        {
            // transform world delta into body frame: body = R^T * world_delta
            double yaw = current_yaw;
            double x_b = std::cos(yaw) * dx + std::sin(yaw) * dy;
            double y_b = -std::sin(yaw) * dx + std::cos(yaw) * dy;
            // normalized direction in body frame times speed
            double heading_factor = std::max(0.0, 1.0 - std::min(1.0, std::abs(err) / (M_PI / 2.0)));
            double speed = linear_speed_ * heading_factor;
            // For pure_pursuit we prefer full speed along x body axis toward target
            if (controller_type_ == "pure_pursuit")
            {
                // drive forward with desired speed projected
                vx = speed * std::cos(alpha);
                vy = speed * std::sin(alpha);
            }
            else
            {
                vx = speed * (x_b / dist);
                vy = speed * (y_b / dist);
            }
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = ang_z_cmd;
        cmd_pub_->publish(cmd);

        // if we are very close to the current_target_idx_ point, advance index
        double close_thresh = 0.3; // meters
        auto [cx2, cy2] = poseXY(current_pose_);
        auto [tpx, tpy] = poseXY(path_poses_[current_target_idx_]);
        double dcur = std::hypot(tpx - cx2, tpy - cy2);
        if (dcur < close_thresh && current_target_idx_ + 1 < path_poses_.size())
        {
            current_target_idx_++;
        }
    }

} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<core_path_follower::PathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
