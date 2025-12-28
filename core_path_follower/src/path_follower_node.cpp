#include "core_path_follower/path_follower.hpp"

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
            path_poses_ = interpolateSpline(incoming, spline_samples_per_segment_);
        }
        else if (interpolation_type_ == "bezier")
        {
            path_poses_ = interpolateBezier(incoming, bezier_samples_);
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

    // Catmull-Rom spline interpolation (Cubic) between waypoints
    std::vector<geometry_msgs::msg::Pose> PathFollower::interpolateSpline(const std::vector<geometry_msgs::msg::Pose> &waypoints, int samples_per_segment)
    {
        std::vector<geometry_msgs::msg::Pose> out;
        if (waypoints.size() < 2)
            return waypoints;

        // For endpoints, duplicate nearest neighbor to provide p0/p3
        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            geometry_msgs::msg::Pose p0 = (i == 0) ? waypoints[i] : waypoints[i - 1];
            geometry_msgs::msg::Pose p1 = waypoints[i];
            geometry_msgs::msg::Pose p2 = waypoints[i + 1];
            geometry_msgs::msg::Pose p3 = (i + 2 < waypoints.size()) ? waypoints[i + 2] : waypoints[i + 1];

            for (int s = 0; s < samples_per_segment; ++s)
            {
                double t = static_cast<double>(s) / static_cast<double>(samples_per_segment);
                double t2 = t * t;
                double t3 = t2 * t;

                double x0 = p0.position.x;
                double x1 = p1.position.x;
                double x2 = p2.position.x;
                double x3 = p3.position.x;
                double y0 = p0.position.y;
                double y1 = p1.position.y;
                double y2 = p2.position.y;
                double y3 = p3.position.y;

                double cx = 0.5 * ((2.0 * x1) + (-x0 + x2) * t + (2.0 * x0 - 5.0 * x1 + 4.0 * x2 - x3) * t2 + (-x0 + 3.0 * x1 - 3.0 * x2 + x3) * t3);
                double cy = 0.5 * ((2.0 * y1) + (-y0 + y2) * t + (2.0 * y0 - 5.0 * y1 + 4.0 * y2 - y3) * t2 + (-y0 + 3.0 * y1 - 3.0 * y2 + y3) * t3);

                geometry_msgs::msg::Pose np;
                np.position.x = cx;
                np.position.y = cy;
                np.position.z = 0.0;
                // orientation from derivative approx: use small delta ahead
                double dx = cx - p1.position.x;
                double dy = cy - p1.position.y;
                double yaw = std::atan2(dy, dx);
                np.orientation.w = std::cos(yaw * 0.5);
                np.orientation.x = 0.0;
                np.orientation.y = 0.0;
                np.orientation.z = std::sin(yaw * 0.5);
                out.push_back(np);
            }
        }
        // finally push last waypoint
        out.push_back(waypoints.back());
        return out;
    }

    // Global Bezier interpolation using all control points (single curve)
    std::vector<geometry_msgs::msg::Pose> PathFollower::interpolateBezier(const std::vector<geometry_msgs::msg::Pose> &waypoints, int samples)
    {
        std::vector<geometry_msgs::msg::Pose> out;
        if (waypoints.size() < 2)
            return waypoints;

        int n = static_cast<int>(waypoints.size()) - 1; // degree = n

        // Precompute factorials for binomial coefficients
        std::vector<double> fact(n + 1, 1.0);
        for (int i = 1; i <= n; ++i)
            fact[i] = fact[i - 1] * static_cast<double>(i);

        auto binom = [&](int i, int n)
        {
            return fact[n] / (fact[i] * fact[n - i]);
        };

        for (int s = 0; s <= samples; ++s)
        {
            double t = static_cast<double>(s) / static_cast<double>(samples);
            double x = 0.0;
            double y = 0.0;
            for (int i = 0; i <= n; ++i)
            {
                double b = binom(i, n) * std::pow(1.0 - t, n - i) * std::pow(t, i);
                x += b * waypoints[i].position.x;
                y += b * waypoints[i].position.y;
            }
            geometry_msgs::msg::Pose np;
            np.position.x = x;
            np.position.y = y;
            np.position.z = 0.0;
            // approximate yaw by forward difference (use small delta ahead)
            double t_a = std::min(1.0, t + 1e-3);
            double xa = 0.0, ya = 0.0;
            for (int i = 0; i <= n; ++i)
            {
                double b = binom(i, n) * std::pow(1.0 - t_a, n - i) * std::pow(t_a, i);
                xa += b * waypoints[i].position.x;
                ya += b * waypoints[i].position.y;
            }
            double yaw = std::atan2(ya - y, xa - x);
            np.orientation.w = std::cos(yaw * 0.5);
            np.orientation.x = 0.0;
            np.orientation.y = 0.0;
            np.orientation.z = std::sin(yaw * 0.5);
            out.push_back(np);
        }
        return out;
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
