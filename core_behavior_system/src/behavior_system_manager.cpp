#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <optional>
#include <string>

namespace {

double PoseDistanceSq(const geometry_msgs::msg::PoseStamped &a,
                      const geometry_msgs::msg::PoseStamped &b) {
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return dx * dx + dy * dy + dz * dz;
}

bool PoseNearlyEqual(const geometry_msgs::msg::PoseStamped &a,
                     const geometry_msgs::msg::PoseStamped &b,
                     double pos_eps) {
  if (a.header.frame_id != b.header.frame_id) {
    return false;
  }
  if (PoseDistanceSq(a, b) > pos_eps * pos_eps) {
    return false;
  }
  return std::abs(a.pose.orientation.x - b.pose.orientation.x) < 1e-6 &&
         std::abs(a.pose.orientation.y - b.pose.orientation.y) < 1e-6 &&
         std::abs(a.pose.orientation.z - b.pose.orientation.z) < 1e-6 &&
         std::abs(a.pose.orientation.w - b.pose.orientation.w) < 1e-6;
}

}  // namespace

class BehaviorSystemManager : public rclcpp::Node {
public:
  BehaviorSystemManager() : rclcpp::Node("behavior_system") {
    manual_mode_topic_ = declare_parameter<std::string>("manual_mode_topic", "/manual_mode");
    auto_point_select_topic_ =
        declare_parameter<std::string>("auto_point_select_topic", "/auto_point_select");
    selected_pose_topic_ =
        declare_parameter<std::string>("selected_pose_topic", "/selected_pose");
    enemy_detected_topic_ =
        declare_parameter<std::string>("enemy_detected_topic", "/enemy_detected");
    goal_reached_topic_ =
        declare_parameter<std::string>("goal_reached_topic", "/goal_reached");
    waypoint_goal_topic_ = declare_parameter<std::string>(
        "waypoint_goal_topic", "/waypoint_selector/goal_pose");
    goal_pose_topic_ =
        declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    rotation_topic_ = declare_parameter<std::string>("rotation_topic", "/rotation");
    state_topic_ =
        declare_parameter<std::string>("state_topic", "/behavior_system/state");
    state_name_topic_ = declare_parameter<std::string>(
        "state_name_topic", "/behavior_system/state_name");
    pause_topic_ =
        declare_parameter<std::string>("pause_topic", "/waypoint_selector/pause");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    pose_equal_eps_ = declare_parameter<double>("pose_equal_eps", 1e-3);

    manual_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
        manual_mode_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          manual_mode_ = msg->data;
        });

    auto_point_select_sub_ = create_subscription<std_msgs::msg::Bool>(
        auto_point_select_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          auto_point_select_ = msg->data;
        });

    selected_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        selected_pose_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          latest_selected_pose_ = *msg;
          have_selected_pose_ = true;
          selected_pose_dirty_ = true;
        });

    enemy_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
        enemy_detected_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          enemy_detected_ = msg->data;
        });

    goal_reached_sub_ = create_subscription<std_msgs::msg::Bool>(
        goal_reached_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          goal_reached_ = msg->data;
        });

    waypoint_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        waypoint_goal_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          latest_waypoint_goal_ = *msg;
          have_waypoint_goal_ = true;
          waypoint_goal_dirty_ = true;
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(50),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_odom_ = *msg;
          have_odom_ = true;
        });

    rclcpp::QoS goal_qos(1);
    goal_qos.transient_local();
    goal_qos.reliable();
    goal_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, goal_qos);

    rotation_pub_ = create_publisher<std_msgs::msg::Bool>(rotation_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::Int32>(state_topic_, 10);
    state_name_pub_ = create_publisher<std_msgs::msg::String>(state_name_topic_, 10);
    pause_pub_ = create_publisher<std_msgs::msg::Bool>(pause_topic_, 10);

    const double safe_rate = publish_rate_hz_ > 0.0 ? publish_rate_hz_ : 1.0;
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / safe_rate)),
        [this]() { onTimer(); });

    RCLCPP_INFO(get_logger(), "behavior_system manager node started");
  }

private:
  enum class GoalSource {
    NONE,
    WAYPOINT,
    SELECTED,
    STOP
  };

  enum class BehaviorState {
    MANUAL = 0,
    ATTACK = 1,
    AUTO_SELECTED = 2,
    AUTO_WAYPOINT = 3,
    AUTO_IDLE = 4
  };

  void onTimer() {
    const BehaviorState next_state = computeNextState();
    publishState(next_state);
    setPause(next_state != BehaviorState::AUTO_WAYPOINT);

    switch (next_state) {
      case BehaviorState::MANUAL:
        setRotation(false);
        last_source_ = GoalSource::NONE;
        break;
      case BehaviorState::AUTO_SELECTED:
        setRotation(false);
        handleSelectedGoal();
        break;
      case BehaviorState::ATTACK:
        setRotation(true);
        publishStopGoal();
        break;
      case BehaviorState::AUTO_WAYPOINT:
        setRotation(false);
        handleWaypointGoal();
        break;
      case BehaviorState::AUTO_IDLE:
        setRotation(false);
        last_source_ = GoalSource::NONE;
        break;
    }
  }

  BehaviorState computeNextState() const {
    if (manual_mode_) {
      return BehaviorState::MANUAL;
    }
    if (!auto_point_select_) {
      // Manual point select: movement has priority, ignore enemy detection.
      return have_selected_pose_ ? BehaviorState::AUTO_SELECTED : BehaviorState::AUTO_IDLE;
    }
    if (enemy_detected_) {
      return BehaviorState::ATTACK;
    }
    if (!have_waypoint_goal_) {
      return BehaviorState::AUTO_IDLE;
    }
    return BehaviorState::AUTO_WAYPOINT;
  }

  void handleSelectedGoal() {
    if (!have_selected_pose_) {
      return;
    }

    if (goal_reached_ && last_source_ == GoalSource::SELECTED && !selected_pose_dirty_) {
      // Already reached and no new selection.
      return;
    }

    publishGoal(latest_selected_pose_, GoalSource::SELECTED, selected_pose_dirty_);
    selected_pose_dirty_ = false;
  }

  void handleWaypointGoal() {
    if (!have_waypoint_goal_) {
      return;
    }

    publishGoal(latest_waypoint_goal_, GoalSource::WAYPOINT, waypoint_goal_dirty_);
    waypoint_goal_dirty_ = false;
  }

  void publishGoal(const geometry_msgs::msg::PoseStamped &goal, GoalSource source,
                   bool force_publish) {
    if (!force_publish && last_source_ == source &&
        PoseNearlyEqual(goal, last_goal_pose_, pose_equal_eps_)) {
      return;
    }

    geometry_msgs::msg::PoseStamped msg = goal;
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      msg.header.stamp = now();
    }
    goal_pose_pub_->publish(msg);
    last_goal_pose_ = msg;
    last_source_ = source;
    goal_reached_ = false;
  }

  void setRotation(bool enabled) {
    if (rotation_state_.has_value() && rotation_state_.value() == enabled) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    rotation_pub_->publish(msg);
    rotation_state_ = enabled;
  }

  void publishStopGoal() {
    if (!have_odom_) {
      return;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = latest_odom_.header.frame_id.empty()
                               ? std::string("map")
                               : latest_odom_.header.frame_id;
    goal.pose = latest_odom_.pose.pose;
    publishGoal(goal, GoalSource::STOP, true);
  }

  void setPause(bool paused) {
    if (pause_state_.has_value() && pause_state_.value() == paused) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = paused;
    pause_pub_->publish(msg);
    pause_state_ = paused;
  }

  void publishState(BehaviorState state) {
    if (last_state_.has_value() && last_state_.value() == state) {
      return;
    }
    if (last_state_.has_value() && last_state_.value() != state) {
      // Only clear the selected pose when we are leaving AUTO_SELECTED.
      if (last_state_.value() == BehaviorState::AUTO_SELECTED &&
          state != BehaviorState::AUTO_SELECTED) {
        have_selected_pose_ = false;
        selected_pose_dirty_ = false;
      }
    }
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int>(state);
    state_pub_->publish(msg);
    std_msgs::msg::String name_msg;
    name_msg.data = StateName(state);
    state_name_pub_->publish(name_msg);
    last_state_ = state;
  }

  static std::string StateName(BehaviorState state) {
    switch (state) {
      case BehaviorState::MANUAL:
        return "MANUAL";
      case BehaviorState::ATTACK:
        return "ATTACK";
      case BehaviorState::AUTO_SELECTED:
        return "AUTO_SELECTED";
      case BehaviorState::AUTO_WAYPOINT:
        return "AUTO_WAYPOINT";
      case BehaviorState::AUTO_IDLE:
        return "AUTO_IDLE";
    }
    return "UNKNOWN";
  }

  // Parameters
  std::string manual_mode_topic_;
  std::string auto_point_select_topic_;
  std::string selected_pose_topic_;
  std::string enemy_detected_topic_;
  std::string goal_reached_topic_;
  std::string waypoint_goal_topic_;
  std::string goal_pose_topic_;
  std::string rotation_topic_;
  std::string state_topic_;
  std::string state_name_topic_;
  std::string pause_topic_;
  std::string odom_topic_;
  double publish_rate_hz_{10.0};
  double pose_equal_eps_{1e-3};

  // State
  bool manual_mode_{false};
  bool auto_point_select_{false};
  bool enemy_detected_{false};
  bool goal_reached_{false};
  bool have_selected_pose_{false};
  bool have_waypoint_goal_{false};
  bool selected_pose_dirty_{false};
  bool waypoint_goal_dirty_{false};
  GoalSource last_source_{GoalSource::NONE};
  geometry_msgs::msg::PoseStamped last_goal_pose_;
  geometry_msgs::msg::PoseStamped latest_selected_pose_;
  geometry_msgs::msg::PoseStamped latest_waypoint_goal_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};
  std::optional<bool> rotation_state_;
  std::optional<bool> pause_state_;
  std::optional<BehaviorState> last_state_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_point_select_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enemy_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rotation_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_name_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorSystemManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
