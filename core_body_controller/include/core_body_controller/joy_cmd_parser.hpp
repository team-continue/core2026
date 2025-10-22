#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

class JoyCmdParser {
 public:
  JoyCmdParser();
  void parse(const std_msgs::msg::ByteMultiArray &msg);
  const std::vector<bool> &get_buttons() const { return current_buttons_; }
  const geometry_msgs::msg::Twist &get_twist() const { return current_twist_; }
  const double &get_pitch() const { return current_pitch_; }

 private:
  std::vector<bool> current_buttons_;
  geometry_msgs::msg::Twist current_twist_;
  double current_pitch_;
};
