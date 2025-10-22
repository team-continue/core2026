#include "core_body_controller/joy_cmd_parser.hpp"

JoyCmdParser::JoyCmdParser() {}

void JoyCmdParser::parse(const std_msgs::msg::ByteMultiArray &msg) {
  // remove first few elements of the array
  constexpr size_t num_ignored_elements = 4;
  constexpr size_t num_required_elements = 7;

  if (msg.data.size() < num_required_elements + num_ignored_elements) {
    throw std::runtime_error("Not enough elements in the array");
  }

  std::vector<float> data(msg.data.begin() + num_ignored_elements,
                          msg.data.end());
  current_twist_.linear.x = data[0];
  current_twist_.linear.y = data[1];

  current_pitch_ = data[2];

  current_twist_.angular.z = data[3];

  current_buttons_.clear();
  // each bit in the byte represents a button
  for (size_t i = 0; i < 8; i++) {
    current_buttons_.push_back((msg.data[3] >> i) & 1);
  }
  // also for data[4] and data[5]
  for (size_t i = 0; i < 8; i++) {
    current_buttons_.push_back((msg.data[4] >> i) & 1);
  }
  for (size_t i = 0; i < 8; i++) {
    current_buttons_.push_back((msg.data[5] >> i) & 1);
  }
}