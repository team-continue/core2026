#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class WirelessParserNode : public rclcpp::Node
{
public:
  WirelessParserNode()
  : Node("wireless_parser_node")
  {
    subscription_ = create_subscription<std_msgs::msg::String>(
      "/wireless", 10,
      std::bind(&WirelessParserNode::wireless_callback, this, std::placeholders::_1));
    parsed_publisher_ = create_publisher<std_msgs::msg::String>("/wireless/parsed", 10);
    
    RCLCPP_INFO(get_logger(), "Subscribed: /wireless (std_msgs/msg/String), Publish: /wireless/parsed (std_msgs/msg/String)");
  }

private:
  std::vector<std::string> split_comma(const std::string & input)
  {
    std::vector<std::string> tokens;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }
    return tokens;
  }

  std::vector<uint8_t> convert_str_to_uint8(std::vector<std::string> strs) {
    std::vector<uint8_t> result;
    // Example:
    // 00,0401,D1:01,91,83,91,89,00,00,00
    for (size_t i = 3; i < strs.size(); ++i) {
        int tmp = std::stoi(strs[i]);

        // Check value range
        if (tmp < 0 || tmp > 255) {
            if (tmp < 0) tmp = 0;
            if (tmp > 255) tmp = 255;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "value out of range.");
        }
        uint8_t value = tmp;
        result.push_back(value);
    }
    return result;
  }

  void wireless_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto tokens = split_comma(msg->data);

    std::vector<uint8_t> values;
    // ex) 91,83,91,89,00,00,00
    values = convert_str_to_uint8(tokens);
    if (values.size() < 7) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "values.size() > 7");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d, %d, %d, %d", values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parsed_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WirelessParserNode>());
  rclcpp::shutdown();
  return 0;
}
