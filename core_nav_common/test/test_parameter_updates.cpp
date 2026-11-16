#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "core_nav_common/parameter_updates.hpp"

namespace
{
using core_nav_common::ParameterUpdateHandle;

class ParameterUpdatesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  // 呼ばれた時の、変わったパラメータの名前と get_parameter の値
  struct Seen
  {
    std::vector<std::string> names;
    double value{0.0};
    int calls{0};
  };

  static void spinUntil(rclcpp::Node::SharedPtr node, const Seen & seen, const int calls)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (seen.calls < calls && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(10));
    }
  }

  static ParameterUpdateHandle record(
    rclcpp::Node & node, Seen & seen,
    ParameterUpdateHandle (* registerFn)(rclcpp::Node &, core_nav_common::ParameterUpdateCallback))
  {
    return registerFn(
      node, [&node, &seen](const std::vector<rclcpp::Parameter> & parameters) {
        seen.names.clear();
        for (const auto & parameter : parameters) {
          seen.names.push_back(parameter.get_name());
        }
        seen.value = node.get_parameter("gain").as_double();
        ++seen.calls;
      });
  }
};

TEST_F(ParameterUpdatesTest, CallsAfterValueIsSet)
{
  auto node = std::make_shared<rclcpp::Node>("parameter_updates_post");
  node->declare_parameter<double>("gain", 1.0);
  Seen seen;
  const auto handle = record(*node, seen, &core_nav_common::onParametersSet);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("gain", 2.5)).successful);
  spinUntil(node, seen, 1);
  EXPECT_EQ(seen.calls, 1);
  EXPECT_EQ(seen.names, std::vector<std::string>{"gain"});
  EXPECT_DOUBLE_EQ(seen.value, 2.5);
}

// Humble で使う、/parameter_events を受ける方法も、この版で確かめる
TEST_F(ParameterUpdatesTest, EventsCallAfterValueIsSetAndIgnoreDeclarations)
{
  auto node = std::make_shared<rclcpp::Node>("parameter_updates_events");
  node->declare_parameter<double>("gain", 1.0);
  Seen seen;
  const auto handle = record(*node, seen, &core_nav_common::detail::onParametersSetByEvents);
  // 宣言では呼ばない
  node->declare_parameter<double>("other", 0.0);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("gain", 3.5)).successful);
  spinUntil(node, seen, 1);
  EXPECT_EQ(seen.calls, 1);
  EXPECT_EQ(seen.names, std::vector<std::string>{"gain"});
  EXPECT_DOUBLE_EQ(seen.value, 3.5);
}

TEST_F(ParameterUpdatesTest, EventsIgnoreOtherNodes)
{
  auto node = std::make_shared<rclcpp::Node>("parameter_updates_mine");
  auto other = std::make_shared<rclcpp::Node>("parameter_updates_other");
  node->declare_parameter<double>("gain", 1.0);
  other->declare_parameter<double>("gain", 1.0);
  Seen seen;
  const auto handle = record(*node, seen, &core_nav_common::detail::onParametersSetByEvents);
  ASSERT_TRUE(other->set_parameter(rclcpp::Parameter("gain", 4.0)).successful);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("gain", 5.0)).successful);
  spinUntil(node, seen, 1);
  EXPECT_EQ(seen.calls, 1);
  EXPECT_DOUBLE_EQ(seen.value, 5.0);
}

// 登録を手放したら呼ばない
TEST_F(ParameterUpdatesTest, EventsStopWhenHandleIsReleased)
{
  auto node = std::make_shared<rclcpp::Node>("parameter_updates_release");
  node->declare_parameter<double>("gain", 1.0);
  Seen seen;
  auto handle = record(*node, seen, &core_nav_common::detail::onParametersSetByEvents);
  handle.reset();
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("gain", 6.0)).successful);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
  while (std::chrono::steady_clock::now() < until) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(seen.calls, 0);
}
}  // namespace
