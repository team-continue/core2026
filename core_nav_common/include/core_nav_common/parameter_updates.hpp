// パラメータが変わった後に呼ぶ処理を登録する共通処理。
//
// Iron 以降の rclcpp は add_post_set_parameters_callback で登録する。Humble（rclcpp 16）にはないので、
// 自分のノードのパラメータ変更の通知（/parameter_events）を受けて呼ぶ。どちらでも、呼ばれた時点で
// get_parameter は新しい値を返す。

#ifndef CORE_NAV_COMMON__PARAMETER_UPDATES_HPP_
#define CORE_NAV_COMMON__PARAMETER_UPDATES_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

namespace core_nav_common
{
// 登録を保つ間だけ呼ばれる。中身は rclcpp の版で違うので、持つだけにする
using ParameterUpdateHandle = std::shared_ptr<void>;
using ParameterUpdateCallback = std::function<void (const std::vector<rclcpp::Parameter> &)>;

namespace detail
{
// /parameter_events で、このノードの値が変わったときに呼ぶ（Humble 用。試験のため、どの版でも使える）。
// 宣言（new_parameters）では呼ばない。add_post_set_parameters_callback と違い、値を変えた要求の応答の後に、
// ノードの実行器で呼ばれる
inline ParameterUpdateHandle onParametersSetByEvents(
  rclcpp::Node & node, ParameterUpdateCallback callback)
{
  struct Registration
  {
    std::shared_ptr<rclcpp::ParameterEventHandler> handler;
    rclcpp::ParameterEventCallbackHandle::SharedPtr callback;
  };
  auto registration = std::make_shared<Registration>();
  registration->handler = std::make_shared<rclcpp::ParameterEventHandler>(&node);
  registration->callback = registration->handler->add_parameter_event_callback(
    [node_name = std::string(node.get_fully_qualified_name()),
    callback = std::move(callback)](const rcl_interfaces::msg::ParameterEvent & event) {
      if (event.node != node_name || event.changed_parameters.empty()) {
        return;
      }
      std::vector<rclcpp::Parameter> parameters;
      parameters.reserve(event.changed_parameters.size());
      for (const auto & parameter : event.changed_parameters) {
        parameters.push_back(rclcpp::Parameter::from_parameter_msg(parameter));
      }
      callback(parameters);
    });
  return registration;
}
}  // namespace detail

inline ParameterUpdateHandle onParametersSet(rclcpp::Node & node, ParameterUpdateCallback callback)
{
#if RCLCPP_VERSION_MAJOR >= 21
  return node.add_post_set_parameters_callback(std::move(callback));
#else
  return detail::onParametersSetByEvents(node, std::move(callback));
#endif
}
}  // namespace core_nav_common

#endif  // CORE_NAV_COMMON__PARAMETER_UPDATES_HPP_
