cmake_minimum_required(VERSION 3.5)
project(core_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CAN.msg"
  "msg/CANArray.msg"
  "msg/DamagePanelInfo.msg"
  "msg/DamagePanelInfoArray.msg"
  "msg/PoseWithWeight.msg"
  "msg/Path.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

ament_package()
