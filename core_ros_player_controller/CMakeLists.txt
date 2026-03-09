cmake_minimum_required(VERSION 3.8)
project(core_ros_player_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wireless_parser_node src/main.cpp)
ament_target_dependencies(wireless_parser_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  wireless_parser_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_wireless_parser_launch.py)
endif()

ament_package()
