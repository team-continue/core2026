cmake_minimum_required(VERSION 3.8)
project(core_behavior_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(waypoint_selector src/waypoint_selector.cpp)
target_include_directories(waypoint_selector PUBLIC include)
target_compile_features(waypoint_selector PUBLIC cxx_std_17)

add_executable(waypoint_selector_node src/behavior_system.cpp)
ament_target_dependencies(waypoint_selector_node rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs tf2 tf2_geometry_msgs)
target_link_libraries(waypoint_selector_node waypoint_selector)
target_compile_features(waypoint_selector_node PUBLIC cxx_std_17)

add_executable(behavior_system_node src/behavior_system_manager.cpp)
ament_target_dependencies(behavior_system_node rclcpp)
target_compile_features(behavior_system_node PUBLIC cxx_std_17)

install(TARGETS waypoint_selector waypoint_selector_node behavior_system_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(waypoint_selector)

ament_package()
