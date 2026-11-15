cmake_minimum_required(VERSION 3.8)
project(core_body_controller)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(backward_ros REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}
  SHARED
  src/joy_cmd_parser.cpp
  include/core_body_controller/joy_cmd_parser.hpp
)

ament_auto_add_executable(body_control_node
  src/body_control_node.cpp
)

ament_auto_add_executable(temp_joy_parser
  src/temp_joy_parser.cpp
)

ament_auto_add_executable(target_angle_node
  src/target_angle_node.cpp
)

# 手動と自動の運転モードを持ち、今のモードの速度指令だけを /control/cmd_vel へ通す
ament_auto_add_executable(control_mode_manager_node
  src/control_mode_manager_node.cpp
)

# 腰の角度に関節名を付けて出す（robot_state_publisher が base_link → chassis_link の TF を出す）
ament_auto_add_executable(chassis_joint_state_node
  src/chassis_joint_state_node.cpp
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_target_angle_controller
    test/test_target_angle_controller.cpp
  )
  target_include_directories(test_target_angle_controller PRIVATE include)

  ament_add_gtest(test_control_mode test/test_control_mode.cpp)
  target_include_directories(test_control_mode PRIVATE include)

  add_launch_test(test/test_body_controller_launch.py TIMEOUT 45)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_backward(body_control_node)
# add_backward(velocity_publisher_node)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
