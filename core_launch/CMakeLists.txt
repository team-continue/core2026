cmake_minimum_required(VERSION 3.8)
project(core_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_server_node.py
  scripts/odom_bridge_node.py
  scripts/navigation.sh
  scripts/debug_log_recorder.py
  scripts/debug_log_timeline.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_debug_log test/test_debug_log.py TIMEOUT 120)
endif()

ament_package()
