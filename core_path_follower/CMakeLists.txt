cmake_minimum_required(VERSION 3.14)
project(core_path_follower)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- library (controller + interpolation, reusable / testable) ---
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/controller.cpp
  src/interpolation.cpp
)

# --- executable (node entry-point + ROS wrapper) ---
ament_auto_add_executable(${PROJECT_NAME}_node
  src/node.cpp
  src/path_follower_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

# --- Python test helper scripts ---
install(PROGRAMS
  test/test_path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_path_publisher
)

# --- tests ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch param schema
)
