cmake_minimum_required(VERSION 3.8)
project(core_path_planner)

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

# Library containing the planner algorithm and node implementation
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/path_planner.cpp
  src/path_planner_node.cpp
  src/costmap_publisher_node.cpp
)

# Executable that runs the path planner node
ament_auto_add_executable(path_planner_node
  src/node.cpp
)
target_link_libraries(path_planner_node ${PROJECT_NAME}_lib)

# Executable that runs the costmap publisher node
ament_auto_add_executable(costmap_publisher_node
  src/costmap_publisher_main.cpp
)
target_link_libraries(costmap_publisher_node ${PROJECT_NAME}_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)