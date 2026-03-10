cmake_minimum_required(VERSION 3.14)
project(core_mppi)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/mppi_controller.cpp
  src/mppi_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch param
)
