cmake_minimum_required(VERSION 3.10)
project(gz_omnidrive_plugin)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_library(gz_omnidrive_plugin SHARED
  src/OmniDrive.cc
)

ament_target_dependencies(gz_omnidrive_plugin
  ignition-gazebo6
  ignition-transport8
  ignition-msgs8
)

install(TARGETS gz_omnidrive_plugin
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()