cmake_minimum_required(VERSION 3.8)
project(core_shooter)

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

set(CORE_SHOOTER_NODES
  shooter_cmd_gate
  shooter_controller
  magazine_manager
  aim_bot
)

foreach(node ${CORE_SHOOTER_NODES})
  ament_auto_add_executable(${node}
    src/${node}.cpp
  )
  target_include_directories(${node} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endforeach()

install(
  PROGRAMS
    scripts/shooter_debug_topic_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shooter_debug_topic_gui
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
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

ament_auto_package(
  INSTALL_TO_SHARE
  USE_SCOPED_HEADER_INSTALL_DIR
)
