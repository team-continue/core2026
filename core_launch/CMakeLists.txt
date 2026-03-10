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
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_server_node.py
  scripts/odom_bridge_node.py
  scripts/start_navigation_fastlio.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install global map image if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../square_wall_10m_400px.png")
  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../square_wall_10m_400px.png"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install curious-house map image if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../curious_house.png")
  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../curious_house.png"
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
