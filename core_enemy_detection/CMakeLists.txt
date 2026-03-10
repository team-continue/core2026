cmake_minimum_required(VERSION 3.8)
project(core_enemy_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(backward_ros REQUIRED)
# find_package(core_msgs REQUIRED)

# Auto
find_package(ament_cmake_auto REQUIRED)
# Manual
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()



ament_auto_add_executable(target_detector
    src/target_detector_main.cpp
    src/target_detector.cpp
)

ament_auto_add_executable(target_selector
    src/target_selector_main.cpp
    src/target_selector.cpp
)

ament_target_dependencies(target_detector
    OpenCV
)

ament_target_dependencies(target_selector
    OpenCV
)

install(TARGETS
    target_detector
    target_selector
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/launch
    ${PROJECT_SOURCE_DIR}/config
    DESTINATION share/${PROJECT_NAME}
)

include_directories(${OpenCV_DIRS})

target_link_libraries(target_detector ${OpenCV_LIBS})
target_link_libraries(target_selector ${OpenCV_LIBS})

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

ament_package()
