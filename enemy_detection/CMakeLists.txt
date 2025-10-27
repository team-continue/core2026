cmake_minimum_required(VERSION 3.8)
project(core_enemy_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(backward_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


add_executable(enemy_detection
    src/enemy_detection_main.cpp
    src/enemy_detection.cpp
    src/image2camera_vector_angle.cpp
)

add_executable(enemy_selector
    src/enemy_selector_main.cpp
    src/enemy_selector.cpp
    src/image2camera_vector_angle.cpp
)

ament_target_dependencies(enemy_detection 
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    image_transport
    tf2_ros
)

ament_target_dependencies(enemy_selector
    rclcpp
    OpenCV
    tf2_ros
)

install(TARGETS
    enemy_detection
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    enemy_selector
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/config
    DESTINATION share/${PROJECT_NAME}
)

include_directories(${OpenCV_DIRS})
target_link_libraries(enemy_detection ${OpenCV_LIBS})
target_link_libraries(enemy_selector ${OpenCV_LIBS})

ament_package()
