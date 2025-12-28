cmake_minimum_required(VERSION 3.5)
project(core_path_follower)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(path_follower_node src/path_follower_node.cpp src/interpolation.cpp)
ament_target_dependencies(path_follower_node rclcpp nav_msgs geometry_msgs)

install(TARGETS
    path_follower_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
