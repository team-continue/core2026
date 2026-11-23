cmake_minimum_required(VERSION 3.8)
project(core_perception)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# launch の composition:=true で 1 つのプロセス（コンテナ）にまとめられるよう、ノードは共有ライブラリの
# component にする。静的ライブラリもそこに入るので、位置独立のコードにする
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(core_nav_common REQUIRED)
find_package(core_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/detection.cpp
  src/lidar_merge.cpp
  src/static_map.cpp
  src/tracker.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  Eigen3::Eigen
  core_nav_common::core_nav_common
  ${PCL_LIBRARIES}
)

# component のノード。同じ名前の実行ファイル（別のプロセスで動かすとき）は rclcpp_components が作る
add_library(${PROJECT_NAME}_nodes SHARED src/perception_node.cpp)
# 足りないリンクを、読み込むときではなくビルドで見つける
target_link_options(${PROJECT_NAME}_nodes PRIVATE -Wl,--no-undefined)
target_link_libraries(${PROJECT_NAME}_nodes
  ${PROJECT_NAME}_core
  ${geometry_msgs_TARGETS}
  ${core_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_perception::PerceptionNode" EXECUTABLE perception_node)

# 2 台の LiDAR の点群を時刻でそろえて合わせ、検出に渡す（別案3-1）
add_executable(lidar_merge_node src/lidar_merge_node.cpp)
target_link_libraries(lidar_merge_node
  ${PROJECT_NAME}_core
  ${livox_ros_driver2_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
)

install(
  TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS lidar_merge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detection test/test_detection.cpp)
  target_link_libraries(test_detection ${PROJECT_NAME}_core)
  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker ${PROJECT_NAME}_core)
  ament_add_gtest(test_lidar_merge test/test_lidar_merge.cpp)
  target_link_libraries(test_lidar_merge ${PROJECT_NAME}_core)
  ament_add_gtest(test_pose_timing test/test_pose_timing.cpp)
  target_link_libraries(test_pose_timing ${PROJECT_NAME}_core)
  ament_add_gtest(test_track_frame test/test_track_frame.cpp)
  target_link_libraries(test_track_frame ${PROJECT_NAME}_core)
endif()

ament_package()
