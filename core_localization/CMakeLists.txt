cmake_minimum_required(VERSION 3.8)
project(core_localization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# launch の composition:=true で 1 つのプロセス（コンテナ）にまとめられるよう、ノードは共有ライブラリの
# component にする。静的ライブラリもそこに入るので、位置独立のコードにする
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(core_nav_common REQUIRED)
find_package(core_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# 点群抽出の計算（ceiling の高さの帯と観客席の箱は crowd_filter.cpp、モードの切り替えと field_objects は
# point_extraction.cpp）。ROS に依存しないので、ノードと分けてテストする
add_library(${PROJECT_NAME}_crowd_filter STATIC src/crowd_filter.cpp)
target_compile_features(${PROJECT_NAME}_crowd_filter PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_crowd_filter PUBLIC include)
target_link_libraries(${PROJECT_NAME}_crowd_filter PUBLIC Eigen3::Eigen)

add_library(${PROJECT_NAME}_point_extraction STATIC src/point_extraction.cpp)
target_link_libraries(${PROJECT_NAME}_point_extraction PUBLIC ${PROJECT_NAME}_crowd_filter)

# scan_filter の点群抽出のパラメータ、TF、追跡の扱い
add_library(${PROJECT_NAME}_point_extraction_stage STATIC src/point_extraction_stage.cpp)
target_link_libraries(${PROJECT_NAME}_point_extraction_stage PUBLIC
  ${PROJECT_NAME}_point_extraction
  core_nav_common::core_nav_common
  ${core_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
)

# 車輪の速度とジャイロの合わせ方（gyro_odometer）と、NDT の姿勢との EKF（ekf_localizer）。
# ROS に依存しないので、ノードと分けてテストする
add_library(${PROJECT_NAME}_gyro_odometer STATIC src/gyro_odometer.cpp)
target_compile_features(${PROJECT_NAME}_gyro_odometer PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_gyro_odometer PUBLIC include)

add_library(${PROJECT_NAME}_ekf_localizer STATIC src/ekf_localizer.cpp)
target_compile_features(${PROJECT_NAME}_ekf_localizer PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_ekf_localizer PUBLIC include)
target_link_libraries(${PROJECT_NAME}_ekf_localizer PUBLIC Eigen3::Eigen)

# component のノード。同じ名前の実行ファイル（別のプロセスで動かすとき）は rclcpp_components が作る
add_library(${PROJECT_NAME}_nodes SHARED
  src/ekf_localizer_node.cpp
  src/gyro_odometer_node.cpp
  src/lio_odom_adapter.cpp
  src/localization_health.cpp
  src/scan_filter.cpp
)
target_include_directories(${PROJECT_NAME}_nodes PRIVATE include)
target_compile_features(${PROJECT_NAME}_nodes PRIVATE cxx_std_17)
# 足りないリンクを、読み込むときではなくビルドで見つける
target_link_options(${PROJECT_NAME}_nodes PRIVATE -Wl,--no-undefined)
target_link_libraries(${PROJECT_NAME}_nodes
  ${PROJECT_NAME}_ekf_localizer
  ${PROJECT_NAME}_gyro_odometer
  ${PROJECT_NAME}_point_extraction_stage
  ${diagnostic_msgs_TARGETS}
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  core_nav_common::core_nav_common
  ${core_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_localization::LioOdomAdapter" EXECUTABLE lio_odom_adapter)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_localization::ScanFilter" EXECUTABLE scan_filter)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_localization::LocalizationHealthNode" EXECUTABLE localization_health)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_localization::GyroOdometerNode" EXECUTABLE gyro_odometer)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_localization::EkfLocalizerNode" EXECUTABLE ekf_localizer)

add_executable(livox_filter src/livox_filter.cpp)
target_include_directories(livox_filter PRIVATE include)
target_compile_features(livox_filter PRIVATE cxx_std_17)
target_link_libraries(livox_filter
  core_nav_common::core_nav_common
  ${core_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
  rclcpp::rclcpp
)

# 2 つの IMU の融合の計算。ROS に依存しないので、ノードと分けてテストする
add_library(${PROJECT_NAME}_imu_fusion STATIC src/imu_fusion.cpp)
target_compile_features(${PROJECT_NAME}_imu_fusion PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_imu_fusion PUBLIC include)
target_link_libraries(${PROJECT_NAME}_imu_fusion PUBLIC Eigen3::Eigen)

add_executable(imu_fusion src/imu_fusion_node.cpp)
target_link_libraries(imu_fusion
  ${PROJECT_NAME}_imu_fusion
  core_nav_common::core_nav_common
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
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
install(TARGETS livox_filter imu_fusion DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_fusion test/test_imu_fusion.cpp)
  target_link_libraries(test_imu_fusion ${PROJECT_NAME}_imu_fusion)
  ament_add_gtest(test_crowd_filter test/test_crowd_filter.cpp)
  target_link_libraries(test_crowd_filter ${PROJECT_NAME}_crowd_filter)
  ament_add_gtest(test_point_extraction test/test_point_extraction.cpp)
  target_link_libraries(test_point_extraction ${PROJECT_NAME}_point_extraction)
  # 自己位置の異常の判定（照合を捨てた回数、跳び、角速度の差の上限）
  ament_add_gtest(test_health_policy test/test_health_policy.cpp)
  target_include_directories(test_health_policy PRIVATE include)
  ament_add_gtest(test_gyro_odometer test/test_gyro_odometer.cpp)
  target_link_libraries(test_gyro_odometer ${PROJECT_NAME}_gyro_odometer)
  ament_add_gtest(test_ekf_localizer test/test_ekf_localizer.cpp)
  target_link_libraries(test_ekf_localizer ${PROJECT_NAME}_ekf_localizer)
endif()

ament_package()
