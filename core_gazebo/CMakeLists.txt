cmake_minimum_required(VERSION 3.8)
project(core_gazebo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz-sim REQUIRED)
find_package(gz-transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(core_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(omni_force_drive SHARED src/omni_force_drive.cpp)
target_compile_features(omni_force_drive PRIVATE cxx_std_17)
target_include_directories(omni_force_drive PRIVATE include)
target_link_libraries(omni_force_drive PRIVATE
  gz-math::core
  gz-msgs::core
  gz-plugin::register
  gz-sim::gz-sim
  gz-transport::core
)

add_library(moving_obstacles SHARED src/moving_obstacles.cpp)
target_compile_features(moving_obstacles PRIVATE cxx_std_17)
target_link_libraries(moving_obstacles PRIVATE
  gz-math::core
  gz-plugin::register
  gz-sim::gz-sim
  yaml-cpp
)

# launch の composition:=true で検出と同じプロセス（コンテナ）に入れられるよう、component にする。
# 同じ名前の実行ファイル（別のプロセスで動かすとき）は rclcpp_components が作る
add_library(${PROJECT_NAME}_nodes SHARED
  src/points_to_livox.cpp src/sim_hardware.cpp src/wheel_odometry_sim.cpp)
target_compile_features(${PROJECT_NAME}_nodes PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME}_nodes PRIVATE include)
# 足りないリンクを、読み込むときではなくビルドで見つける
target_link_options(${PROJECT_NAME}_nodes PRIVATE -Wl,--no-undefined)
target_link_libraries(${PROJECT_NAME}_nodes
  Eigen3::Eigen
  ${core_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_gazebo::PointsToLivox" EXECUTABLE points_to_livox)
# 実機の足回りが返す車輪の速度の代わり（EKF の自己位置、launch の state_source:=ekf で使う）
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_gazebo::WheelOdometrySim" EXECUTABLE wheel_odometry_sim)
# 足回りと腰のハードウェアの代わり（launch の waist:=true。body_controller の CAN の指令で Gazebo を動かす）
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_gazebo::SimHardware" EXECUTABLE sim_hardware)

install(TARGETS moving_obstacles omni_force_drive LIBRARY DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch models rviz scenarios urdf worlds DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/build_truth_map.py
    scripts/check_sim.py
    scripts/make_core1_field_model.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_motion test/test_scan_motion.cpp)
  target_include_directories(test_scan_motion PRIVATE include)
  target_link_libraries(test_scan_motion Eigen3::Eigen)
  ament_add_gtest(test_wheel_odometry_model test/test_wheel_odometry_model.cpp)
  target_include_directories(test_wheel_odometry_model PRIVATE include)
  ament_add_gtest(test_slip_model test/test_slip_model.cpp)
  target_include_directories(test_slip_model PRIVATE include)
  ament_add_gtest(test_command_delay test/test_command_delay.cpp)
  target_include_directories(test_command_delay PRIVATE include)
  ament_add_gtest(test_sim_hardware_model test/test_sim_hardware_model.cpp)
  target_include_directories(test_sim_hardware_model PRIVATE include)
endif()

ament_package()
