cmake_minimum_required(VERSION 3.8)
project(core_planning)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# launch の composition:=true で 1 つのプロセス（コンテナ）にまとめられるよう、ノードは共有ライブラリの
# component にする。静的ライブラリもそこに入るので、位置独立のコードにする
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(core_nav_common REQUIRED)
find_package(core_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/drive_delay.cpp
  src/grid_map.cpp
  src/hybrid_astar.cpp
  src/jps.cpp
  src/mppi.cpp
  src/path_smoother.cpp
  src/safety.cpp
  src/traction_estimator.cpp
  src/yaw_hold.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  core_nav_common::core_nav_common
  ${PCL_LIBRARIES}
)

# component のノード。同じ名前の実行ファイル（別のプロセスで動かすとき）は rclcpp_components が作る
add_library(${PROJECT_NAME}_nodes SHARED
  src/cmd_yaw_hold_node.cpp
  src/contact_monitor_node.cpp
  src/global_planner_node.cpp
  src/mppi_lite_node.cpp
  src/safety_gate_node.cpp
)
# 足りないリンクを、読み込むときではなくビルドで見つける
target_link_options(${PROJECT_NAME}_nodes PRIVATE -Wl,--no-undefined)
target_link_libraries(${PROJECT_NAME}_nodes
  ${PROJECT_NAME}_core
  ${geometry_msgs_TARGETS}
  ${core_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_planning::GlobalPlannerNode" EXECUTABLE global_planner)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_planning::MppiLiteNode" EXECUTABLE mppi_lite)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_planning::SafetyGateNode" EXECUTABLE safety_gate)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_planning::ContactMonitorNode" EXECUTABLE contact_monitor)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "core_planning::CmdYawHoldNode" EXECUTABLE cmd_yaw_hold)

# 接触の頻度を Gazebo を使わずに統計で比べるコマンド。ノードは yaml-cpp を使わないので分ける
add_library(${PROJECT_NAME}_crowd_model STATIC src/crowd_model.cpp)
target_link_libraries(${PROJECT_NAME}_crowd_model PUBLIC ${PROJECT_NAME}_core yaml-cpp)

add_executable(crowd_stats src/crowd_stats.cpp)
target_link_libraries(crowd_stats
  ${PROJECT_NAME}_crowd_model
  ament_index_cpp::ament_index_cpp
  Threads::Threads
)

install(
  TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS crowd_stats DESTINATION lib/${PROJECT_NAME})

# 経路の追従（はみ出し）を、機体の滑りを含めて Gazebo を使わずに比べるコマンド
add_executable(tracking_bench src/tracking_bench.cpp)
target_link_libraries(tracking_bench
  ${PROJECT_NAME}_crowd_model
  ament_index_cpp::ament_index_cpp
  Threads::Threads
)
install(TARGETS tracking_bench DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(name test_contact_detection test_drive_delay test_footprint test_jps test_keep_out test_mppi
    test_path_smoother test_safety
    test_traction_estimator
    test_tracks_sync
    test_yaw_hold
    test_hybrid_astar)
    ament_add_gtest(${name} test/${name}.cpp)
    target_link_libraries(${name} ${PROJECT_NAME}_core)
  endforeach()
  ament_add_gtest(test_crowd_model test/test_crowd_model.cpp)
  target_link_libraries(test_crowd_model ${PROJECT_NAME}_crowd_model)
endif()

ament_package()
