cmake_minimum_required(VERSION 3.8)
project(core_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 自律移動の確認スクリプトの共通処理
ament_python_install_package(core_nav_check)
# 自律移動の launch で共通に使う処理（NodeSpec と composition、IMU の選択など）
ament_python_install_package(core_nav_launch)

install(DIRECTORY
  launch
  urdf
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/navigation.sh
  scripts/check_lio.py
  scripts/check_ekf.py
  scripts/check_localization.py
  scripts/check_localization_jumps.py
  scripts/check_localization_stress.py
  scripts/convert_livox_pointcloud_bag.py
  scripts/record_localization_bag.sh
  scripts/replay_localization.py
  scripts/check_features.py
  scripts/check_long_run.py
  scripts/check_perception.py
  scripts/check_planning.py
  scripts/run_scenario.py
  scripts/setup_thirdparty.sh
  scripts/map_publisher_node.py
  scripts/localization_status_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # 自律移動の確認スクリプトは flake8 の行の長さが合わないので、pytest だけで確かめる
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # launch の補助（自機の除去の箱と、URDF の LiDAR の取り付けの変換）
  ament_add_pytest_test(test_self_filter test/test_self_filter.py)
  # 点群抽出のモードと観客席の箱
  ament_add_pytest_test(test_point_extraction_launch test/test_point_extraction_launch.py)
  # LIO に入れる IMU の選択と外部パラメータ（Damiao は URDF から求める）
  ament_add_pytest_test(test_imu_source test/test_imu_source.py)
  # run_scenario.py の静止物との接触の判定
  ament_add_pytest_test(test_run_scenario_static test/test_run_scenario_static.py)
  # ノードを別のプロセスにするか、コンテナにまとめるか（composition）と、確認スクリプトのプロセスの探し方
  ament_add_pytest_test(test_composition test/test_composition.py)
  # 自己位置の跳びの指標（map→odom の段差、動きの区分ごとの集計）
  ament_add_pytest_test(test_jumps test/test_jumps.py)
  # Livox の PointCloud2（xfer_format 0）を CustomMsg の値にする計算
  ament_add_pytest_test(test_livox_bag test/test_livox_bag.py)
  # 自己位置の出し方（state_source:=ekf の gyro_odometer、ekf_localizer、融合した IMU）
  ament_add_pytest_test(test_state_source test/test_state_source.py)
  # 起動した地図の占有地図（map_publisher_node.py）
  ament_add_pytest_test(test_occupancy_map test/test_occupancy_map.py)
  # RViz に出す自己位置の状態の文字と色（localization_status_marker_node.py）
  ament_add_pytest_test(test_localization_status test/test_localization_status.py)
endif()

ament_package()
