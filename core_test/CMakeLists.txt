cmake_minimum_required(VERSION 3.8)
project(core_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include(cmake/CoreTest.cmake)

if(BUILD_TESTING)
  # テスト対象パッケージのヘッダを使うための依存。BUILD_TESTING=OFF の
  # ビルドにまで core_shooter を要求しないよう、この中で解決する。
  find_package(core_shooter REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)

  core_add_gtest(test_core_smoke test/test_core_smoke.cpp)

  # core_shooter の曲射弾道テーブル（ヘッダオンリー）を ROS 起動なしで検証する。
  core_add_gtest(test_ballistics test/test_ballistics.cpp)
  if(TARGET test_ballistics)
    ament_target_dependencies(test_ballistics core_shooter)
    target_link_libraries(test_ballistics ${YAML_CPP_LIBRARIES})
    # 配布する ballistics_*.yaml をそのまま読んで妥当性を確認するため、
    # ソースツリー上の config ディレクトリを渡す。
    target_compile_definitions(test_ballistics PRIVATE
      CORE_SHOOTER_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../core_shooter/config")
  endif()

  # 砲塔の可動包絡線（ROS非依存のヘッダ）を検証する。
  core_add_gtest(test_turret_envelope test/test_turret_envelope.cpp)
  if(TARGET test_turret_envelope)
    ament_target_dependencies(test_turret_envelope core_shooter)
    target_link_libraries(test_turret_envelope ${YAML_CPP_LIBRARIES})
    target_compile_definitions(test_turret_envelope PRIVATE
      CORE_SHOOTER_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../core_shooter/config")
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
