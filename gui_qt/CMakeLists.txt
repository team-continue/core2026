cmake_minimum_required(VERSION 3.8)
project(gui_qt)

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# find_package(yaml_cpp_vendor REQUIRED)
# find_package(yaml-cpp REQUIRED)
# find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)

# find dependencies for Qt
find_package(Qt5 REQUIRED COMPONENTS Widgets)

ament_auto_find_build_dependencies()

# =====================================
# build gui_qt
# =====================================
ament_auto_add_executable(gui_qt
  src/main.cpp
  src/HUDManager.cpp
  src/HUDNode.cpp
  src/HUDCamera.cpp
  src/HUDCompass.cpp
  src/HUDHPPanel.cpp
  src/HUDAmmoPanel.cpp
  src/HUDQuadrantElevation.cpp
  src/HUDSpeed.cpp
  src/HUDLargeMsg.cpp
  src/HUDReticle.cpp
  src/HUDHazard.cpp
  src/MainWindow.cpp
  src/ToggleSwitch.cpp
  src/SettingMenu.cpp
)

ament_target_dependencies(
  gui_qt
  "Qt5Widgets"
)

install(TARGETS gui_qt
  DESTINATION lib/${PROJECT_NAME}
)

# =====================================
# build hardware_ui_converter_node
# =====================================
ament_auto_add_executable(hardware_ui_converter_node
  src/HardwareUIConverterNode.cpp
)

install(TARGETS hardware_ui_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# =====================================
# others
# =====================================
install(DIRECTORY
  font
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()