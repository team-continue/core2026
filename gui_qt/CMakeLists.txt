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
# build gui_qt (split compilation by widget/source)
# =====================================
add_library(gui_qt_main OBJECT src/main.cpp)
add_library(gui_qt_hud_manager OBJECT src/HUDManager.cpp)
add_library(gui_qt_hud_node OBJECT src/HUDNode.cpp)
add_library(gui_qt_hud_camera OBJECT src/HUDCamera.cpp)
add_library(gui_qt_hud_compass OBJECT src/HUDCompass.cpp)
add_library(gui_qt_hud_hp_panel OBJECT src/HUDHPPanel.cpp)
add_library(gui_qt_hud_ammo_panel OBJECT src/HUDAmmoPanel.cpp)
add_library(gui_qt_hud_quadrant_elevation OBJECT src/HUDQuadrantElevation.cpp)
add_library(gui_qt_hud_speed OBJECT src/HUDSpeed.cpp)
add_library(gui_qt_hud_large_msg OBJECT src/HUDLargeMsg.cpp)
add_library(gui_qt_hud_reticle OBJECT src/HUDReticle.cpp)
add_library(gui_qt_hud_hazard OBJECT src/HUDHazard.cpp)
add_library(gui_qt_hud_debug_overlay OBJECT src/HUDDebugOverlay.cpp)
add_library(gui_qt_main_window OBJECT src/MainWindow.cpp)
add_library(gui_qt_toggle_switch OBJECT src/ToggleSwitch.cpp)
add_library(gui_qt_setting_menu OBJECT src/SettingMenu.cpp)

add_library(gui_qt_deps STATIC src/Deps.cpp)
ament_target_dependencies(gui_qt_deps
  "Qt5Widgets"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "ament_index_cpp"
  "yaml_cpp_vendor"
)
target_include_directories(gui_qt_deps PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

set(GUI_QT_OBJECT_LIBS
  gui_qt_main
  gui_qt_hud_manager
  gui_qt_hud_node
  gui_qt_hud_camera
  gui_qt_hud_compass
  gui_qt_hud_hp_panel
  gui_qt_hud_ammo_panel
  gui_qt_hud_quadrant_elevation
  gui_qt_hud_speed
  gui_qt_hud_large_msg
  gui_qt_hud_reticle
  gui_qt_hud_hazard
  gui_qt_hud_debug_overlay
  gui_qt_main_window
  gui_qt_toggle_switch
  gui_qt_setting_menu
)

foreach(obj_lib IN LISTS GUI_QT_OBJECT_LIBS)
  target_link_libraries(${obj_lib} gui_qt_deps)
endforeach()

add_executable(gui_qt)
target_link_libraries(gui_qt gui_qt_deps)
target_include_directories(gui_qt PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(gui_qt
  $<TARGET_OBJECTS:gui_qt_main>
  $<TARGET_OBJECTS:gui_qt_hud_manager>
  $<TARGET_OBJECTS:gui_qt_hud_node>
  $<TARGET_OBJECTS:gui_qt_hud_camera>
  $<TARGET_OBJECTS:gui_qt_hud_compass>
  $<TARGET_OBJECTS:gui_qt_hud_hp_panel>
  $<TARGET_OBJECTS:gui_qt_hud_ammo_panel>
  $<TARGET_OBJECTS:gui_qt_hud_quadrant_elevation>
  $<TARGET_OBJECTS:gui_qt_hud_speed>
  $<TARGET_OBJECTS:gui_qt_hud_large_msg>
  $<TARGET_OBJECTS:gui_qt_hud_reticle>
  $<TARGET_OBJECTS:gui_qt_hud_hazard>
  $<TARGET_OBJECTS:gui_qt_hud_debug_overlay>
  $<TARGET_OBJECTS:gui_qt_main_window>
  $<TARGET_OBJECTS:gui_qt_toggle_switch>
  $<TARGET_OBJECTS:gui_qt_setting_menu>
)

ament_target_dependencies(gui_qt "Qt5Widgets")
ament_target_dependencies(gui_qt "rclcpp")

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
