cmake_minimum_required(VERSION 3.8)
project(core_enemy_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# VSCode / clangd / IntelliSense用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(backward_ros REQUIRED)
# find_package(core_msgs REQUIRED)
#

# Auto
find_package(ament_cmake_auto REQUIRED)
# Manual
find_package(OpenCV REQUIRED)
# Use the package-local SDK, including when an old depthai_DIR is cached.
set(DEPTHAI_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/depthai_install")
set(depthai_DIR "${DEPTHAI_INSTALL_DIR}/lib/cmake/depthai" CACHE PATH
    "Package-local DepthAI configuration" FORCE)
if(NOT EXISTS "${depthai_DIR}/depthaiConfig.cmake")
  message(FATAL_ERROR
    "DepthAI SDK is missing. Run: bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/depthAI_install.sh")
endif()
list(INSERT CMAKE_PREFIX_PATH 0 "${DEPTHAI_INSTALL_DIR}")
find_package(depthai 3 CONFIG REQUIRED PATHS "${depthai_DIR}" NO_DEFAULT_PATH)

ament_auto_find_build_dependencies()


ament_auto_add_executable(oakd_panel_localizer
    src/oakd_panel_localizer_main.cpp
    src/oakd_panel_localizer.cpp
)

# ROS Humble also exposes DepthAI 2 headers. Prefer the selected SDK headers.
target_include_directories(oakd_panel_localizer SYSTEM BEFORE PRIVATE
    $<TARGET_PROPERTY:depthai::core,INTERFACE_INCLUDE_DIRECTORIES>
)
# The SDK is outside the ROS install prefix; retain its runtime library path.
set_target_properties(oakd_panel_localizer PROPERTIES
    INSTALL_RPATH "$<TARGET_FILE_DIR:depthai::core>"
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # DT_RPATH takes precedence over ROS's LD_LIBRARY_PATH (DepthAI 2 has
  # the same library name). Apply this only to the DepthAI executable.
  set_property(TARGET oakd_panel_localizer APPEND_STRING PROPERTY
      LINK_FLAGS " -Wl,--disable-new-dtags")
endif()

ament_auto_add_executable(oakd_target_detector
    src/oakd_target_detector_main.cpp
    src/oakd_target_detector.cpp
)

ament_auto_add_executable(target_detector
    src/target_detector_main.cpp
    src/target_detector.cpp
)

ament_auto_add_executable(target_selector
    src/target_selector_main.cpp
    src/target_selector.cpp
)

ament_target_dependencies(oakd_panel_localizer
    OpenCV
    depthai
)

ament_target_dependencies(oakd_target_detector
    OpenCV
)

ament_target_dependencies(target_detector
    OpenCV
)

ament_target_dependencies(target_selector
    OpenCV
)

install(TARGETS
    oakd_panel_localizer
    oakd_target_detector
    target_detector
    target_selector
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/launch
    ${PROJECT_SOURCE_DIR}/config
    DESTINATION share/${PROJECT_NAME}
)

include_directories(${OpenCV_DIRS})

target_link_libraries(oakd_panel_localizer depthai::core ${OpenCV_LIBS})
target_link_libraries(oakd_target_detector ${OpenCV_LIBS})
target_link_libraries(target_detector ${OpenCV_LIBS})
target_link_libraries(target_selector ${OpenCV_LIBS})

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

ament_package()
