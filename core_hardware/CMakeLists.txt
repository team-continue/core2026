cmake_minimum_required(VERSION 3.8)
project(core_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

option(
  CORE_HARDWARE_AUTO_INSTALL_SOEM
  "Automatically run install_soem.sh when vendored SOEM is missing"
  ON
)
set(DEFAULT_SOEM_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/soem")
set(
  SOEM_VENDOR_DIR
  "${DEFAULT_SOEM_VENDOR_DIR}"
  CACHE PATH
  "Path to the vendored SOEM installation directory"
)
set(SOEM_CONFIG "${SOEM_VENDOR_DIR}/cmake/soemConfig.cmake")

if(NOT TARGET soem)
  if(NOT EXISTS "${SOEM_CONFIG}")
    if(CORE_HARDWARE_AUTO_INSTALL_SOEM)
      if(NOT SOEM_VENDOR_DIR STREQUAL DEFAULT_SOEM_VENDOR_DIR)
        message(
          FATAL_ERROR
            "SOEM not found at ${SOEM_CONFIG}. "
            "Automatic install only supports SOEM_VENDOR_DIR=${DEFAULT_SOEM_VENDOR_DIR}, "
            "or disable auto install with -DCORE_HARDWARE_AUTO_INSTALL_SOEM=OFF."
        )
      endif()

      message(STATUS "SOEM not found at ${SOEM_CONFIG}. Running install_soem.sh...")
      execute_process(
        COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/install_soem.sh"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        RESULT_VARIABLE SOEM_INSTALL_RESULT
        OUTPUT_VARIABLE SOEM_INSTALL_STDOUT
        ERROR_VARIABLE SOEM_INSTALL_STDERR
      )
      if(NOT SOEM_INSTALL_RESULT EQUAL 0)
        message(
          FATAL_ERROR
            "Failed to install SOEM.\n"
            "Command: bash ${CMAKE_CURRENT_SOURCE_DIR}/install_soem.sh\n"
            "Exit code: ${SOEM_INSTALL_RESULT}\n"
            "stdout:\n${SOEM_INSTALL_STDOUT}\n"
            "stderr:\n${SOEM_INSTALL_STDERR}"
        )
      endif()
    else()
      message(
        FATAL_ERROR
          "Expected SOEM config at ${SOEM_CONFIG}.\n"
          "Run ${CMAKE_CURRENT_SOURCE_DIR}/install_soem.sh manually, "
          "or configure with -DCORE_HARDWARE_AUTO_INSTALL_SOEM=ON."
      )
    endif()
  endif()

  if(NOT EXISTS "${SOEM_CONFIG}")
    message(
      FATAL_ERROR
        "SOEM installation finished but ${SOEM_CONFIG} was not created.\n"
        "Check install_soem.sh and the downloaded archive contents."
    )
  endif()

  set(soem_DIR "${SOEM_VENDOR_DIR}/cmake")
  find_package(soem REQUIRED CONFIG)
endif()

ament_auto_add_executable(core_hardware src/core_hardware.cpp)
ament_auto_add_executable(
  core_hardware_daemon
  src/core_hardware_daemon.cpp
  src/ecat.cpp
  teensy41/upper/soes/soes-esi/objectlist.c
)
ament_auto_add_executable(core_hardware_usb src/core_hardware_usb.cpp)

set(CORE_HARDWARE_COMMON_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/teensy41/upper/soes/soes-esi
  ${CMAKE_CURRENT_SOURCE_DIR}/teensy41/upper/soes/soes
)

target_include_directories(core_hardware
  PRIVATE
    ${CORE_HARDWARE_COMMON_INCLUDE_DIRS}
)

target_include_directories(core_hardware_daemon
  PRIVATE
    ${CORE_HARDWARE_COMMON_INCLUDE_DIRS}
    $<TARGET_PROPERTY:soem,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(core_hardware
  ${core_msgs_LIBRARIES}
)

target_link_libraries(core_hardware_daemon
  soem
)

target_link_libraries(core_hardware_usb
  ${core_msgs_LIBRARIES}
)

install(
  DIRECTORY
    launch
    systemd
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/install_core_hardware_daemon_service.sh
    scripts/remove_core_hardware_daemon_service.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
