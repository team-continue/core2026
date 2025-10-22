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

set(SOEM_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/soem")
if(NOT TARGET soem)
  if(EXISTS "${SOEM_VENDOR_DIR}/cmake/soemConfig.cmake")
    set(soem_DIR "${SOEM_VENDOR_DIR}/cmake")
    find_package(soem REQUIRED CONFIG)

  else()
    message(FATAL_ERROR "Expected SOEM config at ${SOEM_VENDOR_DIR}/cmake/soemConfig.cmake")
  endif()
endif()

#ノード追加
ament_auto_add_executable(core_hardware src/core_hardware.cpp)
target_link_libraries(core_hardware soem)
target_include_directories(core_hardware 
  PRIVATE 
  $<TARGET_PROPERTY:soem,INTERFACE_INCLUDE_DIRECTORIES>
)

ament_auto_package()