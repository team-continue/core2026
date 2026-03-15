cmake_minimum_required(VERSION 3.8)
project(core_hardware_test C CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

set(SOEM_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/soem")
set(soem_DIR "${SOEM_VENDOR_DIR}/cmake")
find_package(soem REQUIRED CONFIG)

add_executable(ecat_zero_check
    main.cpp
    ../src/ecat.cpp
    ../teensy41/upper/soes/soes-esi/objectlist.c
)

target_include_directories(ecat_zero_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../teensy41/upper/soes/soes-esi
    $<TARGET_PROPERTY:soem,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(ecat_zero_check
    soem
    Threads::Threads
)
