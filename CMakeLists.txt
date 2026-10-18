cmake_minimum_required(VERSION 3.16)
project(embedded_motor_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MOTOR_BUILD_TESTS "Build GoogleTest unit tests" ON)
option(MOTOR_BUILD_SIM "Build the host motor simulator" ON)

if(CMAKE_CROSSCOMPILING)
    set(MOTOR_BUILD_TESTS OFF CACHE BOOL "Tests need a host toolchain" FORCE)
    set(MOTOR_BUILD_SIM OFF CACHE BOOL "Simulator links the host C++ library" FORCE)
endif()

function(motor_warnings target)
    target_compile_options(${target} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Wshadow
        -Wswitch-enum
    )
endfunction()

add_library(motor_core
    src/pid.cpp
    src/ramp.cpp
    src/speed_estimator.cpp
    src/faults.cpp
    src/watchdog.cpp
    src/state_machine.cpp
    src/protocol.cpp
    src/controller.cpp
    src/sim_hal.cpp
    src/plant.cpp
)
target_include_directories(motor_core PUBLIC include)
motor_warnings(motor_core)

if(MOTOR_BUILD_SIM)
    add_executable(motor_sim sim/main.cpp)
    target_link_libraries(motor_sim PRIVATE motor_core)
    motor_warnings(motor_sim)
endif()

if(MOTOR_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(motor_tests
        tests/test_pid.cpp
        tests/test_protocol.cpp
        tests/test_state_machine.cpp
    )
    target_link_libraries(motor_tests PRIVATE motor_core GTest::gtest_main)
    motor_warnings(motor_tests)
    add_test(NAME motor_tests COMMAND motor_tests)
endif()
