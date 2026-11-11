cmake_minimum_required(VERSION 3.16)

project(doosan_vf_admittance VERSION 0.1.0 LANGUAGES CXX)

add_library(vf_core INTERFACE)
target_include_directories(vf_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(vf_core INTERFACE cxx_std_17)

add_library(vf_mock STATIC
    src/mock_robot.cpp
)
target_link_libraries(vf_mock PUBLIC vf_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vf_mock PRIVATE -Wall -Wextra -Wpedantic)
endif()
