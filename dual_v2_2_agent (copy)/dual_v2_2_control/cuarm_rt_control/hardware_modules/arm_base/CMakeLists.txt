cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(arm_base LANGUAGES CXX)
endif()

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################### Compile Target ###########################################
cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH PARENT_REPO_DIR)
if(NOT DEFINED ROBOT_PLATFORM_UTILS_DIR)
    set(ROBOT_PLATFORM_UTILS_DIR "${PARENT_REPO_DIR}")
endif()

if(NOT DEFINED ROBOT_PLATFORM_UTILS_BINARY_DIR)
    set(ROBOT_PLATFORM_UTILS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

message(STATUS "--------------------------------- BUILD_LIB (arm_base) ---------------------------------")

if(NOT TARGET robot::platform_utils)
    add_subdirectory(
        "${ROBOT_PLATFORM_UTILS_DIR}/robot_platform_utils/cpp" 
        "${ROBOT_PLATFORM_UTILS_BINARY_DIR}/robot_platform_utils"
    )
endif()


add_library(arm_base
    arm_base.cpp
)

add_library(arm::base ALIAS arm_base)

target_link_libraries(arm_base PUBLIC 
    robot::platform_utils
)

target_include_directories(arm_base PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${ROBOT_PLATFORM_UTILS_DIR}>
)