cmake_minimum_required(VERSION 3.10.0)

project(single_arm_v1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    message(STATUS "Building for Linux-ARM architecture")
    add_definitions(-DARM_ARCH)
    set(ARCH_ARM64 TRUE)
    set(ARCHITECTURE "arm64")
else() 
    message(STATUS "Building for Linux-x86 architecture")
    set(ARCH_ARM64 FALSE)
    set(ARCHITECTURE "x86")
endif()

# if(NOT DEFINED CANFD_LIB_PATH)
#     if(ARCH_ARM64)
#         message(STATUS "ARM64 architecture detected, using arm64 library path")
#         set(CANFD_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/canfd/arm64)
#     else()
#         message(STATUS "x86 architecture detected, using x86 CAN FD library path")
#         set(CANFD_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/canfd/x86)
#     endif()
# endif()
# if(NOT DEFINED CANNORMAL_LIB_PATH)
#     if(ARCH_ARM64)
#         set(CANNORMAL_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/can/arm64)    
#     else()
#         message(STATUS "x86 architecture detected, using x86 CAN NORMAL library path")
#         set(CANNORMAL_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/can/x86)    
#     endif()   
# endif()

# if(ARCH_ARM64)
#     # ARM64架构：优先包含arm64目录
#     include_directories(
#         ${PROJECT_SOURCE_DIR}/include/arm64  # 架构专用头文件
#         ${PROJECT_SOURCE_DIR}/include        # 通用头文件
#     )
# else()
#     # 其他架构：标准包含路径
#     include_directories(
#         ${PROJECT_SOURCE_DIR}/include/x86  # 架构专用头文件
#         ${PROJECT_SOURCE_DIR}/include 
#     )
# endif()

find_package(yaml-cpp)
if (yaml-cpp_VERSION VERSION_GREATER 0.7.0)
    set(yaml_library yaml-cpp::yaml-cpp)
else()
    set(yaml_library yaml-cpp)
endif()

# ###################################Add source files to lib###########################################
add_library(ARM_V1_LIB
    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_fd.c
    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_normal.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/motor.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/torque_sensor.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/robot_arm.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/robot_arm.cpp
    ${PROJECT_SOURCE_DIR}/src/kinematic/kinematics.cpp
    ${PROJECT_SOURCE_DIR}/src/kinematic/robotics_math.cpp
    ${PROJECT_SOURCE_DIR}/src/plan/polynomial_trajectory.cpp
)
target_link_libraries(ARM_V1_LIB PUBLIC 
    ${PROJECT_SOURCE_DIR}/lib/canfd/${ARCHITECTURE}/libusbcanfd.so
    ${PROJECT_SOURCE_DIR}/lib/can/${ARCHITECTURE}/libeu_canable.so #For torque sensor
    -lm -lpthread -lrt -Wall
    ${yaml_library}
)

if(ARCH_ARM64)
    target_link_libraries(ARM_V1_LIB PUBLIC PUBLIC 
        ${PROJECT_SOURCE_DIR}/lib/can/${ARCHITECTURE}/libcyhcs_log.so #For libeu_canable.so on Arm64. libeu_canable.so of Arm64 & x86 is different version. 
    )
endif()

target_include_directories(ARM_V1_LIB PUBLIC 
    ${PROJECT_SOURCE_DIR}/include/${ARCHITECTURE}
    ${PROJECT_SOURCE_DIR}/include
)

###################################Generate Executable######################################

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    function(add_custom_test test_name) 
        add_executable(${test_name}
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_fd.c
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_normal.cpp
                        # ${PROJECT_SOURCE_DIR}/src/hardware/motor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/hardware/torque_sensor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/hardware/robot_arm.cpp
                        ${PROJECT_SOURCE_DIR}/src/plan/ccg_controller.cpp
                        ${PROJECT_SOURCE_DIR}/src/plan/ccg_fric_controller.cpp
                        ${PROJECT_SOURCE_DIR}/src/force_sensor/serial.c
                        ${PROJECT_SOURCE_DIR}/src/force_sensor/serial_sbt_force_sensor.c
                        ${PROJECT_SOURCE_DIR}/src/${test_name}.cpp

                    )
        # target_link_libraries(${test_name} ${CANFD_LIB_PATH}/libusbcanfd.so -lm -lpthread -lrt -Wall)
        # target_link_libraries(${test_name} ${CANNORMAL_LIB_PATH}/libeu_canable.so pthread)
        # target_link_libraries(${test_name} yaml-cpp)
        target_link_libraries(${test_name} ARM_V1_LIB)
        target_compile_definitions(${test_name} PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    endfunction()

    add_custom_test(cur_control)
    add_custom_test(go_home)
    add_custom_test(enable)
    add_custom_test(pos_control)
    add_custom_test(pos_control_task)
    add_custom_test(test_torque_sensor)
    add_custom_test(grav_comp)
    add_custom_test(grav_comp_with_fric)
    add_custom_test(grav_comp_with_tqs)
    add_custom_test(cur_control_calibration)
    add_custom_test(force_reading)
    add_custom_test(cur_test)
    add_custom_test(joint_position_control_from_file_revised)
endif()
