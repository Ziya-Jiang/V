cmake_minimum_required(VERSION 3.10.0)

project(dual_arm_v2)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")   #Use RPATH to find shared libraries at runtime
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")         #Use RPATH to find shared libraries at runtime

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
add_library(REALMAN_DUAL_ARM_LIB
    ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
    ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_fd.c
    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_normal.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_sensor/motor.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_sensor/robot_arm.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
    ${PROJECT_SOURCE_DIR}/src/plan_control/polynomial_trajectory.cpp
    ${PROJECT_SOURCE_DIR}/src/plan_control/pos_controller.cpp
)
target_link_libraries(REALMAN_DUAL_ARM_LIB PUBLIC 
    ${PROJECT_SOURCE_DIR}/lib/canfd/${ARCHITECTURE}/libusbcanfd.so.1.0.10
    ${PROJECT_SOURCE_DIR}/lib/can/${ARCHITECTURE}/libeu_canable.so #For torque sensor
    -lm -lpthread -lrt -Wall
    ${yaml_library}
)

if(ARCH_ARM64)
    target_link_libraries(REALMAN_DUAL_ARM_LIB PUBLIC PUBLIC 
        ${PROJECT_SOURCE_DIR}/lib/can/${ARCHITECTURE}/libcyhcs_log.so #For libeu_canable.so on Arm64. libeu_canable.so of Arm64 & x86 is different version. 
    )
endif()

target_include_directories(REALMAN_DUAL_ARM_LIB PUBLIC 
    ${PROJECT_SOURCE_DIR}/include/${ARCHITECTURE}
    ${PROJECT_SOURCE_DIR}/include
)

###################################Generate Executable######################################
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    function(add_custom_demo demo_name) 
        add_executable(${demo_name}
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_fd.c
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_normal.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/motor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/robot_arm.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/plan_control/polynomial_trajectory.cpp
                        # ${PROJECT_SOURCE_DIR}/src/plan_control/pos_controller.cpp
                        ${PROJECT_SOURCE_DIR}/src/demo/${demo_name}.cpp
                    )
        # target_link_libraries(${demo_name} ${CANFD_LIB_PATH}/libusbcanfd.so -lm -lpthread -lrt -Wall)
        # # target_link_libraries(${demo_name} ${CANNORMAL_LIB_PATH}/libcontrolcan.so pthread)
        # target_link_libraries(${demo_name} ${CANNORMAL_LIB_PATH}/libeu_canable.so pthread)
        # target_link_libraries(${demo_name} yaml-cpp)
        target_link_libraries(${demo_name} REALMAN_DUAL_ARM_LIB)
        target_compile_definitions(${demo_name} PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    endfunction()

    function(add_custom_test test_name) 
        add_executable(${test_name}
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_fd.c
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_normal.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/motor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/robot_arm.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/plan_control/polynomial_trajectory.cpp
                        # ${PROJECT_SOURCE_DIR}/src/plan_control/pos_controller.cpp
                        ${PROJECT_SOURCE_DIR}/src/test/${test_name}.cpp
                    )
        # target_link_libraries(${test_name} ${CANFD_LIB_PATH}/libusbcanfd.so -lm -lpthread -lrt -Wall)
        # target_link_libraries(${test_name} ${CANNORMAL_LIB_PATH}/libeu_canable.so pthread)
        # target_link_libraries(${test_name} yaml-cpp)
        target_link_libraries(${demo_name} REALMAN_DUAL_ARM_LIB)
        target_compile_definitions(${test_name} PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    endfunction()

    add_custom_demo(joint_position_control)
    # add_custom_demo(test1)
    # add_custom_demo(test2)
    # add_custom_demo(test3)
    # add_custom_demo(trajectory_list_move)
    # add_custom_demo(draw_square)
    # add_custom_demo(task_position_control)

    # add_custom_test(test_torque_sensor)
    # add_custom_test(dynamic_calibration)
    # add_custom_test(read_robot_data)
endif()