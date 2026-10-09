cmake_minimum_required(VERSION 3.10.0)

project(dual_arm_eyou)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(EYOU_BUILD_HAND_ONLY "Build only the hand-only communication test target" OFF)

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

if(NOT DEFINED EYOU_LIB_PATH)
    if(ARCH_ARM64)
        message(STATUS "ARM64 architecture detected, using arm64 library path")
        set(EYOU_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic/arm64)
    else()
        message(STATUS "x86 architecture detected, using x86 library path")
        set(EYOU_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic/x86)
    endif()
endif()

if(NOT DEFINED CANBOX_LIB_PATH)
    if(ARCH_ARM64)
        set(CANBOX_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/controlcan/arm64)    
    else()
        set(CANBOX_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/controlcan)    
    endif()   
endif()

if(NOT DEFINED ZLG_CANFD_LIB_PATH)
    set(ZLG_CANFD_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/canfd/${ARCHITECTURE})
endif()

set(LOCAL_RUNTIME_LIB_PATHS
    ${EYOU_LIB_PATH}
    ${CANBOX_LIB_PATH}
    ${ZLG_CANFD_LIB_PATH}
)

function(configure_local_runtime target_name)
    if(UNIX)
        set_target_properties(${target_name} PROPERTIES
            BUILD_RPATH "${LOCAL_RUNTIME_LIB_PATHS}"
            INSTALL_RPATH "${LOCAL_RUNTIME_LIB_PATHS}"
        )
        target_link_options(${target_name} PRIVATE
            "LINKER:-rpath-link,${EYOU_LIB_PATH}"
            "LINKER:-rpath-link,${CANBOX_LIB_PATH}"
            "LINKER:-rpath-link,${ZLG_CANFD_LIB_PATH}"
        )
    endif()
endfunction()

set(BASE_INCLUDES
    ${PROJECT_SOURCE_DIR}/src/demo
    ${PROJECT_SOURCE_DIR}/src/demo/test
    ${PROJECT_SOURCE_DIR}/src/motor_sensor
    ${PROJECT_SOURCE_DIR}/src/robot_func
    ${PROJECT_SOURCE_DIR}/src/trajectory_planing
    ${PROJECT_SOURCE_DIR}/lib/controlcan
    ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic
    ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share
    ${PROJECT_SOURCE_DIR}/../curi_udp/c
    ${PROJECT_SOURCE_DIR}/../serial
    ${PROJECT_SOURCE_DIR}/../serial_uart_servo
)

if(ARCH_ARM64)
    # ARM64架构：优先包含arm64目录
    include_directories(
        ${PROJECT_SOURCE_DIR}/include/arm64  # 架构专用头文件
        ${PROJECT_SOURCE_DIR}/include        # 通用头文件
        ${BASE_INCLUDES}
    )
else()
    # 其他架构：标准包含路径
    include_directories(
        ${PROJECT_SOURCE_DIR}/include/x86  # 架构专用头文件
        ${PROJECT_SOURCE_DIR}/include 
        ${BASE_INCLUDES}
    )
endif()

# add_subdirectory(/ssd/depth_sensor/kinect/cpp ${CMAKE_BINARY_DIR}/kinect)
# link_libraries(KINECT_INTERFACE)
#Add usb camera
# add_subdirectory(/ssd/depth_sensor/usb_cam/cpp ${CMAKE_BINARY_DIR}/usb_cam)
# link_libraries(USB_CAM_INTERFACE)
# find_package(OpenCV REQUIRED)

find_package(yaml-cpp)

function(add_custom_demo demo_name) 
    add_executable(${demo_name}
                    ${PROJECT_SOURCE_DIR}/lib/controlcan/serial_can.c
                    ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
                    ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
                    ${PROJECT_SOURCE_DIR}/src/demo/${demo_name}.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/eyou_joints.cpp
                    ${PROJECT_SOURCE_DIR}/src/trajectory_planing/polynomial_trajectory.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/inspire_hand.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/hand_can_transport.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/zlg_hand_transport.cpp
                    ${PROJECT_SOURCE_DIR}/src/trajectory_planing/pos_controller.cpp
                    ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share/cuarm_state.h 
                    ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share/time_utils.h
                    ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share/time_utils.cpp
                    ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share/cuarm_state.cpp
                    ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share/cuarm_udp.h
                    ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share/cuarm_udp.cpp 
                    ${PROJECT_SOURCE_DIR}/../curi_udp/c/curi_udp.h
                    ${PROJECT_SOURCE_DIR}/../curi_udp/c/curi_udp.c
                    ${PROJECT_SOURCE_DIR}/../serial/serial.h
                    ${PROJECT_SOURCE_DIR}/../serial/serial.c
                    ${PROJECT_SOURCE_DIR}/../serial_uart_servo/serial_uart_servo.h
                    ${PROJECT_SOURCE_DIR}/../serial_uart_servo/serial_uart_servo.c
                    # $<TARGET_OBJECTS:KINECT_OBJ> 
                    # $<TARGET_OBJECTS:USB_CAM_OBJ> 
                )

    target_link_libraries(${demo_name} "-lm -lpthread -lrt -Wall")

    if(WIN32)
        target_link_libraries(${demo_name} ${EYOU_LIB_PATH}/eu_harmonic.dll)
    elseif(UNIX)
        target_link_libraries(${demo_name} ${EYOU_LIB_PATH}/libeu_harmonic.so pthread)
        target_link_libraries(${demo_name} ${EYOU_LIB_PATH}/libeu_canable.so pthread)
        target_link_libraries(${demo_name} ${CANBOX_LIB_PATH}/libcontrolcan.so pthread)
        target_link_libraries(${demo_name} ${ZLG_CANFD_LIB_PATH}/libusbcanfd.so pthread)
        target_link_libraries(${demo_name} yaml-cpp)
    endif()
    configure_local_runtime(${demo_name})
    if(DEFINED EYOU_PRO)
        add_dependencies(${demo_name} eu_harmonic)
        add_dependencies(${test_name} eu_canable)
    endif()
    target_compile_definitions(${demo_name} PRIVATE 
    PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endfunction()

function(add_custom_test test_name) 
    add_executable(${test_name}
                    ${PROJECT_SOURCE_DIR}/lib/controlcan/serial_can.c
                    ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
                    ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
                    ${PROJECT_SOURCE_DIR}/src/demo/test/${test_name}.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/eyou_joints.cpp
                    ${PROJECT_SOURCE_DIR}/src/trajectory_planing/polynomial_trajectory.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/inspire_hand.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/hand_can_transport.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/zlg_hand_transport.cpp
                    ${PROJECT_SOURCE_DIR}/../serial/serial.c
                    ${PROJECT_SOURCE_DIR}/../serial_uart_servo/serial_uart_servo.c
                )
    if(WIN32)
        target_link_libraries(${test_name} ${EYOU_LIB_PATH}/eu_harmonic.dll)
    elseif(UNIX)
        target_link_libraries(${test_name} ${EYOU_LIB_PATH}/libeu_harmonic.so pthread)
        target_link_libraries(${test_name} ${EYOU_LIB_PATH}/libeu_canable.so pthread)
        target_link_libraries(${test_name} ${CANBOX_LIB_PATH}/libcontrolcan.so pthread)
        target_link_libraries(${test_name} ${ZLG_CANFD_LIB_PATH}/libusbcanfd.so pthread)
        target_link_libraries(${test_name} yaml-cpp)
    endif()
    configure_local_runtime(${test_name})
    if(DEFINED EYOU_PRO)
        add_dependencies(${test_name} eu_harmonic)
        add_dependencies(${test_name} eu_canable)
    endif()
    target_compile_definitions(${test_name} PRIVATE 
    PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endfunction()

function(add_hand_only_test test_name)
    add_executable(${test_name}
                    ${PROJECT_SOURCE_DIR}/src/demo/test/${test_name}.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/inspire_hand.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/hand_can_transport.cpp
                    ${PROJECT_SOURCE_DIR}/src/motor_sensor/zlg_hand_transport.cpp
                )
    if(WIN32)
        target_link_libraries(${test_name} ${EYOU_LIB_PATH}/eu_harmonic.dll)
    elseif(UNIX)
        target_link_libraries(${test_name} -Wl,--no-as-needed)
        target_link_libraries(${test_name} ${EYOU_LIB_PATH}/libeu_harmonic.so pthread)
        target_link_libraries(${test_name} ${EYOU_LIB_PATH}/libeu_canable.so pthread)
        target_link_libraries(${test_name} ${EYOU_LIB_PATH}/libcyhcs_log.so pthread)
        target_link_libraries(${test_name} ${CANBOX_LIB_PATH}/libcontrolcan.so pthread)
        target_link_libraries(${test_name} ${ZLG_CANFD_LIB_PATH}/libusbcanfd.so pthread)
        target_link_libraries(${test_name} -Wl,--as-needed)
    endif()
    configure_local_runtime(${test_name})
    target_compile_definitions(${test_name} PRIVATE
    PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endfunction()

if(EYOU_BUILD_HAND_ONLY)
    add_hand_only_test(test_inspire_hand_only)
    return()
endif()

# add_custom_demo(admittance)
# add_custom_demo(joint_position_control)
# add_custom_demo(cur_control_udp)
add_custom_demo(head_control)
# add_custom_demo(pour_tea)
# add_custom_demo(grav_comp)
# add_custom_demo(vel_control)
add_custom_demo(hand_eye_calibrate)
# add_custom_demo(curobo_joint_command_receiver_immutation)
add_custom_demo(curobo_joint_command_receiver_vel_control)
add_custom_demo(teleop_joint_command_receiver_vel_control)
add_custom_demo(curobo_joint_command_receiver_vel_control_two_gripper)
add_custom_demo(teleop_joint_command_receiver_vel_control_two_gripper)
add_custom_demo(curobo_waypoint_receiver)
# add_custom_demo(vision_pro_receiver)
# add_custom_demo(can_test)
# add_custom_demo(forward_kinematics_from_file)

# add_custom_test(test_torque_sensor)
add_custom_test(test_inspire_hand)
# add_custom_test(test_plan)
# add_custom_test(test_joint)
add_custom_test(test_gripper)
add_custom_test(test_inspire_hand_protocol)
add_hand_only_test(test_inspire_hand_only)
