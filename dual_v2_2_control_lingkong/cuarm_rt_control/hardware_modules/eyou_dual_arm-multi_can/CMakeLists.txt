cmake_minimum_required(VERSION 3.10.0)

project(dual_arm_eyou)
set(CMAKE_CXX_STANDARD 17)
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


# if(NOT DEFINED EYOU_LIB_PATH)
#     if(ARCH_ARM64)
#         message(STATUS "ARM64 architecture detected, using arm64 library path")
#         set(EYOU_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic/arm64)
#     else()
#         message(STATUS "x86 architecture detected, using x86 library path")
#         set(EYOU_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic/x86)
#     endif()
# endif()

# if(NOT DEFINED CANBOX_LIB_PATH)
#     if(ARCH_ARM64)
#         set(CANBOX_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/controlcan/arm64)    
#     else()
#         set(CANBOX_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/controlcan/x86)    
#     endif()   
# endif()

# set(BASE_INCLUDES
#     ${PROJECT_SOURCE_DIR}/src/demo
#     ${PROJECT_SOURCE_DIR}/src/demo/test
#     ${PROJECT_SOURCE_DIR}/src/motor_sensor
#     ${PROJECT_SOURCE_DIR}/src/robot_func
#     ${PROJECT_SOURCE_DIR}/src/plan_control
#     ${PROJECT_SOURCE_DIR}/lib/controlcan
#     ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic
#     ${PROJECT_SOURCE_DIR}/../cuarm_configuration/share
#     ${PROJECT_SOURCE_DIR}/../curi_udp/c
#     ${PROJECT_SOURCE_DIR}/../serial
#     ${PROJECT_SOURCE_DIR}/../serial_uart_servo
# )
# if(ARCH_ARM64)
#     # ARM64架构：优先包含arm64目录
#     include_directories(
#         ${PROJECT_SOURCE_DIR}/include/arm64  # 架构专用头文件
#         ${PROJECT_SOURCE_DIR}/include        # 通用头文件
#         ${BASE_INCLUDES}
#     )
# else()
#     # 其他架构：标准包含路径
#     include_directories(
#         ${PROJECT_SOURCE_DIR}/include/x86  # 架构专用头文件
#         ${PROJECT_SOURCE_DIR}/include 
#         ${BASE_INCLUDES}
#     )
# endif()
find_package(yaml-cpp)
find_package(nlohmann_json REQUIRED)

if (yaml-cpp_VERSION VERSION_GREATER 0.7.0)
    set(yaml_library yaml-cpp::yaml-cpp)
else()
    set(yaml_library yaml-cpp)
endif()
# ###################################Add source files to lib###########################################
add_library(EYOU_DUAL_ARM_LIB
    ${PROJECT_SOURCE_DIR}/src/motor_sensor/eyou_joints.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_sensor/inspire_hand.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
    ${PROJECT_SOURCE_DIR}/src/motor_sensor/config_loader.cpp
    ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
    ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
    ${PROJECT_SOURCE_DIR}/src/usb_to_can/usb_to_can.cpp
)
target_link_libraries(EYOU_DUAL_ARM_LIB PUBLIC 
    ${PROJECT_SOURCE_DIR}/lib/controlcan/${ARCHITECTURE}/libcontrolcan.so 
    ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic/${ARCHITECTURE}/libeu_harmonic.so   
    ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic/${ARCHITECTURE}/libeu_canable.so 
    ${PROJECT_SOURCE_DIR}/lib/eyou_harmonic/${ARCHITECTURE}/libcyhcs_log.so 
    -lpthread
    ${yaml_library}
    nlohmann_json::nlohmann_json
)
target_include_directories(EYOU_DUAL_ARM_LIB PUBLIC 
    ${PROJECT_SOURCE_DIR}/include/${ARCHITECTURE}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/lib/controlcan
)

###################################Generate Executable######################################
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    function(add_custom_demo demo_name) 
        add_executable(${demo_name}
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/config_loader.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/eyou_joints.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/inspire_hand.cpp
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/usb_to_can.cpp
                        ${PROJECT_SOURCE_DIR}/src/plan_control/polynomial_trajectory.cpp
                        ${PROJECT_SOURCE_DIR}/src/plan_control/pos_controller.cpp
                        ${PROJECT_SOURCE_DIR}/src/plan_control/cartesian_plan.cpp
                        ${PROJECT_SOURCE_DIR}/../serial/serial.c
                        ${PROJECT_SOURCE_DIR}/../serial_uart_servo/serial_uart_servo.c
                        ${PROJECT_SOURCE_DIR}/src/demo/${demo_name}.cpp
                    )
        include_directories(${PROJECT_SOURCE_DIR}/../serial)
        include_directories(${PROJECT_SOURCE_DIR}/../serial_uart_servo)
        # if(WIN32)
        #     target_link_libraries(${demo_name} ${EYOU_LIB_PATH}/eu_harmonic.dll)
        # elseif(UNIX)
        #     target_link_libraries(${demo_name} ${EYOU_LIB_PATH}/libeu_harmonic.so pthread)
        #     target_link_libraries(${demo_name} ${CANBOX_LIB_PATH}/libcontrolcan.so pthread)
        #     target_link_libraries(${demo_name} ${EYOU_LIB_PATH}/libeu_canable.so pthread)
        #     target_link_libraries(${demo_name} yaml-cpp nlohmann_json::nlohmann_json)
        # endif()
        # if(DEFINED EYOU_PRO)
        #     add_dependencies(${demo_name} eu_harmonic)
        #     add_dependencies(${test_name} eu_canable)
        # endif()
        
        if(WIN32)
            target_link_libraries(${demo_name} 
                ${EYOU_LIB_PATH}/eu_harmonic.dll
                EYOU_DUAL_ARM_LIB
            )
        elseif(UNIX)
            target_link_libraries(${demo_name} 
                EYOU_DUAL_ARM_LIB
            )
        endif()
        target_compile_definitions(${demo_name} PRIVATE 
            PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        )
    endfunction()

    function(add_custom_test test_name) 
        add_executable(${test_name}
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/kinematics.cpp
                        # ${PROJECT_SOURCE_DIR}/src/robot_func/robotics_math.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/config_loader.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/eyou_joints.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/torque_sensor.cpp
                        # ${PROJECT_SOURCE_DIR}/src/motor_sensor/inspire_hand.cpp
                        # ${PROJECT_SOURCE_DIR}/src/usb_to_can/usb_to_can.cpp
                        ${PROJECT_SOURCE_DIR}/src/plan_control/polynomial_trajectory.cpp
                        ${PROJECT_SOURCE_DIR}/src/plan_control/pos_controller.cpp
                        ${PROJECT_SOURCE_DIR}/../serial/serial.c
                        ${PROJECT_SOURCE_DIR}/../serial_uart_servo/serial_uart_servo.c
                        ${PROJECT_SOURCE_DIR}/src/demo/test/${test_name}.cpp
                    )
        # Set RPATH to find libraries at runtime
        set_target_properties(${test_name} PROPERTIES
            INSTALL_RPATH "${EYOU_LIB_PATH};${CANBOX_LIB_PATH}"
            BUILD_WITH_INSTALL_RPATH TRUE
            LINK_FLAGS "-Wl,-rpath,${EYOU_LIB_PATH},-rpath,${CANBOX_LIB_PATH}"
        )
        # add_dependencies(${test_name} serial serial_uart_servo)
        include_directories(${PROJECT_SOURCE_DIR}/../serial)
        include_directories(${PROJECT_SOURCE_DIR}/../serial_uart_servo)
        # if(WIN32)
        #     target_link_libraries(${test_name} ${EYOU_LIB_PATH}/eu_harmonic.dll)
        # elseif(UNIX)
        #     target_link_libraries(${test_name} ${EYOU_LIB_PATH}/libeu_harmonic.so pthread)
        #     target_link_libraries(${test_name} ${EYOU_LIB_PATH}/libeu_canable.so pthread)
        #     target_link_libraries(${test_name} ${CANBOX_LIB_PATH}/libcontrolcan.so pthread)
        #     target_link_libraries(${test_name} yaml-cpp)
            
        # endif()
        # if(DEFINED EYOU_PRO)
        #     add_dependencies(${test_name} eu_harmonic)
        #     add_dependencies(${test_name} eu_canable)
        # endif()
        
        if(WIN32)
            target_link_libraries(${demo_name} 
                ${EYOU_LIB_PATH}/eu_harmonic.dll
                EYOU_DUAL_ARM_LIB
            )
        elseif(UNIX)
            target_link_libraries(${demo_name} 
                EYOU_DUAL_ARM_LIB
            )
        endif()
        target_compile_definitions(${demo_name} PRIVATE 
            PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        )
    endfunction()

    # add_custom_demo(collect_data_udp)
    add_custom_demo(cur_control)
    add_custom_demo(head_control)
    # add_custom_demo(pour_tea)
    add_custom_demo(camera_calibration_record_new_point)
    add_custom_demo(joint_position_control)
    add_custom_demo(joint_position_control_vla)
    add_custom_demo(joint_position_control_from_file_revised)
    add_custom_demo(joint_position_control_from_file_revised_2)
    add_custom_demo(forward_kinematics_from_json)
    add_custom_demo(forward_kinematics_from_jpos_file)
    # add_custom_demo(left_cur_right_pos)
    # add_custom_demo(grav_comp)
    # add_custom_demo(task_position_control)
    # add_custom_demo(admittance)
    # add_custom_test(test_gripper)   
endif()