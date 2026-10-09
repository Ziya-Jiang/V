cmake_minimum_required(VERSION 3.20)
project(cuarm_rt_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

###################################OS Settings###########################################
add_definitions(-DARCHITECTURE)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    message(STATUS "Building for Linux-ARM architecture")
    set(ARCHITECTURE "arm64")
    add_definitions(-DARCHITECTURE_ARM64=1)
else() 
    message(STATUS "Building for Linux-x86 architecture")
    set(ARCHITECTURE "x86")
    add_definitions(-DARCHITECTURE_X86=1)
endif()

if (UNIX AND NOT APPLE) 
    # select system and lib
    OPTION(LINUX "Use linux Option" OFF)
    OPTION(PREEMPT_RT "Use preempt_rt Option" ON)
    OPTION(XENOMAI "Use xenomai Option" OFF)
    OPTION(USE_LOCAL_XENOMAI_LIB "Use local xenomai lib Option" OFF)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")   #Use RPATH to find shared libraries at runtime
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")         #Use RPATH to find shared libraries at runtime

    if (XENOMAI) 
        MESSAGE("Kernel: xenomai")
        set(PREEMPT_RT OFF)
        set(LINUX OFF)
        add_definitions(-DKERNEL_XENOMAI=1)

        if (USE_LOCAL_XENOMAI_LIB)
            set(XENOMAI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/)
            set(XENOMAI_LINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/)
        else()
            set(XENOMAI_INCLUDE_DIR /usr/xenomai/include/)
            set(XENOMAI_LINK_DIR /usr/xenomai/lib/)
        endif()
        MESSAGE("Use lib:" ${XENOMAI_INCLUDE_DIR})

        include_directories(${xenomai_INCLUDE_DIR}alchemy/)
        include_directories(${xenomai_INCLUDE_DIR}include/)
        #include_directories(${xenomai_INCLUDE_DIR}rtdm)
        include_directories(${xenomai_INCLUDE_DIR}cobalt/)
        include_directories(${xenomai_INCLUDE_DIR}posix/)
        include_directories(${xenomai_INCLUDE_DIR}boilerplate/)
        link_directories(${xenomai_LINK_DIR})

        set(xeno_cflags_params "--skin=posix" "--cflags")
        execute_process(
            COMMAND xeno-config ${xeno_cflags_params}
            OUTPUT_VARIABLE xeno_cflags
            OUTPUT_STRIP_TRAILING_WHITESPACE)

        set(xeno_ldflags_params "--skin=posix" "--ldflags")
        execute_process(
            COMMAND xeno-config ${xeno_ldflags_params}
            OUTPUT_VARIABLE xeno_ldflags
            OUTPUT_STRIP_TRAILING_WHITESPACE)

        # Compiler and linker options
        set(CMAKE_C_FLAGS          "${CMAKE_CXX_FLAGS} ${xeno_cflags}")
        set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} ${xeno_cflags}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${xeno_ldflags}")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${xeno_ldflags}")
        # target_link_libraries(${PROJECT_NAME}
        #     PRIVATE soem ${xenomai_LINK_DIR}libalchemy.so
        #     ${xenomai_LINK_DIR}libcopperplate.so
        #     "-lm"
        # )
    else()
        if (PREEMPT_RT)
            MESSAGE("Kernel: preempt_rt")
            set(LINUX OFF)
            add_definitions(-DKERNEL_PREEMPT_RT=1)
            # target_link_libraries(${PROJECT_NAME}
            #     PRIVATE  "-lm -lpthread -lrt -Wall"
            # )
        else()
            MESSAGE("Kernel: linux")
            add_definitions(-DKERNEL_LINUX=1)
            # target_link_libraries(${PROJECT_NAME} 
            #     PRIVATE "-lm"
            # )
        endif()
    endif()
endif()


###################################Add subdirectory#################################################
find_package(Threads REQUIRED)

cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH PARENT_DIR)
set(CUARM_CONFIGURATION_DIR "${PARENT_DIR}" CACHE PATH "Path to the cuarm configuration directory")
set(CONFIG_PREFIX_PATH "/mnt/ssd/user/V/dual_v2_2_configuration_lingkong")
set(ROBOT_PLATFORM_UTILS_DIR "/mnt/ssd/user/V/dual_v2_2_configuration_lingkong")
set(ROBOT_PLATFORM_UTILS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Path to the utils repository binary directory")
add_subdirectory(cuarm_robotics)

add_subdirectory(hardware_modules/arm_base)

# VLM left-hand bridge dependencies reused by dual_v2_2_rt_control.
set(EYOU_MAIN_DIR ${PROJECT_SOURCE_DIR}/hardware_modules/eyou-main)
set(EYOU_MAIN_EYOU_LIB_DIR ${EYOU_MAIN_DIR}/lib/eyou_harmonic/${ARCHITECTURE})
set(EYOU_MAIN_ZLG_CANFD_LIB_DIR ${EYOU_MAIN_DIR}/lib/canfd/${ARCHITECTURE})
if(ARCHITECTURE STREQUAL "arm64")
    set(EYOU_MAIN_CONTROLCAN_LIB_DIR ${EYOU_MAIN_DIR}/lib/controlcan/arm64)
else()
    set(EYOU_MAIN_CONTROLCAN_LIB_DIR ${EYOU_MAIN_DIR}/lib/controlcan)
endif()

#Two finger uart servo gripper
include_directories(${PROJECT_SOURCE_DIR}/hardware_modules/serial)
include_directories(${PROJECT_SOURCE_DIR}/hardware_modules/serial_uart_servo)

#Robot Arm

# set(ROBOT_NAME "spark2_v1")
set(ROBOT_NAME "dual_v2_2")

if (${ROBOT_NAME} STREQUAL "dual_v1")
    add_definitions(-DDUAL_V1=1)
    add_subdirectory(hardware_modules/eyou_dual_arm-multi_can)
    set(ROBOT_LIB EYOU_DUAL_ARM_LIB)
    set(ROBOT_SRCS
        ${PROJECT_SOURCE_DIR}/src/gripper/inspire_hand_wrapper.cpp
        ${PROJECT_SOURCE_DIR}/src/arm/dual_v1_arm.cpp
    )
elseif (${ROBOT_NAME} STREQUAL "dual_v2_1")
    add_definitions(-DDUAL_V2_1=1)
    add_subdirectory(hardware_modules/realman_dual_arm)
    set(ROBOT_LIB REALMAN_DUAL_ARM_LIB)
    set(ROBOT_SRCS
        ${PROJECT_SOURCE_DIR}/src/arm/dual_v2_1_arm.cpp
    )
elseif (${ROBOT_NAME} STREQUAL "dual_v2_2")
    add_definitions(-DDUAL_V2_2=1)
    add_subdirectory(hardware_modules/realman_dual_arm)
    set(ROBOT_LIB REALMAN_DUAL_ARM_LIB)
    set(ROBOT_SRCS
        ${PROJECT_SOURCE_DIR}/src/arm/dual_v2_2_arm.cpp
    )
elseif (${ROBOT_NAME} STREQUAL "spark2_v1")
    add_definitions(-DSPARK2_V1=1)
    add_subdirectory(hardware_modules/single_arm_v1)
    set(ROBOT_LIB ARM_V1_LIB)
    set(ROBOT_SRCS
        ${PROJECT_SOURCE_DIR}/src/arm/spark2_v1.cpp
    )
elseif (${ROBOT_NAME} STREQUAL "spark2_v2")
    add_definitions(-DSPARK2_V2=1)
    add_subdirectory(hardware_modules/spark2)
    set(ROBOT_LIB spark2_arm)
    set(ROBOT_SRCS)
endif()
###################################Generate Executable######################################
set(EXECUTABLE_NAME ${ROBOT_NAME}_rt_control)
message("-----------------------------Add executable: ${EXECUTABLE_NAME}")

add_executable(${EXECUTABLE_NAME}
    ${PROJECT_SOURCE_DIR}/src/circular_queue.cpp
    ${PROJECT_SOURCE_DIR}/src/gripper/abstract_gripper.cpp
    ${ROBOT_SRCS}

    # uart servo gripper
    ${PROJECT_SOURCE_DIR}/hardware_modules/serial/serial.c
    ${PROJECT_SOURCE_DIR}/hardware_modules/serial_uart_servo/serial_uart_servo.c
    ${PROJECT_SOURCE_DIR}/src/gripper/uart_servo_two_finger_gripper.cpp
    ${PROJECT_SOURCE_DIR}/src/gripper/lingkong_can_gripper.cpp

    # VLM-style left Inspire hand control for dual_v2_2
    ${PROJECT_SOURCE_DIR}/hardware_modules/eyou-main/src/motor_sensor/inspire_hand.cpp
    ${PROJECT_SOURCE_DIR}/hardware_modules/eyou-main/src/motor_sensor/hand_can_transport.cpp

    # src
    ${PROJECT_SOURCE_DIR}/example/rt_control.cpp
    # ${PROJECT_SOURCE_DIR}/example/server.cpp
)

target_link_libraries(${EXECUTABLE_NAME}
    pthread
    dl
    cuarm_robotics::planner
    arm::base
    ${ROBOT_LIB}
    -Wl,--no-as-needed
    ${EYOU_MAIN_EYOU_LIB_DIR}/libeu_canable.so
    ${EYOU_MAIN_EYOU_LIB_DIR}/libcyhcs_log.so
    -Wl,--as-needed
)

target_compile_definitions(${EXECUTABLE_NAME} PUBLIC
    CONFIG_PREFIX_PATH="${CONFIG_PREFIX_PATH}"
    PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"
)

if(ROBOT_NAME STREQUAL "spark2_v2")
    get_filename_component(CONTROLCAN_LIBRARY_DIR "${CONTROLCAN_LIBRARY}" DIRECTORY)
    set_target_properties(${EXECUTABLE_NAME} PROPERTIES
        BUILD_RPATH "${CONTROLCAN_LIBRARY_DIR}"
        INSTALL_RPATH "${CONTROLCAN_LIBRARY_DIR}"
    )
endif()

if(ARCHITECTURE STREQUAL "arm64")
    target_compile_definitions(${EXECUTABLE_NAME} PRIVATE ARM_ARCH)
endif()

target_include_directories(${EXECUTABLE_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PARENT_DIR}
    ${PROJECT_SOURCE_DIR}/hardware_modules/serial
    ${PROJECT_SOURCE_DIR}/hardware_modules/serial_uart_servo
    ${EYOU_MAIN_DIR}/include
    ${EYOU_MAIN_DIR}/include/${ARCHITECTURE}
    ${EYOU_MAIN_DIR}/lib/controlcan
)

set_target_properties(${EXECUTABLE_NAME} PROPERTIES
    BUILD_RPATH "${EYOU_MAIN_EYOU_LIB_DIR}"
    INSTALL_RPATH "${EYOU_MAIN_EYOU_LIB_DIR}"
)

enable_testing()
add_executable(
    test_lingkong_can_protocol
    ${PROJECT_SOURCE_DIR}/tests/test_lingkong_can_protocol.cpp
)
target_include_directories(
    test_lingkong_can_protocol
    PRIVATE ${PROJECT_SOURCE_DIR}
)
add_test(
    NAME lingkong_can_protocol
    COMMAND test_lingkong_can_protocol
)
add_executable(
    test_lingkong_can_vcan
    ${PROJECT_SOURCE_DIR}/tests/test_lingkong_can_vcan.cpp
    ${PROJECT_SOURCE_DIR}/src/gripper/abstract_gripper.cpp
    ${PROJECT_SOURCE_DIR}/src/gripper/lingkong_can_gripper.cpp
)
target_include_directories(
    test_lingkong_can_vcan
    PRIVATE
        ${PROJECT_SOURCE_DIR}
        ${ROBOT_PLATFORM_UTILS_DIR}
)
target_link_libraries(
    test_lingkong_can_vcan
    PRIVATE Threads::Threads
)

add_executable(send_dual_v2_2_panel_command
    ${PROJECT_SOURCE_DIR}/example/send_dual_v2_2_panel_command.cpp
)

target_link_libraries(send_dual_v2_2_panel_command
    robot::platform_utils
)

target_include_directories(send_dual_v2_2_panel_command PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PARENT_DIR}
)

target_compile_definitions(send_dual_v2_2_panel_command PUBLIC
    CONFIG_PREFIX_PATH="${CONFIG_PREFIX_PATH}"
)
