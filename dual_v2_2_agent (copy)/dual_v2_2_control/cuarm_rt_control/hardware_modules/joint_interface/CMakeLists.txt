# joint_iface — CAN-bus joint actuator abstraction library
# Aggregates: cxt_can, tl_motor, cxt_can_tl_motor, rmd_motor, cxt_can_rmd_motor, joint_interface

# ControlCAN library detection
if(ARCHITECTURE STREQUAL "arm64")
    set(CONTROLCAN_ARCH_DIR "${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can/linux/arm/64bit")
else()
    set(CONTROLCAN_ARCH_DIR "${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can/linux/x86/64bit")
endif()
set(CONTROLCAN_DEFAULT_LIBRARY "${CONTROLCAN_ARCH_DIR}/libcontrolcan.so")
if(EXISTS "${CONTROLCAN_DEFAULT_LIBRARY}" AND NOT CONTROLCAN_LIBRARY)
    set(CONTROLCAN_LIBRARY "${CONTROLCAN_DEFAULT_LIBRARY}" CACHE FILEPATH
        "Path to libcontrolcan.so / libControlCAN.so" FORCE)
else()
    set(CONTROLCAN_LIBRARY "" CACHE FILEPATH "Path to libcontrolcan.so / libControlCAN.so")
endif()
if(NOT CONTROLCAN_LIBRARY)
    find_library(CONTROLCAN_LIBRARY
        NAMES controlcan ControlCAN
        PATHS "${CONTROLCAN_ARCH_DIR}"
        NO_DEFAULT_PATH
    )
endif()
if(NOT CONTROLCAN_LIBRARY)
    find_library(CONTROLCAN_LIBRARY NAMES controlcan ControlCAN)
endif()
if(NOT CONTROLCAN_LIBRARY)
    message(FATAL_ERROR
        "ControlCAN library was not found. "
        "Configure with -DCONTROLCAN_LIBRARY=/path/to/libcontrolcan.so")
endif()

add_library(joint_iface STATIC
    ${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can/cxt_can.c
    ${PROJECT_SOURCE_DIR}/hardware_modules/tl_motor/tl_motor.c
    ${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can_tl_motor/cxt_can_tl_motor.c
    ${PROJECT_SOURCE_DIR}/hardware_modules/rmd_motor/rmd_motor.c
    ${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can_rmd_motor/cxt_can_rmd_motor.c
    joint.cpp
)

target_include_directories(joint_iface PUBLIC
    ${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can
    ${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can/linux
    ${PROJECT_SOURCE_DIR}/hardware_modules/tl_motor
    ${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can_tl_motor
    ${PROJECT_SOURCE_DIR}/hardware_modules/rmd_motor
    ${PROJECT_SOURCE_DIR}/hardware_modules/cxt_can_rmd_motor
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(joint_iface PUBLIC
    ${CONTROLCAN_LIBRARY}
    m
    pthread
)
