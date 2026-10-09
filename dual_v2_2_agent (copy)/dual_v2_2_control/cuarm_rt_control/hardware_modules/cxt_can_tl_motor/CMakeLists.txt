cmake_minimum_required(VERSION 3.5.0)
project(cxt_can_tl_motor)

if (CMAKE_SIZEOF_VOID_P)
    set(ADDRESS_MODEL 64)
else()
    set(ADDRESS_MODEL 32)
endif()

if (UNIX AND NOT APPLE)
    message("OS: Linux")
    include_directories(${PROJECT_SOURCE_DIR}/../cxt_can/linux)
    include_directories(${PROJECT_SOURCE_DIR}/../cxt_can)
    include_directories(${PROJECT_SOURCE_DIR}/../tl_motor)

    add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/../tl_motor/tl_motor.h
        ${PROJECT_SOURCE_DIR}/../tl_motor/tl_motor.c
        ${PROJECT_SOURCE_DIR}/../cxt_can/linux/controlcan.h
        ${PROJECT_SOURCE_DIR}/../cxt_can/cxt_can.h
        ${PROJECT_SOURCE_DIR}/../cxt_can/cxt_can.c
        ${PROJECT_SOURCE_DIR}/cxt_can_tl_motor.h
        ${PROJECT_SOURCE_DIR}/cxt_can_tl_motor.c
        ${PROJECT_SOURCE_DIR}/linux_main.c
    )

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|arm")
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(CONTROLCAN_LIB ${PROJECT_SOURCE_DIR}/../cxt_can/linux/arm/64bit/libcontrolcan.so)
        else()
            set(CONTROLCAN_LIB ${PROJECT_SOURCE_DIR}/../cxt_can/linux/arm/32bit/libcontrolcan.so)
        endif()
    else()
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(CONTROLCAN_LIB ${PROJECT_SOURCE_DIR}/../cxt_can/linux/x86/64bit/libcontrolcan.so)
        else()
            set(CONTROLCAN_LIB ${PROJECT_SOURCE_DIR}/../cxt_can/linux/x86/32bit/libcontrolcan.so)
        endif()
    endif()
    if(NOT EXISTS "${CONTROLCAN_LIB}")
        message(FATAL_ERROR "ControlCAN library not found: ${CONTROLCAN_LIB}")
    endif()
    message(STATUS "Using ControlCAN library: ${CONTROLCAN_LIB}")

    target_link_libraries(${PROJECT_NAME} PRIVATE ${CONTROLCAN_LIB} m pthread rt)
else()
    message(FATAL_ERROR "cxt_can_tl_motor currently supports Linux only.")
endif()
