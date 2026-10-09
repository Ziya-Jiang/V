cmake_minimum_required(VERSION 3.5.0)
project(cxt_can_rmd_motor)

# Architecture
if (CMAKE_SIZEOF_VOID_P)
    set(ADDRESS_MODEL 64)
    set(NODE_TARGET x64)
else()
    set(ADDRESS_MODEL 32)
    set(NODE_TARGET x86)
endif()

# OS
if (WIN32)
    message("OS: Windows")
    include_directories(${PROJECT_SOURCE_DIR}/../cxt_can/windows)
    include_directories(${PROJECT_SOURCE_DIR}/../cxt_can)
    include_directories(${PROJECT_SOURCE_DIR}/../rmd_motor)
    if (CMAKE_SIZEOF_VOID_P)
        link_directories(${PROJECT_SOURCE_DIR}/windows/x64)
    else()
        link_directories(${PROJECT_SOURCE_DIR}/windows/x86)
    endif()
    add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/../cxt_can/windows/ControlCAN.h
        ${PROJECT_SOURCE_DIR}/../cxt_can/cxt_can.h
        ${PROJECT_SOURCE_DIR}/../cxt_can/cxt_can.c
        ${PROJECT_SOURCE_DIR}/../rmd_motor/rmd_motor.h
        ${PROJECT_SOURCE_DIR}/../rmd_motor/rmd_motor.c
        ${PROJECT_SOURCE_DIR}/cxt_can_rmd_motor.h
        ${PROJECT_SOURCE_DIR}/cxt_can_rmd_motor.c
        ${PROJECT_SOURCE_DIR}/windows_main.c
    )
    if (CMAKE_SIZEOF_VOID_P)
        target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/windows/x64/ControlCAN.lib)
    else()
        target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/windows/x86/ControlCAN.lib)
    endif()
elseif (UNIX AND NOT APPLE)
    message("OS: Linux")
    include_directories(${PROJECT_SOURCE_DIR}/../cxt_can/linux)
    include_directories(${PROJECT_SOURCE_DIR}/../cxt_can)
    include_directories(${PROJECT_SOURCE_DIR}/../rmd_motor)
    if (CMAKE_SIZEOF_VOID_P)
        link_directories(${PROJECT_SOURCE_DIR}/linux/arm/64bit)
    else()
        link_directories(${PROJECT_SOURCE_DIR}/linux/arm/32bit)
    endif()
    add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/../cxt_can/linux/controlcan.h
        ${PROJECT_SOURCE_DIR}/../cxt_can/cxt_can.h
        ${PROJECT_SOURCE_DIR}/../cxt_can/cxt_can.c
        ${PROJECT_SOURCE_DIR}/../rmd_motor/rmd_motor.h
        ${PROJECT_SOURCE_DIR}/../rmd_motor/rmd_motor.c
        ${PROJECT_SOURCE_DIR}/cxt_can_rmd_motor.h
        ${PROJECT_SOURCE_DIR}/cxt_can_rmd_motor.c
        ${PROJECT_SOURCE_DIR}/linux_main.c
    )
    if (CMAKE_SIZEOF_VOID_P)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/../cxt_can/linux/arm/64bit/libcontrolcan.so "-lm -lpthread -lrt -Wall")
    else()
        target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/../cxt_can/linux/arm/32bit/libcontrolcan.so "-lm -lpthread -lrt -Wall")
    endif()
endif()
