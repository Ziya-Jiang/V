cmake_minimum_required(VERSION 3.5)
project(uart_servo)

# Architecture
if (CMAKE_CL_64) 
    set(ADDRESS_MODEL 64)
    set(NODE_TARGET x64)
else()
    set(ADDRESS_MODEL 32)
    set(NODE_TARGET x86)
endif()

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/../serial/serial.h
    ${PROJECT_SOURCE_DIR}/../serial/serial.c
    ${PROJECT_SOURCE_DIR}/serial_uart_servo.h
    ${PROJECT_SOURCE_DIR}/serial_uart_servo.c
    ${PROJECT_SOURCE_DIR}/main.c
)

include_directories(${PROJECT_SOURCE_DIR}/../serial)
# OS
if (WIN32)
    message("OS: Windows")
    target_link_libraries(${PROJECT_NAME}) 
elseif (UNIX AND NOT APPLE) 
    message("OS: Linux")
    target_link_libraries(${PROJECT_NAME} "-lm -lpthread -lrt -Wall")
endif()
