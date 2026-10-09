cmake_minimum_required(VERSION 3.2.0)
project(rmd_motor)

###################################Add library###########################################
add_library(MOTOR_DATA_INTERFACE INTERFACE)
target_include_directories(MOTOR_DATA_INTERFACE INTERFACE ${PROJECT_SOURCE_DIR})
link_libraries(MOTOR_DATA_INTERFACE)

add_library(MOTOR_DATA_OBJ OBJECT
    ${PROJECT_SOURCE_DIR}/rmd_motor.c
    ${PROJECT_SOURCE_DIR}/rmd_motor.h
)
