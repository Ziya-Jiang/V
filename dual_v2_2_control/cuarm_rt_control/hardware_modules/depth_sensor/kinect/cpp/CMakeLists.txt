cmake_minimum_required(VERSION 3.2.0)
project(kinect)

###################################Find Packages###########################################
find_package(OpenCV REQUIRED)
find_package(k4a REQUIRED)

###################################Compile sources into an object library###########################
set(KINECT_LINK_LIBS 
    k4a::k4a 
    ${OpenCV_LIBS}
)

set (KINECT_INCLUDE_DIRS 
    ${PROJECT_SOURCE_DIR} 
    ${PROJECT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

add_library(KINECT_INTERFACE INTERFACE)
target_link_libraries(KINECT_INTERFACE INTERFACE ${KINECT_LINK_LIBS})
target_include_directories(KINECT_INTERFACE INTERFACE ${KINECT_INCLUDE_DIRS})
link_libraries(KINECT_INTERFACE)

add_library(KINECT_OBJ OBJECT
    ${PROJECT_SOURCE_DIR}/src/kinect.h
    ${PROJECT_SOURCE_DIR}/src/kinect.cpp
)

###################################Generate Executable######################################
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set (NAME_LISTS "get_image" "get_video" "get_video_time_cost")
    foreach(VAR ${NAME_LISTS})
        add_executable(${VAR} 
            $<TARGET_OBJECTS:KINECT_OBJ> 
            test/${VAR}.cpp
        )
    endforeach()
endif()