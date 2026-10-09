cmake_minimum_required(VERSION 3.0.0)
project(usb_cam)

###################################Find Packages###########################################
find_package(OpenCV REQUIRED)

###################################Compile sources into an object library###########################
set(USB_CAM_LINK_LIBS ${OpenCV_LIBS})

set (USB_CAM_INCLUDE_DIRS 
    ${PROJECT_SOURCE_DIR} 
    ${PROJECT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

add_library(USB_CAM_INTERFACE INTERFACE)
target_link_libraries(USB_CAM_INTERFACE INTERFACE ${USB_CAM_LINK_LIBS})
target_include_directories(USB_CAM_INTERFACE INTERFACE ${USB_CAM_INCLUDE_DIRS})
link_libraries(USB_CAM_INTERFACE)

add_library(USB_CAM_OBJ OBJECT
    ${PROJECT_SOURCE_DIR}/src/usb_cam.h
    ${PROJECT_SOURCE_DIR}/src/usb_cam.cpp
)

###################################Generate Executable######################################
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set (NAME_LISTS "get_image" "get_video" "get_image_time_cost" "get_video_time_cost")
    foreach(VAR ${NAME_LISTS})
        add_executable(${VAR} 
            $<TARGET_OBJECTS:USB_CAM_OBJ> 
            test/${VAR}.cpp
        )
    endforeach()
endif()