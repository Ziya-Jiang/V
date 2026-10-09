cmake_minimum_required(VERSION 3.2.0)

# Only call project() if this is the root project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(curi_udp LANGUAGES C CXX)
endif()

###################################Add library###########################################
add_library(CURI_UDP_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/curi_udp.c
)
target_include_directories(CURI_UDP_LIB PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

###################################Generate Executable###################################
#Generate Executable if this CMakeLists isn't a subdirectory 
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    link_libraries(CURI_UDP_LIB)
    add_executable(local
        ${CMAKE_CURRENT_SOURCE_DIR}/local.c
    )
    add_executable(remote
        ${CMAKE_CURRENT_SOURCE_DIR}/remote.c
    )
endif()