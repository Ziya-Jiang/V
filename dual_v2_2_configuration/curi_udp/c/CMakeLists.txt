cmake_minimum_required(VERSION 3.5)

project(curi_udp LANGUAGES C)
# # Only call project() if this is the root project
# if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
#     project(curi_udp LANGUAGES C)
# endif()

###################################Add library###########################################
add_library(curi_udp 
    ${CMAKE_CURRENT_SOURCE_DIR}/curi_udp.c
)

target_include_directories(curi_udp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(WIN32)
    target_link_libraries(curi_udp PUBLIC ws2_32)
endif()

# Modernized modern target aliases
add_library(curi::udp ALIAS curi_udp)

###################################Generate Executable###################################
#Generate Executable if this CMakeLists isn't a subdirectory 
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    link_libraries(curi::udp)
    add_executable(local
        ${CMAKE_CURRENT_SOURCE_DIR}/local.c
    )
    add_executable(remote
        ${CMAKE_CURRENT_SOURCE_DIR}/remote.c
    )
endif()