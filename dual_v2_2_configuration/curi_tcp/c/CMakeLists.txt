cmake_minimum_required(VERSION 3.5)

project(curi_tcp LANGUAGES C)

###################################Add library###########################################
add_library(curi_tcp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/curi_tcp.c
)

target_include_directories(curi_tcp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(WIN32)
    target_link_libraries(curi_tcp PUBLIC ws2_32)
endif()

add_library(curi::tcp ALIAS curi_tcp)

###################################Generate Executable###################################
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    link_libraries(curi::tcp)
    add_executable(client
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/client.c
    )
    add_executable(client1
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/client1.c
    )
    add_executable(server_nonstop
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/server_nonstop.c
    )
    add_executable(server
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/server.c
    )
endif()
