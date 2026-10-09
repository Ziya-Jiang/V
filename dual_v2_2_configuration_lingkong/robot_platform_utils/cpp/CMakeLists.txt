cmake_minimum_required(VERSION 3.14)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(robot_platform_utils LANGUAGES C CXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
################################### Architecture Mapping ###########################################
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    if(WIN32)
        message(STATUS "Building for Windows-ARM64 architecture")
        set(LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/../python/lib/win_arm64")
    else()
        message(STATUS "Building for Linux-ARM architecture")
        set(LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/../python/lib/arm64")
    endif()
else() 
    if(WIN32)
        message(STATUS "Building for Windows-x64 architecture")
        set(LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/../python/lib/win_x64")
    else()
        message(STATUS "Building for Linux-x86_64 architecture")
        set(LIB_PATH "${CMAKE_CURRENT_LIST_DIR}/../python/lib/x86_64")
    endif()
endif()

if(WIN32)
    if(MSVC)
        include_directories($ENV{INCLUDE_PATH})
        link_directories($ENV{LIBRARY_PATH})
        add_definitions(-DNOMINMAX)
        # add_definitions(-DYAML_CPP_STATIC_DEFINE)
        # set(yaml_library "$<$<CONFIG:Debug>:yaml-cppd.lib>$<$<NOT:$<CONFIG:Debug>>:yaml-cpp.lib>")
        add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
        add_compile_definitions(WIN32_LEAN_AND_MEAN)
    endif()
endif()
################################### Build Switches ###########################################
option(BUILD_LIB "Build Core Library" ON)
option(BUILD_PYTHON_LIB "Build Python Extension Library" OFF)
option(BUILD_CPP_TESTS "Build C++ Test Infrastructure" OFF)

################################### Core Library Target ###########################################
if(BUILD_LIB OR BUILD_CPP_TESTS)
    message(STATUS "Configuring Library: robot_platform_utils")
    cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH PARENT_DIR)
    cmake_path(GET PARENT_DIR PARENT_PATH PARENT_PARENT_DIR)
    
    find_package(yaml-cpp REQUIRED
        PATHS $ENV{LIBRARY_PATH}
        PATH_SUFFIXES "lib/cmake/yaml-cpp" "lib/cmake" "cmake/yaml-cpp"
    )
    if (yaml-cpp_VERSION VERSION_GREATER 0.7.0)
        set(yaml_library yaml-cpp::yaml-cpp)
    else()
        set(yaml_library yaml-cpp)
    endif()

    if(NOT TARGET curi::udp)
        add_subdirectory("${PARENT_PARENT_DIR}/curi_udp/c" "${CMAKE_BINARY_DIR}/curi_udp")
    endif()

    if(NOT TARGET curi::tcp)
        add_subdirectory("${PARENT_PARENT_DIR}/curi_tcp/c" "${CMAKE_BINARY_DIR}/curi_tcp")
    endif()

    file(GLOB LIB_SOURCES "src/*.cpp")
    set(LIB_SOURCES
        ${LIB_SOURCES}
    )

    add_library(robot_platform_utils ${LIB_SOURCES})
    add_library(robot::platform_utils ALIAS robot_platform_utils)
    target_link_libraries(robot_platform_utils PUBLIC 
        ${yaml_library}
        curi::udp
        curi::tcp
    )
    
    target_include_directories(robot_platform_utils PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PARENT_PARENT_DIR}>
    )
endif()

################################### Python Extension Module ###########################################
if((CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR) AND BUILD_PYTHON_LIB)
    message(STATUS "Configuring Python Bindings...")
    
    set(Python3_FIND_STRATEGY LOCATION)
    set(Python3_FIND_REGISTRY NEVER) 
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

    # Locate pybind11 directly using the discovered Python executable
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE pybind11_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    cmake_path(CONVERT "${pybind11_DIR}" TO_CMAKE_PATH_LIST pybind11_DIR)
    find_package(pybind11 REQUIRED CONFIG PATHS ${pybind11_DIR})

    set(PYTHON_SOURCES
        src/config_loader.cpp
        src/pybind11/bindings.cpp
    )
    pybind11_add_module(config_loader ${PYTHON_SOURCES})

    target_link_libraries(config_loader PRIVATE ${yaml_library})

    target_include_directories(config_loader PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
    
    set_target_properties(config_loader PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${LIB_PATH}"
    )
endif()

################################### Test Suite ###########################################
if((CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR) AND BUILD_CPP_TESTS)
    message(STATUS "Configuring C++ Testing Executables...")
    
    set(TEST_NAMES 
        test_config_loader 
        # test_mcast_client1 
        # test_mcast_client2 
        # test_mcast_priority_server
        test_cuarm_tcp_server
        test_cuarm_tcp_client
        test_cuarm_udp_server
        test_cuarm_udp_client
    )

    foreach(test_target ${TEST_NAMES})
        add_executable(${test_target} "${CMAKE_CURRENT_LIST_DIR}/test/${test_target}.cpp")
        target_link_libraries(${test_target} PRIVATE robot::platform_utils)
        target_compile_definitions(${test_target} PUBLIC PROJECT_ROOT_DIR=\"${PARENT_PARENT_DIR}\")
    endforeach()
endif()
