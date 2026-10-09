cmake_minimum_required(VERSION 3.20)
project(cuarm_robotics)

option(ROBOTICS_STATIC "Build using static variations of dependencies" OFF)
if(MSVC)
    if(ROBOTICS_STATIC)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")    # /MT
    else()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL") # /MD
    endif()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (CMAKE_VERSION VERSION_LESS "3.2.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

################################### Platform Architecture & Toolchain ###########################################
if (WIN32)
    if(MSVC)
        include_directories($ENV{INCLUDE_PATH})
        link_directories($ENV{LIBRARY_PATH})
        add_definitions(-DNOMINMAX)
        add_compile_definitions(
            _WIN32_WINNT=0x0A00
            __PRETTY_FUNCTION__=__FUNCSIG__
            _CRT_SECURE_NO_WARNINGS
            WIN32_LEAN_AND_MEAN
        )
        add_compile_options(/EHsc /permissive-)
    endif()
else()
    message(STATUS "OS: Linux")
    add_compile_options(-ggdb3)
endif()

################################### Package Discovery ###########################################
if(ROBOTICS_STATIC)
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC_RUNTIME ON)
    find_package(console_bridge REQUIRED)
    find_package(tinyxml2 REQUIRED)
else()
    set(Boost_USE_STATIC_LIBS OFF)
    set(Boost_USE_STATIC_RUNTIME OFF)
endif()

find_package(yaml-cpp REQUIRED
    PATHS $ENV{LIBRARY_PATH}
    PATH_SUFFIXES "lib/cmake/yaml-cpp" "lib/cmake" "cmake/yaml-cpp"
)
if (yaml-cpp_VERSION VERSION_GREATER 0.7.0)
    set(yaml_library yaml-cpp::yaml-cpp)
else()
    set(yaml_library yaml-cpp)
endif()

# find_package(Python3 COMPONENTS NumPy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem)

################################### Build Switches ##############################################
option(BUILD_CPP_TESTS "Build C++ Test Infrastructure" OFF)

################################### Internal Subdirectory Linkage #################################
cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH PARENT_DIR)

if(NOT DEFINED ROBOT_PLATFORM_UTILS_DIR)
    if(EXISTS "${PARENT_DIR}/robot_platform_utils")
        set(ROBOT_PLATFORM_UTILS_DIR "${PARENT_DIR}")
    else()
        # Go up one more level if it wasn't found
        cmake_path(GET PARENT_DIR PARENT_PATH GRANDPARENT_DIR)
        set(ROBOT_PLATFORM_UTILS_DIR "${GRANDPARENT_DIR}")
    endif()
endif()
message("ROBOT_PLATFORM_UTILS_DIR: ${ROBOT_PLATFORM_UTILS_DIR}")

if(NOT DEFINED ROBOT_PLATFORM_UTILS_BINARY_DIR)
    set(ROBOT_PLATFORM_UTILS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

# Check if CONFIG_PREFIX_PATH is already defined
if(NOT DEFINED CONFIG_PREFIX_PATH OR CONFIG_PREFIX_PATH STREQUAL "")
    if(DEFINED CUARM_CONFIGURATION_DIR AND NOT CUARM_CONFIGURATION_DIR STREQUAL "")
        set(CONFIG_PREFIX_PATH "${CUARM_CONFIGURATION_DIR}/cuarm_configuration")
    else()
        cmake_path(GET PARENT_DIR PARENT_PATH PARENT_PARENT_DIR)
        set(CONFIG_PREFIX_PATH "${PARENT_PARENT_DIR}/cuarm_configuration")
    endif()
endif()

if(NOT TARGET robot::platform_utils)
    add_subdirectory(${ROBOT_PLATFORM_UTILS_DIR}/robot_platform_utils/cpp ${ROBOT_PLATFORM_UTILS_BINARY_DIR}/robot_platform_utils)
endif()

# ################################### Base Robotics Target ###########################################
file(GLOB ROBOTICS_SRC "src/robotics/robot/*.cpp" "src/robotics/robotics_library/*.cpp")
add_library(robotics_lib   
    ${ROBOTICS_SRC}
    src/robotics/robotics_base.cpp
    src/robotics/robotics.cpp
)
add_library(cuarm_robotics::robotics ALIAS robotics_lib)

target_link_libraries(robotics_lib PUBLIC 
    ${yaml_library}
    Eigen3::Eigen
    # ${Boost_LIBRARIES}
    Boost::headers        
    Boost::serialization  
    Boost::filesystem     
    # pinocchio::pinocchio
    ${PINOCCHIO_LIBRARIES}
    robot::platform_utils
)

target_include_directories(robotics_lib PUBLIC 
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    ${ROBOT_PLATFORM_UTILS_DIR}
)

target_compile_definitions(robotics_lib PUBLIC
    EIGEN_QUATERNIONBASE_PLUGIN="${PROJECT_SOURCE_DIR}/src/3rd_party/Eigen_QuaternionBase_Plugin.hpp" 
)

if(ROBOTICS_STATIC)
    if(MSVC)
        target_link_options(robotics_lib INTERFACE "/WHOLEARCHIVE:$<TARGET_FILE:robot::platform_utils>")
    else()
        target_link_options(robotics_lib INTERFACE "-Wl,--whole-archive" "$<TARGET_FILE:robot::platform_utils>" "-Wl,--no-whole-archive")
    endif()
    target_compile_definitions(robotics_lib PUBLIC
        PINOCCHIO_STATIC
        PINOCCHIO_PARSERS_STATIC
        URDFDOM_STATIC
    )
    set_target_properties(robotics_lib PROPERTIES DISABLE_PRECOMPILE_HEADERS ON)
else()
    set_target_properties(robotics_lib PROPERTIES DISABLE_PRECOMPILE_HEADERS OFF)
    # Precompile Headers to prevent the compiler from reading Pinocchio's 100k+ lines of templates for every cpp file.
    target_precompile_headers(robotics_lib PRIVATE
        <Eigen/Dense>
        <pinocchio/fwd.hpp>
        <pinocchio/multibody/model.hpp>
        <pinocchio/multibody/data.hpp>
        <pinocchio/spatial/se3.hpp>
        <pinocchio/spatial/explog.hpp>
        <pinocchio/algorithm/joint-configuration.hpp>
        <pinocchio/algorithm/kinematics.hpp>
        <pinocchio/algorithm/jacobian.hpp>
        <pinocchio/algorithm/frames.hpp>
        <pinocchio/spatial/force.hpp>
        <pinocchio/parsers/urdf.hpp>
        <pinocchio/algorithm/rnea.hpp>
        <pinocchio/algorithm/crba.hpp>
        <pinocchio/algorithm/center-of-mass.hpp>
        <pinocchio/algorithm/model.hpp>
    )
endif()

################################### Planner Target ###########################################
file(GLOB PLANNER_SRC "src/*.cpp")
add_library(planner_lib ${PLANNER_SRC})
add_library(cuarm_robotics::planner ALIAS planner_lib)

target_link_libraries(planner_lib PUBLIC 
    cuarm_robotics::robotics
)
target_include_directories(planner_lib PUBLIC 
    ${PROJECT_SOURCE_DIR}/src
)

################################### Validation Tests ###########################################
if((CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR) AND BUILD_CPP_TESTS)
    if(NOT DEFINED ROBOT_CONFIG_NAME)
        set(ROBOT_CONFIG_NAME "spark2_v1")
    endif()

    add_executable(test_robot_planner test/test_robot_planner.cpp)
    target_link_libraries(test_robot_planner PRIVATE cuarm_robotics::planner)
    target_include_directories(test_robot_planner PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PARENT_DIR}
    )
    target_compile_definitions(test_robot_planner PRIVATE
        CONFIG_PREFIX_PATH="${CONFIG_PREFIX_PATH}/${ROBOT_CONFIG_NAME}"
    )
endif()