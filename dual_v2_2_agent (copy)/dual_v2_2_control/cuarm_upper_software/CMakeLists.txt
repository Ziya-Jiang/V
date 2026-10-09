cmake_minimum_required(VERSION 3.14)
project(cuarm_upper_software)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt UI compilation automation
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

################################### Platform Architecture & Toolchain ###########################################
if (WIN32)
    message(STATUS "OS: Windows (Configuring Static Toolchain Layout)")
    if(MSVC)
        include_directories($ENV{INCLUDE_PATH})
        link_directories($ENV{LIBRARY_PATH})
        add_definitions(-DNOMINMAX)
        # add_definitions(-DYAML_CPP_STATIC_DEFINE)
        # set(yaml_library "$<$<CONFIG:Debug>:yaml-cppd.lib>$<$<NOT:$<CONFIG:Debug>>:yaml-cpp.lib>")
        add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
        add_compile_definitions(WIN32_LEAN_AND_MEAN)
    endif()
else()
    message(STATUS "OS: Linux")
    add_compile_options(-ggdb3)
endif()

################################### Package Discovery ###########################################
find_package(yaml-cpp REQUIRED
    PATHS $ENV{LIBRARY_PATH}
    PATH_SUFFIXES "lib/cmake/yaml-cpp" "lib/cmake" "cmake/yaml-cpp"
)
if (yaml-cpp_VERSION VERSION_GREATER 0.7.0)
    set(yaml_library yaml-cpp::yaml-cpp)
else()
    set(yaml_library yaml-cpp)
endif()

find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Boost COMPONENTS filesystem)
find_package(Qt6 COMPONENTS Core Widgets PrintSupport Qml Quick QuickWidgets 3DCore 3DExtras 3DRender 3DInput 3DQuick 3DQuickExtras REQUIRED)

qt6_add_resources(QT_RESOURCES ${PROJECT_SOURCE_DIR}/QML/qml.qrc)

################################### Internal Subdirectory Linkage #################################
cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH PARENT_DIR )
set (CONFIG_PREFIX_PATH ${PARENT_DIR}/cuarm_configuration)
set(ROBOT_PLATFORM_UTILS_DIR
    "/mnt/ssd/user/V/dual_v2_2_configuration/robot_platform_utils/cpp"
)
add_subdirectory("${ROBOT_PLATFORM_UTILS_DIR}" "${CMAKE_BINARY_DIR}/robot_platform_utils")

################################### Library Definition ###########################################
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src/panel
    ${PARENT_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    # ${PROJECT_SOURCE_DIR}/src/3rd_party
)

link_libraries(
    Eigen3::Eigen
    robot::platform_utils
    ${yaml_library}
    ${Boost_LIBRARIES}
    urdfdom::urdfdom_model
    Qt6::Core Qt6::Widgets Qt6::PrintSupport 
    Qt6::Qml Qt6::Quick Qt6::QuickWidgets 
    Qt6::3DCore Qt6::3DExtras Qt6::3DRender Qt6::3DInput Qt6::3DQuick Qt6::3DQuickExtras
)

file(GLOB SRC_FILES "src/panel/*.h" "src/panel/*.cpp" "src/panel/*.c")
list(REMOVE_ITEM SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/panel/plot_manager.h)
list(REMOVE_ITEM SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/panel/plot_manager.cpp)
add_library(cuarm_upper_software OBJECT
    ${SRC_FILES}
    ${QT_RESOURCES}
)

# # FIXED: Apply target-specific mapping. This forces internal dependency propagation!
# link_libraries(cuarm_upper_software PUBLIC
#     Eigen3::Eigen
#     robot::platform_utils
#     ${yaml_library}
#     ${Boost_LIBRARIES}
#     urdfdom::urdfdom_model
#     Qt6::Core Qt6::Widgets Qt6::PrintSupport 
#     Qt6::Qml Qt6::Quick Qt6::QuickWidgets 
#     Qt6::3DCore Qt6::3DExtras Qt6::3DRender Qt6::3DInput Qt6::3DQuick Qt6::3DQuickExtras
# )

###################################Generate Executables###########################################
# set (NAME_LISTS "dual_v1")
# set (NAME_LISTS "dual_v2_1")
# set (NAME_LISTS "dual_v1")
# set (NAME_LISTS "dual_v2_1")
set (NAME_LISTS "dual_v2_2")
# set (NAME_LISTS "spark2_v1")
# set (NAME_LISTS "spark2_v2")

if(WIN32)
    find_program(WINDEPLOYQT_EXECUTABLE 
        NAMES windeployqt windeployqt.exe
        DOC "Path to the Qt deployment tool"
    )
    if(WINDEPLOYQT_EXECUTABLE)
        message(STATUS "[DEPLOY] Found windeployqt automation engine at: ${WINDEPLOYQT_EXECUTABLE}")
    else()
        message(WARNING "[DEPLOY ERROR] Could not find windeployqt via CMAKE_PREFIX_PATH mappings.")
    endif()
endif()

foreach(VAR ${NAME_LISTS})
    add_executable(${VAR}_upper_software
        $<TARGET_OBJECTS:cuarm_upper_software> 
        test/panel.cpp
    )

    target_link_libraries(${VAR}_upper_software PRIVATE 
        cuarm_upper_software
    )

    target_compile_definitions(${VAR}_upper_software PUBLIC 
        CONFIG_PREFIX_PATH="/mnt/ssd/user/V/dual_v2_2_configuration"
        PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\" 
    )

    if(WIN32 AND WINDEPLOYQT_EXECUTABLE)
        add_custom_command(TARGET ${VAR}_upper_software POST_BUILD
            COMMAND $<$<NOT:$<CONFIG:Debug>>:${WINDEPLOYQT_EXECUTABLE}> 
                    $<$<NOT:$<CONFIG:Debug>>:--release>
                    --compiler-runtime 
                    --qmldir "${PROJECT_SOURCE_DIR}/QML" 
                    "$<TARGET_FILE:${VAR}_upper_software>"
            COMMENT "Deploying mandatory Qt6 dependencies to build directory for non-Debug variants..."
        )
    endif()
endforeach()
