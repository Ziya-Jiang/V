# spark2_arm — Spark2 robotic arm hardware abstraction
# Depends on joint_iface for CAN-bus joint actuator layer

# yaml-cpp (required by spark2_config_loader)
find_package(yaml-cpp REQUIRED)
if(yaml-cpp_VERSION VERSION_GREATER 0.7.0)
    set(TEST_YAML_LIBRARY yaml-cpp::yaml-cpp)
else()
    set(TEST_YAML_LIBRARY yaml-cpp)
endif()

add_subdirectory(../joint_interface ${CMAKE_CURRENT_BINARY_DIR}/joint_interface)

add_library(spark2_arm STATIC
    spark2.cpp
    spark2_config_loader.cpp
)

target_include_directories(spark2_arm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PARENT_DIR}
)

target_compile_definitions(spark2_arm PUBLIC
    PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"
)

target_link_libraries(spark2_arm PUBLIC
    joint_iface
    ${TEST_YAML_LIBRARY}
)
