# arm_base

`arm_base` provides the common C++ base interface for arm drivers.

## Build Targets

The CMake file in `CMakeLists.txt` defines:

- `arm_base` (default)
- `arm::base` (alias target)

## How Other Repos Use This Repo via `add_subdirectory`

```cmake
add_subdirectory(${CMAKE_SOURCE_DIR}/../arm_base ${CMAKE_BINARY_DIR}/arm_base)
target_link_libraries(your_target PRIVATE arm::base)
```

## Required Layout for Dependencies

`arm_base/CMakeLists.txt` depends on sibling repo `robot_platform_utils/cpp`.
A typical layout is:

Dependency repository:

- `robot_platform_utils`: [https://github.com/hkclr-manipulation-group/robot_platform_utils](https://github.com/hkclr-manipulation-group/robot_platform_utils)

```text
<workspace>/
  arm_base/
  robot_platform_utils/
```

