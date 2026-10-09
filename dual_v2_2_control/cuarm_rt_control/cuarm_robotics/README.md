# cuarm_robotics

C++ robotics library for kinematics, dynamics, and motion planning (Pinocchio-based). It is built as a dependency of **[cuarm_rt_control](../)** and provides the `cuarm_robotics::robotics` and `cuarm_robotics::planner` targets linked by the real-time control executable.

Complete the steps in this document **before** building `cuarm_rt_control`. See the [cuarm_rt_control README](../README.md) for the top-level build and run instructions.

## Tested Systems

| Platform | OS / Toolchain |
|----------|----------------|
| Linux x86_64 | Ubuntu 22.04 |
| Linux arm64 | Ubuntu 22.04 (Jetson Orin) |
| Windows x64 | MSVC (Visual Studio), conda |

## Repository Layout

This library lives inside the `cuarm_panel_control` monorepo as a submodule of `cuarm_rt_control`:

```
cuarm_panel_control/
├── cuarm_rt_control/
│   ├── cuarm_robotics/          # this project (planning & kinematics)
│   └── example/rt_control.cpp   # links cuarm_robotics::planner
├── cuarm_configuration/         # robot URDF and parameters
└── robot_platform_utils/        # shared C++ utilities (linked automatically)
```

`cuarm_rt_control/CMakeLists.txt` adds this directory via `add_subdirectory(cuarm_robotics)`. You normally build it together with `cuarm_rt_control`, not as a standalone application.

## Prerequisites

| Dependency | Version | Notes |
|------------|---------|-------|
| [CMake](https://cmake.org/download/) | ≥ 3.2 | Required on all platforms |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 0.8.0 | Windows: static build required |
| [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) | — | Kinematics / dynamics |
| Eigen3 | — | Matrix math (header-only) |
| Boost | — | Used by Pinocchio and robotics_lib |
| urdfdom | — | URDF parsing (Pinocchio dependency) |
| Python3 + NumPy | — | Required by CMake (`find_package(Python3 COMPONENTS NumPy)`) |

On **Windows**, also install [Visual Studio](https://visualstudio.microsoft.com/) with the **Desktop development with C++** workload.

---

## Install Dependencies

### CMake

Install CMake from the [official download page](https://cmake.org/download/).

### yaml-cpp

#### Linux

Follow the [yaml-cpp build instructions](https://github.com/jbeder/yaml-cpp):

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

This matches the yaml-cpp steps in [`../install.sh`](../install.sh).

#### Windows

Build as a **static library** and expose it through environment variables used by CMake:

1. Clone [yaml-cpp](https://github.com/jbeder/yaml-cpp).
2. Configure and generate the Visual Studio solution:

   ```bat
   mkdir build
   cd build
   cmake .. -G "Visual Studio 17 2022" -A x64 ^
     -DYAML_BUILD_SHARED_LIBS=OFF ^
     -DYAML_CPP_STATIC_DEFINE=ON
   ```

   Use a `-G` generator that matches your installed Visual Studio version if needed (e.g. `"Visual Studio 16 2019"`).

3. Open `YAML_CPP.sln` in Visual Studio, select **Release / x64**, and build **ALL_BUILD**.
4. Set user environment variables (create them if they do not exist):
   - `INCLUDE_PATH` → add `<your_yaml-cpp_path>\include;`
   - `LIBRARY_PATH` → add `<your_yaml-cpp_path>\build\Release;`

### Pinocchio and related C++ libraries

Pinocchio depends on Eigen3, Boost, and urdfdom. On Linux these are installed together; on Windows they are installed into the same conda environment.

#### Linux (apt + build from source)

Follow instructions on [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html) website.

#### Windows (conda)

Install [Miniconda](https://docs.conda.io/en/latest/miniconda.html), then create a dedicated environment:

```bash
conda create --name pinocchio_env python=3.10 -y
conda activate pinocchio_env
conda install -c conda-forge pinocchio -y
conda install -c conda-forge urdfdom -y
conda install -c conda-forge boost
conda install -c conda-forge pybind11
```

When the conda environment is **activated**, CMake can find Pinocchio, urdfdom, Eigen3, and Boost from `CONDA_PREFIX` automatically.

## Build

`cuarm_robotics` is normally built as part of `cuarm_rt_control`. From the monorepo root:

```bash
cd cuarm_rt_control
mkdir build && cd build
cmake ..
cmake --build . --parallel
```

### Windows

Activate `pinocchio_env` before configuring:

```bat
conda activate pinocchio_env
cd cuarm_rt_control
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_CPP_TESTS=ON ^
  -DPython3_EXECUTABLE="<your_conda_env_path>/python.exe" ^
  -DCMAKE_PREFIX_PATH="<your_conda_env_path>;<your_conda_env_path>/Library;" ^
  ..
cmake --build . --config Release --parallel
```

Use a `-G` generator that matches your installed Visual Studio version.

| Placeholder | Example |
|-------------|---------|
| `<your_conda_env_path>` | `D:/miniconda3/envs/pinocchio_env` |

Ensure `INCLUDE_PATH` and `LIBRARY_PATH` point to your yaml-cpp static build before running CMake.

#### VS Code (optional)

Example `.vscode/settings.json` when building `cuarm_rt_control` with conda + yaml-cpp:

```json
{
    "cmake.sourceDirectory": "<your_repo>/cuarm_panel_control/cuarm_rt_control",
    "cmake.environment": {
        "CONDA_PREFIX": "<your_conda_env_path>",
        "PATH": "<your_conda_env_path>;<your_conda_env_path>/Library/bin;<your_conda_env_path>/Scripts;<your_conda_env_path>/bin;${env:PATH}"
    },
    "cmake.configureSettings": {
        "CMAKE_BUILD_TYPE": "Release"
    },
    "cmake.configureArgs": [
        "-DCMAKE_PREFIX_PATH=<your_conda_env_path>;<your_conda_env_path>/Library",
        "-DPython3_EXECUTABLE=<your_conda_env_path>/python.exe"
    ]
}
```

Use **Release** configuration. Build with **F7**, run with **Ctrl+F5**.

### Optional: standalone validation test

To build only `cuarm_robotics` and its `test_robot_planner` target:

```bash
cd cuarm_rt_control/cuarm_robotics
mkdir build && cd build
cmake .. -DBUILD_CPP_TESTS=ON
cmake --build . --parallel
./test_robot_planner
```

---