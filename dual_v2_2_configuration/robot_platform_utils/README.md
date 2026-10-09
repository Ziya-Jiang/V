# robot_platform_utils

Shared C++ and Python utilities used by CuArm-related repositories (for example `cuarm_upper_software`, `cuarm_rt_control`).

## What This Repo Includes

- C++ utility library and headers in `cpp/include` and `cpp/src`
- YAML-based configuration loading/writing helpers (`config_loader`)
- Networking utilities that integrate with `curi_udp` and `curi_tcp`
- Optional C++ test targets in `cpp/test`
- Optional Python extension build via `pybind11`
- Python helper scripts and tests in `python/scripts` and `python/test`

## Tested Systems

| Platform | OS / Toolchain |
|----------|----------------|
| Linux x86_64 | Ubuntu 22.04 |
| Linux arm64 | Ubuntu 22.04 |
| Windows x64 | MSVC (Visual Studio) |

## Repository Layout

`robot_platform_utils` expects sibling repositories for UDP/TCP communication:

```text
<workspace>/
  robot_platform_utils/
  curi_udp/
  curi_tcp/
```

Dependency repositories:

- [curi_udp](https://github.com/CURI-Simulation-and-Software-Group/curi_udp)
- [curi_tcp](https://github.com/CURI-Simulation-and-Software-Group/curi_tcp)

`cpp/CMakeLists.txt` automatically adds `curi_udp/c` and `curi_tcp/c` via `add_subdirectory` when `curi::udp` / `curi::tcp` targets are not already defined.

## Prerequisites

| Dependency | Version | Required for |
|------------|---------|--------------|
| [CMake](https://cmake.org/download/) | ≥ 3.14 | All builds |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 0.8.0 | C++ library, tests, Python module |
| [Python 3](https://www.python.org/) | 3.8+ | Python module only |
| [pybind11](https://github.com/pybind/pybind11) | — | Python module only |
| curi_udp / curi_tcp | — | C++ library (linked automatically) |

On **Windows**, install [Visual Studio](https://visualstudio.microsoft.com/) (any recent version) with the **Desktop development with C++** workload.

---

## Install Dependencies

### CMake

Install CMake 3.14 or newer from the [official download page](https://cmake.org/download/).

### yaml-cpp

Required for the C++ library (`robot_platform_utils`), C++ tests, and the Python `config_loader` module.

#### Linux

Install from source (recommended version 0.8.0):

```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

Or, if a suitable package is available on your distribution:

```bash
sudo apt-get install libyaml-cpp-dev
```

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

   Use a `-G` generator that matches your installed Visual Studio version if needed.

3. Open `YAML_CPP.sln` in Visual Studio, select **Release / x64**, and build **ALL_BUILD**.
4. Set user environment variables (create them if they do not exist):
   - `INCLUDE_PATH` → add `<your_yaml-cpp_path>\include;`
   - `LIBRARY_PATH` → add `<your_yaml-cpp_path>\build\Release;`

### pybind11

Required only when building the Python extension (`-DBUILD_PYTHON_LIB=ON`). CMake locates pybind11 through the Python interpreter you pass to `Python3_EXECUTABLE`.

```bash
pip install pybind11
```

Or with conda:

```bash
conda install -c conda-forge pybind11 -y
```

---

## Build Targets

The CMake file in `cpp/CMakeLists.txt` defines:

| Target / option | Description |
|-----------------|-------------|
| `robot_platform_utils` | Core C++ library (default, `BUILD_LIB=ON`) |
| `robot::platform_utils` | CMake alias for linking |
| `-DBUILD_PYTHON_LIB=ON` | Build `config_loader` Python module |
| `-DBUILD_CPP_TESTS=ON` | Build C++ test executables |

---

## Build

If you installed dependencies (for example **pybind11**, or **yaml-cpp** from conda-forge) inside a conda environment, **activate that environment before configuring and building**:

```bash
conda activate <your_conda_env>
```

On Windows, use the same command in your terminal before running `cmake`.

### Linux

#### C++ library and tests

```bash
conda activate <your_conda_env>   # if dependencies are installed in conda
cd cpp
mkdir -p build && cd build
cmake -DBUILD_CPP_TESTS=ON ..
cmake --build . --parallel
```

#### Python module

```bash
conda activate <your_conda_env>   # recommended: pybind11 in the same env as Python3_EXECUTABLE
pip install pybind11              # skip if already installed in the active env
cd cpp
mkdir -p build && cd build
cmake -DBUILD_PYTHON_LIB=ON -DPython3_EXECUTABLE=$(which python3) ..
cmake --build . --parallel
```

The built module is written to an architecture-specific folder under `python/lib/`, for example `python/lib/x86_64/`.

### Windows

#### C++ library and tests

```bash
conda activate <your_conda_env>   # if dependencies are installed in conda
cd cpp
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_CPP_TESTS=ON ^
  -DCMAKE_PREFIX_PATH="<your_conda_env_path>;<your_conda_env_path>/Library;" ^
  ..
cmake --build . --config Release --parallel
```

Use a `-G` generator that matches your installed Visual Studio version.

If yaml-cpp was **not** installed via conda, ensure `INCLUDE_PATH` and `LIBRARY_PATH` point to your yaml-cpp build before configuring.

#### Python module

```bash
conda activate <your_conda_env>   # recommended: pybind11 in the same env as Python3_EXECUTABLE
cd cpp
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_PYTHON_LIB=ON ^
  -DPython3_EXECUTABLE="<your_conda_env_path>/python.exe" ^
  -DCMAKE_PREFIX_PATH="<your_conda_env_path>;<your_conda_env_path>/Library;" ^
  ..
cmake --build . --config Release --parallel
```

The built module is written to `python/lib/win_x64/` (or `python/lib/win_arm64/` on ARM64).

---

## How Other Repos Use This via `add_subdirectory`

From a parent project CMake, include this repo's CMake directory and link against `robot::platform_utils`:

```cmake
add_subdirectory(${CMAKE_SOURCE_DIR}/../robot_platform_utils/cpp ${CMAKE_BINARY_DIR}/robot_platform_utils)
target_link_libraries(your_target PRIVATE robot::platform_utils)
```

Parent projects must satisfy the same yaml-cpp dependency (and provide `curi_udp` / `curi_tcp` siblings unless those targets already exist).

---

## Troubleshooting

| Issue | Suggestion |
|-------|------------|
| `yaml-cpp` not found on Windows | Verify `INCLUDE_PATH` and `LIBRARY_PATH` point to your static yaml-cpp build |
| `pybind11` not found | Install with `pip install pybind11` in the same env as `Python3_EXECUTABLE` |
| `curi_udp` / `curi_tcp` not found | Clone both repos as siblings of `robot_platform_utils` |
| Python module not importable | Check `python/lib/<arch>/` for the built `.so` / `.pyd` and add it to `PYTHONPATH` |
| Wrong architecture output folder | Reconfigure from a clean build directory on the target platform |
