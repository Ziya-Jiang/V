# cuarm_upper_software

Qt-based upper-computer panel for robot control. It provides a graphical interface for joint control, gripper operation, trajectory record/replay, and 3D URDF visualization.

This panel communicates with **[cuarm_rt_control](../cuarm_rt_control/)** over UDP to send commands and receive robot state. In a typical setup, `cuarm_rt_control` runs on the robot PC (or the same workstation) while this panel runs as the operator UI. Both projects share `cuarm_configuration` for robot parameters, URDF, and meshes.

## Tested Systems

| Platform | OS / Toolchain |
|----------|----------------|
| Linux x86_64 | Ubuntu 22.04 |
| Linux arm64 | Ubuntu 22.04 |
| Windows x64 | MSVC (Visual Studio), Qt 6.8.x |

## Repository Layout

This project is intended to be built inside the `cuarm_panel_control` monorepo. The CMake build expects these sibling directories:

```
cuarm_panel_control/
├── cuarm_upper_software/      # this project (operator panel)
├── cuarm_rt_control/          # real-time robot control (UDP server)
├── cuarm_configuration/       # robot configs, URDF, meshes (shared by both)
├── robot_platform_utils/      # shared C++ utilities (linked automatically)
├── curi_udp/                  # UDP communication (linked via robot_platform_utils)
└── curi_tcp/                  # TCP communication (linked via robot_platform_utils)
```

At runtime, the panel reads `rt_control.ip` and `rt_control.port` from the active configuration and exchanges `PanelCommand` / `PlannerState` messages with the running `cuarm_rt_control` process.

Robot configuration is selected at build time via `NAME_LISTS` in `CMakeLists.txt`. The default target is `dual_v2_1_upper_software`, which loads configs from `cuarm_configuration/dual_v2_1/`.

## Recommended Setup Order

If the panel runs on the **same PC** as rt_control, install **[cuarm_rt_control](../cuarm_rt_control/)** first. That setup normally already provides:

| Library | Used by upper_software for |
|---------|---------------------------|
| urdfdom | URDF parsing in 3D simulation |
| Eigen3 | Kinematics / matrix math |
| Boost | Filesystem utilities |

After `cuarm_rt_control` is installed, you typically only need to add **yaml-cpp** and **Qt 6.8.x** before building this project.

If the panel runs on a **separate machine** (UI-only workstation), install urdfdom, Eigen3, and Boost yourself — see [C++ libraries](#c-libraries-urdfdom-eigen3-boost) below.

## Prerequisites

| Dependency | Version | Notes |
|------------|---------|-------|
| [CMake](https://cmake.org/download/) | ≥ 3.14 | Required on all platforms |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 0.8.0 | Windows: static build required |
| [Qt](https://www.qt.io/download-open-source) | 6.8.x | Online Installer recommended |
| [urdfdom](https://github.com/ros/urdfdom) | — | Usually installed with rt_control |
| Eigen3 | — | Usually installed with rt_control |
| Boost | filesystem | Usually installed with rt_control |

On **Windows**, also install [Visual Studio](https://visualstudio.microsoft.com/) (any recent version) with the **Desktop development with C++** workload. Use a Qt kit and CMake generator that match your installed MSVC toolchain.

---

## Install Dependencies

### CMake

Install CMake 3.14 or newer from the [official download page](https://cmake.org/download/).

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

### C++ libraries (urdfdom, Eigen3, Boost)

Skip this section if you already built `cuarm_rt_control` on the same machine.

#### Linux (apt)

```bash
sudo apt-get install liburdfdom-dev libeigen3-dev libboost-all-dev
```

This matches the dependencies used by `cuarm_rt_control/install.sh`.

#### Windows / cross-platform (conda)

Create or activate a conda environment, then install from conda-forge:

```bash
conda create -n cuarm_env -y
conda activate cuarm_env
conda install -c conda-forge urdfdom -y
conda install -c conda-forge boost -y
conda install -c conda-forge eigen -y
```

If you already use a Pinocchio or robotics conda env that includes urdfdom, you can reuse it instead of creating a new one.

When the conda environment is **activated**, CMake can find urdfdom, Eigen3, and Boost from `CONDA_PREFIX` automatically. You do **not** need to add `<your_conda_env_path>` or `<your_conda_env_path>/Library` to `CMAKE_PREFIX_PATH` in the terminal or in VS Code — only non-conda paths such as Qt need to be listed explicitly.

#### Manual install (without conda)

| Library | Linux | Notes |
|---------|-------|-------|
| Eigen3 | `sudo apt-get install libeigen3-dev` | Header-only; no linking required |
| Boost | `sudo apt-get install libboost-all-dev` | CMake component: `filesystem` |
| urdfdom | `sudo apt-get install liburdfdom-dev` | Or build from [source](https://github.com/ros/urdfdom) |

On Windows without conda, install each library separately and add its CMake prefix to `CMAKE_PREFIX_PATH`.

### Qt 6.8.x

Any **6.8.x** patch release is acceptable. Use the [Qt Online Installer](https://www.qt.io/download-open-source) when possible.

#### Windows

1. Run the Qt Online Installer.
2. Install **Qt 6.8.x** with the **MSVC 64-bit** kit that matches your Visual Studio version.
3. Typical install layout: `<your_qt_install>/6.8.x/msvc2022_64`

Optional CMake variables:

```
Qt6_ROOT=<your_qt_install>/6.8.x/msvc2022_64
Qt6_DIR=<your_qt_install>/6.8.x/msvc2022_64/lib/cmake/Qt6
```

#### Linux

1. Install Qt 6.8.x with the Online Installer.
2. Install additional system packages if needed:

   ```bash
   sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev
   sudo apt-get install libxkbcommon-dev libxkbcommon-x11-dev
   ```

3. Add Qt to `PATH` in `~/.bashrc` (adjust path to your install):

   ```bash
   export PATH=~/Qt/6.8.x/gcc_64/bin:$PATH
   ```

<details>
<summary>Linux: build Qt from source (fallback)</summary>

Use this only if the Online Installer fails. See the [official build guide](https://doc.qt.io/qt-6/build-sources.html).

1. Install build dependencies — X11 packages from the [Linux requirements](https://doc.qt.io/qt-6/linux-requirements.html), plus:

   ```bash
   sudo apt-get install ninja-build libclang-dev clang
   sudo apt-get install libdbus-1-dev libatspi2.0-dev
   sudo apt-get install libxcomposite-dev libxcursor-dev libxrandr-dev \
     libxshmfence-dev libxtst-dev libxkbfile-dev
   ```

2. Download Qt 6.8.x source from [Qt downloads](https://download.qt.io/official_releases/qt/6.8/).

3. Deactivate conda to avoid `libz` / `libpcre2` conflicts:

   ```bash
   conda deactivate
   ```

4. Configure, build, and install:

   ```bash
   mkdir qt_build && cd qt_build
   ../qt-everywhere-src-6.8.x/configure \
     -skip qtwebengine -skip qtpdf -skip qtsensor -skip qtvirtualkeyboard
   cmake --build . --parallel
   cmake --install .
   ```

5. Add the installed Qt binaries to `PATH`:

   ```bash
   export PATH=/usr/local/Qt-6.8.x/bin:$PATH
   ```

</details>

---

## Build

Clone the monorepo (or ensure `cuarm_configuration` is present next to this project):

```bash
git clone git@github.com:CURI-Simulation-and-Software-Group/cuarm_panel_control.git
cd cuarm_panel_control/cuarm_upper_software
```

### Linux

```bash
mkdir build && cd build
cmake ..
cmake --build . --parallel
```

### Windows

Activate your conda environment first, then configure with only the Qt path in `CMAKE_PREFIX_PATH`:

```bat
conda activate <your_conda_env>
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_PREFIX_PATH="<your_conda_env_path>;<your_conda_env_path>/Library;<your_qt_install>/6.8.x/msvc2022_64" ^
  ..
cmake --build . --config Release --parallel
```

Use a `-G` generator and Qt MSVC kit that match your installed Visual Studio version.

| Placeholder | Example |
|-------------|---------|
| `<your_conda_env_path>` | `D:/miniconda3/envs/cuarm_env` |
| `<your_qt_install>` | `D:/Qt` |

If you are **not** using conda, add each dependency prefix to `CMAKE_PREFIX_PATH` manually.

Release builds automatically run `windeployqt` to copy Qt runtime dependencies into the build output directory.

#### VS Code (optional)

Example `.vscode/settings.json` for building `cuarm_upper_software` with conda + Qt:

```json
{
    "cmake.sourceDirectory": "<your_repo>/cuarm_panel_control/cuarm_upper_software",
    "cmake.environment": {
        "CONDA_PREFIX": "<your_conda_env_path>",
        "PATH": "<your_conda_env_path>;<your_conda_env_path>/Library/bin;<your_conda_env_path>/Scripts;<your_conda_env_path>/bin;${env:PATH}"
    },
    "cmake.configureSettings": {
        "CMAKE_BUILD_TYPE": "Release"
    },
    "cmake.configureArgs": [
        "-DCMAKE_PREFIX_PATH=<your_conda_env_path>;<your_conda_env_path>/Library;<your_qt_install>/6.8.x/msvc2022_64",
        "-DPython3_EXECUTABLE=<your_conda_env_path>/python.exe"
    ]
}
```

Use **Release** configuration. Build with **F7**, run with **Ctrl+F5**.

### Change Robot Variant

Edit `NAME_LISTS` in `CMakeLists.txt`:

```cmake
set(NAME_LISTS "dual_v2_1")   # default
# set(NAME_LISTS "dual_v1")
# set(NAME_LISTS "arm_v1")
```

Reconfigure and rebuild. The executable name becomes `<variant>_upper_software` (e.g. `dual_v2_1_upper_software`).

---

## Run

### Linux

```bash
./dual_v2_1_upper_software
```

### Windows

```bat
build\Release\dual_v2_1_upper_software.exe
```

The panel loads configuration from `cuarm_configuration/<variant>/` relative to the monorepo root. Ensure mesh files referenced by the URDF exist under the configuration `share/` directory.

Start `cuarm_rt_control` on the robot PC before connecting from the panel. Verify that the IP and port in the config match the running rt_control process.

---
