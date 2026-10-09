# cuarm_rt_control

Real-time robot control service for multiple robot platforms. It runs on the robot PC (or a co-located workstation), executes motion and gripper commands, and exposes state over UDP.

Connect from any of these clients to operate the robot:

| Client | Description |
|--------|-------------|
| [cuarm_upper_software](../cuarm_upper_software/) | Qt operator panel (joint control, trajectory replay, 3D visualization) |
| SDK | Programmatic control via the project APIs |
| Web | Browser-based control interface |

Robot type and parameters are selected at build time via `NAME_LISTS` in `CMakeLists.txt` and shared configuration under `cuarm_configuration/`.

## Tested Systems

| Platform | OS |
|----------|-----|
| Linux x86_64 | Ubuntu 22.04 |
| Linux arm64 | Ubuntu 22.04 |

## Getting Started

Before building this project, complete the dependency installation and build steps in **[cuarm_robotics/README.md](cuarm_robotics/README.md)**.

Then build `cuarm_rt_control` from this directory:

```bash
mkdir build && cd build
cmake ..
make
```

## Run

The executable name follows `<robot_name>_rt_control` (set by `ROBOT_NAME` in `CMakeLists.txt`). For example, with `ROBOT_NAME` set to `dual_v2_1`, the binary is `dual_v2_1_rt_control`.

Run from the `build` directory with root privileges (required for hardware access):

```bash
sudo ./<robot_name>_rt_control
```
