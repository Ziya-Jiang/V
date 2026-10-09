# Dual Arm V2 Hardware Test and Demo

## Tested Environment
- **OS**: Ubuntu 20.04 LTS
- **IDE**: Visual Studio Code
- **Hardware**: Dual-Arm V2 System with 14 Realman joints equiped with Hefei-Lizhi 's torque sensors
- **Lib**: yaml-cpp (https://github.com/jbeder/yaml-cpp) Follow instructions on yaml-cpp github.
## Hardware Functional Modules
### Core Components
| Module             | Source File                     | Description                   |
|--------------------|---------------------------------|-------------------------------|
| Realman Joints     | `motor_sensor/motor.cpp`        | Realman motor sdk using fdcan |
| FDCAN communication| `usb_to_can/can_fd.cpp`         | FDCAN sdk using ZLGFDCAN-mini |
| Kinematics Model   | `robot_func/kinematic.cpp`      | Forward/Inverse kinematics    |
| Robot Arm          | `motor_sensor/robot_arm.cpp`    | Robot arm class with 7 joints |

### Core function for robot arm
| Function                     | Input param                      | Description                                                      |
|------------------------------|----------------------------------|------------------------------------------------------------------|
| set_mdh_position/vel/cur/pos | `joint_index`, `rad/dps/ mA`  | set position, vel, cur for joint(small interpolated value)       |
| move_joint         | `joint pos`, `dt`, `T`           | robot arm plan to a mdh joint pos using polynomial interpolation |
| move_line_online                        | `cartesian_pos`,  `dt`, `T`      | robot arm online plan to a cartesian pos/ori using line interpolation   |
| move_line_offline                        | `cartesian_pos`,  `dt`, `T`      | robot arm offline plan to a cartesian pos/ori using line interpolation   |
| get_(mdh_pos/vel/cur)        | `joint_index`                    | get a joint mdh position or vel, or  cur                         |


## Quick Start
### Build & Run
```bash
# Build project
cd /path/to/realman_dual_arm/build
cd build
cmake ..
make

# Run demos/tests
sudo ./joint_position_control        # Run joint control demo

If  libusbcanfd.so.1.0.10: No such file or directory 
	cd /path/to/realman_dual_arm/lib/canfd/arm64 or x86/
	sudo ln -s libusbcanfd.so libusbcanfd.so.1.0.10

```

## Notes

1. **Hardware Requirements**  
   Ensure both CAN-FD devices are properly connected. The code is developed and tested with the ZLG-CANFD-100MINI.

2. **Symmetric DH Parameters**  
   Based on the DH coord. Identical joint angles will result in **symmetric** end-effector poses.

3. **CAN Channel Assignment**  
   After plugging or unplugging CAN devices, verify which CAN channel controls which arm. Test by sending a small joint command—if the wrong arm moves, swap the `can_channel` values for the two arms in `config.yaml`.

4. **Units**  
   All positional units are in **mm** and all angular units are in **radians**.