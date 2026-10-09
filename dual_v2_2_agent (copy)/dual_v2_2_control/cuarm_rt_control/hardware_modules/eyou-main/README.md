# Dual Arm V1 Hardware Test and Demo

## Tested Environment
- **OS**: Ubuntu 20.04 LTS
- **IDE**: Visual Studio Code
- **Hardware**: Dual-Arm V1 System with 14 Eyou joints equiped with Hefei-Lizhi 's torque sensors

## Hardware Functional Modules

### Core Components
| Module             | Source File           | Description                  |
|--------------------|-----------------------|------------------------------|
| Eyou Joints        | `eyou_joints.cpp`     | Eyou joints' control sdk using CAN-OPEN|
| Torque Sensors     | `torque_sensor.cpp`   | Torque sensor board sdk using CAN|
| Kinematics Model   | `kinematic.cpp`       | Forward/Inverse kinematics calculations |
| Inspire Hand       | `inspire_hand.cpp`    | inspire hand sdk using Eyou canable |

## Demos and Tests:

### Motion and Force Control Demo
1. **Admittance Control**
   - File: `admittance.cpp`
   - Function: Drag the arm using torque sensors (still need debug parameters, and Gravity feedforward)

2. **Current Control**
   - File: `cur_control.cpp`
   - Function: Implement dragging via joint current control （no gravity compensation yet）

3. **Joint Space Control**
   - File: `joint_position_control.cpp`
   - Function: Joint trajectory tracking based on MDH parameters 

4. **Task Space Control** 
   - File: `task_space_control_test.cpp`
   - Function: Cartesian space trajectory tracking 

### Tests

| Test Case               | File                         | Status      |
|-------------------------|------------------------------|-------------|
| Torque Sensor Comm Test | `test_torque_sensor.cpp`     | Completed |
| System Bandwidth Test   | `test_bandwidth.cpp`         | Ongoing   |
| Kinematics Verification | `test_kinematic.cpp`         | Verified  |
| Inspire Com Test        | `test_inspire_hand.cpp`      | Completed  |


## Quick Start

### Build & Run
```bash
# Switch to root user
su root

# Build project
cd /path/to/dual_arm_v1
rm -rf build
mkdir build && cd build
cmake ..
make

# Set library path (Example path, modify as needed)
export LD_LIBRARY_PATH=/home/xyd/rt_ws/dual_arm_v1/lib/eyou_harmonic:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/ssd/dual_arm_v1/lib/eyou_harmonic/arm64:$LD_LIBRARY_PATH
# Run demos/tests
./cur_control        # Run cur control demo
./test_torque_sensor    # Execute torque sensor read