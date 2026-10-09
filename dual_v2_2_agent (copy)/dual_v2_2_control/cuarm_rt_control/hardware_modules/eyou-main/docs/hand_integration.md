# InspireHand 通信协议接入与机械臂集成说明

本文说明当前项目中 InspireHand 的代码分层、如何切换不同 CAN 通信协议、手是如何接到双臂控制程序里的，以及如何用 `test_inspire_hand_only` 分别测试 CANABLE 与周立功传输层（见 [9.0 编译与快速测试命令](#90-编译与快速测试命令)）。

## 1. 当前代码分层

手相关代码现在分成两层：

```text
业务命令 + InspireHand 寄存器协议
    include/inspire_hand.h
    src/motor_sensor/inspire_hand.cpp

CAN 传输层
    include/hand_can_transport.h
    src/motor_sensor/hand_can_transport.cpp
    include/zlg_hand_transport.h
    src/motor_sensor/zlg_hand_transport.cpp
```

### 1.1 `InspireHand` 层负责什么

`InspireHand` 是手的业务和协议层，文件是：

- `include/inspire_hand.h`
- `src/motor_sensor/inspire_hand.cpp`

它负责：

- 对外提供手的业务接口：
  - `setAngles`
  - `setForce`
  - `setSpeed`
  - `getAngles`
  - `getForces`
  - `getCurrent`
  - `clearError`
  - `calibrateForce`
- 保存手的寄存器地址：
  - `ANGLE_SET_ADDR`
  - `ANGLE_ACT_ADDR`
  - `FORCE_SET_ADDR`
  - `CURRENT_ADDR`
  - `ERROR_ADDR`
  - `STATUS_ADDR`
- 根据 InspireHand 协议生成扩展帧 ID：
  - `build_ext_id`
- 做寄存器读写和数据打包：
  - `write_register`
  - `read_register`
  - `write_multi_param`
  - `read_multi_param`
- 保存读回来的手状态：
  - `joint_angle_`
  - `joint_force_`
  - `joint_current_`
  - `joint_error_`
  - `joint_status_`

这一层不直接关心 CANABLE、周立功、设备号、通道号、回调函数、接收缓存。它只构造统一的 `HandCanFrame`，然后调用 `IHandCanTransport`。

### 1.2 `IHandCanTransport` 层负责什么

传输层接口在：

- `include/hand_can_transport.h`

核心结构是：

```cpp
struct HandCanFrame {
    uint32_t id;
    bool is_extended;
    uint8_t dlc;
    std::array<uint8_t, 8> data;
};
```

核心接口是：

```cpp
class IHandCanTransport {
public:
    virtual int request(int hand_idx,
                        uint32_t expected_hand_id,
                        const HandCanFrame& request,
                        HandCanFrame& response,
                        int timeout_ms) = 0;

    virtual int waitForResponse(int hand_idx,
                                uint32_t expected_hand_id,
                                HandCanFrame& response,
                                int timeout_ms) = 0;
};
```

传输层负责：

- 把 `HandCanFrame` 转成具体 SDK 的 CAN 帧类型。
- 发送 CAN 帧。
- 等待回包。
- 按 `expected_hand_id` 匹配回包。
- 处理超时。
- 处理设备号、通道号、回调或轮询接收。

因此，换通信协议时，原则上只需要换 `IHandCanTransport` 的实现，不需要改 `InspireHand` 里面的寄存器协议。

## 2. 当前支持的通信实现

### 2.1 默认 CANABLE 通信

默认实现是：

- `CanableHandTransport`
- `include/hand_can_transport.h`
- `src/motor_sensor/hand_can_transport.cpp`

默认构造：

```cpp
int hand_ids[2] = {1, 2};
InspireHand hands(hand_ids);
```

等价于：

```cpp
int hand_ids[2] = {1, 2};
auto transport = std::make_shared<CanableHandTransport>(hand_ids);
InspireHand hands(hand_ids, transport);
```

也就是说，旧代码不需要改，仍然默认走 CANABLE。

CANABLE 传输层内部继续使用原来的：

- `canable_writeMessage`
- `harmonic_setReceiveDataCallBack`
- `canable_CanMsg`
- `harmonic_CanMsg`
- 回包缓存和等待逻辑

这些内容已经从 `InspireHand` 中移出，只保留在 `CanableHandTransport` 里。

### 2.2 周立功 CAN FD 通信

周立功 CAN FD 实现在：

- `include/zlg_hand_transport.h`
- `src/motor_sensor/zlg_hand_transport.cpp`

类名：

```cpp
ZlgCanFdHandTransport
```

用法：

```cpp
#include "inspire_hand.h"
#include "zlg_hand_transport.h"

int hand_ids[2] = {1, 2};

auto transport = std::make_shared<ZlgCanFdHandTransport>(0, 0);
InspireHand hands(hand_ids, transport);
```

这里：

- 第一个 `0` 是周立功设备索引 `dev_idx`。
- 第二个 `0` 是周立功通道索引 `chn_idx`。
- 两只手共用同一个设备和通道。

如果左右手在不同设备或不同通道，可以用数组构造：

```cpp
int hand_ids[2] = {1, 2};

std::array<int, 2> dev_indices = {0, 0};
std::array<int, 2> chn_indices = {0, 1};

auto transport = std::make_shared<ZlgCanFdHandTransport>(dev_indices, chn_indices);
InspireHand hands(hand_ids, transport);
```

CAN FD transport 内部使用：

- `VCI_OpenDevice`
- `VCI_InitCAN`
- `VCI_StartCAN`
- `VCI_TransmitFD`
- `VCI_ReceiveFD`
- `VCI_ResetCAN`
- `VCI_CloseDevice`

### 2.3 周立功 CAN 2.0 通信

周立功 CAN 2.0 实现也在：

- `include/zlg_hand_transport.h`
- `src/motor_sensor/zlg_hand_transport.cpp`

类名：

```cpp
ZlgCan20HandTransport
```

用法：

```cpp
#include "inspire_hand.h"
#include "zlg_hand_transport.h"

int hand_ids[2] = {1, 2};

auto transport = std::make_shared<ZlgCan20HandTransport>(0, 0);
InspireHand hands(hand_ids, transport);
```

如果左右手在不同设备或通道：

```cpp
int hand_ids[2] = {1, 2};

std::array<int, 2> dev_indices = {0, 0};
std::array<int, 2> chn_indices = {0, 1};

auto transport = std::make_shared<ZlgCan20HandTransport>(dev_indices, chn_indices);
InspireHand hands(hand_ids, transport);
```

CAN 2.0 transport 内部使用：

- `VCI_OpenDevice`
- `VCI_InitCAN`
- `VCI_StartCAN`
- `VCI_Transmit`
- `VCI_Receive`
- `VCI_ResetCAN`
- `VCI_CloseDevice`

## 3. 新增通信协议时怎么接

如果以后要加新的通信协议，比如另一个 CAN 卡或网络 CAN，步骤是：

1. 保留 `InspireHand` 不动。
2. 新增一个类继承 `IHandCanTransport`。
3. 实现两个函数：

```cpp
int request(int hand_idx,
            uint32_t expected_hand_id,
            const HandCanFrame& request,
            HandCanFrame& response,
            int timeout_ms);

int waitForResponse(int hand_idx,
                    uint32_t expected_hand_id,
                    HandCanFrame& response,
                    int timeout_ms);
```

4. 在 `request` 里做：

```text
HandCanFrame -> 具体 SDK 帧类型 -> 发送 -> 等待回包 -> 具体 SDK 帧类型 -> HandCanFrame
```

5. 构造 `InspireHand` 时注入新 transport：

```cpp
auto transport = std::make_shared<MyHandTransport>(...);
InspireHand hands(hand_ids, transport);
```

只要新 transport 返回的 `HandCanFrame` 和原协议一致，`setAngles/getAngles/write_multi_param/read_multi_param` 都不用改。

## 4. 手是如何装到机械臂控制里的

机械臂主流程里，手不是作为机械臂关节电机的一部分直接接入 `EyouJoints`，而是作为 gripper 单独接入。

典型代码位置：

- `src/demo/curobo_joint_command_receiver_vel_control.cpp`
- `src/demo/curobo_joint_command_receiver_vel_control_two_gripper.cpp`
- `src/demo/teleop_joint_command_receiver_vel_control.cpp`
- `src/demo/teleop_joint_command_receiver_vel_control_two_gripper.cpp`
- `src/demo/curobo_waypoint_receiver.cpp`

### 4.1 从配置读取手 ID

手 ID 来自配置：

```cpp
robot_config.hardware.hand.ids
```

配置加载逻辑在：

- `include/config_loader.h`

相关字段会被读入：

```cpp
hardware.hand.ids[i]
```

然后 demo 里构造手：

```cpp
InspireHand hands(robot_config.hardware.hand.ids);
```

默认这行代码走 CANABLE。

如果要换成周立功 CAN FD，需要把构造方式改成：

```cpp
auto hand_transport = std::make_shared<ZlgCanFdHandTransport>(0, 0);
InspireHand hands(robot_config.hardware.hand.ids, hand_transport);
```

如果要换成周立功 CAN 2.0：

```cpp
auto hand_transport = std::make_shared<ZlgCan20HandTransport>(0, 0);
InspireHand hands(robot_config.hardware.hand.ids, hand_transport);
```

### 4.2 gripper 类型判断

demo 中一般会维护一个 gripper 类型：

```cpp
const std::string gripper_name[2] = {"InspireHand", "TwoFingerGripper"};
```

或者类似结构：

```cpp
const std::map<std::string, GripperInfoStruct> kGripperInfo = {
    {"InspireHand", {"InspireHand", 6, ...}},
    {"TwoFingerGripper", {"TwoFingerGripper", 1, ...}}
};
```

这里的意思是：

- `InspireHand` 是 6 自由度手。
- `TwoFingerGripper` 是 1 自由度夹爪。
- 双臂系统里左右两侧 gripper 可以分别选择不同类型。

### 4.3 控制命令如何发到手

上层一般接收归一化的 gripper 命令，范围是 `[0, 1]`。

然后根据硬件范围映射成手的实际角度命令：

```cpp
cmd[j] = int(gripper_cmd[start_ind+j] * (max_val-min_val) + min_val);
```

如果当前 gripper 是 InspireHand，就调用：

```cpp
hands.setAngles(i, cmd);
```

其中：

- `i` 是左手或右手索引，通常是 `0` 或 `1`。
- `cmd` 是 6 个关节的目标值。

进入 `InspireHand::setAngles` 后，会拆成两帧写寄存器：

```cpp
write_multi_param(hand_idx, ANGLE_SET_ADDR, false, angles, 4, 2);
write_multi_param(hand_idx, ANGLE_SET_ADDR + 8, false, angles + 4, 2, 2);
```

也就是：

- 第一帧写 4 个关节，每个关节 2 字节，共 8 字节。
- 第二帧写剩下 2 个关节，每个关节 2 字节，共 4 字节。

这部分是 InspireHand 寄存器协议，和底层通信协议无关。

### 4.4 手状态如何读回机械臂状态

读手状态时，demo 里会调用：

```cpp
hands.getAngles(i);
```

成功后，角度保存在：

```cpp
hands.joint_angle_[i][j]
```

然后 demo 会把它转换回归一化状态，写入机械臂整体状态：

```cpp
dual_arm_state_.GripperJointPos[start_ind+j] =
    (hands.joint_angle_[i][j] - min_val) / (max_val - min_val);
```

所以从系统状态角度看：

```text
InspireHand 实际角度
    -> hands.joint_angle_
    -> dual_arm_state_.GripperJointPos
    -> 上层控制/UDP/状态发布
```

## 5. 当前调用链总结

### 5.1 发命令

```text
上层 gripper command
    -> set_gripper_command(...)
    -> 判断 gripper_name[i] == "InspireHand"
    -> hands.setAngles(i, cmd)
    -> InspireHand::setAngles
    -> write_multi_param
    -> build_ext_id + data 打包
    -> transport_->request(...)
    -> CANABLE / ZLG FD / ZLG CAN 2.0
    -> 手硬件
```

### 5.2 读状态

```text
read_gripper_state(...)
    -> 判断 gripper_name[i] == "InspireHand"
    -> hands.getAngles(i)
    -> read_multi_param
    -> build_ext_id + read request 打包
    -> transport_->request(...)
    -> CANABLE / ZLG FD / ZLG CAN 2.0
    -> 手硬件回包
    -> transport 返回 HandCanFrame
    -> read_multi_param 解包
    -> hands.joint_angle_
    -> dual_arm_state_.GripperJointPos
```

## 6. 目前项目内需要的周立功文件

周立功相关文件已经放进主项目：

```text
include/x86/zcan.h
include/arm64/zcan.h
lib/canfd/x86/libusbcanfd.so
lib/canfd/x86/libusb-1.0.so
lib/canfd/arm64/libusbcanfd.so
lib/canfd/arm64/libusb-1.0.so
```

`CMakeLists.txt` 现在通过下面路径链接：

```cmake
${PROJECT_SOURCE_DIR}/lib/canfd/${ARCHITECTURE}/libusbcanfd.so
```

因此 `eyou_dual_arm-main` 不再需要依赖外部的 `single_arm_v1` 路径来编译 ZLG transport。

## 7. 切换通信协议的最小改动

### 7.1 保持旧 CANABLE

不改原代码：

```cpp
InspireHand hands(robot_config.hardware.hand.ids);
```

### 7.2 切到周立功 CAN FD

增加头文件：

```cpp
#include "zlg_hand_transport.h"
```

修改构造：

```cpp
auto hand_transport = std::make_shared<ZlgCanFdHandTransport>(0, 0);
InspireHand hands(robot_config.hardware.hand.ids, hand_transport);
```

### 7.3 切到周立功 CAN 2.0

增加头文件：

```cpp
#include "zlg_hand_transport.h"
```

修改构造：

```cpp
auto hand_transport = std::make_shared<ZlgCan20HandTransport>(0, 0);
InspireHand hands(robot_config.hardware.hand.ids, hand_transport);
```

## 8. 边界原则

后续维护时建议保持这个边界：

- `InspireHand` 只放业务命令和 InspireHand 寄存器协议。
- `InspireHand` 不直接 include `zcan.h`、`eu_canable.h` 或其他 CAN SDK 头文件。
- 新通信协议只新增新的 `IHandCanTransport` 实现。
- 机械臂 demo 只负责选择用哪个 transport，不要把 CAN 发送和等待回包逻辑写进 demo。

## 9. 只测试手通信和基础功能

如果只是想验证手是否能通信、能否执行基础命令，不需要启动双臂，也不应该直接用当前的 `test_inspire_hand.cpp`。

原因是 `src/demo/test/test_inspire_hand.cpp` 里虽然主要测手，但它仍然会：

```cpp
ConfigLoader robot_config(config_path);
EyouJoints dual_arm(robot_config.hardware.motor.ids,
                    robot_config.hardware.motor.enabled);
InspireHand inspire_hands(robot_config.hardware.hand.ids);

dual_arm.canable_init();
```

也就是说，它会加载整套双臂配置，并初始化 `EyouJoints`。如果目标只是测手，这个入口不够干净。

当前已经新增 hand-only 测试程序：

```text
src/demo/test/test_inspire_hand_only.cpp
```

这个测试程序只做：

```text
手 ID
    -> hand transport
    -> InspireHand
    -> getAngles / setSpeed / setForce / setAngles / getCurrent / getStatus
```

不要 include `config_loader.h`，不要创建 `EyouJoints`，不要调用 `dual_arm.canable_init()`。

CMake 中也已经新增独立 target：

```cmake
add_hand_only_test(test_inspire_hand_only)
```

这个 target 只编译手相关文件：

```text
src/demo/test/test_inspire_hand_only.cpp
src/motor_sensor/inspire_hand.cpp
src/motor_sensor/hand_can_transport.cpp
src/motor_sensor/zlg_hand_transport.cpp
```

不会编译双臂电机、UDP、轨迹、力矩传感器、串口夹爪等模块。

### 9.0 编译与快速测试命令

两个传输层（CANABLE / 周立功）共用同一个测试程序 `test_inspire_hand_only`，通过第一个命令行参数切换。

#### 编译

```bash
cd /path/to/eyou_dual_arm-main/build
cmake ..
make test_inspire_hand_only
```

若 `build` 目录尚不存在，先在项目根目录执行 `mkdir build && cd build`。

#### 命令行格式

```text
./test_inspire_hand_only [canable|zlg_fd|zlg_can20] [hand_idx] [hand0_id] [hand1_id] [dev_idx] [chn_idx]
```

默认值：

```text
mode=canable  hand_idx=0  hand_ids={2,1}  dev_idx=0  chn_idx=0
```

查看帮助：

```bash
./test_inspire_hand_only --help
```

#### 两条常用测试命令

在 `build` 目录下执行（运行前确认灵巧手已上电、CAN 线已接好）：

**1. CANABLE（默认 CAN 卡）**

```bash
./test_inspire_hand_only canable
```

使用 `CanableHandTransport`，默认测试 `hand_idx=0`、手 ID `{2, 1}`（与 `config.yaml` 中 `hardware.hand.ids` 一致）。

**2. 周立功 ZLG（CAN FD）**

```bash
./test_inspire_hand_only zlg_fd 0 2 1 0 0
```

| 参数位置 | 值 | 说明 |
|----------|-----|------|
| 第 1 个 | `zlg_fd` | 周立功 CAN FD 模式 |
| 第 2 个 | `0` | `hand_idx`，测第 0 只手 |
| 第 3、4 个 | `2` / `1` | 左右手 CAN ID |
| 第 5、6 个 | `0` / `0` | 周立功 `dev_idx` / `chn_idx` |

若硬件为周立功 **CAN 2.0**（非 FD），将第一个参数改为 `zlg_can20`：

```bash
./test_inspire_hand_only zlg_can20 0 2 1 0 0
```

#### 判断测试结果

程序会依次执行：`getAngles` → `setSpeed` / `setForce` → `setAngles`（张开/半握）→ 再次读取角度、力、电流、状态。

- **成功**：打印 `Initial getAngles success.` 和 `Final read success.`，并输出 6 个关节的角度、力、电流等数据。
- **失败**：打印 `Initial getAngles failed. Check power, CAN wiring, hand ID, and transport mode.` 或 `One or more read commands failed`，请检查供电、CAN 接线、手 ID 是否与参数一致、传输模式是否选对。

### 9.1 hand-only 最小 CANABLE 测试

如果使用默认 CANABLE，可以运行：

```bash
./test_inspire_hand_only canable
```

程序中的核心构造是：

```cpp
#include "inspire_hand.h"
#include <unistd.h>
#include <iostream>

int main() {
    int hand_ids[2] = {2, 1};
    InspireHand hand(hand_ids);

    int hand_idx = 0;
    int16_t speed[6] = {100, 100, 100, 100, 100, 100};
    int16_t force[6] = {200, 200, 200, 200, 200, 200};
    int16_t angle[6] = {500, 500, 500, 500, 500, 500};

    hand.setSpeed(hand_idx, speed);
    usleep(2000);

    hand.setForce(hand_idx, force);
    usleep(2000);

    if (hand.getAngles(hand_idx) == 0) {
        std::cout << "hand communication OK" << std::endl;
        for (int i = 0; i < 6; ++i) {
            std::cout << "joint " << i
                      << " angle: " << hand.joint_angle_[hand_idx][i]
                      << std::endl;
        }
    } else {
        std::cout << "failed to read hand angles" << std::endl;
        return -1;
    }

    hand.setAngles(hand_idx, angle);
    usleep(300000);

    hand.getAngles(hand_idx);
    hand.getCurrent(hand_idx);
    hand.getStatus(hand_idx);

    for (int i = 0; i < 6; ++i) {
        std::cout << "joint " << i
                  << " angle=" << hand.joint_angle_[hand_idx][i]
                  << " current=" << hand.joint_current_[hand_idx][i]
                  << " status=" << hand.joint_status_[hand_idx][i]
                  << std::endl;
    }

    return 0;
}
```

这里的：

```cpp
int hand_ids[2] = {2, 1};
```

来自当前 `config.yaml` 中的：

```yaml
hardware:
  hand:
    ids: [2, 1]
```

如果只接了一个手，通常只测：

```cpp
int hand_idx = 0;
```

另一个 ID 可以先保留，但不要调用 `hand_idx = 1` 的命令。

### 9.2 hand-only 周立功 CAN FD 测试

命令与参数说明见 [9.0](#90-编译与快速测试命令)。简要命令：

```bash
./test_inspire_hand_only zlg_fd 0 2 1 0 0
```

参数含义是：

```text
mode=zlg_fd
hand_idx=0
hand0_id=2
hand1_id=1
dev_idx=0
chn_idx=0
```

程序中的核心构造是：

```cpp
#include "inspire_hand.h"
#include "zlg_hand_transport.h"

int main() {
    int hand_ids[2] = {2, 1};

    auto transport = std::make_shared<ZlgCanFdHandTransport>(0, 0);
    InspireHand hand(hand_ids, transport);

    // 后续 getAngles / setSpeed / setForce / setAngles 与 CANABLE 测试相同
}
```

其中：

- 第一个 `0` 是周立功设备索引 `dev_idx`。
- 第二个 `0` 是周立功通道索引 `chn_idx`。
- 如果左右手共用一个 CAN 通道，两个手都可以用这个构造。

如果左右手在不同通道：

```cpp
std::array<int, 2> dev_indices = {0, 0};
std::array<int, 2> chn_indices = {0, 1};

auto transport = std::make_shared<ZlgCanFdHandTransport>(dev_indices, chn_indices);
InspireHand hand(hand_ids, transport);
```

### 9.3 hand-only 周立功 CAN 2.0 测试

命令与参数说明见 [9.0](#90-编译与快速测试命令)。简要命令：

```bash
./test_inspire_hand_only zlg_can20 0 2 1 0 0
```

程序中的核心构造是：

```cpp
#include "inspire_hand.h"
#include "zlg_hand_transport.h"

int main() {
    int hand_ids[2] = {2, 1};

    auto transport = std::make_shared<ZlgCan20HandTransport>(0, 0);
    InspireHand hand(hand_ids, transport);

    // 后续 getAngles / setSpeed / setForce / setAngles 与 CANABLE 测试相同
}
```

左右手不同通道时：

```cpp
std::array<int, 2> dev_indices = {0, 0};
std::array<int, 2> chn_indices = {0, 1};

auto transport = std::make_shared<ZlgCan20HandTransport>(dev_indices, chn_indices);
InspireHand hand(hand_ids, transport);
```

### 9.4 推荐的基础测试顺序

建议按下面顺序测：

```text
getAngles
getStatus
setSpeed
setForce
setAngles
getAngles
getCurrent
getError
```

判断通信成功的标准：

- `getAngles(hand_idx) == 0`。
- 能打印出 6 个关节角度。
- `setAngles` 后手有实际动作。
- 再次 `getAngles` 能读到变化。
- `getCurrent` 能返回合理电流值。
- `getStatus` / `getError` 不超时。

### 9.5 第一次测试的安全建议

第一次不要直接大幅闭合。建议先用较小目标值：

```cpp
int16_t angle[6] = {200, 200, 200, 200, 200, 200};
```

确认通信和动作正常后，再逐步测试：

```cpp
int16_t angle[6] = {500, 500, 500, 500, 500, 500};
```

再进一步测试：

```cpp
int16_t angle[6] = {800, 800, 800, 800, 800, 800};
```

速度和力也建议先保持较低值：

```cpp
int16_t speed[6] = {100, 100, 100, 100, 100, 100};
int16_t force[6] = {200, 200, 200, 200, 200, 200};
```

这样可以把测试范围限制在手本身，避免双臂电机、UDP、轨迹控制、力矩传感器等模块影响判断。
