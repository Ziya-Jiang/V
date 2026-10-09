# dual_v2_2 的 RViz2 显示与排查

本文记录如何在 RViz2 中显示 `dual_v2_2` 机器人结构，以及遇到模型不显示、mesh 加载失败和 TF 缺失时的检查顺序。

## 1. 启动

默认启动：

```bash
source /opt/ros/humble/setup.bash
ros2 launch /mnt/ssd/user/wx/vlm_demo/cuarm_panel_control/cuarm_configuration/dual_v2_2/launch/view_dual_arm_half.launch.py
```

需要通过滑块调整关节时：

```bash
ros2 launch /mnt/ssd/user/wx/vlm_demo/cuarm_panel_control/cuarm_configuration/dual_v2_2/launch/view_dual_arm_half.launch.py \
  use_gui:=true \
  use_joint_state_publisher:=false
```

该 launch 会启动：

- `robot_state_publisher`：解析 URDF，发布机器人 TF。
- `joint_state_publisher` 或 `joint_state_publisher_gui`：发布非固定关节状态。
- `rviz2`：加载 `rviz/tf_model.rviz`。

默认 URDF：

```text
share/left_hand_right_gripper_half/dual_arm_half.urdf
```

## 2. RViz2 基本设置

在 RViz2 左侧确认：

```text
Global Options → Fixed Frame = dummy_world
RobotModel → Enabled = true
RobotModel → Visual Enabled = true
TF → Enabled = true
```

如果只是检查运动学结构，可以暂时关闭 `RobotModel → Visual Enabled`，只观察 `TF` 树。

## 3. 推荐排查顺序

### 3.1 确认进程是否启动

启动日志中应出现：

```text
process started
robot_state_publisher: got segment ...
```

也可以执行：

```bash
ros2 node list
```

正常应至少看到：

```text
/robot_state_publisher
/joint_state_publisher
/rviz2
```

注意：`got segment` 只表示 URDF 解析成功，不代表 RViz2 已经收到 TF。

### 3.2 检查 mesh 文件

mesh 应位于：

```text
share/meshes/
```

例如：

```text
share/meshes/base/SJ4Z.STL
share/meshes/arm/z1.STL
share/meshes/gripper/dz.STL
share/meshes/inspire_hand_l/L_hand_base_link.STL
```

如果日志出现：

```text
Error retrieving file [../meshes/...]: Could not resolve host: ..
```

说明 RViz2 把 `../meshes/...` 当成 URI 解析，不能正确加载本地相对路径。`<mesh filename="..."/>` 应使用 `file://` 绝对 URI，例如：

```xml
<mesh filename="file:///mnt/ssd/user/wx/vlm_demo/cuarm_panel_control/cuarm_configuration/dual_v2_2/share/meshes/arm/z1.STL"/>
```

当前 `dual_arm_half.urdf` 中的 mesh 路径已经按上述格式处理。若工程目录移动，需要同步更新这些绝对路径。

`<mujoco><compiler meshdir="..."/></mujoco>` 是 MuJoCo 配置，不是 RViz2 加载 mesh 的主要依据。

### 3.3 检查关节状态

```bash
ros2 topic echo /joint_states --once
```

如果没有数据，检查发布器：

```bash
ros2 topic info /joint_states --verbose
```

也可改用 GUI 发布器：

```bash
ros2 launch /mnt/ssd/user/wx/vlm_demo/cuarm_panel_control/cuarm_configuration/dual_v2_2/launch/view_dual_arm_half.launch.py \
  use_gui:=true \
  use_joint_state_publisher:=false
```

### 3.4 检查 TF

先确认话题存在：

```bash
ros2 topic list -t
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

再检查根节点到机器人节点的变换：

```bash
ros2 run tf2_ros tf2_echo dummy_world SJ4Z
```

如果 RViz2 报：

```text
Fixed Frame: No tf data
Frame [dummy_world] does not exist
No transform from [L_index_distal] to [dummy_world]
```

说明 RViz2 没有收到 `robot_state_publisher` 发布的 `/tf` 或 `/tf_static`，而不是 Fixed Frame 名字写错。

### 3.5 检查 ROS2 发现环境

检查通信相关环境变量：

```bash
env | grep -E '^(ROS|RMW|FASTRTPS|CYCLONEDDS)_'
```

本次问题中存在：

```text
ROS_DISCOVERY_SERVER=10.30.13.21:11811
```

当该 Discovery Server 不可用时，同一个 launch 启动的 RViz2、`robot_state_publisher` 和 `joint_state_publisher` 也可能互相发现不了。表现为进程都在运行，但：

```bash
ros2 node list
ros2 topic list -t
```

看不到对应节点、`/tf`、`/tf_static` 和 `/joint_states`。

如果当前不需要 Discovery Server，停止 launch 后执行：

```bash
unset ROS_DISCOVERY_SERVER
ros2 daemon stop
ros2 launch /mnt/ssd/user/wx/vlm_demo/cuarm_panel_control/cuarm_configuration/dual_v2_2/launch/view_dual_arm_half.launch.py
```

如果必须使用 Discovery Server，则不要取消变量，而应确认 `10.30.13.21:11811` 的服务正在运行并且网络可达。

所有诊断终端必须使用相同的 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`ROS_LOCALHOST_ONLY` 和 `ROS_DISCOVERY_SERVER`，否则命令行可能看不到 launch 中的节点。

## 4. 最终确认

修复后应满足：

```bash
ros2 node list
ros2 topic echo /joint_states --once
ros2 topic info /tf_static --verbose
ros2 run tf2_ros tf2_echo dummy_world SJ4Z
```

同时 RViz2 中：

- `dummy_world` 不再显示 “Frame does not exist”。
- `TF` 中能看到完整机器人树。
- `RobotModel` 不再出现 “No transform”。
- 终端不再出现 `Could not resolve host: ..`。
- 机器人 mesh 正常显示。

如果 RViz2 日志只有：

```text
Stereo is NOT SUPPORTED
```

通常可以忽略，这不影响普通显示。
