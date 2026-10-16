# V3.1：仅视觉阶段使用直接伺服

入口 `cuart_book_grasp_v3_1.py` 对应独立适配器 `dual_v2_2_upper_planner_v3_1.py`。
**复位、粗定位、夹爪抓取、抬升和下降保留原流程及 CuRobo 规划。** 原 `V3.py`、`dual_v2_2_upper_planner.py`、`scripts/curobo_planner.py` 没有修改。

## 替换范围

只有 `book_v3_xyz_servo` 和 `book_v3_1_stream` 改走 `scripts/v3_1_direct_servo.py`：

1. 感知仍是 Shi–Tomasi 特征点、LK 光流和 RANSAC 书脊中线估计，不是 CoTracker。粗定位后选择腕部画面中央书，不做头腕匹配。
2. 两次探测识别 Y/工具轴旋转的图像 Jacobian，生成 Y、腕部旋转和 X 前伸目标；固定 TCP 基座 Z，X 终点沿用本轮 V1_3 深度计算。图像目标最多 10 Hz 更新，中间不等每步到位。
3. 新控制器以 20 ms 为目标周期读取关节反馈，用 CPU FK 和数值机械臂 Jacobian，将 TCP 误差转换为阻尼最小二乘关节速度。经过速度、加速度和关节指令领先量限制，积分成关节位置。
4. 采样检查关节限制、TCP 高度、自碰撞和已配置的立方体障碍物后，以 `RAW_SINGLE_POINT` 下发小步关节位置。RAW 在底层仍经过 2 Hz 低通滤波；仅视觉命令将 `interpolation_acceleration_time` 设为 0，关闭默认 5 秒目标追赶斜坡。视觉阶段不调用 CuRobo 的 MotionGen、IK 或 CUDA 碰撞计算，也不生成六路点轨迹。
5. 探测和最终到位仍等待反馈；最终确认通过后回到原抓取流程。可靠特征点少于 6 个时先保持并确认，再冻结 Y/姿态，以距离终点完成前伸。保持、探测、前伸和最终到位的 Z 都使用本轮最初参考高度，不能用丢点或保持后的测量 Z 重设平面。

一个进程统一持有面板 UDP，按动作选择后端，避免两个控制进程争用输出。模型几何在启动时从同一机器人配置复制到 CPU；整个程序仍需要 CuRobo/CUDA，用于普通动作。这里是上层关节位置伺服，不是修改电机控制器，也不保证硬实时。

## 当前速度参数

后端配置：`/mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_planner/scripts/v3_1_direct_servo.json`。

- `period_s=0.02`：目标控制周期 20 ms（50 Hz）。
- `raw_velocity_ratio=0.3`：仅视觉 RAW 输出使用底层 30% 速度比例，普通动作恢复自己的参数（默认 10% 和 5 秒时间）。RAW 时间固定设为 0，跳过带整数截断计时问题的时间斜坡分支；没有修改或重新编译实机 RT 控制器。
- `linear_speed_m_s=0.05`：TCP 平移速度上限 50 mm/s（由 40 mm/s 提高 25%；实际还受关节限制、误差大小和缩步约束）。
- `angular_speed_rad_s=0.15`：TCP 角速度上限约 8.6°/s。
- `joint_speed_rad_s=0.35`、`joint_acceleration_rad_s2=1.0`：关节速度、加速度限制。
- `joint_lead_rad=0.025`：指令相对反馈的关节领先上限，整组关节按同一比例缩放，避免分别截断破坏保持高度所需的关节配合。
- `position_gain=3`、`rotation_gain=3`：后端误差收敛系数；不同于视觉端的 `gain=0.6`。

视觉端 `cart_book_grasp_v3_1_stream.json` 使用 `gain=0.6`、`lookahead_mm=null`、最多 10 Hz 更新和旋转前瞻 2°，仍受总路径预算和 X 终点约束。解除底层 5 秒斜坡后，原 gain=5 在实机记录中出现大幅 Y 振荡，已降低视觉纠偏增益；50 mm/s 平移上限、50 Hz 后端和前伸前瞻保持。前瞻无限不表示速度无限。底层面板的速度比例、关节控制和实际负载还可能进一步限制速度。

视觉阶段不再使用原来的 `time_dilation_factor=0.9`。普通 CuRobo 动作继续使用原参数。底层 RAW 时间为 0 并不表示瞬间到位：低通滤波、速度限制、上层关节速度/加速度/领先量约束和独立高度监测继续有效。

## 反馈、保持与高度检查

- 图像与源位姿仍按时间戳配对，不放宽原 3 mm / 1° 支撑范围、0.1 秒间隔及新鲜度限制。
- 连续视觉目标超过 0.4 秒不更新：固定当时关节位置保持；新目标恢复后从最新反馈继续。配对恢复最多 2 秒，持续失败由客户端退出。阻塞探测/最终命令期限为 5 秒。
- 单控制周期间隔超过 0.1 秒、反馈不可用或碰撞检查失败：锁存故障并保持；不继续发送新运动目标。
- 固定高度是右手 TCP 的基座 Z。预测关节路径默认使用 2 mm 高度范围；若反馈已在 2–3 mm 内，只允许预测路径不进一步扩大偏差。独立状态线程在实测偏差达到 3 mm 时保持并锁存故障。客户端 4 mm 停止界限保留。
- 后端测得高度偏差达到 1.5 mm，或候选路径需要缩至 1/8 及以下时，进入 Z 优先恢复：X/Y/姿态目标固定在恢复入口处，Z 指向原高度。保留最后已下发的关节位置指令并清零速度状态，避免把实测跟踪误差当作新的关节指令跳变。偏差回到 0.75 mm 以内恢复原目标；连续 2 秒未恢复则报 `Direct servo Z recovery timed out`。恢复期间不发布运动到位状态。视觉目标断流保持、碰撞检查、3 mm 独立停止及客户端 8 秒无进展期限都继续有效。
- 预测 Z 超限时，将最新反馈到候选关节指令的完整领先量依次缩至 1/2、1/4，最小 1/128；采用第一条符合高度限制的路径，再进行碰撞检查。接受缩步后清除原速度积分状态，下一周期从实际接受的指令继续，不能积累被拒绝的增量。
- 缩步仍不可行时固定当时关节位置保持，最多等待 1 秒恢复；保持路径本身越界或持续无法恢复则锁存故障。保护性缩步/保持不保证加速度连续。`V3.1 direct Z backoff` 日志输出参考高度、实际高度、缩步前后预测偏差、采用比例和限制；日志最多每 0.5 秒一次。
- 同一视觉阶段不可借新请求改变高度参考。最终到位标准仍为 2 mm / 1°。
- ACK 绑定实际下发的请求副本；未下发的结果不确认。左臂保持；故障保持保留夹爪指令，不自动松手或复位。
- CPU 碰撞检查采用配置中的机器人球体、自碰撞忽略对和 cuboid 障碍物；遇到不支持的障碍类型会拒绝启动。当前世界配置没有实时书架/书本障碍模型，不能将该检查理解为实时场景避障。

保持通过原面板 UDP 发送，不是硬件急停。离线模拟未覆盖真实跟踪误差、通信丢包和机械负载，实机表现待验证。

## 运行

需要重启 **V3.1 适配器和抓取程序**。旧版与新版使用同一面板，不要同时运行。通信入口仍为 agent 10175 / planner 10176；启动指纹包含新后端及其配置，防止误连旧进程。

终端一：

```bash
cd /mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_planner
/mnt/ssd/robotarm/env/curobo/bin/python dual_v2_2_upper_planner_v3_1.py --check-config
/mnt/ssd/robotarm/env/curobo/bin/python dual_v2_2_upper_planner_v3_1.py
```

终端二：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python cuart_book_grasp_v3_1.py --check-config
/mnt/ssd/robotarm/env/robotram_wx/bin/python cuart_book_grasp_v3_1.py --assume-head-camera-calibrated
```

进入视觉阶段应看到：`V3.1 visual backend: CPU differential servo, RAW joint positions; MotionGen bypassed; raw_target_time_s=0, raw_velocity_ratio=0.3`。

`--check-config` 不连接设备；正常抓取入口会执行复位和抓取。新适配器启动仍初始化原规划器所需的 GPU 和本地插值桥。

## 离线验证

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_1_direct_servo test_v3_1_execution test_v3_1_feedback test_v3_1_pose_time test_v3_1_stream test_v3_direct_distance test_v3_edges test_v3_centerline test_v3_column test_cart_book_grasp_v3_distance test_cart_book_grasp_v3_wrist_center test_v3_servo_arrival test_v3_planner_microsteps test_v3_camera_stream -q
/mnt/ssd/robotarm/env/curobo/bin/python verify_v3_1_execution_gpu.py
/mnt/ssd/robotarm/env/curobo/bin/python verify_v3_1_execution_gpu.py --lag
/mnt/ssd/robotarm/env/curobo/bin/python verify_v3_1_execution_gpu.py --raw
/mnt/ssd/robotarm/env/curobo/bin/python verify_v3_1_execution_gpu.py --raw --recover-z
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_1_raw_panel -q
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_1_visual_closed_loop -q
```

GPU 脚本不创建 UDP、相机或实机连接。它对照 CPU/GPU FK、碰撞模型，验证普通 MotionGen/插值接口，并在真实适配器中禁止调用 MotionGen，运行 750 个周期，测试 X 运动、Y/旋转纠偏、继续 X 前伸和实际协议打包。默认使用理想关节跟随；`--lag` 加入 60 ms 指令延迟，以及各关节不同的约 60–125 ms 一阶跟踪时间常数。带滞后的一次本机验证控制计算平均 3.37 ms、P99 3.52 ms，最大实际 Z 偏差 0.632 mm。单元测试另覆盖预测偏差 4.8 mm 时缩至 1/4、碰撞拒绝、无可行步保持/超时和积分清除。这些是离线结果，不代表已复现全部实机动力学或验证实机精度。

后续 `--raw` 验证还运行本地 RT 源码的 `InterpolatorFactory::kNone`，按 1 ms 子步执行有状态的真实 RAW 滤波，再加入上述指令延迟和跟踪滞后。750 周期检查通过：滤波路径 Z 偏差最大 0.756 mm，模拟反馈 Z 偏差最大 1.222 mm，最终位置误差 0.026 mm。控制计算平均 3.31 ms、P99 4.10 ms。独立标量对照中，仅将目标时间从 5 秒改为 0，3 秒内位移从 0.014100 rad 提升到 0.200196 rad（目标 0.2 rad）；这不是实机速度倍数保证。普通动作参数恢复测试也通过。

视觉外环回归 `test_v3_1_visual_closed_loop.py` 使用本轮记录的图像 Jacobian、合成相机误差、实际 `run_stream`、关节差分控制器、真实 RAW 滤波和延迟/关节滞后。原 gain=5 可复现累计横向路程过大并触发预算；gain=0.6 完成 100 mm 前伸和约 20 mm 横向纠偏，总路径约 129 mm。运动学和成像模型仍是简化模型，不包含实物特征丢失，不能代替实机闭环验证。路径预算没有放大；失败信息增加已用路程、请求路程和预算上限。

`--raw --recover-z` 在前伸中通过真实模型 Jacobian 注入持续的关节跟踪偏置，将测得 Z 偏差提高到约 2.04 mm，验证恢复高度后继续追踪原请求。该扰动测试从 2 mm 预测范围之外开始，要求恢复全过程的滤波与模拟实测高度都不触及原 3 mm 停止边界；无扰动 RAW 测试仍要求偏差小于 2 mm。单元回归另覆盖丢点后的测量偏差不能改变高度参考、恢复超时停止、断流保持优先以及恢复阶段不新增 X 前伸增量。
