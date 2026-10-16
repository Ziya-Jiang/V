# 手臂实时电流曲线（只读）

入口：http://127.0.0.1:8793/ 。默认右臂 J1–J7，可切左臂。

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
python3 -m arm_current_monitor --port 8793
```

正常启动现有 RT 和面板即可；曲线不依赖视觉规划器，不绑定机器人 UDP 端口，不发送使能、目标、保持或复位指令。Ctrl+C 只关闭此监视器及它创建的抓包子进程。与现有 AR 并行使用。

采集使用本机已安装 tcpdump 和已有 sudo setsid 权限，tcpdump 打开被动抓包后 `-Z` 降权到当前用户，HTTP 只监听 127.0.0.1。未增加系统权限或修改 RT。若迁移到另一台电脑，没有这些已有权限会在页面显示采集失败；不要为此给网页控制权限。

## 使用

1. 接触前在相近姿态静止，点击“记录静止基线（最近2秒）”。只改变显示零点，不给机器人清零。至少1.7秒连续数据、20帧，关节速度≤0.02rad/s、位置波动≤0.003rad才接受。
2. 用户在原控制界面操作手臂，观察原始电流 A 或相对基线 ΔA。七个独立子图方便查看小关节变化；总览可勾选关节。
3. “标记接触时刻”是人工备注，不是自动接触判据。可导出原始电流 CSV 和基线/标记 JSON。
4. 后端保留最近60000帧（约100Hz时10分钟），刷新页面不清后端；关服务会清除。页面图窗15/30/60秒，暂停只冻结显示，仍采集。基线/标记是当前页面内存，刷新前请导出。

## 单位核查及源代码

当前 dual_v2_2 使用 **Realman RobotArms**，不是目录中另一个旧 Eyou 驱动。

- `cuarm_rt_control/hardware_modules/realman_dual_arm/src/motor_sensor/motor.cpp:91,102`：反馈原始电流×0.001；`set_current(cur_amp)`相反乘1000，单位A。
- `.../robot_arm.cpp:556`：`get_current()` 返回 `cur_ * sign_`，保留关节方向约定。
- `cuarm_rt_control/src/arm/dual_v2_2_arm.cpp:106,122`：getTorque/getState 的tor参数实际接收上述电流。
- `cuarm_rt_control/example/rt_control.cpp:1303`：进入 `RobotState.MotorTor`；`sole_arm_planner.cpp:754` 写 `PlannerState.MotorTor`。
- `dual_v2_2_configuration/robot_platform_utils/cpp/src/cuarm_message_handler.cpp:718` 起：RT状态序列化；被动旁听本机 **目的21101**，发送源端口可能是临时端口，不能强制21201。

**不要把空的 `MotorCur` 字段当作实际电流，也不要因字段叫MotorTor就把数值当作N·m。** 单位结论限于本硬件/配置，换驱动后需重新核对。2026-10-13现场读到真实非零七路反馈，页面约97–98Hz反馈、约10Hz刷新（后者包含绘制耗时）。既无电流滤波，也无力换算。

## 证据边界

电流变化包含重力、加减速、摩擦、温漂与外部接触；不能单凭ΔA断言已搭上书，更不能将A直接当作接触力N。可先比较同姿态、相近速度的空载/接触曲线，再判断信号是否足够可靠。

校验PCAP内核时间及RT发送时间，重复/过期/非有限数据拒绝；>0.5秒断流显示离线，曲线断开不补零；反馈恢复自动继续。没有逐电机CAN采样时间，页面“实时”只能确认RT反馈流新鲜，不能证明每关节传感器未卡值。

8项解析/时效/协议字段测试通过，实际页面与静止基线功能已观测；仅终止本监视器新建的tcpdump验证断流提示和自动重连成功。未执行下压动作，未验证接触时电流变化幅度或可分辨性。
