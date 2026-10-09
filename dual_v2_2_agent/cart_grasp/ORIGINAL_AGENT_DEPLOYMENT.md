# 原 Agent 作为主体（2026-09-22）

用户最终选择 vlmdemo@10.30.13.21 上的原项目 `/mnt/ssd/user/V/dual_v2_2_agent`，不用 miniagent 作为入口，也不把运行环境迁到 kara。本次扩展位于原项目的 `cart_grasp` 子目录。

实际核验：

- 原 `RobotRAM/controllor/hkstp_arm.py` 可在 `/mnt/ssd/robotarm/env/curobo/bin/python` 中导入；没有实例化 RobotArm，没有绑定控制 UDP。
- 原规划器为 `/mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_planner/dual_v2_2_upper_planner.py`，共享配置为 `/mnt/ssd/user/V/dual_v2_2_configuration`。
- 目标使用原 `TargetCommand` 格式，经 10165/10166 交给规划器，再经上位机 Remote 接口 30240/30241 转换成实际控制。端口仍是原配置，**尚未修改为双机部署或接通到实体驱动**。
- GPU 0 为 RTX A6000；检查时占用约 607 MiB，GPU 1 已有其他工作。新增脚本限制 BLAS/OpenMP 单线程；检查及报文测试不运行 GPU 推理。后续 CuRobo 仍需单独限制显存、种子数和规划并发，不能把 `CUDA_VISIBLE_DEVICES=0` 当成 GPU 使用率限制。
- 新增目标格式及坐标转换共 14 项测试通过：左右臂顺序、原协议往返、保留未动手臂和手爪、错误单位/四元数/坐标拒绝、测量姿态外展候选、半身根及工具偏移转换。

在 vlmdemo 上进行无动作检查：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent/cart_grasp
bash run_original_agent_check.sh
```

`tools/upper_planner_target.py` 仅生成可审阅的 `__preview__:` 报文，默认和当前唯一模式均不发送网络数据；它不是抓取执行器。原 Agent 的所有原有任务入口、权重及控制源文件均未被本次适配改写。没有运行餐具任务入口或固定复位动作。

## 抓把阶段

当前实测姿态 → 双臂外展 → 分别抬臂 → 车把外侧预抓取 → 重新观测并微调 → 分别接近/闭合 → 确认抓持。

外展候选从当前 V2 关节位置生成，只用于规划器预览，不能直接当作无碰撞路径。两臂绕行必须连同推车一起规划，不使用原餐具任务的复位角、固定高度、手爪姿态或成功判据。接近阶段完成后应单独评估手指闭合碰撞和右爪实际响应，不能用命令回显宣布抓牢。

## 坐标及剩余接线

已有车把观测和 ArmModel 法兰目标在 SJ4Z 下；原规划器使用半身 dummy_world，并在法兰上加入配置的 `offset_from_ee`。应从已核对的 URDF 计算 `half_T_upper`，然后得到 `half_T_tip`，最后将旋转后的配置偏移加到位置。该偏移与真实抓持 TCP 不同，不得重复加/减 342 mm 或直接把接触点当作规划器工具点。

原 CuRobo world 只有 Z=-10 m 的占位 floor，尚无当前推车和下身环境。必须把当前车体及环境接入世界模型；原 Agent 对餐具的预览成功不代表推车路径安全。

erazer 重启后尚需恢复相机 RGB-D 和手爪设备。原生 Upper Software 的 Qt6 运行库、原生 RT 的部分驱动库在 erazer 的默认加载路径下缺失；8780 网页上位机不能直接被当成 30240/30241 原生 Remote 接口。要验证实际选用的上位机/RT、V2 关节约定、断流停止与恢复时不重放，再开放实体执行。

这份记录确认原 Agent 环境与目标格式适配已完成，**不表示双机实机控制或完整抓把已完成**。
