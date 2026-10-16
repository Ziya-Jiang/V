> 历史独立实现说明：当前入口不再使用此流程，见 README_cuart_book_grasp_v4_1.md。

# V4.1：头部上沿粗定位，复用V3.4执行逻辑与规划器

当前入口：`cuart_book_grasp_v4.1.py`。任务实现为`book_top_model_coarse.py`，执行适配为`book_top_v34_runtime.py`，配置为`cuart_book_grasp_v4_1_config.json`。

按2026-10-14用户要求，当前入口使用与`cuart_book_grasp_v3_4.py`相同的`dual_v2_2_upper_planner_v3_1.py`，客户端10175、规划器10176。**不再使用`dual_v2_2_upper_servo_v4_1.py`（10186）**。V3.4入口、共享规划器和RT代码未因本次适配而修改；旧独立V4.1实验入口仍保留，但不是这里的启动方式。

## 复用范围

- 使用V3.4的启动指纹检查、`dedicated_planner`反馈解码器和`RobotArm`。
- 复位沿用`control_joint_position`，并通过V1_3现有热加载函数读取`cart_book_grasp_v3_config.json`的`initial_j6_deg`。其他书本抓取动作参数不决定本流程的上沿目标。
- 闭指调用普通`master_control(right_gripper=1)`，完整保留复位关节指令类型、左右臂目标与指令编号，避免把历史笛卡尔目标重新激活。确认实际夹爪闭合反馈后才感知。
- Y/Z/X/Z移动调用普通`master_control(right_arm=...)`，动作名依次为`move_above_grasp`、`lift_object`、`move_down_to_grasp`、`move_down_to_place`。全部走现有CuRobo普通规划，不发送视觉流式或新V4.1闭指专用命令。
- 保留本任务实际到位3mm/1°和新鲜稳定反馈检查；共享方法打印的默认30mm/8mm之后会打印本段实际3mm标准。执行期间仍检查保持轴、姿态、左臂、夹爪与反馈新鲜度。

## 五步顺序

1. 右臂`first_reset`，保留左臂；关节到位后按此前要求闭合右夹爪。
2. 头部选择一本可靠可见的书；仅沿Y对齐书脊中心，X/Z保持。
3. Y到位后重新识别同一本书的实际上沿；仅抬Z，使名义指端位于上沿上方50mm，X/Y保持。
4. 仅沿X前移，依据同一本书区域实测深度中心计算，Y/Z保持。
5. 仅沿Z下降至上沿下方20mm；从上方50mm目标到这里名义下降70mm，实际从实测高度计算剩余位移。

这版不运行V3.4腕部伺服，也不接它的原抓取、抬升200mm、松爪抽回等后续动作。默认单轮。`--through-step 4`可只运行到上沿上方。

## 几何与感知

默认自动选择一本书，不要求`--target`；该选项仅供指定对象。第二次感知必须保持初次选定的同一本书。

模型明确输出`top_center_uv`（真实物理上沿中心）和`spine_center_uv`（可见书脊中心），同时给出用于显示/辅助采样的书脊和整书区域。当前主流程直接在这些语义点附近读取实测深度，坐标来自共享头部相机的校准RGB-D，使用最高原始分辨率。正常情况仅一次模型调用，之后取三帧新数据检查画面变化和3D散布。书底裁切时中心标为可见书脊区域中心，不虚构不可见全书中心；不能把页块/封面交界或文字边缘当作真实上沿。

已移除本入口的书脊必须包含在整书框内（3px）硬拒绝、400px最小面积/凸四边形门槛，以及整面平面拟合、80%点距平面2mm、平面残差和可观测轴长规则。两个近似区域不要求严格相互包含，也不用其顶点平均数代替模型明确给出的语义中心。仍检查响应身份、模型是否确认同书/上沿、有限数值、坐标在图像内、区域非空等基本输入有效性。原其他任务的面跟踪算法没有删除。

每次感知前（闭指后、Y对齐后）最多等待3秒，确认至少3个新反馈覆盖连续0.3秒：双臂TCP窗口位移不超过0.25mm、转角不超过0.15°，再建立采图锚点。关节速度超过0.005rad/s只警告，不清空稳定窗口、不直接取消任务。重复反馈不累计。此等待不发送新的运动指令。

当前粗定位入口使用独立的`HeadCoarseRobotIO`观测策略：头部RGB-D几何不依赖右手TCP，因此右手超过旧0.5mm门槛不再被判作图像失效。手臂保持仍从观测开始的固定锚点检查，`head_observation_hold_mm=3.0`，不超过本任务位置容差；不会每帧重置锚点掩盖缓慢累积位移。超限明确报`Head observation arm hold exceeded`，日志记录本次观测最大TCP偏移。姿态按本任务1°预算检查；原左臂1mm、夹爪状态、后端状态和反馈新鲜度保护保留。

采图/模型期间，有效关节速度超过0.005rad/s只记录`head_observation_velocity_warning`，包括超过0.02rad/s的情况；每个阶段最多每秒一条，重复反馈不重复警告。速度值本身不触发任务停止，但实际位置/姿态越界、反馈缺失或非有限数值、反馈过期和控制器故障仍会停止。头部内外参从输入帧开始固定并在模型前后核对；所选区域图像变化、新鲜帧及三帧深度散布独立检查。下一动作从最新反馈计算起点和保持轴，不沿用观测前TCP。原独立指腹观测任务仍保留0.5mm等旧规则，原规划器和到位速度参数不变。

局部深度质量检查只警告：MAD超过6mm、有效点少于9个、稳健支持不足不再直接报错或触发GPT复核。以语义点为中心、半径3px读取范围内有效实测深度，继续使用离群过滤后的中位数；警告及质量指标写入终端、depth_samples.json与geometry.json。区域框只提供软采样支持，不到远处搜索替代深度。局部完全没有有效深度或整书深度不足，仍最多启用一次GPT同书复核；再次失败则停止，不编造三维位置。三帧3D散布超限、模型不确定或同书/真实上沿不可见仍停止。局部窗口深度是近似位置测量，降低拒绝率不代表精度提高或接触已确认。

头部观测的两次检查样本间隔超过0.25秒时，只记录`head_observation_sampling_gap`，不当成接收断流。每次检查仍分别验证最新接收时间`received_monotonic`和位姿时间`pose_sample_monotonic`距现在不超过0.25秒，真实过期/时间倒退仍停止。仅检查最新反馈不能证明未检查期间从未短暂断流，因此提示记录`continuous_reception_verified=false`；未扩大反馈有效期。

感知目录保存`failed_depth_frame.npz`、`depth_failure.json`及`depth_review_response.json`、`depth_review_selection.json`、`depth_review.jpg`，区分失败帧、模型复核和最后实测几何。GPT复核是运行时备用路径，可能增加一次API调用；离线测试使用模拟返回，不调用真实API。

令`r = R @ tcp_to_finger_tool_mm`，单位mm：

- Y目标＝书脊中心Y−rY。
- 抬升Z目标＝上沿Z＋50−rZ。
- 前伸X目标＝整书实测深度中心X−rX。
- 最终Z目标＝上沿Z−20−rZ。

名义工具偏移为`[0,0,-50]`，不是重新测得的双指下表面标定。最终位置完成记录`contact_confirmed=false`，不声称已经测到接触或20mm压缩。当前没有启用J1电流判据或拨书动作。

## 完整启动命令

先退出当前抓取程序，并停止旧10186独立V4.1后端；同一机械臂只保留一个占用Panel UDP的规划器。RT/Panel仍按原方式运行，完成Connect/Start。

共享头部相机：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m head_rgbd start
```

规划器终端（与V3.4相同）：

```bash
cd /mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_planner
/mnt/ssd/robotarm/env/curobo/bin/python dual_v2_2_upper_planner_v3_1.py
```

任务终端：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python cuart_book_grasp_v4.1.py --check-config
/mnt/ssd/robotarm/env/robotram_wx/bin/python cuart_book_grasp_v4.1.py --execute
```

`--observe-head`只调用头部模型和感知，不连接机器人；仍会使用已配置的GPT API。`--check-config`不调用API或设备。

## Rerun与退出

默认打开Rerun并保存`RobotRAM/output/v41_model_coarse_时间/head_coarse.rrd`：感知图像、书脊和上沿标注、深度、基座点云、阶段TCP目标、实测轨迹和状态。图像在感知时更新；实测轨迹旁路采样最多10Hz，不抢读UDP。`--no-viewer`只保存记录。查看器未能启动时在最多2秒后退化为文件记录；启动后查看器断线仍可能出现SDK提示。

按`s`无需回车，沿用V1_3语义：当前轮结束后退出。失败时保持并清理后等待`s`。Ctrl+C中断并尝试保持。不会自动松爪、复位、重复下降或清除故障；此入口不启用p松爪复位和Enter重试。

连接时只验证新鲜反馈与机器人可用性；已有失败状态交由一次新的first_reset请求按V3.4原规划器策略处理，不在客户端直接清除故障。新指令失败、碰撞或未到位仍中止，不继续闭指/下降。

粗段最多50mm，下降段最多10mm；分段目标仅改变一轴，但普通关节规划不等于严格笛卡尔直线，保持轴偏差仍在执行中监测。

## 验证边界

离线测试使用真实RobotArm指令构造方法与模拟UDP/反馈，覆盖闭指保留关节目标身份、普通笛卡尔动作与米/毫米转换、原规划器端口和启动检查、J6热加载、失败恢复包装，以及五步顺序。完整实机五步流程尚未验证；研发过程与此前V4.1独立后端问题保留在BOOK_GRASP_RESEARCH_LOG.md。
