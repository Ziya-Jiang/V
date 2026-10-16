# V3.2：丢点后统一参考与有限时间恢复

入口是 `cuart_book_grasp_v3_2.py`，独立配置为 `cart_book_grasp_v3_2_stream.json`。
复用 **V3.1 直接伺服适配器** `dual_v2_2_upper_planner_v3_1.py` 和原通信端口 10175/10176；不是新增一个同时控制面板的进程。原 V3.1 入口、配置、控制循环及适配器代码均未改动，共用运行入口仅增加按 `runtime_version="3.2"` 选择模块的分支。

## 这次改变

- 丢点时一次锁定 Y、完整三维姿态和原始 TCP 基座 Z。保持动作、后续目标、运动中检查和最终确认都使用同一参考。保持后实测误差不再变成另一份参考。
- 检查完整旋转差，不再只检查绕工具轴的 roll。旋转矩阵计算避免欧拉角跨 ±180°造成假报警。
- 运动中 Y 偏差超过 1 mm 或完整姿态偏差超过 0.5°：停止增加 X 前伸目标，继续纠正到锁定的 Y/Z/姿态；原在途命令及机械惯性不会瞬间消失。
- 回到 0.5 mm / 0.25° 内，至少三个不同时间戳反馈且持续 0.2 秒，才恢复前伸。重复反馈不算稳定样本。
- Y 偏差超过 4 mm 或姿态超过 2°立即停止；恢复超过 2 秒仍不收敛则停止。这两个运动中停止阈值独立于最终到位 3 mm / 1°，并非取消偏差检查。原独立高度保护、反馈时效、碰撞、关节速度限制、路径预算均继续生效。
- `direct_pose_feedback` 每 0.5 秒记录参考/实测 XYZ、完整姿态、Y/Z/旋转偏差及恢复状态；恢复开始、完成、超时或越界单独记录。没有增加图像可视化频率。
- 进入最后 20 mm 后，X 目标领先距离缩为剩余距离的 35%，降低 RAW 滤波和跟踪滞后导致的终点超调。最终 X 目标仍是 V1_3 深度终点，原 2 mm 终点边界不放宽。
- 正常连续伺服仍最多 10 Hz 更新视觉目标，后端目标周期 50 Hz、TCP 平移上限 50 mm/s、视觉 gain=0.6。只有恢复期间暂停前伸；不是每一步停稳再规划。复位、粗定位和后续抓取仍使用原 CuRobo 流程。

丢点后的距离前伸仍基于先前选中书与深度一致的假设，完成结果会明确标记 `visual_alignment_verified=false`，不把开环前伸当作视觉对准验证。

## 运行

此次将位置到位阈值调整为 3 mm 后，需重启 V3.1 适配器和 V3.2 抓取程序，使两端标准同步。不要并行运行两个抓取客户端或两个面板控制器。

适配器没有运行时，在一个终端启动：

```bash
cd /mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_planner
/mnt/ssd/robotarm/env/curobo/bin/python dual_v2_2_upper_planner_v3_1.py
```

另一个终端：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python cuart_book_grasp_v3_2.py --check-config
/mnt/ssd/robotarm/env/robotram_wx/bin/python cuart_book_grasp_v3_2.py --assume-head-camera-calibrated
```

`--check-config` 只读配置及标定，不连接设备；正常入口会执行复位和抓取。
恢复旧版只需重新运行 `cuart_book_grasp_v3_1.py`，使用它自己的配置。

## 验证

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_2_stream test_v3_1_direct_servo test_v3_1_stream test_v3_1_execution test_v3_1_feedback test_v3_1_pose_time test_v3_1_visual_closed_loop test_v3_1_raw_panel -q
```

新测试覆盖锁定后停稳偏差、不重复计数旧反馈、短暂超过旧 2 mm 边界后恢复、恢复时不推进 X、持续恢复超时、完整姿态及硬边界、最终确认失败和实际发送姿态一致性。另以真实 RAW 滤波和延迟关节响应驱动简化运动学的完整控制循环，验证视觉前伸中丢点后转距离模式并到达终点。

测试为离线模拟，未执行实机动作，也未声称已复现这次实机的具体 Y/旋转偏差。

V3.2 的位置到位阈值在 `cart_book_grasp_v3_2_column.json` 的 `max_pose_error_mm=3.0`；共享直接伺服后端在 `scripts/v3_1_direct_servo.json` 使用 `arrival_position_mm=3.0`。V3.1 客户端自己的 2 mm 确认不变。X 终点容差仍为 2 mm，姿态为 1°；高度恢复阈值和独立高度停止边界不随到位容差放宽。

## 腕部初始选书：模型检测与边界备用

仅 V3.2 使用新选书模块；V3/V3.1 原选书规则不变。选书仅发生在粗前伸完成后，后续仍跟踪同一目标，不逐帧重新选书。

1. `book` 只定位整书，置信度再高也不直接用于中线或选点；始终再检测 `book spine`，避免斜视时封面进入伺服区域。
2. 书脊候选需满足两侧宽度随行坐标平稳变化、跨度、中心行等几何约束。候选分数 ≥0.15 且至少 90% 位于整书掩码内可接受；独立明确书脊分数 ≥0.5 也可接受。不要求整书与书脊掩码 IoU 相近，因为可见封面本来会扩大整书掩码。重叠候选中优先明确窄表面，避免再选到包含它的宽封面区域，不固定假设封面位于左侧或右侧。
3. 模型路径仍失败，按用户要求从图像左右边界计算中线：Canny/Hough 找长边，合并有连续支持的共线片段，过滤短文字笔画；要求两侧有持续亮度阶跃、跨越图像中心行、共同支持高度至少图高 30%、宽度变化符合透视且位于中央区域。只使用实际观测到的共同区间生成跟踪掩码，不向看不见的上下区域扩展。多组同样居中或共用边界的相邻条带存在歧义时停止，避免选到两本书之间的空隙。
4. 中线由书脊左右边界的中点拟合得到，不使用特征点重心。纹理点只在书脊每行中间 60% 内选取，同时离可见区域边界至少 11 px，为 21 px LK 窗口留出基本余量；保留遮挡孔洞，不为凑够点数扩到封面。选定后初始化纹理点传递中线；若初始纹理不足仍停止，不直接盲目前伸。边界备用结果标记 `semantic_identity_verified=false`：它依赖粗对准后中央书为目标书的假设，不提供书本身份验证。

每轮输出新增 `wrist_detection.json`、`wrist_detection_book.npz`，以及 `wrist_detection_book_spine.npz`、`spine_tracking_mask.png`，记录原始分数、面积、淘汰原因、边界支持和最终使用的方法。仍不增加伺服循环的图像输出频率。

本轮失败图像的真实 SAM3 离线复现：book 0.17264 / book spine 0.20690，掩码 IoU 0.99513；新规则可选中并初始化 80 点。强制两个模型输出都为空时，左右边界得到中心列 653.30 px，与模型掩码的 653.79 px 相差约 0.49 px，也初始化 80 点。该图上的边界选择、诊断写盘及初始化合计约 94 ms（不包含模型推理），不是实机控制频率保证。未执行机器人运动。

结果位于 `RobotRAM/output/v32_wrist_recall_20261011/`；可查看 `edge_only/selected.png`（黄线为观测区间的两侧边界，红线为中线）。此次只需重启 V3.2 抓取程序，适配器无新增改动。

斜书回放（`v3_column_77cb47e224b648d7b3f81646477825ac`）：原整书中线 u=569.95 px，显式书脊中线 u=504.96 px（图像中心行），相差约 65 px；书脊角度 36.42°，内侧初始化 80 点。对比图为 `RobotRAM/output/v32_spine_only_20261011/spine_only_result.png`。模型分割及边界识别仍可能失败，以上是保存图像回放结果，未进行实机运动验证。本次仅需重启 V3.2 抓取程序。

## 头部 SAM3 失败后的 GPT-6 备用分割

V3.2 中头部 `book spine` 检测没有通过原有几何/深度检查的候选时，自动调用一次 `gpt-6-astra`。正常 SAM3 成功路径不调用 GPT。此功能用于粗定位之前的头部识别，不是腕部逐帧伺服。

GPT 接收当前头部 RGB，返回同一本书的书脊四边形及整书四边形；SAM2 复用当前帧已准备的 embedding，通过框和内部点细化分割。书脊候选若大量扩到封面会拒绝，不会简单裁剪后当作成功。之后依然经过 V1_3 的倾斜书脊几何、RGB-D 内参/外参、中心深度及整书深度检查；GPT 不输出机器人位姿。模型无明确结果、API 超时、缺少凭据、SAM2 与定位不符或深度无效时，本轮停止。

配置：`cart_book_grasp_v3_2_head_fallback.json`（默认启用、模型 `gpt-6-astra`、请求超时 60 秒、最低返回置信度 0.7）。模型返回置信度只是准入条件，不是校准后的成功率。优先读取启动终端的 `OPENAI_API_KEY`；未设置时读取 `api_key_file`，当前为 `/mnt/ssd/user/V/cart_book_grasp_openai_key.txt`。文件只放一行 Key；配置中只保存路径，不保存密钥内容。仅触发备用流程时会把该帧图片发送至 OpenAI Responses API，`store=false`。

每次记录保存到 `RobotRAM/output/v32_head_gpt_<scene_id>/`：头部 RGB、模型响应/失败原因、通过分割校验后的书脊和整书掩码。`segmented_pending_geometry_depth_checks` 表示分割阶段通过，后续几何/深度尚未确认，不能当作抓取成功。

此次只需重启 V3.2 抓取程序，原规划器/适配器无需改动。已通过离线模拟 API/SAM2、失败分支、深度检查及调用链测试；已接入用户提供的密钥文件，并通过 OpenAI 官方接口验证 `gpt-6-astra` 模型访问权限；尚未做真实图像分割联调或实机运动。

接口依据：[GPT-6 Astra](https://developers.openai.com/api/docs/models/gpt-6-astra)、[Responses Structured Outputs](https://developers.openai.com/api/docs/guides/structured-outputs?api-mode=responses)。
