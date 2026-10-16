# V3：CoTracker 对准 + V1_3 距离终点

此模式现在通过 `V3.py --legacy-cotracker` 进入。默认 V3 已改为 [快速中心列对准](README_cart_book_grasp_v3_column.md)，只纠偏 Y、保持 Z。本旧模式接受本次约定：第一步沿 Y 粗对齐后，腕部中心最近的书就是头部深度对应的目标书。不进行头腕身份匹配，也不读取原书的最终纹理示教图。

## 运行

旧的抓取进程需在暂停提示输入 `s` 退出，再启动新入口。按回车仅开始旧进程的新一轮，不会加载修改。

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python V3.py --legacy-cotracker --check-config
/mnt/ssd/robotarm/env/robotram_wx/bin/python V3.py --legacy-cotracker --assume-head-camera-calibrated
```

第一条检查本地模型、配置与腕部标定，不打开相机或连接机器人。第二条启动完整抓取循环，先执行原 `first_reset`，不是从当前人工摆放位置直接续跑。

启动提示应为 `V3 模式：CoTracker 当前书点跟踪 + Y/Z 视觉对准 + V1_3 目标 X 停止`。

规划器微步修复：`dual_v2_2_control/cuarm_upper_planner/scripts/curobo_planner.py` 为 `book_v3_xyz_servo` 使用 0.01 mm 的目标变化判断，避免原 5 mm 阈值吞掉 2 mm 探测；规划器到位标准为 1 mm / 1°，该动作只允许右臂参与规划。普通动作保持原判断。首次使用该修复需在抓取程序退出后，按原启动方式重启上层规划器，再启动 V3；仅重启 V3 不会重新加载规划器代码。仍走原有 MotionGen 与碰撞检查。

## 行为

1. 保留 V1_3 的粗对齐、姿态、第一段粗前伸及后续夹取/抬升/放置流程。
2. 本轮开始时固定 V1_3 的 `grasp_tcp_x_mm`：书本基座 X 深度 + 伸入量 − 指尖偏移的 X 投影。
3. 第二段观测时仅用腕部图检测中心最近的可靠书本，在其可见中部初始化最多 8 个分散点，至少需要 6 点支持。每轮只选一次书、初始化一次点。
4. 用 CoTracker3 跟踪同一组点，估计固定跟踪区域中心。可见点不足、点几何不一致、跳变、图像过旧或重复时停止，不重新选择邻书或补选新点。
5. 初始以 Y/Z 各正负 2 mm 探测并返回，测量 2×2 Image Jacobian；不沿 X 探测。探测返回只恢复 Y/Z，保持最新实测 X，避免为反馈微小偏差额外发送 X 回退指令；实测 X 越界检查仍有效。随后保持姿态，用跟踪区域中心误差修正 Y/Z，前伸时结合图像尺度变化调整局部增益。
6. 图像误差不超过 3 px 才前进 X，每次第二段移动最多 5 mm；偏离后先纠偏。最后一步由剩余 X 距离截短，不发送超过目标 X 的前伸命令。
7. 实测 TCP X 误差不超过 1 mm、图像误差不超过 3 px，连续 3 个新鲜观测均满足，才将前伸标为完成并交回 V1_3 合爪。只有图像对准或只有 X 到位都不能放行。

每个 `book_v3_xyz_servo` 小步采用专用到位检查：位置误差 ≤1 mm、姿态误差 ≤1°，连续 3 个不同的 UDP 接收包在 0.5 mm / 0.2° 窗口内稳定。接收时间必须晚于本次指令发送且包龄 ≤0.5 秒；重复读取缓存不能累计到位次数。控制器报告结束后最多再等 10 秒，未到位则停止并打印目标、实际位姿和误差。通过后才返回拍照/跟踪流程。接收时间是本机收到包的时间，协议没有硬件测量时间戳。

在线 Jacobian 更新使用实测 Y/Z 位移，但只在位移达到 `max(1 mm, 2 × max_pose_error_mm)` 时学习（当前 2 mm）；移动前或后的图像误差进入 `max(10 px, 3 × tolerance_px)` 内时暂停学习（当前 10 px）。仍保留图像尺度增益调整。Broyden 候选更新若条件数超过 `min(max_condition, 10, max(3, 1.5 × 原条件数))`、最小奇异值明显下降、行列式变号或矩阵相对变化超过 25%，则丢弃候选，保留原模型。实际像素变化与模型预测相差超过 4 px 时仍停止，不因跳过更新而放行。`events.jsonl` 中的 `jacobian_update` 记录接受/拒绝原因、实际位移、预测/观测像素变化、残差和条件数。

采用停稳拍照后的短窗口跟踪，不是运动中连续高速视频控制。CoTracker3 本地离线模型在最多 8 张已获取帧上推理，滑动窗口运输已有点的查询位置，保留原点身份和原始对准区域。没有预测尚未获取的未来相机帧。

腕部相机由独立常驻采集进程持有：每轮视觉前伸首次拍照时打开一次，运动期间持续读取、丢弃图像，结束或异常时关闭。每次停稳请求后丢弃预热帧（当前 4 帧），返回请求之后开始读取的新帧，并检查请求编号、读取时间、图像年龄和标定分辨率。默认 6 秒采集超时会停止本轮，不使用旧图继续伺服，也不会在本轮自动重连后继续运动。帧时间是本机读取时间，USB 相机没有提供硬件曝光时间戳。采集进程退出慢时仅额外记录清理警告，不覆盖原始采集失败或人工中断。

## 配置

新参数独立保存在 `cart_book_grasp_v3_cotracker.json`。V1_3 的 `cart_book_grasp_v3_config.json` 继续提供指尖偏移、伸入量和粗前伸等参数。

- `target_uv: null`：当前使用去畸变画面中心 `[639.5, 359.5]`，可改为实际标定/示教的夹爪对准像素。画面中心不等于已标定的夹爪对准点；本次没有新增相机—夹爪标定。
- `max_step_mm: 5`、`probe_mm: 2`：第二段 X 前伸及 Y/Z 纠偏的单步上限为 5 mm，初始 Y/Z 探测量仍为 2 mm。纠偏增益当前为 `gain: 0.3`。
- `max_lateral_mm: null`：关闭相对第二段起点的 Y/Z 合成偏移半径限制，不再受原 20 mm 范围约束。仍保留每步上限、累计路径/时间预算、跟踪检查和原规划器的关节限位及碰撞检查。需要恢复时可设为 1～30 mm 内的数值，且不小于探测量。
- `max_forward_mm: 250`：从第二段观测起点计算的前伸上限；超限会停止。实际终点仍由本轮 V1_3 深度确定，不会固定前进 250 mm。
- `max_path_mm: 400`：第二段累计移动路径上限，包括 X 前伸、Y/Z 探测往返和纠偏；不包含第一段粗前伸。每步最多 5 mm，最多 200 次迭代及 600 秒超时限制。
- `max_frame_age_seconds: 2`：包括模型推理时间的图像年龄限制。
- `device: auto`：有 CUDA 时使用 GPU；CPU 模式推理如果超过年龄限制会停止。
- 原来的 `cart_book_grasp_v3_approach.json`、`reference` 和最终书本图片不控制默认新模式。

运行输出位于 `RobotRAM/output/v3_cotracker_<id>/`：初始腕部图、选书 mask、初始化点与目标像素、冻结的目标 X、逐帧点图及 `events.jsonl`。出现 `converged_distance` 才表示本轮前伸条件通过。

## 不动机器人的验证

```bash
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_cart_book_grasp_v3_distance
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_servo_arrival
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_planner_microsteps
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_camera_stream
/mnt/ssd/robotarm/env/robotram_wx/bin/python validate_v3_cotracker.py
```

亦可对实际保存的多帧去畸变腕部 PNG 运行 `V3.py --replay-cotracker FRAME1.png FRAME2.png ... --mask INITIAL_MASK.png`。省略 mask 时，第一帧自动检测中心书；不连接相机或机器人。此工具只验证点跟踪，不代表实机闭环或抓取已验证。

本次 GPU 离线验证结果：三张实拍图上的 8 个点均可见；11 帧已知合成平移/缩放（跨越窗口切换）的中心最大误差约 0.882 px；全黑遮挡图被拒绝（0/8 可见）。单窗口推理实测约 0.050–0.419 秒，包含首次推理；这不是完整控制循环频率。没有运行实机抓取。

源码来自 https://github.com/facebookresearch/co-tracker，保存在 `third_party/co-tracker`；官方权重本地保存在 `third_party/cotracker_checkpoints/scaled_offline.pth`。验证报告记录源码提交与权重 SHA-256；运行时不自动下载或改动 CUDA 可见性。

## 旧模式

- `V3.py --legacy-final-image ...`：原最终示教图 XYZ 模式。
- `V3.py --legacy-yz ...`：原 Y/Z 点示教模式。
- `--match-reference`、`--match-head-wrist`、`--preview-wrist-center` 仍是旧模式的离线诊断工具，不代表默认运行模式。
