# cart_grap_v1：右车把完整单轮

`cart_grap_v1.py` 是独立完整流程；原来的 `cart_grasp.py` 仍用于单独对准。

执行顺序：

1. 记录本轮初始右臂关节和 TCP，要求右夹爪已经张开。
2. 用头部最高原始分辨率 RGB，让 GPT 定位右车把标记区域。用新鲜、对齐的深度图及现有头部手眼标定计算基座坐标；GPT 不提供深度，也不冒充解码 ID。
3. 先保持 X/Y 和姿态，沿基座 +Z 抬高 50 mm；确认到位后保持新高度沿 Y 粗对准，再粗前伸总距离的 40%，最多 100 mm。
4. 腕部实际检测 ArUco ID 0，通过 Image Jacobian 纠正 Y 和工具轴旋转；Z 固定，图像对准时继续沿 +X 前进。
5. 最终图像和位置确认后闭合右夹爪，保持 3 秒，张开夹爪。
6. 确认新鲜的夹爪张开反馈后，沿**基座 −X 撤回 100 mm**；再用 CuRobo 返回本轮记录的初始右臂关节位置。

终点采用用户指定的抓书几何：

`目标 TCP X = 头部标记基座 X + 100 mm 伸入量 − 指尖偏移的 X 投影`

指尖偏移为 50 mm。100 mm 伸入量与松爪后的 100 mm 撤回距离是两个独立参数，均在 `cart_grap_v1_config.json` 中。前伸预算由 `cart_grasp_approach.json` 限制；本入口不使用其中旧对准模式的 20 mm standoff 计算抓取终点。

`alignment_raise_mm: 50.0` 将本轮固定工作高度设为启动时实测 TCP Z + 50 mm。它不是把绝对 Z 设为 50 mm，也不是对齐头部标记的 Z。前伸、视觉纠偏和正常保持均使用该工作高度；最终返回仍使用抬高前记录的关节与位置。抬高失败不继续横移。每次启动重新记录起点，所以失败后从已经抬高的位置重启会再增加 50 mm；应先恢复预期起始姿态再开启新一轮。

## 运行

本入口使用 `cart_grap_v1_alignment.json`，其中 `max_coarse_y_mm: null` 取消粗对准的 200 mm 横移限制。粗对准仍保持 X/Z，目标由 CuRobo 检查可达性及碰撞；腕部视觉纠偏范围仍独立配置。

使用 V3.3/V3.4 对应的现有视觉伺服规划器，以及提供最高原始分辨率的共享头部 RGB-D 服务。规划器须有新鲜反馈且无故障。

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
# 仅离线检查配置
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grap_v1.py --check-config
# GPT + RGB-D 生成方案，不发送运动命令
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grap_v1.py
# 执行完整单轮
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grap_v1.py --execute
```

输出为 `output/cart_grap_v1_<时间戳>/`：头部输入图、GPT 区域、深度与标定、计划和事件日志；腕部标注每 200 帧保存一次。`marker / observe / teach / align` 子命令仍委托给 `cart_grasp.py`。

正常保持通过定点视觉伺服完成，不使用会锁存失败状态的取消接口。发生丢码、反馈过期、偏差越界或夹爪动作超时时暂停；不自动松爪或撤回。闭合检查只确认夹爪位置变化并稳定，没有力反馈，不能据此证明已牢固夹住车把。

本次验证为离线回归与模拟反馈测试；完整硬件抓取仍需现场运行验证。

## 终点收敛与保持（2026-10-12 修复）

视觉对准且距终点不超过单步上限6 mm时，锁定计算出的X终点和当时Y/旋转目标，以10 Hz上限持续维护同一流式目标。到位仍要求X误差≤3 mm、图像水平误差≤3 px、角度误差≤1.5°以及连续新鲜稳定反馈；不在3 mm边界把当前位置反复设成新终点。视觉超差则恢复纠偏，丢码则保持并限时重识别，不盲目前伸。

正常阻塞保持确认完成后，将同一目标交给有租约的流式保持，避免已到位的有限指令在等待图像时触发`Direct probe/final command timed out`。没有加长或关闭控制端5秒超时；无更新时仍由流式租约转为关节保持。54项离线回归通过，包含慢速末段收敛、稳定等待超过5秒、末段丢码和异常清理；未执行实机验证。

## 小标记边缘混合深度的处理

内缩ROI的P90−P10超过30 mm时，尝试中心支持的连通主深度：在原四边形线性尺寸25%的核心区要求至少9个有效像素、有效率75%、P90−P10不超过15 mm；以核心中位数±15 mm筛选原50%ROI，保留包含几何中心的连通分量。该分量需占原ROI总像素至少75%，覆盖核心区至少90%，否则仍拒绝。不会以“最近深度”自动选面。正常一致深度路径不变，在线仍检查三张新帧的基座点稳定性。

`depth_selection`和`depth_fallback`记录所用路径、原始分位数、主区域比例及剔除像素数。失败保存`head_rejected_geometry.npz`等诊断。保存帧 `cart_grap_v1_1791813052690694044` 回放得到140/168像素、深度中位数678.1 mm；是单帧离线检查，不代表已通过在线三帧或实机验证。

## GPT-6 深度异常复核

`gpt_depth_review_enabled: true`（默认）启用一次自动复核。只有头部ROI有效深度不足、混合深度无法通过主深度筛选、三帧深度不稳定等深度区域异常触发；机器人状态异常、图像过期／重复、场景变化、碰撞和腕部丢码不由GPT放行。

每轮最多追加一次 `gpt-6-astra` 请求，输入新鲜头部原图、带原选区的放大局部图、实测深度8×8分区中位数（缺失为null）和具体失败原因。模型返回`decision=continue/stop`、是否同一目标、像素四边形和原因。继续仅表示允许重新验证；区域仍需通过置信度、有效几何和与原目标重叠／位移检查，不能跳到另一个标记。随后丢弃先前样本，重新采集三帧并执行原深度、时效、场景和静止反馈检查，通过后才生成运动方案。API等待期间不发送运动命令。

报告在本轮目录 `gpt_depth_review/`：`evidence.json`、输入图与局部图、`response.json`、`decision.json`、`validation.json`。`continue_pending_depth_validation`不是运动放行；`validation.json`中的`accepted`才表示复核后的本地深度阶段通过，仍需后续运动检查。模型停止、超时、换目标、复核后仍无有效深度均退出，不循环反复请求。

API响应使用 OpenAI 官方 [Responses Structured Outputs](https://developers.openai.com/api/docs/guides/structured-outputs?api-mode=responses)。50项离线测试通过，新增复核测试使用模拟API／RGB-D，尚未调用真实复核API或执行机械臂。
