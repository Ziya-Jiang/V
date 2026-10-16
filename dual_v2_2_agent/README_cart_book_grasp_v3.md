# V3：快速中心线对准 → V1_3 距离终点 → 抓取

**最新默认入口跟踪书脊中心线，同时用 Y 移动与工具轴旋转对准画面中央竖线，TCP Z 保持固定；已恢复特征点光流跟踪；跟踪中可靠点少于 6 个时，退出视觉纠偏，固定切换时实测 Y/Z 和腕部姿态，继续到本轮 V1_3 计算的 `grasp_tcp_x_mm`。按距离完成不表示视觉对准已验证。** 不要求整本书完整入镜，不使用头腕匹配、原书最终纹理图或 CoTracker 权重。完整命令、参数及验证见 [中心线运行说明](README_cart_book_grasp_v3_column.md)，配置为 `cart_book_grasp_v3_column.json`。旧 CoTracker Y/Z 模式使用 `--legacy-cotracker`。

以下内容保留作为 `V3.py --legacy-final-image` 的旧模式参考；其中最终图配置不再用于默认入口。旧模式的检查命令须加 `--legacy-final-image`。

## 旧最终图模式

运行入口是 `V3.py --legacy-final-image`，基于 V1_3。此旧模式使用保存的**最终抓取示教图**作为闭环终点，在前伸过程中同时调整基座 X/Y/Z。所有路点保持进入第二段时的工具方向，仍通过原规划器执行。

### 当前默认：腕部中心选书，不做头腕匹配

`cart_book_grasp_v3_approach.json` 中 `target_selection` 已设为 `wrist_center`。头部仍用于 V1_3 的粗定位与行程约束；到第二段观测位置后，在腕部 RGB 中用现有 SAM3（复用头部模型）检测书本，以 mask 质心到图像中心的二维像素距离排序，选择最近的可靠候选（score ≥0.5，面积 ≥图像的 0.3%）。不按颜色或头部书本身份选取，不再调用 SuperPoint 头腕匹配。低置信度书边碎片不参与排序。

每轮只选一次。随后用腕部局部纹理跟踪所选书的 mask；每帧确认最终示教控制点后更新局部跟踪参考，避免固定起点模板限制前伸尺度。跟踪丢失时停止，不能改抓新的中心书。允许已在腕部画面顶部裁切的书本进入局部跟踪；控制点与实际跟踪区域仍必须有效。最终图匹配仅在锁定书的 mask 中进行，禁止匹配到邻书。

**此修改只替换选书/身份交接，最终示教图仍绑定原书纹理。不同书不能直接复用原书的纹理目标；通用跨书终点需要另行实现手—书相对几何目标。** 目前如果中心书无法匹配最终示教图，会停在最终图跟踪阶段，不能启动 XYZ 探测。

启动时应看到 `V3 选书模式：腕部图像中心最近的书；不执行头腕匹配`。旧进程须在暂停提示输入 `s` 退出，再启动 `V3.py --assume-head-camera-calibrated`；正常启动仍先复位。

不动机器人的选书预览（读取已有去畸变腕部 PNG）：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python V3.py --preview-wrist-center RobotRAM/output/v3_xyz_75b693b3f8fa4002a4394c1be6f760ea/wrist_acquisition.png
```

输出 `RobotRAM/output/v3_wrist_center_<id>/`，包含 `report.json`、候选原始分割、锁定图和 mask。报告分别列出选书/锁定成功与最终图匹配成功；后者失败时退出码为 2。运行时保存 `wrist_center_selection.json` 和 `wrist_center_selected.png`。`--check-config` 在本模式不加载 SuperPoint。

### 可选旧模式：SuperPoint + LightGlue 头腕匹配

以下仅在 `target_selection` 改回 `head_identity` 或显式使用 `--match-head-wrist` 离线工具时适用，默认流程不执行。

头部选中书脊 → 腕部的跨视角匹配改为本地 CPU SuperPoint + LightGlue，同时尝试全图和带背景的 ROI 放大。独立匹配及内点门槛为 6，内点率至少 0.65，RMS 不超过 1.5 px。没有旧 SIFT 的头部 `features >=16` 初始化要求。模型在进入抓取循环前加载，缺依赖或本地权重即报错，不自动下载，不修改 CUDA 可见性。

在线流程还要用头腕单应性传递选书点和掩码，因此保留控制点位于内点凸包内、点分布、投影有效性与尺度检查；全图和 ROI 若都通过却对定位相差超过 5 px，则拒绝。离线报告中的“6 点单帧身份成功”不能直接等同于可将控制点映射后运动。腕部局部跟踪及最终示教图跟踪仍使用原 SIFT，未放宽其阈值。当前共享头部输入仍是 848×530；此修改没有启用 1280×800 在线采集。

已有进程不会热加载 Python 代码。在暂停提示输入 `s` 退出旧循环后重新启动。启动会明确打印 `V3 头腕匹配后端：SuperPoint+LightGlue`；正常运行仍从 `first_reset` 开始，不是从手动观测位置续跑。

可以先不连接相机、不连接机器人，重放本次失败图像：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python V3.py --check-config
/mnt/ssd/robotarm/env/robotram_wx/bin/python V3.py --match-head-wrist RobotRAM/output/v3_xyz_75b693b3f8fa4002a4394c1be6f760ea
```

重放在输入目录下创建 `superpoint_replay_<id>/report.json`，分别报告头腕身份入口、最终图跟踪、是否确认同一示教书；失败退出码为 2。它不会把离线通过当作运动授权。运行时每次采集也保存 `head_wrist_matching.json`，包含全图/ROI 的特征数、匹配数及失败原因。

该现场实际重放结果：全图源区域 10 个特征、0 对匹配；ROI 源区域 49 个特征、3 对匹配，仍不足 6 对。最终示教图的现有 SIFT 跟踪只有 2 对独立匹配，也未通过。此次切换解决的是旧头部 SIFT 初始化阻塞，没有证明该现场已可闭环抓取。46 项离线测试通过；真实图像施加已知变换的 SuperPoint 正对照为 329 个内点，选中点映射误差约 0.0202 px（仅合成变换验证，不是实际抓取精度）。

## 本次示教与观测

用户已确认：近景图拍摄于最终抓取示教位置，之后机械臂手动退到了第二段前伸前的观测位置。

- 最终目标图：`RobotRAM/output/v3_reference_current_20261009.png`。
- 退回后的观测图：`RobotRAM/output/v3_observation_before_stage2_20261009.png`。
- 默认目标配置：`cart_book_grasp_v3_approach.json`，绑定最终图文件哈希、相机与标定指纹、书脊掩码及四个控制点。

四个控制点是最终图中书脊平面上的固定位置。每次通过纹理匹配和单应性，寻找当前图中对应的同四个位置；不会用当前包围框中心替代它们。它们位于匹配内点支持区域内，位置与尺度变化共同提供 XYZ 误差信息。当前参考只适用于这本书及其可见纹理；不同书、不同封面不能直接复用这个最终图，需要重新登记参考图、掩码和控制点。当前没有通用的最终图点击登记界面。

## 不动机器人的检查

使用原来能够运行 V1_3 的 Python 环境：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
python V3.py --check-config
python V3.py --match-reference RobotRAM/output/v3_observation_before_stage2_20261009.png
```

`--check-config`（也可用 `--check-setup`）验证配置、最终图及标定，不打开相机、不连接机器人。离线匹配读取同名 JSON 中的去畸变元数据，输出叠加图和 `report.json` 到 `RobotRAM/output/v3_match_<id>/`。

两张实拍图的离线结果：12/16 个匹配内点，重投影 RMS 约 0.53 px，当前控制区域面积约为最终图的 0.0265 倍。**0.53 px 是匹配拟合残差，不是到达目标的误差或抓取精度。** 此时四点距各自目标约 263–402 px，尚未收敛。

## 完整运行流程

```bash
python V3.py --assume-head-camera-calibrated
```

**这个命令启动完整新循环，先执行 `first_reset`。它不是从当前人工停放的观测位置直接续跑的命令。** `--assume-head-camera-calibrated` 沿用 V1_3 的含义，不表示腕部手眼变换已标定。

1. 沿用 V1_3 的复位、头部感知、选书、J7 调整、Y 粗对齐和必要的 Z 深度补偿。
2. 由 V1_3 深度参数计算原有前伸路点。若有多个路点，只执行其中第一个粗前伸路点；若仅有一个，则全部前伸交给图像闭环。第一段长度取原算法的分段结果，不保证固定为 125 mm。
3. 在观测位置拍摄腕部图像。默认 `wrist_center` 选择离腕部图像中心最近的可靠书本并锁定；不做头腕匹配，最终示教点必须在锁定书本内。可选 `head_identity` 才验证最终图对应的书是本轮头部选中的书。选书、跟踪或最终图匹配失败均停止本轮。
4. 若四点已连续三帧各自达到 3 px 以内，直接通过。否则在基座 X/Y/Z 的正负方向各探测 2 mm，每次返回观测起点，用实测 TCP 位移和图像响应拟合 8×3 局部雅可比。
5. 根据四点误差计算阻尼最小二乘纠偏，每步最多 2 mm。确认路点到达、停稳并取得新图像后，再计算下一步；通过实测响应更新雅可比，适应接近过程中的尺度变化。
6. 连续三帧四点误差均 ≤3 px，才允许执行后续 V1_3 合爪、抬升、J7 反转、下降、松爪、抽回及复位。不会在视觉收敛后补执行原来的剩余前伸路点。

失败沿用 V1_3 的暂停机制，不进入后续合爪。回车开始新一轮并先复位；`p` 沿用 V1_3 的中断、打开右夹爪并复位；`s` 停止后续轮次，不能当作紧急停止。

## Image Jacobian 与执行范围

参考 [Robot Academy — The Image Jacobian](https://robotacademy.net.au/lesson/the-image-jacobian/)。相机运动与图像特征运动通过图像雅可比关联。由于没有完整手眼变换，执行时从实测基座 XYZ 探测估计等效雅可比，不直接将相机速度当作基座速度。

```text
s = [u1, v1, u2, v2, u3, v3, u4, v4]
delta_s ≈ J_base_xyz * delta_xyz_mm
delta_xyz = -gain * inverse(J.T*J + damping²*I) * J.T * (s - s_goal)
```

这是“停稳—拍照—纠偏”的分步闭环，不是高频速度控制或接触力控制。仅调整 XYZ，未控制旋转；若当前工具方向无法重现最终图，可能无法收敛，会在误差、运动范围或时间检查中停止。目标照片没有同步的最终 TCP 姿态记录，因此本版未验证最终姿态角的一致性；运行中检查的是相对第二段起点的方向漂移。

`cart_book_grasp_v3_approach.json` 默认增益 0.4、单步 2 mm、连续三帧 3 px、最多 120 次纠偏、600 秒。相对观测起点允许前进最多 150 mm、后退最多 5 mm、YZ 偏移半径最多 20 mm，累计路点路径最多 250 mm（含探测）。前进上限还受原 V1_3 深度终点限制；深度只作行程约束，视觉误差决定是否到达。如果示教终点超出约束，会停止而不是强行走完。

图像丢失、过期或重复、纹理不足、匹配歧义、雅可比退化、预测响应不一致、误差连续增大、规划失败、位姿偏差或预算超限均中止本轮。端点限制不代表规划器的中间路径严格直线。

第一段粗前伸单独使用 `coarse_pose_error_mm: 8.0`。控制器返回成功后先等待 `coarse_settle_seconds: 0.3` 秒，再最多等待 `coarse_feedback_timeout_seconds: 2.0` 秒获取稳定反馈：三次间隔 0.1 秒的位姿均须在到位阈值内，且相对该稳定窗口首帧变化不超过 0.5 mm / 0.2°。相对指令的方向误差仍限制为 1°。通过时以实际 TCP 为第二段起点；超时仍暂停，不自动补发前伸。通过或失败均输出目标、实际位置、各轴差值和总误差。视觉小步仍使用独立的 `max_pose_error_mm: 1.0`。

## 标定、配置与记录

相机从同目录 `calibrationdata.tar.gz` 的 `ost.yaml` 读取本次 1280×720 标定。统一采用 `initUndistortRectifyMap(K, D, I, K, size)`，输出保持 K 和原尺寸，不裁剪；所有目标及观测像素都在 `undistorted_K` 坐标系，不能混用原图或 P 矩阵对应的像素。相机、分辨率或标定指纹不一致时拒绝参考。

- `cart_book_grasp_v3_config.json`：V1_3 动作参数副本。
- `cart_book_grasp_v3_wrist_config.json`：V1_3 J7 配置副本。
- `cart_book_grasp_v3_camera.json`：腕部相机配置。
- `cart_book_grasp_v3_approach.json`：默认 XYZ 闭环与最终图参考配置。
- `RobotRAM/output/v3_xyz_<id>/`：逐帧叠加图及 `events.jsonl`，记录匹配、位姿、雅可比、命令和退出原因。

进入身份校验前，还会保存 `head_selected_rgb.png`、`head_selected_mask.png`、`wrist_acquisition.png` 与 `acquisition.json`，即使匹配失败也保留输入。纹理不足错误会注明发生在头部到腕部身份校验还是腕部跟踪初始化，并显示特征数、书脊像素大小及掩码收缩前后的面积。身份校验保持必需；头部区域过小、缺少共同纹理时不能仅依靠腕部看到示教书就继续。

原 Rerun 的 **Spine + center** 面板显示腕部闭环图：黄色为当前四点，绿色为最终四点。

## 验证与旧模式

```bash
python -m unittest test_cart_book_grasp_v3_xyz test_cart_book_grasp_v3 test_cart_book_grasp_v1_3 -v
```

离线测试涵盖三轴耦合收敛、透视尺度变化、匹配失败和旧帧阻断、行程上限、真实图像匹配、粗前伸反馈延迟及超限阻断、仅执行第一段后交给视觉、失败禁止后续抓取、抽回起点保留及原 V1_3 中断行为。完整实机 XYZ 闭环抓取尚未验证成功。

旧 Y/Z 对准点模式保留为 `python V3.py --legacy-yz ...`，直接运行 `cart_book_grasp_v3.py` 也进入旧模式。它使用独立的 `cart_book_grasp_v3_servo.json`；旧 `--teach` / `--set-reference` 仅更新这个旧模式，不会更新默认 XYZ 的最终图参考。

`python V3.py --capture-wrist OUTPUT.png` 仍可只打开腕部相机采图，保存去畸变图、原图与同名 JSON，不连接机器人。拍摄本身不会替换已登记的最终目标。

修改前版本保存在 `backups/v3_before_xyz_20261009_161609/`；更早的 V2 基础版本在 `backups/v3_before_v13_servo_20261009_143503/`。V1_3 源文件未修改。
