# V3：特征点跟踪中线，点数不足后按距离完成

`V3.py` 默认 `centerline_distance`。粗前伸后锁定腕部中央附近的书，用特征点跟踪书脊中线；控制基座 Y 与工具局部 Z 轴旋转，使中线与画面中央竖线重合，忽略上下位置。X 终点沿用本轮 V1_3 深度结果。

## 运行

退出旧 V3 后重新启动即可加载本次修改。正常启动会先执行 `first_reset`。如果尚未加载之前的 2 mm 规划器修改，需先重启上层规划器。

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python V3.py --check-config
/mnt/ssd/robotarm/env/robotram_wx/bin/python V3.py --assume-head-camera-calibrated
```

启动日志显示 `特征点跟踪中线（少于 6 点转固定姿态直接前伸）`。配置仍为 `cart_book_grasp_v3_column.json`。

## 感知

1. 初始时用已有 SAM3 分割一次书本，按中线在画面中央行的横坐标选择最近的书；不按颜色选书，不做头腕身份匹配。
2. 从掩膜得到初始中线，在书本内部提取最多 80 个角点；不只依赖左右两侧的边界。初始不足 6 点仍停止，因为尚未建立可靠跟踪。
3. 后续用金字塔 LK 前后向光流跟踪，排除越界、无效、前后向误差超过 1.5 px 的点。
4. 用 RANSAC 相似变换估计当前中线位置和倾角。至少 6 个内点，内点比例至少 60%，点的纵向跨度至少 20 px；尺度、帧间跳变和中线方向还需通过检查。
5. 不逐帧重新分割，不自动重新选书，不依赖 CoTracker 模型。

实现为 `cart_book_grasp_v3_feature_centerline.py`；控制器 `cart_book_grasp_v3_centerline.py` 的 `CenterlineLock` 已恢复指向该跟踪器。直接边界实现保留在 `cart_book_grasp_v3_edges.py`，默认不使用。

## 点数不足后的直接前伸

只有已初始化的跟踪器抛出明确的 `FeaturePointsExhausted`（可靠点少于 6 个）才触发切换。恰好 6 个点仍可继续视觉控制，前提是几何等检查通过。

切换时读取新鲜实测位姿，固定当前 Y、Z 和完整腕部姿态；后续不再取图或计算视觉误差，只沿 X 分段前伸到原来冻结的 V1_3 终点。不会再恢复视觉伺服，不会重选书。到达后继续原夹取流程。

实现为 `cart_book_grasp_v3_direct_distance.py`。保留原碰撞规划、操作员中止、新鲜连续到位反馈、5 mm 最大平移步长和 2 mm 到位容差。直接阶段还检查实测 Y/Z 漂移与姿态漂移。视觉阶段与直接阶段共用累计路径和时间预算，不因切换清零。

日志 `point_loss_direct_switch` 记录剩余点数，`direct_distance_start` 记录冻结位姿，`direct_distance_complete` 表示距离完成。结果明确标记 `visual_alignment_verified=false`，不会把直接前伸报告为视觉对准成功。切换后无法再根据图像修正误差，依赖之前的对准及 V1_3 深度。

初始无纹理、相机错误、陈旧图像、几何拟合不一致、机器人反馈或运动失败仍停止，不触发直接前伸。

## 控制与到位策略

视觉目标为中线横向误差 ≤3 px、倾角误差 ≤1.5°，不要求书的上下中点居中。需要纠偏时，用 Y +3 mm 和工具轴 +2° 探测识别 2×2 响应矩阵，不返回起点、不探测 Z。已对准时可直接沿 X 前伸。

- 平移总步长最多 5 mm；视觉阶段旋转每步最多 3°、相对初始最多 ±30°，TCP Z 固定。
- 位置到位和 X 终点容差均为 2 mm，姿态到位容差为 1°。
- 模型预测有效时，小 Y 修正可扩展到约 2.5 mm；预计明显越过图像目标时不强行放大。
- 执行端检查实际运动进展，至少 3 个不同的新反馈，稳定窗口至少 0.2 秒，不能原地判成功。
- 视觉阶段取图后位置超限最多追加两次新观测，持续超限停止。
- 累计路径 400 mm，第二段前伸上限 250 mm，超时 600 秒；没有原先的 Y/Z 20 mm 半径限制。

仍通过原 MotionGen 逐路点规划执行，尚未改成连续速度控制。工具轴旋转保持抓取轴方向和指尖 X 偏移不变。

## 输出与验证

首帧绘制一次，之后每 200 次观测绘制、显示、保存一次；其余帧只做跟踪和控制。初始 RGB、掩膜和失败图仍保存。输出目录为 `RobotRAM/output/v3_column_<id>/`。`tracks` 事件记录内点数及耗时，`selection.json` 记录初始点和切换规则。

```bash
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_v3_direct_distance test_v3_edges test_v3_centerline test_v3_column test_cart_book_grasp_v3_distance test_cart_book_grasp_v3_wrist_center test_v3_servo_arrival test_v3_planner_microsteps test_v3_camera_stream
```

离线测试覆盖特征点跟踪、点数不足切换、停止取图、固定 Y/Z/姿态、5 mm 步长、共享预算和其他错误停止。没有执行实机抓取验证。头腕目标关系继续使用用户接受的粗对齐假设。旧模式入口保留。
