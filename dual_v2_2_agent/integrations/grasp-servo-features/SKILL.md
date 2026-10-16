---
name: grasp-servo-features
description: 从观测图像和抓取文本描述建立抓取目标视图，使用可执行算法提取、标注和跨帧跟踪固定身份的视觉特征，输出像素坐标与有效性供 Image Jacobian 使用。适用于通用物体的腕部相机抓取特征选择、连续跟踪，以及本机 servor.py 的 Y/工具轴旋转视觉对准。不是仅限书本。
---

# 抓取视觉伺服特征

## 本机 servor 集成

当用户要求输入物体与描述并跟踪、输出控制时，先读 [references/local-planner.md](references/local-planner.md)。使用独立 `servor.py`；不适配或修改 V3.3 抓取流程。当前授权的对准目标只有 Y 和工具轴旋转，X/Z 保持启动值。物体不限书本，表面适用性由实际图像判断。该路径不需要生成目标图片；目标是语义中心水平居中和指定物体长轴竖直。

下文为可移植的纯特征前端流程；其 `control_ready:false` 不改变，本机控制器另行验证位姿、时间戳、图像响应和控制界限。

用户输入观测图像和目标物体/抓取描述；本技能完成目标理解、目标视图准备、特征算法与标注。用户接入 Image Jacobian。不要把图像生成模型绘制的圆点当作测量，也不要声称单张图能验证跨帧稳定性。

交付 `task.json`、模板目录、观测/目标标注 PNG、逐帧 JSON/JSONL、适用条件与实测验证结果。实际提取和标注运行 `scripts/servo_features.py`，保留原图尺寸；可以另输出局部放大预览。提取、跟踪、标注无需图像生成服务。

## 执行流程

1. 检视原图，理解物体、接近方向、夹持部位、相机视向和闭合前终点。可推断的假设写入 task.json；实质影响任务且无法推断的歧义才询问。首帧语义定位由 Codex 视觉理解完成，脚本本身不是文本检测器。
2. 选择始终可见、近似刚性平面的有纹理表面（如标签）。用原始、EXIF 方向归一后的像素坐标给出 `patch_polygon`。它只限定搜索区域，**不是逐帧人工选择特征点**。脚本自动选取有空间分布的纹理点；保存模板后 P1…Pn 的身份和顺序固定。
3. 无纹理、强反射、非刚性、仅可见对称圆盖，或观测/目标看不到同一表面，不适用内置后端。报告 unsupported/insufficient_features，提出贴标记或换视角等可行方案，不把轮廓点或推测点冒充固定物理点。轻微曲面只在小范围内近似平面，并需真实序列验证。
4. 若用户明确需要目标帧比较，优先用真实闭合前目标帧。仅在用户需要示意目标图时，可调用图像生成能力制作**无标注**的想象目标视图，说明相机/夹爪假设并标记 synthetic；10 cm 安装偏移不能独自决定像素尺度。已有目标图无需重生。服务失败则保留观测输出并报告，不能伪称目标已生成。
5. 对目标运行相同模板匹配。通过才标对应点；失败保留无点目标并解释失配，不强制补点、不放宽阈值凑结果。即使通过，生成图也不是已标定的伺服目标。数值目标应来自真实示教帧，或经验证的相机/物体/夹爪几何渲染。
6. 连续帧复用同一 Tracker：SIFT 双向互检和比率筛选、RANSAC 单应约束、LK 光流前后向验证、周期性模板重定位，以及逐点局部纹理验证。拒绝位于内点凸包之外的远距离投影，低重投影误差不能证明外推准确。失效点输出 xy:null，不保留上一帧假坐标；恢复时沿用原 ID。
7. 显示两帧标注，说明有效点、失败原因和目标来源；提供坐标接口，不代替用户控制机器人。

## 入口

依赖 Python 3、OpenCV（SIFT）、NumPy、Pillow。先检查现有环境；缺依赖则在任务局部 venv 安装 `scripts/requirements.txt`，不改系统解释器。

读 [references/interface.md](references/interface.md) 获取配置、API、JSON 与标定约定。`<skill>` 代表本文件目录，各路径替换为实际绝对路径：

```bash
python3 <skill>/scripts/servo_features.py init --image observation.jpg --config task.json --out template
python3 <skill>/scripts/servo_features.py pair --model template --current observation.jpg --target goal.png --target-kind synthetic --out results
python3 <skill>/scripts/servo_features.py track --model template --video sequence.mp4 --out tracked
```

`track --frames DIR` 可按文件名字典序处理图片目录。实时集成调用 `Tracker.process()`，不要逐帧启动 Codex 或图像生成器。默认不平滑坐标，避免隐含延迟，输出处理时间和时间戳。

## 验证

运行 `scripts/test_servo_features.py` 检查已知变换误差、遮挡失效、错误物体拒绝和身份恢复。对用户样本运行 `scripts/benchmark.py`。合成扰动不能证明真实视频鲁棒性或硬实时能力；报告受测范围和未验证条件，不承诺“任意物体/每帧都稳定”。

这是特征前端，`control_ready` 始终 false：标定、深度、可观测性、夹爪几何和控制器由用户验证。提供固定顺序 `s_px` 与逐标量 `valid_mask`，误差只对共同有效点计算。控制器删维度时需同步删 Jacobian 行；6DOF 需足够独立约束和满秩检查。
