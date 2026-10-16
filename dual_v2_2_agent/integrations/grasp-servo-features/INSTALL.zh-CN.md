# 跨机器安装与输出说明

本包为可复制的本地 Codex Skill，包含 SKILL.md、界面元数据、Python 算法、接口文档、依赖清单和测试。包内不含 Python 解释器、第三方依赖、图像生成服务、用户照片或已训练的目标模板。无需依赖原机器的路径。

## 安装到另一台机器

将 ZIP 传到目标机器，解压后把整个 `grasp-servo-features` 目录放到：

- macOS/Linux：`~/.agents/skills/grasp-servo-features/`
- Windows：用户主目录下 `.agents\skills\grasp-servo-features\`

最终必须是 `.../grasp-servo-features/SKILL.md`，避免多套一层同名目录。已安装同名技能时先确认要更新哪一份，不同时安装两份。

这是官方文档列出的用户级本地技能目录；如果技能未出现在选择器中，重新启动 Codex。
来源：[OpenAI 官方技能文档](https://learn.chatgpt.com/docs/build-skills)。

也可以把本 ZIP 提供给目标机器上的 Codex，直接说：

> 请将这个压缩包中的 grasp-servo-features 技能安装到我的用户级技能目录，建立独立 Python 虚拟环境、安装 requirements.txt，并运行包内测试。不要覆盖已有同名技能而不先检查。

## Python 环境

建议 Python 3.10 或更新版本，需能安装 NumPy、OpenCV 和 Pillow。库版本见 `scripts/requirements.txt`。下面在解压后的技能目录运行，不需要激活虚拟环境：

macOS/Linux：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r scripts/requirements.txt
.venv/bin/python scripts/test_servo_features.py
```

Windows PowerShell：

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r scripts\requirements.txt
.\.venv\Scripts\python.exe scripts\test_servo_features.py
```

首次安装依赖通常需要网络。算法不需要 GPU、API key 或下载深度学习权重。图像生成仅在需要想象目标帧时使用目标机器上可用的图像生成工具；其账号/可用性不包含在此 ZIP 内。提供真实目标帧时，特征计算不依赖图像生成工具。

运行测试应看到 7 项通过。本包在 macOS 的现有 Python/OpenCV 环境完成测试；未实际在 Windows/Linux 上运行，目标机器应自行运行测试确认依赖兼容性。实时性能需要在目标机器测量。

## 使用

在 Codex 中输入 `$grasp-servo-features`，同时提供观测图像与抓取描述。例如：

> 使用 $grasp-servo-features。目标是图中的 FARINA 香水瓶，保持直立，夹爪水平接近并夹住瓶身两侧。请建立固定身份的特征模板，输出观测帧/目标帧标注与坐标 JSON。

如果技能目录包含 `.venv`，请使用该环境的 Python 执行脚本。首帧由 Codex 理解目标和选定有纹理表面，之后每帧由 Python 算法处理。输入单张照片只能完成初始化和图像匹配；验证真实跨帧稳定性需要视频或序列。连续运行通过 Python Tracker API 接入相机帧，无需每帧调用 Codex。

## 输出文件

1. `task.json`：目标物体、抓取描述、相机假设和首次选定的纹理区域。
2. `template/`：`reference.png`、`features.npz`、`model.json`。固定模板和点身份。若要在另一台机器继续追踪同一目标，请另行复制整个模板目录；包本身不包含你的照片。
3. 配对模式 `results/current.png`、`results/target.png`：带固定编号的观测/目标标注。匹配失败时显示失效状态，不伪造对应点。
4. `results/current.json`、`results/target.json`：各帧的特征坐标、有效性和诊断指标。
5. `results/comparison.json`：目标来源、共同有效掩码、误差有效掩码与当前减目标的像素误差向量。
6. 视频/序列模式 `tracked/features.jsonl`：每行是一帧 JSON；同时输出 `000000.png` 等逐帧标注图片。
7. 调用 benchmark 时输出 `benchmark.json` 与少量测试图。它是已知图像扰动验证，不是实际抓取测试。

## Image Jacobian 所需字段

- `features`：P1…Pn 的固定身份和 `[u,v]` 像素坐标。
- `s_px`：固定顺序 `[u1,v1,u2,v2,...]`。无效坐标为 JSON null。
- `valid_mask`：与 s_px 同长度的逐标量有效掩码。
- `e_px`：当前 `s_px` 减目标 `s_px`；只计算共同有效且图像尺寸一致的坐标。
- `error_valid_mask`：用于选择 e_px 及对应 Jacobian 的行；尺寸一致仍需另外验证相机几何一致。
- `template_id`：检查是否属于同一模板和相同点身份。
- `status`：ok、partial、lost；`reason` 为失效原因。
- `timestamp`、`frame_index`、`processing_ms`：时间与计算性能信息。

单位是原始测量图像的像素，原点左上，u 向右、v 向下。若你的 Image Jacobian 使用归一化坐标，需要先使用内参转换。不可把 null 当作零误差。删减无效坐标时需同步删减 Jacobian 行。

这个 Skill 输出特征与误差，不输出机械臂速度、关节命令或已标定抓取位姿。`control_ready` 始终为 false：由你的系统验证内参、手眼标定、深度、可观测性和控制条件。生成图标记 synthetic，不能仅凭匹配成功就认定为已标定目标。

当前后端适用于有纹理、局部近似平面、跨帧持续可见的刚性表面。无纹理玻璃、强反光或视角变化后看不到同一表面时，可能无法输出有效点。

完整字段和算法约定见 [references/interface.md](references/interface.md)。
