# 数据接口与集成

## 配置与输入

首次由 Codex 结合图像和抓取描述定位一块有纹理的刚性近似平面，写 task.json。例如 FARINA 正面标签（坐标只适用于本会话 1280×1707 原图，其他图片必须重新定位）：

```json
{
  "object": "FARINA 香水瓶",
  "grasp_description": "瓶子直立，夹爪水平接近，夹持瓶身两侧",
  "camera_assumptions": "腕部相机在夹持中心上方约10cm；准确外参未知",
  "patch_polygon": [[550,733],[705,770],[697,838],[549,794]],
  "point_count": 8
}
```

脚本不理解自然语言。首帧语义理解是技能中 Codex 的工作；循环内全部由代码计算。不要把配置中的示例像素用于别的图。图像读取使用 EXIF 归一方向；视频帧使用解码器输出方向。原图已经去畸变时，后续帧也必须使用相同几何处理；否则先统一去畸变再建模板。读入/预处理后不偷偷缩放图像。

`init` 从搜索区域内的 SIFT 点选最多 point_count 个相隔一定距离的纹理锚点。它们不必是四个标签角点，比人为指定圆角更容易复现。此版本适用纹理丰富、局部平面、光度变化适中、同表面可见的刚性物体。透明玻璃上的高光、被子、仅有轮廓的物体不适用此后端。不会自动下载分割/学习匹配模型。

## Python 接口

把技能 scripts 目录加到模块搜索路径，然后导入：

```python
from servo_features import Tracker, read_image, compare_results

tracker = Tracker('/absolute/path/template')
goal = Tracker('/absolute/path/template').process(read_image('/absolute/path/real_goal.png'))
# 在相机循环内，bgr_frame 为 OpenCV uint8 BGR 图像；timestamp 是采集时间。
current = tracker.process(bgr_frame, timestamp=capture_time_seconds)
error = compare_results(current, goal, target_kind='real')
```

同一条连续序列复用 tracker；独立目标图用独立实例，避免跨无关图像跑光流。process 不支持并发共享同一个实例。实时采集由调用者负责；CLI 只处理现有视频/目录，不连接相机或机器人。

## JSON

- `template_id`：参考图、区域、锚点的摘要；只有一致时才可比较点。
- `image_size`：[width,height]，原始测量图像的像素尺寸。
- `features`：固定顺序 P1…Pn；每点 `xy:[u,v]` 或 null、valid、patch_ncc、support_margin_px（参考坐标中到匹配内点凸包的有符号距离，正值在内部）。凸包外超过1px的点无效，避免远距离单应外推。
- `s_px`：`[u1,v1,u2,v2,...]`，永远固定长度。无效坐标为 null。
- `valid_mask`：与 s_px 同长度，供用户同步选择误差向量和 Jacobian 行。
- `status`：ok=全部锚点纹理验证通过；partial=至少4点、具有空间分布；lost=整体定位未通过，全部点无效。
- `method`：sift 或 lk。`reason` 提供失效原因。
- `homography_reference_to_frame`：近似平面参考→当前帧映射；不能把它当作已恢复的机器人位姿。
- `metrics`：匹配数、独立内点数、内点比例、参考区域覆盖率、重投影中位误差。patch_ncc 是相关系数，不是经过标定的概率。
- `measurement`：homography_projection_with_texture_check。坐标是几何模型投影的固定锚点，并验证对应局部纹理；不是声称每个点都独立亚像素测量，误差具有相关性。
- `frame_index`、`timestamp`、`processing_ms`：时间戳由调用者提供；目录模式为 null，不制造采集时间。
- `control_ready`：始终 false，本前端不认证控制器、标定或接触安全。

`pair` 写 current.json/png、target.json/png 和 comparison.json。comparison 中 e_px 是当前减目标，只对双方都有效且图像尺寸相同的维度计算，其余为 null。`shared_valid_mask` 表示共同可见，`error_valid_mask` 再加入尺寸一致条件；相同尺寸仍不证明相同相机几何。文件还包含 target_kind 和 target_calibrated:false。错误匹配和失效是正常运行状态，不用程序异常代替。无纹理初始化、文件或配置错误则直接抛出明确错误。

## Jacobian 使用约定

像素坐标原点左上，u 向右、v 向下。如果用户的图像 Jacobian 使用归一化坐标，对去畸变像素使用 `x=(u-cx)/fx, y=(v-cy)/fy`；目标图也用对应相机内参，不可混用像素 Jacobian 和归一化误差。不同相机、裁剪或缩放的目标图不能直接做像素差。

P1…Pn 固定对应参考纹理位置，禁止重排。失效时保持向量维数但填 null；用户自行决定停顿/重定位或删行降维，不把 null 置零。4个点并不自动保证任意6DOF控制的满秩、可观测性和全局收敛。深度和手眼外参由调用系统提供。

## 目标来源

- real：真实相机采集的目标，但仍需确认安装、内参和夹爪姿态。
- geometric_render：具有几何模型的渲染；本脚本不验证模型参数。
- synthetic：生成式想象图；即使外观匹配成功也只提供候选特征，不表示厘米、尺度或接触位置正确。

示教目标应为夹爪张开、即将闭合的可见状态。特征表面若在目标中不可见，换可持续观察的特征或视角；不要为了生成好看图制造对应关系。目标图已有人工/生成点标注会污染描述子，尽可能索取或生成干净版本。

## 算法参数与性能

默认每5帧做全图 SIFT 重新定位，间隔帧使用双向 LK；LK 验证失败立即尝试 SIFT。SIFT 双向比率门限0.75；单应 RANSAC阈值3px，至少10个不同像素位置的内点，比例≥0.55，参考覆盖≥0.10。锚点用11×11归一化纹理相关验证，阈值0.65；至少4个具有空间分布的锚点通过。整个补丁出画面则拒绝。旋转、缩放后重采样/模糊会导致保守拒绝，不能随意为提高通过率放宽。

较大图片的全图 SIFT 可能低于相机帧率；processing_ms 必须实测。默认无平滑、无隐式延迟；未实现硬实时和硬件加速。高性能版本可加入有界ROI搜索或经过验证的学习匹配后端，但需重新回归测试。未对多实例相同标签消歧作身份保证。

资料：[OpenCV 特征与单应匹配](https://docs.opencv.org/4.x/d1/de0/tutorial_py_feature_homography.html)、[LK 光流](https://docs.opencv.org/4.10.0/dc/d6b/group__video__track.html)、[视觉伺服基本原理](https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf)。
