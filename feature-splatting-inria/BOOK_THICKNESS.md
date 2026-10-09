# RGB-D 书本厚度与夹爪开口计算

完整算法原理、公式、阈值和侧面剔除机制见
[`BOOK_THICKNESS_ALGORITHM.md`](BOOK_THICKNESS_ALGORITHM.md)。

`book_thickness_pipeline.py` 从通用 RGB-D 文件或 Record3D 扫描中自动完成：

1. 使用 SAM3 在高分辨率 RGB 图像中分割每一本可见书脊。
2. 拟合书脊左右长边，避免把倾斜书本的水平包围框宽度误当厚度。
3. 读取深度和相机内参恢复毫米尺度；多帧 Record3D 模式还会使用深度置信度。
4. 利用 ARKit 位姿将多帧检测投影关联到同一本书。
5. 对多视角厚度做鲁棒融合，输出 95% 工程不确定度。
6. 计算建议夹爪预张开宽度：

   `厚度中位数 + 95% 不确定度 + 安全间隙`

## 运行

当前机器已经具备 SAM3 环境和通用/Record3D 深度读取依赖：

```bash
CUDA_VISIBLE_DEVICES=0 \
/mnt/ssd/robotarm/env/sam3_full/bin/python \
book_thickness_pipeline.py \
2026-09-01--23-22-25
```

默认分析帧为 `0,12,24,36,48,60,71`，参考帧为 24，夹爪额外安全
间隙为 3 mm。可按夹爪机械误差调整：

```bash
CUDA_VISIBLE_DEVICES=0 \
/mnt/ssd/robotarm/env/sam3_full/bin/python \
book_thickness_pipeline.py CAPTURE \
  --frames 0,8,16,24,32,40,48,56,64,71 \
  --reference-frame 24 \
  --gripper-clearance-mm 4
```

## 单帧 RGB-D 模式

### 适用场景

通用单帧模式用于只有一张 RGB 图、一张已配准深度图和一个相机内参文件的情况。
输入不必来自 Record3D。只要同时提供 `--rgb`、`--depth` 和 `--intrinsics`，
程序就会自动进入单帧模式，不需要再提供 `CAPTURE` 或
`--single-frame-rgbd`。该模式：

- 直接读取命令行指定的 RGB、depth 和内参文件。
- 不要求 Record3D 目录结构或 `metadata`。
- 不要求 `.conf`、相机位姿或帧编号。
- 不执行跨帧投影、书本关联或多视角鲁棒融合。
- 使用当前相机坐标系计算书本厚度，不生成依赖世界坐标系的测量。

原有的 Record3D 单帧入口仍然保留：提供 `CAPTURE`、`--single-frame-rgbd`
和 `--reference-frame N` 时，程序从 Record3D 目录读取第 N 帧，但忽略 `.conf` 和位姿。

单帧模式适合数据检查、快速测试和产生夹爪预张开初值。因为没有其他视角用于发现
错误分割、深度异常和边界偏差，它不能提供与多帧模式相同的可靠性。

### 必需输入

通用模式只需要三个路径，文件可以位于任意目录：

```text
/path/to/rgb.png
/path/to/depth.png
/path/to/intrinsics.json
```

支持的 RGB 格式由 Pillow 决定，常用格式包括 JPEG、PNG、TIFF。

支持的深度格式：

- PNG/TIFF 等单通道深度图。
- `.npy`：一个二维 NumPy 数组。
- `.npz`：键名为 `depth` 的二维数组；如果文件只有一个数组，也会自动读取。

深度单位规则：

- 浮点深度默认单位为米，即 `--depth-scale 1.0`。
- 16 位及以上整数深度默认单位为毫米，即 `--depth-scale 0.001`。
- 其他传感器单位必须显式提供 `--depth-scale`，该参数表示“每个深度数值对应多少米”。
- 深度值 0、非有限值以及 `0.1–5.0 m` 范围外的值会被视为无效。

例如，某相机深度单位为 0.1 mm：

```bash
--depth-scale 0.0001
```

支持的内参格式包括 JSON、TXT、NPY、NPZ、OpenCV YAML/XML。推荐 JSON：

```json
{
  "fx": 1348.5952,
  "fy": 1348.5952,
  "cx": 720.9918,
  "cy": 962.8973
}
```

也支持标准的 3×3、按行排列的相机矩阵：

```json
{
  "K": [
    [1348.5952, 0.0, 720.9918],
    [0.0, 1348.5952, 962.8973],
    [0.0, 0.0, 1.0]
  ]
}
```

TXT 可以包含 `fx fy cx cy` 四个数，或按行排列的 3×3 矩阵。NPY/NPZ 同样接受
四个系数或 3×3 矩阵。

内参必须对应 RGB 图像坐标系，而不是未经配准的原始深度相机坐标系。仅凭图像尺寸
不能把像素宽度可靠地换算为毫米。

RGB 和 depth 必须来自同一次采集并完成配准。不能把其他图像、裁剪后的 RGB 或不同
相机的深度直接组合使用，否则边界像素与深度射线不对应。两者可以具有不同分辨率，
但宽高比必须接近；程序会拒绝宽高比相差超过 2% 的输入。

### 运行命令

直接输入三个文件路径：

```bash
CUDA_VISIBLE_DEVICES=0 \
/mnt/ssd/robotarm/env/sam3_full/bin/python \
book_thickness_pipeline.py \
  --rgb /path/to/rgb.png \
  --depth /path/to/depth.png \
  --intrinsics /path/to/intrinsics.json \
  --output /path/to/book_thickness_result
```

参数说明：

- `--rgb`：RGB 图像路径。
- `--depth`：已经和 RGB 配准的单通道深度图或 NumPy 数组路径。
- `--intrinsics`：与 RGB 坐标系对应的相机内参路径。
- `--depth-scale`：可选的米/数值换算系数；整数毫米图和浮点米深度通常无需指定。
- `--output`：将单帧结果写入独立目录，防止覆盖默认的多帧结果。

如果不提供 `--output`，默认写入 RGB 图像所在目录下的
`RGB文件名_book_thickness/`。

兼容原来的 Record3D 单帧命令：

```bash
export CAPTURE=2026-09-01--23-22-25

CUDA_VISIBLE_DEVICES=0 \
/mnt/ssd/robotarm/env/sam3_full/bin/python \
book_thickness_pipeline.py "$CAPTURE" \
  --single-frame-rgbd \
  --reference-frame 24 \
  --output "$CAPTURE/book_thickness_single_frame"
```

调整书本检测阈值和夹爪安全间隙：

```bash
CUDA_VISIBLE_DEVICES=0 \
/mnt/ssd/robotarm/env/sam3_full/bin/python \
book_thickness_pipeline.py \
  --rgb /path/to/rgb.png \
  --depth /path/to/depth.png \
  --intrinsics /path/to/intrinsics.json \
  --min-score 0.55 \
  --gripper-clearance-mm 4 \
  --output /path/to/book_thickness_result
```

### 单帧厚度计算过程

1. SAM3 在高分辨率 RGB 中分割每本可见书脊。
2. 程序保留每个实例的最大连通区域，填充掩码并进行重叠实例去重。
3. 根据书脊轮廓拟合长轴和横向轴，确定左右边界；倾斜书本不会直接使用水平包围框宽度。
4. 将 RGB 掩码按覆盖率映射到低分辨率深度图，只保留有限且位于 `0.1–5.0 m`
   范围内的深度。
5. 无 `.conf` 时，所有通过数值检查的深度点在内部统一作为可用点，然后使用中位数
   和 MAD 去除明显的深度离群值。
6. 使用相机内参把深度像素反投影为相机坐标系中的 3D 点。
7. 每本书使用 RANSAC 最多分解三个深度平面，再用 SVD 对各平面内点重新拟合。
8. 汇总同一帧所有书的候选平面，估计共同的书脊朝向；避免把面积较大的书本侧面
   误选为书脊。
9. 为每本书选择与共同朝向最一致、纵向覆盖充分的主书脊平面。
10. 如果掩码边缘还存在与书脊夹角超过 35° 的第二平面，则将其识别为可见侧面，
    使用书脊平面支持区把左右边界裁剪到两个平面的交界处。
11. 将修正后的左右边界射线与书脊平面求交，两个 3D 交点之间的距离即为书本厚度。
12. 只接受 `3–100 mm` 范围内的厚度，最后计算建议夹爪开口。

为防止侧面导致厚度偏高，以下情况会拒绝测量，而不是输出可能错误的夹爪开口：

- 书脊平面内点比例低于 35%。
- 书脊平面纵向覆盖低于 55%。
- 单书平面法向与全局书脊法向相差超过 32°。
- 书脊观察夹角超过 60°。
- 侧面占掩码深度点的 40% 以上。
- 检测到侧面，但无法稳定确定书脊—侧面交界。
- 修正后的宽度小于原掩码宽度的 35% 或大于原宽度。

夹爪开口仍按以下公式输出：

```text
recommended_gripper_opening_mm
= thickness_mm + uncertainty_95_mm + gripper_clearance_mm
```

不读取 `.conf` 并不代表深度质量与最高置信度相同，只表示当前数据没有可用于筛选坏点的
置信度信息。程序仍会进行范围和 MAD 离群值检查，但无法识别所有多径、反光或边缘混合深度。

### 单帧输出的识别字段

`book_thickness.json` 顶层会明确记录：

```json
{
  "capture": null,
  "mode": "single_frame_rgbd",
  "uses_confidence": false,
  "uses_camera_poses": false,
  "frames_requested": [0],
  "input_files": {
    "rgb": "/path/to/rgb.png",
    "depth": "/path/to/depth.png",
    "intrinsics": "/path/to/intrinsics.json",
    "depth_scale_m_per_unit": 0.001
  }
}
```

通用文件模式没有 Record3D 帧编号，因此内部统一将本次观测记为第 0 帧。
Record3D 兼容模式仍会保留实际参考帧编号。

每本书的关键字段具有以下特点：

- `views_observed = 1`、`views_used = 1`。
- `repeatability_mad_mm = 0` 只表示没有第二次测量，不能解释为“零误差”。
- `quality = "low"`：单视角无法验证重复性，程序故意禁止将其标记为高质量。
- `high_confidence_samples` 不代表传感器的真实高置信度点；单帧模式没有读取
  `.conf`，判断是否使用 confidence 应以顶层 `uses_confidence` 为准。
- `uncertainty_95_mm` 只包含 RGB 边界采样、毫米/像素尺度和 0.8 mm 标定下限，
  不包含真实的跨视角重复性。
- `measurements` 中只有参考帧的一条记录。
- `reference_side_face_detected`：是否发现书脊之外的侧面平面。
- `reference_boundary_corrected`：是否已经按主书脊平面裁剪边界。
- `reference_silhouette_thickness_mm`：未经侧面裁剪的轮廓宽度。
- `reference_side_trim_mm`：侧面裁剪减少的毫米数。
- `reference_view_angle_deg`：书脊平面相对相机正视方向的夹角。
- `reference_spine_plane_inlier_ratio`：主书脊平面的深度内点比例。
- `frame_statistics[].rejected_detections`：被拒绝候选及具体原因。

可执行以下命令确认程序确实使用了单帧分支：

```bash
RESULT=/path/to/book_thickness_result \
/mnt/ssd/robotarm/env/sam3_full/bin/python - <<'PY'
import json
import os
from pathlib import Path

path = Path(os.environ["RESULT"]) / "book_thickness.json"
data = json.loads(path.read_text())

print("mode:", data["mode"])
print("uses_confidence:", data["uses_confidence"])
print("uses_camera_poses:", data["uses_camera_poses"])
print("frames_requested:", data["frames_requested"])
print("input_files:", data["input_files"])
print("book_count:", data["book_count"])
PY
```

预期看到：

```text
mode: single_frame_rgbd
uses_confidence: False
uses_camera_poses: False
frames_requested: [0]
input_files: {'rgb': '...', 'depth': '...', 'intrinsics': '...', 'depth_scale_m_per_unit': 0.001}
book_count: 96
```

### 当前数据集测试结果

已将原始第 24 帧转换成普通 RGB PNG、16 位毫米深度 PNG 和独立内参 JSON，
然后只通过三个文件路径完成端到端测试。输出目录为：

```text
2026-09-01--23-22-25/book_thickness_generic_single_frame/
```

测试结果：

- 共检测并测量 96 本书。
- 第 1、2、3 层分别为 29、33、34 本。
- 96 本均有有效毫米厚度。
- 由于每本只有一次观测，质量统计为
  `{"high": 0, "medium": 0, "low": 96}`。
- 与现有七帧融合结果相比，逐书厚度绝对差的中位数为 0.43 mm。
- 绝对差的 95 百分位为 3.40 mm，最大值为 6.46 mm。
- 通用单帧结果相对七帧结果的中位偏差为 `+0.29 mm`。
- 与直接解码 Record3D 单帧深度相比，中位绝对差为 0.075 mm；16 位 PNG 的
  毫米量化可能令少量局部平面拟合分支发生变化。

中位差较小说明大部分边界和尺度估计一致，但 P95 和最大差值表明少量书本可能出现
3–7 mm 偏差。因此，不能根据中位差宣称单帧模式具备稳定的毫米级精度。

### 真机侧面剔除测试

使用 `dual_v2_1/11` 的 `rgb/000001.png`、`depth/000001.png` 和
`cam_K.txt` 进行测试：

- SAM3 得到 66 个书本候选。
- 57 个候选通过书脊平面几何检查。
- 9 个候选因平面朝向不一致、书脊内点不足或侧面交界不明确而被拒绝。
- 6 本书检测到侧面并完成边界裁剪。
- 6 本书共裁掉 14.58 mm 的侧面宽度，单本裁剪量为 1.86–3.60 mm。
- 被拒绝书本不会写入 `gripper_openings.json`，因此不会产生自动抓取指令。

修正结果在 `dual_v2_1/11/book_thickness_000001_plane_corrected/`。主结果图中的
书本 ID 后带 `*` 表示边界经过侧面裁剪；拒绝候选单独显示在
`rejected_books_overlay.png`。

### 单帧采集建议

为减少测量偏差：

1. 相机尽量正对书架，避免过大的水平透视和书脊侧面遮挡。
2. 确保整本书脊清晰可见，图像不能运动模糊或过曝。
3. 尽量让目标位于图像中部，避免镜头边缘畸变区域。
4. 保持 RGB 与 depth 原始分辨率，不要单独缩放或裁剪其中一个文件。
5. 避免书脊表面的强反光、透明塑封和深色吸光区域。
6. 书本之间应有足够边界特征；紧密挤压会使实例分割边界不确定。
7. 首次部署必须选择不同厚度的书，用卡尺建立视觉厚度与真实厚度的误差统计。

### 使用限制

- 单帧结果只适合作为夹爪预张开初值，不能替代接触检测。
- `quality = "low"` 的结果不应直接触发无人值守抓取。
- 实际闭合必须启用力控、力矩限制或电机电流阈值。
- 如果要求稳定毫米级误差，应使用带置信度和有效位姿的多帧模式，并进行卡尺标定。
- 如果书本紧密排列、夹爪手指没有插入空间，仅增加开口宽度无法解决问题；机器人还需要
  抽书、拨动或建立侧向间隙的动作。

## 输出

多帧模式默认输出到 `CAPTURE/book_thickness_analysis/`。通用单帧模式输出到
`--output` 指定的目录；未指定时输出到 `RGB文件名_book_thickness/`：

- `book_thickness.json`：完整边界、逐帧测量、厚度、不确定度和夹爪开口。
- `book_thickness.csv`：便于机器人程序直接读取的逐书汇总。
- `gripper_openings.json`：按书本 ID 索引的夹爪开口，提供 mm 和 m 两种单位。
- `books_thickness_overlay.png`：参考帧全部书脊边界和厚度。
- `rejected_books_overlay.png`：因侧面、观察角或平面质量不足而被拒绝的候选。
- `shelf_1_thickness.png` 等：每层书架的放大标注。
- `book_crops/S1-01.png` 等：每本书的独立核验图。
- `reference_book_masks.npz`：以书本 ID 为键的原始二值掩码。
- `reference_depth_map.png`：输入深度的可视化。

书本 ID 按层编号，从左到右依次为 `S1-01`、`S1-02`……。

## 精度说明

Record3D 数据在约 1 m 距离处的 RGB 理论采样约为 0.7–0.9 mm/像素，但其 LiDAR
深度仅为 192×256。真机 L515 数据可以使用 960×540 对齐深度，更适合进行书脊/侧面
多平面分解。左右边界仍来自 RGB 与深度平面的联合约束，不会把上采样深度误当作
高分辨率几何。

`uncertainty_95_mm` 综合了多帧重复性、左右边界像素采样和 0.8 mm 标定下限。
它是抓取规划用的工程估计，不是经过标准量块校准的计量认证。首次用于真实夹爪前，
应抽取若干书本用卡尺测量并检查系统偏差；如存在稳定偏差，可据此增加或校正
`--gripper-clearance-mm`。

实际闭合仍应使用夹爪力控或电流阈值，不能只依赖视觉宽度做刚性位置闭合。
