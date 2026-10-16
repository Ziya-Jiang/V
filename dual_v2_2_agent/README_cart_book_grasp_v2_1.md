# V2.1：先识别、显示编号，再选择抓取

入口：`cart_book_grasp_v2_1.py`。默认已改为编号选择，不再输入颜色或语言描述，不调用 GPT 或 SigLIP，也不计算颜色门槛、文本分数或跨视角身份匹配。

## 流程

1. 启动后先执行原有右臂 first_reset，检查实测关节姿态，再获取头部 RGB-D。启用手部相机时保留同期手部图像供查看。
2. 头部 SAM3 使用 `book` 提出候选，按置信分数、掩码面积、重复掩码和基座工作区域筛选。已被两个独立实例解释的整组 mask 被去除。再自动裁出这些候选的联合区域，四周增加 25% 上下文，用 SAM3 的 `bookend` 提示独立分割书立，明确匹配到书立的候选在编号前排除；颜色不参与筛选。
3. 根据头部深度计算候选的基座位置，按机器人视角从左到右（基座 +Y 到 -Y）标成 **1、2、3……**。顺序不依赖 SAM3 输出顺序，也不直接假定相机画面横坐标等于机器人左右方向。
4. **先在 Rerun 和 `head_numbered.png` 显示并保存本轮编号，然后终端等待输入编号。** 输入 `2` 回车就绑定本轮画面中的第 2 个 mask。非法编号会提示重新输入，不会选择相邻书本。
5. 只对所选头部 mask 运行 V1_3 书脊、倾角、整书深度检查，随后执行原有抓放流程。目标几何失败不换书。
6. 完成、失败或 p 中断后，按回车开始新的识别轮次，再根据新画面选编号；不记忆或自动重试旧编号。

手部图像只作查看，不给它强行套用头部编号；它的书本数量、外观和颜色不再阻止选择头部编号。抓取坐标仍完全来自头部标定和 V1_3，未启用 V3 视觉伺服。

SAM3 仍可能给出错误候选，编号是本轮候选列表，不代表书名或持久身份。请按编号预览选择书本。缺少深度会阻止建立可靠的机器人左右顺序；重叠或左右位置过近的实例会显示为不可选。规划器的可达性、关节限位和碰撞检查继续生效。

书立分割在候选局部图上重新计算一次 SAM3 图像特征；多个书立提示共用该局部图特征。输出 mask 平移回原始头部图像坐标后，要求书立分数 ≥ 0.25、覆盖某个候选 ≥ 75%、该候选占书立 mask ≥ 75%，且书立 mask 覆盖其他任一候选不超过 20%，才排除整个候选。不会直接用书/书立两个提示的分数比大小，也不会挖掉真书 mask 中的像素；原始书本深度和抓取几何保持原样。分数不是校准后的概率。

局部图来自本轮工作区内的候选，不使用固定像素位置或“最左侧一定是书立”的规则。没有明确匹配时保留候选并打印说明，不把未检出书立当成真书认证。书立识别仍可能漏检或误检，需要结合编号预览查看。

## 启动

```bash
python cart_book_grasp_v2_1.py --check-config
python cart_book_grasp_v2_1.py --observe-only --assume-head-camera-calibrated
python cart_book_grasp_v2_1.py --assume-head-camera-calibrated
```

`--check-config` 不初始化机器人和相机。`--observe-only` 会执行既有 first_reset、感知、显示编号和等待编号；选中后结束该轮，不计算或执行抓取。最后一条为实际抓放入口。`--assume-head-camera-calibrated` 不会自动完成标定。

输入示例：看到本轮 1、2、3 之后，输入 `2` 回车。输入等待和动作中都支持 `p` 中断并按原逻辑松爪复位；`s` 在选书时取消本轮，动作中停止后续轮次。它们沿用规划器的协作中断机制。

## 配置与时效

- `cart_book_grasp_v2_1_config.json` 每轮读取。`book_min_score=0.25` 是 SAM3 候选分数门槛，**不是颜色门槛**。
- `bookend_filter` 控制 SAM3 书立排除，默认 `enabled=true`、`prompts=["bookend"]`。`min_score`、`min_book_coverage`、`min_bookend_coverage`、`max_other_book_coverage` 分别控制书立分数和掩码关联；`crop_padding_fraction` 控制局部图上下文。将 `enabled` 改为 false 可恢复单独 `book` 候选流程；下一轮生效。
- `head_roi`、`head_base_bounds_mm` 定义图像及机器人基座工作区；`min_area_fraction`、`min_mask_fill` 过滤过小、过稀疏掩码。
- `max_candidates` 是处理上限，超限会报错，不截取前几个；`expected_books` 必须为 null。
- `pose_tolerance_deg=1` 保留 first_reset 到位检查，兼容协议的 2×7 和旧的长度 14 关节数组。
- `max_observation_age_seconds=30` 从头部采集时起计时，包含分割和输入等待；终端会显示剩余时间。超时后本轮编号作废，回车重新采集，禁止拿过期编号执行动作。等待编号期间继续检查机器人是否静止。
- 原配置中的 `semantic_*`、`color_*`、`pair_margin`、`reference_path` 不用于当前编号选择，也不加载本地语义权重。
- `cart_book_grasp_v2_1_motion_config.json` 和 `cart_book_grasp_v2_1_wrist_config.json` 保留独立 V1_3 动作参数：J6=-20.3164°、伸入 100 mm、抬升 200 mm、停顿 3 s、下降 150 mm/段长 50 mm。编号选择没有修改这些动作。

## 离线检查

```bash
python cart_book_grasp_v2_1.py --capture-pair /tmp/new_book_pair
python cart_book_grasp_v2_1.py --preview-pair /tmp/new_book_pair --output /tmp/book_numbers
python cart_book_grasp_v2_1.py --preview-pair /tmp/new_book_pair --book-number 2 --output /tmp/book_number_2
```

第一条只采集相机图像，不连接机器人、不验证 first_reset。后两条读取保存的 RGB-D，生成编号图和报告；不计算或执行抓取。省略 `--book-number` 只展示编号，不默认选中任何一本。`--instruction 2` 是兼容别名，仅接受数字。SAM3 缓存只在 RGB 指纹一致时复用。旧的登记外观参考工具仍保留，但当前编号选择不使用它。

## 可视化与记录

Rerun 主视图使用英文标题 `Numbered books: robot left to right`；轮廓和大号数字标明候选，黄色表示选中编号，红色表示实例存在歧义。`SAM3 book / bookend filtering` 面板用紫红轮廓标出被排除的书立，橙色表示没有明确匹配到单个候选的书立提议；编号仍只属于保留下来的候选。图像保留原色，标签只使用英文。手部面板为 `Wrist context (no shared IDs)`。原有书脊中心、深度、环境点云和抓取轨迹继续显示。

每轮结果在 `RobotRAM/output/v2_1/<scene_id>/`：`head_numbered.png`、`head_selected.png`、`head_bookend_filter.png`、原始 RGB-D、头部 SAM3 mask、标定及关节证据、`decision.json`。`head_bookend_crop.npz` 保存局部图的原始书立 mask、图像指纹和提示列表；离线缓存只在局部图和提示完全匹配时复用。`catalog` 记录本轮编号与 mask/基座位置关系；`proposal_filter.head.bookends` 记录裁剪范围、逐 mask 关联证据、排除原因和数量。编号预览在等待输入前保存并发布。

2026-10-09 对 5 张已保存的同场景头部图像做了离线验证：4 张原先有 4 个候选（含左侧书立），均排除书立后保留 3 本；另一张原先已只有 3 本，过滤后仍为 3 本。完整预览结果保存在 `RobotRAM/output/v2_1_bookend_validation_20261009/`。整图 `bookend` / `metal bookend` 提示在最初测试帧没有检出书立，候选区域放大后 `bookend` 有效，所以采用局部检测。该结果仅覆盖这些保存画面，未验证新场景或实机动作。CPU 实测局部书立推理约 10 秒（另有首次模型加载约 10 秒）；不是 GPU 在线延迟测量，原有 30 秒图像有效期未改变。

测试使用合成数据、保存的真实图像、模拟输入和机器人状态，不替代实机抓取验证。旧语义选择实现保留在源码中供回归比较，当前入口使用 `NumberedBookSelectionAgent`。

## 标准 YOLO26 离线检测对照

`preview_yolo26_books.py` 使用标准 YOLO26 检测模型，默认本地 `models/yolo26x.pt`，不是 YOLO-World。官方 YOLO26-L/X 权重已保存，来源和 SHA-256 在 `models/yolo26_sources.json`。使用现有 YOLO 环境即可，无需修改机器人环境：

```bash
/mnt/ssd/robotarm/env/yolo/bin/python preview_yolo26_books.py --image RobotRAM/output/v2_1/c5eba7f9fe414781a0bdc32512c3551b/head_rgb.png --output /tmp/new_yolo26_preview --imgsz 1024 --conf 0.1 --head nms
```

脚本只读取图片，输出 `detections.jpg` 和 `report.json`；不调用相机、机器人或 SAM3，不自动下载替代模型。可用 `--head end2end` 比较另一检测头。2026-10-09 在含书立的保存画面上，YOLO26-X 的两种检测头均只检出推车内三本书中的两本，右侧灰书漏检；没有独立检测出左侧书立。画面右侧工作区外还有额外 book 框。这是单帧离线结果，主程序使用 SAM3 书本/书立分割再编号，没有启用 YOLO 前置过滤。
