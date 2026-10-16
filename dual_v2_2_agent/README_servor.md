# servor.py：通用物体语义选特征与独立视觉对准

独立于 V3.3 抓取流程。输入物体名称和描述，从当前已去畸变的腕部图像选择一个可跟踪表面、抓取中心和物体长轴，自动提取固定身份特征，再进行 Y／工具轴旋转闭环。TCP X/Z 固定为开始对准时的实测值。不会前伸、复位或闭合夹爪。

物体不限书本，例如包装盒上的标签、工具上的平面标记。当前随附算法要求有纹理、近似平面、刚性的表面；并不是所有杯子、透明玻璃、光滑金属或软物体都可跟踪。抓取部位和对准方向不明确时返回不支持/歧义，不根据物体类别猜深度。

## 启动

现有为 V3.3 准备的 `dual_v2_2_upper_planner_v3_1.py` 继续使用，端口 10176；原规划器和 V3.3 文件均未修改。`servor.py` 占用客户端端口 10175，不要和其他 V3.x 抓取客户端同时运行。启动前由用户将右手置于目标可见、周围留有小幅纠偏空间的静止观测姿态。

仅感知、持续跟踪 120 秒，不连接机器人：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python servor.py \
  --object '包装盒' --description '中间蓝色盒子，选择正面标签，对准标签中心和竖直长轴'
```

实机执行对准：

```bash
/mnt/ssd/robotarm/env/robotram_wx/bin/python servor.py \
  --object '包装盒' --description '中间蓝色盒子，选择正面标签，对准标签中心和竖直长轴' \
  --execute
```

`--execute` 会先进行 +Y 3 mm、工具轴旋转 2° 的两个局部辨识动作，再连续对准。机械臂位姿无需重新复位。这里只完成选定中心的水平对准和长轴竖直对齐；不控制图像竖直位置，也不意味着抓取已经完成。

GPT 首次定位使用 `gpt-6-astra`；环境变量 `OPENAI_API_KEY` 优先，否则读取本机已配置密钥文件。自动定位会把本次腕部图像和描述发送到 OpenAI Responses API。每帧跟踪在本地完成。密钥不复制到 Skill、日志或输出目录。若不能外发图像，使用 `--task` 提供本地审查的像素区域，跳过 API。

## 离线验证

```bash
/mnt/ssd/robotarm/env/robotram_wx/bin/python servor.py \
  --object '工具' --description '对准工具平面标签的中心和纵向轴' \
  --image observation.png --task task.json --frames sequence_directory
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest test_servor -v
/mnt/ssd/robotarm/env/robotram_wx/bin/python integrations/grasp-servo-features/scripts/test_servo_features.py -q
```

`--image` / `--frames` 不允许与 `--execute` 同用。`--out` 必须是新目录，避免覆盖固定身份模板。`task.json` 使用原图像素，示例字段：

```json
{
  "status": "selected",
  "confidence": 0.9,
  "reason": "已审查的刚性平面标签",
  "patch_polygon": [[100,80],[500,80],[500,400],[100,400]],
  "grasp_center": [300,240],
  "alignment_axis": [[300,180],[300,300]]
}
```

这些示例坐标不是可直接用于实机的目标。区域、中心与轴必须来自实际输入图像。

## 实现和边界

- GPT 提供表面四边形及语义中心/轴，本地 SIFT 提取候选点，按区域归一化距离选出围绕语义抓取中心的 4–8 个分布特征，初次分配 ID 后不再改点；跟踪用 LK 双向校验，每 10 帧 SIFT 重定位，并以 RANSAC 单应和逐点纹理校验过滤错误。
- 通过同一表面的单应变换传播语义中心/轴，避免用特征点均值代替抓取中心。中心必须位于当前有效特征支撑范围内。特征身份不变，失效不会偷偷重选其他物体。
- 辨识矩阵由图像误差变化与**实测** Y/工具轴旋转变化计算，含交叉耦合。探测动作位置到位容差 1 mm；连续命令标称位置容差 3 mm。图像目标容差 3 px、1.5°，保持后连续 5 帧且至少 0.3 秒稳定才完成。
- 图像与位姿按主机单调时钟配对。这是相机读帧完成时间和反馈接收时间的近似，未经硬件曝光/编码器同步标定。保留 3 mm/1° 插值支撑限制、0.25 秒帧/反馈年龄上限，配对恢复最多 2 秒。
- 外环目标更新最高 10 Hz，实际受采集和跟踪时间限制；规划器当前内部直接伺服周期 20 ms（50 Hz）。增益 0.35；单次纠偏上限 Y 4 mm / 旋转 1°，整体纠偏范围 Y ±50 mm / 旋转 ±15°。X/Z 实测偏差超过 3 mm、累计位移超过 200 mm 或总时限到达即退出。
- 原规划器负责关节速度、加速度、碰撞及高度保护。线协议仍使用 `book_v3_1_stream` / `book_v3_xyz_servo` / `book_v3_1_hold`；`book` 只是既有协议兼容字段，不限制语义目标种类。
- 特征失效立即请求 hold，下一张新鲜帧立即重新匹配同一模板（不必等第 10 帧），最多 3 秒重新捕获**同一模板**；失败退出，不盲目前伸。通信失效时依靠现有规划器有限目标和租约停止持续推进，软件 hold 不替代硬件急停。

输出在 `output/servor_<时间戳>/`：`task.json`、首帧、特征模板、`selection.png`、带时间戳的 `events.jsonl`。连续感知预览每 200 帧保存一张。执行模式不逐帧保存图片。

验证范围：16 项本地控制/跟踪测试；用户明确授权后，已将指定历史腕部图发送到 OpenAI，完成 GPT 自动局部选区与自动特征初始化。结果位于 `output/servor_validation_gpt_localpatch_20261012/`。20 帧已知合成变换均保持有效特征，点误差 P95 约 0.37 px；另一个包含控制中心支撑检查和同模板恢复的 25 帧序列中 24 帧可用于控制，1 帧因支撑不足被拒绝并在下一帧重新匹配恢复。没有放宽有效性阈值。未执行实机运动，合成序列不等于实机闭环验证。
