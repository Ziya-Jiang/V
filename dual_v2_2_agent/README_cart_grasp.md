# 右车把 ArUco 对准

入口 `cart_grasp.py`。默认执行 **一次粗对准 → 腕部视觉对准**；增加 `--approach` 后，依据头部 RGB-D 深度沿 X 前伸并持续纠偏。两种模式均不夹紧、不抬升、不自动复位。右车把 ID **0**，左车把 ID **1**，底盘最下方 ID **2**；DICT_4X4_50，黑框边长 **20 mm**。旧 `cart_grap_v1.py` 保留为兼容入口。

## 仅腕部视觉对准（当前使用）

直接让腕部相机识别右车把 ID 0，**跳过头部相机和全部粗移动**。仅沿 Y 与工具轴旋转纠偏，X/Z 固定为启动时实测值，夹爪指令不变。无需头部解码、深度或前伸终点；沿用已有腕部内参和实测图像雅可比。启动时标记须完整可见，标记竖边需与车把杆平行。

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
# 只读检查腕部 ID 0；不连接机器人或头部相机
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grasp.py align --wrist-only

# 执行仅腕部视觉对准；仍需现有专用伺服适配器运行
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grasp.py align --wrist-only --execute
```

`--wrist-only` 与 `--approach` 互斥。运行期间标记丢失即保持，超时退出；不自动寻找目标、前伸或闭合。保留原来的头部粗对准/前伸命令作为显式可选模式。

## 运行

在现有 V3.4 专用规划器/直接伺服适配器运行后使用；同一时间只运行一个占用 10175 的抓取客户端。先在已有观测姿态启动；程序不调用 first_reset。

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
# 离线配置检查，不打开设备
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grasp.py align --check-config

# 只读：头部定位 ID 0，并保存粗对准方案，不发运动指令
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grasp.py align

# 执行纯对准：沿 Y 粗对准一次，然后 Y + 手腕绕工具轴旋转对准
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grasp.py align --execute
```

前伸模式（先省略 `--execute` 可只读查看方案）：

```bash
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grasp.py align --approach --execute
```

`cart_grasp_approach.json` 独立配置前伸参数。头部原始深度先经手眼标定变换为 **基座 X**，不能把相机 Z 直接当作机械臂前伸量：

```
指尖 X 偏移 = 50 mm × 工具 -Z 轴在基座 +X 上的投影
目标 TCP X = 标记基座 X - 指尖 X 偏移 - standoff_mm
剩余前伸 = 目标 TCP X - 当前实测 TCP X
```

50 mm 复用现有夹爪几何参数，不加入抓书的 100 mm 伸入。`standoff_mm` 默认 **20 mm**，表示指尖在基座 X 方向停在标记位置之前；这是接近阶段的预留量，不代表已到夹持中心。标记需固定在目标车把上且小车在这一轮中保持静止；头部终点在粗定位时测量并锁定，腕部不重新估计终点深度。若标记在外伸支架上，标记位置与车把接触面可能不同，此模式不承诺抓住车把。

只有腕部水平误差 ≤3 px、角度误差 ≤1.5° 时才给出正向 X 目标；不满足就继续 Y/角度纠偏。X 目标最多领先实测位置 6 mm，接近终点按剩余距离缩短，恢复前伸有 0.4 秒渐增。达到 X 误差 ≤3 mm 且图像连续稳定后停止；不自动后退。单轮最大前伸 250 mm，总反馈路径预算 400 mm。丢标记保持、超时退出，不用旧坐标继续接近。工具轴旋转不会改变上述指尖轴的 X 投影。

结果存到 `output/cart_align_<时间戳>/`：`plan.json`、`head_marker.png`、`events.jsonl`；腕部标注图每 200 帧保存一张。默认不弹出可视化窗口。

## 目标与控制

1. 头部订阅共享相机服务的最高原始分辨率 RGB-D，使用现有头部手眼标定。连续三帧解码 ID 0，在标记内部取有效深度，用畸变参数反投影到机器人基座坐标。深度混入背景、缺失、重复 ID 或三帧位置不稳定时取消粗对准。
2. 一次 CuRobo 粗对准：仅将右臂 TCP 的 Y 改到标记中心对应的基座 Y，保留起点 X/Z 和姿态。使用 3 mm、1° 和连续新鲜反馈确认粗到位。端点 X/Z 固定不等于 CuRobo 中间轨迹完全直线。
3. 腕部在去畸变图像中直接解码 ID 0 四角，不用 SAM/GPT、SIFT 或 CoTracker。目标为标记透视中心处于画面中央列、标记竖向中心轴保持竖直；不控制其上下像素位置。标记的竖边应与车把竖直杆平行，标记中心应贴近要对准的杆中心。此阶段是**标记对准**，尚未建立标记与夹爪最终夹持中心的几何偏移。开启前伸仅使用已配置的指尖偏移与预留量。
4. 实测 3 mm Y 与 2° 工具轴旋转的小探测，使用配对图像和实际位姿计算 2×2 图像雅可比；探测动作等待到位。随后连续更新目标，中间不逐步等待到位。粗动作走 CuRobo，视觉阶段走现有 CPU 差分伺服。
5. 连续 5 帧、至少 0.3 秒图像达标且反馈位置稳定，结束对准并保持。丢标记即请求保持，只允许同一 ID 恢复；2 秒内无法恢复则退出。不会触发书本的丢点后直接前伸。

运动命令沿用现有专用后端的 `book_v3_xyz_servo` / `book_v3_1_stream` / `book_v3_1_hold` 作为协议名称，不代表使用书本感知逻辑。未修改旧规划器或 V3.4 程序。原始反馈时戳、图像配对误差限制与后端碰撞/关节约束继续生效。

## 参数

独立配置 `cart_grasp_alignment.json`：gain 0.35，视觉目标更新上限 10 Hz，每次 Y 最多 4 mm、旋转最多 1°。实际频率取决于新鲜图像、反馈和确认速度，不保证相机提供 10 Hz。粗对准 Y 最大 200 mm；视觉阶段相对粗对准点 Y 最大 50 mm、工具轴旋转 15°；纯对准模式 X/Z 实测偏差上限 3 mm；前伸模式只固定 Z，X 在已计算的前伸区间内运动。两次探测用 1 mm 到位标准，连续阶段沿用 3 mm；图像结束阈值 3 px / 1.5°。两阶段均不改变夹爪指令。

## 打印和只读观察

打印 `output/pdf/cart_three_markers_left1_right0_bottom2_20mm_A4.pdf`，100% 实际尺寸：黑框 20 mm，白边各 4 mm，整体 28 mm。平整固定，不要绕圆杆弯折。标记方向需与杆方向一致；避免放在夹爪遮挡处。

```bash
/mnt/ssd/robotarm/env/robotram_wx/bin/python cart_grasp.py observe \
  --marker-id 0 --marker-mm 20 --seconds 30 --out output/cart_handle_observe_01
```

`marker` 和 `teach` 命令继续可用。`teach` 只读记录闭合前示教，尚未用于本次纯对准流程。前伸已可采用上述头部深度终点；未来真正夹紧仍需确定标记与夹持中心的关系，不能直接套用抓书的伸入量。

## 验证

`python -m unittest test_cart_grap_v1 test_cart_grasp_alignment test_cart_grasp_wrist_only -v`：29 项离线测试通过，覆盖标记身份、尺寸、几何、深度反投影、粗对准轴约束、严格到位检查、连续闭环收敛、丢失保持/恢复、越界停止，以及头部 X 终点/指尖投影计算、连续前伸/减速与丢标记停止。合成闭环通过不等于实机对准验证。
