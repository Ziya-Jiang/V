# 手部相机：ROS 2 camera_calibration

使用官方 ros-perception/image_pipeline 的 Humble 分支，未修改其标定算法。
源码固定在 `d1453cc59063f1cbcf5c9591438834f7f08818d8`（camera_calibration 3.0.9）。
本目录补充 USB 图像发布、一键启动和退出时归档结果。

## 启动

```bash
cd /mnt/ssd/user/V/dual_v2_2_calibration/ros_camera_calibration
bash run_calibration.sh
```

默认使用现有 LRCP G-720P 手部相机的稳定设备路径，分辨率 1280×720、MJPEG、15 fps。
默认棋盘格为 **12×9 个格子 → 11×8 内角点**，每格 **0.01 m**。
如果板上标注的 12×9 指的是内角点，运行：

```bash
BOARD=12x9 bash run_calibration.sh
```

可通过 `DEVICE`、`WIDTH`、`HEIGHT`、`BOARD`、`SQUARE` 环境变量覆盖配置。
脚本不缩放、不裁剪、不去畸变，实际分辨率不符时会停止。
这一步只标定相机内参和畸变；不包含手眼变换，也不控制机械臂。

## 采集与保存

1. 把平整、尺寸准确的棋盘格完整放入相机视野，保证清晰、无反光。
2. 让棋盘格覆盖中央、左、右、上、下，改变距离和倾角，各姿态稍停稳。
   工具自动采集有差异的样本；观察 X、Y、Size、Skew 覆盖条。
3. `CALIBRATE` 可点击后执行标定，等待计算完成。
4. 点击 **SAVE**，然后在图像窗口按 **q** 退出。
   `s` 键仅保存窗口截图，不是保存标定参数。
5. 本发布器没有 `set_camera_info` 服务，因此使用 SAVE，不使用 COMMIT。

官方 SAVE 首先写入 `/tmp/calibrationdata.tar.gz`；退出启动脚本时会把本次新结果复制到
`sessions/<时间_进程号>/calibrationdata.tar.gz`，并提取 `ost.yaml`。
每次会话还保存配置、发布器日志和标定器日志。不要同时运行多个标定窗口，因为官方临时输出路径固定。

`ost.yaml` 包含分辨率、相机矩阵 K、畸变 D、矫正矩阵 R 和投影矩阵 P。
原始图像使用 K/D，去畸变后的图像应按其输出投影矩阵使用；更改分辨率、裁剪或焦距后不能直接沿用参数。
采集覆盖条达标不等于精度达标；应再用未参与采集的棋盘格姿态检查矫正后线条与误差。

## 已有 ROS 图像话题

不需要 USB 发布器时，可以直接运行官方工具：

```bash
source /mnt/ssd/user/V/dual_v2_2_calibration/ros_camera_calibration/env.sh
ros2 run camera_calibration cameracalibrator \
  --size 11x8 --square 0.01 --camera_name wrist_camera --no-service-check \
  --ros-args -r image:=/你的相机/image_raw -r camera:=/你的相机
```

此方式 SAVE 后的结果留在 `/tmp/calibrationdata.tar.gz`，需要自行复制。
单目模式启动时关于 `/left`、`/right` 无发布器的提示可忽略。

## 环境复现

依赖本机已有的 Ubuntu 22.04、ROS 2 Humble、colcon、cv_bridge、image_geometry、message_filters。
系统 NumPy 2 与 Humble cv_bridge 的二进制不兼容，因此用项目内虚拟环境固定 NumPy 1.26.4。
没有修改系统 Python 包；此虚拟环境专用于标定，不用于 ZED 等其他相机 SDK。

```bash
cd /mnt/ssd/user/V/dual_v2_2_calibration/ros_camera_calibration
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
PYTHONNOUSERSITE=1 .venv/bin/python -m pip install -r requirements.txt
mkdir -p vendor
# 仅在 vendor/image_pipeline 不存在时执行 clone
git clone --branch humble https://github.com/ros-perception/image_pipeline.git vendor/image_pipeline
git -C vendor/image_pipeline checkout d1453cc59063f1cbcf5c9591438834f7f08818d8
PYTHONNOUSERSITE=1 .venv/bin/python /usr/bin/colcon --log-base build_logs build \
  --base-paths vendor/image_pipeline/camera_calibration --packages-select camera_calibration --symlink-install
```

## 本机验证（2026-10-09）

- 官方包成功构建，`ros2 run camera_calibration cameracalibrator --help` 可运行。
- NumPy/OpenCV/cv_bridge 图像转换成功，独立环境没有 NumPy ABI 报错。
- 从现有 USB 手部相机成功获取 1280×720 真实图像，见 `verification/wrist_preview.png`。
- 官方标定窗口连接发布器运行 12 秒后主动停止，无启动异常。
- 官方 `TestArtificial.test_monocular` 通过（含棋盘格和圆点图案）。
- 尚未采集真实棋盘格数据，尚无此相机的新标定结果。

重复运行官方测试：

```bash
source env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
  vendor/image_pipeline/camera_calibration/test/test_directed.py \
  -k 'TestArtificial and test_monocular' -q
```

官方教程：https://github.com/ros-perception/image_pipeline/blob/humble/camera_calibration/doc/tutorial_mono.rst
