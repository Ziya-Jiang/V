# 头部 RGB-D 共享源

一台 Orbbec 由独立的 `python -m head_rgbd serve` 进程持续采集，AR 和抓书程序分别订阅。采用本机 HTTP 最新帧服务（127.0.0.1:8792），是类似 ROS 发布/订阅的结构，**当前不是 ROS topic**。

```
头部 Orbbec → 独立 head_rgbd 服务 → AR（8791，金色覆盖层）
                              └→ cart_book_grasp*（原始 RGB + 深度）
```

从项目目录运行，Python 使用 `/mnt/ssd/robotarm/env/robotram_wx/bin/python`：

```bash
python -m head_rgbd start
python -m head_rgbd status
```

AR 和本地抓书公共入口 `start_calibrated_orbbec()` 会自动启动/复用共享服务，原有抓书启动命令无需追加参数。适用于当前 V1、V1_1、V1_2、V1_3、V2、V2_1、V3 入口及 V2_1 的只读 capture-pair。已在运行的旧抓书进程需要在本轮安全退出后重新启动才加载修改；不强制结束它。旧备份、独立直接使用 SDK 的脚本不在此接入范围。`--remote-rgbd` 仍使用原有远程相机路径。

停止 AR 或抓书程序只关闭自己的订阅，服务继续采集。需要释放 USB 时，先退出使用者，再运行：

```bash
python -m head_rgbd stop
```

有存活订阅者时，服务退出会触发订阅者的自动恢复；因此不要用 stop 作为暂停单个消费者的方式。不要另外启动直接占用同一 USB 的旧采集脚本。

传输保证：

- 848×530 原始图像，BGR8 传输；抓书适配器转换为原接口的 RGB8。
- 对齐深度为 float32 毫米，无 JPEG 压缩或深度整数舍入。
- RGB、深度、捕获时间、标定和会话/帧序号在同一个包中；`np.load(..., allow_pickle=False)`。
- 每个消费者有独立游标，只取最新帧，不积压，也不会消耗别人的帧。服务每帧编码一次；消费者拥有独立数组，AR 渲染不修改抓书输入。
- 服务只保留一帧；设备故障自动重新连接。帧过期 500 ms 或重复时不向抓书返回旧数据。抓书原有异常暂停逻辑仍然适用，不会自动继续运动。
- AR 无新数据时继续保留上一完整画面并标示暂停。
- 抓书订阅验证序列号、分辨率及 K/D/R/T 与其启动标定一致；不一致则报错，不偷偷回退 USB。
- 抓书运行时的头部外参校正只影响本消费者，AR 仍使用共享服务的原始标定；本改动不保证校正后两者外参一致。

运行文件位于 `RobotRAM/output/head_rgbd_service/`，日志 `service.log`，实机只读验证结果 `verification.json`。测试（本机 socket 权限必需）为：

```bash
python -m unittest head_rgbd.test_shared hand_workspace_ar.test_runtime -v
```

本服务不绑定机器人控制端口，不发送机械臂/夹爪命令。腕部相机不属于这个共享源。
