---
name: hand-workspace-ar
description: 实时显示右手当前高度平面的工作空间，将水平向前抓取的三维范围叠加到头部 RGB-D 图像，当前显示金色平面，按真实深度将面下区域压暗、面上保持原色，遮挡裁切关闭。用户说“打开手部工作空间 AR”“显示手所在平面的可达范围”“启动/停止实时工作空间”或调用 $hand-workspace-ar 时使用。适用于本机 dual_v2_2_agent 机器人项目，只读可视化，不执行机械臂运动。
---

# 手部平面工作空间 AR

## 直接调用

项目为 `/mnt/ssd/user/V/dual_v2_2_agent`；Python 为 `/mnt/ssd/robotarm/env/robotram_wx/bin/python`。本技能只启动显示与读取，不导入/实例化 RobotArm，不发送运动、复位、夹爪或规划命令。

1. 执行 `bash scripts/control.sh status`。以下脚本路径均相对于本 SKILL.md。
2. 用户要求开启时，执行 `bash scripts/control.sh start`；如果已运行则复用。显示进程订阅独立的 head_rgbd 共享服务（127.0.0.1:8792），首次启动会自动创建唯一采集进程；当前 cart_book_grasp V1～V3 本地相机入口也已接入同一服务。若旧版直接采集进程占用 USB，报告冲突，不杀死或重启机器人进程。
3. 读取返回的 `mode`，直到启动成功或明确失败，最多检查 20 秒。成功需区分：`live` 表示相机和手部状态均有效；`waiting_state` 表示只有实时相机，不能声称已跟随手部。
4. 用 `open_in_codex` 打开返回的本机 URL（默认 `http://127.0.0.1:8791`）。这是持续更新的本地应用，不是旧快照，也不嵌入无网络的 inline artifact。
5. 用户要求停止 AR 时执行 `bash scripts/control.sh stop`，只停止显示订阅者，保留共享源供抓书程序使用。仅在用户要求释放头部相机且所有订阅者已退出时，在项目目录执行 `/mnt/ssd/robotarm/env/robotram_wx/bin/python -m head_rgbd stop`。

工具沙箱禁止本地 socket 或设备访问时，按正常机制申请运行该具体只读命令所需的执行权限。已经授权启动显示，无需另行索取机器人动作许可。

## 接入实际手部状态

优先复用现有新鲜 `RobotRAM/output/hand_workspace_ar_live/state.json`。不存在或已过期时，当前部署使用被动旁路读取发往 `127.0.0.1:30241` 的 PanelState 报文，再以实测右臂关节角做 URDF 正运动学。不得绑定、复用或抢占 10165/10166 控制端口；不得启动另一个控制器或修改运行中的 V3。

运行 `bash scripts/state-tap.sh` 开启旁路状态转发。它需要本机 `sudo tcpdump` 权限，且应在独立终端持续运行，Ctrl+C 停止。先检查 `sudo -n true`；可用时可启动并保存该任务自身的终端会话；不可用时，继续完成显示启动并告诉用户在本机终端执行脚本。不要索取密码、改 sudoers 或 setcap，也不要将权限失败说成已接通。之前本会话 sudo 需要用户密码。

示例用户终端命令（安装后的路径）：

```bash
bash /home/vlmdemo/.codex/skills/hand-workspace-ar/scripts/state-tap.sh
```

这是旁路状态捕获，只过滤本机目标端口。源端口是临时端口，不能固定为 10166。脚本不会保存完整流量；只把必要关节/末端状态写入 JSON。当前直接读取面板发布的实测关节状态。禁止使用 10165 的规划器缓存冒充新状态：上游断开时该端口仍可能重发旧关节值。tcpdump 必须启用 --immediate-mode，避免约一秒的抓包缓冲。脚本启动后持续监听；Remote Disconnect 时等待新报文，重新 Connect 后自动恢复，无需 Ctrl+C 或重复启动。只有用户明确停止旁路进程后才需重新启动。若 30241 没有报文，检查面板状态发布连接；不要自动 Start、重启控制器或发送机器人指令。

## 坐标和显示含义

- 默认右臂 `arm_end_effector_r` 原点所在的水平面；参考系 `dummy_world`。不是指尖或带偏置的 `arm_tool_pose`。若用户要求另一参考点，先明确偏置再改动，不能悄悄替换。
- 工作空间限制：夹爪朝前，EE 的 −Z 对准机器人 +X，滚转自由。不是随手腕姿态倾斜的平面。
- 既有 URDF 样本位于 SJ4Z；转换到 dummy_world 时 Z 增加 342 mm。URDF 哈希不一致会停止加载，必须重新生成样本。
- 金色半透明水平水面（填充不透明度约 42–48%）（无物体轮廓描线、无网格和箭头），表示 URDF 采样的近似可达包络；平滑不构成 IK 保证。未包含碰撞检查。
- 头部相机 CV2TC5100060，848×530，沿用 V3 外参、畸变与硬件/软件 D2C 对齐。按用户最新要求保留深度明暗：根据深度反投影的 dummy_world Z，在当前工作空间 XY 范围内，平面以上和位于平面的可见表面保持原图，平面以下逐渐压暗（40 mm 过渡）；无效深度保持原图。仅关闭逐像素深度遮挡裁切，不能把明暗也关闭。原始深度仍完整提供给抓书程序。
- 手部数据 >500 ms、图像 >600 ms、图像与状态时间差 >200 ms 时保留上一完整帧（RGB、AR 和高度一起冻结），标注“画面已暂停”及旧帧时间；客户端断线时也保留画面，新帧解码完成后再替换。独立 head_rgbd 服务的采集线程持续排空 SDK，只保留最新一帧；AR 与抓书各有独立订阅游标，RGB/float32 毫米深度/标定/时间戳原子打包且无损传输。渲染不阻塞采集；连续读取失败时共享源自动重连，订阅者在服务重启后自动恢复。AR 退出不停止共享相机。SDK 图像时间以设备时间增量映射主机时间，不宣称硬件同步精度。
- 允许保留旧帧，但必须明确标记暂停及时间；不用旧截图、示教位置或猜测高度冒充实时状态。不在用户未要求时运动机械臂来验证。

## 故障与验证

运行日志：`/mnt/ssd/user/V/dual_v2_2_agent/RobotRAM/output/hand_workspace_ar_live/viewer.log`。

共享源状态：在项目目录运行 `/mnt/ssd/robotarm/env/robotram_wx/bin/python -m head_rgbd status`；日志 `RobotRAM/output/head_rgbd_service/service.log`。本机采用类似 ROS 的多订阅者结构，尚未发布 ROS topic。抓书输入是原始 RGB-D，不是金色叠加图；抓书局部外参校正不会改写 AR 标定。

程序与测试：`/mnt/ssd/user/V/dual_v2_2_agent/hand_workspace_ar/`。验证命令：

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
/mnt/ssd/robotarm/env/robotram_wx/bin/python -m unittest head_rgbd.test_shared hand_workspace_ar.test_runtime -v
```

相机变更、支架移动或标定变化后，核对实际投影再使用。需要全新机器人/相机时重新绑定 URDF、坐标系、标定、序列号与状态来源，本技能不自动套用本机参数。
